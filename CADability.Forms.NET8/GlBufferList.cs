using System;
using System.Collections.Generic;
using System.Numerics;
using Silk.NET.OpenGL;
using CADability;
using CADability.GeoObject;

namespace CADability.Forms.NET8
{
    /// <summary>
    /// Replacement for OpenGlList (Display Lists) using VAO/VBO.
    /// 
    /// An IPaintTo3DList is built by calling OpenList(), then a series of
    /// Polyline / Triangle / Points / ... calls, then CloseList().
    /// During recording, geometry is accumulated in CPU-side lists.
    /// On CloseList(), the data is uploaded to the GPU in one VAO per
    /// primitive type (lines, triangles, points).
    /// On List() / SelectedList(), we just bind each VAO and call glDrawArrays.
    /// </summary>
    internal class GlBufferList : IPaintTo3DList
    {
        // ── Sub-buffer: one VAO/VBO pair per primitive type ───────────────
        internal readonly struct SubBuffer
        {
            public readonly uint Vao;
            public readonly uint Vbo;
            public readonly uint VertexCount;
            public readonly PrimitiveType Mode;   // Lines, Triangles, Points
            // Only meaningful when Mode == Points; selects the sprite shape.
            public readonly PointSymbol Symbol;

            public SubBuffer(uint vao, uint vbo, uint vertexCount, PrimitiveType mode,
                             PointSymbol symbol = PointSymbol.Dot)
            {
                Vao = vao; Vbo = vbo; VertexCount = vertexCount; Mode = mode; Symbol = symbol;
            }
        }

        // ── CPU-side staging buffers (filled during OpenList … CloseList) ──
        // Each vertex is: position (3 floats) + normal (3 floats) + color (4 floats) = 10 floats
        internal const int FloatsPerVertex = 10;

        private List<float>? _triangleData;   // lit triangles
        private List<float>? _lineData;        // unlit line strips/segments
        // Point data keyed by PointSymbol so each symbol gets its own SubBuffer
        // (multiple Points() calls in one list may use different symbols).
        private Dictionary<PointSymbol, List<float>>? _pointDataBySymbol;

        // Lists that are nested inside this one.
        // Model == null  → inherit parent's model (MakeList grouping)
        // Model != null  → use stored model (captured from PushMultModOp at record time)
        // Color == null  → inherit parent's override color
        // Color != null  → use as uColorOverride (SetColor active at record time)
        private List<(GlBufferList Sub, Matrix4x4? Model, Vector4? Color)>? _subLists;

        // ── GPU-side buffers (filled on CloseList) ─────────────────────────
        private readonly List<SubBuffer> _gpuBuffers = new();
        private GL? _gl;   // kept for Dispose

        // ── State at recording time ────────────────────────────────────────
        internal Vector4 CurrentColor { get; set; } = new Vector4(1, 1, 1, 1);
        internal Vector3 CurrentNormal { get; set; } = Vector3.UnitZ;
        internal bool HasContents { get; private set; }

        // ── IPaintTo3DList ─────────────────────────────────────────────────
        public string Name { get; set; } = string.Empty;
        // Sub-lists that must stay alive as long as this list lives
        // (mirrors the old "keepAlive" pattern)
        public List<IPaintTo3DList> containedSubLists { set { /* kept via _subLists */ } }

        // ──────────────────────────────────────────────────────────────────
        //  Recording API (called by PaintToOpenGLModern while list is open)
        // ──────────────────────────────────────────────────────────────────

        public void BeginRecording()
        {
            _triangleData      = new List<float>(4096);
            _lineData          = new List<float>(2048);
            _pointDataBySymbol = new Dictionary<PointSymbol, List<float>>();
            _subLists          = new List<(GlBufferList, Matrix4x4?, Vector4?)>();
        }

        /// <summary>Append a triangle mesh (indexed) to the recording buffer.</summary>
        public void RecordTriangles(
            ReadOnlySpan<Vector3> vertices,
            ReadOnlySpan<Vector3> normals,
            ReadOnlySpan<int>     indices)
        {
            if (_triangleData == null) return;
            HasContents = true;

            var color = CurrentColor;
            for (int i = 0; i < indices.Length; i++)
            {
                int idx = indices[i];
                var v = vertices[idx];
                var n = normals[idx];
                _triangleData.Add(v.X); _triangleData.Add(v.Y); _triangleData.Add(v.Z);
                _triangleData.Add(n.X); _triangleData.Add(n.Y); _triangleData.Add(n.Z);
                _triangleData.Add(color.X); _triangleData.Add(color.Y);
                _triangleData.Add(color.Z); _triangleData.Add(color.W);
            }
        }

        /// <summary>Append a polyline (line strip) to the recording buffer.</summary>
        public void RecordPolyline(ReadOnlySpan<Vector3> points)
        {
            if (_lineData == null || points.Length < 2) return;
            HasContents = true;

            var color = CurrentColor;
            // Expand line-strip into individual line segments so we can use a
            // single GL_LINES draw call for all polylines in this list.
            for (int i = 0; i < points.Length - 1; i++)
            {
                AppendLineVertex(_lineData, points[i],   color);
                AppendLineVertex(_lineData, points[i+1], color);
            }
        }

        /// <summary>Append a point cloud to the recording buffer for the given symbol.</summary>
        public void RecordPoints(ReadOnlySpan<Vector3> points, PointSymbol symbol = PointSymbol.Dot)
        {
            if (_pointDataBySymbol == null || points.Length == 0) return;
            HasContents = true;

            if (!_pointDataBySymbol.TryGetValue(symbol, out var bucket))
            {
                bucket = new List<float>(256);
                _pointDataBySymbol[symbol] = bucket;
            }

            var color = CurrentColor;
            foreach (var p in points)
            {
                bucket.Add(p.X); bucket.Add(p.Y); bucket.Add(p.Z);
                bucket.Add(0); bucket.Add(0); bucket.Add(1);   // normal unused
                bucket.Add(color.X); bucket.Add(color.Y);
                bucket.Add(color.Z); bucket.Add(color.W);
            }
        }

        /// <summary>Nested list call inside this list.
        /// <paramref name="modelAtRecordTime"/>: null = inherit parent's transform (MakeList).
        /// <paramref name="colorAtRecordTime"/>: null = inherit parent's override color (MakeList);
        ///   non-null = use as uColorOverride so the list renders with the SetColor active at
        ///   record time (text glyphs whose vertex colors were baked with a different color).</summary>
        public void RecordSubList(GlBufferList sub, Matrix4x4? modelAtRecordTime, Vector4? colorAtRecordTime)
        {
            _subLists?.Add((sub, modelAtRecordTime, colorAtRecordTime));
            if (sub.HasContents) HasContents = true;
        }

        // ──────────────────────────────────────────────────────────────────
        //  Upload to GPU
        // ──────────────────────────────────────────────────────────────────

        /// <summary>
        /// Upload all CPU data to the GPU and free the staging buffers.
        /// Must be called from the OpenGL thread (i.e. inside OnOpenGlRender).
        /// </summary>
        public void UploadToGpu(GL gl)
        {
            _gl = gl;
            UploadBuffer(gl, _triangleData, PrimitiveType.Triangles);
            UploadBuffer(gl, _lineData,     PrimitiveType.Lines);
            if (_pointDataBySymbol != null)
                foreach (var (sym, data) in _pointDataBySymbol)
                    UploadBuffer(gl, data, PrimitiveType.Points, sym);

            // Free CPU memory
            _triangleData      = null;
            _lineData          = null;
            _pointDataBySymbol = null;
        }

        private void UploadBuffer(GL gl, List<float>? data, PrimitiveType mode,
                                   PointSymbol symbol = PointSymbol.Dot)
        {
            if (data == null || data.Count == 0) return;

            uint vao = gl.GenVertexArray();
            uint vbo = gl.GenBuffer();

            gl.BindVertexArray(vao);
            gl.BindBuffer(BufferTargetARB.ArrayBuffer, vbo);

            var span = System.Runtime.InteropServices.CollectionsMarshal.AsSpan(data);
            unsafe
            {
                fixed (float* ptr = span)
                    gl.BufferData(BufferTargetARB.ArrayBuffer,
                                  (nuint)(span.Length * sizeof(float)),
                                  ptr, BufferUsageARB.StaticDraw);
            }

            uint stride = FloatsPerVertex * sizeof(float);
            // location 0: position (vec3)
            gl.EnableVertexAttribArray(0);
            gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, stride, 0);
            // location 1: normal (vec3)
            gl.EnableVertexAttribArray(1);
            gl.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, stride, 3 * sizeof(float));
            // location 2: color (vec4)
            gl.EnableVertexAttribArray(2);
            gl.VertexAttribPointer(2, 4, VertexAttribPointerType.Float, false, stride, 6 * sizeof(float));

            gl.BindVertexArray(0);
            gl.BindBuffer(BufferTargetARB.ArrayBuffer, 0);

            uint count = (uint)(data.Count / FloatsPerVertex);
            _gpuBuffers.Add(new SubBuffer(vao, vbo, count, mode, symbol));
        }

        // ──────────────────────────────────────────────────────────────────
        //  Draw
        // ──────────────────────────────────────────────────────────────────

        /// <summary>Draw all geometry stored in this list (and nested sub-lists).
        /// <paramref name="ownModel"/> and <paramref name="ownOverrideColor"/> describe the caller's
        /// current GPU state and are used to restore it after sub-lists are drawn.
        /// <paramref name="setModelAndColor"/> is a callback that re-uploads model + color override
        /// to both shaders; pass null when drawing a list with no sub-lists.</summary>
        /// <param name="prepareDraw">Called before each sub-buffer draw to activate the
        /// correct shader and set buffer-specific uniforms (e.g. point symbol).</param>
        public void Draw(GL gl,
                         Matrix4x4 ownModel = default,
                         Vector4?  ownOverrideColor = null,
                         Action<Matrix4x4, Vector4?>? setModelAndColor = null,
                         Action<SubBuffer>? prepareDraw = null)
        {
            if (_subLists != null && _subLists.Count > 0)
            {
                foreach (var (sub, storedModel, storedColor) in _subLists)
                {
                    var modelToUse = storedModel ?? ownModel;
                    var colorToUse = ownOverrideColor ?? storedColor;
                    setModelAndColor?.Invoke(modelToUse, colorToUse);
                    sub.Draw(gl, modelToUse, colorToUse, setModelAndColor, prepareDraw);
                }
                setModelAndColor?.Invoke(ownModel, ownOverrideColor);
            }

            foreach (var buf in _gpuBuffers)
            {
                prepareDraw?.Invoke(buf);
                gl.BindVertexArray(buf.Vao);
                gl.DrawArrays(buf.Mode, 0, buf.VertexCount);
            }
            gl.BindVertexArray(0);
        }

        // ──────────────────────────────────────────────────────────────────
        //  Helpers
        // ──────────────────────────────────────────────────────────────────

        private static void AppendLineVertex(List<float> buf, Vector3 p, Vector4 color)
        {
            buf.Add(p.X); buf.Add(p.Y); buf.Add(p.Z);
            buf.Add(0); buf.Add(0); buf.Add(1);   // dummy normal
            buf.Add(color.X); buf.Add(color.Y); buf.Add(color.Z); buf.Add(color.W);
        }

        // ──────────────────────────────────────────────────────────────────
        //  IDisposable / Cleanup
        // ──────────────────────────────────────────────────────────────────

        public void Dispose()
        {
            if (_gl == null) return;
            foreach (var buf in _gpuBuffers)
            {
                _gl.DeleteVertexArray(buf.Vao);
                _gl.DeleteBuffer(buf.Vbo);
            }
            _gpuBuffers.Clear();
            _gl = null;

            if (_subLists != null)
            {
                // Sub-lists are owned by MakeList – do not double-free them here.
                _subLists = null;
            }
        }
    }
}
