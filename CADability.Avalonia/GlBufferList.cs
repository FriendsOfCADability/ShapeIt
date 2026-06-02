using System;
using System.Collections.Generic;
using System.Numerics;
using Silk.NET.OpenGL;
using CADability;
using CADability.GeoObject;

namespace CADability.Avalonia
{
    internal class GlBufferList : IPaintTo3DList
    {
        internal readonly struct SubBuffer
        {
            public readonly uint Vao;
            public readonly uint Vbo;
            public readonly uint VertexCount;
            public readonly PrimitiveType Mode;
            public readonly PointSymbol Symbol;

            public SubBuffer(uint vao, uint vbo, uint vertexCount, PrimitiveType mode,
                             PointSymbol symbol = PointSymbol.Dot)
            {
                Vao = vao; Vbo = vbo; VertexCount = vertexCount; Mode = mode; Symbol = symbol;
            }
        }

        // A rectangular bitmap drawn as a textured quad. Corners are stored in the
        // list's local coordinate system; the model matrix is applied at draw time
        // (same convention as the geometry buffers). The texture is owned by the painter.
        internal readonly struct TexturedQuad
        {
            public readonly Vector3 P0, P1, P2, P3;
            public readonly uint Texture;
            public TexturedQuad(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, uint texture)
            {
                P0 = p0; P1 = p1; P2 = p2; P3 = p3; Texture = texture;
            }
        }

        // Each vertex: position (3) + normal (3) + color (4) = 10 floats
        internal const int FloatsPerVertex = 10;

        private List<float>? _triangleData;
        private List<float>? _lineData;
        private Dictionary<PointSymbol, List<float>>? _pointDataBySymbol;
        private List<(GlBufferList Sub, Matrix4x4? Model, Vector4? Color)>? _subLists;
        private List<TexturedQuad>? _quads;

        private readonly List<SubBuffer> _gpuBuffers = new();
        private GL? _gl;

        internal Vector4 CurrentColor { get; set; } = new Vector4(1, 1, 1, 1);
        internal Vector3 CurrentNormal { get; set; } = Vector3.UnitZ;
        internal bool HasContents { get; private set; }

        public string Name { get; set; } = string.Empty;
        public List<IPaintTo3DList> containedSubLists { set { /* kept via _subLists */ } }

        public void BeginRecording()
        {
            _triangleData      = new List<float>(4096);
            _lineData          = new List<float>(2048);
            _pointDataBySymbol = new Dictionary<PointSymbol, List<float>>();
            _subLists          = new List<(GlBufferList, Matrix4x4?, Vector4?)>();
        }

        public void RecordTexturedQuad(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, uint texture)
        {
            _quads ??= new List<TexturedQuad>();
            HasContents = true;
            _quads.Add(new TexturedQuad(p0, p1, p2, p3, texture));
        }

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

        public void RecordPolyline(ReadOnlySpan<Vector3> points)
        {
            if (_lineData == null || points.Length < 2) return;
            HasContents = true;

            var color = CurrentColor;
            for (int i = 0; i < points.Length - 1; i++)
            {
                AppendLineVertex(_lineData, points[i],   color);
                AppendLineVertex(_lineData, points[i+1], color);
            }
        }

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
                bucket.Add(0); bucket.Add(0); bucket.Add(1);
                bucket.Add(color.X); bucket.Add(color.Y);
                bucket.Add(color.Z); bucket.Add(color.W);
            }
        }

        public void RecordSubList(GlBufferList sub, Matrix4x4? modelAtRecordTime, Vector4? colorAtRecordTime)
        {
            _subLists?.Add((sub, modelAtRecordTime, colorAtRecordTime));
            if (sub.HasContents) HasContents = true;
        }

        public void UploadToGpu(GL gl)
        {
            _gl = gl;
            UploadBuffer(gl, _triangleData, PrimitiveType.Triangles);
            UploadBuffer(gl, _lineData,     PrimitiveType.Lines);
            if (_pointDataBySymbol != null)
                foreach (var (sym, data) in _pointDataBySymbol)
                    UploadBuffer(gl, data, PrimitiveType.Points, sym);

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
            gl.EnableVertexAttribArray(0);
            gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, stride, 0);
            gl.EnableVertexAttribArray(1);
            gl.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, stride, 3 * sizeof(float));
            gl.EnableVertexAttribArray(2);
            gl.VertexAttribPointer(2, 4, VertexAttribPointerType.Float, false, stride, 6 * sizeof(float));

            gl.BindVertexArray(0);
            gl.BindBuffer(BufferTargetARB.ArrayBuffer, 0);

            uint count = (uint)(data.Count / FloatsPerVertex);
            _gpuBuffers.Add(new SubBuffer(vao, vbo, count, mode, symbol));
        }

        public void Draw(GL gl,
                         Matrix4x4 ownModel = default,
                         Vector4?  ownOverrideColor = null,
                         Action<Matrix4x4, Vector4?>? setModelAndColor = null,
                         Action<SubBuffer>? prepareDraw = null,
                         Action<TexturedQuad>? drawTexturedQuad = null)
        {
            if (_subLists != null && _subLists.Count > 0)
            {
                foreach (var (sub, storedModel, storedColor) in _subLists)
                {
                    var modelToUse = storedModel ?? ownModel;
                    var colorToUse = ownOverrideColor ?? storedColor;
                    setModelAndColor?.Invoke(modelToUse, colorToUse);
                    sub.Draw(gl, modelToUse, colorToUse, setModelAndColor, prepareDraw, drawTexturedQuad);
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

            // Textured quads (bitmaps) are drawn by the painter via the callback,
            // with the model matrix already set for this list by setModelAndColor.
            if (_quads != null && drawTexturedQuad != null)
                foreach (var q in _quads)
                    drawTexturedQuad(q);
        }

        private static void AppendLineVertex(List<float> buf, Vector3 p, Vector4 color)
        {
            buf.Add(p.X); buf.Add(p.Y); buf.Add(p.Z);
            buf.Add(0); buf.Add(0); buf.Add(1);
            buf.Add(color.X); buf.Add(color.Y); buf.Add(color.Z); buf.Add(color.W);
        }

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
            _subLists = null;
            _quads = null;   // textures are owned/freed by the painter
        }
    }
}
