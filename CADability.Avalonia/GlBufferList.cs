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
            // The line width of this buffer. For Mode == Lines it is the
            // glLineWidth to apply; for a thick-line buffer (IsThickLine) it is
            // the pixel width fed to the thick-line shader.
            public readonly float LineWidth;
            // True when this is a wide line stored as triangle quads (Mode ==
            // Triangles) to be drawn with the screen-space thick-line shader,
            // because glLineWidth > 1 is not supported under ANGLE/Direct3D.
            public readonly bool IsThickLine;

            public SubBuffer(uint vao, uint vbo, uint vertexCount, PrimitiveType mode,
                             PointSymbol symbol = PointSymbol.Dot, float lineWidth = 1f,
                             bool isThickLine = false)
            {
                Vao = vao; Vbo = vbo; VertexCount = vertexCount; Mode = mode;
                Symbol = symbol; LineWidth = lineWidth; IsThickLine = isThickLine;
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
        // Thick-line vertex: start (3) + end (3) + color (4) + side (1) + endFlag (1) = 12 floats
        internal const int ThickLineFloatsPerVertex = 12;
        // Lines at or below this width use plain GL_LINES (glLineWidth == 1);
        // wider lines are expanded to screen-space quads.
        private const float ThinLineMaxWidth = 1f;

        private List<float>? _triangleData;
        // Unlit line segments, keyed by line width. glLineWidth is global GL state
        // and is not part of the vertex data, so lines of different widths must be
        // stored (and later drawn) as separate buffers.
        private Dictionary<float, List<float>>? _lineDataByWidth;
        // Wide lines expanded to screen-space quads, keyed by pixel width.
        private Dictionary<float, List<float>>? _thickLineDataByWidth;
        private Dictionary<PointSymbol, List<float>>? _pointDataBySymbol;
        private List<(GlBufferList Sub, Matrix4x4? Model, Vector4? Color)>? _subLists;
        private List<TexturedQuad>? _quads;

        private readonly List<SubBuffer> _gpuBuffers = new();
        private GL? _gl;

        internal Vector4 CurrentColor { get; set; } = new Vector4(1, 1, 1, 1);
        internal Vector3 CurrentNormal { get; set; } = Vector3.UnitZ;
        // Line width active at record time; applied to polylines recorded next.
        internal float CurrentLineWidth { get; set; } = 1f;
        internal bool HasContents { get; private set; }

        public string Name { get; set; } = string.Empty;
        public List<IPaintTo3DList> containedSubLists { set { /* kept via _subLists */ } }

        public void BeginRecording()
        {
            _triangleData        = new List<float>(4096);
            _lineDataByWidth     = new Dictionary<float, List<float>>();
            _thickLineDataByWidth = new Dictionary<float, List<float>>();
            _pointDataBySymbol   = new Dictionary<PointSymbol, List<float>>();
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
            if (_lineDataByWidth == null || _thickLineDataByWidth == null || points.Length < 2)
                return;
            HasContents = true;

            var color = CurrentColor;

            if (CurrentLineWidth > ThinLineMaxWidth)
            {
                // Wide line: expand each segment into a screen-space quad (two
                // triangles) and bucket by width. glLineWidth > 1 is unavailable
                // under ANGLE/Direct3D, so the width is applied in the shader.
                if (!_thickLineDataByWidth.TryGetValue(CurrentLineWidth, out var thick))
                {
                    thick = new List<float>(points.Length * 6 * ThickLineFloatsPerVertex);
                    _thickLineDataByWidth[CurrentLineWidth] = thick;
                }
                for (int i = 0; i < points.Length - 1; i++)
                    AppendThickSegment(thick, points[i], points[i + 1], color);
                return;
            }

            // Thin line: plain GL_LINES, bucketed by width (== 1).
            if (!_lineDataByWidth.TryGetValue(CurrentLineWidth, out var lineData))
            {
                lineData = new List<float>(2048);
                _lineDataByWidth[CurrentLineWidth] = lineData;
            }
            for (int i = 0; i < points.Length - 1; i++)
            {
                AppendLineVertex(lineData, points[i],   color);
                AppendLineVertex(lineData, points[i+1], color);
            }
        }

        // Emits 6 vertices (two triangles) forming a quad for one wide segment.
        // Each vertex carries both endpoints; the shader offsets it perpendicular
        // to the segment by ± half the width. Layout per vertex:
        //   start(3) end(3) color(4) side(1) endFlag(1)
        private static void AppendThickSegment(List<float> buf, Vector3 a, Vector3 b, Vector4 color)
        {
            // Quad corners: (a,-) (a,+) (b,+) (b,-) → triangles (0,1,2) and (0,2,3).
            void V(float side, float endFlag)
            {
                buf.Add(a.X); buf.Add(a.Y); buf.Add(a.Z);
                buf.Add(b.X); buf.Add(b.Y); buf.Add(b.Z);
                buf.Add(color.X); buf.Add(color.Y); buf.Add(color.Z); buf.Add(color.W);
                buf.Add(side); buf.Add(endFlag);
            }
            V(-1f, 0f); V(+1f, 0f); V(+1f, 1f);   // triangle 1
            V(-1f, 0f); V(+1f, 1f); V(-1f, 1f);   // triangle 2
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
            if (_lineDataByWidth != null)
                foreach (var (width, data) in _lineDataByWidth)
                    UploadBuffer(gl, data, PrimitiveType.Lines, lineWidth: width);
            if (_thickLineDataByWidth != null)
                foreach (var (width, data) in _thickLineDataByWidth)
                    UploadThickLineBuffer(gl, data, width);
            if (_pointDataBySymbol != null)
                foreach (var (sym, data) in _pointDataBySymbol)
                    UploadBuffer(gl, data, PrimitiveType.Points, sym);

            _triangleData         = null;
            _lineDataByWidth      = null;
            _thickLineDataByWidth = null;
            _pointDataBySymbol    = null;
        }

        private void UploadBuffer(GL gl, List<float>? data, PrimitiveType mode,
                                   PointSymbol symbol = PointSymbol.Dot, float lineWidth = 1f)
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
            _gpuBuffers.Add(new SubBuffer(vao, vbo, count, mode, symbol, lineWidth));
        }

        // Uploads wide-line quad data with its own vertex layout (5 attributes)
        // and registers it as a thick-line triangle buffer.
        private void UploadThickLineBuffer(GL gl, List<float>? data, float lineWidth)
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

            uint stride = ThickLineFloatsPerVertex * sizeof(float);
            // location 0: segment start (vec3)
            gl.EnableVertexAttribArray(0);
            gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, stride, 0);
            // location 1: segment end (vec3)
            gl.EnableVertexAttribArray(1);
            gl.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, stride, 3 * sizeof(float));
            // location 2: color (vec4)
            gl.EnableVertexAttribArray(2);
            gl.VertexAttribPointer(2, 4, VertexAttribPointerType.Float, false, stride, 6 * sizeof(float));
            // location 3: side (float)
            gl.EnableVertexAttribArray(3);
            gl.VertexAttribPointer(3, 1, VertexAttribPointerType.Float, false, stride, 10 * sizeof(float));
            // location 4: endFlag (float)
            gl.EnableVertexAttribArray(4);
            gl.VertexAttribPointer(4, 1, VertexAttribPointerType.Float, false, stride, 11 * sizeof(float));

            gl.BindVertexArray(0);
            gl.BindBuffer(BufferTargetARB.ArrayBuffer, 0);

            uint count = (uint)(data.Count / ThickLineFloatsPerVertex);
            _gpuBuffers.Add(new SubBuffer(vao, vbo, count, PrimitiveType.Triangles,
                                          lineWidth: lineWidth, isThickLine: true));
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
                // glLineWidth is global GL state; set it for this line buffer and
                // restore the default afterwards so it never leaks to other lines
                // (recorded buffers without an explicit width, immediate draws or
                // the next frame).
                bool widthSet = buf.Mode == PrimitiveType.Lines && buf.LineWidth != 1f;
                if (widthSet) gl.LineWidth(buf.LineWidth);
                gl.BindVertexArray(buf.Vao);
                gl.DrawArrays(buf.Mode, 0, buf.VertexCount);
                if (widthSet) gl.LineWidth(1f);
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
