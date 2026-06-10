using System;
using System.Collections.Generic;
using System.Numerics;
using CADability;
using CADability.GeoObject;

namespace ShapeIt.Browser
{
    /// <summary>
    /// Browser equivalent of CADability.Avalonia's GlBufferList: a display list that
    /// records interleaved vertex data (pos+normal+color, 10 floats) on the CPU and,
    /// on Upload(), hands the buffers to the WebGL2 renderer. Replay is driven by
    /// <see cref="PaintToWebGL"/>, which also walks sub-lists.
    /// </summary>
    internal sealed class WebGLList : IPaintTo3DList
    {
        private const int FloatsPerVertex = 10;

        private List<double>? _tri = new(4096);
        private List<double>? _line = new(2048);
        private Dictionary<PointSymbol, List<double>>? _pointsBySymbol = new();
        private List<(double[] Corners, int TexId)>? _quads;
        private readonly List<(WebGLList Sub, Matrix4x4? Model, Vector4? Color)> _subLists = new();

        public int GlId { get; private set; } = -1;
        public bool HasContents { get; private set; }
        public Vector4 CurrentColor { get; set; } = new(1, 1, 1, 1);

        public string Name { get; set; } = string.Empty;
        public List<IPaintTo3DList> containedSubLists { set { /* kept via _subLists */ } }

        public IReadOnlyList<(WebGLList Sub, Matrix4x4? Model, Vector4? Color)> SubLists => _subLists;

        public void RecordTriangles(ReadOnlySpan<Vector3> vertices, ReadOnlySpan<Vector3> normals, ReadOnlySpan<int> indices)
        {
            if (_tri == null) return;
            var c = CurrentColor;
            for (int i = 0; i < indices.Length; i++)
            {
                int idx = indices[i];
                var v = vertices[idx];
                var n = normals[idx];
                _tri.Add(v.X); _tri.Add(v.Y); _tri.Add(v.Z);
                _tri.Add(n.X); _tri.Add(n.Y); _tri.Add(n.Z);
                _tri.Add(c.X); _tri.Add(c.Y); _tri.Add(c.Z); _tri.Add(c.W);
            }
            if (indices.Length > 0) HasContents = true;
        }

        public void RecordPolyline(ReadOnlySpan<Vector3> points)
        {
            if (_line == null || points.Length < 2) return;
            var c = CurrentColor;
            for (int i = 0; i < points.Length - 1; i++)
            {
                AppendLineVertex(points[i], c);
                AppendLineVertex(points[i + 1], c);
            }
            HasContents = true;
        }

        public void RecordPoints(ReadOnlySpan<Vector3> points, PointSymbol symbol)
        {
            if (_pointsBySymbol == null || points.Length == 0) return;
            if (!_pointsBySymbol.TryGetValue(symbol, out var bucket))
            {
                bucket = new List<double>(256);
                _pointsBySymbol[symbol] = bucket;
            }
            var c = CurrentColor;
            foreach (var p in points)
            {
                bucket.Add(p.X); bucket.Add(p.Y); bucket.Add(p.Z);
                bucket.Add(0); bucket.Add(0); bucket.Add(1);
                bucket.Add(c.X); bucket.Add(c.Y); bucket.Add(c.Z); bucket.Add(c.W);
            }
            HasContents = true;
        }

        /// <summary>Record a textured quad (bitmap/text). Corners are baked in list-local space.</summary>
        public void RecordTexturedQuad(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, int texId)
        {
            _quads ??= new List<(double[], int)>();
            _quads.Add((new double[]
            {
                p0.X, p0.Y, p0.Z, p1.X, p1.Y, p1.Z,
                p2.X, p2.Y, p2.Z, p3.X, p3.Y, p3.Z
            }, texId));
            HasContents = true;
        }

        public void RecordSubList(WebGLList sub, Matrix4x4? model, Vector4? color)
        {
            _subLists.Add((sub, model, color));
            if (sub.HasContents) HasContents = true;
        }

        private void AppendLineVertex(Vector3 p, Vector4 c)
        {
            _line!.Add(p.X); _line.Add(p.Y); _line.Add(p.Z);
            _line.Add(0); _line.Add(0); _line.Add(1);
            _line.Add(c.X); _line.Add(c.Y); _line.Add(c.Z); _line.Add(c.W);
        }

        /// <summary>Upload the recorded geometry to the WebGL2 renderer (once).</summary>
        public void Upload()
        {
            var tri = _tri ?? new List<double>();
            var line = _line ?? new List<double>();
            bool hasPoints = _pointsBySymbol != null && _pointsBySymbol.Count > 0;
            bool hasQuads = _quads != null && _quads.Count > 0;

            if (tri.Count > 0 || line.Count > 0 || hasPoints || hasQuads)
            {
                GlId = WebGLInterop.CreateList(tri.ToArray(), line.ToArray());
                if (hasPoints)
                    foreach (var (sym, data) in _pointsBySymbol!)
                        WebGLInterop.AddPointsToList(GlId, (int)sym, data.ToArray());
                if (hasQuads)
                    foreach (var (corners, texId) in _quads!)
                        WebGLInterop.AddQuadToList(GlId, corners, texId);
            }
            _tri = null;
            _line = null;
            _pointsBySymbol = null;
            _quads = null;
        }

        public void Dispose()
        {
            if (GlId >= 0) { WebGLInterop.DeleteList(GlId); GlId = -1; }
        }
    }
}
