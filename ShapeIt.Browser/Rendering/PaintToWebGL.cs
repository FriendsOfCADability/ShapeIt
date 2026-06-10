using System;
using System.Collections.Generic;
using System.Numerics;
using CADability;
using CADability.Attribute;
using CADability.GeoObject;
using CADability.Substitutes;

namespace ShapeIt.Browser
{
    /// <summary>
    /// IPaintTo3D backend that renders into a WebGL2 canvas (via WebGLInterop), mirroring
    /// CADability.Avalonia's PaintToOpenGL but without Silk.NET / a platform GL context.
    /// Camera math, vertex format, shaders and uniform conventions are kept identical so
    /// the result matches the desktop view. Surfaces (lit triangles), edges (lines),
    /// display lists incl. sub-lists, the model-matrix stack and projection are implemented;
    /// text, bitmaps, points and 2-D overlays are stubbed for now.
    /// </summary>
    internal sealed class PaintToWebGL : IPaintTo3D
    {
        private int _width = 1, _height = 1;
        private Matrix4x4 _projection = Matrix4x4.Identity;
        private readonly Matrix4x4 _view = Matrix4x4.Identity;
        private Matrix4x4 _model = Matrix4x4.Identity;
        private readonly Stack<Matrix4x4> _modelStack = new();

        private Vector3 _lightDir = Vector3.UnitZ;
        private double[] _lightArr = { 0, 0, 1 };
        private double _pixelToWorld = 1.0;
        private double _precision;

        private Color _backgroundColor = Color.White;
        private Color _currentColor = Color.Black;
        private bool _colorOverride;
        private Color _overrideColor;

        private bool _paintSurfaces = true;
        private bool _paintEdges = true;
        private bool _paintSurfaceEdges = true;
        private bool _useLineWidth;
        private bool _selectMode;
        private Color _selectColor = Color.FromArgb(180, 180, 255);
        private bool _delayText, _delayAll, _triangulateText, _dontRecalc;
        private bool _useZBuffer = true;
        private bool _blending = true;

        // GL-state stack (UseZBuffer/Blending), mirrors PaintToOpenGL's GlState stack.
        private readonly Stack<(bool ZBuffer, bool Blending)> _stateStack = new();

        // Cached WebGL textures for prepared bitmaps (texId from the renderer).
        private readonly Dictionary<Bitmap, int> _bitmapTextures = new();

        private WebGLList? _recordingList;

        private Vector4 CurrentColorVec => ColorToVec4(_currentColor);

        // ── lifecycle ────────────────────────────────────────────────────
        void IPaintTo3D.MakeCurrent() { }
        void IPaintTo3D.Resize(int width, int height) { _width = Math.Max(1, width); _height = Math.Max(1, height); }
        void IPaintTo3D.Dispose() { }
        void IPaintTo3D.FinishPaint() { }
        void IPaintTo3D.FreeUnusedLists() { }

        // ── properties ───────────────────────────────────────────────────
        bool IPaintTo3D.PaintSurfaces => _paintSurfaces;
        bool IPaintTo3D.PaintEdges => _paintEdges;
        bool IPaintTo3D.PaintSurfaceEdges { get => _paintSurfaceEdges; set => _paintSurfaceEdges = value; }
        bool IPaintTo3D.UseLineWidth { get => _useLineWidth; set => _useLineWidth = value; }
        double IPaintTo3D.Precision { get => _precision; set => _precision = value; }
        double IPaintTo3D.PixelToWorld => _pixelToWorld;
        bool IPaintTo3D.SelectMode { get => _selectMode; set => _selectMode = value; }
        Color IPaintTo3D.SelectColor { get => _selectColor; set => _selectColor = value; }
        bool IPaintTo3D.DelayText { get => _delayText; set => _delayText = value; }
        bool IPaintTo3D.DelayAll { get => _delayAll; set => _delayAll = value; }
        bool IPaintTo3D.TriangulateText { get => _triangulateText; set => _triangulateText = value; }
        bool IPaintTo3D.DontRecalcTriangulation { get => _dontRecalc; set => _dontRecalc = value; }
        bool IPaintTo3D.IsBitmap => false;
        PaintCapabilities IPaintTo3D.Capabilities =>
            PaintCapabilities.Standard | PaintCapabilities.ZoomIndependentDisplayList;
        IDisposable IPaintTo3D.FacesBehindEdgesOffset => new FaceOffsetScope(this);

        // ── frame start ──────────────────────────────────────────────────
        void IPaintTo3D.Clear(Color background)
        {
            _backgroundColor = background;
            WebGLInterop.BeginFrame(background.R / 255.0, background.G / 255.0, background.B / 255.0);
        }
        void IPaintTo3D.AvoidColor(Color color) => _backgroundColor = color;

        // ── projection (identical to PaintToOpenGL) ──────────────────────
        void IPaintTo3D.SetProjection(Projection projection, BoundingBox boundingCube)
        {
            double[,] m = projection.GetOpenGLProjection(0, _width, 0, _height, boundingCube);

            _projection = new Matrix4x4(
                (float)m[0, 0], (float)m[1, 0], (float)m[2, 0], (float)m[3, 0],
                (float)m[0, 1], (float)m[1, 1], (float)m[2, 1], (float)m[3, 1],
                (float)m[0, 2], (float)m[1, 2], (float)m[2, 2], (float)m[3, 2],
                (float)m[0, 3], (float)m[1, 3], (float)m[2, 3], (float)m[3, 3]);

            _model = Matrix4x4.Identity;

            if (Matrix4x4.Invert(_projection, out var inv))
            {
                var p0 = Vector4.Transform(new Vector4(-1f, -1f, 0f, 1f), inv);
                var p1 = Vector4.Transform(new Vector4(-1f + 2f / _width, -1f, 0f, 1f), inv);
                if (p0.W != 0 && p1.W != 0)
                    _pixelToWorld = Vector3.Distance(
                        new Vector3(p0.X / p0.W, p0.Y / p0.W, p0.Z / p0.W),
                        new Vector3(p1.X / p1.W, p1.Y / p1.W, p1.Z / p1.W));
            }

            GeoVector vd = projection.Direction;
            _lightDir = Vector3.Normalize(new Vector3((float)vd.x, (float)vd.y, (float)vd.z));
            _lightArr = new double[] { _lightDir.X, _lightDir.Y, _lightDir.Z };
            _useLineWidth = projection.UseLineWidth;
        }

        // ── color ────────────────────────────────────────────────────────
        void IPaintTo3D.SetColor(Color color, int lockColor)
        {
            if (!_colorOverride)
            {
                if (color.R == _backgroundColor.R && color.G == _backgroundColor.G && color.B == _backgroundColor.B)
                    color = (color.R + color.G + color.B < 3 * 128) ? Color.White : Color.Black;
                _currentColor = color;
            }
            if (lockColor == 1) { _colorOverride = true; _overrideColor = color; }
            else if (lockColor == -1) _colorOverride = false;

            if (_recordingList != null) _recordingList.CurrentColor = CurrentColorVec;
        }
        void IPaintTo3D.SetLinePattern(LinePattern pattern) { }

        // ── geometry ─────────────────────────────────────────────────────
        void IPaintTo3D.Triangle(GeoPoint[] vertex, GeoVector[] normals, int[] indextriples)
        {
            if (indextriples.Length == 0) return;
            var verts = GeoPointsToVec3(vertex);
            var norms = GeoVectorsToVec3(normals);
            if (_recordingList != null)
                _recordingList.RecordTriangles(verts, norms, indextriples);
            else
                DrawImmediate(rec => rec.RecordTriangles(verts, norms, indextriples));
        }

        void IPaintTo3D.Polyline(GeoPoint[] points)
        {
            if (points.Length < 2) return;
            var verts = GeoPointsToVec3(points);
            if (_recordingList != null)
                _recordingList.RecordPolyline(verts);
            else
                DrawImmediate(rec => rec.RecordPolyline(verts));
        }

        void IPaintTo3D.FilledPolyline(GeoPoint[] points)
        {
            // Triangle-fan fill (unlit), mirroring PaintToOpenGL.FilledPolyline.
            if (points.Length < 3) return;
            var color = CurrentColorVec;
            var data = new List<double>(points.Length * 3 * 10);
            var v0 = ToVec3(points[0]);
            for (int i = 1; i < points.Length - 1; i++)
            {
                AppendUnlit(data, v0, color);
                AppendUnlit(data, ToVec3(points[i]), color);
                AppendUnlit(data, ToVec3(points[i + 1]), color);
            }
            var mvp = _model * _view * _projection;
            WebGLInterop.DrawTriangles(Flatten(mvp), data.ToArray(), Array.Empty<double>());
        }

        void IPaintTo3D.Points(GeoPoint[] points, float size, PointSymbol pointSymbol)
        {
            if (points.Length == 0) return;
            var verts = GeoPointsToVec3(points);
            if (_recordingList != null)
            {
                _recordingList.RecordPoints(verts, pointSymbol);
                return;
            }
            var color = CurrentColorVec;
            var data = new List<double>(points.Length * 10);
            foreach (var p in verts) AppendUnlit(data, p, color);
            var mvp = _model * _view * _projection;
            WebGLInterop.DrawPoints(Flatten(mvp), data.ToArray(), Array.Empty<double>(), (int)pointSymbol);
        }

        // ── display lists ────────────────────────────────────────────────
        void IPaintTo3D.OpenList(string name)
        {
            if (_recordingList != null) throw new InvalidOperationException("Nested lists are not allowed.");
            _recordingList = new WebGLList { Name = name ?? string.Empty, CurrentColor = CurrentColorVec };
        }

        IPaintTo3DList IPaintTo3D.CloseList()
        {
            var list = _recordingList;
            _recordingList = null;
            if (list == null || !list.HasContents) { list?.Dispose(); return null!; }
            list.Upload();
            return list;
        }

        IPaintTo3DList IPaintTo3D.MakeList(List<IPaintTo3DList> sublists)
        {
            var container = new WebGLList { Name = "_composite" };
            bool any = false;
            foreach (var sub in sublists)
                if (sub is WebGLList wl) { container.RecordSubList(wl, null, null); any = true; }
            if (!any) { container.Dispose(); return null!; }
            container.Upload();
            return container;
        }

        void IPaintTo3D.List(IPaintTo3DList paintThisList)
        {
            if (paintThisList is not WebGLList wl) return;
            if (_recordingList != null) { _recordingList.RecordSubList(wl, _model, CurrentColorVec); return; }
            DrawList(wl, _model, null);
        }

        void IPaintTo3D.SelectedList(IPaintTo3DList paintThisList, int wobbleRadius)
        {
            if (paintThisList is not WebGLList wl) return;
            if (wobbleRadius <= 0)
            {
                // Mirror PaintToOpenGL: pull the selection slightly toward the viewer
                // (along the light/view direction) so selected curves are not hidden
                // behind the faces they belong to.
                var saved = _model;
                _model = Matrix4x4.CreateTranslation(_lightDir * (float)(-2.0 * _pixelToWorld)) * _model;
                DrawList(wl, _model, ColorToVec4(_selectColor));
                _model = saved;
            }
            else DrawList(wl, _model, ColorToVec4(_selectColor));
        }

        private void DrawList(WebGLList wl, Matrix4x4 model, Vector4? overrideColor)
        {
            foreach (var (sub, subModel, subColor) in wl.SubLists)
                DrawList(sub, subModel ?? model, overrideColor ?? subColor);

            if (wl.GlId >= 0)
            {
                var mvp = model * _view * _projection;
                WebGLInterop.DrawList(wl.GlId, Flatten(mvp), Flatten(model), _lightArr, OvrArr(overrideColor));
            }
        }

        private void DrawImmediate(Action<WebGLList> record)
        {
            var tmp = new WebGLList { CurrentColor = CurrentColorVec };
            record(tmp);
            if (!tmp.HasContents) return;
            tmp.Upload();
            DrawList(tmp, _model, null);
            tmp.Dispose();
        }

        // ── model-matrix stack ───────────────────────────────────────────
        void IPaintTo3D.PushMultModOp(ModOp mm) { _modelStack.Push(_model); _model = ModOpToMatrix4x4(mm) * _model; }
        void IPaintTo3D.PopModOp() { if (_modelStack.Count > 0) _model = _modelStack.Pop(); }

        void IPaintTo3D.PaintFaces(PaintTo3D.PaintMode paintMode)
        {
            _model = Matrix4x4.Identity;
            switch (paintMode)
            {
                case PaintTo3D.PaintMode.FacesOnly:
                    _model = Matrix4x4.CreateTranslation(_lightDir * (float)(2.0 * _pixelToWorld));
                    _paintSurfaces = true; _paintEdges = false; break;
                case PaintTo3D.PaintMode.CurvesOnly:
                    _paintSurfaces = false; _paintEdges = true; break;
                default:
                    _paintSurfaces = true; _paintEdges = true; break;
            }
        }

        // ── GL state (mirror PaintToOpenGL.UseZBuffer/Blending/SetLineWidth) ──
        void IPaintTo3D.UseZBuffer(bool use)
        {
            _useZBuffer = use;
            WebGLInterop.SetDepthTest(use);
        }

        void IPaintTo3D.Blending(bool on)
        {
            _blending = on;
            WebGLInterop.SetBlend(on);
        }

        // ── 2-D overlays (orthographic over pixel space, like PaintToOpenGL) ──
        void IPaintTo3D.Line2D(int sx, int sy, int ex, int ey)
        {
            var mvp = Matrix4x4.CreateOrthographicOffCenter(0, _width, _height, 0, -1, 1);
            var color = CurrentColorVec;
            var data = new List<double>(20);
            AppendUnlit(data, new Vector3(sx, sy, 0), color);
            AppendUnlit(data, new Vector3(ex, ey, 0), color);
            WebGLInterop.DrawLines(Flatten(mvp), data.ToArray(), Array.Empty<double>());
        }

        void IPaintTo3D.Line2D(PointF p1, PointF p2)
            => (this as IPaintTo3D).Line2D((int)p1.X, (int)p1.Y, (int)p2.X, (int)p2.Y);

        void IPaintTo3D.FillRect2D(PointF p1, PointF p2)
        {
            var mvp = Matrix4x4.CreateOrthographicOffCenter(0, _width, _height, 0, -1, 1);
            var color = CurrentColorVec;
            var data = new List<double>(60);
            AppendUnlit(data, new Vector3(p1.X, p1.Y, 0), color);
            AppendUnlit(data, new Vector3(p1.X, p2.Y, 0), color);
            AppendUnlit(data, new Vector3(p2.X, p2.Y, 0), color);
            AppendUnlit(data, new Vector3(p1.X, p1.Y, 0), color);
            AppendUnlit(data, new Vector3(p2.X, p2.Y, 0), color);
            AppendUnlit(data, new Vector3(p2.X, p1.Y, 0), color);
            WebGLInterop.DrawTriangles(Flatten(mvp), data.ToArray(), Array.Empty<double>());
        }

        void IPaintTo3D.Point2D(int x, int y)
        {
            // A single 2-D point: render as a 1px filled rect in pixel space.
            (this as IPaintTo3D).FillRect2D(new PointF(x, y), new PointF(x + 1, y + 1));
        }

        // ── Text (browser-pragmatic: rasterize to a 2-D canvas, draw as a quad) ──
        void IPaintTo3D.PrepareText(string fontName, string textString, object fontStyle) { /* JS rasterizes on demand */ }

        void IPaintTo3D.Text(GeoVector lineDirection, GeoVector glyphDirection, GeoPoint location,
                             string fontName, string textString, object fontStyle,
                             Text.AlignMode alignment, Text.LineAlignMode lineAlignment)
        {
            if (string.IsNullOrEmpty(textString)) return;

            // Up vector = glyphDirection (length is the glyph/em height in world units).
            // Right vector = lineDirection normalised, later scaled to the text aspect ratio.
            var up = new Vector3((float)glyphDirection.x, (float)glyphDirection.y, (float)glyphDirection.z);
            var lineVec = new Vector3((float)lineDirection.x, (float)lineDirection.y, (float)lineDirection.z);
            float glyphHeight = up.Length();
            float lineLen = lineVec.Length();
            if (glyphHeight < 1e-9f || lineLen < 1e-9f) return;
            var rightUnit = lineVec / lineLen;

            // Rasterize on the JS side; pick a pixel font size for crispness.
            const double fontPx = 48.0;
            var c = _currentColor;
            double[] info = WebGLInterop.RasterizeText(textString, fontName ?? "sans-serif",
                fontPx, c.R, c.G, c.B, c.A);
            if (info.Length < 3) return;
            int texId = (int)info[0];
            double pxW = info[1], pxH = info[2];
            if (pxW <= 0 || pxH <= 0) { WebGLInterop.DeleteTexture(texId); return; }

            // World height matches the glyph em height; width keeps the raster aspect.
            float worldH = glyphHeight;
            float worldW = (float)(pxW / pxH) * glyphHeight;
            var rightVec = rightUnit * worldW;
            var upVec = up;

            var origin = new Vector3((float)location.x, (float)location.y, (float)location.z);

            // Alignment offsets (best-effort): shift the origin so 'location' means the
            // requested anchor of the text box.
            switch (lineAlignment)
            {
                case Text.LineAlignMode.Center: origin -= rightVec * 0.5f; break;
                case Text.LineAlignMode.Right: origin -= rightVec; break;
            }
            switch (alignment)
            {
                case Text.AlignMode.Top: origin -= upVec; break;
                case Text.AlignMode.Center: origin -= upVec * 0.5f; break;
                case Text.AlignMode.Baseline: origin -= upVec * 0.2f; break; // approx descent fraction
                // Bottom: origin already at the bottom-left.
            }

            // Quad corners: P0 bottom-left, P1 bottom-right, P2 top-right, P3 top-left.
            var p0 = origin;
            var p1 = origin + rightVec;
            var p2 = origin + rightVec + upVec;
            var p3 = origin + upVec;

            if (_recordingList != null)
            {
                _recordingList.RecordTexturedQuad(p0, p1, p2, p3, texId);
            }
            else
            {
                var mvp = _model * _view * _projection;
                WebGLInterop.DrawTexturedQuad(Flatten(mvp), QuadArr(p0, p1, p2, p3), texId);
                // Immediate text textures are not cached; free after the draw.
                WebGLInterop.DeleteTexture(texId);
            }
        }

        // ── Bitmaps (world-space textured quads) ─────────────────────────
        void IPaintTo3D.PrepareBitmap(object obitmap)
        {
            if (obitmap is not Bitmap bitmap || bitmap.IsEmpty) return;
            if (_bitmapTextures.ContainsKey(bitmap)) return;
            int texId = WebGLInterop.CreateTexture(BytesToDoubles(bitmap.Data), bitmap.Width, bitmap.Height);
            _bitmapTextures[bitmap] = texId;
        }

        void IPaintTo3D.RectangularBitmap(object obitmap, GeoPoint location, GeoVector directionWidth, GeoVector directionHeight)
        {
            if (obitmap is not Bitmap bitmap) return;
            if (!_bitmapTextures.TryGetValue(bitmap, out int texId)) return;

            var p0 = ToVec3(location);
            var p1 = ToVec3(location + directionWidth);
            var p2 = ToVec3(location + directionWidth + directionHeight);
            var p3 = ToVec3(location + directionHeight);

            if (_recordingList != null)
                _recordingList.RecordTexturedQuad(p0, p1, p2, p3, texId);
            else
            {
                var mvp = _model * _view * _projection;
                WebGLInterop.DrawTexturedQuad(Flatten(mvp), QuadArr(p0, p1, p2, p3), texId);
            }
        }

        // ── SetLineWidth (issue gl.lineWidth for parity; browsers clamp to 1px) ──
        void IPaintTo3D.SetLineWidth(LineWidth lineWidth)
        {
            if (!_useLineWidth) return;
            float w = (lineWidth == null || lineWidth.Width == 0.0) ? 1.0f : (float)(lineWidth.Width * 10.0);
            WebGLInterop.SetLineWidth(Math.Clamp(w, 1f, 10f));
        }

        // ── separate GL state stack (UseZBuffer + Blending) ──────────────
        void IPaintTo3D.PushState()
        {
            _stateStack.Push((_useZBuffer, _blending));
        }

        void IPaintTo3D.PopState()
        {
            if (_stateStack.Count == 0) return;
            var (z, b) = _stateStack.Pop();
            (this as IPaintTo3D).UseZBuffer(z);
            (this as IPaintTo3D).Blending(b);
        }

        // ── not needed for this milestone (no-ops / stubs) ───────────────
        void IPaintTo3D.SetClip(Rectangle clipRectangle) { }
        void IPaintTo3D.PreparePointSymbol(PointSymbol pointSymbol) { }
        void IPaintTo3D.PrepareIcon(object icon) { }
        void IPaintTo3D.PrepareBitmap(object bitmap, int xoffset, int yoffset) { }
        void IPaintTo3D.Nurbs(GeoPoint[] poles, double[] weights, double[] knots, int degree) { }
        void IPaintTo3D.DisplayIcon(GeoPoint p, object icon) { }
        void IPaintTo3D.DisplayBitmap(GeoPoint p, object bitmap) { }
        void IPaintTo3D.OpenPath() { }
        void IPaintTo3D.ClosePath(Color color) { }
        void IPaintTo3D.CloseFigure() { }
        void IPaintTo3D.Arc(GeoPoint center, GeoVector majorAxis, GeoVector minorAxis, double startParameter, double sweepParameter) { }

        // ── helpers (copied from PaintToOpenGL) ──────────────────────────
        private static Vector3[] GeoPointsToVec3(GeoPoint[] pts)
        {
            var r = new Vector3[pts.Length];
            for (int i = 0; i < pts.Length; i++) r[i] = new Vector3((float)pts[i].x, (float)pts[i].y, (float)pts[i].z);
            return r;
        }
        private static Vector3[] GeoVectorsToVec3(GeoVector[] vecs)
        {
            var r = new Vector3[vecs.Length];
            for (int i = 0; i < vecs.Length; i++) r[i] = new Vector3((float)vecs[i].x, (float)vecs[i].y, (float)vecs[i].z);
            return r;
        }
        private static Vector4 ColorToVec4(Color c) => new(c.R / 255f, c.G / 255f, c.B / 255f, c.A / 255f);
        private static Vector3 ToVec3(GeoPoint p) => new((float)p.x, (float)p.y, (float)p.z);

        // Append one unlit vertex (pos + dummy normal + color) in the 10-float format.
        private static void AppendUnlit(List<double> buf, Vector3 p, Vector4 color)
        {
            buf.Add(p.X); buf.Add(p.Y); buf.Add(p.Z);
            buf.Add(0); buf.Add(0); buf.Add(1);
            buf.Add(color.X); buf.Add(color.Y); buf.Add(color.Z); buf.Add(color.W);
        }

        // Pack 4 quad corners into 12 doubles (xyz each), matching addQuadToList/drawTexturedQuad.
        private static double[] QuadArr(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3) => new double[]
        {
            p0.X, p0.Y, p0.Z, p1.X, p1.Y, p1.Z,
            p2.X, p2.Y, p2.Z, p3.X, p3.Y, p3.Z,
        };

        private static double[] BytesToDoubles(byte[]? data)
        {
            if (data == null) return Array.Empty<double>();
            var r = new double[data.Length];
            for (int i = 0; i < data.Length; i++) r[i] = data[i];
            return r;
        }

        private static Matrix4x4 ModOpToMatrix4x4(ModOp m) => new(
            (float)m[0, 0], (float)m[1, 0], (float)m[2, 0], 0,
            (float)m[0, 1], (float)m[1, 1], (float)m[2, 1], 0,
            (float)m[0, 2], (float)m[1, 2], (float)m[2, 2], 0,
            (float)m[0, 3], (float)m[1, 3], (float)m[2, 3], 1);

        private static double[] Flatten(Matrix4x4 m) => new double[]
        {
            m.M11, m.M12, m.M13, m.M14,
            m.M21, m.M22, m.M23, m.M24,
            m.M31, m.M32, m.M33, m.M34,
            m.M41, m.M42, m.M43, m.M44,
        };
        private static double[] OvrArr(Vector4? c) =>
            c.HasValue ? new double[] { c.Value.X, c.Value.Y, c.Value.Z } : Array.Empty<double>();

        private sealed class FaceOffsetScope : IDisposable
        {
            private readonly PaintToWebGL _p;
            private readonly Matrix4x4 _saved;
            public FaceOffsetScope(PaintToWebGL p)
            {
                _p = p; _saved = p._model;
                p._model = Matrix4x4.CreateTranslation(p._lightDir * (float)p._pixelToWorld) * p._model;
            }
            public void Dispose() => _p._model = _saved;
        }
    }
}
