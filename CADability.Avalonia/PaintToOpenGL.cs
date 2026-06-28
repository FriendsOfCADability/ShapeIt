using System;
using System.Collections.Generic;
using System.Numerics;
using Silk.NET.OpenGL;
using Avalonia.OpenGL;
using Avalonia.Media.Imaging;
using Avalonia.Platform;
using CADability;
using CADability.Attribute;
using CADability.GeoObject;

namespace CADability.Avalonia
{
    /// <summary>
    /// Avalonia port of PaintToOpenGLModern.
    /// Context management is handled by Avalonia (IGlContext / OpenGlControlBase);
    /// WGL and System.Drawing are not used.
    /// Text rendering is stubbed out pending a cross-platform glyph implementation.
    /// </summary>
    public class PaintToOpenGL : IPaintTo3D
    {
        // ── OpenGL state ───────────────────────────────────────────────────
        private GL _gl = null!;
        private ShaderProgram _litShader = null!;
        private ShaderProgram _unlitShader = null!;
        // Wide lines (> 1 px) rendered as screen-space quads; glLineWidth is
        // clamped to 1 under ANGLE/Direct3D.
        private ShaderProgram _thickLineShader = null!;

        // ── Point sprite rendering ─────────────────────────────────────────
        private ShaderProgram? _pointShader;
        private const float PointSpritePixels = 11f;

        // ── Text billboard rendering ───────────────────────────────────────
        private ShaderProgram? _textShader;
        private uint _textVao, _textVbo;
        private readonly Dictionary<(string, int, char), uint> _charTextures = new();
        private const int CharTexSize = 20;

        // ── Rectangular bitmap rendering (textured quads) ──────────────────
        private ShaderProgram? _textureShader;
        private uint _texVao, _texVbo;
        private readonly Dictionary<Substitutes.Bitmap, uint> _bitmapTextures = new();

        // ── Viewport / projection ──────────────────────────────────────────
        private int _width, _height;
        private Matrix4x4 _projection = Matrix4x4.Identity;
        private Matrix4x4 _view = Matrix4x4.Identity;
        private Matrix4x4 _model = Matrix4x4.Identity;
        private readonly Stack<Matrix4x4> _modelStack = new();
        private Vector3 _lightDir = Vector3.Normalize(new Vector3(1, 1, 2));

        // ── IPaintTo3D state ───────────────────────────────────────────────
        private bool _paintSurfaces = true;
        private bool _paintEdges = true;
        private bool _paintSurfaceEdges = true;
        private bool _useLineWidth = false;
        private double _precision = 1e-6;
        private double _pixelToWorld = 1.0;
        private bool _selectMode = false;
        private Substitutes.Color _selectColor = Substitutes.Color.Yellow;
        private Substitutes.Color _backgroundColor = Substitutes.Color.Black;
        private Substitutes.Color _currentColor = Substitutes.Color.White;
        private bool _colorOverride = false;
        private Substitutes.Color _overrideColor;
        private bool _delayText = false;
        private bool _delayAll = false;
        private bool _triangulateText = true;
        private bool _dontRecalcTriang = false;
        private bool _isBitmap = false;
        private bool _useZBuffer = true;
        private bool _blending = false;

        private readonly record struct GlState(bool UseZBuffer, bool Blending);
        private readonly Stack<GlState> _stateStack = new();

        // ── Active recording list ──────────────────────────────────────────
        private GlBufferList? _recordingList;

        // ── Offscreen FBO ──────────────────────────────────────────────────
        private bool _renderingOffscreen;
        private uint _fboId, _fboColorRb, _fboDepthRb;
        private int _savedWidth, _savedHeight;
        // Framebuffer to restore after offscreen rendering (Avalonia may use non-zero fb)
        private uint _defaultFramebuffer = 0;

        // ── GL flavour ─────────────────────────────────────────────────────
        private bool _isGles;

        // ─────────────────────────────────────────────────────────────────

        public PaintToOpenGL(double precision = 1e-6)
        {
            _precision = precision;
        }

        /// <summary>
        /// Initialise the painter using Avalonia's GL interface.
        /// Must be called from within <c>OnOpenGlInit</c>.
        /// </summary>
        public void Init(GlInterface glInterface, int width, int height)
        {
            _isBitmap = false;
            _width = width;
            _height = height;
            _gl = GL.GetApi(name => glInterface.GetProcAddress(name));
            // glInterface.Version is the GL_VERSION string, e.g. "OpenGL ES 3.0 (ANGLE ...)"
            _isGles = (glInterface.Version ?? "").Contains("OpenGL ES", StringComparison.OrdinalIgnoreCase);
            FinishInit();
        }

        /// <summary>
        /// Sets the framebuffer ID that Avalonia considers the default.
        /// Call from <c>OnOpenGlRender(gl, fb)</c> before triggering the paint cycle.
        /// </summary>
        public void SetDefaultFramebuffer(uint fb) => _defaultFramebuffer = fb;

        // Replace the #version header to match the actual GL context.
        // Desktop GL 3.3 uses "330 core"; OpenGL ES 3.0 (ANGLE) uses "300 es".
        private string AdaptShader(string src, bool isFragment)
        {
            if (!_isGles) return src;
            string header = isFragment
                ? "#version 300 es\nprecision mediump float;"
                : "#version 300 es";
            return src.Replace("#version 330 core", header);
        }

        private void FinishInit()
        {
            string vert = AdaptShader(ShaderSources.VertexShader, isFragment: false);
            string lit = AdaptShader(ShaderSources.LitFragmentShader, isFragment: true);
            string unlit = AdaptShader(ShaderSources.UnlitFragmentShader, isFragment: true);
            string ptVert = AdaptShader(ShaderSources.PointVertexShader, isFragment: false);
            string ptFrag = AdaptShader(ShaderSources.PointFragmentShader, isFragment: true);
            string txVert = AdaptShader(ShaderSources.TextVertexShader, isFragment: false);
            string txFrag = AdaptShader(ShaderSources.TextFragmentShader, isFragment: true);
            string texVert = AdaptShader(ShaderSources.TextureVertexShader, isFragment: false);
            string texFrag = AdaptShader(ShaderSources.TextureFragmentShader, isFragment: true);

            string thickVert = AdaptShader(ShaderSources.ThickLineVertexShader, isFragment: false);

            _litShader = new ShaderProgram(_gl, vert, lit);
            _unlitShader = new ShaderProgram(_gl, vert, unlit);
            _thickLineShader = new ShaderProgram(_gl, thickVert, unlit);
            _pointShader = new ShaderProgram(_gl, ptVert, ptFrag);
            _textShader = new ShaderProgram(_gl, txVert, txFrag);
            _textureShader = new ShaderProgram(_gl, texVert, texFrag);

            _gl.Enable(EnableCap.DepthTest);
            _gl.Enable(EnableCap.Blend);
            _gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            _gl.Enable(EnableCap.ProgramPointSize);

            // Text VAO/VBO (quad updated per glyph draw)
            _textVao = _gl.GenVertexArray();
            _textVbo = _gl.GenBuffer();
            _gl.BindVertexArray(_textVao);
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, _textVbo);
            unsafe
            {
                _gl.BufferData(BufferTargetARB.ArrayBuffer,
                               (nuint)(16 * sizeof(float)),
                               (void*)null, BufferUsageARB.DynamicDraw);
            }
            uint stride = 4 * sizeof(float);
            _gl.EnableVertexAttribArray(0);
            _gl.VertexAttribPointer(0, 2, VertexAttribPointerType.Float, false, stride, 0);
            _gl.EnableVertexAttribArray(1);
            _gl.VertexAttribPointer(1, 2, VertexAttribPointerType.Float, false, stride, 2 * sizeof(float));
            _gl.BindVertexArray(0);
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, 0);

            // Textured-quad VAO/VBO: position (3) + UV (2), 6 vertices (two triangles),
            // re-uploaded per RectangularBitmap draw.
            _texVao = _gl.GenVertexArray();
            _texVbo = _gl.GenBuffer();
            _gl.BindVertexArray(_texVao);
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, _texVbo);
            unsafe
            {
                _gl.BufferData(BufferTargetARB.ArrayBuffer,
                               (nuint)(6 * 5 * sizeof(float)),
                               (void*)null, BufferUsageARB.DynamicDraw);
            }
            uint texStride = 5 * sizeof(float);
            _gl.EnableVertexAttribArray(0);
            _gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, texStride, 0);
            _gl.EnableVertexAttribArray(1);
            _gl.VertexAttribPointer(1, 2, VertexAttribPointerType.Float, false, texStride, 3 * sizeof(float));
            _gl.BindVertexArray(0);
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, 0);
        }

        // ─────────────────────────────────────────────────────────────────
        //  IPaintTo3D properties
        // ─────────────────────────────────────────────────────────────────

        bool IPaintTo3D.PaintSurfaces => _paintSurfaces;
        bool IPaintTo3D.PaintEdges => _paintEdges;
        bool IPaintTo3D.PaintSurfaceEdges { get => _paintSurfaceEdges; set => _paintSurfaceEdges = value; }
        bool IPaintTo3D.UseLineWidth { get => _useLineWidth; set => _useLineWidth = value; }
        double IPaintTo3D.Precision { get => _precision; set => _precision = value; }
        double IPaintTo3D.PixelToWorld => _pixelToWorld;
        bool IPaintTo3D.SelectMode { get => _selectMode; set => _selectMode = value; }
        Substitutes.Color IPaintTo3D.SelectColor { get => _selectColor; set => _selectColor = value; }
        bool IPaintTo3D.DelayText { get => _delayText; set => _delayText = value; }
        bool IPaintTo3D.DelayAll { get => _delayAll; set => _delayAll = value; }
        bool IPaintTo3D.TriangulateText { get => _triangulateText; set => _triangulateText = value; }
        bool IPaintTo3D.DontRecalcTriangulation { get => _dontRecalcTriang; set => _dontRecalcTriang = value; }
        bool IPaintTo3D.IsBitmap => _isBitmap;
        PaintCapabilities IPaintTo3D.Capabilities =>
            PaintCapabilities.Standard | PaintCapabilities.ZoomIndependentDisplayList;

        // ─────────────────────────────────────────────────────────────────
        //  Context / lifecycle
        // ─────────────────────────────────────────────────────────────────

        // Avalonia activates the GL context before calling OnOpenGlRender; no action needed.
        void IPaintTo3D.MakeCurrent() { }

        void IPaintTo3D.Resize(int width, int height)
        {
            _width = width;
            _height = height;
        }

        void IPaintTo3D.Dispose()
        {
            _litShader?.Dispose();
            _unlitShader?.Dispose();
            _thickLineShader?.Dispose();
            _pointShader?.Dispose();
            _textShader?.Dispose();
            _textureShader?.Dispose();
            if (_textVao != 0) { _gl.DeleteVertexArray(_textVao); _textVao = 0; }
            if (_textVbo != 0) { _gl.DeleteBuffer(_textVbo); _textVbo = 0; }
            if (_texVao != 0) { _gl.DeleteVertexArray(_texVao); _texVao = 0; }
            if (_texVbo != 0) { _gl.DeleteBuffer(_texVbo); _texVbo = 0; }
            foreach (var tex in _charTextures.Values) _gl.DeleteTexture(tex);
            _charTextures.Clear();
            foreach (var tex in _bitmapTextures.Values) _gl.DeleteTexture(tex);
            _bitmapTextures.Clear();
        }

        // ─────────────────────────────────────────────────────────────────
        //  Frame start
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.Clear(Substitutes.Color background)
        {
            _backgroundColor = background;
            _gl.Viewport(0, 0, (uint)_width, (uint)_height);
            _gl.ClearColor(background.R / 255f, background.G / 255f,
                           background.B / 255f, 1f);
            _gl.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            // Reset the global line width at the start of every frame. glLineWidth
            // is global GL state that is not stored per recorded buffer, so without
            // this a thick width set during one frame (e.g. an immediate-mode line)
            // could leak into the recorded geometry of the next frame.
            _gl.LineWidth(1f);
        }

        void IPaintTo3D.AvoidColor(Substitutes.Color color) => _backgroundColor = color;

        // ─────────────────────────────────────────────────────────────────
        //  Projection
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.SetProjection(Projection projection, BoundingBox boundingCube)
        {
            double[,] m = projection.GetOpenGLProjection(
                0, _width, 0, _height, boundingCube);

            _projection = new Matrix4x4(
                (float)m[0, 0], (float)m[1, 0], (float)m[2, 0], (float)m[3, 0],
                (float)m[0, 1], (float)m[1, 1], (float)m[2, 1], (float)m[3, 1],
                (float)m[0, 2], (float)m[1, 2], (float)m[2, 2], (float)m[3, 2],
                (float)m[0, 3], (float)m[1, 3], (float)m[2, 3], (float)m[3, 3]);

            _view = Matrix4x4.Identity;
            _model = Matrix4x4.Identity;

            if (Matrix4x4.Invert(_projection, out var inv))
            {
                var p0 = Vector4.Transform(new Vector4(-1f, -1f, 0f, 1f), inv);
                var p1 = Vector4.Transform(new Vector4(-1f + 2f / _width, -1f, 0f, 1f), inv);
                if (p0.W != 0 && p1.W != 0)
                    _pixelToWorld = (double)(Vector3.Distance(
                        new Vector3(p0.X / p0.W, p0.Y / p0.W, p0.Z / p0.W),
                        new Vector3(p1.X / p1.W, p1.Y / p1.W, p1.Z / p1.W)));
            }

            GeoVector vd = projection.Direction;
            var ld = new Vector3((float)vd.x, (float)vd.y, (float)vd.z);
            // For axis-aligned views (e.g. exactly from the top) projection.Direction can
            // return a zero or NaN vector. Vector3.Normalize would then yield NaN, which
            // poisons the lit shader and renders every face black. Guard against that.
            _lightDir = (ld.LengthSquared() > 1e-12f)
                        ? Vector3.Normalize(ld)
                        : new Vector3(0, 0, 1);

            _gl.Enable(EnableCap.DepthTest);
            _gl.DepthFunc(DepthFunction.Lequal);
            _gl.Viewport(0, 0, (uint)_width, (uint)_height);
            _useLineWidth = projection.UseLineWidth;
        }

        // ─────────────────────────────────────────────────────────────────
        //  Color
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.SetColor(Substitutes.Color color, int lockColor)
        {
            if (!_colorOverride)
            {
                if (color.R == _backgroundColor.R &&
                    color.G == _backgroundColor.G &&
                    color.B == _backgroundColor.B)
                {
                    color = (color.R + color.G + color.B < 3 * 128)
                            ? Substitutes.Color.White
                            : Substitutes.Color.Black;
                }
                _currentColor = color;
            }

            if (lockColor == 1) { _colorOverride = true; _overrideColor = color; }
            else if (lockColor == -1) _colorOverride = false;

            if (_recordingList != null)
                _recordingList.CurrentColor = ColorToVec4(_currentColor);
        }

        // ─────────────────────────────────────────────────────────────────
        //  Line style
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.SetLineWidth(LineWidth lineWidth)
        {
            if (!_useLineWidth) return;
            float w = (lineWidth == null || lineWidth.Width == 0.0)
                      ? 1.0f : (float)(lineWidth.Width * 10.0);
            float applied = Math.Clamp(w, 1f, 10f);

            if (_recordingList != null)
            {
                // While recording, the width must NOT touch the shared global
                // glLineWidth (that leaks into later frames). Instead it is stored
                // on the list so the polylines recorded next are bucketed by width
                // and each buffer applies its own width at draw time.
                _recordingList.CurrentLineWidth = applied;
                return;
            }

            _gl.LineWidth(applied);
        }

        void IPaintTo3D.SetLinePattern(LinePattern pattern) { }

        // ─────────────────────────────────────────────────────────────────
        //  3-D geometry
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.Polyline(GeoPoint[] points)
        {
            if (points.Length < 2) return;
            var verts = GeoPointsToVec3(points);
            if (_recordingList != null)
                _recordingList.RecordPolyline(verts);
            else
                DrawImmediateLines(verts);
        }

        void IPaintTo3D.FilledPolyline(GeoPoint[] points)
        {
            if (points.Length < 3) return;
            var tris = new List<float>();
            var color = ColorToVec4(_currentColor);
            var v0 = ToVec3(points[0]);
            for (int i = 1; i < points.Length - 1; i++)
            {
                AppendUnlitVertex(tris, v0, color);
                AppendUnlitVertex(tris, ToVec3(points[i]), color);
                AppendUnlitVertex(tris, ToVec3(points[i + 1]), color);
            }
            DrawImmediateTriangles(tris, lit: false);
        }

        void IPaintTo3D.Points(GeoPoint[] points, float size, PointSymbol pointSymbol)
        {
            if (points.Length == 0) return;
            var verts = GeoPointsToVec3(points);
            if (_recordingList != null)
                _recordingList.RecordPoints(verts, pointSymbol);
            else
                DrawImmediatePoints(verts, pointSymbol);
        }

        void IPaintTo3D.Triangle(GeoPoint[] vertex, GeoVector[] normals, int[] indextriples)
        {
            if (indextriples.Length == 0) return;
            var verts = GeoPointsToVec3(vertex);
            var norms = GeoVectorsToVec3(normals);
            var indices = FixNormalOrientation(verts, norms, indextriples);
            if (_recordingList != null)
                _recordingList.RecordTriangles(verts, norms, indices);
            else
                DrawImmediateIndexedTriangles(verts, norms, indices);
        }

        // ─────────────────────────────────────────────────────────────────
        //  Display lists
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.OpenList(string name)
        {
            if (_recordingList != null)
                throw new InvalidOperationException("Nested lists are not allowed.");
            _recordingList = new GlBufferList { Name = name ?? string.Empty };
            _recordingList.BeginRecording();
            _recordingList.CurrentColor = ColorToVec4(_currentColor);
        }

        IPaintTo3DList IPaintTo3D.CloseList()
        {
            var list = _recordingList;
            _recordingList = null;
            if (list == null) return null!;
            if (!list.HasContents) { list.Dispose(); return null!; }
            list.UploadToGpu(_gl);
            return list;
        }

        IPaintTo3DList IPaintTo3D.MakeList(List<IPaintTo3DList> sublists)
        {
            var container = new GlBufferList { Name = "_composite" };
            container.BeginRecording();
            bool any = false;
            foreach (var sub in sublists)
            {
                if (sub is GlBufferList gbl)
                {
                    container.RecordSubList(gbl, null, null);
                    any = true;
                }
            }
            if (!any) { container.Dispose(); return null!; }
            container.UploadToGpu(_gl);
            container.containedSubLists = sublists;
            return container;
        }

        void IPaintTo3D.List(IPaintTo3DList paintThisList)
        {
            if (paintThisList is not GlBufferList gbl) return;
            if (_recordingList != null)
            {
                _recordingList.RecordSubList(gbl, _model, ColorToVec4(_currentColor));
                return;
            }
            DrawBufferList(gbl, overrideColor: null);
        }

        void IPaintTo3D.SelectedList(IPaintTo3DList paintThisList, int wobbleRadius)
        {
            if (paintThisList is not GlBufferList gbl) return;
            var sc = ColorToVec4(_selectColor);

            if (wobbleRadius <= 0)
            {
                var offset = Matrix4x4.CreateTranslation(
                    _lightDir * (float)(-2.0 * _pixelToWorld));
                var savedModel = _model;
                _model = offset * _model;
                DrawBufferList(gbl, overrideColor: sc);
                _model = savedModel;
            }
            else
            {
                _gl.Disable(EnableCap.DepthTest);
                _gl.ClearStencil(0);
                _gl.Enable(EnableCap.StencilTest);
                _gl.Clear(ClearBufferMask.StencilBufferBit);

                _gl.StencilFunc(StencilFunction.Always, 1, 1);
                _gl.StencilOp(StencilOp.Replace, StencilOp.Replace, StencilOp.Replace);
                _gl.ColorMask(false, false, false, false);
                DrawBufferList(gbl, overrideColor: null);

                _gl.StencilFunc(StencilFunction.Notequal, 1, 1);
                _gl.StencilOp(StencilOp.Keep, StencilOp.Keep, StencilOp.Keep);
                _gl.ColorMask(true, true, true, true);
                DrawWobbled(gbl, wobbleRadius, sc);

                _gl.Disable(EnableCap.StencilTest);
                _gl.Enable(EnableCap.DepthTest);
            }
        }

        // ─────────────────────────────────────────────────────────────────
        //  2-D overlay
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.Line2D(int sx, int sy, int ex, int ey)
        {
            var ortho = Matrix4x4.CreateOrthographicOffCenter(0, _width, _height, 0, -1, 1);
            var savedProj = _projection;
            _projection = ortho;
            _model = Matrix4x4.Identity;
            DrawImmediateLines(new[] { new Vector3(sx, sy, 0), new Vector3(ex, ey, 0) });
            _projection = savedProj;
        }

        void IPaintTo3D.Line2D(Substitutes.PointF p1, Substitutes.PointF p2)
            => (this as IPaintTo3D).Line2D((int)p1.X, (int)p1.Y, (int)p2.X, (int)p2.Y);

        void IPaintTo3D.FillRect2D(Substitutes.PointF p1, Substitutes.PointF p2)
        {
            var ortho = Matrix4x4.CreateOrthographicOffCenter(0, _width, _height, 0, -1, 1);
            var savedProj = _projection;
            _projection = ortho;
            _model = Matrix4x4.Identity;

            var color = ColorToVec4(_currentColor);
            var tris = new List<float>();
            AppendUnlitVertex(tris, new Vector3(p1.X, p1.Y, 0), color);
            AppendUnlitVertex(tris, new Vector3(p1.X, p2.Y, 0), color);
            AppendUnlitVertex(tris, new Vector3(p2.X, p2.Y, 0), color);
            AppendUnlitVertex(tris, new Vector3(p1.X, p1.Y, 0), color);
            AppendUnlitVertex(tris, new Vector3(p2.X, p2.Y, 0), color);
            AppendUnlitVertex(tris, new Vector3(p2.X, p1.Y, 0), color);
            DrawImmediateTriangles(tris, lit: false);

            _projection = savedProj;
        }

        void IPaintTo3D.Point2D(int x, int y) { }

        // ─────────────────────────────────────────────────────────────────
        //  Text (stubbed – cross-platform glyph rendering not yet implemented)
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.PrepareText(string fontName, string textString, object fontStyle) { }

        void IPaintTo3D.Text(GeoVector lineDirection, GeoVector glyphDirection,
                              GeoPoint location, string fontName, string textString,
                              object fontStyle,
                              CADability.GeoObject.Text.AlignMode alignment,
                              CADability.GeoObject.Text.LineAlignMode lineAlignment)
        { }

        // ─────────────────────────────────────────────────────────────────
        //  Misc stubs
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.PreparePointSymbol(PointSymbol symbol) { }
        void IPaintTo3D.PrepareIcon(object icon) { }
        void IPaintTo3D.PrepareBitmap(object bitmap, int xoffset, int yoffset) { }

        void IPaintTo3D.PrepareBitmap(object obitmap)
        {
            if (obitmap is not Substitutes.Bitmap bitmap)
                throw new ArgumentException("PrepareBitmap needs a CADability Bitmap");
            if (bitmap.IsEmpty) return;
            if (_bitmapTextures.ContainsKey(bitmap)) return;

            uint tex = _gl.GenTexture();
            _gl.BindTexture(TextureTarget.Texture2D, tex);
            _gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapS, (int)TextureWrapMode.Repeat);
            _gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapT, (int)TextureWrapMode.Repeat);
            _gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Nearest);
            _gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Nearest);
            _gl.PixelStore(PixelStoreParameter.UnpackAlignment, 1);

            // Data is tightly packed RGBA bytes (see Substitutes.Bitmap.GetPixel).
            // Uploaded unflipped; the vertical orientation is handled in the quad UVs.
            _gl.TexImage2D<byte>(TextureTarget.Texture2D, 0, InternalFormat.Rgba,
                                 (uint)bitmap.Width, (uint)bitmap.Height, 0,
                                 Silk.NET.OpenGL.PixelFormat.Rgba, PixelType.UnsignedByte,
                                 (ReadOnlySpan<byte>)bitmap.Data);
            _gl.BindTexture(TextureTarget.Texture2D, 0);
            _bitmapTextures[bitmap] = tex;
        }

        void IPaintTo3D.RectangularBitmap(object obitmap, GeoPoint location,
                                           GeoVector directionWidth, GeoVector directionHeight)
        {
            if (obitmap is not Substitutes.Bitmap bitmap)
                throw new ArgumentException("RectangularBitmap needs a CADability Bitmap");
            if (!_bitmapTextures.TryGetValue(bitmap, out uint tex)) return;

            var p0 = ToVec3(location);
            var p1 = ToVec3(location + directionWidth);
            var p2 = ToVec3(location + directionWidth + directionHeight);
            var p3 = ToVec3(location + directionHeight);

            if (_recordingList != null)
                _recordingList.RecordTexturedQuad(p0, p1, p2, p3, tex);
            else
                DrawTexturedQuad(new GlBufferList.TexturedQuad(p0, p1, p2, p3, tex));
        }
        void IPaintTo3D.DisplayIcon(GeoPoint p, object icon) { }
        void IPaintTo3D.DisplayBitmap(GeoPoint p, object bitmap) { }
        void IPaintTo3D.Nurbs(GeoPoint[] poles, double[] weights, double[] knots, int degree) { }

        void IPaintTo3D.OpenPath() => throw new NotSupportedException();
        void IPaintTo3D.ClosePath(Substitutes.Color color) => throw new NotSupportedException();
        void IPaintTo3D.CloseFigure() => throw new NotSupportedException();
        void IPaintTo3D.Arc(GeoPoint center, GeoVector majorAxis, GeoVector minorAxis,
                             double startParameter, double sweepParameter)
            => throw new NotSupportedException();

        void IPaintTo3D.FreeUnusedLists() { }

        // ─────────────────────────────────────────────────────────────────
        //  Z-buffer / blending / state
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.UseZBuffer(bool use)
        {
            _useZBuffer = use;
            if (use) _gl.Enable(EnableCap.DepthTest);
            else _gl.Disable(EnableCap.DepthTest);
        }

        void IPaintTo3D.Blending(bool on)
        {
            _blending = on;
            if (on)
            {
                _gl.Enable(EnableCap.Blend);
                _gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            }
            else
            {
                _gl.Disable(EnableCap.Blend);
            }
        }

        void IPaintTo3D.PushState() => _stateStack.Push(new GlState(_useZBuffer, _blending));

        void IPaintTo3D.PopState()
        {
            if (_stateStack.Count == 0) return;
            var s = _stateStack.Pop();
            (this as IPaintTo3D).UseZBuffer(s.UseZBuffer);
            (this as IPaintTo3D).Blending(s.Blending);
        }

        // ─────────────────────────────────────────────────────────────────
        //  Model matrix stack
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.PushMultModOp(ModOp mm)
        {
            _modelStack.Push(_model);
            _model = ModOpToMatrix4x4(mm) * _model;
        }

        void IPaintTo3D.PopModOp()
        {
            if (_modelStack.Count > 0) _model = _modelStack.Pop();
        }

        // ─────────────────────────────────────────────────────────────────
        //  PaintFaces
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.PaintFaces(PaintTo3D.PaintMode paintMode)
        {
            _model = Matrix4x4.Identity;
            switch (paintMode)
            {
                case PaintTo3D.PaintMode.FacesOnly:
                    _model = Matrix4x4.CreateTranslation(
                        _lightDir * (float)(2.0 * _pixelToWorld));
                    _paintSurfaces = true;
                    _paintEdges = false;
                    break;
                case PaintTo3D.PaintMode.CurvesOnly:
                    _paintSurfaces = false;
                    _paintEdges = true;
                    break;
                default:
                    _paintSurfaces = true;
                    _paintEdges = true;
                    break;
            }
        }

        IDisposable IPaintTo3D.FacesBehindEdgesOffset => new FaceOffsetScope(this);

        private sealed class FaceOffsetScope : IDisposable
        {
            private readonly PaintToOpenGL _p;
            private readonly Matrix4x4 _saved;
            public FaceOffsetScope(PaintToOpenGL p)
            {
                _p = p;
                _saved = p._model;
                p._model = Matrix4x4.CreateTranslation(
                    p._lightDir * (float)(p._pixelToWorld)) * p._model;
            }
            public void Dispose() => _p._model = _saved;
        }

        // ─────────────────────────────────────────────────────────────────
        //  Clipping
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.SetClip(Substitutes.Rectangle clipRectangle)
        {
            if (clipRectangle.IsEmpty)
            {
                _gl.ClearStencil(0);
                _gl.Clear(ClearBufferMask.StencilBufferBit);
                _gl.Disable(EnableCap.StencilTest);
            }
            else
            {
                (this as IPaintTo3D).PushState();
                _gl.Disable(EnableCap.DepthTest);
                _gl.Disable(EnableCap.Blend);
                _gl.ClearStencil(0);
                _gl.Enable(EnableCap.StencilTest);
                _gl.Clear(ClearBufferMask.StencilBufferBit);
                _gl.StencilFunc(StencilFunction.Always, 1, 1);
                _gl.StencilOp(StencilOp.Replace, StencilOp.Replace, StencilOp.Replace);
                _gl.ColorMask(false, false, false, false);

                (this as IPaintTo3D).FillRect2D(
                    new Substitutes.PointF((float)clipRectangle.Left, (float)clipRectangle.Bottom),
                    new Substitutes.PointF((float)clipRectangle.Right, (float)clipRectangle.Top));

                _gl.StencilFunc(StencilFunction.Equal, 1, 1);
                _gl.StencilOp(StencilOp.Keep, StencilOp.Keep, StencilOp.Keep);
                _gl.ColorMask(true, true, true, true);
                (this as IPaintTo3D).PopState();
            }
        }

        // ─────────────────────────────────────────────────────────────────
        //  Frame end
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.FinishPaint()
        {
            _gl.Flush();
            _gl.Finish();
            // Buffer swap is handled by Avalonia's OpenGlControlBase.
        }

        // ─────────────────────────────────────────────────────────────────
        //  Offscreen FBO rendering
        // ─────────────────────────────────────────────────────────────────

        /// <summary>
        /// Binds an FBO for offscreen rendering.
        /// Call <see cref="EndOffscreenAsBitmap"/> to read back the result.
        /// </summary>
        public void BeginOffscreen(int width, int height)
        {
            _savedWidth = _width;
            _savedHeight = _height;
            _width = width;
            _height = height;
            _renderingOffscreen = true;

            _fboId = _gl.GenFramebuffer();
            _fboColorRb = _gl.GenRenderbuffer();
            _fboDepthRb = _gl.GenRenderbuffer();

            _gl.BindFramebuffer(FramebufferTarget.Framebuffer, _fboId);

            _gl.BindRenderbuffer(RenderbufferTarget.Renderbuffer, _fboColorRb);
            _gl.RenderbufferStorage(RenderbufferTarget.Renderbuffer,
                InternalFormat.Rgba8, (uint)width, (uint)height);
            _gl.FramebufferRenderbuffer(FramebufferTarget.Framebuffer,
                FramebufferAttachment.ColorAttachment0,
                RenderbufferTarget.Renderbuffer, _fboColorRb);

            _gl.BindRenderbuffer(RenderbufferTarget.Renderbuffer, _fboDepthRb);
            _gl.RenderbufferStorage(RenderbufferTarget.Renderbuffer,
                InternalFormat.DepthComponent24, (uint)width, (uint)height);
            _gl.FramebufferRenderbuffer(FramebufferTarget.Framebuffer,
                FramebufferAttachment.DepthAttachment,
                RenderbufferTarget.Renderbuffer, _fboDepthRb);

            _gl.BindRenderbuffer(RenderbufferTarget.Renderbuffer, 0);

            var status = _gl.CheckFramebufferStatus(FramebufferTarget.Framebuffer);
            if (status != GLEnum.FramebufferComplete)
                throw new InvalidOperationException($"Offscreen FBO incomplete: {status}");
        }

        /// <summary>
        /// Reads back the FBO and returns an Avalonia <see cref="WriteableBitmap"/> (BGRA 32 bpp).
        /// OpenGL Y=0-at-bottom is flipped to top-down.
        /// </summary>
        public WriteableBitmap EndOffscreenAsBitmap()
        {
            _gl.Flush();
            _gl.Finish();

            int width = _width;
            int height = _height;
            var pixels = new byte[width * height * 4];

            unsafe
            {
                fixed (byte* p = pixels)
                    _gl.ReadPixels(0, 0, (uint)width, (uint)height,
                        Silk.NET.OpenGL.PixelFormat.Rgba, PixelType.UnsignedByte, p);
            }

            _gl.BindFramebuffer(FramebufferTarget.Framebuffer, _defaultFramebuffer);
            _gl.DeleteFramebuffer(_fboId);
            _gl.DeleteRenderbuffer(_fboColorRb);
            _gl.DeleteRenderbuffer(_fboDepthRb);
            _fboId = _fboColorRb = _fboDepthRb = 0;

            _renderingOffscreen = false;
            _width = _savedWidth;
            _height = _savedHeight;

            var bitmap = new global::Avalonia.Media.Imaging.WriteableBitmap(
                new global::Avalonia.PixelSize(width, height),
                new global::Avalonia.Vector(96, 96),
                global::Avalonia.Platform.PixelFormat.Bgra8888,
                global::Avalonia.Platform.AlphaFormat.Premul);

            using var locked = bitmap.Lock();
            unsafe
            {
                byte* dst = (byte*)locked.Address;
                int stride = locked.RowBytes;
                for (int y = 0; y < height; y++)
                {
                    int srcRow = height - 1 - y;   // flip OpenGL Y
                    int srcBase = srcRow * width * 4;
                    int dstBase = y * stride;
                    for (int x = 0; x < width; x++)
                    {
                        int si = srcBase + x * 4;
                        int di = dstBase + x * 4;
                        dst[di + 0] = pixels[si + 2]; // R→B  (RGBA→BGRA)
                        dst[di + 1] = pixels[si + 1]; // G→G
                        dst[di + 2] = pixels[si + 0]; // B→R
                        dst[di + 3] = pixels[si + 3]; // A→A
                    }
                }
            }

            return bitmap;
        }

        // ─────────────────────────────────────────────────────────────────
        //  Private drawing helpers
        // ─────────────────────────────────────────────────────────────────

        private void SetupLitShader(Vector4? overrideColor)
        {
            _litShader.Use();
            _litShader.SetMatrix4("uMVP", _model * _view * _projection);
            _litShader.SetMatrix4("uModel", _model);
            _litShader.SetVec3("uLightDir", _lightDir);
            _litShader.SetVec3("uLightColor", new Vector3(1f, 1f, 1f));
            _litShader.SetVec3("uAmbient", new Vector3(0.2f, 0.2f, 0.2f));
            _litShader.SetVec4("uColorOverride", overrideColor ?? new Vector4(0, 0, 0, 0));
        }

        private void SetupUnlitShader(Vector4? overrideColor)
        {
            _unlitShader.Use();
            _unlitShader.SetMatrix4("uMVP", _model * _view * _projection);
            _unlitShader.SetMatrix4("uModel", _model);
            _unlitShader.SetVec4("uColorOverride", overrideColor ?? new Vector4(0, 0, 0, 0));
        }

        private void SetupThickLineShader(Vector4? overrideColor, float lineWidth)
        {
            _thickLineShader.Use();
            _thickLineShader.SetMatrix4("uMVP", _model * _view * _projection);
            _thickLineShader.SetVec4("uColorOverride", overrideColor ?? new Vector4(0, 0, 0, 0));
            _thickLineShader.SetVec2("uViewport", new Vector2(_width, _height));
            _thickLineShader.SetFloat("uHalfWidth", lineWidth * 0.5f);
        }

        private void SetupPointShader(PointSymbol symbol, Vector4? overrideColor)
        {
            _pointShader!.Use();
            _pointShader.SetMatrix4("uMVP", _model * _view * _projection);
            _pointShader.SetVec4("uColorOverride", overrideColor ?? new Vector4(0, 0, 0, 0));
            _pointShader.SetFloat("uPointSize", PointSpritePixels);
            _pointShader.SetInt("uPointSymbol", (int)symbol);
        }

        private void DrawBufferList(GlBufferList gbl, Vector4? overrideColor)
        {
            var ownModel = _model;
            var currentOverride = overrideColor;

            void SetModelAndColor(Matrix4x4 m, Vector4? color)
            {
                _model = m;
                currentOverride = color;
            }

            void PrepareDraw(GlBufferList.SubBuffer buf)
            {
                switch (buf.Mode)
                {
                    case PrimitiveType.Triangles:
                        // Wide lines are stored as triangle quads but use their own
                        // shader (screen-space expansion), not the lit surface shader.
                        if (buf.IsThickLine)
                            SetupThickLineShader(currentOverride, buf.LineWidth);
                        else
                            SetupLitShader(currentOverride);
                        break;
                    case PrimitiveType.Points:
                        SetupPointShader(buf.Symbol, currentOverride);
                        break;
                    default:
                        SetupUnlitShader(currentOverride);
                        break;
                }
            }

            gbl.Draw(_gl, ownModel, overrideColor, SetModelAndColor, PrepareDraw, DrawTexturedQuad);
            _model = ownModel;
        }

        private unsafe void DrawTexturedQuad(GlBufferList.TexturedQuad q)
        {
            // UVs map the texture so it appears upright for directionHeight pointing "up",
            // matching the legacy PaintToOpenGL.RectangularBitmap (texture uploaded unflipped).
            ReadOnlySpan<float> verts = stackalloc float[]
            {
                q.P0.X, q.P0.Y, q.P0.Z, 0f, 1f,
                q.P1.X, q.P1.Y, q.P1.Z, 1f, 1f,
                q.P2.X, q.P2.Y, q.P2.Z, 1f, 0f,
                q.P0.X, q.P0.Y, q.P0.Z, 0f, 1f,
                q.P2.X, q.P2.Y, q.P2.Z, 1f, 0f,
                q.P3.X, q.P3.Y, q.P3.Z, 0f, 0f,
            };

            _textureShader!.Use();
            _textureShader.SetMatrix4("uMVP", _model * _view * _projection);
            _textureShader.SetInt("uTexture", 0);

            _gl.ActiveTexture(TextureUnit.Texture0);
            _gl.BindTexture(TextureTarget.Texture2D, q.Texture);

            _gl.BindVertexArray(_texVao);
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, _texVbo);
            fixed (float* ptr = verts)
                _gl.BufferSubData(BufferTargetARB.ArrayBuffer, 0,
                                  (nuint)(verts.Length * sizeof(float)), ptr);
            _gl.DrawArrays(PrimitiveType.Triangles, 0, 6);

            _gl.BindVertexArray(0);
            _gl.BindTexture(TextureTarget.Texture2D, 0);
        }

        private void DrawImmediateLines(ReadOnlySpan<Vector3> points)
        {
            var data = new List<float>(points.Length * 2 * GlBufferList.FloatsPerVertex);
            var color = ColorToVec4(_currentColor);
            for (int i = 0; i < points.Length - 1; i++)
            {
                AppendUnlitVertex(data, points[i], color);
                AppendUnlitVertex(data, points[i + 1], color);
            }
            DrawImmediate(data, PrimitiveType.Lines, lit: false);
        }

        private void DrawImmediatePoints(ReadOnlySpan<Vector3> points, PointSymbol symbol)
        {
            var data = new List<float>(points.Length * GlBufferList.FloatsPerVertex);
            var color = ColorToVec4(_currentColor);
            foreach (var p in points) AppendUnlitVertex(data, p, color);
            DrawImmediate(data, PrimitiveType.Points, lit: false, symbol);
        }

        private void DrawImmediateTriangles(List<float> data, bool lit)
            => DrawImmediate(data, PrimitiveType.Triangles, lit);

        private void DrawImmediateIndexedTriangles(
            ReadOnlySpan<Vector3> verts,
            ReadOnlySpan<Vector3> norms,
            ReadOnlySpan<int> indices)
        {
            var color = ColorToVec4(_currentColor);
            var data = new List<float>(indices.Length * GlBufferList.FloatsPerVertex);
            foreach (int idx in indices)
            {
                var v = verts[idx];
                var n = norms[idx];
                data.Add(v.X); data.Add(v.Y); data.Add(v.Z);
                data.Add(n.X); data.Add(n.Y); data.Add(n.Z);
                data.Add(color.X); data.Add(color.Y); data.Add(color.Z); data.Add(color.W);
            }
            DrawImmediate(data, PrimitiveType.Triangles, lit: true);
        }

        private unsafe void DrawImmediate(List<float> data, PrimitiveType mode, bool lit,
                                           PointSymbol pointSymbol = PointSymbol.Dot)
        {
            if (data.Count == 0) return;

            uint vao = _gl.GenVertexArray();
            uint vbo = _gl.GenBuffer();

            _gl.BindVertexArray(vao);
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, vbo);

            var span = System.Runtime.InteropServices.CollectionsMarshal.AsSpan(data);
            fixed (float* ptr = span)
                _gl.BufferData(BufferTargetARB.ArrayBuffer,
                               (nuint)(span.Length * sizeof(float)),
                               ptr, BufferUsageARB.StreamDraw);

            uint stride = GlBufferList.FloatsPerVertex * sizeof(float);
            _gl.EnableVertexAttribArray(0);
            _gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, stride, 0);
            _gl.EnableVertexAttribArray(1);
            _gl.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, stride, 3 * sizeof(float));
            _gl.EnableVertexAttribArray(2);
            _gl.VertexAttribPointer(2, 4, VertexAttribPointerType.Float, false, stride, 6 * sizeof(float));

            if (mode == PrimitiveType.Points)
                SetupPointShader(pointSymbol, overrideColor: null);
            else if (lit)
                SetupLitShader(overrideColor: null);
            else
                SetupUnlitShader(overrideColor: null);

            uint count = (uint)(data.Count / GlBufferList.FloatsPerVertex);
            _gl.DrawArrays(mode, 0, count);

            _gl.BindVertexArray(0);
            _gl.DeleteVertexArray(vao);
            _gl.DeleteBuffer(vbo);
        }

        private void DrawWobbled(GlBufferList gbl, int radius, Vector4 overrideColor)
        {
            int a = radius, b = radius;
            int a2 = a * a, b2 = b * b, fa2 = 4 * a2;
            float pw = (float)_pixelToWorld;

            void DrawOffset(int dx, int dy)
            {
                var saved = _model;
                _model = Matrix4x4.CreateTranslation(dx * pw, -dy * pw, 0) * _model;
                DrawBufferList(gbl, overrideColor);
                _model = saved;
            }

            for (int x = 0, y = b, sigma = 2 * b2 + a2 * (1 - 2 * b);
                 b2 * x <= a2 * y; x++)
            {
                DrawOffset(+x, +y); DrawOffset(-x, +y);
                DrawOffset(+x, -y); DrawOffset(-x, -y);
                if (sigma >= 0) { sigma += fa2 * (1 - y); y--; }
                sigma += b2 * (4 * x + 6);
            }
            for (int x = a, y = 0, sigma = 2 * a2 + b2 * (1 - 2 * a);
                 a2 * y <= b2 * x; y++)
            {
                DrawOffset(+x, +y); DrawOffset(-x, +y);
                DrawOffset(+x, -y); DrawOffset(-x, -y);
                if (sigma >= 0) { sigma += fa2 * (1 - x); x--; }
                sigma += a2 * (4 * y + 6);
            }
        }

        // ─────────────────────────────────────────────────────────────────
        //  Conversion helpers
        // ─────────────────────────────────────────────────────────────────

        private static Vector3[] GeoPointsToVec3(GeoPoint[] pts)
        {
            var r = new Vector3[pts.Length];
            for (int i = 0; i < pts.Length; i++)
                r[i] = new Vector3((float)pts[i].x, (float)pts[i].y, (float)pts[i].z);
            return r;
        }

        private static Vector3[] GeoVectorsToVec3(GeoVector[] vecs)
        {
            var r = new Vector3[vecs.Length];
            for (int i = 0; i < vecs.Length; i++)
                r[i] = new Vector3((float)vecs[i].x, (float)vecs[i].y, (float)vecs[i].z);
            return r;
        }

        private static Vector3 ToVec3(GeoPoint p)
            => new Vector3((float)p.x, (float)p.y, (float)p.z);

        private static Vector4 ColorToVec4(Substitutes.Color c)
            => new Vector4(c.R / 255f, c.G / 255f, c.B / 255f, c.A / 255f);

        private static void AppendUnlitVertex(List<float> buf, Vector3 p, Vector4 color)
        {
            buf.Add(p.X); buf.Add(p.Y); buf.Add(p.Z);
            buf.Add(0); buf.Add(0); buf.Add(1);
            buf.Add(color.X); buf.Add(color.Y); buf.Add(color.Z); buf.Add(color.W);
        }

        private static ReadOnlySpan<int> FixNormalOrientation(
            ReadOnlySpan<Vector3> verts,
            ReadOnlySpan<Vector3> norms,
            int[] indices)
        {
            var result = new int[indices.Length];
            for (int i = 0; i < indices.Length; i += 3)
            {
                int i0 = indices[i], i1 = indices[i + 1], i2 = indices[i + 2];
                Vector3 v1 = verts[i0], v2 = verts[i1], v3 = verts[i2];
                Vector3 faceNormal = Vector3.Cross(v1 - v2, v3 - v2);
                if (Vector3.Dot(faceNormal, norms[i0]) < 0)
                {
                    result[i] = i0; result[i + 1] = i2; result[i + 2] = i1;
                }
                else
                {
                    result[i] = i0; result[i + 1] = i1; result[i + 2] = i2;
                }
            }
            return result;
        }

        private static Matrix4x4 ModOpToMatrix4x4(ModOp m)
        {
            return new Matrix4x4(
                (float)m[0, 0], (float)m[1, 0], (float)m[2, 0], 0,
                (float)m[0, 1], (float)m[1, 1], (float)m[2, 1], 0,
                (float)m[0, 2], (float)m[1, 2], (float)m[2, 2], 0,
                (float)m[0, 3], (float)m[1, 3], (float)m[2, 3], 1);
        }
    }
}
