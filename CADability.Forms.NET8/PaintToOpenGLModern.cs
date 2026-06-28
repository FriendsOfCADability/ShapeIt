using System;
using System.Collections.Generic;
using System.Numerics;
using System.Windows.Forms;
using System.Drawing;
using System.Drawing.Imaging;
using System.Drawing.Text;
using Silk.NET.OpenGL;
using CADability;
using CADability.Attribute;
using CADability.GeoObject;

// Usage (Windows Forms, matches old PaintToOpenGL pattern):
//
//   var painter = new PaintToOpenGLModern();
//   painter.Init(ctrl);                 // attach to a Control
//   // -- or --
//   painter.Init(dc, width, height, toBitmap);   // raw DC (bitmap rendering)
//
//   // In paint handler:
//   IPaintTo3D p = painter;
//   p.MakeCurrent();
//   p.Clear(...);
//   p.SetProjection(...);
//   // ... scene painting ...
//   p.FinishPaint();
//
// Later (Avalonia migration): only Init() and FinishPaint() change.
// Everything else (shaders, VAO/VBO, IPaintTo3D methods) stays identical.

namespace CADability.Forms.NET8
{
    // ─────────────────────────────────────────────────────────────────────
    /// <summary>
    /// Modern OpenGL 3.3 Core implementation of <see cref="IPaintTo3D"/>.
    ///
    /// Replaces the legacy fixed-function <c>PaintToOpenGL</c> (glBegin/glEnd,
    /// Display Lists, fixed lighting) with VAO/VBO + GLSL shaders.
    ///
    /// Context is managed by <see cref="WglContext"/> which requests a
    /// Core Profile via wglCreateContextAttribsARB and loads Silk.NET bindings
    /// through wglGetProcAddress – no Avalonia dependency required.
    ///
    /// Display-list replacement strategy
    /// ──────────────────────────────────
    /// <see cref="IPaintTo3DList"/> maps to <see cref="GlBufferList"/>.
    /// Between <see cref="OpenList"/> and <see cref="CloseList"/> every
    /// geometry call (Triangle, Polyline, Points …) appends vertices to
    /// CPU-side staging buffers inside the GlBufferList.
    /// <see cref="CloseList"/> uploads the data to the GPU (VAO/VBO).
    /// <see cref="List"/> and <see cref="SelectedList"/> just call
    /// glDrawArrays on the cached buffers.
    /// </summary>
    public class PaintToOpenGLModern : IPaintTo3D
    {
        // ── Context (WGL) ──────────────────────────────────────────────────
        private WglContext _context = null!;
        // For bitmap rendering via Graphics.GetHdc()
        private System.Drawing.Graphics? _graphics;

        // ── OpenGL state ───────────────────────────────────────────────────
        private GL _gl = null!;
        private ShaderProgram _litShader   = null!;
        private ShaderProgram _unlitShader = null!;

        // ── Point sprite rendering ─────────────────────────────────────────
        private ShaderProgram? _pointShader;
        // Fixed on-screen diameter (pixels) of point sprites, zoom-independent
        private const float PointSpritePixels = 11f;

        // ── Text billboard rendering ───────────────────────────────────────
        private ShaderProgram? _textShader;
        private uint _textVao, _textVbo;
        // White-on-transparent glyph textures keyed by (fontName, fontStyle, char)
        private readonly Dictionary<(string, int, char), uint> _charTextures = new();
        // On-screen pixel size of each glyph quad
        private const int CharTexSize = 20;

        // ── Viewport / projection ──────────────────────────────────────────
        private int _width, _height;
        private Matrix4x4 _projection = Matrix4x4.Identity;
        private Matrix4x4 _view       = Matrix4x4.Identity;
        private Matrix4x4 _model      = Matrix4x4.Identity;  // PushMultModOp stack
        private readonly Stack<Matrix4x4> _modelStack = new();

        // Light direction is updated in SetProjection to match the view direction
        private Vector3 _lightDir = Vector3.Normalize(new Vector3(1, 1, 2));

        // ── IPaintTo3D state ───────────────────────────────────────────────
        private bool _paintSurfaces    = true;
        private bool _paintEdges       = true;
        private bool _paintSurfaceEdges = true;
        private bool _useLineWidth      = false;
        private double _precision       = 1e-6;
        private double _pixelToWorld    = 1.0;
        private bool _selectMode        = false;
        private Substitutes.Color _selectColor     = Substitutes.Color.Yellow;
        private Substitutes.Color _backgroundColor = Substitutes.Color.Black;
        private Substitutes.Color _currentColor    = Substitutes.Color.White;
        private bool _colorLocked       = false;
        private bool _colorOverride     = false;
        private Substitutes.Color _overrideColor;
        private bool _delayText         = false;
        private bool _delayAll          = false;
        private bool _triangulateText   = true;
        private bool _dontRecalcTriang  = false;
        private bool _isBitmap          = false;
        private bool _useZBuffer        = true;
        private bool _blending          = false;

        // State stack (PushState / PopState)
        private readonly record struct GlState(bool UseZBuffer, bool Blending);
        private readonly Stack<GlState> _stateStack = new();

        // ── Active recording list ──────────────────────────────────────────
        // Non-null between OpenList() and CloseList()
        private GlBufferList? _recordingList;

        // ── PaintFaces offset ──────────────────────────────────────────────
        // Small Z offset applied to faces so they don't z-fight with edges
        private bool _faceOffset = false;

        // ── Offscreen FBO ──────────────────────────────────────────────────
        private bool _renderingOffscreen;
        private uint _fboId, _fboColorRb, _fboDepthRb;
        private int  _savedWidth, _savedHeight;

        // ─────────────────────────────────────────────────────────────────
        //  Initialisation  (matches old PaintToOpenGL.Init() signatures)
        // ─────────────────────────────────────────────────────────────────

        public PaintToOpenGLModern(double precision = 1e-6)
        {
            _precision = precision;
        }

        /// <summary>Attach to a Windows Forms control (normal window rendering).</summary>
        public void Init(Control ctrl)
        {
            _isBitmap = false;
            _width    = ctrl.ClientSize.Width;
            _height   = ctrl.ClientSize.Height;
            _context  = new WglContext();
            _context.Init(ctrl);
            FinishInit();
            ctrl.HandleDestroyed += (_, _) => (this as IPaintTo3D).Dispose();
        }

        /// <summary>Attach to a raw device context (e.g. for bitmap rendering).</summary>
        public void Init(IntPtr deviceContext, int width, int height, bool toBitmap)
        {
            _isBitmap = toBitmap;
            _width    = width;
            _height   = height;
            _context  = new WglContext();
            _context.Init(deviceContext, width, height, toBitmap);
            FinishInit();
        }

        /// <summary>Convenience overload: attach to a Bitmap (mirrors old API).</summary>
        public void Init(System.Drawing.Bitmap bitmap)
        {
            _graphics = System.Drawing.Graphics.FromImage(bitmap);
            var dc    = _graphics.GetHdc();
            Init(dc, bitmap.Width, bitmap.Height, toBitmap: true);
        }

        private void FinishInit()
        {
            _gl = _context.CreateSilkBinding();
            _litShader   = new ShaderProgram(_gl, ShaderSources.VertexShader,      ShaderSources.LitFragmentShader);
            _unlitShader = new ShaderProgram(_gl, ShaderSources.VertexShader,      ShaderSources.UnlitFragmentShader);
            _pointShader = new ShaderProgram(_gl, ShaderSources.PointVertexShader, ShaderSources.PointFragmentShader);
            _textShader  = new ShaderProgram(_gl, ShaderSources.TextVertexShader,  ShaderSources.TextFragmentShader);
            _gl.Enable(EnableCap.DepthTest);
            _gl.Enable(EnableCap.Blend);
            _gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            _gl.Enable(EnableCap.ProgramPointSize);

            // One VAO/VBO pair shared by all glyph quads (4 vertices, updated per draw)
            _textVao = _gl.GenVertexArray();
            _textVbo = _gl.GenBuffer();
            _gl.BindVertexArray(_textVao);
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, _textVbo);
            unsafe
            {
                // 4 vertices × (vec2 pos + vec2 uv) = 16 floats — DynamicDraw because
                // the content is replaced for every character rendered
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
        }

        // ─────────────────────────────────────────────────────────────────
        //  IPaintTo3D properties
        // ─────────────────────────────────────────────────────────────────

        bool IPaintTo3D.PaintSurfaces        => _paintSurfaces;
        bool IPaintTo3D.PaintEdges           => _paintEdges;
        bool IPaintTo3D.PaintSurfaceEdges    { get => _paintSurfaceEdges; set => _paintSurfaceEdges = value; }
        bool IPaintTo3D.UseLineWidth         { get => _useLineWidth;      set => _useLineWidth = value; }
        double IPaintTo3D.Precision          { get => _precision;         set => _precision = value; }
        double IPaintTo3D.PixelToWorld       => _pixelToWorld;
        bool IPaintTo3D.SelectMode           { get => _selectMode;        set => _selectMode = value; }
        Substitutes.Color IPaintTo3D.SelectColor { get => _selectColor;   set => _selectColor = value; }
        bool IPaintTo3D.DelayText            { get => _delayText;         set => _delayText = value; }
        bool IPaintTo3D.DelayAll             { get => _delayAll;          set => _delayAll = value; }
        bool IPaintTo3D.TriangulateText      { get => _triangulateText;   set => _triangulateText = value; }
        bool IPaintTo3D.DontRecalcTriangulation { get => _dontRecalcTriang; set => _dontRecalcTriang = value; }
        bool IPaintTo3D.IsBitmap             => _isBitmap;
        PaintCapabilities IPaintTo3D.Capabilities =>
            PaintCapabilities.Standard | PaintCapabilities.ZoomIndependentDisplayList;

        // ─────────────────────────────────────────────────────────────────
        //  Context / lifecycle
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.MakeCurrent() => _context?.MakeCurrent();

        void IPaintTo3D.Resize(int width, int height)
        {
            _width  = width;
            _height = height;
        }

        void IPaintTo3D.Dispose()
        {
            _litShader?.Dispose();
            _unlitShader?.Dispose();
            _pointShader?.Dispose();
            _textShader?.Dispose();
            if (_textVao != 0) { _gl.DeleteVertexArray(_textVao); _textVao = 0; }
            if (_textVbo != 0) { _gl.DeleteBuffer(_textVbo); _textVbo = 0; }
            foreach (var tex in _charTextures.Values) _gl.DeleteTexture(tex);
            _charTextures.Clear();
            _context?.Dispose();
            if (_graphics != null)
            {
                _graphics.Dispose();
                _graphics = null;
            }
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
            // Use CADability's own matrix builder – same as old glLoadMatrixd call.
            // GetOpenGLProjection returns a column-major [4,4] double array.
            double[,] m = projection.GetOpenGLProjection(
                0, _width, 0, _height, boundingCube);

            // double[row,col] column-major → System.Numerics.Matrix4x4 (also column-major)
            _projection = new Matrix4x4(
                (float)m[0,0], (float)m[1,0], (float)m[2,0], (float)m[3,0],
                (float)m[0,1], (float)m[1,1], (float)m[2,1], (float)m[3,1],
                (float)m[0,2], (float)m[1,2], (float)m[2,2], (float)m[3,2],
                (float)m[0,3], (float)m[1,3], (float)m[2,3], (float)m[3,3]);

            _view  = Matrix4x4.Identity;
            _model = Matrix4x4.Identity;

            // PixelToWorld: size of one pixel in world coordinates
            // (used for wobble offsets and face/edge separation)
            if (Matrix4x4.Invert(_projection, out var inv))
            {
                // Project two adjacent pixels back to world space
                var p0 = Vector4.Transform(new Vector4(-1f, -1f, 0f, 1f), inv);
                var p1 = Vector4.Transform(new Vector4(-1f + 2f / _width, -1f, 0f, 1f), inv);
                if (p0.W != 0 && p1.W != 0)
                    _pixelToWorld = (double)(Vector3.Distance(
                        new Vector3(p0.X/p0.W, p0.Y/p0.W, p0.Z/p0.W),
                        new Vector3(p1.X/p1.W, p1.Y/p1.W, p1.Z/p1.W)));
            }

            // Light follows the view direction (same convention as old code)
            GeoVector vd = projection.Direction;
            _lightDir = Vector3.Normalize(new Vector3((float)vd.x, (float)vd.y, (float)vd.z));

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
                // Never use the background color – swap to its complement
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

            if (lockColor == 1)  { _colorOverride = true;  _overrideColor = color; }
            else if (lockColor == -1) _colorOverride = false;

            // Propagate to open recording list
            if (_recordingList != null)
                _recordingList.CurrentColor = ColorToVec4(_currentColor);
        }

        // ─────────────────────────────────────────────────────────────────
        //  Line style  (stored, applied via shader uniforms / gl state)
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

        void IPaintTo3D.SetLinePattern(LinePattern pattern)
        {
            // GL_LINE_STIPPLE is removed in Core profile.
            // A proper implementation would use a geometry shader or a
            // texture-based stipple.  For now we silently ignore the pattern;
            // all lines are drawn solid.  This matches common CAD behaviour
            // where stipple is a minor visual aid.
        }

        // ─────────────────────────────────────────────────────────────────
        //  3-D geometry  (immediate mode OR recorded into a list)
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.Polyline(GeoPoint[] points)
        {
            if (points.Length < 2) return;
            var verts = GeoPointsToVec3(points);

            if (_recordingList != null)
            {
                _recordingList.RecordPolyline(verts);
            }
            else
            {
                DrawImmediateLines(verts);
            }
        }

        void IPaintTo3D.FilledPolyline(GeoPoint[] points)
        {
            // Treat as an unlit polygon – upload as triangle fan.
            // This is rarely called; a simple fan triangulation suffices.
            if (points.Length < 3) return;
            var tris = new List<float>();
            var color = ColorToVec4(_currentColor);
            var v0 = ToVec3(points[0]);
            for (int i = 1; i < points.Length - 1; i++)
            {
                AppendUnlitVertex(tris, v0, color);
                AppendUnlitVertex(tris, ToVec3(points[i]),   color);
                AppendUnlitVertex(tris, ToVec3(points[i+1]), color);
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

            var verts   = GeoPointsToVec3(vertex);
            var norms   = GeoVectorsToVec3(normals);

            // Normal orientation fix (same logic as old code)
            // Build a corrected index list that ensures normals point outward
            var indices = FixNormalOrientation(verts, norms, indextriples);

            if (_recordingList != null)
            {
                _recordingList.RecordTriangles(verts, norms, indices);
            }
            else
            {
                DrawImmediateIndexedTriangles(verts, norms, indices);
            }
        }

        // ─────────────────────────────────────────────────────────────────
        //  Display lists  (OpenList / CloseList / List / MakeList)
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

            if (!list.HasContents)
            {
                list.Dispose();
                return null!;
            }

            list.UploadToGpu(_gl);
            return list;
        }

        IPaintTo3DList IPaintTo3D.MakeList(List<IPaintTo3DList> sublists)
        {
            // Create a container list that holds references to the sub-lists.
            // On Draw() it will call Draw() on each sub-list in order.
            var container = new GlBufferList { Name = "_composite" };
            container.BeginRecording();
            bool any = false;
            foreach (var sub in sublists)
            {
                if (sub is GlBufferList gbl)
                {
                    container.RecordSubList(gbl, null, null); // inherit parent's model and color
                    any = true;
                }
            }
            if (!any) { container.Dispose(); return null!; }
            container.UploadToGpu(_gl); // uploads nothing (empty CPU buffers) but sets _gl
            // Keep sub-lists alive via the containedSubLists setter pattern
            container.containedSubLists = sublists;
            return container;
        }

        void IPaintTo3D.List(IPaintTo3DList paintThisList)
        {
            if (paintThisList is not GlBufferList gbl) return;

            if (_recordingList != null)
            {
                // Capture model AND color so they are restored when the parent list is replayed.
                // This is what makes text glyphs render with the color that SetColor() set for
                // the text object, even though the glyph's vertex colors may be different
                // (e.g. built with a neutral color during PrePaintTo3D).
                _recordingList.RecordSubList(gbl, _model, ColorToVec4(_currentColor));
                return;
            }

            // Direct rendering: use vertex colors as-is (null = no override).
            // The color is already correct because the caller set it via SetColor() and
            // the list was built with that same color baked into its vertex data.
            DrawBufferList(gbl, overrideColor: null);
        }

        void IPaintTo3D.SelectedList(IPaintTo3DList paintThisList, int wobbleRadius)
        {
            if (paintThisList is not GlBufferList gbl) return;

            var sc = ColorToVec4(_selectColor);

            if (wobbleRadius <= 0)
            {
                // Draw slightly in front of the scene along the view direction.
                // Use pixelToWorld so the offset is always a few screen pixels,
                // independent of scene scale.
                var offset = Matrix4x4.CreateTranslation(
                    _lightDir * (float)(-2.0 * _pixelToWorld));
                var savedModel = _model;
                _model = offset * _model;

                DrawBufferList(gbl, overrideColor: sc);

                _model = savedModel;
            }
            else
            {
                // Stencil-based silhouette highlight (same algorithm as old code)
                _gl.Disable(EnableCap.DepthTest);
                _gl.ClearStencil(0);
                _gl.Enable(EnableCap.StencilTest);
                _gl.Clear(ClearBufferMask.StencilBufferBit);

                // Step 1: write stencil (color output masked – shader doesn't matter)
                _gl.StencilFunc(StencilFunction.Always, 1, 1);
                _gl.StencilOp(StencilOp.Replace, StencilOp.Replace, StencilOp.Replace);
                _gl.ColorMask(false, false, false, false);
                DrawBufferList(gbl, overrideColor: null);

                // Step 2: draw wobbled copies using select color where stencil == 0
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
            var ortho = Matrix4x4.CreateOrthographicOffCenter(
                0, _width, _height, 0, -1, 1);
            var savedProj = _projection;
            _projection = ortho;
            _model       = Matrix4x4.Identity;

            DrawImmediateLines(new[]
            {
                new Vector3(sx, sy, 0),
                new Vector3(ex, ey, 0)
            });
            _projection = savedProj;
        }

        void IPaintTo3D.Line2D(Substitutes.PointF p1, Substitutes.PointF p2)
            => (this as IPaintTo3D).Line2D((int)p1.X, (int)p1.Y, (int)p2.X, (int)p2.Y);

        void IPaintTo3D.FillRect2D(Substitutes.PointF p1, Substitutes.PointF p2)
        {
            var ortho = Matrix4x4.CreateOrthographicOffCenter(
                0, _width, _height, 0, -1, 1);
            var savedProj = _projection;
            _projection = ortho;
            _model       = Matrix4x4.Identity;

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

        void IPaintTo3D.Point2D(int x, int y) { /* deprecated, not used */ }

        // ─────────────────────────────────────────────────────────────────
        //  Unsupported / deprecated stubs
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.PrepareText(string fontName, string textString, object fontStyle)
        {
            if (_textShader == null || _isBitmap) return;
            int fs = fontStyle is FontStyle f ? (int)f : 0;
            foreach (char c in textString)
                EnsureCharTexture(fontName, fs, c);
        }

        void IPaintTo3D.PreparePointSymbol(PointSymbol symbol) { }
        void IPaintTo3D.PrepareIcon(object icon) { }
        void IPaintTo3D.PrepareBitmap(object bitmap, int xoffset, int yoffset) { }
        void IPaintTo3D.PrepareBitmap(object bitmap) { }

        void IPaintTo3D.RectangularBitmap(object bitmap, GeoPoint location,
                                           GeoVector directionWidth, GeoVector directionHeight)
        { /* TODO: texture quad */ }

        void IPaintTo3D.DisplayIcon(GeoPoint p, object icon) { /* TODO: billboard sprite */ }
        void IPaintTo3D.DisplayBitmap(GeoPoint p, object bitmap) { /* TODO: billboard sprite */ }

        void IPaintTo3D.Text(GeoVector lineDirection, GeoVector glyphDirection,
                              GeoPoint location, string fontName, string textString,
                              object fontStyle,
                              CADability.GeoObject.Text.AlignMode alignment,
                              CADability.GeoObject.Text.LineAlignMode lineAlignment)
        {
            if (_textShader == null || _isBitmap || textString.Length == 0) return;
            // Text is always rendered directly; skip when building a display list
            if (_recordingList != null) return;

            int fs = fontStyle is FontStyle f ? (int)f : 0;
            foreach (char c in textString)
                EnsureCharTexture(fontName, fs, c);

            // Project world anchor into NDC
            var mvp  = _model * _view * _projection;
            var clip = Vector4.Transform(
                new Vector4((float)location.x, (float)location.y, (float)location.z, 1f), mvp);
            if (clip.W <= 0f) return;
            float ndcX = clip.X / clip.W;
            float ndcY = clip.Y / clip.W;

            // Glyph half-extents in NDC (CharTexSize pixels on screen)
            float halfW = CharTexSize / (float)_width;
            float halfH = CharTexSize / (float)_height;

            // First-character centre X, based on horizontal alignment
            float cx = lineAlignment switch {
                CADability.GeoObject.Text.LineAlignMode.Center =>
                    ndcX - (textString.Length - 1) * halfW,
                CADability.GeoObject.Text.LineAlignMode.Right  =>
                    ndcX - (2 * textString.Length - 1) * halfW,
                _ => ndcX + halfW   // Left: left edge at ndcX
            };

            // Glyph centre Y, based on vertical alignment
            // The texture is pre-centred vertically, so AlignMode.Center needs no offset.
            float cy = alignment switch {
                CADability.GeoObject.Text.AlignMode.Top    => ndcY - halfH,
                CADability.GeoObject.Text.AlignMode.Bottom => ndcY + halfH,
                _ => ndcY
            };

            _textShader.Use();
            _textShader.SetVec4("uColor", ColorToVec4(_currentColor));
            _gl.ActiveTexture(TextureUnit.Texture0);
            _textShader.SetInt("uTexture", 0);

            _gl.Enable(EnableCap.Blend);
            _gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            _gl.BindVertexArray(_textVao);
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, _textVbo);

            foreach (char c in textString)
            {
                if (!_charTextures.TryGetValue((fontName, fs, c), out uint tex)) { cx += 2 * halfW; continue; }
                _gl.BindTexture(TextureTarget.Texture2D, tex);

                // Triangle strip: top-left, bottom-left, top-right, bottom-right.
                // OpenGL UV origin is bottom-left (V↑), but System.Drawing bitmap
                // origin is top-left (Y↓), so row 0 of the bitmap lands at V=0.
                // Use V=0 at the top of the quad so the character is right-side up.
                float[] verts = {
                    cx - halfW, cy + halfH,  0f, 0f,
                    cx - halfW, cy - halfH,  0f, 1f,
                    cx + halfW, cy + halfH,  1f, 0f,
                    cx + halfW, cy - halfH,  1f, 1f,
                };
                unsafe
                {
                    fixed (float* ptr = verts)
                        _gl.BufferSubData(BufferTargetARB.ArrayBuffer,
                                          (nint)0,
                                          (nuint)(verts.Length * sizeof(float)),
                                          ptr);
                }
                _gl.DrawArrays(PrimitiveType.TriangleStrip, 0, 4);
                cx += 2 * halfW;
            }

            _gl.BindTexture(TextureTarget.Texture2D, 0);
            _gl.BindVertexArray(0);
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, 0);
        }

        // ── Text helper ───────────────────────────────────────────────────

        private uint EnsureCharTexture(string fontName, int fontStyle, char c)
        {
            var key = (fontName, fontStyle, c);
            if (_charTextures.TryGetValue(key, out uint existing)) return existing;

            const int sz = CharTexSize;
            uint tex = _gl.GenTexture();

            using var bmp = new Bitmap(sz, sz, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
            using (var g = Graphics.FromImage(bmp))
            {
                g.Clear(Color.Transparent);
                g.TextRenderingHint = TextRenderingHint.AntiAlias;
                // Font size fills ~80% of the bitmap height for good legibility
                using var font = new Font(fontName, sz * 0.8f,
                                          (FontStyle)fontStyle, GraphicsUnit.Pixel);
                using var sf = new StringFormat {
                    Alignment     = StringAlignment.Center,
                    LineAlignment = StringAlignment.Center };
                g.DrawString(c.ToString(), font, Brushes.White,
                             new RectangleF(0, 0, sz, sz), sf);
            }

            _gl.BindTexture(TextureTarget.Texture2D, tex);
            var bd = bmp.LockBits(new Rectangle(0, 0, sz, sz),
                                  ImageLockMode.ReadOnly,
                                  System.Drawing.Imaging.PixelFormat.Format32bppArgb);
            unsafe
            {
                _gl.TexImage2D(TextureTarget.Texture2D, 0, InternalFormat.Rgba,
                               (uint)sz, (uint)sz, 0,
                               Silk.NET.OpenGL.PixelFormat.Bgra,
                               PixelType.UnsignedByte, (void*)bd.Scan0);
            }
            bmp.UnlockBits(bd);

            _gl.TexParameter(TextureTarget.Texture2D,
                TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Linear);
            _gl.TexParameter(TextureTarget.Texture2D,
                TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
            _gl.TexParameter(TextureTarget.Texture2D,
                TextureParameterName.TextureWrapS, (int)TextureWrapMode.ClampToEdge);
            _gl.TexParameter(TextureTarget.Texture2D,
                TextureParameterName.TextureWrapT, (int)TextureWrapMode.ClampToEdge);
            _gl.BindTexture(TextureTarget.Texture2D, 0);

            _charTextures[key] = tex;
            return tex;
        }

        void IPaintTo3D.Nurbs(GeoPoint[] poles, double[] weights,
                               double[] knots, int degree)
        { /* deprecated */ }

        void IPaintTo3D.OpenPath()  => throw new NotSupportedException();
        void IPaintTo3D.ClosePath(Substitutes.Color color) => throw new NotSupportedException();
        void IPaintTo3D.CloseFigure() => throw new NotSupportedException();
        void IPaintTo3D.Arc(GeoPoint center, GeoVector majorAxis, GeoVector minorAxis,
                             double startParameter, double sweepParameter)
            => throw new NotSupportedException();

        void IPaintTo3D.FreeUnusedLists() { /* GC handles it */ }

        // ─────────────────────────────────────────────────────────────────
        //  Z-buffer / blending / state
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.UseZBuffer(bool use)
        {
            _useZBuffer = use;
            if (use) _gl.Enable(EnableCap.DepthTest);
            else     _gl.Disable(EnableCap.DepthTest);
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

        void IPaintTo3D.PushState()
        {
            _stateStack.Push(new GlState(_useZBuffer, _blending));
        }

        void IPaintTo3D.PopState()
        {
            if (_stateStack.Count == 0) return;
            var s = _stateStack.Pop();
            (this as IPaintTo3D).UseZBuffer(s.UseZBuffer);
            (this as IPaintTo3D).Blending(s.Blending);
        }

        // ─────────────────────────────────────────────────────────────────
        //  Model matrix stack  (PushMultModOp / PopModOp)
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
        //  PaintFaces  (face / edge offset mode)
        // ─────────────────────────────────────────────────────────────────

        void IPaintTo3D.PaintFaces(PaintTo3D.PaintMode paintMode)
        {
            _model = Matrix4x4.Identity;
            switch (paintMode)
            {
                case PaintTo3D.PaintMode.FacesOnly:
                    // Shift faces a tiny bit along the view direction so
                    // they don't z-fight with edges painted on top.
                    _model = Matrix4x4.CreateTranslation(
                        _lightDir * (float)(2.0 * _pixelToWorld));
                    _paintSurfaces = true;
                    _paintEdges    = false;
                    break;
                case PaintTo3D.PaintMode.CurvesOnly:
                    _paintSurfaces = false;
                    _paintEdges    = true;
                    break;
                default:
                    _paintSurfaces = true;
                    _paintEdges    = true;
                    break;
            }
        }

        IDisposable IPaintTo3D.FacesBehindEdgesOffset
            => new FaceOffsetScope(this);

        private sealed class FaceOffsetScope : IDisposable
        {
            private readonly PaintToOpenGLModern _p;
            private readonly Matrix4x4 _saved;
            public FaceOffsetScope(PaintToOpenGLModern p)
            {
                _p = p;
                _saved = p._model;
                p._model = Matrix4x4.CreateTranslation(
                    p._lightDir * (float)(p._pixelToWorld)) * p._model;
            }
            public void Dispose() => _p._model = _saved;
        }

        // ─────────────────────────────────────────────────────────────────
        //  Clipping (stencil-based rectangle clip)
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
                    new Substitutes.PointF((float)clipRectangle.Left,  (float)clipRectangle.Bottom),
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
            if (!_renderingOffscreen)
                _context?.SwapBuffers();
        }

        // ─────────────────────────────────────────────────────────────────
        //  Offscreen FBO rendering
        // ─────────────────────────────────────────────────────────────────

        /// <summary>
        /// Binds an FBO so subsequent rendering goes to GPU memory instead of the
        /// screen. Call <see cref="EndOffscreenAsBitmap"/> to read back the result.
        /// Must be called on the UI thread that owns the GL context.
        /// </summary>
        public void BeginOffscreen(int width, int height)
        {
            _savedWidth  = _width;
            _savedHeight = _height;
            _width  = width;
            _height = height;
            _renderingOffscreen = true;

            _fboId      = _gl.GenFramebuffer();
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
        /// Reads back the FBO, deletes GPU resources, restores the default framebuffer,
        /// and returns a <see cref="System.Drawing.Bitmap"/> (BGRA, 32 bpp).
        /// OpenGL's Y=0-at-bottom convention is flipped so the bitmap is top-down.
        /// </summary>
        public System.Drawing.Bitmap EndOffscreenAsBitmap()
        {
            _gl.Flush();
            _gl.Finish();

            int width  = _width;
            int height = _height;
            var pixels = new byte[width * height * 4];

            unsafe
            {
                fixed (byte* p = pixels)
                    _gl.ReadPixels(0, 0, (uint)width, (uint)height,
                        Silk.NET.OpenGL.PixelFormat.Rgba, PixelType.UnsignedByte, p);
            }

            _gl.BindFramebuffer(FramebufferTarget.Framebuffer, 0);
            _gl.DeleteFramebuffer(_fboId);
            _gl.DeleteRenderbuffer(_fboColorRb);
            _gl.DeleteRenderbuffer(_fboDepthRb);
            _fboId = _fboColorRb = _fboDepthRb = 0;

            _renderingOffscreen = false;
            _width  = _savedWidth;
            _height = _savedHeight;

            var bmp = new System.Drawing.Bitmap(width, height,
                System.Drawing.Imaging.PixelFormat.Format32bppArgb);
            var bmpData = bmp.LockBits(
                new System.Drawing.Rectangle(0, 0, width, height),
                System.Drawing.Imaging.ImageLockMode.WriteOnly,
                System.Drawing.Imaging.PixelFormat.Format32bppArgb);

            unsafe
            {
                byte* dst = (byte*)bmpData.Scan0;
                for (int y = 0; y < height; y++)
                {
                    int srcRow  = height - 1 - y;   // flip OpenGL Y
                    int srcBase = srcRow * width * 4;
                    int dstBase = y * bmpData.Stride;
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

            bmp.UnlockBits(bmpData);
            return bmp;
        }

        // ─────────────────────────────────────────────────────────────────
        //  Private drawing helpers
        // ─────────────────────────────────────────────────────────────────

        private void SetupLitShader(Vector4? overrideColor)
        {
            _litShader.Use();
            _litShader.SetMatrix4("uMVP",   _model * _view * _projection);
            _litShader.SetMatrix4("uModel", _model);
            _litShader.SetVec3("uLightDir",   _lightDir);
            _litShader.SetVec3("uLightColor", new Vector3(1f, 1f, 1f));
            _litShader.SetVec3("uAmbient",    new Vector3(0.2f, 0.2f, 0.2f));
            _litShader.SetVec4("uColorOverride",
                overrideColor ?? new Vector4(0, 0, 0, 0));
        }

        private void SetupUnlitShader(Vector4? overrideColor)
        {
            _unlitShader.Use();
            _unlitShader.SetMatrix4("uMVP",   _model * _view * _projection);
            _unlitShader.SetMatrix4("uModel", _model);
            _unlitShader.SetVec4("uColorOverride",
                overrideColor ?? new Vector4(0, 0, 0, 0));
        }

        private void SetupPointShader(PointSymbol symbol, Vector4? overrideColor)
        {
            _pointShader!.Use();
            _pointShader.SetMatrix4("uMVP", _model * _view * _projection);
            _pointShader.SetVec4("uColorOverride", overrideColor ?? new Vector4(0, 0, 0, 0));
            _pointShader.SetFloat("uPointSize",   PointSpritePixels);
            _pointShader.SetInt("uPointSymbol",   (int)symbol);
        }

        /// <summary>
        /// Draws a GlBufferList, propagating model matrix and color override into nested sub-lists.
        /// Must be called with _model already set to the desired transform.
        /// <paramref name="overrideColor"/>: null = use vertex colors; non-null = uColorOverride.
        /// </summary>
        private void DrawBufferList(GlBufferList gbl, Vector4? overrideColor)
        {
            var ownModel = _model;
            // Tracks the current override color as sub-lists may change it.
            var currentOverride = overrideColor;

            // Called when a sub-list changes model/color; updates painter state so
            // PrepareDraw (below) picks up the correct values for the next buffers.
            void SetModelAndColor(Matrix4x4 m, Vector4? color)
            {
                _model = m;
                currentOverride = color;
            }

            // Called before each VAO draw to activate the right shader and set
            // per-buffer uniforms (primitive type, point symbol).
            void PrepareDraw(GlBufferList.SubBuffer buf)
            {
                switch (buf.Mode)
                {
                    case PrimitiveType.Triangles:
                        SetupLitShader(currentOverride);
                        break;
                    case PrimitiveType.Points:
                        SetupPointShader(buf.Symbol, currentOverride);
                        break;
                    default: // Lines
                        SetupUnlitShader(currentOverride);
                        break;
                }
            }

            gbl.Draw(_gl, ownModel, overrideColor, SetModelAndColor, PrepareDraw);
            _model = ownModel;
        }

        // ── Immediate draw: upload a temporary VAO, draw, delete ──────────

        private void DrawImmediateLines(ReadOnlySpan<Vector3> points)
        {
            var data  = new List<float>(points.Length * 2 * GlBufferList.FloatsPerVertex);
            var color = ColorToVec4(_currentColor);
            for (int i = 0; i < points.Length - 1; i++)
            {
                AppendUnlitVertex(data, points[i],   color);
                AppendUnlitVertex(data, points[i+1], color);
            }
            DrawImmediate(data, PrimitiveType.Lines, lit: false);
        }

        private void DrawImmediatePoints(ReadOnlySpan<Vector3> points, PointSymbol symbol)
        {
            var data  = new List<float>(points.Length * GlBufferList.FloatsPerVertex);
            var color = ColorToVec4(_currentColor);
            foreach (var p in points) AppendUnlitVertex(data, p, color);
            DrawImmediate(data, PrimitiveType.Points, lit: false, symbol);
        }

        private void DrawImmediateTriangles(List<float> data, bool lit)
            => DrawImmediate(data, PrimitiveType.Triangles, lit);

        private void DrawImmediateIndexedTriangles(
            ReadOnlySpan<Vector3> verts,
            ReadOnlySpan<Vector3> norms,
            ReadOnlySpan<int>     indices)
        {
            var color = ColorToVec4(_currentColor);
            var data  = new List<float>(indices.Length * GlBufferList.FloatsPerVertex);
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

        // ── Wobble draw for SelectedList ──────────────────────────────────

        private void DrawWobbled(GlBufferList gbl, int radius, Vector4 overrideColor)
        {
            // Bresenham ellipse, same as old code – draw copies offset in pixel space
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
                if (sigma >= 0) { sigma += fa2 * (1 - x); x--; } // note: fa2 → fb2
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

        /// <summary>
        /// Reorder triangle indices so normals point outward.
        /// Mirrors the old glNormal/glVertex logic in PaintToOpenGL.Triangle().
        /// </summary>
        private static ReadOnlySpan<int> FixNormalOrientation(
            ReadOnlySpan<Vector3> verts,
            ReadOnlySpan<Vector3> norms,
            int[] indices)
        {
            var result = new int[indices.Length];
            for (int i = 0; i < indices.Length; i += 3)
            {
                int i0 = indices[i], i1 = indices[i+1], i2 = indices[i+2];
                Vector3 v1 = verts[i0], v2 = verts[i1], v3 = verts[i2];
                Vector3 faceNormal = Vector3.Cross(v1 - v2, v3 - v2);
                if (Vector3.Dot(faceNormal, norms[i0]) < 0)
                {
                    // Flip: swap i1 and i2
                    result[i] = i0; result[i+1] = i2; result[i+2] = i1;
                }
                else
                {
                    result[i] = i0; result[i+1] = i1; result[i+2] = i2;
                }
            }
            return result;
        }

        private static Matrix4x4 ModOpToMatrix4x4(ModOp m)
        {
            // ModOp[row, col]; Matrix4x4 constructor is row-major but OpenGL reads
            // it as column-major, so we transpose: Sys.Row[i] = ModOp.Col[i].
            return new Matrix4x4(
                (float)m[0,0], (float)m[1,0], (float)m[2,0], 0,
                (float)m[0,1], (float)m[1,1], (float)m[2,1], 0,
                (float)m[0,2], (float)m[1,2], (float)m[2,2], 0,
                (float)m[0,3], (float)m[1,3], (float)m[2,3], 1);
        }
    }
}
