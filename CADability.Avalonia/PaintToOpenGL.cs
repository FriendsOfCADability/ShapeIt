using System;
using System.Collections.Generic;
using System.Numerics;
using Avalonia.Controls;
using Avalonia.OpenGL;
using Avalonia.OpenGL.Controls;
using CADability;
using CADability.Attribute;
using CADability.GeoObject;
using CADability.Substitutes;

using static Avalonia.OpenGL.GlConsts;

namespace CADability.Avalonia
{
    class PaintToOpenGL : OpenGlControlBase, IPaintTo3D
    {
        public const int GL_POINTS = 0;
        public const int GL_TRUE   = 1;
        public const int GL_FALSE  = 0;

        private int _shaderProgram;
        private int _vertexShader;
        private int _fragmentShader;
        private int _vertexBufferObject;
        private int _vertexArrayObject;
        private int _indexBufferObject;

        private bool paintSurfaces;
        private bool paintEdges;
        private bool paintSurfaceEdges;
        private bool useLineWidth;
        private bool selectMode;
        private bool delayText;
        private bool delayAll;
        private bool triangulateText;
        private bool dontRecalcTriangulation;
        private bool isBitmap;
        private double precision;
        private double pixelToWorld;
        private Color selectColor;
        private PaintCapabilities capabilities;

        private static void GlCheckError(GlInterface gl)
        {
            int err;
            while ((err = gl.GetError()) != GL_NO_ERROR) {
                Console.WriteLine(err);
            }
        }

        private string GetShader(string shader)
        {
            string data = "#version 330\n";
            if (GlVersion.Type == GlProfileType.OpenGLES) {
                data += "precision mediump float;\n";
            }
            data += shader;
            return data;
        }

        private void ConfigureShaders(GlInterface gl)
        {
            _shaderProgram = gl.CreateProgram();

            _vertexShader = gl.CreateShader(GL_VERTEX_SHADER);
            Console.WriteLine(gl.CompileShaderAndGetError(_vertexShader, VertexShaderSource));
            gl.AttachShader(_shaderProgram, _vertexShader);

            _fragmentShader = gl.CreateShader(GL_FRAGMENT_SHADER);
            Console.WriteLine(gl.CompileShaderAndGetError(_fragmentShader, FragmentShaderSource));
            gl.AttachShader(_shaderProgram, _fragmentShader);

            Console.WriteLine(gl.LinkProgramAndGetError(_shaderProgram));
            gl.UseProgram(_shaderProgram);
        }

        private unsafe void CreateVertexBuffer(GlInterface gl)
        {
            Vector3[] vertices = new Vector3[]
            {
                new Vector3(-1.0f, -1.0f, 0.0f),
                new Vector3(1.0f, -1.0f, 0.0f),
                new Vector3(0.0f, 1.0f, 0.0f),
            };

            _vertexBufferObject = gl.GenBuffer();
            gl.BindBuffer(GL_ARRAY_BUFFER, _vertexBufferObject);

            fixed(void * pData = vertices)
                gl.BufferData(GL_ARRAY_BUFFER, new IntPtr(sizeof(Vector3) * vertices.Length),
                new IntPtr(pData), GL_STATIC_DRAW);

            _vertexArrayObject = gl.GenVertexArray();
            gl.BindVertexArray(_vertexArrayObject);
            gl.VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vector3), IntPtr.Zero);
            gl.EnableVertexAttribArray(0);
        }

        protected override void OnOpenGlInit(GlInterface gl)
        {
            base.OnOpenGlInit(gl);

            ConfigureShaders(gl);
            CreateVertexBuffer(gl);

            GlCheckError(gl);
        }

        protected override void OnOpenGlDeinit(GlInterface gl)
        {
            base.OnOpenGlDeinit(gl);

            gl.BindBuffer(GL_ARRAY_BUFFER, 0);
            gl.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
            gl.BindVertexArray(0);
            gl.UseProgram(0);
            gl.DeleteBuffer(_vertexBufferObject);
            gl.DeleteVertexArray(_vertexArrayObject);
            gl.DeleteProgram(_shaderProgram);
            gl.DeleteShader(_vertexShader);
            gl.DeleteShader(_fragmentShader);

            GlCheckError(gl);
        }

        protected override void OnOpenGlRender(GlInterface gl, int fb)
        {
            gl.ClearColor(0.85f, 0.90f, 0.98f, 1.0f);
            gl.Clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            gl.Viewport(0, 0, (int)Bounds.Width, (int)Bounds.Height);
            gl.DrawArrays(GL_TRIANGLES, 0, new IntPtr(3));

            GlCheckError(gl);
        }

        string VertexShaderSource => GetShader(@"
            layout(location = 0) in vec3 Position;

            void main()
            {
                gl_Position.xyz = Position;
                gl_Position.w   = 1.0;
            }
        ");

        string FragmentShaderSource => GetShader(@"
            void main()
            {
                gl_FragColor = vec4(1, 0, 1, 1);
            }
        ");

        bool IPaintTo3D.PaintSurfaces
        {
            get { return paintSurfaces; }
        }

        bool IPaintTo3D.PaintEdges
        {
            get { return paintEdges; }
        }

        bool IPaintTo3D.PaintSurfaceEdges
        {
            get { return paintSurfaceEdges; }
            set { paintSurfaceEdges = value; }
        }

        bool IPaintTo3D.UseLineWidth
        {
            get { return useLineWidth; }
            set { useLineWidth = value; }
        }

        double IPaintTo3D.Precision
        {
            get { return precision; }
            set { precision = value; }
        }

        double IPaintTo3D.PixelToWorld
        {
            get { return pixelToWorld; }
        }

        bool IPaintTo3D.SelectMode
        {
            get { return selectMode; }
            set { selectMode = value; }
        }

        Color IPaintTo3D.SelectColor
        {
            get { return selectColor; }
            set { selectColor = value; }
        }

        bool IPaintTo3D.DelayText
        {
            get { return delayText; }
            set { delayText = value; }
        }

        bool IPaintTo3D.DelayAll
        {
            get { return delayAll; }
            set { delayAll = value; }
        }

        bool IPaintTo3D.TriangulateText
        {
            get { return triangulateText; }
            set { triangulateText = value; }
        }

        bool IPaintTo3D.DontRecalcTriangulation
        {
            get { return dontRecalcTriangulation; }
            set { dontRecalcTriangulation = value; }
        }

        PaintCapabilities IPaintTo3D.Capabilities
        {
            get { return capabilities; }
        }

        bool IPaintTo3D.IsBitmap
        {
            get { return isBitmap; }
        }
        IDisposable IPaintTo3D.FacesBehindEdgesOffset {
            get { return null; }
            // TODO
        }

        void IPaintTo3D.MakeCurrent()
        {

        }
        void IPaintTo3D.SetColor(Color color, int lockColor = 0)
        {
        }
        void IPaintTo3D.AvoidColor(Color color)
        {

        }
        void IPaintTo3D.SetLineWidth(LineWidth lineWidth)
        {

        }
        void IPaintTo3D.SetLinePattern(LinePattern pattern)
        {

        }
        void IPaintTo3D.Polyline(GeoPoint[] points)
        {

        }
        void IPaintTo3D.FilledPolyline(GeoPoint[] points)
        {

        }
        void IPaintTo3D.Points(GeoPoint[] points, float size, PointSymbol pointSymbol)
        {

        }
        void IPaintTo3D.Triangle(GeoPoint[] vertex, GeoVector[] normals, int[] indextriples)
        {

        }
        void IPaintTo3D.PrepareText(string fontName, string textString, object fontStyle)
        {

        }
        void IPaintTo3D.PreparePointSymbol(PointSymbol pointSymbol)
        {

        }
        void IPaintTo3D.PrepareIcon(object icon)
        {

        }
        void IPaintTo3D.PrepareBitmap(object bitmap, int xoffset, int yoffset)
        {

        }
        void IPaintTo3D.PrepareBitmap(object bitmap)
        {

        }
        void IPaintTo3D.RectangularBitmap(object bitmap, GeoPoint location, GeoVector directionWidth, GeoVector directionHeight)
        {

        }
        void IPaintTo3D.Text(GeoVector lineDirection, GeoVector glyphDirection, GeoPoint location, string fontName, string textString, object fontStyle, CADability.GeoObject.Text.AlignMode alignment, CADability.GeoObject.Text.LineAlignMode lineAlignment)
        {

        }
        void IPaintTo3D.List(IPaintTo3DList paintThisList)
        {

        }
        void IPaintTo3D.SelectedList(IPaintTo3DList paintThisList, int wobbleRadius)
        {

        }
        void IPaintTo3D.Nurbs(GeoPoint[] poles, double[] weights, double[] knots, int degree)
        {

        }
        void IPaintTo3D.Line2D(int sx, int sy, int ex, int ey)
        {

        }
        void IPaintTo3D.Line2D(PointF p1, PointF p2)
        {

        }
        void IPaintTo3D.FillRect2D(PointF p1, PointF p2)
        {

        }
        void IPaintTo3D.Point2D(int x, int y)
        {

        }
        void IPaintTo3D.DisplayIcon(GeoPoint p, object icon)
        {

        }
        void IPaintTo3D.DisplayBitmap(GeoPoint p, object bitmap)
        {

        }
        void IPaintTo3D.SetProjection(Projection projection, BoundingCube boundingCube)
        {

        }
        void IPaintTo3D.Clear(Color background)
        {

        }
        void IPaintTo3D.Resize(int width, int height)
        {

        }
        void IPaintTo3D.OpenList(string name = null)
        {

        }
        IPaintTo3DList IPaintTo3D.CloseList()
        {
            throw new NotImplementedException();
        }
        IPaintTo3DList IPaintTo3D.MakeList(List<IPaintTo3DList> sublists)
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.OpenPath()
        {
            throw new NotSupportedException("OpenGL does not support paths");
        }
        void IPaintTo3D.ClosePath(Color color)
        {
            throw new NotSupportedException("OpenGL does not support paths");
        }
        void IPaintTo3D.CloseFigure()
        {
            throw new NotSupportedException("OpenGL does not support paths");
        }
        void IPaintTo3D.Arc(GeoPoint center, GeoVector majorAxis, GeoVector minorAxis, double startParameter, double sweepParameter)
        {
            throw new NotSupportedException("OpenGL does not support paths");
        }
        void IPaintTo3D.FreeUnusedLists()
        {
        }
        void IPaintTo3D.UseZBuffer(bool use)
        {
        }
        void IPaintTo3D.Blending(bool on)
        {
        }
        void IPaintTo3D.FinishPaint()
        {
        }
        void IPaintTo3D.PaintFaces(PaintTo3D.PaintMode paintMode)
        {
        }
        void IPaintTo3D.Dispose()
        {

        }
        void IPaintTo3D.PushState()
        {

        }
        void IPaintTo3D.PopState()
        {

        }
        void IPaintTo3D.PushMultModOp(ModOp insertion)
        {

        }
        void IPaintTo3D.PopModOp()
        {

        }
        void IPaintTo3D.SetClip(Rectangle clipRectangle)
        {

        }
    }
}
