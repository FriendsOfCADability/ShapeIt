using System;
using System.Collections.Generic;
using System.Numerics;
using System.Linq;
using Silk.NET.OpenGL;
using Avalonia.Controls;
using Avalonia.OpenGL;
using Avalonia.OpenGL.Controls;
using Avalonia.Threading;
using CADability;
using CADability.Attribute;
using CADability.GeoObject;
using CADability.Substitutes;
using MathNet.Numerics.LinearAlgebra.Single;

using static Avalonia.OpenGL.GlConsts;

namespace CADability.Avalonia
{
    class PaintToOpenGL : OpenGlControlBase, IPaintTo3D
    {
        private uint _shaderProgram;
        private uint _vertexShader;
        private uint _fragmentShader;
        private uint _vertexBufferObject;
        private uint _vertexArrayObject;
        private uint _indexBufferObject;
        private int modelViewLocation;
        private int projectionLocation;
        private int colorLocation;
        private int lightPositionLocation;
        private int ambientFactorLocation;

        private IView _view;
        private GL _gl; // use Silk.NET gl interface, as Avalonia only has incomplete bindings
        private GlInterface _aGl; // Avalonia gl interface
        private Color _backgroundColor;

        private VertexArrayObject currentVao;
        private Dictionary<string, VertexArrayObject> vaos;

        private GeoVector projectionDirection;
        private bool isPerspective;
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
        private bool colorOverride;
        private double precision;
        private double pixelToWorld;
        private Color selectColor;
        private PaintCapabilities capabilities;

        public PaintToOpenGL(double precision = 1e-6)
        {
            this.precision = precision;
            paintSurfaces = true;
            paintEdges = true;
            paintSurfaceEdges = true;
            selectColor = Substitutes.Color.Yellow;
            vaos = new Dictionary<string, VertexArrayObject>();
        }

        private void GlCheckError()
        {
            GLEnum err;
            while ((err = _gl.GetError()) != GLEnum.NoError) {
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

        private void ConfigureShaders()
        {
            _shaderProgram = _gl.CreateProgram();

            _vertexShader = _gl.CreateShader(ShaderType.VertexShader);
            Console.WriteLine(_aGl.CompileShaderAndGetError((int)_vertexShader, VertexShaderSource));
            _gl.AttachShader(_shaderProgram, _vertexShader);

            _fragmentShader = _gl.CreateShader(ShaderType.FragmentShader);
            Console.WriteLine(_aGl.CompileShaderAndGetError((int)_fragmentShader, FragmentShaderSource));
            _gl.AttachShader(_shaderProgram, _fragmentShader);

            Console.WriteLine(_aGl.LinkProgramAndGetError((int)_shaderProgram));
            _gl.UseProgram(_shaderProgram);

            modelViewLocation = _gl.GetUniformLocation(_shaderProgram, "modelview");
            projectionLocation = _gl.GetUniformLocation(_shaderProgram, "projection");
            colorLocation = _gl.GetUniformLocation(_shaderProgram, "color");
            lightPositionLocation = _gl.GetUniformLocation(_shaderProgram, "lightPosition");
            ambientFactorLocation = _gl.GetUniformLocation(_shaderProgram, "ambientFactor");
            Console.WriteLine("modelViewLocation: " + modelViewLocation);
            Console.WriteLine("projectionLocation: " + projectionLocation);
            Console.WriteLine("colorLocation: " + colorLocation);
            GlCheckError();
        }

        private unsafe void CreateVertexBuffer()
        {
            Vector3[] vertices = new Vector3[]
            {
                new Vector3(-1.0f, -1.0f, 0.0f),
                new Vector3(1.0f, -1.0f, 0.0f),
                new Vector3(0.0f, 1.0f, 0.0f),
            };

            _vertexBufferObject = _gl.GenBuffer();
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, _vertexBufferObject);

            fixed(void * pData = vertices)
                _gl.BufferData(BufferTargetARB.ArrayBuffer, (nuint) (sizeof(Vector3) * vertices.Length),
                pData, BufferUsageARB.StaticDraw);

            _vertexArrayObject = _gl.GenVertexArray();
            _gl.BindVertexArray(_vertexArrayObject);
            _gl.VertexAttribPointer(0, 3, GLEnum.Float, false, (uint) sizeof(Vector3), (void*)0);
            _gl.EnableVertexAttribArray(0);
        }

        protected override void OnOpenGlInit(GlInterface gl)
        {
            base.OnOpenGlInit(gl);

            _aGl = gl;
            _gl = GL.GetApi(gl.GetProcAddress);

            ConfigureShaders();
            CreateVertexBuffer();

            GlCheckError();
        }

        protected override void OnOpenGlDeinit(GlInterface gl)
        {
            base.OnOpenGlDeinit(gl);

            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, 0);
            _gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, 0);
            _gl.BindVertexArray(0);
            _gl.UseProgram(0);
            _gl.DeleteBuffer(_vertexBufferObject);
            _gl.DeleteVertexArray(_vertexArrayObject);
            _gl.DeleteProgram(_shaderProgram);
            _gl.DeleteShader(_vertexShader);
            _gl.DeleteShader(_fragmentShader);

            GlCheckError();
        }

        protected override void OnOpenGlRender(GlInterface _, int fb)
        {
            if (_view != null) {
                Substitutes.PaintEventArgs paintEventArgs = new Substitutes.PaintEventArgs()
                {
                    ClipRectangle = new Substitutes.Rectangle(0, 0, (int)Bounds.Width, (int)Bounds.Height),
                    // ClipRectangle = new Substitutes.Rectangle((int)Bounds.X, (int)Bounds.Y, (int)Bounds.Width, (int)Bounds.Height),
                    Graphics = null // TODO should be fine, is there a better solution?
                };
                _view.OnPaint(paintEventArgs);
            }
            GlCheckError();

            // disabled redraw for debugging
            // Dispatcher.UIThread.Post(InvalidateVisual, DispatcherPriority.Background);
        }

        string VertexShaderSource => GetShader(@"
            layout(location = 0) in vec3 vertex;
            layout(location = 1) in vec3 normal;
            out vec3 fragmentPosition;
            out vec3 fragmentNormal;
            uniform mat4 modelview;
            uniform mat4 projection;

            void main()
            {
                fragmentPosition = vertex;
                fragmentNormal = normal;
                gl_Position = projection * modelview * vec4(vertex, 1.0);
            }
        ");

        string FragmentShaderSource => GetShader(@"
            in vec3 fragmentPosition;
            in vec3 fragmentNormal;
            layout(location = 0) out vec4 diffuseColor;
            uniform vec4 color;
            uniform vec3 lightPosition;
            uniform float ambientFactor;
            vec3 lightColor = vec3(1.0, 1.0, 1.0);

            void main()
            {
                vec3 normal = normalize(fragmentNormal);
                vec3 lightDirection = normalize(lightPosition - fragmentPosition);
                float diffuse = max(dot(normal, lightDirection), 0.0);
                diffuseColor = vec4((ambientFactor + diffuse) * lightColor * color.rgb, color.a);
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
            get { throw new NotImplementedException(); }
        }

        void IPaintTo3D.MakeCurrent()
        {
            // noop, handled by Avalonia
        }

        private void SetFragmentColor(Color color)
        {
            // _gl.Uniform4(colorLocation, (float)color.R / 255f, (float)color.G / 255f, (float)color.B / 255f, (float)color.A / 255f);
            _gl.Uniform4(colorLocation, (float)color.R / 255f, (float)color.G / 255f, (float)color.B / 255f, 1.0f);
            GlCheckError();
        }

        void IPaintTo3D.SetColor(Color color, int lockColor = 0)
        {
            Console.WriteLine("Setting color: " + color);
            Color res;
            if (!colorOverride)
            {
                if (color.R == _backgroundColor.R && color.G == _backgroundColor.G && color.B == _backgroundColor.B)
                {
                    if (color.R + color.G + color.B < 3 * 128)
                    {
                        res = Color.FromArgb(color.A, 255, 255, 255);
                    }
                    else
                    {
                        res = Color.FromArgb(color.A, 0, 0, 0);
                    }
                }
                else
                {
                    res = color;
                }

                if (currentVao != null) {
                    currentVao.Color = res;
                }
                else
                {
                    this.SetFragmentColor(res);
                }
            }
            if (lockColor == 1) colorOverride = true;
            else if (lockColor == -1) colorOverride = false;
        }
        void IPaintTo3D.AvoidColor(Color color)
        {
            _backgroundColor = color;
        }
        void IPaintTo3D.SetLineWidth(LineWidth lineWidth)
        {
            if (lineWidth != null) {
                Console.WriteLine("SetLineWidth: " + lineWidth.Width);
            }
            // throw new NotImplementedException();
        }
        void IPaintTo3D.SetLinePattern(LinePattern pattern)
        {
            // throw new NotImplementedException();
        }
        unsafe void IPaintTo3D.Polyline(GeoPoint[] points)
        {
            Console.WriteLine("Paint Polyline");

            VertexArrayObject vao = currentVao;
            if (vao == null) vao = new VertexArrayObject("single Polyline VAO", _gl);

            vao.addVertices(points, GLEnum.LineStrip, (uint)(3 * sizeof(float)));

            if (currentVao == null) {
                vao.Close();

                (this as IPaintTo3D).List(vao);
            }

            GlCheckError();
        }

        void IPaintTo3D.FilledPolyline(GeoPoint[] points)
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.Points(GeoPoint[] points, float size, PointSymbol pointSymbol)
        {
            throw new NotImplementedException();
        }
        unsafe void IPaintTo3D.Triangle(GeoPoint[] vertices, GeoVector[] normals, int[] indextriples)
        {
            VertexArrayObject vao = currentVao;
            if (vao == null) vao = new VertexArrayObject("single Triangle VAO", _gl);

            // TODO correctly sort indices depending on direction and enable culling

            Console.WriteLine("normals.Length: " + normals.Length);
            vao.addIndexedVertices(vertices, indextriples, GLEnum.Triangles, (uint)(6 * sizeof(float)), normals);
            // vao.addIndexedVertices(vertices, indextriples, GLEnum.Triangles, (uint)(3 * sizeof(float)));

            if (currentVao == null) {
                vao.Close();

                (this as IPaintTo3D).List(vao);
            }

            GlCheckError();
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
            throw new NotImplementedException();
        }
        void IPaintTo3D.PrepareBitmap(object bitmap)
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.RectangularBitmap(object bitmap, GeoPoint location, GeoVector directionWidth, GeoVector directionHeight)
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.Text(GeoVector lineDirection, GeoVector glyphDirection, GeoPoint location, string fontName, string textString, object fontStyle, CADability.GeoObject.Text.AlignMode alignment, CADability.GeoObject.Text.LineAlignMode lineAlignment)
        {

        }
        unsafe void IPaintTo3D.List(IPaintTo3DList paintThisList)
        {
            VertexArrayObject vao;
            if (paintThisList is VertexArrayObject) {
                vao = (VertexArrayObject)paintThisList;
            }
            else if (!vaos.TryGetValue(paintThisList.Name, out vao))
            {
                throw new ApplicationException("paintThisList does not exist: " + paintThisList.Name);
            }
            Console.WriteLine("IpaintTo3D.List: " + vao.Name);

            if (!vao.IsClosed) throw new ApplicationException("Can only paint closed VAOs");

            _gl.BindVertexArray(vao.VAO);

            if (vao.Color != null) {
                this.SetFragmentColor(vao.Color);
            }
            if (vao.ModelView != null) {
                _gl.UniformMatrix4(modelViewLocation, false, vao.ModelView);
                // TODO should we reset to identity after drawing?
            }

            if (!vao.HasIndices) {
                // TODO draw using vao.Segments to prevent line segments between different lines
                _gl.DrawArrays(vao.Primitive, 0, vao.FloatCount);
            }
            else
            {
                _gl.DrawElements(vao.Primitive, vao.IndicesCount, DrawElementsType.UnsignedInt, (void*)0);
            }

            _gl.BindVertexArray(0);
        }
        void IPaintTo3D.SelectedList(IPaintTo3DList paintThisList, int wobbleRadius)
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.Nurbs(GeoPoint[] poles, double[] weights, double[] knots, int degree)
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.Line2D(int sx, int sy, int ex, int ey)
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.Line2D(PointF p1, PointF p2)
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.FillRect2D(PointF p1, PointF p2)
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.Point2D(int x, int y)
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.DisplayIcon(GeoPoint p, object icon)
        {

        }
        void IPaintTo3D.DisplayBitmap(GeoPoint p, object bitmap)
        {
            throw new NotImplementedException();
        }

        // helper to get an identity matrix. TODO use library function?
        private float[] getIdentity4x4() {
            float[] mat = new float[16];
            mat[0] = 1.0f;
            mat[1] = 0.0f;
            mat[2] = 0.0f;
            mat[3] = 0.0f;
            mat[4] = 0.0f;
            mat[5] = 1.0f;
            mat[6] = 0.0f;
            mat[7] = 0.0f;
            mat[8] = 0.0f;
            mat[9] = 0.0f;
            mat[10] = 1.0f;
            mat[11] = 0.0f;
            mat[12] = 0.0f;
            mat[13] = 0.0f;
            mat[14] = 0.0f;
            mat[15] = 1.0f;
            return mat;
        }

        void IPaintTo3D.SetProjection(Projection projection, BoundingCube boundingCube)
        {
            _gl.Viewport(0, 0, (uint)Bounds.Width, (uint)Bounds.Height);

            // TODO correct culling
            _gl.Enable(GLEnum.CullFace);

            // the role of the bounding cube is to tell the projection, which z values are relevant
            // when the drawing is flat (z always 0) you cannot show temporary objects, which have a positive or negative z value
            // e.g. there is a circle and you want to shoe a sphere with the circle as equator. The sphere has positive and negative z values.
            // When the model changes, the bounding cube is automatically updated, but temporary objects don't change the bounding cube
            // There is no foolproof way to handle this, but at least the most common cases should work when we alway use a equilateral (regular) cube
            double size = Math.Max(boundingCube.XDiff, Math.Max(boundingCube.YDiff, boundingCube.ZDiff));
            GeoPoint center = boundingCube.GetCenter();
            BoundingCube boundingCubeEquilateral = new BoundingCube(new GeoPoint(center.x - size / 2, center.y - size / 2, center.z - size / 2),
                                                                    new GeoPoint(center.x + size / 2, center.y + size / 2, center.z + size / 2));

            double [,] mm = projection.GetOpenGLProjection(0, (int)Bounds.Width, 0, (int)Bounds.Height, boundingCubeEquilateral);
            float [,] mmFloat = new float[,] {
                { (float)mm[0, 0], (float)mm[0, 1], (float)mm[0, 2], (float)mm[0, 3]},
                { (float)mm[1, 0], (float)mm[1, 1], (float)mm[1, 2], (float)mm[1, 3]},
                { (float)mm[2, 0], (float)mm[2, 1], (float)mm[2, 2], (float)mm[2, 3]},
                { (float)mm[3, 0], (float)mm[3, 1], (float)mm[3, 2], (float)mm[3, 3]}
            };
            Matrix debugMatrix = DenseMatrix.OfArray(mmFloat);
            Console.WriteLine(debugMatrix);
            float[] pmat = new float[16];
            // ACHTUNG: Matrix ist vertauscht!!!
            pmat[0] = (float) mm[0, 0];
            pmat[1] = (float) mm[1, 0];
            pmat[2] = (float) mm[2, 0];
            pmat[3] = (float) mm[3, 0];
            pmat[4] = (float) mm[0, 1];
            pmat[5] = (float) mm[1, 1];
            pmat[6] = (float) mm[2, 1];
            pmat[7] = (float) mm[3, 1];
            pmat[8] = (float) mm[0, 2];
            pmat[9] = (float) mm[1, 2];
            pmat[10] = (float) mm[2, 2];
            pmat[11] = (float) mm[3, 2];
            pmat[12] = (float) mm[0, 3];
            pmat[13] = (float) mm[1, 3];
            pmat[14] = (float) mm[2, 3];
            pmat[15] = (float) mm[3, 3];

            // GeoVector v = projection.Direction;
            projectionDirection = projection.Direction;
            isPerspective = projection.IsPerspective;
            // TODO use v for lighting position
            GeoVector v;
            // v = projection.InverseProjection * new GeoVector(0.5, 0.3, -1.0);
            v = projection.InverseProjection * new GeoVector(100.0, 300.0, 1000.0);
            Console.WriteLine("Light Position: " + v);
            pixelToWorld = projection.DeviceToWorldFactor;
            _gl.Enable(GLEnum.DepthTest);

            float[] modelViewMat = getIdentity4x4();
            _gl.UniformMatrix4(modelViewLocation, 1, false, modelViewMat);
            _gl.UniformMatrix4(projectionLocation, 1, false, pmat);

            _gl.Uniform3(lightPositionLocation, (float)v.x, (float)v.y, (float)v.z);
            _gl.Uniform1(ambientFactorLocation, 0.2f);
            GlCheckError();
        }
        void IPaintTo3D.Clear(Color background)
        {
            _backgroundColor = background;
            _gl.Viewport(0, 0, (uint)Bounds.Width, (uint)Bounds.Height);
            _gl.ClearColor(background.R / 255.0f, background.G / 255.0f, background.B / 255.0f, 1.0f);
            _gl.Clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            GlCheckError();
        }
        void IPaintTo3D.Resize(int width, int height)
        {
            // TODO save bounds?
        }
        void IPaintTo3D.OpenList(string name)
        {
            if (name == null) throw new ApplicationException("List name cannot be empty.");
            if (currentVao != null) throw new ApplicationException("VAOs cannot be nested!");

            Console.WriteLine("OpenList: " + name);

            currentVao = new VertexArrayObject(name, _gl);
            // this overwrites any previously existing vao with this name
            vaos[name] = currentVao;
            GlCheckError();
        }
        IPaintTo3DList IPaintTo3D.CloseList()
        {
            Console.WriteLine("Close List: " + currentVao.Name);
            if (currentVao != null) currentVao.Close();
            VertexArrayObject res = currentVao;
            currentVao = null;
            GlCheckError();
            return res;
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
            throw new NotImplementedException();
        }
        void IPaintTo3D.UseZBuffer(bool use)
        {
            if (use)
            {
                _gl.Enable(GLEnum.DepthTest);
                _gl.DepthFunc(GLEnum.Always); // TODO this makes the test always pass
            }
            else
            {
                _gl.Disable(GLEnum.DepthTest);
            }
        }
        void IPaintTo3D.Blending(bool on)
        {
            // throw new NotImplementedException();
        }
        void IPaintTo3D.FinishPaint()
        {
            // throw new NotImplementedException();
        }
        unsafe void IPaintTo3D.PaintFaces(PaintTo3D.PaintMode paintMode)
        {
            Console.WriteLine("PaintFaces: " + paintMode);
            if (paintMode == PaintTo3D.PaintMode.FacesOnly)
            {
                if (isPerspective)
                {
                    Console.WriteLine("Perspective Mode");
                }
                else
                {
                    // TODO is this a operation used in display list?
                    // then we would have to save the matrix to VertexBufferObject
                    Matrix modelViewMat = DenseMatrix.OfArray(new float[,] {
                        { 1.0f, 0.0f, 0.0f, (float)(2 * precision * projectionDirection.x)},
                        { 0.0f, 1.0f, 0.0f, (float)(2 * precision * projectionDirection.y)},
                        { 0.0f, 0.0f, 1.0f, (float)(2 * precision * projectionDirection.z)},
                        { 0.0f, 0.0f, 0.0f, 1.0f }
                    });
                    Console.WriteLine("Paint Faces, modelView Matrix: " + modelViewMat.ToString());

                    if (currentVao != null)
                    {
                        currentVao.ModelView = modelViewMat.ToColumnMajorArray();
                    }
                    else
                    {
                        _gl.UniformMatrix4(modelViewLocation, false, modelViewMat.ToColumnMajorArray());
                    }
                    GlCheckError();

                }
                paintSurfaces = true;
                paintEdges = false;
            }
            else if (paintMode == PaintTo3D.PaintMode.CurvesOnly)
            {
                Matrix identity = DenseMatrix.CreateIdentity(4);
                _gl.UniformMatrix4(modelViewLocation, false, identity.ToColumnMajorArray());
                paintSurfaces = false;
                paintEdges = true;
            }
            else if (paintMode == PaintTo3D.PaintMode.All)
            {
                Matrix identity = DenseMatrix.CreateIdentity(4);
                _gl.UniformMatrix4(modelViewLocation, false, identity.ToColumnMajorArray());
                paintSurfaces = true;
                paintEdges = true;
            }
            else
            {
                throw new NotImplementedException("Invalid paintMode");
            }
            GlCheckError();
        }
        void IPaintTo3D.Dispose()
        {

        }
        void IPaintTo3D.PushState()
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.PopState()
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.PushMultModOp(ModOp insertion)
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.PopModOp()
        {
            throw new NotImplementedException();
        }
        void IPaintTo3D.SetClip(Substitutes.Rectangle clipRectangle)
        {
            throw new NotImplementedException();
        }

        public IView View {
            set => _view = value;
        }

        internal class VertexArrayObject : IPaintTo3DList
        {
            private string name;
            private GL _gl;
            private uint vertexArrayObject;
            private uint vertexBufferObject;
            private uint elementBufferObject;
            private List<float> vertices;
            private List<uint> indices;
            private GLEnum primitiveType;
            private bool hasIndices;
            private bool hasNormals;
            private bool closed;
            private uint floatCount;
            private uint indicesCount;
            private uint stride;
            private Substitutes.Color color;
            private float[] modelView; // column major order 4x4 matrix
            private List<(uint, uint)> segments;

            public VertexArrayObject(string name, GL gl)
            {
                this.name = name;
                this._gl = gl;
                this.closed = false;
                this.hasIndices = false;
                this.hasNormals = false;
                this.primitiveType = GLEnum.False;
                this.stride = 0;
                vertexArrayObject = _gl.GenVertexArray();
                vertexBufferObject = _gl.GenBuffer();
                elementBufferObject = _gl.GenBuffer();
                vertices = new List<float>();
                indices = new List<uint>();
                segments = new List<(uint, uint)>();
            }

            public string Name
            {
                get { return name; }
                set { throw new NotImplementedException(); }
            }
            public List<IPaintTo3DList> containedSubLists
            {
                set { throw new NotImplementedException(); }
            }

            public uint VAO => vertexArrayObject;
            public uint VBO => vertexBufferObject;
            public uint EBO => elementBufferObject;
            public GLEnum Primitive => primitiveType;

            public bool HasIndices => hasIndices;
            public uint FloatCount => floatCount;
            public uint IndicesCount => indicesCount;
            public uint Stride => stride;
            public bool IsClosed => closed;
            public Substitutes.Color Color { set; get; }
            public float[] ModelView { set; get; }

            public void addVertices(GeoPoint[] points, GLEnum primitiveType, uint stride, GeoVector[] normals = null) {
                if (this.primitiveType != GLEnum.False && primitiveType != this.primitiveType) {
                    throw new ApplicationException("Only supports one primitive type in a VertexArrayObject");
                }
                if (this.stride != 0 && this.stride != stride) {
                    throw new ApplicationException("Cannot change stride of VertexArrayObject");
                }
                if (closed) throw new ApplicationException("Cannot add to closed VertexArrayObject");

                if (normals == null)
                {
                    this.vertices.AddRange(points.SelectMany<GeoPoint, float>( vertex => new float[]{(float)vertex.x, (float)vertex.y, (float)vertex.z}));
                }
                else {
                    this.hasNormals = true;
                    this.vertices.AddRange(
                        points
                            .Zip(normals, (p, n) => new float[]{(float)p.x, (float)p.y, (float)p.z, (float)n.x, (float)n.y, (float)n.z})
                            .SelectMany(f => f)
                        );
                }
                this.stride = stride;
                this.primitiveType = primitiveType;
            }

            public void addIndexedVertices(GeoPoint[] points, int[] indices, GLEnum primitiveType, uint stride, GeoVector[] normals = null) {
                // only apply an offset if not drawing in segments and offsetting the vertex attrib pointer
                uint offset = (uint) (this.vertices.Count / (stride / sizeof(float)));

                this.addVertices(points, primitiveType, stride, normals);

                this.indices.AddRange(indices.Select(i => (uint) i + offset));
                this.hasIndices = true;
                this.primitiveType = primitiveType;
            }

            unsafe public void Close()
            {
                closed = true;
                _gl.BindVertexArray(vertexArrayObject);
                _gl.BindBuffer(BufferTargetARB.ArrayBuffer, vertexBufferObject);
                float[] verticesArray = vertices.ToArray();
                fixed(float* pData = verticesArray)
                    _gl.BufferData(BufferTargetARB.ArrayBuffer, (nuint)(sizeof(float) * vertices.Count),
                    pData, BufferUsageARB.StaticDraw);

                if (hasIndices) {
                    _gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, elementBufferObject);
                    uint[] indicesArray = indices.ToArray();
                    fixed(uint* pIData = indicesArray)
                        _gl.BufferData(BufferTargetARB.ElementArrayBuffer, (nuint)(sizeof(uint) * indices.Count),
                        pIData, BufferUsageARB.StaticDraw);
                }

                // TODO VertexAttribPointer here or when drawing in List?
                _gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, stride, (void*)0);
                _gl.EnableVertexAttribArray(0);

                if (this.hasNormals)
                {
                    _gl.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, stride, (void*)(3 * sizeof(float)));
                    _gl.EnableVertexAttribArray(1);
                }

                _gl.BindVertexArray(vertexArrayObject);
                floatCount = (uint)vertices.Count;
                indicesCount = (uint)indices.Count;
                vertices.Clear();
                indices.Clear();
                vertices = null;
                indices = null;
            }

            public void Dispose()
            {
                _gl.DeleteBuffer(vertexBufferObject);
                _gl.DeleteBuffer(elementBufferObject);
                _gl.DeleteVertexArray(vertexArrayObject);
            }
        }
    }
}
