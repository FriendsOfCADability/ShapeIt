using Avalonia;
using Avalonia.Controls;
using Avalonia.OpenGL;
using Avalonia.OpenGL.Controls;
using CADability.GeoObject;
using CADability.Substitutes;
using CADability.UserInterface;
using System;
using System.Numerics;
using static Avalonia.OpenGL.GlConsts;

namespace CADability.Avalonia
{
    public partial class CadCanvas: UserControl, ICanvas
    {
        private static CADability.Substitutes.Rectangle Subst(Rect v)
        {
            return new Substitutes.Rectangle((int)v.X, (int)v.Y, (int)v.Width, (int)v.Height);
        }

        private PaintToOpenGL paintTo3D;
        private IFrame frame;
        private IView view;
        private String currentCursor;

        public CadCanvas()
        {
            InitializeComponent();
            // CadCanvasControl canvasControl = new CadCanvasControl();
            paintTo3D = new PaintToOpenGL(1e-6);
            this.Content = paintTo3D;
        }

        void ICanvas.Invalidate() {}

        Rectangle ICanvas.ClientRectangle => Subst(base.Bounds);
        public IFrame Frame { get; set; }

        string ICanvas.Cursor
        {
            get { return currentCursor; }
            set { currentCursor = value; } //TODO
        }

        IPaintTo3D ICanvas.PaintTo3D
        {
            get { return paintTo3D; }
        }

        public event Action<ICanvas> OnPaintDone;

        void ICanvas.ShowView(IView toShow)
        {
            view = toShow;
            // TODO init paintTo3D here or in constructor?
            paintTo3D.View = view;
            // TODO view.Connect needed?
            view.Connect(this);
        }

        IView ICanvas.GetView()
        {
            return view;
        }

        Substitutes.Point ICanvas.PointToClient(Substitutes.Point mousePosition)
        {
            throw new NotImplementedException();
        }

        void ICanvas.ShowContextMenu(MenuWithHandler[] contextMenu, Substitutes.Point viewPosition, System.Action<int> collapsed) {}

        Substitutes.DragDropEffects ICanvas.DoDragDrop(GeoObjectList dragList, Substitutes.DragDropEffects all)
        {
            throw new NotImplementedException();
        }

        void ICanvas.ShowToolTip(string toDisplay) {}
    }
}
