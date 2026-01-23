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

        private IPaintTo3D paintTo3D;
        private IFrame frame;
        private IView view;
        private String currentCursor;

        public CadCanvas()
        {
            InitializeComponent();
            // CadCanvasControl canvasControl = new CadCanvasControl();
            PaintToOpenGL openGlControl = new PaintToOpenGL();
            this.Content = openGlControl;
        }

        void ICanvas.Invalidate() {}

        Rectangle ICanvas.ClientRectangle => Subst(base.Bounds);

        IFrame ICanvas.Frame
        {
            get { return frame; }
        }

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

        void ICanvas.ShowView(IView toShow) {}

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
