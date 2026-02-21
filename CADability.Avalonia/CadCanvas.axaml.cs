using AvaloniaBase = Avalonia;
using Avalonia.Controls;
using AvaloniaKeyEventArgs = Avalonia.Input.KeyEventArgs;
using Avalonia.Input;
using Avalonia.Interactivity;
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
        private static CADability.Substitutes.Rectangle Subst(AvaloniaBase.Rect v)
        {
            return new Substitutes.Rectangle((int)v.X, (int)v.Y, (int)v.Width, (int)v.Height);
        }

        private PaintToOpenGL paintTo3D;
        private IFrame frame;
        private IView view;
        private String currentCursor;
        public Substitutes.Point CurrentMousePosition { get; private set; }
        public Substitutes.Keys ModifierKeys { get; private set; }

        public CadCanvas()
        {
            InitializeComponent();
            // CadCanvasControl canvasControl = new CadCanvasControl();
            paintTo3D = new PaintToOpenGL(1e-6);
            paintTo3D.CadCanvas = this;
            this.Content = paintTo3D;
            this.CurrentMousePosition = new Substitutes.Point(0, 0);
            this.ModifierKeys = Substitutes.Keys.None;
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

        public void OnPaint(PaintEventArgs e)
        {
            view.OnPaint(e);
            OnPaintDone?.Invoke(this);
        }

        void ICanvas.ShowView(IView toShow)
        {
            view = toShow;
            // TODO init paintTo3D here or in constructor?
            // TODO view.Connect needed?
            view.Connect(this);
        }

        IView ICanvas.GetView()
        {
            return view;
        }

        Substitutes.Point ICanvas.PointToClient(Substitutes.Point mousePosition)
        {
            // should already be client coordinates with avalonia
            return mousePosition;
        }

        void ICanvas.ShowContextMenu(MenuWithHandler[] contextMenu, Substitutes.Point viewPosition, System.Action<int> collapsed) {
            throw new NotImplementedException();
        }

        Substitutes.DragDropEffects ICanvas.DoDragDrop(GeoObjectList dragList, Substitutes.DragDropEffects all)
        {
            throw new NotImplementedException();
        }

        void ICanvas.ShowToolTip(string toDisplay) {}

        protected override void OnLoaded(RoutedEventArgs e)
        {
            var topLevel = TopLevel.GetTopLevel(this)!;
            topLevel.KeyDown += OnKeyDown;
            topLevel.KeyUp += OnKeyUp;
            topLevel.PointerPressed += OnPointerPressed;
            topLevel.PointerReleased += OnPointerReleased;
            topLevel.PointerEntered += OnPointerEntered;
            topLevel.PointerExited += OnPointerExited;
            topLevel.PointerMoved += OnPointerMoved;
            topLevel.PointerWheelChanged += OnPointerWheel;
            base.OnLoaded(e);
        }

        private Substitutes.Point Subst(AvaloniaBase.Point mousePosition)
        {
            return new Substitutes.Point((int)mousePosition.X, (int)mousePosition.Y);
        }

        private Substitutes.MouseButtons SubstButton(PointerEventArgs e)
        {
            if (e is PointerReleasedEventArgs) {
                var mb = (e as PointerReleasedEventArgs)?.InitialPressMouseButton;
                if (Enum.TryParse(mb.ToString(), out Substitutes.MouseButtons res)) return res;

            } else {
                if (e.Properties.IsLeftButtonPressed) return Substitutes.MouseButtons.Left;
                if (e.Properties.IsRightButtonPressed) return Substitutes.MouseButtons.Right;
                if (e.Properties.IsMiddleButtonPressed) return Substitutes.MouseButtons.Middle;
                if (e.Properties.IsXButton1Pressed) return Substitutes.MouseButtons.XButton1;
                if (e.Properties.IsXButton2Pressed) return Substitutes.MouseButtons.XButton2;
            }
            return Substitutes.MouseButtons.None;
        }

        private CADability.Substitutes.MouseEventArgs Subst(PointerEventArgs v)
        {
            int clicks = (v is PointerPressedEventArgs) ? (v as PointerPressedEventArgs).ClickCount : 0;
            int delta = (v is PointerWheelEventArgs) ? (int)((v as PointerWheelEventArgs).Delta.Y*10) : 0;
            return new Substitutes.MouseEventArgs()
            {
                Button = SubstButton(v),
                Clicks = clicks,
                X = (int)v.GetPosition(this).X,
                Y = (int)v.GetPosition(this).Y,
                Delta = delta,
                Location = new Substitutes.Point((int)v.GetPosition(this).X, (int)v.GetPosition(this).Y)
            };
        }

        private Substitutes.Keys Subst(KeyModifiers key)
        {
            if (key == KeyModifiers.Meta) {
                return Keys.LWin;
            }
            // TODO check if this works with multiple modifiers
            return Enum.TryParse(key.ToString(), out Substitutes.Keys res) ? res : Substitutes.Keys.None;
        }

        private void OnKeyDown(object sender, AvaloniaKeyEventArgs e)
        {
            ModifierKeys = Subst(e.KeyModifiers);
            base.OnKeyDown(e);
        }

        private void OnKeyUp(object sender, AvaloniaKeyEventArgs e)
        {
            ModifierKeys = Subst(e.KeyModifiers);
            base.OnKeyUp(e);
        }

        private void OnPointerPressed(object sender, PointerPressedEventArgs e)
        {
            ModifierKeys = Subst(e.KeyModifiers);
            view?.OnMouseDown(Subst(e));
            base.OnPointerPressed(e);
        }

        private void OnPointerReleased(object sender, PointerReleasedEventArgs e)
        {
            ModifierKeys = Subst(e.KeyModifiers);
            view?.OnMouseUp(Subst(e));
            base.OnPointerReleased(e);
        }

        private void OnPointerEntered(object sender, PointerEventArgs e)
        {
            view?.OnMouseEnter(e);
            base.OnPointerEntered(e);
        }

        private void OnPointerExited(object sender, PointerEventArgs e)
        {
            view?.OnMouseLeave(e);
            base.OnPointerEntered(e);
        }

        private void OnPointerMoved(object sender, PointerEventArgs e)
        {
            ModifierKeys = Subst(e.KeyModifiers);
            CurrentMousePosition = Subst(e.GetPosition(this));
            view?.OnMouseMove(Subst(e));
            base.OnPointerMoved(e);
        }

        private void OnPointerWheel(object sender, PointerWheelEventArgs e)
        {
            ModifierKeys = Subst(e.KeyModifiers);
            view?.OnMouseWheel(Subst(e));
            base.OnPointerWheelChanged(e);
        }
        // TODO more mouse handling
    }
}
