using System;
using System.Collections.Generic;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Primitives;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Platform;
using Avalonia.Threading;
using CADability;
using CADability.Avalonia;
using CADability.GeoObject;
using CADability.UserInterface;
using Point = CADability.Substitutes.Point;
using Rectangle = CADability.Substitutes.Rectangle;
using DragDropEffects = CADability.Substitutes.DragDropEffects;
using Substitutes = CADability.Substitutes;

namespace ShapeIt.Browser
{
    /// <summary>
    /// Browser replacement for CADability.Avalonia's CadCanvas. A plain Avalonia Control
    /// (NOT OpenGlControlBase, which the browser backend does not support) implementing
    /// ICanvas. The 3-D scene is rendered offscreen via WebGL (PaintToWebGL), the pixels are
    /// read back into a WriteableBitmap, and this control DRAWS that bitmap — so the 3-D is
    /// real Avalonia content and popups/menus naturally render on top of it (the desktop
    /// behaviour). Input, cursor, keyboard and drag handling mirror CadCanvas.
    /// </summary>
    public partial class Gl3DViewport : Control, ICanvas, CADability.Avalonia.IModifierKeyProvider
    {
        private readonly PaintToWebGL _painter = new();
        private IView? _view;
        private string _cursor = "";
        private Rectangle _lastClientRect;
        private bool _glInitialised;
        private bool _renderScheduled;

        private WriteableBitmap? _bmp;
        private byte[]? _buf;

        // Letzte bekannte Modifier-Tasten und Screen-Position aus Pointer-Events.
        // Werden von CadFrame.ModifierKeys / CurrentMousePosition über IModifierKeyProvider gelesen.
        private PointerEventArgs? _lastPointerArgs;
        private global::Avalonia.PixelPoint _lastScreenPos;
        public global::Avalonia.Input.KeyModifiers LastKeyModifiers
            => (_lastPointerArgs?.KeyModifiers ?? global::Avalonia.Input.KeyModifiers.None) | _injectedModifiers;
        public global::Avalonia.PixelPoint LastScreenPosition => _lastScreenPos;

        public Gl3DViewport()
        {
            Focusable = true;
            PaintTo3D = _painter;
        }

        public override void Render(DrawingContext context)
        {
            if (_bmp != null)
                context.DrawImage(_bmp, new Rect(Bounds.Size));
            else
                context.FillRectangle(Brushes.Transparent, new Rect(Bounds.Size)); // hit-testable until first frame
            base.Render(context);
        }

        private double RenderScale => TopLevel.GetTopLevel(this)?.RenderScaling ?? 1.0;
        private int PhysWidth => Math.Max(1, (int)Bounds.Width);
        private int PhysHeight => Math.Max(1, (int)Bounds.Height);

        // ── ICanvas: Frame / PaintTo3D ─────────────────────────────────────
        public IFrame? Frame { get; set; }
        IFrame ICanvas.Frame => Frame!;
        public IPaintTo3D? PaintTo3D { get; private set; }
        IPaintTo3D ICanvas.PaintTo3D => PaintTo3D!;

        // ── ICanvas: Cursor ────────────────────────────────────────────────
        // Avalonia.Input.InputElement already defines Cursor (type Cursor?), so
        // ICanvas.Cursor (type string) must be implemented explicitly.
        //
        // Resolution mirrors CadCanvas.ApplyCursor in structure (cache → resolve →
        // apply). NOTE: the embedded SVG / *.cur cursor helpers used by CadCanvas
        // (SvgCursorHelper, CurCursorHelper, SkiaSvgRenderer) are 'internal' to the
        // CADability.Avalonia assembly and there is no InternalsVisibleTo for
        // ShapeIt.Browser, so they cannot be called from here. The browser head also
        // does not reference SkiaSharp/Svg.Skia. We therefore resolve to the matching
        // StandardCursorType and fall back to the default arrow. The cache and the
        // name → StandardCursorType table mirror CadCanvas so behaviour stays in sync.
        private static readonly Dictionary<string, Cursor?> _cursorCache = new();

        string ICanvas.Cursor
        {
            get => _cursor;
            set
            {
                value ??= "";
                if (_cursor == value) return;
                _cursor = value;
                if (Dispatcher.UIThread.CheckAccess())
                    ApplyCursor(value);
                else
                    Dispatcher.UIThread.Post(() => ApplyCursor(value));
            }
        }

        private void ApplyCursor(string value)
        {
            if (!_cursorCache.TryGetValue(value, out var cursor))
            {
                cursor = MapStandardCursor(value) ?? Cursor.Default;
                _cursorCache[value] = cursor;
            }
            Cursor = cursor ?? Cursor.Default;
        }

        private static Cursor? MapStandardCursor(string value) => value switch
        {
            "Cross" => new Cursor(StandardCursorType.Cross),
            "Help" => new Cursor(StandardCursorType.Help),
            "IBeam" => new Cursor(StandardCursorType.Ibeam),
            "No" => new Cursor(StandardCursorType.No),
            "Move" or "SizeAll" => new Cursor(StandardCursorType.SizeAll),
            "SizeNESW" => new Cursor(StandardCursorType.BottomLeftCorner),
            "SizeNWSE" => new Cursor(StandardCursorType.TopLeftCorner),
            "SizeNS" => new Cursor(StandardCursorType.SizeNorthSouth),
            "SizeWE" => new Cursor(StandardCursorType.SizeWestEast),
            "UpArrow" => new Cursor(StandardCursorType.UpArrow),
            "WaitCursor" => new Cursor(StandardCursorType.Wait),
            "Hand" => new Cursor(StandardCursorType.Hand),
            "Arrow" or "Default" => Cursor.Default,
            _ => null,
        };

        // ── ICanvas: layout / view ─────────────────────────────────────────
        public Rectangle ClientRectangle => new(0, 0, PhysWidth, PhysHeight);
        public event Action<ICanvas>? OnPaintDone;

        public void ShowView(IView toShow)
        {
            _view?.Disconnect(this);
            _view = toShow;
            // Enable drag-drop if the view supports it (mirrors CadCanvas).
            DragDrop.SetAllowDrop(this, toShow.AllowDrop);
            toShow.Connect(this);
            Invalidate();
        }

        public IView GetView() => _view!;

        public void Invalidate()
        {
            if (!Dispatcher.UIThread.CheckAccess()) { Dispatcher.UIThread.Post(Invalidate); return; }
            if (_renderScheduled) return;
            _renderScheduled = true;
            Dispatcher.UIThread.Post(RenderFrame, DispatcherPriority.Render);
        }

        private unsafe void RenderFrame()
        {
            _renderScheduled = false;
            if (_view == null) return;
            int w = PhysWidth, h = PhysHeight;
            if (w <= 1 || h <= 1) return;

            if (!_glInitialised) { WebGLInterop.Init("gl"); _glInitialised = true; }

            // Render the scene offscreen into the WebGL canvas.
            WebGLInterop.SetSize(w, h);
            ((IPaintTo3D)_painter).Resize(w, h);
            _view.OnPaint(new Substitutes.PaintEventArgs { ClipRectangle = new Rectangle(0, 0, w, h) });

            // Read the pixels back and copy (Y-flipped) into the Avalonia bitmap.
            int needed = w * h * 4;
            if (_buf == null || _buf.Length != needed) _buf = new byte[needed];
            WebGLInterop.ReadPixels(new ArraySegment<byte>(_buf), w, h);

            if (_bmp == null || _bmp.PixelSize.Width != w || _bmp.PixelSize.Height != h)
                _bmp = new WriteableBitmap(new PixelSize(w, h), new Vector(96.0, 96.0),
                                           PixelFormat.Rgba8888, AlphaFormat.Opaque);

            int rowBytes = w * 4;
            using (var fb = _bmp.Lock())
            fixed (byte* src = _buf)
            {
                byte* dst = (byte*)fb.Address;
                for (int y = 0; y < h; y++)
                    Buffer.MemoryCopy(src + (h - 1 - y) * rowBytes, dst + y * fb.RowBytes, rowBytes, rowBytes);
            }

            OnPaintDone?.Invoke(this);
            InvalidateVisual(); // draw the updated bitmap
        }

        public void ShowToolTip(string? toDisplay)
        {
            // Simple tooltip wiring on top of the viewport itself. CadCanvas leaves this
            // as a no-op ("tooltip control not yet wired"); the browser head can manage a
            // single tip via the attached ToolTip properties without extra plumbing.
            if (!Dispatcher.UIThread.CheckAccess())
            {
                Dispatcher.UIThread.Post(() => ShowToolTip(toDisplay));
                return;
            }
            if (string.IsNullOrEmpty(toDisplay))
            {
                ToolTip.SetIsOpen(this, false);
                ToolTip.SetTip(this, null);
            }
            else
            {
                ToolTip.SetTip(this, toDisplay);
                ToolTip.SetIsOpen(this, true);
            }
        }

        public void ShowContextMenu(MenuWithHandler[] contextMenu, Point viewPosition, Action<int>? collapsed = null)
        {
            // CadCanvas uses MenuManager.MakeContextMenu(...) which returns the internal
            // ContextMenuWithHandler. Both the factory method and its return type are
            // 'internal' to CADability.Avalonia and there is no InternalsVisibleTo for
            // ShapeIt.Browser, so we cannot reuse them. We build an equivalent Avalonia
            // ContextMenu from the same MenuWithHandler definitions using only public
            // APIs (MenuWithHandler.ID/Text/Target/SubMenus, MenuManager.LoadMenuIcon,
            // MenuManager.TryParseShortcut), refresh command state on open, fire the
            // 'collapsed' callback on close, place it at the pointer, and open it on this.
            var cm = BuildContextMenu(contextMenu);
            Trace($"CTXMENU n={contextMenu.Length} items={cm.Items.Count} at={viewPosition.X},{viewPosition.Y}");
            cm.Closing += (s, e) => collapsed?.Invoke(0);
            cm.Placement = PlacementMode.Pointer;
            cm.Open(this);
        }

        private static ContextMenu BuildContextMenu(MenuWithHandler[] definitions)
        {
            var cm = new ContextMenu();
            foreach (var def in definitions)
            {
                if (def.Text == "-")
                    cm.Items.Add(new Separator());
                else
                    cm.Items.Add(BuildMenuItem(def));
            }
            // Refresh enabled/checked state just before the menu becomes visible
            // (mirrors MenuManager.MakeContextMenu's Opening → UpdateAll).
            cm.Opening += (s, e) => UpdateMenuItems(cm.Items);
            return cm;
        }

        private static MenuItem BuildMenuItem(MenuWithHandler def)
        {
            var item = new MenuItem { Header = def.Text, Tag = def };

            if (!string.IsNullOrEmpty(def.Shortcut) &&
                MenuManager.TryParseShortcut(def.Shortcut, out var gesture))
            {
                item.InputGesture = gesture;
            }

            var icon = MenuManager.LoadMenuIcon(def.ID);
            if (icon != null)
                item.Icon = new Image { Source = icon, Width = 16, Height = 16 };

            if (def.SubMenus != null)
            {
                foreach (var sub in def.SubMenus)
                {
                    if (sub.Text == "-")
                        item.Items.Add(new Separator());
                    else
                        item.Items.Add(BuildMenuItem(sub));
                }
            }
            else
            {
                item.Click += OnMenuItemClick;
            }
            return item;
        }

        private static void OnMenuItemClick(object? sender, RoutedEventArgs e)
        {
            if (sender is MenuItem mi && mi.Tag is MenuWithHandler def)
                def.Target?.OnCommand(def.ID);
        }

        private static void UpdateMenuItems(ItemCollection items)
        {
            foreach (var it in items)
            {
                if (it is not MenuItem mi) continue;
                if (mi.Tag is MenuWithHandler def && def.Target != null)
                {
                    var state = new CommandState();
                    try { def.Target.OnUpdateCommand(def.ID, state); }
                    catch { /* keep menu usable even if a handler throws */ }
                    mi.IsEnabled = state.Enabled;
                    if (state.Checked || state.Radio)
                    {
                        mi.ToggleType = MenuItemToggleType.CheckBox;
                        mi.IsChecked = true;
                    }
                    else
                    {
                        mi.ToggleType = MenuItemToggleType.None;
                        mi.IsChecked = false;
                    }
                }
                if (mi.Items.Count > 0)
                    UpdateMenuItems(mi.Items);
            }
        }

        public DragDropEffects DoDragDrop(GeoObjectList dragList, DragDropEffects all)
            => throw new NotImplementedException("Drag initiation not implemented in the browser.");

        public Point PointToClient(Point screenPoint)
        {
            var topLeft = this.PointToScreen(new global::Avalonia.Point(0, 0));
            return new Point(screenPoint.X - topLeft.X, screenPoint.Y - topLeft.Y);
        }

        // ── size changes → re-render at the new resolution ────────────────
        protected override void OnPropertyChanged(AvaloniaPropertyChangedEventArgs change)
        {
            base.OnPropertyChanged(change);
            if (change.Property == BoundsProperty)
            {
                if (_view != null) _view.OnSizeChanged(_lastClientRect);
                Rectangle clr = ClientRectangle;
                if (clr.Width > 1 && clr.Height > 1) _lastClientRect = clr;
                Invalidate();
            }
        }

        // ── pointer / wheel (mirrors CadCanvas) ────────────────────────────
        protected override void OnPointerPressed(PointerPressedEventArgs e)
        {
            base.OnPointerPressed(e);
            _lastPointerArgs = e;
            _lastScreenPos = this.PointToScreen(e.GetPosition(this));
            Focus();
            if (_view == null) return;
            if (e.Pointer.Type == PointerType.Touch) { OnTouchPressed(e); return; }
            var pos = LocalPos(e);
            var button = MapPressed(e.GetCurrentPoint(this).Properties.PointerUpdateKind);
            // Remember the press for the stationary-left-click detection in OnPointerReleased
            // (mouse edge-priority pick). Multi-clicks keep the standard double-click handling.
            _mouseLeftPress = (button == Substitutes.MouseButtons.Left && e.ClickCount == 1) ? pos : null;
            // Keep pointer capture for middle-button drags and prevent the browser from
            // starting its autoscroll gesture (which otherwise hijacks the middle button).
            if (button == Substitutes.MouseButtons.Middle)
            {
                e.Pointer.Capture(this);
                e.Handled = true;
            }
            var args = MakeArgs(button, (int)pos.X, (int)pos.Y, clicks: e.ClickCount);
            if (e.ClickCount >= 2) _view.OnMouseDoubleClick(args); else _view.OnMouseDown(args);
            e.Handled = true;
        }

        protected override void OnPointerMoved(PointerEventArgs e)
        {
            base.OnPointerMoved(e);
            _lastPointerArgs = e;
            _lastScreenPos = this.PointToScreen(e.GetPosition(this));
            if (_view == null) return;
            if (e.Pointer.Type == PointerType.Touch) { OnTouchMoved(e); return; }
            var pos = LocalPos(e); var s = 1.0;
            _view.OnMouseMove(MakeArgs(MapButtons(e.GetCurrentPoint(this).Properties), (int)(pos.X * s), (int)(pos.Y * s)));
        }

        protected override void OnPointerReleased(PointerReleasedEventArgs e)
        {
            base.OnPointerReleased(e);
            _lastPointerArgs = e;
            _lastScreenPos = this.PointToScreen(e.GetPosition(this));
            if (_view == null) return;
            if (e.Pointer.Type == PointerType.Touch) { OnTouchReleased(e); return; }
            var pos = LocalPos(e); var s = 1.0;
            var button = e.InitialPressMouseButton switch
            {
                MouseButton.Left => Substitutes.MouseButtons.Left,
                MouseButton.Right => Substitutes.MouseButtons.Right,
                MouseButton.Middle => Substitutes.MouseButtons.Middle,
                _ => Substitutes.MouseButtons.None
            };
            _view.OnMouseUp(MakeArgs(button, (int)(pos.X * s), (int)(pos.Y * s)));
            // Mouse edge-priority pick: a stationary left CLICK (no drag) re-runs the modelling
            // pick with the same edge-priority logic the touch tap uses, at the mouse's precise
            // radius — the standard pick's depth filter drops edges next to a face for the mouse
            // too. Not with Ctrl (the click's own add/remove pick already toggled the selection;
            // a second compose would toggle it back) and not while an action runs.
            if (button == Substitutes.MouseButtons.Left && _mouseLeftPress is { } down
                && Dist(pos, down) <= 4 && !e.KeyModifiers.HasFlag(KeyModifiers.Control) && !IsActionRunning())
            {
                int radius = 5;
                try { radius = Frame?.GetIntSetting("Select.PickRadius", 5) ?? 5; } catch { }
                try { if (ModellingPick?.Invoke(new Point((int)pos.X, (int)pos.Y), radius, false) == true) Invalidate(); }
                catch { }
            }
            _mouseLeftPress = null;
        }

        private global::Avalonia.Point? _mouseLeftPress; // left-press position for click-vs-drag detection

        protected override void OnPointerWheelChanged(PointerWheelEventArgs e)
        {
            base.OnPointerWheelChanged(e);
            if (_view == null) return;
            var pos = LocalPos(e); var s = 1.0;
            _view.OnMouseWheel(MakeArgs(Substitutes.MouseButtons.None, (int)(pos.X * s), (int)(pos.Y * s), delta: (int)(e.Delta.Y * 120)));
        }

        protected override void OnPointerEntered(PointerEventArgs e) { base.OnPointerEntered(e); _view?.OnMouseEnter(EventArgs.Empty); }
        protected override void OnPointerExited(PointerEventArgs e) { base.OnPointerExited(e); _view?.OnMouseLeave(EventArgs.Empty); }

        protected override void OnKeyDown(KeyEventArgs e)
        {
            base.OnKeyDown(e);
            if (Frame == null) return;
            bool ctrl = e.KeyModifiers.HasFlag(KeyModifiers.Control);
            bool alt = e.KeyModifiers.HasFlag(KeyModifiers.Alt);
            bool pre = (e.Key >= Key.F1 && e.Key <= Key.F24) || e.Key is Key.Escape or Key.Delete or Key.Up or Key.Down or Key.Tab or Key.Enter || ctrl || alt;
            if (!pre) return;
            int vk = KeyToVk(e.Key);
            if (vk == 0) return;
            int mods = 0;
            if (ctrl) mods |= 0x20000;
            if (e.KeyModifiers.HasFlag(KeyModifiers.Shift)) mods |= 0x10000;
            if (alt) mods |= 0x40000;
            var subArgs = new Substitutes.KeyEventArgs((Substitutes.Keys)(vk | mods));
            (Frame as FrameImpl)?.PreProcessKeyDown(subArgs);
            if (subArgs.Handled) e.Handled = true;
        }

        private static int KeyToVk(Key key)
        {
            switch (key)
            {
                case Key.Back: return 0x08;
                case Key.Tab: return 0x09;
                case Key.Enter: return 0x0D;
                case Key.Escape: return 0x1B;
                case Key.Space: return 0x20;
                case Key.Left: return 0x25;
                case Key.Up: return 0x26;
                case Key.Right: return 0x27;
                case Key.Down: return 0x28;
                case Key.Delete: return 0x2E;
            }
            int ik = (int)key;
            if (ik >= (int)Key.D0 && ik <= (int)Key.D9) return ik - (int)Key.D0 + 0x30;
            if (ik >= (int)Key.A && ik <= (int)Key.Z) return ik - (int)Key.A + 0x41;
            if (ik >= (int)Key.F1 && ik <= (int)Key.F24) return ik - (int)Key.F1 + 0x70;
            return 0;
        }

        // ── Drag-drop receiving (mirrors CadCanvas) ────────────────────────
        protected override void OnAttachedToVisualTree(VisualTreeAttachmentEventArgs e)
        {
            base.OnAttachedToVisualTree(e);
            AddHandler(DragDrop.DropEvent,      OnDragDropReceived);
            AddHandler(DragDrop.DragOverEvent,  OnDragOverReceived);
            AddHandler(DragDrop.DragEnterEvent, OnDragEnterReceived);
            AddHandler(DragDrop.DragLeaveEvent, OnDragLeaveReceived);
            // Suppress the browser/OS native context menu so only CADability's own
            // context menu (opened on right-mouse-up via SelectActionContextMenu →
            // ShowContextMenu) appears. Handling ContextRequested in the tunnel phase
            // stops Avalonia/the host from surfacing the default menu.
            AddHandler(ContextRequestedEvent, OnContextRequested, RoutingStrategies.Tunnel | RoutingStrategies.Bubble);
        }

        protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
        {
            RemoveHandler(DragDrop.DropEvent,      OnDragDropReceived);
            RemoveHandler(DragDrop.DragOverEvent,  OnDragOverReceived);
            RemoveHandler(DragDrop.DragEnterEvent, OnDragEnterReceived);
            RemoveHandler(DragDrop.DragLeaveEvent, OnDragLeaveReceived);
            RemoveHandler(ContextRequestedEvent, OnContextRequested);
            base.OnDetachedFromVisualTree(e);
        }

        private static void OnContextRequested(object? sender, ContextRequestedEventArgs e)
        {
            // CADability builds and opens its own context menu from OnMouseUp; mark the
            // request handled so the native browser context menu is not shown as well.
            e.Handled = true;
        }

        private void OnDragDropReceived(object? sender, global::Avalonia.Input.DragEventArgs e)
        {
            if (_view == null) return;
            var pos = e.GetPosition(this); var s = RenderScale;
            var args = MakeDragArgs(e, (int)(pos.X * s), (int)(pos.Y * s));
            _view.OnDragDrop(args);
            e.DragEffects = (global::Avalonia.Input.DragDropEffects)(int)args.Effect;
        }

        private void OnDragOverReceived(object? sender, global::Avalonia.Input.DragEventArgs e)
        {
            if (_view == null) return;
            var pos = e.GetPosition(this); var s = RenderScale;
            var args = MakeDragArgs(e, (int)(pos.X * s), (int)(pos.Y * s));
            _view.OnDragOver(args);
            e.DragEffects = (global::Avalonia.Input.DragDropEffects)(int)args.Effect;
        }

        private void OnDragEnterReceived(object? sender, global::Avalonia.Input.DragEventArgs e)
        {
            if (_view == null) return;
            var pos = e.GetPosition(this); var s = RenderScale;
            _view.OnDragEnter(MakeDragArgs(e, (int)(pos.X * s), (int)(pos.Y * s)));
        }

        private void OnDragLeaveReceived(object? sender, global::Avalonia.Input.DragEventArgs e)
        {
            _view?.OnDragLeave(EventArgs.Empty);
        }

        private static Substitutes.DragEventArgs MakeDragArgs(
            global::Avalonia.Input.DragEventArgs e, int x, int y)
        {
            // DragDropEffects values match between Avalonia and CADability.Substitutes (Copy=1, Move=2, Link=4).
            var allowed = (DragDropEffects)(int)e.DragEffects;
            return new Substitutes.DragEventArgs
            {
                Data          = e.Data,
                X             = x,
                Y             = y,
                AllowedEffect = allowed,
                Effect        = allowed
            };
        }

        // ── helpers ────────────────────────────────────────────────────────
        // Control-local pointer position in the SAME (logical) units the projection /
        // framebuffer use (PhysWidth/PhysHeight == Bounds, see RenderFrame).
        //
        // Avalonia.Browser sizes its surface via ResizeObserver: physical px = CSS px ×
        // devicePixelRatio (devicePixelContentBoxSize where available), logical = physical /
        // RenderScaling = CSS px — and DOM pointer events are delivered in CSS px. So on real
        // hardware e.GetPosition(this) IS the correct logical position at every scale, and any
        // extra ×/÷ RenderScaling here lands clicks at the wrong spot on hi-DPI (iPad dpr=2,
        // 4K at 150/200%: picks registered at a fraction of the visual offset).
        //
        // Headless-test caveat: Chromium's DevTools dpr EMULATION reports
        // devicePixelContentBoxSize in unscaled CSS px (emulation artifact), which makes
        // Avalonia lay out at CSS/dpr while routing pointers in CSS px — clicks then land on
        // the wrong CONTROL at the Avalonia level. That is an artifact, not app behaviour;
        // do NOT compensate for it here (a previous /RenderScaling "fix" derived from such a
        // test is exactly what broke real devices).
        private global::Avalonia.Point LocalPos(PointerEventArgs e) => e.GetPosition(this);

        private static Substitutes.MouseEventArgs MakeArgs(Substitutes.MouseButtons button, int x, int y, int clicks = 0, int delta = 0) =>
            new() { Button = button, Clicks = clicks, X = x, Y = y, Delta = delta, Location = new Point(x, y) };

        private static Substitutes.MouseButtons MapPressed(PointerUpdateKind kind) => kind switch
        {
            PointerUpdateKind.LeftButtonPressed => Substitutes.MouseButtons.Left,
            PointerUpdateKind.RightButtonPressed => Substitutes.MouseButtons.Right,
            PointerUpdateKind.MiddleButtonPressed => Substitutes.MouseButtons.Middle,
            _ => Substitutes.MouseButtons.None
        };

        private static Substitutes.MouseButtons MapButtons(PointerPointProperties p)
        {
            var b = Substitutes.MouseButtons.None;
            if (p.IsLeftButtonPressed) b |= Substitutes.MouseButtons.Left;
            if (p.IsRightButtonPressed) b |= Substitutes.MouseButtons.Right;
            if (p.IsMiddleButtonPressed) b |= Substitutes.MouseButtons.Middle;
            return b;
        }
    }
}
