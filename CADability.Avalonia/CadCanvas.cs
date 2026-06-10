using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Primitives;
using Avalonia.Input;
using Avalonia.Media;
using Avalonia.OpenGL;
using Avalonia.OpenGL.Controls;
using Avalonia.Threading;
using CADability;
using CADability.GeoObject;
using CADability.UserInterface;
using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using Point = CADability.Substitutes.Point;
using Rectangle = CADability.Substitutes.Rectangle;
using DragDropEffects = CADability.Substitutes.DragDropEffects;
using Substitutes = CADability.Substitutes;

namespace CADability.Avalonia;

/// <summary>
/// Avalonia OpenGL render control implementing ICanvas.
/// Replaces the Windows-Forms-based CadCanvas from CADability.Forms.NET8.
/// The OpenGL context is managed by Avalonia (IGlContext), not by WglContext.
/// </summary>
public class CadCanvas : OpenGlControlBase, ICanvas, IModifierKeyProvider
{
    public CadCanvas()
    {
        Focusable = true;
    }

    public override void Render(DrawingContext context)
    {
        // OpenGlControlBase has no Avalonia-visible content, so Avalonia's hit-testing
        // skips it and pointer events are never delivered. Drawing a transparent fill
        // registers the full bounds as a hit target without affecting the GL output.
        context.FillRectangle(Brushes.Transparent, new Rect(Bounds.Size));
        base.Render(context);
    }

    private PaintToOpenGL? _painter;
    private IView? _view;
    private string _cursor = "";
    private Rectangle _lastClientRect;

    // ── DPI / render scaling ───────────────────────────────────────────────
    // Avalonia Bounds are in logical pixels; the OpenGL framebuffer and
    // WinForms-originated CADability code all expect physical (device) pixels.
    // RenderScaling is 1.0 at 96 dpi (100 %), 1.25 at 125 %, 1.5 at 150 %, etc.
    private double RenderScale =>
        TopLevel.GetTopLevel(this)?.RenderScaling ?? 1.0;

    // Physical pixel width / height of the control.
    private int PhysWidth  => (int)(Bounds.Width  * RenderScale);
    private int PhysHeight => (int)(Bounds.Height * RenderScale);

    // ── ICanvas: Frame / PaintTo3D ─────────────────────────────────────────

    public IFrame? Frame { get; set; }
    IFrame ICanvas.Frame => Frame!;

    public IPaintTo3D? PaintTo3D { get; private set; }
    IPaintTo3D ICanvas.PaintTo3D => PaintTo3D!;

    // ── ICanvas: Cursor ────────────────────────────────────────────────────
    // Avalonia.Input.InputElement already defines Cursor (type Cursor?),
    // so ICanvas.Cursor (type string) must be implemented explicitly.
    //
    // Resolution mirrors CADability.Forms.NET8.CadCanvas: try an embedded SVG
    // cursor first, then a matching Avalonia StandardCursorType, then fall back
    // to the default arrow. Cursors that only exist as *.cur (a WinForms-only
    // format) are not embedded and therefore resolve to the arrow.
    private static readonly System.Collections.Generic.Dictionary<string, Cursor?> _cursorCache = new();

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
            // Render at a DPI-scaled size so the cursor stays crisp on hi-DPI displays.
            int size = Math.Max(16, (int)Math.Round(32 * RenderScale));
            // Resolution order mirrors CADability.Forms.NET8.CadCanvas:
            // embedded SVG → matching standard cursor → embedded *.cur → default arrow.
            cursor = SvgCursorHelper.CreateCursorFromEmbeddedSvg("Cursors." + value, size)
                     ?? MapStandardCursor(value)
                     ?? CurCursorHelper.CreateCursorFromEmbeddedCur("Cursors." + value)
                     ?? Cursor.Default;
            _cursorCache[value] = cursor;
        }
        Cursor = cursor ?? Cursor.Default;
    }

    private static Cursor? MapStandardCursor(string value) => value switch
    {
        "Cross"              => new Cursor(StandardCursorType.Cross),
        "Help"               => new Cursor(StandardCursorType.Help),
        "IBeam"              => new Cursor(StandardCursorType.Ibeam),
        "No"                 => new Cursor(StandardCursorType.No),
        "Move" or "SizeAll"  => new Cursor(StandardCursorType.SizeAll),
        "SizeNESW"           => new Cursor(StandardCursorType.BottomLeftCorner),
        "SizeNWSE"           => new Cursor(StandardCursorType.TopLeftCorner),
        "SizeNS"             => new Cursor(StandardCursorType.SizeNorthSouth),
        "SizeWE"             => new Cursor(StandardCursorType.SizeWestEast),
        "UpArrow"            => new Cursor(StandardCursorType.UpArrow),
        "WaitCursor"         => new Cursor(StandardCursorType.Wait),
        "Hand"               => new Cursor(StandardCursorType.Hand),
        "Arrow" or "Default" => Cursor.Default,
        // Unknown names return null so the caller can try a *.cur fallback before
        // settling on the default arrow.
        _                    => null,
    };

    // ── ICanvas: layout ────────────────────────────────────────────────────

    // ClientRectangle in physical pixels — matches WinForms behaviour and is the
    // coordinate space CADability uses for projections, hit-testing, etc.
    public Rectangle ClientRectangle =>
        new Rectangle(0, 0, PhysWidth, PhysHeight);

    public event Action<ICanvas>? OnPaintDone;

    // ── ICanvas: view management ───────────────────────────────────────────

    public void ShowView(IView toShow)
    {
        _view?.Disconnect(this);
        _view = toShow;
        // Enable drag-drop if the view supports it
        DragDrop.SetAllowDrop(this, toShow.AllowDrop);
        toShow.Connect(this);
        Invalidate();
    }

    public IView GetView() => _view!;

    // ── ICanvas: interactions ──────────────────────────────────────────────

    public void Invalidate()
    {
        if (Dispatcher.UIThread.CheckAccess())
            RequestNextFrameRendering();
        else
            Dispatcher.UIThread.Post(RequestNextFrameRendering);
    }

    public void ShowToolTip(string? toDisplay) { /* no-op — tooltip control not yet wired */ }

    public Point PointToClient(Point screenPoint)
    {
        // Convert screen pixel coordinates to control-local pixel coordinates.
        // PointToScreen(0,0) gives the control's top-left corner in screen pixels.
        var topLeft = this.PointToScreen(new global::Avalonia.Point(0, 0));
        return new Point(screenPoint.X - topLeft.X, screenPoint.Y - topLeft.Y);
    }

    public void ShowContextMenu(MenuWithHandler[] contextMenu, Point viewPosition, Action<int>? collapsed = null)
    {
        var cm = MenuManager.MakeContextMenu(contextMenu);
        cm.Closing += (s, e) => collapsed?.Invoke(0);
        cm.UpdateCommand();
        cm.Placement = PlacementMode.Pointer;
        cm.Open(this);
    }

    public DragDropEffects DoDragDrop(GeoObjectList dragList, DragDropEffects all)
    {
        // A drag is always triggered from a pointer move with the button down; that event
        // was cached in OnPointerMoved and is required by Avalonia as the drag trigger.
        if (_lastPointerArgs == null || dragList == null || dragList.Count == 0)
            return DragDropEffects.None;

        var data = new global::Avalonia.Input.DataObject();
        // In-process: carry the live object so there is no (potentially lossy) round-trip.
        data.Set(CadFrame.DragObjectFormat, dragList);
        // Cross-process fallback: the same JSON format the clipboard uses.
        try
        {
            using var ms = new MemoryStream();
            new JsonSerialize().ToStream(ms, dragList, closeStream: false);
            data.Set(CadFrame.ClipFormat, ms.ToArray());
        }
        catch { /* serialization is best-effort; the in-process reference still works */ }

        // CADability.Substitutes.DragDropEffects.All carries WinForms-only bits (Scroll);
        // mask to the flags Avalonia understands (Copy=1 | Move=2 | Link=4).
        var allowed = (global::Avalonia.Input.DragDropEffects)((int)all & 0x7);
        var task = global::Avalonia.Input.DragDrop.DoDragDrop(_lastPointerArgs, data, allowed);

        // The CADability core calls this synchronously and expects the resulting effect back.
        // On the Windows backend the drag runs a modal loop and the task is already complete
        // on return; on the managed backends we pump a nested dispatcher loop until the drag
        // finishes (same async→sync bridging rationale as CadFrame.RunDialogSync).
        if (!task.IsCompleted)
        {
            var cts = new CancellationTokenSource();
            task.ContinueWith(_ => cts.Cancel(), TaskScheduler.Default);
            try { Dispatcher.UIThread.MainLoop(cts.Token); }
            catch (OperationCanceledException) { }
        }

        return (DragDropEffects)(int)task.GetAwaiter().GetResult();
    }

    // Last pointer event — used as the trigger for DoDragDrop.
    private PointerEventArgs? _lastPointerArgs;

    // Letzte bekannte Modifier-Tasten und Screen-Position aus Pointer-Events.
    // Werden von CadFrame.ModifierKeys / CurrentMousePosition auf nicht-Windows-Plattformen genutzt.
    public KeyModifiers LastKeyModifiers => _lastPointerArgs?.KeyModifiers ?? KeyModifiers.None;
    private global::Avalonia.PixelPoint _lastScreenPos;
    public global::Avalonia.PixelPoint LastScreenPosition => _lastScreenPos;

    // ── Mouse / pointer events ─────────────────────────────────────────────

    protected override void OnPointerPressed(PointerPressedEventArgs e)
    {
        base.OnPointerPressed(e);
        _lastPointerArgs = e;
        _lastScreenPos = this.PointToScreen(e.GetPosition(this));
        Focus();
        if (_view == null) return;
        var pos    = e.GetPosition(this);
        var scale  = RenderScale;
        var button = MapPressedButton(e.GetCurrentPoint(this).Properties.PointerUpdateKind);
        var args   = MakeMouseArgs(button, (int)(pos.X * scale), (int)(pos.Y * scale), clicks: e.ClickCount);
        if (e.ClickCount >= 2)
            _view.OnMouseDoubleClick(args);
        else
            _view.OnMouseDown(args);
        e.Handled = true;
    }

    protected override void OnPointerMoved(PointerEventArgs e)
    {
        base.OnPointerMoved(e);
        _lastPointerArgs = e;
        _lastScreenPos = this.PointToScreen(e.GetPosition(this));
        if (_view == null) return;
        var pos    = e.GetPosition(this);
        var scale  = RenderScale;
        var button = MapCurrentButtons(e.GetCurrentPoint(this).Properties);
        _view.OnMouseMove(MakeMouseArgs(button, (int)(pos.X * scale), (int)(pos.Y * scale)));
    }

    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        base.OnPointerReleased(e);
        if (_view == null) return;
        var pos    = e.GetPosition(this);
        var scale  = RenderScale;
        var button = e.InitialPressMouseButton switch
        {
            MouseButton.Left   => Substitutes.MouseButtons.Left,
            MouseButton.Right  => Substitutes.MouseButtons.Right,
            MouseButton.Middle => Substitutes.MouseButtons.Middle,
            _                  => Substitutes.MouseButtons.None
        };
        _view.OnMouseUp(MakeMouseArgs(button, (int)(pos.X * scale), (int)(pos.Y * scale)));
    }

    protected override void OnPointerWheelChanged(PointerWheelEventArgs e)
    {
        base.OnPointerWheelChanged(e);
        if (_view == null) return;
        var pos   = e.GetPosition(this);
        var scale = RenderScale;
        // Avalonia Delta.Y is in scroll lines; WinForms uses 120 units per notch.
        int delta = (int)(e.Delta.Y * 120);
        _view.OnMouseWheel(MakeMouseArgs(Substitutes.MouseButtons.None,
            (int)(pos.X * scale), (int)(pos.Y * scale), delta: delta));
    }

    protected override void OnPointerEntered(PointerEventArgs e)
    {
        base.OnPointerEntered(e);
        _view?.OnMouseEnter(EventArgs.Empty);
    }

    protected override void OnPointerExited(PointerEventArgs e)
    {
        base.OnPointerExited(e);
        _view?.OnMouseLeave(EventArgs.Empty);
    }

    // ── Size change ────────────────────────────────────────────────────────

    protected override void OnPropertyChanged(AvaloniaPropertyChangedEventArgs change)
    {
        base.OnPropertyChanged(change);
        if (change.Property == BoundsProperty && _view != null)
        {
            _view.OnSizeChanged(_lastClientRect);
            // Wie in der WinForms-Variante: nur bei gültiger Größe merken, damit
            // _lastClientRect nie auf einen leeren Rect (z.B. beim ersten Layout
            // oder beim Ausblenden) gesetzt wird.
            Rectangle clr = ClientRectangle;
            if (clr.Width > 0 && clr.Height > 0) _lastClientRect = clr;
        }
    }

    // ── Keyboard (mirrors WinForms CadControl.ProcessCmdKey) ──────────────

    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);
        if (Frame == null) return;

        bool ctrl  = e.KeyModifiers.HasFlag(KeyModifiers.Control);
        bool alt   = e.KeyModifiers.HasFlag(KeyModifiers.Alt);

        // Pre-process the same set of keys as WinForms CadControl.ProcessCmdKey:
        // F-keys, Escape, Up/Down, Tab/Enter, Delete, and any Ctrl/Alt combination.
        bool preProcess = e.Key >= Key.F1 && e.Key <= Key.F24;
        preProcess |= e.Key is Key.Escape or Key.Delete;
        preProcess |= e.Key is Key.Up or Key.Down;
        preProcess |= e.Key is Key.Tab or Key.Enter;
        preProcess |= ctrl || alt;

        if (!preProcess) return;

        int vk = AvaloniaKeyToVk(e.Key);
        if (vk == 0) return;

        int mods = 0;
        if (ctrl)                                       mods |= 0x20000; // Keys.Control
        if (e.KeyModifiers.HasFlag(KeyModifiers.Shift)) mods |= 0x10000; // Keys.Shift
        if (alt)                                        mods |= 0x40000; // Keys.Alt

        var subArgs = new Substitutes.KeyEventArgs((Substitutes.Keys)(vk | mods));
        (Frame as FrameImpl)?.PreProcessKeyDown(subArgs);

        if (subArgs.Handled) e.Handled = true;
    }

    /// <summary>
    /// Maps an Avalonia <see cref="Key"/> to its Windows virtual-key code so that
    /// <see cref="CADability.Substitutes.KeyEventArgs"/> can be constructed.
    /// Covers every key used in <see cref="FrameImpl.PreProcessKeyDown"/> and in
    /// the property-entry shortcut format <c>[[modifier key]]</c>.
    /// </summary>
    private static int AvaloniaKeyToVk(Key key)
    {
        switch (key)
        {
            case Key.Back:   return 0x08;
            case Key.Tab:    return 0x09;
            case Key.Enter:  return 0x0D;
            case Key.Escape: return 0x1B;
            case Key.Space:  return 0x20;
            case Key.Left:   return 0x25;
            case Key.Up:     return 0x26;
            case Key.Right:  return 0x27;
            case Key.Down:   return 0x28;
            case Key.Delete: return 0x2E;
        }
        int ik = (int)key;
        // Key.D0=34…D9=43  →  VK_0=0x30…VK_9=0x39
        if (ik >= (int)Key.D0 && ik <= (int)Key.D9)  return ik - (int)Key.D0 + 0x30;
        // Key.A=44…Z=69    →  VK_A=0x41…VK_Z=0x5A
        if (ik >= (int)Key.A  && ik <= (int)Key.Z)   return ik - (int)Key.A  + 0x41;
        // Key.F1=90…F24=113 →  VK_F1=0x70…VK_F24=0x87
        if (ik >= (int)Key.F1 && ik <= (int)Key.F24) return ik - (int)Key.F1 + 0x70;
        return 0;
    }

    // ── Drag-drop receiving ────────────────────────────────────────────────

    protected override void OnAttachedToVisualTree(VisualTreeAttachmentEventArgs e)
    {
        base.OnAttachedToVisualTree(e);
        AddHandler(DragDrop.DropEvent,      OnDragDropReceived);
        AddHandler(DragDrop.DragOverEvent,  OnDragOverReceived);
        AddHandler(DragDrop.DragEnterEvent, OnDragEnterReceived);
        AddHandler(DragDrop.DragLeaveEvent, OnDragLeaveReceived);
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        RemoveHandler(DragDrop.DropEvent,      OnDragDropReceived);
        RemoveHandler(DragDrop.DragOverEvent,  OnDragOverReceived);
        RemoveHandler(DragDrop.DragEnterEvent, OnDragEnterReceived);
        RemoveHandler(DragDrop.DragLeaveEvent, OnDragLeaveReceived);
        base.OnDetachedFromVisualTree(e);
    }

    private void OnDragDropReceived(object? sender, global::Avalonia.Input.DragEventArgs e)
    {
        if (_view == null) return;
        var pos   = e.GetPosition(this);
        var scale = RenderScale;
        var args  = MakeDragArgs(e, (int)(pos.X * scale), (int)(pos.Y * scale));
        _view.OnDragDrop(args);
        e.DragEffects = (global::Avalonia.Input.DragDropEffects)(int)args.Effect;
    }

    private void OnDragOverReceived(object? sender, global::Avalonia.Input.DragEventArgs e)
    {
        if (_view == null) return;
        var pos   = e.GetPosition(this);
        var scale = RenderScale;
        var args  = MakeDragArgs(e, (int)(pos.X * scale), (int)(pos.Y * scale));
        _view.OnDragOver(args);
        e.DragEffects = (global::Avalonia.Input.DragDropEffects)(int)args.Effect;
    }

    private void OnDragEnterReceived(object? sender, global::Avalonia.Input.DragEventArgs e)
    {
        if (_view == null) return;
        var pos   = e.GetPosition(this);
        var scale = RenderScale;
        _view.OnDragEnter(MakeDragArgs(e, (int)(pos.X * scale), (int)(pos.Y * scale)));
    }

    private void OnDragLeaveReceived(object? sender, global::Avalonia.Input.DragEventArgs e)
    {
        _view?.OnDragLeave(EventArgs.Empty);
    }

    // ── OpenGL lifecycle ───────────────────────────────────────────────────

    protected override void OnOpenGlInit(GlInterface gl)
    {
        base.OnOpenGlInit(gl);
        _painter = new PaintToOpenGL();
        // Pass physical pixels — that is the size of the GL framebuffer Avalonia allocates.
        _painter.Init(gl, PhysWidth, PhysHeight);
        PaintTo3D = _painter;
    }

    protected override void OnOpenGlRender(GlInterface gl, int fb)
    {
        if (_painter == null || _view == null) return;
        // Resize to physical pixels every frame so that DPI or window-resize changes
        // are picked up immediately (glViewport and projection must match the framebuffer).
        int w = PhysWidth;
        int h = PhysHeight;
        ((IPaintTo3D)_painter).Resize(w, h);
        _painter.SetDefaultFramebuffer((uint)fb);
        _view.OnPaint(new Substitutes.PaintEventArgs
        {
            ClipRectangle = new Rectangle(0, 0, w, h)
        });
        OnPaintDone?.Invoke(this);
    }

    protected override void OnOpenGlDeinit(GlInterface gl)
    {
        (PaintTo3D as IPaintTo3D)?.Dispose();
        _painter  = null;
        PaintTo3D = null;
        base.OnOpenGlDeinit(gl);
    }

    // ── Helpers ────────────────────────────────────────────────────────────

    private static Substitutes.MouseEventArgs MakeMouseArgs(
        Substitutes.MouseButtons button, int x, int y, int clicks = 0, int delta = 0) =>
        new Substitutes.MouseEventArgs
        {
            Button   = button,
            Clicks   = clicks,
            X        = x,
            Y        = y,
            Delta    = delta,
            Location = new Point(x, y)
        };

    private static Substitutes.MouseButtons MapPressedButton(PointerUpdateKind kind) =>
        kind switch
        {
            PointerUpdateKind.LeftButtonPressed   => Substitutes.MouseButtons.Left,
            PointerUpdateKind.RightButtonPressed  => Substitutes.MouseButtons.Right,
            PointerUpdateKind.MiddleButtonPressed => Substitutes.MouseButtons.Middle,
            _                                     => Substitutes.MouseButtons.None
        };

    private static Substitutes.MouseButtons MapCurrentButtons(PointerPointProperties props)
    {
        var b = Substitutes.MouseButtons.None;
        if (props.IsLeftButtonPressed)   b |= Substitutes.MouseButtons.Left;
        if (props.IsRightButtonPressed)  b |= Substitutes.MouseButtons.Right;
        if (props.IsMiddleButtonPressed) b |= Substitutes.MouseButtons.Middle;
        return b;
    }

    private static Substitutes.DragEventArgs MakeDragArgs(
        global::Avalonia.Input.DragEventArgs e, int x, int y)
    {
        // DragDropEffects values match between Avalonia and CADability.Substitutes (Copy=1, Move=2, Link=4)
        var allowed = (DragDropEffects)(int)e.DragEffects;
        var args = new Substitutes.DragEventArgs
        {
            Data          = e.Data,
            KeyState      = MapKeyState(e.KeyModifiers),
            X             = x,
            Y             = y,
            AllowedEffect = allowed,
            Effect        = allowed
        };
        return args;
    }

    // WinForms-style drag key-state bits the CADability core inspects (e.g. Ctrl → copy
    // instead of move): MK_SHIFT=4, MK_CONTROL=8, MK_ALT=32. Mouse-button bits aren't used here.
    private static int MapKeyState(KeyModifiers mods)
    {
        int state = 0;
        if ((mods & KeyModifiers.Shift)   != 0) state |= 4;
        if ((mods & KeyModifiers.Control) != 0) state |= 8;
        if ((mods & KeyModifiers.Alt)     != 0) state |= 32;
        return state;
    }
}
