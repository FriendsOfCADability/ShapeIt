using System;
using System.Collections.Generic;
using Avalonia;
using Avalonia.Input;
using Avalonia.Threading;
using CADability;
using CADability.UserInterface;
using Point = CADability.Substitutes.Point;
using Rectangle = CADability.Substitutes.Rectangle;
using Substitutes = CADability.Substitutes;

namespace ShapeIt.Browser
{
    /// <summary>
    /// Touch support for the browser viewport — kept as a separate partial-class file so the
    /// touch/tablet code can be developed independently of the core viewport (rendering,
    /// ICanvas plumbing, mouse handling in Gl3DViewport.cs). Contains the touch gesture state
    /// machine (tap/orbit/pan/pinch/long-press), the synthesized mouse-event emitters, the
    /// modelling-pick delegates and the touch helper-bar API. The only integration points in
    /// the core file are the PointerType.Touch dispatch lines in OnPointerPressed/Moved/
    /// Released and the stationary-click re-pick in OnPointerReleased.
    /// </summary>
    public partial class Gl3DViewport
    {
        // Modifier keys synthesized for touch gestures (e.g. Control during a touch-rotate so
        // CADability interprets the synthesized middle-drag as an orbit). OR-ed into the real
        // keys by LastKeyModifiers (IModifierKeyProvider, core file).
        private global::Avalonia.Input.KeyModifiers _injectedModifiers = global::Avalonia.Input.KeyModifiers.None;

        // ── Touch gestures (tablet / iPad) ─────────────────────────────────
        // Touch is translated into the SAME mouse events the desktop uses, so behaviour
        // matches a mouse exactly — only the trigger is a finger:
        //   • 1 finger drag      → orbit/rotate   (= Ctrl + middle-drag)
        //   • 2 finger drag      → pan            (= middle-drag)
        //   • pinch              → zoom           (= mouse wheel, centred on the pinch)
        //   • tap                → pick/select    (= left click)
        //   • long press (0.5 s) → context menu   (= right click)
        // Mouse and pen pointers keep the standard handling above (a stylus stays precise).
        private enum TouchState { None, OneFingerPending, Rotating, TwoFinger, ActionDragging, Suppressed }
        private TouchState _touchState = TouchState.None;
        private global::Avalonia.Point _actionDragLast;

        /// <summary>Set by MainView: true when the modelling page currently has a selection (so a
        /// successful modelling pick isn't clobbered by the tap fallback). The action's own
        /// SelectedObjects list is always empty in modelling mode, so we can't key off it.</summary>
        public Func<bool>? ModellingHasSelection;

        // A non-Select action (e.g. extrude) is running and wants the pointer input itself.
        private bool IsActionRunning() => Frame?.ActiveAction is { } a && a is not CADability.Actions.SelectObjectsAction;
        private readonly Dictionary<int, global::Avalonia.Point> _touch = new();
        private global::Avalonia.Point _touchStart;
        private int _firstTouchId = -1;
        private DispatcherTimer? _longPress;
        private double _pinchDist;
        private global::Avalonia.Point _panLastMid;
        private bool _twoFingerActive;
        private global::Avalonia.Point _rotatePrev, _rotateVirtual;
        private const double TouchDragThreshold = 14;  // DIPs before a press becomes a drag (tap tolerance)
        private const double RotateSensitivity = 0.4;  // touch orbit damping (lower = gentler; 1.0 == raw mouse feel)
        public const int TouchPickRadius = 15;         // finger-sized pick aperture for taps (mouse keeps the default 5)

        /// <summary>Set by MainView: edge-priority modelling pick (ModellingPropertyEntries.TouchPick),
        /// called as (viewPosition, pickRadius, fromTouch). Used by touch taps (finger radius) AND by
        /// stationary mouse left clicks (precise radius) — the standard pick's depth filter makes edges
        /// nearly unselectable for both. fromTouch lets the handler apply touch-only modifiers (the
        /// Multi toggle must not apply to the mouse re-pick: the click's own pick already ran, a second
        /// add/remove pass would toggle the selection back out). Returns true when handled.</summary>
        public Func<Point, int, bool, bool>? ModellingPick;

        /// <summary>Raised the first time a touch contact is seen, so the UI can reveal the touch helper bar.</summary>
        public event Action? TouchDetected;
        private bool _touchAnnounced;

        /// <summary>Diagnostic: one short line per touch lifecycle step (shown in the "?" overlay).</summary>
        public event Action<string>? TouchTrace;
        private void Trace(string s) { Console.WriteLine("[touch] " + s); TouchTrace?.Invoke(s); }

        private void OnTouchPressed(PointerPressedEventArgs e)
        {
            if (!_touchAnnounced) { _touchAnnounced = true; TouchDetected?.Invoke(); }
            e.Pointer.Capture(this);
            e.Handled = true;
            var p = LocalPos(e);   // same logical units as the mouse path / the projection
            _touch[e.Pointer.Id] = p;
            Trace($"DOWN {e.Pointer.Type} n={_touch.Count} sc={RenderScale:0.##}");

            if (_touch.Count == 1)
            {
                _firstTouchId = e.Pointer.Id;
                _touchStart = p;
                _touchState = TouchState.OneFingerPending;
                StartLongPress();
            }
            else if (_touch.Count == 2)
            {
                CancelLongPress();
                if (_touchState == TouchState.Rotating)
                {
                    // finish the in-progress rotate before switching to two-finger navigation
                    EmitUp(Substitutes.MouseButtons.Middle, _touch.TryGetValue(_firstTouchId, out var fp) ? fp : _touchStart);
                    _injectedModifiers = global::Avalonia.Input.KeyModifiers.None;
                }
                BeginTwoFinger();
            }
        }

        private void OnTouchMoved(PointerEventArgs e)
        {
            if (!_touch.ContainsKey(e.Pointer.Id)) return;
            var p = LocalPos(e);
            _touch[e.Pointer.Id] = p;
            e.Handled = true;

            switch (_touchState)
            {
                case TouchState.OneFingerPending:
                    if (Dist(p, _touchStart) > TouchDragThreshold)
                    {
                        CancelLongPress();
                        if (IsActionRunning())
                        {
                            // A modelling action (e.g. extrude) is running and wants the input:
                            // drive its live preview with a button-LESS move instead of orbiting.
                            _touchState = TouchState.ActionDragging;
                            _actionDragLast = p;
                            Trace("DRAG>action");
                            EmitMove(Substitutes.MouseButtons.None, p);
                        }
                        else
                        {
                            // rotate = Ctrl + middle-drag, anchored at the press point
                            _injectedModifiers = global::Avalonia.Input.KeyModifiers.Control;
                            _rotatePrev = _touchStart;
                            _rotateVirtual = _touchStart;
                            EmitDown(Substitutes.MouseButtons.Middle, _touchStart);
                            _touchState = TouchState.Rotating;
                            Trace($"DRAG>rotate d={Dist(p, _touchStart):0}");
                            ProcessRotate(p);
                        }
                    }
                    break;
                case TouchState.Rotating:
                    ProcessRotate(p);
                    break;
                case TouchState.ActionDragging:
                    _actionDragLast = p;
                    EmitMove(Substitutes.MouseButtons.None, p);   // live action preview (e.g. extrude height)
                    break;
                case TouchState.TwoFinger:
                    UpdateTwoFinger();
                    break;
            }
        }

        private void OnTouchReleased(PointerReleasedEventArgs e)
        {
            bool hadId = _touch.Remove(e.Pointer.Id, out var lastPos);
            e.Handled = true;

            if (_touchState == TouchState.TwoFinger)
            {
                if (_twoFingerActive) { EmitUp(Substitutes.MouseButtons.Middle, _panLastMid); _twoFingerActive = false; }
                // a finger may still be down after a pinch/pan: ignore it until fully released
                _touchState = _touch.Count == 0 ? TouchState.None : TouchState.Suppressed;
            }
            else if (_touchState == TouchState.Rotating)
            {
                EmitUp(Substitutes.MouseButtons.Middle, hadId ? lastPos : _touchStart);
                _injectedModifiers = global::Avalonia.Input.KeyModifiers.None;
            }
            else if (_touchState == TouchState.ActionDragging)
            {
                // confirm the running action's step (desktop equivalent: left click at the point)
                EmitDown(Substitutes.MouseButtons.Left, _actionDragLast, clicks: 1);
                EmitUp(Substitutes.MouseButtons.Left, _actionDragLast);
                Trace("UP>action confirm");
            }
            else if (_touchState == TouchState.OneFingerPending)
            {
                EmitTap();   // quick tap → left click (pick). Shared with the pointercancel path.
            }
            else Trace($"UP state={_touchState}");

            if (_touch.Count == 0)
            {
                _touchState = TouchState.None;
                _injectedModifiers = global::Avalonia.Input.KeyModifiers.None;
                _firstTouchId = -1;
                CancelLongPress();
            }
        }

        protected override void OnPointerCaptureLost(PointerCaptureLostEventArgs e)
        {
            base.OnPointerCaptureLost(e);
            _touch.Remove(e.Pointer.Id);
            // iOS Safari/WebKit ends a quick TAP with pointercancel, which Avalonia surfaces as
            // capture-lost (NOT PointerReleased). So the tap pick must ALSO fire here, otherwise
            // taps never select on iPad. EmitTap is idempotent (state guard) — a tap that ends
            // via pointerup OR pointercancel picks exactly once.
            EmitTap();
            // A lost capture during an active synthesized drag must ALWAYS end that drag and
            // clear the injected Ctrl — even if this id was never tracked or other fingers
            // remain — so no middle-button / Control state leaks into later input.
            if (_touchState == TouchState.Rotating || _twoFingerActive)
            {
                EmitUp(Substitutes.MouseButtons.Middle, _touchStart);
                _twoFingerActive = false;
                Trace("CAPTURELOST end-drag");
            }
            _injectedModifiers = global::Avalonia.Input.KeyModifiers.None;
            if (_touch.Count == 0)
            {
                CancelLongPress();
                _touchState = TouchState.None;
                _firstTouchId = -1;
            }
            else
            {
                _touchState = TouchState.Suppressed;   // ignore remaining fingers until all lift
            }
        }

        private void BeginTwoFinger()
        {
            _injectedModifiers = global::Avalonia.Input.KeyModifiers.None;   // pan = middle-drag, no Ctrl
            _touchState = TouchState.TwoFinger;
            var (mid, dist) = MidDist();
            _panLastMid = mid;
            _pinchDist = dist > 0 ? dist : 1;
            EmitDown(Substitutes.MouseButtons.Middle, mid);   // establishes the pan reference point
            _twoFingerActive = true;
        }

        private void UpdateTwoFinger()
        {
            if (!_twoFingerActive || _touch.Count < 2) return;   // no pan/zoom without an active middle-down
            var (mid, dist) = MidDist();
            // pan: middle-drag to the new midpoint
            EmitMove(Substitutes.MouseButtons.Middle, mid);
            _panLastMid = mid;
            // zoom: one wheel step per ±10% change in finger spread, centred on the midpoint.
            // OnMouseWheel zooms around PointToClient(CurrentMousePosition == LastScreenPosition);
            // PointToClient subtracts PointToScreen(0,0), so composing the screen position as
            // PointToScreen(0,0) + mid makes that round trip yield exactly `mid` in the logical
            // units the projection uses — independent of how PointToScreen scales.
            if (_pinchDist > 0 && dist > 0)
            {
                var tl = this.PointToScreen(new global::Avalonia.Point(0, 0));
                _lastScreenPos = new global::Avalonia.PixelPoint(tl.X + (int)mid.X, tl.Y + (int)mid.Y);
                const double step = 1.1;     // == mouseWheelZoomFactor
                double ratio = dist / _pinchDist;
                int guard = 0;
                while (ratio >= step && guard++ < 20) { EmitWheel(mid, +120); _pinchDist *= step; ratio = dist / _pinchDist; }
                while (ratio <= 1.0 / step && guard++ < 20) { EmitWheel(mid, -120); _pinchDist /= step; ratio = dist / _pinchDist; }
            }
        }

        // Damped touch orbit. CADability rotates by ~(pixelDelta/5)° per middle-drag step;
        // feeding it the raw finger delta felt "viel zu stark". Instead we advance a VIRTUAL
        // drag position by only RotateSensitivity × the finger delta.
        private void ProcessRotate(global::Avalonia.Point p)
        {
            double k = RotateSensitivity;
            _rotateVirtual = new global::Avalonia.Point(
                _rotateVirtual.X + (p.X - _rotatePrev.X) * k,
                _rotateVirtual.Y + (p.Y - _rotatePrev.Y) * k);
            _rotatePrev = p;
            EmitMove(Substitutes.MouseButtons.Middle, _rotateVirtual);
        }

        private (global::Avalonia.Point mid, double dist) MidDist()
        {
            global::Avalonia.Point a = default, b = default; int i = 0;
            foreach (var kv in _touch) { if (i == 0) a = kv.Value; else { b = kv.Value; break; } i++; }
            var mid = new global::Avalonia.Point((a.X + b.X) / 2, (a.Y + b.Y) / 2);
            double dist = Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
            return (mid, dist);
        }

        private static double Dist(global::Avalonia.Point a, global::Avalonia.Point b)
            => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

        private void StartLongPress()
        {
            CancelLongPress();
            _longPress = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(500) };
            _longPress.Tick += (_, _) =>
            {
                CancelLongPress();
                if (_touchState != TouchState.OneFingerPending) return;
                // long press → right click → CADability opens its context menu on right-up
                EmitDown(Substitutes.MouseButtons.Right, _touchStart, clicks: 1);
                EmitUp(Substitutes.MouseButtons.Right, _touchStart);
                _touchState = TouchState.Suppressed;
            };
            _longPress.Start();
        }

        private void CancelLongPress() { var t = _longPress; _longPress = null; t?.Stop(); }

        // The tap (left-click) pick, shared by OnTouchReleased and OnPointerCaptureLost so a tap
        // selects whether iOS ends it with pointerup or (frequently) pointercancel. The state
        // guard makes it idempotent — only the first terminal event for a pending tap picks.
        private void EmitTap()
        {
            if (_touchState != TouchState.OneFingerPending) return;
            CancelLongPress();
            // Preferred path: the edge-priority modelling pick (ModellingPropertyEntries.TouchPick).
            // The synthesized click's pick depth-filters edges away under a finger-sized aperture
            // (face always wins) — TouchPick fixes the priority. Not while an action runs: there
            // the tap IS the action's input (point confirm) and must stay a click.
            bool modellingHandled = false;
            if (!IsActionRunning())
            {
                try { modellingHandled = ModellingPick?.Invoke(new Point((int)_touchStart.X, (int)_touchStart.Y), TouchPickRadius, true) ?? false; }
                catch { modellingHandled = false; }
            }
            if (modellingHandled)
            {
                Invalidate();
                Trace("TAP>modelling");
            }
            else
            {
                // Fallback: synthesized left click (running action, hotspot under the finger, or
                // modelling page not active). A fingertip can't aim at thin objects with the 5 px
                // mouse aperture, so widen the pick radius ONLY for the duration of this click and
                // restore it right after — the mouse keeps its precise radius.
                int oldRadius = 5;
                try { oldRadius = Frame?.GetIntSetting("Select.PickRadius", 5) ?? 5; } catch { }
                try { Settings.GlobalSettings.SetValue("Select.PickRadius", TouchPickRadius); } catch { }
                try
                {
                    EmitDown(Substitutes.MouseButtons.Left, _touchStart, clicks: 1);
                    EmitUp(Substitutes.MouseButtons.Left, _touchStart);
                }
                finally
                {
                    try { Settings.GlobalSettings.SetValue("Select.PickRadius", oldRadius); } catch { }
                }
                // Last resort for the NORMAL select mode only (modelling taps are fully handled
                // above): if the precise pick selected nothing, select the nearest whole object
                // within a finger radius (e.g. a thin wireframe sketch). Never during a running
                // action, and never when the modelling page holds a selection.
                try
                {
                    bool normalEmpty = (Frame?.SelectedObjects?.Count ?? 0) == 0;
                    bool modellingHit = ModellingHasSelection?.Invoke() ?? false;
                    if (normalEmpty && !modellingHit && !IsActionRunning() && _view != null)
                    {
                        int ex = (int)_touchStart.X, ey = (int)_touchStart.Y;
                        var anyLayer = new Wintellect.PowerCollections.Set<CADability.Attribute.Layer>(System.Array.Empty<CADability.Attribute.Layer>());
                        foreach (int rad in new[] { 25, 50 })
                        {
                            var pa = _view.Projection.GetPickSpace(new Rectangle(ex - rad, ey - rad, rad * 2, rad * 2));
                            var near = _view.Model.GetObjectsFromRect(pa, anyLayer, PickMode.single, Frame?.Project?.FilterList);
                            if (near != null && near.Count > 0) { Frame!.SelectedObjects = near; break; }
                        }
                    }
                }
                catch { /* best-effort fallback */ }
            }
            _touchState = TouchState.Suppressed;
            TracePick("TAP", _touchStart);
        }

        // Diagnostic shared by tap and mouse-click: reports the effective pick radius, whether the
        // real selection hit (sel), the projection size, and the nearest-geometry distance via an
        // expanding empty-layer probe (nearPx). Used to compare coordinate behaviour across scales.
        private void TracePick(string tag, global::Avalonia.Point dip)
        {
            int ex = (int)dip.X, ey = (int)dip.Y;
            int r = 5, sel = -1, pjw = -1, pjh = -1, nearPx = -1;
            try { r = Frame?.GetIntSetting("Select.PickRadius", 5) ?? 5; } catch { }
            try { sel = Frame?.SelectedObjects?.Count ?? -1; } catch { }
            try
            {
                pjw = _view!.Projection.Width; pjh = _view.Projection.Height;
                var emptyLayers = new Wintellect.PowerCollections.Set<CADability.Attribute.Layer>(System.Array.Empty<CADability.Attribute.Layer>());
                foreach (int rad in new[] { 20, 40, 80, 160, 320, 640 })
                {
                    var pa = _view.Projection.GetPickSpace(new Rectangle(ex - rad, ey - rad, rad * 2, rad * 2));
                    var near = _view.Model.GetObjectsFromRect(pa, emptyLayers, PickMode.single, Frame?.Project?.FilterList);
                    if ((near?.Count ?? 0) > 0) { nearPx = rad; break; }
                }
            }
            catch { nearPx = -2; }
            Trace($"{tag} e({ex},{ey}) ph({PhysWidth}x{PhysHeight}) pj({pjw}x{pjh}) r={r} sc={RenderScale:0.#} sel={sel} nearPx={nearPx}");
        }

        // Synthesized mouse events to the view. Positions are LocalPos-based logical units —
        // the SAME units the mouse handlers feed (the projection/framebuffer use Bounds, see
        // RenderFrame), so touch and mouse picking behave identically at any device scale.
        // Invalidate() schedules the read-back/redraw.
        private void EmitDown(Substitutes.MouseButtons b, global::Avalonia.Point p, int clicks = 0)
        { _view!.OnMouseDown(MakeArgs(b, (int)p.X, (int)p.Y, clicks: clicks)); Invalidate(); }
        private void EmitMove(Substitutes.MouseButtons b, global::Avalonia.Point p)
        { _view!.OnMouseMove(MakeArgs(b, (int)p.X, (int)p.Y)); Invalidate(); }
        private void EmitUp(Substitutes.MouseButtons b, global::Avalonia.Point p)
        { _view!.OnMouseUp(MakeArgs(b, (int)p.X, (int)p.Y)); Invalidate(); }
        private void EmitWheel(global::Avalonia.Point p, int delta)
        { _view!.OnMouseWheel(MakeArgs(Substitutes.MouseButtons.None, (int)p.X, (int)p.Y, delta: delta)); Invalidate(); }

        // ── touch helper bar API (zoom buttons + fit) ──────────────────────
        public void TouchZoom(bool zoomIn) { _view?.ZoomDelta(zoomIn ? 1.0 / 1.25 : 1.25); Invalidate(); }
        public void TouchZoomAll() { try { _view?.ZoomTotal(1.2); } catch { /* empty extent */ } Invalidate(); }
    }
}
