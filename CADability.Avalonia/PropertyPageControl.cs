using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Threading;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text.RegularExpressions;

namespace CADability.Avalonia;

/// <summary>
/// Custom-drawn Avalonia control that mirrors WinForms PropertyPage.
/// PaintItem → Render(), OnMouseClick/Move/Down → OnPointer*.
/// The control sets its own Height so a ScrollViewer can wrap it.
/// </summary>
public class PropertyPageControl : Control
{
    // ── References ─────────────────────────────────────────────────────────
    private PropertyPage? _page;
    private IReadOnlyList<IPropertyEntry> _entries = Array.Empty<IPropertyEntry>();

    // Callbacks to PropertiesExplorer for the floating TextBox
    public Action<Rect, string, IPropertyEntry>?            RequestShowTextBox;
    public Action?                                          RequestHideTextBox;

    // Callbacks to PropertiesExplorer for the floating DropDown ListBox
    public Action<Rect, string[], int, IPropertyEntry>?     RequestShowDropDown;
    public Action?                                          RequestHideDropDown;

    // ── Metrics (recomputed when width changes) ────────────────────────────
    private const double BaseFontSize = 12.0;
    private double _lineHeight;   // BaseFontSize * 1.6
    private double _square;       // ~0.6 * lineHeight, even integer
    private double _buttonWidth;  // ~0.8 * lineHeight

    // ── Middle (label / value split, x in local coords) ───────────────────
    private double _middle;
    private double _lastWidth;    // used to detect resize → reset middle
    private bool   _draggingMiddle;

    // ── Label extension (full label shown over truncated ones on hover) ────
    // Mirrors WinForms PropertyPage.labelNeedsExtension + ShowLabelExtension:
    // when a label doesn't fit and the cursor lingers over it, the full label
    // is drawn at the same location, extended over the value column.
    private bool[] _labelTruncated = Array.Empty<bool>();
    private int    _labelExtensionIdx = -1;   // entry whose extension is shown
    private int    _hoverLabelIdx     = -1;   // entry the cursor currently lingers on
    private DispatcherTimer? _hoverTimer;

    // ── Tooltip (full help text shown when hovering a label) ───────────────
    // Mirrors WinForms PropertyPage: a tooltip built from the entry's ResourceId
    // (StringTable tip → label) appears after a short delay over the label and
    // auto-hides after a few seconds. ToolTip.ServiceEnabled is turned off so the
    // built-in hover behaviour doesn't interfere with this manual, per-label control.
    private string?          _currentToolTip;   // text currently associated (mirrors Forms currentToolTip)
    private string?          _pendingToolTip;   // text the delay timer will show
    private DispatcherTimer? _toolTipDelay;     // show delay (Forms InitialDelay = 500 ms)
    private DispatcherTimer? _toolTipOff;       // auto-hide delay (Forms ToolTipOff = 3500 ms)

    // ── Scrollbar reserve ─────────────────────────────────────────────────
    // In Avalonia's Fluent theme, the vertical scrollbar is an overlay that
    // expands on hover (typically up to ~17 px).  Setting this reserve shrinks
    // the logical drawing width so right-edge buttons stay accessible.
    private double _scrollBarReserve;

    /// <summary>
    /// Width in pixels reserved on the right edge for the vertical scroll bar.
    /// When set, all rendering and hit-testing use <c>Bounds.Width - ScrollBarReserve</c>
    /// as the effective width, keeping ContextMenu, DirectMenu and Shortcut badges
    /// accessible even when the Fluent overlay scrollbar is fully expanded.
    /// </summary>
    public double ScrollBarReserve
    {
        get => _scrollBarReserve;
        set
        {
            if (Math.Abs(_scrollBarReserve - value) < 0.5) return;
            _scrollBarReserve = value;
            _lastWidth = 0;   // force _middle reset on next Render
            InvalidateVisual();
        }
    }

    /// <summary>Logical drawing width, excluding the scrollbar reserve.</summary>
    private double EffW => Bounds.Width - _scrollBarReserve;

    // ── Static brushes / pens ─────────────────────────────────────────────
    private static readonly IBrush BrushWindow   = Brushes.White;
    private static readonly IBrush BrushIndent   = new SolidColorBrush(Color.FromRgb(0xF0, 0xF0, 0xF0));
    private static readonly IBrush BrushSelected = new SolidColorBrush(Color.FromRgb(0x00, 0x78, 0xD4));
    private static readonly IBrush BrushHiText   = Brushes.White;
    private static readonly IBrush BrushText     = new SolidColorBrush(Color.FromRgb(0x20, 0x20, 0x20));
    private static readonly IBrush BrushHighlight = Brushes.Red;
    private static readonly IBrush BrushGroup    = new SolidColorBrush(Color.FromRgb(0xF0, 0xF0, 0xF0));

    private static readonly IPen PenGrid    = new Pen(new SolidColorBrush(Color.FromRgb(0xE4, 0xE4, 0xE4)), 0.5);
    private static readonly IPen PenMiddle  = new Pen(new SolidColorBrush(Color.FromRgb(0xC0, 0xC0, 0xC0)), 1);
    private static readonly IPen PenText    = new Pen(new SolidColorBrush(Color.FromRgb(0x40, 0x40, 0x40)), 1);
    private static readonly IPen PenHiText  = new Pen(Brushes.White, 1);

    private static readonly Typeface TfNormal =
        new Typeface("Segoe UI, Arial, sans-serif");
    private static readonly Typeface TfBold =
        new Typeface("Segoe UI, Arial, sans-serif", FontStyle.Normal, FontWeight.Bold);

    // ── Shortcut parsing (mirrors WinForms PropertyPage) ──────────────────
    private static readonly Dictionary<char, string> _keyboardModifier = new()
    {
        ['c'] = "Ctrl",
        ['a'] = "Alt",
        ['s'] = "Shift",
    };
    private static readonly Regex _shortcutRegex = new Regex(
        @"^(.*?)(?:\s*\[\[(?:(?<modifier1>[sca])?(?<modifier2>[sca])?(?<key>Esc|Del|F\d{1,2}|[A-Z]))\]\])\s*$",
        RegexOptions.Compiled);

    // ── Setup ──────────────────────────────────────────────────────────────

    public PropertyPageControl()
    {
        Focusable = true;
        // DoubleTapped is not overridable in all Avalonia 11.x builds;
        // subscribe via the event instead.
        DoubleTapped += OnDoubleTappedHandler;

        // Disable the automatic hover tooltip; we drive Tip/IsOpen manually so the
        // text can depend on which label the cursor is over (mirrors Forms PropertyPage).
        ToolTip.SetServiceEnabled(this, false);
        ToolTip.SetPlacement(this, PlacementMode.Pointer);
    }

    public void Attach(PropertyPage page)
    {
        if (_page != null)
        {
            _page.Changed -= OnPageChanged;
            _page.ShowTextBoxRequested -= OnShowTextBoxRequested;
        }
        _page = page;
        _page.Changed += OnPageChanged;
        _page.ShowTextBoxRequested += OnShowTextBoxRequested;
        OnPageChanged();
    }

    private void OnShowTextBoxRequested(IPropertyEntry entry, bool isValue)
    {
        int idx = -1;
        for (int i = 0; i < _entries.Count; i++)
            if (_entries[i] == entry) { idx = i; break; }
        if (idx < 0) return;
        Rect   rect = isValue ? ValueRect(idx) : LabelRect(idx);
        string text = isValue ? (entry.Value ?? "") : (entry.Label ?? "");
        RequestShowTextBox?.Invoke(rect, text, entry);
    }

    /// <summary>First shown entry with the given ResourceId, or null — used to re-bind a
    /// running edit after a page rebuild replaced all entry instances.</summary>
    public IPropertyEntry? FindShownEntry(string resourceId)
    {
        foreach (var e in _entries)
            if (e.ResourceId == resourceId) return e;
        return null;
    }

    /// <summary>Value-cell rectangle of a shown entry (for repositioning the floating editor).</summary>
    public bool TryGetValueRect(IPropertyEntry entry, out Rect rect)
    {
        for (int i = 0; i < _entries.Count; i++)
            if (_entries[i] == entry) { rect = ValueRect(i); return true; }
        rect = default;
        return false;
    }

    // True while a value/label is being edited via the floating TextBox. During an edit the
    // per-keystroke live-preview fires PropertyPage.Changed; rewriting Height here forces an
    // Avalonia layout pass that steals keyboard focus from the sibling overlay TextBox, which
    // then commits the edit after a single keystroke (you couldn't type "44"). Set by the explorer.
    private bool _editing;
    public void SetEditing(bool v) => _editing = v;

    private void OnPageChanged()
    {
        if (!Dispatcher.UIThread.CheckAccess())
        {
            Dispatcher.UIThread.Post(OnPageChanged, DispatcherPriority.Normal);
            return;
        }
        int oldCount = _entries.Count;
        _entries = _page?.FlattenVisible() ?? Array.Empty<IPropertyEntry>();
        _labelExtensionIdx = -1;
        _hoverLabelIdx = -1;
        _hoverTimer?.Stop();
        _currentToolTip = null;
        HideToolTip();
        RecomputeMetrics();
        // While editing, only rewrite Height when the entry set actually changed (a Height write
        // triggers the focus-stealing relayout). Mid-keystroke the entry set is unchanged → repaint only.
        if (_editing && _entries.Count == oldCount)
            InvalidateVisual();
        else
        {
            Height = _entries.Count * _lineHeight;
            InvalidateVisual();
        }
    }

    private void RecomputeMetrics()
    {
        _lineHeight  = Math.Round(BaseFontSize * 1.6);
        _square      = Math.Floor(0.6 * _lineHeight / 2) * 2;   // even
        _buttonWidth = Math.Round(0.8 * _lineHeight);
    }

    public double LineHeight => _lineHeight > 0 ? _lineHeight : Math.Round(BaseFontSize * 1.6);

    // ── Render (mirrors WinForms PaintItem) ────────────────────────────────

    public override void Render(DrawingContext ctx)
    {
        if (Bounds.Width <= 0 || _entries.Count == 0) return;
        double w = EffW;   // effective width — excludes scrollbar reserve

        // Reset middle on width change (mirrors WinForms OnSizeChanged)
        if (Math.Abs(w - _lastWidth) > 0.5)
        {
            _middle    = w * 0.5;
            _lastWidth = w;
        }

        RecomputeMetrics();

        // White background for the whole bounds (including the scrollbar reserve strip)
        ctx.FillRectangle(BrushWindow,
            new Rect(0, 0, Bounds.Width, _entries.Count * _lineHeight));

        // Per-render truncation flags, filled by DrawEntry.
        if (_labelTruncated.Length != _entries.Count)
            _labelTruncated = new bool[_entries.Count];
        Array.Clear(_labelTruncated, 0, _labelTruncated.Length);

        var selected = _page?.GetCurrentSelection();
        for (int i = 0; i < _entries.Count; i++)
            DrawEntry(ctx, i, selected, w);

        // Label extension overlay drawn last so it sits on top of the value column.
        if (_labelExtensionIdx >= 0 && _labelExtensionIdx < _entries.Count)
            DrawLabelExtension(ctx, _labelExtensionIdx, w);
    }

    private void DrawEntry(DrawingContext ctx, int idx,
                           IPropertyEntry? selected, double w)
    {
        var e    = _entries[idx];
        double y = idx * _lineHeight;
        var area = new Rect(0, y, w, _lineHeight);

        var   flags      = e.Flags;
        bool  isSel      = e == selected;
        bool  isSep      = flags.HasFlag(PropertyEntryType.Seperator);
        bool  isGroup    = flags.HasFlag(PropertyEntryType.GroupTitle);
        bool  hasChild   = flags.HasFlag(PropertyEntryType.HasSubEntries);
        bool  isBold     = flags.HasFlag(PropertyEntryType.Bold) || isGroup;
        bool  isCheckable = flags.HasFlag(PropertyEntryType.Checkable);
        bool  showValue  = e.Value != null && !isCheckable;
        bool  hasCtx        = flags.HasFlag(PropertyEntryType.ContextMenu);
        bool  hasDD         = flags.HasFlag(PropertyEntryType.DropDown);
        bool  hasDirectMenu = flags.HasFlag(PropertyEntryType.DirectMenu);
        bool  hasOKBtn      = flags.HasFlag(PropertyEntryType.OKButton);
        bool  hasCancelBtn  = flags.HasFlag(PropertyEntryType.CancelButton);
        // Total pixel width reserved by OK (✔) and Cancel (✖) buttons on the right edge.
        // Each button is _lineHeight wide — same as WinForms (area.Height).
        double okCancelWidth = (hasOKBtn ? _lineHeight : 0) + (hasCancelBtn ? _lineHeight : 0);

        // Row background + subtle bottom grid line
        ctx.FillRectangle(BrushWindow, area);
        ctx.DrawLine(PenGrid,
            new Point(0,  y + _lineHeight - 0.5),
            new Point(w,  y + _lineHeight - 0.5));

        // Shaded indent column
        double indentPx = e.IndentLevel * _buttonWidth;
        if (indentPx > 0)
            ctx.FillRectangle(BrushIndent, new Rect(0, y, indentPx, _lineHeight));

        // ── Separator ──────────────────────────────────────────────────────
        if (isSep)
        {
            double ym  = y + _lineHeight / 2;
            double lft = indentPx + _buttonWidth;
            double rgt = w - _buttonWidth;
            string sepText = e.Label ?? "";

            // Draw the line interrupted under the centered text, so it doesn't
            // look struck through (Forms.NET8 uses a near-invisible light grey;
            // here the line is darker, so the gap is needed).
            if (!string.IsNullOrEmpty(sepText))
            {
                var ft = new FormattedText(
                    sepText, CultureInfo.CurrentCulture, FlowDirection.LeftToRight,
                    TfNormal, BaseFontSize, BrushText);
                double cx   = (lft + rgt) / 2;
                double gapL = Math.Max(lft, cx - ft.Width / 2 - 4);
                double gapR = Math.Min(rgt, cx + ft.Width / 2 + 4);
                if (gapL > lft) ctx.DrawLine(PenMiddle, new Point(lft, ym),  new Point(gapL, ym));
                if (gapR < rgt) ctx.DrawLine(PenMiddle, new Point(gapR, ym), new Point(rgt, ym));
            }
            else
            {
                ctx.DrawLine(PenMiddle, new Point(lft, ym), new Point(rgt, ym));
            }

            PutText(ctx, sepText, false, false,
                new Rect(lft, y, rgt - lft, _lineHeight), center: true);
            return;
        }

        // Label rect — stops at _middle when there is a value; otherwise runs to the right
        // edge minus any right-side buttons (ContextMenu, DirectMenu, OK, Cancel).
        double textLeft = indentPx + _buttonWidth;
        double labelRight = showValue
            ? _middle
            : w - (hasCtx || hasDirectMenu ? _buttonWidth : 0) - okCancelWidth;
        Rect labelRect = new Rect(textLeft, y, Math.Max(0, labelRight - textLeft), _lineHeight);

        // Selection / group fill
        if (isSel)
            ctx.FillRectangle(BrushSelected, labelRect);
        else if (isGroup)
            ctx.FillRectangle(BrushGroup, area);

        // ── Expand / collapse square (mirrors WinForms +/- box) ────────────
        if (hasChild)
        {
            double xm = indentPx + _buttonWidth / 2;
            double ym = y + _lineHeight / 2;
            double s2 = _square / 2;
            double s3 = Math.Max(1, _square / 3);
            var pen = PenText;  // tree button is left of labelRect, never on blue background
            ctx.DrawRectangle(null, pen,
                new Rect(xm - s2, ym - s2, _square, _square));
            ctx.DrawLine(pen, new Point(xm - s3, ym), new Point(xm + s3, ym)); // −
            if (!e.IsOpen)
                ctx.DrawLine(pen, new Point(xm, ym - s3), new Point(xm, ym + s3)); // |
        }

        // ── Checkbox (mirrors WinForms ControlPaint.DrawCheckBox) ──────────
        // Checkable entries show a box in the indent column, checked when Value == "1".
        if (isCheckable)
        {
            double xm = indentPx + _buttonWidth / 2;
            double ym = y + _lineHeight / 2;
            double s2 = _square / 2;
            var box = new Rect(xm - s2, ym - s2, _square, _square);
            ctx.DrawRectangle(BrushWindow, PenText, box);
            if (e.Value == "1")
            {
                // Simple check mark inside the box.
                double q = _square / 4;
                ctx.DrawLine(PenText, new Point(xm - q, ym),
                                       new Point(xm - q / 2, ym + q));
                ctx.DrawLine(PenText, new Point(xm - q / 2, ym + q),
                                       new Point(xm + q, ym - q));
            }
        }

        // Label text — strip and display shortcut badge when Shortcut flag is set
        string labelDisplayText = e.Label ?? "";
        char scMod1 = '\0', scMod2 = '\0';
        string scKey = string.Empty;
        if (flags.HasFlag(PropertyEntryType.Shortcut))
            (labelDisplayText, scMod1, scMod2, scKey) = SplitLabelText(labelDisplayText);

        bool isHighlight = flags.HasFlag(PropertyEntryType.Highlight);
        PutText(ctx, labelDisplayText, isBold && !isHighlight, isSel, labelRect,
                center: false, highlight: isHighlight);

        // Flag truncated labels for the hover extension. Mirrors WinForms:
        // only when there is a value column (otherwise the label already runs
        // to the right edge and an extension wouldn't reveal more).
        if (showValue && idx < _labelTruncated.Length && !string.IsNullOrEmpty(labelDisplayText))
        {
            var measure = new FormattedText(
                labelDisplayText, CultureInfo.CurrentCulture, FlowDirection.LeftToRight,
                (isBold && !isHighlight) ? TfBold : TfNormal, BaseFontSize, BrushText);
            if (measure.Width > labelRect.Width - 2)
                _labelTruncated[idx] = true;
        }

        if (flags.HasFlag(PropertyEntryType.Shortcut) && !string.IsNullOrEmpty(scKey))
            DrawShortcut(ctx, labelRect, scMod1, scMod2, scKey);

        // Middle divider — only for entries that have a value
        if (showValue)
            ctx.DrawLine(PenMiddle,
                new Point(_middle, y), new Point(_middle, y + _lineHeight));

        // Value text (leave room for dropdown / context-menu / OK / Cancel buttons)
        if (showValue)
        {
            double vRight = w
                - (hasCtx       ? _buttonWidth : 0)
                - (hasDD        ? _buttonWidth : 0)
                - okCancelWidth;
            PutValueText(ctx, e.Value!, new Rect(_middle, y, Math.Max(0, vRight - _middle), _lineHeight));
        }

        // DropDown ▾ button (mirrors WinForms ControlPaint.DrawComboButton)
        if (hasDD)
        {
            double btnLeft = w - (hasCtx ? _buttonWidth : 0) - _buttonWidth;
            var ddRect = new Rect(btnLeft + 0.5, y + 0.5, _buttonWidth - 1, _lineHeight - 1);
            ctx.DrawRectangle(BrushIndent, PenMiddle, ddRect);
            DrawTriangle(ctx, new Rect(btnLeft, y, _buttonWidth, _lineHeight), pointRight: false);
        }

        // Context-menu ⋮ button — drawn as three filled dots so it stays markant
        // and matches the size of the DropDown/DirectMenu triangles (the "⋮" glyph
        // renders too small/thin).
        if (hasCtx)
        {
            double cx = w - _buttonWidth / 2;
            double cy = y + _lineHeight / 2;
            double rd = Math.Max(1, _square / 6);   // dot radius
            double gp = rd * 2.5;                    // vertical spacing between dot centres
            foreach (double dy in new[] { -gp, 0.0, gp })
                ctx.DrawEllipse(BrushText, null, new Point(cx, cy + dy), rd, rd);
        }

        // DirectMenu ▶ button (only when no ContextMenu — mirrors WinForms else-if logic)
        if (!hasCtx && hasDirectMenu)
        {
            var dmRect = new Rect(w - _buttonWidth + 0.5, y + 0.5, _buttonWidth - 1, _lineHeight - 1);
            ctx.DrawRectangle(BrushIndent, PenMiddle, dmRect);
            DrawTriangle(ctx, new Rect(w - _buttonWidth, y, _buttonWidth, _lineHeight), pointRight: true);
        }

        // ✖ Cancel button — rightmost, width = _lineHeight (mirrors WinForms area.Height)
        if (hasCancelBtn)
        {
            var btnRect = new Rect(w - _lineHeight, y, _lineHeight, _lineHeight);
            ctx.DrawRectangle(BrushIndent, PenMiddle, btnRect.Deflate(0.5));
            DrawCross(ctx, btnRect);
        }

        // ✔ OK button — one position left of Cancel (or at rightmost when no Cancel button)
        if (hasOKBtn)
        {
            double left = w - (hasCancelBtn ? 2 : 1) * _lineHeight;
            var btnRect = new Rect(left, y, _lineHeight, _lineHeight);
            ctx.DrawRectangle(BrushIndent, PenMiddle, btnRect.Deflate(0.5));
            DrawCheck(ctx, btnRect);
        }
    }

    // ── Geometric button symbols ────────────────────────────────────────────
    // The browser/WASM font set lacks the ▼ ▶ ✔ ✖ glyphs (they render as the
    // missing-glyph box), so the buttons draw their symbols as plain geometry —
    // the same approach as the main menu's submenu triangle and the ⋮ dots above.

    private static void DrawTriangle(DrawingContext ctx, Rect r, bool pointRight)
    {
        double cx = r.X + r.Width / 2, cy = r.Y + r.Height / 2;
        double s = Math.Min(r.Width, r.Height) * 0.28;
        var g = new StreamGeometry();
        using (var gc = g.Open())
        {
            if (pointRight)
            {
                gc.BeginFigure(new Point(cx - s * 0.7, cy - s), true);
                gc.LineTo(new Point(cx + s, cy));
                gc.LineTo(new Point(cx - s * 0.7, cy + s));
            }
            else
            {
                gc.BeginFigure(new Point(cx - s, cy - s * 0.7), true);
                gc.LineTo(new Point(cx + s, cy - s * 0.7));
                gc.LineTo(new Point(cx, cy + s));
            }
            gc.EndFigure(true);
        }
        ctx.DrawGeometry(BrushText, null, g);
    }

    private static void DrawCross(DrawingContext ctx, Rect r)
    {
        double cx = r.X + r.Width / 2, cy = r.Y + r.Height / 2;
        double s = Math.Min(r.Width, r.Height) * 0.22;
        var pen = new Pen(BrushText, 1.6);
        ctx.DrawLine(pen, new Point(cx - s, cy - s), new Point(cx + s, cy + s));
        ctx.DrawLine(pen, new Point(cx - s, cy + s), new Point(cx + s, cy - s));
    }

    private static void DrawCheck(DrawingContext ctx, Rect r)
    {
        double cx = r.X + r.Width / 2, cy = r.Y + r.Height / 2;
        double s = Math.Min(r.Width, r.Height) * 0.26;
        var pen = new Pen(BrushText, 1.6);
        ctx.DrawLine(pen, new Point(cx - s, cy + s * 0.1), new Point(cx - s * 0.25, cy + s * 0.8));
        ctx.DrawLine(pen, new Point(cx - s * 0.25, cy + s * 0.8), new Point(cx + s, cy - s * 0.7));
    }

    // ── Shortcut helpers (ported from WinForms PropertyPage) ──────────────

    /// <summary>
    /// Splits a label string like "LabelText [[scX]]" into its text part and
    /// shortcut components (mirrors WinForms PropertyPage.SplitLabelText).
    /// </summary>
    private static (string text, char mod1, char mod2, string key) SplitLabelText(string labelText)
    {
        var match = _shortcutRegex.Match(labelText);
        if (!match.Success)
            return (labelText, '\0', '\0', string.Empty);

        string textPart  = match.Groups[1].Value.TrimEnd();
        char   firstMod  = match.Groups["modifier1"].Success && match.Groups["modifier1"].Length > 0
                               ? match.Groups["modifier1"].Value[0] : '\0';
        char   secondMod = match.Groups["modifier2"].Success && match.Groups["modifier2"].Length > 0
                               ? match.Groups["modifier2"].Value[0] : '\0';
        string key       = match.Groups["key"].Success ? match.Groups["key"].Value : string.Empty;
        return (textPart, firstMod, secondMod, key);
    }

    /// <summary>
    /// Draws keyboard-shortcut key-cap badges right-aligned inside <paramref name="labelRect"/>,
    /// mirroring WinForms PropertyPage.DrawShortcut.
    /// </summary>
    private void DrawShortcut(DrawingContext ctx, Rect labelRect, char mod1, char mod2, string key)
    {
        var parts = new List<string>(3);
        if (mod1 != '\0' && _keyboardModifier.TryGetValue(mod1, out var s1)) parts.Add(s1);
        if (mod2 != '\0' && _keyboardModifier.TryGetValue(mod2, out var s2)) parts.Add(s2);
        parts.Add(key);

        const double hPad   = 3.0;
        double fontSize = BaseFontSize * 0.85;
        double capH     = labelRect.Height - 4;
        double cornerR  = capH / 4;

        // Measure all key caps
        var fts    = new FormattedText[parts.Count];
        double totalW = 0;
        for (int i = 0; i < parts.Count; i++)
        {
            fts[i] = new FormattedText(parts[i], CultureInfo.CurrentCulture,
                FlowDirection.LeftToRight, TfNormal, fontSize, BrushText);
            totalW += fts[i].Width + hPad * 2;
        }

        double x    = labelRect.Right - totalW;
        double yMid = labelRect.Top + labelRect.Height / 2;

        for (int i = 0; i < parts.Count; i++)
        {
            double capW     = fts[i].Width + hPad * 2;
            var    capRect  = new Rect(x, yMid - capH / 2, capW, capH);
            ctx.DrawRectangle(Brushes.White, PenMiddle, capRect, cornerR, cornerR);
            ctx.DrawText(fts[i],
                new Point(capRect.Left + hPad, capRect.Top + (capRect.Height - fts[i].Height) / 2));
            x += capW;
        }
    }

    /// <summary>
    /// Called by <see cref="PropertiesExplorer.PreProcessKeyDown"/> when keyboard focus
    /// is in the canvas (not in this control).  Matches <paramref name="vk"/> (a Windows
    /// virtual-key code produced by <c>CadCanvas.AvaloniaKeyToVk</c>) against the shortcut
    /// embedded in every visible entry that carries <see cref="PropertyEntryType.Shortcut"/>.
    /// Selects and returns true on first match.
    /// </summary>
    internal bool TryHandleShortcutVk(int vk, bool ctrl, bool alt, bool shift)
    {
        if (_page == null) return false;

        // Map Windows VK code → shortcut key string (same format as SplitLabelText output)
        string scKey;
        if      (vk == 0x1B)                  scKey = "Esc";
        else if (vk == 0x2E)                  scKey = "Del";
        else if (vk >= 0x70 && vk <= 0x87)   scKey = "F" + (vk - 0x70 + 1);  // F1–F24
        else if (vk >= 0x41 && vk <= 0x5A)   scKey = ((char)vk).ToString();   // A–Z
        else return false;

        foreach (var entry in _entries)
        {
            if (!entry.Flags.HasFlag(PropertyEntryType.Shortcut)) continue;
            var (_, mod1, mod2, key) = SplitLabelText(entry.Label ?? "");
            if (key != scKey) continue;

            bool needCtrl  = mod1 == 'c' || mod2 == 'c';
            bool needAlt   = mod1 == 'a' || mod2 == 'a';
            bool needShift = mod1 == 's' || mod2 == 's';

            if (ctrl == needCtrl && alt == needAlt && shift == needShift)
            {
                _page.SelectEntry(entry);
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Returns true when the Avalonia <paramref name="key"/> corresponds to the
    /// shortcut key string extracted by <see cref="SplitLabelText"/>.
    /// </summary>
    private static bool KeyMatchesShortcut(Key key, string shortcutKey)
    {
        if (shortcutKey == "Esc") return key == Key.Escape;
        if (shortcutKey == "Del") return key == Key.Delete;
        if (shortcutKey.Length > 1 && shortcutKey[0] == 'F' &&
            int.TryParse(shortcutKey.Substring(1), out int fn) && fn >= 1 && fn <= 25)
            return (int)key == (int)Key.F1 + (fn - 1);
        if (shortcutKey.Length == 1 && shortcutKey[0] >= 'A' && shortcutKey[0] <= 'Z')
            return (int)key == (int)Key.A + (shortcutKey[0] - 'A');
        return false;
    }

    // ── Text helper ────────────────────────────────────────────────────────

    private void PutText(DrawingContext ctx, string text, bool bold, bool selected,
                         Rect clip, bool center, bool highlight = false)
    {
        if (string.IsNullOrEmpty(text) || clip.Width < 3) return;

        // Mirrors WinForms PropertyPage: selected → white on blue; otherwise a
        // Highlight-flagged label is drawn in red, regardless of Bold.
        IBrush brush = selected
            ? BrushHiText
            : (highlight ? BrushHighlight : BrushText);

        var ft = new FormattedText(
            text,
            CultureInfo.CurrentCulture,
            FlowDirection.LeftToRight,
            bold ? TfBold : TfNormal,
            BaseFontSize,
            brush);

        double x = center
            ? clip.Left + (clip.Width  - ft.Width)  / 2
            : clip.Left + 2;
        double ty = clip.Top + (clip.Height - ft.Height) / 2;

        using (ctx.PushClip(clip))
            ctx.DrawText(ft, new Point(x, ty));
    }

    // ── Label extension (full label over a truncated one) ──────────────────

    /// <summary>
    /// Draws the full label of a truncated entry over the value column, as a
    /// white box with a border (mirrors WinForms PropertiesExplorer.ShowLabelExtension /
    /// LabelExtension_Paint, which uses the textbox background and ControlText).
    /// </summary>
    private void DrawLabelExtension(DrawingContext ctx, int idx, double w)
    {
        var e = _entries[idx];
        double y        = idx * _lineHeight;
        double indentPx = e.IndentLevel * _buttonWidth;
        double left     = indentPx + _buttonWidth;
        var box = new Rect(left, y, Math.Max(0, w - left), _lineHeight);
        if (box.Width < 3) return;

        ctx.FillRectangle(BrushWindow, box);
        ctx.DrawRectangle(null, PenMiddle, box);

        bool isBold      = e.Flags.HasFlag(PropertyEntryType.Bold)
                           || e.Flags.HasFlag(PropertyEntryType.GroupTitle);
        bool isHighlight = e.Flags.HasFlag(PropertyEntryType.Highlight);
        string text = e.Label ?? "";
        if (e.Flags.HasFlag(PropertyEntryType.Shortcut))
            (text, _, _, _) = SplitLabelText(text);

        PutText(ctx, text, isBold && !isHighlight, selected: false, box,
                center: false, highlight: isHighlight);
    }

    /// <summary>
    /// Starts/cancels the hover delay that shows the label extension. Passing -1
    /// hides any current extension. Mirrors the WinForms 1 s tooltip-style delay.
    /// </summary>
    private void UpdateLabelExtensionHover(int idx)
    {
        if (idx == _hoverLabelIdx) return;
        _hoverLabelIdx = idx;
        _hoverTimer?.Stop();

        if (_labelExtensionIdx != -1)
        {
            _labelExtensionIdx = -1;
            InvalidateVisual();
        }
        if (idx < 0) return;

        if (_hoverTimer == null)
        {
            _hoverTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(700) };
            _hoverTimer.Tick += (_, _) =>
            {
                _hoverTimer!.Stop();
                if (_hoverLabelIdx >= 0 && _hoverLabelIdx < _labelTruncated.Length
                    && _labelTruncated[_hoverLabelIdx])
                {
                    _labelExtensionIdx = _hoverLabelIdx;
                    InvalidateVisual();
                }
            };
        }
        _hoverTimer.Start();
    }

    // ── Tooltip (mirrors WinForms PropertyPage tooltip over a label) ───────

    /// <summary>
    /// Builds the tooltip text for an entry from its <see cref="IPropertyEntry.ResourceId"/>,
    /// mirroring WinForms PropertyPage.OnMouseMove: a defined ResourceId (or one starting
    /// with "@") resolves through the StringTable (tip category, falling back to label);
    /// anything else is shown verbatim. Returns null when no tooltip should appear.
    /// </summary>
    private static string? ComputeToolTip(IPropertyEntry entry)
    {
        string? rid = entry.ResourceId;
        if (rid == null) return null;
        if (StringTable.IsStringDefined(rid) || rid.StartsWith("@"))
        {
            string? toDisplay = StringTable.GetString(rid, StringTable.Category.tip);
            if (toDisplay == null) toDisplay = StringTable.GetString(rid, StringTable.Category.label);
            return toDisplay;
        }
        return rid;
    }

    /// <summary>
    /// Updates the hover tooltip for the entry currently under the cursor. Called from
    /// OnPointerMoved. Only labels carry a tooltip; moving to a different tooltip text
    /// restarts the show delay, moving off a label hides it. Mirrors the Forms logic that
    /// keys the displayed text on the entry's ResourceId and shows it after a short delay.
    /// </summary>
    private void UpdateToolTip(int idx, EMousePos pos)
    {
        string? toDisplay = (pos == EMousePos.OnLabel && idx >= 0 && idx < _entries.Count)
            ? ComputeToolTip(_entries[idx])
            : null;

        if (_currentToolTip == toDisplay) return;   // unchanged → keep current state / running delay
        _currentToolTip = toDisplay;

        HideToolTip();
        if (!string.IsNullOrEmpty(toDisplay))
        {
            _pendingToolTip = toDisplay;
            _toolTipDelay ??= new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(500) };
            _toolTipDelay.Stop();
            _toolTipDelay.Tick -= OnToolTipDelayTick;
            _toolTipDelay.Tick += OnToolTipDelayTick;
            _toolTipDelay.Start();
        }
    }

    private void OnToolTipDelayTick(object? sender, EventArgs e)
    {
        _toolTipDelay!.Stop();
        if (string.IsNullOrEmpty(_pendingToolTip)) return;

        ToolTip.SetTip(this, _pendingToolTip);
        ToolTip.SetIsOpen(this, true);

        // Auto-hide after a few seconds (Forms ToolTipOff). _currentToolTip is left set so
        // the tooltip does not immediately reopen while the cursor lingers on the same label.
        _toolTipOff ??= new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(3500) };
        _toolTipOff.Stop();
        _toolTipOff.Tick -= OnToolTipOffTick;
        _toolTipOff.Tick += OnToolTipOffTick;
        _toolTipOff.Start();
    }

    private void OnToolTipOffTick(object? sender, EventArgs e)
    {
        _toolTipOff!.Stop();
        ToolTip.SetIsOpen(this, false);
    }

    /// <summary>Cancels any pending/visible tooltip without clearing <see cref="_currentToolTip"/>.</summary>
    private void HideToolTip()
    {
        _toolTipDelay?.Stop();
        _toolTipOff?.Stop();
        if (ToolTip.GetIsOpen(this)) ToolTip.SetIsOpen(this, false);
    }

    // ── Hit testing (mirrors WinForms GetMousePosition) ────────────────────

    private enum EMousePos
    {
        Outside, OnTreeButton, OnLabel, OnValue,
        OnContextMenu, OnMiddleLine,
        OnDropDown, OnOkButton, OnCancelButton, OnLockButton, OnCheckbox, OnDirectMenu
    }

    private (int idx, EMousePos pos) HitTest(Point p)
    {
        if (_entries.Count == 0) return (-1, EMousePos.Outside);
        int i = (int)Math.Floor(p.Y / _lineHeight);
        if (i < 0 || i >= _entries.Count) return (-1, EMousePos.Outside);

        var e         = _entries[i];
        double tL     = e.IndentLevel * _buttonWidth;
        double tR     = tL + _buttonWidth;
        double width  = EffW;   // exclude scrollbar reserve
        bool   hasVal = e.Value != null;

        if (e.Flags.HasFlag(PropertyEntryType.Checkable) && p.X >= tL && p.X <= tR)
            return (i, EMousePos.OnCheckbox);

        if (e.Flags.HasFlag(PropertyEntryType.HasSubEntries) && p.X >= tL && p.X <= tR)
            return (i, EMousePos.OnTreeButton);

        if (hasVal && Math.Abs(p.X - _middle) <= 3)
            return (i, EMousePos.OnMiddleLine);

        if (e.Flags.HasFlag(PropertyEntryType.ContextMenu) && p.X >= width - _buttonWidth)
            return (i, EMousePos.OnContextMenu);

        if (e.Flags.HasFlag(PropertyEntryType.DirectMenu)
            && !e.Flags.HasFlag(PropertyEntryType.ContextMenu)
            && p.X >= width - _buttonWidth)
            return (i, EMousePos.OnDirectMenu);

        if (e.Flags.HasFlag(PropertyEntryType.DropDown) && hasVal && p.X >= width - _buttonWidth)
            return (i, EMousePos.OnDropDown);

        // Cancel (✖) is at the rightmost position, width = _lineHeight.
        // OK (✔) is directly to its left. Order matters: check Cancel before OK.
        if (e.Flags.HasFlag(PropertyEntryType.CancelButton) && p.X >= width - _lineHeight)
            return (i, EMousePos.OnCancelButton);

        if (e.Flags.HasFlag(PropertyEntryType.OKButton) && p.X >= width - 2 * _lineHeight)
            return (i, EMousePos.OnOkButton);

        if (hasVal && p.X >= tL && p.X < _middle)  return (i, EMousePos.OnLabel);
        if (!hasVal && p.X >= tL)                   return (i, EMousePos.OnLabel);
        if (hasVal && p.X >= _middle)               return (i, EMousePos.OnValue);

        return (i, EMousePos.Outside);
    }

    // ── Pointer events ─────────────────────────────────────────────────────

    protected override void OnPointerPressed(PointerPressedEventArgs e)
    {
        base.OnPointerPressed(e);
        UpdateLabelExtensionHover(-1);   // dismiss extension on click (mirrors Forms OnMouseDown)
        _currentToolTip = null;
        HideToolTip();                   // dismiss tooltip on click (mirrors Forms OnMouseDown)
        if (!e.GetCurrentPoint(this).Properties.IsLeftButtonPressed) return;
        if (HitTest(e.GetPosition(this)).pos == EMousePos.OnMiddleLine)
        {
            _draggingMiddle = true;
            e.Pointer.Capture(this);
            e.Handled = true;
        }
    }

    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        base.OnPointerReleased(e);

        if (_draggingMiddle)
        {
            _draggingMiddle = false;
            e.Pointer.Capture(null);
            e.Handled = true;
            return;
        }
        if (e.InitialPressMouseButton != MouseButton.Left) return;

        var (idx, pos) = HitTest(e.GetPosition(this));
        if (idx < 0 || _page == null) return;
        var entry = _entries[idx];

        // DropDown entries treat any value-area click as dropdown trigger (mirrors Forms.NET8 line 649)
        if (pos == EMousePos.OnValue && entry.Flags.HasFlag(PropertyEntryType.DropDown))
            pos = EMousePos.OnDropDown;

        // When a TextBox or DropDown will take keyboard focus we must NOT call Focus()
        // on this control afterwards — that would steal focus back and immediately fire
        // LostFocus on the floating control, closing it before the user can type.
        bool takeFocus = true;

        switch (pos)
        {
            case EMousePos.OnTreeButton:
                RequestHideTextBox?.Invoke();
                RequestHideDropDown?.Invoke();
                _page.OpenSubEntries(entry, !entry.IsOpen);
                _page.SelectEntry(entry);
                break;

            case EMousePos.OnLabel:
                if (entry.Flags.HasFlag(PropertyEntryType.LabelEditable)
                    && entry == _page.GetCurrentSelection())
                {
                    // Second click on an already-selected label-editable entry → edit label.
                    // Mirrors WinForms: selected == index && LabelEditable → ShowTextBox(index, loc, false)
                    RequestHideTextBox?.Invoke();
                    RequestHideDropDown?.Invoke();
                    entry.StartEdit(false);
                    RequestShowTextBox?.Invoke(LabelRect(idx), entry.Label ?? "", entry);
                    takeFocus = false;
                }
                else if (entry.Flags.HasFlag(PropertyEntryType.Selectable))
                {
                    RequestHideTextBox?.Invoke();
                    RequestHideDropDown?.Invoke();
                    _page.SelectEntry(entry);
                }
                break;

            case EMousePos.OnDropDown:
            {
                _page.SelectEntry(entry);
                string[] items = entry.GetDropDownList();
                int selInd = -1;
                for (int k = 0; k < items.Length; k++)
                    if (entry.Value == items[k]) { selInd = k; break; }
                RequestShowDropDown?.Invoke(ValueRect(idx), items, selInd, entry);
                takeFocus = false;
                break;
            }

            case EMousePos.OnValue:
                // Close any currently open TextBox (calls EndEdit on the previous entry)
                // before selecting the new one — mirrors WinForms comment on SelectedIndex:
                // "before ShowTextBox, because this calls unselected and thus updates the
                //  currently textBox or listBox (if any)".
                RequestHideTextBox?.Invoke();
                RequestHideDropDown?.Invoke();
                if (entry.Flags.HasFlag(PropertyEntryType.Selectable))
                    _page.SelectEntry(entry);
                if (entry.Flags.HasFlag(PropertyEntryType.ValueEditable) && !entry.ReadOnly)
                {
                    entry.StartEdit(true);
                    RequestShowTextBox?.Invoke(ValueRect(idx), entry.Value ?? "", entry);
                    takeFocus = false;
                }
                else if (entry.Flags.HasFlag(PropertyEntryType.ValueAsButton))
                    entry.ButtonClicked(PropertyEntryButton.value);
                break;

            case EMousePos.OnOkButton:
                RequestHideTextBox?.Invoke();
                RequestHideDropDown?.Invoke();
                entry.ButtonClicked(PropertyEntryButton.ok);
                break;

            case EMousePos.OnCancelButton:
                RequestHideTextBox?.Invoke();
                RequestHideDropDown?.Invoke();
                entry.ButtonClicked(PropertyEntryButton.cancel);
                break;

            case EMousePos.OnDirectMenu:
                _page.SelectEntry(entry);
                entry.ButtonClicked(PropertyEntryButton.directMenu);
                break;

            case EMousePos.OnCheckbox:
                entry.ButtonClicked(PropertyEntryButton.check);
                break;

            case EMousePos.OnContextMenu:
                if (entry.ContextMenu == null)
                    throw new NotImplementedException(
                        "implement ContextMenu of " + entry.GetType() + ", " + entry.ResourceId);
                {
                    var cm = MenuManager.MakeContextMenu(entry.ContextMenu);
                    cm.UpdateCommand();
                    cm.Placement = PlacementMode.Pointer;
                    cm.Open(this);
                }
                break;
        }

        // Only grab keyboard focus when no floating control (TextBox / ListBox) took it.
        // Calling Focus() here after RequestShowTextBox would fire LostFocus on the TextBox
        // and immediately close it via HideTextBox.
        if (takeFocus) Focus();
    }

    protected override void OnPointerMoved(PointerEventArgs e)
    {
        base.OnPointerMoved(e);
        var pt = e.GetPosition(this);

        if (_draggingMiddle)
        {
            _currentToolTip = null;
            HideToolTip();
            double fct = Math.Clamp(pt.X / Math.Max(1, Bounds.Width), 0.1, 0.9);
            _middle = fct * Bounds.Width;

            // Keep floating TextBox aligned with the new middle position
            int si = FindSelectedIndex();
            if (si >= 0 && _entries[si].Flags.HasFlag(PropertyEntryType.ValueEditable))
                RequestShowTextBox?.Invoke(ValueRect(si), _entries[si].Value ?? "", _entries[si]);

            InvalidateVisual();
            e.Handled = true;
            return;
        }

        var (hitIdx, hitPos) = HitTest(pt);
        Cursor = hitPos switch
        {
            EMousePos.OnMiddleLine   => new Cursor(StandardCursorType.SizeWestEast),
            EMousePos.OnTreeButton   => new Cursor(StandardCursorType.Hand),
            EMousePos.OnCheckbox     => new Cursor(StandardCursorType.Hand),
            EMousePos.OnContextMenu  => new Cursor(StandardCursorType.Hand),
            EMousePos.OnDirectMenu   => new Cursor(StandardCursorType.Hand),
            EMousePos.OnDropDown     => new Cursor(StandardCursorType.Hand),
            EMousePos.OnOkButton     => new Cursor(StandardCursorType.Hand),
            EMousePos.OnCancelButton => new Cursor(StandardCursorType.Hand),
            // ValueAsButton: hand cursor so the user sees it is clickable
            EMousePos.OnValue when hitIdx >= 0 &&
                _entries[hitIdx].Flags.HasFlag(PropertyEntryType.ValueAsButton)
                                 => new Cursor(StandardCursorType.Hand),
            EMousePos.OnValue    => new Cursor(StandardCursorType.Ibeam),
            _                    => Cursor.Default,
        };

        // Label extension: keep showing while the cursor stays on the extension's
        // own row+area (it overlays the value column, so HitTest would report
        // OnValue there); otherwise arm/clear the hover delay for truncated labels.
        int rowUnder = (int)Math.Floor(pt.Y / _lineHeight);
        bool stayOnExtension =
            _labelExtensionIdx >= 0 && rowUnder == _labelExtensionIdx
            && pt.X >= _entries[_labelExtensionIdx].IndentLevel * _buttonWidth + _buttonWidth
            && pt.X <= EffW;
        if (!stayOnExtension)
        {
            bool overTruncated = hitPos == EMousePos.OnLabel && hitIdx >= 0
                && hitIdx < _labelTruncated.Length && _labelTruncated[hitIdx];
            UpdateLabelExtensionHover(overTruncated ? hitIdx : -1);
        }

        UpdateToolTip(hitIdx, hitPos);
    }

    protected override void OnPointerExited(PointerEventArgs e)
    {
        base.OnPointerExited(e);
        UpdateLabelExtensionHover(-1);
        _currentToolTip = null;
        HideToolTip();
    }

    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);
        if (_page == null) return;

        bool ctrl  = e.KeyModifiers.HasFlag(KeyModifiers.Control);
        bool shift = e.KeyModifiers.HasFlag(KeyModifiers.Shift);

        switch (e.Key)
        {
            case Key.Tab:
                RequestHideTextBox?.Invoke();
                RequestHideDropDown?.Invoke();
                NavigateToNextEntry(!shift);
                e.Handled = true;
                break;
            case Key.Enter:
                TriggerEnterKey(ctrl);
                e.Handled = true;
                break;
            case Key.Escape:
            {
                var sel = _page.GetCurrentSelection();
                if (sel != null && sel.Flags.HasFlag(PropertyEntryType.CancelButton))
                    sel.ButtonClicked(PropertyEntryButton.cancel);
                e.Handled = true;
                break;
            }
        }

        // Check if any visible entry claims this key as a shortcut
        if (!e.Handled)
        {
            bool alt = e.KeyModifiers.HasFlag(KeyModifiers.Alt);
            foreach (var entry in _entries)
            {
                if (!entry.Flags.HasFlag(PropertyEntryType.Shortcut)) continue;
                var (_, scMod1, scMod2, scKey) = SplitLabelText(entry.Label ?? "");
                if (string.IsNullOrEmpty(scKey)) continue;
                if (!KeyMatchesShortcut(e.Key, scKey)) continue;

                bool entryNeedsCtrl  = scMod1 == 'c' || scMod2 == 'c';
                bool entryNeedsAlt   = scMod1 == 'a' || scMod2 == 'a';
                bool entryNeedsShift = scMod1 == 's' || scMod2 == 's';

                if (ctrl == entryNeedsCtrl && alt == entryNeedsAlt && shift == entryNeedsShift)
                {
                    _page.SelectEntry(entry);
                    e.Handled = true;
                    break;
                }
            }
        }
    }

    /// <summary>Navigate to the next (forward=true) or previous selectable entry and open an input control if appropriate.</summary>
    public void NavigateToNextEntry(bool forward)
    {
        if (_page == null) return;
        _page.SelectNextPropertyEntry(forward);
        ShowInputForCurrentSelection();
    }

    /// <summary>Execute the Enter-key action on the currently selected entry, mirroring WinForms PropertyPage.OnEnter.</summary>
    public void TriggerEnterKey(bool ctrl)
    {
        if (_page == null) return;
        var sel = _page.GetCurrentSelection();
        if (sel == null) return;

        if (ctrl && sel.Flags.HasFlag(PropertyEntryType.HasSubEntries))
        {
            RequestHideTextBox?.Invoke();
            RequestHideDropDown?.Invoke();
            _page.OpenSubEntries(sel, !sel.IsOpen);
            _page.SelectEntry(sel);
        }
        else if (sel.Flags.HasFlag(PropertyEntryType.ValueEditable))
        {
            int si = FindSelectedIndex();
            if (si >= 0) { sel.StartEdit(true); RequestShowTextBox?.Invoke(ValueRect(si), sel.Value ?? "", sel); }
        }
        else if (sel.Flags.HasFlag(PropertyEntryType.DropDown))
        {
            int si = FindSelectedIndex();
            if (si >= 0)
            {
                string[] items = sel.GetDropDownList();
                int selInd = -1;
                for (int k = 0; k < items.Length; k++)
                    if (sel.Value == items[k]) { selInd = k; break; }
                RequestShowDropDown?.Invoke(ValueRect(si), items, selInd, sel);
            }
        }
        else if (sel.Flags.HasFlag(PropertyEntryType.HasSubEntries))
        {
            _page.OpenSubEntries(sel, !sel.IsOpen);
            _page.SelectEntry(sel);
        }
        else if (sel.Flags.HasFlag(PropertyEntryType.DirectMenu))
        {
            sel.ButtonClicked(PropertyEntryButton.directMenu);
        }
        else if (sel.Flags.HasFlag(PropertyEntryType.OKButton))
        {
            sel.ButtonClicked(PropertyEntryButton.ok);
        }
    }

    internal void ShowInputForCurrentSelection()
    {
        var sel = _page?.GetCurrentSelection();
        if (sel == null) return;
        int si = FindSelectedIndex();
        if (si < 0) return;
        if (sel.Flags.HasFlag(PropertyEntryType.ValueEditable) && !sel.Flags.HasFlag(PropertyEntryType.LabelEditable))
        {
            sel.StartEdit(true);
            RequestShowTextBox?.Invoke(ValueRect(si), sel.Value ?? "", sel);
        }
        else if (sel.Flags.HasFlag(PropertyEntryType.DropDown) && !sel.Flags.HasFlag(PropertyEntryType.LabelEditable))
        {
            string[] items = sel.GetDropDownList();
            int selInd = -1;
            for (int k = 0; k < items.Length; k++)
                if (sel.Value == items[k]) { selInd = k; break; }
            RequestShowDropDown?.Invoke(ValueRect(si), items, selInd, sel);
        }
    }

    private void OnDoubleTappedHandler(object? sender, TappedEventArgs e)
    {
        var (idx, pos) = HitTest(e.GetPosition(this));
        if (idx < 0 || _page == null) return;
        var entry = _entries[idx];
        if (pos == EMousePos.OnLabel && entry == _page.GetCurrentSelection())
            entry.ButtonClicked(PropertyEntryButton.doubleclick);
    }

    // ── Rect helpers ───────────────────────────────────────────────────────

    public Rect ValueRect(int idx)
    {
        if (idx < 0 || idx >= _entries.Count) return default;
        var e = _entries[idx];
        double vRight = EffW
            - (e.Flags.HasFlag(PropertyEntryType.ContextMenu) ? _buttonWidth : 0);
        return new Rect(_middle, idx * _lineHeight, vRight - _middle, _lineHeight);
    }

    /// <summary>
    /// Returns the label area for entry <paramref name="idx"/> in local coordinates.
    /// Mirrors WinForms PropertyPage.LabelArea, used to position the floating TextBox
    /// when editing an entry whose label is editable (PropertyEntryType.LabelEditable).
    /// </summary>
    public Rect LabelRect(int idx)
    {
        if (idx < 0 || idx >= _entries.Count) return default;
        var e    = _entries[idx];
        double y       = idx * _lineHeight;
        double textLeft = e.IndentLevel * _buttonWidth + _buttonWidth;
        bool   hasValue = e.Value != null && !e.Flags.HasFlag(PropertyEntryType.Checkable);
        // When a value column exists the label stops at _middle; otherwise it fills to the right.
        double right = hasValue
            ? _middle
            : EffW - (e.Flags.HasFlag(PropertyEntryType.ContextMenu) ? _buttonWidth : 0);
        return new Rect(textLeft, y, Math.Max(0, right - textLeft), _lineHeight);
    }

    public int FindSelectedIndex()
    {
        var sel = _page?.GetCurrentSelection();
        if (sel == null) return -1;
        for (int i = 0; i < _entries.Count; i++)
            if (_entries[i] == sel) return i;
        return -1;
    }

    // ── Value text with ColorBox support (mirrors Forms.NET8 DrawString) ──────

    /// <summary>
    /// Draws a value string inside <paramref name="clip"/>.
    /// If the text starts with <c>[[ColorBox:R:G:B]]</c> a filled colour square is
    /// drawn first and the remaining text follows to its right — exactly as in the
    /// WinForms PropertyPage.DrawString implementation.
    /// Any other <c>[[…]]</c> hint prefix is silently stripped.
    /// </summary>
    private void PutValueText(DrawingContext ctx, string text, Rect clip)
    {
        if (string.IsNullOrEmpty(text) || clip.Width < 3) return;

        if (text.StartsWith("[["))
        {
            string[] parts   = text.Split(["]]"], StringSplitOptions.None);
            string   rest    = parts[parts.Length - 1].TrimStart();
            string[] command = parts[0].Substring(2).Split(':');

            if (command[0] == "ColorBox" && command.Length == 4 &&
                int.TryParse(command[1], out int r) &&
                int.TryParse(command[2], out int g) &&
                int.TryParse(command[3], out int b))
            {
                double sz      = clip.Height - 2;
                var    boxRect = new Rect(clip.Left + 1, clip.Top + 1, sz, sz);
                ctx.FillRectangle(new SolidColorBrush(Color.FromRgb((byte)r, (byte)g, (byte)b)), boxRect);
                ctx.DrawRectangle(null, PenText, boxRect.Deflate(0.5));
                double newLeft = clip.Left + sz + 3;
                clip = new Rect(newLeft, clip.Top, Math.Max(0, clip.Right - newLeft), clip.Height);
            }

            text = rest;
        }

        PutText(ctx, text, false, false, clip, center: false);
    }
}
