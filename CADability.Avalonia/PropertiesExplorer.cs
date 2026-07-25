using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Primitives;  // ScrollBarVisibility
using Avalonia.Controls.Templates;   // FuncDataTemplate
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Styling;
using Avalonia.Threading;
using CADability;
using CADability.UserInterface;
using System.Collections.Generic;
using System.Linq;
using AvaloniaGrid = Avalonia.Controls.Grid;
using KeyEventArgs = CADability.Substitutes.KeyEventArgs;

namespace CADability.Avalonia;

/// <summary>
/// Avalonia UserControl implementing <see cref="IControlCenter"/>.
///
/// Layout:
///   ┌─────────────────────────────────────┐  ← 22 px compact tab strip
///   │ [Tab 1] [Tab 2] [Tab 3] [Tab 4] …  │    (one entry-row high, star-columns)
///   ├─────────────────────────────────────┤
///   │  active PropertyPageControl         │  ← fills remaining height
///   │  + floating TextBox overlay         │
///   └─────────────────────────────────────┘
///
/// The tab strip is a plain <see cref="AvaloniaGrid"/> with equal star-columns,
/// so all riders are always the same width regardless of text length.
/// Text is clipped with ellipsis; FontSize 10 keeps everything within 22 px.
///
/// The active page is shown in a <see cref="ContentControl"/> (_pageArea) whose
/// content is swapped on tab selection — no Fluent TabControl involved.
/// </summary>
public class PropertiesExplorer : UserControl, IControlCenter
{
    // ── Outer layout ───────────────────────────────────────────────────────

    private readonly AvaloniaGrid    _layout   = new();
    private readonly AvaloniaGrid    _tabStrip = new();
    private readonly ContentControl  _pageArea = new()
    {
        HorizontalContentAlignment = HorizontalAlignment.Stretch,
        VerticalContentAlignment   = VerticalAlignment.Stretch,
    };

    private string? _activeTabId;

    // ── Tab entries ────────────────────────────────────────────────────────

    private record TabEntry(
        PropertyPage        Page,
        Border              TabBorder,
        TextBlock           TabLabel,
        PropertyPageControl Control,
        TextBox             FloatingTextBox,
        ListBox             FloatingListBox,
        ScrollViewer        Scroll);

    private readonly Dictionary<string, TabEntry> _tabs = new();

    // ── Editing state (one floating TextBox active at a time) ──────────────

    private IPropertyEntry? _editingEntry;
    private string          _editingOriginalValue = "";

    // Live-preview state (mirrors WinForms PropertiesExplorer.Refresh / textBox.Modified):
    // while a value is being edited, external value changes (e.g. a point/length following the mouse)
    // are reflected into the editor text — but only until the user starts typing.
    //
    // _programmaticText holds the last text WE pushed into the editor (on open or live preview).
    // Avalonia raises TextBox.TextChanged ASYNCHRONOUSLY, so a bool flag set around the assignment is
    // already reset by the time the event fires; we therefore recognise our own updates by comparing
    // the value. A TextChanged whose text differs from _programmaticText is a genuine user edit — only
    // then do we forward it via EditTextChanged. This matters a lot: forwarding the editor's initial
    // value on open would call GeoPointProperty.OnSetValue → SetFixed(true), fixing a ConstructAction's
    // point input the instant its editor opens and breaking the mouse-follow.
    private string? _programmaticText;
    // Latches once the user has typed, which stops further mouse-driven live-preview refreshes.
    private bool _editUserModified;

    // ── DropDown state (one floating ListBox active at a time) ─────────────

    private IPropertyEntry? _dropDownEntry;
    private string?         _dropDownTitleId;

    // ── Tab-strip palette ──────────────────────────────────────────────────

    private const double TabHeight = 22;   // = one PropertyPageControl row

    private static readonly IBrush TabActiveBg   = new SolidColorBrush(Color.FromRgb(0x00, 0x78, 0xD4));
    private static readonly IBrush TabInactiveBg = new SolidColorBrush(Color.FromRgb(0xE8, 0xE8, 0xE8));
    private static readonly IBrush TabActiveFg   = Brushes.White;
    private static readonly IBrush TabInactiveFg = new SolidColorBrush(Color.FromRgb(0x20, 0x20, 0x20));
    private static readonly IBrush StripSepBrush = new SolidColorBrush(Color.FromRgb(0xC0, 0xC0, 0xC0));

    // ── IControlCenter: Frame ──────────────────────────────────────────────

    public IFrame Frame { get; set; } = null!;

    // ── Constructor ────────────────────────────────────────────────────────

    public PropertiesExplorer()
    {
        // Row 0: compact tab strip; Row 1: active page fills remaining space
        _layout.RowDefinitions.Add(new RowDefinition(new GridLength(TabHeight)));
        _layout.RowDefinitions.Add(new RowDefinition(new GridLength(1, GridUnitType.Star)));

        // Wrap strip in a Border so we get a 1 px separator line at the bottom
        var stripWrap = new Border
        {
            Child           = _tabStrip,
            Background      = TabInactiveBg,
            BorderBrush     = StripSepBrush,
            BorderThickness = new Thickness(0, 0, 0, 1),
        };

        AvaloniaGrid.SetRow(stripWrap, 0);
        AvaloniaGrid.SetRow(_pageArea,  1);
        _layout.Children.Add(stripWrap);
        _layout.Children.Add(_pageArea);

        Content = _layout;
    }

    // ── IControlCenter ─────────────────────────────────────────────────────

    public IPropertyPage AddPropertyPage(string titleId, int iconId)
    {
        if (_tabs.TryGetValue(titleId, out var existing))
            return existing.Page;

        var page = new PropertyPage(titleId, Frame);
        page.BringToFrontRequested += () => ShowPropertyPage(titleId);

        // Safety net: if the entry currently being edited is removed (e.g. a ConstructAction
        // finished via Enter and tore down its property entries) hide the dangling TextBox.
        //
        // IMPORTANT: some pages REBUILD all their entries on every change — the ShapeIt
        // modelling page recomposes after each value keystroke, replacing every entry with a
        // NEW instance while an equivalent row (same ResourceId) is still shown. Killing the
        // edit on the instance check alone closed the value editor after a single keystroke
        // (couldn't type a second digit into Breite/Höhe of a finished drawing). So the
        // decision is deferred until the rebuild settled; then the running edit is RE-BOUND
        // to the equivalent new entry (text, caret and focus stay), and only an entry that
        // is truly gone ends the edit.
        page.Changed += () =>
        {
            if (_editingEntry == null || _activeTabId != titleId) return;
            if (!_tabs.TryGetValue(titleId, out var tab)) return;

            // Live preview (mirrors WinForms PropertiesExplorer.Refresh): while the edited entry is
            // still present and the user has not started typing, reflect its current value — which may
            // have just changed externally, e.g. a point/length following the mouse — into the editor.
            if (page.ContainsEntry(_editingEntry))
            {
                if (tab.FloatingTextBox.IsVisible && !_editUserModified)
                    RefreshEditorTextFromValue(tab.FloatingTextBox, _editingEntry);
                return;
            }
            string editingId = _editingEntry.ResourceId;
            Dispatcher.UIThread.Post(() =>
            {
                if (_editingEntry == null || _activeTabId != titleId) return;
                if (page.ContainsEntry(_editingEntry)) return;
                if (!_tabs.TryGetValue(titleId, out var t)) return;
                var replacement = t.FloatingTextBox.IsVisible ? t.Control.FindShownEntry(editingId) : null;
                if (replacement != null)
                {
                    _editingEntry = replacement;
                    if (t.Control.TryGetValueRect(replacement, out var r))
                    {
                        t.FloatingTextBox.Margin = new Thickness(r.Left, r.Top, 0, 0);
                        t.FloatingTextBox.Width  = r.Width;
                        t.FloatingTextBox.Height = r.Height;
                    }
                    if (!t.FloatingTextBox.IsFocused) t.FloatingTextBox.Focus();
                    // Same live preview after a rebuild replaced the entry instance.
                    if (!_editUserModified) RefreshEditorTextFromValue(t.FloatingTextBox, replacement);
                }
                else
                {
                    if (t.FloatingTextBox.IsVisible) t.FloatingTextBox.IsVisible = false;
                    t.Control.SetEditing(false);
                    _editingEntry = null;
                }
            }, DispatcherPriority.Background);
        };

        // ── Custom-drawn entry list ────────────────────────────────────────
        var control = new PropertyPageControl
        {
            HorizontalAlignment = HorizontalAlignment.Stretch,
            VerticalAlignment   = VerticalAlignment.Top,
        };
        control.Attach(page);

        // ── Floating TextBox ───────────────────────────────────────────────
        var floatingTb = new TextBox
        {
            IsVisible                  = false,
            HorizontalAlignment        = HorizontalAlignment.Left,
            VerticalAlignment          = VerticalAlignment.Top,
            ZIndex                     = 10,
            AcceptsReturn              = false,
            MaxLines                   = 1,
            MinHeight                  = 0,
            VerticalContentAlignment   = VerticalAlignment.Center,
            Padding                    = new Thickness(2, 0),
            BorderBrush                = new SolidColorBrush(Color.FromRgb(0x00, 0x78, 0xD4)),
            BorderThickness            = new Thickness(1),
            Background                 = Brushes.White,
            Foreground                 = Brushes.Black,
            CaretBrush                 = Brushes.Black,
        };

        floatingTb.LostFocus   += (_, _) => HideTextBox(titleId, aborted: false);
        floatingTb.KeyDown     += (_, e) =>
        {
            switch (e.Key)
            {
                case Key.Escape:
                    HideTextBox(titleId, aborted: true);
                    e.Handled = true;
                    break;
                case Key.Enter:
                    HandleEditingEnterOrTab(titleId, isTab: false, e.KeyModifiers);
                    e.Handled = true;
                    break;
                case Key.Tab:
                    HandleEditingEnterOrTab(titleId, isTab: true, e.KeyModifiers);
                    e.Handled = true;
                    break;
            }
        };
        floatingTb.TextChanged += (_, _) =>
        {
            string cur = floatingTb.Text ?? "";
            // Text we pushed ourselves (open or live mouse preview) → not a user edit. Compare by value
            // because Avalonia raises this event asynchronously (a timing flag would be unreliable).
            if (cur == _programmaticText) return;
            _editUserModified = true;   // user typed → stop mouse-driven live preview from now on
            if (_editingEntry == null) return;
            bool ok = _editingEntry.EditTextChanged(cur);
            floatingTb.Foreground = ok ? Brushes.Black : Brushes.Red;
        };

        // ── Wire PropertyPageControl callbacks ─────────────────────────────
        control.RequestShowTextBox = (rect, text, entry) =>
        {
            // Already editing this exact entry → only keep the editor aligned (e.g. the selection
            // funnel opened it and the pointer handler calls again, or the middle divider is dragged).
            // Do NOT reset the text (that would discard what the user typed) or re-focus/re-select.
            if (_editingEntry == entry && floatingTb.IsVisible)
            {
                floatingTb.Margin = new Thickness(rect.Left, rect.Top, 0, 0);
                floatingTb.Width  = rect.Width;
                floatingTb.Height = rect.Height;
                return;
            }
            _editingEntry         = entry;
            _editingOriginalValue = text;
            _editUserModified     = false;   // fresh editor → allow mouse-driven live preview again
            _programmaticText     = text;    // setting Text below is programmatic, not a user edit
            floatingTb.Text       = text;
            floatingTb.Margin     = new Thickness(rect.Left, rect.Top, 0, 0);
            floatingTb.Width      = rect.Width;
            floatingTb.Height     = rect.Height;
            floatingTb.IsVisible  = true;
            floatingTb.Focus();
            floatingTb.SelectAll();
            control.SetEditing(true);   // suppress focus-stealing relayout while typing a value
        };
        control.RequestHideTextBox = () => HideTextBox(titleId, aborted: false);

        // ── Floating ListBox (dropdown) ────────────────────────────────────
        var floatingLb = new ListBox
        {
            IsVisible           = false,
            HorizontalAlignment = HorizontalAlignment.Left,
            VerticalAlignment   = VerticalAlignment.Top,
            ZIndex              = 11,           // above TextBox (ZIndex 10)
            Background          = Brushes.White,
            BorderBrush         = new SolidColorBrush(Color.FromRgb(0x00, 0x78, 0xD4)),
            BorderThickness     = new Thickness(1),
            Padding             = new Thickness(0),
            FontSize            = 12,
        };
        // Reduce default Fluent-theme item height to match PropertyPageControl rows
        floatingLb.Styles.Add(new Style(s => s.OfType<ListBoxItem>())
        {
            Setters =
            {
                new Setter(ListBoxItem.PaddingProperty,   new Thickness(4, 1)),
                new Setter(ListBoxItem.MinHeightProperty, 0.0),
            }
        });
        // Item text may carry a [[ColorBox:R:G:B]] prefix (mirrors Forms.NET8
        // PropertyPage.DrawString): render it as a filled colour square followed
        // by the remaining text. Any other [[…]] prefix is silently stripped.
        floatingLb.ItemTemplate = new FuncDataTemplate<string>((item, _) =>
        {
            string text = item ?? string.Empty;
            if (text.StartsWith("[["))
            {
                string[] parts   = text.Split(["]]"], System.StringSplitOptions.None);
                string   rest    = parts[parts.Length - 1].TrimStart();
                string[] command = parts[0].Substring(2).Split(':');

                if (command[0] == "ColorBox" && command.Length == 4 &&
                    int.TryParse(command[1], out int r) &&
                    int.TryParse(command[2], out int g) &&
                    int.TryParse(command[3], out int b))
                {
                    double sz    = control.LineHeight - 4;
                    var    panel = new StackPanel { Orientation = Orientation.Horizontal };
                    panel.Children.Add(new Border
                    {
                        Width             = sz,
                        Height            = sz,
                        Background        = new SolidColorBrush(Color.FromRgb((byte)r, (byte)g, (byte)b)),
                        BorderBrush       = new SolidColorBrush(Color.FromRgb(0x40, 0x40, 0x40)),
                        BorderThickness   = new Thickness(1),
                        VerticalAlignment = VerticalAlignment.Center,
                        Margin            = new Thickness(0, 0, 4, 0),
                    });
                    panel.Children.Add(new TextBlock { Text = rest, VerticalAlignment = VerticalAlignment.Center });
                    return panel;
                }
                return new TextBlock { Text = rest, VerticalAlignment = VerticalAlignment.Center };
            }
            return new TextBlock { Text = text, VerticalAlignment = VerticalAlignment.Center };
        }, supportsRecycling: true);

        floatingLb.Tapped += (_, _) =>
        {
            if (floatingLb.SelectedIndex >= 0)
            {
                var dropEntry = _dropDownEntry;
                int  selIdx   = floatingLb.SelectedIndex;
                HideDropDown(titleId);
                dropEntry?.ListBoxSelected(selIdx);
            }
        };
        floatingLb.KeyDown += (_, ev) =>
        {
            switch (ev.Key)
            {
                case Key.Enter:
                    if (floatingLb.SelectedIndex >= 0)
                    {
                        var dropEntry = _dropDownEntry;
                        int  selIdx   = floatingLb.SelectedIndex;
                        HideDropDown(titleId);
                        dropEntry?.ListBoxSelected(selIdx);
                    }
                    ev.Handled = true;
                    break;
                case Key.Escape:
                    HideDropDown(titleId);
                    ev.Handled = true;
                    break;
            }
        };
        floatingLb.LostFocus += (_, _) => HideDropDown(titleId);

        // ── Wire PropertyPageControl dropdown callbacks ────────────────────
        control.RequestShowDropDown = (rect, items, selectedIndex, entry) =>
        {
            // Toggle: clicking the same entry closes the dropdown
            if (_dropDownEntry == entry)
            {
                HideDropDown(titleId);
                return;
            }
            // Close any existing dropdown or textbox first
            if (_dropDownTitleId != null) HideDropDown(_dropDownTitleId);
            HideTextBox(titleId, aborted: false);

            _dropDownEntry   = entry;
            _dropDownTitleId = titleId;

            // Determine position: above or below the entry (mirrors Forms.NET8 PropertiesExplorer.ShowListBox)
            // Look up the tab at call time — 'scroll' is not yet assigned at lambda-definition time
            if (!_tabs.TryGetValue(titleId, out var curTab)) return;
            double lineHeight      = curTab.Control.LineHeight;
            double scrollOffset    = curTab.Scroll.Offset.Y;
            double viewportHeight  = curTab.Scroll.Viewport.Height;
            double entryTopInVp    = rect.Top  - scrollOffset;
            double entryBottomInVp = rect.Bottom - scrollOffset;
            double totalHeight     = items.Length * lineHeight + 4;  // +4 for borders

            double top, height;
            if (entryTopInVp > viewportHeight - entryBottomInVp)
            {
                // More space above the entry
                height = Math.Min(totalHeight, scrollOffset + entryTopInVp);
                top    = rect.Top - height;
                if (top < 0) { height += top; top = 0; }
            }
            else
            {
                // More space below the entry
                height = Math.Min(totalHeight, viewportHeight - entryBottomInVp + scrollOffset);
                top    = rect.Bottom;
            }
            height = Math.Max(height, lineHeight);

            floatingLb.ItemsSource   = items;
            floatingLb.Margin        = new Thickness(rect.Left, top, 0, 0);
            floatingLb.Width         = rect.Width;
            floatingLb.Height        = height;
            floatingLb.IsVisible     = true;
            floatingLb.SelectedIndex = selectedIndex;
            floatingLb.Focus();
        };
        control.RequestHideDropDown = () => HideDropDown(titleId);

        // ── Container: control + floating TextBox + floating ListBox share coordinate space ───
        var container = new AvaloniaGrid();
        container.Children.Add(control);
        container.Children.Add(floatingTb);
        container.Children.Add(floatingLb);

        var scroll = new ScrollViewer
        {
            Content                       = container,
            HorizontalScrollBarVisibility = ScrollBarVisibility.Disabled,
            VerticalScrollBarVisibility   = ScrollBarVisibility.Auto,
        };

        // When the vertical scrollbar appears or disappears, shrink the drawing
        // area by the scrollbar's expanded width so right-edge buttons and shortcuts
        // remain accessible even when the Fluent overlay scrollbar is fully expanded.
        const double ExpandedScrollBarWidth = 17.0;
        scroll.PropertyChanged += (_, pe) =>
        {
            if (pe.Property != ScrollViewer.ExtentProperty &&
                pe.Property != ScrollViewer.ViewportProperty) return;
            double vp = scroll.Viewport.Height;
            if (vp <= 0) return;   // not yet laid out
            bool   needs   = scroll.Extent.Height > vp;
            double reserve = needs ? ExpandedScrollBarWidth : 0.0;
            if (Math.Abs(control.ScrollBarReserve - reserve) > 0.5)
                control.ScrollBarReserve = reserve;
        };

        // ── Compact tab rider ──────────────────────────────────────────────
        string header = ResolveTabHeader(titleId);

        _tabStrip.ColumnDefinitions.Add(
            new ColumnDefinition(new GridLength(1, GridUnitType.Star)));

        var tabLabel = new TextBlock
        {
            Text                = header,
            FontSize            = 10,
            TextTrimming        = TextTrimming.CharacterEllipsis,
            VerticalAlignment   = VerticalAlignment.Center,
            HorizontalAlignment = HorizontalAlignment.Center,
            Padding             = new Thickness(2, 0),
        };

        var tabBorder = new Border
        {
            Child           = tabLabel,
            Background      = TabInactiveBg,
            // Right-edge separator between riders
            BorderBrush     = StripSepBrush,
            BorderThickness = new Thickness(0, 0, 1, 0),
            Cursor          = new Cursor(StandardCursorType.Hand),
        };

        int col = _tabStrip.ColumnDefinitions.Count - 1;
        AvaloniaGrid.SetColumn(tabBorder, col);
        _tabStrip.Children.Add(tabBorder);

        tabBorder.PointerPressed += (_, ev) =>
        {
            if (ev.GetCurrentPoint(tabBorder).Properties.IsLeftButtonPressed)
                ShowPropertyPage(titleId);
        };

        _tabs[titleId] = new TabEntry(page, tabBorder, tabLabel, control, floatingTb, floatingLb, scroll);

        // Auto-select the very first tab
        if (_tabs.Count == 1)
            ShowPropertyPage(titleId);

        return page;
    }

    // ── Tab header text resolution ─────────────────────────────────────────

    private static string ResolveTabHeader(string titleId)
    {
        try
        {
            // Most XML entries only have <info> (no <label>/<tip>) → try all three.
            foreach (var cat in new[]
            {
                StringTable.Category.label,
                StringTable.Category.tip,
                StringTable.Category.info,
            })
            {
                string s = StringTable.GetString(titleId + "TabPage", cat);
                if (!s.StartsWith("missing string:")) return s;
            }
        }
        catch { /* fall back */ }
        return titleId;
    }

    // ── Floating TextBox lifecycle ─────────────────────────────────────────

    /// <summary>
    /// Reflects the entry's current value into the floating editor without treating it as a user edit,
    /// preserving the caret position and a full-text selection. Mirrors WinForms
    /// PropertiesExplorer.Refresh (textBox.Text = value; keep caret; SelectAll if it was all selected).
    /// Called from the live-preview path so a value following the mouse shows up while editing.
    /// </summary>
    private void RefreshEditorTextFromValue(TextBox tb, IPropertyEntry entry)
    {
        string value = entry.Value ?? "";
        if (tb.Text == value) return;

        int    caret  = tb.CaretIndex;
        int    len    = tb.Text?.Length ?? 0;
        bool   allSel = len > 0 && Math.Abs(tb.SelectionEnd - tb.SelectionStart) == len;

        _programmaticText = value;   // recognised as our own update in TextChanged (async → value compare)
        tb.Text = value;

        if (allSel) tb.SelectAll();
        else tb.CaretIndex = Math.Min(caret, value.Length);
    }

    /// <summary>
    /// Hides the floating TextBox and calls EndEdit on the active entry.
    /// Guarded against double-calls (LostFocus + explicit hide from pointer event).
    /// </summary>
    private void HideTextBox(string titleId, bool aborted)
    {
        if (!_tabs.TryGetValue(titleId, out var te)) return;
        var tb = te.FloatingTextBox;
        if (!tb.IsVisible) return;    // guard re-entrance

        tb.IsVisible = false;
        te.Control.SetEditing(false);   // editing finished → allow normal relayout again

        if (_editingEntry == null) return;
        var entry = _editingEntry;
        _editingEntry = null;

        // "modified" must mean the USER typed — not that the mouse-driven live preview changed the
        // text. And "aborted" must mean an explicit cancel (Escape) — NOT merely "unmodified".
        //
        // EditableProperty.EndEdit reverts to valueBeforeEdit on aborted, and both abort and commit go
        // through SetValue → (for a ConstructAction point) OnSetValue → SetFixed(true). So if we mapped
        // "unmodified" to aborted, clicking to fix a mouse-followed point would revert it to the editor's
        // open-time value AND fix the input there — swallowing the click's own fix/advance (OnMouse bails
        // on an already-fixed input). Passing aborted=false, modified=false makes EndEdit a no-op, leaving
        // the point where the mouse put it so the click itself fixes it and advances. Mirrors WinForms,
        // which forwards textBox.Modified and only aborts on Escape.
        string newValue = tb.Text ?? "";
        bool   modified = _editUserModified;
        entry.EndEdit(aborted, modified, newValue);
    }

    /// <summary>
    /// Handles Enter/Tab while the floating value TextBox has keyboard focus.
    ///
    /// Reproduces the WinForms key routing: there CadControl.ProcessCmdKey intercepts
    /// Tab/Enter *before* the focused edit control and forwards them to
    /// Frame.PreProcessKeyDown, which gives the ActiveAction the first chance
    /// (Action.OnEnter / Action.OnTab). A ConstructAction uses that to fix the current
    /// input field and advance to the next mandatory field (SetNextInputIndex), or to
    /// finish the action (OnDone) once every required field is filled.
    ///
    /// In Avalonia the floating TextBox receives the key directly, so we mirror the same
    /// ordering here: offer the key to the ActiveAction first, then reflect the resulting
    /// input-field change in the floating editor. If no action handles it we fall back to
    /// the plain property-page behaviour (commit, and on Tab move to the next entry).
    /// </summary>
    private void HandleEditingEnterOrTab(string titleId, bool isTab, KeyModifiers mods)
    {
        if (!_tabs.TryGetValue(titleId, out var te)) return;
        var page = te.Page;

        var action = (Frame as FrameImpl)?.ActiveAction;
        var before = page.GetCurrentSelection();

        var keys = ToSubstituteKeys(isTab ? 0x09 : 0x0D, mods);
        bool handled = action != null && (isTab ? action.OnTab(keys) : action.OnEnter(keys));

        if (handled)
        {
            // The action consumed the key. If it moved to another input field, commit the
            // field we were editing and open the editor of the new field. If the editing
            // entry was removed (the action finished) CommitOrHideFloatingTextBox just hides.
            if (!ReferenceEquals(before, page.GetCurrentSelection()))
            {
                CommitOrHideFloatingTextBox(titleId, te, page);
                te.Control.ShowInputForCurrentSelection();
            }
            else
            {
                CommitOrHideFloatingTextBox(titleId, te, page);
            }
        }
        else
        {
            // No ConstructAction handled it → behave like a plain property page.
            HideTextBox(titleId, aborted: false);
            if (isTab) te.Control.NavigateToNextEntry(!mods.HasFlag(KeyModifiers.Shift));
        }
    }

    /// <summary>
    /// Commits the floating TextBox via EndEdit when its entry is still present in the page,
    /// otherwise hides it silently (the entry was removed, e.g. because the action finished).
    /// </summary>
    private void CommitOrHideFloatingTextBox(string titleId, TabEntry te, PropertyPage page)
    {
        if (!te.FloatingTextBox.IsVisible) return;
        if (_editingEntry != null && page.ContainsEntry(_editingEntry))
        {
            HideTextBox(titleId, aborted: false);
        }
        else
        {
            te.FloatingTextBox.IsVisible = false;
            _editingEntry = null;
        }
    }

    private static CADability.Substitutes.Keys ToSubstituteKeys(int vk, KeyModifiers mods)
    {
        int v = vk;
        if (mods.HasFlag(KeyModifiers.Control)) v |= 0x20000; // Keys.Control
        if (mods.HasFlag(KeyModifiers.Shift))   v |= 0x10000; // Keys.Shift
        if (mods.HasFlag(KeyModifiers.Alt))     v |= 0x40000; // Keys.Alt
        return (CADability.Substitutes.Keys)v;
    }

    // ── Floating DropDown ListBox lifecycle ───────────────────────────────

    private void HideDropDown(string titleId)
    {
        if (!_tabs.TryGetValue(titleId, out var te)) return;
        var lb = te.FloatingListBox;
        if (!lb.IsVisible) return;          // guard re-entrance
        lb.IsVisible     = false;
        lb.ItemsSource   = null;
        _dropDownEntry   = null;
        _dropDownTitleId = null;
    }

    // ── IControlCenter: remaining members ─────────────────────────────────

    public IPropertyPage ActivePropertyPage
    {
        get
        {
            if (_activeTabId != null && _tabs.TryGetValue(_activeTabId, out var e))
                return e.Page;
            return null!;
        }
    }

    public IPropertyPage GetPropertyPage(string titleId)
    {
        _tabs.TryGetValue(titleId, out var e);
        return e?.Page!;
    }

    public bool ShowPropertyPage(string titleId)
    {
        if (!_tabs.TryGetValue(titleId, out var te)) return false;

        void Activate()
        {
            _activeTabId      = titleId;
            _pageArea.Content = te.Scroll;

            // Update visual state of every rider
            foreach (var (id, entry) in _tabs)
            {
                bool active = id == titleId;
                entry.TabBorder.Background = active ? TabActiveBg   : TabInactiveBg;
                entry.TabLabel.Foreground  = active ? TabActiveFg   : TabInactiveFg;
            }
        }

        if (Dispatcher.UIThread.CheckAccess()) Activate();
        else Dispatcher.UIThread.Post(Activate, DispatcherPriority.Normal);

        return true;
    }

    public bool RemovePropertyPage(string titleId)
    {
        if (!_tabs.TryGetValue(titleId, out _)) return false;

        _tabs.Remove(titleId);
        RebuildTabStrip();

        if (_activeTabId == titleId)
        {
            _activeTabId      = null;
            _pageArea.Content = null;
            if (_tabs.Count > 0)
                ShowPropertyPage(_tabs.Keys.First());
        }

        return true;
    }

    /// <summary>
    /// Reconstructs the tab strip columns after a tab is removed.
    /// Columns cannot be individually removed from a Grid, so we clear and re-add.
    /// </summary>
    private void RebuildTabStrip()
    {
        _tabStrip.Children.Clear();
        _tabStrip.ColumnDefinitions.Clear();
        int col = 0;
        foreach (var (_, te) in _tabs)
        {
            _tabStrip.ColumnDefinitions.Add(
                new ColumnDefinition(new GridLength(1, GridUnitType.Star)));
            AvaloniaGrid.SetColumn(te.TabBorder, col++);
            _tabStrip.Children.Add(te.TabBorder);
        }
    }

    public void DisplayHelp(string helpID) { }

    public void PreProcessKeyDown(KeyEventArgs ev)
    {
        if (_activeTabId == null || !_tabs.TryGetValue(_activeTabId, out var te)) return;

        bool ctrl  = (ev.KeyData & CADability.Substitutes.Keys.Control) != 0;
        bool shift = (ev.KeyData & CADability.Substitutes.Keys.Shift)   != 0;
        bool alt   = (ev.KeyData & CADability.Substitutes.Keys.Alt)     != 0;

        switch ((CADability.Substitutes.Keys)((int)ev.KeyData & 0x0FFFF))
        {
            case CADability.Substitutes.Keys.Tab:
                if (ctrl)
                {   // Ctrl+Tab: switch between tabs
                    var tabIds = _tabs.Keys.ToList();
                    int idx = tabIds.IndexOf(_activeTabId);
                    if (idx >= 0)
                    {
                        idx = shift
                            ? (idx + tabIds.Count - 1) % tabIds.Count
                            : (idx + 1) % tabIds.Count;
                        ShowPropertyPage(tabIds[idx]);
                    }
                }
                else
                {
                    if (_editingEntry != null && te.FloatingTextBox.IsVisible)
                        HideTextBox(_activeTabId, aborted: false);
                    else if (_dropDownEntry != null && te.FloatingListBox.IsVisible)
                    {
                        if (te.FloatingListBox.SelectedIndex >= 0)
                        {
                            var dropEntry = _dropDownEntry;
                            int selIdx    = te.FloatingListBox.SelectedIndex;
                            HideDropDown(_activeTabId);
                            dropEntry?.ListBoxSelected(selIdx);
                        }
                        else HideDropDown(_activeTabId);
                    }
                    te.Control.NavigateToNextEntry(!shift);
                }
                ev.Handled = true;
                ev.SuppressKeyPress = true;
                break;

            case CADability.Substitutes.Keys.Enter:
                if (_editingEntry != null && te.FloatingTextBox.IsVisible)
                    HideTextBox(_activeTabId, aborted: false);
                else if (_dropDownEntry != null && te.FloatingListBox.IsVisible)
                {
                    if (te.FloatingListBox.SelectedIndex >= 0)
                    {
                        var dropEntry = _dropDownEntry;
                        int selIdx    = te.FloatingListBox.SelectedIndex;
                        HideDropDown(_activeTabId);
                        dropEntry?.ListBoxSelected(selIdx);
                    }
                }
                else
                    te.Control.TriggerEnterKey(ctrl);
                ev.Handled = true;
                break;

            case CADability.Substitutes.Keys.Escape:
                if (_editingEntry != null && te.FloatingTextBox.IsVisible)
                {
                    HideTextBox(_activeTabId, aborted: true);
                    ev.Handled = true;
                }
                else if (_dropDownEntry != null && te.FloatingListBox.IsVisible)
                {
                    HideDropDown(_activeTabId);
                    ev.Handled = true;
                }
                else
                    ev.Handled = te.Page.OnEscape(ctrl);
                break;

            case CADability.Substitutes.Keys.Down:
                te.Control.NavigateToNextEntry(true);
                ev.Handled = true;
                break;

            case CADability.Substitutes.Keys.Up:
                te.Control.NavigateToNextEntry(false);
                ev.Handled = true;
                break;
        }

        // Forward unhandled keys to the active PropertyPageControl for shortcut matching.
        // Mirrors Forms.NET8 PropertiesExplorer: (ActivePropertyPage as PropertyPage)?.PreProcessKeyDown(e)
        if (!ev.Handled && !ev.SuppressKeyPress)
        {
            int vk = (int)ev.KeyData & 0x0FFFF;
            if (te.Control.TryHandleShortcutVk(vk, ctrl, alt, shift))
                ev.Handled = true;
        }
    }

    public void HideEntry(string entryId, bool hide) { }
    public IPropertyEntry FindItem(string name) => null!;
}
