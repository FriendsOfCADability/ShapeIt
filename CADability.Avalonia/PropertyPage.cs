using CADability.UserInterface;
using System;
using System.Collections.Generic;

namespace CADability.Avalonia;

/// <summary>
/// Avalonia port of the WinForms PropertyPage.
/// Manages root entries and a flat visible list, mirroring the WinForms
/// SetEntries / RefreshEntries / GetSubEntries logic exactly so that every
/// entry receives the correct Added / Removed lifecycle calls.
/// </summary>
public class PropertyPage : IPropertyPage
{
    // All root entries (not including sub-entries of open entries)
    private readonly List<IPropertyEntry> _rootEntries = new();

    // The currently visible flat list (root + open sub-trees), mirrors WinForms entries[]
    private IPropertyEntry[] _visibleEntries = Array.Empty<IPropertyEntry>();

    private IPropertyEntry? _selected;

    public PropertyPage(string titleId, IFrame frame)
    {
        TitleId = titleId;
        Frame = frame;
    }

    /// <summary>The ID passed to <see cref="IControlCenter.AddPropertyPage"/>.</summary>
    public string TitleId { get; }
    public IFrame Frame { get; }

    /// <summary>Fires whenever the displayed entries should be rebuilt.</summary>
    public event Action? Changed;

    /// <summary>
    /// Fires when the selected entry actually changes (not on every <see cref="Changed"/>).
    /// Mirrors the funnel role of the WinForms PropertyPage.SelectedIndex setter: it lets the
    /// PropertyPageControl auto-open the floating value editor for a value-editable entry the
    /// moment it becomes selected, so the user can start typing without clicking into the value
    /// cell first (WinForms parity).
    /// </summary>
    internal event Action? SelectionChanged;

    /// <summary>Fires when <see cref="BringToFront"/> is called.</summary>
    public event Action? BringToFrontRequested;

    // ── IPropertyPage ──────────────────────────────────────────────────────

    public void BringToFront() => BringToFrontRequested?.Invoke();

    /// <summary>
    /// Adds a root entry.  Does NOT call Added() directly — SetEntries handles that.
    /// Mirrors WinForms PropertyPage.Add exactly.
    /// </summary>
    public void Add(IPropertyEntry toAdd, bool showOpen)
    {
        _rootEntries.Add(toAdd);
        RefreshEntries();
        if (showOpen) OpenSubEntries(toAdd, true);
        Selected = toAdd;
    }

    public void Remove(IPropertyEntry toRemove)
    {
        _rootEntries.Remove(toRemove);
        RefreshEntries();
    }

    public void Clear()
    {
        _rootEntries.Clear();
        RefreshEntries();
    }

    /// <summary>
    /// Mirrors WinForms PropertyPage.Refresh: if the entry has open sub-entries,
    /// close then reopen to force a sub-entry count refresh; otherwise just repaint.
    /// </summary>
    public void Refresh(IPropertyEntry toRefresh)
    {
        int index = FindEntry(toRefresh);
        if (index >= 0
            && _visibleEntries[index].Flags.HasFlag(PropertyEntryType.HasSubEntries)
            && _visibleEntries[index].IsOpen)
        {
            // Close + reopen forces GetSubEntries to re-read the child list.
            OpenSubEntries(toRefresh, false);
            OpenSubEntries(toRefresh, true);
            return;
        }
        Changed?.Invoke();
    }

    public IPropertyEntry Selected
    {
        get => _selected!;
        set
        {
            // Only accept entries that are currently visible
            if (value != null)
            {
                bool found = false;
                for (int i = 0; i < _visibleEntries.Length; i++)
                    if (_visibleEntries[i] == value) { found = true; break; }
                if (!found) return;
            }

            if (_selected == value) return;

            var prev = _selected;
            _selected = value;
            prev?.UnSelected(value!);
            value?.Selected(prev!);
            Changed?.Invoke();
            // After the entry list has settled, notify listeners that the selection changed so
            // the control can open the value editor for the newly selected entry (WinForms parity).
            SelectionChanged?.Invoke();
        }
    }

    /// <summary>
    /// Mirrors WinForms OpenSubEntries: calls OpenOrCloseSubEntries() to toggle IsOpen
    /// (which also returns the count of items added/removed), then refreshes.
    /// </summary>
    public void OpenSubEntries(IPropertyEntry toOpenOrClose, bool open)
    {
        if (open != toOpenOrClose.IsOpen)
        {
            toOpenOrClose.OpenOrCloseSubEntries(); // toggles IsOpen, returns item delta
            RefreshEntries();
        }
        else if (toOpenOrClose.IsOpen)
        {
            // Called as a refresh — rebuild sub-tree in place.
            RefreshEntries();
        }
    }

    public IPropertyEntry GetCurrentSelection() => _selected!;

    public void SelectEntry(IPropertyEntry toSelect)
    {
        if (toSelect == null) return;
        Selected = toSelect;
    }

    public void MakeVisible(IPropertyEntry toShow) { /* TODO: scroll into view */ }

    public IPropertyEntry FindFromHelpLink(string helpResourceID, bool searchTreeAndOpen = false)
    {
        for (int i = 0; i < _visibleEntries.Length; i++)
        {
            var e = _visibleEntries[i];
            if (e.ResourceId == helpResourceID
                || (string.IsNullOrEmpty(e.ResourceId) && e.Label == helpResourceID))
                return e;
        }
        if (searchTreeAndOpen)
        {
            foreach (var root in _rootEntries)
            {
                var found = root.FindSubItem(helpResourceID);
                if (found != null) return found;
            }
        }
        return null!;
    }

    /// <summary>
    /// Fired by StartEditLabel so PropertyPageControl can position and show the floating TextBox.
    /// Parameters: entry to edit, isValue (false = label, true = value).
    /// </summary>
    internal event Action<IPropertyEntry, bool>? ShowTextBoxRequested;

    public void StartEditLabel(IPropertyEntry ToEdit)
    {
        int index = FindEntry(ToEdit);
        if (index < 0) return;
        _visibleEntries[index].StartEdit(false);
        ShowTextBoxRequested?.Invoke(_visibleEntries[index], false);
    }

    // Deprecated adapters
    public IView ActiveView => Frame?.ActiveView!;
    public IPropertyEntry GetParent(IShowProperty child)
        => (child as IPropertyEntry)?.Parent as IPropertyEntry ?? null!;
    public IFrame GetFrame() => Frame;
    public bool IsOpen(IShowProperty toTest) => toTest is IPropertyEntry pe && pe.IsOpen;
    public bool IsOnTop() => true;

    public bool ContainsEntry(IPropertyEntry entryWithTextBox)
    {
        for (int i = 0; i < _visibleEntries.Length; i++)
            if (_visibleEntries[i] == entryWithTextBox) return true;
        return false;
    }

    public event PreProcessKeyDown OnPreProcessKeyDown { add { } remove { } }
    public event SelectionChanged OnSelectionChanged { add { } remove { } }

    internal void SelectNextPropertyEntry(bool forward)
    {
        int selected = FindEntry(_selected);
        if (selected < 0) selected = 0;

        if (forward)
        {
            for (int i = selected + 1; i < _visibleEntries.Length; i++)
            {
                if (_visibleEntries[i].Flags.HasFlag(PropertyEntryType.Selectable))
                { Selected = _visibleEntries[i]; return; }
            }
            for (int i = 0; i < selected; i++)
            {
                if (_visibleEntries[i].Flags.HasFlag(PropertyEntryType.Selectable))
                { Selected = _visibleEntries[i]; return; }
            }
        }
        else
        {
            for (int i = selected - 1; i >= 0; --i)
            {
                if (_visibleEntries[i].Flags.HasFlag(PropertyEntryType.Selectable))
                { Selected = _visibleEntries[i]; return; }
            }
            for (int i = _visibleEntries.Length - 1; i > selected; --i)
            {
                if (_visibleEntries[i].Flags.HasFlag(PropertyEntryType.Selectable))
                { Selected = _visibleEntries[i]; return; }
            }
        }
    }

    internal bool OnEscape(bool ctrl) => false;

    // ── Visible list (used by PropertiesExplorer to build rows) ───────────

    /// <summary>Returns the flat, ordered list of entries currently visible.</summary>
    public IReadOnlyList<IPropertyEntry> FlattenVisible() => _visibleEntries;

    // ── Core WinForms logic, ported 1-to-1 ────────────────────────────────

    /// <summary>
    /// Rebuilds the flat visible list from root entries + open sub-trees,
    /// then calls SetEntries to diff and fire Added/Removed notifications.
    /// Mirrors WinForms PropertyPage.RefreshEntries.
    /// </summary>
    private void RefreshEntries()
    {
        var list = new List<IPropertyEntry>();
        for (int i = 0; i < _rootEntries.Count; i++)
        {
            _rootEntries[i].IndentLevel = 0;
            list.Add(_rootEntries[i]);
            list.AddRange(GetSubEntries(_rootEntries[i], 0));
        }
        SetEntries(list.ToArray());
    }

    /// <summary>
    /// Recursively collects open sub-entries, setting IndentLevel.
    /// Mirrors WinForms PropertyPage.GetSubEntries.
    /// </summary>
    private static List<IPropertyEntry> GetSubEntries(IPropertyEntry pe, int level)
    {
        var res = new List<IPropertyEntry>();
        if (pe.IsOpen
            && pe.Flags.HasFlag(PropertyEntryType.HasSubEntries)
            && pe.SubItems != null)
        {
            for (int i = 0; i < pe.SubItems.Length; i++)
            {
                if (pe.SubItems[i] == null) continue;
                pe.SubItems[i].IndentLevel = level + 1;
                res.Add(pe.SubItems[i]);
                res.AddRange(GetSubEntries(pe.SubItems[i], level + 1));
            }
        }
        return res;
    }

    /// <summary>
    /// Diffs old vs. new visible entries:
    ///  - calls Removed on entries no longer visible
    ///  - calls Added + sets Parent on newly visible entries
    ///  - always updates Index on every visible entry
    /// Mirrors WinForms PropertyPage.SetEntries.
    /// </summary>
    private void SetEntries(IPropertyEntry[] newEntries)
    {
        var oldSet = new HashSet<IPropertyEntry>(_visibleEntries);
        var newSet = new HashSet<IPropertyEntry>(newEntries);

        // Removed: in old but not in new
        for (int i = 0; i < _visibleEntries.Length; i++)
            if (!newSet.Contains(_visibleEntries[i]))
                _visibleEntries[i].Removed(this);

        _visibleEntries = newEntries;

        // Added: in new but not in old; update Index for all
        for (int i = 0; i < _visibleEntries.Length; i++)
        {
            _visibleEntries[i].Index = i;
            if (!oldSet.Contains(_visibleEntries[i]))
            {
                _visibleEntries[i].Added(this);
                _visibleEntries[i].Parent = this;
            }
        }

        Changed?.Invoke();
    }

    private int FindEntry(IPropertyEntry toFind)
    {
        for (int i = 0; i < _visibleEntries.Length; i++)
            if (_visibleEntries[i] == toFind) return i;
        return -1;
    }
}
