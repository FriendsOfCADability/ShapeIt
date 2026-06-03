using Avalonia.Controls;
using Avalonia.Threading;
using CADability;
using CADability.Avalonia;
using CADability.UserInterface;
using System;
using System.IO;
using System.Reflection;
using System.Xml;

namespace ShapeIt;

public partial class MainWindow : Window, ICommandHandler
{
    private CadFrame? cadFrame;
    private ModellingPropertyEntries? modellingPropertyEntries;
    private DateTime lastSaved;
    private ToolBars? toolBars;
    private DispatcherTimer? toolbarUpdateTimer;
    // Checkable "View → Toolbars" entries, paired with their toolbar name.
    private readonly System.Collections.Generic.List<(string name, MenuItem item)> toolbarCheckItems = new();
    // Checkable "View → Toolbars → Large icons" entry.
    private MenuItem? largeIconsItem;

    public MainWindow()
    {
        InitializeComponent();
        // Defer initialisation until the visual tree (including CadControl) is fully
        // loaded so that the OpenGL surface and all named controls are accessible.
        Dispatcher.UIThread.Post(Initialize, DispatcherPriority.Loaded);
    }

    // ── Initialisation (mirrors MainForm constructor) ──────────────────────

    private void Initialize()
    {
        // CadControl (created by InitializeComponent) already owns CadCanvas,
        // PropertiesExplorer, and a fully wired CadFrame.
        cadFrame = cadControl.CadFrame;

        var assembly = Assembly.GetExecutingAssembly();

        // 1. Load ShapeIt string tables so menu labels are correct from the start.
        LoadStringTable(assembly, "ShapeIt.StringTableDeutsch.xml");
        LoadStringTable(assembly, "ShapeIt.StringTableEnglish.xml");

        // 2. Load the ShapeIt menu resource and build the Avalonia top-level menu.
        LoadMenuResource(assembly);

        // 2a. Append a "Help" top-level menu (Avalonia-only; not part of the shared
        //     MenuResource.xml, which the WinForms variant has no handler for).
        AddHelpMenu();

        // 2b. Build the standard toolbars (same set as CADability.Forms.NET8) and
        //     drive their Enabled/Checked state from an idle timer (mirrors Application.Idle).
        toolBars = new ToolBars(cadFrame);
        toolBars.AddStandardToolBars(toolbarPanel);
        toolbarUpdateTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(250) };
        toolbarUpdateTimer.Tick += (_, _) => toolBars.UpdateCommandState();
        toolbarUpdateTimer.Start();

        // 2c. Populate the "View → Toolbars" submenu with a visibility checkbox per toolbar,
        //     and keep its checks in sync when toolbars are reordered/hidden via the grips.
        AddToolbarsMenu();
        toolBars.LayoutChanged += RefreshToolbarChecks;

        // 3. Subscribe to frame lifecycle events.
        cadFrame.FileNameChangedEvent += name =>
            Title = string.IsNullOrEmpty(name) ? "ShapeIt" : $"ShapeIt — {name}";
        cadFrame.ProjectClosedEvent += OnProjectClosed;
        cadFrame.ProjectOpenedEvent += OnProjectOpened;

        // 4. Wire up the progress bar so long operations (e.g. STEP import) show feedback.
        cadFrame.ProgressAction = (show, percent, title) =>
        {
            progressPanel.IsVisible = show;
            if (show)
            {
                if (title != null) progressLabel.Text = title;
                progressBar.Value = percent;
            }
            // Force an immediate render pass so the bar is visible before the
            // calling thread continues — without this the UI only updates after
            // the long operation finishes and the bar is never seen.
            Dispatcher.UIThread.RunJobs(DispatcherPriority.Render);
        };

        // 5. Create a new project — this sets cadFrame.ActiveView, which is required
        //    by ModellingPropertyEntries (feedback.Attach calls view.Canvas.Frame…).
        cadFrame.GenerateNewProject();
        lastSaved = DateTime.Now;

        // 6. Remove the generic "View" tab added by FrameImpl; ShapeIt manages views.
        cadFrame.ControlCenter.RemovePropertyPage("View");

        // 7. Best-effort crash saver. AppDomain.UnhandledException covers background-thread
        //    crashes (cross-platform), but UI-thread exceptions are routed through Avalonia's
        //    dispatcher and never reach it — so we also hook Dispatcher.UIThread.UnhandledException.
        //    We do NOT set e.Handled there: we only save, then let the app crash, because
        //    continuing after an unhandled exception would run on corrupted state.
        AppDomain.CurrentDomain.UnhandledException += (_, _) => SaveCrashBackup();
        Dispatcher.UIThread.UnhandledException += (_, _) => SaveCrashBackup();

        // 8. Wire up the ShapeIt modelling property page.
        //    ActiveView is guaranteed non-null after step 5.
        IPropertyPage modellingPage = cadFrame.ControlCenter.AddPropertyPage("Modelling", 6);
        modellingPropertyEntries = new ModellingPropertyEntries(cadFrame);
        modellingPage.Add(modellingPropertyEntries, false);
        cadFrame.ControlCenter.ShowPropertyPage("Modelling");

        // Route main-menu / toolbar commands to ModellingPropertyEntries first — this is what
        // MainForm.OnCommand does by override in the WinForms variant. The Avalonia app does not
        // subclass the frame, so we hook the frame's command events instead. (Context menus are
        // already handled: ModellingPropertyEntries subscribes to ProcessContextMenuEvent itself.)
        cadFrame.ProcessCommandEvent += (string menuId, ref bool processed) =>
        {
            // Forward to ModellingPropertyEntries first (mirrors MainForm.OnCommand).
            if (modellingPropertyEntries.OnCommand(menuId)) { processed = true; return; }
            // "Exit" cannot be handled by CADability.dll. Closing the main window triggers
            // OnClosing → SaveModified (save-on-exit prompt), like Application.Exit() in WinForms.
            if (menuId == "MenuId.App.Exit") { processed = true; Close(); }
        };
        cadFrame.UpdateCommandEvent += (string menuId, CommandState state, ref bool processed) =>
        {
            if (modellingPropertyEntries.OnUpdateCommand(menuId, state)) processed = true;
        };

        // 9. After a previous crash, offer to restore the auto-saved project (mirrors
        //    MainForm.OnShown in the WinForms variant).
        CheckForCrashRestore();

        // 10. Leichtgewichtige Update-Pruefung (nur Windows; nicht blockierend, best-effort).
        //     Liest latest.txt vom Server und bietet bei neuer Version die Download-Seite an.
        UpdateChecker.CheckInBackground(cadFrame);

        // 11. Aufbau abgeschlossen: Start-Logo ausblenden, damit Menü, Toolbars und
        //     PropertiesExplorer sichtbar werden.
        splashOverlay.IsVisible = false;
    }

    // ── String / menu helpers ──────────────────────────────────────────────

    private static void LoadStringTable(Assembly assembly, string resourceName)
    {
        using Stream? stream = assembly.GetManifestResourceStream(resourceName);
        if (stream == null) return;
        var doc = new XmlDocument();
        doc.Load(stream);
        StringTable.AddStrings(doc);
    }

    private void LoadMenuResource(Assembly assembly)
    {
        using Stream? stream = assembly.GetManifestResourceStream("ShapeIt.MenuResource.xml");
        if (stream == null) return;

        var doc = new XmlDocument();
        doc.Load(stream);

#if DEBUG
        // Inject an extra "Debug" menu item into the Extras popup (mirrors MainForm).
        XmlNode? extrasPopup = doc.SelectSingleNode("Menus/MainMenu/Popup[@MenuId='MenuId.Extras']");
        if (extrasPopup != null)
        {
            XmlElement debugItem = doc.CreateElement("MenuItem");
            debugItem.SetAttribute("MenuId", "MenuId.Debug");
            extrasPopup.AppendChild(debugItem);
        }
#endif

        MenuResource.SetMenuResource(doc);
        MenuWithHandler[] menuDef = MenuResource.LoadMenuDefinition("SDI Menu", true, cadFrame!);
        BuildMenu(mainMenu, menuDef);
    }

    // ── Avalonia menu builder ──────────────────────────────────────────────

    private void BuildMenu(Menu menu, MenuWithHandler[] items)
    {
        menu.Items.Clear();
        foreach (var mwh in items)
        {
            if (mwh.SubMenus is { Length: > 0 })
            {
                var topItem = new MenuItem { Header = mwh.Text, Tag = mwh };
                topItem.SubmenuOpened += (_, _) => UpdateMenuItems(topItem);
                AddSubItems(topItem, mwh.SubMenus);
                menu.Items.Add(topItem);
            }
        }
    }

    // ── Help menu ───────────────────────────────────────────────────────────

    private void AddHelpMenu()
    {
        bool de = StringTable.ActiveLanguage?
            .StartsWith("deutsch", StringComparison.OrdinalIgnoreCase) ?? false;

        var aboutItem = new MenuItem { Header = de ? "Über ShapeIt…" : "About ShapeIt…" };
        aboutItem.Click += (_, _) => new AboutWindow().ShowDialog(this);

        var helpMenu = new MenuItem { Header = de ? "Hilfe" : "Help" };
        helpMenu.Items.Add(aboutItem);
        mainMenu.Items.Add(helpMenu);
    }

    // ── View → Toolbars visibility submenu ─────────────────────────────────

    /// <summary>Appends a checkable entry per toolbar to the existing "View → Toolbars" submenu.</summary>
    private void AddToolbarsMenu()
    {
        if (toolBars == null) return;
        var toolbarsMenu = FindMenuItem(mainMenu, "MenuId.View.Toolbars");
        if (toolbarsMenu == null) return;

        if (toolbarsMenu.Items.Count > 0)
            toolbarsMenu.Items.Add(new Separator());

        foreach (var name in toolBars.ToolBarNames)
        {
            var local = name;
            var item = new MenuItem
            {
                Header = ToolBars.DisplayName(local),
                ToggleType = MenuItemToggleType.CheckBox,
                IsChecked = toolBars.IsVisible(local)
            };
            item.Click += (_, _) => toolBars.SetVisible(local, !toolBars.IsVisible(local));
            toolbarsMenu.Items.Add(item);
            toolbarCheckItems.Add((local, item));
        }

        // Toggle between the compact and the larger (~1.4×) icon variant.
        toolbarsMenu.Items.Add(new Separator());
        largeIconsItem = new MenuItem
        {
            Header = StringTable.GetString("Toolbar.LargeIcons"),
            ToggleType = MenuItemToggleType.CheckBox,
            IsChecked = toolBars.LargeIcons
        };
        largeIconsItem.Click += (_, _) => toolBars.LargeIcons = !toolBars.LargeIcons;
        toolbarsMenu.Items.Add(largeIconsItem);

        // Re-sync checks each time the submenu opens (covers grip-driven changes).
        toolbarsMenu.SubmenuOpened += (_, _) => RefreshToolbarChecks();
    }

    private void RefreshToolbarChecks()
    {
        if (toolBars == null) return;
        foreach (var (name, item) in toolbarCheckItems)
            item.IsChecked = toolBars.IsVisible(name);
        if (largeIconsItem != null) largeIconsItem.IsChecked = toolBars.LargeIcons;
    }

    /// <summary>Depth-first search for a MenuItem whose Tag is a MenuWithHandler with the given ID.</summary>
    private static MenuItem? FindMenuItem(Menu menu, string id)
    {
        foreach (var obj in menu.Items)
            if (obj is MenuItem mi)
            {
                var found = FindMenuItem(mi, id);
                if (found != null) return found;
            }
        return null;
    }

    private static MenuItem? FindMenuItem(MenuItem item, string id)
    {
        if (item.Tag is MenuWithHandler mwh && mwh.ID == id) return item;
        foreach (var obj in item.Items)
            if (obj is MenuItem child)
            {
                var found = FindMenuItem(child, id);
                if (found != null) return found;
            }
        return null;
    }

    private void AddSubItems(MenuItem parent, MenuWithHandler[] items)
    {
        foreach (var mwh in items)
        {
            if (mwh.ID == "SEPARATOR")
            {
                parent.Items.Add(new Separator());
                continue;
            }

            var item = new MenuItem { Header = mwh.Text ?? mwh.ID, Tag = mwh };

            if (mwh.SubMenus is { Length: > 0 })
            {
                item.SubmenuOpened += (_, _) => UpdateMenuItems(item);
                AddSubItems(item, mwh.SubMenus);
            }
            else
            {
                item.Click += (_, _) =>
                {
                    if (item.Tag is MenuWithHandler h)
                        h.Target?.OnCommand(h.ID);
                };
            }

            parent.Items.Add(item);
        }
    }

    private void UpdateMenuItems(MenuItem parent)
    {
        foreach (var child in parent.Items)
        {
            if (child is not MenuItem item) continue;
            if (item.Tag is not MenuWithHandler mwh) continue;

            var state = new CommandState();
            cadFrame?.OnUpdateCommand(mwh.ID, state);
            item.IsEnabled = state.Enabled;
        }
    }

    // ── Frame event handlers ───────────────────────────────────────────────

    private void OnProjectClosed(Project project, IFrame frame)
        => Title = "ShapeIt";

    private void OnProjectOpened(Project project, IFrame frame)
    {
        lastSaved = DateTime.Now;
        Title = string.IsNullOrEmpty(project.FileName)
            ? "ShapeIt"
            : $"ShapeIt — {project.FileName}";
    }

    private static string CrashDir => Path.Combine(Path.GetTempPath(), "ShapeIt");
    private static string CrashMarker => Path.Combine(CrashDir, "Crash.txt");

    // Writes the current project to a temp file and drops a marker so the next start can offer
    // to restore it. Cross-platform: all paths are built with Path.Combine (no literal '\').
    // May be invoked from any thread, so it must not touch UI; best-effort, never throws.
    private void SaveCrashBackup()
    {
        if (cadFrame?.Project == null) return;
        try
        {
            Directory.CreateDirectory(CrashDir);

            string originalName = cadFrame.Project.FileName;
            string fn;
            if (string.IsNullOrEmpty(originalName))
            {
                fn = "crash_" + DateTime.Now.ToString("yyMMddHHmm") + ".cdb.json";
                originalName = "unknown";
            }
            else
            {
                string baseName = Path.GetFileNameWithoutExtension(
                    Path.GetFileNameWithoutExtension(originalName)); // strip .cdb.json
                fn = baseName + "_X.cdb.json";
            }

            string backupPath = Path.Combine(CrashDir, fn);
            cadFrame.Project.WriteToFile(backupPath);
            File.WriteAllText(CrashMarker, originalName + "\n" + backupPath);
        }
        catch { /* best-effort */ }
    }

    // On startup, if a crash marker exists, ask the user whether to restore the auto-saved
    // project (mirrors MainForm.OnShown). The marker is always deleted afterwards.
    private void CheckForCrashRestore()
    {
        if (cadFrame == null || !File.Exists(CrashMarker)) return;
        try
        {
            string[] lines = File.ReadAllLines(CrashMarker);
            if (lines.Length == 2 && File.Exists(lines[1]))
            {
                string ask = StringTable.GetFormattedString("ShapeIt.RestoreAfterCrash", lines[0]);
                if (cadFrame.UIService.ShowMessageBox(ask, "ShapeIt",
                        CADability.Substitutes.MessageBoxButtons.YesNo) == CADability.Substitutes.DialogResult.Yes)
                {
                    cadFrame.Project = Project.ReadFromFile(lines[1]);
                    // line[0] is "unknown" when the crashed project had no file yet — leave the
                    // restored project unnamed in that case so a later save prompts for a path.
                    if (lines[0] != "unknown")
                    {
                        cadFrame.Project.FileName = lines[0];
                        Title = "ShapeIt — " + lines[0];
                    }
                }
            }
            File.Delete(CrashMarker);
        }
        catch { /* best-effort */ }
    }

    // ── Shutdown ───────────────────────────────────────────────────────────

    protected override void OnClosing(WindowClosingEventArgs e)
    {
        // Give the user a chance to save the modified project (mirrors MainForm.OnFormClosing
        // in the WinForms variant). SaveModified returns false only when the user pressed Cancel.
        if (cadFrame?.Project != null && !cadFrame.Project.SaveModified())
        {
            e.Cancel = true;
            base.OnClosing(e);
            return;
        }

        // Persist toolbar layout (order + visibility) to CADability.GlobalSettings.json.
        try { Settings.SaveGlobalSettings(); } catch { /* best-effort */ }
        base.OnClosing(e);
    }

    // ── ICommandHandler (fallback; real routing goes via cadFrame) ─────────

    bool ICommandHandler.OnCommand(string MenuId) => false;
    bool ICommandHandler.OnUpdateCommand(string MenuId, CommandState CommandState) => false;
    void ICommandHandler.OnSelected(MenuWithHandler selectedMenuItem, bool selected) { }
}
