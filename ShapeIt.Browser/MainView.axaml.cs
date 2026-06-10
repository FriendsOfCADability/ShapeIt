using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Xml;
using Avalonia.Controls;
using Avalonia.Threading;
using Avalonia.VisualTree;
using CADability;
using CADability.Attribute;
using CADability.Avalonia;
using CADability.GeoObject;
using CADability.UserInterface;

namespace ShapeIt.Browser
{
    public partial class MainView : UserControl
    {
        private CadFrame? _frame;
        private ToolBars? _toolBars;
        private DispatcherTimer? _toolbarTimer;
        private global::ShapeIt.ModellingPropertyEntries? _modellingPropertyEntries;
        private BrowserFileService? _fileService;
        private readonly System.Collections.Generic.List<string> _touchTrace = new();
        private string TouchTraceText() => _touchTrace.Count == 0 ? "(noch nichts)" : string.Join("\n", _touchTrace);

        public MainView()
        {
            InitializeComponent();
            Dispatcher.UIThread.Post(Initialize, DispatcherPriority.Loaded);
        }

        private void Initialize()
        {
            try
            {
                _frame = new CadFrame(propertiesExplorer, viewport, null);
                viewport.Frame = _frame;

                // The global Select.PickRadius default (5) is kept so MOUSE picking stays precise;
                // touch taps widen their pick radius locally in Gl3DViewport.EmitTap instead.

                var assembly = Assembly.GetExecutingAssembly();

                // 1. String tables (menu/command labels).
                LoadStringTable(assembly, "ShapeIt.StringTableDeutsch.xml");
                LoadStringTable(assembly, "ShapeIt.StringTableEnglish.xml");

                // 2. Menu from MenuResource.xml.
                LoadMenuResource(assembly);

                // 3. Standard toolbars + idle-style update timer (mirrors MainWindow).
                _toolBars = new ToolBars(_frame);
                _toolBars.AddStandardToolBars(toolbarPanel);
                _toolbarTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(250) };
                _toolbarTimer.Tick += (_, _) =>
                {
                    _toolBars!.UpdateCommandState();
                    // Push the project's dirty state to the JS beforeunload guard (feature 3).
                    // Polled here (no IsModified event on IFrame) so a reload/close warns the
                    // user about unsaved changes — browser analogue of MainWindow.OnClosing's
                    // SaveModified prompt.
                    UpdateUnsavedGuard();
                };
                _toolbarTimer.Start();

                // 3b. Progress feedback (mirrors MainWindow.Initialize step 4). Long operations
                //     (e.g. STEP import) call ProgressAction; we show/update a centered overlay and
                //     force a render pass so the bar paints before the calling thread continues.
                _frame.ProgressAction = (show, percent, title) =>
                {
                    progressPanel.IsVisible = show;
                    if (show)
                    {
                        if (title != null) progressLabel.Text = title;
                        progressBar.Value = percent;
                    }
                    Dispatcher.UIThread.RunJobs(DispatcherPriority.Render);
                };

                // 3c. Document title + project lifecycle (mirrors MainWindow.Initialize step 3).
                //     The browser tab title is set via [JSImport] setTitle(string).
                _frame.FileNameChangedEvent += name => SetDocumentTitle(name);
                _frame.ProjectOpenedEvent += (project, _) => SetDocumentTitle(project.FileName);
                _frame.ProjectClosedEvent += (_, _) => SetDocumentTitle(null);


                // 4. New, empty project (sets ActiveView). Start in an isometric view.
                _frame.GenerateNewProject();
                _frame.ControlCenter.RemovePropertyPage("View");

                var view = _frame.ActiveView;
                if (view != null)
                {
                    var model = _frame.Project.GetActiveModel();
                    view.Projection.SetDirection(new GeoVector(-1, -1, -1), new GeoVector(0, 0, 1), model.Extent);
                    view.ZoomTotal(1.2);
                }
                viewport.Invalidate();

                // 8. Wire up the ShapeIt modelling property page (mirrors MainWindow.Initialize step 8).
                //    ModellingPropertyEntries lives in namespace ShapeIt; fully-qualify to avoid
                //    ambiguity with the ShapeIt.Browser namespace this file is in.
                var modellingPage = _frame.ControlCenter.AddPropertyPage("Modelling", 6);
                _modellingPropertyEntries = new global::ShapeIt.ModellingPropertyEntries(_frame);
                modellingPage.Add(_modellingPropertyEntries, false);
                _frame.ControlCenter.ShowPropertyPage("Modelling");

                // Let the touch viewport know when the modelling pick already selected something
                // (in modelling mode the action's SelectedObjects list stays empty), so the tap
                // fallback doesn't clobber a good modelling pick / extrude entry.
                viewport.ModellingHasSelection = () => _modellingPropertyEntries?.HasModellingSelection ?? false;

                // Touch taps AND stationary mouse clicks go through the edge-priority modelling
                // pick (the standard pick's depth filter made thin edges nearly unselectable).
                // The touch-bar toggles extend it: Multi = add/remove like Ctrl+click,
                // Körper = select the whole object instead of face/edge.
                viewport.ModellingPick = (pt, radius, fromTouch) =>
                {
                    var view = _frame?.ActiveView;
                    if (view == null || _modellingPropertyEntries == null) return false;
                    bool multi = fromTouch && touchMultiButton.IsChecked == true;
                    bool whole = touchSolidButton.IsChecked == true;
                    return _modellingPropertyEntries.TouchPick(pt, view, radius, multi, whole);
                };

                // Route main-menu / toolbar commands to ModellingPropertyEntries first (mirrors
                // MainForm.OnCommand in the WinForms variant; the frame is not subclassed here, so we
                // hook the frame's command events instead). The desktop-only "Exit" handling is dropped
                // since there is no window to close in the browser single-view lifetime.
                // Browser File/Import/Export I/O. The desktop flow (ShowOpenFileDlg ->
                // ReadFromFile(path)) cannot work in WASM because the picker's LocalPath is not a
                // real filesystem path. Intercept the file menu commands here and run stream-based
                // I/O via BrowserFileService instead.
                //
                // IMPORTANT: the picker calls MUST run synchronously within the click's call stack
                // (NOT via Dispatcher.Post) — the browser file picker (showSaveFilePicker/
                // showOpenFilePicker) requires an active user gesture; deferring it loses the
                // gesture and the browser rejects the picker (SecurityError). The async methods are
                // started directly (fire-and-forget); the picker call runs before the first await,
                // so the gesture is preserved. Exceptions are handled inside the service methods.
                _fileService = new BrowserFileService(_frame, this);

                // Export format picker overlay (replaces the desktop save-format dialog). Wired once.
                exportStlButton.Click += (_, _) => { exportOverlay.IsVisible = false; _ = _fileService!.ExportAsync("stl"); };
                exportDxfButton.Click += (_, _) => { exportOverlay.IsVisible = false; _ = _fileService!.ExportAsync("dxf"); };
                exportStepButton.Click += (_, _) => { exportOverlay.IsVisible = false; _ = _fileService!.ExportAsync("stp"); };
                exportCancelButton.Click += (_, _) => exportOverlay.IsVisible = false;

                // Touch (iPad & Co.): Gl3DViewport translates touch gestures into the same mouse
                // events the desktop uses (1 finger = rotate, 2 finger = pan, pinch = zoom,
                // tap = select, long press = context menu). Mouse and pen input are untouched.
                // The helper bar (zoom/fit/help) appears on the first touch contact only, so
                // mouse-only users never see it.
                viewport.TouchDetected += () => touchBar.IsVisible = true;
                touchZoomInButton.Click += (_, _) => viewport.TouchZoom(zoomIn: true);
                touchZoomOutButton.Click += (_, _) => viewport.TouchZoom(zoomIn: false);
                touchZoomAllButton.Click += (_, _) => viewport.TouchZoomAll();
                touchHelpButton.Click += (_, _) =>
                {
                    touchDebugText.Text = TouchTraceText();
                    touchHelpOverlay.IsVisible = true;
                };
                touchHelpCloseButton.Click += (_, _) => touchHelpOverlay.IsVisible = false;
                viewport.TouchTrace += line =>
                {
                    _touchTrace.Add(line);
                    if (_touchTrace.Count > 14) _touchTrace.RemoveAt(0);
                    if (touchHelpOverlay.IsVisible) touchDebugText.Text = TouchTraceText();
                };

                _frame.ProcessCommandEvent += (string menuId, ref bool processed) =>
                {
                    switch (menuId)
                    {
                        case "MenuId.File.New":
                            _fileService.NewProject();
                            processed = true;
                            return;
                        case "MenuId.File.Open":
                            _ = _fileService.OpenAsync();
                            processed = true;
                            return;
                        case "MenuId.File.Save":
                            _ = _fileService.SaveAsync(false);
                            processed = true;
                            return;
                        case "MenuId.File.Save.As":
                            _ = _fileService.SaveAsync(true);
                            processed = true;
                            return;
                        case "MenuId.Import":
                            _ = _fileService.ImportAsync();
                            processed = true;
                            return;
                        case "MenuId.Export":
                            ShowExportOverlay();
                            processed = true;
                            return;
                    }
                    if (_modellingPropertyEntries.OnCommand(menuId)) { processed = true; return; }
                };
                _frame.UpdateCommandEvent += (string menuId, CommandState state, ref bool processed) =>
                {
                    if (_modellingPropertyEntries.OnUpdateCommand(menuId, state)) processed = true;
                };
            }
            catch (Exception ex)
            {
                Console.WriteLine("ShapeIt.Browser MainView init failed: " + ex);
            }
        }

        // ── string / menu helpers (ported from MainWindow) ─────────────────
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
            MenuResource.SetMenuResource(doc);
            MenuWithHandler[] menuDef = MenuResource.LoadMenuDefinition("SDI Menu", true, _frame!);
            BuildMenu(mainMenu, menuDef);

            // Append a "Help" top-level menu (Avalonia-only; mirrors MainWindow.AddHelpMenu).
            // Not part of the shared MenuResource.xml, which the WinForms variant has no handler for.
            AddHelpMenu();
        }

        // ── Help menu + About overlay (feature 4) ──────────────────────────
        // The browser single-view lifetime has no Window, so the desktop AboutWindow.ShowDialog
        // is replaced by a dimmed in-canvas overlay (aboutOverlay) layered over the app chrome.
        private void AddHelpMenu()
        {
            bool de = StringTable.ActiveLanguage?
                .StartsWith("deutsch", StringComparison.OrdinalIgnoreCase) ?? false;

            var aboutItem = new MenuItem { Header = de ? "Über ShapeIt…" : "About ShapeIt…" };
            aboutItem.Click += (_, _) => ShowAbout();

            var helpMenu = new MenuItem { Header = de ? "Hilfe" : "Help" };
            helpMenu.Items.Add(aboutItem);
            mainMenu.Items.Add(helpMenu);

            // Close button on the About overlay.
            aboutCloseButton.Click += (_, _) => aboutOverlay.IsVisible = false;
        }

        // Shows the export format picker overlay. The chosen format button triggers the
        // actual export + download (see the button wiring in Initialize and
        // BrowserFileService.ExportAsync). Localizes the title to match the UI language.
        private void ShowExportOverlay()
        {
            if (_frame?.Project == null) return;
            bool de = StringTable.ActiveLanguage?
                .StartsWith("deutsch", StringComparison.OrdinalIgnoreCase) ?? false;
            exportTitle.Text = de ? "Exportieren als…" : "Export as…";
            exportCancelButton.Content = de ? "Abbrechen" : "Cancel";
            exportOverlay.IsVisible = true;
        }

        private void ShowAbout()
        {
            bool de = StringTable.ActiveLanguage?
                .StartsWith("deutsch", StringComparison.OrdinalIgnoreCase) ?? false;

            aboutVersionText.Text = "Version " + ReadVersion();
            aboutBasedOnText.Text = de
                ? "ShapeIt ist ein 3D-CAD-Programm, das auf der quelloffenen Bibliothek "
                  + "CADability (MIT-Lizenz, © SOFA GmbH) aufbaut. Diese Web-Version läuft "
                  + "vollständig im Browser (WebAssembly)."
                : "ShapeIt is a 3D CAD application built on top of the open-source library "
                  + "CADability (MIT License, © SOFA GmbH). This web version runs entirely in "
                  + "the browser (WebAssembly).";
            aboutOverlay.IsVisible = true;
        }

        // Prefer the embedded "App.Version" (version.txt); fall back to the assembly version
        // (mirrors AboutWindow.ReadVersion in the desktop head).
        private static string ReadVersion()
        {
            try
            {
                var asm = Assembly.GetExecutingAssembly();
                using Stream? s = asm.GetManifestResourceStream("App.Version");
                if (s != null)
                {
                    using var sr = new StreamReader(s);
                    string v = sr.ReadToEnd().Trim();
                    if (!string.IsNullOrEmpty(v)) return v;
                }
            }
            catch { /* fall through */ }

            return Assembly.GetExecutingAssembly().GetName().Version?.ToString() ?? "?";
        }

        // ── Browser-lifecycle bridge helpers (features 2 + 3) ──────────────
        private void SetDocumentTitle(string? fileName)
        {
            string title = string.IsNullOrEmpty(fileName) ? "ShapeIt" : $"ShapeIt — {fileName}";
            try { BrowserHostInterop.SetTitle(title); }
            catch (Exception ex) { Console.WriteLine("setTitle failed: " + ex); }
        }

        private bool _lastUnsavedGuard;
        private void UpdateUnsavedGuard()
        {
            bool unsaved = _frame?.Project?.IsModified ?? false;
            if (unsaved == _lastUnsavedGuard) return; // only call JS on change
            _lastUnsavedGuard = unsaved;
            try { BrowserHostInterop.SetUnsavedGuard(unsaved); }
            catch (Exception ex) { Console.WriteLine("setUnsavedGuard failed: " + ex); }
        }

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

        private void AddSubItems(MenuItem parent, MenuWithHandler[] items)
        {
            foreach (var mwh in items)
            {
                if (mwh.ID == "SEPARATOR") { parent.Items.Add(new Separator()); continue; }

                var item = new TriangleMenuItem { Header = mwh.Text ?? mwh.ID, Tag = mwh };

                var icon = MenuManager.LoadMenuIcon(mwh.ID);
                if (icon != null)
                    item.Icon = new Image { Source = icon, Width = 16, Height = 16 };

                if (mwh.SubMenus is { Length: > 0 })
                {
                    item.SubmenuOpened += (_, _) => UpdateMenuItems(item);
                    AddSubItems(item, mwh.SubMenus);
                }
                else
                {
                    item.Click += (_, _) =>
                    {
                        if (item.Tag is MenuWithHandler h) h.Target?.OnCommand(h.ID);
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
                _frame?.OnUpdateCommand(mwh.ID, state);
                item.IsEnabled = state.Enabled;
            }
        }
    }
}
