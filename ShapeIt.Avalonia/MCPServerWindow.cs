using Avalonia.Controls;
using Avalonia.Controls.Primitives;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Threading;
using CADability;
using CADability.Avalonia;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;

namespace ShapeIt
{
    /// <summary>
    /// Avalonia-Entsprechung von <c>MCPServerForm</c> (WinForms). Zeigt drei Tabs:
    /// "RPC Code" (JSON-RPC eingeben/ausführen), "Workspace" (eingebettetes
    /// <see cref="CadControl"/> mit den benannten MCP-Objekten) und "Info".
    /// </summary>
    public class MCPServerWindow : Window
    {
        // TextBox, die beim Einfügen (Strg+V) nur die geschweiften JSON-Fragmente
        // aus der Zwischenablage übernimmt – Gegenstück zu JsonFilteringTextBox.
        private sealed class JsonFilteringTextBox : TextBox
        {
            // Without this the derived type has no ControlTheme (the Fluent theme is keyed on
            // typeof(TextBox)), so the control gets no template and no text can be entered.
            protected override Type StyleKeyOverride => typeof(TextBox);

            protected override void OnKeyDown(KeyEventArgs e)
            {
                if (e.KeyModifiers.HasFlag(KeyModifiers.Control) && e.Key == Key.V)
                {
                    e.Handled = true;
                    _ = PasteFilteredAsync();
                    return;
                }
                base.OnKeyDown(e);
            }

            private async Task PasteFilteredAsync()
            {
                var clipboard = TopLevel.GetTopLevel(this)?.Clipboard;
                if (clipboard == null) return;

                string? text = await clipboard.GetTextAsync();
                if (string.IsNullOrEmpty(text)) return;

                var fragments = ExtractBracedFragments(text);
                string filtered = string.Join(Environment.NewLine + Environment.NewLine, fragments);

                int start = Math.Min(SelectionStart, SelectionEnd);
                int end = Math.Max(SelectionStart, SelectionEnd);
                string current = Text ?? "";
                if (end > start) current = current.Remove(start, end - start);
                Text = current.Insert(start, filtered);
                CaretIndex = start + filtered.Length;
            }
        }

        private JsonFilteringTextBox textBox = null!;
        private Button okButton = null!;
        private TextBlock rpcMethodLabel = null!;
        private TextBlock rpcProgressLabel = null!;
        private TabItem workspaceTab = null!;

        private CadControl? cadControl;
        private Project? cadProject;
        private Model? cadModel;
        private IPropertyPage? wsPropPage;
        private GroupProperty? workspaceEntry = null;
        private bool workspaceInitialized = false;

        private readonly MCPServer server;

        public MCPServerWindow(MCPServer server)
        {
            this.server = server;
            InitializeComponents();
        }

        private void InitializeComponents()
        {
            Title = "MCP Server";
            Width = 900;
            Height = 600;
            WindowStartupLocation = WindowStartupLocation.CenterScreen;

            var tabControl = new TabControl();

            // --- Tab 1: RPC Code ---
            textBox = new JsonFilteringTextBox
            {
                AcceptsReturn = true,
                AcceptsTab = true,
                TextWrapping = TextWrapping.NoWrap,
                FontFamily = new FontFamily("Consolas, monospace"),
            };
            ScrollViewer.SetHorizontalScrollBarVisibility(textBox, ScrollBarVisibility.Auto);
            ScrollViewer.SetVerticalScrollBarVisibility(textBox, ScrollBarVisibility.Auto);

            rpcMethodLabel = new TextBlock
            {
                Text = "rpcMethodLabel",
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Avalonia.Thickness(4, 0, 0, 0),
            };
            rpcProgressLabel = new TextBlock
            {
                Text = "rpcProgressLabel",
                Width = 120,
                VerticalAlignment = VerticalAlignment.Center,
            };
            okButton = new Button
            {
                Content = "Ausführen",
                Width = 120,
                IsDefault = true,
            };
            okButton.Click += (_, _) => OkButton_Click();

            var bottomPanel = new DockPanel { Height = 40, LastChildFill = true };
            DockPanel.SetDock(okButton, Dock.Right);
            DockPanel.SetDock(rpcProgressLabel, Dock.Right);
            bottomPanel.Children.Add(okButton);
            bottomPanel.Children.Add(rpcProgressLabel);
            bottomPanel.Children.Add(rpcMethodLabel); // fills remaining space

            var rpcPanel = new DockPanel { LastChildFill = true };
            DockPanel.SetDock(bottomPanel, Dock.Bottom);
            rpcPanel.Children.Add(bottomPanel);
            rpcPanel.Children.Add(textBox);

            var rpcTab = new TabItem { Header = "RPC Code", Content = rpcPanel };

            // --- Tab 2: Workspace ---
            // Das CadControl wird erst beim ersten Anzeigen des Tabs initialisiert,
            // damit die OpenGL-Fläche sicher geladen ist.
            cadControl = new CadControl();
            workspaceTab = new TabItem { Header = "Workspace", Content = cadControl };

            // --- Tab 3: Info ---
            var infoTab = new TabItem { Header = "Info", Content = BuildInfoContent() };

            tabControl.Items.Add(rpcTab);
            tabControl.Items.Add(workspaceTab);
            tabControl.Items.Add(infoTab);

            tabControl.SelectionChanged += (_, _) =>
            {
                if (!workspaceInitialized && ReferenceEquals(tabControl.SelectedItem, workspaceTab))
                {
                    workspaceInitialized = true;
                    Dispatcher.UIThread.Post(() =>
                    {
                        InitializeWorkspaceControl();
                        if (server.namedItems?.Dict != null) InitWorkspace();
                    }, DispatcherPriority.Loaded);
                }
            };

            Content = tabControl;
        }

        private Control BuildInfoContent()
        {
            const string url = "https://www.cadability.de/ShapeIt/shapeit_ai_rpc_info.html";

            Button MakeLink() => new Button
            {
                Content = url,
                Foreground = Brushes.SteelBlue,
                Background = Brushes.Transparent,
                BorderThickness = new Avalonia.Thickness(0),
                Padding = new Avalonia.Thickness(0),
                Cursor = new Cursor(StandardCursorType.Hand),
            };

            var link1 = MakeLink();
            var link2 = MakeLink();
            link1.Click += (_, _) => OpenUrl(url);
            link2.Click += (_, _) => OpenUrl(url);

            return new StackPanel
            {
                Margin = new Avalonia.Thickness(12),
                Spacing = 4,
                Children =
                {
                    new TextBlock { Text = "Mehr Infos zur KI-Modellierung:" },
                    link1,
                    new TextBlock { Text = "", Height = 12 },
                    new TextBlock { Text = "More information about AI modeling:" },
                    link2,
                }
            };
        }

        private static void OpenUrl(string url)
        {
            try
            {
                System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo
                {
                    FileName = url,
                    UseShellExecute = true
                });
            }
            catch { /* best-effort */ }
        }

        // Erstellt Projekt/Modell/Property-Page im eingebetteten CadControl.
        private void InitializeWorkspaceControl()
        {
            if (cadControl == null) return;
            cadProject = cadControl.CadFrame.Project = Project.CreateSimpleProject();
            cadModel = cadProject.GetActiveModel();
            wsPropPage = cadControl.PropertiesExplorer.AddPropertyPage("Workspace", 2);
            cadControl.PropertiesExplorer.RemovePropertyPage("Project");
            cadControl.PropertiesExplorer.RemovePropertyPage("Global");
            cadControl.PropertiesExplorer.RemovePropertyPage("View");
            cadControl.PropertiesExplorer.ShowPropertyPage("Workspace");
        }

        public void AppendRpcCall(string rpcJson)
        {
            if (!string.IsNullOrEmpty(textBox.Text))
                textBox.Text += Environment.NewLine + Environment.NewLine;
            textBox.Text += rpcJson;
            textBox.CaretIndex = textBox.Text?.Length ?? 0;
        }

        private void OkButton_Click()
        {
            ProcessText(textBox.Text ?? "");
            if (workspaceInitialized) InitWorkspace();
        }

        private void UpdateRpcStatus(string status) => rpcMethodLabel.Text = status;

        private void InitWorkspace()
        {
            if (wsPropPage == null || cadControl == null || cadModel == null || cadProject == null)
                return;

            if (workspaceEntry != null) wsPropPage.Remove(workspaceEntry);
            workspaceEntry = new GroupProperty("Workspace", new IPropertyEntry[] { });
            List<IPropertyEntry> subEntries = new List<IPropertyEntry>();
            foreach (var item in server.namedItems.Dict)
            {
                if (item.Value is double d)
                {
                    DoubleProperty dp = new DoubleProperty(cadControl.CadFrame, item.Key);
                    dp.LabelText = item.Key;
                    dp.SetDouble(d);
                    dp.ReadOnly = true;
                    subEntries.Add(dp);
                }
                if (item.Value is Solid sld)
                {
                    SelectEntry se = new SelectEntry("", false);
                    se.LabelText = item.Key + " (Solid)";
                    se.IsSelected = (selected, frame) =>
                    {
                        if (selected)
                        {
                            cadModel.RemoveAll();
                            cadModel.Add(sld.Clone());
                            foreach (IGeoObject go in cadModel.AllObjects) go.UpdateAttributes(cadProject);
                            frame.ActiveView.ZoomTotal(1.1);
                        }
                        return true;
                    };
                    subEntries.Add(se);
                }
                if (item.Value is List<Solid> slds)
                {
                    SelectEntry se = new SelectEntry("", false);
                    se.LabelText = item.Key + " (Solids)";
                    se.IsSelected = (selected, frame) =>
                    {
                        if (selected)
                        {
                            cadModel.RemoveAll();
                            slds.ForEach(s => cadModel.Add(s.Clone())); // add clones to avoid issues with multiple parents
                            foreach (IGeoObject go in cadModel.AllObjects) go.UpdateAttributes(cadProject);
                            frame.ActiveView.ZoomTotal(1.1);
                        }
                        return true;
                    };
                    subEntries.Add(se);
                }
                if (item.Value is MCPServer.Sketch sketch)
                {
                    SelectEntry se = new SelectEntry("", false);
                    se.LabelText = item.Key + " (Sketch)";
                    se.IsSelected = (selected, frame) =>
                    {
                        if (selected)
                        {
                            cadModel.RemoveAll();
                            sketch.Curves?.ForEach(c => cadModel.Add(c.MakeGeoObject(Plane.XYPlane)));
                            sketch.Shapes?.ForEach(sh => cadModel.Add(sh.MakePaths(Plane.XYPlane)));
                            frame.ActiveView.Projection = Projection.FromTop;
                            frame.ActiveView.ZoomTotal(1.1);
                        }
                        return true;
                    };
                    subEntries.Add(se);
                }
                if (item.Value is ICurve2D c2d)
                {
                    SelectEntry se = new SelectEntry("", false);
                    se.LabelText = item.Key + " (2d Curve)";
                    se.IsSelected = (selected, frame) =>
                    {
                        if (selected)
                        {
                            cadModel.RemoveAll();
                            cadModel.Add(c2d.MakeGeoObject(Plane.XYPlane));
                            frame.ActiveView.Projection = Projection.FromTop;
                            frame.ActiveView.ZoomTotal(1.1);
                        }
                        return true;
                    };
                    subEntries.Add(se);
                }
                if (item.Value is List<ICurve2D> c2ds)
                {
                    SelectEntry se = new SelectEntry("", false);
                    se.LabelText = item.Key + " (2d Curves)";
                    se.IsSelected = (selected, frame) =>
                    {
                        if (selected)
                        {
                            cadModel.RemoveAll();
                            c2ds.ForEach(c => cadModel.Add(c.MakeGeoObject(Plane.XYPlane)));
                            frame.ActiveView.Projection = Projection.FromTop;
                            frame.ActiveView.ZoomTotal(1.1);
                        }
                        return true;
                    };
                    subEntries.Add(se);
                }
                if (item.Value is CompoundShape cs)
                {
                    SelectEntry se = new SelectEntry("", false);
                    se.LabelText = item.Key + " (2d closed Profile)";
                    se.IsSelected = (selected, frame) =>
                    {
                        if (selected)
                        {
                            cadModel.RemoveAll();
                            cadModel.Add(cs.MakePaths(Plane.XYPlane));
                            frame.ActiveView.Projection = Projection.FromTop;
                            frame.ActiveView.ZoomTotal(1.1);
                        }
                        return true;
                    };
                    subEntries.Add(se);
                }
                if (item.Value is List<CompoundShape> css)
                {
                    SelectEntry se = new SelectEntry("", false);
                    se.LabelText = item.Key + " (2d closed Profiles)";
                    se.IsSelected = (selected, frame) =>
                    {
                        if (selected)
                        {
                            cadModel.RemoveAll();
                            css.ForEach(sh => cadModel.Add(sh.MakePaths(Plane.XYPlane)));
                            frame.ActiveView.Projection = Projection.FromTop;
                            frame.ActiveView.ZoomTotal(1.1);
                        }
                        return true;
                    };
                    subEntries.Add(se);
                }
            }
            workspaceEntry.SetSubEntries(subEntries.ToArray());
            wsPropPage.Add(workspaceEntry, false);
        }

        private void ProcessText(string text)
        {
            foreach (var jsonBlock in ReadJsonObjects(text))
            {
                TryParseRpcBlock(jsonBlock);
                if (server.stopExecution) break;
            }
        }

        private bool TryParseRpcBlock(string json)
        {
            if (string.IsNullOrWhiteSpace(json)) { return false; }
            try
            {
                server.currentRpcString = json;
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;
                string message = "";
                if (root.TryGetProperty("id", out var idEl))
                {
                    if (idEl.ValueKind == JsonValueKind.Number) message = "id: " + idEl.GetInt32().ToString() + ", ";
                }
                if (root.TryGetProperty("method", out var m) && m.ValueKind == JsonValueKind.String)
                {
                    message += m.GetString();
                }
                UpdateRpcStatus(message);

                server.ProcessMethod(root);
                return true;
            }
            catch (Exception ex)
            {
                if (!server.ReportError(ex.Message)) server.stopExecution = true;
                return false;
            }
        }

        private static IEnumerable<string> ReadJsonObjects(string text)
        {
            var sb = new StringBuilder();

            int braceDepth = 0;
            bool inString = false;
            bool escape = false;

            foreach (char c in text)
            {
                sb.Append(c);

                if (escape) { escape = false; continue; }
                if (c == '\\') { escape = true; continue; }
                if (c == '"') { inString = !inString; continue; }

                if (!inString)
                {
                    if (c == '{')
                    {
                        braceDepth++;
                    }
                    else if (c == '}')
                    {
                        braceDepth--;
                        if (braceDepth == 0)
                        {
                            yield return sb.ToString();
                            sb.Clear();
                        }
                    }
                }
            }
        }

        public static List<string> ExtractBracedFragments(string text)
        {
            var result = new List<string>();

            int depth = 0;
            int start = -1;
            bool inString = false;
            bool escape = false;

            for (int i = 0; i < text.Length; i++)
            {
                char ch = text[i];

                if (inString)
                {
                    if (escape) escape = false;
                    else if (ch == '\\') escape = true;
                    else if (ch == '"') inString = false;
                    continue;
                }

                if (ch == '"')
                {
                    inString = true;
                }
                else if (ch == '{')
                {
                    if (depth == 0) start = i;
                    depth++;
                }
                else if (ch == '}')
                {
                    if (depth > 0)
                    {
                        depth--;
                        if (depth == 0 && start >= 0)
                        {
                            result.Add(text.Substring(start, i - start + 1));
                            start = -1;
                        }
                    }
                }
            }

            return result;
        }
    }
}
