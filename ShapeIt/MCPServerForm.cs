using CADability;
using CADability.Curve2D;
using CADability.Forms.NET8;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Windows.Forms;

namespace ShapeIt
{
    public class MCPServerForm : Form
    {
        public class JsonFilteringTextBox : RichTextBox
        {
            protected override void OnKeyDown(KeyEventArgs e)
            {
                if (e.Control && e.KeyCode == Keys.V)
                {
                    e.SuppressKeyPress = true;

                    string text = Clipboard.GetText();

                    var fragments = ExtractBracedFragments(text);

                    string filtered = string.Join(
                        Environment.NewLine + Environment.NewLine,
                        fragments);

                    SelectedText = filtered;
                    return;
                }

                base.OnKeyDown(e);
            }

            protected override void WndProc(ref Message m)
            {
                const int WM_PASTE = 0x0302;

                if (m.Msg == WM_PASTE)
                {
                    string text = Clipboard.GetText();

                    var fragments = ExtractBracedFragments(text);

                    string filtered = string.Join(
                        Environment.NewLine + Environment.NewLine,
                        fragments);

                    SelectedText = filtered;
                    return;
                }

                base.WndProc(ref m);
            }
        }

        private JsonFilteringTextBox textBox;
        private Button okButton;
        private Label rpcMethodLabel;
        private Label rpcProgressLabel;
        private CadControl cadControl;
        Project cadProject;
        Model cadModel;
        private MCPServer server;
        private IPropertyPage wsPropPage;
        private GroupProperty? workspaceEntry = null;


        private static IEnumerable<string> ReadJsonObjects(string text)
        {
            var sb = new StringBuilder();

            int braceDepth = 0;
            bool inString = false;
            bool escape = false;

            foreach (char c in text)
            {
                sb.Append(c);

                if (escape)
                {
                    escape = false;
                    continue;
                }

                if (c == '\\')
                {
                    escape = true;
                    continue;
                }

                if (c == '"')
                {
                    inString = !inString;
                    continue;
                }

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
        public MCPServerForm(MCPServer server)
        {
            this.server = server;
            InitializeComponents();
        }

        private void InitializeComponents()
        {
            // Bildschirmgröße bestimmen
            Rectangle screen = Screen.PrimaryScreen.WorkingArea;

            // Dialoggröße = halber Bildschirm
            this.Size = new Size(screen.Width / 2, screen.Height / 2);

            // Zentriert anzeigen
            this.StartPosition = FormStartPosition.CenterScreen;

            this.Text = "MCP Server";
            this.FormBorderStyle = FormBorderStyle.Sizable;
            this.MinimizeBox = true;
            this.MaximizeBox = true;

            // TabControl
            TabControl tabControl = new TabControl();
            tabControl.Dock = DockStyle.Fill;

            // --- Tab 1: RPC Code ---
            TabPage rpcTab = new TabPage("RPC Code");

            textBox = new JsonFilteringTextBox();
            textBox.Multiline = true;
            textBox.Dock = DockStyle.Fill;
            textBox.ScrollBars = RichTextBoxScrollBars.Both;
            textBox.AcceptsTab = true;

            Panel bottomPanel = new Panel();
            bottomPanel.Dock = DockStyle.Bottom;
            bottomPanel.Height = 40;

            rpcMethodLabel = new Label();
            rpcMethodLabel.Dock = DockStyle.Fill;
            rpcMethodLabel.TextAlign = ContentAlignment.MiddleLeft;
            rpcMethodLabel.Text = "rpcMethodLabel";

            rpcProgressLabel = new Label();
            rpcProgressLabel.Dock = DockStyle.Right;
            rpcProgressLabel.Width = 120;
            rpcProgressLabel.TextAlign = ContentAlignment.MiddleLeft;
            rpcProgressLabel.Text = "rpcProgressLabel";

            okButton = new Button();
            okButton.Text = "Ausführen";
            okButton.Dock = DockStyle.Right;
            okButton.Width = 120;
            okButton.Click += OkButton_Click;

            bottomPanel.Controls.Add(rpcMethodLabel);
            bottomPanel.Controls.Add(rpcProgressLabel);
            bottomPanel.Controls.Add(okButton);

            rpcTab.Controls.Add(textBox);
            rpcTab.Controls.Add(bottomPanel);

            // --- Tab 2: Workspace ---
            TabPage workspaceTab = new TabPage("Workspace");

            cadControl = new CadControl();
            cadControl.Dock = DockStyle.Fill;
            cadProject = cadControl.CadFrame.Project = Project.CreateSimpleProject();
            cadModel = cadProject.GetActiveModel();
            wsPropPage = cadControl.PropertiesExplorer.AddPropertyPage("Workspace", 2);
            // cadControl.PropertiesExplorer.RemovePropertyPage("Action");
            cadControl.PropertiesExplorer.RemovePropertyPage("Project");
            cadControl.PropertiesExplorer.RemovePropertyPage("Global");
            cadControl.PropertiesExplorer.RemovePropertyPage("View");
            cadControl.PropertiesExplorer.ShowPropertyPage("Workspace");
            cadControl.ToolbarsVisible = false;

            workspaceTab.Controls.Add(cadControl);

            // --- Tab 3: Info ---
            TabPage infoTab = new TabPage("Info");

            LinkLabel infoLabel = new LinkLabel();
            infoLabel.Dock = DockStyle.Fill;

            string url = "https://www.cadability.de/ShapeIt/shapeit_ai_rpc_info.html";

            infoLabel.Text =
                "Mehr Infos zur KI-Modellierung:\n" +
                url + "\n\n" +
                "More information about AI modeling:\n" +
                url;

            // beide Links klickbar machen
            int firstIndex = infoLabel.Text.IndexOf(url);
            int secondIndex = infoLabel.Text.LastIndexOf(url);

            infoLabel.Links.Add(firstIndex, url.Length, url);
            infoLabel.Links.Add(secondIndex, url.Length, url);

            infoLabel.LinkClicked += (sender, e) =>
            {
                string url = e.Link.LinkData as string ?? infoLabel.Text.Substring(e.Link.Start, e.Link.Length);

                System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo
                {
                    FileName = url,
                    UseShellExecute = true
                });
            };

            infoTab.Controls.Add(infoLabel);

            // Tabs hinzufügen
            tabControl.TabPages.Add(rpcTab);
            tabControl.TabPages.Add(workspaceTab);
            tabControl.TabPages.Add(infoTab);

            this.Controls.Add(tabControl);

            this.AcceptButton = okButton;

            if (server.namedItems != null && server.namedItems.Dict != null) InitWorkspace();
        }

        private void OkButton_Click(object? sender, EventArgs e)
        {
            ProcessText(textBox.Text);
            this.DialogResult = DialogResult.OK;
            InitWorkspace();
        }
        private void UpdateRpcStatus(string status)
        {
            rpcMethodLabel.Text = status;

            rpcMethodLabel.Refresh();
        }
        private void InitWorkspace()
        {
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

        bool TryParseRpcBlock(string json)
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
                    if (escape)
                    {
                        escape = false;
                    }
                    else if (ch == '\\')
                    {
                        escape = true;
                    }
                    else if (ch == '"')
                    {
                        inString = false;
                    }

                    continue;
                }

                // außerhalb eines String-Literals
                if (ch == '"')
                {
                    inString = true;
                }
                else if (ch == '{')
                {
                    if (depth == 0)
                        start = i;

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