using CADability;
using CADability.Actions;
using CADability.Attribute;
using CADability.Curve2D;
using CADability.Forms.NET8;
using CADability.GeoObject;
using CADability.UserInterface;
using MathNet.Numerics.LinearAlgebra.Factorization;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml;
using System.Xml.Linq;
using static ShapeIt.MainForm;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;
using Point = System.Drawing.Point;

namespace ShapeIt
{

    public partial class MainForm : CadForm
    {
        private PictureBox logoBox;
        private ModellingPropertyEntries modellingPropertyEntries;
        private DateTime lastSaved; // time, when the current file has been saved the last time, see OnIdle
        private bool modifiedSinceLastAutosave = false;
        bool projectionChanged = false; // to handle projection changes in OnIdle
        bool crashChecked = false;

        private Control FindControlByName(Control parent, string name)
        {
            foreach (Control child in parent.Controls)
            {
                if (child.Name == name)
                    return child;

                Control found = FindControlByName(child, name);
                if (found != null)
                    return found;
            }

            return null;
        }

        void FadeOutPictureBox(PictureBox pb)
        {
            var timer = new System.Windows.Forms.Timer();
            timer.Interval = 50;
            double alpha = 1.0;

            Image original = pb.Image;
            Bitmap faded = new Bitmap(original.Width, original.Height);

            timer.Tick += (s, e) =>
            {
                alpha -= 0.01;
                if (alpha <= 0)
                {
                    timer.Stop();
                    pb.Parent.Controls.Remove(pb);
                    //pb.Visible = false;
                    pb.Dispose();
                    return;
                }

                using (Graphics g = Graphics.FromImage(faded))
                {
                    g.Clear(Color.Transparent);
                    ColorMatrix matrix = new ColorMatrix
                    {
                        Matrix33 = (float)alpha // Alpha-Kanal
                    };
                    ImageAttributes attributes = new ImageAttributes();
                    attributes.SetColorMatrix(matrix, ColorMatrixFlag.Default, ColorAdjustType.Bitmap);

                    g.DrawImage(original,
                        new Rectangle(0, 0, faded.Width, faded.Height),
                        0, 0, original.Width, original.Height,
                        GraphicsUnit.Pixel,
                        attributes);
                }

                pb.Image = (Image)faded.Clone(); // neues Bild setzen
            };

            timer.Start();
        }

        private void ShowLogo()
        {
            Control pex = FindControlByName(this, "propertiesExplorer");
            // Create PictureBox 
            logoBox = new PictureBox();
            Assembly ThisAssembly = Assembly.GetExecutingAssembly();
            using (System.IO.Stream str = ThisAssembly.GetManifestResourceStream("ShapeIt.Resources.ShapeIt2.png"))
            {
                logoBox.Image = new Bitmap(str);
            }
            logoBox.SizeMode = PictureBoxSizeMode.Zoom;

            double aspectRatio = (double)logoBox.Image.Height / logoBox.Image.Width;

            // Zielbreite übernehmen
            int targetWidth = pex.ClientSize.Width - 4;
            int berechneteHoehe = (int)(targetWidth * aspectRatio);

            // Größe setzen
            logoBox.Size = new Size(targetWidth, berechneteHoehe);

            // Position am unteren Rand
            logoBox.Location = new Point(2, pex.ClientSize.Height - berechneteHoehe - 2);

            // Logo zum Ziel-Control hinzufügen
            pex.Controls.Add(logoBox);
            logoBox.BringToFront();

            FadeOutPictureBox(logoBox);

            pex.Resize += (s, e) =>
            {
                int newWidth = pex.ClientSize.Width - 4;
                int newHeight = (int)(newWidth * aspectRatio);
                logoBox.Size = new Size(newWidth, newHeight);
                logoBox.Location = new Point(2, pex.ClientSize.Height - newHeight - 2);
            };
        }

        private string ReadEmbeddedVersion()
        {
            var asm = typeof(Program).Assembly;
            using var s = asm.GetManifestResourceStream("App.Version");
            using var sr = new StreamReader(s!);
            return sr.ReadToEnd().Trim();
        }
        public MainForm(string[] args) : base(args)
        {   // interpret the command line arguments as a name of a file, which should be opened
            // The parallelization of MathNet.Numerics is switched off in CADability.NumericsConfiguration now,
            // for every host and both configurations - see there for what it costs and why it used to be here.
            string fileName = "";
            bool debugBRep = false;
            bool nofile = false;
            bool debugRPC = false;
            bool autoDebug = false;
            int repeatCount = 1;
            // "-e:<list>", "-f:<list>", "-v:<list>": break in the debugger when an edge, face or vertex
            // with one of the given hashCodes is created, e.g. "-e:29196" or "-e:1468,1469,2000-2010"
            // "-hcoffset:<n>": start the hashCode counters at n instead of 0, which changes the enumeration
            // order of every CADability.Set and thereby exposes algorithms that depend on it. Run the same
            // file with "-d file -hcoffset:0", "-hcoffset:1", ... - any change in the result is such a case.
            DebugBreak.ParseCommandLine(args);
            for (int i = 0; i < args.Length; i++)
            {
                if (!args[i].StartsWith("-"))
                {
                    fileName = args[i];
                }
                else if (args[i] == "-d")
                {
                    debugBRep = true;
                }
                else if (args[i] == "-r")
                {
                    debugRPC = true;
                }
                else if (args[i] == "-a")
                {
                    autoDebug = true;
                }
                else if (args[i] == "-x")
                {   // so I can leave the file name in the command line, but don't want to open it, e.g. for debugging
                    nofile = true;
                }
                else if (args[i].StartsWith("-c:"))
                {   // "-c:8": run the RPC case 8 times instead of once, to make a flaky result show itself
                    if (!int.TryParse(args[i].Substring(3), out repeatCount) || repeatCount < 1)
                    {
                        Trace.WriteLine($"invalid repeat count in \"{args[i]}\", expected e.g. -c:8");
                        repeatCount = 1;
                    }
                }
            }

            if (debugBRep)
            {   // use the collected fileName, not args[1], so the order of the arguments doesn't matter
                DebugBRep(fileName);
                Close();
                return;
            }
            if (debugRPC)
            {
                DebugRPC(fileName, repeatCount);
            }
            if (autoDebug)
            {
                AutoDebug();
            }

            ShowLogo();
            // this.Icon = Properties.Resources.Icon;
            Assembly ThisAssembly = Assembly.GetExecutingAssembly();
            System.IO.Stream? str;
            using (str = ThisAssembly.GetManifestResourceStream("ShapeIt.Resources.Icon.ico"))
            {
                this.Icon = new System.Drawing.Icon(str);
            }

            if (!debugRPC)
            {
                Project toOpen = null;
                if (!String.IsNullOrWhiteSpace(fileName) && !nofile)
                {
                    try
                    {
                        toOpen = Project.ReadFromFile(fileName);
                    }
                    catch { }
                }
                if (toOpen == null) CadFrame.GenerateNewProject();
                else CadFrame.Project = toOpen;
            }

            string version = ReadEmbeddedVersion(); // version from version.txt
            this.Text = $"ShapeIt with CADability – Version: {version}";

            if (!Settings.GlobalSettings.ContainsSetting("UserInterface"))
            {
                Settings UserInterface = new Settings("UserInterface");
                Settings.GlobalSettings.AddSetting("UserInterface", UserInterface);
                IntegerProperty ToolbarButtonSize = new IntegerProperty("ToolbarButtonSize", "ToolbarButtonSize");
                ToolbarButtonSize.IntegerValue = 0;
                UserInterface.AddSetting("ToolbarButtonSize", ToolbarButtonSize);
                IntegerProperty MenuSize = new IntegerProperty("MenuSize", "MenuSize");
                MenuSize.IntegerValue = 0;
                UserInterface.AddSetting("MenuSize", MenuSize);
            }
            Settings.GlobalSettings.SetValue("Construct.3D_Delete2DBase", false);
            bool exp = Settings.GlobalSettings.GetBoolValue("Experimental.TestNewContextMenu", false);
            bool tst = Settings.GlobalSettings.GetBoolValue("ShapeIt.Initialized", false);
            if (!Settings.GlobalSettings.GetBoolValue("ShapeIt.Initialized", false))
            {
                Settings colorSettings = Settings.GlobalSettings.GetSubSetting("Colors");

            }
            Settings.GlobalSettings.SetValue("ShapeIt.Initialized", true);
            CadFrame.FileNameChangedEvent += (name) =>
            {
                if (string.IsNullOrEmpty(name)) this.Text = "ShapeIt with CADability";
                else this.Text = "ShapeIt -- " + name;
                lastSaved = DateTime.Now; // a new file has been opened
            };
            CadFrame.ProjectClosedEvent += OnProjectClosed;
            CadFrame.ProjectOpenedEvent += OnProjectOpened;
            CadFrame.UIService.ApplicationIdle += OnIdle;
            CadFrame.ViewsChangedEvent += OnViewsChanged;
            if (CadFrame.ActiveView != null) OnViewsChanged(CadFrame);
            CadFrame.ControlCenter.RemovePropertyPage("View");
            using (str = ThisAssembly.GetManifestResourceStream("ShapeIt.StringTableDeutsch.xml"))
            {
                XmlDocument stringXmlDocument = new XmlDocument();
                stringXmlDocument.Load(str);
                StringTable.AddStrings(stringXmlDocument);
            }

            using (str = ThisAssembly.GetManifestResourceStream("ShapeIt.StringTableEnglish.xml"))
            {
                XmlDocument stringXmlDocument = new XmlDocument();
                stringXmlDocument.Load(str);
                StringTable.AddStrings(stringXmlDocument);
            }
            using (str = ThisAssembly.GetManifestResourceStream("ShapeIt.MenuResource.xml"))
            {
                XmlDocument menuDocument = new XmlDocument();
                menuDocument.Load(str);
#if DEBUG
                // inject an additional menu "Debug", which we can use to directly execute some debugging code
                XmlNode mainMenu = menuDocument.SelectSingleNode("Menus/MainMenu/Popup[@MenuId='MenuId.Extras']");
                if (mainMenu != null)
                {
                    // Create a new MenuItem element.
                    XmlElement debugMenuItem = menuDocument.CreateElement("MenuItem");
                    // Set the MenuId attribute to "MenuId.Debug".
                    debugMenuItem.SetAttribute("MenuId", "MenuId.Debug");
                    // Append the new MenuItem element to the MainMenu node.
                    mainMenu.AppendChild(debugMenuItem);
                }
#endif
                XmlNode toolbar = menuDocument.SelectSingleNode("Menus/Popup[@MenuId='Toolbar']");
                SetToolbar(toolbar);
                MenuResource.SetMenuResource(menuDocument);
                ResetMainMenu(null);
            }

            lastSaved = DateTime.Now;
            AppDomain.CurrentDomain.UnhandledException += (sender, exobj) =>
            {
                // exobj.ExceptionObject as Exception;
                try
                {
                    string path = System.IO.Path.GetTempPath();
                    path = System.IO.Path.Combine(path, "ShapeIt");
                    DirectoryInfo dirInfo = Directory.CreateDirectory(path);
                    string currentFileName = CadFrame.Project.FileName;
                    if (string.IsNullOrEmpty(CadFrame.Project.FileName))
                    {
                        path = System.IO.Path.Combine(path, "crash_" + DateTime.Now.ToString("yyMMddHHmm") + ".cdb.json");
                        currentFileName = "unknown";
                    }
                    else
                    {
                        string crashFileName = System.IO.Path.GetFileNameWithoutExtension(CadFrame.Project.FileName);
                        if (crashFileName.EndsWith(".cdb")) crashFileName = System.IO.Path.GetFileNameWithoutExtension(crashFileName); // we usually have two extensions: .cdb.json
                        path = System.IO.Path.Combine(path, crashFileName + "_X.cdb.json");
                    }
                    CadFrame.Project.WriteToFile(path);
                    File.WriteAllText(System.IO.Path.Combine(System.IO.Path.GetTempPath(), @"ShapeIt\Crash.txt"), currentFileName + "\n" + path);
                }
                catch (Exception) { }
                ;
            };
            // the following installs the property page for modelling. This connects all modelling
            // tasks of ShapeIt with CADability
            IPropertyPage modellingPropPage = CadFrame.ControlCenter.AddPropertyPage("Modelling", 6);
            modellingPropertyEntries = new ModellingPropertyEntries(CadFrame);
            modellingPropPage.Add(modellingPropertyEntries, false);
            CadFrame.ControlCenter.ShowPropertyPage("Modelling");
        }

        private void OnViewsChanged(IFrame theFrame)
        {
            theFrame.ActiveView.Projection.ProjectionChangedEvent -= OnProjectionChanged; // not to double it?
            theFrame.ActiveView.Projection.ProjectionChangedEvent += OnProjectionChanged;
        }

        private void OnProjectionChanged(Projection sender, EventArgs args)
        {
            projectionChanged = true;
        }

        protected override void OnShown(EventArgs e)
        {
            // check for crash
            if (!crashChecked)
            {
                crashChecked = true;
                string crashPath = System.IO.Path.Combine(System.IO.Path.GetTempPath(), @"ShapeIt\Crash.txt");
                if (File.Exists(crashPath))
                {
                    string[] lines = File.ReadAllLines(System.IO.Path.Combine(System.IO.Path.GetTempPath(), @"ShapeIt\Crash.txt"));
                    if (lines.Length == 2)
                    {
                        string ask = StringTable.GetFormattedString("ShapeIt.RestoreAfterCrash", lines[0]);
                        if (CadFrame.UIService.ShowMessageBox(ask, "ShapeIt", CADability.Substitutes.MessageBoxButtons.YesNo) == CADability.Substitutes.DialogResult.Yes)
                        {
                            CadFrame.Project = Project.ReadFromFile(lines[1]);
                            CadFrame.Project.FileName = lines[0];

                            this.Text = "ShapeIt -- " + lines[0];
                        }
                    }
                    File.Delete(crashPath);
                }
#if DEBUG
                // AutoDebug();
#endif
            }
            base.OnActivated(e);
        }

        /// <summary>
        /// Debug helper: executes the JSON-RPC calls of an RPC case file (see tests/CADability.Tests/Files/RPC).
        /// Such a file is a JSON object with a single array "RPCCalls", each entry being a complete JSON-RPC
        /// request. The requests are handed to <see cref="MCPServer.ProcessMethod(JsonElement, bool)"/> directly,
        /// i.e. without HTTP and without an MCP client, so the whole call chain can be stepped through here.
        /// <para>
        /// The calls typically build one or more solids and insert them into the project with
        /// document.commit_objects; this is the same path a regression test will take later, which is why
        /// everything is logged the way the test would compare it.
        /// </para>
        /// </summary>
        /// <param name="filename">The RPC case file to run, given on the command line as -r &lt;file&gt;.</param>
        /// <param name="repeatCount">How often to run it, from "-c:&lt;n&gt;". Every run starts from a fresh
        /// project and a fresh server, so the runs are independent - which is the point: a result that
        /// differs between them is not reproducible, and the summary at the end says which runs agree.</param>
        private void DebugRPC(string filename, int repeatCount = 1)
        {
            if (string.IsNullOrEmpty(filename)) return;
            if (!File.Exists(filename))
            {   // never fail silently, a mistyped path would otherwise look like an empty run
                Trace.WriteLine($"DebugRPC: file not found: {filename}");
                return;
            }
            if (repeatCount < 1) repeatCount = 1;
            string caseName = System.IO.Path.GetFileNameWithoutExtension(filename);
            List<SortedDictionary<string, string>> results = new List<SortedDictionary<string, string>>();
            for (int run = 1; run <= repeatCount; run++)
            {
                if (repeatCount > 1) Trace.WriteLine($"DebugRPC {caseName}: ---------- run {run} of {repeatCount} ----------");
                results.Add(DebugRPCOnce(filename, caseName));
            }
            if (repeatCount > 1) ReportRepeatedRuns(caseName, results);
        }

        /// <summary>
        /// One run of an RPC case file. Returns the fingerprint of what it produced, see
        /// <see cref="Fingerprint"/>.
        /// </summary>
        private SortedDictionary<string, string> DebugRPCOnce(string filename, string caseName)
        {
            // The calls need a project: document.commit_objects adds the solids to its active model.
            if (CadFrame.Project != null) CadFrame.Project.IsModified = false; // to avoid messagebox asking for saving modified project
            CadFrame.GenerateNewProject();
            MCPServer server = new MCPServer(CadFrame, CadFrame.Project);
            // Run unattended: errors end up in the protocol instead of in a modal message box.
            server.SuppressDialogs = true;

            int executed = 0;
            int total = 0;
            Stopwatch watch = Stopwatch.StartNew();
            try
            {
                // The JsonElements handed to ProcessMethod stay valid only as long as the document lives,
                // so all calls are executed inside this using block.
                using JsonDocument doc = JsonDocument.Parse(File.ReadAllText(filename));
                if (!doc.RootElement.TryGetProperty("RPCCalls", out JsonElement calls) || calls.ValueKind != JsonValueKind.Array)
                {
                    Trace.WriteLine($"DebugRPC {caseName}: no array 'RPCCalls' in {filename}");
                    return new SortedDictionary<string, string>(StringComparer.Ordinal);
                }
                total = calls.GetArrayLength();
                foreach (JsonElement call in calls.EnumerateArray())
                {
                    server.ProcessMethod(call); // this is the line to step into
                    ++executed;
                    if (server.stopExecution) break; // an error occurred and further processing was canceled
                }
            }
            catch (Exception e)
            {
                Trace.WriteLine($"DebugRPC {caseName}: exception after {executed} calls: {e.Message}");
            }
            watch.Stop();

            Trace.WriteLine($"DebugRPC {caseName}: {executed} of {total} calls executed"
                + $" ({watch.ElapsedMilliseconds} ms, stopped={server.stopExecution})");
            Trace.WriteLine(server.Protocol); // request and response of every call, as in the "Protokoll" tab

            // What ended up in the document is the actual result of the run and what the regression test
            // will have to check.
            Model model = CadFrame.Project.GetActiveModel();
            GeoObjectList committed = model.AllObjects;
            Trace.WriteLine($"DebugRPC {caseName}: {committed.Count} object(s) in the model:");
            for (int i = 0; i < committed.Count; i++)
            {
                IGeoObject go = committed[i];
                string name = (go as Solid)?.Name;
                BoundingBox extent = go.GetExtent(0.0);
                Trace.WriteLine($"  {(string.IsNullOrEmpty(name) ? "(unnamed)" : name)}: {go.GetType().Name}"
                    + $", extent ({extent.Xmin:F3},{extent.Ymin:F3},{extent.Zmin:F3})-({extent.Xmax:F3},{extent.Ymax:F3},{extent.Zmax:F3})");
            }
            try
            {
                return Fingerprint(server);
            }
            catch (Exception e)
            {   // the fingerprint is a diagnostic, it must never take the application down at startup
                Trace.WriteLine($"DebugRPC {caseName}: could not build the fingerprint: {e.Message}");
                return new SortedDictionary<string, string>(StringComparer.Ordinal);
            }
        }

        /// <summary>
        /// The invariants of everything the run left in the workspace, keyed "name.field". Taken from the
        /// NAMED items rather than from the model: a case need not commit anything - several do not - and
        /// the regression harness compares the same names with the same <see cref="ShellMetrics"/> code, so
        /// a difference seen here is a difference the test would report as well.
        /// </summary>
        private static SortedDictionary<string, string> Fingerprint(MCPServer server)
        {
            SortedDictionary<string, string> res = new SortedDictionary<string, string>(StringComparer.Ordinal);
            foreach (string name in server.namedItems.Keys)
            {
                List<Shell> shells = new List<Shell>();
                object item = server.namedItems[name];
                AddShells(item, shells);
                if (shells.Count == 0) continue;
                // Canonical order, never the order the operation happened to return: that order has been
                // seen to vary between runs all by itself and would drown the difference we are looking for.
                Shell[] sorted = ShellMetrics.SortCanonically(shells);
                BRepSummary summary = new BRepSummary();
                if (sorted.Length == 1) ShellMetrics.Describe(summary, name + ".", sorted[0]);
                else
                {
                    summary.Add(name + ".solids", sorted.Length);
                    for (int i = 0; i < sorted.Length; i++) ShellMetrics.Describe(summary, name + "#" + i + ".", sorted[i]);
                }
                foreach (KeyValuePair<string, string> entry in summary.Entries) res[entry.Key] = entry.Value;
            }
            return res;
        }

        /// <summary>
        /// Groups the runs by what they produced and names the fields that move. With "-c:8" this is the
        /// line to read: one group means the case is reproducible and stepping through it is worth
        /// something, several groups mean it is not, and say which runs to compare against each other.
        /// </summary>
        private static void ReportRepeatedRuns(string caseName, List<SortedDictionary<string, string>> results)
        {
            List<List<int>> groups = new List<List<int>>();      // run numbers, 1 based
            List<SortedDictionary<string, string>> distinct = new List<SortedDictionary<string, string>>();
            for (int i = 0; i < results.Count; i++)
            {
                int found = -1;
                for (int g = 0; g < distinct.Count && found < 0; g++)
                {
                    if (SameResult(distinct[g], results[i])) found = g;
                }
                if (found < 0)
                {
                    distinct.Add(results[i]);
                    groups.Add(new List<int>());
                    found = distinct.Count - 1;
                }
                groups[found].Add(i + 1);
            }

            Trace.WriteLine("DebugRPC " + caseName + ": " + results.Count + " runs, " + distinct.Count
                + " distinct result(s)" + (distinct.Count == 1 ? " - reproducible" : " - NOT reproducible"));
            for (int g = 0; g < groups.Count; g++)
            {
                Trace.WriteLine("  result " + (char)('A' + g) + ": run(s) " + string.Join(", ", groups[g]));
            }
            if (distinct.Count < 2) return;

            // which fields actually move
            SortedSet<string> keys = new SortedSet<string>(StringComparer.Ordinal);
            foreach (SortedDictionary<string, string> result in distinct)
            {
                foreach (string key in result.Keys) keys.Add(key);
            }
            Trace.WriteLine("  fields that differ between the results:");
            foreach (string key in keys)
            {
                List<string> values = new List<string>();
                bool differs = false;
                foreach (SortedDictionary<string, string> result in distinct)
                {
                    string value = result.TryGetValue(key, out string found) ? found : "(missing)";
                    if (values.Count > 0 && value != values[0]) differs = true;
                    values.Add(value);
                }
                if (differs) Trace.WriteLine("    " + key + ": " + string.Join(" | ", values));
            }
        }

        /// <summary>Collects the shells of a named workspace item, which may be a solid, a shell or a list
        /// of either. A solid without a shell is skipped rather than throwing: this is a diagnostic.</summary>
        private static void AddShells(object item, List<Shell> shells)
        {
            if (item is Solid solid)
            {
                if (solid.Shells != null && solid.Shells.Length > 0 && solid.Shells[0] != null) shells.Add(solid.Shells[0]);
            }
            else if (item is Shell shell) shells.Add(shell);
            else if (item is System.Collections.IEnumerable list)
            {
                foreach (object o in list) AddShells(o, shells);
            }
        }

        /// <summary>Two runs count as the same result when every recorded field is identical.</summary>
        private static bool SameResult(SortedDictionary<string, string> a, SortedDictionary<string, string> b)
        {
            if (a.Count != b.Count) return false;
            foreach (KeyValuePair<string, string> entry in a)
            {
                if (!b.TryGetValue(entry.Key, out string other) || other != entry.Value) return false;
            }
            return true;
        }

        /// <summary>
        /// Debug helper: opens a project that describes a BRep test case and executes it. The case is described
        /// inside the project itself - the styles "Operand1"/"Operand2"/"EdgeMarker" plus a text object naming
        /// the operation, see <see cref="BRepCaseReader"/>.
        /// <para>
        /// This takes the same path as the BRep regression tests (tests/CADability.Tests/BRepRegressionTests.cs),
        /// so a failing test can be reproduced here: reading the same file always yields the same hash codes,
        /// which is what makes conditional breakpoints usable. Unlike the tests, the operation runs on this
        /// thread and without a time budget, so stepping and breakpoints behave normally.
        /// </para>
        /// </summary>
        /// <param name="filename">The project to run; the most recently used file when empty.</param>
        private void DebugBRep(string filename)
        {
            if (string.IsNullOrEmpty(filename))
            {
                string[] mru = MRUFiles.GetMRUFiles();
                if (mru.Length > 0) filename = mru.Last().Split(';')[0];
            }
            if (string.IsNullOrEmpty(filename)) return;
            // Open the project in the app first and build the case from exactly those objects, so that what you
            // see on the screen is what the operation works on.
            CadFrame.Project = Project.ReadFromFile(filename);
            BRepCase testCase = BRepCaseReader.FromModel(CadFrame.Project.GetActiveModel(), BRepCaseReader.CaseName(filename));
            foreach (string warning in testCase.Warnings) Trace.WriteLine($"AutoDebug {testCase.Name}: warning: {warning}");
            if (!testCase.IsRunnable)
            {   // never fail silently - that is how a typo in the text object used to make this method do nothing
                foreach (string problem in testCase.Problems) Trace.WriteLine($"AutoDebug {testCase.Name}: {problem}");
                return;
            }

            BRepRunResult run = new BRepRunResult();
            Stopwatch watch = Stopwatch.StartNew();
            try
            {
                run.Shells = BRepRunner.Execute(testCase); // this is the line to step into
            }
            catch (Exception e)
            {
                run.Error = e;
            }
            watch.Stop();
            run.ElapsedMilliseconds = watch.ElapsedMilliseconds;

            Trace.WriteLine($"AutoDebug {testCase.Name}: {testCase.Operation} -> {run.Describe()}"
                + $" ({run.ElapsedMilliseconds} ms, valid={run.IsValid})");
            // the very same summary the regression test compares against its baseline
            Trace.WriteLine(BRepSummary.Describe(testCase, run).ToText());
        }

        /// <summary>
        /// Filter the escape key for the modelling property page
        /// </summary>
        /// <param name="msg"></param>
        /// <param name="keyData"></param>
        /// <returns></returns>
        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            Keys nmKeyData = (Keys)((int)keyData & 0x0FFFF);
            CADability.Substitutes.KeyEventArgs e = new CADability.Substitutes.KeyEventArgs((CADability.Substitutes.Keys)keyData);
            if (nmKeyData == Keys.Escape)
            {
                if (modellingPropertyEntries.OnEscape()) return true;
            }
            return base.ProcessCmdKey(ref msg, keyData);
        }
        /// <summary>
        /// Called when CADability is idle. We use it to save the current project data to a temp file in case of a crash
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        // slowdown OnIdle polling:
        readonly Stopwatch _idleSw = Stopwatch.StartNew();
        const int MinIdleCheckMs = 250;
        bool _idleBusy;
        void OnIdle(object sender, EventArgs e)
        {
            if (_idleBusy) return;
            if (_idleSw.ElapsedMilliseconds < MinIdleCheckMs) return;

            _idleBusy = true;
            _idleSw.Restart();
            try
            {
                if (projectionChanged)
                {
                    projectionChanged = false;
                    modellingPropertyEntries.OnProjectionChanged(); // to update the feedback objects, which are projection dependant
                }
                if (modifiedSinceLastAutosave && (DateTime.Now - lastSaved).TotalMinutes > 2)
                {
                    modifiedSinceLastAutosave = false;
                    string path = System.IO.Path.GetTempPath();
                    path = System.IO.Path.Combine(path, "ShapeIt");
                    DirectoryInfo dirInfo = Directory.CreateDirectory(path);
                    string currentFileName = CadFrame.Project.FileName;
                    if (string.IsNullOrEmpty(CadFrame.Project.FileName)) path = System.IO.Path.Combine(path, "noname.cdb.json");
                    else
                    {
                        string fileName = System.IO.Path.GetFileNameWithoutExtension(CadFrame.Project.FileName);
                        if (fileName.EndsWith(".cdb")) fileName = System.IO.Path.GetFileNameWithoutExtension(fileName); // we usually have two extensions: .cdb.json
                        path = System.IO.Path.Combine(path, fileName + "_.cdb.json");
                    }
                    CadFrame.Project.WriteToFile(path);
                    CadFrame.Project.FileName = currentFileName; // Project.WriteToFile changes the Project.FileName, restore the current name
                    lastSaved = DateTime.Now;
                }
            }
            finally { _idleBusy = false; }
        }
        void OnProjectClosed(Project theProject, IFrame theFrame)
        {
            // manage autosave OnIdle, remove autosaved files
        }
        /// <summary>
        /// When a new or exisiting project has been opened
        /// </summary>
        /// <param name="theProject"></param>
        /// <param name="theFrame"></param>
        private void OnProjectOpened(Project theProject, IFrame theFrame)
        {
            theProject.ProjectModifiedEvent += (Project sender) =>
            {   // register modifications of the project to manage autosave
                if (sender == theProject) modifiedSinceLastAutosave = true;
            };
        }
        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);

            // this is for recording the session with 1280x720 pixel. 
            this.Size = new Size(1294, 727);

        }
        /// <summary>
        /// Give the user a chance to save the modified project
        /// </summary>
        /// <param name="e"></param>
        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            if (!CadFrame.Project.SaveModified()) e.Cancel = true;
            base.OnFormClosing(e);
        }
        public override bool OnCommand(string MenuId)
        {
            // forward to modellingPropertyEntries first
            if (modellingPropertyEntries.OnCommand(MenuId)) return true;
            if (MenuId == "MenuId.App.Exit")
            {   // this command cannot be handled by CADability.dll
                Application.Exit();
                return true;
            }
            else if (MenuId == "MenuId.Debug")
            {
                AutoDebug();
                return true;
            }
            else return base.OnCommand(MenuId);
        }
        public override bool OnUpdateCommand(string MenuId, CommandState CommandState)
        {
            // forward to modellingPropertyEntries first
            if (modellingPropertyEntries.OnUpdateCommand(MenuId, CommandState)) return true;
            return base.OnUpdateCommand(MenuId, CommandState);
        }
        public override void OnSelected(MenuWithHandler selectedMenuItem, bool selected)
        {
            modellingPropertyEntries.OnSelected(selectedMenuItem, selected);
            base.OnSelected(selectedMenuItem, selected);
        }
        private void AutoDebug()
        {
            Solid box = Make3D.MakeBox(new GeoPoint(0, 0, 0), new GeoVector(10, 0, 0), new GeoVector(0, 10, 0), new GeoVector(0, 0, 10));
            Shell shell = box.Shell;
            shell.AddAndRemoveFaces([], [shell.Faces[0]]);
            shell.Thicken(1.0, 1.0);
        }
#if DEBUG
        private static Random rnd = new Random();
        private GeoVector RandomVector(double len)
        {
            double fx = 1.0, fy = 1.0, fz = 1.0;
            if (rnd.NextDouble() < 0.5) fx = -fx;
            if (rnd.NextDouble() < 0.5) fy = -fy;
            if (rnd.NextDouble() < 0.5) fz = -fz;
            return len * (new GeoVector(fx * rnd.NextDouble(), fy * rnd.NextDouble(), fz * rnd.NextDouble())).Normalized;
        }
        private double RandomDouble(double min, double max)
        {
            return min + (max - min) * rnd.NextDouble();
        }
        /// <summary>
        /// Here we can add some debug code
        /// </summary>


#endif
    }
}
