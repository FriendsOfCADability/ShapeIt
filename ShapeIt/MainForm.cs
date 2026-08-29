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
#if DEBUG
            // to make debugging easier, we disable the parallelization in MathNet.Numerics, which is used for some boolean
            // operations. This avoids the debugger message: "Cannot evaluate expression since the function evaluation requires
            // all threads to run.". In release mode, we keep the parallelization enabled for better performance.
            MathNet.Numerics.Control.MaxDegreeOfParallelism = 1;
#endif
            string fileName = "";
            bool debugBRep = false;
            bool nofile = false;
            bool debugRPC = false;
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
            if (CadFrame.Project!=null) CadFrame.Project.IsModified = false; // to avoid messagebox asking for saving modified project
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
#if DEBUG
            else if (MenuId == "MenuId.Debug")
            {
                Debug();
                return true;
            }
#endif
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
        private void DebugX()
        {
            Model model = CadFrame.Project.GetActiveModel();
            Style stl = CadFrame.Project.StyleList.GetDefault(CADability.Attribute.Style.EDefaultFor.Solids);
            for (int i = 0; i < 64; i++)
            {
                double fx = 1.0, fy = 1.0, fz = 1.0;
                if ((i & 0x1) != 0) fx = -fx;
                if ((i & 0x2) != 0) fy = -fy;
                if ((i & 0x4) != 0) fz = -fz;
                GeoVector dir = RandomVector(RandomDouble(40, 60));
                GeoPoint center = GeoPoint.Origin + dir;
                Solid sld = Make3D.MakeSphere(center, 10 + 20 * rnd.NextDouble());
                ModOp rotate = ModOp.Rotate(center, RandomVector(1.0), SweepAngle.Deg(rnd.NextDouble() * 360.0));
                sld.Modify(rotate);
                sld.Style = stl;
                model.Add(sld);
            }
            Solid mainSphere = Make3D.MakeSphere(GeoPoint.Origin, 50);
            mainSphere.Style = stl;
            model.Add(mainSphere);
        }


        private void DebugY()
        {
            List<CADability.GeoObject.Solid> slds = new List<CADability.GeoObject.Solid>();
            CADability.GeoObject.Solid sldbig = null;
            Face cone = null;
            Face upper = null;
            Face lower = null;
            foreach (CADability.GeoObject.IGeoObject go in CadFrame.Project.GetActiveModel().AllObjects)
            {
                if (go is CADability.GeoObject.Solid sld)
                {
                    if (sld.Volume(0.1) > 500000) sldbig = sld;
                    else slds.Add(sld);
                }
            }

            var rng = new Random(71);
            var order = Enumerable.Range(0, slds.Count).ToArray();

            // Fisher–Yates
            for (int i = slds.Count - 1; i > 0; i--)
            {
                int j = rng.Next(i + 1);
                (order[i], order[j]) = (order[j], order[i]);
            }


            if (slds.Count > 1 && sldbig != null)
            {

                for (int i = 0; i < slds.Count; i++)
                {
                    Solid[] res = NewBooleanOperation.Subtract(sldbig, slds[order[i]]);
                    if (res != null && res.Length >= 1) sldbig = res[0];
                    else { }
                    //System.Diagnostics.Debug.WriteLine("Unite " + i.ToString() + " -> " + sldbig.GetExtent(0.0).Size.ToString("F0"));
                }
            }
        }
        private void Debug()
        {
            GeoObjectList l = modellingPropertyEntries.SelectedObjects;
            if (l.Count == 1)
            {
                if (l[0] is Polyline pl && pl.IsRectangle)
                {
                    GeoPoint sp = pl.StartPoint;
                    GeoVector dx = pl.GetPoint(1) - sp;
                    GeoVector dy = pl.GetPoint(2) - pl.GetPoint(1);
                    GeoVector n = (dx ^ dy).Normalized;
                    double m = (dx.Length + dy.Length) / 4.0;
                    m *= 0.4;
                    Random rnd = new Random();
                    // Handle rectangle case
                    int xc = 10, yc = 10;
                    GeoPoint[,] poles = new GeoPoint[xc, yc];
                    for (int i = 0; i < xc; i++)
                    {
                        for (int j = 0; j < yc; j++)
                        {
                            GeoPoint p = sp + ((double)i / (xc - 1)) * dx + ((double)j / (yc - 1)) * dy;

                            // lx, ly: 0 am Rand, 1 in der Mitte (linear)
                            double lx = 1.0 - Math.Abs(2.0 * i / (xc - 1) - 1.0);
                            double ly = 1.0 - Math.Abs(2.0 * j / (yc - 1) - 1.0);

                            double rm = (1 + rnd.NextDouble()) * 0.8 * m;
                            double t = rm * Math.Sqrt(1 - (1 - lx) * (1 - lx)) * Math.Sqrt(1 - (1 - ly) * (1 - ly));
                            p = p + t * n;
                            poles[i, j] = p;
                        }
                    }
                    double[] uKnots = new double[xc + 3 + 1];
                    double[] vKnots = new double[yc + 3 + 1];
                    double dk = 1.0 / (xc - 3);
                    for (int i = 0; i < uKnots.Length; i++)
                    {
                        double nom = i - 3;
                        if (nom < 0) nom = 0;
                        if (nom > xc - 3) nom = xc - 3;
                        uKnots[i] = nom * dk;
                    }
                    dk = 1.0 / (yc - 3);
                    for (int i = 0; i < vKnots.Length; i++)
                    {
                        double nom = i - 3;
                        if (nom < 0) nom = 0;
                        if (nom > yc - 3) nom = yc - 3;
                        vKnots[i] = nom * dk;
                    }
                    NurbsSurface ns = new NurbsSurface(poles, null, uKnots, vKnots, 3, 3, false, false);
                    Face f1 = Face.MakeFace(ns, new BoundingRect(0, 0, 1, 1));
                    Face f2 = Face.MakeFace(new GeoObjectList(pl));
                    Shell[] shs = Make3D.SewFaces(new Face[] { f1, f2 });
                    if (shs.Length == 1 && shs[0].OpenEdges.Length == 0)
                    {
                        Solid sld = Solid.MakeSolid(shs[0]);
                        CadFrame.Project.SetDefaults(sld);
                        pl.Owner.Add(sld);
                    }
                }
            }
            else if (l.Count == 2)
            {
                int vnum = 5;
                if (l.Count != 2) return;
                BSpline? b1 = l[0] as BSpline;
                BSpline? b2 = l[1] as BSpline;
                if (b1 == null || b2 == null) return;
                if ((b1 as ICurve).StartPoint.x > (b2 as ICurve).StartPoint.x) (b1, b2) = (b2, b1);
                if (b1.Poles.Length != b2.Poles.Length) return;
                GeoPoint[,] poles = new GeoPoint[b1.Poles.Length, vnum];
                for (int i = 0; i < poles.GetLength(0); i++)
                {
                    GeoPoint p1 = b1.Poles[i];
                    GeoPoint p2 = b2.Poles[i];
                    double d = p1 | p2;
                    double r = d / 2;
                    double step = d / (vnum - 1);
                    double da = Math.PI / (vnum - 1);
                    GeoPoint cnt = new GeoPoint(p1, p2);
                    Plane arcPlane = new Plane(cnt, p1 - p2, GeoVector.ZAxis);
                    for (int j = 0; j < vnum; j++)
                    {
                        if (j == 0) poles[i, j] = p1;
                        else if (j == vnum - 1) poles[i, j] = p2;
                        else
                        {
                            double rrnd = rnd.NextDouble() / 2 + 0.5;
                            poles[i, j] = arcPlane.ToGlobal(new GeoPoint2D(rrnd * r * Math.Cos(da * j), rrnd * 0.7 * r * Math.Sin(da * j)));
                        }
                    }
                }
                double[] uKnots = new double[b1.Poles.Length + 3 + 1];
                for (int i = 0; i < uKnots.Length; i++)
                {
                    int ind = i - 3;
                    if (ind < 0) ind = 0;
                    if (ind >= b1.Knots.Length) ind = b1.Knots.Length - 1;
                    uKnots[i] = b1.Knots[ind];
                }
                double[] vKnots = new double[vnum + 3 + 1];
                double dk = 1.0 / (vnum - 3);
                for (int i = 0; i < vKnots.Length; i++)
                {
                    double nom = i - 3;
                    if (nom < 0) nom = 0;
                    if (nom > vnum - 3) nom = vnum - 3;
                    vKnots[i] = nom * dk;
                }
                NurbsSurface ns = new NurbsSurface(poles, null, uKnots, vKnots, 3, 3, false, false);
                Face f1 = Face.MakeFace(ns, new BoundingRect(0, 0, 1, 1));
                ICurve c1 = ns.FixedU(0.0, 0, 1);
                ICurve c2 = ns.FixedU(1.0, 0, 1);
                Plane pl1 = new Plane(c1.StartPoint, -c1.StartDirection);
                Plane pl2 = new Plane(c1.EndPoint, c1.EndDirection);

                Face f2 = Face.MakeFace(new GeoObjectList(c1 as IGeoObject, Line.TwoPoints(c1.EndPoint, c1.StartPoint)));
                Face f3 = Face.MakeFace(new GeoObjectList(c2 as IGeoObject, Line.TwoPoints(c2.EndPoint, c2.StartPoint)));

                ICurve c3 = ns.FixedV(0, 0, 1);
                ICurve c4 = ns.FixedV(1.0, 0, 1);
                c3.Reverse();
                Face f4 = Face.MakeFace(new GeoObjectList(c3 as IGeoObject, Line.TwoPoints(c3.EndPoint, c4.StartPoint), c4 as IGeoObject, Line.TwoPoints(c4.EndPoint, c3.StartPoint)));
                Shell[] shs = Make3D.SewFaces(new Face[] { f1, f2, f3, f4 });
                if (shs.Length == 1 && shs[0].OpenEdges.Length == 0)
                {
                    Solid sld = Solid.MakeSolid(shs[0]);
                    CadFrame.Project.SetDefaults(sld);
                    b1.Owner.Add(sld);
                }
            }
        }
#endif
    }
}
