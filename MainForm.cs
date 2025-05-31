using CADability.Forms;
using CADability;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Reflection;
using System.Xml;
using CADability.UserInterface;
using System.Runtime.InteropServices;
using static ShapeIt.MainForm;
using System.IO;
using CADability.Actions;
using System.Drawing.Imaging;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;
using System.Xml.Linq;

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
            var timer = new Timer();
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
            logoBox.Image = Properties.Resources.ShapeIt2;
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
        public MainForm(string[] args) : base(args)
        {   // interpret the command line arguments as a name of a file, which should be opened

            //InitializeComponent();
            ShowLogo();

            this.Icon = Properties.Resources.Icon;

            string fileName = "";
            for (int i = 0; i < args.Length; i++)
            {
                if (!args[i].StartsWith("-"))
                {
                    fileName = args[i];
                    break;
                }
            }
            Project toOpen = null;
            if (!String.IsNullOrWhiteSpace(fileName))
            {
                try
                {
                    toOpen = Project.ReadFromFile(fileName);
                }
                catch { }
            }
            if (toOpen == null) CadFrame.GenerateNewProject();
            else CadFrame.Project = toOpen;
            this.Text = "ShapeIt with CADability";
            Settings.GlobalSettings.SetValue("Construct.3D_Delete2DBase", false); // we start face extrus with newly generated face, which has no owner
            bool exp = Settings.GlobalSettings.GetBoolValue("Experimental.TestNewContextMenu", false);
            bool tst = Settings.GlobalSettings.GetBoolValue("ShapeIt.Initialized", false);
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
            Assembly ThisAssembly = Assembly.GetExecutingAssembly();
            System.IO.Stream str;
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
                XmlNode mainMenu = menuDocument.SelectSingleNode("Menus/MainMenu");
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
                MenuResource.SetMenuResource(menuDocument);
                ResetMainMenu(null);
            }

            lastSaved = DateTime.Now;
            AppDomain.CurrentDomain.UnhandledException += (sender, exobj) =>
            {
                // exobj.ExceptionObject as Exception;
                try
                {
                    string path = Path.GetTempPath();
                    path = Path.Combine(path, "ShapeIt");
                    DirectoryInfo dirInfo = Directory.CreateDirectory(path);
                    string currentFileName = CadFrame.Project.FileName;
                    if (string.IsNullOrEmpty(CadFrame.Project.FileName))
                    {
                        path = Path.Combine(path, "crash_" + DateTime.Now.ToString("yyMMddHHmm") + ".cdb.json");
                        currentFileName = "unknown";
                    }
                    else
                    {
                        string crashFileName = Path.GetFileNameWithoutExtension(CadFrame.Project.FileName);
                        if (crashFileName.EndsWith(".cdb")) crashFileName = Path.GetFileNameWithoutExtension(crashFileName); // we usually have two extensions: .cdb.json
                        path = Path.Combine(path, crashFileName + "_X.cdb.json");
                    }
                    CadFrame.Project.WriteToFile(path);
                    File.WriteAllText(Path.Combine(Path.GetTempPath(), @"ShapeIt\Crash.txt"), currentFileName + "\n" + path);
                }
                catch (Exception) { };
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
                string crashPath = Path.Combine(Path.GetTempPath(), @"ShapeIt\Crash.txt");
                if (File.Exists(crashPath))
                {
                    string[] lines = File.ReadAllLines(Path.Combine(Path.GetTempPath(), @"ShapeIt\Crash.txt"));
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
            }
            base.OnActivated(e);
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
        void OnIdle(object sender, EventArgs e)
        {
            if (projectionChanged)
            {
                projectionChanged = false;
                modellingPropertyEntries.OnProjectionChanged(); // to update the feedback objects, which are projection dependant
            }
            if (modifiedSinceLastAutosave && (DateTime.Now - lastSaved).TotalMinutes > 2)
            {
                modifiedSinceLastAutosave = false;
                string path = Path.GetTempPath();
                path = Path.Combine(path, "ShapeIt");
                DirectoryInfo dirInfo = Directory.CreateDirectory(path);
                string currentFileName = CadFrame.Project.FileName;
                if (string.IsNullOrEmpty(CadFrame.Project.FileName)) path = Path.Combine(path, "noname.cdb.json");
                else
                {
                    string fileName = Path.GetFileNameWithoutExtension(CadFrame.Project.FileName);
                    if (fileName.EndsWith(".cdb")) fileName = Path.GetFileNameWithoutExtension(fileName); // we usually have two extensions: .cdb.json
                    path = Path.Combine(path, fileName + "_.cdb.json");
                }
                CadFrame.Project.WriteToFile(path);
                CadFrame.Project.FileName = currentFileName; // Project.WriteToFile changes the Project.FileName, restore the current name
                lastSaved = DateTime.Now;
            }
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
        /// <summary>
        /// Here we can add some debug code
        /// </summary>
        private void Debug()
        {
            CADability.GeoObject.Solid sld1 = null;
            CADability.GeoObject.Solid sld2 = null;
            CADability.GeoObject.Solid sld3 = null;
            foreach (CADability.GeoObject.IGeoObject go in CadFrame.Project.GetActiveModel().AllObjects)
            {
                if (go is CADability.GeoObject.Solid sld)
                {
                    if (sld1 == null) sld1 = sld;
                    else if (sld2 == null) sld2 = sld;
                    else sld3 = sld;
                }
            }
            if (sld1 != null)
            {
                sld1.Shells[0].GetOffsetNew(0.50);
                //CADability.GeoObject.Solid[] res = CADability.GeoObject.Solid.Subtract(sld2, sld1);
                //bool ok = res[0].Shells[0].CheckConsistency();
            }
        }
#endif
    }
}
