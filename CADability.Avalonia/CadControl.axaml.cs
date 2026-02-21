using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Text;
using System.Xml;


namespace CADability.Avalonia
{
    public partial class CadControl : UserControl, ICommandHandler
    {
        private CadFrame cadFrame; // the frame, which knows about the views, the ControlCenter (PropertiesExplorer), the menu
        // private ProgressForm progressForm;
        public CadControl()
        {
            InitializeComponent(); // makes the cadCanvas and the propertiesExplorer
            // KeyPreview = true; // used to filter the escape key (and maybe some more?)
            // cadFrame = new CadFrame(propertiesExplorer, cadCanvas, this);
            cadFrame = new CadFrame(propertiesExplorer, cadCanvas, this);
            // cadFrame.ProgressAction = (show, percent, title) => { this.ProgressForm.ShowProgressBar(show, percent, title); };
            cadCanvas.Frame = cadFrame;
            propertiesExplorer.Frame = cadFrame;
            // show this menu in the MainForm
            // MenuWithHandler[] mainMenu = MenuResource.LoadMenuDefinition("SDI Menu", true, cadFrame);
            // MenuManager.MakeMainMenu(mainMenu, dockPanel, this); TODO moved to Main Window

            // MainMenuStrip = MenuManager.MakeMainMenu(mainMenu);
            // Controls.Add(MainMenuStrip);
            // cadFrame.FormMenu = MainMenuStrip;
            // open an existing Project or create a new one
            // ToolBars.CreateOrRestoreToolbars(topToolStripContainer, cadFrame);
            // Application.Idle += new EventHandler(OnIdle); // update the toolbars (menus are updated when they popup)
        }
        /// <summary>
        /// Resets the main menu of the form. After MenuResource.SetMenuResource has been loaded, the mainmenu in CADability
        /// is changed to the new resource. In order to set this menu in the form, call this method.
        /// </summary>
        /// <param name="debuggerPlaygroundClass">if not null, specifies an additional menue class, which will be created by reflection</param>
        // TODO
        // public void ResetMainMenu(string debuggerPlaygroundClass)
        // {
        //     MenuWithHandler[] mainMenu = MenuResource.LoadMenuDefinition("SDI Menu", true, cadFrame);
        //     if (!string.IsNullOrEmpty(debuggerPlaygroundClass))
        //     {
        //         // in the following lines a "DebuggerPlayground" object is created via reflection. This class is a playground to write testcode
        //         // which is not included in the sources. This is why it is constructed via reflection, there is no need to have this class in the project.
        //         Type dbgplygnd = Type.GetType(debuggerPlaygroundClass, false);
        //         if (dbgplygnd != null)
        //         {
        //             MethodInfo connect = dbgplygnd.GetMethod("Connect");
        //             if (connect != null) mainMenu = connect.Invoke(null, new object[] { cadFrame, mainMenu }) as MenuWithHandler[];
        //         }
        //     }

        //     if (this.MainMenuStrip != null)
        //     {
        //         this.Controls.Remove(this.MainMenuStrip);
        //         this.MainMenuStrip.Dispose();
        //     }
        //     MainMenuStrip = MenuManager.MakeMainMenu(mainMenu);
        //     this.Controls.Add(MainMenuStrip);
        //     cadFrame.FormMenu = MainMenuStrip;
        // }
        // Access the components of the MainForm from the CadFrame.
        // TODO
        // public void SetToolbar(XmlNode definition)
        // {
        //     ToolBars.SetToolBar(topToolStripContainer, cadFrame, definition);
        // }
        // TODO
        // public ProgressForm ProgressForm
        // {
        //     get
        //     {
        //         if (progressForm == null)
        //         {
        //             progressForm = new ProgressForm
        //             {
        //                 TopLevel = true,
        //                 Owner = this,
        //                 Visible = false
        //             };
        //         }
        //         return progressForm;
        //     }
        // }
        public PropertiesExplorer PropertiesExplorer => propertiesExplorer;
        public CadCanvas CadCanvas => cadCanvas;
        public CadFrame CadFrame => cadFrame;

        // slowdown OnIdle polling:
        readonly Stopwatch _idleSw = Stopwatch.StartNew();
        const int MinIdleCheckMs = 250;
        bool _idleBusy;
        // TODO Avalonia
        // private void OnIdle(object sender, EventArgs e)
        // {
        //     if (_idleBusy) return;
        //     if (_idleSw.ElapsedMilliseconds < MinIdleCheckMs) return;

        //     _idleBusy = true;
        //     _idleSw.Restart();
        //     try { ToolBars.UpdateCommandState(topToolStripContainer.TopToolStripPanel); }
        //     finally { _idleBusy = false; }
        // }
        // TODO
        // protected override void OnClosing(CancelEventArgs e)
        // {   // maybe we need to save the project
        //     ToolBars.SaveToolbarPositions(topToolStripContainer);
        //     Settings.SaveGlobalSettings();
        //     ToolStripManager.SaveSettings(this); // save the positions of the toolbars (doesn't work correctly)
        //     base.OnClosing(e);
        // }
        // TODO
        // protected override void OnFormClosed(FormClosedEventArgs e)
        // {
        //     cadFrame.Dispose();
        //     MainMenuStrip.Dispose();
        //     this.Dispose();
        //     cadCanvas.Dispose();
        //     propertiesExplorer.Dispose();
        //     topToolStripContainer.Dispose();
        //     splitContainer.Dispose();
        //     if (progressForm != null) progressForm.Dispose();

        //     MainMenuStrip = null;
        //     cadFrame = null;
        //     cadCanvas = null;
        //     propertiesExplorer = null;
        //     topToolStripContainer = null;
        //     splitContainer = null;
        //     progressForm = null;

        //     base.OnFormClosed(e);
        // }

        protected override void OnLoaded(RoutedEventArgs e)
        {
            var topLevel = TopLevel.GetTopLevel(this)!;
            topLevel.KeyDown += OnKeyDown;
            base.OnLoaded(e);
        }

        private Substitutes.Keys Subst(Key key)
        {
            return Enum.TryParse(key.ToString(), out Substitutes.Keys res) ? res : Substitutes.Keys.None;
        }

        private void OnKeyDown(object sender, KeyEventArgs keyEvent)
        {
            Key key = keyEvent.Key;
            bool preProcess = key >= Key.F1 && key <= Key.F24;
            preProcess = preProcess || (key == Key.Escape);
            preProcess = preProcess || (key == Key.Up) || (key == Key.Down);
            preProcess = preProcess || (key == Key.Tab) || (key == Key.Enter);
            preProcess = preProcess || keyEvent.KeyModifiers.HasFlag(KeyModifiers.Control) || keyEvent.KeyModifiers.HasFlag(KeyModifiers.Alt); // menu shortcut
            if (propertiesExplorer.EntryWithTextBox == null) preProcess |= (key == Key.Delete); // the delete key is preferred by the textbox, if there is one
            Substitutes.KeyEventArgs e = new Substitutes.KeyEventArgs(Subst(key));
            if (preProcess)
            {
                e.Handled = false;
                cadFrame.PreProcessKeyDown(e);
                if (e.Handled) {
                    keyEvent.Handled = true;
                    return;
                }
            }
            CadFrame.PreProcessKeyDown(e);
            if (e.Handled) {
                keyEvent.Handled = true;
                return;
            }
            base.OnKeyDown(keyEvent);
        }

        public virtual bool OnCommand(string MenuId)
        {
            return false;
        }

        public virtual bool OnUpdateCommand(string MenuId, CommandState CommandState)
        {
            return false;
        }

        public virtual void OnSelected(MenuWithHandler selectedMenuItem, bool selected)
        {

        }

    }
}
