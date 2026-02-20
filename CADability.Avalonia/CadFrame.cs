using CADability.GeoObject;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
// using System.Drawing;
using System.Drawing.Printing;
using System.IO;
using System.Threading;
using Action = CADability.Actions.Action;

namespace CADability.Avalonia
{
    /* TODO: change IFrame for "KernelOnly" KO
     * Step by Step implement missing methods (copy implementation from SingleDocumentFrame)
     * and remove interface methods from IFrame with "#if KO" ...
     */
    /// <summary>
    /// Implementation of the abstract FrameImpl, doing things, you cannot do in .NET Core
    /// </summary>
    public class CadFrame : FrameImpl, IUIService
    {
        #region PRIVATE FIELDS

        private ICommandHandler commandHandler;

        private const string ClipFormat = "CADability.GeoObjectList.Json";

        // Optional: COM-freie Verfügbarkeitsprüfung (robuster im Idle)
        // TODO reimplement in Avalonia
        // static class NativeClipboard
        // {
        //     [DllImport("user32.dll", CharSet = CharSet.Unicode)]
        //     private static extern uint RegisterClipboardFormat(string lpszFormat);

        //     [DllImport("user32.dll")]
        //     private static extern bool IsClipboardFormatAvailable(uint format);

        //     private static uint _fmtId;
        //     public static bool HasMyFormat()
        //     {
        //         if (_fmtId == 0) _fmtId = RegisterClipboardFormat(ClipFormat);
        //         return IsClipboardFormatAvailable(_fmtId);
        //     }
        // }

        #endregion PRIVATE FIELDS

        #region PUBLIC PROPERTIES

        /// <summary>
        /// The parent form menu
        /// </summary>
        // TODO reimplement in Avalonia
        // public MenuStrip FormMenu { set; private get; }

        /// <summary>
        /// Action that delegate the progress.
        /// In this way we can use a custom progress ui.
        /// </summary>
        public Action<bool, double, string> ProgressAction { get; set; }

        #endregion PUBLIC PROPERTIES

        /// <summary>
        /// Constructor without form dependency.
        /// </summary>
        /// <param name="propertiesExplorer"></param>
        /// <param name="cadCanvas"></param>
        /// <param name="commandHandler"></param>
        // TODO reimplement in Avalonia
        // public CadFrame(PropertiesExplorer propertiesExplorer, CadCanvas cadCanvas, ICommandHandler commandHandler)
        //     : base(propertiesExplorer, cadCanvas)
        // {
        //     this.commandHandler = commandHandler;
        // }
        /// <summary>
        /// Constructor without form dependency.
        /// </summary>
        /// <param name="cadCanvas"></param>
        /// <param name="commandHandler"></param>
        //
        // TODO add propertyExplorer to parameters and add to base call
        // allows cadFrame.ControlCenter to work
        public CadFrame(PropertiesExplorer propertiesExplorer, CadCanvas cadCanvas, ICommandHandler commandHandler)
            : base(propertiesExplorer, cadCanvas)
        {
            this.commandHandler = commandHandler;
        }

        #region FrameImpl override

        public override bool OnCommand(string MenuId)
        {
            if (commandHandler != null && commandHandler.OnCommand(MenuId)) return true;
            return base.OnCommand(MenuId);
        }

        public override bool OnUpdateCommand(string MenuId, CommandState CommandState)
        {
            if (commandHandler != null && commandHandler.OnUpdateCommand(MenuId, CommandState)) return true;
            return base.OnUpdateCommand(MenuId, CommandState);
        }

        public override void UpdateMRUMenu(string[] mruFiles)
        {
            // if (this.FormMenu != null)
            // {
                // TODO reimplement in Avalonia
                // foreach (ToolStripMenuItem mi in this.FormMenu.Items)
                // {
                //     UpdateMRUMenu(mi, mruFiles);
                // }
            // }
        }

        // TODO reimplement in Avalonia
        // private void UpdateMRUMenu(ToolStripMenuItem mi, string[] mruFiles)
        // {
        //     if (mi.HasDropDownItems)
        //     {
        //         foreach (ToolStripItem tsi in mi.DropDownItems)
        //         {
        //             if (tsi is ToolStripMenuItem mmi) UpdateMRUMenu(mmi, mruFiles);
        //         }
        //     }
        //     else
        //     {
        //         if (mi is MenuItemWithHandler mid)
        //         {
        //             if (mid.Tag is MenuWithHandler mwh)
        //             {
        //                 string MenuId = mwh.ID;
        //                 if (MenuId.StartsWith("MenuId.File.Mru.File"))
        //                 {
        //                     string filenr = MenuId.Substring("MenuId.File.Mru.File".Length);
        //                     try
        //                     {
        //                         int n = int.Parse(filenr);
        //                         if (n <= mruFiles.Length && n > 0)
        //                         {
        //                             string[] parts = mruFiles[mruFiles.Length - n].Split(';');
        //                             if (parts.Length > 1)
        //                                 mid.Text = parts[0];
        //                         }
        //                     }
        //                     catch (FormatException) { }
        //                     catch (OverflowException) { }
        //                 }
        //             }
        //         }
        //     }
        // }

        #endregion FrameImpl override

        #region IUIService implementation
        // TODO reimplement in Avalonia
        public override IUIService UIService => this;
        GeoObjectList IUIService.GetDataPresent(object data)
        {
            throw new NotImplementedException();
        }
        // {
        //     if (data is IDataObject idata)
        //     {
        //         if (idata.GetDataPresent(System.Windows.Forms.DataFormats.Serializable))
        //         {
        //             return idata.GetData(System.Windows.Forms.DataFormats.Serializable) as GeoObjectList;
        //         }
        //     }
        //     return null;
        // }
        // Substitutes.Keys IUIService.ModifierKeys => (Substitutes.Keys)Control.ModifierKeys;
        // Substitutes.Point IUIService.CurrentMousePosition => Subst(Control.MousePosition);
        Substitutes.Keys IUIService.ModifierKeys
        {
            get { throw new NotImplementedException(); }
        }
        Substitutes.Point IUIService.CurrentMousePosition
        {
            get { throw new NotImplementedException(); }
        }

        private Substitutes.Point Subst(System.Drawing.Point mousePosition)
        {
            return new Substitutes.Point(mousePosition.X, mousePosition.Y);
        }

        private static Dictionary<string, string> directories = new Dictionary<string, string>();
        // TODO reimplement in Avalonia
        Substitutes.DialogResult IUIService.ShowOpenFileDlg(string id, string title, string filter, ref int filterIndex, out string fileName)
        {
            throw new NotImplementedException();
        }
        // {
        //     OpenFileDialog openFileDialog = new OpenFileDialog();
        //     openFileDialog.Filter = filter;
        //     openFileDialog.FilterIndex = filterIndex;
        //     if (!string.IsNullOrWhiteSpace(title)) openFileDialog.Title = title;
        //     if (!string.IsNullOrWhiteSpace(id) && directories.TryGetValue(id, out string directory))
        //     {
        //         openFileDialog.InitialDirectory = directory;
        //     }
        //     else
        //     {
        //         openFileDialog.RestoreDirectory = true;
        //     }

        //     Substitutes.DialogResult res = (Substitutes.DialogResult)openFileDialog.ShowDialog(Application.OpenForms[0]);
        //     if (res == Substitutes.DialogResult.OK)
        //     {
        //         filterIndex = openFileDialog.FilterIndex;
        //         fileName = openFileDialog.FileName;
        //         if (!string.IsNullOrWhiteSpace(id))
        //         {
        //             directory = System.IO.Path.GetDirectoryName(fileName);
        //             directories[id] = directory;
        //         }
        //     }
        //     else
        //     {
        //         fileName = null;
        //     }
        //     return res;
        // }
        // TODO reimplement in Avalonia
        Substitutes.DialogResult IUIService.ShowSaveFileDlg(string id, string title, string filter, ref int filterIndex, ref string fileName)
        {
            throw new NotImplementedException();
        }
        // {
        //     SaveFileDialog saveFileDialog = new SaveFileDialog();
        //     saveFileDialog.Filter = filter;
        //     saveFileDialog.FilterIndex = filterIndex;
        //     if (!string.IsNullOrWhiteSpace(title)) saveFileDialog.Title = title;
        //     if (!string.IsNullOrWhiteSpace(id) && directories.TryGetValue(id, out string directory))
        //     {
        //         saveFileDialog.InitialDirectory = directory;
        //     }
        //     else
        //     {
        //         saveFileDialog.RestoreDirectory = true;
        //     }
        //     if (!string.IsNullOrWhiteSpace(fileName))
        //     {
        //         saveFileDialog.FileName = fileName;
        //     }
        //     Substitutes.DialogResult res = (Substitutes.DialogResult)saveFileDialog.ShowDialog(Application.OpenForms[0]);
        //     if (res == Substitutes.DialogResult.OK)
        //     {
        //         filterIndex = saveFileDialog.FilterIndex;
        //         fileName = saveFileDialog.FileName;
        //         if (!string.IsNullOrWhiteSpace(id))
        //         {
        //             directory = System.IO.Path.GetDirectoryName(fileName);
        //             directories[id] = directory;
        //         }
        //     }
        //     return res;
        // }
        // TODO reimplement in Avalonia
        Substitutes.DialogResult IUIService.ShowMessageBox(string text, string caption, Substitutes.MessageBoxButtons buttons)
        {
            throw new NotImplementedException();
        }
        // {
        //     return (Substitutes.DialogResult)MessageBox.Show(Application.OpenForms[0], text, caption, (System.Windows.Forms.MessageBoxButtons)buttons);
        // }
        // TODO reimplement in Avalonia
        Substitutes.DialogResult IUIService.ShowColorDialog(ref Substitutes.Color color)
        {
            throw new NotImplementedException();
        }
        // {
        //     ColorDialog colorDialog = new ColorDialog();
        //     colorDialog.Color = Drawing(color);
        //     Substitutes.DialogResult dlgres = (Substitutes.DialogResult)colorDialog.ShowDialog(Application.OpenForms[0]);
        //     color = Subst(colorDialog.Color);
        //     return dlgres;
        // }

        private Substitutes.Color Drawing(Substitutes.Color color)
        {
            return color;
            // return Substitutes.Color.FromArgb(color.ToArgb());
        }

        private Substitutes.Color Subst(Substitutes.Color color) // TODO correct argument type
        {
            return color;
            // return Substitutes.Color.FromArgb(color.ToArgb());
        }

        // TODO reimplement in Avalonia
        void IUIService.ShowProgressBar(bool show, double percent, string title)
        {
            throw new NotImplementedException();
        }
        // {
        //     this.ProgressAction?.Invoke(show, percent, title);
        // }
        /// <summary>
        /// Returns a bitmap from the specified embeded resource. the name is in the form filename:index
        /// </summary>
        /// <param name="name"></param>
        /// <returns></returns>
        // TODO reimplement in Avalonia
        object IUIService.GetBitmap(string name)
        {
            throw new NotImplementedException();
        }
        // {
        //     string[] parts = name.Split(':');
        //     if (parts.Length == 2)
        //     {
        //         ImageList il = BitmapTable.GetImageList(parts[0], 15, 15);
        //         if (il != null)
        //         {
        //             try
        //             {
        //                 int ind = int.Parse(parts[1]);
        //                 return il.Images[ind] as Bitmap;
        //             }
        //             catch (FormatException) { }
        //             catch (OverflowException) { }
        //             catch (ArgumentOutOfRangeException) { }
        //         }
        //     }
        //     return null;
        // }
        // TODO reimplement in Avalonia
        IPaintTo3D IUIService.CreatePaintInterface(object paintToBitmap, double precision)
        {
            throw new NotImplementedException();
        }
        // {
        //     PaintToOpenGL paintTo3D = new PaintToOpenGL(precision);
        //     paintTo3D.Init(paintToBitmap as Bitmap);
        //     return paintTo3D;
        // }
        // TODO reimplement in Avalonia
        Substitutes.DialogResult IUIService.ShowPageSetupDlg(object printDocument, object pageSettings, out int width, out int height, out bool landscape)
        {
            throw new NotImplementedException();
        }
        // {
        //     PageSetupDialog psd = new PageSetupDialog();
        //     psd.AllowPrinter = true;
        //     psd.EnableMetric = true;
        //     psd.Document = (PrintDocument?)printDocument;
        //     Substitutes.DialogResult res = (Substitutes.DialogResult)(int)psd.ShowDialog();
        //     if (res == Substitutes.DialogResult.OK)
        //     {
        //         psd.Document.OriginAtMargins = false;
        //         printDocument = psd.Document;
        //         width = psd.PageSettings.PaperSize.Width;
        //         height = psd.PageSettings.PaperSize.Height;
        //         landscape = psd.PageSettings.Landscape;
        //     }
        //     else
        //     {
        //         width = height = 0;
        //         landscape = false;
        //     }
        //     return res;
        // }
        // TODO reimplement in Avalonia
        Substitutes.DialogResult IUIService.ShowPrintDlg(object printDocument)
        {
            throw new NotImplementedException();
        }
        // {
        //     PrintDialog printDialog = new PrintDialog();
        //     printDialog.Document = (PrintDocument?)printDocument;
        //     printDialog.AllowSomePages = false;
        //     printDialog.AllowCurrentPage = false;
        //     printDialog.AllowSelection = false;
        //     printDialog.AllowPrintToFile = false;
        //     Substitutes.DialogResult res = (Substitutes.DialogResult)printDialog.ShowDialog();
        //     if (res == Substitutes.DialogResult.OK)
        //     {
        //         printDocument = printDialog.Document;
        //     }
        //     return res;
        // }
        // TODO reimplement in Avalonia
        void IUIService.SetClipboardData(GeoObjectList objects, bool copy)
        {
            throw new NotImplementedException();
        }
        // {
        //     // -> JSON in Byte-Array serialisieren
        //     byte[] payload;
        //     using (var ms = new MemoryStream())
        //     {
        //         var js = new JsonSerialize();
        //         js.ToStream(ms, objects, closeStream: false);
        //         payload = ms.ToArray();
        //     }

        //     // DataObject mit eigenem Format befüllen (ohne Auto-Konvertierung)
        //     var dob = new DataObject();
        //     dob.SetData(ClipFormat, false, payload);

        //     // Optional: eine menschenlesbare Text-Notiz (damit Paste in Notepad nicht leer ist)
        //     dob.SetText($"CADability GeoObjectList ({objects?.Count ?? 0} items)");

        //     // copy=true: Inhalte bleiben erhalten, auch wenn die App endet
        //     Clipboard.SetDataObject(dob, copy);
        // }
        // TODO reimplement in Avalonia
        object IUIService.GetClipboardData(Type typeOfdata)
        {
            throw new NotImplementedException();
        }
        // {
        //     try
        //     {
        //         // defensiv prüfen, ohne GetDataObject()
        //         if (!Clipboard.ContainsData(ClipFormat))
        //             return null;

        //         var data = Clipboard.GetData(ClipFormat);
        //         if (data is byte[] bytes)
        //         {
        //             using var ms = new MemoryStream(bytes);
        //             var js = new JsonSerialize();
        //             return js.FromStream(ms); // -> GeoObjectList
        //         }

        //         // Falls ein Stream geliefert wird (kann je nach Host passieren)
        //         if (data is Stream s)
        //         {
        //             using var ms = new MemoryStream();
        //             s.CopyTo(ms);
        //             ms.Position = 0;
        //             var js = new JsonSerialize();
        //             return js.FromStream(ms);
        //         }

        //         // Worst case: String (sollten wir nicht bekommen, aber behandeln)
        //         if (data is string json)
        //         {
        //             return JsonSerialize.FromString(json);
        //         }

        //         return null;
        //     }
        //     catch (ExternalException)
        //     {
        //         // Clipboard gerade gelockt o.ä. – einfach "kein Inhalt" signalisieren
        //         return null;
        //     }
        // }
        // TODO reimplement in Avalonia
        bool IUIService.HasClipboardData(Type typeOfdata)
        {
            throw new NotImplementedException();
        }
        // {
        //     try
        //     {
        //         // Leichtgewichtige .NET-Variante:
        //         if (Clipboard.ContainsData(ClipFormat)) return true;

        //         // Optional, noch robuster & COM-frei (empfohlen im Idle):
        //         // return NativeClipboard.HasMyFormat();

        //         return false;
        //     }
        //     catch (ExternalException)
        //     {
        //         // z. B. wenn gerade ein anderer Prozess das Clipboard geöffnet hat
        //         return false;
        //     }
        // }

        public Substitutes.FontFamily GetFontFamily(string fontFamilyName)
        {
            throw new NotImplementedException();
            // if (string.IsNullOrEmpty(fontFamilyName)) return new FontFamilyImpl();
            // else return new FontFamilyImpl(fontFamilyName);
        }

        public string[] GetFontFamilies()
        {
            throw new NotImplementedException();
            // FontFamily[] ffs = FontFamily.Families;
            // List<string> result = new List<string>();
            // for (int i = 0; i < ffs.Length; i++) result.Add(ffs[i].Name);
            // return result.ToArray();
        }

        // TODO reimplement in Avalonia
        event EventHandler IUIService.ApplicationIdle
        {
            add
            {
                // Application.Idle += value;
            }

            remove
            {
                // Application.Idle -= value;
            }
        }
        #endregion
    }
}
