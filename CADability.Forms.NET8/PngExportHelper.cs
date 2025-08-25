using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.IO;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.TrayNotify;

// !!!Remove when done!!!
namespace CADability.Forms.NET8
{

    // this code in Menuresource was also needed:
    //static public string[] AllPopupMenus()
    //{   // liefert alle popup menues auf unterster Ebene, die mit Shortcuts="true" gekennzeichnet sind
    //    List<string> res = new List<string>();
    //    XmlNodeList nl = mMenuDocument.SelectNodes("/Menus/Popup");
    //    foreach (XmlNode o in nl)
    //    {
    //        res.Add(o.Attributes["MenuId"].Value);
    //    }
    //    return res.ToArray();
    //}


    // this code in MainForm created all Bitmap Icons
    //PngExportHelper.CreateAllIcons(mainMenu);
    //string[] popups = MenuResource.AllPopupMenus();
    //foreach (string popup in popups)
    //{
    //    MenuWithHandler[] popupMenu = MenuResource.LoadMenuDefinition(popup, false, cadFrame);
    //    PngExportHelper.CreateAllIcons(popupMenu);
    //}

    public static class PngExportHelper
    {
        /// <summary>
        /// Exports all bitmaps to a folder as 64x64 PNGs (or custom size).
        /// By default keeps aspect ratio and pads with transparency to the target box.
        /// Filenames are sanitized; existing files are overwritten by default.
        /// </summary>
        /// <param name="images">Dictionary of name -> Bitmap</param>
        /// <param name="targetFolder">Destination folder (created if missing)</param>
        /// <param name="targetSize">Target edge in pixels (square), default 64</param>
        /// <param name="keepAspect">If true, preserve aspect and pad; if false, stretch to fill</param>
        /// <param name="background">Optional background (null = transparent)</param>
        /// <param name="overwrite">If false, adds a numeric suffix when file exists</param>
        public static void ExportToFolder(
            IDictionary<string, Bitmap> images,
            string targetFolder,
            int targetSize = 64,
            bool keepAspect = true,
            Color? background = null,
            bool overwrite = true)
        {
            if (images == null) throw new ArgumentNullException(nameof(images));
            if (string.IsNullOrWhiteSpace(targetFolder)) throw new ArgumentException("Target folder is empty.", nameof(targetFolder));

            Directory.CreateDirectory(targetFolder);

            foreach (var kv in images)
            {
                string safeName = MakeSafeFileName(kv.Key);
                if (string.IsNullOrEmpty(safeName)) safeName = "image";
                string filePath = Path.Combine(targetFolder, safeName + ".png");

                if (!overwrite && File.Exists(filePath))
                {
                    int i = 1;
                    while (File.Exists(filePath = Path.Combine(targetFolder, $"{safeName}_{i}.png"))) i++;
                }

                using var scaled = ResizeToSquare(kv.Value, targetSize, keepAspect, background);
                // Save as PNG (keeps alpha)
                scaled.Save(filePath, ImageFormat.Png);
            }
        }

        /// <summary>
        /// Resizes a bitmap to a square canvas size x size using nearest-neighbor.
        /// If keepAspect is true, the image is uniformly scaled and centered; otherwise stretched.
        /// Optional background fills the canvas (default transparent).
        /// </summary>
        private static Bitmap ResizeToSquare(Bitmap src, int size, bool keepAspect, Color? background)
        {
            var dst = new Bitmap(size, size, PixelFormat.Format32bppPArgb);

            using var g = Graphics.FromImage(dst);
            // Keep hard pixel edges (good for tiny 16x16 sources)
            g.InterpolationMode = InterpolationMode.NearestNeighbor;
            g.PixelOffsetMode = PixelOffsetMode.HighQuality;
            g.SmoothingMode = SmoothingMode.None;
            g.CompositingMode = CompositingMode.SourceCopy;
            g.CompositingQuality = CompositingQuality.HighQuality;

            // Background (transparent by default)
            if (background.HasValue)
                g.Clear(background.Value);
            else
                g.Clear(Color.Transparent);

            Rectangle destRect;
            if (keepAspect)
            {
                float scale = Math.Min(size / (float)src.Width, size / (float)src.Height);
                int w = Math.Max(1, (int)Math.Round(src.Width * scale));
                int h = Math.Max(1, (int)Math.Round(src.Height * scale));
                int x = (size - w) / 2;
                int y = (size - h) / 2;
                destRect = new Rectangle(x, y, w, h);
            }
            else
            {
                destRect = new Rectangle(0, 0, size, size);
            }

            g.DrawImage(src, destRect, new Rectangle(0, 0, src.Width, src.Height), GraphicsUnit.Pixel);
            return dst;
        }

        /// <summary>
        /// Produces a filesystem-safe filename from an arbitrary key string.
        /// </summary>
        private static string MakeSafeFileName(string name)
        {
            var invalid = Path.GetInvalidFileNameChars();
            var chars = name.ToCharArray();
            for (int i = 0; i < chars.Length; i++)
            {
                if (Array.IndexOf(invalid, chars[i]) >= 0)
                    chars[i] = '_';
            }
            // Trim and avoid leading/trailing dots/spaces
            var s = new string(chars).Trim().Trim('.', ' ');
            return string.IsNullOrEmpty(s) ? "image" : s;
        }

        public static void CreateAllIcons(MenuWithHandler[] mainMenu)
        {
            Dictionary<string, Bitmap> icons = new Dictionary<string, Bitmap>();
            for (int i = 0; mainMenu.Length > i; i++)
            {
                HandleMenu(mainMenu[i], icons);
            }
            ExportToFolder(icons, @"C:\Temp\Icons");
        }
        private static void HandleMenu(MenuWithHandler mh, Dictionary<string, Bitmap> icons)
        {
            if (mh.SubMenus != null)
            {
                foreach (var menu in mh.SubMenus) HandleMenu(menu, icons);
            }
            if (mh.ImageIndex >= 0)
            {
                Bitmap src = new Bitmap(ButtonImages.ButtonImageList.Images[mh.ImageIndex]);
                icons[mh.ID] = src;
            }
        }
    }
}