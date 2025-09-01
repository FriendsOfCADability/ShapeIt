using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using CADability.UserInterface;
using Svg; // Svg.NET

namespace CADability.Forms.NET8
{
    public static class SvgBitmapHelper
    {
        // Win32 system metrics for small icons (toolbars)
        private const int SM_CXSMICON = 49;
        private const int SM_CYSMICON = 50;

        [DllImport("user32.dll")] private static extern uint GetDpiForWindow(IntPtr hWnd);
        [DllImport("user32.dll")] private static extern int GetSystemMetricsForDpi(int nIndex, uint dpi);

        // Cache generated bitmaps per (resourceName, targetSize)
        private static readonly Dictionary<(string, Size), Bitmap> cachedBitmaps = new();

        /// <summary>
        /// Create a toolbar-sized bitmap from an embedded SVG resource using Svg.NET.
        /// Fallback to embedded raster ImageList and upscale if needed.
        /// </summary>
        /// <param name="resourceBaseName">
        /// Base name without extension, e.g. "MyNamespace.Images.zoom_in".
        /// We will try "Icons/{name}.svg" first.
        /// </param>
        /// <param name="contextControl">
        /// Any control in the target window (used to query per-monitor DPI).
        /// </param>
        /// <param name="overridePixelSize">
        /// Optional explicit target size. If null, system small-icon size is used.
        /// </param>
        /// <param name="assembly">
        /// Optional assembly to search; defaults to the calling assembly.
        /// </param>
        /// <returns>Bitmap or null if nothing found.</returns>
        public static Bitmap? CreateBitmapFromEmbeddedSvg(
            string resourceBaseName,
            Control contextControl,
            Size? overridePixelSize = null,
            Assembly? assembly = null)
        {
            if (string.IsNullOrWhiteSpace(resourceBaseName) || contextControl == null)
                return null;

            assembly ??= Assembly.GetCallingAssembly();

            // 1) Determine target pixel size for toolbar icons.
            Size targetSize = overridePixelSize ?? GetToolbarTargetSize(contextControl);

            if (cachedBitmaps.TryGetValue((resourceBaseName, targetSize), out var cached) && cached != null)
                return cached;

            // 2) Try SVG first: look up "Icons/{resourceBaseName}.svg" (exact or suffix match)
            using (var svgStream = TryOpenResourceStream(assembly, "Icons/" + resourceBaseName, ".svg"))
            {
                if (svgStream != null)
                {
                    try
                    {
                        // Load SVG document with Svg.NET
                        // Note: stream stays open only for parsing; Draw creates a new Bitmap.
                        var doc = SvgDocument.Open<SvgDocument>(svgStream);

                        // Draw directly to the requested pixel size (32bpp ARGB with alpha).
                        var bmp = doc.Draw(targetSize.Width, targetSize.Height);

                        // Cache and return
                        cachedBitmaps[(resourceBaseName, targetSize)] = bmp;
                        return bmp;
                    }
                    catch
                    {
                        // fall through to raster fallback if SVG fails
                    }
                }
            }

            // 3) Raster fallback via ImageList
            int imageIndex = MenuResource.FindImageIndex(resourceBaseName);
            if (imageIndex < 0 || ButtonImages.ButtonImageList.Images.Count <= imageIndex)
                return null; // not found

            using var srcTmp = new Bitmap(ButtonImages.ButtonImageList.Images[imageIndex]); // ensure we own a Bitmap
            if (srcTmp.Size == targetSize)
            {
                var same = new Bitmap(srcTmp);
                cachedBitmaps[(resourceBaseName, targetSize)] = same;
                return same;
            }

            // Upscale (or downscale) to target; keep alpha
            var scaled = ResizeBitmap(srcTmp, targetSize);
            cachedBitmaps[(resourceBaseName, targetSize)] = scaled;
            return scaled;
        }

        /// <summary>
        /// Determine the correct toolbar image size for the given control's DPI.
        /// </summary>
        public static Size GetToolbarTargetSize(Control c)
        {
            uint dpi = GetDpiForWindow(c.Handle);
            int w = GetSystemMetricsForDpi(SM_CXSMICON, dpi);
            int h = GetSystemMetricsForDpi(SM_CYSMICON, dpi);
            return new Size(Math.Max(1, w), Math.Max(1, h));
        }

        /// <summary>
        /// High-quality resize for raster fallback (keeps alpha).
        /// </summary>
        private static Bitmap ResizeBitmap(Bitmap src, Size target)
        {
            var dst = new Bitmap(target.Width, target.Height, System.Drawing.Imaging.PixelFormat.Format32bppPArgb);
            using var g = Graphics.FromImage(dst);
            g.CompositingMode = System.Drawing.Drawing2D.CompositingMode.SourceCopy;
            g.CompositingQuality = System.Drawing.Drawing2D.CompositingQuality.HighQuality;
            g.InterpolationMode = InterpolationMode.HighQualityBicubic; // switch to NearestNeighbor if you prefer crisp pixel edges
            g.PixelOffsetMode = PixelOffsetMode.HighQuality;
            g.SmoothingMode = SmoothingMode.AntiAlias;

            g.DrawImage(src, new Rectangle(Point.Empty, target), new Rectangle(Point.Empty, src.Size), GraphicsUnit.Pixel);
            return dst;
        }

        /// <summary>
        /// Try to open an embedded resource stream. Tries exact name, then suffix match.
        /// </summary>
        private static Stream? TryOpenResourceStream(Assembly asm, string baseName, string ext)
        {
            // Exact
            string exact = baseName.EndsWith(ext, StringComparison.OrdinalIgnoreCase) ? baseName : baseName + ext;
            var s = asm.GetManifestResourceStream(exact);
            if (s != null) return s;

            // Suffix search (handles namespace prefixes)
            string suffix = (baseName + ext).Replace('\\', '.').Replace('/', '.');
            var name = asm.GetManifestResourceNames().FirstOrDefault(n => n.EndsWith(suffix, StringComparison.OrdinalIgnoreCase));
            return name != null ? asm.GetManifestResourceStream(name) : null;
        }

        /// <summary>
        /// Optional: clears the internal bitmap cache. Call when sizes/theme change if you want to free memory.
        /// Be careful not to dispose images that are still in use by UI controls.
        /// </summary>
        public static void InvalidateCache(bool disposeBitmaps = false)
        {
            if (disposeBitmaps)
            {
                foreach (var kv in cachedBitmaps)
                {
                    try { kv.Value.Dispose(); } catch { /* ignore */ }
                }
            }
            cachedBitmaps.Clear();
        }
    }
}
