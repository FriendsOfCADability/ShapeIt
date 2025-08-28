using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using CADability.UserInterface;
using SkiaSharp;
using Svg.Skia;

namespace CADability.Forms.NET8
{
    public static class SvgBitmapHelper
    {
        // Win32 system metrics for small icons (toolbars)
        private const int SM_CXSMICON = 49;
        private const int SM_CYSMICON = 50;
        [DllImport("user32.dll")] private static extern uint GetDpiForWindow(IntPtr hWnd);
        [DllImport("user32.dll")] private static extern int GetSystemMetricsForDpi(int nIndex, uint dpi);

        private static Dictionary<(string, Size), Bitmap> cachedBitmaps = new Dictionary<(string, Size), Bitmap>();
        /// <summary>
        /// Create a toolbar-sized bitmap from an embedded SVG resource.
        /// Fallback to embedded raster (.png/.bmp/.ico) and upscale if needed.
        /// </summary>
        /// <param name="resourceBaseName">
        /// Base name without extension, e.g. "MyNamespace.Images.zoom_in".
        /// We'll try ".svg" first, then ".png/.bmp/.ico".
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
        public static Bitmap? CreateBitmapFromEmbeddedSvg(string resourceBaseName, Control contextControl, Size? overridePixelSize = null, Assembly? assembly = null)
        {
            if (string.IsNullOrWhiteSpace(resourceBaseName) || contextControl == null)
                return null;

            assembly ??= Assembly.GetCallingAssembly();

            // 1) Determine target pixel size for toolbar icons.
            Size targetSize = overridePixelSize ?? GetToolbarTargetSize(contextControl);
            if (cachedBitmaps.TryGetValue((resourceBaseName, targetSize), out var bitmap) && bitmap != null) { return bitmap; }

            // 2) Try SVG first.
            using (var svgStream = TryOpenResourceStream(assembly, "Icons/"+resourceBaseName, ".svg"))
            {
                if (svgStream != null)
                {
                    try
                    {
                        var svg = new SKSvg();
                        svg.Load(svgStream);
                        if (svg.Picture == null)
                            throw new InvalidOperationException("SVG picture is null.");

                        // Render exactly in the target size
                        Bitmap res = RenderSvgToBitmap(svg.Picture, targetSize);
                        cachedBitmaps[(resourceBaseName, targetSize)] = res;
                        return res;
                    }
                    catch
                    {
                        // fall through to raster fallback
                    }
                }
            }

            // 3) Raster fallback: 
            int ImageIndex = MenuResource.FindImageIndex(resourceBaseName);
            if (ImageIndex < 0 || ButtonImages.ButtonImageList.Images.Count <= ImageIndex) return null; // not found
            Bitmap src = new Bitmap(ButtonImages.ButtonImageList.Images[ImageIndex]);

            if (src.Size == targetSize) return src;

            // upscale (or downscale) to target; keep alpha
            Bitmap scaled = ResizeBitmap(src, targetSize);
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
        /// Render an SKPicture (from SVG) into a 32bpp premultiplied-ARGB Bitmap of the given size.
        /// </summary>
        private static Bitmap RenderSvgToBitmap(SKPicture picture, Size target)
        {
            var cull = picture.CullRect;
            float svgW = Math.Max(1f, cull.Width);
            float svgH = Math.Max(1f, cull.Height);
            float sx = target.Width / svgW;
            float sy = target.Height / svgH;

            using var surface = SKSurface.Create(new SKImageInfo(target.Width, target.Height, SKColorType.Bgra8888, SKAlphaType.Premul));
            var canvas = surface.Canvas;
            canvas.Clear(SKColors.Transparent);
            canvas.Scale(sx, sy);
            canvas.DrawPicture(picture);
            canvas.Flush();

            using var image = surface.Snapshot();
            using var data = image.Encode(SKEncodedImageFormat.Png, 100);
            using var ms = new MemoryStream(data.ToArray());
            return new Bitmap(ms); // will be 32bpp ARGB with alpha
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
            g.InterpolationMode = InterpolationMode.HighQualityBicubic;
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
    }
}