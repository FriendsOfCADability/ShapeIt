using Microsoft.Win32;
using SkiaSharp;
using Svg.Skia;
using System;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using System.Xml.Linq;

public static class SvgCursorHelper
{
    // --- Win32 DPI & Metriken ---
    private const int SM_CXCURSOR = 13;
    private const int SM_CYCURSOR = 14;

    [DllImport("user32.dll")]
    private static extern uint GetDpiForWindow(IntPtr hWnd);

    [DllImport("user32.dll")]
    private static extern int GetSystemMetricsForDpi(int nIndex, uint dpi);

    [DllImport("user32.dll")]
    private static extern IntPtr CreateIconIndirect(ref ICONINFO icon);

    [DllImport("gdi32.dll")]
    private static extern bool DeleteObject(IntPtr hObject);

    [DllImport("gdi32.dll")]
    private static extern IntPtr CreateBitmap(int nWidth, int nHeight, uint cPlanes, uint cBitsPerPel, IntPtr lpvBits);

    // Lies <desc id="hotspot" x="…" y="…"/>
    public static bool TryReadHotspotFromSvgXml(Stream svgStream, out PointF hotspot)
    {
        hotspot = default;

        // Sicherstellen, dass Position am Anfang ist
        if (svgStream.CanSeek) svgStream.Position = 0;

        using var reader = new StreamReader(svgStream, leaveOpen: true);
        string xml = reader.ReadToEnd();
        if (string.IsNullOrWhiteSpace(xml)) return false;

        var doc = XDocument.Parse(xml);
        var svg = doc.Root;
        if (svg == null || !string.Equals(svg.Name.LocalName, "svg", StringComparison.OrdinalIgnoreCase))
            return false;

        // Look for <desc id="hotspot" .../>
        foreach (var desc in svg.Elements())
        {
            if (!string.Equals(desc.Name.LocalName, "desc", StringComparison.OrdinalIgnoreCase))
                continue;

            var id = (string?)desc.Attribute("id");
            if (!string.Equals(id, "hotspot", StringComparison.OrdinalIgnoreCase))
                continue;

            // x/y lesen (InvariantCulture, Komma-Workaround)
            if (TryParseFloatAttr(desc, "x", out float x) &&
                TryParseFloatAttr(desc, "y", out float y))
            {
                hotspot = new PointF(x, y);
                return true;
            }
        }

        return false;
    }

    private static bool TryParseFloatAttr(XElement el, string name, out float value)
    {
        value = 0f;
        var s = (string?)el.Attribute(name);
        if (string.IsNullOrWhiteSpace(s)) return false;

        // Erlaubt sowohl "." als auch "," als Dezimaltrenner
        s = s.Trim().Replace(',', '.');
        return float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
    }
    private static Cursor CreateCursorFromBitmap(Bitmap bmp, Point hotspot)
    {
        using var iconInfo = new IconInfo(bmp, hotspot);
        IntPtr hIcon = CreateIconIndirect(ref iconInfo.info);
        return new Cursor(hIcon);
    }
    public static (int cx, int cy) GetTargetCursorSize(Control ctl)
    {
        uint dpi = GetDpiForWindow(ctl.Handle);
        int baseCx = GetSystemMetricsForDpi(SM_CXCURSOR, dpi); // i.d.R. 32 bei 96 DPI
        int baseCy = GetSystemMetricsForDpi(SM_CYCURSOR, dpi);

        // 1) Direkte Basisgröße (Pixel) – bevorzugt verwenden
        var baseSizeObj = Registry.GetValue(
            @"HKEY_CURRENT_USER\Control Panel\Cursors", "CursorBaseSize", null);
        if (baseSizeObj is int baseSize && baseSize > 0)
        {
            double s = baseSize / 32.0; // 32 ist Default-Basis bei 96 DPI
            return ((int)Math.Round(baseCx * s), (int)Math.Round(baseCy * s));
        }

        // 2) Slider-Stufe (1..15) -> Größe ableiten
        var multObj = Registry.GetValue(
            @"HKEY_CURRENT_USER\Software\Microsoft\Accessibility", "CursorSize", null);
        if (multObj is int m && m >= 1)
        {
            // bewährte Ableitung aus dem Sliderwert (siehe Quelle)
            double newH = baseCy + (m - 1) * (baseCy / 2.0);
            double s = newH / baseCy;
            return ((int)Math.Round(baseCx * s), (int)Math.Round(newH));
        }

        return (baseCx, baseCy); // Fallback: nur DPI-Größe
    }

    public static Cursor? CreateCursorFromEmbeddedSvg(string resourceNameWithoutExtension, Control contextControl)
    {
        Assembly ThisAssembly = Assembly.GetExecutingAssembly();
        // 1) Resource lesen und in Byte-Array kopieren (zweifach nutztbar)
        var asm = ThisAssembly; // falls du das als Typ brauchst: typeof(Program).Assembly o.ä.
        using Stream? resource = ThisAssembly.GetManifestResourceStream(resourceNameWithoutExtension + ".svg");
        if (resource == null) return null;

        byte[] svgBytes;
        using (var ms = new MemoryStream())
        {
            resource.CopyTo(ms);
            svgBytes = ms.ToArray();
        }

        // 2) Hotspot aus XML lesen
        PointF hotspotSvg = default;
        bool hasHotspot;
        using (var msXml = new MemoryStream(svgBytes, writable: false))
            hasHotspot = TryReadHotspotFromSvgXml(msXml, out hotspotSvg);

        // 3) SVG mit Svg.Skia laden
        var skSvg = new SKSvg();
        using (var msSvg = new MemoryStream(svgBytes, writable: false))
        {
            skSvg.Load(msSvg);
        }

        if (skSvg.Picture == null) return null;

        // 4) Zielgröße gemäß aktuellem DPI-Kontext (Cursorgröße)
        (int cx, int cy) = GetTargetCursorSize(contextControl);
        // 5) Intrinsische SVG-Größe ermitteln
        var cull = skSvg.Picture.CullRect; // "logische" SVG-Größe
        float svgW = Math.Max(1f, cull.Width);
        float svgH = Math.Max(1f, cull.Height);

        float sx = cx / svgW;
        float sy = cy / svgH;

        // 6) Rendern in genau cx×cy
        using var surface = SKSurface.Create(new SKImageInfo(cx, cy, SKColorType.Bgra8888, SKAlphaType.Premul));
        var canvas = surface.Canvas;
        canvas.Clear(SKColors.Transparent);
        canvas.Scale(sx, sy);
        canvas.DrawPicture(skSvg.Picture);
        canvas.Flush();

        using var image = surface.Snapshot();
        using var data = image.Encode(); // PNG
        using var bmp = new Bitmap(new MemoryStream(data.ToArray())); // ARGB

        // 7) Hotspot skalieren (SVG -> Pixel)
        Point hotspotPx;
        if (hasHotspot)
        {
            hotspotPx = new Point(
                (int)Math.Round(hotspotSvg.X * sx),
                (int)Math.Round(hotspotSvg.Y * sy)
            );
        }
        else
        {
            hotspotPx = Point.Empty; // fallback: 0,0
        }

        // 8) Cursor erzeugen über deine bestehende Methode
        return CreateCursorFromBitmap(bmp, hotspotPx);
    }
    public static IntPtr CreateDummyMask(Size size)
    {
        return CreateBitmap(size.Width, size.Height, 1, 1, IntPtr.Zero);
    }

    private class IconInfo : IDisposable
    {
        public ICONINFO info;
        private IntPtr maskHandle;

        public IconInfo(Bitmap bmp, Point hotspot)
        {
            maskHandle = CreateDummyMask(bmp.Size);
            info = new ICONINFO
            {
                fIcon = false,
                xHotspot = hotspot.X,
                yHotspot = hotspot.Y,
                hbmMask = maskHandle,
                hbmColor = bmp.GetHbitmap()
            };
        }

        public void Dispose()
        {
            if (info.hbmMask != IntPtr.Zero) DeleteObject(info.hbmMask);
            if (info.hbmColor != IntPtr.Zero) DeleteObject(info.hbmColor);
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct ICONINFO
    {
        public bool fIcon;
        public int xHotspot;
        public int yHotspot;
        public IntPtr hbmMask;
        public IntPtr hbmColor;
    }

}



/* alt:


using SkiaSharp;
using Svg.Skia;
using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Windows.Forms;

namespace CADability.Forms.NET8
{
    public static class SvgCursorHelper
    {
        public static Cursor CreateCursorFromSvg(byte[] svgData, int dpi = 96, Point? hotspot = null, Size? size = null)
        {
            var svg = new SKSvg();
            using var stream = new MemoryStream(svgData);
            svg.Load(stream);

            var picture = svg.Picture ?? throw new InvalidOperationException("Invalid SVG");
            var bounds = picture.CullRect;

            var targetSize = size ?? new Size(32, 32); // Default-Cursorgröße (skalieren wir gleich)

            float scale = Math.Min((float)targetSize.Width / bounds.Width, (float)targetSize.Height / bounds.Height);

            using var surface = SKSurface.Create(new SKImageInfo(targetSize.Width, targetSize.Height));
            var canvas = surface.Canvas;
            canvas.Clear(SKColors.Transparent);
            canvas.Scale(scale);
            canvas.DrawPicture(picture);
            canvas.Flush();

            using var image = surface.Snapshot();
            using var data = image.Encode(SKEncodedImageFormat.Png, 100);
            using var bmpStream = new MemoryStream(data.ToArray());

            using var bmp = new Bitmap(bmpStream);
            return CreateCursorFromBitmap(bmp, hotspot ?? new Point(0, 0));
        }

        private static Cursor CreateCursorFromBitmap(Bitmap bmp, Point hotspot)
        {
            using var iconInfo = new IconInfo(bmp, hotspot);
            IntPtr hIcon = CreateIconIndirect(ref iconInfo.info);
            return new Cursor(hIcon);
        }
        public static IntPtr CreateDummyMask(Size size)
        {
            
            return CreateBitmap(size.Width, size.Height, 1, 1, IntPtr.Zero);

        }
        private class IconInfo : IDisposable
        {
            public ICONINFO info;
            private IntPtr maskHandle;

            public IconInfo(Bitmap bmp, Point hotspot)
            {
                maskHandle = CreateDummyMask(bmp.Size);
                info = new ICONINFO
                {
                    fIcon = false,
                    xHotspot = hotspot.X,
                    yHotspot = hotspot.Y,
                    hbmMask = maskHandle,
                    hbmColor = bmp.GetHbitmap()
                };
            }

            public void Dispose()
            {
                if (info.hbmMask != IntPtr.Zero) DeleteObject(info.hbmMask);
                if (info.hbmColor != IntPtr.Zero) DeleteObject(info.hbmColor);
            }
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct ICONINFO
        {
            public bool fIcon;
            public int xHotspot;
            public int yHotspot;
            public IntPtr hbmMask;
            public IntPtr hbmColor;
        }

        [DllImport("user32.dll")]
        private static extern IntPtr CreateIconIndirect(ref ICONINFO icon);

        [DllImport("gdi32.dll")]
        private static extern bool DeleteObject(IntPtr hObject);
        [DllImport("gdi32.dll")]
        private static extern IntPtr CreateBitmap(int nWidth, int nHeight, uint cPlanes, uint cBitsPerPel, IntPtr lpvBits);
    }
}
*/