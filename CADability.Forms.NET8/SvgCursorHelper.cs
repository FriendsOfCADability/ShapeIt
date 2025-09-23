using Microsoft.Win32;
using Svg; // Svg.NET
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
    // --- Win32 DPI & metrics ---
    private const int SM_CXCURSOR = 13;
    private const int SM_CYCURSOR = 14;

    [DllImport("user32.dll")] private static extern uint GetDpiForWindow(IntPtr hWnd);
    [DllImport("user32.dll")] private static extern int GetSystemMetricsForDpi(int nIndex, uint dpi);
    [DllImport("user32.dll")] private static extern IntPtr CreateIconIndirect(ref ICONINFO icon);
    [DllImport("gdi32.dll")] private static extern bool DeleteObject(IntPtr hObject);
    [DllImport("gdi32.dll")] private static extern IntPtr CreateBitmap(int nWidth, int nHeight, uint cPlanes, uint cBitsPerPel, IntPtr lpvBits);

    /// <summary>
    /// Calculates the center point of the "hotspot" element within the specified SVG document and hides the element.
    /// </summary>
    /// <param name="doc">The <see cref="SvgDocument"/> containing the "hotspot" element. Cannot be null.</param>
    /// <param name="cx">The width, in pixels, of the temporary rendering surface used for layout calculations.</param>
    /// <param name="cy">The height, in pixels, of the temporary rendering surface used for layout calculations.</param>
    /// <returns>A <see cref="PointF"/> representing the center point of the "hotspot" element in the SVG document's coordinate
    /// space. Returns <see cref="PointF.Empty"/> if the "hotspot" element is not found.</returns>
    static PointF ReadHotspotFromCircleAndHide(SvgDocument doc, int cx, int cy)
    {
        var el = doc.GetElementById("hotspot") as SvgVisualElement;
        if (el == null) return default;

        // 1) Determine the circle center in SVG user space
        // For <circle>, these are CenterX/CenterY (SvgUnit). This also works correctly with transforms,
        // if we take the document transform into account.
        // Simpler: use the bounding box and take the average:
        // We use a temporary renderer instance so Svg.NET resolves all transforms/aspect ratio.
        using var tmp = new Bitmap(cx, cy);
        using (var renderer = SvgRenderer.FromImage(tmp))
        {
            // Perform a "layout": this draws on tmp, but it's just a throwaway bitmap
            doc.Draw(renderer);

            var path = el.Path(renderer);               // GraphicsPath of the element in the target pixel space
            var b = path.GetBounds();
            var px = b.Left + b.Width * 0.5f;
            var py = b.Top + b.Height * 0.5f;

            // 2) Make the hotspot element invisible so it doesn't appear in the final bitmap
            el.Visibility = "Hidden";       // or: el.Display = "none";

            return new PointF(px, py);
        }
    }

    /// <summary>
    /// Reads the logical SVG viewport size from viewBox or width/height (in px, pt, mm, cm).
    /// Returns false if neither is present; out values are then 0.
    /// </summary>
    private static bool TryReadSvgLogicalSize(byte[] svgBytes, out SizeF logical)
    {
        logical = default;
        using var ms = new MemoryStream(svgBytes, writable: false);
        var doc = XDocument.Load(ms);
        var root = doc.Root;
        if (root == null || !string.Equals(root.Name.LocalName, "svg", StringComparison.OrdinalIgnoreCase))
            return false;

        // Prefer viewBox="minx miny width height"
        var viewBoxAttr = (string?)root.Attribute("viewBox");
        if (!string.IsNullOrWhiteSpace(viewBoxAttr))
        {
            var parts = viewBoxAttr.Split(new[] { ' ', ',', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length == 4 &&
                TryParseFloat(parts[2], out float vw) &&
                TryParseFloat(parts[3], out float vh) &&
                vw > 0 && vh > 0)
            {
                logical = new SizeF(vw, vh);
                return true;
            }
        }

        // Fallback: width / height attributes
        if (TryParseSvgLength((string?)root.Attribute("width"), out float w) &&
            TryParseSvgLength((string?)root.Attribute("height"), out float h) &&
            w > 0 && h > 0)
        {
            logical = new SizeF(w, h);
            return true;
        }

        return false;
    }

    private static bool TryParseFloatAttr(XElement el, string name, out float value)
    {
        value = 0f;
        var s = (string?)el.Attribute(name);
        return TryParseFloat(s, out value);
    }

    private static bool TryParseFloat(string? s, out float value)
    {
        value = 0f;
        if (string.IsNullOrWhiteSpace(s)) return false;
        s = s.Trim().Replace(',', '.');
        return float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
    }

    /// <summary>
    /// Parses an SVG length (px/pt/mm/cm or unitless). Returns pixels at 96 DPI.
    /// </summary>
    private static bool TryParseSvgLength(string? s, out float px)
    {
        px = 0f;
        if (string.IsNullOrWhiteSpace(s)) return false;
        s = s.Trim();

        // Extract numeric part + unit
        int i = 0;
        while (i < s.Length && (char.IsDigit(s[i]) || s[i] == '.' || s[i] == ',' || s[i] == '+' || s[i] == '-')) i++;
        var num = s.Substring(0, i).Trim().Replace(',', '.');
        var unit = s.Substring(i).Trim().ToLowerInvariant();

        if (!float.TryParse(num, NumberStyles.Float, CultureInfo.InvariantCulture, out float v))
            return false;

        const float dpi = 96f;
        px = unit switch
        {
            "" or "px" => v,
            "pt" => v * dpi / 72f,
            "mm" => v * dpi / 25.4f,
            "cm" => v * dpi / 2.54f,
            _ => v // fallback: treat as px
        };
        return true;
    }

    private static Cursor CreateCursorFromBitmap(Bitmap bmp, Point hotspot)
    {
        using var iconInfo = new IconInfo(bmp, hotspot);
        IntPtr hIcon = CreateIconIndirect(ref iconInfo.info);
        return new Cursor(hIcon);
    }

    /// <summary>
    /// Returns the effective cursor size in pixels for the given control,
    /// combining DPI metrics with the system-wide accessibility cursor size.
    /// </summary>
    public static (int cx, int cy) GetTargetCursorSize(Control ctl)
    {
        uint dpi = GetDpiForWindow(ctl.Handle);
        int baseCx = GetSystemMetricsForDpi(SM_CXCURSOR, dpi);
        int baseCy = GetSystemMetricsForDpi(SM_CYCURSOR, dpi);

        // Preferred: absolute base size in pixels
        var baseSizeObj = Registry.GetValue(@"HKEY_CURRENT_USER\Control Panel\Cursors", "CursorBaseSize", null);
        if (baseSizeObj is int baseSize && baseSize > 0)
        {
            double s = baseSize / 32.0; // 32 is the default at 96 DPI
            return ((int)Math.Round(baseCx * s), (int)Math.Round(baseCy * s));
        }

        // Slider level 1..15 → derive approx. size
        var multObj = Registry.GetValue(@"HKEY_CURRENT_USER\Software\Microsoft\Accessibility", "CursorSize", null);
        if (multObj is int m && m >= 1)
        {
            double newH = baseCy + (m - 1) * (baseCy / 2.0);
            double s = newH / baseCy;
            return ((int)Math.Round(baseCx * s), (int)Math.Round(newH));
        }

        return (baseCx, baseCy); // Fallback: DPI size
    }

    /// <summary>
    /// Loads an embedded SVG (resourceNameWithoutExtension + ".svg"), renders it as a cursor bitmap
    /// at the system/Accessibility target size, maps hotspot from SVG coords, and returns a Cursor.
    /// </summary>
    public static Cursor? CreateCursorFromEmbeddedSvg(string resourceNameWithoutExtension, Control contextControl)
    {
        Assembly asm = Assembly.GetExecutingAssembly();

        using Stream? resource = asm.GetManifestResourceStream(resourceNameWithoutExtension + ".svg");
        if (resource == null) return null;

        // Read once into memory (for XML + Svg.NET)
        byte[] svgBytes;
        using (var ms = new MemoryStream())
        {
            resource.CopyTo(ms);
            svgBytes = ms.ToArray();
        }

        // 1) Hotspot from XML
        PointF hotspotSvg = default;

        // 2) Logical SVG size (viewBox / width/height)
        SizeF logical;
        bool hasLogical = TryReadSvgLogicalSize(svgBytes, out logical);
        if (!hasLogical || logical.Width <= 0 || logical.Height <= 0)
            logical = new SizeF(32f, 32f); // safe default

        // 3) Target cursor size (DPI + Accessibility)
        (int cx, int cy) = GetTargetCursorSize(contextControl);

        // 4) Render with Svg.NET; Svg.NET uses preserveAspectRatio="xMidYMid meet" by default.
        Bitmap bmp;
        Point hotspotPx;
        using (var msSvg = new MemoryStream(svgBytes, writable: false))
        {
            var doc = SvgDocument.Open<SvgDocument>(msSvg);
            hotspotSvg = ReadHotspotFromCircleAndHide(doc, cx, cy);
            bmp = doc.Draw(cx, cy); // returns a 32bpp ARGB bitmap
        }

        // 5) Map hotspot from SVG coords to pixels (meet + centered)
        // scale = min; offsets center the image inside cx×cy
        float scale = Math.Min(cx / logical.Width, cy / logical.Height);
        int offsetX = (int)Math.Round(hotspotSvg.X * scale + (cx - logical.Width * scale) * 0.5f);
        int offsetY = (int)Math.Round(hotspotSvg.Y * scale + (cy - logical.Height * scale) * 0.5f);
        hotspotPx = new Point(offsetX, offsetY);
        // 6) Create HCURSOR
        using (bmp)
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
