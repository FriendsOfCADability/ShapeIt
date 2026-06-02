// SPDX-License-Identifier: MIT

using Avalonia;
using Avalonia.Input;
using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Xml.Linq;

using AvBitmap = Avalonia.Media.Imaging.Bitmap;

namespace CADability.Avalonia
{
    /// <summary>
    /// Creates Avalonia <see cref="Cursor"/> objects from embedded SVG resources.
    /// Avalonia counterpart of SvgCursorHelper in CADability.Forms.NET8.
    /// Rendering uses SkiaSharp (via <see cref="SkiaSvgRenderer"/>) so it works on
    /// Windows and Linux; the Forms version builds a Win32 HCURSOR via CreateIconIndirect.
    /// The hotspot is read from an element with id="hotspot" (a circle by authoring
    /// convention); that element is removed before rendering so it does not appear in
    /// the cursor bitmap.
    /// </summary>
    internal static class SvgCursorHelper
    {
        /// <summary>
        /// Loads an embedded SVG (logical name "{resourceBaseName}.svg"), renders it to a
        /// cursor bitmap of <paramref name="targetSize"/> pixels, maps the hotspot from SVG
        /// coordinates, and returns an Avalonia <see cref="Cursor"/>. Returns null if the
        /// resource is not found or rendering fails.
        /// </summary>
        public static Cursor? CreateCursorFromEmbeddedSvg(
            string resourceBaseName,
            int targetSize,
            Assembly? assembly = null)
        {
            if (string.IsNullOrWhiteSpace(resourceBaseName)) return null;
            assembly ??= Assembly.GetExecutingAssembly();

            byte[] svgBytes;
            using (var resource = TryOpenResourceStream(assembly, resourceBaseName, ".svg"))
            {
                if (resource == null) return null;
                using var ms = new MemoryStream();
                resource.CopyTo(ms);
                svgBytes = ms.ToArray();
            }

            try
            {
                XDocument xdoc;
                using (var ms = new MemoryStream(svgBytes, writable: false))
                    xdoc = XDocument.Load(ms);

                var root = xdoc.Root;
                if (root == null) return null;

                // Logical SVG size (viewBox / width-height), needed to map the hotspot.
                if (!TryReadSvgLogicalSize(root, out float logicalW, out float logicalH) ||
                    logicalW <= 0 || logicalH <= 0)
                {
                    logicalW = logicalH = 32f; // safe default
                }

                int cx = Math.Max(1, targetSize);
                int cy = Math.Max(1, targetSize);

                // Read the hotspot center (SVG user space) and remove the element so it
                // does not get rendered into the cursor.
                bool hasHotspot = TryTakeHotspot(root, out float hotspotX, out float hotspotY);

                // Re-serialize without the hotspot element.
                byte[] renderBytes = SerializeSvg(xdoc);

                AvBitmap? avBmp;
                using (var renderStream = new MemoryStream(renderBytes, writable: false))
                    avBmp = SkiaSvgRenderer.RenderToBitmap(renderStream, cx, cy);
                if (avBmp == null) return null;

                // Map hotspot from SVG user space to pixels (meet + centered),
                // matching SkiaSvgRenderer's preserveAspectRatio="xMidYMid meet".
                int offsetX = 0, offsetY = 0;
                if (hasHotspot)
                {
                    float scale = Math.Min(cx / logicalW, cy / logicalH);
                    offsetX = (int)Math.Round(hotspotX * scale + (cx - logicalW * scale) * 0.5f);
                    offsetY = (int)Math.Round(hotspotY * scale + (cy - logicalH * scale) * 0.5f);
                    offsetX = Math.Clamp(offsetX, 0, cx - 1);
                    offsetY = Math.Clamp(offsetY, 0, cy - 1);
                }

                return new Cursor(avBmp, new PixelPoint(offsetX, offsetY));
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Finds the element with id="hotspot", computes its center in SVG user space
        /// (circle/ellipse cx/cy or rect x+width/2, y+height/2) and removes it from the
        /// document so it is not rendered. Returns false if no hotspot element exists.
        /// </summary>
        private static bool TryTakeHotspot(XElement root, out float x, out float y)
        {
            x = 0f; y = 0f;
            var el = root.Descendants()
                .FirstOrDefault(e => string.Equals((string?)e.Attribute("id"), "hotspot",
                    StringComparison.Ordinal));
            if (el == null) return false;

            string local = el.Name.LocalName.ToLowerInvariant();
            bool ok;
            if (local == "circle" || local == "ellipse")
            {
                ok = TryParseFloat((string?)el.Attribute("cx"), out x) &
                     TryParseFloat((string?)el.Attribute("cy"), out y);
            }
            else if (local == "rect")
            {
                ok = TryParseFloat((string?)el.Attribute("x"), out float rx) &
                     TryParseFloat((string?)el.Attribute("y"), out float ry) &
                     TryParseFloat((string?)el.Attribute("width"), out float rw) &
                     TryParseFloat((string?)el.Attribute("height"), out float rh);
                x = rx + rw * 0.5f;
                y = ry + rh * 0.5f;
            }
            else
            {
                ok = false;
            }

            el.Remove();
            return ok;
        }

        private static byte[] SerializeSvg(XDocument xdoc)
        {
            using var ms = new MemoryStream();
            // SaveOptions.DisableFormatting keeps it compact; encoding does not matter for Skia.
            xdoc.Save(ms, SaveOptions.DisableFormatting);
            return ms.ToArray();
        }

        /// <summary>
        /// Reads the logical SVG viewport size from viewBox or width/height (px, pt, mm, cm).
        /// Returns false if neither is present.
        /// </summary>
        private static bool TryReadSvgLogicalSize(XElement root, out float width, out float height)
        {
            width = 0f; height = 0f;
            if (!string.Equals(root.Name.LocalName, "svg", StringComparison.OrdinalIgnoreCase))
                return false;

            // Prefer viewBox="minx miny width height"
            var viewBoxAttr = (string?)root.Attribute("viewBox");
            if (!string.IsNullOrWhiteSpace(viewBoxAttr))
            {
                var parts = viewBoxAttr.Split(new[] { ' ', ',', '\t', '\r', '\n' },
                    StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length == 4 &&
                    TryParseFloat(parts[2], out float vw) &&
                    TryParseFloat(parts[3], out float vh) &&
                    vw > 0 && vh > 0)
                {
                    width = vw; height = vh;
                    return true;
                }
            }

            // Fallback: width / height attributes
            if (TryParseSvgLength((string?)root.Attribute("width"), out float w) &&
                TryParseSvgLength((string?)root.Attribute("height"), out float h) &&
                w > 0 && h > 0)
            {
                width = w; height = h;
                return true;
            }

            return false;
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

        /// <summary>
        /// Opens a manifest resource stream by exact name or by suffix match
        /// (handles namespace-prefixed resource names).
        /// </summary>
        private static Stream? TryOpenResourceStream(Assembly asm, string baseName, string ext)
        {
            string exact = baseName.EndsWith(ext, StringComparison.OrdinalIgnoreCase)
                ? baseName
                : baseName + ext;

            var s = asm.GetManifestResourceStream(exact);
            if (s != null) return s;

            string suffix = exact.Replace('\\', '.').Replace('/', '.');
            var name = asm.GetManifestResourceNames()
                .FirstOrDefault(n => n.EndsWith(suffix, StringComparison.OrdinalIgnoreCase));
            return name != null ? asm.GetManifestResourceStream(name) : null;
        }
    }
}
