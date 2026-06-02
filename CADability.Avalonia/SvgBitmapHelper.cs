// SPDX-License-Identifier: MIT

using Avalonia.Media.Imaging;
using CADability.UserInterface;
using SkiaSharp;
using Svg.Skia;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;

namespace CADability.Avalonia
{
    /// <summary>
    /// Creates Avalonia bitmaps from embedded SVG resources.
    /// Avalonia counterpart of SvgBitmapHelper in CADability.Forms.NET8.
    /// Rendering uses SkiaSharp (Svg.Skia) so it works cross-platform; the
    /// Forms version relies on System.Drawing.Common, which is Windows-only on .NET 8.
    /// </summary>
    internal static class SvgBitmapHelper
    {
        private const int DefaultIconSize = 16;

        // Cache per (resourceBaseName, pixelSize) to avoid re-rendering the same SVG repeatedly
        private static readonly Dictionary<(string name, int size), Bitmap> cachedBitmaps = new();

        /// <summary>
        /// Renders an embedded SVG resource to an Avalonia Bitmap at the requested pixel size.
        /// Searches for "Icons/{resourceBaseName}.svg" in the given assembly.
        /// Returns null if the resource is not found or rendering fails.
        /// </summary>
        /// <param name="resourceBaseName">
        /// Base name without extension, e.g. the menu command ID.
        /// Searched as "Icons/{resourceBaseName}.svg" in the assembly manifest.
        /// </param>
        /// <param name="pixelSize">Target width and height in pixels (square icon).</param>
        /// <param name="assembly">Assembly to search; defaults to the calling assembly.</param>
        public static Bitmap? CreateBitmapFromEmbeddedSvg(
            string resourceBaseName,
            int pixelSize = DefaultIconSize,
            Assembly? assembly = null)
        {
            if (string.IsNullOrWhiteSpace(resourceBaseName)) return null;

            assembly ??= Assembly.GetCallingAssembly();

            if (cachedBitmaps.TryGetValue((resourceBaseName, pixelSize), out var cached))
                return cached;

            // Try SVG resource: "Icons/{resourceBaseName}.svg"
            using var svgStream = TryOpenResourceStream(assembly, "Icons/" + resourceBaseName, ".svg");
            if (svgStream != null)
            {
                var avaloniaBmp = SkiaSvgRenderer.RenderToBitmap(svgStream, pixelSize, pixelSize);
                if (avaloniaBmp != null)
                {
                    cachedBitmaps[(resourceBaseName, pixelSize)] = avaloniaBmp;
                    return avaloniaBmp;
                }
            }

            // Raster fallback: the embedded BMP button strips (mirrors ButtonImages in
            // CADability.Forms.NET8). Used for toolbar/menu entries that have no SVG file.
            int imageIndex = MenuResource.FindImageIndex(resourceBaseName);
            if (imageIndex >= 0)
            {
                var rasterBmp = ButtonImagesHelper.CreateImage(imageIndex, pixelSize, assembly);
                if (rasterBmp != null)
                {
                    cachedBitmaps[(resourceBaseName, pixelSize)] = rasterBmp;
                    return rasterBmp;
                }
            }

            return null;
        }

        /// <summary>
        /// Tries to open a manifest resource stream by exact name or by suffix match
        /// (to handle namespace-prefixed resource names like "MyApp.Icons.zoom_in.svg").
        /// </summary>
        private static Stream? TryOpenResourceStream(Assembly asm, string baseName, string ext)
        {
            string exact = baseName.EndsWith(ext, StringComparison.OrdinalIgnoreCase)
                ? baseName
                : baseName + ext;

            var s = asm.GetManifestResourceStream(exact);
            if (s != null) return s;

            string suffix = (baseName + ext).Replace('\\', '.').Replace('/', '.');
            var name = asm.GetManifestResourceNames()
                .FirstOrDefault(n => n.EndsWith(suffix, StringComparison.OrdinalIgnoreCase));
            return name != null ? asm.GetManifestResourceStream(name) : null;
        }

        /// <summary>
        /// Clears the bitmap cache. Call when DPI or theme changes if necessary.
        /// Do not call while bitmaps are still in use by UI controls.
        /// </summary>
        public static void InvalidateCache()
        {
            foreach (var kv in cachedBitmaps)
            {
                try { kv.Value.Dispose(); } catch { }
            }
            cachedBitmaps.Clear();
        }
    }
}
