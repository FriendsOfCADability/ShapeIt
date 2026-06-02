// SPDX-License-Identifier: MIT

using Avalonia.Media.Imaging;
using SkiaSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;

namespace CADability.Avalonia
{
    /// <summary>
    /// Cross-platform raster fallback for toolbar/menu icons that have no SVG file,
    /// mirroring ButtonImages in CADability.Forms.NET8.
    ///
    /// The icons live in five embedded BMP strips (Buttons1..5.bmp), each an 800x15
    /// image holding fifty 16x15 tiles. They are concatenated in order, so the global
    /// image index returned by <c>MenuResource.FindImageIndex</c> addresses tile
    /// <c>index</c> across all strips. The pixel at (0,0) of each strip is the
    /// transparency key (classic WinForms control-gray); every pixel of that color is
    /// made fully transparent, just like <c>Bitmap.MakeTransparent</c> in the Forms version.
    ///
    /// Decoding and scaling use SkiaSharp so this works on Windows and Linux alike.
    /// </summary>
    internal static class ButtonImagesHelper
    {
        private const int TileWidth = 16;
        private const int TileHeight = 15;

        // Logical resource names; strips are embedded with LogicalName "Bitmaps.<file>".
        private static readonly string[] StripResources =
        {
            "Bitmaps.Buttons1.bmp",
            "Bitmaps.Buttons2.bmp",
            "Bitmaps.Buttons3.bmp",
            "Bitmaps.Buttons4.bmp",
            "Bitmaps.Buttons5.bmp",
        };

        private static readonly object gate = new();
        private static List<SKBitmap>? tiles; // lazily loaded; null until first use

        /// <summary>
        /// Returns the raster icon at the given global image index, scaled to a square
        /// of <paramref name="pixelSize"/>. Returns null if the index is out of range or
        /// the strips cannot be loaded.
        /// </summary>
        public static Bitmap? CreateImage(int index, int pixelSize, Assembly assembly)
        {
            if (index < 0 || pixelSize <= 0) return null;

            var list = EnsureLoaded(assembly);
            if (list == null || index >= list.Count) return null;

            try
            {
                var tile = list[index];
                var info = new SKImageInfo(pixelSize, pixelSize, SKColorType.Bgra8888, SKAlphaType.Premul);
                using var surface = SKSurface.Create(info);
                var canvas = surface.Canvas;
                canvas.Clear(SKColors.Transparent);

                // Stretch the 16x15 tile into the full square target, matching the
                // Forms ResizeBitmap behaviour. Nearest sampling keeps the pixel icons
                // crisp and avoids dark halos from the transparent key pixels.
                var dest = SKRect.Create(0, 0, pixelSize, pixelSize);
                using (var paint = new SKPaint { FilterQuality = SKFilterQuality.None, IsAntialias = false })
                    canvas.DrawBitmap(tile, dest, paint);
                canvas.Flush();

                using var image = surface.Snapshot();
                using var data = image.Encode(SKEncodedImageFormat.Png, 100);
                using var ms = new MemoryStream();
                data.SaveTo(ms);
                ms.Position = 0;
                return new Bitmap(ms);
            }
            catch
            {
                return null;
            }
        }

        private static List<SKBitmap>? EnsureLoaded(Assembly assembly)
        {
            if (tiles != null) return tiles;
            lock (gate)
            {
                if (tiles != null) return tiles;

                var list = new List<SKBitmap>();
                foreach (var res in StripResources)
                {
                    using var stream = TryOpenResourceStream(assembly, res);
                    if (stream == null) continue;

                    using var ms = new MemoryStream();
                    stream.CopyTo(ms);
                    ms.Position = 0;

                    using var strip = SKBitmap.Decode(ms);
                    if (strip == null || strip.Width < TileWidth || strip.Height < TileHeight)
                        continue;

                    SplitStrip(strip, list);
                }

                tiles = list;
                return tiles;
            }
        }

        /// <summary>
        /// Splits a horizontal strip into 16x15 tiles, applying the transparency key
        /// (color of the strip's top-left pixel) to every matching pixel.
        /// </summary>
        private static void SplitStrip(SKBitmap strip, List<SKBitmap> target)
        {
            SKColor key = strip.GetPixel(0, 0);
            int count = strip.Width / TileWidth;

            for (int i = 0; i < count; i++)
            {
                int baseX = i * TileWidth;
                var tile = new SKBitmap(TileWidth, TileHeight, SKColorType.Bgra8888, SKAlphaType.Unpremul);

                for (int y = 0; y < TileHeight; y++)
                {
                    for (int x = 0; x < TileWidth; x++)
                    {
                        SKColor c = strip.GetPixel(baseX + x, y);
                        bool isKey = c.Red == key.Red && c.Green == key.Green && c.Blue == key.Blue;
                        tile.SetPixel(x, y, isKey ? SKColors.Transparent : c);
                    }
                }

                target.Add(tile);
            }
        }

        /// <summary>
        /// Opens a manifest resource stream by exact name or by suffix match
        /// (handles namespace-prefixed resource names).
        /// </summary>
        private static Stream? TryOpenResourceStream(Assembly asm, string logicalName)
        {
            var s = asm.GetManifestResourceStream(logicalName);
            if (s != null) return s;

            var name = asm.GetManifestResourceNames()
                .FirstOrDefault(n => n.EndsWith(logicalName, StringComparison.OrdinalIgnoreCase));
            return name != null ? asm.GetManifestResourceStream(name) : null;
        }
    }
}
