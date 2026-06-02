// SPDX-License-Identifier: MIT

using Avalonia.Media.Imaging;
using SkiaSharp;
using Svg.Skia;
using System.IO;

namespace CADability.Avalonia
{
    /// <summary>
    /// Cross-platform SVG rasterizer based on SkiaSharp (Svg.Skia). Used by both
    /// <see cref="SvgBitmapHelper"/> and <see cref="SvgCursorHelper"/> so the rendering
    /// path is identical and works on Windows and Linux alike (unlike System.Drawing,
    /// which is Windows-only on .NET 8).
    /// </summary>
    internal static class SkiaSvgRenderer
    {
        /// <summary>
        /// Renders an SVG to an Avalonia <see cref="Bitmap"/> of the given pixel size.
        /// The picture is scaled to fit (preserve aspect ratio, "meet") and centered,
        /// matching the SVG default preserveAspectRatio="xMidYMid meet".
        /// Returns null if loading or rendering fails.
        /// </summary>
        public static Bitmap? RenderToBitmap(Stream svgStream, int pixelWidth, int pixelHeight)
        {
            if (pixelWidth <= 0 || pixelHeight <= 0) return null;

            try
            {
                using var svg = new SKSvg();
                if (svg.Load(svgStream) is null || svg.Picture is null) return null;

                var pic = svg.Picture;
                var cull = pic.CullRect;
                if (cull.Width <= 0 || cull.Height <= 0) return null;

                float scale = System.Math.Min(pixelWidth / cull.Width, pixelHeight / cull.Height);
                float dx = (pixelWidth - cull.Width * scale) * 0.5f;
                float dy = (pixelHeight - cull.Height * scale) * 0.5f;

                var info = new SKImageInfo(pixelWidth, pixelHeight, SKColorType.Bgra8888, SKAlphaType.Premul);
                using var surface = SKSurface.Create(info);
                var canvas = surface.Canvas;
                canvas.Clear(SKColors.Transparent);

                canvas.Save();
                canvas.Translate(dx, dy);
                canvas.Scale(scale, scale);
                canvas.Translate(-cull.Left, -cull.Top);
                canvas.DrawPicture(pic);
                canvas.Restore();
                canvas.Flush();

                using var image = surface.Snapshot();
                using var data = image.Encode(SKEncodedImageFormat.Png, 100);
                using var outMs = new MemoryStream();
                data.SaveTo(outMs);
                outMs.Position = 0;
                return new Bitmap(outMs);
            }
            catch
            {
                return null;
            }
        }
    }
}
