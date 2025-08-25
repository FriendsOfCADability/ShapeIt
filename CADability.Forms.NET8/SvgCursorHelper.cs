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
