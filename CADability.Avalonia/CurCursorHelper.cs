// SPDX-License-Identifier: MIT

using Avalonia;
using Avalonia.Input;
using SkiaSharp;
using System;
using System.Buffers.Binary;
using System.IO;
using System.Linq;
using System.Reflection;

using AvBitmap = Avalonia.Media.Imaging.Bitmap;

namespace CADability.Avalonia
{
    /// <summary>
    /// Creates Avalonia <see cref="Cursor"/> objects from embedded Windows *.cur resources.
    /// This is the raster fallback for cursors that have no SVG variant (mirrors the
    /// <c>new Cursor(stream)</c> path in CADability.Forms.NET8.CadCanvas).
    ///
    /// A .cur file is an icon container (ICONDIR + ICONDIRENTRY[]) whose entries store the
    /// cursor hotspot in the two words that an .ico would use for color planes / bit count.
    /// We parse the directory ourselves to read the hotspot, then wrap the chosen image in a
    /// one-entry .ico and let SkiaSharp decode it (BMP/PNG inside ICO, including the 1-bit AND
    /// transparency mask). This keeps everything cross-platform - no System.Drawing, no Win32.
    /// </summary>
    internal static class CurCursorHelper
    {
        /// <summary>
        /// Loads an embedded cursor ("{resourceBaseName}.cur"), decodes its best image and
        /// returns an Avalonia <see cref="Cursor"/> with the correct hotspot. Returns null if
        /// the resource is missing or cannot be decoded.
        /// </summary>
        public static Cursor? CreateCursorFromEmbeddedCur(string resourceBaseName, Assembly? assembly = null)
        {
            if (string.IsNullOrWhiteSpace(resourceBaseName)) return null;
            assembly ??= Assembly.GetExecutingAssembly();

            byte[] cur;
            using (var resource = TryOpenResourceStream(assembly, resourceBaseName, ".cur"))
            {
                if (resource == null) return null;
                using var ms = new MemoryStream();
                resource.CopyTo(ms);
                cur = ms.ToArray();
            }

            try
            {
                if (cur.Length < 6) return null;
                ushort count = BinaryPrimitives.ReadUInt16LittleEndian(cur.AsSpan(4, 2));
                if (count == 0 || cur.Length < 6 + count * 16) return null;

                // Pick the entry with the largest area (first wins on a tie). The chosen
                // entry's hotspot is the authoritative one.
                int bestArea = -1, hotspotX = 0, hotspotY = 0, imageSize = 0, imageOffset = 0;
                bool found = false;
                for (int i = 0; i < count; i++)
                {
                    int o = 6 + i * 16;
                    int w = cur[o] == 0 ? 256 : cur[o];
                    int h = cur[o + 1] == 0 ? 256 : cur[o + 1];
                    int hx = BinaryPrimitives.ReadUInt16LittleEndian(cur.AsSpan(o + 4, 2));
                    int hy = BinaryPrimitives.ReadUInt16LittleEndian(cur.AsSpan(o + 6, 2));
                    int size = (int)BinaryPrimitives.ReadUInt32LittleEndian(cur.AsSpan(o + 8, 4));
                    int off = (int)BinaryPrimitives.ReadUInt32LittleEndian(cur.AsSpan(o + 12, 4));

                    if (size <= 0 || off < 0 || off + size > cur.Length) continue;
                    if (w * h > bestArea)
                    {
                        bestArea = w * h;
                        hotspotX = hx; hotspotY = hy;
                        imageSize = size; imageOffset = off;
                        found = true;
                    }
                }
                if (!found) return null;

                var skBitmap = DecodeImageAsIco(cur, imageOffset, imageSize);
                if (skBitmap == null) return null;

                using (skBitmap)
                {
                    var avBmp = ToAvaloniaBitmap(skBitmap);
                    if (avBmp == null) return null;

                    int hx = Math.Clamp(hotspotX, 0, Math.Max(0, skBitmap.Width - 1));
                    int hy = Math.Clamp(hotspotY, 0, Math.Max(0, skBitmap.Height - 1));
                    return new Cursor(avBmp, new PixelPoint(hx, hy));
                }
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Wraps a single cursor image (BMP DIB or PNG) in a minimal one-entry .ico container
        /// and decodes it with SkiaSharp. The hotspot words are cleared because an .ico expects
        /// color-plane / bit-count there.
        /// </summary>
        private static SKBitmap? DecodeImageAsIco(byte[] cur, int imageOffset, int imageSize)
        {
            byte[] ico = new byte[6 + 16 + imageSize];

            // ICONDIR: reserved=0, type=1 (icon), count=1
            ico[2] = 1;
            ico[4] = 1;

            // ICONDIRENTRY: copy width/height/colorCount/reserved from the cursor entry.
            // The cursor's directory entry sits at offset 6 in the file for the matching image;
            // but width/height are per-entry, so re-read them from the byte just before.
            // We copy the 16-byte entry then patch the fields that differ for an .ico.
            // Locate the entry whose image offset matches.
            int entryOffset = FindEntryOffset(cur, imageOffset);
            Array.Copy(cur, entryOffset, ico, 6, 16);

            // Clear hotspot words (planes/bitcount for ico) and fix size + offset.
            ico[6 + 4] = 0; ico[6 + 5] = 0; // planes
            ico[6 + 6] = 0; ico[6 + 7] = 0; // bit count
            BinaryPrimitives.WriteUInt32LittleEndian(ico.AsSpan(6 + 8, 4), (uint)imageSize);
            BinaryPrimitives.WriteUInt32LittleEndian(ico.AsSpan(6 + 12, 4), (uint)(6 + 16));

            Array.Copy(cur, imageOffset, ico, 6 + 16, imageSize);

            return SKBitmap.Decode(ico);
        }

        private static int FindEntryOffset(byte[] cur, int imageOffset)
        {
            ushort count = BinaryPrimitives.ReadUInt16LittleEndian(cur.AsSpan(4, 2));
            for (int i = 0; i < count; i++)
            {
                int o = 6 + i * 16;
                int off = (int)BinaryPrimitives.ReadUInt32LittleEndian(cur.AsSpan(o + 12, 4));
                if (off == imageOffset) return o;
            }
            return 6; // fallback to first entry
        }

        private static AvBitmap? ToAvaloniaBitmap(SKBitmap skBitmap)
        {
            using var image = SKImage.FromBitmap(skBitmap);
            if (image == null) return null;
            using var data = image.Encode(SKEncodedImageFormat.Png, 100);
            using var ms = new MemoryStream();
            data.SaveTo(ms);
            ms.Position = 0;
            return new AvBitmap(ms);
        }

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
