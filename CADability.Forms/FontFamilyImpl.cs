using CADability.Curve2D;
using System;
using System.Collections.Generic;
using System.Drawing.Drawing2D;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Wintellect.PowerCollections;

namespace CADability.Forms.NET8
{
    public class FontFamilyImpl: CADability.Substitutes.FontFamily
    {
        private System.Drawing.FontFamily ff;
        public FontFamilyImpl(string name)
        {
            ff = new System.Drawing.FontFamily(name);
        }
        public FontFamilyImpl()
        {
            ff = System.Drawing.FontFamily.GenericSansSerif;
        }
        public override string Name => ff.Name;
        public override int GetEmHeight(CADability.Substitutes.FontStyle style)
        {
            return ff.GetEmHeight((System.Drawing.FontStyle)style);
        }

        /// <summary>
        /// Returns the ascender metric for Windows.
        /// </summary>
        public override int GetCellAscent(CADability.Substitutes.FontStyle style)
        {
            return ff.GetCellAscent((System.Drawing.FontStyle)style);
        }

        /// <summary>
        /// Returns the descender metric for Windows.
        /// </summary>
        public override int GetCellDescent(CADability.Substitutes.FontStyle style)
        {
            return ff.GetCellDescent((System.Drawing.FontStyle)style);
        }

        /// <summary>
        /// Returns the distance between two consecutive lines of text for this <see cref='FontFamily'/> with the
        /// specified <see cref='FontStyle'/>.
        /// </summary>
        public override int GetLineSpacing(CADability.Substitutes.FontStyle style)
        {
            return ff.GetLineSpacing((System.Drawing.FontStyle)style);
        }

        struct KerningKey
        {
            public readonly string fontName;
            public readonly int fontStyle;
            public KerningKey(string fontName, int fontStyle)
            {
                this.fontName = fontName;
                this.fontStyle = fontStyle;
            }
            public override int GetHashCode()
            {
                return fontName.GetHashCode() ^ fontStyle;
            }
            public override bool Equals(object obj)
            {
                if (obj is KerningKey other)
                {
                    return other.fontName == fontName && other.fontStyle == fontStyle;
                }
                return false;
            }
        }

        static IntPtr hDC = Gdi.CreateCompatibleDC(IntPtr.Zero);
        static readonly Dictionary<KerningKey, Dictionary<Pair<char, char>, double>> kerning = new Dictionary<KerningKey, Dictionary<Pair<char, char>, double>>(); // Kerningtabellen
        private static void AddToPath2D(List<ICurve2D> addto, List<GeoPoint2D> points, bool spline, bool close, int FontPrecision)
        {
            if (spline)
            {
                for (int i = 0; i < points.Count - 3; i += 3)
                {

                    double[] knots = new double[2];
                    int[] multiplicities = new int[2];
                    knots[0] = 0;
                    knots[1] = 1;
                    multiplicities[0] = 4;
                    multiplicities[1] = 4;
                    GeoPoint2D[] pp = new GeoPoint2D[4];
                    points.CopyTo(i, pp, 0, 4);
                    BSpline2D bsp = new BSpline2D(pp, null, knots, multiplicities, 3, false, 0.0, 1.0);
                    // addto.Add(bsp);
                    switch (FontPrecision)
                    {
                        case 0: // grob
                            addto.Add(bsp.Approximate(true, 0.2));
                            break;
                        case 1: // mittel
                            addto.Add(bsp.Approximate(true, 0.05));
                            break;
                        case 2:
                            // addto.Add(bsp.Approximate(true, 0.005));
                            addto.Add(bsp); // wenn man den BSpline selbst zufügt, dann könnte man auch mit dynamischer Auflösung arbeiten
                            break;
                    }
                }
            }
            else
            {
                try
                {
                    if (points.Count > 1)
                    {
                        if (points.Count > 2 || points[0] != points[1])
                        {   // zwei identische Punkte gibt exception und ist langsam. Deshalb hier ausschließen
                            Polyline2D pl = new Polyline2D(points.ToArray());
                            addto.Add(pl);
                        }
                    }
                }
                catch (Polyline2DException) { } // nur zwei identische Punkte
            }
            points.RemoveRange(0, points.Count - 1); // den letzten als ersten drinlassen
        }

        public override Path2D[] GetOutline2D(int fontStyle, char c, int FontPrecision, out double width)
        {
            GraphicsPath path = new GraphicsPath();
            StringFormat sf = StringFormat.GenericTypographic.Clone() as StringFormat;
            sf.LineAlignment = StringAlignment.Near;
            sf.Alignment = StringAlignment.Near;
            if (!ff.IsStyleAvailable((System.Drawing.FontStyle)fontStyle))
            {
                if (ff.IsStyleAvailable(System.Drawing.FontStyle.Regular)) fontStyle = (int)FontStyle.Regular;
                if (ff.IsStyleAvailable(System.Drawing.FontStyle.Bold)) fontStyle = (int)FontStyle.Bold;
                if (ff.IsStyleAvailable(System.Drawing.FontStyle.Italic)) fontStyle = (int)FontStyle.Italic;
                if (ff.IsStyleAvailable(System.Drawing.FontStyle.Strikeout)) fontStyle = (int)FontStyle.Strikeout;
                if (ff.IsStyleAvailable(System.Drawing.FontStyle.Underline)) fontStyle = (int)FontStyle.Underline;
            }
            int fs = fontStyle;
            int em = ff.GetEmHeight((System.Drawing.FontStyle)fs);
            if (em == 0) em = 1000;
            Font font = new Font(ff, em, (System.Drawing.FontStyle)fs, GraphicsUnit.Pixel);
            IntPtr hfont = font.ToHfont();
            IntPtr oldfont = Gdi.SelectObject(hDC, hfont);
            Gdi.ABC[] abc = new Gdi.ABC[1];
            Gdi.GetCharABCWidths(hDC, (uint)c, (uint)c, abc);
            //int[] widths = new int[1]; // liefert das selbe wie GetCharABCWidths, nur die Summe halt.
            //Gdi.GetCharWidth32(hDC, (uint)c, (uint)c, widths);
            if (!kerning.ContainsKey(new KerningKey(ff.Name, fontStyle)))
            {
                KerningKey kk = new KerningKey(ff.Name, fontStyle);
                Dictionary<Pair<char, char>, double> pairs = new Dictionary<Pair<char, char>, double>();
                kerning[kk] = pairs;
                int num = Gdi.GetKerningPairs(hDC, 0, null);
                if (num > 0)
                {
                    Gdi.KERNINGPAIR[] kp = new Gdi.KERNINGPAIR[num];
                    int ok = Gdi.GetKerningPairs(hDC, num, kp);
                    for (int i = 0; i < kp.Length; ++i)
                    {
                        pairs[new Pair<char, char>((char)kp[i].wFirst, (char)kp[i].wSecond)] = kp[i].iKernAmount / (double)em;
                    }
                }
            }
            Gdi.SelectObject(hDC, oldfont);
            Gdi.DeleteObject(hfont);

            width = (abc[0].abcA + abc[0].abcB + abc[0].abcC) / (double)em;

            path.AddString(c.ToString(), ff, fs, 1.0f, new System.Drawing.PointF(0.0f, 0.0f), sf);
            List<Path2D> res = new List<Path2D>();
            if (path.PointCount > 0)
            {
                List<System.Drawing.PointF> pp = new List<System.Drawing.PointF>(path.PathPoints);
                List<byte> pt = new List<byte>(path.PathTypes);
                int last0 = -1;
                for (int i = 0; i < pt.Count; ++i)
                {
                    pp[i] = new System.Drawing.PointF(pp[i].X, (float)(1.0f - pp[i].Y));
                    if ((pt[i] & 0x01) == 0) last0 = i;
                    if ((pt[i] & 0x80) != 0 && last0 >= 0)
                    {
                        pt[i] = (byte)(pt[i] & 0x7F);
                        pt.Insert(i + 1, (byte)(pt[last0] | 0x81));
                        pp.Insert(i + 1, pp[last0]);
                        ++i;
                        last0 = -1;
                    }

                }
                if (pp == null || pp.Count == 0)
                {
                    return res.ToArray();
                }
                List<GeoPoint2D> current = new List<GeoPoint2D>();
                int mode = 0; // 0 noch nicht bekannt, 1: Linie, 3 Spline
                GeoPoint2D startPoint = GeoPoint2D.Origin;
                List<ICurve2D> segment = new List<ICurve2D>();
                bool close = false;
                for (int i = 0; i < pp.Count; ++i)
                {
                    switch (pt[i] & 0x03)
                    {
                        case 0: // neuer Anfang
                            if (current.Count > 1)
                            {
                                AddToPath2D(segment, current, mode == 3, close, FontPrecision);
                                res.Add(new Path2D(segment.ToArray()));
                            }
                            segment.Clear();
                            current.Clear();
                            current.Add(new GeoPoint2D(pp[i].X, pp[i].Y));
                            mode = 0;
                            break;
                        case 1:
                            if (mode == 3)
                            {   // Spline beenden, polylinie anfangen
                                AddToPath2D(segment, current, true, false, FontPrecision);
                            }
                            current.Add(new GeoPoint2D(pp[i].X, pp[i].Y));
                            mode = 1;
                            break;
                        case 2:
                        case 3:
                            if (mode == 1)
                            {
                                AddToPath2D(segment, current, false, false, FontPrecision);
                            }
                            current.Add(new GeoPoint2D(pp[i].X, pp[i].Y));
                            mode = 3;
                            break;
                    }
                    close = (pt[i] & 0x80) != 0;
                }

                if (current.Count > 1)
                {
                    AddToPath2D(segment, current, mode == 3, close, FontPrecision);
                    res.Add(new Path2D(segment.ToArray()));
                }
            }
            return res.ToArray();
        }

        public override bool IsStyleAvailable(CADability.Substitutes.FontStyle fs)
        {
            return ff.IsStyleAvailable((System.Drawing.FontStyle)fs);
        }

        public override CADability.Substitutes.SizeF GetExtent(string textString, double length)
        {
            using (Graphics graphics = Graphics.FromImage(new System.Drawing.Bitmap(1000, 100)))
            {
                using (Font font = new Font(Name, (float)length, GraphicsUnit.Pixel))
                {
                    System.Drawing.SizeF s = graphics.MeasureString(textString, font);
                    return new CADability.Substitutes.SizeF(s.Width, s.Height);
                }
            }
        }
    }
}
