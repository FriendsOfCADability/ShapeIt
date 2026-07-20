using CADability;
using CADability.Curve2D;
using CADability.Substitutes;
using SkiaSharp;
using System;
using System.Collections.Generic;

namespace CADability.Avalonia;

/// <summary>
/// Avalonia implementation of the FontFamily substitute. Glyph outlines,
/// metrics and advance widths are provided by SkiaSharp (cross-platform),
/// replacing the GDI+ path used in the WinForms implementation.
/// </summary>
public class FontFamilyImpl : Substitutes.FontFamily
{
    private readonly string _name;
    // One typeface per style-bit combination (Bold/Italic); created lazily.
    private readonly Dictionary<int, SKTypeface> _typefaces = new();

    public FontFamilyImpl(string? name = null)
    {
        _name = string.IsNullOrEmpty(name) ? "Inter" : name;
    }

    public override string Name => _name;

    // ── Typeface resolution ────────────────────────────────────────────
    private SKTypeface Resolve(int fontStyle)
    {
        if (_typefaces.TryGetValue(fontStyle, out var tf)) return tf;

        bool bold   = (fontStyle & (int)FontStyle.Bold)   != 0;
        bool italic = (fontStyle & (int)FontStyle.Italic) != 0;
        tf = SKTypeface.FromFamilyName(
                 _name,
                 bold ? SKFontStyleWeight.Bold : SKFontStyleWeight.Normal,
                 SKFontStyleWidth.Normal,
                 italic ? SKFontStyleSlant.Italic : SKFontStyleSlant.Upright)
             ?? SKTypeface.Default;
        _typefaces[fontStyle] = tf;
        return tf;
    }

    // ── Metrics (in font design units, mirroring the GDI FontFamily) ────
    public override int GetEmHeight(FontStyle style) => Resolve((int)style).UnitsPerEm;

    public override int GetCellAscent(FontStyle style)
    {
        var tf = Resolve((int)style);
        using var f = new SKFont(tf, tf.UnitsPerEm);
        f.GetFontMetrics(out var m);
        return (int)Math.Round(-m.Ascent);      // Skia ascent is negative (above baseline)
    }

    public override int GetCellDescent(FontStyle style)
    {
        var tf = Resolve((int)style);
        using var f = new SKFont(tf, tf.UnitsPerEm);
        f.GetFontMetrics(out var m);
        return (int)Math.Round(m.Descent);
    }

    public override int GetLineSpacing(FontStyle style)
    {
        var tf = Resolve((int)style);
        using var f = new SKFont(tf, tf.UnitsPerEm);
        f.GetFontMetrics(out var m);
        return (int)Math.Round(-m.Ascent + m.Descent + m.Leading);
    }

    public override bool IsStyleAvailable(FontStyle fs) => true;

    // ── Glyph outline ──────────────────────────────────────────────────
    public override Path2D[] GetOutline2D(int fontStyle, char c, int FontPrecision, out double width)
    {
        SKTypeface tf = Resolve(fontStyle);
        // Size 1.0 → path coordinates come out in "size-1" units, matching the
        // GDI AddString(size 1.0) convention used by the WinForms implementation.
        using SKFont font = new SKFont(tf, 1.0f);

        ushort glyph = font.GetGlyph(c);
        width = font.MeasureText(new ushort[] { glyph });   // advance in size-1 units

        if (glyph == 0) return Array.Empty<Path2D>();   // no glyph (e.g. .notdef)

        using SKPath gp = font.GetGlyphPath(glyph);
        if (gp == null || gp.PointCount == 0) return Array.Empty<Path2D>();

        // Vertical transform: y_final = yShift - y_skia, so the baseline (y_skia == 0)
        // lands at 1 - ascent/em — identical to the GDI outline after its 1.0 - y flip.
        double em     = tf.UnitsPerEm;
        double ascent = GetCellAscent((FontStyle)fontStyle);
        double yShift = 1.0 - ascent / em;
        GeoPoint2D P(SKPoint s) => new GeoPoint2D(s.X, yShift - s.Y);

        // Chord tolerance for bezier flattening (in size-1 units).
        double tol = FontPrecision switch { 0 => 0.08, 2 => 0.005, _ => 0.02 };

        var result = new List<Path2D>();
        var contour = new List<GeoPoint2D>();

        void FlushContour()
        {
            if (contour.Count >= 2)
            {
                // Close the loop so Path2D/Polyline2D report IsClosed == true.
                if (Geometry.Dist(contour[0], contour[contour.Count - 1]) > Precision.eps)
                    contour.Add(contour[0]);
                var pl = Polyline2D.MakePolyline2D(contour.ToArray());
                if (pl != null) result.Add(new Path2D(new ICurve2D[] { pl }));
            }
            contour.Clear();
        }

        var pts = new SKPoint[4];
        using var it = gp.CreateRawIterator();
        SKPathVerb verb;
        while ((verb = it.Next(pts)) != SKPathVerb.Done)
        {
            switch (verb)
            {
                case SKPathVerb.Move:
                    FlushContour();
                    contour.Add(P(pts[0]));
                    break;
                case SKPathVerb.Line:
                    contour.Add(P(pts[1]));
                    break;
                case SKPathVerb.Quad:
                    FlattenQuad(contour, P(pts[0]), P(pts[1]), P(pts[2]), tol, 0);
                    break;
                case SKPathVerb.Conic:
                    // Rare in glyphs; treat like a quad (weight ignored — visually adequate).
                    FlattenQuad(contour, P(pts[0]), P(pts[1]), P(pts[2]), tol, 0);
                    break;
                case SKPathVerb.Cubic:
                    FlattenCubic(contour, P(pts[0]), P(pts[1]), P(pts[2]), P(pts[3]), tol, 0);
                    break;
                case SKPathVerb.Close:
                    FlushContour();
                    break;
            }
        }
        FlushContour();
        return result.ToArray();
    }

    // ── Adaptive bezier flattening (the start point is already in the list) ──
    private const int MaxDepth = 10;

    private static void FlattenQuad(List<GeoPoint2D> outp, GeoPoint2D p0, GeoPoint2D p1,
                                    GeoPoint2D p2, double tol, int depth)
    {
        if (depth >= MaxDepth || Geometry.DistPL(p1, p0, p2) <= tol)
        {
            outp.Add(p2);
            return;
        }
        GeoPoint2D p01 = new GeoPoint2D(p0, p1);   // midpoint
        GeoPoint2D p12 = new GeoPoint2D(p1, p2);
        GeoPoint2D mid = new GeoPoint2D(p01, p12);
        FlattenQuad(outp, p0, p01, mid, tol, depth + 1);
        FlattenQuad(outp, mid, p12, p2, tol, depth + 1);
    }

    private static void FlattenCubic(List<GeoPoint2D> outp, GeoPoint2D p0, GeoPoint2D p1,
                                     GeoPoint2D p2, GeoPoint2D p3, double tol, int depth)
    {
        double d = Math.Max(Geometry.DistPL(p1, p0, p3), Geometry.DistPL(p2, p0, p3));
        if (depth >= MaxDepth || d <= tol)
        {
            outp.Add(p3);
            return;
        }
        GeoPoint2D p01  = new GeoPoint2D(p0, p1);
        GeoPoint2D p12  = new GeoPoint2D(p1, p2);
        GeoPoint2D p23  = new GeoPoint2D(p2, p3);
        GeoPoint2D p012 = new GeoPoint2D(p01, p12);
        GeoPoint2D p123 = new GeoPoint2D(p12, p23);
        GeoPoint2D mid  = new GeoPoint2D(p012, p123);
        FlattenCubic(outp, p0, p01, p012, mid, tol, depth + 1);
        FlattenCubic(outp, mid, p123, p23, p3, tol, depth + 1);
    }

    public override SizeF GetExtent(string textString, double length)
    {
        using var paint = new SKPaint { Typeface = Resolve(0), TextSize = (float)length };
        return new SizeF(paint.MeasureText(textString), (float)length);
    }
}
