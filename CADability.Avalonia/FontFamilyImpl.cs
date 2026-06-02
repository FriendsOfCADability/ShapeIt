using CADability.Curve2D;
using CADability.Substitutes;
using System;

namespace CADability.Avalonia;

/// <summary>
/// Minimal Avalonia implementation of the abstract FontFamily substitute.
/// Text-outline rendering (GetOutline2D) is not yet implemented.
/// </summary>
public class FontFamilyImpl : Substitutes.FontFamily
{
    private readonly string _name;

    public FontFamilyImpl(string? name = null)
    {
        _name = string.IsNullOrEmpty(name) ? "Inter" : name;
    }

    public override string Name => _name;

    // Reasonable TrueType-like em metrics (2048 units per em)
    public override int GetEmHeight(FontStyle style) => 2048;
    public override int GetCellAscent(FontStyle style) => 1638;
    public override int GetCellDescent(FontStyle style) => 410;
    public override int GetLineSpacing(FontStyle style) => 2048;

    public override bool IsStyleAvailable(FontStyle fs) => true;

    public override Path2D[] GetOutline2D(int fontStyle, char c, int FontPrecision, out double width)
    {
        width = 0.5;
        return Array.Empty<Path2D>();
    }

    public override SizeF GetExtent(string textString, double length)
        => new SizeF((float)(length * 0.6 * textString.Length), (float)length);
}
