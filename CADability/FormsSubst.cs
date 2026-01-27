using CADability.Curve2D;
using StbImageSharp;
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.InteropServices;
using Wintellect.PowerCollections;


// MouseEventArgs, MouseButtons, Keys, 
// MessageBox, Clipboard, IDataObject, Control.ModifierKeys

/// <summary>
/// Simple substitues for Windows.Forms enum and simple data objects. Copy of the Windows.Forms code.
/// </summary>
namespace CADability.Substitutes
{
    public abstract class FontFamily
    {
        public virtual string Name { get; }
        public abstract int GetEmHeight(FontStyle style);

        /// <summary>
        /// Returns the ascender metric for Windows.
        /// </summary>
        public abstract int GetCellAscent(FontStyle style);

        /// <summary>
        /// Returns the descender metric for Windows.
        /// </summary>
        public abstract int GetCellDescent(FontStyle style);
        /// <summary>
        /// Returns the distance between two consecutive lines of text for this <see cref='FontFamily'/> with the
        /// specified <see cref='FontStyle'/>.
        /// </summary>
        public abstract int GetLineSpacing(FontStyle style);

        public abstract Path2D[] GetOutline2D(int fontStyle, char c, int FontPrecision, out double width);

        public abstract bool IsStyleAvailable(FontStyle fs);

        public abstract SizeF GetExtent(string textString, double length);
    }
    public struct Bitmap
    {
        public static Bitmap Empty => new Bitmap { Data = null, Width = 0, Height = 0 };
        public bool IsEmpty => Data == null || Data.Length == 0;
        public int Width { get; set; }
        public int Height { get; set; }
        public byte[] Data { get; set; }
        public Color GetPixel(int x, int y)
        {
            if (x >= Width || y >= Height) throw new ArgumentOutOfRangeException();

            int i = (y * Width + x) * 4;
            byte r = Data[i + 0];
            byte g = Data[i + 1];
            byte b = Data[i + 2];
            byte a = Data[i + 3];

            return Color.FromArgb(a, r, g, b);
        }
    }
    public struct Rectangle : IEquatable<Rectangle>
    {
        public int X { get; set; }
        public int Y { get; set; }
        public int Width { get; set; }
        public int Height { get; set; }

        // ----------- Abgeleitete Properties -----------

        public int Left => X;
        public int Top => Y;
        public int Right => X + Width;
        public int Bottom => Y + Height;

        public static Rectangle Empty => new Rectangle(0, 0, 0, 0);
        public bool IsEmpty => Width <= 0 || Height <= 0;

        public Point Location
        {
            get => new Point { X = X, Y = Y };
            set { X = value.X; Y = value.Y; }
        }

        public Size Size
        {
            get => new Size { Width = Width, Height = Height };
            set { Width = value.Width; Height = value.Height; }
        }

        // ----------- Konstruktor -----------

        public Rectangle(int x, int y, int width, int height)
        {
            X = x;
            Y = y;
            Width = width;
            Height = height;
        }

        // ----------- Methoden -----------

        public bool Contains(int x, int y)
            => x >= Left && x < Right && y >= Top && y < Bottom;

        public bool Contains(Point p)
            => Contains(p.X, p.Y);

        public bool IntersectsWith(Rectangle r)
            => r.Left < Right && Left < r.Right &&
               r.Top < Bottom && Top < r.Bottom;

        public static Rectangle Intersect(Rectangle a, Rectangle b)
        {
            int left = Math.Max(a.Left, b.Left);
            int top = Math.Max(a.Top, b.Top);
            int right = Math.Min(a.Right, b.Right);
            int bottom = Math.Min(a.Bottom, b.Bottom);

            if (right <= left || bottom <= top)
                return new Rectangle();

            return new Rectangle(left, top, right - left, bottom - top);
        }

        // ----------- Equality -----------

        public override bool Equals(object obj)
            => obj is Rectangle r && Equals(r);

        public bool Equals(Rectangle other)
            => X == other.X && Y == other.Y &&
               Width == other.Width && Height == other.Height;

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + X;
                hash = hash * 31 + Y;
                hash = hash * 31 + Width;
                hash = hash * 31 + Height;
                return hash;
            }
        }

        public static bool operator ==(Rectangle a, Rectangle b) => a.Equals(b);
        public static bool operator !=(Rectangle a, Rectangle b) => !a.Equals(b);

        public override string ToString()
            => $"Rectangle [X={X}, Y={Y}, Width={Width}, Height={Height}]";

        public void Inflate(int v1, int v2)
        {
            X -= v1;
            Y -= v2;
            Width += 2 * v1;
            Height += 2 * v2;
        }

        public static Rectangle FromLTRB(int left, int top, int right, int bottom)
        {
            return new Rectangle(left, top, right - left, bottom - top);
        }

    }
    public struct RectangleF : IEquatable<RectangleF>
    {
        public float X { get; set; }
        public float Y { get; set; }
        public float Width { get; set; }
        public float Height { get; set; }

        // -------- Derived Properties --------

        public float Left => X;
        public float Top => Y;
        public float Right => X + Width;
        public float Bottom => Y + Height;

        public bool IsEmpty => Width <= 0f || Height <= 0f;

        public PointF Location
        {
            get => new PointF(X, Y);
            set { X = value.X; Y = value.Y; }
        }

        public SizeF SizeF
        {
            get => new SizeF(Width, Height);
        }

        public PointF Center => new PointF(X + Width / 2f, Y + Height / 2f);

        // -------- Constructors --------

        public RectangleF(float x, float y, float width, float height)
        {
            X = x;
            Y = y;
            Width = width;
            Height = height;
        }

        public static readonly RectangleF Empty = new RectangleF();

        // -------- Methods --------

        public bool Contains(float x, float y)
            => x >= Left && x < Right && y >= Top && y < Bottom;

        public bool Contains(PointF p)
            => Contains(p.X, p.Y);

        public bool IntersectsWith(RectangleF r)
            => r.Left < Right && Left < r.Right &&
               r.Top < Bottom && Top < r.Bottom;

        public static RectangleF Intersect(RectangleF a, RectangleF b)
        {
            float left = Math.Max(a.Left, b.Left);
            float top = Math.Max(a.Top, b.Top);
            float right = Math.Min(a.Right, b.Right);
            float bottom = Math.Min(a.Bottom, b.Bottom);

            if (right <= left || bottom <= top)
                return Empty;

            return new RectangleF(left, top, right - left, bottom - top);
        }

        // -------- Equality --------

        public override bool Equals(object obj)
            => obj is RectangleF r && Equals(r);

        public bool Equals(RectangleF other)
            => X == other.X && Y == other.Y &&
               Width == other.Width && Height == other.Height;

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + X.GetHashCode();
                hash = hash * 31 + Y.GetHashCode();
                hash = hash * 31 + Width.GetHashCode();
                hash = hash * 31 + Height.GetHashCode();
                return hash;
            }
        }

        public static bool operator ==(RectangleF a, RectangleF b) => a.Equals(b);
        public static bool operator !=(RectangleF a, RectangleF b) => !a.Equals(b);
        public static implicit operator RectangleF(Rectangle r)
            => new RectangleF(r.X, r.Y, r.Width, r.Height);

        public override string ToString()
            => $"RectangleF [X={X}, Y={Y}, Width={Width}, Height={Height}]";
    }
    public struct Size
    {
        public static readonly Size Empty = new Size { Width = 0, Height = 0 };

        public Size(int width, int height) : this()
        {
            Width = width;
            Height = height;
        }

        public int Width { get; set; }
        public int Height { get; set; }
    }
    public struct SizeF
    {
        public static readonly Size Empty = new Size { Width = 0, Height = 0 };

        public SizeF(float width, float height) : this()
        {
            Width = width;
            Height = height;
        }

        public float Width { get; set; }
        public float Height { get; set; }
    }
    public struct Point
    {
        public Point(int x, int y)
        {
            X = x;
            Y = y;
        }
        public int X { get; set; }
        public int Y { get; set; }
    }
    public struct PointF : IEquatable<PointF>
    {
        public float X { get; set; }
        public float Y { get; set; }

        public PointF(float x, float y)
        {
            X = x;
            Y = y;
        }

        public static readonly PointF Empty = new PointF(0, 0);

        public bool IsEmpty => X == 0f && Y == 0f;

        public override bool Equals(object obj)
            => obj is PointF p && Equals(p);

        public bool Equals(PointF other)
            => X == other.X && Y == other.Y;

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + X.GetHashCode();
                hash = hash * 31 + Y.GetHashCode();
                return hash;
            }
        }

        public static bool operator ==(PointF a, PointF b) => a.Equals(b);
        public static bool operator !=(PointF a, PointF b) => !a.Equals(b);

        public override string ToString() => $"PointF [X={X}, Y={Y}]";
    }

    [JsonVersion(serializeAsStruct = true, version = 1)]
    public struct Color : IEquatable<Color>, IJsonSerialize
    {
        private readonly int _argb;
        private readonly string _name;
        private readonly bool _isNamed;

        public static readonly Color Empty = new Color(0, null, false);


        private Color(int argb, string name, bool isNamed)
        {
            _argb = argb;
            _name = name;
            _isNamed = isNamed;
        }

        // ---------------- Properties ----------------

        public byte A => (byte)(_argb >> 24);
        public byte R => (byte)(_argb >> 16);
        public byte G => (byte)(_argb >> 8);
        public byte B => (byte)(_argb);

        public string Name => _name ?? string.Empty;
        public bool IsNamedColor => _isNamed;
        public bool IsEmpty => _argb == 0 && !_isNamed;

        public static Color AliceBlue => FromArgb(unchecked((int)0xFFF0F8FF));
        public static Color AntiqueWhite => FromArgb(unchecked((int)0xFFFAEBD7));
        public static Color Aqua => FromArgb(unchecked((int)0xFF00FFFF));
        public static Color Aquamarine => FromArgb(unchecked((int)0xFF7FFFD4));
        public static Color Azure => FromArgb(unchecked((int)0xFFF0FFFF));
        public static Color Beige => FromArgb(unchecked((int)0xFFF5F5DC));
        public static Color Bisque => FromArgb(unchecked((int)0xFFFFE4C4));
        public static Color Black => FromArgb(unchecked((int)0xFF000000));
        public static Color BlanchedAlmond => FromArgb(unchecked((int)0xFFFFEBCD));
        public static Color Blue => FromArgb(unchecked((int)0xFF0000FF));
        public static Color BlueViolet => FromArgb(unchecked((int)0xFF8A2BE2));
        public static Color Brown => FromArgb(unchecked((int)0xFFA52A2A));
        public static Color BurlyWood => FromArgb(unchecked((int)0xFFDEB887));
        public static Color CadetBlue => FromArgb(unchecked((int)0xFF5F9EA0));
        public static Color Chartreuse => FromArgb(unchecked((int)0xFF7FFF00));
        public static Color Chocolate => FromArgb(unchecked((int)0xFFD2691E));
        public static Color Coral => FromArgb(unchecked((int)0xFFFF7F50));
        public static Color CornflowerBlue => FromArgb(unchecked((int)0xFF6495ED));
        public static Color Cornsilk => FromArgb(unchecked((int)0xFFFFF8DC));
        public static Color Crimson => FromArgb(unchecked((int)0xFFDC143C));
        public static Color Cyan => FromArgb(unchecked((int)0xFF00FFFF));
        public static Color DarkBlue => FromArgb(unchecked((int)0xFF00008B));
        public static Color DarkCyan => FromArgb(unchecked((int)0xFF008B8B));
        public static Color DarkGoldenRod => FromArgb(unchecked((int)0xFFB8860B));
        public static Color DarkGray => FromArgb(unchecked((int)0xFFA9A9A9));
        public static Color DarkGrey => FromArgb(unchecked((int)0xFFA9A9A9));
        public static Color DarkGreen => FromArgb(unchecked((int)0xFF006400));
        public static Color DarkKhaki => FromArgb(unchecked((int)0xFFBDB76B));
        public static Color DarkMagenta => FromArgb(unchecked((int)0xFF8B008B));
        public static Color DarkOliveGreen => FromArgb(unchecked((int)0xFF556B2F));
        public static Color DarkOrange => FromArgb(unchecked((int)0xFFFF8C00));
        public static Color DarkOrchid => FromArgb(unchecked((int)0xFF9932CC));
        public static Color DarkRed => FromArgb(unchecked((int)0xFF8B0000));
        public static Color DarkSalmon => FromArgb(unchecked((int)0xFFE9967A));
        public static Color DarkSeaGreen => FromArgb(unchecked((int)0xFF8FBC8F));
        public static Color DarkSlateBlue => FromArgb(unchecked((int)0xFF483D8B));
        public static Color DarkSlateGray => FromArgb(unchecked((int)0xFF2F4F4F));
        public static Color DarkSlateGrey => FromArgb(unchecked((int)0xFF2F4F4F));
        public static Color DarkTurquoise => FromArgb(unchecked((int)0xFF00CED1));
        public static Color DarkViolet => FromArgb(unchecked((int)0xFF9400D3));
        public static Color DeepPink => FromArgb(unchecked((int)0xFFFF1493));
        public static Color DeepSkyBlue => FromArgb(unchecked((int)0xFF00BFFF));
        public static Color DimGray => FromArgb(unchecked((int)0xFF696969));
        public static Color DimGrey => FromArgb(unchecked((int)0xFF696969));
        public static Color DodgerBlue => FromArgb(unchecked((int)0xFF1E90FF));
        public static Color FireBrick => FromArgb(unchecked((int)0xFFB22222));
        public static Color FloralWhite => FromArgb(unchecked((int)0xFFFFFAF0));
        public static Color ForestGreen => FromArgb(unchecked((int)0xFF228B22));
        public static Color Fuchsia => FromArgb(unchecked((int)0xFFFF00FF));
        public static Color Gainsboro => FromArgb(unchecked((int)0xFFDCDCDC));
        public static Color GhostWhite => FromArgb(unchecked((int)0xFFF8F8FF));
        public static Color Gold => FromArgb(unchecked((int)0xFFFFD700));
        public static Color GoldenRod => FromArgb(unchecked((int)0xFFDAA520));
        public static Color Gray => FromArgb(unchecked((int)0xFF808080));
        public static Color Grey => FromArgb(unchecked((int)0xFF808080));
        public static Color Green => FromArgb(unchecked((int)0xFF008000));
        public static Color GreenYellow => FromArgb(unchecked((int)0xFFADFF2F));
        public static Color HoneyDew => FromArgb(unchecked((int)0xFFF0FFF0));
        public static Color HotPink => FromArgb(unchecked((int)0xFFFF69B4));
        public static Color IndianRed => FromArgb(unchecked((int)0xFFCD5C5C));
        public static Color Indigo => FromArgb(unchecked((int)0xFF4B0082));
        public static Color Ivory => FromArgb(unchecked((int)0xFFFFFFF0));
        public static Color Khaki => FromArgb(unchecked((int)0xFFF0E68C));
        public static Color Lavender => FromArgb(unchecked((int)0xFFE6E6FA));
        public static Color LavenderBlush => FromArgb(unchecked((int)0xFFFFF0F5));
        public static Color LawnGreen => FromArgb(unchecked((int)0xFF7CFC00));
        public static Color LemonChiffon => FromArgb(unchecked((int)0xFFFFFACD));
        public static Color LightBlue => FromArgb(unchecked((int)0xFFADD8E6));
        public static Color LightCoral => FromArgb(unchecked((int)0xFFF08080));
        public static Color LightCyan => FromArgb(unchecked((int)0xFFE0FFFF));
        public static Color LightGoldenRodYellow => FromArgb(unchecked((int)0xFFFAFAD2));
        public static Color LightGray => FromArgb(unchecked((int)0xFFD3D3D3));
        public static Color LightGrey => FromArgb(unchecked((int)0xFFD3D3D3));
        public static Color LightGreen => FromArgb(unchecked((int)0xFF90EE90));
        public static Color LightPink => FromArgb(unchecked((int)0xFFFFB6C1));
        public static Color LightSalmon => FromArgb(unchecked((int)0xFFFFA07A));
        public static Color LightSeaGreen => FromArgb(unchecked((int)0xFF20B2AA));
        public static Color LightSkyBlue => FromArgb(unchecked((int)0xFF87CEFA));
        public static Color LightSlateGray => FromArgb(unchecked((int)0xFF778899));
        public static Color LightSlateGrey => FromArgb(unchecked((int)0xFF778899));
        public static Color LightSteelBlue => FromArgb(unchecked((int)0xFFB0C4DE));
        public static Color LightYellow => FromArgb(unchecked((int)0xFFFFFFE0));
        public static Color Lime => FromArgb(unchecked((int)0xFF00FF00));
        public static Color LimeGreen => FromArgb(unchecked((int)0xFF32CD32));
        public static Color Linen => FromArgb(unchecked((int)0xFFFAF0E6));
        public static Color Magenta => FromArgb(unchecked((int)0xFFFF00FF));
        public static Color Maroon => FromArgb(unchecked((int)0xFF800000));
        public static Color MediumAquaMarine => FromArgb(unchecked((int)0xFF66CDAA));
        public static Color MediumBlue => FromArgb(unchecked((int)0xFF0000CD));
        public static Color MediumOrchid => FromArgb(unchecked((int)0xFFBA55D3));
        public static Color MediumPurple => FromArgb(unchecked((int)0xFF9370DB));
        public static Color MediumSeaGreen => FromArgb(unchecked((int)0xFF3CB371));
        public static Color MediumSlateBlue => FromArgb(unchecked((int)0xFF7B68EE));
        public static Color MediumSpringGreen => FromArgb(unchecked((int)0xFF00FA9A));
        public static Color MediumTurquoise => FromArgb(unchecked((int)0xFF48D1CC));
        public static Color MediumVioletRed => FromArgb(unchecked((int)0xFFC71585));
        public static Color MidnightBlue => FromArgb(unchecked((int)0xFF191970));
        public static Color MintCream => FromArgb(unchecked((int)0xFFF5FFFA));
        public static Color MistyRose => FromArgb(unchecked((int)0xFFFFE4E1));
        public static Color Moccasin => FromArgb(unchecked((int)0xFFFFE4B5));
        public static Color NavajoWhite => FromArgb(unchecked((int)0xFFFFDEAD));
        public static Color Navy => FromArgb(unchecked((int)0xFF000080));
        public static Color OldLace => FromArgb(unchecked((int)0xFFFDF5E6));
        public static Color Olive => FromArgb(unchecked((int)0xFF808000));
        public static Color OliveDrab => FromArgb(unchecked((int)0xFF6B8E23));
        public static Color Orange => FromArgb(unchecked((int)0xFFFFA500));
        public static Color OrangeRed => FromArgb(unchecked((int)0xFFFF4500));
        public static Color Orchid => FromArgb(unchecked((int)0xFFDA70D6));
        public static Color PaleGoldenRod => FromArgb(unchecked((int)0xFFEEE8AA));
        public static Color PaleGreen => FromArgb(unchecked((int)0xFF98FB98));
        public static Color PaleTurquoise => FromArgb(unchecked((int)0xFFAFEEEE));
        public static Color PaleVioletRed => FromArgb(unchecked((int)0xFFDB7093));
        public static Color PapayaWhip => FromArgb(unchecked((int)0xFFFFEFD5));
        public static Color PeachPuff => FromArgb(unchecked((int)0xFFFFDAB9));
        public static Color Peru => FromArgb(unchecked((int)0xFFCD853F));
        public static Color Pink => FromArgb(unchecked((int)0xFFFFC0CB));
        public static Color Plum => FromArgb(unchecked((int)0xFFDDA0DD));
        public static Color PowderBlue => FromArgb(unchecked((int)0xFFB0E0E6));
        public static Color Purple => FromArgb(unchecked((int)0xFF800080));
        public static Color RebeccaPurple => FromArgb(unchecked((int)0xFF663399));
        public static Color Red => FromArgb(unchecked((int)0xFFFF0000));
        public static Color RosyBrown => FromArgb(unchecked((int)0xFFBC8F8F));
        public static Color RoyalBlue => FromArgb(unchecked((int)0xFF4169E1));
        public static Color SaddleBrown => FromArgb(unchecked((int)0xFF8B4513));
        public static Color Salmon => FromArgb(unchecked((int)0xFFFA8072));
        public static Color SandyBrown => FromArgb(unchecked((int)0xFFF4A460));
        public static Color SeaGreen => FromArgb(unchecked((int)0xFF2E8B57));
        public static Color SeaShell => FromArgb(unchecked((int)0xFFFFF5EE));
        public static Color Sienna => FromArgb(unchecked((int)0xFFA0522D));
        public static Color Silver => FromArgb(unchecked((int)0xFFC0C0C0));
        public static Color SkyBlue => FromArgb(unchecked((int)0xFF87CEEB));
        public static Color SlateBlue => FromArgb(unchecked((int)0xFF6A5ACD));
        public static Color SlateGray => FromArgb(unchecked((int)0xFF708090));
        public static Color SlateGrey => FromArgb(unchecked((int)0xFF708090));
        public static Color Snow => FromArgb(unchecked((int)0xFFFFFAFA));
        public static Color SpringGreen => FromArgb(unchecked((int)0xFF00FF7F));
        public static Color SteelBlue => FromArgb(unchecked((int)0xFF4682B4));
        public static Color Tan => FromArgb(unchecked((int)0xFFD2B48C));
        public static Color Teal => FromArgb(unchecked((int)0xFF008080));
        public static Color Thistle => FromArgb(unchecked((int)0xFFD8BFD8));
        public static Color Tomato => FromArgb(unchecked((int)0xFFFF6347));
        public static Color Turquoise => FromArgb(unchecked((int)0xFF40E0D0));
        public static Color Violet => FromArgb(unchecked((int)0xFFEE82EE));
        public static Color Wheat => FromArgb(unchecked((int)0xFFF5DEB3));
        public static Color White => FromArgb(unchecked((int)0xFFFFFFFF));
        public static Color WhiteSmoke => FromArgb(unchecked((int)0xFFF5F5F5));
        public static Color Yellow => FromArgb(unchecked((int)0xFFFFFF00));
        public static Color YellowGreen => FromArgb(unchecked((int)0xFF9ACD32));

        // ---------------- Factory ----------------

        public static Color FromArgb(int argb)
            => new Color(argb, null, false);

        public static Color FromArgb(int alpha, Color baseColor)
            => FromArgb(alpha, baseColor.R, baseColor.G, baseColor.B);

        public static Color FromArgb(int red, int green, int blue)
            => FromArgb(255, red, green, blue);

        public static Color FromArgb(int alpha, int red, int green, int blue)
        {
            Validate(alpha);
            Validate(red);
            Validate(green);
            Validate(blue);

            int argb = (alpha << 24) | (red << 16) | (green << 8) | blue;
            return new Color(argb, null, false);
        }

        public static Color FromName(string name)
        {
            if (string.IsNullOrWhiteSpace(name))
                return Empty;

            var prop = typeof(Color).GetProperty(name, BindingFlags.Public | BindingFlags.Static | BindingFlags.IgnoreCase);

            if (prop != null && prop.PropertyType == typeof(Color))
            {
                return (Color)prop.GetValue(null);
            }

            return Empty;
        }

        // ---------------- Conversion ----------------

        public int ToArgb() => _argb;
        public static Color FromString(string text)
        {
            if (string.IsNullOrWhiteSpace(text))
                return Empty;

            text = text.Trim();

            if (text[0] == '#')
                return FromHex(text);

            // Named colors (HTML standard)
            return FromHtmlName(text);
        }
        private static Color FromHex(string hex)
        {
            hex = hex.TrimStart('#');

            if (hex.Length == 3) // #RGB
            {
                int r = Convert.ToInt32($"{hex[0]}{hex[0]}", 16);
                int g = Convert.ToInt32($"{hex[1]}{hex[1]}", 16);
                int b = Convert.ToInt32($"{hex[2]}{hex[2]}", 16);
                return FromArgb(255, r, g, b);
            }

            if (hex.Length == 6) // #RRGGBB
            {
                int r = Convert.ToInt32(hex.Substring(0, 2), 16);
                int g = Convert.ToInt32(hex.Substring(2, 2), 16);
                int b = Convert.ToInt32(hex.Substring(4, 2), 16);
                return FromArgb(255, r, g, b);
            }

            if (hex.Length == 8) // #RRGGBBAA
            {
                int r = Convert.ToInt32(hex.Substring(0, 2), 16);
                int g = Convert.ToInt32(hex.Substring(2, 2), 16);
                int b = Convert.ToInt32(hex.Substring(4, 2), 16);
                int a = Convert.ToInt32(hex.Substring(6, 2), 16);
                return FromArgb(a, r, g, b);
            }

            throw new FormatException("Invalid hex color format.");
        }
        private static Color FromHtmlName(string name)
        {
            switch (name.ToLowerInvariant())
            {
                case "black": return FromArgb(0, 0, 0);
                case "white": return FromArgb(255, 255, 255);
                case "red": return FromArgb(255, 0, 0);
                case "green": return FromArgb(0, 128, 0);
                case "blue": return FromArgb(0, 0, 255);
                case "yellow": return FromArgb(255, 255, 0);
                case "cyan": return FromArgb(0, 255, 255);
                case "magenta": return FromArgb(255, 0, 255);
                case "gray":
                case "grey": return FromArgb(128, 128, 128);
                case "lightgray":
                case "lightgrey": return FromArgb(211, 211, 211);
                case "darkgray":
                case "darkgrey": return FromArgb(169, 169, 169);
                case "orange": return FromArgb(255, 165, 0);
                case "pink": return FromArgb(255, 192, 203);
                case "brown": return FromArgb(165, 42, 42);
                case "purple": return FromArgb(128, 0, 128);
                case "transparent": return FromArgb(0, 0, 0, 0);
                default:
                    return FromArgb(0, 0, 255); // better return the correct color
            }
        }


        public override string ToString()
        {
            if (IsNamedColor) return Name;
            return $"Color [A={A}, R={R}, G={G}, B={B}]";
        }

        // ---------------- Equality ----------------

        public override bool Equals(object obj)
            => obj is Color c && Equals(c);

        public bool Equals(Color other)
            => _argb == other._argb && _isNamed == other._isNamed && _name == other._name;

        public override int GetHashCode() => _argb;
        public static bool operator ==(Color left, Color right) => left.Equals(right);
        public static bool operator !=(Color left, Color right) => !left.Equals(right);

        // ---------------- HSB ----------------

        public float GetBrightness()
        {
            float r = R / 255f;
            float g = G / 255f;
            float b = B / 255f;
            return Math.Max(r, Math.Max(g, b));
        }

        public float GetSaturation()
        {
            float r = R / 255f;
            float g = G / 255f;
            float b = B / 255f;

            float max = Math.Max(r, Math.Max(g, b));
            float min = Math.Min(r, Math.Min(g, b));

            if (max == 0) return 0;
            return (max - min) / max;
        }

        public float GetHue()
        {
            float r = R / 255f;
            float g = G / 255f;
            float b = B / 255f;

            float max = Math.Max(r, Math.Max(g, b));
            float min = Math.Min(r, Math.Min(g, b));

            if (max == min) return 0;

            float hue;
            if (max == r)
                hue = (g - b) / (max - min);
            else if (max == g)
                hue = 2 + (b - r) / (max - min);
            else
                hue = 4 + (r - g) / (max - min);

            hue *= 60;
            if (hue < 0) hue += 360;
            return hue;
        }

        // ---------------- Helper ----------------

        private static void Validate(int v)
        {
            if ((uint)v > 255)
                throw new ArgumentException("Color component out of range (0-255).");
        }

        public Color(IJsonReadStruct data)
        {
            _argb = data.GetValue<int>();
        }
        public void GetObjectData(IJsonWriteData data)
        {
            data.AddValues(_argb);
        }

        public void SetObjectData(IJsonReadData data)
        {
            throw new NotImplementedException();
        }
    }
    //
    // Summary:
    //     Specifies style information applied to text.
    [Flags]
    public enum FontStyle
    {
        //
        // Summary:
        //     Normal text.
        Regular = 0,
        //
        // Summary:
        //     Bold text.
        Bold = 1,
        //
        // Summary:
        //     Italic text.
        Italic = 2,
        //
        // Summary:
        //     Underlined text.
        Underline = 4,
        //
        // Summary:
        //     Text with a line through the middle.
        Strikeout = 8
    }

    [Flags]
    public enum MouseButtons
    {
        None = 0,
        Left = 1048576,
        Right = 2097152,
        Middle = 4194304,
        XButton1 = 8388608,
        XButton2 = 16777216
    }

    public struct MouseEventArgs
    {
        public MouseButtons Button { get; set; }
        public int Clicks { get; set; }
        public int X { get; set; }
        public int Y { get; set; }
        public int Delta { get; set; }
        public Point Location { get; set; }
    }

    public class KeyEventArgs
    {
        public KeyEventArgs(Keys keyData)
        {
            KeyCode = KeyData = keyData;
        }
        public virtual bool Alt { get; }
        public bool Control { get; }
        public bool Handled { get; set; }
        public Keys KeyCode { get; }
        public int KeyValue { get; }
        public Keys KeyData { get; }
        public Keys Modifiers { get; }
        public virtual bool Shift { get; }
        public bool SuppressKeyPress { get; set; }
    }

    public class PaintEventArgs
    {
        public Rectangle ClipRectangle { get; set; }
        public object Graphics { get; set; } // we cannot access Graphics here (in DebuggerVisualizer
    }

    public enum CheckState
    {
        Unchecked = 0,
        Checked = 1,
        Indeterminate = 2
    }

    [Flags]
    public enum Keys
    {
        Modifiers = -65536,
        None = 0,
        LButton = 1,
        RButton = 2,
        Cancel = 3,
        MButton = 4,
        XButton1 = 5,
        XButton2 = 6,
        Back = 8,
        Tab = 9,
        LineFeed = 10,
        Clear = 12,
        Return = 13,
        Enter = 13,
        ShiftKey = 16,
        ControlKey = 17,
        Menu = 18,
        Pause = 19,
        Capital = 20,
        CapsLock = 20,
        KanaMode = 21,
        HanguelMode = 21,
        HangulMode = 21,
        JunjaMode = 23,
        FinalMode = 24,
        HanjaMode = 25,
        KanjiMode = 25,
        Escape = 27,
        IMEConvert = 28,
        IMENonconvert = 29,
        IMEAccept = 30,
        IMEAceept = 30,
        IMEModeChange = 31,
        Space = 32,
        Prior = 33,
        PageUp = 33,
        Next = 34,
        PageDown = 34,
        End = 35,
        Home = 36,
        Left = 37,
        Up = 38,
        Right = 39,
        Down = 40,
        Select = 41,
        Print = 42,
        Execute = 43,
        Snapshot = 44,
        PrintScreen = 44,
        Insert = 45,
        Delete = 46,
        Help = 47,
        D0 = 48,
        D1 = 49,
        D2 = 50,
        D3 = 51,
        D4 = 52,
        D5 = 53,
        D6 = 54,
        D7 = 55,
        D8 = 56,
        D9 = 57,
        A = 65,
        B = 66,
        C = 67,
        D = 68,
        E = 69,
        F = 70,
        G = 71,
        H = 72,
        I = 73,
        J = 74,
        K = 75,
        L = 76,
        M = 77,
        N = 78,
        O = 79,
        P = 80,
        Q = 81,
        R = 82,
        S = 83,
        T = 84,
        U = 85,
        V = 86,
        W = 87,
        X = 88,
        Y = 89,
        Z = 90,
        LWin = 91,
        RWin = 92,
        Apps = 93,
        Sleep = 95,
        NumPad0 = 96,
        NumPad1 = 97,
        NumPad2 = 98,
        NumPad3 = 99,
        NumPad4 = 100,
        NumPad5 = 101,
        NumPad6 = 102,
        NumPad7 = 103,
        NumPad8 = 104,
        NumPad9 = 105,
        Multiply = 106,
        Add = 107,
        Separator = 108,
        Subtract = 109,
        Decimal = 110,
        Divide = 111,
        F1 = 112,
        F2 = 113,
        F3 = 114,
        F4 = 115,
        F5 = 116,
        F6 = 117,
        F7 = 118,
        F8 = 119,
        F9 = 120,
        F10 = 121,
        F11 = 122,
        F12 = 123,
        F13 = 124,
        F14 = 125,
        F15 = 126,
        F16 = 127,
        F17 = 128,
        F18 = 129,
        F19 = 130,
        F20 = 131,
        F21 = 132,
        F22 = 133,
        F23 = 134,
        F24 = 135,
        NumLock = 144,
        Scroll = 145,
        LShiftKey = 160,
        RShiftKey = 161,
        LControlKey = 162,
        RControlKey = 163,
        LMenu = 164,
        RMenu = 165,
        BrowserBack = 166,
        BrowserForward = 167,
        BrowserRefresh = 168,
        BrowserStop = 169,
        BrowserSearch = 170,
        BrowserFavorites = 171,
        BrowserHome = 172,
        VolumeMute = 173,
        VolumeDown = 174,
        VolumeUp = 175,
        MediaNextTrack = 176,
        MediaPreviousTrack = 177,
        MediaStop = 178,
        MediaPlayPause = 179,
        LaunchMail = 180,
        SelectMedia = 181,
        LaunchApplication1 = 182,
        LaunchApplication2 = 183,
        OemSemicolon = 186,
        Oem1 = 186,
        Oemplus = 187,
        Oemcomma = 188,
        OemMinus = 189,
        OemPeriod = 190,
        OemQuestion = 191,
        Oem2 = 191,
        Oemtilde = 192,
        Oem3 = 192,
        OemOpenBrackets = 219,
        Oem4 = 219,
        OemPipe = 220,
        Oem5 = 220,
        OemCloseBrackets = 221,
        Oem6 = 221,
        OemQuotes = 222,
        Oem7 = 222,
        Oem8 = 223,
        OemBackslash = 226,
        Oem102 = 226,
        ProcessKey = 229,
        Packet = 231,
        Attn = 246,
        Crsel = 247,
        Exsel = 248,
        EraseEof = 249,
        Play = 250,
        Zoom = 251,
        NoName = 252,
        Pa1 = 253,
        OemClear = 254,
        KeyCode = 65535,
        Shift = 65536,
        Control = 131072,
        Alt = 262144
    }

    public enum MessageBoxButtons
    {
        OK = 0,
        OKCancel = 1,
        AbortRetryIgnore = 2,
        YesNoCancel = 3,
        YesNo = 4,
        RetryCancel = 5
    }

    public enum DialogResult
    {
        None = 0,
        OK = 1,
        Cancel = 2,
        Abort = 3,
        Retry = 4,
        Ignore = 5,
        Yes = 6,
        No = 7
    }

    [Flags]
    public enum DragDropEffects
    {
        Scroll = int.MinValue,
        All = -2147483645,
        None = 0,
        Copy = 1,
        Move = 2,
        Link = 4
    }

    public class DragEventArgs : EventArgs
    {
        public object Data { get; set; } //IDataObject
        public int KeyState { get; set; }
        public int X { get; set; }
        public int Y { get; set; }
        public DragDropEffects AllowedEffect { get; set; }
        private DragDropEffects effect;
        public DragDropEffects Effect
        {
            get
            {
                return effect;
            }
            set
            {
                effect = value;
                EffectChanged?.Invoke(this);
            }
        }
        public delegate void ChangedDelegate(DragEventArgs e);
        public event ChangedDelegate EffectChanged;
    }

    public enum Shortcut
    {
        None = 0,
        Ins = 45,
        Del = 46,
        F1 = 112,
        F2 = 113,
        F3 = 114,
        F4 = 115,
        F5 = 116,
        F6 = 117,
        F7 = 118,
        F8 = 119,
        F9 = 120,
        F10 = 121,
        F11 = 122,
        F12 = 123,
        ShiftIns = 65581,
        ShiftDel = 65582,
        ShiftF1 = 65648,
        ShiftF2 = 65649,
        ShiftF3 = 65650,
        ShiftF4 = 65651,
        ShiftF5 = 65652,
        ShiftF6 = 65653,
        ShiftF7 = 65654,
        ShiftF8 = 65655,
        ShiftF9 = 65656,
        ShiftF10 = 65657,
        ShiftF11 = 65658,
        ShiftF12 = 65659,
        CtrlIns = 131117,
        CtrlDel = 131118,
        Ctrl0 = 131120,
        Ctrl1 = 131121,
        Ctrl2 = 131122,
        Ctrl3 = 131123,
        Ctrl4 = 131124,
        Ctrl5 = 131125,
        Ctrl6 = 131126,
        Ctrl7 = 131127,
        Ctrl8 = 131128,
        Ctrl9 = 131129,
        CtrlA = 131137,
        CtrlB = 131138,
        CtrlC = 131139,
        CtrlD = 131140,
        CtrlE = 131141,
        CtrlF = 131142,
        CtrlG = 131143,
        CtrlH = 131144,
        CtrlI = 131145,
        CtrlJ = 131146,
        CtrlK = 131147,
        CtrlL = 131148,
        CtrlM = 131149,
        CtrlN = 131150,
        CtrlO = 131151,
        CtrlP = 131152,
        CtrlQ = 131153,
        CtrlR = 131154,
        CtrlS = 131155,
        CtrlT = 131156,
        CtrlU = 131157,
        CtrlV = 131158,
        CtrlW = 131159,
        CtrlX = 131160,
        CtrlY = 131161,
        CtrlZ = 131162,
        CtrlF1 = 131184,
        CtrlF2 = 131185,
        CtrlF3 = 131186,
        CtrlF4 = 131187,
        CtrlF5 = 131188,
        CtrlF6 = 131189,
        CtrlF7 = 131190,
        CtrlF8 = 131191,
        CtrlF9 = 131192,
        CtrlF10 = 131193,
        CtrlF11 = 131194,
        CtrlF12 = 131195,
        CtrlShift0 = 196656,
        CtrlShift1 = 196657,
        CtrlShift2 = 196658,
        CtrlShift3 = 196659,
        CtrlShift4 = 196660,
        CtrlShift5 = 196661,
        CtrlShift6 = 196662,
        CtrlShift7 = 196663,
        CtrlShift8 = 196664,
        CtrlShift9 = 196665,
        CtrlShiftA = 196673,
        CtrlShiftB = 196674,
        CtrlShiftC = 196675,
        CtrlShiftD = 196676,
        CtrlShiftE = 196677,
        CtrlShiftF = 196678,
        CtrlShiftG = 196679,
        CtrlShiftH = 196680,
        CtrlShiftI = 196681,
        CtrlShiftJ = 196682,
        CtrlShiftK = 196683,
        CtrlShiftL = 196684,
        CtrlShiftM = 196685,
        CtrlShiftN = 196686,
        CtrlShiftO = 196687,
        CtrlShiftP = 196688,
        CtrlShiftQ = 196689,
        CtrlShiftR = 196690,
        CtrlShiftS = 196691,
        CtrlShiftT = 196692,
        CtrlShiftU = 196693,
        CtrlShiftV = 196694,
        CtrlShiftW = 196695,
        CtrlShiftX = 196696,
        CtrlShiftY = 196697,
        CtrlShiftZ = 196698,
        CtrlShiftF1 = 196720,
        CtrlShiftF2 = 196721,
        CtrlShiftF3 = 196722,
        CtrlShiftF4 = 196723,
        CtrlShiftF5 = 196724,
        CtrlShiftF6 = 196725,
        CtrlShiftF7 = 196726,
        CtrlShiftF8 = 196727,
        CtrlShiftF9 = 196728,
        CtrlShiftF10 = 196729,
        CtrlShiftF11 = 196730,
        CtrlShiftF12 = 196731,
        AltBksp = 262152,
        AltLeftArrow = 262181,
        AltUpArrow = 262182,
        AltRightArrow = 262183,
        AltDownArrow = 262184,
        Alt0 = 262192,
        Alt1 = 262193,
        Alt2 = 262194,
        Alt3 = 262195,
        Alt4 = 262196,
        Alt5 = 262197,
        Alt6 = 262198,
        Alt7 = 262199,
        Alt8 = 262200,
        Alt9 = 262201,
        AltF1 = 262256,
        AltF2 = 262257,
        AltF3 = 262258,
        AltF4 = 262259,
        AltF5 = 262260,
        AltF6 = 262261,
        AltF7 = 262262,
        AltF8 = 262263,
        AltF9 = 262264,
        AltF10 = 262265,
        AltF11 = 262266,
        AltF12 = 262267
    }

}
