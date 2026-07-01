using CADability;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using CdlToCSharp;
using MathNet.Numerics;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;


#region Geometry helpers (type operations)

class MathStub
{   // helps with syntax like "Math.PI"
    public MathStub() { }
    public double PI => Math.PI;
    public double E => Math.E;
    public double Sqrt(double s) => Math.Sqrt(s);
    public double Sign(double s) => Math.Sign(s);
    public double Pow(double x, double y) => Math.Pow(x, y);
    public double Abs(double x) => Math.Abs(x);
    public double Cos(double x) => Math.Cos(x);
    public double Acos(double x) => Math.Acos(x);
    public double Cosh(double x) => Math.Cosh(x);
    public double Sin(double x) => Math.Sin(x);
    public double Sinh(double x) => Math.Sinh(x);
    public double Asin(double x) => Math.Asin(x);
    public double Tan(double x) => Math.Tan(x);
    public double Tanh(double x) => Math.Tanh(x);
    public double Atan(double x) => Math.Atan(x);
    public double Atan2(double y, double x) => Math.Atan2(y, x);
    public double Ceiling(double a) => Math.Ceiling(a);
    public double Floor(double a) => Math.Floor(a);
    public double Exp(double a) => Math.Exp(a);
    public double Log(double a) => Math.Log(a);
    public double Log10(double a) => Math.Log10(a);
    public double Round(double a) => Math.Round(a);
    public double Max(double a, double b) => Math.Max(a, b);
    public double Min(double a, double b) => Math.Min(a, b);


}
public static class GeometryOps
{
    public static object UnaryMinus(object v)
    {
        if (v is double d)
            return -d;
        if (v is int i)
            return -i;
        if (v is GeoVector vec)
            return new GeoVector(-vec.x, -vec.y, -vec.z);
        if (v is GeoVector2D vec2)
            return new GeoVector2D(-vec2.x, -vec2.y);
        throw new InvalidOperationException($"Unary '-' is not defined for type {v.GetType()}.");
    }
    public static object Add(object a, object b)
    {
        if (a is IConvertible && b is IConvertible)
            return Convert.ToDouble(a) + Convert.ToDouble(b);
        if (a is double da && b is double db)
            return da + db;
        if (a is GeoVector va && b is GeoVector vb)
            return new GeoVector(va.x + vb.x, va.y + vb.y, va.z + vb.z);
        if (a is GeoPoint pa && b is GeoVector vbb)
            return new GeoPoint(pa.x + vbb.x, pa.y + vbb.y, pa.z + vbb.z);
        if (a is GeoPoint2D pa2 && b is GeoVector2D vb2)
            return new GeoPoint2D(pa2.x + vb2.x, pa2.y + vb2.y);
        throw new InvalidOperationException($"Operator '+' is not defined for {a.GetType()} + {b.GetType()}.");
    }
    public static object Sub(object a, object b)
    {
        if (a is IConvertible && b is IConvertible)
            return Convert.ToDouble(a) - Convert.ToDouble(b);
        if (a is GeoVector va && b is GeoVector vb)
            return new GeoVector(va.x - vb.x, va.y - vb.y, va.z - vb.z);
        if (a is GeoPoint pa && b is GeoVector vbb)
            return new GeoPoint(pa.x - vbb.x, pa.y - vbb.y, pa.z - vbb.z);
        if (a is GeoPoint2D pa2 && b is GeoVector2D vb2)
            return new GeoPoint2D(pa2.x - vb2.x, pa2.y - vb2.y);
        if (a is GeoPoint pa1 && b is GeoPoint pb1)
            return new GeoVector(pa1.x - pb1.x, pa1.y - pb1.y, pa1.z - pb1.z);
        if (a is GeoPoint2D pa12 && b is GeoPoint2D pb12)
            return new GeoVector2D(pa12.x - pb12.x, pa12.y - pb12.y);
        throw new InvalidOperationException($"Operator '-' is not defined for {a.GetType()} - {b.GetType()}.");
    }
    public static object Mul(object a, object b)
    {
        if (a is Angle anga) a = anga.Radian;
        if (b is Angle angb) b = angb.Radian;
        if (a is IConvertible && b is IConvertible)
            return Convert.ToDouble(a) * Convert.ToDouble(b);
        if (a is double da2 && b is GeoVector vb)
            return new GeoVector(da2 * vb.x, da2 * vb.y, da2 * vb.z);
        if (a is double da22 && b is GeoVector2D vb2)
            return new GeoVector2D(da22 * vb2.x, da22 * vb2.y);
        if (a is GeoVector va && b is double db2)
            return new GeoVector(db2 * va.x, db2 * va.y, db2 * va.z);
        if (a is GeoVector2D va2 && b is double db22)
            return new GeoVector2D(db22 * va2.x, db22 * va2.y);
        if (a is GeoVector va3 && b is GeoVector vb3)
            return va3.x * vb3.x + va3.y * vb3.y + va3.z * vb3.z;
        if (a is ModOp m1 && b is ModOp m2)
            return m1 * m2;
        throw new InvalidOperationException($"Operator '*' is not defined for {a.GetType()} * {b.GetType()}.");
    }
    public static object Div(object a, object b)
    {
        if (a is IConvertible && b is IConvertible)
            return Convert.ToDouble(a) / Convert.ToDouble(b);
        if (a is GeoVector va && b is double db2)
            return new GeoVector(va.x / db2, va.y / db2, va.z / db2);
        if (a is GeoVector2D va2 && b is double db22)
            return new GeoVector2D(va2.x / db22, va2.y / db22);
        throw new InvalidOperationException($"Operator '/' is not defined for {a.GetType()} * {b.GetType()}.");
    }
    public static object Pow(object a, object b)
    {
        if (a is IConvertible && b is IConvertible)
            return Math.Pow(Convert.ToDouble(a), Convert.ToDouble(b));
        throw new InvalidOperationException($"Function 'pow' is not defined for {a.GetType()} * {b.GetType()}.");
    }
    public static object Cross(object a, object b)
    {
        if (a is GeoVector va && b is GeoVector vb)
            return new GeoVector(
                va.y * vb.z - va.z * vb.y,
                va.z * vb.x - va.x * vb.z,
                va.x * vb.y - va.y * vb.x);
        if (a is IConvertible && b is IConvertible)
            return Math.Pow(Convert.ToDouble(a), Convert.ToDouble(b));
        throw new InvalidOperationException($"Operator '^' is only defined for GeoVector ^ GeoVector or double ^ double.");
    }

    public static object Distance(object a, object b)
    {
        if (a is GeoPoint pa && b is GeoPoint pb) return pa | pb;
        if (a is GeoPoint2D pa2 && b is GeoPoint2D pb2) return pa2 | pb2;
        throw new InvalidOperationException($"Distance is only defined for GeoPoint and GeoPoint2D");
    }
    public static object Normalize(object v)
    {
        if (v is GeoVector gv) return gv.Normalized;
        if (v is GeoVector2D gv2) return gv2.Normalized;
        throw new InvalidOperationException("Normalize is only defined for vectors.");
    }

    public static object FuncSin(object x)
    {
        if (x is double d) return Math.Sin(d);
        throw new InvalidOperationException("sin(x): x must be a scalar.");
    }
    public static object FuncCos(object x)
    {
        if (x is double d) return Math.Cos(d);
        throw new InvalidOperationException("cos(x): x must be a scalar.");
    }
    public static object FuncTan(object x)
    {
        if (x is double d) return Math.Tan(d);
        throw new InvalidOperationException("tan(x): x must be a scalar.");
    }
    public static object FuncAtan(object x)
    {
        if (x is double d) return Math.Atan(d);
        throw new InvalidOperationException("atan(x): x must be a scalar.");
    }
    public static object FuncAtan2(object x, object y)
    {
        if (x is double d && y is double f) return Math.Atan2(f, d);
        throw new InvalidOperationException("atan2(y, x): x and y must be scalar.");
    }
    public static object FuncSqrt(object x)
    {
        if (x is IConvertible) return Math.Sqrt(Convert.ToDouble(x));
        throw new InvalidOperationException("sqrt(x): x must be a scalar.");
    }

    public static object FuncAbs(object x)
    {
        if (x is IConvertible) return Math.Abs(Convert.ToDouble(x));
        throw new InvalidOperationException("abs(x): x must be a scalar.");
    }

    public static object FuncLen(object x)
    {
        if (x is GeoVector gv) return gv.Length;
        if (x is GeoVector2D gv2) return gv2.Length;
        throw new InvalidOperationException("len(v): v must be a vector.");
    }
}

#endregion

#region Tokenizer

public enum TokenType
{
    Number, String,
    Identifier,
    Plus, Minus, Star, Slash, Caret, Pipe,
    LParen, RParen,
    Comma,
    Dot,
    Equal, GreaterThan, GreaterThanOrEqual, LessThan, LessThanOrEqual,
    And, Or, Not, NotEqual,
}

public readonly struct Token
{
    public TokenType Type { get; }
    public string Text { get; } // für Identifier / Number

    public Token(TokenType type, string text)
    {
        Type = type;
        Text = text;
    }

    public override string ToString() => $"{Type}:{Text}";
}

public static class Lexer
{
    public static List<Token> Tokenize(string expr)
    {
        var tokens = new List<Token>();
        int i = 0;

        while (i < expr.Length)
        {
            char c = expr[i];

            if (char.IsWhiteSpace(c))
            {
                i++;
                continue;
            }

            // Number? (culture-invariant, nur '.' als decimal)
            if (char.IsDigit(c) || (c == '.' && i + 1 < expr.Length && char.IsDigit(expr[i + 1])))
            {
                int start = i;
                i++;
                while (i < expr.Length &&
                       (char.IsDigit(expr[i]) || expr[i] == '.'))
                {
                    i++;
                }
                string numText = expr.Substring(start, i - start);
                tokens.Add(new Token(TokenType.Number, numText));
                continue;
            }

            // Identifier? (Buchstabe oder _ am Anfang, dann Buchstabe/Ziffer/_ )
            if (char.IsLetter(c) || c == '_')
            {
                int start = i;
                i++;
                while (i < expr.Length && (char.IsLetterOrDigit(expr[i]) || expr[i] == '_'))
                {
                    i++;
                }
                string identText = expr.Substring(start, i - start);
                tokens.Add(new Token(TokenType.Identifier, identText));
                continue;
            }
            if (c == '"')
            {
                int start = i;
                i++;
                while (i < expr.Length && expr[i] != '"')
                {
                    i++;
                }
                string stringText = expr.Substring(start + 1, i - start - 1);
                ++i;
                tokens.Add(new Token(TokenType.String, stringText));
                continue;
            }

            // Single-char operators / parentheses / comma
            switch (c)
            {
                case '+': tokens.Add(new Token(TokenType.Plus, "+")); i++; continue;
                case '-': tokens.Add(new Token(TokenType.Minus, "-")); i++; continue;
                case '*': tokens.Add(new Token(TokenType.Star, "*")); i++; continue;
                case '/': tokens.Add(new Token(TokenType.Slash, "/")); i++; continue;
                case '^': tokens.Add(new Token(TokenType.Caret, "^")); i++; continue;
                case '|':
                    if (expr[i + 1] == '|') { tokens.Add(new Token(TokenType.Or, "||")); i += 2; continue; }
                    else tokens.Add(new Token(TokenType.Pipe, "|")); i++; continue;
                case '(': tokens.Add(new Token(TokenType.LParen, "(")); i++; continue;
                case ')': tokens.Add(new Token(TokenType.RParen, ")")); i++; continue;
                case ',': tokens.Add(new Token(TokenType.Comma, ",")); i++; continue;
                case '.': tokens.Add(new Token(TokenType.Dot, ",")); i++; continue;
                case '<':
                    if (expr[i + 1] == '=') { tokens.Add(new Token(TokenType.LessThanOrEqual, ",")); i += 2; continue; }
                    else tokens.Add(new Token(TokenType.LessThan, "<")); i++; continue;
                case '>':
                    if (expr[i + 1] == '=') { tokens.Add(new Token(TokenType.GreaterThanOrEqual, ",")); i += 2; continue; }
                    else tokens.Add(new Token(TokenType.GreaterThan, ">")); i++; continue;
                case '=':
                    if (expr[i + 1] == '=') { tokens.Add(new Token(TokenType.Equal, "==")); i += 2; continue; }
                    break;
                case '&':
                    if (expr[i + 1] == '&') { tokens.Add(new Token(TokenType.And, "&&")); i += 2; continue; }
                    break;
                case '!':
                    if (expr[i + 1] == '=') { tokens.Add(new Token(TokenType.NotEqual, "!=")); i += 2; continue; }
                    else tokens.Add(new Token(TokenType.Not, ",")); i++; continue;

            }

            // Unexpected character '{c}' at position {i}.
            throw new Exception($"Unexpected character '{c}' at position {i}.");
        }

        return tokens;
    }
}

#endregion

#region Parser (Shunting-Yard nach Dijkstra)

public enum Assoc { Left, Right }

public static class Parser
{
    private class OpInfo
    {
        public int Prec;
        public Assoc Assoc;
        public string Symbol;
    }

    // Operator-Prioritäten
    // ^        : 3
    // * and |  : 2
    // + and -  : 1
    private static readonly Dictionary<TokenType, OpInfo> BinaryOps = new Dictionary<TokenType, OpInfo>
    {
        { TokenType.Dot, new OpInfo { Prec = 5, Assoc = Assoc.Right, Symbol="." } },
        { TokenType.Caret, new OpInfo { Prec = 4, Assoc = Assoc.Left, Symbol="^" } },
        { TokenType.Star,  new OpInfo { Prec = 3, Assoc = Assoc.Left, Symbol="*" } },
        { TokenType.Slash,  new OpInfo { Prec = 3, Assoc = Assoc.Left, Symbol="/" } },
        { TokenType.Pipe,  new OpInfo { Prec = 3, Assoc = Assoc.Left, Symbol="|" } },
        { TokenType.Plus,  new OpInfo { Prec = 2, Assoc = Assoc.Left, Symbol="+" } },
        { TokenType.Minus, new OpInfo { Prec = 2, Assoc = Assoc.Left, Symbol="-" } },
        { TokenType.Equal, new OpInfo { Prec = 1, Assoc = Assoc.Left, Symbol="==" } },
        { TokenType.GreaterThan, new OpInfo { Prec = 1, Assoc = Assoc.Left, Symbol=">" } },
        { TokenType.GreaterThanOrEqual, new OpInfo { Prec = 1, Assoc = Assoc.Left, Symbol = ">=" } },
        { TokenType.LessThan, new OpInfo { Prec = 1, Assoc = Assoc.Left, Symbol = "<" } },
        { TokenType.LessThanOrEqual,new OpInfo { Prec = 1, Assoc = Assoc.Left, Symbol = "<=" } },
        { TokenType.And, new OpInfo { Prec = 0, Assoc = Assoc.Left, Symbol = "&&" } },
        { TokenType.Or, new OpInfo { Prec = 0, Assoc = Assoc.Left, Symbol = "||" } },
        { TokenType.Not, new OpInfo { Prec = 0, Assoc = Assoc.Left, Symbol = "!" } },
        { TokenType.NotEqual,new OpInfo { Prec = 0, Assoc = Assoc.Left, Symbol = "!=" } },
};

    // Stack-Eintrag für eine offene Funktion: Name + wie viele Argumente bisher gezählt
    private class OpenFunctionInfo
    {
        public string Name;
        public int ArgCount;
        public bool IsObjectMehtod;
    }

    public static List<object> ToRpn(List<Token> tokens)
    {
        var output = new List<object>();
        var opStack = new Stack<object>();
        var funcStack = new Stack<OpenFunctionInfo>();

        bool expectUnary = true;
        Stack<bool> paranIsFunc = new Stack<bool>();

        for (int i = 0; i < tokens.Count; i++)
        {
            Token t = tokens[i];
            switch (t.Type)
            {
                case TokenType.Number:
                    output.Add(t);
                    expectUnary = false;
                    break;

                case TokenType.String:
                    output.Add(t);
                    expectUnary = false;
                    break;

                case TokenType.Identifier:
                    {
                        bool isFunction = (i + 1 < tokens.Count && tokens[i + 1].Type == TokenType.LParen);
                        bool isProperty = (i > 0 && tokens[i - 1].Type == TokenType.Dot);
                        if (isProperty)
                        {
                            // Wir erlauben auch "p1.x" als Zugriff auf Eigenschaften von Punkten/Vektoren.
                            // In diesem Fall ist "p1" die Variable, und "x" die Eigenschaft.
                            // Wir markieren das im Output entsprechend, damit der Evaluator das später weiß.
                            output.Add(new FunctionOrVariableMarker
                            {
                                Name = t.Text,
                                IsFunction = isFunction, // a method of the property? not implemented yet
                                IsProperty = true
                            });
                            if (opStack.Peek() is Token tt && tt.Type == TokenType.Dot)
                            {   // we need to add the dot here because of chains of properties a.x.y to apper in the correct order.
                                output.Add(opStack.Pop());
                            }
                            expectUnary = false; // nach "p1.x" kommt ja kein '(' mehr, sondern vielleicht ein Operator oder Ende
                        }
                        else
                        {
                            if (isFunction)
                            {
                                // Wir legen uns die Funktion erstmal im Output ab als Marker,
                                // und merken sie uns parallel auf funcStack, sobald die '(' kommt.
                                output.Add(new FunctionOrVariableMarker
                                {
                                    Name = t.Text,
                                    IsFunction = true
                                });
                            }
                            else
                            {
                                // Variable
                                output.Add(new FunctionOrVariableMarker
                                {
                                    Name = t.Text,
                                    IsFunction = false
                                });
                            }
                        }

                        expectUnary = isFunction; // nach Funktionsnamen kommt '(' -> noch kein fertiger Operand
                    }
                    break;

                case TokenType.LParen:
                    {
                        // Prüfen, ob das '(' zu einer gerade gesehenen Funktions-ID gehört:
                        // Das ist der Fall, wenn der letzte Output-Eintrag ein FunctionOrVariableMarker mit IsFunction==true ist,
                        // und wir haben noch keinen OpenFunctionInfo dafür erzeugt.
                        bool isFunction = (i > 0 && tokens[i - 1].Type == TokenType.Identifier);
                        FunctionOrVariableMarker? function = null;
                        if (output.Count > 1 && output[output.Count - 1] is Token tk && tk.Type == TokenType.Dot && output[output.Count - 2] is FunctionOrVariableMarker m && m.IsFunction) function = m; // .func(x)
                        else
                            if (isFunction && output.Count > 0 && output[output.Count - 1] is FunctionOrVariableMarker m1 && m1.IsFunction) function = m1;
                        if (function.HasValue)
                        {
                            // Neue offene Funktion
                            funcStack.Push(new OpenFunctionInfo
                            {
                                Name = function.Value.Name,
                                ArgCount = 1, // sobald wir in Klammern sind, erwarten wir mindestens 1 Argument
                                IsObjectMehtod = function.Value.IsProperty // there was a dot left of the name
                            });
                            if (function.Value.IsProperty && opStack.Peek() is Token ptk && ptk.Type == TokenType.Dot) opStack.Pop(); // pop the dot operand, because it is not needed after the function call
                            paranIsFunc.Push(true);
                        }
                        else
                        {
                            paranIsFunc.Push(false);
                        }

                        // Push '(' auf den Operator-Stack
                        opStack.Push(t);

                        // Wichtig: wenn es KEINE Funktion ist (also nur normale Klammergruppe),
                        // dann ist expectUnary = true (z.B. "(-3)")
                        // Wenn es eine Funktion ist, erwarten wir das erste Argument, also auch unary möglich.
                        expectUnary = true;
                    }
                    break;

                case TokenType.RParen:
                    {
                        bool foundLParen = false;
                        while (opStack.Count > 0)
                        {
                            var obj = opStack.Pop();
                            if (obj is Token tok && tok.Type == TokenType.LParen)
                            {
                                foundLParen = true;
                                break;
                            }
                            output.Add(obj);
                        }
                        if (!foundLParen)
                            throw new Exception("Missing '('.");

                        // Jetzt prüfen: War das eine Funktion?
                        // Falls ja, holen wir die Funktionsinfo und erzeugen FunctionCallMarker
                        if (output.Count > 0 && output[output.Count - 1] is FunctionOrVariableMarker fm && fm.IsFunction)
                        {
                            // Sonderfall: leere Argumentliste erlaubt? z.B. foo()
                            // Ja/nein? In unserem Fall brauchen wir das nicht, aber
                            // falls jemand p() tippt -> ist Unsinn. ArgCount=1 wäre falsch.
                            // Wir lösen das so: wenn direkt "func(" dann ")" ohne Arg -> ArgCount=0
                            // Das erkennen wir aber nur über funcStack.
                        }
                        bool isFunc = paranIsFunc.Pop();
                        if (funcStack.Count > 0 && isFunc)
                        {
                            // Aber Achtung: Wir wissen NICHT sicher, ob dieses ')' wirklich zu der obersten Funktion gehört
                            // oder nur eine normale Klammer war. Das sehen wir so:
                            // Die oberste '(' auf opStack wurde gerade gepoppt. Wenn NACH dieser '('
                            // eine FunctionOrVariableMarker mit IsFunction==true direkt vor den Argument-Token stand,
                            // dann muss das diese Funktion gewesen sein.
                            // Praxis-Heuristik: wir schauen, ob am Output irgendwo hinten
                            // ein FunctionOrVariableMarker(IsFunction=true) existiert,
                            // der noch "offen" ist.
                            // Wir machen es pragmatisch:
                            //   - Hol die oberste FunctionInfo aus funcStack
                            //   - Versuche, im Output von hinten den letzten FunctionOrVariableMarker
                            //     mit demselben Namen und IsFunction==true zu finden.
                            //   - Wenn gefunden: dann war das wirklich ein Funktionsaufruf.
                            var topFunc = funcStack.Peek();
                            int idx = FindLastFunctionMarker(output, topFunc.Name);
                            if (idx >= 0)
                            {
                                // Dann committen wir den Funktionscall:
                                funcStack.Pop();
                                fm = (FunctionOrVariableMarker)output[idx];

                                // Entferne den Marker aus output
                                if (fm.IsProperty)
                                {
                                    // this is a function call where the function is already on the stack
                                    // since the funtion is a method of an object
                                    output.Add(new FunctionCallMarker
                                    {
                                        Name = topFunc.Name,
                                        ArgCount = topFunc.ArgCount,
                                        IsObjectMember = fm.IsProperty // a dot left of the name
                                    });
                                }
                                else
                                {
                                    output.RemoveAt(idx);

                                    // Erzeuge Call-Marker mit ArgCount:
                                    output.Add(new FunctionCallMarker
                                    {
                                        Name = topFunc.Name,
                                        ArgCount = topFunc.ArgCount,
                                        IsObjectMember = fm.IsProperty // a dot left of the name
                                    });
                                }
                            }
                            // Sonst: war wohl doch nur eine normale Klammergruppe, also keine Funktion.
                        }

                        expectUnary = false;
                    }
                    break;

                case TokenType.Comma:
                    {
                        // Komma trennt Funktionsargumente.
                        // Also: poppe Operatoren bis zur letzten '('.
                        bool hitParen = false;
                        while (opStack.Count > 0)
                        {
                            if (opStack.Peek() is Token tok && tok.Type == TokenType.LParen)
                            {
                                hitParen = true;
                                break;
                            }
                            output.Add(opStack.Pop());
                        }
                        if (!hitParen)
                            throw new Exception("Unexpected ',' outside of a function call?");

                        // Erhöhe ArgCount der aktuellen Funktion
                        if (funcStack.Count == 0)
                            throw new Exception("',' found, but no open function.");

                        funcStack.Peek().ArgCount++;

                        // Nach Komma erwarten wir wieder einen neuen Operand
                        expectUnary = true;
                    }
                    break;
                case TokenType.Dot:
                    {
                        opStack.Push(t);
                    }
                    break;

                case TokenType.Plus:
                case TokenType.Minus:
                case TokenType.Star:
                case TokenType.Slash:
                case TokenType.Caret:
                case TokenType.Pipe:
                case TokenType.Equal:
                case TokenType.GreaterThan:
                case TokenType.GreaterThanOrEqual:
                case TokenType.LessThan:
                case TokenType.LessThanOrEqual:
                case TokenType.And:
                case TokenType.Or:
                case TokenType.NotEqual:
                    {
                        if (expectUnary && t.Type == TokenType.Not)
                        {
                            opStack.Push(new UnaryNotMarker());
                            // expectUnary bleibt true (wir erwarten weiter Operand)
                        }
                        else if (expectUnary && t.Type == TokenType.Minus)
                        {
                            opStack.Push(new UnaryMinusMarker());
                            // expectUnary bleibt true (wir erwarten weiter Operand)
                        }
                        else if (expectUnary && t.Type == TokenType.Plus)
                        {
                            opStack.Push(new UnaryPlusMarker());
                            // expectUnary bleibt true (wir erwarten weiter Operand)
                        }
                        else
                        {
                            var thisOp = BinaryOps[t.Type];

                            while (opStack.Count > 0)
                            {
                                if (opStack.Peek() is UnaryMinusMarker)
                                {
                                    output.Add(opStack.Pop());
                                    continue;
                                }
                                if (opStack.Peek() is UnaryNotMarker)
                                {
                                    output.Add(opStack.Pop());
                                    continue;
                                }
                                if (opStack.Peek() is UnaryPlusMarker)
                                {
                                    output.Add(opStack.Pop());
                                    continue;
                                }
                                if (opStack.Peek() is Token topTok &&
                                    BinaryOps.TryGetValue(topTok.Type, out var topOp))
                                {
                                    bool popIt =
                                        (topOp.Prec > thisOp.Prec) ||
                                        (topOp.Prec == thisOp.Prec && thisOp.Assoc == Assoc.Left);

                                    if (popIt)
                                    {
                                        output.Add(opStack.Pop());
                                        continue;
                                    }
                                }
                                break;
                            }

                            opStack.Push(t);
                            expectUnary = true;
                        }
                    }
                    break;

                default:
                    throw new Exception($"Unexpected token {t} in parser.");
            }
        }

        // Stack leeren
        while (opStack.Count > 0)
        {
            var obj = opStack.Pop();
            if (obj is Token tok2 &&
                (tok2.Type == TokenType.LParen || tok2.Type == TokenType.RParen))
            {
                throw new Exception("Mismatched parentheses.");
            }
            output.Add(obj);
        }

        if (funcStack.Count > 0)
        {
            throw new Exception("Missing ')' in function call.");
        }

        return output;
    }

    private static int FindLastFunctionMarker(List<object> output, string funcName)
    {
        for (int i = output.Count - 1; i >= 0; i--)
        {
            if (output[i] is FunctionOrVariableMarker m &&
                m.IsFunction &&
                m.Name == funcName)
            {
                return i;
            }
        }
        return -1;
    }
}

internal struct FunctionOrVariableMarker
{
    public string Name;
    public bool IsFunction; // true = "sin(...)", false = "p1"
    public bool IsProperty;
}

internal struct FunctionCallMarker
{
    public string Name;
    public int ArgCount;
    public bool IsObjectMember;
    public FunctionCallMarker(string name, int argCount, bool isObjectMember = false)
    {
        Name = name;
        IsObjectMember = isObjectMember;
        ArgCount = argCount;
    }
}

internal struct UnaryMinusMarker { }
internal struct UnaryNotMarker { }
internal struct UnaryPlusMarker { }

#endregion

#region Evaluator (RPN auswerten)

public static class Evaluator
{
    static bool TryCoerceArg(object? arg, ParameterInfo p, out object? coerced)
    {
        coerced = arg;

        // ref/out: ParameterType ist z.B. Double& -> ElementType ist Double
        var pt = p.ParameterType;
        var targetType = pt.IsByRef ? pt.GetElementType()! : pt;

        // null-Handling
        if (arg is null)
        {
            // null ist ok bei Referenztypen oder Nullable<T>
            if (!targetType.IsValueType || Nullable.GetUnderlyingType(targetType) != null)
                return true;

            return false;
        }

        // Schon passend?
        if (targetType.IsInstanceOfType(arg))
            return true;

        // Nullable<T> behandeln
        var nonNullTarget = Nullable.GetUnderlyingType(targetType) ?? targetType;

        try
        {
            // Enum: z.B. "Red" oder 1
            if (nonNullTarget.IsEnum)
            {
                if (arg is string s)
                {
                    coerced = Enum.Parse(nonNullTarget, s, ignoreCase: true);
                    return true;
                }
                coerced = Enum.ToObject(nonNullTarget, Convert.ChangeType(arg, Enum.GetUnderlyingType(nonNullTarget), CultureInfo.InvariantCulture)!);
                return true;
            }

            // string -> Guid
            if (nonNullTarget == typeof(Guid) && arg is string gs)
            {
                if (Guid.TryParse(gs, out var g))
                {
                    coerced = g;
                    return true;
                }
                return false;
            }

            // IConvertible-Konvertierung (Zahlen, bool, DateTime je nach Eingabe)
            if (arg is IConvertible && typeof(IConvertible).IsAssignableFrom(nonNullTarget))
            {
                coerced = Convert.ChangeType(arg, nonNullTarget, CultureInfo.InvariantCulture);
                return true;
            }
        }
        catch
        {
            // Konvertierung fehlgeschlagen -> passt nicht
        }

        return false;
    }
    private class ObjectMethodPair
    {
        public MethodInfo method;
        public object obj;
        public ObjectMethodPair(MethodInfo method, object obj)
        {
            this.method = method;
            this.obj = obj;
        }
    }
    public static object Evaluate(string expr, Dictionary<string, object> namedValues)
    {
        if (string.IsNullOrWhiteSpace(expr)) return null;
        if (Regex.IsMatch(expr, @"^\s*-?\d+(,\d*)?\s*$"))
        {   // replace "," by "." when there is only numbers and a single comma
            // this should not happen, but there might be some cases left, where while typing a string like "2," occures
            expr = expr.Replace(',', '.');
        }
        var tokens = Lexer.Tokenize(expr);
        var rpn = Parser.ToRpn(tokens);

        var stack = new Stack<object>();

        foreach (var item in rpn)
        {
            switch (item)
            {
                case Token t when t.Type == TokenType.Number:
                    {
                        // double mit invariant culture
                        double d = double.Parse(t.Text, CultureInfo.InvariantCulture);
                        stack.Push(d);
                        break;
                    }
                case Token t when t.Type == TokenType.String:
                    {
                        stack.Push(t.Text);
                        break;
                    }
                case Token t when t.Type == TokenType.Dot:
                    {
                        if (stack.Count < 2)
                            throw new Exception("Too few operands for binary operator.");

                        object b = stack.Pop(); // the property name
                        object a = stack.Pop(); // the object with the property
                        if (!(b is string bs))
                            throw new Exception("Expected property name as string on the right side of '.' operator.");
                        object aa = a;
                        if (a is IEnumerable<object> seq && seq.Count() == 1)
                        {   // MCP Server makes no difference between a List<T> of a single object and
                            // the object itself, when the list only contains a single object
                            aa = seq.First();
                        }

                        PropertyInfo pi = aa.GetType().GetProperty(bs, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.IgnoreCase);
                        if (pi != null)
                        {
                            object propValue = pi.GetValue(aa);
                            stack.Push(propValue);
                            break;
                        }
                        FieldInfo fi = aa.GetType().GetField(bs, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.IgnoreCase);
                        if (fi != null)
                        {
                            object f = fi.GetValue(aa);
                            stack.Push(f);
                            break;
                        }
                        // GetMethod must also be implemented, what about number and type of parameters?
                        MethodInfo[] methods = aa.GetType().GetMethods(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.IgnoreCase);
                        bool found = false;
                        for (int i = 0; i < methods.Length; i++)
                        {
                            if (methods[i].Name.Equals(bs, StringComparison.OrdinalIgnoreCase))
                            {
                                stack.Push(new ObjectMethodPair(methods[i], aa)); // target object and method
                                found = true;
                                break;
                            }
                        }
                        if (found) break;

                        // for List<T> we want to accept a few properties here
                        if (a is IEnumerable<object> seqa)
                        {
                            if (bs.Equals("count", StringComparison.OrdinalIgnoreCase))
                            {
                                stack.Push(seqa.Count());
                                break;
                            }
                            if (bs.Equals("bounds"))
                            {
                                BoundingRect br = BoundingRect.EmptyBoundingRect;
                                BoundingBox bc = BoundingBox.EmptyBoundingBox;
                                foreach (object obj in seqa)
                                {
                                    if (obj is CompoundShape cs) br.MinMax(cs.GetExtent());
                                    else if (obj is ICurve2D c2) br.MinMax(c2.GetExtent());
                                    else if (obj is IGeoObject go) bc.MinMax(go.GetExtent(0.0));
                                }
                                if (!br.IsEmpty()) { stack.Push(br); break; }
                                else if (!bc.IsEmpty) { stack.Push(bc); break; }
                            }
                        }
                        // Property '{b}' not found on type {a.GetType()}.
                        throw new Exception($"Property '{b}' not found on type {a.GetType()}.");

                    }
                    break;

                case Token t when t.Type == TokenType.Plus ||
                                   t.Type == TokenType.Minus ||
                                   t.Type == TokenType.Star ||
                                   t.Type == TokenType.Slash ||
                                   t.Type == TokenType.Caret ||
                                   t.Type == TokenType.Pipe ||
                                    t.Type == TokenType.Equal ||
                                    t.Type == TokenType.GreaterThan ||
                                    t.Type == TokenType.GreaterThanOrEqual ||
                                    t.Type == TokenType.LessThan ||
                                    t.Type == TokenType.LessThanOrEqual ||
                                    t.Type == TokenType.And ||
                                     t.Type == TokenType.Or ||
                                    t.Type == TokenType.Not ||
                                    t.Type == TokenType.NotEqual:
                    {
                        if (stack.Count < 2)
                            throw new Exception("Too few operands for binary operator.");

                        object b = stack.Pop();
                        object a = stack.Pop();

                        object res;
                        switch (t.Type)
                        {
                            case TokenType.Plus:
                                res = GeometryOps.Add(a, b);
                                break;
                            case TokenType.Minus:
                                res = GeometryOps.Sub(a, b);
                                break;
                            case TokenType.Star:
                                res = GeometryOps.Mul(a, b);
                                break;
                            case TokenType.Slash:
                                res = GeometryOps.Div(a, b);
                                break;
                            case TokenType.Caret:
                                res = GeometryOps.Cross(a, b);
                                break;
                            case TokenType.Pipe:
                                res = GeometryOps.Distance(a, b);
                                break;
                            case TokenType.Equal:
                                if (IsNumeric(a) && IsNumeric(b)) res = Convert.ToDouble(a) == Convert.ToDouble(b);
                                else res = a.Equals(b);
                                break;
                            case TokenType.GreaterThan:
                                {
                                    res = false;
                                    if (IsNumeric(a) && IsNumeric(b)) res = Convert.ToDouble(a) > Convert.ToDouble(b);
                                }
                                break;
                            case TokenType.GreaterThanOrEqual:
                                {
                                    res = false;
                                    if (IsNumeric(a) && IsNumeric(b)) res = Convert.ToDouble(a) >= Convert.ToDouble(b);
                                }
                                break;
                            case TokenType.LessThan:
                                {
                                    res = false;
                                    if (IsNumeric(a) && IsNumeric(b)) res = Convert.ToDouble(a) < Convert.ToDouble(b);
                                }
                                break;
                            case TokenType.LessThanOrEqual:
                                {
                                    res = false;
                                    if (IsNumeric(a) && IsNumeric(b)) res = Convert.ToDouble(a) <= Convert.ToDouble(b);
                                }
                                break;
                            case TokenType.And:
                                {
                                    res = false;
                                    if (a is bool aa && b is bool bb) res = aa && bb;
                                }
                                break;
                            case TokenType.Or:
                                {
                                    res = false;
                                    if (a is bool aa && b is bool bb) res = aa || bb;
                                }
                                break;
                            case TokenType.NotEqual:
                                res = !a.Equals(b);
                                break;
                            default:
                                throw new Exception("Unexpected operator.");
                        }

                        stack.Push(res);
                        break;
                    }

                case UnaryMinusMarker _:
                    {
                        if (stack.Count < 1)
                            throw new Exception("Too few operands for unary '-'.");
                        var v = stack.Pop();
                        stack.Push(GeometryOps.UnaryMinus(v));
                        break;
                    }
                case UnaryPlusMarker _:
                    {
                        if (stack.Count < 1)
                            throw new Exception("Too few operands for unary '+'.");
                        // stack remains unchanged
                        break;
                    }
                case UnaryNotMarker _:
                    {
                        if (stack.Count < 1)
                            throw new Exception("Too few operands for unary '!='.");
                        var v = stack.Pop();
                        stack.Push(!(bool)(v));
                        break;
                    }
                case FunctionOrVariableMarker marker:
                    {
                        if (marker.IsProperty)
                        {
                            stack.Push(marker.Name);
                        }
                        else
                        {
                            if (marker.IsFunction)
                            {
                                // Funktionsaufruf wird nicht hier,
                                // sondern durch FunctionCallMarker behandelt.
                                // Hier machen wir NICHTS, denn der echte Call
                                // kommt später.
                                // ABER: für ein nacktes "sin" ohne () wäre das falsch,
                                // aber so etwas wollen wir eh nicht erlauben.
                            }
                            else
                            {
                                // Variable
                                if (namedValues.TryGetValue(marker.Name, out object obj))
                                {
                                    stack.Push(obj);
                                }
                                else if (marker.Name == "pi")
                                {
                                    stack.Push(Math.PI);
                                }
                                else if (marker.Name == "e")
                                {
                                    stack.Push(Math.E);
                                }
                                else if (marker.Name == "Math")
                                {
                                    stack.Push(new MathStub());
                                }
                                else throw new Exception($"Unknown name '{marker.Name}'.");

                            }
                        }
                        break;
                    }

                case FunctionCallMarker call:
                    {
                        if (stack.Count < call.ArgCount)
                            throw new Exception(
                                $"Function {call.Name} expects {call.ArgCount} argument(s), but only {stack.Count} are present.");

                        // Argumente rückwärts vom Stack holen
                        var argsReversed = new List<object>();
                        for (int k = 0; k < call.ArgCount; k++)
                            argsReversed.Add(stack.Pop());

                        // wieder in richtige Reihenfolge bringen (erstes Argument zuerst)
                        argsReversed.Reverse();
                        var args = argsReversed.ToArray();

                        object fres = null;

                        if (call.IsObjectMember)
                        {
                            object toCallWith = stack.Pop();
                            MethodInfo? toCall = null;
                            object target = null;
                            if (toCallWith is ObjectMethodPair om)
                            {
                                toCall = om.method;
                                target = om.obj;
                            }
                            else
                            {
                                MethodInfo[] methods = toCallWith.GetType().GetMethods(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                                for (int i = 0; i < methods.Length; i++)
                                {
                                    if (!string.Equals(methods[i].Name, call.Name, StringComparison.OrdinalIgnoreCase))
                                        continue;

                                    var parameters = methods[i].GetParameters();
                                    if (parameters.Length != call.ArgCount)
                                        continue;
                                    toCall = methods[i];
                                    target = toCallWith;
                                    break;
                                }
                            }
                            if (toCall != null)
                            {
                                var parameters = toCall.GetParameters();

                                object?[] coercedArgs = new object?[parameters.Length];
                                bool ok = true;

                                for (int j = 0; j < parameters.Length; j++)
                                {
                                    if (!TryCoerceArg(args[j], parameters[j], out var c))
                                    {
                                        ok = false;
                                        break;
                                    }
                                    coercedArgs[j] = c;
                                }

                                if (!ok) continue;

                                fres = toCall.Invoke(target, coercedArgs);
                            }
                        }
                        else
                        {

                            switch (call.Name)
                            {
                                // 1-Argument-Funktionen (wie vorher)
                                case "sin":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.FuncSin(args[0]);
                                    break;

                                case "cos":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.FuncCos(args[0]);
                                    break;

                                case "tan":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.FuncTan(args[0]);
                                    break;

                                case "sinh":
                                    CheckArgCount(call, args, 1);
                                    fres = Math.Sinh((double)args[0]);
                                    break;
                                case "cosh":
                                    CheckArgCount(call, args, 1);
                                    fres = Math.Cosh((double)args[0]);
                                    break;
                                case "tanh":
                                    CheckArgCount(call, args, 1);
                                    fres = Math.Tanh((double)args[0]);
                                    break;
                                case "asin":
                                    CheckArgCount(call, args, 1);
                                    fres = Math.Asin((double)args[0]);
                                    break;
                                case "acos":
                                    CheckArgCount(call, args, 1);
                                    fres = Math.Acos((double)args[0]);
                                    break;

                                case "atan":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.FuncAtan(args[0]);
                                    break;

                                case "atan2":
                                    CheckArgCount(call, args, 2);
                                    fres = GeometryOps.FuncAtan2(args[0], args[1]);
                                    break;

                                case "sqrt":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.FuncSqrt(args[0]);
                                    break;

                                case "pow":
                                    CheckArgCount(call, args, 2);
                                    fres = GeometryOps.Pow(args[0], args[1]);
                                    break;
                                case "abs":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.FuncAbs(args[0]);
                                    break;
                                case "sign":
                                    CheckArgCount(call, args, 1);
                                    if (args[0] is IConvertible) fres = Math.Sign(Convert.ToDouble(args[0]));
                                    else throw new InvalidOperationException("sign(x): x must be a scalar.");
                                    break;
                                case "len":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.FuncLen(args[0]);
                                    break;
                                case "ceil":
                                    CheckArgCount(call, args, 1);
                                    fres = (int)Math.Ceiling(Convert.ToDouble(args[0]));
                                    break;
                                case "floor":
                                    CheckArgCount(call, args, 1);
                                    fres = (int)Math.Floor(Convert.ToDouble(args[0]));
                                    break;
                                case "round":
                                    CheckArgCount(call, args, 1);
                                    fres = (int)Math.Round(Convert.ToDouble(args[0]));
                                    break;
                                case "p":
                                    fres = MakePoint(args);
                                    break;
                                case "v":
                                    fres = MakeVector(args);
                                    break;
                                case "translate":
                                case "move":
                                    {
                                        if (args.Length == 3) fres = ModOp.Translate((double)args[0], (double)args[1], (double)args[2]);
                                        else if (args.Length == 1 && args[0] is GeoVector v) fres = ModOp.Translate(v);
                                        else throw new Exception($"Function {call.Name} expects a vector or thre double values as argument.");
                                    }
                                    break;
                                case "rotate":
                                    {
                                        if (args.Length == 3 && args[0] is GeoPoint p && args[1] is GeoVector v && args[2] is double d)
                                            fres = ModOp.Rotate(p, v, new SweepAngle(d));
                                        else throw new Exception($"Function {call.Name} expects a point (fixpoint), a vector (axis direction) and a double (rotation angle in radiants) as arguments.");
                                    }
                                    break;
                                case "scale":
                                    {
                                        if (args.Length == 3 && args[0] is double fx && args[1] is double fy && args[2] is double fz) fres = ModOp.Scale(fx, fy, fz);
                                        else if (args.Length == 2 && args[0] is GeoPoint p && args[1] is double f) fres = ModOp.Scale(p, f);
                                        // and more configurations
                                        else throw new Exception($"Function {call.Name} expects a vector or three double values as argument.");
                                    }
                                    break;
                                case "reflect":
                                    {
                                        if (args.Length == 2 && args[0] is GeoPoint origin && args[1] is GeoVector normal) 
                                            fres = ModOp.ReflectPlane(new Plane(origin,normal));
                                        // and more configurations
                                        else throw new Exception($"Function {call.Name} expects a point and a vector as argument.");
                                    }
                                    break;
                                case "distance":
                                    CheckArgCount(call, args, 2);
                                    fres = GeometryOps.Distance(args[0], args[1]);
                                    break;
                                case "normalize":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.Normalize(args[0]);
                                    break;
                                case "min":
                                    {
                                        if (args.Length < 2) throw new Exception($"Function {call.Name} expects at least 2 argument(s), got: {args.Length}.");
                                        double min = double.MaxValue;
                                        bool intResult = true;
                                        for (int i = 0; i < args.Length; i++)
                                        {
                                            if ((double)args[i] < min) min = (double)args[i];
                                            if (!(args[i] is int)) intResult = false;
                                        }
                                        if (intResult) fres = (int)min;
                                        else fres = min;
                                    }
                                    break;
                                case "max":
                                    {
                                        if (args.Length < 2) throw new Exception($"Function {call.Name} expects at least 2 argument(s), got: {args.Length}.");
                                        double max = double.MinValue;
                                        bool intResult = true;
                                        for (int i = 0; i < args.Length; i++)
                                        {
                                            if ((double)args[i] > max) max = (double)args[i];
                                            if (!(args[i] is int)) intResult = false;
                                        }
                                        if (intResult) fres = (int)max;
                                        else fres = max;
                                    }
                                    break;
                                case "near":
                                    {
                                        if (args.Length < 2 || args.Length > 3) throw new Exception($"Function {call.Name} expects at least 2 argument(s), got: {args.Length}.");
                                        if (args.Length == 2) fres = Math.Abs(((double)args[0]) - ((double)args[1])) < 1e-6;
                                        else if (args.Length == 3) fres = Math.Abs(((double)args[0]) - ((double)args[1])) < (double)args[2];
                                    }
                                    break;
                                default:
                                    throw new Exception($"Unknown function '{call.Name}'.");
                            }
                        }

                        stack.Push(fres);
                        break;
                    }

                default:
                    throw new Exception($"Unexpected RPN element: {item}");
            }
        }

        if (stack.Count != 1)
            throw new Exception("Expression incomplete or overdetermined.");

        return stack.Pop();
    }

    private static bool IsNumeric(object b)
    {
        return b is byte or sbyte or short or ushort or int or uint or long or ulong or float or double or decimal;
    }

    private static void CheckArgCount(FunctionCallMarker call, object[] args, int expected)
    {
        if (args.Length != expected)
            throw new Exception($"Function {call.Name} expects {expected} argument(s), got: {args.Length}.");
    }

    private static object MakePoint(object[] args)
    {
        if (args.Length == 2)
        {
            if (!(args[0] is double a0) || !(args[1] is double a1))
                throw new Exception("p(x,y): all arguments must be scalar.");
            return new GeoPoint2D(a0, a1);
        }
        else
        {
            // alle müssen Skalar sein
            if (!(args[0] is double a0) || !(args[1] is double a1) || !(args[2] is double a2))
                throw new Exception("p(x,y,z): all arguments must be scalar.");

            return new GeoPoint(a0, a1, a2);
        }
    }

    private static object MakeVector(object[] args)
    {
        if (args.Length == 2)
        {
            if (!(args[0] is double a0) || !(args[1] is double a1))
                throw new Exception("v(x,y): all arguments must be scalar.");
            return new GeoVector2D(a0, a1);
        }
        else
        {
            // alle müssen Skalar sein
            if (!(args[0] is double a0) || !(args[1] is double a1) || !(args[2] is double a2))
                throw new Exception("v(x,y,z): all arguments must be scalar.");

            return new GeoVector(a0, a1, a2);
        }
    }

}

#endregion
