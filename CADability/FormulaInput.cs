using CADability;
using System;
using System.Collections.Generic;
using System.Globalization;

#region Value wrapper

public enum ValueKind { Scalar, Vector, Point }

public readonly struct Value
{
    public ValueKind Kind { get; }
    public double Scalar { get; }
    public GeoVector Vector { get; }
    public GeoPoint Point { get; }

    private Value(ValueKind kind, double scalar, GeoVector vec, GeoPoint pt)
    {
        Kind = kind;
        Scalar = scalar;
        Vector = vec;
        Point = pt;
    }

    public static Value FromScalar(double d) => new Value(ValueKind.Scalar, d, default, default);
    public static Value FromVector(GeoVector v) => new Value(ValueKind.Vector, 0.0, v, default);
    public static Value FromPoint(GeoPoint p) => new Value(ValueKind.Point, 0.0, default, p);

    public object ToObject()
    {
        return Kind switch
        {
            ValueKind.Scalar => (object)Scalar,
            ValueKind.Vector => (object)Vector,
            ValueKind.Point => (object)Point,
            _ => throw new InvalidOperationException()
        };
    }
}

#endregion

#region Geometry helpers (type operations)

public static class GeometryOps
{
    public static Value UnaryMinus(Value v)
    {
        switch (v.Kind)
        {
            case ValueKind.Scalar:
                return Value.FromScalar(-v.Scalar);

            case ValueKind.Vector:
                return Value.FromVector(new GeoVector(
                    -v.Vector.x, -v.Vector.y, -v.Vector.z));

            case ValueKind.Point:
                // Unary '-' for GeoPoint is not defined.
                throw new InvalidOperationException("Unary '-' für GeoPoint ist nicht definiert.");

            default:
                throw new InvalidOperationException();
        }
    }

    public static Value Add(Value a, Value b)
    {
        // p + v -> p
        if (a.Kind == ValueKind.Point && b.Kind == ValueKind.Vector)
        {
            return Value.FromPoint(new GeoPoint(
                a.Point.x + b.Vector.x,
                a.Point.y + b.Vector.y,
                a.Point.z + b.Vector.z));
        }

        // v + v -> v
        if (a.Kind == ValueKind.Vector && b.Kind == ValueKind.Vector)
        {
            return Value.FromVector(new GeoVector(
                a.Vector.x + b.Vector.x,
                a.Vector.y + b.Vector.y,
                a.Vector.z + b.Vector.z));
        }

        // scalar + scalar -> scalar
        if (a.Kind == ValueKind.Scalar && b.Kind == ValueKind.Scalar)
        {
            return Value.FromScalar(a.Scalar + b.Scalar);
        }

        // Operator '+' is not defined for {a.Kind} + {b.Kind}.
        throw new InvalidOperationException($"Operator '+' ist nicht definiert für {a.Kind} + {b.Kind}.");
    }

    public static Value Sub(Value a, Value b)
    {
        // p - v -> p
        if (a.Kind == ValueKind.Point && b.Kind == ValueKind.Vector)
        {
            return Value.FromPoint(new GeoPoint(
                a.Point.x - b.Vector.x,
                a.Point.y - b.Vector.y,
                a.Point.z - b.Vector.z));
        }

        // p2 - p1 -> v
        if (a.Kind == ValueKind.Point && b.Kind == ValueKind.Point)
        {
            return Value.FromVector(new GeoVector(
                a.Point.x - b.Point.x,
                a.Point.y - b.Point.y,
                a.Point.z - b.Point.z));
        }

        // v - v -> v
        if (a.Kind == ValueKind.Vector && b.Kind == ValueKind.Vector)
        {
            return Value.FromVector(new GeoVector(
                a.Vector.x - b.Vector.x,
                a.Vector.y - b.Vector.y,
                a.Vector.z - b.Vector.z));
        }

        // scalar - scalar -> scalar
        if (a.Kind == ValueKind.Scalar && b.Kind == ValueKind.Scalar)
        {
            return Value.FromScalar(a.Scalar - b.Scalar);
        }

        // Operator '-' is not defined for {a.Kind} - {b.Kind}.
        throw new InvalidOperationException($"Operator '-' ist nicht definiert für {a.Kind} - {b.Kind}.");
    }

    public static Value Mul(Value a, Value b)
    {
        // d * v -> v
        if (a.Kind == ValueKind.Scalar && b.Kind == ValueKind.Vector)
        {
            return Value.FromVector(new GeoVector(
                a.Scalar * b.Vector.x,
                a.Scalar * b.Vector.y,
                a.Scalar * b.Vector.z));
        }

        // v * d -> v (symmetric is nice)
        if (a.Kind == ValueKind.Vector && b.Kind == ValueKind.Scalar)
        {
            return Value.FromVector(new GeoVector(
                b.Scalar * a.Vector.x,
                b.Scalar * a.Vector.y,
                b.Scalar * a.Vector.z));
        }

        // v * v -> scalar (dot product)
        if (a.Kind == ValueKind.Vector && b.Kind == ValueKind.Vector)
        {
            double dot = a.Vector.x * b.Vector.x +
                         a.Vector.y * b.Vector.y +
                         a.Vector.z * b.Vector.z;
            return Value.FromScalar(dot);
        }

        // scalar * scalar -> scalar
        if (a.Kind == ValueKind.Scalar && b.Kind == ValueKind.Scalar)
        {
            return Value.FromScalar(a.Scalar * b.Scalar);
        }

        // Operator '*' is not defined for {a.Kind} * {b.Kind}.
        throw new InvalidOperationException($"Operator '*' ist nicht definiert für {a.Kind} * {b.Kind}.");
    }

    public static Value Cross(Value a, Value b)
    {
        // v ^ v -> v (cross product)
        if (a.Kind == ValueKind.Vector && b.Kind == ValueKind.Vector)
        {
            return Value.FromVector(new GeoVector(
                a.Vector.y * b.Vector.z - a.Vector.z * b.Vector.y,
                a.Vector.z * b.Vector.x - a.Vector.x * b.Vector.z,
                a.Vector.x * b.Vector.y - a.Vector.y * b.Vector.x));
        }

        // Operator '^' is only defined for GeoVector ^ GeoVector.
        throw new InvalidOperationException($"Operator '^' ist nur für GeoVector ^ GeoVector definiert.");
    }

    public static Value Distance(Value a, Value b)
    {
        // p | p -> scalar (distance)
        if (a.Kind == ValueKind.Point && b.Kind == ValueKind.Point)
        {
            double dx = a.Point.x - b.Point.x;
            double dy = a.Point.y - b.Point.y;
            double dz = a.Point.z - b.Point.z;
            double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            return Value.FromScalar(dist);
        }

        // Operator '|' is only defined for GeoPoint | GeoPoint.
        throw new InvalidOperationException($"Operator '|' ist nur für GeoPoint | GeoPoint definiert.");
    }

    // Funktionen
    public static Value FuncSin(Value x)
    {
        if (x.Kind != ValueKind.Scalar)
            // sin(x): x must be a scalar.
            throw new InvalidOperationException("sin(x): x muss ein Skalar sein.");
        return Value.FromScalar(Math.Sin(x.Scalar));
    }

    public static Value FuncSqrt(Value x)
    {
        if (x.Kind != ValueKind.Scalar)
            // sqrt(x): x must be a scalar.
            throw new InvalidOperationException("sqrt(x): x muss ein Skalar sein.");
        return Value.FromScalar(Math.Sqrt(x.Scalar));
    }

    public static Value FuncAbs(Value x)
    {
        if (x.Kind != ValueKind.Scalar)
            // abs(x): x must be a scalar.
            throw new InvalidOperationException("abs(x): x muss ein Skalar sein.");
        return Value.FromScalar(Math.Abs(x.Scalar));
    }

    public static Value FuncLen(Value x)
    {
        if (x.Kind != ValueKind.Vector)
            // len(v): v must be a vector.
            throw new InvalidOperationException("len(v): v muss ein Vektor sein.");
        double l = Math.Sqrt(
            x.Vector.x * x.Vector.x +
            x.Vector.y * x.Vector.y +
            x.Vector.z * x.Vector.z);
        return Value.FromScalar(l);
    }
}

#endregion

#region Tokenizer

public enum TokenType
{
    Number,
    Identifier,
    Plus, Minus, Star, Caret, Pipe,
    LParen, RParen,
    Comma, // für evtl. mehrargumentige Funktionen in Zukunft
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

            // Single-char operators / parentheses / comma
            switch (c)
            {
                case '+': tokens.Add(new Token(TokenType.Plus, "+")); i++; continue;
                case '-': tokens.Add(new Token(TokenType.Minus, "-")); i++; continue;
                case '*': tokens.Add(new Token(TokenType.Star, "*")); i++; continue;
                case '^': tokens.Add(new Token(TokenType.Caret, "^")); i++; continue;
                case '|': tokens.Add(new Token(TokenType.Pipe, "|")); i++; continue;
                case '(': tokens.Add(new Token(TokenType.LParen, "(")); i++; continue;
                case ')': tokens.Add(new Token(TokenType.RParen, ")")); i++; continue;
                case ',': tokens.Add(new Token(TokenType.Comma, ",")); i++; continue;
            }

            // Unexpected character '{c}' at position {i}.
            throw new Exception($"Unerwartetes Zeichen '{c}' an Position {i}.");
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
        { TokenType.Caret, new OpInfo { Prec = 3, Assoc = Assoc.Left, Symbol="^" } },
        { TokenType.Star,  new OpInfo { Prec = 2, Assoc = Assoc.Left, Symbol="*" } },
        { TokenType.Pipe,  new OpInfo { Prec = 2, Assoc = Assoc.Left, Symbol="|" } },
        { TokenType.Plus,  new OpInfo { Prec = 1, Assoc = Assoc.Left, Symbol="+" } },
        { TokenType.Minus, new OpInfo { Prec = 1, Assoc = Assoc.Left, Symbol="-" } },
    };

    // Stack-Eintrag für eine offene Funktion: Name + wie viele Argumente bisher gezählt
    private class OpenFunctionInfo
    {
        public string Name;
        public int ArgCount;
    }

    public static List<object> ToRpn(List<Token> tokens)
    {
        var output = new List<object>();
        var opStack = new Stack<object>();
        var funcStack = new Stack<OpenFunctionInfo>();

        bool expectUnary = true;

        for (int i = 0; i < tokens.Count; i++)
        {
            Token t = tokens[i];
            switch (t.Type)
            {
                case TokenType.Number:
                    output.Add(t);
                    expectUnary = false;
                    break;

                case TokenType.Identifier:
                    {
                        bool isFunction = (i + 1 < tokens.Count && tokens[i + 1].Type == TokenType.LParen);

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

                        expectUnary = isFunction; // nach Funktionsnamen kommt '(' -> noch kein fertiger Operand
                    }
                    break;

                case TokenType.LParen:
                    {
                        // Prüfen, ob das '(' zu einer gerade gesehenen Funktions-ID gehört:
                        // Das ist der Fall, wenn der letzte Output-Eintrag ein FunctionOrVariableMarker mit IsFunction==true ist,
                        // und wir haben noch keinen OpenFunctionInfo dafür erzeugt.
                        bool isFuncCall = false;
                        if (output.Count > 0 && output[output.Count - 1] is FunctionOrVariableMarker m && m.IsFunction)
                        {
                            // Neue offene Funktion
                            funcStack.Push(new OpenFunctionInfo
                            {
                                Name = m.Name,
                                ArgCount = 1 // sobald wir in Klammern sind, erwarten wir mindestens 1 Argument
                            });
                            isFuncCall = true;
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
                            throw new Exception("Fehlende '('.");

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

                        if (funcStack.Count > 0)
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

                                // Entferne den Marker aus output
                                output.RemoveAt(idx);

                                // Erzeuge Call-Marker mit ArgCount:
                                output.Add(new FunctionCallMarker
                                {
                                    Name = topFunc.Name,
                                    ArgCount = topFunc.ArgCount
                                });
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
                            throw new Exception("Unerwartetes ',' außerhalb eines Funktionsaufrufs?");

                        // Erhöhe ArgCount der aktuellen Funktion
                        if (funcStack.Count == 0)
                            throw new Exception("',' gefunden, aber keine offene Funktion.");

                        funcStack.Peek().ArgCount++;

                        // Nach Komma erwarten wir wieder einen neuen Operand
                        expectUnary = true;
                    }
                    break;

                case TokenType.Plus:
                case TokenType.Minus:
                case TokenType.Star:
                case TokenType.Caret:
                case TokenType.Pipe:
                    {
                        if (expectUnary && t.Type == TokenType.Minus)
                        {
                            opStack.Push(new UnaryMinusMarker());
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
                    throw new Exception($"Unerwartetes Token {t} im Parser.");
            }
        }

        // Stack leeren
        while (opStack.Count > 0)
        {
            var obj = opStack.Pop();
            if (obj is Token tok2 &&
                (tok2.Type == TokenType.LParen || tok2.Type == TokenType.RParen))
            {
                throw new Exception("Klammern unausgeglichen.");
            }
            output.Add(obj);
        }

        if (funcStack.Count > 0)
        {
            throw new Exception("Fehlende ')' bei Funktionsaufruf.");
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
}

internal struct FunctionCallMarker
{
    public string Name;
    public int ArgCount;
    public FunctionCallMarker(string name, int argCount)
    {
        Name = name;
        ArgCount = argCount;
    }
}

internal struct UnaryMinusMarker { }

#endregion

#region Evaluator (RPN auswerten)

public static class Evaluator
{
    public static object Evaluate(string expr, Dictionary<string, object> namedValues)
    {
        try
        {
            var tokens = Lexer.Tokenize(expr);
            var rpn = Parser.ToRpn(tokens);

            var stack = new Stack<Value>();

            foreach (var item in rpn)
            {
                switch (item)
                {
                    case Token t when t.Type == TokenType.Number:
                        {
                            // double mit invariant culture
                            double d = double.Parse(t.Text, CultureInfo.InvariantCulture);
                            stack.Push(Value.FromScalar(d));
                            break;
                        }

                    case Token t when t.Type == TokenType.Plus ||
                                       t.Type == TokenType.Minus ||
                                       t.Type == TokenType.Star ||
                                       t.Type == TokenType.Caret ||
                                       t.Type == TokenType.Pipe:
                        {
                            if (stack.Count < 2)
                                throw new Exception("Zu wenige Operanden für binären Operator.");

                            Value b = stack.Pop();
                            Value a = stack.Pop();

                            Value res;
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
                                case TokenType.Caret:
                                    res = GeometryOps.Cross(a, b);
                                    break;
                                case TokenType.Pipe:
                                    res = GeometryOps.Distance(a, b);
                                    break;
                                default:
                                    throw new Exception("Unerwarteter Operator.");
                            }

                            stack.Push(res);
                            break;
                        }

                    case UnaryMinusMarker _:
                        {
                            if (stack.Count < 1)
                                throw new Exception("Zu wenige Operanden für unary '-'.");
                            var v = stack.Pop();
                            stack.Push(GeometryOps.UnaryMinus(v));
                            break;
                        }

                    case FunctionOrVariableMarker marker:
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
                                if (!namedValues.TryGetValue(marker.Name, out object obj))
                                    throw new Exception($"Unbekannter Name '{marker.Name}'.");

                                stack.Push(ObjectToValue(obj, marker.Name));
                            }
                            break;
                        }

                    case FunctionCallMarker call:
                        {
                            if (stack.Count < call.ArgCount)
                                throw new Exception(
                                    $"Funktion {call.Name} erwartet {call.ArgCount} Argument(e), aber es sind nur {stack.Count} da.");

                            // Argumente rückwärts vom Stack holen
                            var argsReversed = new List<Value>();
                            for (int k = 0; k < call.ArgCount; k++)
                                argsReversed.Add(stack.Pop());

                            // wieder in richtige Reihenfolge bringen (erstes Argument zuerst)
                            argsReversed.Reverse();
                            var args = argsReversed.ToArray();

                            Value fres;

                            switch (call.Name)
                            {
                                // 1-Argument-Funktionen (wie vorher)
                                case "sin":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.FuncSin(args[0]);
                                    break;

                                case "sqrt":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.FuncSqrt(args[0]);
                                    break;

                                case "abs":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.FuncAbs(args[0]);
                                    break;

                                case "len":
                                    CheckArgCount(call, args, 1);
                                    fres = GeometryOps.FuncLen(args[0]);
                                    break;

                                // NEU: p(x,y,z) => GeoPoint
                                case "p":
                                    CheckArgCount(call, args, 3);
                                    fres = MakePoint(args);
                                    break;

                                // NEU: v(x,y,z) => GeoVector
                                case "v":
                                    CheckArgCount(call, args, 3);
                                    fres = MakeVector(args);
                                    break;

                                default:
                                    throw new Exception($"Unbekannte Funktion '{call.Name}'.");
                            }

                            stack.Push(fres);
                            break;
                        }

                    default:
                        throw new Exception($"Unerwartetes RPN-Element: {item}");
                }
            }

            if (stack.Count != 1)
                throw new Exception("Ausdruck unvollständig oder überbestimmt.");

            return stack.Pop().ToObject();
        }
        catch (Exception ex)
        {
            return ex.Message;
        }
    }

    private static void CheckArgCount(FunctionCallMarker call, Value[] args, int expected)
    {
        if (args.Length != expected)
            throw new Exception($"Funktion {call.Name} erwartet {expected} Argument(e), bekommen: {args.Length}.");
    }

    private static Value MakePoint(Value[] args)
    {
        // alle müssen Skalar sein
        if (args[0].Kind != ValueKind.Scalar ||
            args[1].Kind != ValueKind.Scalar ||
            args[2].Kind != ValueKind.Scalar)
            throw new Exception("p(x,y,z): alle Argumente müssen Skalare sein.");

        var pt = new GeoPoint(
            args[0].Scalar,
            args[1].Scalar,
            args[2].Scalar
        );
        return Value.FromPoint(pt);
    }

    private static Value MakeVector(Value[] args)
    {
        if (args[0].Kind != ValueKind.Scalar ||
            args[1].Kind != ValueKind.Scalar ||
            args[2].Kind != ValueKind.Scalar)
            throw new Exception("v(x,y,z): alle Argumente müssen Skalare sein.");

        var vec = new GeoVector(
            args[0].Scalar,
            args[1].Scalar,
            args[2].Scalar
        );
        return Value.FromVector(vec);
    }

    private static Value ObjectToValue(object obj, string nameForError)
    {
        switch (obj)
        {
            case double d:
                return Value.FromScalar(d);

            case GeoPoint p:
                return Value.FromPoint(p);

            case GeoVector v:
                return Value.FromVector(v);

            default:
                // Der Name '{nameForError}' hat einen nicht unterstützten Typ ({obj.GetType().Name}).
                throw new Exception($"Der Name '{nameForError}' hat einen nicht unterstützten Typ ({obj.GetType().Name}).");
        }
    }
}

#endregion
