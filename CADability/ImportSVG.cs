using System;
using System.Collections.Generic;
using MathNet.Numerics;
using System.Text.RegularExpressions;
using System.Xml;
using CADability;
using CADability.GeoObject;
using System.Linq;
using CADability.Curve2D;
using netDxf;
using System.Runtime.InteropServices.ComTypes;
using CADability.Attribute;
using MathNet.Numerics.LinearAlgebra.Factorization;
using System.Numerics;
using CADability.Shapes;
using CADability.Substitutes;

namespace CADability
{
    /// <summary>
    /// Gerüst zum Einlesen einfacher SVG-Elemente und Aufrufe von CreateXXX-Methoden.
    /// Transformationen werden geschachtelt und als Matrix3x2 verwaltet.
    /// </summary>
    public class ImportSVG
    {
        protected struct Vector2
        {
            public Vector2(float x, float y)
            {
                this.x = x;
                this.y = y;
            }
            public float x, y;
            public static Vector2 operator +(Vector2 v1, Vector2 v2)
            {
                return new Vector2(v1.x + v2.x, v1.y + v2.y);
            }
            public static Vector2 operator -(Vector2 v1, Vector2 v2)
            {
                return new Vector2(v1.x - v2.x, v1.y - v2.y);
            }
            public override bool Equals(object obj)
            {
                if (obj is Vector2 v2) return x == v2.x && y == v2.y;
                else return base.Equals(obj);
            }
            public override int GetHashCode()
            {
                return x.GetHashCode() | y.GetHashCode();
            }
        }
        private readonly Stack<ModOp2D> _transformStack;
        public Stack<GeoObjectList> listStack;
        Dictionary<string, string> styles; // current element styles
        Dictionary<string, ColorDef> FillStyles = new Dictionary<string, ColorDef>();
        private readonly Stack<Dictionary<string, string>> _styleStack = new Stack<Dictionary<string, string>>();
        public enum SvgFillRule { NonZero, EvenOdd }
        public enum SvgLineCap { Butt, Round, Square }
        public enum SvgLineJoin { Miter, Round, Bevel }
        /// <summary>
        /// Wenn true, werden Strich-Konturen (stroke) als Flächen (Faces) erzeugt, welche die in SVG
        /// angegebene Linienbreite (stroke-width), Enden (stroke-linecap), Ecken (stroke-linejoin) und
        /// Strichmuster (stroke-dasharray) nachbilden - statt nur die Mittellinie als Kurve zu importieren.
        /// Betrifft Linien, Rechtecke, Kreise, Ellipsen, Polylinien, Pfade und Splines.
        /// </summary>
        public bool StrokeToFaces = false;
        // true, solange CreatePath die Segmente einer Pfad-Definition einsammelt: dann muss Add die
        // Mittellinie liefern (fuer die Pfad-Assemblierung), nicht direkt Stroke-Faces erzeugen.
        private bool _collectingPath = false;
        public ImportSVG()
        {
            _transformStack = new Stack<ModOp2D>();
            _transformStack.Push(ModOp2D.Identity);
            listStack = new Stack<GeoObjectList>();
            listStack.Push(new GeoObjectList());
            // Basis-Style (root)
            _styleStack.Push(new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase));
        }

        /// <summary>
        /// Importiert die SVG-Datei und ruft für gefundene Elemente die entsprechenden Methoden auf.
        /// </summary>
        /// <param name="fileName">Pfad zur SVG-Datei.</param>
        /// <returns>True, wenn erfolgreich importiert wurde.</returns>
        public GeoObjectList Import(string fileName)
        {
            try
            {
                XmlReaderSettings rs = new XmlReaderSettings
                {
                    DtdProcessing = DtdProcessing.Parse,
                    IgnoreComments = true,
                    IgnoreWhitespace = true
                };
                using (XmlReader reader = XmlReader.Create(fileName, rs))
                {
                    while (reader.Read())
                    {
                        if (reader.NodeType == XmlNodeType.Element)
                        {
                            ImportElement(reader);
                        }
                    }
                }
                GeoObjectList result = listStack.Pop();
                BoundingBox ext = result.GetExtent();
                ModOp reflect = ModOp.ReflectPlane(new Plane(new GeoPoint(0, (ext.Ymax + ext.Ymin) / 2.0, 0), GeoVector.YAxis));
                result.Modify(reflect);
                return result;
            }
            catch (Exception ex)
            {
                // TODO: Fehlerbehandlung erweitern (Logging etc.)
                Console.Error.WriteLine(ex.Message);
                return null;
            }
        }

        private void ImportElement(XmlReader reader)
        {
            if (reader.NodeType != XmlNodeType.Element)
                return;

            bool isEmpty = reader.IsEmptyElement;
            string transformAttr = reader.GetAttribute("transform");
            if (!string.IsNullOrEmpty(transformAttr))
            {
                ModOp2D t = ParseTransform(transformAttr);
                ModOp2D current = _transformStack.Peek();
                // SVG: Bildschirm = Eltern(außen) · Element(innen) · p
                // Die Eltern-Transformation steht links, die eigene rechts.
                _transformStack.Push(current * t);
            }
            var computed = ComputeElementStyles(reader);  // geerbte + Präsentationsattribute + inline style
            styles = new Dictionary<string, string>(computed, StringComparer.OrdinalIgnoreCase); // falls du 'styles' später brauchst (z.B. für fill)
            string styleAttr = reader.GetAttribute("style");
            if (!string.IsNullOrEmpty(styleAttr))
            {
                var declarations = styleAttr.Split(';');
                foreach (var decl in declarations)
                {
                    var kv = decl.Split(new[] { ':' }, 2);
                    if (kv.Length == 2)
                    {
                        var name = kv[0].Trim();
                        var value = kv[1].Trim();
                        if (name.Length > 0)
                            styles[name] = value;
                    }
                }
            }

            // Aktuelle Transformationsmatrix
            ModOp2D currentTransform = _transformStack.Peek();

            // Gruppeneinstieg
            if (reader.Name.Equals("g", StringComparison.OrdinalIgnoreCase))
            {
                // Die Gruppe bekommt ihre eigenen Styles: wir mergen sie in einen neuen Stack-Frame
                var parent = _styleStack.Peek();
                var groupFrame = new Dictionary<string, string>(parent, StringComparer.OrdinalIgnoreCase);

                // Präsentationsattribute + inline-style der Gruppe nochmals direkt lesen (damit auch dann klappt,
                // wenn ComputeElementStyles oben schon styles erzeugt hat)
                ReadPresentationAttributes(reader, groupFrame);
                var inlineGroup = ParseStyleAttribute(reader.GetAttribute("style"));
                MergeInto(groupFrame, inlineGroup);

                _styleStack.Push(groupFrame);
                EnterGroup(_transformStack.Peek(), groupFrame);
            }
            // Element-Typ prüfen und Aufruf generieren
            switch (reader.Name)
            {
                case "line":
                    float x1 = ParseFloat(reader.GetAttribute("x1"));
                    float y1 = ParseFloat(reader.GetAttribute("y1"));
                    float x2 = ParseFloat(reader.GetAttribute("x2"));
                    float y2 = ParseFloat(reader.GetAttribute("y2"));
                    CreateLine(x1, y1, x2, y2, currentTransform);
                    break;

                case "rect":
                    float x = ParseFloat(reader.GetAttribute("x"));
                    float y = ParseFloat(reader.GetAttribute("y"));
                    float width = ParseFloat(reader.GetAttribute("width"));
                    float height = ParseFloat(reader.GetAttribute("height"));
                    CreateRect(x, y, width, height, currentTransform);
                    break;

                case "circle":
                    float cx = ParseFloat(reader.GetAttribute("cx"));
                    float cy = ParseFloat(reader.GetAttribute("cy"));
                    float r = ParseFloat(reader.GetAttribute("r"));
                    CreateCircle(cx, cy, r, currentTransform);
                    break;

                case "ellipse":
                    float ecx = ParseFloat(reader.GetAttribute("cx"));
                    float ecy = ParseFloat(reader.GetAttribute("cy"));
                    float rx = ParseFloat(reader.GetAttribute("rx"));
                    float ry = ParseFloat(reader.GetAttribute("ry"));
                    CreateEllipse(ecx, ecy, rx, ry, currentTransform);
                    break;

                case "polyline":
                    string pointsPL = reader.GetAttribute("points");
                    CreatePolyline(pointsPL, currentTransform);
                    break;

                case "polygon":
                    string pointsPG = reader.GetAttribute("points");
                    CreatePolygon(pointsPG, currentTransform);
                    break;

                case "path":
                    string d = reader.GetAttribute("d");
                    CreatePath(d, currentTransform);
                    break;

                    // Weitere Elemente je nach Bedarf hinzufügen
            }

            // Falls keine Selbstschluss-Elemente, rekursiv Kinder verarbeiten
            if (!isEmpty)
            {
                while (reader.Read())
                {
                    if (reader.NodeType == XmlNodeType.Whitespace) continue;
                    if (reader.NodeType == XmlNodeType.Element)
                    {
                        ImportElement(reader);
                    }
                    else if (reader.NodeType == XmlNodeType.EndElement)
                    {
                        break;
                    }
                }
            }

            // Gruppenende
            if (reader.Name.Equals("g", StringComparison.OrdinalIgnoreCase))
            {
                ExitGroup();
                if (_styleStack.Count > 1) _styleStack.Pop();
            }
            // Nach Verlassen des Elements Transformationsmatrix zurücksetzen
            if (!string.IsNullOrEmpty(transformAttr))
            {
                _transformStack.Pop();
            }
        }
        #region Stubs_für_Gruppierung
        protected virtual void EnterGroup(ModOp2D transform, Dictionary<string, string> unused)
        {
            // Effektive Gruppen-Styles aus Attributen/inline-style des <g> berechnen.
            // Wir haben hier keinen Reader – deshalb: Aufrufer (ImportElement) übergibt uns vorher die berechneten Styles.
            // => Wir ändern ImportElement so, dass EnterGroup styles = computedStyles übergeben bekommt.
            // Hier einfach pushen:
            _styleStack.Push(new Dictionary<string, string>(_styleStack.Peek(), StringComparer.OrdinalIgnoreCase));
        }

        protected virtual void ExitGroup()
        {
            if (_styleStack.Count > 1) _styleStack.Pop();
        }
        #endregion

        #region Stub-Methoden zum Überschreiben
        private void Add(ICurve2D curve, ModOp2D transform)
        {
            if (!_collectingPath && StrokeToFaces && CurrentHasStroke())
            {   // einfache Form (Linie, Kreis, Ellipse, Rechteck, Polylinie) mit Stroke -> Flaeche
                AddStrokeFaces(curve, transform);
                return;
            }
            listStack.Peek().Add(curve.GetModified(transform).MakeGeoObject(Plane.XYPlane));
        }

        protected virtual void CreateLine(float x1, float y1, float x2, float y2, ModOp2D transform)
        {
            Line2D l2d = new Line2D(new GeoPoint2D(x1, y1), new GeoPoint2D(x2, y2));
            Add(l2d, transform);
        }

        protected virtual void CreateRect(float x, float y, float width, float height, ModOp2D transform)
        {
            Polyline2D p2d = new Polyline2D(new GeoPoint2D[] { new GeoPoint2D(x, y), new GeoPoint2D(x + width, y), new GeoPoint2D(x + width, y + height), new GeoPoint2D(x, y + height), new GeoPoint2D(x, y) });
            Add(p2d, transform);
        }

        protected virtual void CreateCircle(float cx, float cy, float r, ModOp2D transform)
        {
            Circle2D c2d = new Circle2D(new GeoPoint2D(cx, cy), r);
            Add(c2d, transform);
        }

        protected virtual void CreateEllipse(float cx, float cy, float rx, float ry, ModOp2D transform)
        {
            Ellipse2D e2d = new Ellipse2D(new GeoPoint2D(cx, cy), new GeoVector2D(rx, 0), new GeoVector2D(0, ry));
            Add(e2d, transform);
        }

        protected virtual void CreatePolyline(string points, ModOp2D transform)
        {
            var matches = Regex.Matches(points, @"(-?\d*\.?\d+(?:[eE][+-]?\d+)?)");
            var tokens = new List<string>();
            foreach (Match m in matches)
                tokens.Add(m.Value);
            int i = 0;
            List<GeoPoint2D> pointList = new List<GeoPoint2D>();
            while (i < tokens.Count)
            {
                float x = ParseFloat(tokens[i++]);
                float y = ParseFloat(tokens[i++]);
                pointList.Add(new GeoPoint2D(x, y));
            }
            Polyline2D pl2d = new Polyline2D(pointList.ToArray());
            Add(pl2d, transform);
        }

        protected virtual void CreatePolygon(string points, ModOp2D transform)
        {
            var matches = Regex.Matches(points, @"(-?\d*\.?\d+(?:[eE][+-]?\d+)?)");
            var tokens = new List<string>();
            foreach (Match m in matches)
                tokens.Add(m.Value);
            int i = 0;
            List<GeoPoint2D> pointList = new List<GeoPoint2D>();
            while (i < tokens.Count)
            {
                float x = ParseFloat(tokens[i++]);
                float y = ParseFloat(tokens[i++]);
                pointList.Add(new GeoPoint2D(x, y));
            }
            pointList.Add(pointList[0]);
            Polyline2D pl2d = new Polyline2D(pointList.ToArray());
            Add(pl2d, transform);
        }

        protected virtual void CreateCubicBezier(Vector2 start, Vector2 control1, Vector2 control2, Vector2 end, ModOp2D transform)
        {
            if (control2.x == end.x && control2.y == end.y) CreateQuadraticBezier(start, control1, end, transform);
            else if (control2.x == control1.x && control2.y == control1.y) CreateQuadraticBezier(start, control1, end, transform);
            else if (start.x == control1.x && start.y == control1.y) CreateQuadraticBezier(start, control2, end, transform);
            else
            {
                BSpline2D bsp2d = new BSpline2D(new GeoPoint2D[] { new GeoPoint2D(start.x, start.y), new GeoPoint2D(control1.x, control1.y), new GeoPoint2D(control2.x, control2.y), new GeoPoint2D(end.x, end.y) },
                    new double[] { 1.0, 1.0, 1.0, 1.0 }, new double[] { 0.0, 1.0 }, new int[] { 4, 4 }, 3, false, 0.0, 1.0);
                Add(bsp2d, transform);
            }
        }

        protected virtual void CreateQuadraticBezier(Vector2 start, Vector2 control, Vector2 end, ModOp2D transform)
        {
            BSpline2D bsp2d = new BSpline2D(new GeoPoint2D[] { new GeoPoint2D(start.x, start.y), new GeoPoint2D(control.x, control.y), new GeoPoint2D(end.x, end.y) },
                new double[] { 1.0, 1.0, 1.0 }, new double[] { 0.0, 1.0 }, new int[] { 3, 3 }, 2, false, 0.0, 1.0);
            Add(bsp2d, transform);
        }

        protected virtual void CreateEllipticalArc(Vector2 start, float frx, float fry, float xAxisRotation, bool largeArcFlag, bool sweepFlag, Vector2 end, ModOp2D transform)
        {
            double rx = frx;
            double ry = fry;
            // Winkel in Radiant
            double phi = xAxisRotation * (Math.PI / 180.0);

            // Schritt 1: Koordinaten in Ellipsen-Raum transformieren
            double dx2 = (start.x - end.x) / 2f;
            double dy2 = (start.y - end.y) / 2f;
            double x1p = (Math.Cos(phi) * dx2 + Math.Sin(phi) * dy2);
            double y1p = (-Math.Sin(phi) * dx2 + Math.Cos(phi) * dy2);

            // Schritt 2: Radien anpassen
            double rxSq = rx * rx;
            double rySq = ry * ry;
            double x1pSq = x1p * x1p;
            double y1pSq = y1p * y1p;
            double lambda = x1pSq / rxSq + y1pSq / rySq;
            if (lambda > 1)
            {
                double factor = Math.Sqrt(lambda);
                rx *= factor;
                ry *= factor;
                rxSq = rx * rx;
                rySq = ry * ry;
            }

            // Schritt 3: Mittelpunkt in Rotiertem Raum
            double sign = (largeArcFlag == sweepFlag) ? -1f : 1f;
            double num = rxSq * rySq - rxSq * y1pSq - rySq * x1pSq;
            double denom = rxSq * y1pSq + rySq * x1pSq;
            double coef = sign * Math.Sqrt(Math.Max(0, num / denom));
            double cxp = coef * ((rx * y1p) / ry);
            double cyp = coef * (-(ry * x1p) / rx);

            // Schritt 4: zurücktransformieren
            double cx = (Math.Cos(phi) * cxp - Math.Sin(phi) * cyp + (start.x + end.x) / 2f);
            double cy = (Math.Sin(phi) * cxp + Math.Cos(phi) * cyp + (start.y + end.y) / 2f);

            // Achsen-Vektoren
            var majorAxis = new GeoVector2D((rx * Math.Cos(phi)), (rx * Math.Sin(phi)));
            var minorAxis = new GeoVector2D((-ry * Math.Sin(phi)), (ry * Math.Cos(phi)));

            // Punkte
            var center = new GeoPoint2D(cx, cy);
            var startPoint = new GeoPoint2D(start.x, start.y);
            var endPoint = new GeoPoint2D(end.x, end.y);

            // sweepFlag: true = CW → counterClock = false
            bool counterClockwise = sweepFlag;

            // Transform auf Punkte/Achsen anwenden
            // Erzeugung
            EllipseArc2D ea = EllipseArc2D.Create(center, majorAxis, minorAxis, startPoint, endPoint, counterClockwise);
            Add(ea, transform);
        }

        protected virtual void CreatePath(string data, ModOp2D transform)
        {
            if (string.IsNullOrWhiteSpace(data))
                return;
            listStack.Push(new GeoObjectList());
            List<GeoObjectList> subPaths = new List<GeoObjectList>();
            // Tokenize: Befehle und Zahlen
            var matches = Regex.Matches(data, @"([MmZzLlHhVvCcQqAaSsTt])|(-?\d*\.?\d+(?:[eE][+-]?\d+)?)");

            var tokens = new List<string>();
            foreach (Match m in matches)
                tokens.Add(m.Value);

            int i = 0;
            char cmd = ' ';
            char prevCmd = ' ';

            Vector2 current = new Vector2(0.0f, 0.0f);
            Vector2 startPoint = new Vector2();
            Vector2 lastCp = new Vector2();

            _collectingPath = true; // Segmente einsammeln: Add liefert Mittellinien, keine Stroke-Faces
            while (i < tokens.Count)
            {
                string token = tokens[i++];
                if (Regex.IsMatch(token, "[MmZzLlHhVvCcQqAaSsTt]"))
                {
                    cmd = token[0];
                }
                else
                {
                    --i;
                    if (cmd == 'M') cmd = 'L'; // nach Move automatisch Line
                    if (cmd == 'm') cmd = 'l';
                }

                bool isRelative = char.IsLower(cmd);
                char uc = char.ToUpper(cmd);
                switch (uc)
                {
                    case 'M':
                        float x = ParseFloat(tokens[i++]);
                        float y = ParseFloat(tokens[i++]);
                        var p = new Vector2(x, y);
                        if (isRelative) p += current;
                        current = p;
                        startPoint = p;
                        // lastControl zurücksetzen, da kein vorheriger Cubic
                        lastCp = current;
                        if (listStack.Peek().Count > 0)
                        {
                            subPaths.Add(listStack.Pop());
                            listStack.Push(new GeoObjectList());
                        }
                        break;

                    case 'L':
                        x = ParseFloat(tokens[i++]);
                        y = ParseFloat(tokens[i++]);
                        p = new Vector2(x, y);
                        if (isRelative) p += current;
                        CreateLine(current.x, current.y, p.x, p.y, transform);
                        current = p;
                        break;

                    case 'H':
                        x = ParseFloat(tokens[i++]);
                        p = new Vector2(isRelative ? current.x + x : x, current.y);
                        CreateLine(current.x, current.y, p.x, p.y, transform);
                        current = p;
                        break;

                    case 'V':
                        y = ParseFloat(tokens[i++]);
                        p = new Vector2(current.x, isRelative ? current.y + y : y);
                        CreateLine(current.x, current.y, p.x, p.y, transform);
                        current = p;
                        break;

                    case 'C':
                        float x1 = ParseFloat(tokens[i++]);
                        float y1 = ParseFloat(tokens[i++]);
                        float x2 = ParseFloat(tokens[i++]);
                        float y2 = ParseFloat(tokens[i++]);
                        x = ParseFloat(tokens[i++]);
                        y = ParseFloat(tokens[i++]);
                        var cp1 = new Vector2(x1, y1);
                        var cp2 = new Vector2(x2, y2);
                        p = new Vector2(x, y);
                        if (isRelative)
                        {
                            cp1 += current;
                            cp2 += current;
                            p += current;
                        }
                        lastCp = cp2;
                        CreateCubicBezier(current, cp1, cp2, p, transform);
                        current = p;
                        break;
                    case 'S': // smooth cubic
                              // Berechne ersten Kontrollpunkt als Spiegelung:
                        Vector2 reflected;
                        // Spiegelung: reflektiere lastControl über current
                        reflected = current + (current - lastCp);
                        // 2) Lese (x2,y2) und (x,y) (ggf. relativ addieren)
                        x1 = ParseFloat(tokens[i++]);
                        y1 = ParseFloat(tokens[i++]);
                        lastCp = new Vector2(x1, y1);
                        x = ParseFloat(tokens[i++]);
                        y = ParseFloat(tokens[i++]);
                        p = new Vector2(x, y);
                        if (isRelative)
                        {
                            lastCp += current;
                            p += current;
                        }
                        if (prevCmd == 'C' || prevCmd == 'c' || prevCmd == 'S' || prevCmd == 's')
                        {
                            CreateCubicBezier(current, reflected, lastCp, p, transform);
                        }
                        else
                        {
                            CreateQuadraticBezier(current, lastCp, p, transform);
                        }
                        current = p;
                        break;
                    case 'Q':
                        x1 = ParseFloat(tokens[i++]);
                        y1 = ParseFloat(tokens[i++]);
                        x = ParseFloat(tokens[i++]);
                        y = ParseFloat(tokens[i++]);
                        lastCp = new Vector2(x1, y1);
                        p = new Vector2(x, y);
                        if (isRelative)
                        {
                            lastCp += current;
                            p += current;
                        }
                        CreateQuadraticBezier(current, lastCp, p, transform);
                        current = p;
                        break;

                    case 'T':
                        x = ParseFloat(tokens[i++]);
                        y = ParseFloat(tokens[i++]);
                        lastCp = current + (current - lastCp);
                        p = new Vector2(x, y);
                        if (isRelative)
                        {
                            p += current;
                        }
                        CreateQuadraticBezier(current, lastCp, p, transform);
                        current = p;
                        break;

                    case 'A':
                        float rx = ParseFloat(tokens[i++]);
                        float ry = ParseFloat(tokens[i++]);
                        float angle = ParseFloat(tokens[i++]);
                        bool largeArc = tokens[i++] == "1";
                        bool sweep = tokens[i++] == "1";
                        x = ParseFloat(tokens[i++]);
                        y = ParseFloat(tokens[i++]);
                        p = new Vector2(x, y);
                        if (isRelative)
                            p += current;
                        CreateEllipticalArc(current, rx, ry, angle, largeArc, sweep, p, transform);
                        current = p;
                        break;

                    case 'Z':
                        CreateLine(current.x, current.y, startPoint.x, startPoint.y, transform);
                        //if (!isRelative) current = startPoint;
                        current = startPoint;
                        break;

                    default:
                        // Unhandled
                        break;
                }
                prevCmd = cmd;
            }

            _collectingPath = false;
            GeoObjectList list = listStack.Pop();
            if (list.Count > 0) subPaths.Add(list);
            var fillRule = GetEffectiveFillRule(styles);
            ColorDef cd = ColorDef.CDfromParent;
            bool fill = false;
            if (styles.TryGetValue("fill", out string color) && !color.Trim().Equals("none", StringComparison.OrdinalIgnoreCase))
            {
                Color clr = ParseSvgColor(color);
                if (!clr.IsEmpty)
                {
                    fill = true;
                    // Hex-Farben tragen keinen Namen (clr.Name == ""), daher als eindeutigen
                    // Schlüssel den ARGB-Wert verwenden - sonst landen alle Farben unter "SVG+"
                    // und alle Flächen bekommen die zuerst erzeugte ColorDef.
                    string key = "SVG+" + clr.ToArgb().ToString("X8");
                    if (!FillStyles.TryGetValue(key, out cd))
                    {
                        cd = new ColorDef(key, clr);
                        FillStyles[key] = cd;
                    }
                }
            }
            // Alle 2D-Segmente der (beim Füllen implizit geschlossenen) Subpfade sammeln.
            // CADability bestimmt daraus selbst die Flächen-Hierarchie (Inseln/Löcher).
            List<ICurve2D> fillSegments = new List<ICurve2D>();
            double maxGap = 0.0;
            for (int j = 0; j < subPaths.Count; j++)
            {
                List<ICurve> lgo = new List<ICurve>(subPaths[j].OfType<ICurve>());
                Path path = Path.FromSegments(lgo, true);
                if (path == null) continue;
                double prec = path.GetExtent(0.0).Size * 0.002;
                maxGap = Math.Max(maxGap, prec);
                path.RemoveShortSegments(prec);
                // SVG schließt gefüllte Subpfade implizit (gerade Linie vom End- zum Startpunkt).
                // Daher hier jeden noch offenen Subpfad schließen, nicht nur die vorher geschlossenen.
                if (fill && !path.IsClosed)
                {
                    if ((path.EndPoint | path.StartPoint) < prec)
                    {   // kleiner Versatz am Ende: Start- und Endpunkt zusammenziehen
                        GeoPoint mp = new GeoPoint(path.EndPoint, path.StartPoint);
                        path.StartPoint = mp;
                        path.EndPoint = mp;
                    }
                    else
                    {
                        List<ICurve> curves = new List<ICurve>(path.Curves);
                        curves.Add(Line.TwoPoints(path.EndPoint, path.StartPoint));
                        path = Path.FromSegments(curves, true);
                    }
                }
                if (fill)
                {
                    foreach (ICurve c in path.Curves)
                    {
                        ICurve2D c2d = c.GetProjectedCurve(Plane.XYPlane);
                        if (c2d != null && c2d.Length > prec) fillSegments.Add(c2d);
                    }
                }
                // Stroke nur, wenn tatsächlich gezeichnet wird (nicht "none"/leer). Sonst
                // entstehen schwarze Outline-Pfade, die die gefüllten Flächen überlagern.
                bool hasStroke = styles.TryGetValue("stroke", out string strokeVal)
                    && !string.IsNullOrWhiteSpace(strokeVal)
                    && !strokeVal.Trim().Equals("none", StringComparison.OrdinalIgnoreCase);
                if (StrokeToFaces && hasStroke)
                {
                    // Strich-Kontur als Flaeche erzeugen. Dazu in lokale (untransformierte)
                    // Koordinaten zurueckrechnen, damit die Strichbreite korrekt mitskaliert wird.
                    ICurve2D centerline2d = path.GetProjectedCurve(Plane.XYPlane);
                    if (centerline2d != null) AddStrokeFaces(centerline2d.GetModified(transform.GetInverse()), transform);
                }
                else if (!fill || hasStroke)
                {
                    listStack.Peek().Add(path);
                }
            }
            // Aus allen gesammelten Segmenten eine CompoundShape bauen. CreateFromList ermittelt
            // die Verschachtelung selbst (partInPart=true => Inseln werden zu Löchern); jede
            // resultierende SimpleShape wird zu einer eigenen Face.
            if (fill && fillSegments.Count > 0)
            {
                CompoundShape cs = CompoundShape.CreateFromList(fillSegments.ToArray(), maxGap, true, out GeoObjectList dead);
                if (cs != null)
                {
                    foreach (SimpleShape ss in cs.SimpleShapes)
                    {
                        Face fc = Face.MakeFace(new PlaneSurface(Plane.XYPlane), ss);
                        fc.ColorDef = cd;
                        listStack.Peek().Add(fc);
                    }
                }
            }
        }

        #endregion

        #region Stroke-zu-Flaeche (StrokeToFaces)

        private bool CurrentHasStroke()
        {
            if (styles == null) return false;
            if (!styles.TryGetValue("stroke", out string stroke) || string.IsNullOrWhiteSpace(stroke)) return false;
            return !stroke.Trim().Equals("none", StringComparison.OrdinalIgnoreCase);
        }

        private bool TryGetStrokeStyle(out ColorDef cd, out double width, out SvgLineCap cap, out SvgLineJoin join, out double miterLimit, out double[] dashes)
        {
            cd = ColorDef.CDfromParent;
            width = 1.0;                 // SVG-Default
            cap = SvgLineCap.Butt;       // SVG-Default
            join = SvgLineJoin.Miter;    // SVG-Default
            miterLimit = 4.0;            // SVG-Default
            dashes = null;
            if (!CurrentHasStroke()) return false;

            string stroke = styles["stroke"].Trim();
            Color clr = ParseSvgColor(stroke);
            if (clr.IsEmpty) clr = Color.FromArgb(0, 0, 0);
            string key = "SVG+" + clr.ToArgb().ToString("X8");
            if (!FillStyles.TryGetValue(key, out cd))
            {
                cd = new ColorDef(key, clr);
                FillStyles[key] = cd;
            }
            if (styles.TryGetValue("stroke-width", out string sw))
            {
                double w = ParseLength(sw);
                if (w > 0) width = w;
            }
            if (styles.TryGetValue("stroke-linecap", out string lc))
            {
                lc = lc.Trim();
                if (lc.Equals("round", StringComparison.OrdinalIgnoreCase)) cap = SvgLineCap.Round;
                else if (lc.Equals("square", StringComparison.OrdinalIgnoreCase)) cap = SvgLineCap.Square;
            }
            if (styles.TryGetValue("stroke-linejoin", out string lj))
            {
                lj = lj.Trim();
                if (lj.Equals("round", StringComparison.OrdinalIgnoreCase)) join = SvgLineJoin.Round;
                else if (lj.Equals("bevel", StringComparison.OrdinalIgnoreCase)) join = SvgLineJoin.Bevel;
            }
            if (styles.TryGetValue("stroke-miterlimit", out string ml))
            {
                double m = ParseLength(ml);
                if (m > 0) miterLimit = m;
            }
            if (styles.TryGetValue("stroke-dasharray", out string da)) dashes = ParseDashArray(da);
            return true;
        }

        // Erzeugt aus einer (lokalen, untransformierten) Mittellinie die Stroke-Flaeche(n),
        // transformiert sie und legt sie als Faces ab.
        private void AddStrokeFaces(ICurve2D localCurve, ModOp2D transform)
        {
            if (localCurve == null) return;
            if (!TryGetStrokeStyle(out ColorDef cd, out double width, out SvgLineCap cap, out SvgLineJoin join, out double miterLimit, out double[] dashes)) return;
            List<SimpleShape> shapes = BuildStrokeShapes(localCurve, width, cap, join, dashes);
            foreach (SimpleShape ss in shapes)
            {
                SimpleShape tss = ss.GetModified(transform);
                Face fc = Face.MakeFace(new PlaneSurface(Plane.XYPlane), tss);
                fc.ColorDef = cd;
                listStack.Peek().Add(fc);
            }
        }

        private List<SimpleShape> BuildStrokeShapes(ICurve2D curve, double width, SvgLineCap cap, SvgLineJoin join, double[] dashes)
        {
            List<SimpleShape> result = new List<SimpleShape>();
            if (curve == null || width <= 0) return result;
            double d = width / 2.0;
            IEnumerable<ICurve2D> pieces;
            if (dashes != null && dashes.Length > 0 && !curve.IsClosed)
                pieces = SplitIntoDashes(curve, dashes);
            else
                pieces = new ICurve2D[] { curve };
            foreach (ICurve2D piece in pieces)
            {
                SimpleShape ss = StrokePiece(piece, d, cap, join);
                if (ss != null) result.Add(ss);
            }
            return result;
        }

        // Erzeugt die Kontur-Flaeche eines einzelnen (offenen oder geschlossenen) Kurvenstuecks.
        private SimpleShape StrokePiece(ICurve2D curve, double d, SvgLineCap cap, SvgLineJoin join)
        {
            if (curve == null || curve.Length < d * 1e-4) return null;
            double prec = d * 0.001;
            double roundAngle;
            switch (join)
            {
                case SvgLineJoin.Round: roundAngle = Math.PI; break; // immer runde Aussenecken
                case SvgLineJoin.Bevel: roundAngle = Math.PI; break; // vereinfacht wie 'round'
                default: roundAngle = 0.0; break;                    // Miter: spitze Ecke (zwei Linien)
            }
            try
            {
                if (curve.IsClosed)
                {   // geschlossen: zwei parallele Raender -> Ring (aeusserer Rand mit Loch)
                    ICurve2D off1 = curve.Parallel(d, false, prec, roundAngle);
                    ICurve2D off2 = curve.Parallel(-d, false, prec, roundAngle);
                    if (off1 == null || off2 == null) return null;
                    Border b1 = new Border(AsSegments(off1), true);
                    Border b2 = new Border(AsSegments(off2), true);
                    if (Math.Abs(b1.Area) >= Math.Abs(b2.Area)) return new SimpleShape(b1, b2);
                    return new SimpleShape(b2, b1);
                }
                else
                {   // offen: rechter Versatz vor, Endkappe, linker Versatz zurueck, Startkappe
                    ICurve2D right = curve.Parallel(d, false, prec, roundAngle);
                    ICurve2D left = curve.Parallel(-d, false, prec, roundAngle);
                    if (right == null || left == null) return null;
                    List<ICurve2D> loop = new List<ICurve2D>();
                    loop.AddRange(AsSegments(left));
                    AddCap(loop, cap, curve.EndPoint, curve.EndDirection, left.EndPoint, right.EndPoint, d);
                    ICurve2D rightRev = right.Clone();
                    rightRev.Reverse();
                    loop.AddRange(AsSegments(rightRev));
                    GeoVector2D startOut = curve.StartDirection;
                    AddCap(loop, cap, curve.StartPoint, new GeoVector2D(-startOut.x, -startOut.y), right.StartPoint, left.StartPoint, d);
                    Border b = new Border(loop.ToArray(), true);
                    return new SimpleShape(b);
                }
            }
            catch (Exception)
            {
                return null;
            }
        }

        // Haengt die Endkappe (von 'from' nach 'to', Mittelpunkt 'center', nach aussen 'outward') an.
        private static void AddCap(List<ICurve2D> loop, SvgLineCap cap, GeoPoint2D center, GeoVector2D outward, GeoPoint2D from, GeoPoint2D to, double d)
        {
            double olen = Math.Sqrt(outward.x * outward.x + outward.y * outward.y);
            switch (cap)
            {
                case SvgLineCap.Square:
                    if (olen < 1e-12) { loop.Add(new Line2D(from, to)); break; }
                    {
                        double ux = outward.x / olen * d, uy = outward.y / olen * d;
                        GeoPoint2D p1 = new GeoPoint2D(from.x + ux, from.y + uy);
                        GeoPoint2D p2 = new GeoPoint2D(to.x + ux, to.y + uy);
                        loop.Add(new Line2D(from, p1));
                        loop.Add(new Line2D(p1, p2));
                        loop.Add(new Line2D(p2, to));
                    }
                    break;
                case SvgLineCap.Round:
                    {
                        double dx = from.x - to.x, dy = from.y - to.y;
                        if (dx * dx + dy * dy < 1e-18) break;
                        Arc2D arc = new Arc2D(center, d, from, to, true);
                        GeoPoint2D mid = arc.PointAt(0.5);
                        double dot = (mid.x - center.x) * outward.x + (mid.y - center.y) * outward.y;
                        if (dot < 0) arc = new Arc2D(center, d, from, to, false); // Bogen nach aussen woelben
                        loop.Add(arc);
                    }
                    break;
                default: // Butt
                    loop.Add(new Line2D(from, to));
                    break;
            }
        }

        private static ICurve2D[] AsSegments(ICurve2D c)
        {
            if (c is Path2D p) return p.SubCurves;
            return new ICurve2D[] { c };
        }

        // Zerlegt eine offene Kurve gemaess Strichmuster in die "an"-Stuecke.
        private static IEnumerable<ICurve2D> SplitIntoDashes(ICurve2D curve, double[] dashes)
        {
            List<ICurve2D> res = new List<ICurve2D>();
            double total = curve.Length;
            if (total <= 1e-9) return res;
            int di = 0;
            double pos = 0.0;
            bool on = true;
            int guard = 0;
            while (pos < total - 1e-9 && guard++ < 1000000)
            {
                double dash = dashes[di % dashes.Length];
                double next = Math.Min(pos + dash, total);
                if (on && dash > 1e-9 && next > pos)
                {
                    double t0 = curve.PositionAtLength(pos);
                    double t1 = curve.PositionAtLength(next);
                    try
                    {
                        ICurve2D piece = curve.Trim(t0, t1);
                        if (piece != null && piece.Length > 1e-9) res.Add(piece);
                    }
                    catch (Exception) { }
                }
                pos = next;
                di++;
                on = !on;
            }
            return res;
        }

        // Parst eine Laengenangabe (ignoriert Einheiten-Suffixe wie px).
        private static double ParseLength(string s)
        {
            if (string.IsNullOrWhiteSpace(s)) return 0.0;
            s = s.Trim();
            int n = 0;
            while (n < s.Length && (char.IsDigit(s[n]) || s[n] == '.' || s[n] == '-' || s[n] == '+' || s[n] == 'e' || s[n] == 'E')) n++;
            string num = s.Substring(0, n);
            return double.TryParse(num, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out double v) ? v : 0.0;
        }

        private static double[] ParseDashArray(string s)
        {
            if (string.IsNullOrWhiteSpace(s) || s.Trim().Equals("none", StringComparison.OrdinalIgnoreCase)) return null;
            var parts = Regex.Split(s.Trim(), "[,\\s]+");
            List<double> vals = new List<double>();
            foreach (var p in parts)
            {
                if (string.IsNullOrWhiteSpace(p)) continue;
                double v = ParseLength(p);
                vals.Add(v < 0 ? 0 : v);
            }
            if (vals.Count == 0) return null;
            if (vals.Count % 2 == 1) { int c = vals.Count; for (int k = 0; k < c; k++) vals.Add(vals[k]); } // ungerade -> verdoppeln
            double sum = 0; foreach (var v in vals) sum += v;
            if (sum <= 1e-9) return null;
            return vals.ToArray();
        }

        #endregion

        private static float ParseFloat(string s)
        {
            return string.IsNullOrEmpty(s) ? 0f : float.Parse(s, System.Globalization.CultureInfo.InvariantCulture);
        }
        public static Color ParseSvgColor(string value)
        {
            if (string.IsNullOrEmpty(value) || value.Equals("none", StringComparison.OrdinalIgnoreCase))
                return Color.Empty;
            value = value.Trim();
            // hex #RGB or #RRGGBB or #RRGGBBAA or named color
            if (value.StartsWith("#"))
            {
                return Color.FromString(value);
            }
            // rgb() or rgba()
            if (value.StartsWith("rgb(", StringComparison.OrdinalIgnoreCase) || value.StartsWith("rgba(", StringComparison.OrdinalIgnoreCase))
            {
                string inner = value.Substring(value.IndexOf('(') + 1).TrimEnd(')');
                var parts = inner.Split(',');
                int r = ParseComponent(parts[0]);
                int g = ParseComponent(parts[1]);
                int b = ParseComponent(parts[2]);
                int a = 255;
                if (parts.Length == 4)
                {
                    if (parts[3].Trim().EndsWith("%"))
                    {
                        float p = float.Parse(parts[3].Trim().TrimEnd('%'), System.Globalization.CultureInfo.InvariantCulture) / 100f;
                        a = (int)(p * 255);
                    }
                    else
                    {
                        float fa = float.Parse(parts[3], System.Globalization.CultureInfo.InvariantCulture);
                        a = (int)(fa <= 1 ? fa * 255 : fa);
                    }
                }
                return Color.FromArgb(a, r, g, b);
            }
            // named color
            try
            {
                return Color.FromString(value);
            }
            catch
            {
                return Color.Empty;
            }
        }

        private static int ParseComponent(string s)
        {
            s = s.Trim();
            if (s.EndsWith("%"))
            {
                float p = float.Parse(s.TrimEnd('%'), System.Globalization.CultureInfo.InvariantCulture) / 100f;
                return (int)(p * 255);
            }
            return int.Parse(s, System.Globalization.CultureInfo.InvariantCulture);
        }

        private static ModOp2D ParseTransform(string transform)
        {
            var result = ModOp2D.Identity;
            // Matcht Funktionen wie "translate(10,20)" oder "rotate(45)"
            var regex = new Regex("(\\w+)\\([^)]*\\)", RegexOptions.Compiled);
            foreach (Match m in regex.Matches(transform))
            {
                string fn = m.Groups[1].Value;
                // Innerhalb der Klammern: Zahlen, durch , oder Leerzeichen getrennt
                string inner = transform.Substring(m.Index + fn.Length + 1, m.Length - fn.Length - 2);
                var parts = Regex.Split(inner, "[,\\s]+");
                var args = new List<float>();
                foreach (var p in parts)
                    if (!string.IsNullOrWhiteSpace(p))
                        args.Add(ParseFloat(p));

                switch (fn)
                {
                    case "matrix":
                        // args: a, b, c, d, e, f
                        float a = args[0], b = args[1], c = args[2], d = args[3], e = args[4], f = args[5];
                        // TODO: Ersetze YourMatrix mit deinem Matrix-Konstruktor
                        result = new ModOp2D(a, c, e, b, d, f);
                        break;
                    case "translate":
                        float tx = args[0];
                        float ty = args.Count > 1 ? args[1] : 0;
                        result = ModOp2D.Translate(tx, ty);
                        break;
                    case "scale":
                        float sx = args[0];
                        float sy = args.Count > 1 ? args[1] : sx;
                        result = ModOp2D.Scale(sx, sy);
                        break;
                    case "rotate":
                        float angle = args[0];
                        if (args.Count > 2)
                        {
                            float cx = args[1], cy = args[2];
                            result = ModOp2D.Rotate(new GeoPoint2D(cx, cy), SweepAngle.Deg(angle));
                        }
                        else
                        {
                            result = ModOp2D.Rotate(SweepAngle.Deg(angle));
                        }
                        break;
                    case "skewX":
                        float ax = args[0];
                        result = ModOp2D.Scale(ax, 1);
                        break;
                    case "skewY":
                        float ay = args[0];
                        result = ModOp2D.Scale(1, ay);
                        break;
                    default:
                        // Unbekanntes Transform-Element ignorieren
                        break;
                }
            }
            return result;
        }
        private static void MergeInto(Dictionary<string, string> target, IDictionary<string, string> src)
        {
            if (src == null) return;
            foreach (var kv in src)
                target[kv.Key] = kv.Value;
        }

        private static Dictionary<string, string> ParseStyleAttribute(string styleAttr)
        {
            var dict = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            if (string.IsNullOrEmpty(styleAttr)) return dict;
            var declarations = styleAttr.Split(';');
            foreach (var decl in declarations)
            {
                var kv = decl.Split(new[] { ':' }, 2);
                if (kv.Length == 2)
                {
                    var name = kv[0].Trim();
                    var value = kv[1].Trim();
                    if (name.Length > 0) dict[name] = value;
                }
            }
            return dict;
        }

        // Nimmt Präsentationsattribute direkt am Element mit auf (z.B. fill, fill-rule, stroke, fill-opacity …)
        private static void ReadPresentationAttributes(XmlReader reader, Dictionary<string, string> dict)
        {
            // Die wichtigsten fürs Füllen
            var fr = reader.GetAttribute("fill-rule");
            if (!string.IsNullOrEmpty(fr)) dict["fill-rule"] = fr;

            var fill = reader.GetAttribute("fill");
            if (!string.IsNullOrEmpty(fill)) dict["fill"] = fill;

            var fillOpacity = reader.GetAttribute("fill-opacity");
            if (!string.IsNullOrEmpty(fillOpacity)) dict["fill-opacity"] = fillOpacity;

            var stroke = reader.GetAttribute("stroke");
            if (!string.IsNullOrEmpty(stroke)) dict["stroke"] = stroke;

            var strokeWidth = reader.GetAttribute("stroke-width");
            if (!string.IsNullOrEmpty(strokeWidth)) dict["stroke-width"] = strokeWidth;
        }

        // Effektive Styles für ein Element berechnen: geerbte Werte (Stack Top) + Präsentationsattribute + inline style
        private Dictionary<string, string> ComputeElementStyles(XmlReader reader)
        {
            var computed = new Dictionary<string, string>(_styleStack.Peek(), StringComparer.OrdinalIgnoreCase);

            // Präsentationsattribute zuerst (damit inline style sie überschreiben kann)
            var presentational = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            ReadPresentationAttributes(reader, presentational);
            MergeInto(computed, presentational);

            // inline style="..."
            string styleAttr = reader.GetAttribute("style");
            var inline = ParseStyleAttribute(styleAttr);
            MergeInto(computed, inline);

            return computed;
        }
        private static SvgFillRule GetEffectiveFillRule(IReadOnlyDictionary<string, string> computedStyles)
        {
            if (computedStyles != null && computedStyles.TryGetValue("fill-rule", out var fr))
            {
                // SVG erlaubt "nonzero" (Default) und "evenodd"
                if (fr.Equals("evenodd", StringComparison.OrdinalIgnoreCase)) return SvgFillRule.EvenOdd;
                // Alles andere wie "nonzero" behandeln (inkl. leer/unkannt -> nonzero)
            }
            return SvgFillRule.NonZero;
        }

    }
}