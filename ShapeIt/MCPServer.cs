using CADability;
using CADability.Attribute;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.Substitutes;
using CdlToCSharp;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Reflection.Metadata.Ecma335;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml.Linq;
using static ShapeIt.ShellExtensions;
using static System.Runtime.InteropServices.JavaScript.JSType;
using Plane = CADability.Plane;

namespace ShapeIt
{
    public partial class MCPServer
    {
        public class NamedItemsDictionary
        {
            private readonly Dictionary<string, object> dict = new(StringComparer.Ordinal);
            public NamedItemsDictionary() { }
            public NamedItemsDictionary(NamedItemsDictionary other)
            {
                foreach (var item in other.dict)
                {
                    dict[item.Key] = item.Value;
                }
            }

            public IEnumerable<string> Keys => dict.Keys;
            public IEnumerable<object> Values => dict.Values;
            public IEnumerable<KeyValuePair<string, object>> Items => dict;

            public bool ContainsKey(string key) => dict.ContainsKey(key);
            public IEnumerator<KeyValuePair<string, object>> GetEnumerator() => dict.GetEnumerator();

            public Dictionary<string, object> Dict => dict;
            public object this[string key]
            {
                get => dict[key];
                set
                {
                    dict[key] = value;
                    if (value is Solid sld) sld.Name = key;
                }
            }
            public bool TryGetValue(string key, out object? value) => dict.TryGetValue(key, out value);

            internal void Remove(string name)
            {
                dict.Remove(name);
            }
        }

        // Named workspace items and created objects.
        // Names are chosen by the caller (LLM/client). 
        private NamedItemsDictionary namedItems = new();
        public Dictionary<string, List<JsonElement>> templates = [];

        private class NamedItemOverride : IDisposable
        {
            private object? oldNamedItem;
            private string name;
            NamedItemsDictionary namedItems;
            public NamedItemOverride(NamedItemsDictionary namedItems, object temp, string name = "this")
            {
                this.namedItems = namedItems;
                this.name = name;
                if (!namedItems.TryGetValue(name, out oldNamedItem)) oldNamedItem = null;
                object? wrappedItem = wrapForEvaluator(temp);
                if (wrappedItem != null) namedItems[name] = wrappedItem;
            }
            public void Dispose()
            {
                if (oldNamedItem == null) namedItems.Remove(name);
                else namedItems[name] = oldNamedItem;
            }
        }
        private class NamedItemClone : IDisposable
        {
            NamedItemsDictionary namedItems;
            MCPServer server;
            public NamedItemClone(MCPServer server)
            {
                this.server = server;
                namedItems = server.namedItems;
                server.namedItems = new NamedItemsDictionary(namedItems);
                // this is a flat copy, so in theory we could change the values. But I cannot think of a way where values are changed
                // typically they are overwritten (in the new dictionary) with new values, which is not a problem here
            }

            public void Dispose()
            {
                server.namedItems = namedItems;
            }
        }

        private Stack<NamedItemClone> namedItemClones = new();

        private int nextId = 1;
        private int nextUndo = 1;
        public MCPServer() { }

        private void StoreNamed(string name, object value)
        {
            namedItems[name] = value;
        }

        private void AddNamed<T>(string name, T value)
        {
            if (!namedItems.TryGetValue(name, out var existing))
            {
                namedItems[name] = value!;
                return;
            }

            if (existing is List<T> list)
            {
                list.Add(value!);
                return;
            }

            if (existing is T existingT)
            {
                namedItems[name] = new List<T> { existingT, value! };
                return;
            }

            throw new InvalidOperationException(
                $"Name '{name}' is already bound to a value of type '{existing.GetType().FullName}', cannot add '{typeof(T).FullName}'.");
        }
        private void Rebind(Shell oldShell, Shell newShell)
        {
            foreach (var item in namedItems)
            {
                if (item.Value is Edge edge && edge.Owner.Owner == oldShell)
                {
                    Edge? newEdge = newShell.FindSimilarEdge(edge);
                    if (newEdge != null) namedItems[item.Key] = newEdge;
                }
                if (item.Value is List<Edge> ledge)
                {
                    List<Edge> newList = [];
                    for (int i = 0; i < ledge.Count; i++)
                    {
                        if (ledge[i].Owner.Owner == oldShell)
                        {
                            Edge? newEdgel = newShell.FindSimilarEdge(ledge[i]);
                            if (newEdgel != null) newList.Add(newEdgel);
                            else newList.Add(ledge[i]);
                        }
                        else newList.Add(ledge[i]);
                    }
                    namedItems[item.Key] = newList;
                }
                if (item.Value is Face face && face.Owner == oldShell)
                {
                    Face? newFace = newShell.FindSimilarFace(face);
                    if (newFace != null) namedItems[item.Key] = newFace;
                }
                if (item.Value is List<Face> lface)
                {
                    List<Face> newList = [];
                    for (int i = 0; i < lface.Count; i++)
                    {
                        if (lface[i].Owner == oldShell)
                        {
                            Face? newFacel = newShell.FindSimilarFace(lface[i]);
                            if (newFacel != null) newList.Add(newFacel);
                            else newList.Add(lface[i]);
                        }
                        else newList.Add(lface[i]);
                    }
                    namedItems[item.Key] = newList;
                }
            }
        }
        private void Rebind(Solid oldSolid, Solid[] newSolids)
        {
            foreach (Solid solid in newSolids)
            {
                Rebind(oldSolid.Shell, solid.Shell);
            }
        }
        private void Rebind(IEnumerable<Solid> oldSolids, IEnumerable<Solid> newSolids)
        {
            foreach (Solid solid1 in oldSolids)
            {
                foreach (Solid solid in newSolids)
                {
                    Rebind(solid1.Shell, solid.Shell);
                }
            }
        }
        private void Rebind(IEnumerable<Solid> oldSolids, Solid newSolid)
        {
            foreach (Solid solid1 in oldSolids)
            {
                Rebind(solid1.Shell, newSolid.Shell);
            }
        }
        private string? FindName(object entity)
        {
            foreach (var item in namedItems)
            {
                if (item.Value == entity) return item.Key;
            }
            return null;
        }

        private JsonNode DocumentGetStateImpl() => throw new NotImplementedException();
        private void UndoBeginImpl(string label)
        {

        }
        private void UndoEndImpl(string undoFrameId)
        {

        }
        private JsonNode UndoCancelImpl(string undoFrameId) => throw new NotImplementedException();

        private void WorkspaceSetImpl(string name, JsonElement value, string? label, JsonElement input)
        {
            if (value.ValueKind == JsonValueKind.Number)
            {   // only get numbers here, expressions are evaluated below
                double d = GetOptionalDouble(value, null, double.NaN);
                if (!double.IsNaN(d))
                {
                    namedItems[name] = d;
                }
            }
            else if (value.ValueKind == JsonValueKind.String) // e.g. "v(1,2,3)" to define a vector
            {
                namedItems[name] = Evaluator.Evaluate(value.GetString(), namedItems.Dict);
            }
            else if (value.ValueKind == JsonValueKind.Object && value.TryGetProperty("expr", out var JeExpr) && JeExpr.ValueKind == JsonValueKind.String)
            {
                namedItems[name] = Evaluator.Evaluate(JeExpr.GetString(), namedItems.Dict);
            }
            else
            {
                List<object> selected = IterateSelector<object>(value).ToList();
                object? typedList = MakeTypedList(selected);
                if (typedList != null) namedItems[name] = typedList;


            }
        }

        private object? MakeTypedList(List<object> selected)
        {
            Type? t = selected.FirstOrDefault()?.GetType();
            if (t != null && selected.All(x => x?.GetType() == t))
            {
                if (t == typeof(Edge))
                {
                    return selected.Cast<Edge>().ToList();
                }
                else if (t == typeof(Face))
                {
                    return selected.Cast<Face>().ToList();
                }
                else if (t == typeof(Solid))
                {
                    return selected.Cast<Solid>().ToList();
                }
                else if (t == typeof(ICurve))
                {
                    return selected.Cast<ICurve>().ToList();
                }
                else if (t == typeof(ICurve2D))
                {
                    return selected.Cast<ICurve2D>().ToList();
                }
                else if (t == typeof(CompoundShape))
                {
                    return selected.Cast<CompoundShape>().ToList();
                }
            }
            return null;
        }

        private void WorkspaceDeleteImpl(JsonElement objects) => throw new NotImplementedException();

        private void SketchCreateImpl(Plane plane, string? name)
        {
            Sketch sketch = new Sketch(plane);
            if (name != null) namedItems[name] = sketch;
        }
        private void SketchCreateOnFaceImpl(JsonElement face, GeoPoint origin, GeoVector xAxis, string? name)
        {
            List<Face> lf = IterateSelector<Face>(face).ToList();
            if (lf.Count == 0) throw new JsonRpcException(-32602, "No face found for SketchCreateOnFace.");
            GeoVector xdir = xAxis;
            GeoPoint2D uv = lf[0].PositionOf(origin);
            GeoVector normal = lf[0].Surface.GetNormal(uv);
            GeoVector ydir = xdir ^ normal;
            Sketch? sketch = new Sketch(new Plane(origin, xdir, ydir));
            if (name != null) namedItems[name] = sketch;
        }
        private void SketchAddArcImpl(Sketch sketch, GeoPoint2D center, double radius, double startAngleDeg, double sweepAngleDeg, GeoPoint2D start, GeoPoint2D end, GeoPoint2D middle, bool ccw, string name)
        {
            ICurve2D? curve = null;
            if (center.IsValid && !double.IsNaN(radius) && !double.IsNaN(startAngleDeg) && !double.IsNaN(sweepAngleDeg))
            {
                curve = new Arc2D(center, radius, Angle.Deg(startAngleDeg), SweepAngle.Deg(sweepAngleDeg));
            }
            else if (center.IsValid && start.IsValid && end.IsValid)
            {
                curve = new Arc2D(center, center | start, start, end, ccw);
            }
            else if (middle.IsValid && start.IsValid && end.IsValid)
            {   // quick and dirty: use the existin 3d construction of an arc and project it back to 2d
                Ellipse tmp = Ellipse.Construct();
                Plane pln = sketch.Plane;
                tmp.SetArc3Points(pln.ToGlobal(start), pln.ToGlobal(middle), pln.ToGlobal(end), pln);
                curve = tmp.GetProjectedCurve(pln);
            }

            if (curve != null)
            {
                sketch.Add(curve);
                if (name != null) namedItems[name] = curve;
            }
            else throw new JsonRpcException(-32602, "Invalid parameters for sketch.add_arc (or not implemented).");
        }

        private void SketchAddCircleImpl(Sketch sketch, GeoPoint2D center, double radius, double diameter, string name)
        {
            if (double.IsNaN(radius) && double.IsNaN(diameter))
                throw new JsonRpcException(-32602, "Circle must have either radius or diameter.");
            if (double.IsNaN(radius)) radius = diameter / 2.0;
            ICurve2D curve = new Circle2D(center, radius);
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private object? GetSketchPointOrGeometry(JsonElement item)
        {
            string? expr = null;
            if (item.ValueKind == JsonValueKind.String) expr = item.GetString();
            if (item.ValueKind == JsonValueKind.Object)
            {
                if (item.TryGetProperty("name", out var nameEl))
                {
                    if (nameEl.ValueKind == JsonValueKind.String) expr = nameEl.GetString();
                }
                if (item.TryGetProperty("expr", out var exprEl))
                {
                    if (exprEl.ValueKind == JsonValueKind.String) expr = exprEl.GetString();
                }
                if (expr != null) return Evaluator.Evaluate(expr, namedItems.Dict);
                if (item.TryGetProperty("x", out var xEl) && item.TryGetProperty("y", out var yEl))
                {
                    return RequirePoint2D(item); // everything managed there
                }
            }
            if (item.ValueKind == JsonValueKind.Array) return RequirePoint2D(item); // everything managed there
            return null;
        }
        private void SketchAddCircleByConstraintsImpl(Sketch sketch, JsonElement constraints, double radius, GeoPoint2D center, GeoPoint2D preferredCenter, double tolerance, string name, JsonElement capture)
        {
            if (constraints.ValueKind != JsonValueKind.Array) throw new JsonRpcException("E_INVALID_PARAMETER", "constaints must be an array.");
            List<object> constaintObjects = [];
            foreach (var item in constraints.EnumerateArray())
            {
                object? constr = GetSketchPointOrGeometry(item);
                if (constr == null) throw new JsonRpcException("E_INVALID_PARAMETER", "object constraints not recognized.");
                constaintObjects.Add(constr);
            }
            Circle2D? circle = null;
            if (constaintObjects.Count == 3)
            {   // three tangents, may be curves or points
                List<ICurve2D> lc = [];
                List<GeoPoint2D> lp = [];
                List<GeoPoint2D> touchpoints = [];
                for (int i = 0; i < constaintObjects.Count; i++)
                {
                    if (constaintObjects[i] is ICurve2D c2d) lc.Add(c2d);
                    if (constaintObjects[i] is GeoPoint2D p2d) lp.Add(p2d);
                }
                if (lc.Count == 1)
                {
                    lc.Add(new Circle2D(lp[0], 0.0));
                    lc.Add(new Circle2D(lp[1], 0.0));
                }
                if (lc.Count == 2)
                {
                    lc.Add(new Circle2D(lp[0], 0.0));
                }
                if (lc.Count == 3)
                {   // tangential to 3 curves
                    GeoPoint2D[] circlePoints = Curves2D.TangentCircle(lc[0], lc[1], lc[2], GeoPoint2D.Invalid, GeoPoint2D.Invalid, GeoPoint2D.Invalid);
                    // the result is quadruples: center, point on first curve ...
                    int ind = -1;
                    if (preferredCenter.IsValid)
                    {
                        double minDist = double.MaxValue;
                        for (int j = 0; j < circlePoints.Length; j += 4)
                        {
                            double d = circlePoints[j] | preferredCenter;
                            if (d < minDist)
                            {
                                minDist = d;
                                ind = j;
                            }
                        }
                    }
                    else if (circlePoints.Length > 0) ind = 0;
                    circle = new Circle2D(circlePoints[ind], circlePoints[ind + 1] | circlePoints[ind]);
                    touchpoints.AddRange([circlePoints[ind + 1], circlePoints[ind + 2], circlePoints[ind + 3]]);
                }
                if (lp.Count == 3)
                {
                    if (Geometry.CircleFit(lp[0], lp[1], lp[2], out GeoPoint2D cnt, out double r))
                    {
                        circle = new Circle2D(cnt, r);
                    }
                    touchpoints.AddRange(lp); // 
                }
                if (circle == null) throw new JsonRpcException("E_OPERATION_FAILED", "could not construct circle from constraints.");
                sketch.Add(circle);
                if (name != null) namedItems[name] = circle;
                string? centerName = GetOptionalString(capture, "center");
                string? touch0Name = GetOptionalString(capture, "touch0");
                string? touch1Name = GetOptionalString(capture, "touch1");
                string? touch2Name = GetOptionalString(capture, "touch2");
                if (centerName != null) namedItems[centerName] = circle.Center;
                if (touch0Name != null) namedItems[touch0Name] = touchpoints[0];
                if (touch1Name != null) namedItems[touch1Name] = touchpoints[1];
                if (touch2Name != null) namedItems[touch2Name] = touchpoints[2];
            }
        }
        private void SketchAddLineByConstraintsImpl(Sketch sketch, GeoPoint2D start, JsonElement target0, JsonElement target1, GeoPoint2D preferredStart, GeoPoint2D preferredEnd, double tolerance, string name)
        {
            throw new NotImplementedException();
        }
        private void SketchSetCurveEndpointsImpl(Sketch sketch, JsonElement curve, GeoPoint2D start, GeoPoint2D end, string direction, bool projectToCurve, bool copy, string name, JsonElement capture)
        {
            List<ICurve2D> ca = IterateSelector<ICurve2D>(curve).ToList(); // should only be one
            if (ca.Count != 1) throw new JsonRpcException("E_INVALID_PARAMETER", "curve must contain exactely one curve.");
            ICurve2D crv = ca[0];
            if (copy || name != null) crv = crv.Clone();
            if (crv is Line2D l)
            {
                if (projectToCurve)
                {
                    if (start.IsValid)
                    {
                        GeoPoint2D sp = Geometry.DropPL(start, l.StartPoint, l.EndPoint);
                        l.StartPoint = sp;
                    }
                    if (end.IsValid)
                    {
                        GeoPoint2D ep = Geometry.DropPL(end, l.StartPoint, l.EndPoint);
                        l.EndPoint = ep;
                    }
                }
                else
                {
                    if (start.IsValid) l.StartPoint = start;
                    if (end.IsValid) l.EndPoint = end;
                }
            }
            if (crv is Circle2D c2d) // which is both circle and arc
            {
                if (!(c2d is Arc2D a2d))
                {
                    if (direction == null) direction = "shortest";
                    if (!start.IsValid || !end.IsValid) throw new JsonRpcException("E_INVALID_PARAMETER", "bot start and end must be set to make an arc from a circle.");
                    if ((direction == "ccw" && c2d.Sweep < 0) || (direction == "cw" && c2d.Sweep > 0)) c2d.Reverse();

                    a2d = new Arc2D(c2d.Center, c2d.Radius, start, end, c2d.Sweep > 0);
                }
                else
                {
                    if (!start.IsValid) start = a2d.StartPoint;
                    if (!end.IsValid) end = a2d.EndPoint;
                    a2d = new Arc2D(c2d.Center, c2d.Radius, start, end, direction == "cw");
                }
                if (direction == "shortest")
                {
                    if (Math.Abs(a2d.SweepAngle) > Math.PI) a2d.Complement();
                }
                if (direction == "longest")
                {
                    if (Math.Abs(a2d.SweepAngle) < Math.PI) a2d.Complement();
                }
                crv = a2d;
            }
            if (!sketch.Curves.Contains(crv)) sketch.Add(crv);
            if (name != null) namedItems[name] = crv;
        }

        private void SketchAddEllipseImpl(Sketch sketch, GeoPoint2D center, double radiusMajor, double radiusMinor, double rotationDeg, string name)
        {
            double major = radiusMajor;
            double minor = radiusMajor;
            double rotation = rotationDeg;
            ModOp2D rot = ModOp2D.Rotate(center, SweepAngle.Deg(rotation));
            GeoVector2D majAxis = rot * (major * GeoVector2D.XAxis);
            GeoVector2D minAxis = rot * (minor * GeoVector2D.YAxis);
            ICurve2D curve = new Ellipse2D(center, majAxis, minAxis);
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private void SketchAddEllipseArcImpl(Sketch sketch, GeoPoint2D center, double radiusMajor, double radiusMinor, double rotationDeg, double startAngleDeg, double sweepAngleDeg, string name)
        {
            throw new NotImplementedException();
        }

        private void SketchAddLineImpl(Sketch sketch, GeoPoint2D start, GeoPoint2D end, string name)
        {
            ICurve2D curve = new Line2D(start, end);
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private void SketchAddNurbsImpl(Sketch sketch, int degree, JsonElement controlPointsEl, JsonElement throughPointsEl, JsonElement knotsEl, JsonElement weightsEl, bool isPeriodic, JsonElement approximation, string name)
        {
            BSpline2D? curve = null;
            if (approximation.ValueKind == JsonValueKind.Object && approximation.TryGetProperty("parameter", out var _) && approximation.TryGetProperty("pointExpr", out var _))
            {
                string parameter = RequireString(approximation, "parameter");
                JsonElement pointExpr = RequireProperty(approximation, "pointExpr");
                string? xExpr = null, yExpr = null;
                if (pointExpr.ValueKind == JsonValueKind.Array)
                {
                    foreach (var xy in pointExpr.EnumerateArray())
                    {
                        if (xy.ValueKind == JsonValueKind.String)
                        {
                            if (xExpr == null) xExpr = xy.GetString();
                            else yExpr = xy.GetString();
                        }
                    }
                }
                else if (pointExpr.ValueKind == JsonValueKind.Object)
                {
                    xExpr = RequireString(pointExpr, "x");
                    yExpr = RequireString(pointExpr, "y");
                }
                if (xExpr == null || yExpr == null) throw new JsonRpcException("E_INVALID_PARAMS", $"Point must be defined as Expression depending on {parameter}");
                double tMin = RequireDouble(approximation, "tMin");
                double tMax = RequireDouble(approximation, "tMax");
                int minSamples = GetOptionalInteger(approximation, "minSamples", 4);
                int maxSamples = GetOptionalInteger(approximation, "maxSamples", 40);
                double tolerance = GetOptionalDouble(approximation, "tolerance", Precision.eps);
                namedItems.TryGetValue(parameter, out var oldValue);
                Func<double, GeoPoint2D> crv = (d) =>
                {
                    namedItems[parameter] = d;
                    double x = (double)Evaluator.Evaluate(xExpr, namedItems.Dict);
                    double y = (double)Evaluator.Evaluate(yExpr, namedItems.Dict);
                    return new GeoPoint2D(x, y);
                };
                // curve = BSpline2D.Approximate(crv, tolerance, tMin, tMax, maxSamples);
                // a problemhere: Approximate uses the parameters and the points. When crv has a different speed at the beginning and end
                // the resulting curve has bad conditions. We need an approximate without parameter synchronisation
                GeoPoint2D[] pnts = new GeoPoint2D[20];
                for (int i = 0; i < pnts.Length; i++)
                {
                    pnts[i] = crv(tMin + i * (tMax - tMin) / (pnts.Length - 1));
                }
                curve = new BSpline2D(pnts, 3, isPeriodic);
                if (oldValue == null) namedItems.Remove(parameter);
                else namedItems[parameter] = oldValue;
            }
            else if (throughPointsEl.ValueKind == JsonValueKind.Array)
            {
                List<GeoPoint2D> throughPoints = [];
                foreach (var pnt in throughPointsEl.EnumerateArray())
                {
                    throughPoints.Add(RequirePoint2D(pnt, null));
                }
                curve = new BSpline2D(throughPoints.ToArray(), degree, isPeriodic);
            }
            else
            {
                throw new NotImplementedException();
            }
            if (curve != null)
            {
                sketch.Add(curve);
                if (name != null) namedItems[name] = curve;
            }
        }

        private void SketchAddPolycurveImpl(Sketch sketch, JsonElement verticesEl, bool isClosed, string name)
        {
            if (verticesEl.ValueKind != JsonValueKind.Array)
                throw new JsonRpcException(-32602, "vertices must be array");

            List<(double x, double y, double b)> vertices = [];

            foreach (var seg in verticesEl.EnumerateArray())
            {
                double vx = RequireDouble(seg, "x");
                double vy = RequireDouble(seg, "y");
                double vb = GetOptionalDouble(seg, "b", 0.0);
                vertices.Add((vx, vy, vb));
            }

            List<ICurve2D> curves = [];
            for (int i = 0; i < vertices.Count; i++)
            {
                GeoPoint2D pn;

                if (i == vertices.Count - 1)
                {
                    if (!isClosed) continue;
                    pn = new GeoPoint2D(vertices[0].x, vertices[0].y);
                }
                else pn = new GeoPoint2D(vertices[i + 1].x, vertices[i + 1].y);
                GeoPoint2D pi = new GeoPoint2D(vertices[i].x, vertices[i].y);
                if (vertices[i].b == 0.0)
                {
                    curves.Add(new Line2D(pi, pn));
                }
                else
                {
                    GeoVector2D d = pn - pi;
                    double b = vertices[i].b;
                    double c = d.Length;
                    double r = Math.Abs(c * (1 + b * b) / (4 * b));
                    double h = c * (1 - b * b) / (4 * b);
                    GeoVector2D n = d.ToLeft();
                    GeoPoint2D center = new GeoPoint2D(pi, pn) + h / c * n;
                    curves.Add(new Arc2D(center, r, pi, pn, h >= 0));
                }
            }
            ICurve2D curve = new Path2D(curves.ToArray());
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private void SketchAddRectangleImpl(Sketch sketch, double width, double height, double cornerRadius, GeoPoint2D center, double rotationDeg, string name)
        {
            ICurve2D curve;
            if (cornerRadius == 0)
                curve = Polyline2D.MakeRectangle(center, width, height, SweepAngle.Deg(rotationDeg));
            else
                curve = Path2D.CreateRoundedRectangle(center, width, height, cornerRadius, SweepAngle.Deg(rotationDeg));
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }


        private void SketchAddRegularPolygonImpl(Sketch sketch, GeoPoint2D center, double innerRadius, double outerRadius, int sides, double rotationDeg, string name)
        {
            if (double.IsNaN(outerRadius) || outerRadius == 0.0) outerRadius = innerRadius / Math.Cos(Math.PI / sides);
            ICurve2D curve = Polyline2D.MakeRegularPolygon(center, outerRadius, rotationDeg * Math.PI / 180.0, sides);
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private void SketchAddSlotImpl(Sketch sketch, GeoPoint2D center, double length, double width, double rotationDeg, string name)
        {
            GeoVector2D dir = GeoVector2D.XAxis;
            if (!double.IsNaN(rotationDeg)) dir = GeoVector2D.FromAngle(Angle.Deg(rotationDeg));
            double radius = width / 2;
            GeoPoint2D startPoint = center - (length / 2 - radius) * dir;
            GeoPoint2D endPoint = center + (length / 2 - radius) * dir;
            Border lh = Border.MakeLongHole(startPoint, endPoint, radius, radius);
            Path2D curve = lh.AsPath();
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }
        private void SketchAddTextImpl(Sketch sketch, string text, GeoPoint2D location, double height, JsonElement font, JsonElement horizontalAlign, JsonElement verticalAlign, double characterSpacing, double wordSpacing, string name)
        {
            string fontFamily = RequireString(font, "family");
            bool bold = GetOptionalBool(font, "bold", false);
            bool italic = GetOptionalBool(font, "italic", false);
            bool underline = GetOptionalBool(font, "underline", false);
            bool strikeout = GetOptionalBool(font, "strikeout", false);


            Text goText = Text.Construct();
            goText.TextString = text;
            goText.Font = fontFamily;
            goText.Location = GeoPoint.Origin;
            goText.LineDirection = GeoVector.XAxis;
            goText.GlyphDirection = GeoVector.YAxis;

            CompoundShape[] shapes = goText.GetShapes();
            List<CompoundShape> res = [];
            for (int i = 0; i < shapes.Length; i++)
            {
                CompoundShape cs = shapes[i].GetModified(ModOp2D.Translate(location.x, location.y) * ModOp2D.Scale(height));
                res.Add(cs);
                sketch.Add(cs);
            }
            if (name != null) namedItems[name] = res;

        }

        private void SketchBooleanImpl(Sketch sketch, string op, JsonElement inputs, JsonElement subtract, string? name)
        {
            CompoundShape? result = null;
            List<CompoundShape> inputshapes = IterateSelector<CompoundShape>(inputs).ToList();
            List<ICurve2D> inputcurves = IterateSelector<ICurve2D>(inputs).ToList();
            for (int i = 0; i < inputcurves.Count; i++)
            {
                if (inputcurves[i].IsClosed)
                {
                    inputshapes.Add(new CompoundShape(new SimpleShape(new Border(inputcurves[i]))));
                }
            }
            List<CompoundShape> subtractshapes = [];
            if (subtract.ValueKind != JsonValueKind.Undefined)
            {
                subtractshapes = IterateSelector<CompoundShape>(subtract).ToList();
                List<ICurve2D> subtractcurves = IterateSelector<ICurve2D>(subtract).ToList();
                for (int i = 0; i < subtractcurves.Count; i++)
                {
                    if (subtractcurves[i].IsClosed)
                    {
                        subtractshapes.Add(new CompoundShape(new SimpleShape(new Border(subtractcurves[i]))));
                    }
                }
            }
            switch (op)
            {
                case "union":
                case "unite":
                    {
                        if (inputshapes.Count < 2) throw new JsonRpcException(-32602, "We need at least 2 shapes for union.");
                        result = inputshapes[0];
                        for (int i = 1; i < inputshapes.Count; i++)
                        {
                            result = CompoundShape.Union(result, inputshapes[i]);
                        }
                    }
                    break;
                case "intersect":
                    {
                        if (inputshapes.Count < 2) throw new JsonRpcException(-32602, "We need at least 2 shapes for intersection.");
                        result = inputshapes[0];
                        for (int i = 1; i < inputshapes.Count; i++)
                        {
                            result = CompoundShape.Intersection(result, inputshapes[i]);
                        }
                    }
                    break;
                case "subtract":
                case "difference":
                    {
                        if (inputshapes.Count != 1) throw new JsonRpcException(-32602, "We need at least 1 shape to subtract from.");
                        if (subtractshapes.Count < 1) throw new JsonRpcException(-32602, "We need at least 1 shape to subtract with.");
                        result = inputshapes[0];
                        for (int i = 0; i < subtractshapes.Count; i++)
                        {
                            result = CompoundShape.Difference(result, subtractshapes[i]);
                        }
                    }
                    break;
                default:
                    throw new JsonRpcException(-32601, $"Unknown sketch boolean operation '{op}'");
            }

            if (name != null && result != null)
            {
                result.UserData.Add("MCPServer.Sketch", sketch);
                StoreNamed(name, result);
            }

        }


        private void SketchConnectImpl(Sketch sketch, JsonElement entities, double precision, bool closeGaps, string name)
        {
            List<ICurve2D> toConnect = IterateSelector<ICurve2D>(entities).ToList();
            if (toConnect.Count < 2) throw new JsonRpcException(-32602, "We need at least 2 curves to connect.");
            Reduce2D r2d = new Reduce2D();
            r2d.OutputMode = Reduce2D.Mode.Paths;
            r2d.Add(toConnect.ToArray());
            ICurve2D[] reduced = r2d.Reduced;
            if (name != null) namedItems[name] = reduced.ToList();
            if (sketch != null)
            {
                foreach (ICurve2D c in reduced)
                {
                    sketch.Add(c);
                }
            }
            ;
        }
        private void ProfileFromRegionsImpl(JsonElement regions, string? name)
        {   // regins are for extracting. There is no difference between CompoundShapes from a sketch and a region.
            // Maybe we should connect open curves.
            List<CompoundShape> inputshapes = new List<CompoundShape>();
            if (regions.ValueKind == JsonValueKind.Array)
            {
                foreach (var inp in regions.EnumerateArray())
                {
                    object obj = ResolveObjectRef(inp);
                    if (obj != null)
                    {
                        if (obj is ICurve2D c2d && c2d.IsClosed)
                        {
                            Border bdr = new Border(c2d);
                            CompoundShape cs = new CompoundShape(new SimpleShape(new Border(c2d)));
                            cs.UserData.Add("MCPServer.Sketch", c2d.UserData["MCPServer.Sketch"]);
                            inputshapes.Add(cs);
                        }
                        else if (obj is CompoundShape cs)
                        {
                            inputshapes.Add(cs);
                        }
                        else throw new JsonRpcException(-32602, "All regions must be closed shapes.");
                    }
                    else throw new JsonRpcException(-32602, "All regions must be sketch shapes.");
                }
            }
            if (name != null) namedItems[name] = inputshapes;
        }

        private List<SimpleShape> GetProfiles(JsonElement profile, out Sketch? sketch)
        {
            List<object> profiles = IterateSelector<object>(profile).ToList(); // should return a single sketch or a compoundShape or a closed curve
            sketch = null;
            List<SimpleShape> simpleShapes = new List<SimpleShape>();
            foreach (object obj in profiles)
            {
                if (obj is Sketch ps)
                {
                    sketch = ps;
                    CompoundShape? pcs = sketch.GetCompoundShape();
                    if (pcs != null) simpleShapes.AddRange(pcs.SimpleShapes);
                }
                else if (obj is CompoundShape cs)
                {
                    sketch = cs.UserData["MCPServer.Sketch"] as Sketch;
                    simpleShapes.AddRange(cs.SimpleShapes);
                }
                else if (obj is List<CompoundShape> cslist)
                {
                    for (int i = 0; i < cslist.Count; i++)
                    {
                        if (sketch == null) sketch = cslist[i].UserData["MCPServer.Sketch"] as Sketch;
                        foreach (SimpleShape simpleShape in cslist[i].SimpleShapes)
                        {
                            simpleShapes.Add(simpleShape);
                        }
                    }
                }
                else if (obj is ICurve2D c2d && c2d.IsClosed)
                {
                    if (sketch == null) sketch = c2d.UserData["MCPServer.Sketch"] as Sketch;
                    simpleShapes.Add(new SimpleShape(new Border(c2d)));
                }
                else throw new JsonRpcException(-32602, "Profile must be a sketch shape.");
            }
            return simpleShapes;
        }
        private void SolidExtrudeImpl(JsonElement profile, double length, GeoVector direction, double offset, string? name, JsonElement capture)
        {
            List<SimpleShape> simpleShapes = GetProfiles(profile, out Sketch? sketch);

            if (sketch != null)
            {
                string? startEdges = GetOptionalString(capture, "startEdges");
                string? endEdges = GetOptionalString(capture, "endEdges");
                string? startFace = GetOptionalString(capture, "startFace");
                string? endFace = GetOptionalString(capture, "endFace");
                List<Solid> solids = new List<Solid>();
                PlaneSurface ps = new PlaneSurface(sketch.Plane);
                GeoVector dir = direction.IsValid() ? direction : ps.Normal.Normalized;
                dir.Length = length;
                for (int i = 0; i < simpleShapes.Count; i++)
                {
                    Face face = Face.MakeFace(ps, simpleShapes[i]);
                    if (face != null)
                    {
                        if (offset != 0.0) face.Modify(ModOp.Translate(offset * dir.Normalized));
                        Solid? sld = Make3D.Extrude(face, dir, null) as Solid;
                        if (sld != null)
                        {
                            if (startEdges != null || startFace != null)
                            {
                                GeoPoint2D point2dOnFace = face.Area.GetSomeInnerPoint();
                                GeoPoint point3dOnFace = face.Surface.PointAt(point2dOnFace);
                                Face startFaceOfExtrusion = sld.FindFace(point3dOnFace);
                                if (!string.IsNullOrEmpty(startFace))
                                {
                                    namedItems[startFace] = startFaceOfExtrusion;
                                }
                                if (!string.IsNullOrEmpty(startEdges))
                                {
                                    namedItems[startEdges] = new List<Edge>(startFaceOfExtrusion.Edges); ;
                                }
                            }
                            if (endEdges != null || endFace != null)
                            {
                                GeoPoint2D point2dOnFace = face.Area.GetSomeInnerPoint();
                                GeoPoint point3dOnFace = face.Surface.PointAt(point2dOnFace) + dir;
                                Face endFaceOfExtrusion = sld.FindFace(point3dOnFace);
                                if (!string.IsNullOrEmpty(endFace))
                                {
                                    namedItems[endFace] = endFaceOfExtrusion;
                                }
                                if (!string.IsNullOrEmpty(endEdges))
                                {
                                    namedItems[endEdges] = new List<Edge>(endFaceOfExtrusion.Edges); ;
                                }
                            }
                            solids.Add(sld);
                        }
                    }
                }
                if (name != null) namedItems[name] = solids;
            }
        }
        private void SolidHelicalExtrudeImpl(JsonElement profile, Axis axis, double angle, double offset, double pitch, string name, JsonElement capture)
        {
            List<SimpleShape> simpleShapes = GetProfiles(profile, out Sketch? sketch);

            if (sketch != null)
            {
                string? startEdges = GetOptionalString(capture, "startEdges");
                string? endEdges = GetOptionalString(capture, "endEdges");
                string? startFace = GetOptionalString(capture, "startFace");
                string? endFace = GetOptionalString(capture, "endFace");
                List<Solid> solids = new List<Solid>();
                PlaneSurface ps = new PlaneSurface(sketch.Plane);
                for (int i = 0; i < simpleShapes.Count; i++)
                {
                    Face face = Face.MakeFace(ps, simpleShapes[i]);
                    if (face != null)
                    {
                        Shell shl = Make3D.MakeHelicalSolid(face, axis, pitch, pitch * angle / 360, 0.0, true);
                        if (shl != null)
                        {
                            Solid sld = Solid.MakeSolid(shl);
                            if (sld != null)
                            {
                                if (startEdges != null || startFace != null)
                                {
                                    GeoPoint2D point2dOnFace = face.Area.GetSomeInnerPoint();
                                    GeoPoint point3dOnFace = face.Surface.PointAt(point2dOnFace);
                                    Face startFaceOfExtrusion = sld.FindFace(point3dOnFace);
                                    if (!string.IsNullOrEmpty(startFace))
                                    {
                                        namedItems[startFace] = startFaceOfExtrusion;
                                    }
                                    if (!string.IsNullOrEmpty(startEdges))
                                    {
                                        namedItems[startEdges] = new List<Edge>(startFaceOfExtrusion.Edges);
                                    }
                                }
                                if (endEdges != null || endFace != null)
                                {
                                }
                                solids.Add(sld);
                            }
                        }
                    }
                }
                if (name != null) namedItems[name] = solids;
            }
        }


        private void SolidBoxImpl(GeoPoint origin, GeoVector axisX, GeoVector axisY, double sizeX, double sizeY, double sizeZ, string name)
        {
            GeoVector axisZ;
            if (axisX.IsValid() && axisY.IsValid())
            {
                axisZ = axisX ^ axisY;
                axisX.Norm();
                axisY.Norm();
                axisZ.Norm();
            }
            else
            {
                axisX = GeoVector.XAxis;
                axisY = GeoVector.YAxis;
                axisZ = GeoVector.ZAxis;
            }
            Solid res = Make3D.MakeBox(origin, sizeX * axisX, sizeY * axisY, sizeZ * axisZ);
            if (name != null) namedItems[name] = res;
        }

        private void SolidCapsuleImpl(GeoPoint start, GeoPoint end, double radius, string cap, double coneTipDistance, string name)
        {
            Solid? res = null;
            double l = end | start; // the length of the capsule
            Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
            Plane profilePlane = new Plane(start, end - start, pln.DirectionX); // in this plane we construct a profile for rotation along the x-axis of the plane
            GeoVector dirx = radius * pln.ToGlobal(GeoVector2D.XAxis);
            Solid cylinder = Make3D.MakeCylinder(start, dirx, end - start);
            if (double.IsNaN(coneTipDistance))
            {
                // sphericalTips
                Arc2D arcstart = new Arc2D(GeoPoint2D.Origin, radius, Angle.Deg(180), SweepAngle.Deg(90));
                Arc2D arcend = new Arc2D(new GeoPoint2D(l, 0), radius, Angle.Deg(270), SweepAngle.Deg(90));
                Line2D line1 = new Line2D(arcstart.EndPoint, arcend.StartPoint);
                Line2D line2 = new Line2D(arcend.EndPoint, arcstart.StartPoint);
                Path2D profile = new Path2D(new ICurve2D[] { arcstart, line1, arcend, line2 });
                Path? profile3D = profile.MakeGeoObject(profilePlane) as Path;
                var rotated = Make3D.Rotate(profile3D, new Axis(start, end), SweepAngle.Full, 0.0, null);
                if (rotated is Solid sld) res = sld;
            }
            else
            {
                Solid cone1 = Make3D.MakeCone(start, dirx, coneTipDistance * (start - end).Normalized, radius, 0.0);
                Solid cone2 = Make3D.MakeCone(end, dirx, coneTipDistance * (end - start).Normalized, radius, 0.0);
                res = BooleanOperation.Unite(cone1, cylinder);
                res = BooleanOperation.Unite(cone2, res);
            }
            if (res != null && name != null) namedItems[name] = res;
        }
        private void SolidConeImpl(GeoPoint start, GeoPoint end, double radiusStart, double radiusEnd, string name)
        {
            Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
            GeoVector dirx = pln.ToGlobal(GeoVector2D.XAxis);
            Solid res = Make3D.MakeCone(start, dirx, end - start, radiusStart, radiusEnd);
            if (res != null && name != null) namedItems[name] = res;
        }

        private void SolidCylinderImpl(GeoPoint start, GeoPoint end, double radius, string name)
        {
            Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
            GeoVector dirx = radius * pln.ToGlobal(GeoVector2D.XAxis);
            Solid res = Make3D.MakeCylinder(start, dirx, end - start);
            if (res != null && name != null) namedItems[name] = res;
        }


        private void SolidPipeImpl(GeoPoint start, GeoPoint end, double outerRadius, double innerRadius, string name)
        {
            if (innerRadius == 0)
            {
                SolidCylinderImpl(start, end, outerRadius, name);
                return;
            }
            Solid? res = null;
            Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
            GeoVector dirx = outerRadius * pln.ToGlobal(GeoVector2D.XAxis);
            Solid cylinder1 = Make3D.MakeCylinder(start, dirx, end - start);
            dirx = innerRadius * pln.ToGlobal(GeoVector2D.XAxis);
            Solid cylinder2 = Make3D.MakeCylinder(start, dirx, end - start);
            Solid[] diff = BooleanOperation.Subtract(cylinder1, cylinder2);
            if (diff != null && diff.Length == 1) res = diff[0];
            if (res != null && name != null) namedItems[name] = res;
        }

        private void SolidSphereImpl(GeoPoint center, double radius, JsonElement points, string name)
        {
            Solid? res = Make3D.MakeSphere(center, radius);
            if (res != null && name != null) namedItems[name] = res;
        }
        private void SolidTorusImpl(GeoPoint center, GeoVector axis, double majorRadius, double minorRadius, string name)
        {
            Plane pln = new Plane(center, axis); // to use the arbitrary axis algorithm
            Solid? res = Make3D.MakeTorus(center, axis, majorRadius, minorRadius);
            if (res != null && name != null) namedItems[name] = res;
        }

        private void SurfaceNurbsImpl(int degreeU, int degreeV, JsonElement controlPoints, JsonElement uKnots, JsonElement vKnots, JsonElement weights, bool uPeriodic, bool vPeriodic, string name)
        {
            throw new NotImplementedException();
        }
        private void SurfaceParametricImpl(int degreeU, int degreeV, JsonElement approximation, string name)
        {
            int minSamplesU = RequireInteger(approximation, "minSamplesU");
            int minSamplesV = RequireInteger(approximation, "minSamplesV");

            string uParameter = RequireString(approximation, "uParameter");
            string vParameter = RequireString(approximation, "vParameter");
            string xExpr = RequireString(approximation, "xExpr");
            string yExpr = RequireString(approximation, "yExpr");
            string zExpr = RequireString(approximation, "zExpr");
            double uMin = RequireDouble(approximation, "uMin");
            double uMax = RequireDouble(approximation, "uMax");
            double vMin = RequireDouble(approximation, "vMin");
            double vMax = RequireDouble(approximation, "vMax");
            double tolerance = RequireDouble(approximation, "tolerance");
            bool uPeriodic = RequireBool(approximation, "uPeriodic");
            bool vPeriodic = RequireBool(approximation, "vPeriodic");

            GeoPoint[,] throughPoints = new GeoPoint[minSamplesU, minSamplesV];
            double du = (uMax - uMin) / (minSamplesU - 1);
            double dv = (vMax - vMin) / (minSamplesV - 1);
            for (int i = 0; i < minSamplesU; i++)
            {
                for (int j = 0; j < minSamplesV; ++j)
                {
                    using var uu = new NamedItemOverride(namedItems, uMin + i * du, uParameter);
                    using var vv = new NamedItemOverride(namedItems, vMin + j * dv, vParameter);
                    double x = (double)Evaluator.Evaluate(xExpr, namedItems.Dict);
                    double y = (double)Evaluator.Evaluate(yExpr, namedItems.Dict);
                    double z = (double)Evaluator.Evaluate(zExpr, namedItems.Dict);
                    throughPoints[i, j] = new GeoPoint(x, y, z);
                }
            }
            NurbsSurface ns = new NurbsSurface(throughPoints, degreeU, degreeV, uPeriodic, vPeriodic);
            BoundingRect ext = new BoundingRect(ns.UKnots.First(), ns.VKnots.First(), ns.UKnots.Last(), ns.VKnots.Last());
            ns.SetBounds(ext);
            namedItems[name] = ns;
        }

        private void SystemGetInfoImpl()
        {
            throw new NotImplementedException();
        }

        private void TemplateBeginImpl(string name, string label, string description, string category, JsonElement tags, JsonElement parameters, bool allowDocumentCommit)
        {
            namedItemClones.Push(new NamedItemClone(this));
            if (parameters.ValueKind != JsonValueKind.Array) { throw new JsonRpcException("E_INVALID_PARAMETER", $"'parameters must be an array'."); }
            foreach (var item in parameters.EnumerateArray())
            {
                string parName = RequireString(item, "name");
                if (!item.TryGetProperty("value", out JsonElement parValue)) { throw new JsonRpcException("E_INVALID_PARAMETER", $"No value found for {parName}."); }
                string? parLabel = GetOptionalString(item, "label");
                item.TryGetProperty("input", out var parInput);
                WorkspaceSetImpl(parName, parValue, parLabel, parInput);
            }
        }

        private object? TemplateCommitImpl(JsonElement result, string? resultKind, bool suffixInternalNames)
        {
            List<object> resultingObjects = IterateSelector<object>(result).ToList();
            namedItemClones.Pop().Dispose();
            return MakeTypedList(resultingObjects);
        }

        private object? TemplateInstantiateImpl(string template, JsonElement arguments, string transform, string? name, bool explodeResult)
        {

            if (!templates.TryGetValue(template, out var jsons)) throw new JsonRpcException("E_INVALID_PARAMETER", $"Template '{template}' not found.");
            object? res = null; // the result
            {   // use a clone of the named items dictionary during evaluation of the template
                foreach (var element in jsons)
                {
                    string methodName = RequireString(element, "method");
                    if (methodName == "template.commit")
                    {
                        if (!element.TryGetProperty("params", out var parameters)) throw new JsonRpcException("E_INTERNAL_ERROR", $"Template '{template}' has invalid commit method.");
                        JsonElement result = RequireProperty(parameters, "result");
                        var resultKind = GetOptionalString(parameters, "resultKind");
                        var suffixInternalNames = GetOptionalBool(parameters, "suffixInternalNames", true);

                        res = TemplateCommitImpl(result, resultKind, suffixInternalNames);
                        // template.commit restored the old named items
                    }
                    else
                    {
                        ProcessMethod(element, true);
                        if (methodName == "template.begin")
                        {   // here we overwrite the workspace values of the parameters
                            // template.begin createt a new copy of the named items, so we can safely overwrite values here without affecting the outside
                            if (arguments.ValueKind == JsonValueKind.Object)
                            {
                                foreach (var item in arguments.EnumerateObject())
                                {
                                    string parName = item.Name;
                                    if (item.Value.ValueKind == JsonValueKind.Number) namedItems[parName] = item.Value.GetDouble();
                                    else if (item.Value.ValueKind == JsonValueKind.String) namedItems[parName] = Evaluator.Evaluate(item.Value.GetString(), namedItems.Dict);
                                }
                            }
                        }
                    }
                }
            }
            // now apply the transformation if there is one
            // (here the named items contain the result of the template, so the transform can refer to it)
            if (!string.IsNullOrEmpty(transform))
            {
                object m = Evaluator.Evaluate(transform, namedItems.Dict);
                if (m is ModOp mop)
                {
                    if (res is List<Solid> solids)
                    {
                        foreach (Solid s in solids)
                        {
                            s.Modify(mop);
                        }
                    }
                    else if (res is IGeoObject go)
                    {
                        go.Modify(mop);
                    }
                }
                else
                {
                    throw new JsonRpcException("E_INVALID_PARAMETER", $"Transform expression did not evaluate to a transformation.");
                }
            }

            // now save in the original named items dictionary
            if (res != null && name != null) namedItems[name] = res;
            return res;
        }


        private void SolidRuledImpl(JsonElement profile1, JsonElement profile2, string synchronization, string alignment, JsonElement matchPoints1, JsonElement matchPoints2, string name)
        {
            Path? path1 = null, path2 = null;
            List<ICurve2D> curves1 = IterateSelector<ICurve2D>(profile1).ToList();
            if (curves1.Count > 0)
            {
                if (curves1.Count > 1) throw new JsonRpcException("E_INVALID_PARAMETER", $"we need exactely one closed profile in profile1.");
                Sketch? sketch = curves1[0].UserData.GetData("MCPServer.Sketch") as Sketch;
                if (sketch == null) throw new JsonRpcException("E_INTERNAL_ERROR", "Sketch not found for profile1.");
                IGeoObject go = curves1[0].MakeGeoObject(sketch.Plane);
                if (go is Path p) path1 = p;
                else if (go is ICurve crv)
                {
                    path1 = Path.FromSegments(new ICurve[] { crv }, true);
                    path1.Flatten();
                }
                else throw new JsonRpcException("E_INVALID_PARAMETER", $"profile1 must be closed.");
            }
            List<ICurve2D> curves2 = IterateSelector<ICurve2D>(profile2).ToList();
            if (curves2.Count > 0)
            {
                if (curves2.Count > 1) throw new JsonRpcException("E_INVALID_PARAMETER", $"we need exactely one closed profile in profile2.");
                Sketch? sketch = curves2[0].UserData.GetData("MCPServer.Sketch") as Sketch;
                if (sketch == null) throw new JsonRpcException("E_INTERNAL_ERROR", "Sketch not found for profile2.");
                IGeoObject go = curves2[0].MakeGeoObject(sketch.Plane);
                if (go is Path p) path2 = p;
                else if (go is ICurve crv)
                {
                    path2 = Path.FromSegments(new ICurve[] { crv }, true);
                }
                else throw new JsonRpcException("E_INVALID_PARAMETER", $"profile2 must be closed.");
            }
            if (path1 != null && path2 != null)
            {
                Solid sld = Make3D.MakeRuledSolid(path1, path2, null);
                if (sld != null)
                {
                    if (name != null) namedItems[name] = sld;
                }
                else throw new JsonRpcException("E_OPERATION_FAILED", "Could not create ruled solid.");
            }
        }

        List<CompoundShape> GetProfiles(JsonElement selector)
        {
            List<CompoundShape> lcs = IterateSelector<CompoundShape>(selector).ToList();
            List<ICurve2D> lc2 = IterateSelector<ICurve2D>(selector).ToList();
            List<ICurve2D> remainingCurves = [];
            for (int i = 0; i < lc2.Count; i++)
            {
                if (lc2[i].IsClosed || Precision.IsEqual(lc2[i].StartPoint, lc2[i].EndPoint))
                {
                    Border bdr = new Border(lc2[i]);
                    CompoundShape cs = new CompoundShape(new SimpleShape(bdr));
                    lcs.Add(cs);
                    cs.UserData.Add("MCPServer.Sketch", lc2[i].UserData["MCPServer.Sketch"]);
                }
                else
                {
                    remainingCurves.Add(lc2[i]);
                }
            }
            if (remainingCurves.Count > 0)
            {
                Reduce2D r2d = new Reduce2D();
                r2d.Add(remainingCurves.ToArray());
                r2d.OutputMode = Reduce2D.Mode.Paths;
                ICurve2D[] r = r2d.Reduced;
                for (int j = 0; j < r.Length; j++)
                {
                    if (r[j].IsClosed || Precision.IsEqual(r[j].StartPoint, r[j].EndPoint))
                    {
                        Border bdr = new Border(r[j]);
                        CompoundShape cs = new CompoundShape(new SimpleShape(bdr));
                        lcs.Add(cs);
                        cs.UserData.Add("MCPServer.Sketch", remainingCurves[0].UserData["MCPServer.Sketch"]);
                    }
                }
            }
            return lcs;
        }
        List<ICurve> GetSketchCurves(JsonElement selector)
        {
            List<CompoundShape> lcs = IterateSelector<CompoundShape>(selector).ToList();
            List<ICurve2D> lc2 = IterateSelector<ICurve2D>(selector).ToList();
            List<ICurve> res = [];
            if (lc2.Count > 0)
            {
                Sketch? sketch = lc2[0].UserData["MCPServer.Sketch"] as Sketch;
                if (sketch == null) throw new JsonRpcException("E_INTERNAL_ERROR", "No sketch assoziated with curve.");

                Reduce2D r2d = new Reduce2D();
                r2d.Add(lc2.ToArray());
                r2d.OutputMode = Reduce2D.Mode.Paths;
                foreach (ICurve2D curve2D in r2d.Reduced)
                {
                    ICurve? toAdd = curve2D.MakeGeoObject(sketch.Plane) as ICurve;
                    if (toAdd != null) res.Add(toAdd);
                }
            }
            if (lcs.Count > 0)
            {
                Sketch? sketch = lcs[0].UserData["MCPServer.Sketch"] as Sketch;
                if (sketch == null) throw new JsonRpcException("E_INTERNAL_ERROR", "No sketch assoziated with profile.");
                foreach (CompoundShape cs in lcs)
                {
                    res.AddRange(cs.MakePaths(sketch.Plane));
                }
            }
            return res;
        }
        private void SolidSweepImpl(JsonElement profile, JsonElement path, string? orientation, string? name, JsonElement capture)
        {
            List<CompoundShape> profiles = GetProfiles(profile);
            List<ICurve> paths = GetSketchCurves(path);
            if (profiles.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be exactely one profile.");
            if (paths.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be exactely one path.");
            Sketch? sketch = profiles[0].UserData["MCPServer.Sketch"] as Sketch;
            if (sketch == null) throw new JsonRpcException("E_INTERNAL_ERROR", "No sketch assoziated with profile.");

            Face toSweep = Face.MakeFace(new PlaneSurface(sketch.Plane), profiles[0].SimpleShapes[0]); // the profile should not consist of multiple SimpleShapes
            if (!(paths[0] is Path)) paths[0] = Path.FromSegments(paths)[0]; // there must be at least one!
            Path? p = paths[0] as Path;
            if (p != null)
            {
                IGeoObject sweptSolid = Make3D.MakePipe(toSweep, p, null);
                if (sweptSolid is Solid sld)
                {
                    if (name != null) namedItems[name] = sld;
                    if (capture.ValueKind != JsonValueKind.Undefined)
                    {
                        string? startEdgesName = GetOptionalString(capture, "startEdges");
                        string? endEdgesName = GetOptionalString(capture, "endEdges");
                        string? startFaceName = GetOptionalString(capture, "startFace");
                        string? endFaceName = GetOptionalString(capture, "endFace");

                        Face endFace = (toSweep.Clone() as Face)!;
                        endFace.Modify(ModOp.Fit(p.StartPoint, [p.StartDirection], p.EndPoint, [p.EndDirection]));
                        Face? startingFace = sld.Shell.FindSimilarFace(toSweep);
                        Face? endingFace = sld.Shell.FindSimilarFace(endFace);
                        if (startingFace != null)
                        {
                            if (startFaceName != null) namedItems[startFaceName] = startingFace; // there should only be one
                            if (startEdgesName != null) namedItems[startEdgesName] = startingFace.AllEdges.ToList();
                        }
                        if (endingFace != null)
                        {
                            if (endFaceName != null) namedItems[endFaceName] = endingFace; // there should only be one
                            if (endEdgesName != null) namedItems[endEdgesName] = endingFace.AllEdges.ToList();
                        }
                    }
                }
            }
        }
        private void SolidRotateImpl(JsonElement profile, Axis axis, double angle, string? name, JsonElement capture)
        {
            List<CompoundShape> profiles = GetProfiles(profile);
            List<Solid> res = [];
            for (int i = 0; i < profiles.Count; i++)
            {
                Sketch? sketch = profiles[i].UserData["MCPServer.Sketch"] as Sketch;
                if (sketch == null) throw new JsonRpcException("E_OPERATION_FAILED", "No suitable sketch found for profile");
                for (int j = 0; j < profiles[i].SimpleShapes.Length; j++)
                {
                    Face toRotate = Face.MakeFace(new PlaneSurface(sketch.Plane), profiles[i].SimpleShapes[j]);
                    IGeoObject go = Make3D.Rotate(toRotate, axis, SweepAngle.Deg(angle), 0, null);
                    if (go is Solid sld) res.Add(sld);
                }
            }
            if (name != null) namedItems[name] = res;
        }
        private void PatternCircularSketchImpl(Sketch sketch, JsonElement entities, GeoPoint2D center, int count, double angle, bool merge, string name, bool nameWithSuffix)
        {
            // only implemented for closed shapes for now, which we convert to CompoundShape for easier boolean operations. We can add support for open curves later if needed.
            List<CompoundShape> inputshapes = new List<CompoundShape>();
            if (entities.ValueKind == JsonValueKind.Array)
            {
                foreach (var inp in entities.EnumerateArray())
                {
                    object obj = ResolveObjectRef(inp);
                    if (obj != null)
                    {
                        foreach (object oo in IterateListOrSingleObject(obj))
                        {
                            if (oo is ICurve2D c2d && c2d.IsClosed)
                            {
                                Border bdr = new Border(c2d);
                                inputshapes.Add(new CompoundShape(new SimpleShape(new Border(c2d))));
                            }
                            else if (oo is CompoundShape cs)
                            {
                                cs.UserData.Add("MCPServer.Sketch", sketch);
                                inputshapes.Add(cs);
                            }
                            else throw new JsonRpcException(-32602, "All inputs must be closed shapes.");
                        }
                    }
                    else throw new JsonRpcException(-32602, "All inputs must be sketch shapes.");
                }
            }
            List<CompoundShape> resultshapes = new List<CompoundShape>();
            if (double.IsNaN(angle)) angle = 360;
            SweepAngle angleStep = SweepAngle.Deg(angle / count);
            ModOp2D rot = ModOp2D.Rotate(center, angleStep);
            resultshapes.AddRange(inputshapes); // first not rotatet
            for (int i = 1; i < count; i++)
            {
                List<CompoundShape> newshapes = new List<CompoundShape>();
                foreach (var s in inputshapes)
                {
                    CompoundShape copy = s.GetModified(rot);
                    newshapes.Add(copy);
                }
                inputshapes = newshapes; // already rotated for next iteration
                resultshapes.AddRange(newshapes);
            }
            if (merge)
            {
                CompoundShape? merged = null;
                foreach (var s in resultshapes)
                {
                    if (merged == null) merged = s;
                    else merged = CompoundShape.Union(merged, s);
                }
                if (name != null && merged != null) StoreNamed(name, merged);
                if (merged != null) sketch.Add(merged);
            }
            else
            {
                foreach (var s in resultshapes) sketch.Add(s);
                if (name != null) StoreNamed(name, resultshapes); // a List of CompoundShape
            }
        }

        private IEnumerable<object> IterateListOrSingleObject(object obj)
        {
            if (obj is IEnumerable<object> list)
            {
                foreach (var item in list) yield return item;
            }
            else
            {
                yield return obj;
            }
        }

        private void SketchRoundVerticesImpl(Sketch? sketch, JsonElement entity, double radius, JsonElement nearPoints, JsonElement indices, double tolerance, string name)
        {
            if (nearPoints.ValueKind == JsonValueKind.Undefined || indices.ValueKind == JsonValueKind.Undefined)
            {
                throw new NotImplementedException("Vertex selection for sketch.round_vertices not implemented.");
            }
            List<object> entities = IterateObjectRefs(entity).ToList();
            if (entities.Count == 0 && sketch != null)
            {
                entities.Add(sketch.Curves);
                entities.Add(sketch.Shapes);
            }
            if (entities.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "No entities provided for rounding edges.");
            for (int i = 0; i < entities.Count; i++)
            {
                Path2D? p2d = null;
                CompoundShape? cs = null;
                string? currentName = null;
                if (entities[i] is Path2D ep2d)
                {
                    if (sketch == null) sketch = ep2d.UserData["MCPServer.Sketch"] as Sketch;
                    currentName = FindName(ep2d);
                    p2d = ep2d;
                }
                else if (entities[i] is ICurve2D c2d)
                {
                    if (sketch == null) sketch = c2d.UserData["MCPServer.Sketch"] as Sketch;
                    currentName = FindName(c2d);
                    p2d = new Path2D([c2d]);
                }
                else if (entities[i] is CompoundShape ecs)
                {
                    if (sketch == null) sketch = ecs.UserData["MCPServer.Sketch"] as Sketch;
                    currentName = FindName(ecs);
                    throw new NotImplementedException("round vertices of sketch shape");
                }
                if (p2d != null)
                {
                    Path2D rounded = p2d.RoundVertices(radius);
                    if (rounded != null)
                    {
                        rounded.UserData.Add("MCPServer.Sketch", sketch);
                        if (string.IsNullOrEmpty(name) && currentName != null) namedItems[currentName] = rounded;
                        else if (name != null) namedItems[name] = rounded;
                    }
                }
                if (cs != null)
                {
                    CompoundShape rounded = cs.RoundVertices(radius);
                    if (rounded != null)
                    {
                        rounded.UserData.Add("MCPServer.Sketch", sketch);
                        if (string.IsNullOrEmpty(name) && currentName != null) namedItems[currentName] = rounded;
                        else if (name != null) namedItems[name] = rounded;
                    }
                }
            }

        }
        private Solid UniteWithMany(Solid a, List<Solid> b, out List<Solid> unused)
        {
            Solid accumulate = a;
            HashSet<Solid> bb = [.. b];
            while (bb.Count > 0)
            {
                bool united = false;
                foreach (Solid sld in bb.Clone())
                {
                    Solid tmp = NewBooleanOperation.Unite(sld, accumulate);
                    if (tmp != null)
                    {
                        accumulate = tmp;
                        bb.Remove(sld);
                        united = true;
                    }
                }
                if (!united) break; // there is nothing we could unite with, so we are done
            }
            unused = bb.ToList();
            return accumulate;
        }

        private List<Solid> SubtractMany(List<Solid> toSubtractFrom, List<Solid> subtractItems)
        {
            List<Solid> fragments = new List<Solid>();
            fragments.AddRange(toSubtractFrom);
            foreach (Solid sld in subtractItems)
            {
                List<Solid> newfragments = new List<Solid>();
                for (int i = 0; i < fragments.Count; i++)
                {
                    Solid[] res = NewBooleanOperation.Subtract(fragments[i], sld);
                    if (res != null && res.Length > 0)
                    {
                        newfragments.AddRange(res);
                    }
                    else
                    {
                        newfragments.Add(fragments[i]);
                    }
                }
                fragments = newfragments;
            }
            return fragments;
        }
        Solid[] IntersectMany(Solid solid, List<Solid> other)
        {
            List<Solid> fragments = new List<Solid>();
            fragments.Add(solid);
            foreach (Solid sld in other)
            {
                List<Solid> newfragments = new List<Solid>();
                for (int i = 0; i < fragments.Count; i++)
                {
                    Solid[] res = NewBooleanOperation.Intersect(fragments[i], sld);
                    if (res != null && res.Length > 0)
                    {
                        newfragments.AddRange(res);
                    }
                    else
                    {
                        newfragments.Add(fragments[i]);
                    }
                }
                fragments = newfragments;
            }
            return fragments.ToArray();
        }
        private void SolidBooleanImpl(string op, JsonElement a, JsonElement b, string? name, bool rebind, JsonElement rebindTargets)
        {
            List<Solid> slda = IterateSelector<Solid>(a).ToList();
            List<Solid> sldb = IterateSelector<Solid>(b).ToList();
            if (slda.Count == 0 || sldb.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "Boolean operations require at least one solid 'a' and at least one other solid 'b' to operate with.");
            if (name == null && slda[0] is IGeoObject go) name = go.UserData["CADablity.MCP.Name"] as string;
            object? res = null;
            Solid s1 = slda[0];
            List<Solid> s2 = [.. sldb, .. slda.Skip(1)];
            // if there are more than one solid in a, we also treat them as solids to operate with, so we add them to the list of b solids. This is a bit unintuitive but it allows for more complex operations without needing to call boolean multiple times. For example, if you want to unite 3 solids, you can just put them all in a and leave b empty.
            var sw = Stopwatch.StartNew();
            switch (op.ToLower())
            {
                case "union":
                case "unite":
                    {
                        Solid sld = UniteWithMany(s1, s2, out List<Solid> unused);
                        if (sld != null && unused.Count > 0)
                        {
                            // we were not able to unite with all solids, so we return the result as a list of solids (the united one and the ones we could not unite with)
                            List<Solid> resList = [sld];
                            while (unused.Count > 1)
                            {
                                s1 = unused[0];
                                s2 = [.. unused.Skip(1)];
                                sld = UniteWithMany(s1, s2, out unused);
                                resList.Add(sld);
                            }
                            resList.AddRange(unused);
                            res = resList;
                        }
                        else res = sld;
                    }
                    break;
                case "difference":
                case "subtract":
                    {
                        res = SubtractMany(slda, sldb);
                    }
                    break;
                case "intersect":
                    {
                        res = new List<Solid>(IntersectMany(s1, s2));
                    }
                    break;
                default: throw new JsonRpcException("E_INVALID_PARAMS", $"'solid.boolean' unknown operator {op}");
            }
            sw.Stop();
            if (res != null && name != null) namedItems[name] = res;
            if (rebind)
            {
                if (res is Solid sres)
                {
                    Rebind(slda, sres);
                    Rebind(sldb, sres);
                }
                else if (res is List<Solid> lres)
                {
                    Rebind(slda, lres);
                    Rebind(sldb, lres);
                }
            }
        }

        private void PatternCircularSolidsImpl(JsonElement objects, Axis axis, int count, double angle, bool copy, string name, bool suffix)
        {

            List<Solid> list = IterateObjectRefs<Solid>(objects).ToList(); // all objects assiziated via userdat by name
            double rotationAngle = double.IsNaN(angle) ? 360 : angle;
            if (rotationAngle <= Precision.eps) throw new JsonRpcException("E_INVALID_PARAMS", "Angle must be greater than 0.");
            double stepAngle = rotationAngle / count;
            List<Solid> current = new List<Solid>();
            foreach (Solid s in list) current.Add(s.Clone() as Solid);
            List<Solid> total = new List<Solid>(current);
            ModOp rot = ModOp.Rotate(axis.Location, axis.Direction, SweepAngle.Deg(stepAngle));
            for (int i = 1; i < count; i++)
            {
                List<Solid> next = new List<Solid>();
                foreach (Solid s in current)
                {
                    Solid clone = s.Clone() as Solid;
                    if (clone != null)
                    {
                        clone.Modify(rot);
                        next.Add(clone);
                        if (suffix)
                        {
                            if (name != null) AddNamed($"{name}_{i}", clone);
                            else
                            {
                                string? cname = clone.UserData["CADablity.MCP.Name"] as string;
                                if (cname != null) AddNamed($"{name}_{i}", clone);
                            }
                        }
                    }
                }
                total.AddRange(next);
                current = next;
            }
            if (name != null) namedItems[name] = total;
        }

        private void PatternGridSketchImpl(Sketch sketch, JsonElement entities, int countX, int countY, JsonElement stepX, JsonElement stepY, bool merge, string name, bool nameWithSuffix)
        {
            throw new NotImplementedException();
        }

        private void PatternGridSolidsImpl(JsonElement objects, int countX, int countY, int countXNegative, int countYNegative, GeoVector stepX, GeoVector stepY, bool copy, string? name, bool suffix)
        {
            GeoVector sx = stepX;
            if (sx.Length <= Precision.eps) throw new JsonRpcException("E_INVALID_PARAMS", "stepX must be a non-zero vector.");
            GeoVector sy = stepY.IsValid() ? stepY : GeoVector.NullVector;
            if (countXNegative > 0) countXNegative = -countXNegative; // make sure countXNegative is negative or zero (usually provided positiv)
            if (countYNegative > 0) countYNegative = -countYNegative; // make sure countYNegative is negative or zero (usually provided positiv)
            if (countY - countYNegative > 0 && sy.Length <= Precision.eps) throw new JsonRpcException("E_INVALID_PARAMS", "stepY must be a non-zero vector when y counts are provided");
            List<Solid> list = IterateObjectRefs<Solid>(objects).ToList(); // all objects to pattern
            List<Solid> total = new List<Solid>();
            for (int ix = countXNegative; ix <= countX; ix++)
            {
                for (int iy = countYNegative; iy <= countY; iy++)
                {
                    GeoVector moveVec = ix * sx;
                    if (iy != 0) moveVec += iy * sy;
                    ModOp move = ModOp.Translate(moveVec);
                    List<Solid> subList = [];
                    foreach (Solid s in list)
                    {
                        Solid? clone = s.Clone() as Solid;
                        if (clone != null)
                        {
                            clone.Modify(move);
                            subList.Add(clone);
                            if (suffix)
                            {
                                string? sn = FindName(s);
                                if (sn != null) AddNamed($"{sn}_{ix}_{iy}", clone);
                                else if (name != null) AddNamed($"{name}_{ix}_{iy}", clone);
                            }
                        }
                    }
                    total.AddRange(subList);
                }
            }
            if (name != null) AddNamed(name, total);
        }

        private void PatternByFormulaSolidsImpl(JsonElement solids, string template, JsonElement variables, JsonElement formulas, string? condition, JsonElement arguments, string transform, bool includeSource, bool copy, string? name, bool suffix, string? indexName, bool skipInvalidInstances)
        {
            List<Solid> solidsToInsert = [];
            if (solids.ValueKind == JsonValueKind.Object) solidsToInsert = IterateSelector<Solid>(solids).ToList();
            List<(string name, double start, double step, int count)> loopVariables = [];
            if (variables.ValueKind != JsonValueKind.Array) throw new JsonRpcException("E_INVALID_PARAMS", "'variables' must be an array.");
            foreach (var variable in variables.EnumerateArray())
            {
                if (!(variable.TryGetProperty("name", out var nameEl) && nameEl.ValueKind == JsonValueKind.String))
                    throw new JsonRpcException("E_INVALID_PARAMS", "'variable' must have a 'name' proerty.");
                string varname = nameEl.GetString()!;
                if (!(variable.TryGetProperty("start", out var startEl)))
                    throw new JsonRpcException("E_INVALID_PARAMS", $"'variable' {name} must have a 'start' proerty.");
                double start = double.NaN;
                if (startEl.ValueKind == JsonValueKind.Number) start = startEl.GetDouble();
                if (startEl.ValueKind == JsonValueKind.String) start = (double)Evaluator.Evaluate(startEl.GetString(), namedItems.Dict);
                if (double.IsNaN(start)) throw new JsonRpcException("E_INVALID_PARAMS", $"could not evaluate start value of variable {name}.");
                if (!(variable.TryGetProperty("step", out var stepEl)))
                    throw new JsonRpcException("E_INVALID_PARAMS", $"'variable' {name} must have a 'step' proerty.");
                double step = double.NaN;
                if (stepEl.ValueKind == JsonValueKind.Number) step = stepEl.GetDouble();
                if (stepEl.ValueKind == JsonValueKind.String) step = (double)Evaluator.Evaluate(stepEl.GetString(), namedItems.Dict);
                if (double.IsNaN(step)) throw new JsonRpcException("E_INVALID_PARAMS", $"could not evaluate step value of variable {name}.");
                if (!(variable.TryGetProperty("count", out var countEl)))
                    throw new JsonRpcException("E_INVALID_PARAMS", $"'variable' {name} must have a 'count' proerty.");
                int count = 0;
                if (countEl.ValueKind == JsonValueKind.Number) count = countEl.GetInt32();
                if (countEl.ValueKind == JsonValueKind.String)
                {
                    object eva = Evaluator.Evaluate(countEl.GetString(), namedItems.Dict);
                    if (eva is double d) count = (int)d;
                    if (eva is int i) count = i;
                }
                if (count <= 0) throw new JsonRpcException("E_INVALID_PARAMS", $"could not evaluate count value of variable {name}.");
                loopVariables.Add((varname, start, step, count));
            }
            List<Solid> result = [];
            IterateLoops(loopVariables, () =>
            {
                if (formulas.ValueKind == JsonValueKind.Object)
                {
                    foreach (var item in formulas.EnumerateObject())
                    {
                        if (item.Value.ValueKind == JsonValueKind.String)
                        {
                            namedItems[item.Name] = Evaluator.Evaluate(item.Value.GetString(), namedItems.Dict);
                        }
                    }
                }
                if (condition != null)
                {
                    if (Evaluator.Evaluate(condition, namedItems.Dict) is bool ok)
                    {
                        if (!ok) return;
                    }
                }
                ModOp transforModOp = ModOp.Identity;
                if (Evaluator.Evaluate(transform, namedItems.Dict) is ModOp t) transforModOp = t;
                if (template != null)
                {
                    object? templInst = TemplateInstantiateImpl(template, arguments, transform, null, false);
                    if (templInst is Solid sld)
                    {
                        sld.Modify(transforModOp);
                        result.Add(sld);
                    }
                    else if (templInst is List<Solid> lsld)
                    {
                        foreach (Solid solid in lsld) solid.Modify(transforModOp);
                        result.AddRange(lsld);

                    }
                }
                else if (solidsToInsert.Count > 0)
                {
                    for (int i = 0; i < solidsToInsert.Count; i++)
                    {
                        var clone = solidsToInsert[i].Clone();
                        clone.Modify(transforModOp);
                        result.Add((clone as Solid)!);
                    }
                }
            });
            if (name != null) namedItems[name] = result;
            if (suffix)
            {
                string? n = indexName != null ? indexName : name;
                if (n != null)
                {
                    for (int i = 0; i < result.Count; i++)
                    {
                        namedItems[n + "_" + i.ToString()] = result[i];
                    }
                }
            }
        }

        void IterateLoops(List<(string name, double start, double step, int count)> loopVariables, Action body)
        {
            int n = loopVariables.Count;
            int[] indices = new int[n];

            while (true)
            {
                // Aktuelle Werte setzen
                for (int i = 0; i < n; i++)
                {
                    var (name, start, step, count) = loopVariables[i];
                    double val = start + indices[i] * step;
                    namedItems[name] = val;
                }

                // Das eigentliche "Innere" der Schleife
                body();

                // "Zähler erhöhen" (wie bei verschachtelten Schleifen)
                int k = n - 1;
                while (k >= 0)
                {
                    indices[k]++;
                    if (indices[k] < loopVariables[k].count)
                        break;

                    indices[k] = 0;
                    k--;
                }

                // Wenn wir über die erste Schleife hinaus sind: fertig
                if (k < 0)
                    break;
            }
        }
        private void SketchOffsetImpl(Sketch sketch, JsonElement sketchGeometry, double distance, string joinType, double miterLimit, bool makeRegion, string capType, string name)
        {
            object? pathOrShape = null;
            if (sketchGeometry.ValueKind == JsonValueKind.Object)
            {
                List<CompoundShape> inputshapes = IterateSelector<CompoundShape>(sketchGeometry).ToList();
                List<ICurve2D> inputcurves = IterateSelector<ICurve2D>(sketchGeometry).ToList();
                if (inputshapes.Count > 0)
                {
                    pathOrShape = inputshapes[0];
                }
                else if (inputcurves.Count > 0) pathOrShape = inputcurves[0];
            }
            else
            {   // from sketch
                if (sketch.Shapes.Count > 0) { pathOrShape = sketch.Shapes[0]; }
                else if (sketch.Curves.Count > 0)
                {
                    if (sketch.Curves.Count == 1) pathOrShape = sketch.Curves[0];
                }
                else if (sketch.Curves.Count > 1)
                {
                    // combine all curves to a path?
                }
            }
            if (pathOrShape == null) throw new JsonRpcException("E_INVALID_PARAMS", "No input found to offset.");
            if (double.IsNaN(miterLimit)) miterLimit = Math.PI;
            object? result = null;
            if (pathOrShape is ICurve2D c2d)
            {
                result = c2d.Parallel(distance, true, Precision.eps, miterLimit);
                if (makeRegion && !c2d.IsClosed && result is ICurve2D rc2d)
                {
                    rc2d.Reverse();
                    Border bdr = new Border([c2d, new Line2D(c2d.EndPoint, rc2d.StartPoint), rc2d, new Line2D(rc2d.EndPoint, c2d.StartPoint)], true, true);
                    result = new CompoundShape(new SimpleShape(bdr));
                }
            }
            else if (pathOrShape is CompoundShape cs)
            {
                if (distance > 0) result = cs.Expand(distance);
                else result = cs.Shrink(distance);
            }
            if (result == null) throw new JsonRpcException("E_OPERATION_FAILED", "Failed to calculate offset.");
            if (name != null) namedItems[name] = result;
            if (result != null)
            {
                if (result is ICurve2D ic2d) sketch.Add(ic2d);
                if (result is CompoundShape cs) sketch.Add(cs);
            }
        }
        private void SketchGetVerticesImpl(Sketch sketch, JsonElement sketchGeometry, string name, bool suffix, bool includeEndpoints, bool unique, double tolerance)
        {
            throw new NotImplementedException();
        }
        private void SketchPerpendicularThroughImpl(Sketch sketch, JsonElement curve, GeoPoint2D point, double length, string name)
        {
            List<ICurve2D> curves = IterateSelector<ICurve2D>(curve).ToList(); // should only be one
            if (curves.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be exactly one curve in 'curve'.");
            GeoPoint2D[] ftpts = curves[0].PerpendicularFoot(point);
            if (ftpts.Length == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Failed to find foot point.");
            GeoPoint2D ftpt = ftpts.MinBy(f => f | point); // in case there are multiple foot points, we take the one closest to the given point);
            GeoVector2D dir = ftpt - point;
            if (dir.IsNullVector()) dir = curves[0].DirectionAt(curves[0].PositionOf(ftpt)).Normalized.ToLeft();
            Line2D nl = new Line2D(ftpt, ftpt + length * dir.Normalized);
            sketch.Add(nl);
            if (name != null) namedItems[name] = nl;
        }
        private void SketchFootPointOnCurveImpl(Sketch sketch, GeoPoint2D point, JsonElement curve, string mode, bool clamp, string name, JsonElement captured)
        {
            List<ICurve2D> c = IterateSelector<ICurve2D>(curve).ToList(); // should only be one
            if (c.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be exactly one curve in 'curve'.");
            GeoPoint2D[] ftpts = c[0].PerpendicularFoot(point);
            if (ftpts.Length == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Failed to find foot point.");
            GeoPoint2D ftpt = ftpts.MinBy(f => f | point); // in case there are multiple foot points, we take the one closest to the given point);
            if (name != null) namedItems[name] = ftpt;
        }
        private void SketchIntersectionsImpl(Sketch sketch, JsonElement a, JsonElement b, string mode, double tolerance, string name, bool suffix)
        {
            List<ICurve2D> ca = IterateSelector<ICurve2D>(a).ToList(); // should only be one
            List<ICurve2D> cb = IterateSelector<ICurve2D>(b).ToList(); // should only be one
            if (ca.Count != 1 || cb.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be exactly one curve in 'a' and one curve in 'b'.");
            GeoPoint2DWithParameter[] ips = ca[0].Intersect(cb[0]);
            if (ips == null || ips.Length == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Failed to find intersection points.");
            List<GeoPoint2D> points = ips.Select(ip => ip.p).ToList();
            if (name != null)
            {
                if (points.Count == 1)
                {
                    namedItems[name] = points[0];
                }
                else
                {
                    if (suffix)
                    {
                        for (int i = 0; i < points.Count; i++)
                        {
                            AddNamed($"{name}_{i}", points[i]);
                        }
                    }
                    namedItems[name] = points;
                }
            }
        }


        private void SketchAngleBisectorImpl(Sketch sketch, JsonElement a, JsonElement b, string which, GeoPoint2D at, double length, string name)
        {
            List<Line2D> linea = IterateSelector<Line2D>(a).ToList(); // should only be one
            List<Line2D> lineb = IterateSelector<Line2D>(b).ToList(); // should only be one
            if (linea.Count != 1 || lineb.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be one line in a and one line in b.");
            if (!Geometry.IntersectLL(linea[0].StartPoint, linea[0].StartDirection, lineb[0].StartPoint, lineb[0].StartDirection, out GeoPoint2D intersectionPoint)) throw new JsonRpcException("E_INVALID_PARAMS", "The lines don't intersect.");

            GeoVector2D dir1 = linea[0].StartDirection.Normalized + lineb[0].StartDirection.Normalized;
            GeoVector2D dir2 = linea[0].StartDirection.Normalized - lineb[0].StartDirection.Normalized;
            GeoVector2D dir = GeoVector2D.NullVector;
            if (which != null)
            {
                switch (which)
                {
                    case "acute":
                    case "inner":
                        dir = dir1.Length < dir2.Length ? dir2.Normalized : dir1.Normalized;
                        break;
                    case "obtuse":
                    case "outer":
                        dir = dir1.Length > dir2.Length ? dir2.Normalized : dir1.Normalized;
                        break;
                    case "ccw":
                        dir = GeoVector2D.Orientation(dir, dir2) > 0 ? dir1.Normalized : dir2.Normalized;
                        break;
                    case "cw":
                        dir = GeoVector2D.Orientation(dir, dir2) < 0 ? dir1.Normalized : dir2.Normalized;
                        break;
                }
            }

            Line2D res = new Line2D(intersectionPoint, intersectionPoint + length * dir);
            sketch.Add(res);
            if (name != null) namedItems[name] = res;

        }


        private void FeatureHoleImpl(JsonElement solid, JsonElement face, JsonElement centerOnFace, GeoPoint center, double diameter, bool through, double depth, string? name, JsonElement capture, bool rebind, JsonElement rebindTargets)
        {
            List<Solid> solids = IterateSelector<Solid>(solid).ToList(); // should only be one
            List<Face> faces = IterateSelector<Face>(face).ToList(); // should only be one
            if (solids.Count != 1 && faces.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be one solid and one face in 'feature.hole'.");
            Face onFace = faces[0];
            Solid onSolid = solids[0];
            GeoPoint holeCenter = center;
            GeoPoint2D uv = onFace.PositionOf(holeCenter);
            holeCenter = onFace.Surface.PointAt(uv);
            GeoVector holeDir = -onFace.Surface.GetNormal(uv).Normalized;
            double holeDepth = 0.0;
            if (through)
            {
                GeoPoint[] ips = onSolid.Shells[0].GetLineIntersection(holeCenter, holeDir);
                // now the length of the cylinder to remove may be longer than the first intersection
                // the best result would be to intersect a long cylinder (bbox) with the solid and find the part
                // which is closest to holeCenter.
                double minLength = double.MaxValue;
                foreach (GeoPoint ip in ips)
                {
                    double par = Geometry.LinePar(holeCenter, holeDir, ip);
                    if (par > Precision.eps && par < minLength) minLength = par;
                }
                if (minLength != double.MaxValue) holeDepth = minLength;
                else holeDepth = onSolid.GetBoundingCube().DiagonalLength;
            }
            else
            {
                holeDepth = depth;
            }
            double radius = diameter / 2;
            Plane pln = new Plane(holeCenter, holeDir); // arbitrary axis
            Solid cyl = Make3D.MakeCylinder(holeCenter, radius * pln.DirectionX, holeDepth * holeDir);
            if (cyl != null)
            {
                string? resName = name;
                if (resName == null) resName = FindName(onSolid);
                Solid[] res = NewBooleanOperation.Subtract(onSolid, cyl);
                if (res != null && res.Length > 0 && resName != null)
                {
                    if (res.Length == 1)
                    {
                        namedItems[resName] = res[0];
                    }
                    else
                    {
                        namedItems[resName] = new List<Solid>(res);
                    }
                    if (capture.ValueKind != JsonValueKind.Undefined)
                    {
                        string? entryEdgesName = null, exitEdgesName = null, wallFacesName = null;
                        if (capture.TryGetProperty("entryEdges", out var entryEl)) entryEdgesName = entryEl.GetString();
                        if (capture.TryGetProperty("exitEdges", out var exitEl)) exitEdgesName = exitEl.GetString();
                        if (capture.TryGetProperty("wallFaces", out var wallEl)) wallFacesName = wallEl.GetString();

                        List<Face> cylindricalFaces = [];
                        foreach (Face fc in cyl.Shells[0].Faces)
                        {
                            if (fc.Surface is ICylinder) cylindricalFaces.Add(fc);
                        }
                        if (entryEdgesName != null || exitEdgesName != null)
                        {
                            List<Edge> entryEdges = [];
                            List<Edge> exitEdges = [];
                            foreach (Edge edge in res[0].Edges)
                            {
                                foreach (Face fc in cylindricalFaces)
                                {
                                    if (Precision.IsNull(fc.Distance(edge.Vertex1.Position))
                                        && Precision.IsNull(fc.Distance(edge.Vertex2.Position))
                                        && fc.Surface.IsCurveOnSurface(edge.Curve3D))
                                    {   // either entry or exit
                                        if (Precision.IsNull(onFace.Distance(edge.Vertex1.Position))
                                        && Precision.IsNull(onFace.Distance(edge.Vertex2.Position))
                                        && onFace.Surface.IsCurveOnSurface(edge.Curve3D))
                                            entryEdges.Add(edge);
                                        else exitEdges.Add(edge);
                                        break;
                                    }
                                }
                            }
                            if (entryEdgesName != null) namedItems[entryEdgesName] = entryEdges;
                            if (exitEdgesName != null) namedItems[exitEdgesName] = exitEdges;
                        }
                        if (wallFacesName != null)
                        {
                            List<Face> wallFaces = [];
                            foreach (Face fc in cylindricalFaces)
                            {
                                foreach (Face fc1 in res[0].Shells[0].Faces)
                                {
                                    if (fc.SameSurface(fc1)) wallFaces.Add(fc1);
                                }
                            }
                            namedItems[wallFacesName] = wallFaces;
                        }
                    }
                    if (rebind) Rebind(onSolid, res);
                }
            }
        }
        private void FeatureSplitImpl(JsonElement solid, JsonElement splitBy, string nameInner, string nameOuter, bool rebind, JsonElement rebindTargets)
        {
            List<Solid> solids = IterateSelector<Solid>(solid).ToList(); // should only be one
            Plane pln = Plane.Invalid;
            Shell? shell = null;
            if (splitBy.TryGetProperty("standard", out var _) || splitBy.TryGetProperty("origin", out var _))
            {
                pln = RequirePlane(splitBy);
            }
            else
            {
                List<ISurface> surfaces = IterateSelector<ISurface>(splitBy).ToList(); // should only be one
                                                                                       // make a shell from this surface
                List<Face> faces = [];
                for (int i = 0; i < surfaces.Count; i++)
                {
                    BoundingRect ext = surfaces[i].GetBounds();
                    // TODO: both u and v are periodic!
                    if (surfaces[i].IsUPeriodic && ext.Width > surfaces[i].UPeriod * 0.9)
                    {
                        BoundingRect extl = new BoundingRect(ext);
                        extl.Right = ext.Left + ext.Width / 2;
                        BoundingRect extr = new BoundingRect(ext);
                        extr.Left = ext.Left + ext.Width / 2;
                        faces.Add(Face.MakeFace(surfaces[i].Clone(), extl));
                        faces.Add(Face.MakeFace(surfaces[i].Clone(), extr));
                    }
                    else if (surfaces[i].IsVPeriodic && ext.Height > surfaces[i].VPeriod * 0.9)
                    {
                        BoundingRect extb = new BoundingRect(ext);
                        extb.Top = ext.Bottom + ext.Height / 2;
                        BoundingRect extt = new BoundingRect(ext);
                        extt.Bottom = ext.Bottom + ext.Height / 2;
                        faces.Add(Face.MakeFace(surfaces[i].Clone(), extb));
                        faces.Add(Face.MakeFace(surfaces[i].Clone(), extt));
                    }
                    else
                    {
                        faces.Add(Face.MakeFace(surfaces[i].Clone(), ext));
                    }
                }
                Shell[] shells = Make3D.SewFaces(faces.ToArray());
                if (shells.Length > 0) shell = shells[0]; // should only be one
            }
            if (nameOuter != null)
            {
                List<Solid> res = [];
                for (int i = 0; i < solids.Count; i++)
                {
                    if (pln.IsValid()) res.AddRange(BooleanOperation.SplitSolidByPlane(solids[i], pln, true));
                    else if (shell != null) res.AddRange(BooleanOperation.SplitSolidByShell(solids[i], shell, true));
                }
                if (res.Count == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Splitting reveald no outer part.");
                namedItems[nameOuter] = res;
            }
            if (nameInner != null)
            {
                List<Solid> res = [];
                if (pln.IsValid()) pln.Reverse();
                else if (shell != null) shell.ReverseOrientation();
                for (int i = 0; i < solids.Count; i++)
                {
                    if (pln.IsValid()) res.AddRange(BooleanOperation.SplitSolidByPlane(solids[i], pln, true));
                    else if (shell != null) res.AddRange(BooleanOperation.SplitSolidByShell(solids[i], shell, true));
                }
                if (res.Count == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Splitting reveald no inner part.");
                namedItems[nameInner] = res;
            }
        }

        private void FeatureChamferImpl(JsonElement solid, JsonElement edges, double distance, JsonElement primaryFace, double secondaryDistance, string name, bool rebind, JsonElement rebindTargets)
        {
            List<Edge> edgesToRound = IterateSelector<Edge>(edges).ToList();
            if (edgesToRound.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "No edges found to fillet.");
            Shell? shell = edgesToRound.First().Owner.Owner as Shell;
            if (shell == null) throw new JsonRpcException("E_INVALID_PARAMS", "Edge is not part of a solid.");
            if (double.IsNaN(secondaryDistance)) secondaryDistance = distance;
            // maybe flip distances
            ChamferEdges ce = new ChamferEdges(shell, edgesToRound, distance, secondaryDistance);
            Shell? rounded = ce.Execute();
            if (rounded == null) throw new JsonRpcException("E_OPERATION_FAILED", "Filletting failed.");
            Solid sld = Solid.MakeSolid(rounded);
            if (string.IsNullOrEmpty(name))
            {
                string? originalName = FindName(solid);
                if (originalName != null) namedItems[originalName] = sld;
            }
            else
            {
                namedItems[name] = sld;
            }
            if (rebind) Rebind(shell, rounded);
        }

        private void FeatureFilletImpl(object solid, JsonElement edges, double radius, string? name, bool rebind, JsonElement rebindTargets)
        {
            List<Edge> edgesToRound = IterateSelector<Edge>(edges).ToList();
            if (edgesToRound.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "No edges found to fillet.");
            Shell? shell = edgesToRound.First().Owner.Owner as Shell;
            if (shell == null) throw new JsonRpcException("E_INVALID_PARAMS", "Edge is not part of a solid.");
            RoundEdges re = new RoundEdges(shell, edgesToRound, radius);
            Shell? rounded = re.Execute();
            if (rounded == null) throw new JsonRpcException("E_OPERATION_FAILED", "Filletting failed.");
            Solid sld = Solid.MakeSolid(rounded);
            if (string.IsNullOrEmpty(name))
            {
                string? originalName = FindName(solid);
                if (originalName != null) namedItems[originalName] = sld;
            }
            else
            {
                namedItems[name] = sld;
            }
            if (rebind) Rebind(shell, rounded);
        }

        private List<Edge> EdgesFromEdgeSelector(JsonElement edges)
        {
            List<Edge> res = [];
            // cases: name, id, query, op
            string? expr = null;
            if (edges.ValueKind == JsonValueKind.String)
            {
                expr = edges.GetString();
            }
            else if (edges.TryGetProperty("expr", out JsonElement exprEl))
            {
                expr = exprEl.GetString();
            }
            if (expr != null)
            {
                object evalRes = Evaluator.Evaluate(expr, namedItems.Dict);
                if (evalRes is Edge e) res.Add(e);
                if (evalRes is List<Edge> le) res.AddRange(le);
            }
            else if (edges.TryGetProperty("name", out JsonElement name))
            {
                string? nname = null;
                if (name.ValueKind == JsonValueKind.String) nname = name.GetString();
                if (nname != null && namedItems.TryGetValue(nname, out var named))
                {
                    if (named is Edge e) res.Add(e);
                    if (named is List<Edge> le) res.AddRange(le);
                }
            }
            // we ignore id
            else if (edges.TryGetProperty("query", out JsonElement queryEl))
            {
                res.AddRange(EdgesFromQuery(queryEl));
            }
            else if (edges.TryGetProperty("op", out JsonElement opEl))
            {
                res.AddRange(EdgesFromBoolean(opEl));
            }
            else
            {
                throw new JsonRpcException("E_INVALID_PARAMS", "Edge selector: none of the required parameter 'name', 'query' or 'op' provided.");
            }

            return res;
        }

        private IEnumerable<Edge> EdgesFromBoolean(JsonElement opEl)
        {
            throw new NotImplementedException();
        }

        private IEnumerable<Edge> EdgesFromQuery(JsonElement queryEl)
        {
            // from, filter
            object from = RequireObjectRef<object>(queryEl, "from");
            List<Edge> fromEdges = [];
            if (from is Edge edg) { fromEdges.Add(edg); }
            if (from is List<Edge> ledg) { fromEdges.AddRange(ledg); }
            if (from is Face fce) { fromEdges.AddRange(fce.AllEdges); }
            if (from is List<Face> lfce) { foreach (Face f in lfce) fromEdges.AddRange(f.AllEdges); }
            if (from is Solid sld) { fromEdges.AddRange(sld.Edges); }
            if (from is List<Solid> lsld) { foreach (Solid s in lsld) fromEdges.AddRange(s.Edges); }
            if (fromEdges.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "Edge query: no edges found.");

            if (queryEl.TryGetProperty("filter", out JsonElement filterEl)) return FilterEdges(filterEl, fromEdges);
            else return fromEdges;
        }

        private IEnumerable<Edge> FilterEdges(JsonElement filterEl, List<Edge> fromEdges)
        {
            // filter: isConvex, notOnFace, onFace, dihedralAngleDeg, length
            HashSet<Edge> res = [.. fromEdges];
            if (filterEl.TryGetProperty("isConvex", out JsonElement isConvexEl))
            {
                bool convex = isConvexEl.ValueKind == JsonValueKind.True;
                foreach (Edge edge in res.Clone())
                {
                    if (convex && edge.Adjacency() == AdjacencyType.Concave) res.Remove(edge);
                    if (!convex && edge.Adjacency() == AdjacencyType.Convex) res.Remove(edge);
                }
            }
            if (filterEl.TryGetProperty("notOnFace", out JsonElement notOnFaceEl))
            {
                foreach (object o in IterateSelector<Face>(notOnFaceEl))
                {
                    if (o is Face nof)
                    {
                        foreach (Edge edge in res.Clone())
                        {
                            if (edge.PrimaryFace == nof) res.Remove(edge);
                            if (edge.SecondaryFace == nof) res.Remove(edge);
                        }
                    }
                }
            }
            if (filterEl.TryGetProperty("onFace", out JsonElement onFaceEl))
            {
                foreach (object o in IterateSelector<Face>(onFaceEl))
                {
                    if (o is Face of)
                    {
                        foreach (Edge edge in res.Clone())
                        {
                            if (edge.PrimaryFace != of && edge.SecondaryFace != of) res.Remove(edge);
                        }
                    }
                }
            }
            if (filterEl.TryGetProperty("dihedralAngleDeg", out JsonElement dihedralAngleDegEl))
            {
                throw new NotImplementedException("Property 'dihedralAngleDeg' in edge filter not implemented");
            }
            if (filterEl.TryGetProperty("length", out JsonElement lengthEl))
            {
                double min = GetOptionalNumber(lengthEl, "minValue", double.MinValue);
                double max = GetOptionalNumber(lengthEl, "maxValue", double.MaxValue);
                foreach (Edge edge in res.Clone())
                {
                    if (edge.Curve3D.Length < min || edge.Curve3D.Length > max) res.Remove(edge);
                }
            }
            if (filterEl.TryGetProperty("boundingBox", out JsonElement boundingBoxEl))
            {
                double min = GetOptionalNumber(boundingBoxEl, "minValue", double.MinValue);
                double max = GetOptionalNumber(boundingBoxEl, "maxValue", double.MaxValue);
                string component = RequireString(boundingBoxEl, "component");
                foreach (Edge edge in res.Clone())
                {
                    BoundingBox ext = edge.Curve3D.GetExtent();
                    switch (component)
                    {

                        case "xMin":
                            if (ext.Xmin < min || ext.Xmin > max) res.Remove(edge);
                            break;
                        case "xMax":
                            if (ext.Xmax < min || ext.Xmax > max) res.Remove(edge);
                            break;
                        case "yMin":
                            if (ext.Ymin < min || ext.Ymin > max) res.Remove(edge);
                            break;
                        case "yMax":
                            if (ext.Ymax < min || ext.Ymax > max) res.Remove(edge);
                            break;
                        case "zMin":
                            if (ext.Zmin < min || ext.Zmin > max) res.Remove(edge);
                            break;
                        case "zMax":
                            if (ext.Zmax < min || ext.Zmax > max) res.Remove(edge);
                            break;
                        case "xDiff":
                            if (ext.XDiff < min || ext.XDiff > max) res.Remove(edge);
                            break;
                        case "yDiff":
                            if (ext.YDiff < min || ext.YDiff > max) res.Remove(edge);
                            break;
                        case "zDiff":
                            if (ext.ZDiff < min || ext.ZDiff > max) res.Remove(edge);
                            break;

                    }
                }
            }
            return res;
        }
        private void AssertCheckImpl(JsonElement objects, string? condition, int minCount, int maxCount, string message, string name)
        {
            List<object> selected = IterateSelector<object>(objects).ToList();
            // "this.", FaceWrapperForEval with properties
            if (minCount >= 0) // default: -1
            {
                if (selected.Count < minCount) throw new JsonRpcException("E_ASSERTION_FAILED", $"Assertion failed. Count={selected.Count}. {message}");
            }
            if (maxCount >= 0)
            {
                if (selected.Count > maxCount) throw new JsonRpcException("E_ASSERTION_FAILED", $"Assertion failed. Count={selected.Count}. {message}");
            }
            if (condition != null)
            {
                object? oldValue = null;
                try
                {
                    namedItems.TryGetValue("this", out oldValue);
                    foreach (var item in selected)
                    {
                        object? wrappedItem = wrapForEvaluator(item);
                        if (wrappedItem != null)
                        {
                            namedItems["this"] = wrappedItem;
                            object evalRes = Evaluator.Evaluate(condition, namedItems.Dict);
                            if (evalRes is bool b)
                            {
                                if (!b) throw new JsonRpcException("E_ASSERTION_FAILED", $"Assertion failed. {message}");
                            }
                        }
                        else throw new NotImplementedException("assert.check not yet fully implemented");
                    }
                    if (selected.Count == 0)
                    {   // a condition without objects
                        object evalRes = Evaluator.Evaluate(condition, namedItems.Dict);
                        if (evalRes is bool b)
                        {
                            if (!b) throw new JsonRpcException("E_ASSERTION_FAILED", $"Assertion failed. {message}");
                        }
                    }
                }
                finally
                {
                    if (oldValue != null) namedItems["this"] = oldValue;
                    else namedItems.Remove("this");
                }
            }
        }
        private class FaceWrapperForEvaluator
        {
            Face face;
            public FaceWrapperForEvaluator(Face face)
            {
                this.face = face;
            }
            public string SurfaceType
            {
                get
                {
                    if (face.Surface is PlaneSurface) return "planar";
                    return "other";
                }
            }
            public int EdgeCount => face.AllEdges.Length;
            public BoundingBox bounds => face.GetExtent(0.0);
        }
        private class EdgeWrapperForEvaluator
        {
            Edge edge;
            public EdgeWrapperForEvaluator(Edge edge)
            {
                this.edge = edge;
            }
            public string CurveType
            {
                get
                {
                    if (edge.Curve3D is Line) return "line";
                    if (edge.Curve3D is Ellipse elli)
                    {
                        if (elli.IsCircle)
                        {
                            if (elli.IsClosed) return "circle";
                            else return "arc";
                        }
                        else
                        {
                            if (elli.IsClosed) return "ellipse";
                            else return "ellipse arc";
                        }
                    }
                    return "other";
                }
            }
            public GeoPoint startPoint => edge.Curve3D.StartPoint;
            public GeoPoint endPoint => edge.Curve3D.EndPoint;
            public GeoPoint pointAt(double u) => edge.Curve3D.PointAt(u);
            public GeoVector directionAt(double u) => edge.Curve3D.DirectionAt(u);
            public GeoVector startDirection => edge.Curve3D.StartDirection;
            public GeoVector endDirection => edge.Curve3D.EndDirection;
            public BoundingBox bounds => edge.Curve3D.GetExtent();
        }
        private static object? wrapForEvaluator(object item)
        {
            if (item is Face fc) return new FaceWrapperForEvaluator(fc);
            if (item is Edge edg) return new EdgeWrapperForEvaluator(edg);
            // TODO implement other wrappers
            return item;
        }

        private void DocumentCommitObjectsImpl(JsonElement objects)
        {
            foreach (Solid sld in IterateSelector<Solid>(objects))
            {
                Project? project = FrameImpl.MainFrame?.Project;
                if (project != null)
                {
                    Style style = project.StyleList.GetDefault(Style.EDefaultFor.Solids);
                    if (style != null) { sld.Style = style; }
                    FrameImpl.MainFrame?.Project?.GetActiveModel()?.Add(sld);
                }
            }
        }
        private void TransformScaleImpl(JsonElement objectsEl, GeoPoint center, double factor, JsonElement factorsEl, string name, string copySuffix)
        {
            throw new NotImplementedException();
        }
        private void TransformReflectImpl(JsonElement objectsEl, Plane plane, Axis axis3d, Axis2D axis2d, string name, string copySuffix)
        {
            List<object> objects = IterateObjectRefs<object>(objectsEl).ToList();
            if (objects.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "No objects to reflect.");
            if (objects[0] is Solid)
            {   // 3d reflection

            }
            else
            {   // 2d reflection
                if (axis2d.IsValid)
                {
                    ModOp2D reflect = ModOp2D.Reflect(axis2d.Location, axis2d.Direction);
                    foreach (object obj in objects)
                    {
                        if (obj is ICurve2D c2d)
                        {
                            ICurve2D? clone = c2d.GetModified(reflect) as ICurve2D;
                            if (clone != null)
                            {
                                if (!string.IsNullOrEmpty(name)) namedItems[name] = clone;
                                else
                                {
                                    string? currentName = FindName(c2d);
                                    if (currentName != null) namedItems[currentName] = clone;
                                }
                            }
                        }
                        else if (obj is CompoundShape cs)
                        {
                            CompoundShape? clone = cs.GetModified(reflect) as CompoundShape;
                            if (clone != null)
                            {
                                if (!string.IsNullOrEmpty(name)) namedItems[name] = clone;
                                else
                                {
                                    string? currentName = FindName(cs);
                                    if (currentName != null) namedItems[currentName] = clone;
                                }
                            }
                        }
                    }
                }
                else throw new JsonRpcException("E_INVALID_PARAMS", "For 2D reflection, a valid axis2d must be provided.");
            }
        }

        private void TransformRotateImpl(JsonElement objectsEl, Axis axis, double angle, string name, string copySuffix)
        {
            List<Solid> objects = IterateObjectRefs<Solid>(objectsEl).ToList();
            GeoPoint origin = axis.Location;
            GeoVector direction = axis.Direction;
            ModOp rot = ModOp.Rotate(origin, direction, SweepAngle.Deg(angle));
            bool copy = !string.IsNullOrEmpty(name);
            List<Solid> modified = [];
            foreach (Solid s in objects)
            {
                Solid? clone;
                if (copy) clone = s.Clone() as Solid;
                else clone = s;
                if (clone != null)
                {
#if DEBUG
                    clone.Shell.CheckConsistency();
#endif
                    clone.Modify(rot);
#if DEBUG
                    clone.Shell.CheckConsistency();
#endif
                    modified.Add(clone);
                    if (copy && !string.IsNullOrEmpty(copySuffix))
                    {
                        string newName = name + copySuffix;
                        namedItems[newName] = clone;
                    }
                }
            }
            if (copy) namedItems[name] = modified;
        }
        private void TransformMoveImpl(JsonElement objects, GeoVector delta, string name, string copySuffix)
        {
            List<Solid> toMove = IterateSelector<Solid>(objects).ToList();
            List<Solid> modified = [];
            ModOp move = ModOp.Translate(delta);
            for (int i = 0; i < toMove.Count; i++)
            {
                if (name == null) toMove[i].Modify(move);
                else
                {
                    Solid clone = toMove[i].Clone() as Solid;
                    modified.Add(clone);
                    clone.Modify(move);
                    if (name != null && !string.IsNullOrEmpty(copySuffix))
                    {
                        string newName = name + copySuffix;
                        namedItems[newName] = clone;
                    }
                }
            }
            if (name != null) namedItems[name] = modified;
        }


        private void InspectPropertiesImpl(string target, JsonElement properties)
        {
            throw new NotImplementedException();
        }

        private void InspectSceneImpl(JsonElement targets, bool includeBoundingBoxes, string geometryFormat, bool includeImage)
        {
            throw new NotImplementedException();
        }

        private void InspectSummaryImpl(JsonElement targets)
        {
            throw new NotImplementedException();
        }


        private Axis AxisFromJson(JsonElement axisRef)
        {
            // AxisRef can be either {standard:"X"|"Y"|"Z"} or {origin:{x,y,z}, direction:{x,y,z}}
            if (axisRef.TryGetProperty("standard", out JsonElement stdEl) && stdEl.ValueKind == JsonValueKind.String)
            {
                string std = stdEl.GetString()?.ToUpper() ?? "Z";
                return std switch
                {
                    "X" => new Axis(GeoPoint.Origin, GeoVector.XAxis),
                    "Y" => new Axis(GeoPoint.Origin, GeoVector.YAxis),
                    "Z" => new Axis(GeoPoint.Origin, GeoVector.ZAxis),
                    _ => throw new JsonRpcException("E_INVALID_PARAMS", "Unknown standard axis.")
                };
            }
            if (axisRef.TryGetProperty("origin", out JsonElement orgEl) && axisRef.TryGetProperty("direction", out JsonElement dirEl))
            {
                GeoPoint org = RequirePoint3D(orgEl, null);
                GeoVector dir = RequireVector3D(dirEl);
                try
                {
                    return new Axis(org, dir);
                }
                catch (ArgumentException ex)
                {
                    throw new JsonRpcException("E_INVALID_PARAMS", "Invalid axis: " + ex.Message);
                }
            }
            throw new JsonRpcException("E_INVALID_PARAMS", "Invalid AxisRef.");
        }

        public static readonly Dictionary<string, int> ErrorNumbers = new()
        {
            ["E_INVALID_PARAMS"] = 1001,
            ["E_NOT_FOUND"] = 1002,
            ["E_DOC_CHANGED"] = 1003,
            ["E_CENTER_OUTSIDE_FACE"] = 1203,
            ["E_REF_GONE"] = 1302,
            ["E_BOOLEAN_FAIL"] = 1401
        };
        public struct ParameterInfo
        {
            public string label;
            public object? defaultValue;
            public string? kind;
            public string? description;
            public string? group;
            public int order;
        }

        internal string? GetTemplateLabel(string key)
        {
            if (templates.TryGetValue(key, out var jsons))
            {
                if (jsons.Count > 0 && jsons[0].ValueKind == JsonValueKind.Object)
                {
                    if (jsons[0].TryGetProperty("params", out var parameters) && parameters.ValueKind == JsonValueKind.Object)
                    {
                        if (parameters.TryGetProperty("label", out var label) && label.ValueKind == JsonValueKind.String)
                            return label.GetString();
                    }
                }
            }
            return null;
        }
        internal string? GetTemplateDescription(string key)
        {
            if (templates.TryGetValue(key, out var jsons))
            {
                if (jsons.Count > 0 && jsons[0].ValueKind == JsonValueKind.Object)
                {
                    if (jsons[0].TryGetProperty("params", out var parameters) && parameters.ValueKind == JsonValueKind.Object)
                    {
                        if (parameters.TryGetProperty("description", out var description) && description.ValueKind == JsonValueKind.String)
                            return description.GetString();
                    }
                }
            }
            return null;
        }
        public ParameterInfo GetTemplateParameterInfo(string templateName, string parameterName)
        {
            ParameterInfo res = new ParameterInfo();
            if (templates.TryGetValue(templateName, out var jsons))
            {
                if (jsons.Count > 0 && jsons[0].ValueKind == JsonValueKind.Object)
                {
                    using (new NamedItemClone(this))
                    {
                        ProcessMethod(jsons[0], false); // now we should find the values of the parameters in the (temporary) namedItems
                        if (jsons[0].TryGetProperty("params", out var prms) && prms.ValueKind == JsonValueKind.Object)
                        {
                            if (prms.TryGetProperty("parameters", out var parameters) && parameters.ValueKind == JsonValueKind.Array)
                            {
                                foreach (var item in parameters.EnumerateArray())
                                {
                                    if (item.ValueKind == JsonValueKind.Object)
                                    {
                                        if (item.TryGetProperty("name", out var propName) && propName.ValueKind == JsonValueKind.String)
                                        {
                                            if (propName.GetString() == parameterName)
                                            {
                                                if (item.TryGetProperty("label", out var label) && label.ValueKind == JsonValueKind.String)
                                                    res.label = label.GetString()!;
                                                namedItems.TryGetValue(parameterName, out res.defaultValue);
                                                if (item.TryGetProperty("input", out var input) && input.ValueKind == JsonValueKind.Object)
                                                {
                                                    if (input.TryGetProperty("kind", out var kind) && kind.ValueKind == JsonValueKind.String)
                                                        res.kind = kind.GetString();
                                                    if (input.TryGetProperty("description", out var description) && description.ValueKind == JsonValueKind.String)
                                                        res.description = description.GetString();
                                                    if (input.TryGetProperty("group", out var group) && group.ValueKind == JsonValueKind.String)
                                                        res.group = group.GetString();
                                                    if (input.TryGetProperty("order", out var order) && order.ValueKind == JsonValueKind.Number)
                                                        res.order = order.GetInt32();
                                                }
                                            }

                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return res;
        }
        internal List<string> GetTemplateParameters(string templateName)
        {
            List<string> res = [];
            if (templates.TryGetValue(templateName, out var jsons))
            {
                if (jsons.Count > 0 && jsons[0].ValueKind == JsonValueKind.Object)
                {
                    if (jsons[0].TryGetProperty("params", out var prms) && prms.ValueKind == JsonValueKind.Object)
                    {
                        if (prms.TryGetProperty("parameters", out var parameters) && parameters.ValueKind == JsonValueKind.Array)
                        {
                            foreach (var item in parameters.EnumerateArray())
                            {
                                if (item.ValueKind == JsonValueKind.Object)
                                {
                                    if (item.TryGetProperty("name", out var propName) && propName.ValueKind == JsonValueKind.String)
                                    {
                                        res.Add(propName.GetString()!);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return res;
        }
        public object? ExecuteTemplate(string template, Dictionary<string, object> parameterValues)
        {
            try
            {
                if (templates.TryGetValue(template, out var jsons))
                {
                    using (new NamedItemClone(this))
                    {
                        foreach (var element in jsons)
                        {
                            string methodName = RequireString(element, "method");
                            if (methodName == "template.commit")
                            {
                                if (!element.TryGetProperty("params", out var parameters)) throw new JsonRpcException("E_INTERNAL_ERROR", $"Template '{template}' has invalid commit method.");
                                JsonElement result = RequireProperty(parameters, "result");
                                var resultKind = GetOptionalString(parameters, "resultKind");
                                var suffixInternalNames = GetOptionalBool(parameters, "suffixInternalNames", true);

                                object? res = TemplateCommitImpl(result, resultKind, suffixInternalNames);
                                return res;

                            }
                            else
                            {
                                ProcessMethod(element, true);
                                if (methodName == "template.begin")
                                {   // here we overwrite the workspace values of the parameters
                                    foreach (var item in parameterValues)
                                    {
                                        namedItems[item.Key] = item.Value;
                                    }
                                }
                            }
                        }
                    }
                }
                return null;
            }
            catch (Exception ex)
            {
                return null;
            }
        }


        // Placeholder type for "profile" objects created from sketches.
        // Replace with the real CADability/ShapeIt type when you wire it up.
        internal sealed class Profile
        {
            public string? Name { get; set; }
        }
        internal class Sketch
        {
            Plane plane;
            List<ICurve2D> curves = [];
            List<CompoundShape> shapes = [];

            public Sketch(Plane plane)
            {
                this.plane = plane;
            }
            public void Add(ICurve2D curve)
            {
                curve.UserData.Add("MCPServer.Sketch", this);
                curves.Add(curve);
            }

            public void Add(CompoundShape shape)
            {
                shape.UserData.Add("MCPServer.Sketch", this);
                shapes.Add(shape);
            }

            internal CompoundShape? GetCompoundShape()
            {
                if (curves.Count == 0 && shapes.Count == 1) return shapes[0];
                if (curves.Count == 0 && shapes.Count == 0) return null;
                if (shapes.Count > 0)
                {   // we must somhow combine the compound shapes 
                    CompoundShape? shape = shapes[0];
                    for (int i = 1; i < shapes.Count; i++)
                    {
                        shape = CompoundShape.Union(shape, shapes[i]);
                    }
                    shape.UserData.Add("MCPServer.Sketch", this);
                    return shape;
                }
                if (curves.Count == 1 && curves[0].IsClosed && shapes.Count == 0) return new CompoundShape(new SimpleShape(new Border(curves[0])));
                if (curves.Count > 1)
                {
                    List<SimpleShape> simpleShapes = new List<SimpleShape>();
                    for (int i = 0; i < curves.Count; i++)
                    {
                        if (curves[i].IsClosed) simpleShapes.Add(new SimpleShape(new Border(curves[i])));
                    }
                    // we should check all SimpleShapes against each other.
                    // but for now, quick and dirty
                    simpleShapes.Sort((a, b) => b.Area.CompareTo(a.Area));
                    CompoundShape? res = null;
                    for (int i = 0; i < simpleShapes.Count; i++)
                    {
                        if (simpleShapes[i] == null) continue;
                        CompoundShape cs = new CompoundShape(simpleShapes[i]);
                        for (int j = i + 1; j < simpleShapes.Count; j++)
                        {
                            if (simpleShapes[j] == null) continue;
                            if (SimpleShape.GetPosition(simpleShapes[i], simpleShapes[j]) == SimpleShape.Position.firstcontainscecond)
                            {
                                cs = CompoundShape.Difference(cs, new CompoundShape(simpleShapes[j]));
                                simpleShapes[j] = null; // mark as used
                            }
                        }
                        if (res == null) res = cs;
                        else res = CompoundShape.Union(res, cs);
                    }
                    res.UserData.Add("MCPServer.Sketch", this);
                    return res;
                }
                return null;
            }
            public Plane Plane => plane;
            public List<ICurve2D> Curves => curves;
            public List<CompoundShape> Shapes => shapes;
        }
    }
}