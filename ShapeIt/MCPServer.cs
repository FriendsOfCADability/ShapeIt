using CADability;
using CADability.Attribute;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
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
    internal partial class MCPServer
    {
        // Named workspace items and created objects.
        // Names are chosen by the caller (LLM/client). IDs are opaque strings returned by the server.
        private readonly Dictionary<string, object> namedItems = new(StringComparer.Ordinal);
        private readonly Dictionary<string, object> idItems = new(StringComparer.Ordinal);
        private readonly Dictionary<string, string> nameToId = new(StringComparer.Ordinal);

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
        private string? FindName(object entity)
        {
            foreach (var item in namedItems)
            {
                if (item.Value == entity) return item.Key;
            }
            return null;
        }
        private void StoreId(string id, object value)
        {
            idItems[id] = value;
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
            else if (value.ValueKind == JsonValueKind.String)
            {
                namedItems[name] = Evaluator.Evaluate(value.GetString(), namedItems);
            }
            else if (value.ValueKind == JsonValueKind.Object && value.TryGetProperty("expr", out var JeExpr) && JeExpr.ValueKind == JsonValueKind.String)
            {
                namedItems[name] = Evaluator.Evaluate(JeExpr.GetString(), namedItems);
            }
            else
            {
                GeoVector v = GetOptionalVector3D(value, null, GeoVector.Invalid);
                if (v.IsValid())
                {
                    namedItems[name] = v;
                }
                else
                {
                    List<Face> faces = IterateSelector<Face>(value).ToList();
                    if (faces.Count > 0)
                    {
                        namedItems[name] = faces;
                        return;
                    }
                    List<Solid> solids = IterateSelector<Solid>(value).ToList();
                    if (solids.Count > 0)
                    {
                        namedItems[name] = solids;
                        return;
                    }
                    // and more iteators for different types 
                }
            }
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
            if (radius == double.MinValue && diameter == double.MinValue)
                throw new JsonRpcException(-32602, "Circle must have either radius or diameter.");
            if (radius == double.MinValue) radius = diameter / 2.0;
            ICurve2D curve = new Circle2D(center, radius);
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
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
                    double x = (double)Evaluator.Evaluate(xExpr, namedItems);
                    double y = (double)Evaluator.Evaluate(yExpr, namedItems);
                    return new GeoPoint2D(x, y);
                };
                curve = BSpline2D.Approximate(crv, tolerance, tMin, tMax, maxSamples);
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
                    curves.Add(new Arc2D(center, r, pi, pn, h > 0));
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
            if (outerRadius == 0.0) outerRadius = innerRadius / Math.Cos(Math.PI / sides);
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
            string fontFamily =  RequireString(font, "family");
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

        private void SolidExtrudeImpl(JsonElement profile, double length, GeoVector direction, string? name, double offset, double pitch, JsonElement capture)
        {
            List<object> profiles = IterateSelector<object>(profile).ToList(); // should return a single sketch or a compoundShape or a closed curve
            Sketch? sketch = null;
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
            if (coneTipDistance == double.MinValue)
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
            throw new NotImplementedException();
        }


        private void SolidCreatePrimitiveImpl(string kind, JsonElement sparams, string name)
        {
            Solid? res = null;
            switch (kind)
            {
                case "box":
                    {
                        GeoPoint origin = RequirePoint3D(sparams, "origin");
                        GeoVector axisX = GetOptionalVector3D(sparams, "axisX", GeoVector.Invalid);
                        GeoVector axisY = GetOptionalVector3D(sparams, "axisY", GeoVector.Invalid);
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
                        double sizeX = RequireDouble(sparams, "sizeX");
                        double sizeY = RequireDouble(sparams, "sizeY");
                        double sizeZ = RequireDouble(sparams, "sizeZ");
                        res = Make3D.MakeBox(origin, sizeX * axisX, sizeY * axisY, sizeZ * axisZ);
                    }
                    break;
                case "sphere":
                    {
                        GeoPoint center = RequirePoint3D(sparams, "center");
                        double radius = RequireDouble(sparams, "radius");
                        res = Make3D.MakeSphere(center, radius);
                    }
                    break;
                case "cylinder":
                    {
                        GeoPoint start = RequirePoint3D(sparams, "start");
                        GeoPoint end = RequirePoint3D(sparams, "end");
                        double radius = RequireDouble(sparams, "radius");
                        Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
                        GeoVector dirx = radius * pln.ToGlobal(GeoVector2D.XAxis);
                        res = Make3D.MakeCylinder(start, dirx, end - start);
                    }
                    break;
                case "cone":
                    {
                        GeoPoint start = RequirePoint3D(sparams, "start");
                        GeoPoint end = RequirePoint3D(sparams, "end");
                        double radiusStart = RequireDouble(sparams, "radiusStart");
                        double radiusEnd = RequireDouble(sparams, "radiusEnd");
                        Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
                        GeoVector dirx = pln.ToGlobal(GeoVector2D.XAxis);
                        res = Make3D.MakeCone(start, dirx, end - start, radiusStart, radiusEnd);
                    }
                    break;
                case "torus":
                    {
                        GeoPoint center = RequirePoint3D(sparams, "center");
                        GeoVector axis = RequireVector3D(sparams, "axis");
                        double majorRadius = RequireDouble(sparams, "majorRadius");
                        double minorRadius = RequireDouble(sparams, "minorRadius");
                        Plane pln = new Plane(center, axis); // to use the arbitrary axis algorithm
                        res = Make3D.MakeTorus(center, axis, majorRadius, minorRadius);
                    }
                    break;
                case "capsule":
                    {
                        GeoPoint start = RequirePoint3D(sparams, "start");
                        GeoPoint end = RequirePoint3D(sparams, "end");
                        double l = end | start; // the length of the capsule
                        double radius = RequireDouble(sparams, "radius");
                        Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
                        Plane profilePlane = new Plane(start, end - start, pln.DirectionX); // in this plane we construct a profile for rotation along the x-axis of the plane
                        GeoVector dirx = radius * pln.ToGlobal(GeoVector2D.XAxis);
                        Solid cylinder = Make3D.MakeCylinder(start, dirx, end - start);
                        string cap = RequireString(sparams, "cap");
                        double coneTipDistance = GetOptionalDouble(sparams, "coneTipDistance", double.MinValue);
                        if (coneTipDistance == double.MinValue)
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
                    }
                    break;
                case "pipe":
                    {
                        GeoPoint start = RequirePoint3D(sparams, "start");
                        GeoPoint end = RequirePoint3D(sparams, "end");
                        double outerRadius = RequireDouble(sparams, "outerRadius");
                        double innerRadius = RequireDouble(sparams, "innerRadius");
                        Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
                        GeoVector dirx = outerRadius * pln.ToGlobal(GeoVector2D.XAxis);
                        Solid cylinder1 = Make3D.MakeCylinder(start, dirx, end - start);
                        dirx = innerRadius * pln.ToGlobal(GeoVector2D.XAxis);
                        Solid cylinder2 = Make3D.MakeCylinder(start, dirx, end - start);
                        Solid[] diff = BooleanOperation.Subtract(cylinder1, cylinder2);
                        if (diff != null && diff.Length == 1) res = diff[0];
                    }
                    break;
                default: throw new JsonRpcException("E_INVALID_PARAMS", "Invalid 'kind' parameter.");

            }
            if (res != null)
            {
                if (name != null) namedItems[name] = res;
            }
            else
            {
                throw new JsonRpcException("E_OPERATION_FAILED", $"Failed to create primitive {kind}.");
            }
        }


        private void SystemGetInfoImpl()
        {
            throw new NotImplementedException();
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
            if (remainingCurves.Count>0)
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
        private object SolidSweepImpl(object profile, object path, string? orientation, string? name, JsonElement capture) => throw new NotImplementedException();
        private void SolidRotateImpl(JsonElement profile, Axis axis, double angle, string? name, JsonElement capture)
        {
            List<CompoundShape> profiles = GetProfiles(profile);
            List<Solid> res = [];
            for (int i = 0; i < profiles.Count; i++)
            {
                Sketch? sketch = profiles[i].UserData["MCPServer.Sketch"] as Sketch;
                if (sketch==null) throw new JsonRpcException("E_OPERATION_FAILED", "No suitable sketch found for profile");
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
            if (nearPoints.ValueKind != JsonValueKind.Undefined || indices.ValueKind != JsonValueKind.Undefined)
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
        private Solid UniteWithMany(Solid a, List<Solid> b)
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
            return accumulate;
        }

        private Solid[] SubtractMany(Solid toSubtractFrom, List<Solid> subtractItems)
        {
            List<Solid> fragments = new List<Solid>();
            fragments.Add(toSubtractFrom);
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
            return fragments.ToArray();
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
            if (slda == null || sldb.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "Boolean operations require at least one solid 'a' and at least one other solid 'b' to operate with.");
            if (name == null && slda[0] is IGeoObject go) name = go.UserData["CADablity.MCP.Name"] as string;
            object? res = null;
            Solid s1 = slda[0];
            List<Solid> s2 = [.. sldb, .. slda.Skip(1)];
            // if there are more than one solid in a, we also treat them as solids to operate with, so we add them to the list of b solids. This is a bit unintuitive but it allows for more complex operations without needing to call boolean multiple times. For example, if you want to unite 3 solids, you can just put them all in a and leave b empty.
            switch (op.ToLower())
            {
                case "union":
                case "unite":
                    {
                        res = UniteWithMany(s1, s2);
                    }
                    break;
                case "difference":
                case "subtract":
                    {
                        res = new List<Solid>(SubtractMany(s1, s2));
                    }
                    break;
                case "intersect":
                    {
                        res = new List<Solid>(IntersectMany(s1, s2));
                    }
                    break;
                default: throw new JsonRpcException("E_INVALID_PARAMS", $"'solid.boolean' unknown operator {op}");
            }

            if (res != null && name != null) namedItems[name] = res;
        }

        private void PatternCircularSolidsImpl(JsonElement objects, GeoPoint center, GeoVector axis, int count, double angle, bool copy, string name, bool suffix)
        {

            List<Solid> list = IterateObjectRefs<Solid>(objects).ToList(); // all objects assiziated via userdat by name
            double rotationAngle = double.IsNaN(angle) ? 360 : angle;
            if (rotationAngle <= Precision.eps) throw new JsonRpcException("E_INVALID_PARAMS", "Angle must be greater than 0.");
            double stepAngle = rotationAngle / count;
            List<Solid> current = new List<Solid>();
            foreach (Solid s in list) current.Add(s.Clone() as Solid);
            List<Solid> total = new List<Solid>(current);
            ModOp rot = ModOp.Rotate(center, axis, SweepAngle.Deg(stepAngle));
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
        private void SketchOffsetImpl(Sketch sketch, JsonElement sketchGeometry, double distance, string joinType, double miterLimit, bool makeRegion, string capType, string name)
        {
            throw new NotImplementedException();
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
                }
            }
            // TODO: capture, rebind
        }
        private void FeatureSplitImpl(JsonElement solid, Plane splitBy, string nameInner, string nameOuter, bool rebind, JsonElement rebindTargets)
        {
            List<Solid> solids = IterateSelector<Solid>(solid).ToList(); // should only be one
            if (nameOuter != null)
            {
                List<Solid> res = [];
                for (int i = 0; i < solids.Count; i++)
                {
                    res.AddRange(BooleanOperation.SplitSolidByPlane(solids[i], splitBy, true));
                }
                if (res.Count == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Splitting reveald no outer part.");
                namedItems[nameOuter] = res;
            }
            if (nameInner != null)
            {
                List<Solid> res = [];
                splitBy.Reverse();
                for (int i = 0; i < solids.Count; i++)
                {
                    res.AddRange(BooleanOperation.SplitSolidByPlane(solids[i], splitBy, true));
                }
                if (res.Count == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Splitting reveald no inner part.");
                namedItems[nameInner] = res;
            }
        }

        private void FeatureChamferImpl(JsonElement solid, JsonElement edges, double distance, JsonElement primaryFace, double secondaryDistance, string name, bool rebind, JsonElement rebindTargets)
        {
            throw new NotImplementedException();
        }

        private void FeatureFilletImpl(object solid, JsonElement edges, double radius, string? name, bool rebind, JsonElement rebindTargets)
        {
            List<Edge> edgesToRound = EdgesFromEdgeSelector(edges);
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
        }

        private List<Edge> EdgesFromEdgeSelector(JsonElement edges)
        {
            List<Edge> res = [];
            // cases: name, id, query, op
            if (edges.TryGetProperty("name", out JsonElement name))
            {
                string? nname = null;
                if (name.ValueKind == JsonValueKind.String) nname = name.GetString();
                if (nname != null && namedItems.TryGetValue(nname, out object named))
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
                            object evalRes = Evaluator.Evaluate(condition, namedItems);
                            if (evalRes is bool b)
                            {
                                if (!b) throw new JsonRpcException("E_ASSERTION_FAILED", $"Assertion failed. {message}");
                            }
                        }
                        else throw new NotImplementedException("assert.check not yet fully implemented");
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
            public int EdgeCount
            {
                get
                {
                    return face.AllEdges.Length;
                }
            }
        }
        private object? wrapForEvaluator(object item)
        {
            if (item is Face fc) return new FaceWrapperForEvaluator(fc);
            // TODO implement other wrappers
            return null;
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
                    clone.Modify(rot);
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