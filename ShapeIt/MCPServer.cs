using CADability;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using CdlToCSharp;
using System;
using System.CodeDom;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Security.Policy;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Windows.Forms.VisualStyles;
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

        private JsonNode DocGetStateImpl() => throw new NotImplementedException();
        private void UndoBeginImpl(string label)
        {

        }
        private void UndoEndImpl(string undoFrameId)
        {

        }
        private JsonNode UndoCancelImpl(string undoFrameId) => throw new NotImplementedException();

        private void WorkspaceNameImpl(string name, JsonElement value)
        {
            if (value.ValueKind == JsonValueKind.Number)
            {
                namedItems[name] = value.GetDouble();
            }
        }
        private void WorkspaceDeleteImpl(JsonElement objects) => throw new NotImplementedException();

        private void SketchCreateImpl(JsonElement plane, string? name)
        {
            Plane pl = PlaneFromJson(plane);
            Sketch sketch = new Sketch(pl);
            if (name != null) namedItems[name] = sketch;
        }
        private object SketchCreateOnFaceImpl(object face, string? name) => throw new NotImplementedException();
        private void SketchAddEntityImpl(Sketch sketch, string kind, JsonElement entityParams, string? name)
        {
            ICurve2D? curve = null;

            switch (kind)
            {
                case "line":
                    {
                        GeoPoint2D start = RequirePoint2D(entityParams, "start");
                        GeoPoint2D end = RequirePoint2D(entityParams, "end");
                        curve = new Line2D(start, end);
                        break;
                    }

                case "arc":
                    {
                        GeoPoint2D center = RequirePoint2D(entityParams, "center");
                        double radius = RequireLength(entityParams, "radius");
                        double startAngle = RequireLength(entityParams, "startAngleDeg");
                        double sweepAngle = RequireLength(entityParams, "sweepAngleDeg");

                        curve = new Arc2D(center, radius,
                                          Angle.Deg(startAngle),
                                          SweepAngle.Deg(sweepAngle));
                        break;
                    }

                case "circle":
                    {
                        GeoPoint2D c = RequirePoint2D(entityParams, "center");
                        double radius = GetOptionalLength(entityParams, "radius", double.MinValue);
                        double diameter = GetOptionalLength(entityParams, "diameter", double.MinValue);
                        if (radius == double.MinValue && diameter == double.MinValue)
                            throw new JsonRpcException(-32602, "Circle must have either radius or diameter.");
                        if (radius == double.MinValue) radius = diameter / 2.0;
                        curve = new Circle2D(c, radius);
                        break;
                    }

                case "ellipse":
                    {
                        GeoPoint2D center = RequirePoint2D(entityParams, "center");
                        double major = RequireLength(entityParams, "majorRadius");
                        double minor = RequireLength(entityParams, "minorRadius");
                        double rotation = GetOptionalLength(entityParams, "rotation", 0.0);
                        ModOp2D rot = ModOp2D.Rotate(center, SweepAngle.Deg(rotation));
                        GeoVector2D majAxis = rot * (major * GeoVector2D.XAxis);
                        GeoVector2D minAxis = rot * (minor * GeoVector2D.YAxis);
                        curve = new Ellipse2D(center, majAxis, minAxis);
                        break;
                    }

                case "regular_polygon":
                    {
                        GeoPoint2D center = RequirePoint2D(entityParams, "center");
                        double outerRadius = GetOptionalLength(entityParams, "outerRadius", 0.0);
                        double innerRadius = GetOptionalLength(entityParams, "innerRadius", 0.0);
                        int sides = (int)RequireLength(entityParams, "sides");
                        double rotation = GetOptionalLength(entityParams, "rotation", 0.0);
                        if (outerRadius == 0.0) outerRadius = innerRadius / Math.Cos(Math.PI / sides);
                        curve = Polyline2D.MakeRegularPolygon(center, outerRadius, rotation * Math.PI / 180.0, sides);
                        break;
                    }

                case "slot":
                    {
                        GeoPoint2D center = RequirePoint2D(entityParams, "center");
                        double length = RequireLength(entityParams, "length");
                        double width = RequireLength(entityParams, "width");
                        double rotation = GetOptionalLength(entityParams, "rotation", 0.0);

                        // curve = Path2D.CreateSlot(center, length, width, rotation * Math.PI / 180.0);
                        throw new NotImplementedException();
                        break;
                    }

                case "nurbs":
                    {
                        int degree = (int)RequireLength(entityParams, "degree");

                        var cpsEl = RequireProperty(entityParams, "controlPoints");
                        if (cpsEl.ValueKind != JsonValueKind.Array)
                            throw new JsonRpcException(-32602, "controlPoints must be array");

                        List<GeoPoint2D> cps = new List<GeoPoint2D>();
                        //foreach (var cp in cpsEl.EnumerateArray())
                        //    cps.Add(ParsePoint2D(cp));

                        //curve = new Nurbs2D(cps.ToArray(), degree);
                        throw new NotImplementedException();
                        break;
                    }

                case "polycurve":
                    {
                        var segsEl = RequireProperty(entityParams, "segments");
                        if (segsEl.ValueKind != JsonValueKind.Array)
                            throw new JsonRpcException(-32602, "segments must be array");

                        List<ICurve2D> segments = new List<ICurve2D>();

                        foreach (var seg in segsEl.EnumerateArray())
                        {
                            string segKind = RequireString(seg, "kind");
                            var segParams = RequireProperty(seg, "params");

                            // recursive call
                            throw new NotImplementedException();
                            //ICurve2D? sub = CreateCurveFromKind(segKind, segParams);
                            //if (sub == null)
                            //    throw new JsonRpcException(-32602, $"Unsupported segment kind '{segKind}'");

                            //segments.Add(sub);
                        }

                        curve = new Path2D(segments.ToArray());
                        break;
                    }

                case "rectangle":
                    {
                        double width = RequireLength(entityParams, "width");
                        double height = RequireLength(entityParams, "height");
                        double cornerRadius = RequireLength(entityParams, "cornerRadius");
                        GeoPoint2D center = RequirePoint2D(entityParams, "center");
                        double rotation = GetOptionalLength(entityParams, "rotation", 0.0);

                        if (cornerRadius == 0)
                            curve = Polyline2D.MakeRectangle(center, width, height, 0.0);
                        else
                            curve = Path2D.CreateRoundedRectangle(center, width, height, cornerRadius, SweepAngle.Deg(rotation));

                        break;
                    }

                default:
                    throw new JsonRpcException(-32601, $"Unknown sketch entity kind '{kind}'");
            }

            if (curve != null)
            {
                if (name != null)
                    namedItems[name] = curve;

                sketch.Add(curve);
            }
        }

        private void SketchBooleanImpl(Sketch sketch, string op, JsonElement inputs, JsonElement subtract, string? name)
        {
            CompoundShape? result = null;
            List<CompoundShape> inputshapes = new List<CompoundShape>();
            if (inputs.ValueKind == JsonValueKind.Array)
            {
                foreach (var inp in inputs.EnumerateArray())
                {
                    object obj = ResolveObjectRef(inp);
                    if (obj != null)
                    {
                        if (obj is ICurve2D c2d && c2d.IsClosed)
                        {
                            Border bdr = new Border(c2d);
                            inputshapes.Add(new CompoundShape(new SimpleShape(new Border(c2d))));
                        }
                        else if (obj is CompoundShape cs)
                        {
                            inputshapes.Add(cs);
                        }
                        else throw new JsonRpcException(-32602, "All boolean inputs must be closed shapes.");
                    }
                    else throw new JsonRpcException(-32602, "All boolean inputs must be sketch shapes.");
                }
            }
            List<CompoundShape> subtractshapes = new List<CompoundShape>();
            if (subtract.ValueKind == JsonValueKind.Array)
            {
                foreach (var inp in subtract.EnumerateArray())
                {
                    object obj = ResolveObjectRef(inp);
                    if (obj != null)
                    {
                        if (obj is ICurve2D c2d && c2d.IsClosed)
                        {
                            Border bdr = new Border(c2d);
                            subtractshapes.Add(new CompoundShape(new SimpleShape(new Border(c2d))));
                        }
                        else if (obj is CompoundShape cs)
                        {
                            subtractshapes.Add(cs);
                        }
                        else throw new JsonRpcException(-32602, "All boolean inputs must be closed shapes.");
                    }
                    else throw new JsonRpcException(-32602, "All boolean inputs must be sketch shapes.");
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


        private void SketchConnectImpl(Sketch sketch, JsonElement entities, JsonElement precision, bool closeGaps, string name)
        {
            throw new NotImplementedException();
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

        private void SolidExtrudeImpl(object profile, JsonElement length, JsonElement direction, string? name, JsonElement offset, JsonElement pitch, JsonElement capture)
        {
            Sketch? sketch = null;
            List<SimpleShape> simpleShapes = new List<SimpleShape>();
            if (profile is CompoundShape cs)
            {
                sketch = cs.UserData["MCPServer.Sketch"] as Sketch;
                simpleShapes.AddRange(cs.SimpleShapes);
            }
            else if (profile is List<CompoundShape> cslist)
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
            else if (profile is ICurve2D c2d && c2d.IsClosed)
            {
                if (sketch == null) sketch = c2d.UserData["MCPServer.Sketch"] as Sketch;
                simpleShapes.Add(new SimpleShape(new Border(c2d)));
            }
            else throw new JsonRpcException(-32602, "Profile must be a sketch shape.");
            if (sketch != null)
            {
                string? startEdges = GetOptionalString(capture, "startEdges");
                string? endEdges = GetOptionalString(capture, "endEdges");
                string? startFace = GetOptionalString(capture, "startFace");
                string? endFace = GetOptionalString(capture, "endFace");
                List<Solid> solids = new List<Solid>();
                PlaneSurface ps = new PlaneSurface(sketch.Plane);
                double l = RequireLength(length, null);
                GeoVector dir = l * ps.Normal.Normalized;
                for (int i = 0; i < simpleShapes.Count; i++)
                {
                    Face face = Face.MakeFace(ps, simpleShapes[i]);
                    if (face != null)
                    {
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
                        double sizeX = RequireLength(sparams, "sizeX");
                        double sizeY = RequireLength(sparams, "sizeY");
                        double sizeZ = RequireLength(sparams, "sizeZ");
                        res = Make3D.MakeBox(origin, sizeX * axisX, sizeY * axisY, sizeZ * axisZ);
                    }
                    break;
                case "sphere":
                    {
                        GeoPoint center = RequirePoint3D(sparams, "center");
                        double radius = RequireLength(sparams, "radius");
                        res = Make3D.MakeSphere(center, radius);
                    }
                    break;
                case "cylinder":
                    {
                        GeoPoint start = RequirePoint3D(sparams, "start");
                        GeoPoint end = RequirePoint3D(sparams, "end");
                        double radius = RequireLength(sparams, "radius");
                        Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
                        GeoVector dirx = radius * pln.ToGlobal(GeoVector2D.XAxis);
                        res = Make3D.MakeCylinder(start, dirx, end - start);
                    }
                    break;
                case "cone":
                    {
                        GeoPoint start = RequirePoint3D(sparams, "start");
                        GeoPoint end = RequirePoint3D(sparams, "end");
                        double radiusStart = RequireLength(sparams, "radiusStart");
                        double radiusEnd = RequireLength(sparams, "radiusEnd");
                        Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
                        GeoVector dirx = pln.ToGlobal(GeoVector2D.XAxis);
                        res = Make3D.MakeCone(start, dirx, end - start, radiusStart, radiusEnd);
                    }
                    break;
                case "torus":
                    {
                        GeoPoint center = RequirePoint3D(sparams, "center");
                        GeoVector axis = RequireVector3D(sparams, "axis");
                        double majorRadius = RequireLength(sparams, "majorRadius");
                        double minorRadius = RequireLength(sparams, "minorRadius");
                        Plane pln = new Plane(center, axis); // to use the arbitrary axis algorithm
                        res = Make3D.MakeTorus(center, axis, majorRadius, minorRadius);
                    }
                    break;
                case "capsule":
                    {
                        GeoPoint start = RequirePoint3D(sparams, "start");
                        GeoPoint end = RequirePoint3D(sparams, "end");
                        double radius = RequireLength(sparams, "radius");
                        Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
                        GeoVector dirx = radius * pln.ToGlobal(GeoVector2D.XAxis);
                        Solid cylinder = Make3D.MakeCylinder(start, dirx, end - start);
                        string cap = RequireString(sparams, "cap");
                        double coneTipDistance = GetOptionalLength(sparams, "coneTipDistance", double.MinValue);
                        if (coneTipDistance == double.MinValue)
                        {
                            // sphericalTips
                            Solid sphere1 = Make3D.MakeSphere(start, radius);
                            Solid sphere2 = Make3D.MakeSphere(end, radius);
                            res = BooleanOperation.Unite(sphere1, cylinder);
                            res = BooleanOperation.Unite(sphere2, res);
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
                        double outerRadius = RequireLength(sparams, "outerRadius");
                        double innerRadius = RequireLength(sparams, "innerRadius");
                        Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
                        GeoVector dirx = outerRadius * pln.ToGlobal(GeoVector2D.XAxis);
                        Solid cylinder1 = Make3D.MakeCylinder(start, dirx, end - start);
                        dirx = innerRadius * pln.ToGlobal(GeoVector2D.XAxis);
                        Solid cylinder2 = Make3D.MakeCylinder(start, dirx, end - start);
                        Solid[] diff = BooleanOperation.Subtract(cylinder1,cylinder2);
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


        private void ToolsetGetInfoImpl()
        {
            throw new NotImplementedException();
        }


        private object SolidSweepImpl(object profile, object path, string? orientation, string? name, JsonElement capture) => throw new NotImplementedException();
        private object SolidRotateImpl(object profile, JsonElement axis, JsonElement angle, string? name, JsonElement capture) => throw new NotImplementedException();

        private void BooleanUnionImpl(JsonElement solids, string? name, bool rebind, JsonElement rebindTargets)
        {
        }
        private void BooleanSubtractImpl(object target, JsonElement tools, string? name, bool rebind, JsonElement rebindTargets)
        {

        }
        private void BooleanIntersectImpl(object a, object b, string? name, bool rebind, JsonElement rebindTargets)
        {

        }

        private void TopologyFindFacesImpl(object solid, JsonElement filter, int limit, string? name)
        {

        }
        private JsonNode TopologyPickFaceImpl(object solid, JsonElement nearPoint, JsonElement filter, string? name) => throw new NotImplementedException();
        private JsonNode TopologyFindEdgesImpl(object solid, JsonElement filter, int limit, string? name) => throw new NotImplementedException();

        private JsonNode PatternInsetRectanglePointsImpl(object face, JsonElement insetX, JsonElement insetY, string mode) => throw new NotImplementedException();
        private void PatternCircularEntitiesImpl(Sketch sketch, JsonElement entities, JsonElement center, int count, JsonElement angle, bool merge, string name, bool nameWithSuffix)
        {
            // only implemented for closed shapes for now, which we convert to CompoundShape for easier boolean operations. We can add support for open curves later if needed.
            double a = RequireLength(angle, null);
            List<CompoundShape> inputshapes = new List<CompoundShape>();
            if (entities.ValueKind == JsonValueKind.Array)
            {
                foreach (var inp in entities.EnumerateArray())
                {
                    object obj = ResolveObjectRef(inp);
                    if (obj != null)
                    {
                        if (obj is ICurve2D c2d && c2d.IsClosed)
                        {
                            Border bdr = new Border(c2d);
                            inputshapes.Add(new CompoundShape(new SimpleShape(new Border(c2d))));
                        }
                        else if (obj is CompoundShape cs)
                        {
                            cs.UserData.Add("MCPServer.Sketch", sketch);
                            inputshapes.Add(cs);
                        }
                        else throw new JsonRpcException(-32602, "All inputs must be closed shapes.");
                    }
                    else throw new JsonRpcException(-32602, "All inputs must be sketch shapes.");
                }
            }
            List<CompoundShape> resultshapes = new List<CompoundShape>();
            SweepAngle angleStep = SweepAngle.Deg(a / count);
            ModOp2D rot = ModOp2D.Rotate(RequirePoint2D(center, null), angleStep);
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
        private void SketchRoundVerticesImpl(Sketch? sketch, JsonElement entity, JsonElement radius, JsonElement nearPoints, JsonElement indices, JsonElement tolerance, string name)
        {
            if (nearPoints.ValueKind != JsonValueKind.Undefined || indices.ValueKind != JsonValueKind.Undefined)
            {
                throw new NotImplementedException("Vertex selection for sketch.round_vertices not implemented.");
            }
            double r = RequireLength(radius, null);
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
                    Path2D rounded = p2d.RoundVertices(r);
                    if (rounded != null)
                    {
                        rounded.UserData.Add("MCPServer.Sketch", sketch);
                        if (string.IsNullOrEmpty(name) && currentName != null) namedItems[currentName] = rounded;
                        else if (name != null) namedItems[name] = rounded;
                    }
                }
                if (cs != null)
                {
                    CompoundShape rounded = cs.RoundVertices(r);
                    if (rounded != null)
                    {
                        rounded.UserData.Add("MCPServer.Sketch", sketch);
                        if (string.IsNullOrEmpty(name) && currentName != null) namedItems[currentName] = rounded;
                        else if (name != null) namedItems[name] = rounded;
                    }
                }
            }

        }
        private void SolidBooleanImpl(string op, object a, JsonElement b, string name, bool rebind, JsonElement rebindTargets)
        {
            throw new NotImplementedException();
        }

        private void PatternCircularObjectsImpl(JsonElement objects, JsonElement center, JsonElement axis, int count, JsonElement angle, bool copy, string name, bool nameWithSuffix)
        {
            throw new NotImplementedException();
        }

        private void PatternGridEntitiesImpl(Sketch sketch, JsonElement entities, int countX, int countY, JsonElement stepX, JsonElement stepY, bool merge, string name, bool nameWithSuffix)
        {
            throw new NotImplementedException();
        }

        private void PatternGridObjectsImpl(JsonElement objects, int countX, int countY, JsonElement stepX, JsonElement stepY, bool copy, string namePrefix)
        {
            throw new NotImplementedException();
        }

        private object FeatureHoleImpl(object solid, object face, JsonElement centerOnFace, JsonElement center, JsonElement diameter, bool through, JsonElement depth, string? name, JsonElement capture, bool rebind, JsonElement rebindTargets) => throw new NotImplementedException();
        private void FeatureFilletEdgesImpl(object solid, JsonElement edges, JsonElement radius, string? name, bool rebind, JsonElement rebindTargets)
        {
            double r = RequireLength(radius, null);
            List<Edge> edgesToRound = EdgesFromEdgeSelector(edges);
            if (edgesToRound.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "No edges found to fillet.");
            Shell? shell = edgesToRound.First().Owner.Owner as Shell;
            if (shell == null) throw new JsonRpcException("E_INVALID_PARAMS", "Edge is not part of a solid.");
            RoundEdges re = new RoundEdges(shell, edgesToRound, r);
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
                foreach (object o in IterateObjectRefs(notOnFaceEl))
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
                foreach (object o in IterateObjectRefs(onFaceEl))
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
                double min = GetOptionalNumber(lengthEl, "min", double.MinValue);
                double max = GetOptionalNumber(lengthEl, "max", double.MaxValue);
                foreach (Edge edge in res.Clone())
                {
                    if (edge.Curve3D.Length < min || edge.Curve3D.Length > max) res.Remove(edge);
                }
            }
            return res;
        }

        private void DocCommitObjectsImpl(JsonElement objects)
        {
            foreach (var inp in objects.EnumerateArray())
            {
                foreach (object obj in IterateObjectRefs(inp))
                {
                    if (obj is Solid sld)
                    {
                        FrameImpl.MainFrame?.Project?.GetActiveModel()?.Add(sld);
                    }
                    else throw new JsonRpcException(-32602, "Object to commit must be a solid.");
                }
            }

        }
        private JsonNode TransformScaleImpl(JsonElement objectsEl, JsonElement centerEl, JsonElement factorEl, JsonElement factorsEl, bool copy, string? namePrefix, bool copyNamed, string? nameSuffix, string? nameConflict)
        {
            throw new NotImplementedException();
        }
        private JsonNode TransformReflectImpl(JsonElement objectsEl, JsonElement planeEl, bool copy, string? namePrefix, bool copyNamed, string? nameSuffix, string? nameConflict)
        {
            PlaneFromJson(planeEl);
            throw new NotImplementedException();
        }

        private void TransformRotateImpl(JsonElement objectsEl, JsonElement axisEl, JsonElement angleEl, bool copy, string? namePrefix, bool copyNamed, string? nameSuffix, string? nameConflict)
        {
            throw new NotImplementedException();
        }
        private void TransformMoveImpl(JsonElement objects, JsonElement delta, bool copy, string namePrefix, bool copyNamed, string nameSuffix, string nameConflict)
        {
            throw new NotImplementedException();
        }



        private Plane PlaneFromJson(JsonElement planeRef)
        {
            // PlaneRef can be either {standard:"XY"|"YZ"|"XZ"} or {origin:{x,y,z}, normal:{x,y,z}, xAxis?:{x,y,z}}
            if (planeRef.TryGetProperty("standard", out JsonElement stdEl) && stdEl.ValueKind == JsonValueKind.String)
            {
                string std = stdEl.GetString() ?? "XY";
                return std switch
                {
                    "XY" => Plane.XYPlane,
                    "YZ" => Plane.YZPlane,
                    "XZ" => Plane.XZPlane,
                    _ => throw new JsonRpcException("E_INVALID_PARAMS", "Unknown standard plane.")
                };
            }

            if (planeRef.TryGetProperty("origin", out JsonElement orgEl) && planeRef.TryGetProperty("xAxis", out JsonElement xEl))
            {
                GeoPoint org = ReadPoint3(orgEl);
                GeoVector dirx = ReadVec3(xEl);
                //if (dirx == null) throw new JsonRpcException("E_INVALID_PARAMS", "Invalid plane xAxis.");
                GeoVector diry = GeoVector.Invalid;
                if (planeRef.TryGetProperty("yAxis", out JsonElement yEl))
                {
                    diry = ReadVec3(yEl);
                }
                else if (planeRef.TryGetProperty("normal", out JsonElement nEl))
                {
                    diry = ReadVec3(nEl) ^ dirx;
                }
                try
                {
                    return new Plane(org, dirx, diry);
                }
                catch (PlaneException ex)
                {
                    throw new JsonRpcException("E_INVALID_PARAMS", "Invalid plane: " + ex.Message);
                }
            }
            throw new JsonRpcException("E_INVALID_PARAMS", "Invalid PlaneRef.");
        }


        private List<Profile> ProfileFromSketchImpl(Sketch sketch, JsonElement parameters)
        {
            throw new NotImplementedException();
        }

        private Solid SolidExtrudeImpl(object profile, double? length, string? lengthExpr, Vector3? direction, string? operation)
        {
            throw new NotImplementedException();
        }

        private List<Face> TopologyFindFacesImpl(Solid solid, JsonElement filter, int limit)
        {
            throw new NotImplementedException();
        }

        private List<GeoPoint> PatternInsetRectanglePointsImpl(Face face, string? insetXExpr, string? insetYExpr, string mode)
        {
            throw new NotImplementedException();
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