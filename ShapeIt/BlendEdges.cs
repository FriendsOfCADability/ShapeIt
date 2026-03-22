using CADability;
using CADability.GeoObject;
using CADability.Substitutes;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeIt
{
    internal class BlendEdges
    {
        public Shell shell;
        public IEnumerable<Edge> convexEdges;
        public IEnumerable<Edge> concaveEdges;
        public Dictionary<Edge, Shell>? edgeToCutter;

        public BlendEdges(Shell shell, IEnumerable<Edge> edges)
        {
            this.shell = shell;
            // convex and concave: an edge is convex when the angle between the two faces is less than 108°, concave when greater than 180°
            convexEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Convex);
            concaveEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Concave);
        }

        public Dictionary<Vertex, List<Edge>> createVertexToEdges(IEnumerable<Edge> edges)
        {
            Dictionary<Vertex, List<Edge>> vertexToEdges = new Dictionary<Vertex, List<Edge>>();
            foreach (Edge edge in edges)
            {
                foreach (Vertex vertex in new List<Vertex>([edge.Vertex1, edge.Vertex2]))
                {
                    if (!vertexToEdges.TryGetValue(vertex, out List<Edge>? vedges)) vertexToEdges[vertex] = vedges = [];
                    vedges.Add(edge);
                }
            }
            return vertexToEdges;
        }
        private double minBeamDist(GeoPoint from, GeoVector direction)
        {
            GeoPoint[] ints = shell.GetLineIntersection(from, direction);
            double dist = double.MaxValue;
            for (int i = 0; i < ints.Length; i++)
            {
                double pos = Geometry.LinePar(from, direction, ints[i]);
                if (pos > Precision.eps && pos < dist)
                {
                    dist = pos;
                }
            }
            if (dist == double.MaxValue) dist = 0.0; // means no outside intersection
            return dist;
        }
        private Face MakeBigFace(ISurface surface)
        {   // extent the bounds of the surface at least double the area when possible and make a face to split something else with
            BoundingRect br = surface.GetBounds();
            double left = br.Left;
            double right = br.Right;
            if (surface.IsUPeriodic)
            {
                left = (br.Left + br.Right) / 2 - surface.UPeriod * 0.49;
                right = (br.Left + br.Right) / 2 + surface.UPeriod * 0.49;
            }
            else
            {
                left = br.Left - br.Width;
                right = br.Right + br.Width;
            }
            double bottom = br.Bottom;
            double top = br.Top;
            if (surface.IsVPeriodic)
            {
                bottom = (br.Bottom + br.Top) / 2 - surface.VPeriod * 0.49;
                top = (br.Bottom + br.Top) / 2 + surface.VPeriod * 0.49;
            }
            else
            {
                bottom = br.Bottom - br.Height;
                top = br.Top + br.Height;
            }
            return Face.MakeFace(surface, new BoundingRect(left, bottom, right, top));

        }
        protected HashSet<Shell>? createDeadEndExtension(Vertex vtx, Edge edge, double length)
        {   // rounding end here at vertex vtx. vtx and edge is on the shell to be rounded
            if (edgeToCutter == null) return null;
            Shell cutter = edgeToCutter[edge];
            Face? endFace = cutter.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            if (endFace == null) return [cutter]; // should not happen
            Edge freeEdge = endFace.AllEdges.First(e => !Precision.IsEqual(e.Vertex1.Position,vtx.Position) && !Precision.IsEqual(e.Vertex2.Position, vtx.Position)); // the chamfer or rounding edge
            if (freeEdge == null) return [cutter]; // should not happen

            HashSet<Face> endingFaces = []; // faces on the shell to be rounded where the edge ends
            // this is only one face in most cases. 
            foreach (Edge edg in vtx.Edges)
            {
                endingFaces.Add(edg.PrimaryFace);
                endingFaces.Add(edg.SecondaryFace);
            }
            endingFaces.Remove(edge.PrimaryFace);
            endingFaces.Remove(edge.SecondaryFace);
            // beamDirection: the direction where the fillet is pointing to
            GeoPoint2D uv = vtx.GetPositionOnFace(endFace);
            GeoVector beamDirection = endFace.Surface.GetNormal(uv).Normalized;
            Shell? extension = (Make3D.Extrude(endFace.Clone(), 3 * length * beamDirection, null) as Solid)?.Shells[0];
            bool useExtension = false;
            // extension of the cutter, maybe we need part of it
            foreach (Face fc in endingFaces)
            {
                ISurface surface = fc.Surface.Clone(); // with this surface we try to trim the cutter or the extension
                GeoPoint2D ip1 = surface.PositionOf(vtx.Position); // vtx is on surface
                if (surface.GetNormal(ip1) * beamDirection < 0) surface.ReverseOrientation(); // below is good, above is bad
                GeoPoint2D ip2 = surface.GetLineIntersection(freeEdge.Curve3D.StartPoint, beamDirection).MinByWithDefault(GeoPoint2D.Invalid, uv => surface.PointAt(uv) | vtx.Position);
                GeoPoint2D ip3 = surface.GetLineIntersection(freeEdge.Curve3D.EndPoint, beamDirection).MinByWithDefault(GeoPoint2D.Invalid, uv => surface.PointAt(uv) | vtx.Position);
                if (ip2.IsValid && ip3.IsValid)
                {
                    surface.SetBoundsTo(vtx.Position, surface.PointAt(ip2), surface.PointAt(ip3));
                    Face splitWith = MakeBigFace(surface);
                    (Shell[] upperPart, Shell[] lowerPart) = BooleanOperation.SplitByFace(cutter, splitWith);
                    if (upperPart.Length > 0 && lowerPart.Length > 0)
                    {   // the ending face did split the cutter
                        cutter.UserData.Add("CADability.RepleceShellBy", lowerPart[0]);
                        cutter = lowerPart[0];
                        edgeToCutter[edge] = cutter; // overwrite existing
                    }
                    (upperPart, lowerPart) = BooleanOperation.SplitByFace(extension, splitWith);
                    if (upperPart.Length > 0 && lowerPart.Length > 0)
                    {   // the ending face did split the cutter
                        extension = lowerPart[0];
                        useExtension = true;
                    }
                }
            }
            if (useExtension) return [cutter, extension];
            else return [cutter];

        }
        protected HashSet<Shell>? createExtensionTwoEdges(Vertex vtx, Edge edge1, Edge edge2, double length)
        {
            Shell? fillet1 = edgeToCutter?[edge1];
            Shell? fillet2 = edgeToCutter?[edge2];
            if (fillet1 == null || fillet2 == null) return null;
            Face? commonFace = Edge.CommonFace(edge1, edge2);
            Edge? thirdEdge = vtx.AllEdges.Except([edge1, edge2]).TheOnlyOrDefault();
            // in most cases we have a vertex with three faces meeting, so one common face and one third edge
            // if there are more than three faces meeting at the vertex, we cannot handle this currently
            if (commonFace == null)
            {
                GeoVector dir1, dir2;
                if (edge1.Vertex1 == vtx) dir1 = edge1.Curve3D.StartDirection;
                else dir1 = edge1.Curve3D.EndDirection;
                if (edge2.Vertex1 == vtx) dir2 = edge2.Curve3D.StartDirection;
                else dir2 = edge2.Curve3D.EndDirection;
                if (Precision.SameDirection(dir1, dir2, false)) return [fillet1, fillet2]; // tangential connection, we need the two fillets without any connection patch in between
                return null; // there must be a common face other cases are not implemented yet 
            }
            if (commonFace == null) return null; // there must be a common face
            if (edge2.EndVertex(commonFace) == edge1.StartVertex(commonFace)) (edge1, edge2) = (edge2, edge1); // tm make sure, the edges are in the order of the outline
            System.Diagnostics.Debug.Assert(edge1.EndVertex(commonFace) == edge2.StartVertex(commonFace));

            Face? endFace1 = fillet1.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            Face? endFace2 = fillet2.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            if (endFace1 == null || endFace2 == null) return null;
            GeoVector n1 = (endFace1.Surface as PlaneSurface)!.Normal.Normalized; // endfaces are always PlaneSurfaces
            GeoVector n2 = (endFace2.Surface as PlaneSurface)!.Normal.Normalized;

            SweepAngle sw = new SweepAngle(edge1.Curve2D(commonFace).EndDirection, edge2.Curve2D(commonFace).StartDirection);
            if (Math.Abs(sw) < 1e-5)
            {   // tangential connection, we need the two fillets without any connection patch in between
                return [fillet1, fillet2];
            }
            if (sw > 0)
            {
                // convex connection, we have to extent the fillets at this point
                Shell? extension1 = (Make3D.Extrude(endFace1.Clone(), length * n1, null) as Solid)?.Shells[0];
                Shell? extension2 = (Make3D.Extrude(endFace2.Clone(), length * n2, null) as Solid)?.Shells[0];
                if (extension1 != null && extension2 != null)
                {
                    extension1.CopyAttributes(edge1.PrimaryFace);
                    extension2.CopyAttributes(edge2.PrimaryFace);
                    return [fillet1, fillet2, extension1, extension2];
                }
            }
            else
            {
                Shell? extensionPatch = CreateConcavePatch(fillet1, fillet2, commonFace, vtx, edge1, edge2);
                if (extensionPatch != null)
                {
                    extensionPatch.CopyAttributes(edge1.PrimaryFace);
                    return [fillet1, fillet2, extensionPatch];
                }
            }
            return [fillet1, fillet2];

        }
        protected virtual Shell? CreateConcavePatch(Shell chamfer1, Shell chamfer2, Face commonFace, Vertex vtx, Edge edge1, Edge edge2)
        {
            Face? endFace1 = chamfer1.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            Face? endFace2 = chamfer2.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            if (endFace1 == null || endFace2 == null) return null;
            List<(Vertex v1, Vertex v2)> cv = ConnectedVertices(endFace1.Vertices, endFace2.Vertices);
            if (cv.Count == 2)
            {
                Vertex? v3Fillet1 = endFace1.Vertices.Except([cv[0].v1, cv[1].v1]).TheOnlyOrDefault();
                Vertex? v3Fillet2 = endFace2.Vertices.Except([cv[0].v2, cv[1].v2]).TheOnlyOrDefault();
                if (v3Fillet1 != null && v3Fillet2 != null)
                {
                    Axis axis = new Axis(cv[0].v1.Position, cv[1].v1.Position);
                    Plane pln = new Plane(axis.Location, axis.Direction);
                    SweepAngle sw = new SweepAngle(pln.Project(v3Fillet1.Position) - GeoPoint2D.Origin, pln.Project(v3Fillet2.Position) - GeoPoint2D.Origin);
                    IGeoObject sld = Make3D.Rotate(endFace1, axis, sw, 0, null);
                    if (sld is Solid s) return s.Shells[0];
                }
            }
            return null;
        }

        protected void Combine(List<HashSet<Shell>> sets)
        {
            // some cutter shells may have been splitted and we have the unsplitted part in the set, so replace it
            for (int i = 0; i < sets.Count; i++)
            {
                foreach (Shell shell in sets[i].Clone())
                {
                    Shell? replaceWith = shell.UserData["CADability.RepleceShellBy"] as Shell;
                    if (replaceWith != null)
                    {
                        sets[i].Remove(shell);
                        sets[i].Add(replaceWith);
                    }
                }

            }
            bool mergedSomething;
            do
            {
                mergedSomething = false;

                for (int i = 0; i < sets.Count; i++)
                {
                    for (int j = i + 1; j < sets.Count; j++)
                    {
                        if (sets[i].Overlaps(sets[j]))
                        {
                            sets[i].UnionWith(sets[j]);
                            sets.RemoveAt(j);
                            mergedSomething = true;
                            break;
                        }
                    }
                    if (mergedSomething) break;
                }
            }
            while (mergedSomething);
        }

        protected void TrimCurve(ICurve curve, GeoPoint startPoint, GeoPoint endPoint)
        {
            double pos1 = curve.PositionOf(startPoint);
            double pos2 = curve.PositionOf(endPoint);
            if (pos1 > pos2)
            {
                curve.Reverse();
                pos1 = 1 - pos1;
                pos2 = 1 - pos2;
            }
            if (Math.Abs(pos1) > 1e-6 || Math.Abs(1 - pos2) > 1e-6)
            {
                curve.Trim(pos1, pos2);
            }

        }
        public static List<(Vertex, Vertex)> ConnectedVertices(IEnumerable<Vertex> v1, IEnumerable<Vertex> v2)
        {
            // Result list containing matching vertex pairs
            var result = new List<(Vertex, Vertex)>();

            // Brute-force comparison of all pairs
            foreach (var a in v1)
            {
                foreach (var b in v2)
                {
                    // Check if positions are equal using given precision logic
                    if (Precision.IsEqual(a.Position, b.Position))
                    {
                        result.Add((a, b));
                    }
                }
            }

            return result;
        }

        protected void AppendLookup(Dictionary<Face, Face> start, Dictionary<Face, Face> append)
        {
            foreach (var kv in start.ToList()) // ToList(), weil wir d1 verändern
            {
                if (append.TryGetValue(kv.Value, out var a))
                {
                    start[kv.Key] = a;
                }
                else
                {
                    // leave unchanged
                }
            }
        }
        protected void AppendLookup(Dictionary<Face, Face> start, Dictionary<Face, List<Face>> append)
        {
            foreach (var kv in start.ToList()) // ToList(), weil wir d1 verändern
            {
                if (append.TryGetValue(kv.Value, out var a))
                {   // in most cases, there is only one face in the list: the face, which has been trimmed
                    // sometimes faces have been split into two or more faces. Then we don't have a good solution anyhow
                    start[kv.Key] = a.First();
                }
                else
                {
                    // leave unchanged
                }
            }
        }
        protected void Lookup(Dictionary<Edge, (Face face, bool forward)> edgeToFace, Dictionary<Face, Face> lookup)
        {
            foreach (var kv in edgeToFace.ToList())
            {
                if (lookup.TryGetValue(kv.Value.face, out Face? lookedup)) edgeToFace[kv.Key] = (lookedup, kv.Value.forward);
            }
        }
        protected void Lookup(Dictionary<Edge, HashSet<Face>> edgeToFaces, Dictionary<Face, Face> lookup)
        {
            foreach (var kv in edgeToFaces.ToList())
            {
                HashSet<Face> faces = [];
                foreach (var item in kv.Value)
                {
                    if (lookup.TryGetValue(item, out Face? found)) faces.Add(found);
                }
                edgeToFaces[kv.Key] = faces;
            }
        }
    }
}
