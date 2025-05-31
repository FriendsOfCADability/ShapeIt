using CADability.GeoObject;
using CADability;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.CompilerServices;
using CADability.Curve2D;
using CADability.Shapes;
using Wintellect.PowerCollections;

namespace ShapeIt
{
    internal static class ShellExtensions
    {
        public static int GetFaceDistances(this Shell shell, Face distanceFrom, GeoPoint touchingPoint, out List<Face> distanceTo, out List<double> distance, out List<GeoPoint> pointsFrom, out List<GeoPoint> pointsTo)
        {
            distanceTo = new List<Face>();
            distance = new List<double>();
            pointsFrom = new List<GeoPoint>();
            pointsTo = new List<GeoPoint>();
            foreach (Face face in shell.Faces)
            {
                if (face == distanceFrom) continue;
                if (Surfaces.ParallelDistance(distanceFrom.Surface, distanceFrom.Domain, face.Surface, face.Domain, touchingPoint, out GeoPoint2D uv1, out GeoPoint2D uv2))
                {
                    GeoPoint pFrom = distanceFrom.Surface.PointAt(uv1);
                    GeoPoint pTo = face.Surface.PointAt(uv2);
                    double dist = pFrom | pTo;
                    if (dist > Precision.eps)
                    {
                        distanceTo.Add(face);
                        distance.Add(dist);
                        pointsFrom.Add(pFrom);
                        pointsTo.Add(pTo);
                    }
                }
            }
            return distanceTo.Count;
        }
        public enum AdjacencyType
        {
            Unknown,
            Open,
            SameSurface,
            Tangent,
            Convex,
            Concave,
            Mixed
        }
        public static AdjacencyType Adjacency(this Edge edge)
        {
            if (edge.SecondaryFace != null)
            {
                if (edge.PrimaryFace.Surface.SameGeometry(edge.PrimaryFace.Domain, edge.SecondaryFace.Surface, edge.SecondaryFace.Domain, Precision.eps, out ModOp2D _)) return AdjacencyType.SameSurface;
                if (edge.IsTangentialEdge()) return AdjacencyType.Tangent;
                // there should be a test for "mixed"
                Vertex v1 = edge.StartVertex(edge.PrimaryFace);
                GeoPoint2D uvp = v1.GetPositionOnFace(edge.PrimaryFace);
                GeoPoint2D uvs = v1.GetPositionOnFace(edge.SecondaryFace);
                GeoVector curveDir;
                if (edge.Forward(edge.PrimaryFace)) curveDir = edge.Curve3D.StartDirection;
                else curveDir = -edge.Curve3D.EndDirection;
                double orientation = curveDir * (edge.PrimaryFace.Surface.GetNormal(uvp) ^ edge.SecondaryFace.Surface.GetNormal(uvs));
                if (orientation > 0) return AdjacencyType.Convex;
                else return AdjacencyType.Concave;
            }
            else
            {
                return AdjacencyType.Open;
            }
        }
        public static bool AllEdgesAreConvex(this Face face, bool reverse)
        {
            AdjacencyType toFollow = reverse ? AdjacencyType.Concave : AdjacencyType.Convex;
            foreach (Edge edge in face.AllEdges)
            {
                if (edge.Adjacency() != AdjacencyType.Tangent && edge.Adjacency() != toFollow) return false;
            }
            return true;
        }
        public static List<List<Face>> GetConvexParts(IEnumerable<Face> faces, bool reverse)
        {
            // HashSet<Face> faces = new HashSet<Face>(shell.Faces);
            var visited = new HashSet<Face>();
            var result = new List<List<Face>>();

            AdjacencyType toFollow = reverse ? AdjacencyType.Concave : AdjacencyType.Convex;

            foreach (var face in faces)
            {
                if (visited.Contains(face))
                    continue;

                // start a new region
                var region = new List<Face>();
                var queue = new Queue<Face>();
                queue.Enqueue(face);
                visited.Add(face);

                while (queue.Count > 0)
                {
                    var current = queue.Dequeue();
                    region.Add(current);

                    foreach (var edge in current.Edges)
                    {
                        // only respect convex or tangential connections
                        if (edge.Adjacency() != toFollow &&
                            edge.Adjacency() != AdjacencyType.Tangent)
                            continue;

                        Face neighbor = edge.OtherFace(current);

                        if (neighbor == null || visited.Contains(neighbor))
                            continue;

                        if (faces.Contains(neighbor))
                        {
                            queue.Enqueue(neighbor);
                            visited.Add(neighbor);
                        }
                    }
                }

                result.Add(region);
            }

            return result;
        }
        /// <summary>
        /// Make a fillet like face, which fills the gap of two offset faces at the provided <paramref name="axis"/>
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="radius"></param>
        /// <param name="forward"></param>
        /// <param name="backward"></param>
        /// <returns></returns>
        public static Face MakeOffsetFillet(Edge axis, double radius, Edge forward, Edge backward)
        {
            ISurface surface = null;
            GeoVector toInside;
            if (axis.Curve3D is Line line)
            {
                if (axis.Forward(axis.PrimaryFace))
                {
                    Vertex v = axis.StartVertex(axis.PrimaryFace);
                    GeoPoint2D uvp = v.GetPositionOnFace(axis.PrimaryFace);
                    GeoPoint2D uvs = v.GetPositionOnFace(axis.SecondaryFace);
                    toInside = -(axis.PrimaryFace.Surface.GetNormal(uvp).Normalized + axis.SecondaryFace.Surface.GetNormal(uvs).Normalized); // points away from the edge
                    GeoVector dir = line.StartDirection;
                    GeoVector majorAxis = radius * toInside.Normalized;
                    GeoVector minorAxis = radius * (dir ^ majorAxis).Normalized;
                    surface = new CylindricalSurface(v.Position, majorAxis, minorAxis, dir);
                }
                else
                {
                    Vertex v = axis.EndVertex(axis.PrimaryFace);
                    GeoPoint2D uvp = v.GetPositionOnFace(axis.PrimaryFace);
                    GeoPoint2D uvs = v.GetPositionOnFace(axis.SecondaryFace);
                    toInside = -(axis.PrimaryFace.Surface.GetNormal(uvp).Normalized + axis.SecondaryFace.Surface.GetNormal(uvs).Normalized); // points away from the edge
                    GeoVector dir = -line.EndDirection;
                    GeoVector majorAxis = radius * toInside.Normalized;
                    GeoVector minorAxis = radius * (dir ^ majorAxis).Normalized;
                    surface = new CylindricalSurface(v.Position, majorAxis, minorAxis, dir);
                }
            }
            else if (axis.Curve3D is Ellipse bigCircle)
            {
                if (axis.Forward(axis.PrimaryFace))
                {
                    Vertex v = axis.StartVertex(axis.PrimaryFace);
                    GeoVector dirx = (v.Position - bigCircle.Center).Normalized;
                    GeoVector dirz = (bigCircle.SweepParameter > 0) ? bigCircle.Plane.Normal : -bigCircle.Plane.Normal;
                    GeoVector diry = dirz ^ dirx; // still to test
                    surface = new ToroidalSurface(bigCircle.Center, dirx, diry, dirz, bigCircle.MajorRadius, radius);
                }
                else
                {
                    Vertex v = axis.EndVertex(axis.PrimaryFace);
                    GeoVector dirx = -(v.Position - bigCircle.Center).Normalized;
                    GeoVector dirz = (bigCircle.SweepParameter > 0) ? bigCircle.Plane.Normal : -bigCircle.Plane.Normal;
                    GeoVector diry = dirz ^ dirx; // still to test
                    surface = new ToroidalSurface(bigCircle.Center, dirx, diry, dirz, bigCircle.MajorRadius, radius);
                }
            }
            else
            {
                // make a NURBS surface defined by a certain number of circles
            }
            if (surface != null)
            {
                BoundingRect domain = new BoundingRect(surface.PositionOf(forward.Vertex1.Position));
                GeoPoint2D uv = surface.PositionOf(forward.Vertex2.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);
                uv = surface.PositionOf(backward.Vertex1.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);
                uv = surface.PositionOf(backward.Vertex2.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);

                // we need 4 edges to make this face. Two edges are already provided as parameters, the other two edges are arcs at the start- and endpoint of the axis
                GeoPoint center = axis.Vertex1.Position; // this is the startpoint of axis.Curve3D
                GeoVector toOutside = (axis.PrimaryFace.Surface.GetNormal(axis.Vertex1.GetPositionOnFace(axis.PrimaryFace)).Normalized +
                             axis.SecondaryFace.Surface.GetNormal(axis.Vertex1.GetPositionOnFace(axis.SecondaryFace)).Normalized);
                Plane pln = new Plane(center, -toOutside, toOutside ^ axis.Curve3D.StartDirection);
                Ellipse elli1 = Ellipse.Construct();
                Vertex v1 = Math.Abs(pln.Distance(forward.Vertex1.Position)) < Math.Abs(pln.Distance(forward.Vertex2.Position)) ? forward.Vertex1 : forward.Vertex2;
                Vertex v2 = Math.Abs(pln.Distance(backward.Vertex1.Position)) < Math.Abs(pln.Distance(backward.Vertex2.Position)) ? backward.Vertex1 : backward.Vertex2;
                elli1.SetArc3Points(v1.Position, center + radius * toOutside.Normalized, v2.Position, pln);

                center = axis.Vertex2.Position; // this is the endpoint of axis.Curve3D
                toOutside = (axis.PrimaryFace.Surface.GetNormal(axis.Vertex2.GetPositionOnFace(axis.PrimaryFace)).Normalized +
                             axis.SecondaryFace.Surface.GetNormal(axis.Vertex2.GetPositionOnFace(axis.SecondaryFace)).Normalized);
                pln = new Plane(center, -toOutside, toOutside ^ axis.Curve3D.EndDirection);
                Ellipse elli2 = Ellipse.Construct();
                v1 = forward.Vertex1 == v1 ? forward.Vertex2 : forward.Vertex1;
                v2 = backward.Vertex1 == v2 ? backward.Vertex2 : backward.Vertex1;
                elli2.SetArc3Points(v1.Position, center + radius * toOutside.Normalized, v2.Position, pln);
                ICurve2D[] curve2Ds = new ICurve2D[4];
                curve2Ds[0] = surface.GetProjectedCurve(forward.Curve3D, Precision.eps);
                curve2Ds[2] = surface.GetProjectedCurve(backward.Curve3D, Precision.eps);
                curve2Ds[1] = surface.GetProjectedCurve(elli1, Precision.eps);
                curve2Ds[3] = surface.GetProjectedCurve(elli2, Precision.eps);
                for (int i = 0; i < 4; ++i) SurfaceHelper.AdjustPeriodic(surface, domain, curve2Ds[i]);
                SimpleShape ss = new SimpleShape(Border.FromUnorientedList(curve2Ds, true));
                return Face.MakeFace(surface, ss);
            }
            return null;
        }
        public static Shell[] GetOffsetNew(this Shell shell, double offset)
        {
            List<Shell> result = new List<Shell>();
            List<Shell> blocks = new List<Shell>(); // a "block" is the perpendicular extrusion of each face, which sits exactely on the face
            List<Shell> wedges = new List<Shell>(); // a "wedge" is the wedge which fills the gap between adjacen blocks
            List<Shell> sphericalWedges = new List<Shell>(); // a "spherical wedge" is "hand axe" like solid, which fills the gap on a convex vertex 
            List<Face> faces = new List<Face>(); // the faces which are equidistant to the shell. Not clipped, not sure we need them
            Dictionary<(Face, Edge), Edge> faceEdgeToParallelEdge = new Dictionary<(Face, Edge), Edge>(); // the parallel edges to the original edges, also depend on the face
            Dictionary<(Face, Vertex), Edge> faceVertexToSphericalEdge = new Dictionary<(Face, Vertex), Edge>();
            Dictionary<(Face, Edge), Face> faceEdgeToSide = new Dictionary<(Face, Edge), Face>(); // the sides which stand perpendicular to the face on an edge
            foreach (Face face in shell.Faces)
            {   // makeparallel faces with the provided offset
                ISurface offsetSurface = face.Surface.GetOffsetSurface(offset);
                GeoPoint2D cnt = face.Domain.GetCenter();
                // if the orentation is reversed (e.g. a cylinder will have a negativ radius) or the surface disappears, don't use it
                if (offsetSurface != null && offsetSurface.GetNormal(cnt) * face.Surface.GetNormal(cnt) > 0)
                {
                    List<ICurve2D> outline = new List<ICurve2D>();
                    foreach (Edge edge in face.OutlineEdges)
                    {
                        if (offsetSurface is ConicalSurface) // or any other surface, where the u/v system of the offset differs from the u/v system of the original (which are those?)
                        {
                            ICurve2D c2d = offsetSurface.GetProjectedCurve(edge.Curve3D, Precision.eps);
                            if (c2d != null)
                            {
                                SurfaceHelper.AdjustPeriodic(offsetSurface, face.Domain, c2d);
                                outline.Add(c2d);
                                c2d.UserData.Add("CADability.CurveToEdge", edge);
                            }
                        }
                        else
                        {   // surfaces and offset surfaces usually have the same u/v system, i.e. 2d curves on both are parallel with the distance "offset"
                            ICurve2D c2d = edge.Curve2D(face).Clone();
                            outline.Add(c2d);
                            c2d.UserData.Add("CADability.CurveToEdge", edge);
                        }
                    }
                    Border outlineBorder = new Border(outline.ToArray(), true);
                    List<Border> holes = new List<Border>();
                    for (int i = 0; i < face.HoleCount; i++)
                    {
                        List<ICurve2D> hole = new List<ICurve2D>();
                        foreach (Edge edge in face.HoleEdges(i))
                        {
                            if (offsetSurface is ConicalSurface) // or any other surface, where the u/v system of the offset differs from the u/v system of the original (which are those?)
                            {
                                ICurve2D c2d = offsetSurface.GetProjectedCurve(edge.Curve3D, Precision.eps);
                                if (c2d != null)
                                {
                                    SurfaceHelper.AdjustPeriodic(offsetSurface, face.Domain, c2d);
                                    hole.Add(c2d);
                                    c2d.UserData.Add("CADability.CurveToEdge", edge);
                                }
                            }
                            else
                            {   // surfaces and offset surfaces usually have the same u/v system, i.e. 2d curves on both are parallel with the distance "offset"
                                ICurve2D c2d = edge.Curve2D(face).Clone();
                                hole.Add(c2d);
                                c2d.UserData.Add("CADability.CurveToEdge", edge);
                            }
                        }
                        Border holeBorder = new Border(hole.ToArray(), true);
                        holes.Add(holeBorder);
                    }
                    SimpleShape ss = new SimpleShape(outlineBorder, holes.ToArray());
                    Face offsetFace = Face.MakeFace(offsetSurface, ss);
                    faces.Add(offsetFace);
                    // we need a reference from edges of the original face to the edges of the paralell face.
                    foreach (Edge edg in offsetFace.Edges)
                    {
                        Edge originalEdge = edg.Curve2D(offsetFace).UserData["CADability.CurveToEdge"] as Edge;
                        if (originalEdge != null) faceEdgeToParallelEdge[(face, originalEdge)] = edg;
                        edg.Curve2D(offsetFace).UserData.Remove("CADability.CurveToEdge"); // no longer needed
                    }
                    // the offsetFace together with the reversed original face and the side faces, which are constructed by the parallel edges build a shell
                    // which has to be added to (united with) the original shell
                    List<Face> facesOfBlock = new List<Face>();
                    foreach (Edge edg in face.AllEdges)
                    {
                        ICurve c1 = edg.Curve3D.Clone();
                        if (!edg.Forward(face)) c1.Reverse();
                        ICurve c2 = faceEdgeToParallelEdge[(face, edg)].Curve3D.Clone();
                        ISurface surface = Make3D.MakeRuledSurface(c1, c2);
                        ICurve2D c12d = surface.GetProjectedCurve(c1, 0.0);
                        ICurve2D c22d = surface.GetProjectedCurve(c2, 0.0);
                        c22d.Reverse(); // c12d and c22d are parallel
                        Border border = new Border(new ICurve2D[] { c12d, new Line2D(c12d.EndPoint, c22d.StartPoint), c22d, new Line2D(c22d.EndPoint, c12d.StartPoint) });
                        Face side = Face.MakeFace(surface, new SimpleShape(border));
                        //GeoPoint2D uvstart = surface.PositionOf(c1.StartPoint);
                        //GeoPoint2D uvend = surface.PositionOf(c2.EndPoint);
                        //SurfaceHelper.AdjustPeriodic(surface, new BoundingRect(uvstart), ref uvend); // for cylindrical surfaces we need the shorter arc
                        //BoundingRect br = new BoundingRect(uvstart.x, uvstart.y, uvend.x, uvend.y);
                        //Face side = Face.MakeFace(surface, new SimpleShape(br.ToBorder()));
                        faceEdgeToSide[(face, edg)] = side; // remember to later make the wedges
                        facesOfBlock.Add(side);
                    }
                    Face baseFace = face.Clone() as Face;
                    baseFace.ReverseOrientation();
                    facesOfBlock.Add(baseFace);
                    facesOfBlock.Add(offsetFace);
                    Shell block = Shell.FromFaces(facesOfBlock.ToArray(), true);
                    if (block != null) blocks.Add(block);
                }
            }

            foreach (Edge sedge in shell.Edges)
            {
                if (sedge.Adjacency() == AdjacencyType.Convex && faceEdgeToParallelEdge.TryGetValue((sedge.PrimaryFace, sedge), out Edge e1) && faceEdgeToParallelEdge.TryGetValue((sedge.SecondaryFace, sedge), out Edge e2))
                {
                    Face fillet = MakeOffsetFillet(sedge, offset, e1, e2);
                    if (fillet != null)
                    {
                        faces.Add(fillet);
                        // create the "wedge" which fills the gap between the two blocks
                        Face side1 = faceEdgeToSide[(sedge.PrimaryFace, sedge)].Clone() as Face;
                        side1.ReverseOrientation();
                        Face side2 = faceEdgeToSide[(sedge.SecondaryFace, sedge)].Clone() as Face;
                        side2.ReverseOrientation();
                        Plane frontPlane = new Plane(sedge.Curve3D.StartPoint, -sedge.Curve3D.StartDirection);
                        Plane backPlane = new Plane(sedge.Curve3D.EndPoint, sedge.Curve3D.EndDirection);
                        GeoPoint f1 = Math.Abs(frontPlane.Distance(e1.Curve3D.StartPoint)) < Math.Abs(frontPlane.Distance(e1.Curve3D.EndPoint)) ? e1.Curve3D.StartPoint : e1.Curve3D.EndPoint;
                        GeoPoint f2 = Math.Abs(frontPlane.Distance(e2.Curve3D.StartPoint)) < Math.Abs(frontPlane.Distance(e2.Curve3D.EndPoint)) ? e2.Curve3D.StartPoint : e2.Curve3D.EndPoint;
                        GeoPoint b1 = Math.Abs(backPlane.Distance(e1.Curve3D.StartPoint)) < Math.Abs(backPlane.Distance(e1.Curve3D.EndPoint)) ? e1.Curve3D.StartPoint : e1.Curve3D.EndPoint;
                        GeoPoint b2 = Math.Abs(backPlane.Distance(e2.Curve3D.StartPoint)) < Math.Abs(backPlane.Distance(e2.Curve3D.EndPoint)) ? e2.Curve3D.StartPoint : e2.Curve3D.EndPoint;

                        Arc2D arc = new Arc2D(GeoPoint2D.Origin, offset, frontPlane.Project(f1), frontPlane.Project(f2), true);
                        if (arc.Sweep > Math.PI) arc = new Arc2D(GeoPoint2D.Origin, offset, frontPlane.Project(f2), frontPlane.Project(f1), true);
                        Border bdr = new Border(new ICurve2D[] { new Line2D(GeoPoint2D.Origin, arc.StartPoint), arc, new Line2D(arc.EndPoint, GeoPoint2D.Origin) });
                        Face frontFace = Face.MakeFace(new PlaneSurface(frontPlane), new SimpleShape(bdr));

                        arc = new Arc2D(GeoPoint2D.Origin, offset, backPlane.Project(b1), backPlane.Project(b2), true);
                        if (arc.Sweep > Math.PI) arc = new Arc2D(GeoPoint2D.Origin, offset, backPlane.Project(b2), backPlane.Project(b1), true);
                        bdr = new Border(new ICurve2D[] { new Line2D(GeoPoint2D.Origin, arc.StartPoint), arc, new Line2D(arc.EndPoint, GeoPoint2D.Origin) });
                        Face backFace = Face.MakeFace(new PlaneSurface(backPlane), new SimpleShape(bdr));

                        Shell wshell = Shell.FromFaces(new Face[] { fillet, frontFace, backFace, side1, side2 }, true);
                        wedges.Add(wshell);
                    }
                }
            }

            List<Shell> current = new List<Shell>();
            current.Add(shell);
            //foreach (Shell s in blocks.Concat(wedges))
            for (int j = 0; j < blocks.Count; j++)
            {
                Shell s = blocks[j];
                List<Shell> next = new List<Shell>();
                for (int i = 0; i < current.Count; i++)
                {
                    BRepOperation unite = new BRepOperation(current[i], s, BRepOperation.Operation.union);
                    Shell[] r = unite.Result();
                    if (r!=null) next.AddRange(r);
                }
                if (next.Count > 0) current = next;
            }
            for (int j = 0; j < wedges.Count; j++)
            {
                Shell s = wedges[j];
                List<Shell> next = new List<Shell>();
                for (int i = 0; i < current.Count; i++)
                {
                    BRepOperation unite = new BRepOperation(current[i], s, BRepOperation.Operation.union);
                    Shell[] r = unite.Result();
                    if (r != null) next.AddRange(r);
                }
                if (next.Count > 0) current = next;
            }

            return current.ToArray();
        }
        public static Shell[] GetOffsetNewX(this Shell shell, double offset)
        {
            List<Face> faces = new List<Face>(); // these faces make the row offset shell, which has to be clipped later
            Dictionary<(Face, Edge), Edge> faceEdgeToParallelEdge = new Dictionary<(Face, Edge), Edge>(); // the parallel edges to the original edges, also depend on the face
            foreach (Face face in shell.Faces)
            {   // makeparallel faces with the provided offset
                ISurface offsetSurface = face.Surface.GetOffsetSurface(offset);
                GeoPoint2D cnt = face.Domain.GetCenter();
                // if the orentation is reversed (e.g. a cylinder will have a negativ radius) or the surface disappears, don't use it
                if (offsetSurface != null && offsetSurface.GetNormal(cnt) * face.Surface.GetNormal(cnt) > 0)
                {
                    List<ICurve2D> outline = new List<ICurve2D>();
                    foreach (Edge edge in face.OutlineEdges)
                    {
                        if (offsetSurface is ConicalSurface) // or any other surface, where the u/v system of the offset differs from the u/v system of the original (which are those?)
                        {
                            ICurve2D c2d = offsetSurface.GetProjectedCurve(edge.Curve3D, Precision.eps);
                            if (c2d != null)
                            {
                                SurfaceHelper.AdjustPeriodic(offsetSurface, face.Domain, c2d);
                                outline.Add(c2d);
                                c2d.UserData.Add("CADability.CurveToEdge", edge);
                            }
                        }
                        else
                        {   // surfaces and offset surfaces usually have the same u/v system, i.e. 2d curves on both are parallel with the distance "offset"
                            ICurve2D c2d = edge.Curve2D(face).Clone();
                            outline.Add(c2d);
                            c2d.UserData.Add("CADability.CurveToEdge", edge);
                        }
                    }
                    Border outlineBorder = new Border(outline.ToArray(), true);
                    List<Border> holes = new List<Border>();
                    for (int i = 0; i < face.HoleCount; i++)
                    {
                        List<ICurve2D> hole = new List<ICurve2D>();
                        foreach (Edge edge in face.HoleEdges(i))
                        {
                            if (offsetSurface is ConicalSurface) // or any other surface, where the u/v system of the offset differs from the u/v system of the original (which are those?)
                            {
                                ICurve2D c2d = offsetSurface.GetProjectedCurve(edge.Curve3D, Precision.eps);
                                if (c2d != null)
                                {
                                    SurfaceHelper.AdjustPeriodic(offsetSurface, face.Domain, c2d);
                                    hole.Add(c2d);
                                    c2d.UserData.Add("CADability.CurveToEdge", edge);
                                }
                            }
                            else
                            {   // surfaces and offset surfaces usually have the same u/v system, i.e. 2d curves on both are parallel with the distance "offset"
                                ICurve2D c2d = edge.Curve2D(face).Clone();
                                hole.Add(c2d);
                                c2d.UserData.Add("CADability.CurveToEdge", edge);
                            }
                        }
                        Border holeBorder = new Border(hole.ToArray(), true);
                        holes.Add(holeBorder);
                    }
                    SimpleShape ss = new SimpleShape(outlineBorder, holes.ToArray());
                    Face offsetFace = Face.MakeFace(offsetSurface, ss);
                    faces.Add(offsetFace);
                    // we need a reference from edges of the original face to the edges of the paralell face.
                    foreach (Edge edg in offsetFace.Edges)
                    {
                        Edge originalEdge = edg.Curve2D(offsetFace).UserData["CADability.CurveToEdge"] as Edge;
                        if (originalEdge != null) faceEdgeToParallelEdge[(face, originalEdge)] = edg;
                        edg.Curve2D(offsetFace).UserData.Remove("CADability.CurveToEdge"); // no longer needed
                    }
                }
            }
            // for each convex edge make a rounded face between the two offset edges
            // this might be a cylinder, a torus or a swept arc
            foreach (Edge sedge in shell.Edges)
            {
                if (sedge.Adjacency() == AdjacencyType.Convex && faceEdgeToParallelEdge.TryGetValue((sedge.PrimaryFace, sedge), out Edge e1) && faceEdgeToParallelEdge.TryGetValue((sedge.SecondaryFace, sedge), out Edge e2))
                {
                    Face fillet = MakeOffsetFillet(sedge, offset, e1, e2);
                    faces.Add(fillet);
                }
            }
            // for each vertex, where all edges are convex we need a spherical face, which fills the gap between the fillets
            // TODO: implement
            foreach (Face fc in faces)
            {
                fc.ReverseOrientation();
            }
            // the faces in "faces" are unconnected, there are duplicate edges and vertices, which now will be connected
            Shell.ConnectFaces(faces.ToArray(), Precision.eps);
            foreach (Face fc in faces) fc.UserData.Add("ShapeIt.OriginalFace", new FaceReference(fc));
            BRepOperation breptest = new BRepOperation(faces);
            HashSet<(Face, Face)> intersectingPairs = breptest.IntersectingFaces;
            Dictionary<Face, IEnumerable<Face>> originalToClipped = new Dictionary<Face, IEnumerable<Face>>();
            while (intersectingPairs.Any())
            {
                IEnumerable<Face> first, second;
                Face ip1 = intersectingPairs.First().Item1; // the original faces, which will be clipped
                Face ip2 = intersectingPairs.First().Item2;
                if (!originalToClipped.TryGetValue(ip1, out first)) first = new Face[] { ip1 };
                if (!originalToClipped.TryGetValue(ip2, out second)) second = new Face[] { ip2 };
                List<Face> f1parts = new List<Face>();
                List<Face> f2parts = new List<Face>();
                foreach (Face f1 in first)
                {
                    foreach (Face f2 in second)
                    {
                        // we need to clone to make the face independant of the old edge connections
                        BRepOperation brepIntersect = new BRepOperation(Shell.FromFaces(f1.Clone() as Face), Shell.FromFaces(f2.Clone() as Face), BRepOperation.Operation.intersection);
                        brepIntersect.AllowOpenEdges = true;
                        Shell[] intersected = brepIntersect.Result();
                        for (int i = 0; i < intersected.Length; i++)
                        {
                            for (int j = 0; j < intersected[i].Faces.Length; j++)
                            {
                                Face original = (intersected[i].Faces[j].UserData["ShapeIt.OriginalFace"] as FaceReference)?.Face;
                                if (original == ip1) f1parts.Add(intersected[i].Faces[j]);
                                if (original == ip2) f2parts.Add(intersected[i].Faces[j]);
                            }
                        }
                    }
                }
                if (f1parts.Count > 0) originalToClipped[ip1] = f1parts;
                if (f2parts.Count > 0) originalToClipped[ip2] = f2parts;
                intersectingPairs.Remove(intersectingPairs.First());
            }
            List<Face> clippedAndUnclipped = new List<Face>();
            foreach (Face f in faces)
            {
                if (originalToClipped.TryGetValue(f, out IEnumerable<Face> replacements)) clippedAndUnclipped.AddRange(replacements);
                else clippedAndUnclipped.Add(f);
            }



            //BRepOperation brepop = new BRepOperation(new Face[] { faces[37], faces[38] });
            //brepop.AllowOpenEdges = true;
            //Shell[] r = brepop.Result();

            //foreach (Face fc in faces)
            //{
            //    fc.ReverseOrientation();
            //}
            // the faces in "faces" are partially connected and some parts overhang (protrusion)
            // the overhanging parts must now be trimmed to build a fully connected shell
            //BRepOperation brepop = new BRepOperation(faces);
            //Shell[] res = brepop.Result();
            //return res;
            return null;
        }
    }
}
