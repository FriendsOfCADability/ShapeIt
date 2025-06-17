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

    /// <summary>
    /// A reference to a edge. This is mainly used as a UserData: the Clone method does not clone the edge but returns this object, so you won't have an infinite loop
    /// when the userdata of a edge refers to itself.
    /// </summary>
    public class EdgeReference : ICloneable
    {
        public EdgeReference(Edge edge) { Edge = edge; }
        public Edge Edge { get; }
        public object Clone()
        {
            return this; // don't clone the edge, this would result in an infinite loop
        }
    }

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
            else if (axis.Curve3D is Ellipse bigCircle && bigCircle.IsCircle)
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
                GeoVector normal = axis.Curve3D.StartDirection ^ axis.Curve3D.EndDirection;
                if (normal.IsNullVector()) axis.Curve3D.StartDirection.ArbitraryNormals(out normal, out GeoVector _);
                int n = 20; // number of intermediate points, need some adaptive algorithm
                List<Ellipse> throughEllipses = new List<Ellipse>(n+2);
                for (int i = 0; i < n; i++)
                {
                    double par = (double)i / (n - 1);
                    GeoVector dir = axis.Curve3D.DirectionAt(par);
                    GeoPoint pos = axis.Curve3D.PointAt(par);
                    GeoVector n1 = axis.PrimaryFace.Surface.GetNormal(axis.PrimaryFace.Surface.PositionOf(pos));
                    GeoVector n2 = axis.SecondaryFace.Surface.GetNormal(axis.SecondaryFace.Surface.PositionOf(pos));
                    GeoVector nn = (n1.Normalized + n2.Normalized).Normalized;
                    Plane epln = new Plane(pos, nn ^ dir, nn);
                    Ellipse elli = Ellipse.Construct();
                    elli.SetArcPlaneCenterStartEndPoint(epln, GeoPoint2D.Origin, new GeoPoint2D(-Math.Abs(radius), 0), new GeoPoint2D(Math.Abs(radius), 0), epln, true);
                    double pp = elli.PositionOf(pos + Math.Abs(radius) * nn);
                    throughEllipses.Add(elli);
                }
                //GeoVector ddir = -(throughEllipses[1].Center | throughEllipses[0].Center) * axis.Curve3D.StartDirection.Normalized;
                //Ellipse efirst = throughEllipses[0].Clone() as Ellipse;
                //efirst.Modify(ModOp.Translate(ddir));
                //throughEllipses.Insert(0,efirst);
                //int tn = throughEllipses.Count - 1;
                //ddir = (throughEllipses[tn].Center | throughEllipses[tn-1].Center) * axis.Curve3D.EndDirection.Normalized;
                //Ellipse elast = throughEllipses[tn].Clone() as Ellipse;
                //elast.Modify(ModOp.Translate(ddir));
                //throughEllipses.Add(elast);
                surface = new NurbsSurface(throughEllipses.ToArray());
            }
            if (surface != null)
            {
                BoundingRect domain = new BoundingRect(surface.PositionOf(forward.Curve3D.PointAt(0.5))); // toroidal fillets may sweep over 180°. so it would be ambiguous which part to use
                GeoPoint2D uv = surface.PositionOf(forward.Vertex2.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);
                uv = surface.PositionOf(forward.Vertex1.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);
                uv = surface.PositionOf(backward.Vertex1.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);
                uv = surface.PositionOf(backward.Vertex2.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);

                // we need 4 edges to make this face. Two edges are already provided as parameters, the other two edges are arcs at the start- and endpoint of the axis
                // maybe the two provided edges meet in a single point, then the ellipses are null and we only have 3 edges for the face
                // is there a case with both ellipses==null?
                GeoPoint center = axis.Vertex1.Position; // this is the startpoint of axis.Curve3D
                GeoVector toOutside = (axis.PrimaryFace.Surface.GetNormal(axis.Vertex1.GetPositionOnFace(axis.PrimaryFace)).Normalized +
                             axis.SecondaryFace.Surface.GetNormal(axis.Vertex1.GetPositionOnFace(axis.SecondaryFace)).Normalized);
                Plane pln = new Plane(center, axis.Curve3D.StartDirection);
                Ellipse elli1 = null;
                double d1 = (center | forward.Vertex1.Position) + (center | backward.Vertex2.Position);
                double d2 = (center | forward.Vertex2.Position) + (center | backward.Vertex1.Position);
                Vertex v1 = d1 < d2 ? forward.Vertex1 : forward.Vertex2;
                Vertex v2 = d1 < d2 ? backward.Vertex2 : backward.Vertex1;
                if (!Precision.IsEqual(v1.Position, v2.Position))
                {
                    elli1 = Ellipse.Construct();
                    elli1.SetArc3Points(v1.Position, center + radius * toOutside.Normalized, v2.Position, pln);
                    if (Math.Abs(elli1.SweepParameter) > Math.PI) elli1.SetArc3Points(v1.Position, center - radius * toOutside.Normalized, v2.Position, pln);
                }

                center = axis.Vertex2.Position; // this is the endpoint of axis.Curve3D
                toOutside = (axis.PrimaryFace.Surface.GetNormal(axis.Vertex2.GetPositionOnFace(axis.PrimaryFace)).Normalized +
                             axis.SecondaryFace.Surface.GetNormal(axis.Vertex2.GetPositionOnFace(axis.SecondaryFace)).Normalized);
                pln = new Plane(center, axis.Curve3D.EndDirection);
                Ellipse elli2 = null;
                v1 = forward.Vertex1 == v1 ? forward.Vertex2 : forward.Vertex1;
                v2 = backward.Vertex1 == v2 ? backward.Vertex2 : backward.Vertex1;
                if (!Precision.IsEqual(v1.Position, v2.Position))
                {
                    elli2 = Ellipse.Construct();
                    elli2.SetArc3Points(v1.Position, center + radius * toOutside.Normalized, v2.Position, pln);
                    if (Math.Abs(elli2.SweepParameter) > Math.PI) elli2.SetArc3Points(v1.Position, center - radius * toOutside.Normalized, v2.Position, pln);
                }

                List<ICurve2D> curve2Ds = new List<ICurve2D>();

                curve2Ds.Add(surface.GetProjectedCurve(forward.Curve3D, Precision.eps));
                if (elli1 != null) curve2Ds.Add(surface.GetProjectedCurve(elli1, Precision.eps));
                curve2Ds.Add(surface.GetProjectedCurve(backward.Curve3D, Precision.eps));
                if (elli2 != null) curve2Ds.Add(surface.GetProjectedCurve(elli2, Precision.eps));
                for (int i = 0; i < curve2Ds.Count; ++i) SurfaceHelper.AdjustPeriodic(surface, domain, curve2Ds[i]);
                SimpleShape ss = new SimpleShape(Border.FromUnorientedList(curve2Ds.ToArray(), true));
                Face res = Face.MakeFace(surface, ss);
                res.UseEdge(forward);
                res.UseEdge(backward);
                return res;
            }
            return null;
        }
        static List<Face> ConnectedList(Face startWith, HashSet<Face> visited = null, List<Face> result = null)
        {
            if (visited == null)
            {
                visited = new HashSet<Face>();
                result = new List<Face>();
            }
            result.Add(startWith);
            visited.Add(startWith);
            foreach (Edge edg in startWith.AllEdges)
            {
                Face other = edg.OtherFace(startWith);
                if (visited.Contains(other)) continue;
                visited.Add(other);
                ConnectedList(other, visited, result);
            }
            return result;
        }
        public static Shell[] GetOffsetNew(this Shell shell, double offset)
        {
            Dictionary<(Face, Edge), Edge> faceEdgeToParallelEdge = new Dictionary<(Face, Edge), Edge>(); // the parallel edges to the original edges, also depend on the face
            Dictionary<Vertex, List<Edge>> vertexToArcs = new Dictionary<Vertex, List<Edge>>(); // for each vertex there are the sides of the wedges, which build spherical wedges
            Dictionary<Face, Face> faceToOffsetFace = new Dictionary<Face, Face>(); // for each face of the original shell we have a face with the required offset here
            Dictionary<Edge, Face> edgeToFillet = new Dictionary<Edge, Face>(); // for each convex edge we create a "fillet" face
            List<Face> sphercalWegdes = new List<Face>(); // the sphreical faces fill the gaps between the fillets
            // for the brep operation we need edge-face pairs, which should not be tested for intersection, because they connect adjacent parts like the offset faces with the fillets
            // or the fillets with the spherical faces. Since each breop operation creates new faces and edges, we attach this information to the user data of the original parts
            // and retrieve them after the brep operation is done.
            HashSet<(Edge, Face)> dontIntersect = new HashSet<(Edge, Face)>(); // these pairs will be connected and there is no need to calculate the intersection
            // set UserData with the original face and edge references to 
            foreach (Face face in shell.Faces)
            {   // makeparallel faces with the provided offset
                ISurface offsetSurface = face.Surface.GetOffsetSurface(offset);
                if (offsetSurface == null) continue; // a sphere, cylinder or torus shrinking to 0
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
                            // what about orientation?
                            if (!edge.Forward(face)) c2d.Reverse(); // not tested!
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
                            if (c2d is InterpolatedDualSurfaceCurve.ProjectedCurve pc) c2d = pc.ToBSpline(0.0);
                            if (c2d is Path2D)
                            {
                                c2d = face.Surface.GetProjectedCurve(edge.Curve3D, 0.0); // sine curve is not maintained (14.6.25) but was converted to Path2D
                                if (!edge.Forward(face)) c2d.Reverse();
                            }
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
                    offsetFace.UserData.Add("ShapeIt.OriginalFace", new FaceReference(offsetFace));
#if DEBUG
                    DebuggerContainer dc = new DebuggerContainer();
                    dc.Add(face);
                    dc.Add(offsetFace);
                    foreach (Vertex vtx in face.Vertices)
                    {
                        Line l = Line.TwoPoints(vtx.Position, vtx.Position + offset * face.Surface.GetNormal(vtx.GetPositionOnFace(face)).Normalized);
                        dc.Add(l);
                    }
#endif

                    // faces.Add(offsetFace);
                    // we need a reference from edges of the original face to the edges of the paralell face.
                    foreach (Edge edg in offsetFace.Edges)
                    {
                        Edge originalEdge = edg.Curve2D(offsetFace).UserData["CADability.CurveToEdge"] as Edge;
                        if (originalEdge != null) faceEdgeToParallelEdge[(face, originalEdge)] = edg;
                        edg.Curve2D(offsetFace).UserData.Remove("CADability.CurveToEdge"); // no longer needed
                        if (edg.Curve3D is IGeoObject go)
                        {
                            go.UserData.Add("ShapeIt.OriginalEdge", new EdgeReference(edg));
                        }
                    }
                    //offsetFace.ReverseOrientation();
                    faceToOffsetFace[face] = offsetFace;
                }
            }

            foreach (Edge sedge in shell.Edges)
            {
                AdjacencyType toCheckFor = offset > 0 ? AdjacencyType.Convex : AdjacencyType.Concave;
                if (sedge.Adjacency() == toCheckFor && faceEdgeToParallelEdge.TryGetValue((sedge.PrimaryFace, sedge), out Edge e1) && faceEdgeToParallelEdge.TryGetValue((sedge.SecondaryFace, sedge), out Edge e2))
                {
                    Face fillet = MakeOffsetFillet(sedge, offset, e1, e2);
                    //fillet.ReverseOrientation();
                    if (fillet != null)
                    {
                        fillet.UserData.Add("ShapeIt.OriginalFace", new FaceReference(fillet));
                        dontIntersect.Add((e1, fillet));
                        dontIntersect.Add((e2, fillet));
                        edgeToFillet[sedge] = fillet;
                        Face primaryOffset = faceToOffsetFace[sedge.PrimaryFace];
                        Face secondaryOffset = faceToOffsetFace[sedge.SecondaryFace]; // must both exist!
                                                                                      // look for the arcs in the fillet and add them to vertexToArcs
                        foreach (Edge edg in fillet.AllEdges)
                        {
                            if (edg.Curve3D is IGeoObject go)
                            {
                                go.UserData.Add("ShapeIt.OriginalEdge", new EdgeReference(edg));
                            }
                            if (edg.Curve3D is Ellipse elli && Math.Abs(elli.Radius - Math.Abs(offset)) < Precision.eps)
                            {
                                if (Precision.IsEqual(elli.Center, sedge.Vertex1.Position))
                                {
                                    if (!vertexToArcs.TryGetValue(sedge.Vertex1, out List<Edge> arcs))
                                    {
                                        vertexToArcs[sedge.Vertex1] = arcs = new List<Edge>();
                                    }
                                    arcs.Add(edg);
                                }
                                if (Precision.IsEqual(elli.Center, sedge.Vertex2.Position))
                                {
                                    if (!vertexToArcs.TryGetValue(sedge.Vertex2, out List<Edge> arcs))
                                    {
                                        vertexToArcs[sedge.Vertex2] = arcs = new List<Edge>();
                                    }
                                    arcs.Add(edg);
                                }
                            }
                            if (Precision.IsEqual(edg.Vertex1.Position, e1.Vertex2.Position) && Precision.IsEqual(edg.Vertex2.Position, e1.Vertex1.Position)) // edges must be reverse oriented, i.e. we can use Vertex1, Vertex2
                            {
                                dontIntersect.Add((edg, primaryOffset));
                            }
                            if (Precision.IsEqual(edg.Vertex1.Position, e2.Vertex2.Position) && Precision.IsEqual(edg.Vertex2.Position, e2.Vertex1.Position))
                            {
                                dontIntersect.Add((edg, secondaryOffset));
                            }
                        }
                    }
                }
            }
            //#if DEBUG
            //            foreach ((Edge, Face) item in dontIntersect)
            //            {
            //                DebuggerContainer dc = new DebuggerContainer();
            //                dc.Add(item.Item1.Curve3D as IGeoObject);
            //                dc.Add(item.Item2);
            //                System.Diagnostics.Trace.WriteLine(item.Item1.GetHashCode().ToString() + " - - " + item.Item2.GetHashCode().ToString());
            //            }
            //#endif

            foreach (Vertex vtx in shell.Vertices)
            {
                if (!vertexToArcs.TryGetValue(vtx, out List<Edge> arcs)) continue;
                for (int i = 0; i < arcs.Count - 1; i++)
                {
                    for (int j = i + 1; j < arcs.Count; j++)
                    {
                        if (Curves.GetCommonPlane(new ICurve[] { arcs[i].Curve3D, arcs[j].Curve3D }, out Plane _))
                        {   // two tangential edges in a vertex create the same arc edge and should be connected
                            dontIntersect.Add((arcs[i], arcs[j].PrimaryFace));
                            dontIntersect.Add((arcs[j], arcs[i].PrimaryFace));
                        }
                    }
                }
                if (arcs.Count < 3) continue; // this is not a valid spherical face
                GeoVector toOutside = GeoVector.NullVector;
                foreach (Face face in vtx.Faces)
                {
                    GeoVector n = face.Surface.GetNormal(vtx.GetPositionOnFace(face));
                    toOutside += n.Normalized;
                }
                toOutside.ArbitraryNormals(out GeoVector dirx, out GeoVector diry);
                SphericalSurface sphericalSurface = new SphericalSurface(vtx.Position, offset * toOutside.Normalized, offset * dirx.Normalized, offset * diry.Normalized);
                List<ICurve2D> curvesOnSphere = new List<ICurve2D>();
                BoundingRect domain = BoundingRect.EmptyBoundingRect;
                for (int i = 0; i < arcs.Count; i++)
                {
                    ICurve2D c2d = sphericalSurface.GetProjectedCurve(arcs[i].Curve3D, Precision.eps);
                    if (domain.IsEmpty()) domain.MinMax(c2d.GetExtent());
                    else
                    {
                        SurfaceHelper.AdjustPeriodic(sphericalSurface, domain, c2d);
                        domain.MinMax(c2d.GetExtent());
                    }
                    curvesOnSphere.Add(c2d);
                }
                Border bdr = Border.FromUnorientedList(curvesOnSphere.ToArray(), true); // we would need to sort the curves when more than 3
                Face sphere = Face.MakeFace(sphericalSurface, new SimpleShape(bdr));
                for (int i = 0; i < arcs.Count; i++) sphere.UseEdge(arcs[i]);

                //sphere.ReverseOrientation();
                sphercalWegdes.Add(sphere);
                foreach (Edge edg in sphere.Edges)
                {
                    for (int i = 0; i < arcs.Count; i++)
                    {
                        if (Curves.GetCommonPlane(edg.Curve3D, arcs[i].Curve3D, out Plane _))
                        {
                            dontIntersect.Add((edg, arcs[i].PrimaryFace));
                            dontIntersect.Add((arcs[i], sphere));
                        }
                    }
                }
            }
            HashSet<Face> faces = new HashSet<Face>(faceToOffsetFace.Values); // the raw offset faces
            faces.UnionWith(edgeToFillet.Values); // the fillets on the edges
            faces.UnionWith(sphercalWegdes); // the spherical faces on the vertices
                                             // faces contains all the faces for the offset shell, the edges are properly connected but some parts are standing out
            Shell.ConnectFaces(faces.ToArray(), Precision.eps);
            BRepOperation bo = new BRepOperation(faces);
            Shell[] res = bo.Result();
            return res;
        }
        public static Shell[] GetOffsetNewZ(this Shell shell, double offset)
        {
            Dictionary<(Face, Edge), Edge> faceEdgeToParallelEdge = new Dictionary<(Face, Edge), Edge>(); // the parallel edges to the original edges, also depend on the face
            Dictionary<Vertex, List<Edge>> vertexToArcs = new Dictionary<Vertex, List<Edge>>(); // for each vertex there are the sides of the wedges, which build spherical wedges
            Dictionary<Face, Face> faceToOffsetFace = new Dictionary<Face, Face>(); // for each face of the original shell we have a face with the required offset here
            Dictionary<Edge, Face> edgeToFillet = new Dictionary<Edge, Face>(); // for each convex edge we create a "fillet" face
            List<Face> sphercalWegdes = new List<Face>(); // the sphreical faces fill the gaps between the fillets
            // for the brep operation we need edge-face pairs, which should not be tested for intersection, because they connect adjacent parts like the offset faces with the fillets
            // or the fillets with the spherical faces. Since each breop operation creates new faces and edges, we attach this information to the user data of the original parts
            // and retrieve them after the brep operation is done.
            HashSet<(Edge, Face)> dontIntersect = new HashSet<(Edge, Face)>(); // these pairs will be connected and there is no need to calculate the intersection
            // set UserData with the original face and edge references to 
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
                            // what about orientation?
                            if (!edge.Forward(face)) c2d.Reverse(); // not tested!
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
                            if (c2d is InterpolatedDualSurfaceCurve.ProjectedCurve pc) c2d = pc.ToBSpline(0.0);
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
                    offsetFace.UserData.Add("ShapeIt.OriginalFace", new FaceReference(offsetFace));

                    // faces.Add(offsetFace);
                    // we need a reference from edges of the original face to the edges of the paralell face.
                    foreach (Edge edg in offsetFace.Edges)
                    {
                        Edge originalEdge = edg.Curve2D(offsetFace).UserData["CADability.CurveToEdge"] as Edge;
                        if (originalEdge != null) faceEdgeToParallelEdge[(face, originalEdge)] = edg;
                        edg.Curve2D(offsetFace).UserData.Remove("CADability.CurveToEdge"); // no longer needed
                        if (edg.Curve3D is IGeoObject go)
                        {
                            go.UserData.Add("ShapeIt.OriginalEdge", new EdgeReference(edg));
                        }
                    }
                    //offsetFace.ReverseOrientation();
                    faceToOffsetFace[face] = offsetFace;
                }
            }

            foreach (Edge sedge in shell.Edges)
            {
                AdjacencyType toCheckFor = offset > 0 ? AdjacencyType.Convex : AdjacencyType.Concave;
                if (sedge.Adjacency() == toCheckFor && faceEdgeToParallelEdge.TryGetValue((sedge.PrimaryFace, sedge), out Edge e1) && faceEdgeToParallelEdge.TryGetValue((sedge.SecondaryFace, sedge), out Edge e2))
                {
                    Face fillet = MakeOffsetFillet(sedge, offset, e1, e2);
                    //fillet.ReverseOrientation();
                    if (fillet != null)
                    {
                        fillet.UserData.Add("ShapeIt.OriginalFace", new FaceReference(fillet));
                        dontIntersect.Add((e1, fillet));
                        dontIntersect.Add((e2, fillet));
                        edgeToFillet[sedge] = fillet;
                        Face primaryOffset = faceToOffsetFace[sedge.PrimaryFace];
                        Face secondaryOffset = faceToOffsetFace[sedge.SecondaryFace]; // must both exist!
                                                                                      // look for the arcs in the fillet and add them to vertexToArcs
                        foreach (Edge edg in fillet.AllEdges)
                        {
                            if (edg.Curve3D is IGeoObject go)
                            {
                                go.UserData.Add("ShapeIt.OriginalEdge", new EdgeReference(edg));
                            }
                            if (edg.Curve3D is Ellipse elli && Math.Abs(elli.Radius - offset) < Precision.eps)
                            {
                                if (Precision.IsEqual(elli.Center, sedge.Vertex1.Position))
                                {
                                    if (!vertexToArcs.TryGetValue(sedge.Vertex1, out List<Edge> arcs))
                                    {
                                        vertexToArcs[sedge.Vertex1] = arcs = new List<Edge>();
                                    }
                                    arcs.Add(edg);
                                }
                                if (Precision.IsEqual(elli.Center, sedge.Vertex2.Position))
                                {
                                    if (!vertexToArcs.TryGetValue(sedge.Vertex2, out List<Edge> arcs))
                                    {
                                        vertexToArcs[sedge.Vertex2] = arcs = new List<Edge>();
                                    }
                                    arcs.Add(edg);
                                }
                            }
                            if (Precision.IsEqual(edg.Vertex1.Position, e1.Vertex2.Position) && Precision.IsEqual(edg.Vertex2.Position, e1.Vertex1.Position)) // edges must be reverse oriented, i.e. we can use Vertex1, Vertex2
                            {
                                dontIntersect.Add((edg, primaryOffset));
                            }
                            if (Precision.IsEqual(edg.Vertex1.Position, e2.Vertex2.Position) && Precision.IsEqual(edg.Vertex2.Position, e2.Vertex1.Position))
                            {
                                dontIntersect.Add((edg, secondaryOffset));
                            }
                        }
                    }
                }
            }
            //#if DEBUG
            //            foreach ((Edge, Face) item in dontIntersect)
            //            {
            //                DebuggerContainer dc = new DebuggerContainer();
            //                dc.Add(item.Item1.Curve3D as IGeoObject);
            //                dc.Add(item.Item2);
            //                System.Diagnostics.Trace.WriteLine(item.Item1.GetHashCode().ToString() + " - - " + item.Item2.GetHashCode().ToString());
            //            }
            //#endif

            foreach (Vertex vtx in shell.Vertices)
            {
                if (!vertexToArcs.TryGetValue(vtx, out List<Edge> arcs)) continue;
                for (int i = 0; i < arcs.Count - 1; i++)
                {
                    for (int j = i + 1; j < arcs.Count; j++)
                    {
                        if (Curves.GetCommonPlane(new ICurve[] { arcs[i].Curve3D, arcs[j].Curve3D }, out Plane _))
                        {   // two tangential edges in a vertex create the same arc edge and should be connected
                            dontIntersect.Add((arcs[i], arcs[j].PrimaryFace));
                            dontIntersect.Add((arcs[j], arcs[i].PrimaryFace));
                        }
                    }
                }
                if (arcs.Count < 3) continue; // this is not a valid spherical face
                GeoVector toOutside = GeoVector.NullVector;
                foreach (Face face in vtx.Faces)
                {
                    GeoVector n = face.Surface.GetNormal(vtx.GetPositionOnFace(face));
                    toOutside += n.Normalized;
                }
                toOutside.ArbitraryNormals(out GeoVector dirx, out GeoVector diry);
                SphericalSurface sphericalSurface = new SphericalSurface(vtx.Position, offset * toOutside.Normalized, offset * dirx.Normalized, offset * diry.Normalized);
                List<ICurve2D> curvesOnSphere = new List<ICurve2D>();
                BoundingRect domain = BoundingRect.EmptyBoundingRect;
                for (int i = 0; i < arcs.Count; i++)
                {
                    ICurve2D c2d = sphericalSurface.GetProjectedCurve(arcs[i].Curve3D, Precision.eps);
                    if (domain.IsEmpty()) domain.MinMax(c2d.GetExtent());
                    else
                    {
                        SurfaceHelper.AdjustPeriodic(sphericalSurface, domain, c2d);
                        domain.MinMax(c2d.GetExtent());
                    }
                    curvesOnSphere.Add(c2d);
                }
                Border bdr = Border.FromUnorientedList(curvesOnSphere.ToArray(), true); // we would need to sort the curves when more than 3
                Face sphere = Face.MakeFace(sphericalSurface, new SimpleShape(bdr));
                //sphere.ReverseOrientation();
                sphercalWegdes.Add(sphere);
                foreach (Edge edg in sphere.Edges)
                {
                    for (int i = 0; i < arcs.Count; i++)
                    {
                        if (Curves.GetCommonPlane(edg.Curve3D, arcs[i].Curve3D, out Plane _))
                        {
                            dontIntersect.Add((edg, arcs[i].PrimaryFace));
                            dontIntersect.Add((arcs[i], sphere));
                        }
                    }
                }
            }

            HashSet<Edge> visitedEdges = new HashSet<Edge>();
            List<Shell> result = new List<Shell>();
            KeyValuePair<Face, Face> startWith = faceToOffsetFace.First();
            List<Face> connected = ConnectedList(startWith.Key);
            // we need to have the faces sorted so that the next face is connected to one the previous faces. BRepOperation doesn't like to union 
            // unconnected parts
            foreach (Face face in connected)
            {
                // first add the offset face to the already accumulated parts
                if (faceToOffsetFace.TryGetValue(face, out Face offsetFace))
                {
                    if (result.Count == 0) result.Add(Shell.FromFaces(offsetFace));
                    else
                    {
                        List<Shell> next = new List<Shell>();
                        for (int i = 0; i < result.Count; i++)
                        {
                            HashSet<(Edge, Face)> dontIntersectUpdated = UpdateEdgeFacePairs(dontIntersect, result[i].Faces.Append(offsetFace));
                            BRepOperation bo = new BRepOperation(result[i], Shell.FromFaces(offsetFace), dontIntersectUpdated, BRepOperation.Operation.intersection);
                            bo.AllowOpenEdges = true;
                            bo.DontCombineConnectedFaces = true; // to keep the faces and edges in "dontIntersect" valid
                            Shell[] r = bo.Result();
                            if (r.Length > 0) next.AddRange(r);
                        }
                        result = next;
                    }
                }
                // now add all edge fillets around this face, so in the next step the next face will have a place to dock onto
                foreach (Edge edg in face.AllEdges)
                {
                    if (edgeToFillet.TryGetValue(edg, out Face fillet))
                    {
                        if (visitedEdges.Contains(edg)) continue;
                        visitedEdges.Add(edg);
                        List<Shell> next = new List<Shell>();
                        for (int i = 0; i < result.Count; i++)
                        {
                            HashSet<(Edge, Face)> dontIntersectUpdated = UpdateEdgeFacePairs(dontIntersect, result[i].Faces.Append(fillet));
                            BRepOperation bo = new BRepOperation(result[i], Shell.FromFaces(fillet), dontIntersectUpdated, BRepOperation.Operation.intersection);
                            bo.AllowOpenEdges = true;
                            bo.DontCombineConnectedFaces = true;
                            Shell[] r = bo.Result();
                            if (r.Length > 0) next.AddRange(r);
                        }
                        result = next;
                    }
                }
            }
            // now the spheres fill the remaining gaps
            foreach (Face sphere in sphercalWegdes)
            {
                List<Shell> next = new List<Shell>();
                for (int i = 0; i < result.Count; i++)
                {
                    HashSet<(Edge, Face)> dontIntersectUpdated = UpdateEdgeFacePairs(dontIntersect, result[i].Faces.Append(sphere));
                    BRepOperation bo = new BRepOperation(result[i], Shell.FromFaces(sphere), dontIntersectUpdated, BRepOperation.Operation.intersection);
                    bo.AllowOpenEdges = true;
                    bo.DontCombineConnectedFaces = true;
                    Shell[] r = bo.Result();
                    if (r.Length > 0) next.AddRange(r);
                }
                result = next;
            }
            result.RemoveAll(item => item.HasOpenEdgesExceptPoles());
            for (int i = 0; i < result.Count; i++)
            {   // remove all the userdata, which are references to the raw offset faces and edges
                foreach (Face face in result[i].Faces)
                {
                    face.UserData.RemoveUserData("ShapeIt.OriginalFace");
                }
                foreach (Edge edg in result[i].Edges)
                {
                    (edg.Curve3D as IGeoObject)?.UserData.RemoveUserData("ShapeIt.OriginalEdge");
                }
                //result[i].ReverseOrientation();
                result[i].CombineConnectedFaces();
            }
            return result.ToArray();
        }

        private static HashSet<(Edge, Face)> UpdateEdgeFacePairs(HashSet<(Edge, Face)> dontIntersect, IEnumerable<Face> faces)
        {
            HashSet<(Edge, Face)> res = new HashSet<(Edge, Face)>();
            HashSet<Edge> allEdges = new HashSet<Edge>();
            Dictionary<Face, Face> originalFaceToCurrent = new Dictionary<Face, Face>();
            Dictionary<Edge, Edge> originalEdgeToCurrent = new Dictionary<Edge, Edge>();
            foreach (Face face in faces)
            {
                allEdges.UnionWith(face.Edges);
                Face current = (face.UserData.GetData("ShapeIt.OriginalFace") as FaceReference)?.Face;
                if (current != null) originalFaceToCurrent[face] = current;
            }
            foreach (Edge edge in allEdges)
            {
                Edge current = ((edge.Curve3D as IGeoObject)?.UserData.GetData("ShapeIt.OriginalEdge") as EdgeReference)?.Edge;
                if (current != null) originalEdgeToCurrent[edge] = current;
            }
            foreach ((Edge edge, Face face) ef in dontIntersect)
            {
                if (originalEdgeToCurrent.TryGetValue(ef.edge, out Edge d1)) { }
                if (originalFaceToCurrent.TryGetValue(ef.face, out Face d2)) { }
                if (originalEdgeToCurrent.TryGetValue(ef.edge, out Edge cedge) && originalFaceToCurrent.TryGetValue(ef.face, out Face cface)) res.Add((cedge, cface));
            }
            return res;
        }

        public static Shell[] GetOffsetNewY(this Shell shell, double offset)
        {
            List<Shell> result = new List<Shell>();
            List<Shell> blocks = new List<Shell>(); // a "block" is the perpendicular extrusion of each face, which sits exactely on the face
            List<Shell> wedges = new List<Shell>(); // a "wedge" is the wedge which fills the gap between adjacen blocks
            List<Shell> sphericalWedges = new List<Shell>(); // a "spherical wedge" is "hand axe" like solid, which fills the gap on a convex vertex 
                                                             // List<Face> faces = new List<Face>(); // the faces which are equidistant to the shell. Not clipped, not sure we need them
            Dictionary<(Face, Edge), Edge> faceEdgeToParallelEdge = new Dictionary<(Face, Edge), Edge>(); // the parallel edges to the original edges, also depend on the face
            Dictionary<(Face, Edge), Face> faceEdgeToSide = new Dictionary<(Face, Edge), Face>(); // the sides which stand perpendicular to the face on an edge
            Dictionary<Vertex, List<Face>> vertexToSide = new Dictionary<Vertex, List<Face>>(); // for each vertex there are the sides of the wedges, which build spherical wedges
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
                            // what about orientation?
                            if (!edge.Forward(face)) c2d.Reverse(); // not tested!
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
                            if (c2d is InterpolatedDualSurfaceCurve.ProjectedCurve pc) c2d = pc.ToBSpline(0.0);
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
                    // faces.Add(offsetFace);
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
                        if (surface is RuledSurface)
                        {   // then c12d and c22d are simple lines
                            c12d = new Line2D(new GeoPoint2D(0, 0), new GeoPoint2D(1, 0));
                            c22d = new Line2D(new GeoPoint2D(0, 1), new GeoPoint2D(1, 1));
                        }
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
                        // faces.Add(fillet);
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
                        if (!vertexToSide.TryGetValue(sedge.Vertex1, out List<Face> sphericalSides)) vertexToSide[sedge.Vertex1] = sphericalSides = new List<Face>();
                        sphericalSides.Add(frontFace);
                        if (!vertexToSide.TryGetValue(sedge.Vertex2, out sphericalSides)) vertexToSide[sedge.Vertex2] = sphericalSides = new List<Face>();
                        sphericalSides.Add(backFace);
                    }
                }
            }

            foreach (Vertex vtx in shell.Vertices)
            {
                if (vertexToSide.TryGetValue(vtx, out List<Face> sphericalSides) && sphericalSides.Count >= 3)
                {
                    GeoVector toOutside = GeoVector.NullVector;
                    foreach (Face face in vtx.Faces)
                    {
                        GeoVector n = face.Surface.GetNormal(vtx.GetPositionOnFace(face));
                        toOutside += n.Normalized;
                    }
                    toOutside.ArbitraryNormals(out GeoVector dirx, out GeoVector diry);
                    SphericalSurface sphericalSurface = new SphericalSurface(vtx.Position, offset * toOutside.Normalized, offset * dirx.Normalized, offset * diry.Normalized);
                    List<ICurve2D> curvesOnSphere = new List<ICurve2D>();
                    BoundingRect domain = BoundingRect.EmptyBoundingRect;
                    foreach (Face fc in sphericalSides)
                    {
                        // the sphericalSide has exactely 3 edges: two lines and an arc. We look for the arc
                        foreach (Edge edge in fc.AllEdges)
                        {
                            if (edge.Curve3D is Ellipse)
                            {
                                ICurve2D c2d = sphericalSurface.GetProjectedCurve(edge.Curve3D, Precision.eps);
                                if (domain.IsEmpty()) domain.MinMax(c2d.GetExtent());
                                else
                                {
                                    SurfaceHelper.AdjustPeriodic(sphericalSurface, domain, c2d);
                                    domain.MinMax(c2d.GetExtent());
                                }
                                curvesOnSphere.Add(c2d);
                            }
                        }
                    }
                    Border bdr = Border.FromUnorientedList(curvesOnSphere.ToArray(), true); // we would need to sort the curves when more than 3
                    Face sphere = Face.MakeFace(sphericalSurface, new SimpleShape(bdr));
                    List<Face> spericalWedge = new List<Face>();
                    foreach (Face fc in sphericalSides)
                    {
                        Face cloned = fc.Clone() as Face;
                        cloned.ReverseOrientation();
                        spericalWedge.Add(cloned);
                    }
                    spericalWedge.Add(sphere);
                    Shell swshell = Shell.FromFaces(spericalWedge.ToArray(), true);

                    sphericalWedges.Add(swshell);
                }
            }

            faceEdgeToParallelEdge.Clear();
            faceEdgeToSide.Clear();
            vertexToSide.Clear();

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
                    if (r != null) next.AddRange(r);
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
            for (int j = 0; j < sphericalWedges.Count; j++)
            {
                Shell s = sphericalWedges[j];
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
