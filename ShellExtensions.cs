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
                List<Ellipse> throughEllipses = new List<Ellipse>(n);
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
                ICurve2D[] selfIntersection = surface.GetSelfIntersections(res.Domain);

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
            HashSet<Face> inverseFaces = new HashSet<Face>();
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
                if (offsetSurface != null) // 
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
                    if (offsetSurface.GetNormal(cnt) * face.Surface.GetNormal(cnt) < 0)
                    {   // this face is reversed, e.g. a cylinder, which now has negative radius
                        // we still need to create it, because we need the edges for the fillets, but we do not use it in the result
                        inverseFaces.Add(offsetFace);
                    }
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
            faces.ExceptWith(inverseFaces); // these faces will not appear in the result
            faces.UnionWith(edgeToFillet.Values); // the fillets on the edges
            faces.UnionWith(sphercalWegdes); // the spherical faces on the vertices
                                             // faces contains all the faces for the offset shell, the edges are properly connected but some parts are standing out
            Shell.ConnectFaces(faces.ToArray(), Precision.eps);
            BRepOperation bo = new BRepOperation(faces);
            Shell[] res = bo.Result();
            return res;
        }
    }
}
