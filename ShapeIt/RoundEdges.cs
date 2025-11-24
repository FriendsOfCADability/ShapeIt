using CADability;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms.VisualStyles;
using static ShapeIt.ShellExtensions;

namespace ShapeIt
{
    internal class RoundEdges
    {
        Shell shell;
        IEnumerable<Edge> convexEdges;
        IEnumerable<Edge> concaveEdges;
        double radius;
        /// <summary>
        /// - fillet: the face which rounds the edge
        /// - frontEnd: the two arcs at the end of the fillet face
        /// - tangent: the edges tangential to the primary face [0] and secondary face [1]
        /// </summary>
        Dictionary<Edge, (Face fillet, Edge[]? frontEnd, Edge[]? tangent)>? edgeToFillet;
        /// <summary>
        /// Tool to round the given edges of a shell with the given radius.
        /// </summary>
        /// <param name="shell"></param>
        /// <param name="edges"></param>
        /// <param name="radius"></param>
        public RoundEdges(Shell shell, IEnumerable<Edge> edges, double radius)
        {
            this.shell = shell;
            this.radius = Math.Abs(radius); // radius may not be negative
            // convex and concave: an edge is convex when the angle between the two faces is less than 108°, concave when greater than 180°
            convexEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Convex);
            concaveEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Concave);
        }

        public Shell? Execute()
        {
            // 1. make a raw fillet for each edge. The fillet is a swept circle around a spine curve. The spine curve is the intersection
            // of two offset surfaces of the adjacent faces. The fillet starts and ends with a circular arc and has two tangential edges to the adjacent faces.
            // Later we have to trim or extent the fillet faces to get a proper result. The end vertices of the edge lie in the planes of the front arcs.
            edgeToFillet = createFillets(convexEdges);
            // there are one or more edges meeting at a vertex.
            Dictionary<Vertex, List<Edge>> vertexToEdges = createVertexToEdges(convexEdges);

            List<HashSet<Face>> roundingShells = []; // each hashset contains the faces of one rounding shell: the fillet and maybe some patches
            foreach (var ve in vertexToEdges)
            {
                if (ve.Value.Count == 1)
                { // the fillet ends here, there are different cases:
                    // There is one or more "impact" faces
                    HashSet<Face>? filletAndExtension = createDeadEndExtension(ve.Key, ve.Value[0]);
                    if (filletAndExtension != null) roundingShells.Add(filletAndExtension);
                }
            }
            Combine(roundingShells);

            // here we have the convex rounded edges as shells, which have to be subtracted from the shell on which the edges are to be rounded
            Shell? toOperateOn = shell.Clone() as Shell;
            for (int i = 0; i < roundingShells.Count; i++)
            {
                Face[] s = roundingShells[i].ToArray();
                Shell.ConnectFaces(s, Precision.eps);
                Shell toIntersectWith = Shell.FromFaces(s);
                BooleanOperation bo = new BooleanOperation();
                bo.SetShells(toOperateOn, toIntersectWith, BooleanOperation.Operation.intersection);
                bo.SetClosedShells(true, false);
                // bo.SetTangentialEdges(tangentialEdges);
                Shell[] roundedShells = bo.Execute();
                if (roundedShells != null && roundedShells.Length == 1) toOperateOn = roundedShells[0];
            }

            return toOperateOn;
        }

        /// <summary>
        /// Create a fillet as a single face, consisting of a ISurfaceOfExtrusion surface (cylinder, torus, sweptCircle) and four edges:
        /// two arcs at the ends and two tangential curves to the adjacent faces
        /// </summary>
        /// <param name="edges"></param>
        /// <returns> a dictionary with
        /// - fillet: the face the bounds of the edge
        /// - frontEnd: the two arcs at the end of the fillet face
        /// - tangent: the edges tangential to the primary face [0] and secondary face [1]
        /// for each edge
        /// </returns>
        private Dictionary<Edge, (Face fillet, Edge[]? frontEnd, Edge[]? tangent)> createFillets(IEnumerable<Edge> edges)
        {
            Dictionary<Edge, (Face fillet, Edge[]? frontEnd, Edge[]? tangent)> edgeToFillet = new Dictionary<Edge, (Face fillet, Edge[]? frontEnd, Edge[]? tangent)>();
            foreach (Edge edgeToRound in edges)
            {
                Face? filletFace = ShellExtensions.MakeConvexFilletFace(edgeToRound, radius, out Edge[]? frontEnd, out Edge[]? tangential);
                if (filletFace != null)
                {
                    edgeToFillet[edgeToRound] = (filletFace, frontEnd, tangential);
                }
            }
            return edgeToFillet;
        }

        private Dictionary<Vertex, List<Edge>> createVertexToEdges(IEnumerable<Edge> edges)
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

        private HashSet<Face>? createDeadEndExtension(Vertex vtx, Edge edge)
        {
            if (edgeToFillet == null) return null;
            Face fillet = edgeToFillet[edge].fillet;
            ISurfaceOfExtrusion? filletSurface = fillet.Surface as ISurfaceOfExtrusion;
            if (filletSurface == null) return null;
            Edge? openEdge = edgeToFillet[edge].frontEnd?.MinBy(edg => edg.Curve3D.DistanceTo(vtx.Position));
            Ellipse? ellipse = openEdge?.Curve3D as Ellipse;
            // we need to check whether we need a face extension on the third (impact) face here
            if (ellipse == null) return null; // should not happen
            GeoPoint cnt = ellipse.Center;
            GeoVector edgeDirAtTheEnd; // the direction at the end of the fillet, at the position of this vertex, poiting outward in the extension of the fillet
            if (vtx == edge.Vertex1) edgeDirAtTheEnd = -edge.Curve3D.StartDirection.Normalized;
            else if (vtx == edge.Vertex2) edgeDirAtTheEnd = edge.Curve3D.EndDirection.Normalized;
            else return null; // this should not happen
            IEnumerable<Face> thirdFaces = vtx.InvolvedFaces.Except([edge.PrimaryFace, edge.SecondaryFace]);
            GeoVector dirOfImpactFaces = GeoVector.NullVector;
            foreach (Face fc in thirdFaces)
            {
                dirOfImpactFaces += fc.Surface.GetNormal(vtx.GetPositionOnFace(fc)).Normalized;
            }
            if (dirOfImpactFaces.IsNullVector()) return null; // no impact faces?
                                                              // we have to decide how to extent the fillet in this direction, there is no simple, definite and obvious solution
                                                              // first check, whether the two tangents of the fillet intersect the shell
            if (edgeToFillet[edge].tangent == null || edgeToFillet[edge].tangent?.Length != 2) return null; // there must be two tangents where the fillet touches the shell
            bool shortExtensionIsOk = true;
            bool allOutside = true;
            foreach (GeoPoint startPoint in new List<GeoPoint>([ellipse.PointAt(0.05), ellipse.PointAt(0.5), ellipse.PointAt(0.95)]))
            {
                // lets see, whether the extension intersects the shell close to the point
                if (shell.IsInside(startPoint)) // this point on the fillet ellipse is inside, check whether we can savely expand it
                {
                    allOutside = false;
                    GeoPoint[] extIntsect = shell.GetLineIntersection(startPoint, edgeDirAtTheEnd);
                    bool ok = false;
                    for (int i = 0; i < extIntsect.Length; i++)
                    {
                        double par = Geometry.LinePar(startPoint, edgeDirAtTheEnd, extIntsect[i]);
                        if (par > 0 && par < 2 * radius)
                        {
                            ok = true;
                            break;
                        }
                    }
                    if (!ok)
                    {
                        shortExtensionIsOk = false;
                        break;
                    }
                }
            }
            if (allOutside)
            {   // the whole ending arc seems to be outside the shell, no extension needed (impact face is bended towards the edge
                if (!curveIntersectsShell(ellipse)) return [fillet];
            }
            double impact = dirOfImpactFaces.Normalized * edgeDirAtTheEnd.Normalized;
            // if the impact faces point in the same direction as the fillet, they are probably trimmed by the fillet
            // if it points the other way, it may have to be extended
            Face? extruded = Make3D.Extrude(ellipse, 2 * radius * edgeDirAtTheEnd, null) as Face;
            if (shortExtensionIsOk && impact > 0 && extruded != null)
            {   // the linear extension of the fillet intersects the shell in less than 2*radius so lets try a simple cylindrical extension fo 2*radius
                return [extruded, fillet];
            }
            Face impactFace = vtx.Faces.Except([edge.PrimaryFace, edge.SecondaryFace]).First(); // how to deal with multiple?
            if (impact < 0 && impactFace != null)
            {   // we brutally end the fillet with a perpendicular plane surface
                // the vertex should lie in the ellipse plane
                if (Math.Abs(ellipse.Plane.Distance(vtx.Position)) < Precision.eps)
                {   // one edge of the plane is the ellipse, the other edges is a circular arc through the vertex and the two ellipse end points
                    //GeoPoint intersectionByImpactFace = GeoPoint.Invalid;
                    //double mindist = double.MaxValue;
                    //for (int i = 0; i < edgeToFillet[edge].tangent.Length; i++)
                    //{
                    //    impactFace.Surface.Intersect(edgeToFillet[edge].tangent[i].Curve3D, impactFace.Domain, out GeoPoint[] ips, out GeoPoint2D[] uvOnFace, out double[] uOnCurve);
                    //    for (int j = 0; j < ips.Length; j++)
                    //    {
                    //        if (uOnCurve[j] < 1.0 && uOnCurve[j] > 0.0)
                    //        {
                    //            double d = ips[j] | vtx.Position;
                    //            if (d < mindist)
                    //            {
                    //                intersectionByImpactFace = ips[j];
                    //            }
                    //        }
                    //    }
                    //}
                    //Plane plane = ellipse.Plane;
                    //if (intersectionByImpactFace.IsValid)
                    //{
                    //    ICurve spine = filletSurface.Axis(fillet.Domain);
                    //    double pos = spine.PositionOf(intersectionByImpactFace);
                    //    GeoVector dir = spine.DirectionAt(pos);
                    //    plane = new Plane(spine.PointAt(pos), spine.DirectionAt(pos));
                    //    BooleanOperation splitFillet = new BooleanOperation();
                    //    splitFillet.SetShells(Shell.FromFaces(fillet), Shell.FromFaces(Face.MakeFace(new PlaneSurface(plane),new BoundingRect(GeoPoint2D.Origin,2*radius,2*radius))),BooleanOperation.Operation.clip);
                    //    Shell[] splitted = splitFillet.Execute();
                    //}
                    /* so sollte es gehen: erzeuge die Flächen für ein Solid (nach innenorientiert) aus dem fillet, den beiden ebenen Endkappen und den beiden anliegenden Faces, die durch
                     * die abzurundende Kante, die Tangenten und die Kanten der Endkappe definiert sind. Die Endkappen entstehen durch den Endbogen und einem ebenen Schnitt (Ellipsenebene)
                     * mit den anliegenden Faces. Sollte dieser ebene Schnitt außerhal des jeweiligen anliegenden Faces liegen, dann Linie nehmen (wir befinden und dann im Körper.
                     */
                    Plane plane = new Plane(vtx.Position, edgeDirAtTheEnd); // origin is vtx.Position
                    Arc2D? filletEndArc = ellipse.GetProjectedCurve(plane) as Arc2D;
                    Line2D l1 = new Line2D(filletEndArc.EndPoint, GeoPoint2D.Origin);
                    Line2D l2 = new Line2D(GeoPoint2D.Origin, filletEndArc.StartPoint);

                        SimpleShape ss = new SimpleShape(Border.FromUnorientedList([filletEndArc,l1,l2], true));
                        PlaneSurface pln = new PlaneSurface(plane);
                        Face planarEnd = Face.MakeFace(pln, ss);
                        if (pln.Normal * edgeDirAtTheEnd > 0) planarEnd.ReverseOrientation();
                        //return [fillet, planarEnd];
                }
            }
            #region tried_impact_face
            //if (impact < 0 && impactFace != null && extruded != null)
            //{   // maybe we can extend the impact face to the rounding extension
            //    GeoPoint2D[] spInts = impactFace.Surface.GetLineIntersection(ellipse.StartPoint, edgeDirAtTheEnd);
            //    GeoPoint2D[] epInts = impactFace.Surface.GetLineIntersection(ellipse.EndPoint, edgeDirAtTheEnd);
            //    if (spInts != null && spInts.Length > 0 && epInts != null && epInts.Length > 0)
            //    {
            //        GeoPoint sp = GeoPoint.Invalid;
            //        GeoPoint ep = GeoPoint.Invalid;
            //        double mindist = double.MaxValue;
            //        for (int i = 0; i < spInts.Length; i++)
            //        {
            //            GeoPoint p = impactFace.Surface.PointAt(spInts[i]);
            //            double par = Geometry.LinePar(ellipse.StartPoint, edgeDirAtTheEnd, p);
            //            if (par > 0 && par < mindist)
            //            {
            //                mindist = par;
            //                sp = p;
            //            }
            //        }
            //        mindist = double.MaxValue;
            //        for (int i = 0; i < spInts.Length; i++)
            //        {
            //            GeoPoint p = impactFace.Surface.PointAt(epInts[i]);
            //            double par = Geometry.LinePar(ellipse.EndPoint, edgeDirAtTheEnd, p);
            //            if (par >= 0 && par < mindist)
            //            {
            //                mindist = par;
            //                ep = p;
            //            }
            //        }
            //        if (sp.IsValid && ep.IsValid)
            //        {
            //            // both edges of the extruded face lie on the impact face.
            //            // now there must be two edges of the impact face at this vertex, which contain the points sp and ep
            //            List<Edge> edgesOnImpactFace = vtx.EdgesOnFace(impactFace);
            //            List<ICurve2D> curvesForImpactFaceExtension = [];
            //            for (int i = 0; i < edgesOnImpactFace.Count; i++)
            //            {
            //                if (edgesOnImpactFace[i].Curve3D.DistanceTo(sp) < Precision.eps)
            //                {
            //                    ICurve curve = edgesOnImpactFace[i].Curve3D.Clone();
            //                    double pos = curve.PositionOf(sp);
            //                    if (edgesOnImpactFace[i].Vertex1 == vtx) curve.Trim(0, pos);
            //                    else curve.Trim(pos, 1.0);
            //                    curvesForImpactFaceExtension.Add(impactFace.Surface.GetProjectedCurve(curve, 0.0));
            //                }
            //            }
            //            if (curvesForImpactFaceExtension.Count == 2)
            //            {
            //                // now create a face from the two curves and the intersection of the extruded face with the impactFace
            //                BoundingRect ext = impactFace.Domain;
            //                GeoPoint2D uv = impactFace.Surface.PositionOf(sp);
            //                SurfaceHelper.AdjustPeriodic(impactFace.Surface, ext, ref uv);
            //                ext.MinMax(uv);
            //                uv = impactFace.Surface.PositionOf(ep);
            //                SurfaceHelper.AdjustPeriodic(impactFace.Surface, ext, ref uv);
            //                ext.MinMax(uv);
            //                IDualSurfaceCurve[] iscurve = impactFace.Surface.GetDualSurfaceCurves(ext, extruded.Surface, extruded.Domain, [sp, ep]);
            //                if (iscurve != null && iscurve.Length == 1)
            //                {
            //                    curvesForImpactFaceExtension.Add(iscurve[0].Curve2D1);
            //                    Face? extendedImpactFace = Face.MakeFace(impactFace.Surface.Clone(), [curvesForImpactFaceExtension]);
            //                    if (extendedImpactFace != null)
            //                    {
            //                        return [extendedImpactFace, fillet];
            //                    }
            //                }
            //            }
            //        }
            //    }
            //}
            #endregion
            // if nothing worked out, 
            GeoVector torusXAxis = (vtx.Position - cnt).Normalized;
            ToroidalSurface ts = new ToroidalSurface(vtx.Position, torusXAxis, edgeDirAtTheEnd.Normalized, -(torusXAxis ^ edgeDirAtTheEnd).Normalized, vtx.Position | cnt, radius);
            // the ellipse lies on the torus surface, but we need to define a domain that covers the ellipse
            GeoPoint2D uvsp = ts.PositionOf(ellipse.StartPoint);
            GeoPoint2D uvep = ts.PositionOf(ellipse.EndPoint);
            BoundingRect torusDomain = new BoundingRect(uvep, uvsp); // this should work without periodic adjustments, because we are at the inside of the torus surface
            torusDomain.Left = torusDomain.Right - Math.PI; // a 180° segment is sufficient
            Face torusFace = Face.MakeFace(ts, torusDomain);
            torusFace.ReverseOrientation();
            return [torusFace, fillet];
        }
        private bool curveIntersectsShell(ICurve curve)
        {
            for (int i = 0; i < shell.Faces.Length; i++)
            {
                shell.Faces[i].Intersect(curve, out GeoPoint[] ip, out GeoPoint2D[] uvOnFace, out double[] uOnCurve);
                for (int k = 0; k < uOnCurve.Length; k++)
                {
                    if (-Precision.eps <= uOnCurve[k] && uOnCurve[k] <= 1.0 + Precision.eps && shell.Faces[i].Contains(ref uvOnFace[k], true)) return true;
                }
            }
            return false;
        }
        /// <summary>
        /// Combine overlapping sets into single sets
        /// </summary>
        /// <param name="sets"></param>
        private void Combine(List<HashSet<Face>> sets)
        {
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
    }
}
