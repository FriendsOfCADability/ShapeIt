using CADability;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using ExCSS;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms.VisualStyles;
using static ShapeIt.ShellExtensions;

namespace ShapeIt
{
    internal class RoundEdges : BlendEdges
    {
        double radius;
        /// <summary>
        /// - fillet: the face which rounds the edge
        /// - frontEnd: the two arcs at the end of the fillet face
        /// - tangent: the edges tangential to the primary face [0] and secondary face [1]
        /// </summary>
        /// <summary>
        /// Tool to round the given edges of a shell with the given radius.
        /// </summary>
        /// <param name="shell"></param>
        /// <param name="edges"></param>
        /// <param name="radius"></param>
        public RoundEdges(Shell shell, IEnumerable<Edge> edges, double radius) : base(shell, edges)
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
            edgeToCutter = createFillets(convexEdges);
            // there are one or more edges meeting at a vertex.
            Dictionary<Vertex, List<Edge>> vertexToEdges = createVertexToEdges(convexEdges);

            List<HashSet<Shell>> roundingShells = []; // each hashset contains the faces of one rounding shell: the fillet and maybe some patches
            foreach (var ve in vertexToEdges)
            {
                if (ve.Value.Count == 1)
                { // the fillet ends here, there are different cases:
                    // There is one or more "impact" faces
                    HashSet<Shell>? filletAndExtension = createDeadEndExtension(ve.Key, ve.Value[0], radius);
                    if (filletAndExtension != null) roundingShells.Add(filletAndExtension);
                }
                else if (ve.Value.Count == 2)
                {
                    HashSet<Shell>? filletAndExtension = createExtensionTwoEdges(ve.Key, ve.Value[0], ve.Value[1], 2 * radius);
                    if (filletAndExtension != null) roundingShells.Add(filletAndExtension);
                }
                else if (ve.Value.Count == 3)
                {
                    HashSet<Shell>? filletAndExtension = createExtensionThreeEdges(ve.Key, ve.Value[0], ve.Value[1], ve.Value[2]);
                    if (filletAndExtension != null) roundingShells.Add(filletAndExtension);
                }
            }
            Combine(roundingShells);

            // here we have the convex rounded edges as shells, which have to be subtracted from the shell on which the edges are to be rounded
            Shell? toOperateOn = shell.Clone() as Shell;
            for (int i = 0; i < roundingShells.Count; i++)
            {
                foreach (var item in roundingShells[i])
                {
                    BooleanOperation bo = new BooleanOperation();
                    bo.SetShells(toOperateOn, item, BooleanOperation.Operation.difference);

                    Shell[] roundedShells = bo.Execute();
                    if (roundedShells != null && roundedShells.Length == 1) toOperateOn = roundedShells[0];
                }
            }

            return toOperateOn;
        }

        private HashSet<Shell>? createExtensionThreeEdges(Vertex vtx, Edge edge1, Edge edge2, Edge edge3)
        {
            // find the fillets for these edges
            Shell? fillet1 = edgeToCutter?[edge1];
            Shell? fillet2 = edgeToCutter?[edge2];
            Shell? fillet3 = edgeToCutter?[edge3];
            if (fillet1 == null || fillet2 == null || fillet3 == null) return null;

            // find the end faces of the fillets at this vertex
            Face? endFace1 = fillet1.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            Face? endFace2 = fillet2.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            Face? endFace3 = fillet3.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            if (endFace1 == null || endFace2 == null || endFace3 == null) return null;
            // find the swept faces of the fillets
            Face? sweptFace1 = fillet1.Faces.Where(f => f.UserData.Contains("CADability.Cutter.SweptFace")).TheOnlyOrDefault();
            Face? sweptFace2 = fillet2.Faces.Where(f => f.UserData.Contains("CADability.Cutter.SweptFace")).TheOnlyOrDefault();
            Face? sweptFace3 = fillet3.Faces.Where(f => f.UserData.Contains("CADability.Cutter.SweptFace")).TheOnlyOrDefault();
            if (sweptFace1 == null || sweptFace2 == null || sweptFace3 == null) return null;
            ISurfaceOfExtrusion? sweptSurface1 = sweptFace1.Surface as ISurfaceOfExtrusion;
            ISurfaceOfExtrusion? sweptSurface2 = sweptFace2.Surface as ISurfaceOfExtrusion;
            ISurfaceOfExtrusion? sweptSurface3 = sweptFace3.Surface as ISurfaceOfExtrusion;
            if (sweptSurface1 == null || sweptSurface2 == null || sweptSurface3 == null) return null;
            ICurve spine1 = sweptSurface1.Axis(sweptFace1.Domain);
            ICurve spine2 = sweptSurface2.Axis(sweptFace2.Domain);
            ICurve spine3 = sweptSurface3.Axis(sweptFace3.Domain);
            var cp = Curves.FindCommonPoint([spine1, spine2, spine3], vtx.Position, null, 1E-6, 1E-6, 1E-8, 100);
            if (cp.SumOfSquaredDistances<Precision.eps)
            {
                // this is the center of the sphere, no we need the three arcs at the end faces to define the sphere
                ICurve arc1 = sweptSurface1.ExtrusionDirectionIsV? sweptFace1.Surface.FixedV(cp.Parameters[0], sweptFace1.Domain.Left, sweptFace1.Domain.Right) 
                    : sweptFace1.Surface.FixedU(cp.Parameters[0], sweptFace1.Domain.Bottom, sweptFace1.Domain.Top);
                ICurve arc2 = sweptSurface2.ExtrusionDirectionIsV ? sweptFace2.Surface.FixedV(cp.Parameters[1], sweptFace2.Domain.Left, sweptFace2.Domain.Right)
                    : sweptFace2.Surface.FixedU(cp.Parameters[1], sweptFace2.Domain.Bottom, sweptFace2.Domain.Top);
                ICurve arc3 = sweptSurface3.ExtrusionDirectionIsV ? sweptFace3.Surface.FixedV(cp.Parameters[2], sweptFace3.Domain.Left, sweptFace3.Domain.Right)
                    : sweptFace3.Surface.FixedU(cp.Parameters[2], sweptFace3.Domain.Bottom, sweptFace3.Domain.Top);

            }
            double[] si1 = Curves.Intersect(spine1, spine2, true);
            double[] si2 = Curves.Intersect(spine2, spine3, true);
            double[] si3 = Curves.Intersect(spine3, spine1, true);
            GeoPoint p1 = spine1.PointAt(si1[0]);
            GeoPoint p2 = spine2.PointAt(si2[0]);
            GeoPoint p3 = spine3.PointAt(si3[0]);

            return null;
        }

        protected Shell? CreateConcavePatchX(Shell fillet1Shell, Shell fillet2Shell, Face commonFace, Vertex vtx, Edge edge1, Edge edge2)
        {
            // this is the connection of two fillets which meet at the vertex
            // try to make a torus-like segment to fill the gap
            Edge? thirdEdge = vtx.AllEdges.Except([edge1, edge2]).TheOnlyOrDefault();
            // in most cases we have a vertex with three faces meeting, so one common face and one third edge
            // if there are more than three faces meeting at the vertex, we cannot handle this currently
            if (commonFace == null || thirdEdge == null) return null;
            GeoVector normalCommon = commonFace.Surface.GetNormal(vtx.GetPositionOnFace(commonFace)).Normalized;
            // for brevity:
            Face touchedByFillet1 = Edge.CommonFace(edge1, thirdEdge);
            Face touchedByFillet2 = Edge.CommonFace(edge2, thirdEdge);
            GeoVector normal1 = touchedByFillet1.Surface.GetNormal(vtx.GetPositionOnFace(touchedByFillet1)).Normalized;
            GeoVector normal2 = touchedByFillet2.Surface.GetNormal(vtx.GetPositionOnFace(touchedByFillet2)).Normalized;
            bool forwardConnected;
            if (edge1.EndVertex(commonFace) == edge2.StartVertex(commonFace)) forwardConnected = true;
            else if (edge1.StartVertex(commonFace) == edge2.EndVertex(commonFace)) forwardConnected = false;
            else return null; // edges are not connected properly
            double orientation = (forwardConnected ? 1 : -1) * normalCommon * (normal1 ^ normal2); // >0: convex, <0: concave, ==0: tangential
            Face? fillet1 = fillet1Shell.Faces.Where(f => f.UserData.Contains("CADability.Cutter.SweptFace")).TheOnlyOrDefault();
            Face? fillet2 = fillet2Shell.Faces.Where(f => f.UserData.Contains("CADability.Cutter.SweptFace")).TheOnlyOrDefault();
            ICurve? spine1 = (fillet1.Surface as ISurfaceOfExtrusion)?.Axis(fillet1.Domain);
            ICurve? spine2 = (fillet2.Surface as ISurfaceOfExtrusion)?.Axis(fillet2.Domain);
            // the fillets "overshoot" the leading edge, which they are rounding, so we should find an intersection with the third edge.
            // the fillets should stop ehere and a toroidal surface should connect the two fillets
            // it is a tangential intersection here, there should be only one intersection point
            fillet1.Surface.Intersect(thirdEdge.Curve3D, fillet1.Domain, out GeoPoint[] ips1, out GeoPoint2D[] uvs1, out double[] uOnCurve1);
            fillet2.Surface.Intersect(thirdEdge.Curve3D, fillet2.Domain, out GeoPoint[] ips2, out GeoPoint2D[] uvs2, out double[] uOnCurve2);
            if (ips1 == null || ips2 == null || ips1.Length != 1 || ips2.Length != 1) return null; // no intersection found, should not happen
            ISurface aroundThirdEdge = SweptCircle.MakePipeSurface(thirdEdge.Curve3D, radius, -(fillet1.Surface.GetNormal(uvs1[0]).Normalized + fillet2.Surface.GetNormal(uvs2[0]).Normalized));
            BoundingRect aroundThirdEdgeDomain;
            if ((aroundThirdEdge as ISurfaceOfExtrusion)!.ExtrusionDirectionIsV) aroundThirdEdgeDomain = new BoundingRect(0, 0, 2 * Math.PI, 1);
            else aroundThirdEdgeDomain = new BoundingRect(0, 0, 1, 2 * Math.PI);
            // two points (start and endpoint) on the toroidal spine:
            GeoPoint tor1 = ips1[0] - radius * fillet1.Surface.GetNormal(uvs1[0]).Normalized;
            GeoPoint tor2 = ips2[0] - radius * fillet2.Surface.GetNormal(uvs2[0]).Normalized;
#if DEBUG
            Face dbgAround = Face.MakeFace(aroundThirdEdge, aroundThirdEdgeDomain);
#endif

            ISurface offset = commonFace.Surface.GetOffsetSurface(-radius);
            if (offset != null)
            {
                IDualSurfaceCurve[] dsc = offset.GetDualSurfaceCurves(commonFace.Domain, aroundThirdEdge, aroundThirdEdgeDomain, [ips1[0], ips2[0]]);
                ICurve? toroidalSpine = null;
                double minLength = double.MaxValue;
                for (int i = 0; i < dsc.Length; i++)
                {
                    if (dsc[i].Curve3D.IsClosed)
                    {   // we might cross to 1/0 bound
                        // but it should not cross as we have choosen the seam of aroundThirdEdge accordingly
                    }
                    ICurve ts = dsc[i].Curve3D.Clone();
                    double pos1 = ts.PositionOf(tor1);
                    double pos2 = ts.PositionOf(tor2);
                    if (Math.Abs(pos2 - pos1) > 0.5 && ts.IsClosed)
                    {
                        ts.Trim(Math.Max(pos1, pos2), Math.Min(pos1, pos2));
                        if ((ts.StartPoint | tor1) + (ts.EndPoint | tor2) > (ts.StartPoint | tor2) + (ts.EndPoint | tor1)) ts.Reverse();
                    }
                    else
                    {
                        if (pos1 > pos2)
                        {
                            ts.Reverse();
                            pos1 = 1 - pos1;
                            pos2 = 1 - pos2;
                        }
                        ts.Trim(pos1, pos2);
                    }
                    if (ts.Length < minLength)
                    {
                        minLength = ts.Length;
                        toroidalSpine = ts;
                    }
                }
                if (toroidalSpine != null)
                {
                    ISurface connectingToroid = SweptCircle.MakePipeSurface(toroidalSpine, radius, commonFace.Surface.GetNormal(commonFace.Surface.PositionOf(vtx.Position)));
                    Face dbgpatch = Face.MakeFace(connectingToroid, new BoundingRect(0, -Math.PI / 2, 1, Math.PI / 2));
                    return null;
                    PlaneSurface pln1 = new PlaneSurface(new Plane(toroidalSpine.StartPoint, -toroidalSpine.StartDirection));
                    Face toClipWith1 = Face.MakeFace(pln1, new BoundingRect(GeoPoint2D.Origin, radius * 1.1, radius * 1.1));
                    PlaneSurface pln2 = new PlaneSurface(new Plane(toroidalSpine.EndPoint, toroidalSpine.EndDirection));
                    Face toClipWith2 = Face.MakeFace(pln2, new BoundingRect(GeoPoint2D.Origin, radius * 1.1, radius * 1.1));
                    Face fillet1Clipped = BooleanOperation.ClipFace(fillet1, toClipWith1).TheOnlyOrDefault();
                    Face fillet2Clipped = BooleanOperation.ClipFace(fillet2, toClipWith2).TheOnlyOrDefault();
                    bool tangentialEdgeGoesFrom1to2 = false; // the tangential edge on the common face connects fillet1 and fillet2, but in which direction?
                    if (fillet1Clipped != null && fillet2Clipped != null)
                    {
                        // create the torus face, which is not a real torus but a segment of a swept circle surface
                        // the two fillet faces have two edges of the torus face a third edge is the (tangential) intersection with the common face, the fourth
                        // edge is the segment of the third edge
                        // find the edges
                        Edge? arcEdgeOn1 = null, arcEdgeOn2 = null, tangentialEdge = null, thirdEdgeSegment = null;
                        GeoPoint tangentialp1 = GeoPoint.Invalid, tangentialp2 = GeoPoint.Invalid; // points on the tangential edge
                        GeoPoint thirdEdgep1 = GeoPoint.Invalid, thirdEdgep2 = GeoPoint.Invalid; // points on the third edge
                        Face torusFace = Face.Construct();
                        foreach (Edge edge in fillet1Clipped.AllEdges)
                        {
                            if (Math.Abs(pln1.GetDistance(edge.Vertex1.Position)) < Precision.eps && Math.Abs(pln1.GetDistance(edge.Vertex2.Position)) < Precision.eps)
                            {
                                ICurve2D onct = connectingToroid.GetProjectedCurve(edge.Curve3D, 0.0);
                                if (edge.Forward(fillet1Clipped))
                                {
                                    onct.Reverse();
                                    edge.SetFace(torusFace, onct, false);
                                }
                                else
                                {
                                    edge.SetFace(torusFace, onct, true);
                                }
                                arcEdgeOn1 = edge;
                                if (Math.Abs(commonFace.Surface.GetDistance(edge.StartVertex(fillet1Clipped).Position)) < Precision.eps)
                                {
                                    tangentialp1 = edge.StartVertex(fillet1Clipped).Position;
                                    tangentialEdgeGoesFrom1to2 = false;
                                }
                                else thirdEdgep1 = edge.StartVertex(fillet1Clipped).Position;
                                if (Math.Abs(commonFace.Surface.GetDistance(edge.EndVertex(fillet1Clipped).Position)) < Precision.eps)
                                {
                                    tangentialp1 = edge.EndVertex(fillet1Clipped).Position;
                                    tangentialEdgeGoesFrom1to2 = true;
                                }
                                else thirdEdgep1 = edge.EndVertex(fillet1Clipped).Position;
                            }
                        }
                        foreach (Edge edge in fillet2Clipped.AllEdges)
                        {
                            if (Math.Abs(pln2.GetDistance(edge.Vertex1.Position)) < Precision.eps && Math.Abs(pln2.GetDistance(edge.Vertex2.Position)) < Precision.eps)
                            {
                                ICurve2D onct = connectingToroid.GetProjectedCurve(edge.Curve3D, 0.0);
                                if (edge.Forward(fillet2Clipped))
                                {
                                    onct.Reverse();
                                    edge.SetFace(torusFace, onct, false);
                                }
                                else
                                {
                                    edge.SetFace(torusFace, onct, true);
                                }
                                arcEdgeOn2 = edge;
                                if (Math.Abs(commonFace.Surface.GetDistance(edge.Vertex1.Position)) < Precision.eps) tangentialp2 = edge.Vertex1.Position;
                                else thirdEdgep2 = edge.Vertex1.Position;
                                if (Math.Abs(commonFace.Surface.GetDistance(edge.Vertex2.Position)) < Precision.eps) tangentialp2 = edge.Vertex2.Position;
                                else thirdEdgep2 = edge.Vertex2.Position;
                            }
                        }
                        if (arcEdgeOn1 == null || arcEdgeOn2 == null) return null; // no edges found, should not happen
                        if (!tangentialp1.IsValid || !tangentialp2.IsValid) return null; // we need two points to make the tangential edge
                                                                                         // find a domain on connectingToroid. it is in both directions (u and v) less than 180°
                                                                                         // so we can adjust the uv points close to each other
                        BoundingRect toroidDomain = BoundingRect.EmptyBoundingRect;
                        foreach (Edge edg in (List<Edge>)[arcEdgeOn1, arcEdgeOn2])
                        {
                            GeoPoint2D uv = connectingToroid.PositionOf(edg.Vertex1.Position);
                            if (!toroidDomain.IsEmpty()) SurfaceHelper.AdjustPeriodic(connectingToroid, toroidDomain, ref uv);
                            toroidDomain.MinMax(uv);
                            uv = connectingToroid.PositionOf(edg.Vertex2.Position);
                            SurfaceHelper.AdjustPeriodic(connectingToroid, toroidDomain, ref uv);
                            toroidDomain.MinMax(uv);
                        }
                        // we must have two points on the common face here and two curves (arcs)

                        // construct the edge segment on the third edge between the two intersection points
                        double pos1 = thirdEdge.Curve3D.PositionOf(ips1[0]);
                        double pos2 = thirdEdge.Curve3D.PositionOf(ips2[0]);
                        if (pos1 > pos2) (pos1, pos2) = (pos2, pos1);
                        ICurve tes = thirdEdge.Curve3D.Clone();
                        tes.Trim(pos1, pos2); // this is the connection between the two arcs on thirdEdge
                        if (tangentialEdgeGoesFrom1to2)
                        {
                            if ((tes.StartPoint | thirdEdgep1) + (tes.EndPoint | thirdEdgep2) > (tes.StartPoint | thirdEdgep2) + (tes.EndPoint | thirdEdgep1)) tes.Reverse();
                        }
                        else
                        {
                            if ((tes.StartPoint | thirdEdgep2) + (tes.EndPoint | thirdEdgep1) > (tes.StartPoint | thirdEdgep1) + (tes.EndPoint | thirdEdgep2)) tes.Reverse();
                        }
                        thirdEdgeSegment = new Edge(torusFace, tes, torusFace, connectingToroid.GetProjectedCurve(tes, 0.0), true);

                        // construct the tangential edge on the common face
                        IDualSurfaceCurve? onCommonFace = connectingToroid.GetDualSurfaceCurves(toroidDomain, commonFace.Surface, commonFace.Domain, [tangentialp1, tangentialp2]).TheOnlyOrDefault();
                        if (onCommonFace == null)
                        {
                            onCommonFace = new InterpolatedDualSurfaceCurve(commonFace.Surface, commonFace.Domain, connectingToroid, toroidDomain, tangentialp1, tangentialp2, true);
                        }
                        if (onCommonFace == null) return null; // there must be a tangential intersection between the connecting toroid and the common face
                        if (tangentialEdgeGoesFrom1to2) onCommonFace.Trim(tangentialp2, tangentialp1);
                        else onCommonFace.Trim(tangentialp1, tangentialp2);
                        tangentialEdge = new Edge(torusFace, onCommonFace.Curve3D, torusFace, onCommonFace.Curve2D1, true);
#if DEBUG
                        DebuggerContainer dbg3d = new DebuggerContainer();
                        dbg3d.Add(arcEdgeOn1.Curve3D as IGeoObject, arcEdgeOn1.GetHashCode());
                        dbg3d.Add(arcEdgeOn2.Curve3D as IGeoObject, arcEdgeOn2.GetHashCode());
                        dbg3d.Add(tangentialEdge.Curve3D as IGeoObject, tangentialEdge.GetHashCode());
                        dbg3d.Add(thirdEdgeSegment.Curve3D as IGeoObject, thirdEdgeSegment.GetHashCode());
                        DebuggerContainer dbg2d = new DebuggerContainer();
                        dbg2d.Add(arcEdgeOn1.Curve2D(torusFace), System.Drawing.Color.Red, arcEdgeOn1.GetHashCode());
                        dbg2d.Add(arcEdgeOn2.Curve2D(torusFace), System.Drawing.Color.Red, arcEdgeOn2.GetHashCode());
                        dbg2d.Add(tangentialEdge.Curve2D(torusFace), System.Drawing.Color.Red, tangentialEdge.GetHashCode());
                        dbg2d.Add(thirdEdgeSegment.Curve2D(torusFace), System.Drawing.Color.Red, thirdEdgeSegment.GetHashCode());
#endif

                        // construct the "torus" face
                        if (tangentialEdgeGoesFrom1to2) torusFace.Set(connectingToroid, [[arcEdgeOn2, tangentialEdge, arcEdgeOn1, thirdEdgeSegment]], false);
                        else torusFace.Set(connectingToroid, [[arcEdgeOn1, tangentialEdge, arcEdgeOn2, thirdEdgeSegment]], false);
                        // now we substitute fillet1 and fillet2 by fillet1Clipped and fillet2Clipped
                        // we must do this in edgeToFillet and patchToFillets

                    }
                }
            }
            return null;
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
        private Dictionary<Edge, Shell> createFillets(IEnumerable<Edge> edges)
        {
            Dictionary<Edge, Shell> edgeToFillet = new();
            foreach (Edge edgeToRound in edges)
            {
                Shell? filletShell = MakeConvexFilletShell(edgeToRound, radius);
                if (filletShell != null)
                {
                    filletShell.CopyAttributes(edgeToRound.PrimaryFace);
                    edgeToFillet[edgeToRound] = filletShell;
                }
            }
            return edgeToFillet;
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
        private Shell? MakeConvexFilletShell(Edge edgeToRound, double radius)
        {
            ISurface topSurface = edgeToRound.PrimaryFace.Surface; // for previty
            ISurface bottomSurface = edgeToRound.SecondaryFace.Surface;
            ICurve leadingEdge = edgeToRound.Curve3D.Clone();
            if (!edgeToRound.Forward(edgeToRound.PrimaryFace)) leadingEdge.Reverse(); // always forward on topSurface
            ISurface topOffset = topSurface.GetOffsetSurface(-radius);
            ISurface bottomOffset = bottomSurface.GetOffsetSurface(-radius);
            if (topOffset == null || bottomOffset == null) return null; // e.g. a sphere shrinking to a point
            topOffset.SetBounds(edgeToRound.PrimaryFace.Domain);
            bottomOffset.SetBounds(edgeToRound.SecondaryFace.Domain);
            // construct the two planes at the front and end side of the fillet
            // we did move them a little bit outwards but rejected this solution again, because we need it at the exact endposition sometimes
            PlaneSurface leftPlane = new PlaneSurface(new Plane(leadingEdge.StartPoint, -leadingEdge.StartDirection));
            PlaneSurface rightPlane = new PlaneSurface(new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection));
            GeoPoint filletAxisLeft = leadingEdge.StartPoint; // a first guess for the intersection, typically a good start
            GeoPoint filletAxisRight = leadingEdge.EndPoint; // the start and endpoint of the axis (spine) of the swept circle
            BoundingRect plnBounds = new BoundingRect(GeoPoint2D.Origin, radius, radius);
            BoundingRect topDomain = edgeToRound.PrimaryFace.Domain; // the domains of the top and bottom surfaces may be a little bit bigger than the original domains
            BoundingRect bottomDomain = edgeToRound.SecondaryFace.Domain;
            if (!CADability.GeoObject.Surfaces.IntersectThreeSurfaces(topOffset, edgeToRound.PrimaryFace.Domain, bottomOffset, edgeToRound.SecondaryFace.Domain, leftPlane, plnBounds,
                ref filletAxisLeft, out GeoPoint2D uv11, out GeoPoint2D uv12, out GeoPoint2D uv13)) return null;
            SurfaceHelper.AdjustPeriodic(topSurface, topDomain, ref uv11);
            SurfaceHelper.AdjustPeriodic(bottomSurface, bottomDomain, ref uv12);
            topDomain.MinMax(uv11);
            bottomDomain.MinMax(uv12);
            if (!CADability.GeoObject.Surfaces.IntersectThreeSurfaces(topOffset, edgeToRound.PrimaryFace.Domain, bottomOffset, edgeToRound.SecondaryFace.Domain, rightPlane, plnBounds,
                ref filletAxisRight, out GeoPoint2D uv21, out GeoPoint2D uv22, out GeoPoint2D uv23)) return null;
            SurfaceHelper.AdjustPeriodic(topSurface, topDomain, ref uv21);
            SurfaceHelper.AdjustPeriodic(bottomSurface, bottomDomain, ref uv22);
            topDomain.MinMax(uv21);
            bottomDomain.MinMax(uv22);
            IDualSurfaceCurve? filletAxisCurve = topOffset.GetDualSurfaceCurves(topDomain, bottomOffset, bottomDomain, new List<GeoPoint>([filletAxisLeft, filletAxisRight]))
                .MinBy(dsc => dsc.Curve3D.DistanceTo(filletAxisLeft) + dsc.Curve3D.DistanceTo(filletAxisRight));
            if (filletAxisCurve == null) return null;
            filletAxisCurve.Trim(filletAxisLeft, filletAxisRight);
            ISurface sweptCircle;
            sweptCircle = SweptCircle.MakePipeSurface(filletAxisCurve.Curve3D, radius, filletAxisCurve.Curve3D.PointAt(0.5) - leadingEdge.PointAt(0.5));
            ISurfaceOfExtrusion? sweptCircleExtrusion = sweptCircle as ISurfaceOfExtrusion;
            if (sweptCircleExtrusion == null) return null; // sweptCircle must always be a ISurfaceOfExtrusion. this is to satisfy the compiler

            // we need bounds for sweptCircle to enable Makeface to use BoxedSurface methods
            // we use the middle point of the axis to round as a starting position for the bounds
            GeoPoint2D uvm = sweptCircle.PositionOf(edgeToRound.Curve3D.PointAt(0.3));
            BoundingRect ext = new BoundingRect(uvm);
            uvm = sweptCircle.PositionOf(edgeToRound.Curve3D.PointAt(0.7));
            SurfaceHelper.AdjustPeriodic(sweptCircle, ext, ref uvm);
            ext.MinMax(uvm);
            sweptCircle.SetBounds(ext);
            uvm = sweptCircle.PositionOf(edgeToRound.Curve3D.StartPoint); // sweptCircle mus adjust the period according to its bounds
            ext.MinMax(uvm);
            uvm = sweptCircle.PositionOf(edgeToRound.Curve3D.EndPoint); // sweptCircle mus adjust the period according to its bounds
            ext.MinMax(uvm);
            // the extent goes along the spine of the sweptCircle, in the other direction (of the circle) we go from -90° to +90° relative to the edge
            if (sweptCircleExtrusion.ExtrusionDirectionIsV)
            {
                ext.Left -= Math.PI / 2;
                ext.Right += Math.PI / 2;
            }
            else
            {
                ext.Bottom -= Math.PI / 2;
                ext.Top += Math.PI / 2;
            }
            sweptCircle.SetBounds(ext);
#if DEBUG
            GeoObjectList dbgsws = (sweptCircle as ISurfaceImpl).DebugGrid;
            GeoObjectList dbgswd = (sweptCircle as ISurfaceImpl).DebugDirectionsGrid;
#endif
            Face sweptFace;
            // how to test for correct orientation of the sweptCircle?
            // The normal of the swept circle must point towards the filletAxisCurve, it must be a concave surface 
            GeoPoint facnt = filletAxisCurve.Curve3D.PointAt(0.5);
            GeoPoint2D facnt2d = sweptCircle.PositionOf(facnt);
            GeoVector testNormal = sweptCircle.GetNormal(facnt2d);
            GeoPoint testPoint = sweptCircle.PointAt(facnt2d);
            if (testNormal * (filletAxisCurve.Curve3D.PointAt(0.5) - testPoint) < 0) sweptCircle.ReverseOrientation();
            // find the tangential curves at the "long" sides of the sweptSurface

            GeoPoint2D uvswc, uvs;
            GeoPoint rb, rt;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(rightPlane.Location, rightPlane.Normal, sweptCircle, bottomSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = bottomSurface.PointAt(uvs);
                rb = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(rightPlane.Location, rightPlane.Normal, sweptCircle, topSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = topSurface.PointAt(uvs);
                rt = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            Arc2D arc2DOnRightPlane = new Arc2D(rightPlane.PositionOf(filletAxisRight), radius, rightPlane.PositionOf(rb), rightPlane.PositionOf(rt), false);
            ICurve lid2crv3 = rightPlane.Make3dCurve(arc2DOnRightPlane);
            lid2crv3.StartPoint = rb;
            lid2crv3.EndPoint = rt;
            lid2crv3.Reverse();
            Edge e2 = new Edge(null, lid2crv3, null, sweptCircle.GetProjectedCurve(lid2crv3, 0.0), true);


            GeoPoint lb, lt;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(leftPlane.Location, leftPlane.Normal, sweptCircle, bottomSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = bottomSurface.PointAt(uvs);
                lb = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(leftPlane.Location, leftPlane.Normal, sweptCircle, topSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = topSurface.PointAt(uvs);
                lt = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            Arc2D arc2DOnLeftPlane = new Arc2D(leftPlane.PositionOf(filletAxisLeft), radius, leftPlane.PositionOf(lt), leftPlane.PositionOf(lb), false);
            ICurve lid1crv3 = leftPlane.Make3dCurve(arc2DOnLeftPlane);
            lid1crv3.StartPoint = lt; // for better precision, should be almost equal
            lid1crv3.EndPoint = lb;
            lid1crv3.Reverse();
            Edge e4 = new Edge(null, lid1crv3, null, sweptCircle.GetProjectedCurve(lid1crv3, 0.0), true);
            // if sweptCircle has a periodic spine, which is more than half the period, it is not egnough to use the middle point and the two endpoints
            // for calculationg the domain. instead we use points at 0.3 and 0.7, which is with a distanc of 0.4 less than half the period
            // and the other points are close to the bounds of the interval
            GeoPoint2D cntuv = sweptCircle.PositionOf(edgeToRound.Curve3D.PointAt(0.3));
            BoundingRect sweptBounds = new BoundingRect(cntuv);
            foreach (GeoPoint p in new List<GeoPoint>([edgeToRound.Curve3D.PointAt(0.7), rt, rb, lt, lb]))
            {
                GeoPoint2D uv = sweptCircle.PositionOf(p);
                SurfaceHelper.AdjustPeriodic(sweptCircle, sweptBounds, ref uv);
                sweptBounds.MinMax(uv);
            }

            //BoundingRect sweptBounds = new BoundingRect(sweptCircle.PositionOf(rt), sweptCircle.PositionOf(rb), sweptCircle.PositionOf(lt), sweptCircle.PositionOf(lb));
            IDualSurfaceCurve[] tcCandidates = topSurface.GetDualSurfaceCurves(edgeToRound.PrimaryFace.Domain, sweptCircle, sweptBounds, new List<GeoPoint>([lt, rt]));
            if (tcCandidates == null || tcCandidates.Length == 0) return null;
            IDualSurfaceCurve[] bcCandidates = bottomSurface.GetDualSurfaceCurves(edgeToRound.SecondaryFace.Domain, sweptCircle, sweptBounds, new List<GeoPoint>([rb, lb]));
            if (bcCandidates == null || bcCandidates.Length == 0) return null;
            if (tcCandidates.Length > 1)
            {
                // select best solution here
            }
            if (bcCandidates.Length > 1)
            {
                // select best solution here
            }
            tcCandidates[0].Trim(lt, rt);
            bcCandidates[0].Trim(rb, lb);
            if ((bcCandidates[0].Curve3D.StartPoint | rb) + (bcCandidates[0].Curve3D.EndPoint | lb) > (bcCandidates[0].Curve3D.StartPoint | lb) + (bcCandidates[0].Curve3D.EndPoint | rb))
            {
                bcCandidates[0].Reverse();
            }
            if ((tcCandidates[0].Curve3D.StartPoint | lt) + (tcCandidates[0].Curve3D.EndPoint | rt) > (tcCandidates[0].Curve3D.StartPoint | rt) + (tcCandidates[0].Curve3D.EndPoint | lt))
            {
                tcCandidates[0].Reverse();
            }
            Edge e1 = new Edge(null, tcCandidates[0].Curve3D, null, tcCandidates[0].Curve2D2, true); // this is primary face of the edge with swept circle face
            Edge e3 = new Edge(null, bcCandidates[0].Curve3D, null, bcCandidates[0].Curve2D2, true); // this is secondary face of the edge with swept circle face


            sweptFace = Face.MakeFace(sweptCircle, [e1, e2, e3, e4]);

            // when the leading edge goes from left to right, we look from top onto the primary face. the leading edge is the lower outline of the primary face. The e2 is on the right side.
            // Now we want to construct the right end face of the rounding wedge. All edges are forward on the swept circle, i.e. Vertex1 and Vertex2 are start and endpoints.
            // The swept circle points to the outward.
            ICurve? topRight = null, topLeft = null, bottomRight = null, bottomLeft = null;
            IDualSurfaceCurve[] dsctr = rightPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.PrimaryFace.Surface, edgeToRound.PrimaryFace.Domain, [leadingEdge.EndPoint, e2.Vertex1.Position], null);
            topRight = dsctr.MinBy(dsc => dsc.Curve3D.DistanceTo(leadingEdge.EndPoint) + dsc.Curve3D.DistanceTo(e2.Vertex1.Position))?.Curve3D; // when there are more, , take the one closest to the endpoints
            topRight?.Trim(topRight.PositionOf(leadingEdge.EndPoint), topRight.PositionOf(e2.Vertex1.Position));
            IDualSurfaceCurve[] dsctl = leftPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.PrimaryFace.Surface, edgeToRound.PrimaryFace.Domain, [e4.Vertex2.Position, leadingEdge.StartPoint], null);
            topLeft = dsctl.MinBy(dsc => dsc.Curve3D.DistanceTo(e4.Vertex2.Position) + dsc.Curve3D.DistanceTo(leadingEdge.StartPoint))?.Curve3D; // when there are more, take the one closest to the endpoints
            topLeft?.Trim(topLeft.PositionOf(e4.Vertex2.Position), topLeft.PositionOf(leadingEdge.StartPoint));
            Face topFace = Face.Construct();
            topSurface = edgeToRound.PrimaryFace.Surface.Clone();
            topSurface.SetBounds(topDomain);
            Edge etr = new Edge(topFace, topRight, topFace, topSurface.GetProjectedCurve(topRight, 0.0), true);
            e1.SetFace(topFace, topSurface.GetProjectedCurve(e1.Curve3D, 0.0), false); // the tangential edge on the primary face, but reverse (SetFace with forward==false reverses the 2d curve)
            Edge etl = new Edge(topFace, topLeft, topFace, topSurface.GetProjectedCurve(topLeft, 0.0), true);
            Edge lde = new Edge(topFace, leadingEdge, topFace, topSurface.GetProjectedCurve(leadingEdge, 0.0), true); // leading edge, which is identical to the edgeToRound
            topFace.Set(topSurface, [[etr, e1, etl, lde]]);

            IDualSurfaceCurve[] dscbr = rightPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.SecondaryFace.Surface, edgeToRound.SecondaryFace.Domain, [e2.Vertex2.Position, leadingEdge.EndPoint], null);
            bottomRight = dscbr.MinBy(dsc => dsc.Curve3D.DistanceTo(e2.Vertex2.Position) + dsc.Curve3D.DistanceTo(leadingEdge.EndPoint))?.Curve3D; // when there are more, take the shortest
            bottomRight?.Trim(bottomRight.PositionOf(e2.Vertex2.Position), bottomRight.PositionOf(leadingEdge.EndPoint));
            IDualSurfaceCurve[] dscbl = leftPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.SecondaryFace.Surface, edgeToRound.SecondaryFace.Domain, [leadingEdge.StartPoint, e4.Vertex1.Position], null);
            bottomLeft = dscbl.MinBy(dsc => dsc.Curve3D.DistanceTo(leadingEdge.StartPoint) + dsc.Curve3D.DistanceTo(e4.Vertex1.Position))?.Curve3D; // when there are more, take the shortest
            bottomLeft?.Trim(bottomLeft.PositionOf(leadingEdge.StartPoint), bottomLeft.PositionOf(e4.Vertex1.Position));

            Face bottomFace = Face.Construct();
            bottomSurface = edgeToRound.SecondaryFace.Surface.Clone();
            bottomSurface.SetBounds(bottomDomain);
            Edge ebr = new Edge(bottomFace, bottomRight, bottomFace, bottomSurface.GetProjectedCurve(bottomRight, 0.0), true);
            e3.SetFace(bottomFace, bottomSurface.GetProjectedCurve(e3.Curve3D, 0.0), false); // the tangential edge on the secondary face, but reverse (SetFace with forward==false reverses the 2d curve)
            Edge ebl = new Edge(bottomFace, bottomLeft, bottomFace, bottomSurface.GetProjectedCurve(bottomLeft, 0.0), true);
            lde.SetFace(bottomFace, bottomSurface.GetProjectedCurve(leadingEdge, 0.0), false); // leading edge, which is identical to the edgeToRound, now in reverse direction
            bottomFace.Set(bottomSurface, [[lde, ebl, e3, ebr]]);

            Face rightEndFace = Face.Construct();
            e2.SetFace(rightEndFace, rightPlane.GetProjectedCurve(e2.Curve3D, 0.0), false); // the arc of the right hand side
            etr.SetFace(rightEndFace, rightPlane.GetProjectedCurve(etr.Curve3D, 0.0), false); // the top bound
            ebr.SetFace(rightEndFace, rightPlane.GetProjectedCurve(ebr.Curve3D, 0.0), false);
            rightEndFace.Set(rightPlane, [[e2, etr, ebr]]);
            Face leftEndFace = Face.Construct();
            e4.SetFace(leftEndFace, leftPlane.GetProjectedCurve(e4.Curve3D, 0.0), false);
            ebl.SetFace(leftEndFace, leftPlane.GetProjectedCurve(ebl.Curve3D, 0.0), false);
            etl.SetFace(leftEndFace, leftPlane.GetProjectedCurve(etl.Curve3D, 0.0), false);
            leftEndFace.Set(leftPlane, [[e4, ebl, etl]]);
            Shell res = Shell.FromFaces(sweptFace, topFace, bottomFace, rightEndFace, leftEndFace);
            res.RecalcVertices();
            rightEndFace.UserData.Add("CADability.Cutter.EndFace", "endface"); // categorize faces
            leftEndFace.UserData.Add("CADability.Cutter.EndFace", "endface");
            sweptFace.UserData.Add("CADability.Cutter.SweptFace", "sweptface");
#if DEBUG
            bool ok = res.CheckConsistency();
#endif
            return res;
        }

    }
}
