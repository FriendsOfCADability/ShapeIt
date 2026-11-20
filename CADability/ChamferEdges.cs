using CADability;
using CADability.Curve2D;
using CADability.GeoObject;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using static CADability.GeoObject.ShellExtensions;

namespace CADability.GeoObject
{
    public class ChamferEdges : BlendEdges
    {
        double length1, length2;
        Face? primaryFace;
        /// <param name="primaryFace">
        /// The face <paramref name="length1"/> is measured on. Optional: without it the distances follow
        /// the topology of each edge - <paramref name="length1"/> lands on the edge's PrimaryFace - which
        /// is fine for a symmetric chamfer but may swap the two distances for an asymmetric one. Pass the
        /// face the caller means, and the distances are swapped per edge wherever that face happens to be
        /// the edge's secondary one.
        /// </param>
        public ChamferEdges(Shell shell, IEnumerable<Edge> edges, double length1, double length2, Face? primaryFace = null) : base(shell, edges)
        {
            this.length1 = Math.Abs(length1); // distances may not be negative
            this.length2 = Math.Abs(length2); // distances may not be negative
            this.primaryFace = primaryFace;
        }

        /// <summary>
        /// The two distances for this edge, ordered so that the first one belongs to the edge's
        /// PrimaryFace - which is what <see cref="MakeChamferShell"/> expects.
        /// </summary>
        private (double first, double second) lengthsFor(Edge edge)
        {
            if (primaryFace != null && edge.SecondaryFace == primaryFace && edge.PrimaryFace != primaryFace)
                return (length2, length1);
            return (length1, length2);
        }

        /// <summary>
        /// Chamfers the edges. Where convex and concave edges meet, this is done in stages, see <see cref="BlendEdges.PlanStages"/>.
        /// </summary>
        /// <returns>the chamfered shell or null, if the selection is invalid (see <see cref="BlendEdges.CheckSelection"/>) or chamfering failed</returns>
        public Shell? Execute()
        {
            return ExecuteStaged((s, edges) => new ChamferEdges(s, edges, length1, length2, FaceInShell(s, primaryFace)).ExecuteStage());
        }

        /// <summary>
        /// The face of <paramref name="inShell"/> on the same surface as <paramref name="face"/>, used to find the primary
        /// face again in the result of a previous stage.
        /// </summary>
        private static Face? FaceInShell(Shell inShell, Face? face)
        {
            if (face == null || inShell.Faces.Contains(face)) return face;
            return inShell.Faces.FirstOrDefault(f => f.Surface.SameGeometry(f.Domain, face.Surface, face.Domain, Precision.eps, out ModOp2D _));
        }

        /// <summary>
        /// Chamfers all edges in one operation. Convex and concave edges must not meet.
        /// </summary>
        private Shell? ExecuteStage()
        {
            // 1. make a raw chamfer for each edge. The chamfer is a swept circle around a spine curve. The spine curve is the intersection
            // of two offset surfaces of the adjacent faces. The chamfer starts and ends with a circular arc and has two tangential edges to the adjacent faces.
            // Later we have to trim or extent the chamfer faces to get a proper result. The end vertices of the edge lie in the planes of the front arcs.
            edgeToCutter = createChamfers();
#if DEBUG
            DebuggerContainer dccut = new DebuggerContainer();
            foreach (Shell s in edgeToCutter.Values)
            {
                dccut.Add(s);
            }
#endif
            // there are one or more edges meeting at a vertex.
            Dictionary<Vertex, List<Edge>> vertexToEdges = createVertexToEdges(convexEdges.Concat(concaveEdges));
            // at the dead ends the chamfers must reach beyond the vertex, so that trimming them with the ending face closes the gap
            Dictionary<Edge, Dictionary<Vertex, double>> deadEndExtensions = ExtendCuttersAtDeadEnds(createVertexToEdges(convexEdges).Concat(createVertexToEdges(concaveEdges)));
            // chamfers which continue each other tangentially are made as one cutter
            BuildChamferChains(FindChamferJunctions(vertexToEdges), deadEndExtensions);

            List<HashSet<Shell>> convexRoundingShells = []; // each hashset contains the faces of one rounding shell: the chamfer and maybe some patches
            List<HashSet<Shell>> concaveRoundingShells = []; // each hashset contains the faces of one rounding shell: the chamfer and maybe some patches
            foreach (var ve in vertexToEdges)
            {
                HashSet<Shell>? chamferAndExtension = null;
                List<Edge> convexEdges = [];
                List<Edge> concaveEdges = [];
                foreach (Edge edge in ve.Value)
                {
                    if (edge.Adjacency() == AdjacencyType.Convex) convexEdges.Add(edge);
                    else if (edge.Adjacency() == AdjacencyType.Concave) concaveEdges.Add(edge);
                }
                if (convexEdges.Count == 1)
                { // the chamfer ends here, there are different cases:
                    // There is one or more "impact" faces
                    chamferAndExtension = createDeadEndExtension(ve.Key, convexEdges[0], Math.Max(length1, length2));
                    if (chamferAndExtension != null) convexRoundingShells.Add(chamferAndExtension);
                }
                if (convexEdges.Count == 2 )
                {   // both convex
                    chamferAndExtension = createExtensionTwoEdges(ve.Key, convexEdges[0], convexEdges[1], Math.Max(length1, length2));
                    if (chamferAndExtension != null) convexRoundingShells.Add(chamferAndExtension);
                }
                if (concaveEdges.Count == 1)
                { // the chamfer ends here, there are different cases:
                    // There is one or more "impact" faces
                    chamferAndExtension = createDeadEndExtension(ve.Key, concaveEdges[0], Math.Max(length1, length2));
                    if (chamferAndExtension != null) concaveRoundingShells.Add(chamferAndExtension);
                }
                if (concaveEdges.Count == 2)
                {   // both concave 
                    chamferAndExtension = createExtensionTwoEdges(ve.Key, concaveEdges[0], concaveEdges[1], Math.Max(length1, length2));
                    if (chamferAndExtension != null) concaveRoundingShells.Add(chamferAndExtension);
                }
                if (chamferAndExtension==null) // no other case
                {   // three or more edges or mixed concave/convex
                    foreach (Edge edge in ve.Value)
                    {
                        Shell? cutter = CutterOf(edge);
                        if (cutter == null) continue; // creating the cutter failed
                        if (edge.Adjacency() == AdjacencyType.Convex) convexRoundingShells.Add([cutter]);
                        else if (edge.Adjacency() == AdjacencyType.Concave) concaveRoundingShells.Add([cutter]);
                    }
                }
            }
            Combine(convexRoundingShells);
            Combine(concaveRoundingShells);

            // here we have the convex rounded edges as shells, which have to be subtracted from the shell on which the edges are to be rounded
            // here we have the convex rounded edges as shells, which have to be subtracted from the shell on which the edges are to be rounded
            Shell? toOperateOn = shell.Clone() as Shell;
            for (int i = 0; i < convexRoundingShells.Count; i++)
            {
                //foreach (var item in convexRoundingShells[i])
                List<Shell> dbgList = convexRoundingShells[i].ToList();
                    for (int j = 0;j< dbgList.Count;j++)
                {
                    BooleanOperation bo = new BooleanOperation();
                    bo.SetShells(toOperateOn, dbgList[j], BooleanOperation.Operation.difference);

                    Shell[] roundedShells = bo.Execute();
                    if (roundedShells != null && roundedShells.Length == 1) toOperateOn = roundedShells[0];
                }
            }
            for (int i = 0; i < concaveRoundingShells.Count; i++)
            {
                foreach (var item in concaveRoundingShells[i])
                {
                    BooleanOperation bo = new BooleanOperation();
                    bo.SetShells(toOperateOn, item, BooleanOperation.Operation.union);

                    Shell[] roundedShells = bo.Execute();
                    if (roundedShells != null && roundedShells.Length == 1) toOperateOn = roundedShells[0];
                }
            }

            return toOperateOn;
        }
        private Dictionary<Edge, Shell> createChamfers()
        {
            Dictionary<Edge, Shell> edgeToCutter = new();
            var edgesToRound = convexEdges.Select(e => (edge: e, isConvex: true)).Concat(concaveEdges.Select(e => (edge: e, isConvex: false)));

            foreach (var (edgeToRound, isConvex) in edgesToRound)
            {
                (double first, double second) = lengthsFor(edgeToRound);
                Shell? chamferShell = MakeChamferShell(edgeToRound, first, second, isConvex);
                if (chamferShell != null)
                {
                    chamferShell.CopyAttributes(edgeToRound.PrimaryFace);
                    edgeToCutter[edgeToRound] = chamferShell;
                }
            }
            return edgeToCutter;
        }

        /// <summary>
        /// The relative precision of the boundary curves of the chamfer surface, see <see cref="PreciseCurve"/>.
        /// </summary>
        private const double boundaryPrecision = 1e-9;

        /// <summary>
        /// A precise replacement of <paramref name="curve"/>, when it is an <see cref="InterpolatedDualSurfaceCurve"/>, to
        /// build the chamfer surface with. Such a curve is exact only at its base points, in between it is off its surfaces
        /// by up to 1e-6 of its size. The ruled chamfer surface between two such curves would inherit this error: its
        /// boundary would not lie in the faces of the edge, and the edges, which are later recalculated as the intersection
        /// of the chamfer surface with these faces, would run beside the boundary of the chamfer surface (1e-4 at the dead
        /// ends of tiltedCylinders, where the plane of a cap then cuts the edge outside the chamfer surface).
        /// Only for the surface: as the curves of the edges, the BSplines make the unions fail (teeSaddles).
        /// </summary>
        private static ICurve PreciseCurve(ICurve curve)
        {
            if (curve is InterpolatedDualSurfaceCurve idsc)
            {
                BSpline? precise = idsc.PreciseApproximation(boundaryPrecision);
                if (precise != null) return precise;
            }
            return curve;
        }

        private static ISurface? hullAroundEdge(ICurve edge, double radius, GeoVector seam)
        {
            ISurface sweptCircle = SweptCircleSurface.MakePipeSurface(edge, radius, seam); // a cylindrical hull around the edge
            ISurfaceOfExtrusion? sweptCircleExtrusion = sweptCircle as ISurfaceOfExtrusion;
            if (sweptCircleExtrusion == null) return null; // to satisfy the compiler
            BoundingRect sweptCircleDomain = BoundingRect.EmptyBoundingRect;
            sweptCircleDomain.MinMax(sweptCircle.PositionOf(edge.StartPoint));
            GeoPoint2D uv = sweptCircle.PositionOf(edge.PointAt(0.3));
            SurfaceHelper.AdjustPeriodic(sweptCircle, sweptCircleDomain, ref uv);
            sweptCircleDomain.MinMax(uv);
            sweptCircle.Domain = sweptCircleDomain; // now PositionOf is adjusted to the period
            sweptCircleDomain.MinMax(sweptCircle.PositionOf(edge.PointAt(0.7)));
            sweptCircle.Domain = sweptCircleDomain; // now PositionOf is adjusted to the period
            sweptCircleDomain.MinMax(sweptCircle.PositionOf(edge.EndPoint));
            sweptCircle.Domain = sweptCircleDomain; // now PositionOf is adjusted to the period
            if (sweptCircleExtrusion.ExtrusionDirectionIsV)
            {
                sweptCircleDomain.Left -= Math.PI / 2;
                sweptCircleDomain.Right += Math.PI / 2;
            }
            else
            {
                sweptCircleDomain.Bottom -= Math.PI / 2;
                sweptCircleDomain.Top += Math.PI / 2;
            }
            sweptCircle.Domain = sweptCircleDomain;
            return sweptCircle;
        }
        public Shell? MakeChamferShell(Edge edgeToCutter, double length1, double length2, bool convex)
        {
            return MakeExtendedChamferShell(edgeToCutter, length1, length2, convex, 0.0, 0.0);
        }

        /// <summary>
        /// Rebuilds the chamfer of <paramref name="edge"/> along a longer edge, which reaches the provided distances beyond
        /// the vertices of the edge, see <see cref="BlendEdges.ExtendCuttersAtDeadEnds"/>.
        /// </summary>
        protected override Shell? MakeExtendedCutter(Edge edge, Dictionary<Vertex, double> extensions)
        {
            // MakeExtendedChamferShell works on the edge oriented forward on the primary face
            Vertex start = edge.StartVertex(edge.PrimaryFace);
            Vertex end = edge.EndVertex(edge.PrimaryFace);
            extensions.TryGetValue(start, out double extendStart);
            extensions.TryGetValue(end, out double extendEnd);
            bool convex = convexEdges.Contains(edge);
            (double first, double second) = lengthsFor(edge);
            Shell? res = MakeExtendedChamferShell(edge, first, second, convex, extendStart, extendEnd);
            res?.CopyAttributes(edge.PrimaryFace);
            return res;
        }

        /// <summary>
        /// Like <see cref="MakeChamferShell"/>, but the chamfer is made along the edge extended by <paramref name="extendStart"/>
        /// and <paramref name="extendEnd"/> beyond its start and end point (with respect to the primary face).
        /// </summary>
        private Shell? MakeExtendedChamferShell(Edge edgeToCutter, double length1, double length2, bool convex, double extendStart, double extendEnd)
        {
            ChamferPiece? piece = MakeChamferPiece(edgeToCutter, length1, length2, convex, extendStart, extendEnd, null);
            if (piece == null) return null;
            Shell[] res = Make3D.SewFaces(piece.Faces.ToArray());
            if (res == null || res.Length == 0) return null;
            piece.Tag(extendStart > 0.0, extendEnd > 0.0);
#if DEBUG
            bool ok = res[0].CheckConsistency();
#endif
            return res[0];
        }

        /// <summary>
        /// How a chamfer piece ends at a vertex of a chain (see <see cref="BuildChamferChains"/>): there is no end face, the
        /// faces of the piece continue in the neighbouring piece or in a transition piece.
        /// </summary>
        private class ChamferEnd
        {
            /// <summary>the leading edge ends here instead of at the vertex (the foot point of the transition piece)</summary>
            public GeoPoint? TrimAt;
            /// <summary>the tangent edge, which is the end curve on <see cref="SideFace"/> (a perpendicular junction)</summary>
            public ICurve? TangentEdge;
            public Face? SideFace;
        }

        /// <summary>
        /// The faces of the chamfer of one edge, not yet sewn together, and the curves at its ends, which a transition piece
        /// shares with it.
        /// </summary>
        private class ChamferPiece
        {
            public Edge Edge = null!;
            public List<Face> Faces = [];
            public Face ChamferFace = null!;
            public Face? LeftEndFace, RightEndFace;
            public ICurve LeadingEdge = null!; // forward on the primary face
            // the curves in the end planes: on the primary face, on the secondary face, and the end line of the chamfer face
            public ICurve LeftTop = null!, LeftBottom = null!, LeftLine = null!, RightTop = null!, RightBottom = null!, RightLine = null!;

            public void Tag(bool extendedStart, bool extendedEnd)
            {
                RightEndFace?.UserData.Add("CADability.Cutter.EndFace", "endface"); // categorize faces
                LeftEndFace?.UserData.Add("CADability.Cutter.EndFace", "endface");
                if (extendedStart) LeftEndFace?.UserData.Add("CADability.Cutter.Extended", "extended"); // reaches beyond the edge
                if (extendedEnd) RightEndFace?.UserData.Add("CADability.Cutter.Extended", "extended");
                ChamferFace.UserData.Add("CADability.Cutter.SweptFace", "chamferface");
            }

            /// <summary>
            /// The end curves at <paramref name="vtx"/> (a vertex of the edge, or the trimmed end next to it): the one on
            /// <paramref name="face"/> and the end line of the chamfer face.
            /// </summary>
            public (ICurve onFace, ICurve line) EndCurves(Vertex vtx, Face face)
            {
                bool left = Edge.StartVertex(Edge.PrimaryFace) == vtx;
                ICurve onFace = face == Edge.PrimaryFace ? (left ? LeftTop : RightTop) : (left ? LeftBottom : RightBottom);
                return (onFace, left ? LeftLine : RightLine);
            }

            /// <summary>The end point of the leading edge at <paramref name="vtx"/>.</summary>
            public GeoPoint LeadingEnd(Vertex vtx) => Edge.StartVertex(Edge.PrimaryFace) == vtx ? LeadingEdge.StartPoint : LeadingEdge.EndPoint;
        }

        /// <summary>
        /// The faces of the chamfer of <paramref name="edgeToCutter"/>, see <see cref="MakeExtendedChamferShell"/>. At the vertices
        /// in <paramref name="ends"/> there is no end face (see <see cref="ChamferEnd"/>).
        /// </summary>
        private ChamferPiece? MakeChamferPiece(Edge edgeToCutter, double length1, double length2, bool convex, double extendStart, double extendEnd,
            Dictionary<Vertex, ChamferEnd>? ends)
        {
            ChamferEnd? startEnd = null, endEnd = null;
            ends?.TryGetValue(edgeToCutter.StartVertex(edgeToCutter.PrimaryFace), out startEnd);
            ends?.TryGetValue(edgeToCutter.EndVertex(edgeToCutter.PrimaryFace), out endEnd);
            ISurface topSurface = edgeToCutter.PrimaryFace.Surface.Clone(); // for previty
            ISurface bottomSurface = edgeToCutter.SecondaryFace.Surface.Clone();
            ICurve leadingEdge = edgeToCutter.Curve3D.Clone();
            if (!edgeToCutter.Forward(edgeToCutter.PrimaryFace)) leadingEdge.Reverse(); // always forward on topSurface
            // the domains of the two faces, extended when the chamfer has to reach beyond the edge
            BoundingRect topFaceDomain = edgeToCutter.PrimaryFace.Domain;
            BoundingRect bottomFaceDomain = edgeToCutter.SecondaryFace.Domain;
            if (extendStart > 0.0 || extendEnd > 0.0)
            {   // the chamfer is made along a longer edge: the intersection of the two surfaces beyond the vertices
                ICurve? longer = ExtendIntersectionCurve(leadingEdge, topSurface, ref topFaceDomain, bottomSurface, ref bottomFaceDomain, extendStart, extendEnd);
                if (longer == null) return null;
                leadingEdge = longer;
            }
            // a transition piece takes the part of the edge next to the vertex
            if (startEnd?.TrimAt is GeoPoint trimStart) leadingEdge.Trim(leadingEdge.PositionOf(trimStart), 1.0);
            if (endEnd?.TrimAt is GeoPoint trimEnd) leadingEdge.Trim(0.0, leadingEdge.PositionOf(trimEnd));
            GeoPoint tstPoint = leadingEdge.PointAt(0.5);
            GeoVector n1 = topSurface.GetNormal(topSurface.PositionOf(tstPoint));
            GeoVector n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(tstPoint));
            ISurface? sweptCircle = hullAroundEdge(leadingEdge, length1, n1 + n2); // a cylindrical hull around the edge
            ISurfaceOfExtrusion? sweptCircleExtrusion = sweptCircle as ISurfaceOfExtrusion;
            if (sweptCircle == null || sweptCircleExtrusion == null) return null; // to satisfy the compiler
            foreach (GeoPoint p in new List<GeoPoint>([leadingEdge.PointAt(0.33), leadingEdge.PointAt(0.67), leadingEdge.StartPoint, leadingEdge.EndPoint]))
            {
                sweptCircle.ExtendDomainTo(p);
            }
            BoundingRect swcbounds = sweptCircle.Domain;
            if (sweptCircleExtrusion.ExtrusionDirectionIsV)
            {
                swcbounds.Left = 0.0;
                swcbounds.Right = 2 * Math.PI;
            }
            else
            {
                swcbounds.Bottom = 0.0;
                swcbounds.Top = 2 * Math.PI;
            }
            sweptCircle.Domain = swcbounds;

            int convexFactor = convex ? 1 : -1;
            Ellipse tstCircle = Ellipse.Construct();
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.StartPoint, leadingEdge.StartDirection), leadingEdge.StartPoint, length1);
            topSurface.Intersect(tstCircle, topFaceDomain, out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.StartPoint));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.StartPoint));
            GeoPoint sp = ips.MinBy(p => convexFactor*(p - leadingEdge.StartPoint) * (n1 + n2)); // the one to the inside (concave: outlide)
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection), leadingEdge.EndPoint, length1);
            topSurface.Intersect(tstCircle, topFaceDomain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.EndPoint));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.EndPoint));
            GeoPoint ep = ips.MinBy(p => convexFactor * (p - leadingEdge.EndPoint) * (n1 + n2)); // the one to the inside (concave: outlide)
            // we need a middle point to discriminate between the two halves of a circle
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.PointAt(0.5), leadingEdge.DirectionAt(0.5)), leadingEdge.PointAt(0.5), length1);
            topSurface.Intersect(tstCircle, topFaceDomain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.PointAt(0.5)));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.PointAt(0.5)));
            GeoPoint mp = ips.MinBy(p => convexFactor*(p - leadingEdge.PointAt(0.5)) * (n1 + n2)); // the one to the inside

            IDualSurfaceCurve[] dscs = edgeToCutter.PrimaryFace.Surface.GetDualSurfaceCurves(topFaceDomain, sweptCircle, sweptCircle.Domain, [sp, ep]);
            if (dscs == null) return null; // there should only be one
            ICurve? topCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(sp) + c.DistanceTo(ep) + c.DistanceTo(mp));
            if (topCurve == null) return null;
            TrimCurve(topCurve, sp, ep, mp); // mp decides which part of a closed curve is meant

            if (length2 != length1) sweptCircle = hullAroundEdge(leadingEdge, length2, n1 + n2);
            sweptCircleExtrusion = sweptCircle as ISurfaceOfExtrusion;
            if (sweptCircle == null || sweptCircleExtrusion == null) return null; // to satisfy the compiler
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.StartPoint, leadingEdge.StartDirection), leadingEdge.StartPoint, length2);
            bottomSurface.Intersect(tstCircle, bottomFaceDomain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.StartPoint));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.StartPoint));
            sp = ips.MinBy(p => convexFactor * (p - leadingEdge.StartPoint) * (n1 + n2)); // the one to the inside
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection), leadingEdge.EndPoint, length2);
            bottomSurface.Intersect(tstCircle, bottomFaceDomain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.EndPoint));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.EndPoint));
            ep = ips.MinBy(p => convexFactor * (p - leadingEdge.EndPoint) * (n1 + n2)); // the one to the inside
            // we need a middle point to discriminate between the two halves of a circle
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.PointAt(0.5), leadingEdge.DirectionAt(0.5)), leadingEdge.PointAt(0.5), length2);
            bottomSurface.Intersect(tstCircle, bottomFaceDomain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.PointAt(0.5)));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.PointAt(0.5)));
            mp = ips.MinBy(p => convexFactor * (p - leadingEdge.PointAt(0.5)) * (n1 + n2)); // the one to the inside

            dscs = edgeToCutter.SecondaryFace.Surface.GetDualSurfaceCurves(bottomFaceDomain, sweptCircle, sweptCircle.Domain, [sp, ep]);
            if (dscs == null) return null; // there should only be one
            ICurve? bottomCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(sp) + c.DistanceTo(ep) + c.DistanceTo(mp));
            if (bottomCurve == null) return null;
            TrimCurve(bottomCurve, sp, ep, mp);

            // the chamfer surface is ruled between these curves, so it is only as precise as they are. The edges keep the
            // intersection curves: they are recalculated as the intersections of the chamfer surface with the faces anyhow
            ISurface chamferSurface = Make3D.MakeRuledSurface(PreciseCurve(topCurve), PreciseCurve(bottomCurve));
            foreach (GeoPoint p in new List<GeoPoint>([topCurve.StartPoint, topCurve.PointAt(0.5), topCurve.EndPoint, bottomCurve.StartPoint, bottomCurve.EndPoint]))
            {
                chamferSurface.ExtendDomainTo(p);
            }
            ICurve leftChamferEdge = Line.TwoPoints(topCurve.StartPoint, bottomCurve.StartPoint);
            ICurve rightChamferEdge = Line.TwoPoints(topCurve.EndPoint, bottomCurve.EndPoint);
            Plane leftPlane = new Plane(leadingEdge.StartPoint, -leadingEdge.StartDirection);
            Plane rightPlane = new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection);
            ISurface leftSurface = new PlaneSurface(leftPlane);
            ISurface rightSurface = new PlaneSurface(rightPlane);
            double d = Math.Max(length1, length2);
            dscs = topSurface.GetDualSurfaceCurves(topFaceDomain, leftSurface, new BoundingRect(GeoPoint2D.Origin, d, d), [leadingEdge.StartPoint, topCurve.StartPoint]);
            ICurve? leftTopCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.StartPoint) + c.DistanceTo(topCurve.StartPoint));
            if (leftTopCurve == null) return null;
            TrimCurve(leftTopCurve, leadingEdge.StartPoint, topCurve.StartPoint);
            dscs = bottomSurface.GetDualSurfaceCurves(bottomFaceDomain, leftSurface, new BoundingRect(GeoPoint2D.Origin, d, d), [leadingEdge.StartPoint, bottomCurve.StartPoint]);
            ICurve? leftBottomCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.StartPoint) + c.DistanceTo(bottomCurve.StartPoint));
            if (leftBottomCurve == null) return null;
            TrimCurve(leftBottomCurve, leadingEdge.StartPoint, bottomCurve.StartPoint);
            dscs = topSurface.GetDualSurfaceCurves(topFaceDomain, rightSurface, new BoundingRect(GeoPoint2D.Origin, d, d), [leadingEdge.EndPoint, topCurve.EndPoint]);
            ICurve? rightTopCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.EndPoint) + c.DistanceTo(topCurve.EndPoint));
            if (rightTopCurve == null) return null;
            TrimCurve(rightTopCurve, leadingEdge.EndPoint, topCurve.EndPoint);
            dscs = bottomSurface.GetDualSurfaceCurves(bottomFaceDomain, rightSurface, new BoundingRect(GeoPoint2D.Origin, d, d), [leadingEdge.EndPoint, bottomCurve.EndPoint]);
            ICurve? rightBottomCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.EndPoint) + c.DistanceTo(bottomCurve.EndPoint));
            if (rightBottomCurve == null) return null;
            TrimCurve(rightBottomCurve, leadingEdge.EndPoint, bottomCurve.EndPoint);

            // at a perpendicular junction the end curve on the side face is the tangent edge, which both pieces share
            ICurve? tangentEnd(ChamferEnd? end, Face face, GeoPoint vertex, GeoPoint onFace)
            {
                if (end?.TangentEdge == null || end.SideFace != face) return null;
                ICurve segment = end.TangentEdge.Clone();
                TrimCurve(segment, vertex, onFace);
                return segment;
            }
            leftTopCurve = tangentEnd(startEnd, edgeToCutter.PrimaryFace, leadingEdge.StartPoint, topCurve.StartPoint) ?? leftTopCurve;
            leftBottomCurve = tangentEnd(startEnd, edgeToCutter.SecondaryFace, leadingEdge.StartPoint, bottomCurve.StartPoint) ?? leftBottomCurve;
            rightTopCurve = tangentEnd(endEnd, edgeToCutter.PrimaryFace, leadingEdge.EndPoint, topCurve.EndPoint) ?? rightTopCurve;
            rightBottomCurve = tangentEnd(endEnd, edgeToCutter.SecondaryFace, leadingEdge.EndPoint, bottomCurve.EndPoint) ?? rightBottomCurve;

            // the same order of creating and sewing the faces as always, so that the cutter of a single edge doesn't change
            Face chamferFace = Face.MakeFace(chamferSurface, new List<ICurve>([topCurve, bottomCurve, leftChamferEdge, rightChamferEdge]));
            Face? leftFace = startEnd == null ? Face.MakeFace(leftSurface, new List<ICurve>([leftTopCurve, leftBottomCurve, leftChamferEdge])) : null;
            Face? rightFace = endEnd == null ? Face.MakeFace(rightSurface, new List<ICurve>([rightTopCurve, rightBottomCurve, rightChamferEdge])) : null;
            Face topFace = Face.MakeFace(topSurface.Clone(), new List<ICurve>([leadingEdge, leftTopCurve, rightTopCurve, topCurve]));
            Face bottomFace = Face.MakeFace(bottomSurface.Clone(), new List<ICurve>([leadingEdge, leftBottomCurve, rightBottomCurve, bottomCurve]));
            ChamferPiece piece = new ChamferPiece
            {
                Edge = edgeToCutter, ChamferFace = chamferFace, LeadingEdge = leadingEdge, LeftEndFace = leftFace, RightEndFace = rightFace,
                LeftTop = leftTopCurve, LeftBottom = leftBottomCurve, LeftLine = leftChamferEdge,
                RightTop = rightTopCurve, RightBottom = rightBottomCurve, RightLine = rightChamferEdge
            };
            piece.Faces.AddRange(new Face?[] { chamferFace, leftFace, rightFace, topFace, bottomFace }.OfType<Face>());
            return piece;
        }

        /// <summary>
        /// A vertex where two chamfers continue each other tangentially (see <see cref="BlendEdges.TryTangentJunction"/>).
        /// The chamfers of both edges end in the plane through the vertex perpendicular to the edges, and there they have
        /// the same cross section, as long as the tangent edge between their side faces lies in this plane (perpendicular).
        /// When the tangent edge is oblique, it leans into the side face of one of the edges (<see cref="Edge2"/>): the
        /// boundary curve of the chamfer on the side faces crosses the tangent edge only beyond the plane, at the point which
        /// has the distance of the chamfer from the foot point <see cref="Foot"/> on <see cref="Edge2"/>. Between the plane
        /// through the vertex and the cross section through this foot point there is a transition piece
        /// (<see cref="MakeTransition"/>), whose part on the side faces is split by the tangent edge. All faces of the
        /// cutter then lie in the faces of the shell, and no planar end has to cross the tangent edge.
        /// At a seam of one surface the planar ends fit together in any case, there is no transition piece.
        /// </summary>
        private class ChamferJunction
        {
            public Vertex Vertex = null!;
            public Edge Edge1 = null!, Edge2 = null!; // the tangent edge leans into the side face of Edge2
            public Face Common = null!, Side1 = null!, Side2 = null!;
            public Edge TangentEdge = null!;
            public bool Oblique;
            public bool Seam; // both side faces are on the same surface
            public GeoPoint Foot; // on Edge2, where the transition piece ends (only when oblique)
            public double CommonLength, SideLength; // the distances of the chamfer on the common face and on the side faces
        }

        /// <summary>
        /// The tangential junctions of the edges in <paramref name="vertexToEdges"/>, see <see cref="ChamferJunction"/>.
        /// </summary>
        private List<ChamferJunction> FindChamferJunctions(IEnumerable<KeyValuePair<Vertex, List<Edge>>> vertexToEdges)
        {
            List<ChamferJunction> res = [];
            HashSet<Edge> shellEdges = new HashSet<Edge>(shell.Edges);
            foreach (KeyValuePair<Vertex, List<Edge>> ve in vertexToEdges)
            {
                if (ve.Value.Count != 2) continue;
                Vertex vtx = ve.Key;
                Edge e1 = ve.Value[0], e2 = ve.Value[1];
                if (convexEdges.Contains(e1) != convexEdges.Contains(e2)) continue;
                if (!TryTangentJunction(vtx, e1, e2, shellEdges, out Face common, out Face side1, out Face side2, out Edge tangentEdge)) continue;
                // both chamfers must have the same distances on the common face and on the side faces
                (double first1, double second1) = lengthsFor(e1);
                (double first2, double second2) = lengthsFor(e2);
                double common1 = e1.PrimaryFace == common ? first1 : second1, sideLength1 = e1.PrimaryFace == common ? second1 : first1;
                double common2 = e2.PrimaryFace == common ? first2 : second2, sideLength2 = e2.PrimaryFace == common ? second2 : first2;
                if (Math.Abs(common1 - common2) > Precision.eps || Math.Abs(sideLength1 - sideLength2) > Precision.eps) continue;
                ChamferJunction junction = new ChamferJunction
                {
                    Vertex = vtx, Edge1 = e1, Edge2 = e2, Common = common, Side1 = side1, Side2 = side2, TangentEdge = tangentEdge,
                    CommonLength = common1, SideLength = sideLength1
                };
                // into which side face does the tangent edge lean?
                double lean = DirectionFrom(tangentEdge, vtx) * DirectionFrom(e2, vtx);
                // a seam of one surface (e.g. two halves of a cylinder): the planar ends of the chamfers fit together, even when
                // the seam is oblique, the curves on both sides of the seam are the same
                junction.Seam = side1.Surface.SameGeometry(side1.Domain, side2.Surface, side2.Domain, Precision.eps, out _);
                if (lean < 0.0)
                {
                    (junction.Edge1, junction.Edge2) = (e2, e1);
                    (junction.Side1, junction.Side2) = (side2, side1);
                }
                if (!junction.Seam && Math.Abs(lean) > 1e-8)
                {   // the point on the tangent edge with the distance of the chamfer from Edge2
                    ICurve tangent = tangentEdge.Curve3D;
                    ICurve leanEdge = junction.Edge2.Curve3D;
                    GeoPoint foot(GeoPoint p) => leanEdge.PointAt(Math.Max(0.0, Math.Min(1.0, leanEdge.PositionOf(p))));
                    double distanceError(double t) => (tangent.PointAt(t) - foot(tangent.PointAt(t))).Length - junction.SideLength;
                    double tVertex = (tangent.StartPoint | vtx.Position) < (tangent.EndPoint | vtx.Position) ? 0.0 : 1.0;
                    double t0 = tVertex, f0 = distanceError(t0), tFound = double.NaN;
                    const int steps = 100;
                    for (int i = 1; i <= steps; i++)
                    {   // the first sign change, walking from the vertex along the tangent edge
                        double t1 = tVertex + (1.0 - 2.0 * tVertex) * i / steps, f1 = distanceError(t1);
                        if (Math.Sign(f1) != Math.Sign(f0))
                        {
                            for (int j = 0; j < 60; j++)
                            {
                                double tm = (t0 + t1) / 2.0, fm = distanceError(tm);
                                if (Math.Sign(fm) == Math.Sign(f0)) { t0 = tm; f0 = fm; }
                                else t1 = tm;
                            }
                            tFound = (t0 + t1) / 2.0;
                            break;
                        }
                        t0 = t1; f0 = f1;
                    }
                    if (double.IsNaN(tFound)) continue; // the tangent edge is too short
                    double footPosition = leanEdge.PositionOf(tangent.PointAt(tFound));
                    if (footPosition <= 0.0 || footPosition >= 1.0) continue; // the edge is too short for the transition piece
                    junction.Foot = leanEdge.PointAt(footPosition);
                    junction.Oblique = (junction.Foot | vtx.Position) > Precision.eps;
                }
                res.Add(junction);
            }
            return res;
        }

        /// <summary>
        /// Makes one cutter for each chain of edges connected by <paramref name="junctions"/>: the chamfers of the edges
        /// (<see cref="MakeChamferPiece"/>) without end faces at the junctions and the transition pieces at the oblique
        /// junctions (<see cref="MakeTransition"/>), sewn together. All edges of the chain refer to this cutter in
        /// <see cref="BlendEdges.edgeToCutter"/>. When something fails, the edges keep their own cutters.
        /// <paramref name="extensions"/> are the extensions at the dead ends (see <see cref="BlendEdges.ExtendCuttersAtDeadEnds"/>).
        /// </summary>
        private void BuildChamferChains(List<ChamferJunction> junctions, Dictionary<Edge, Dictionary<Vertex, double>> extensions)
        {
            if (edgeToCutter == null || junctions.Count == 0) return;
            foreach (List<Edge> chain in Chains(junctions.Select(j => (j.Edge1, j.Edge2))))
            {
                List<ChamferJunction> chainJunctions = junctions.Where(j => chain.Contains(j.Edge1)).ToList();
                Dictionary<Edge, Dictionary<Vertex, ChamferEnd>> ends = chain.ToDictionary(e => e, e => new Dictionary<Vertex, ChamferEnd>());
                foreach (ChamferJunction j in chainJunctions)
                {
                    if (j.Oblique)
                    {
                        ends[j.Edge1][j.Vertex] = new ChamferEnd();
                        ends[j.Edge2][j.Vertex] = new ChamferEnd { TrimAt = j.Foot };
                    }
                    else if (j.Seam)
                    {   // the end curves in the plane through the vertex are the same on both sides of the seam
                        ends[j.Edge1][j.Vertex] = new ChamferEnd();
                        ends[j.Edge2][j.Vertex] = new ChamferEnd();
                    }
                    else
                    {
                        ends[j.Edge1][j.Vertex] = new ChamferEnd { TangentEdge = j.TangentEdge.Curve3D, SideFace = j.Side1 };
                        ends[j.Edge2][j.Vertex] = new ChamferEnd { TangentEdge = j.TangentEdge.Curve3D, SideFace = j.Side2 };
                    }
                }
                Dictionary<Edge, ChamferPiece> pieces = [];
                foreach (Edge edge in chain)
                {
                    extensions.TryGetValue(edge, out Dictionary<Vertex, double>? ext);
                    double extendStart = 0.0, extendEnd = 0.0;
                    ext?.TryGetValue(edge.StartVertex(edge.PrimaryFace), out extendStart);
                    ext?.TryGetValue(edge.EndVertex(edge.PrimaryFace), out extendEnd);
                    (double first, double second) = lengthsFor(edge);
                    ChamferPiece? piece = MakeChamferPiece(edge, first, second, convexEdges.Contains(edge), extendStart, extendEnd, ends[edge]);
                    if (piece == null) break;
                    piece.Tag(extendStart > 0.0, extendEnd > 0.0);
                    pieces[edge] = piece;
                }
                if (pieces.Count != chain.Count) continue; // a chamfer could not be made, the edges keep their own cutters
                List<Face> faces = pieces.Values.SelectMany(p => p.Faces).ToList();
                bool ok = true;
                foreach (ChamferJunction j in chainJunctions.Where(j => j.Oblique))
                {
                    List<Face>? transition = MakeTransition(j, pieces[j.Edge1], pieces[j.Edge2]);
                    if (transition == null) { ok = false; break; }
                    faces.AddRange(transition);
                }
                if (!ok) continue;
                Shell[] sewn = Make3D.SewFaces(faces.ToArray());
                if (sewn == null || sewn.Length != 1 || sewn[0].HasOpenEdgesExceptPoles()) continue; // the pieces don't fit together
                Shell cutter = sewn[0];
                cutter.CopyAttributes(chain[0].PrimaryFace);
                foreach (Edge edge in chain) edgeToCutter[edge] = cutter;
            }
        }

        /// <summary>
        /// The faces of the transition piece of an oblique junction <paramref name="j"/>, see <see cref="ChamferJunction"/>,
        /// between the open end of <paramref name="piece1"/> at the vertex and the open end of <paramref name="piece2"/> at the
        /// foot point. Null, if it cannot be made.
        /// </summary>
        private List<Face>? MakeTransition(ChamferJunction j, ChamferPiece piece1, ChamferPiece piece2)
        {
            GeoPoint vertex = j.Vertex.Position;
            // the curves at the open ends of the two pieces, the transition piece shares them
            (ICurve commonCurve1, ICurve line1) = piece1.EndCurves(j.Vertex, j.Common);
            ICurve sideCurve1 = piece1.EndCurves(j.Vertex, j.Side1).onFace;
            (ICurve commonCurve2, ICurve line2) = piece2.EndCurves(j.Vertex, j.Common);
            ICurve sideCurve2 = piece2.EndCurves(j.Vertex, j.Side2).onFace;
            GeoPoint foot = piece2.LeadingEnd(j.Vertex);
            GeoPoint farEnd(ICurve c, GeoPoint from) => (c.StartPoint | from) > (c.EndPoint | from) ? c.StartPoint : c.EndPoint;
            GeoPoint onCommon1 = farEnd(commonCurve1, vertex), onSide1 = farEnd(sideCurve1, vertex);
            GeoPoint onCommon2 = farEnd(commonCurve2, foot), onTangent = farEnd(sideCurve2, foot);
            // the part of Edge2 and of the tangent edge, which belong to the transition piece
            ICurve edgePart = j.Edge2.Curve3D.Clone();
            TrimCurve(edgePart, vertex, foot);
            ICurve tangentPart = j.TangentEdge.Curve3D.Clone();
            TrimCurve(tangentPart, vertex, onTangent);
            // the boundary curves of the chamfer: the hulls around this part of Edge2 intersected with the common face and
            // the side face of Edge1 (the tangent edge leans into Edge2, so this part of the side faces is on Side1)
            GeoVector seam = NormalAt(j.Common, vertex) + NormalAt(j.Side1, vertex);
            ISurface? sideHull = hullAroundEdge(edgePart, j.SideLength, seam);
            ISurface? commonHull = j.CommonLength == j.SideLength ? sideHull : hullAroundEdge(edgePart, j.CommonLength, seam);
            if (sideHull == null || commonHull == null) return null;
            ICurve? boundary(Face face, ISurface hull, GeoPoint from, GeoPoint to)
            {
                IDualSurfaceCurve[] dscs = face.Surface.GetDualSurfaceCurves(face.Domain, hull, hull.Domain, [from, to]);
                ICurve? c = dscs?.Select(d => d.Curve3D).MinBy(cv => cv.DistanceTo(from) + cv.DistanceTo(to));
                if (c == null) return null;
                TrimCurve(c, from, to);
                return c;
            }
            ICurve? commonBoundary = boundary(j.Common, commonHull, onCommon1, onCommon2);
            ICurve? sideBoundary = boundary(j.Side1, sideHull, onSide1, onTangent);
            if (commonBoundary == null || sideBoundary == null) return null;
            ISurface chamferSurface = Make3D.MakeRuledSurface(PreciseCurve(commonBoundary), PreciseCurve(sideBoundary));
            foreach (GeoPoint p in new List<GeoPoint>([onCommon1, onCommon2, onSide1, onTangent, commonBoundary.PointAt(0.5)])) chamferSurface.ExtendDomainTo(p);
            Face chamferFace = Face.MakeFace(chamferSurface, new List<ICurve>([commonBoundary, sideBoundary, line1.Clone(), line2.Clone()]));
            Face commonFace = Face.MakeFace(j.Common.Surface.Clone(), new List<ICurve>([edgePart, commonCurve1.Clone(), commonBoundary, commonCurve2.Clone()]));
            Face side1Face = Face.MakeFace(j.Side1.Surface.Clone(), new List<ICurve>([sideCurve1.Clone(), sideBoundary, tangentPart]));
            Face side2Face = Face.MakeFace(j.Side2.Surface.Clone(), new List<ICurve>([edgePart, tangentPart, sideCurve2.Clone()]));
            chamferFace.UserData.Add("CADability.Cutter.SweptFace", "chamferface");
            return [chamferFace, commonFace, side1Face, side2Face];
        }

    }
}
