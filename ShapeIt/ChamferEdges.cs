using CADability;
using CADability.Curve2D;
using CADability.GeoObject;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeIt
{
    internal class ChamferEdges : BlendEdges
    {
        double length1, length2;
        public ChamferEdges(Shell shell, IEnumerable<Edge> edges, double length1, double length2) : base(shell, edges)
        {
            this.length1 = Math.Abs(length1); // distances may not be negative
            this.length2 = Math.Abs(length2); // distances may not be negative
        }

        public Shell? Execute()
        {
            // 1. make a raw chamfer for each edge. The chamfer is a swept circle around a spine curve. The spine curve is the intersection
            // of two offset surfaces of the adjacent faces. The chamfer starts and ends with a circular arc and has two tangential edges to the adjacent faces.
            // Later we have to trim or extent the chamfer faces to get a proper result. The end vertices of the edge lie in the planes of the front arcs.
            edgeToCutter = createChamfers(convexEdges);
            // there are one or more edges meeting at a vertex.
            Dictionary<Vertex, List<Edge>> vertexToEdges = createVertexToEdges(convexEdges);

            List<HashSet<Shell>> roundingShells = []; // each hashset contains the faces of one rounding shell: the chamfer and maybe some patches
            foreach (var ve in vertexToEdges)
            {
                if (ve.Value.Count == 1)
                { // the chamfer ends here, there are different cases:
                    // There is one or more "impact" faces
                    HashSet<Shell>? chamferAndExtension = createDeadEndExtension(ve.Key, ve.Value[0], Math.Max(length1, length2));
                    if (chamferAndExtension != null) roundingShells.Add(chamferAndExtension);
                }
                else if (ve.Value.Count == 2)
                {
                    HashSet<Shell>? chamferAndExtension = createExtensionTwoEdges(ve.Key, ve.Value[0], ve.Value[1], Math.Max(length1, length2));
                    if (chamferAndExtension != null) roundingShells.Add(chamferAndExtension);
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
        private Dictionary<Edge, Shell> createChamfers(IEnumerable<Edge> edges)
        {
            Dictionary<Edge, Shell> edgeToCutter = new();
            foreach (Edge edgeToRound in edges)
            {
                Shell? chamferShell = MakeConvexChamferShell(edgeToRound, length1, length2);
                if (chamferShell != null)
                {
                    chamferShell.CopyAttributes(edgeToRound.PrimaryFace);
                    edgeToCutter[edgeToRound] = chamferShell;
                }
            }
            return edgeToCutter;
        }

        private static ISurface? hullAroundEdge(ICurve edge, double radius, GeoVector seam)
        {
            ISurface sweptCircle = SweptCircle.MakePipeSurface(edge, radius, seam); // a cylindrical hull around the edge
            ISurfaceOfExtrusion? sweptCircleExtrusion = sweptCircle as ISurfaceOfExtrusion;
            if (sweptCircleExtrusion == null) return null; // to satisfy the compiler
            BoundingRect sweptCircleDomain = BoundingRect.EmptyBoundingRect;
            sweptCircleDomain.MinMax(sweptCircle.PositionOf(edge.StartPoint));
            GeoPoint2D uv = sweptCircle.PositionOf(edge.PointAt(0.3));
            SurfaceHelper.AdjustPeriodic(sweptCircle, sweptCircleDomain, ref uv);
            sweptCircleDomain.MinMax(uv);
            sweptCircle.SetBounds(sweptCircleDomain); // now PositionOf is adjusted to the period
            sweptCircleDomain.MinMax(sweptCircle.PositionOf(edge.PointAt(0.7)));
            sweptCircle.SetBounds(sweptCircleDomain); // now PositionOf is adjusted to the period
            sweptCircleDomain.MinMax(sweptCircle.PositionOf(edge.EndPoint));
            sweptCircle.SetBounds(sweptCircleDomain); // now PositionOf is adjusted to the period
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
            sweptCircle.SetBounds(sweptCircleDomain);
            return sweptCircle;
        }
        public Shell? MakeConvexChamferShell(Edge edgeToCutter, double length1, double length2)
        {
            ISurface topSurface = edgeToCutter.PrimaryFace.Surface; // for previty
            ISurface bottomSurface = edgeToCutter.SecondaryFace.Surface;
            ICurve leadingEdge = edgeToCutter.Curve3D.Clone();
            if (!edgeToCutter.Forward(edgeToCutter.PrimaryFace)) leadingEdge.Reverse(); // always forward on topSurface
            GeoPoint tstPoint = leadingEdge.PointAt(0.5);
            GeoVector n1 = topSurface.GetNormal(topSurface.PositionOf(tstPoint));
            GeoVector n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(tstPoint));
            ISurface? sweptCircle = hullAroundEdge(leadingEdge, length1, n1 + n2); // a cylindrical hull around the edge
            ISurfaceOfExtrusion? sweptCircleExtrusion = sweptCircle as ISurfaceOfExtrusion;
            if (sweptCircle == null || sweptCircleExtrusion == null) return null; // to satisfy the compiler
            foreach (GeoPoint p in new List<GeoPoint>([leadingEdge.PointAt(0.33), leadingEdge.PointAt(0.67), leadingEdge.StartPoint, leadingEdge.EndPoint]))
            {
                sweptCircle.ExtendBoundsTo(p);
            }
            BoundingRect swcbounds = sweptCircle.GetBounds();
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
            sweptCircle.SetBounds(swcbounds);

            Ellipse tstCircle = Ellipse.Construct();
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.StartPoint, leadingEdge.StartDirection), leadingEdge.StartPoint, length1);
            topSurface.Intersect(tstCircle, edgeToCutter.PrimaryFace.Domain, out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.StartPoint));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.StartPoint));
            GeoPoint sp = ips.MinBy(p => (p - leadingEdge.StartPoint) * (n1 + n2)); // the one to the inside
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection), leadingEdge.EndPoint, length1);
            topSurface.Intersect(tstCircle, edgeToCutter.PrimaryFace.Domain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.EndPoint));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.EndPoint));
            GeoPoint ep = ips.MinBy(p => (p - leadingEdge.EndPoint) * (n1 + n2)); // the one to the inside
            // we need a middle point to discriminate between the two halves of a circle
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.PointAt(0.5), leadingEdge.DirectionAt(0.5)), leadingEdge.PointAt(0.5), length1);
            topSurface.Intersect(tstCircle, edgeToCutter.PrimaryFace.Domain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.PointAt(0.5)));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.PointAt(0.5)));
            GeoPoint mp = ips.MinBy(p => (p - leadingEdge.PointAt(0.5)) * (n1 + n2)); // the one to the inside

            IDualSurfaceCurve[] dscs = edgeToCutter.PrimaryFace.Surface.GetDualSurfaceCurves(edgeToCutter.PrimaryFace.Domain, sweptCircle, sweptCircle.GetBounds(), [sp, ep]);
            if (dscs == null) return null; // there should only be one
            ICurve? topCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(sp) + c.DistanceTo(sp) + c.DistanceTo(mp));
            if (topCurve == null) return null;
            TrimCurve(topCurve, sp, ep);

            if (length2 != length1) sweptCircle = hullAroundEdge(leadingEdge, length2, n1 + n2);
            sweptCircleExtrusion = sweptCircle as ISurfaceOfExtrusion;
            if (sweptCircle == null || sweptCircleExtrusion == null) return null; // to satisfy the compiler
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.StartPoint, leadingEdge.StartDirection), leadingEdge.StartPoint, length2);
            bottomSurface.Intersect(tstCircle, edgeToCutter.SecondaryFace.Domain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            sp = ips.MinBy(p => (p - leadingEdge.StartPoint) * (n1 + n2)); // the one to the inside
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection), leadingEdge.EndPoint, length2);
            bottomSurface.Intersect(tstCircle, edgeToCutter.SecondaryFace.Domain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.EndPoint));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.EndPoint));
            ep = ips.MinBy(p => (p - leadingEdge.EndPoint) * (n1 + n2)); // the one to the inside
            // we need a middle point to discriminate between the two halves of a circle
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.PointAt(0.5), leadingEdge.DirectionAt(0.5)), leadingEdge.PointAt(0.5), length2);
            bottomSurface.Intersect(tstCircle, edgeToCutter.PrimaryFace.Domain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.PointAt(0.5)));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.PointAt(0.5)));
            mp = ips.MinBy(p => (p - leadingEdge.PointAt(0.5)) * (n1 + n2)); // the one to the inside

            dscs = edgeToCutter.SecondaryFace.Surface.GetDualSurfaceCurves(edgeToCutter.SecondaryFace.Domain, sweptCircle, sweptCircle.GetBounds(), [sp, ep]);
            if (dscs == null) return null; // there should only be one
            ICurve? bottomCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(sp) + c.DistanceTo(sp) + c.DistanceTo(mp));
            if (bottomCurve == null) return null;
            TrimCurve(bottomCurve, sp, ep);

            ISurface chamferSurface = Make3D.MakeRuledSurface(topCurve, bottomCurve);
            foreach (GeoPoint p in new List<GeoPoint>([topCurve.StartPoint, topCurve.PointAt(0.5), topCurve.EndPoint, bottomCurve.StartPoint, bottomCurve.EndPoint]))
            {
                chamferSurface.ExtendBoundsTo(p);
            }
            ICurve leftChamferEdge = Line.TwoPoints(topCurve.StartPoint, bottomCurve.StartPoint);
            ICurve rightChamferEdge = Line.TwoPoints(topCurve.EndPoint, bottomCurve.EndPoint);
            Plane leftPlane = new Plane(leadingEdge.StartPoint, -leadingEdge.StartDirection);
            Plane rightPlane = new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection);
            ISurface leftSurface = new PlaneSurface(leftPlane);
            ISurface rightSurface = new PlaneSurface(rightPlane);
            double d = Math.Max(length1, length2);
            dscs = topSurface.GetDualSurfaceCurves(edgeToCutter.PrimaryFace.Domain, leftSurface, new BoundingRect(GeoPoint2D.Origin, d, d), [leadingEdge.StartPoint, topCurve.StartPoint]);
            ICurve? leftTopCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.StartPoint) + c.DistanceTo(topCurve.StartPoint));
            if (leftTopCurve == null) return null;
            TrimCurve(leftTopCurve, leadingEdge.StartPoint, topCurve.StartPoint);
            dscs = bottomSurface.GetDualSurfaceCurves(edgeToCutter.SecondaryFace.Domain, leftSurface, new BoundingRect(GeoPoint2D.Origin, d, d), [leadingEdge.StartPoint, bottomCurve.StartPoint]);
            ICurve? leftBottomCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.StartPoint) + c.DistanceTo(bottomCurve.StartPoint));
            if (leftBottomCurve == null) return null;
            TrimCurve(leftBottomCurve, leadingEdge.StartPoint, bottomCurve.StartPoint);
            dscs = topSurface.GetDualSurfaceCurves(edgeToCutter.PrimaryFace.Domain, rightSurface, new BoundingRect(GeoPoint2D.Origin, d, d), [leadingEdge.EndPoint, topCurve.EndPoint]);
            ICurve? rightTopCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.EndPoint) + c.DistanceTo(topCurve.EndPoint));
            if (rightTopCurve == null) return null;
            TrimCurve(rightTopCurve, leadingEdge.EndPoint, topCurve.EndPoint);
            dscs = bottomSurface.GetDualSurfaceCurves(edgeToCutter.SecondaryFace.Domain, rightSurface, new BoundingRect(GeoPoint2D.Origin, d, d), [leadingEdge.EndPoint, bottomCurve.EndPoint]);
            ICurve? rightBottomCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.EndPoint) + c.DistanceTo(bottomCurve.EndPoint));
            if (rightBottomCurve == null) return null;
            TrimCurve(rightBottomCurve, leadingEdge.EndPoint, bottomCurve.EndPoint);

            Face chamferFace = Face.MakeFace(chamferSurface, new List<ICurve>([topCurve, bottomCurve, leftChamferEdge, rightChamferEdge]));
            Face leftFace = Face.MakeFace(leftSurface, new List<ICurve>([leftTopCurve, leftBottomCurve, leftChamferEdge]));
            Face rightFace = Face.MakeFace(rightSurface, new List<ICurve>([rightTopCurve, rightBottomCurve, rightChamferEdge]));
            Face topFace = Face.MakeFace(topSurface.Clone(), new List<ICurve>([leadingEdge, leftTopCurve, rightTopCurve, topCurve]));
            Face bottomFace = Face.MakeFace(bottomSurface.Clone(), new List<ICurve>([leadingEdge, leftBottomCurve, rightBottomCurve, bottomCurve]));
            Shell[] res = Make3D.SewFaces([chamferFace, leftFace, rightFace, topFace, bottomFace]);
            if (res==null||res.Length==0) return null;

            rightFace.UserData.Add("CADability.Cutter.EndFace", "endface"); // categorize faces
            leftFace.UserData.Add("CADability.Cutter.EndFace", "endface");
            chamferFace.UserData.Add("CADability.Cutter.SweptFace", "chamferface");

#if DEBUG
            bool ok = res[0].CheckConsistency();
#endif
            return res[0];
        }

    }
}
