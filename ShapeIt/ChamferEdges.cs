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
    internal class ChamferEdges
    {
        Shell shell;
        IEnumerable<Edge> convexEdges;
        IEnumerable<Edge> concaveEdges;
        double length1, length2;
        Dictionary<Edge, Shell>? edgeToChamfer;
        public ChamferEdges(Shell shell, IEnumerable<Edge> edges, double length1, double length2)
        {
            this.shell = shell;
            this.length1 = Math.Abs(length1); // distances may not be negative
            this.length2 = Math.Abs(length2); // distances may not be negative
            // convex and concave: an edge is convex when the angle between the two faces is less than 108°, concave when greater than 180°
            convexEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Convex);
            concaveEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Concave);
        }

        public Shell? Execute()
        {
            // 1. make a raw fillet for each edge. The fillet is a swept circle around a spine curve. The spine curve is the intersection
            // of two offset surfaces of the adjacent faces. The fillet starts and ends with a circular arc and has two tangential edges to the adjacent faces.
            // Later we have to trim or extent the fillet faces to get a proper result. The end vertices of the edge lie in the planes of the front arcs.
            edgeToChamfer = createChamfers(convexEdges);
            // there are one or more edges meeting at a vertex.
            Dictionary<Vertex, List<Edge>> vertexToEdges = createVertexToEdges(convexEdges);

            //List<HashSet<Shell>> roundingShells = []; // each hashset contains the faces of one rounding shell: the fillet and maybe some patches
            //foreach (var ve in vertexToEdges)
            //{
            //    if (ve.Value.Count == 1)
            //    { // the fillet ends here, there are different cases:
            //        // There is one or more "impact" faces
            //        HashSet<Shell>? filletAndExtension = createDeadEndExtension(ve.Key, ve.Value[0]);
            //        if (filletAndExtension != null) roundingShells.Add(filletAndExtension);
            //    }
            //    else if (ve.Value.Count == 2)
            //    {
            //        HashSet<Shell>? filletAndExtension = createExtensionTwoEdges(ve.Key, ve.Value[0], ve.Value[1]);
            //        if (filletAndExtension != null) roundingShells.Add(filletAndExtension);
            //    }
            //}
            //Combine(roundingShells);

            //// here we have the convex rounded edges as shells, which have to be subtracted from the shell on which the edges are to be rounded
            //Shell? toOperateOn = shell.Clone() as Shell;
            //for (int i = 0; i < roundingShells.Count; i++)
            //{
            //    foreach (var item in roundingShells[i])
            //    {
            //        BooleanOperation bo = new BooleanOperation();
            //        bo.SetShells(toOperateOn, item, BooleanOperation.Operation.difference);

            //        Shell[] roundedShells = bo.Execute();
            //        if (roundedShells != null && roundedShells.Length == 1) toOperateOn = roundedShells[0];
            //    }
            //}

            //return toOperateOn;
            return null;
        }
        private Dictionary<Edge, Shell> createChamfers(IEnumerable<Edge> edges)
        {
            Dictionary<Edge, Shell> edgeToChamfer = new();
            foreach (Edge edgeToRound in edges)
            {
                Shell? filletShell = MakeConvexChamferShell(edgeToRound, length1, length2);
                if (filletShell != null)
                {
                    edgeToChamfer[edgeToRound] = filletShell;
                }
            }
            return edgeToChamfer;
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
        private static void TrimCurve(ICurve curve, GeoPoint startPoint , GeoPoint endPoint)
        {
            double pos1 = curve.PositionOf(startPoint);
            double pos2 = curve.PositionOf(endPoint);
            if (Math.Abs(pos1) > 1e-6 || Math.Abs(1 - pos2) > 1e-6)
            {
                curve.Trim(pos1, pos2);
            }

        }
        public static Shell? MakeConvexChamferShell(Edge edgeToChamfer, double length1, double length2)
        {
            ISurface topSurface = edgeToChamfer.PrimaryFace.Surface; // for previty
            ISurface bottomSurface = edgeToChamfer.SecondaryFace.Surface;
            ICurve leadingEdge = edgeToChamfer.Curve3D.Clone();
            if (!edgeToChamfer.Forward(edgeToChamfer.PrimaryFace)) leadingEdge.Reverse(); // always forward on topSurface
            GeoPoint tstPoint = leadingEdge.PointAt(0.5);
            GeoVector n1 = topSurface.GetNormal(topSurface.PositionOf(tstPoint));
            GeoVector n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(tstPoint));
            ISurface? sweptCircle = hullAroundEdge(leadingEdge, length1, n1 + n2); // a cylindrical hull around the edge
            ISurfaceOfExtrusion? sweptCircleExtrusion = sweptCircle as ISurfaceOfExtrusion;
            if (sweptCircle == null || sweptCircleExtrusion == null) return null; // to satisfy the compiler

            Ellipse tstCircle = Ellipse.Construct();
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.StartPoint, leadingEdge.StartDirection), leadingEdge.StartPoint, length1);
            topSurface.Intersect(tstCircle, edgeToChamfer.PrimaryFace.Domain, out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve);
            if (ips.Length == 0) return null;
            GeoPoint sp = ips.MinBy(p => (p - leadingEdge.StartPoint) * (n1 + n2)); // the one to the inside
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection), leadingEdge.EndPoint, length1);
            topSurface.Intersect(tstCircle, edgeToChamfer.PrimaryFace.Domain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.EndPoint));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.EndPoint));
            GeoPoint ep = ips.MinBy(p => (p - leadingEdge.EndPoint) * (n1 + n2)); // the one to the inside

            IDualSurfaceCurve[] dscs = edgeToChamfer.PrimaryFace.Surface.GetDualSurfaceCurves(edgeToChamfer.PrimaryFace.Domain, sweptCircle, sweptCircle.GetBounds(), [sp, ep]);
            if (dscs == null) return null; // there should only be one
            ICurve? topCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(sp) + c.DistanceTo(sp));
            if (topCurve == null) return null;
            TrimCurve(topCurve, sp, ep);

            if (length2 != length1) sweptCircle = hullAroundEdge(leadingEdge, length2, n1 + n2);
            sweptCircleExtrusion = sweptCircle as ISurfaceOfExtrusion;
            if (sweptCircle == null || sweptCircleExtrusion == null) return null; // to satisfy the compiler
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.StartPoint, leadingEdge.StartDirection), leadingEdge.StartPoint, length2);
            bottomSurface.Intersect(tstCircle, edgeToChamfer.SecondaryFace.Domain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            sp = ips.MinBy(p => (p - leadingEdge.StartPoint) * (n1 + n2)); // the one to the inside
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection), leadingEdge.EndPoint, length1);
            bottomSurface.Intersect(tstCircle, edgeToChamfer.SecondaryFace.Domain, out ips, out uvOnFaces, out uOnCurve);
            if (ips.Length == 0) return null;
            n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.EndPoint));
            n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.EndPoint));
            ep = ips.MinBy(p => (p - leadingEdge.EndPoint) * (n1 + n2)); // the one to the inside

            dscs = edgeToChamfer.SecondaryFace.Surface.GetDualSurfaceCurves(edgeToChamfer.SecondaryFace.Domain, sweptCircle, sweptCircle.GetBounds(), [sp, ep]);
            if (dscs == null) return null; // there should only be one
            ICurve? bottomCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(sp) + c.DistanceTo(sp));
            if (bottomCurve == null) return null;
            TrimCurve(bottomCurve, sp, ep);

            ISurface chamferSurface = Make3D.MakeRuledSurface(topCurve, bottomCurve);
            foreach (GeoPoint p in new List<GeoPoint>([topCurve.StartPoint, topCurve.PointAt(0.5), topCurve.EndPoint,bottomCurve.StartPoint,bottomCurve.EndPoint]))
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
            dscs = topSurface.GetDualSurfaceCurves(edgeToChamfer.PrimaryFace.Domain, leftSurface, new BoundingRect(GeoPoint2D.Origin,d,d), [leadingEdge.StartPoint, topCurve.StartPoint]);
            ICurve? leftTopCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.StartPoint) + c.DistanceTo(topCurve.StartPoint));
            if (leftTopCurve == null) return null;
            TrimCurve(leftTopCurve, leadingEdge.StartPoint, topCurve.StartPoint);
            dscs = bottomSurface.GetDualSurfaceCurves(edgeToChamfer.SecondaryFace.Domain, leftSurface, new BoundingRect(GeoPoint2D.Origin, d, d), [leadingEdge.StartPoint, bottomCurve.StartPoint]);
            ICurve? leftBottomCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.StartPoint) + c.DistanceTo(bottomCurve.StartPoint));
            if (leftBottomCurve == null) return null;
            TrimCurve(leftBottomCurve, leadingEdge.StartPoint, bottomCurve.StartPoint);
            dscs = topSurface.GetDualSurfaceCurves(edgeToChamfer.PrimaryFace.Domain, rightSurface, new BoundingRect(GeoPoint2D.Origin, d, d), [leadingEdge.EndPoint, topCurve.EndPoint]);
            ICurve? rightTopCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.EndPoint) + c.DistanceTo(topCurve.EndPoint));
            if (rightTopCurve == null) return null;
            TrimCurve(rightTopCurve, leadingEdge.EndPoint, topCurve.EndPoint);
            dscs = bottomSurface.GetDualSurfaceCurves(edgeToChamfer.SecondaryFace.Domain, rightSurface, new BoundingRect(GeoPoint2D.Origin, d, d), [leadingEdge.EndPoint, bottomCurve.EndPoint]);
            ICurve? rightBottomCurve = dscs.Select(c => c.Curve3D).MinBy(c => c.DistanceTo(leadingEdge.EndPoint) + c.DistanceTo(bottomCurve.EndPoint));
            if (rightBottomCurve == null) return null;
            TrimCurve(rightBottomCurve, leadingEdge.EndPoint, bottomCurve.EndPoint);

            Face chamferFace = Face.MakeFace(chamferSurface, new List<ICurve>([topCurve,bottomCurve,leftChamferEdge,rightChamferEdge]));
            Face leftFace = Face.MakeFace(leftSurface, new List<ICurve>([leftTopCurve,leftBottomCurve,leftChamferEdge]));
            Face rightFace = Face.MakeFace(rightSurface, new List<ICurve>([rightTopCurve, rightBottomCurve, rightChamferEdge]));
            Face topFace = Face.MakeFace(topSurface.Clone(), new List<ICurve>([leadingEdge,leftTopCurve,rightTopCurve,topCurve]));
            Face bottomFace = Face.MakeFace(bottomSurface.Clone(), new List<ICurve>([leadingEdge,leftBottomCurve, rightBottomCurve, bottomCurve]));

            Shell res = Shell.FromFaces(chamferFace, topFace, bottomFace, rightFace, leftFace);
            rightFace.UserData.Add("CADability.Fillet.EndFace", "endface"); // categorize faces
            leftFace.UserData.Add("CADability.Fillet.EndFace", "endface");
            chamferFace.UserData.Add("CADability.Fillet.ChamferFace", "chamferface");

#if DEBUG
            bool ok = res.CheckConsistency();
#endif
            return res;
        }

    }
}
