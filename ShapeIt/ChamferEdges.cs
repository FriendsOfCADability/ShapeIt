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
        public ChamferEdges(Shell shell, IEnumerable<Edge> edges, double length1, double length2)
        {
            this.shell = shell;
            this.length1 = Math.Abs(length1); // distances may not be negative
            this.length2 = Math.Abs(length2); // distances may not be negative
            // convex and concave: an edge is convex when the angle between the two faces is less than 108°, concave when greater than 180°
            convexEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Convex);
            concaveEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Concave);
        }
        public static Shell? MakeConvexChamferShell(Edge edgeToChamfer, double length1, double length2)
        {
            ISurface topSurface = edgeToChamfer.PrimaryFace.Surface; // for previty
            ISurface bottomSurface = edgeToChamfer.SecondaryFace.Surface;
            ICurve leadingEdge = edgeToChamfer.Curve3D.Clone();
            if (!edgeToChamfer.Forward(edgeToChamfer.PrimaryFace)) leadingEdge.Reverse(); // always forward on topSurface
            GeoPoint tstPoint = leadingEdge.PointAt(0.5);
            GeoVector n1 = topSurface.GetNormal(topSurface.PositionOf(leadingEdge.StartPoint));
            GeoVector n2 = bottomSurface.GetNormal(bottomSurface.PositionOf(leadingEdge.StartPoint));
            ISurface primSweptCircle = SweptCircle.MakePipeSurface(leadingEdge, length1, n1 + n2);
            Ellipse tstCircle = Ellipse.Construct();
            tstCircle.SetCirclePlaneCenterRadius(new Plane(leadingEdge.StartPoint, leadingEdge.StartDirection), leadingEdge.StartPoint, length1);
            topSurface.Intersect(tstCircle, edgeToChamfer.PrimaryFace.Domain, out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve);


            /*
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
            rightEndFace.UserData.Add("CADability.Fillet.EndFace", "endface"); // categorize faces
            leftEndFace.UserData.Add("CADability.Fillet.EndFace", "endface");
            sweptFace.UserData.Add("CADability.Fillet.SweptFace", "sweptface");
#if DEBUG
            bool ok = res.CheckConsistency();
#endif
            return res;
            */
            return null;
        }

    }
}
