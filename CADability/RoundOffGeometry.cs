using CADability.Curve2D;
using CADability.GeoObject;
using System;
using System.Collections;

namespace CADability
{
    /// <summary>
    /// Pure geometric core of the "round off corner" tool (see <see cref="CADability.Actions.RoundObjectsAction"/>), free of
    /// any interaction state so it can be unit tested. Given two curves that form a corner, a pick position and a radius
    /// it computes the tangential fillet arc.
    /// </summary>
    public static class RoundOffGeometry
    {
        /// <summary>
        /// Returns the intersection of the two 2D curves closest to <paramref name="pick2D"/> (the corner near the pick),
        /// or (0,0) if the curves do not intersect.
        /// </summary>
        internal static GeoPoint2D FindNearestIntersection(ICurve2D curve1_2D, ICurve2D curve2_2D, GeoPoint2D pick2D)
        {
            GeoPoint2DWithParameter[] intersectPoints = curve1_2D.Intersect(curve2_2D);
            GeoPoint2D corner = new GeoPoint2D(0.0, 0.0);
            double distS = double.MaxValue;
            for (int i = 0; i < intersectPoints.Length; ++i)
            {
                double distLoc = Geometry.Dist(intersectPoints[i].p, pick2D);
                if (distLoc < distS)
                {
                    distS = distLoc;
                    corner = intersectPoints[i].p;
                }
            }
            return corner;
        }

        /// <summary>
        /// Computes the tangential fillet arc of the given <paramref name="radius"/> in the corner where
        /// <paramref name="curve1"/> and <paramref name="curve2"/> meet, near <paramref name="pick"/>.
        /// The arc center is searched among the intersections of the ±radius parallels of both curves; a valid center has
        /// a perpendicular foot (within the curves' parameter range) on each curve at distance <paramref name="radius"/>.
        /// When <paramref name="skipQuadrantFilter"/> is false, only centers on the same side as the pick (relative to
        /// both curve tangents at the corner) are accepted, so the fillet ends up in the picked quadrant. When the two
        /// curves and their shared corner are already known, pass true: the perpendicular-foot constraints then already
        /// isolate the inner-angle fillet, so the (pick- and view-dependent) quadrant filter is unnecessary and would
        /// only reject the correct center when the pick happens to fall outside the corner's wedge.
        /// </summary>
        /// <param name="curve1">first curve of the corner</param>
        /// <param name="curve2">second curve of the corner</param>
        /// <param name="pick">the pick point in world coordinates, near the corner</param>
        /// <param name="radius">the fillet radius, must be &gt; 0</param>
        /// <param name="skipQuadrantFilter">true to skip the pick-based quadrant filter (curves and corner are known)</param>
        /// <param name="drawingPlane">the active drawing plane, used to orient the resulting arc</param>
        /// <param name="arc">the resulting fillet arc (the smaller of the two possible arcs), or null</param>
        /// <param name="cornerPoint">the corner in world coordinates, or the origin</param>
        /// <returns>true if a fillet arc was found</returns>
        public static bool TryComputeRoundOff(ICurve curve1, ICurve curve2, GeoPoint pick, double radius,
            bool skipQuadrantFilter, Plane drawingPlane, out Ellipse arc, out GeoPoint cornerPoint)
        {
            arc = null;
            cornerPoint = GeoPoint.Origin;
            if (curve1 == null || curve2 == null || curve1 == curve2) return false;
            if (!Curves.GetCommonPlane(curve1, curve2, out Plane pl)) return false;

            ICurve2D curve1_2D = curve1.GetProjectedCurve(pl);
            ICurve2D curve2_2D = curve2.GetProjectedCurve(pl);
            if (curve1_2D is Path2D p1) p1.Flatten();
            if (curve2_2D is Path2D p2) p2.Flatten();

            // the corner is the intersection of the two curves closest to the pick
            GeoPoint2D pick2D = pl.Project(pick);
            GeoPoint2D objectPoint2D = FindNearestIntersection(curve1_2D, curve2_2D, pick2D);
            GeoVector2D v1CutPoint = curve1_2D.DirectionAt(curve1_2D.PositionOf(objectPoint2D));
            GeoVector2D v2CutPoint = curve2_2D.DirectionAt(curve2_2D.PositionOf(objectPoint2D));

            // parameter bounds, extended when the corner is a virtual intersection beyond a curve's end
            double locmin1 = 0.0, locmin2 = 0.0, locmax1 = 1.0, locmax2 = 1.0;
            double locPar = curve1_2D.PositionOf(objectPoint2D);
            if (locPar > 1.0) locmax1 = locPar;
            if (locPar < 0.0) locmin1 = locPar;
            locPar = curve2_2D.PositionOf(objectPoint2D);
            if (locPar > 1.0) locmax2 = locPar;
            if (locPar < 0.0) locmin2 = locPar;

            // the ±radius parallels of both curves; their mutual intersections are the fillet center candidates
            ICurve2D P1L1 = curve1_2D.Parallel(radius, false, 0.0, 0.0);
            ICurve2D P1L2 = curve1_2D.Parallel(-radius, false, 0.0, 0.0);
            ICurve2D P2L1 = curve2_2D.Parallel(radius, false, 0.0, 0.0);
            ICurve2D P2L2 = curve2_2D.Parallel(-radius, false, 0.0, 0.0);
            ArrayList centers = new ArrayList();
            if (P1L1 != null && P2L1 != null) centers.AddRange(P1L1.Intersect(P2L1));
            if (P1L1 != null && P2L2 != null) centers.AddRange(P1L1.Intersect(P2L2));
            if (P1L2 != null && P2L1 != null) centers.AddRange(P1L2.Intersect(P2L1));
            if (P1L2 != null && P2L2 != null) centers.AddRange(P1L2.Intersect(P2L2));
            GeoPoint2DWithParameter[] centerPoints = (GeoPoint2DWithParameter[])centers.ToArray(typeof(GeoPoint2DWithParameter));

            bool rndPos = false;
            GeoPoint2D arcP1 = new GeoPoint2D(0.0, 0.0);
            GeoPoint2D arcP2 = new GeoPoint2D(0.0, 0.0);
            GeoPoint2D arcCenter = new GeoPoint2D(0.0, 0.0);
            double distCS = double.MaxValue; // distance of the chosen center from the pick
            for (int i = 0; i < centerPoints.Length; ++i) // over all center candidates
            {
                bool ok1 = false, ok2 = false;
                GeoPoint2D arcP1Loc = new GeoPoint2D(0.0, 0.0);
                GeoPoint2D arcP2Loc = new GeoPoint2D(0.0, 0.0);
                GeoPoint2D[] perpP = curve1_2D.PerpendicularFoot(centerPoints[i].p); // feet on curve 1
                double distCP = double.MaxValue;
                for (int j = 0; j < perpP.Length; ++j)
                {   // the foot must lie inside the curve and be at distance radius from the center
                    double loc = curve1_2D.PositionOf(perpP[j]);
                    if ((loc > locmin1) & (loc < locmax1) & (Math.Abs(Geometry.Dist(perpP[j], centerPoints[i].p) - radius) < Precision.eps))
                    {
                        double distLoc = Geometry.Dist(perpP[j], objectPoint2D);
                        if (distLoc < distCP) { distCP = distLoc; arcP1Loc = perpP[j]; }
                        ok1 = true;
                    }
                }
                if (ok1) // curve 1 ok, now the same for curve 2
                {
                    perpP = curve2_2D.PerpendicularFoot(centerPoints[i].p); // feet on curve 2
                    distCP = double.MaxValue;
                    for (int j = 0; j < perpP.Length; ++j)
                    {
                        double loc = curve2_2D.PositionOf(perpP[j]);
                        if ((loc > locmin2) & (loc < locmax2) & (Math.Abs(Geometry.Dist(perpP[j], centerPoints[i].p) - radius) < Precision.eps))
                        {
                            double distLoc = Geometry.Dist(perpP[j], objectPoint2D);
                            if (distLoc < distCP) { distCP = distLoc; arcP2Loc = perpP[j]; }
                            ok2 = true;
                        }
                    }
                }
                if (ok2)
                {
                    bool sel;
                    if (skipQuadrantFilter) // curves and corner are known: the foot constraints already isolate the inner fillet
                        sel = true;
                    else // take the center in the picked quadrant (same side of both tangents as the pick)
                        sel = Geometry.OnSameSide(centerPoints[i].p, pick2D, objectPoint2D, v1CutPoint) &&
                              Geometry.OnSameSide(centerPoints[i].p, pick2D, objectPoint2D, v2CutPoint);
                    if (sel)
                    {   // among all valid centers take the one closest to the pick
                        double distLoc = Geometry.Dist(centerPoints[i].p, pick2D);
                        if (distLoc < distCS)
                        {
                            distCS = distLoc;
                            arcCenter = centerPoints[i].p;
                            arcP1 = arcP1Loc;
                            arcP2 = arcP2Loc;
                            rndPos = true;
                        }
                    }
                }
            }
            if (rndPos && !Precision.IsEqual(arcP1, arcP2))
            {
                cornerPoint = pl.ToGlobal(objectPoint2D);
                Ellipse arc0 = Ellipse.Construct();
                Ellipse arc1 = Ellipse.Construct();
                arc0.SetArcPlaneCenterStartEndPoint(drawingPlane, arcCenter, arcP1, arcP2, pl, false);
                arc1.SetArcPlaneCenterStartEndPoint(drawingPlane, arcCenter, arcP1, arcP2, pl, true);
                // it is always the smaller of the two arcs
                arc = Math.Abs(arc0.SweepParameter) > Math.Abs(arc1.SweepParameter) ? arc1 : arc0;
                return true;
            }
            return false;
        }
    }
}
