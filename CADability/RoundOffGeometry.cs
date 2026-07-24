using CADability.Curve2D;
using CADability.GeoObject;
using System;

namespace CADability
{
    /// <summary>
    /// Pure geometric core of the "round off corner" tool (see <see cref="CADability.Actions.RoundObjectsAction"/>), free
    /// of any interaction state so it can be unit tested. Given two curves that form a corner, a pick position and a
    /// radius it computes the tangential fillet arc.
    /// </summary>
    public static class RoundOffGeometry
    {
        /// <summary>
        /// Computes the tangential fillet arc of the given <paramref name="radius"/> in the corner where
        /// <paramref name="curve1"/> and <paramref name="curve2"/> meet, near <paramref name="pick"/>.
        /// The fillet centre is at distance <paramref name="radius"/> from both curves on the inner side of the corner,
        /// so it is found directly as the intersection of the two inner parallels (offset by the radius toward the region
        /// between the curves) — no trial of all offset combinations is needed. The tangent points are the perpendicular
        /// feet of the centre on the two curves.
        /// </summary>
        /// <param name="curve1">first curve of the corner</param>
        /// <param name="curve2">second curve of the corner</param>
        /// <param name="pick">the pick point in world coordinates, used to pick the corner when the curves cross more than once</param>
        /// <param name="radius">the fillet radius, must be &gt; 0</param>
        /// <param name="drawingPlane">the active drawing plane, used to orient the resulting arc</param>
        /// <param name="arc">the resulting fillet arc (the smaller of the two possible arcs), or null</param>
        /// <param name="cornerPoint">the corner in world coordinates, or the origin</param>
        /// <returns>true if a fillet arc was found</returns>
        public static bool TryComputeRoundOff(ICurve curve1, ICurve curve2, GeoPoint pick, double radius,
            Plane drawingPlane, out Ellipse arc, out GeoPoint cornerPoint)
        {
            arc = null;
            cornerPoint = GeoPoint.Origin;
            if (curve1 == null || curve2 == null || curve1 == curve2 || radius <= 0.0) return false;
            if (!Curves.GetCommonPlane(curve1, curve2, out Plane pl)) return false;

            ICurve2D c1 = curve1.GetProjectedCurve(pl);
            ICurve2D c2 = curve2.GetProjectedCurve(pl);
            if (c1 is Path2D p1) p1.Flatten();
            if (c2 is Path2D p2) p2.Flatten();

            // the corner is the intersection of the two curves closest to the pick
            GeoPoint2D pick2D = pl.Project(pick);
            GeoPoint2DWithParameter[] crossings = c1.Intersect(c2);
            if (crossings.Length == 0) return false;
            GeoPoint2D corner = crossings[0].p;
            for (int i = 1; i < crossings.Length; i++)
                if (Geometry.Dist(crossings[i].p, pick2D) < Geometry.Dist(corner, pick2D)) corner = crossings[i].p;
            double cornerPar1 = c1.PositionOf(corner);
            double cornerPar2 = c2.PositionOf(corner);

            // The into-curve tangents at the corner (pointing away from the corner into each curve's body) span the inner
            // angle; their normalized sum bisects it, i.e. points into the region where the fillet has to sit.
            GeoVector2D bisector = IntoDirection(c1, cornerPar1).Normalized + IntoDirection(c2, cornerPar2).Normalized;
            if (bisector.Length < 1e-9) return false; // the curves run collinearly here: no real corner

            ICurve2D inner1 = InnerParallel(c1, radius, corner, bisector);
            ICurve2D inner2 = InnerParallel(c2, radius, corner, bisector);
            if (inner1 == null || inner2 == null) return false;

            // the centre is where the two inner parallels meet; among the candidates take the valid one nearest the corner
            GeoPoint2D center = GeoPoint2D.Origin, tangent1 = GeoPoint2D.Origin, tangent2 = GeoPoint2D.Origin;
            double bestDist = double.MaxValue;
            foreach (GeoPoint2DWithParameter ip in inner1.Intersect(inner2))
            {
                // the tangent points are the perpendicular feet of the centre on the two curves; both must lie within
                // the curves (allowing a virtual corner beyond a curve's end) at exactly the radius
                if (!TangentPoint(c1, ip.p, cornerPar1, radius, out GeoPoint2D t1)) continue;
                if (!TangentPoint(c2, ip.p, cornerPar2, radius, out GeoPoint2D t2)) continue;
                if (Precision.IsEqual(t1, t2)) continue; // degenerate
                double dist = Geometry.Dist(ip.p, corner);
                if (dist < bestDist) { bestDist = dist; center = ip.p; tangent1 = t1; tangent2 = t2; }
            }
            if (bestDist == double.MaxValue) return false;

            cornerPoint = pl.ToGlobal(corner);
            Ellipse arc0 = Ellipse.Construct();
            Ellipse arc1 = Ellipse.Construct();
            arc0.SetArcPlaneCenterStartEndPoint(drawingPlane, center, tangent1, tangent2, pl, false);
            arc1.SetArcPlaneCenterStartEndPoint(drawingPlane, center, tangent1, tangent2, pl, true);
            // it is always the smaller of the two arcs
            arc = Math.Abs(arc0.SweepParameter) > Math.Abs(arc1.SweepParameter) ? arc1 : arc0;
            return true;
        }

        /// <summary>
        /// The tangent of <paramref name="curve"/> at the corner, pointing away from the corner into the curve's body.
        /// The corner is (near) an end of the curve, so the direction is flipped when the corner is at the curve's end.
        /// </summary>
        private static GeoVector2D IntoDirection(ICurve2D curve, double cornerPar)
        {
            GeoVector2D tangent = curve.DirectionAt(cornerPar);
            return Math.Abs(cornerPar - 1.0) < Math.Abs(cornerPar - 0.0) ? -tangent : tangent;
        }

        /// <summary>
        /// The parallel of <paramref name="curve"/> at distance <paramref name="radius"/> offset toward the inner side of
        /// the corner (the side the <paramref name="bisector"/> points to).
        /// </summary>
        private static ICurve2D InnerParallel(ICurve2D curve, double radius, GeoPoint2D corner, GeoVector2D bisector)
        {
            ICurve2D positive = curve.Parallel(radius, false, 0.0, 0.0);
            ICurve2D negative = curve.Parallel(-radius, false, 0.0, 0.0);
            if (positive == null) return negative;
            if (negative == null) return positive;
            // the offset that moves the corner toward the inner region (positive dot with the bisector) is the inner one;
            // the perpendicular foot of the corner on the parallel gives the true offset direction
            GeoPoint2D[] feet = positive.PerpendicularFoot(corner);
            if (feet.Length == 0) return positive;
            GeoVector2D toPositive = feet[0] - corner;
            return toPositive * bisector > 0.0 ? positive : negative;
        }

        /// <summary>
        /// The tangent point where a fillet centred at <paramref name="center"/> touches <paramref name="curve"/>: the
        /// perpendicular foot at distance <paramref name="radius"/> that lies within the curve (the range is extended
        /// toward the corner when the corner itself is a virtual intersection beyond the curve's end). Returns false if
        /// there is no such foot (the fillet does not fit).
        /// </summary>
        private static bool TangentPoint(ICurve2D curve, GeoPoint2D center, double cornerPar, double radius, out GeoPoint2D tangent)
        {
            tangent = GeoPoint2D.Origin;
            double bestError = radius * 1e-3; // the foot must sit at the radius; a farther foot is a different branch
            bool found = false;
            foreach (GeoPoint2D foot in curve.PerpendicularFoot(center))
            {
                if (!WithinCurve(curve.PositionOf(foot), cornerPar)) continue;
                double error = Math.Abs(Geometry.Dist(foot, center) - radius);
                if (error < bestError) { bestError = error; tangent = foot; found = true; }
            }
            return found;
        }

        // true if 'par' lies within the curve; the range is extended toward the corner when the corner itself is a
        // virtual intersection beyond the curve's end
        private static bool WithinCurve(double par, double cornerPar)
        {
            return par > Math.Min(0.0, cornerPar) && par < Math.Max(1.0, cornerPar);
        }
    }
}
