using CADability.Curve2D;
using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// The uv values an <see cref="InterpolatedDualSurfaceCurve"/> stores on a periodic surface, and the periods the 2d
    /// curves of the faces refer to. A cone with a bore across its axis: the bore is a cylinder made of two faces, one
    /// with the domain [0, pi], the other one with [-pi, 0], so the intersection curves on the second one run up to the
    /// seam at u == 0 and the domain is not the standard period of PositionOf.
    /// </summary>
    [TestClass]
    public class IntersectionCurvePeriodTests
    {
        private static readonly GeoPoint coneBase = new GeoPoint(100.0, 50.0, 20.0);

        private static Solid Cone() => Make3D.MakeCone(coneBase, GeoVector.XAxis, 30.0 * GeoVector.ZAxis, 20.0, 0.0);
        private static Solid Bore() => Make3D.MakeCylinder(coneBase + new GeoVector(0.0, -50.0, 10.0), 3.0 * GeoVector.XAxis, 100.0 * GeoVector.YAxis);

        /// <summary>The largest step between neighbouring points of <paramref name="c2d"/>: a jump by a period shows up here.</summary>
        private static double LargestStep(ICurve2D c2d)
        {
            double res = 0.0;
            GeoPoint2D last = c2d.StartPoint;
            for (int i = 1; i <= 40; i++)
            {
                GeoPoint2D p = c2d.PointAt(i / 40.0);
                res = Math.Max(res, p | last);
                last = p;
            }
            return Math.Max(res, last | c2d.EndPoint);
        }

        [TestMethod]
        public void a_trimmed_intersection_curve_stays_in_one_period_at_the_seam()
        {
            Face coneFace = Cone().Shells[0].Faces.First(f => f.Surface is ConicalSurface && f.Domain.Right > 4.0); // u in [pi, 2 pi]
            Face cylFace = Bore().Shells[0].Faces.First(f => f.Surface is CylindricalSurface && f.Domain.Left > 1.0); // u in [pi, 2 pi]
            IDualSurfaceCurve[] dscs = coneFace.Surface.GetDualSurfaceCurves(coneFace.Domain, cylFace.Surface, cylFace.Domain, new List<GeoPoint>(), null);
            InterpolatedDualSurfaceCurve idsc = dscs.Select(d => d.Curve3D).OfType<InterpolatedDualSurfaceCurve>().FirstOrDefault();
            Assert.IsNotNull(idsc, "the cone and the bore intersect in an InterpolatedDualSurfaceCurve");

            // The curve runs a little beyond both seams of the cylinder face. Trimmed to the seams, as a Boolean operation
            // does it at the vertices there, its new end points lie exactly on u == pi and u == 2 pi.
            double r = 20.0 * (50.0 - 30.0) / 30.0; // radius of the cone at the height of the bore axis
            double y = Math.Sqrt(r * r - 9.0);
            GeoPoint onSeam1 = new GeoPoint(97.0, 50.0 + (idsc.StartPoint.y > 50.0 ? y : -y), 30.0);
            GeoPoint onSeam2 = new GeoPoint(103.0, onSeam1.y, 30.0);
            double p1 = idsc.PositionOf(onSeam1), p2 = idsc.PositionOf(onSeam2);
            Assert.IsTrue((idsc.PointAt(p1) | onSeam1) < 1e-4 && (idsc.PointAt(p2) | onSeam2) < 1e-4, "the seam points are on the curve (PointAt is the approximating spline)");

            InterpolatedDualSurfaceCurve trimmed = idsc.Clone() as InterpolatedDualSurfaceCurve;
            trimmed.Trim(Math.Min(p1, p2), Math.Max(p1, p2));
            ICurve2D onCylinder = trimmed.CurveOnSurface2;
            Assert.IsTrue(LargestStep(onCylinder) < 0.5, $"the 2d curve on the cylinder jumps: {onCylinder.StartPoint} .. {onCylinder.PointAt(0.5)} .. {onCylinder.EndPoint}");
            Assert.AreEqual(Math.PI, Math.Abs(onCylinder.EndPoint.x - onCylinder.StartPoint.x), 1e-6, "half a turn around the bore");
        }
    }
}
