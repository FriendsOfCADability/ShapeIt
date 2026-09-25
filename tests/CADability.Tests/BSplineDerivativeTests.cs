using CADability;
using CADability.Curve2D;
using CADability.GeoObject;
using System;

namespace CADability.Tests
{
    /// <summary>
    /// <see cref="ICurve.TryPointDeriv2At"/> on a BSpline. It reads the nurbs helper, which is built lazily,
    /// so it has to ask for that helper like every other accessor does - it used to read the fields straight
    /// and threw on a spline nobody had evaluated yet. The symptom was nasty because it depended on the order
    /// of the calls: the same spline worked as soon as anything had called PointAt on it.
    /// </summary>
    [TestClass]
    public class BSplineDerivativeTests
    {
        /// <summary>
        /// The nine pole rational quadratic circle - the exact circle in NURBS form. Rational and planar, which
        /// is the combination that used to fail: only nurbs2d is set for it, and that was the branch reached
        /// through a bare else.
        /// </summary>
        private static BSpline RationalCircle(Plane plane, double r)
        {
            GeoPoint2D[] poles2d =
            {
                new GeoPoint2D(r, 0), new GeoPoint2D(r, r), new GeoPoint2D(0, r), new GeoPoint2D(-r, r),
                new GeoPoint2D(-r, 0), new GeoPoint2D(-r, -r), new GeoPoint2D(0, -r), new GeoPoint2D(r, -r),
                new GeoPoint2D(r, 0)
            };
            GeoPoint[] poles = new GeoPoint[poles2d.Length];
            for (int i = 0; i < poles.Length; i++) poles[i] = plane.ToGlobal(poles2d[i]);
            double w = Math.Sqrt(2.0) / 2.0;
            BSpline spline = BSpline.Construct();
            spline.SetData(2, poles, new double[] { 1, w, 1, w, 1, w, 1, w, 1 },
                           new double[] { 0, 1, 2, 3, 4 }, new int[] { 3, 2, 2, 2, 3 }, false);
            return spline;
        }

        private static BSpline NonRationalSpline()
        {
            BSpline spline = BSpline.Construct();
            spline.ThroughPoints(new GeoPoint[]
            {
                new GeoPoint(0, 0, 0), new GeoPoint(10, 4, 1), new GeoPoint(20, -3, 5), new GeoPoint(30, 6, 2)
            }, 3, false);
            return spline;
        }

        [TestMethod]
        public void the_second_derivative_works_on_a_spline_nobody_has_touched_yet()
        {
            // No warm up call on purpose: this is the whole point of the test.
            ICurve fresh = RationalCircle(Plane.XYPlane, 30.0);
            Assert.IsTrue(fresh.TryPointDeriv2At(0.3, out GeoPoint point, out GeoVector _, out GeoVector _),
                "a rational planar spline has to deliver its second derivative");
            Assert.IsTrue((point | fresh.PointAt(0.3)) < 1e-9, "and at the right place");
        }

        [TestMethod]
        public void the_result_does_not_depend_on_what_was_called_before()
        {
            ICurve fresh = RationalCircle(Plane.XYPlane, 30.0);
            ICurve warm = RationalCircle(Plane.XYPlane, 30.0);
            GeoPoint _ = warm.PointAt(0.5); // this alone used to be the difference between throwing and working

            fresh.TryPointDeriv2At(0.3, out GeoPoint p1, out GeoVector d1, out GeoVector dd1);
            warm.TryPointDeriv2At(0.3, out GeoPoint p2, out GeoVector d2, out GeoVector dd2);
            Assert.IsTrue((p1 | p2) < 1e-12, "point");
            Assert.IsTrue((d1 - d2).Length < 1e-12, "first derivative");
            Assert.IsTrue((dd1 - dd2).Length < 1e-12, "second derivative");
        }

        [TestMethod]
        public void the_derivatives_are_the_derivatives()
        {
            foreach ((string what, ICurve curve) in new (string, ICurve)[]
            {
                ("rational circle in the xy plane", RationalCircle(Plane.XYPlane, 30.0)),
                ("rational circle in a tilted plane",
                    RationalCircle(new Plane(new GeoPoint(30, 0, 0), GeoVector.XAxis, GeoVector.ZAxis), 8.0)),
                ("non rational spline", NonRationalSpline())
            })
            {
                const double h = 1e-6;
                // Away from the knots on purpose. The nine pole circle has its interior knots at multiplicity
                // 2, which for degree 2 leaves the curve C1 but not C2: the second derivative jumps there, and
                // a central difference across the jump would be comparing against an average of two different
                // one sided values. Position 0.5 is exactly such a knot - parameter 2.0 of 0..4.
                for (int i = 1; i < 10; i++)
                {
                    double u = (i + 0.37) / 10.0;
                    Assert.IsTrue(curve.TryPointDeriv2At(u, out GeoPoint point, out GeoVector deriv1, out GeoVector deriv2),
                        $"{what}: no second derivative at {u}");
                    Assert.IsTrue((point | curve.PointAt(u)) < 1e-9, $"{what} at {u}: point");

                    GeoVector numeric1 = (curve.PointAt(u + h) - curve.PointAt(u - h)) / (2.0 * h);
                    AssertClose($"{what} at {u}: first derivative", numeric1, deriv1);

                    curve.TryPointDeriv2At(u + h, out GeoPoint _, out GeoVector after, out GeoVector _);
                    curve.TryPointDeriv2At(u - h, out GeoPoint _, out GeoVector before, out GeoVector _);
                    AssertClose($"{what} at {u}: second derivative", (after - before) / (2.0 * h), deriv2);
                }
            }
        }

        private static void AssertClose(string what, GeoVector expected, GeoVector actual)
        {
            double error = (expected - actual).Length / Math.Max(1.0, expected.Length);
            Assert.IsTrue(error < 1e-5, $"{what}: expected about {expected}, got {actual} (relative {error:E3})");
        }

        /// <summary>
        /// The same for <see cref="BSpline2D"/>: it returned the derivatives by the knot parameter, while DirectionAt
        /// and the interface ask for the derivatives by the normalized position. Wrong by the length of the knot
        /// range (squared for the second derivative), which made the Newton steps of GeneralCurve2D too long by that
        /// factor and HelicalSurface.Derivative2At wrong for a BSpline2D profile.
        /// </summary>
        [TestMethod]
        public void the_derivatives_of_a_BSpline2D_are_the_derivatives()
        {
            double r = 5.0;
            double w = Math.Sqrt(2.0) / 2.0;
            BSpline2D circle = new BSpline2D(new GeoPoint2D[]
            {
                new GeoPoint2D(r, 0), new GeoPoint2D(r, r), new GeoPoint2D(0, r), new GeoPoint2D(-r, r),
                new GeoPoint2D(-r, 0), new GeoPoint2D(-r, -r), new GeoPoint2D(0, -r), new GeoPoint2D(r, -r),
                new GeoPoint2D(r, 0)
            }, new double[] { 1, w, 1, w, 1, w, 1, w, 1 }, new double[] { 0, 1, 2, 3, 4 }, new int[] { 3, 2, 2, 2, 3 }, 2, false, 0, 4);
            BSpline2D spline = new BSpline2D(new GeoPoint2D[]
            {
                new GeoPoint2D(5, 0), new GeoPoint2D(8, 1), new GeoPoint2D(6, 3), new GeoPoint2D(9, 5), new GeoPoint2D(7, 8)
            }, null, new double[] { 2, 3.5, 5 }, new int[] { 4, 1, 4 }, 3, false, 2, 5);
            foreach ((string what, ICurve2D curve) in new (string, ICurve2D)[]
            {
                ("rational 2d circle with knots from 0 to 4", circle),
                ("non rational 2d spline with knots from 2 to 5", spline)
            })
            {
                const double h = 1e-6;
                for (int i = 1; i < 10; i++)
                {
                    double u = (i + 0.37) / 10.0; // away from the knots, see above
                    Assert.IsTrue(curve.TryPointDeriv2At(u, out GeoPoint2D point, out GeoVector2D deriv1, out GeoVector2D deriv2),
                        $"{what}: no second derivative at {u}");
                    Assert.IsTrue((point | curve.PointAt(u)) < 1e-9, $"{what} at {u}: point");

                    GeoVector2D numeric1 = (1.0 / (2.0 * h)) * (curve.PointAt(u + h) - curve.PointAt(u - h));
                    AssertClose($"{what} at {u}: first derivative", numeric1, deriv1);
                    AssertClose($"{what} at {u}: DirectionAt", numeric1, curve.DirectionAt(u));

                    curve.TryPointDeriv2At(u + h, out GeoPoint2D _, out GeoVector2D after, out GeoVector2D _);
                    curve.TryPointDeriv2At(u - h, out GeoPoint2D _, out GeoVector2D before, out GeoVector2D _);
                    AssertClose($"{what} at {u}: second derivative", (1.0 / (2.0 * h)) * (after - before), deriv2);
                }
            }
        }

        private static void AssertClose(string what, GeoVector2D expected, GeoVector2D actual)
        {
            double error = (expected - actual).Length / Math.Max(1.0, expected.Length);
            Assert.IsTrue(error < 1e-5, $"{what}: expected about ({expected.x}, {expected.y}), got ({actual.x}, {actual.y}) (relative {error:E3})");
        }
    }
}
