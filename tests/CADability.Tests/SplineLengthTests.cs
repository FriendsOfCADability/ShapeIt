using CADability.Curve2D;
using CADability.GeoObject;

namespace CADability.Tests
{
    // ICurve.Length of a BSpline used to return the chord polygon through the spans instead of the arc
    // length: Approximate(true, negative maxError) makes exactly one line per interpolation interval, and
    // for the nine pole rational quadratic circle those intervals are the four 90 degree spans. The circle
    // of radius 30 therefore measured 4*30*sqrt(2) = 169.7056 instead of 2*pi*30 = 188.4956, 10 percent short.
    [TestClass]
    public class SplineLengthTests
    {
        /// <summary>
        /// The exact rational quadratic circle: nine poles, weights 1 and sqrt(2)/2 alternating, four spans
        /// of 90 degrees each.
        /// </summary>
        private static BSpline2D NurbsCircle(double radius)
        {
            GeoPoint2D[] poles = new GeoPoint2D[]
            {
                new GeoPoint2D(radius, 0), new GeoPoint2D(radius, radius),
                new GeoPoint2D(0, radius), new GeoPoint2D(-radius, radius),
                new GeoPoint2D(-radius, 0), new GeoPoint2D(-radius, -radius),
                new GeoPoint2D(0, -radius), new GeoPoint2D(radius, -radius),
                new GeoPoint2D(radius, 0)
            };
            double w = Math.Sqrt(2.0) / 2.0;
            double[] weights = new double[] { 1, w, 1, w, 1, w, 1, w, 1 };
            double[] knots = new double[] { 0, 1, 2, 3, 4 };
            int[] mults = new int[] { 3, 2, 2, 2, 3 };
            return new BSpline2D(poles, weights, knots, mults, 2, false, 0, 4);
        }

        [TestMethod]
        public void nurbs_circle_is_exact()
        {   // the curve itself is right - what follows is purely about measuring it
            BSpline2D bsp = NurbsCircle(30);
            double maxError = 0.0;
            for (int i = 0; i <= 200; ++i)
            {
                double r = bsp.PointAt(i / 200.0).ToVector().Length;
                maxError = Math.Max(maxError, Math.Abs(r - 30));
            }
            Assert.IsTrue(maxError < 1e-12, "rational circle is off by " + maxError);
        }

        [TestMethod]
        public void bspline2d_length_is_arc_length()
        {
            BSpline2D bsp = NurbsCircle(30);
            double expected = 2 * Math.PI * 30;
            Assert.AreEqual(expected, bsp.Length, expected * 1e-6,
                "BSpline2D.Length must be the arc length, not the chord polygon (" + 4 * 30 * Math.Sqrt(2) + ")");
        }

        [TestMethod]
        public void bspline3d_length_is_arc_length()
        {
            BSpline bsp = (BSpline)NurbsCircle(30).MakeGeoObject(Plane.XYPlane);
            double expected = 2 * Math.PI * 30;
            Assert.AreEqual(expected, ((ICurve)bsp).Length, expected * 1e-6,
                "BSpline.Length must be the arc length, not the chord polygon");
        }

        [TestMethod]
        public void non_planar_bspline3d_length_is_arc_length()
        {   // a helix of two turns: r = 10, pitch 20, so one turn is sqrt((2*pi*10)^2 + 20^2)
            const int n = 61;
            GeoPoint[] pnts = new GeoPoint[n];
            for (int i = 0; i < n; ++i)
            {
                double a = i * 4 * Math.PI / (n - 1);
                pnts[i] = new GeoPoint(10 * Math.Cos(a), 10 * Math.Sin(a), 20 * a / (2 * Math.PI));
            }
            BSpline bsp = BSpline.Construct();
            Assert.IsTrue(bsp.ThroughPoints(pnts, 3, false));
            double expected = 2 * Math.Sqrt(2 * Math.PI * 10 * (2 * Math.PI * 10) + 20 * 20);
            Assert.AreEqual(expected, ((ICurve)bsp).Length, expected * 1e-4,
                "helix length");
        }

        [TestMethod]
        public void line_like_spline_length_is_the_distance()
        {   // degenerate but common: a spline through collinear points must measure the straight distance
            BSpline bsp = BSpline.Construct();
            Assert.IsTrue(bsp.ThroughPoints(new GeoPoint[]
            {
                new GeoPoint(0, 0, 0), new GeoPoint(1, 2, 2), new GeoPoint(2, 4, 4), new GeoPoint(3, 6, 6)
            }, 3, false));
            double expected = new GeoPoint(3, 6, 6) | new GeoPoint(0, 0, 0);
            Assert.AreEqual(expected, ((ICurve)bsp).Length, expected * 1e-6);
        }

        [TestMethod]
        public void trimmed_spline_length_adds_up()
        {   // Length must respect startParam/endParam: two halves of the circle make the whole
            BSpline2D bsp = NurbsCircle(30);
            ICurve2D[] halves = bsp.Split(0.5);
            Assert.AreEqual(2, halves.Length);
            double sum = halves[0].Length + halves[1].Length;
            Assert.AreEqual(2 * Math.PI * 30, sum, 2 * Math.PI * 30 * 1e-6);
        }
    }
}
