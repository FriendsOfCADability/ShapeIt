using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// A periodic NurbsSurface is a clamped surface which is closed in u or v: the periodic flags only move a parameter
    /// outside the knot span into the span, the NURBS evaluator is the same as for the non periodic surface. Before, the
    /// initialization appended poles for the periodic directions without matching knots, so a rational surface periodic in u
    /// (e.g. a sphere from a STEP file) and any surface periodic in u and v could not even be constructed or cloned.
    /// </summary>
    [TestClass]
    public class NurbsSurfacePeriodicTests
    {
        private const double radius = 20.0;

        /// <summary>
        /// A sphere of radius 20 around the origin as a clamped rational surface of degree 2: u runs around the z-axis
        /// (9 poles, 4 spans, closed), v from the south pole to the north pole (5 poles, 2 spans).
        /// </summary>
        private static void SphereData(out GeoPoint[,] poles, out double[,] weights, out double[] uKnots, out int[] uMults, out double[] vKnots, out int[] vMults)
        {
            double w = Math.Sqrt(0.5);
            // the control polygon of a circle: the corners of a square have the weight w
            double[] cx = { 1, 1, 0, -1, -1, -1, 0, 1, 1 };
            double[] cy = { 0, 1, 1, 1, 0, -1, -1, -1, 0 };
            double[] wu = { 1, w, 1, w, 1, w, 1, w, 1 };
            // the meridian in the xz-plane from the south pole to the north pole
            double[] mr = { 0, 1, 1, 1, 0 };
            double[] mz = { -1, -1, 0, 1, 1 };
            double[] wv = { 1, w, 1, w, 1 };
            poles = new GeoPoint[9, 5];
            weights = new double[9, 5];
            for (int i = 0; i < 9; i++)
                for (int j = 0; j < 5; j++)
                {
                    poles[i, j] = new GeoPoint(radius * cx[i] * mr[j], radius * cy[i] * mr[j], radius * mz[j]);
                    weights[i, j] = wu[i] * wv[j];
                }
            uKnots = new double[] { 0, 1, 2, 3, 4 };
            uMults = new int[] { 3, 2, 2, 2, 3 };
            vKnots = new double[] { 0, 1, 2 };
            vMults = new int[] { 3, 2, 3 };
        }

        private static NurbsSurface Sphere(bool rational, bool uPeriodic, bool vPeriodic)
        {
            SphereData(out GeoPoint[,] poles, out double[,] weights, out double[] uKnots, out int[] uMults, out double[] vKnots, out int[] vMults);
            return new NurbsSurface(poles, rational ? weights : null, uKnots, vKnots, uMults, vMults, 2, 2, uPeriodic, vPeriodic);
        }

        private static IEnumerable<GeoPoint2D> Grid()
        {
            for (int i = 0; i <= 16; i++)
                for (int j = 0; j <= 8; j++)
                    yield return new GeoPoint2D(i * 4.0 / 16, j * 2.0 / 8);
        }

        [DataTestMethod]
        [DataRow(true, true, false)]
        [DataRow(true, false, true)]
        [DataRow(true, true, true)]
        [DataRow(false, true, false)]
        [DataRow(false, false, true)]
        [DataRow(false, true, true)]
        public void a_periodic_surface_is_the_clamped_surface(bool rational, bool uPeriodic, bool vPeriodic)
        {
            NurbsSurface clamped = Sphere(rational, false, false);
            NurbsSurface periodic = Sphere(rational, uPeriodic, vPeriodic);
            Assert.AreEqual(uPeriodic, periodic.IsUPeriodic);
            Assert.AreEqual(vPeriodic, periodic.IsVPeriodic);
            ISurface clone = periodic.Clone();
            foreach (GeoPoint2D uv in Grid())
            {
                // the end of the knot span is mapped to the start in the periodic direction, which is the same point on a closed surface.
                // The sphere is not closed in v (south and north pole), so the end of v is skipped when v is flagged as periodic
                if (vPeriodic && uv.y == 2.0) continue;
                GeoPoint expected = clamped.PointAt(uv);
                Assert.AreEqual(0.0, expected | periodic.PointAt(uv), 1e-9, $"periodic surface at {uv}");
                Assert.AreEqual(0.0, expected | clone.PointAt(uv), 1e-9, $"clone at {uv}");
                if (rational) Assert.AreEqual(radius, expected | GeoPoint.Origin, 1e-9, $"point on the sphere at {uv}");
            }
        }

        [TestMethod]
        public void parameters_outside_the_period_are_moved_into_the_knot_span()
        {
            NurbsSurface sphere = Sphere(true, true, false);
            foreach (GeoPoint2D uv in Grid())
            {
                GeoPoint p = sphere.PointAt(uv);
                Assert.AreEqual(0.0, p | sphere.PointAt(new GeoPoint2D(uv.x + 4, uv.y)), 1e-9, $"one period later at {uv}");
                Assert.AreEqual(0.0, p | sphere.PointAt(new GeoPoint2D(uv.x - 8, uv.y)), 1e-9, $"two periods earlier at {uv}");
            }
            // close to the seam from both sides
            for (int j = 1; j < 8; j++)
            {
                double v = j * 2.0 / 8;
                GeoPoint atSeam = sphere.PointAt(new GeoPoint2D(0, v));
                Assert.AreEqual(0.0, atSeam | sphere.PointAt(new GeoPoint2D(4 - 1e-12, v)), 1e-9, "just before the end of the period");
                Assert.AreEqual(0.0, atSeam | sphere.PointAt(new GeoPoint2D(-1e-12, v)), 1e-9, "just before the start of the period");
            }
        }

        /// <summary>
        /// The surface through full circles is rational and periodic in u, it could not be constructed before.
        /// </summary>
        [TestMethod]
        public void a_surface_through_full_circles_can_be_constructed()
        {
            Ellipse[] circles = new Ellipse[4];
            for (int i = 0; i < circles.Length; i++)
            {
                circles[i] = Ellipse.Construct();
                circles[i].SetCirclePlaneCenterRadius(Plane.XYPlane, new GeoPoint(0, 0, 10 * i), 5 + i);
            }
            NurbsSurface surface = new NurbsSurface(circles);
            Assert.IsTrue(surface.IsUPeriodic, "closed around the circles");
            ISurface clone = surface.Clone();
            for (int i = 0; i < circles.Length; i++)
            {
                GeoPoint2D uv = surface.PositionOf(circles[i].PointAt(0.3));
                Assert.AreEqual(0.0, circles[i].PointAt(0.3) | surface.PointAt(uv), 1e-6, $"through circle {i}");
                Assert.AreEqual(0.0, surface.PointAt(uv) | clone.PointAt(uv), 1e-12, $"clone at circle {i}");
            }
        }
    }
}
