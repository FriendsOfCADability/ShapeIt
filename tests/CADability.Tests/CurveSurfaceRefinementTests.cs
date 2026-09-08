using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="ISurfaceImpl.RefineCurveIntersection"/>: the Newton refinement of an approximate
    /// intersection point between a surface and a curve. Every case here has an answer that can be written down,
    /// so the tests state what is right rather than what the code happened to produce.
    /// </summary>
    [TestClass]
    public class CurveSurfaceRefinementTests
    {
        public TestContext TestContext { get; set; }

        /// <summary>The accuracy a Newton iteration should reach with geometry of this size.</summary>
        private const double exact = 1e-9;

        private static CylindricalSurface CylinderAroundZAxis(double radius)
        {
            return new CylindricalSurface(GeoPoint.Origin, radius * GeoVector.XAxis, radius * GeoVector.YAxis, GeoVector.ZAxis);
        }

        private static SphericalSurface Sphere(GeoPoint centre, double radius)
        {
            return new SphericalSurface(centre, radius * GeoVector.XAxis, radius * GeoVector.YAxis, radius * GeoVector.ZAxis);
        }

        private static Ellipse Circle(Plane plane, GeoPoint centre, double radius)
        {
            Ellipse res = Ellipse.Construct();
            res.SetCirclePlaneCenterRadius(plane, centre, radius);
            return res;
        }

        /// <summary>
        /// Starting values as they come from an approximate intersection: the exact point moved by
        /// <paramref name="offset"/> and projected onto surface and curve.
        /// </summary>
        private static void ApproximatePosition(ISurface surface, ICurve curve, GeoPoint exactPoint, GeoVector offset,
            out double uOnCurve, out GeoPoint2D uvOnSurface)
        {
            GeoPoint approximation = exactPoint + offset;
            uvOnSurface = surface.PositionOf(approximation);
            uOnCurve = curve.PositionOf(approximation);
        }

        private void Dump(string what, ISurface surface, ICurve curve, double uOnCurve, GeoPoint2D uvOnSurface, GeoPoint ip, bool found)
        {
            TestContext.WriteLine($"{what}: found = {found}, point = {ip}, u = {uOnCurve}, uv = {uvOnSurface}, " +
                $"gap = {surface.PointAt(uvOnSurface) | curve.PointAt(uOnCurve):E3}");
        }

        /// <summary>
        /// A line through a sphere, a plain transversal intersection. The line y==3, z==4 meets the sphere with
        /// radius 10 where x*x == 100-9-16, i.e. at x == sqrt(75).
        /// </summary>
        [TestMethod]
        public void LineThroughSphereBecomesExact()
        {
            SphericalSurface sphere = Sphere(GeoPoint.Origin, 10);
            Line line = Line.TwoPoints(new GeoPoint(-20, 3, 4), new GeoPoint(20, 3, 4));
            GeoPoint expected = new GeoPoint(Math.Sqrt(75), 3, 4);

            ApproximatePosition(sphere, line, expected, new GeoVector(0.05, 0.03, -0.02), out double u, out GeoPoint2D uv);
            bool found = sphere.RefineCurveIntersection(line, ref u, ref uv, out GeoPoint ip);
            Dump("line/sphere", sphere, line, u, uv, ip, found);

            Assert.IsTrue(found);
            Assert.AreEqual(0.0, expected | ip, exact);
            Assert.AreEqual(0.0, sphere.PointAt(uv) | line.PointAt(u), exact); // the two parameters describe the same point
        }

        /// <summary>
        /// A line through the tube of a torus: the line x==10, y==0 meets the torus with major radius 10 and minor
        /// radius 3 at z == 3 and z == -3.
        /// </summary>
        [TestMethod]
        public void LineThroughTorusBecomesExact()
        {
            ToroidalSurface torus = new ToroidalSurface(GeoPoint.Origin, GeoVector.XAxis, GeoVector.YAxis, GeoVector.ZAxis, 10, 3);
            Line line = Line.TwoPoints(new GeoPoint(10, 0, -10), new GeoPoint(10, 0, 10));
            GeoPoint expected = new GeoPoint(10, 0, 3);

            ApproximatePosition(torus, line, expected, new GeoVector(0.04, 0.0, 0.06), out double u, out GeoPoint2D uv);
            bool found = torus.RefineCurveIntersection(line, ref u, ref uv, out GeoPoint ip);
            Dump("line/torus", torus, line, u, uv, ip, found);

            Assert.IsTrue(found);
            Assert.AreEqual(0.0, expected | ip, exact);
        }

        /// <summary>
        /// A line which touches a cylinder: here the system for the transversal intersection is singular, the
        /// contact point can only be found with the tangential system.
        /// </summary>
        [TestMethod]
        public void LineTangentialToCylinderFindsContact()
        {
            CylindricalSurface cylinder = CylinderAroundZAxis(5);
            Line line = Line.TwoPoints(new GeoPoint(-10, 5, 2), new GeoPoint(10, 5, 2));
            GeoPoint expected = new GeoPoint(0, 5, 2);

            ApproximatePosition(cylinder, line, expected, new GeoVector(0.1, 0.0, 0.05), out double u, out GeoPoint2D uv);
            bool found = cylinder.RefineCurveIntersection(line, ref u, ref uv, out GeoPoint ip);
            Dump("tangential line/cylinder", cylinder, line, u, uv, ip, found);

            Assert.IsTrue(found);
            Assert.AreEqual(0.0, expected | ip, exact);
            Assert.AreEqual(0.0, cylinder.PointAt(uv) | line.PointAt(u), exact);
        }

        /// <summary>
        /// A circle which touches a plane in a single point. The distance between curve and surface has a fourth
        /// order zero here, which no distance minimization can resolve, but the tangential system can.
        /// </summary>
        [TestMethod]
        public void CircleTangentialToPlaneFindsContact()
        {
            PlaneSurface plane = new PlaneSurface(new Plane(GeoPoint.Origin, GeoVector.ZAxis));
            Ellipse circle = Circle(new Plane(new GeoPoint(0, 0, 3), GeoVector.XAxis, GeoVector.ZAxis), new GeoPoint(0, 0, 3), 3);
            GeoPoint expected = GeoPoint.Origin;

            ApproximatePosition(plane, circle, expected, new GeoVector(0.08, 0.0, 0.03), out double u, out GeoPoint2D uv);
            bool found = plane.RefineCurveIntersection(circle, ref u, ref uv, out GeoPoint ip);
            Dump("tangential circle/plane", plane, circle, u, uv, ip, found);

            Assert.IsTrue(found);
            Assert.AreEqual(0.0, expected | ip, exact);
        }

        /// <summary>
        /// A circle which touches a cylinder from outside: the circle with radius 5 around (10,0,2) and the
        /// cylinder with radius 5 around the z-axis meet in the single point (5,0,2).
        /// </summary>
        [TestMethod]
        public void CircleTangentialToCylinderFindsContact()
        {
            CylindricalSurface cylinder = CylinderAroundZAxis(5);
            Ellipse circle = Circle(new Plane(new GeoPoint(0, 0, 2), GeoVector.ZAxis), new GeoPoint(10, 0, 2), 5);
            GeoPoint expected = new GeoPoint(5, 0, 2);

            ApproximatePosition(cylinder, circle, expected, new GeoVector(0.0, 0.07, 0.0), out double u, out GeoPoint2D uv);
            bool found = cylinder.RefineCurveIntersection(circle, ref u, ref uv, out GeoPoint ip);
            Dump("tangential circle/cylinder", cylinder, circle, u, uv, ip, found);

            Assert.IsTrue(found);
            Assert.AreEqual(0.0, expected | ip, exact);
        }

        /// <summary>
        /// A single span bicubic Bézier patch, flat except for the four inner poles at z==1: x==3u, y==3v and z is the
        /// product of the two cubic Bézier polynomials over (0,1,1,0), which has its maximum 0.75 at 0.5. So the patch
        /// has a single summit at (1.5, 1.5, 0.5625) with a horizontal tangent plane.
        /// </summary>
        private static NurbsSurface BumpPatch()
        {
            GeoPoint[,] poles = new GeoPoint[4, 4];
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    double z = (i > 0 && i < 3 && j > 0 && j < 3) ? 1.0 : 0.0;
                    poles[i, j] = new GeoPoint(i, j, z);
                }
            }
            double[] knots = new double[] { 0, 0, 0, 0, 1, 1, 1, 1 };
            return new NurbsSurface(poles, null, knots, knots, 3, 3, false, false);
        }

        /// <summary>
        /// A line which grazes the summit of a spline patch, the case where a hull based intersection typically
        /// delivers a point which is off by the square root of the accuracy.
        /// </summary>
        [TestMethod]
        public void LineTangentialToNurbsPatchFindsContact()
        {
            NurbsSurface patch = BumpPatch();
            Line line = Line.TwoPoints(new GeoPoint(-1, 1.5, 0.5625), new GeoPoint(4, 1.5, 0.5625));
            GeoPoint expected = new GeoPoint(1.5, 1.5, 0.5625);

            ApproximatePosition(patch, line, expected, new GeoVector(0.06, 0.0, 0.0), out double u, out GeoPoint2D uv);
            bool found = patch.RefineCurveIntersection(line, ref u, ref uv, out GeoPoint ip);
            Dump("tangential line/nurbs patch", patch, line, u, uv, ip, found);

            Assert.IsTrue(found);
            Assert.AreEqual(0.0, expected | ip, exact);
        }

        /// <summary>
        /// A spline curve which touches a plane. The second derivative of a <see cref="BSpline"/> is calculated
        /// numerically, which only affects the speed of the convergence, not the solution itself: the equations of the
        /// tangential system contain the first derivative only.
        /// </summary>
        [TestMethod]
        public void BSplineTangentialToPlaneFindsContact()
        {
            // a single span cubic Bézier over the poles (0,0,0), (1,0,1), (2,0,1), (3,0,0): x==3t and z is the cubic
            // Bézier polynomial over (0,1,1,0), which has its maximum 0.75 at t==0.5
            BSpline arch = BSpline.Construct();
            arch.SetData(3, new GeoPoint[] { new GeoPoint(0, 0, 0), new GeoPoint(1, 0, 1), new GeoPoint(2, 0, 1), new GeoPoint(3, 0, 0) },
                null, new double[] { 0, 1 }, new int[] { 4, 4 }, false);
            PlaneSurface plane = new PlaneSurface(new Plane(new GeoPoint(0, 0, 0.75), GeoVector.ZAxis));
            GeoPoint expected = new GeoPoint(1.5, 0, 0.75);

            ApproximatePosition(plane, arch, expected, new GeoVector(0.05, 0.0, 0.0), out double u, out GeoPoint2D uv);
            bool found = plane.RefineCurveIntersection(arch, ref u, ref uv, out GeoPoint ip);
            Dump("tangential bspline/plane", plane, arch, u, uv, ip, found);

            Assert.IsTrue(found);
            Assert.AreEqual(0.0, expected | ip, exact);
        }

        /// <summary>
        /// A line which passes by the cylinder: there is no intersection, but the returned point must be the point
        /// of the closest approach, which is in the middle between the two closest points.
        /// </summary>
        [TestMethod]
        public void LineMissingCylinderReturnsClosestApproach()
        {
            CylindricalSurface cylinder = CylinderAroundZAxis(5);
            Line line = Line.TwoPoints(new GeoPoint(-10, 5.01, 2), new GeoPoint(10, 5.01, 2));

            ApproximatePosition(cylinder, line, new GeoPoint(0.1, 5.005, 2.05), GeoVector.NullVector, out double u, out GeoPoint2D uv);
            bool found = cylinder.RefineCurveIntersection(line, ref u, ref uv, out GeoPoint ip);
            Dump("line missing cylinder", cylinder, line, u, uv, ip, found);

            Assert.IsFalse(found); // 0.01 apart is more than Precision.eps
            Assert.AreEqual(0.0, new GeoPoint(0, 5.005, 2) | ip, exact);
            Assert.AreEqual(0.01, cylinder.PointAt(uv) | line.PointAt(u), exact); // the gap between the two
        }

        /// <summary>
        /// A line which lies inside a plane: every point of the line is a solution, the system is degenerate. The
        /// iteration must not run away, it must return a point which is on the line and on the plane.
        /// </summary>
        [TestMethod]
        public void LineInsidePlaneStaysWhereItIs()
        {
            PlaneSurface plane = new PlaneSurface(new Plane(GeoPoint.Origin, GeoVector.ZAxis));
            Line line = Line.TwoPoints(new GeoPoint(-5, -5, 0), new GeoPoint(5, 5, 0));

            double u = 0.4;
            GeoPoint2D uv = plane.PositionOf(new GeoPoint(-1.2, -0.8, 0)); // close to, but not exactly on the line
            bool found = plane.RefineCurveIntersection(line, ref u, ref uv, out GeoPoint ip);
            Dump("line inside plane", plane, line, u, uv, ip, found);

            Assert.IsTrue(found);
            Assert.AreEqual(0.0, plane.PointAt(uv) | line.PointAt(u), exact);
            Assert.AreEqual(0.0, (line as ICurve).DistanceTo(ip), exact);
            Assert.IsTrue(u >= 0.0 && u <= 1.0); // it did not run away along the line
        }

        /// <summary>
        /// Bad starting values must never make the result worse than what was provided.
        /// </summary>
        [TestMethod]
        public void PoorStartingValuesDoNotGetWorse()
        {
            SphericalSurface sphere = Sphere(new GeoPoint(1, 2, 3), 10);
            Ellipse circle = Circle(new Plane(new GeoPoint(0, 0, 4), GeoVector.ZAxis), new GeoPoint(0, 0, 4), 20);

            double u = 0.1;
            GeoPoint2D uv = sphere.PositionOf(circle.PointAt(0.13));
            double before = sphere.PointAt(uv) | circle.PointAt(u);
            bool found = sphere.RefineCurveIntersection(circle, ref u, ref uv, out GeoPoint ip);
            double after = sphere.PointAt(uv) | circle.PointAt(u);
            TestContext.WriteLine($"poor start: gap {before:E3} -> {after:E3}, found = {found}, point = {ip}");

            Assert.IsTrue(after <= before + exact); // the tolerance covers the numeric precision the contract allows for
        }
    }
}
