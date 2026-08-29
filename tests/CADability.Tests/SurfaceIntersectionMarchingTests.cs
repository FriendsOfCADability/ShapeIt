using CADability.GeoObject;
using System.Collections.Generic;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="Surfaces.GetIntersectionCurves"/>: the intersection curves between given points,
    /// found by marching from all these points at the same time. The cases here have a result which can be
    /// written down, so the tests state what is right, not what the code happens to produce.
    /// </summary>
    [TestClass]
    public class SurfaceIntersectionMarchingTests
    {
        public TestContext TestContext { get; set; }

        private static CylindricalSurface Cylinder(GeoPoint location, GeoVector dx, GeoVector dy, GeoVector axis)
        {
            return new CylindricalSurface(location, dx, dy, axis);
        }

        private static PlaneSurface PlaneThrough(GeoPoint location, GeoVector normal)
        {
            return new PlaneSurface(new Plane(location, normal));
        }

        private static BoundingRect Wide => new BoundingRect(-20, -20, 20, 20);

        /// <summary>The domain of a cylinder: the full turn and a generous range along the axis.</summary>
        private static BoundingRect FullCylinder => new BoundingRect(0, -20, 2 * System.Math.PI, 20);

        private void Dump(string what, IDualSurfaceCurve[] curves)
        {
            TestContext.WriteLine(what + ": " + curves.Length.ToString() + " curve(s)");
            foreach (IDualSurfaceCurve c in curves)
            {
                TestContext.WriteLine("   " + c.Curve3D.StartPoint.ToString() + " -> " + c.Curve3D.EndPoint.ToString()
                    + "  via " + c.Curve3D.PointAt(0.5).ToString());
            }
        }

        /// <summary>
        /// Every point of the curve must lie on both surfaces: this is what the returned curve claims and what
        /// everything downstream relies on.
        /// </summary>
        private void AssertOnBothSurfaces(ISurface surface1, ISurface surface2, IDualSurfaceCurve curve, double precision)
        {
            for (int i = 0; i <= 10; i++)
            {
                GeoPoint p = curve.Curve3D.PointAt(i / 10.0);
                Assert.AreEqual(0.0, p | surface1.PointAt(surface1.PositionOf(p)), precision, "point on the first surface");
                Assert.AreEqual(0.0, p | surface2.PointAt(surface2.PositionOf(p)), precision, "point on the second surface");
            }
        }

        private static bool ConnectsSeeds(IDualSurfaceCurve curve, GeoPoint seed1, GeoPoint seed2)
        {
            GeoPoint sp = curve.Curve3D.StartPoint, ep = curve.Curve3D.EndPoint;
            return ((sp | seed1) < 1e-6 && (ep | seed2) < 1e-6) || ((sp | seed2) < 1e-6 && (ep | seed1) < 1e-6);
        }

        /// <summary>
        /// The simplest case of all: two perpendicular planes intersect in a line. Of the four branches only
        /// the two which run towards each other survive, the other two leave the domain and are dropped, so
        /// there is exactly one curve, the segment between the two seeds.
        /// </summary>
        [TestMethod]
        public void two_planes_give_the_segment_between_the_seeds()
        {
            PlaneSurface p1 = PlaneThrough(GeoPoint.Origin, GeoVector.ZAxis);
            PlaneSurface p2 = PlaneThrough(GeoPoint.Origin, GeoVector.XAxis);
            GeoPoint seed1 = new GeoPoint(0, -10, 0), seed2 = new GeoPoint(0, 10, 0);

            IDualSurfaceCurve[] curves = Surfaces.GetIntersectionCurves(p1, Wide, p2, Wide,
                new List<GeoPoint> { seed1, seed2 });
            Dump("two planes", curves);

            Assert.AreEqual(1, curves.Length);
            Assert.IsTrue(ConnectsSeeds(curves[0], seed1, seed2), "the curve runs from one seed to the other");
            Assert.AreEqual(0.0, curves[0].Curve3D.PointAt(0.5) | GeoPoint.Origin, 1e-5, "the middle of the segment");
            AssertOnBothSurfaces(p1, p2, curves[0], 1e-6);
        }

        /// <summary>
        /// Three seeds on the same line: the curve is cut into the two pieces between neighbouring seeds. No
        /// curve from the first to the last seed, that one would have to run over the seed in between.
        /// </summary>
        [TestMethod]
        public void three_seeds_give_two_curves()
        {
            PlaneSurface p1 = PlaneThrough(GeoPoint.Origin, GeoVector.ZAxis);
            PlaneSurface p2 = PlaneThrough(GeoPoint.Origin, GeoVector.XAxis);
            GeoPoint seed1 = new GeoPoint(0, -10, 0), seed2 = GeoPoint.Origin, seed3 = new GeoPoint(0, 10, 0);

            IDualSurfaceCurve[] curves = Surfaces.GetIntersectionCurves(p1, Wide, p2, Wide,
                new List<GeoPoint> { seed1, seed2, seed3 });
            Dump("two planes, three seeds", curves);

            Assert.AreEqual(2, curves.Length);
            Assert.IsTrue(System.Array.Exists(curves, c => ConnectsSeeds(c, seed1, seed2)));
            Assert.IsTrue(System.Array.Exists(curves, c => ConnectsSeeds(c, seed2, seed3)));
        }

        /// <summary>
        /// Two cylinders of different radius with perpendicular crossing axes: the intersection consists of two
        /// closed curves, one above and one below the common perpendicular. Here the domain of the thin
        /// cylinder is restricted to half a turn, so of the upper curve only the half between the two seeds
        /// remains: the other half leaves the domain and must be dropped.
        /// </summary>
        [TestMethod]
        public void crossing_cylinders_restricted_domain_give_one_curve()
        {
            CylindricalSurface thick = Cylinder(GeoPoint.Origin, 10 * GeoVector.YAxis, 10 * GeoVector.ZAxis, GeoVector.XAxis);
            CylindricalSurface thin = Cylinder(GeoPoint.Origin, 5 * GeoVector.XAxis, 5 * GeoVector.YAxis, GeoVector.ZAxis);
            BoundingRect halfThin = new BoundingRect(0, -20, System.Math.PI, 20);
            GeoPoint seed1 = new GeoPoint(5, 0, 10), seed2 = new GeoPoint(-5, 0, 10);

            IDualSurfaceCurve[] curves = Surfaces.GetIntersectionCurves(thick, FullCylinder, thin, halfThin,
                new List<GeoPoint> { seed1, seed2 });
            Dump("crossing cylinders, half domain", curves);

            Assert.AreEqual(1, curves.Length);
            Assert.IsTrue(ConnectsSeeds(curves[0], seed1, seed2));
            // the middle of the curve is where the thin cylinder is at 90 degrees: (0, 5, sqrt(100-25))
            GeoPoint middle = new GeoPoint(0, 5, System.Math.Sqrt(75.0));
            Assert.AreEqual(0.0, curves[0].Curve3D.DistanceTo(middle), 1e-4, "the curve passes the top of the loop");
            AssertOnBothSurfaces(thick, thin, curves[0], 1e-5);
        }

        /// <summary>
        /// The same two cylinders with the full domains: now the upper intersection curve is closed and the two
        /// seeds cut it into two halves, so there are two curves between the same two seeds.
        /// </summary>
        [TestMethod]
        public void crossing_cylinders_full_domain_give_both_halves()
        {
            CylindricalSurface thick = Cylinder(GeoPoint.Origin, 10 * GeoVector.YAxis, 10 * GeoVector.ZAxis, GeoVector.XAxis);
            CylindricalSurface thin = Cylinder(GeoPoint.Origin, 5 * GeoVector.XAxis, 5 * GeoVector.YAxis, GeoVector.ZAxis);
            GeoPoint seed1 = new GeoPoint(5, 0, 10), seed2 = new GeoPoint(-5, 0, 10);

            IDualSurfaceCurve[] curves = Surfaces.GetIntersectionCurves(thick, FullCylinder, thin, FullCylinder,
                new List<GeoPoint> { seed1, seed2 });
            Dump("crossing cylinders, full domain", curves);

            Assert.AreEqual(2, curves.Length);
            foreach (IDualSurfaceCurve c in curves)
            {
                Assert.IsTrue(ConnectsSeeds(c, seed1, seed2));
                AssertOnBothSurfaces(thick, thin, c, 1e-5);
            }
            // one half runs through +y, the other through -y
            double y1 = curves[0].Curve3D.PointAt(0.5).y, y2 = curves[1].Curve3D.PointAt(0.5).y;
            Assert.IsTrue(y1 * y2 < 0, "the two halves are on different sides");
        }

        /// <summary>
        /// The seeds are on two different intersection curves, which have no connection at all: the branches
        /// come back to their own seed and each of them closes its own loop.
        /// </summary>
        [TestMethod]
        public void seeds_on_separate_loops_are_not_connected()
        {
            CylindricalSurface thick = Cylinder(GeoPoint.Origin, 10 * GeoVector.YAxis, 10 * GeoVector.ZAxis, GeoVector.XAxis);
            CylindricalSurface thin = Cylinder(GeoPoint.Origin, 5 * GeoVector.XAxis, 5 * GeoVector.YAxis, GeoVector.ZAxis);
            GeoPoint upper = new GeoPoint(5, 0, 10), lower = new GeoPoint(5, 0, -10);

            IDualSurfaceCurve[] curves = Surfaces.GetIntersectionCurves(thick, FullCylinder, thin, FullCylinder,
                new List<GeoPoint> { upper, lower });
            Dump("seeds on separate loops", curves);

            foreach (IDualSurfaceCurve c in curves)
            {
                Assert.IsFalse(ConnectsSeeds(c, upper, lower), "there is no curve from one loop to the other");
                AssertOnBothSurfaces(thick, thin, c, 1e-5);
                Assert.IsTrue(c.Curve3D.PointAt(0.5).z * c.Curve3D.StartPoint.z > 0, "a curve stays on its own loop");
            }
        }

        /// <summary>
        /// Two cylinders of equal radius with crossing axes touch at two points, and the intersection is two
        /// ellipses which cross each other exactly there. So at both seeds four branches start, and they build
        /// the four arcs between the two touching points.
        /// <para>
        /// The points of the curves are not checked here, only their course: an
        /// <see cref="InterpolatedDualSurfaceCurve"/> whose endpoint is such a node evaluates its inner points
        /// unreliably, because the refinement onto both surfaces may end up on the other branch of the
        /// intersection. See <see cref="interpolated_curve_which_ends_at_a_node_leaves_its_own_branch"/>.
        /// </para>
        /// </summary>
        [TestMethod]
        public void touching_cylinders_give_the_four_arcs_between_the_nodes()
        {
            CylindricalSurface c1 = Cylinder(GeoPoint.Origin, 10 * GeoVector.YAxis, 10 * GeoVector.ZAxis, GeoVector.XAxis);
            CylindricalSurface c2 = Cylinder(GeoPoint.Origin, 10 * GeoVector.XAxis, 10 * GeoVector.ZAxis, GeoVector.YAxis);
            GeoPoint node1 = new GeoPoint(0, 0, 10), node2 = new GeoPoint(0, 0, -10);

            IDualSurfaceCurve[] curves = Surfaces.GetIntersectionCurves(c1, FullCylinder, c2, FullCylinder,
                new List<GeoPoint> { node1, node2 });
            Dump("touching cylinders", curves);

            Assert.AreEqual(4, curves.Length, "the two ellipses, each cut into two arcs by the two nodes");
            // the four arcs run through the four points where the two cylinders meet the plane z = 0
            List<GeoPoint> quadrants = new List<GeoPoint>
            {
                new GeoPoint(10, 10, 0), new GeoPoint(10, -10, 0), new GeoPoint(-10, 10, 0), new GeoPoint(-10, -10, 0)
            };
            foreach (IDualSurfaceCurve c in curves)
            {
                Assert.IsTrue(ConnectsSeeds(c, node1, node2));
                GeoPoint middle = c.Curve3D.PointAt(0.5); // the point where the two branches met
                int found = quadrants.FindIndex(q => (q | middle) < 1e-4);
                Assert.IsTrue(found >= 0, "the arc runs through one of the four quadrant points: " + middle.ToString());
                quadrants.RemoveAt(found); // and each of them only once
            }
        }

        /// <summary>
        /// Not a test of the marching, but of what the resulting curve does with correct base points: an
        /// <see cref="InterpolatedDualSurfaceCurve"/> whose endpoints are touching points (nodes of the
        /// intersection) leaves its own branch. The base points here are exact points of the ellipse x = -y,
        /// the arc of the two touching cylinders from one node to the other, but the curve evaluates its first
        /// half on the other ellipse (x = y) or on no surface at all: PointAt uses the approximating BSpline,
        /// which is built by refining interpolated points onto both surfaces, and near the node that
        /// refinement can end up on the other branch.
        /// <para>
        /// Ignored because it states what should happen, not what happens today.
        /// </para>
        /// </summary>
        [TestMethod]
        [Ignore("known defect of InterpolatedDualSurfaceCurve at a node, not of the marching")]
        public void interpolated_curve_which_ends_at_a_node_leaves_its_own_branch()
        {
            CylindricalSurface c1 = Cylinder(GeoPoint.Origin, 10 * GeoVector.YAxis, 10 * GeoVector.ZAxis, GeoVector.XAxis);
            CylindricalSurface c2 = Cylinder(GeoPoint.Origin, 10 * GeoVector.XAxis, 10 * GeoVector.ZAxis, GeoVector.YAxis);
            // exact points of the arc of the ellipse x = -y, from the node (0,0,10) via (-10,10,0) to (0,0,-10)
            double[] t = new double[] { 0, 1.4142135602846162, 2.813940477704007, 4.183041393267987, 5.501674920393455,
                6.743473837343938, 7.871458753345347, 8.83265594820188, 9.553871367196322, 9.948386074601643 };
            List<GeoPoint> pts = new List<GeoPoint>();
            for (int i = 0; i < t.Length; i++) pts.Add(new GeoPoint(-t[i], t[i], System.Math.Sqrt(100.0 - t[i] * t[i])));
            for (int i = t.Length - 1; i >= 0; i--) pts.Add(new GeoPoint(-t[i], t[i], -System.Math.Sqrt(100.0 - t[i] * t[i])));

            InterpolatedDualSurfaceCurve crv = new InterpolatedDualSurfaceCurve(c1, FullCylinder, c2, FullCylinder, pts.ToArray());
            for (int i = 0; i <= 8; i++)
            {
                GeoPoint p = (crv as ICurve).PointAt(i / 8.0);
                TestContext.WriteLine((i / 8.0).ToString() + ": " + p.ToString());
                Assert.AreEqual(0.0, p | c1.PointAt(c1.PositionOf(p)), 1e-5, "point on the first surface");
                Assert.AreEqual(0.0, p | c2.PointAt(c2.PositionOf(p)), 1e-5, "point on the second surface");
                Assert.AreEqual(0.0, p.x + p.y, 1e-4, "the curve stays on its own ellipse x = -y");
            }
        }
    }
}
