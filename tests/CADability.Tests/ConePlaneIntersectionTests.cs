using CADability.Curve2D;
using CADability.GeoObject;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="ConicalSurface.GetPlaneIntersection(PlaneSurface, double, double, double, double, double)"/>.
    /// Two independent properties are checked for a set of cones and planes covering all types of conic sections:
    /// <list type="bullet">
    /// <item>soundness: every point of every returned curve lies on the cone, on the plane and inside the given bounds,
    /// and the two 2d curves of the <see cref="IDualSurfaceCurve"/> describe the same curve as the 3d curve</item>
    /// <item>completeness: every intersection point found by an independent (and trivial) calculation is covered by one
    /// of the returned curves. The independent calculation uses the fact that the surface is linear in v, so the
    /// intersection of the line at a fixed u with the plane can be calculated directly.</item>
    /// </list>
    /// </summary>
    [TestClass]
    public class ConePlaneIntersectionTests
    {
        public TestContext TestContext { get; set; }

        private const double Tolerance = 1e-5; // the objects have a size of about 100

        /// <summary>
        /// A cone with the apex at (0,0,0), the axis in z direction and a semi angle of 30 degrees, used for most tests.
        /// </summary>
        private static ConicalSurface StandardCone()
        {
            return new ConicalSurface(GeoPoint.Origin, GeoVector.XAxis, GeoVector.YAxis, GeoVector.ZAxis, Math.PI / 6.0);
        }

        /// <summary>
        /// The same cone, but arbitrarily placed in space, so no coordinate direction is preferred.
        /// </summary>
        private static ConicalSurface TiltedCone()
        {
            ModOp m = ModOp.Rotate(new GeoPoint(1, 2, 3), new GeoVector(1, 2, 3), new SweepAngle(0.7)) * ModOp.Translate(5, -3, 2);
            return StandardCone().GetModified(m) as ConicalSurface;
        }

        [TestMethod]
        public void CircleWhenPlaneIsPerpendicularToAxis()
        {
            ConicalSurface cone = StandardCone();
            PlaneSurface pl = new PlaneSurface(new Plane(new GeoPoint(0, 0, 50), GeoVector.ZAxis));
            IDualSurfaceCurve[] dsc = Check(cone, pl, 0.0, 2 * Math.PI, 10.0, 100.0);
            Assert.AreEqual(1, dsc.Length);
            Ellipse elli = dsc[0].Curve3D as Ellipse;
            Assert.IsNotNull(elli, "the intersection with a perpendicular plane must be a circle");
            Assert.IsTrue(elli.IsCircle);
            // the radius at v == 50 is 50*tan(30°)
            Assert.AreEqual(50.0 * Math.Tan(Math.PI / 6.0), elli.Radius, Tolerance);
            Assert.AreEqual(Math.PI * 2.0, Math.Abs(elli.SweepParameter), Tolerance, "a full cone must yield a full circle");
        }

        [TestMethod]
        public void CircleClippedToUBounds()
        {
            ConicalSurface cone = StandardCone();
            PlaneSurface pl = new PlaneSurface(new Plane(new GeoPoint(0, 0, 50), GeoVector.ZAxis));
            IDualSurfaceCurve[] dsc = Check(cone, pl, 0.5, 2.0, 10.0, 100.0);
            Assert.AreEqual(1, dsc.Length);
            Assert.AreEqual(1.5, Math.Abs((dsc[0].Curve3D as Ellipse).SweepParameter), Tolerance);
        }

        [TestMethod]
        public void EllipseWhenPlaneIsSlightlyTilted()
        {
            ConicalSurface cone = StandardCone();
            PlaneSurface pl = new PlaneSurface(new Plane(new GeoPoint(0, 0, 50), new GeoVector(0.2, 0.0, 1.0)));
            IDualSurfaceCurve[] dsc = Check(cone, pl, 0.0, 2 * Math.PI, 10.0, 100.0);
            Assert.AreEqual(1, dsc.Length);
            Ellipse elli = dsc[0].Curve3D as Ellipse;
            Assert.IsNotNull(elli, "a tilted plane must yield an ellipse");
            Assert.IsFalse(elli.IsCircle);
        }

        [TestMethod]
        public void ParabolaWhenPlaneIsParallelToSurfaceLine()
        {
            ConicalSurface cone = StandardCone();
            // the surface lines have an angle of 30 degrees to the axis, a plane with this inclination yields a parabola
            double semi = Math.PI / 6.0;
            GeoVector normal = new GeoVector(Math.Cos(semi), 0.0, Math.Sin(semi));
            PlaneSurface pl = new PlaneSurface(new Plane(new GeoPoint(10, 0, 50), normal));
            Check(cone, pl, 0.0, 2 * Math.PI, 0.0, 100.0);
        }

        [TestMethod]
        public void HyperbolaWhenPlaneIsParallelToAxis()
        {
            ConicalSurface cone = StandardCone();
            PlaneSurface pl = new PlaneSurface(new Plane(new GeoPoint(10, 0, 0), GeoVector.XAxis));
            IDualSurfaceCurve[] dsc = Check(cone, pl, 0.0, 2 * Math.PI, 0.0, 100.0);
            Assert.AreEqual(1, dsc.Length, "a plane parallel to the axis cuts one branch of a hyperbola");
            Assert.IsTrue(dsc[0].Curve2D1.StartPoint.x > 2 * Math.PI || dsc[0].Curve2D1.EndPoint.x > 2 * Math.PI, "the curve crosses the periodic seam and must not be split there");
        }

        [TestMethod]
        public void TwoLinesWhenPlaneContainsApex()
        {
            ConicalSurface cone = StandardCone();
            PlaneSurface pl = new PlaneSurface(new Plane(GeoPoint.Origin, GeoVector.XAxis));
            IDualSurfaceCurve[] dsc = Check(cone, pl, 0.0, 2 * Math.PI, 0.0, 100.0);
            Assert.AreEqual(2, dsc.Length, "a plane through the apex cuts two surface lines");
            foreach (IDualSurfaceCurve c in dsc) Assert.IsInstanceOfType(c.Curve3D, typeof(Line));
        }

        [TestMethod]
        public void SingleLineWhenPlaneTouchesCone()
        {
            ConicalSurface cone = StandardCone();
            double semi = Math.PI / 6.0;
            // a plane through the apex containing the surface line at u == 0
            GeoVector normal = new GeoVector(Math.Cos(semi), 0.0, -Math.Sin(semi));
            PlaneSurface pl = new PlaneSurface(new Plane(GeoPoint.Origin, normal));
            IDualSurfaceCurve[] dsc = Check(cone, pl, 0.0, 2 * Math.PI, 0.0, 100.0);
            Assert.AreEqual(1, dsc.Length, "a tangential plane through the apex touches a single surface line");
        }

        [TestMethod]
        public void NothingWhenPlaneMissesCone()
        {
            ConicalSurface cone = StandardCone();
            // a plane which only touches the apex, but from outside the cone
            PlaneSurface pl = new PlaneSurface(new Plane(GeoPoint.Origin, GeoVector.ZAxis));
            Assert.AreEqual(0, Check(cone, pl, 0.0, 2 * Math.PI, 0.0, 100.0).Length);
            // a plane above the used part of the cone
            pl = new PlaneSurface(new Plane(new GeoPoint(0, 0, 200), GeoVector.ZAxis));
            Assert.AreEqual(0, Check(cone, pl, 0.0, 2 * Math.PI, 10.0, 100.0).Length);
        }

        [TestMethod]
        public void HyperbolaSplitByVBounds()
        {
            ConicalSurface cone = StandardCone();
            // a plane parallel to the axis, which cuts the far side of the cone: the two ends of the hyperbola leave
            // the vmax bound, the middle part is below vmin, so two separate curves must be returned
            PlaneSurface pl = new PlaneSurface(new Plane(new GeoPoint(10, 0, 0), GeoVector.XAxis));
            IDualSurfaceCurve[] dsc = Check(cone, pl, 0.0, 2 * Math.PI, 30.0, 60.0);
            Assert.AreEqual(2, dsc.Length, "the hyperbola enters and leaves the v bounds twice");
        }

        [TestMethod]
        public void EllipseSplitByVBounds()
        {
            ConicalSurface cone = StandardCone();
            // a tilted plane whose intersection ellipse partially lies above vmax
            PlaneSurface pl = new PlaneSurface(new Plane(new GeoPoint(0, 0, 50), new GeoVector(0.5, 0.0, 1.0)));
            IDualSurfaceCurve[] dsc = Check(cone, pl, 0.0, 2 * Math.PI, 10.0, 55.0);
            Assert.AreEqual(1, dsc.Length, "only the part of the ellipse below vmax is expected");
            Assert.IsInstanceOfType(dsc[0].Curve3D, typeof(Ellipse));
        }

        [TestMethod]
        public void AllConicSectionsOnATiltedCone()
        {
            // a systematic test: many planes with all possible inclinations and positions against an arbitrarily
            // placed cone. Only the general properties (soundness and completeness) are checked here.
            ConicalSurface cone = TiltedCone();
            int count = 0;
            foreach (PlaneSurface pl in ManyPlanes(cone)) count += Check(cone, pl, 0.0, 2 * Math.PI, 0.0, 100.0).Length;
            TestContext?.WriteLine($"{count} intersection curves checked");
            Assert.IsTrue(count > 100, "the test configuration should produce many intersection curves");
        }

        [TestMethod]
        public void PartialConeWithAllInclinations()
        {
            // the same, but only a part of the cone is used, so the curves must be clipped in u and v direction
            ConicalSurface cone = TiltedCone();
            int count = 0;
            foreach (PlaneSurface pl in ManyPlanes(cone)) count += Check(cone, pl, 0.3, 4.0, 20.0, 80.0).Length;
            TestContext?.WriteLine($"{count} intersection curves checked");
            Assert.IsTrue(count > 50, "the test configuration should produce many intersection curves");
        }

        /// <summary>
        /// A set of planes with all inclinations against the axis of the cone and at various distances from it.
        /// </summary>
        private static IEnumerable<PlaneSurface> ManyPlanes(ConicalSurface cone)
        {
            GeoVector axis = cone.Axis.Normalized, dirx = cone.XAxis.Normalized;
            for (int i = 0; i <= 12; i++)
            {   // the inclination of the plane against the axis, from perpendicular to parallel and beyond
                double inclination = i / 12.0 * Math.PI;
                GeoVector normal = Math.Cos(inclination) * axis + Math.Sin(inclination) * dirx;
                for (int j = 0; j <= 4; j++)
                {   // the distance of the plane from the apex, in the direction of the axis
                    for (int k = 0; k <= 2; k++)
                    {   // and perpendicular to the axis
                        GeoPoint location = cone.Location + (j * 25.0) * axis + (k * 20.0) * dirx;
                        yield return new PlaneSurface(new Plane(location, normal));
                    }
                }
            }
        }

        /// <summary>
        /// Calculates the intersection and verifies soundness and completeness of the result.
        /// </summary>
        private IDualSurfaceCurve[] Check(ConicalSurface cone, PlaneSurface pl, double umin, double umax, double vmin, double vmax)
        {
            IDualSurfaceCurve[] dsc = cone.GetPlaneIntersection(pl, umin, umax, vmin, vmax, Tolerance);
            Assert.IsNotNull(dsc);
            BoundingRect domain = new BoundingRect(umin, vmin, umax, vmax);
            double sizeTolerance = Tolerance * 100; // the bounds may be exceeded by rounding errors only
            foreach (IDualSurfaceCurve c in dsc)
            {
                ICurve curve3d = c.Curve3D;
                Assert.IsNotNull(curve3d);
                ICurve2D onCone = c.Surface1 == cone ? c.Curve2D1 : c.Curve2D2;
                ICurve2D onPlane = c.Surface1 == cone ? c.Curve2D2 : c.Curve2D1;
                Assert.IsTrue(curve3d.Length > Tolerance, "degenerate intersection curve");
                for (int i = 0; i <= 20; i++)
                {
                    double t = i / 20.0;
                    GeoPoint p = curve3d.PointAt(t);
                    Assert.AreEqual(0.0, pl.Plane.Distance(p), Tolerance, "the intersection curve is not on the plane");
                    GeoPoint2D uv = cone.PositionOf(p);
                    Assert.AreEqual(0.0, cone.PointAt(uv) | p, Tolerance, "the intersection curve is not on the cone");
                    SurfaceHelper.AdjustPeriodic(cone, domain, ref uv);
                    Assert.IsTrue(uv.x >= umin - sizeTolerance && uv.x <= umax + sizeTolerance, $"u == {uv.x} is outside the bounds {umin}...{umax}");
                    Assert.IsTrue(uv.y >= vmin - sizeTolerance && uv.y <= vmax + sizeTolerance, $"v == {uv.y} is outside the bounds {vmin}...{vmax}");
                    // the 2d curves must describe the same curve as the 3d curve. The curve on the cone is a
                    // ProjectedCurve, which internally uses an approximation with a precision of length*1e-5 in the
                    // parameter space, so the deviation in 3d may be that much times the length of the derivatives.
                    double coneTolerance = sizeTolerance + curve3d.Length * 1e-5 * (cone.UDirection(uv).Length + cone.VDirection(uv).Length);
                    Assert.IsTrue(curve3d.DistanceTo(cone.PointAt(onCone.PointAt(t))) < coneTolerance, "the 2d curve on the cone deviates from the 3d curve");
                    Assert.IsTrue(curve3d.DistanceTo(pl.PointAt(onPlane.PointAt(t))) < sizeTolerance, "the 2d curve on the plane deviates from the 3d curve");
                }
                // the endpoints of the 2d curves must correspond to the endpoints of the 3d curve
                Assert.AreEqual(0.0, cone.PointAt(onCone.StartPoint) | curve3d.StartPoint, sizeTolerance);
                Assert.AreEqual(0.0, cone.PointAt(onCone.EndPoint) | curve3d.EndPoint, sizeTolerance);
                Assert.AreEqual(0.0, pl.PointAt(onPlane.StartPoint) | curve3d.StartPoint, sizeTolerance);
                Assert.AreEqual(0.0, pl.PointAt(onPlane.EndPoint) | curve3d.EndPoint, sizeTolerance);
            }
            CheckCompleteness(cone, pl, umin, umax, vmin, vmax, dsc);
            return dsc;
        }

        /// <summary>
        /// Independent calculation of intersection points: the surface is linear in v, so for a fixed u the intersection
        /// of the surface line with the plane can be calculated directly. Every such point must be covered by one of the
        /// returned curves.
        /// </summary>
        private void CheckCompleteness(ConicalSurface cone, PlaneSurface pl, double umin, double umax, double vmin, double vmax, IDualSurfaceCurve[] dsc)
        {
            Plane plane = pl.Plane;
            int missing = 0;
            for (int i = 0; i <= 200; i++)
            {
                double u = umin + i / 200.0 * (umax - umin);
                double d1 = plane.Distance(cone.PointAt(new GeoPoint2D(u, vmin)));
                double d2 = plane.Distance(cone.PointAt(new GeoPoint2D(u, vmax)));
                if (d1 == d2) continue; // the surface line is parallel to the plane
                double v = vmin + d1 / (d1 - d2) * (vmax - vmin);
                if (v < vmin || v > vmax) continue; // the intersection is outside the used part of the cone
                // stay away from the bounds, there the point may be missed by a curve which just touches the bounds
                if (Math.Min(v - vmin, vmax - v) < 1e-3 * (vmax - vmin)) continue;
                GeoPoint p = cone.PointAt(new GeoPoint2D(u, v));
                double dist = double.MaxValue;
                foreach (IDualSurfaceCurve c in dsc) dist = Math.Min(dist, c.Curve3D.DistanceTo(p));
                if (dist > 1e-4) ++missing;
            }
            Assert.AreEqual(0, missing, "there are intersection points which are not covered by the returned curves");
        }
    }
}
