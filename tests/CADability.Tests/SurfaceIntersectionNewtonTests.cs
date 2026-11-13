using CADability.GeoObject;
using System;
using System.Collections.Generic;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="SurfaceIntersectionNewton.PlaneAndSurfaces"/>: the point of a plane and two surfaces,
    /// compared with the Levenberg-Marquardt solver it replaces and checked against the equations it solves.
    /// </summary>
    [TestClass]
    public class SurfaceIntersectionNewtonTests
    {
        private static IEnumerable<(string what, ISurface s1, ISurface s2, GeoPoint near)> Pairs()
        {
            // two cylinders with crossing axes
            yield return ("cylinder r=5 about z and cylinder r=3 about x",
                new CylindricalSurface(GeoPoint.Origin, 5 * GeoVector.XAxis, 5 * GeoVector.YAxis, GeoVector.ZAxis),
                new CylindricalSurface(new GeoPoint(0, 0, 1), 3 * GeoVector.YAxis, 3 * GeoVector.ZAxis, GeoVector.XAxis),
                new GeoPoint(4.5, 2.2, 3.5));
            // a sphere and an inclined plane, far from the origin (precision relative to the coordinates)
            GeoPoint far = new GeoPoint(1000, -2000, 500);
            yield return ("sphere r=4 and an inclined plane, far from the origin",
                new SphericalSurface(far, 4 * GeoVector.XAxis, 4 * GeoVector.YAxis, 4 * GeoVector.ZAxis),
                new PlaneSurface(new Plane(far + new GeoVector(0, 0, 1.3), new GeoVector(0.3, 0.2, 1).Normalized)),
                far + new GeoVector(3.5, 0.5, 1.2));
            // a torus and a cylinder through its tube
            yield return ("torus R=10 r=3 and cylinder r=2 about x",
                new ToroidalSurface(GeoPoint.Origin, GeoVector.XAxis, GeoVector.YAxis, GeoVector.ZAxis, 10, 3),
                new CylindricalSurface(GeoPoint.Origin, 2 * GeoVector.YAxis, 2 * GeoVector.ZAxis, GeoVector.XAxis),
                new GeoPoint(11, 1.5, 1.3));
            // a cone and a cylinder
            yield return ("cone and cylinder",
                new ConicalSurface(new GeoPoint(0, 0, -10), GeoVector.XAxis, GeoVector.YAxis, GeoVector.ZAxis, 0.4),
                new CylindricalSurface(new GeoPoint(2, 0, 0), 2.5 * GeoVector.XAxis, 2.5 * GeoVector.ZAxis, -GeoVector.YAxis),
                new GeoPoint(4.0, 1.0, 0.5));
        }

        /// <summary>
        /// A point of the intersection curve near <paramref name="near"/> (by the LM solver, from a rough start) and
        /// the tangent of the curve there.
        /// </summary>
        private static void PointOnCurve(ISurface s1, ISurface s2, GeoPoint near, out GeoPoint2D uv1, out GeoPoint2D uv2,
            out GeoPoint p, out GeoVector tangent)
        {
            uv1 = s1.PositionOf(near);
            uv2 = s2.PositionOf(near);
            GeoVector guess = s1.GetNormal(uv1) ^ s2.GetNormal(uv2);
            PlaneSurface ps = new PlaneSurface(new Plane(near, guess));
            GeoPoint2D uvPlane = GeoPoint2D.Origin;
            p = near;
            Assert.IsTrue(BoxedSurfaceExtension.SurfacesIntersectionLM(ps, s1, s2, ref uvPlane, ref uv1, ref uv2, ref p),
                "no point of the intersection curve found to start with");
            tangent = (s1.GetNormal(uv1).Normalized ^ s2.GetNormal(uv2).Normalized).Normalized;
        }

        [TestMethod]
        public void finds_the_same_point_as_the_LM_solver_and_satisfies_the_equations()
        {
            foreach ((string what, ISurface s1, ISurface s2, GeoPoint near) in Pairs())
            {
                PointOnCurve(s1, s2, near, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p, out GeoVector tangent);
                double scale = 1.0 + Math.Max(Math.Abs(p.x), Math.Max(Math.Abs(p.y), Math.Abs(p.z)));
                foreach (double step in new double[] { 0.01, 0.1, 0.5 })
                {
                    // the next point of a marching step: the plane a step further along the tangent, starting from the
                    // parameters of the last point
                    Plane plane = new Plane(p + step * tangent, tangent);
                    GeoPoint2D n1 = uv1, n2 = uv2;
                    Assert.IsTrue(SurfaceIntersectionNewton.PlaneAndSurfaces(plane, s1, s2, ref n1, ref n2, out GeoPoint ip),
                        $"{what}, step {step}: Newton did not converge");
                    GeoPoint q1 = s1.PointAt(n1), q2 = s2.PointAt(n2);
                    Assert.IsTrue((q1 | q2) < 1e-9 * scale, $"{what}, step {step}: the points of the surfaces differ by {q1 | q2}");
                    Assert.IsTrue(Math.Abs(plane.Distance(ip)) < 1e-9 * scale, $"{what}, step {step}: not in the plane, {plane.Distance(ip)}");

                    GeoPoint2D l1 = uv1, l2 = uv2, uvPlane = GeoPoint2D.Origin;
                    GeoPoint lip = plane.Location;
                    Assert.IsTrue(SurfaceIntersectionSolvers.SurfacesIntersectionLM_Analytic9(new PlaneSurface(plane), s1, s2, ref uvPlane, ref l1, ref l2, ref lip),
                        $"{what}, step {step}: LM did not converge");
                    Assert.IsTrue((ip | lip) < 1e-7 * scale, $"{what}, step {step}: Newton {ip} and LM {lip} differ by {ip | lip}");
                }
            }
        }

        private static IEnumerable<(string what, ISurface s1, ISurface s2, ISurface s3, GeoPoint near)> Triples()
        {
            // x^2 + y^2 = 25 and y^2 + (z - 1)^2 = 9 meet at y = 2.2 in this point, the plane goes through it
            GeoPoint common = new GeoPoint(Math.Sqrt(25 - 2.2 * 2.2), 2.2, 1 + Math.Sqrt(9 - 2.2 * 2.2));
            yield return ("two crossing cylinders and an inclined plane",
                new CylindricalSurface(GeoPoint.Origin, 5 * GeoVector.XAxis, 5 * GeoVector.YAxis, GeoVector.ZAxis),
                new CylindricalSurface(new GeoPoint(0, 0, 1), 3 * GeoVector.YAxis, 3 * GeoVector.ZAxis, GeoVector.XAxis),
                new PlaneSurface(new Plane(common, new GeoVector(1, 0.3, 0.1).Normalized)),
                common + new GeoVector(0.1, -0.1, 0.05));
            GeoPoint far = new GeoPoint(1000, -2000, 500);
            yield return ("sphere and two planes, far from the origin",
                new SphericalSurface(far, 4 * GeoVector.XAxis, 4 * GeoVector.YAxis, 4 * GeoVector.ZAxis),
                new PlaneSurface(new Plane(far + new GeoVector(0, 0, 1.3), new GeoVector(0.3, 0.2, 1).Normalized)),
                new PlaneSurface(new Plane(far + new GeoVector(1, 0, 0), new GeoVector(1, -0.4, 0.2).Normalized)),
                far + new GeoVector(1.2, 3.4, 1.0));
            // torus (sqrt(x^2 + y^2) - 10)^2 + z^2 = 9 and cylinder y^2 + z^2 = 4 at z = 1.3, a sphere through that point
            double y = Math.Sqrt(4 - 1.3 * 1.3), rho = 10 + Math.Sqrt(9 - 1.3 * 1.3);
            GeoPoint onTorusAndCylinder = new GeoPoint(Math.Sqrt(rho * rho - y * y), y, 1.3);
            GeoPoint center = new GeoPoint(5, -3, 2);
            double r = center | onTorusAndCylinder;
            yield return ("torus, cylinder and sphere",
                new ToroidalSurface(GeoPoint.Origin, GeoVector.XAxis, GeoVector.YAxis, GeoVector.ZAxis, 10, 3),
                new CylindricalSurface(GeoPoint.Origin, 2 * GeoVector.YAxis, 2 * GeoVector.ZAxis, GeoVector.XAxis),
                new SphericalSurface(center, r * GeoVector.XAxis, r * GeoVector.YAxis, r * GeoVector.ZAxis),
                onTorusAndCylinder + new GeoVector(0.05, 0.1, -0.05));
        }

        [TestMethod]
        public void finds_the_point_of_three_surfaces_like_the_LM_solver()
        {
            foreach ((string what, ISurface s1, ISurface s2, ISurface s3, GeoPoint near) in Triples())
            {
                // the reference, by the LM solver from a rough start
                GeoPoint2D r1 = s1.PositionOf(near), r2 = s2.PositionOf(near), r3 = s3.PositionOf(near);
                GeoPoint reference = near;
                Assert.IsTrue(BoxedSurfaceExtension.SurfacesIntersectionLM(s1, s2, s3, ref r1, ref r2, ref r3, ref reference), $"{what}: LM found no point");
                double scale = 1.0 + Math.Max(Math.Abs(reference.x), Math.Max(Math.Abs(reference.y), Math.Abs(reference.z)));
                Assert.IsTrue((s1.PointAt(r1) | s2.PointAt(r2)) < 1e-7 * scale && (s2.PointAt(r2) | s3.PointAt(r3)) < 1e-7 * scale, $"{what}: LM reference not on all surfaces");
                foreach (double offset in new double[] { 0.01, 0.1, 0.4 })
                {
                    // start from the projections of a point beside the solution, as a caller refining a vertex does
                    GeoPoint start = reference + offset * new GeoVector(0.6, -0.5, 0.62);
                    GeoPoint2D uv1 = s1.PositionOf(start), uv2 = s2.PositionOf(start), uv3 = s3.PositionOf(start);
                    Assert.IsTrue(SurfaceIntersectionNewton.ThreeSurfaces(s1, s2, s3, ref uv1, ref uv2, ref uv3, out GeoPoint ip),
                        $"{what}, offset {offset}: Newton did not converge");
                    GeoPoint q1 = s1.PointAt(uv1), q2 = s2.PointAt(uv2), q3 = s3.PointAt(uv3);
                    Assert.IsTrue((q1 | q2) < 1e-9 * scale && (q2 | q3) < 1e-9 * scale && (q1 | q3) < 1e-9 * scale,
                        $"{what}, offset {offset}: the points of the surfaces differ ({q1 | q2}, {q2 | q3})");
                    Assert.IsTrue((ip | reference) < 1e-7 * scale, $"{what}, offset {offset}: Newton {ip} and LM {reference} differ by {ip | reference}");
                }
            }
        }

        [TestMethod]
        public void three_surfaces_through_a_common_line_give_up()
        {
            // three planes through the z axis: no single common point, the Jacobian is singular
            PlaneSurface p1 = new PlaneSurface(new Plane(GeoPoint.Origin, GeoVector.XAxis));
            PlaneSurface p2 = new PlaneSurface(new Plane(GeoPoint.Origin, GeoVector.YAxis));
            PlaneSurface p3 = new PlaneSurface(new Plane(GeoPoint.Origin, new GeoVector(1, 1, 0).Normalized));
            GeoPoint start = new GeoPoint(0.3, -0.2, 4);
            GeoPoint2D uv1 = p1.PositionOf(start), uv2 = p2.PositionOf(start), uv3 = p3.PositionOf(start);
            GeoPoint2D b1 = uv1, b2 = uv2, b3 = uv3;
            Assert.IsFalse(SurfaceIntersectionNewton.ThreeSurfaces(p1, p2, p3, ref uv1, ref uv2, ref uv3, out _));
            Assert.AreEqual(b1, uv1, "the start values must be unchanged on failure");
            Assert.AreEqual(b2, uv2, "the start values must be unchanged on failure");
            Assert.AreEqual(b3, uv3, "the start values must be unchanged on failure");
        }

        private static IEnumerable<(string what, ISurface surface, ICurve curve)> CurvesThroughSurfaces()
        {
            yield return ("oblique line through a cylinder",
                new CylindricalSurface(GeoPoint.Origin, 5 * GeoVector.XAxis, 5 * GeoVector.YAxis, GeoVector.ZAxis),
                Line.TwoPoints(new GeoPoint(1, 0.5, -2), new GeoPoint(9, 3, 4)));
            GeoPoint far = new GeoPoint(1000, -2000, 500);
            BSpline spline = BSpline.Construct();
            spline.ThroughPoints(new GeoPoint[] { far + new GeoVector(-1, 0.5, 0.2), far + new GeoVector(1.5, 1, 1), far + new GeoVector(4, 2, 1.5), far + new GeoVector(7, 2.5, 0.5) }, 3, false);
            yield return ("BSpline out of a sphere, far from the origin",
                new SphericalSurface(far, 4 * GeoVector.XAxis, 4 * GeoVector.YAxis, 4 * GeoVector.ZAxis), spline);
            Ellipse arc = Ellipse.Construct();
            arc.SetArcPlaneCenterRadiusAngles(new Plane(new GeoPoint(0, 0, 0.5), GeoVector.XAxis, GeoVector.ZAxis), new GeoPoint(14, 0, 0.5), 5, 1.5, 2.0); // crosses the tube at about 2.6
            yield return ("arc through a torus",
                new ToroidalSurface(GeoPoint.Origin, GeoVector.XAxis, GeoVector.YAxis, GeoVector.ZAxis, 10, 3), arc);
            Polyline polyline = Polyline.Construct();
            polyline.SetPoints(new GeoPoint[] { new GeoPoint(0, 0, -3), new GeoPoint(1, 1, -1), new GeoPoint(2, 1, 2), new GeoPoint(4, 0, 3) }, false);
            yield return ("Polyline through an inclined plane (no analytic derivative)",
                new PlaneSurface(new Plane(new GeoPoint(0, 0, 0.5), new GeoVector(0.2, 0.1, 1).Normalized)), polyline);
        }

        [TestMethod]
        public void finds_the_intersection_of_a_curve_and_a_surface_from_far_away()
        {
            foreach ((string what, ISurface surface, ICurve curve) in CurvesThroughSurfaces())
            {
                foreach (double startPosition in new double[] { 0.1, 0.5, 0.9 })
                {
                    // a finder starts in the middle of a uv patch and somewhere on the curve
                    GeoPoint2D uv = surface.PositionOf(curve.PointAt(1.0 - startPosition));
                    double t = startPosition;
                    Assert.IsTrue(SurfaceIntersectionNewton.CurveAndSurface(surface, curve, ref uv, ref t, out GeoPoint ip),
                        $"{what}, start {startPosition}: did not settle");
                    GeoPoint s = surface.PointAt(uv), c = curve.PointAt(t);
                    double scale = 1.0 + Math.Max(Math.Abs(s.x), Math.Max(Math.Abs(s.y), Math.Abs(s.z)));
                    Assert.IsTrue((s | c) < 1e-9 * scale, $"{what}, start {startPosition}: no intersection, distance {s | c}");
                    Assert.IsTrue((ip | s) < 1e-9 * scale, $"{what}, start {startPosition}: ip is not the intersection");
                }
            }
        }

        [TestMethod]
        public void a_curve_beside_the_surface_ends_in_the_point_of_minimal_distance()
        {
            // a line passing a sphere of radius 4 at the distance 6 from its center: the gap is 2
            SphericalSurface sphere = new SphericalSurface(GeoPoint.Origin, 4 * GeoVector.XAxis, 4 * GeoVector.YAxis, 4 * GeoVector.ZAxis);
            Line line = Line.TwoPoints(new GeoPoint(-10, 6, 0), new GeoPoint(10, 6, 0));
            GeoPoint2D uv = sphere.PositionOf(new GeoPoint(3, 2, 1));
            double t = 0.2;
            GeoPoint2D before = uv;
            bool settled = SurfaceIntersectionNewton.CurveAndSurface(sphere, line, ref uv, ref t, out GeoPoint _);
            if (settled)
            {
                Assert.AreEqual(2.0, sphere.PointAt(uv) | line.PointAt(t), 1e-6, "the point of minimal distance");
                Assert.AreEqual(0.5, t, 1e-6, "the minimal distance is in the middle of the line");
            }
            else Assert.AreEqual(before, uv, "the start values must be unchanged on failure");
        }

        [TestMethod]
        public void gives_up_where_the_surfaces_touch()
        {
            // a plane touching a cylinder along a line: the Jacobian is singular there
            CylindricalSurface cylinder = new CylindricalSurface(GeoPoint.Origin, 5 * GeoVector.YAxis, 5 * GeoVector.ZAxis, GeoVector.XAxis);
            PlaneSurface touching = new PlaneSurface(new Plane(new GeoPoint(0, 0, 5), GeoVector.ZAxis));
            Plane plane = new Plane(new GeoPoint(1, 0, 5), GeoVector.XAxis);
            GeoPoint start = new GeoPoint(1, 0.3, 5.0);
            GeoPoint2D uv1 = cylinder.PositionOf(start), uv2 = touching.PositionOf(start + new GeoVector(0, 0.2, 0));
            GeoPoint2D before1 = uv1, before2 = uv2;
            bool found = SurfaceIntersectionNewton.PlaneAndSurfaces(plane, cylinder, touching, ref uv1, ref uv2, out GeoPoint ip);
            if (found)
            {   // if it does converge (the singularity is reached only in the limit), the point must be right
                Assert.IsTrue((cylinder.PointAt(uv1) | touching.PointAt(uv2)) < 1e-9, "a wrong touching point");
            }
            else
            {
                Assert.AreEqual(before1, uv1, "the start values must be unchanged on failure");
                Assert.AreEqual(before2, uv2, "the start values must be unchanged on failure");
            }
        }
    }
}
