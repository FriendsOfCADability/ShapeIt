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
