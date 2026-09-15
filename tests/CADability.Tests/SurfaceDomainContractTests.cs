using System;
using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// The contract the domain unification rests on: on a periodic surface,
    /// <see cref="ISurface.PositionOf(GeoPoint)"/> must return the uv value in the period that
    /// <see cref="ISurface.Domain"/> selects, not in whatever period the parametrisation happens to prefer.
    /// <para>
    /// Every test here sets a domain that is deliberately a whole period away from where PositionOf would
    /// naturally land and then asks for a point of that domain back. A surface that ignores its domain
    /// returns the unshifted value and fails. This is what makes the hundreds of SurfaceHelper.AdjustPeriodic
    /// calls scattered over the code removable - or, where a test here is red, not yet removable.
    /// </para>
    /// <para>
    /// The 3d point is checked as well, because the adjustment must only change which period the uv value
    /// lies in, never which point of space it denotes.
    /// </para>
    /// </summary>
    [TestClass]
    public class SurfaceDomainContractTests
    {
        public TestContext TestContext { get; set; }

        /// <summary>
        /// Moves <paramref name="patch"/> by whole periods, makes that the domain of the surface and checks
        /// that PositionOf follows it for a grid of points inside.
        /// </summary>
        private static void AssertPositionOfHonoursDomain(ISurface surface, BoundingRect patch,
                                                          int uPeriods, int vPeriods, double pointTolerance = 1e-5)
        {
            Assert.IsTrue(surface.IsUPeriodic || surface.IsVPeriodic,
                          "the surface is not periodic at all, the test would be vacuous");

            BoundingRect shifted = patch;
            shifted.Move(new GeoVector2D(uPeriods * (surface.IsUPeriodic ? surface.UPeriod : 0.0),
                                         vPeriods * (surface.IsVPeriodic ? surface.VPeriod : 0.0)));
            Assert.AreNotEqual(0, uPeriods + vPeriods, "nothing was shifted, the test would be vacuous");

            surface.Domain = shifted;

            double tolU = Math.Max(Math.Abs(shifted.Width), 1.0) * 1e-6;
            double tolV = Math.Max(Math.Abs(shifted.Height), 1.0) * 1e-6;

            for (int i = 1; i <= 3; i++)
            {
                for (int j = 1; j <= 3; j++)
                {
                    GeoPoint2D uv = new GeoPoint2D(shifted.Left + i * shifted.Width / 4.0,
                                                   shifted.Bottom + j * shifted.Height / 4.0);
                    GeoPoint p = surface.PointAt(uv);
                    GeoPoint2D back = surface.PositionOf(p);

                    // the adjustment may only move the uv value between periods, never off the surface point
                    Assert.IsTrue((surface.PointAt(back) | p) < pointTolerance,
                                  $"PositionOf lost the point at {uv}: PointAt(PositionOf(p)) is "
                                  + $"{surface.PointAt(back)} instead of {p}");

                    // the contract itself
                    bool inside = back.x >= shifted.Left - tolU && back.x <= shifted.Right + tolU
                               && back.y >= shifted.Bottom - tolV && back.y <= shifted.Top + tolV;
                    Assert.IsTrue(inside,
                                  $"PositionOf ignored the domain: asked for {uv} inside "
                                  + $"[{shifted.Left},{shifted.Bottom} .. {shifted.Right},{shifted.Top}], got {back}");
                }
            }
        }

        // ------------------------------------------------------------------ surfaces that already comply --

        [TestMethod]
        public void cylinder_position_of_stays_in_the_domain()
        {
            CylindricalSurface cylinder = new CylindricalSurface(GeoPoint.Origin, 2.0 * GeoVector.XAxis,
                                                                 2.0 * GeoVector.YAxis, GeoVector.ZAxis);
            AssertPositionOfHonoursDomain(cylinder, new BoundingRect(0.3, 0.0, 1.3, 4.0), 1, 0);
        }

        [TestMethod]
        public void sphere_position_of_stays_in_the_domain()
        {
            SphericalSurface sphere = new SphericalSurface(GeoPoint.Origin, 3.0 * GeoVector.XAxis,
                                                           3.0 * GeoVector.YAxis, 3.0 * GeoVector.ZAxis);
            AssertPositionOfHonoursDomain(sphere, new BoundingRect(0.4, -0.4, 1.2, 0.4), 1, 0);
        }

        [TestMethod]
        public void cone_position_of_stays_in_the_domain()
        {
            ConicalSurface cone = new ConicalSurface(GeoPoint.Origin, GeoVector.XAxis, GeoVector.YAxis,
                                                     GeoVector.ZAxis, 0.4);
            AssertPositionOfHonoursDomain(cone, new BoundingRect(0.3, 1.0, 1.1, 3.0), 1, 0);
        }

        [TestMethod]
        public void torus_position_of_stays_in_the_domain()
        {
            ToroidalSurface torus = new ToroidalSurface(GeoPoint.Origin, GeoVector.XAxis, GeoVector.YAxis,
                                                        GeoVector.ZAxis, 5.0, 1.0);
            AssertPositionOfHonoursDomain(torus, new BoundingRect(0.3, 0.2, 1.2, 1.0), 1, 1);
        }

        // ------------------------------------------------------- surfaces the contract does not cover yet --

        [TestMethod]
        public void surface_of_revolution_position_of_stays_in_the_domain()
        {
            // a BSpline, not a line or an arc: those would be turned into a cylinder, cone or torus and the
            // test would silently be about a different class
            AssertPositionOfHonoursDomain(RevolutionOfBSpline(), new BoundingRect(0.3, 0.2, 1.2, 0.8), 1, 0, 1e-4);
        }

        [TestMethod]
        public void surface_of_linear_extrusion_position_of_stays_in_the_domain()
        {
            Ellipse circle = Ellipse.Construct();
            circle.SetCirclePlaneCenterRadius(Plane.XYPlane, GeoPoint.Origin, 2.0);
            SurfaceOfLinearExtrusion extrusion = new SurfaceOfLinearExtrusion(circle, 4.0 * GeoVector.ZAxis, 0.0, 1.0);
            AssertPositionOfHonoursDomain(extrusion, new BoundingRect(0.1, 0.1, 0.6, 0.9), 1, 0);
        }

        [TestMethod]
        public void ruled_surface_position_of_stays_in_the_domain()
        {
            Ellipse bottom = Ellipse.Construct();
            bottom.SetCirclePlaneCenterRadius(Plane.XYPlane, GeoPoint.Origin, 2.0);
            Ellipse top = Ellipse.Construct();
            top.SetCirclePlaneCenterRadius(Plane.XYPlane, new GeoPoint(0.0, 0.0, 3.0), 3.0);
            RuledSurface ruled = new RuledSurface(bottom, top);
            AssertPositionOfHonoursDomain(ruled, new BoundingRect(0.1, 0.2, 0.6, 0.8), 1, 0);
        }

        // GeneralSweptCurve is not tested here: modOpAt casts the spine to IOrientation unconditionally, so
        // the class can only be built on an InterpolatedDualSurfaceCurve, which is a disproportionate amount
        // of setup for this test. It needs no own case either - like RuledSurface it does not override
        // PositionOf, so the ruled surface above already covers that path.

        [TestMethod]
        public void offset_surface_position_of_stays_in_the_domain()
        {
            CylindricalSurface cylinder = new CylindricalSurface(GeoPoint.Origin, 2.0 * GeoVector.XAxis,
                                                                 2.0 * GeoVector.YAxis, GeoVector.ZAxis);
            OffsetSurface offset = new OffsetSurface(cylinder, 0.5);
            AssertPositionOfHonoursDomain(offset, new BoundingRect(0.3, 0.0, 1.3, 4.0), 1, 0);
        }

        // ------------------------------------------------------------------------------------- helpers --

        /// <summary>
        /// A surface of revolution whose generatrix is a BSpline in a plane through the axis, so it cannot be
        /// simplified into one of the analytic surfaces.
        /// </summary>
        private static SurfaceOfRevolution RevolutionOfBSpline()
        {
            GeoPoint[] points =
            {
                new GeoPoint(2.0, 0.0, 0.0),
                new GeoPoint(3.0, 0.0, 1.0),
                new GeoPoint(2.4, 0.0, 2.0),
                new GeoPoint(3.6, 0.0, 3.0),
                new GeoPoint(2.8, 0.0, 4.0)
            };
            BSpline generatrix = BSpline.Construct();
            Assert.IsTrue(generatrix.ThroughPoints(points, 3, false), "could not build the generatrix");
            return new SurfaceOfRevolution(generatrix, GeoPoint.Origin, GeoVector.ZAxis);
        }
    }
}
