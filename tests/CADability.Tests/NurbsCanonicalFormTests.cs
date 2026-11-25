using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// NurbsSurface.GetCanonicalForm recognizes NURBS surfaces, which describe a plane, cylinder, cone, sphere or torus, and
    /// returns the analytic surface with the same orientation. The surfaces of revolution are exact rational NURBS: a
    /// profile in the xz-plane rotated around the z-axis.
    /// </summary>
    [TestClass]
    public class NurbsCanonicalFormTests
    {
        private static readonly double w = Math.Sqrt(0.5);

        /// <summary>
        /// The surface of revolution of the rational profile (x, z, weight) of degree <paramref name="vDegree"/>.
        /// u runs around the z-axis (a full circle of 4 quarter arcs), v along the profile.
        /// </summary>
        private static NurbsSurface Revolution(double[] x, double[] z, double[] wv, int vDegree, double[] vKnots, int[] vMults)
        {
            double[] cx = { 1, 1, 0, -1, -1, -1, 0, 1, 1 };
            double[] cy = { 0, 1, 1, 1, 0, -1, -1, -1, 0 };
            double[] wu = { 1, w, 1, w, 1, w, 1, w, 1 };
            GeoPoint[,] poles = new GeoPoint[9, x.Length];
            double[,] weights = new double[9, x.Length];
            for (int i = 0; i < 9; i++)
                for (int j = 0; j < x.Length; j++)
                {
                    poles[i, j] = new GeoPoint(cx[i] * x[j], cy[i] * x[j], z[j]);
                    weights[i, j] = wu[i] * wv[j];
                }
            return new NurbsSurface(poles, weights, new double[] { 0, 1, 2, 3, 4 }, vKnots, new int[] { 3, 2, 2, 2, 3 }, vMults, 2, vDegree, true, false);
        }

        private static NurbsSurface Line(double x0, double z0, double x1, double z1)
            => Revolution(new[] { x0, x1 }, new[] { z0, z1 }, new[] { 1.0, 1.0 }, 1, new double[] { 0, 1 }, new[] { 2, 2 });

        private static NurbsSurface Sphere(double r)
            => Revolution(new[] { 0, r, r, r, 0 }, new[] { -r, -r, 0, r, r }, new[] { 1, w, 1, w, 1 }, 2, new double[] { 0, 1, 2 }, new[] { 3, 2, 3 });

        /// <summary>
        /// A torus with the major radius <paramref name="major"/> and the minor radius <paramref name="minor"/>: the profile is
        /// the full circle around (major, 0, 0).
        /// </summary>
        private static NurbsSurface Torus(double major, double minor)
        {
            double[] px = { 1, 1, 0, -1, -1, -1, 0, 1, 1 };
            double[] pz = { 0, 1, 1, 1, 0, -1, -1, -1, 0 };
            double[] x = new double[9], z = new double[9];
            for (int j = 0; j < 9; j++)
            {
                x[j] = major + minor * px[j];
                z[j] = minor * pz[j];
            }
            return Revolution(x, z, new[] { 1, w, 1, w, 1, w, 1, w, 1 }, 2, new double[] { 0, 1, 2, 3, 4 }, new[] { 3, 2, 2, 2, 3 });
        }

        /// <summary>
        /// Checks the type of the canonical form, that it contains the points of the NURBS surface and that the normals agree.
        /// </summary>
        private static void AssertCanonical(NurbsSurface nurbs, Type expected)
        {
            ISurface canonical = nurbs.GetCanonicalForm(1e-6, null);
            Assert.IsNotNull(canonical, $"{expected.Name} not recognized");
            Assert.IsInstanceOfType(canonical, expected);
            nurbs.GetNaturalBounds(out double umin, out double umax, out double vmin, out double vmax);
            for (int i = 0; i <= 10; i++)
                for (int j = 1; j < 10; j++)
                {   // v without the ends, which may be poles
                    GeoPoint2D uv = new GeoPoint2D(umin + i * (umax - umin) / 10, vmin + j * (vmax - vmin) / 10);
                    GeoPoint p = nurbs.PointAt(uv);
                    Assert.AreEqual(0.0, canonical.GetDistance(p), 1e-6, $"point of the NURBS surface at {uv}");
                    GeoVector n = nurbs.GetNormal(uv).Normalized;
                    Assert.AreEqual(1.0, canonical.GetNormal(canonical.PositionOf(p)).Normalized * n, 1e-6, $"normal at {uv}");
                }
        }

        [TestMethod]
        public void a_plane_is_recognized()
        {
            GeoPoint[,] poles = { { new GeoPoint(0, 0, 1), new GeoPoint(0, 10, 2) }, { new GeoPoint(10, 0, 3), new GeoPoint(10, 10, 4) } };
            NurbsSurface plane = new NurbsSurface(poles, null, new double[] { 0, 1 }, new double[] { 0, 1 }, new[] { 2, 2 }, new[] { 2, 2 }, 1, 1, false, false);
            AssertCanonical(plane, typeof(PlaneSurface));
        }

        /// <summary>
        /// A cylinder was not recognized before: the branch for constant radii did nothing.
        /// </summary>
        [TestMethod]
        public void a_cylinder_is_recognized()
        {
            AssertCanonical(Line(10, 0, 10, 30), typeof(CylindricalSurface));
            AssertCanonical(Line(10, 30, 10, 0), typeof(CylindricalSurface)); // the other orientation
        }

        [TestMethod]
        public void a_cone_is_recognized()
        {
            AssertCanonical(Line(10, 0, 4, 30), typeof(ConicalSurface));
            AssertCanonical(Line(4, 30, 10, 0), typeof(ConicalSurface));
        }

        [TestMethod]
        public void a_sphere_is_recognized()
        {
            AssertCanonical(Sphere(20), typeof(SphericalSurface));
            AssertCanonical(Sphere(0.05), typeof(SphericalSurface)); // the fit of the sphere used an error, which depends on the radius
        }

        [TestMethod]
        public void a_torus_is_recognized()
        {
            AssertCanonical(Torus(20, 5), typeof(ToroidalSurface));
        }

        /// <summary>
        /// A sphere with a displaced inner pole is a freeform surface: the deviation from the sphere is about 1 % of its size,
        /// far beyond any tolerance, it must not be replaced by a sphere.
        /// </summary>
        [TestMethod]
        public void a_deformed_sphere_is_not_recognized()
        {
            NurbsSurface sphere = Sphere(20);
            GeoPoint[,] poles = (GeoPoint[,])sphere.Poles.Clone();
            poles[3, 2] = poles[3, 2] + new GeoVector(1.5, 1.5, 0);
            NurbsSurface deformed = new NurbsSurface(poles, sphere.Weights, new double[] { 0, 1, 2, 3, 4 }, new double[] { 0, 1, 2 },
                new[] { 3, 2, 2, 2, 3 }, new[] { 3, 2, 3 }, 2, 2, true, false);
            Assert.IsNull(deformed.GetCanonicalForm(1e-6, null));
        }
    }
}
