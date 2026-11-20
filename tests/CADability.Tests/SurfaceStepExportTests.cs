using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// Surfaces which have no entity in STEP (here a <see cref="SweptCircleSurface"/>) are exported as approximating NURBS
    /// surfaces over their domain, see <see cref="ISurfaceImpl.Approximate"/>.
    /// </summary>
    [TestClass]
    public class SurfaceStepExportTests
    {
        private static SweptCircleSurface WavyPipe()
        {
            GeoPoint[] points = new GeoPoint[7];
            for (int i = 0; i < points.Length; i++) points[i] = new GeoPoint(3.0 * i, 2.0 * System.Math.Sin(0.9 * i), 1.0);
            BSpline spine = BSpline.Construct();
            spine.ThroughPoints(points, 3, false);
            return new SweptCircleSurface(spine, 1.0);
        }

        [TestMethod]
        public void the_approximation_has_the_same_uv_system_and_the_requested_precision()
        {
            SweptCircleSurface pipe = WavyPipe();
            BoundingRect domain = new BoundingRect(0.1, 0.0, 0.9, 2.0 * System.Math.PI);
            NurbsSurface ns = pipe.Approximate(domain.Left, domain.Right, domain.Bottom, domain.Top, 1e-6);
            Assert.IsNotNull(ns);
            System.Random rnd = new System.Random(17);
            for (int i = 0; i < 500; i++)
            {
                GeoPoint2D uv = new GeoPoint2D(domain.Left + rnd.NextDouble() * domain.Width, domain.Bottom + rnd.NextDouble() * domain.Height);
                Assert.AreEqual(0.0, ns.PointAt(uv) | pipe.PointAt(uv), 2e-6, $"at {uv}");
            }
            // the same orientation
            GeoPoint2D mid = domain.GetCenter();
            Assert.IsTrue(ns.GetNormal(mid).Normalized * pipe.GetNormal(mid).Normalized > 0.999, "same orientation");
        }

        /// <summary>
        /// Faces on the pipe are exported as ADVANCED_FACEs on NURBS surfaces, also when they cross v=0, the seam of the
        /// pipe, or span more than pi.
        /// </summary>
        [TestMethod]
        public void faces_on_a_pipe_are_exported_with_nurbs_surfaces()
        {
            foreach (BoundingRect domain in new[] { new BoundingRect(0.1, 0.5, 0.9, 2.5), new BoundingRect(0.1, -1.0, 0.9, 4.0) })
            {
                Face face = Face.MakeFace(WavyPipe(), domain);
                Assert.IsNotNull(face);
                Project pr = Project.CreateSimpleProject();
                pr.GetActiveModel().Add(face);
                string step = System.Text.Encoding.UTF8.GetString(new ExportStep().WriteToByteArray(pr));
                string name = $"{domain.Bottom}..{domain.Top}";
                Assert.IsTrue(step.Contains("ADVANCED_FACE"), $"a face for {name}");
                Assert.IsTrue(step.Contains("B_SPLINE_SURFACE_WITH_KNOTS"), $"the pipe is exported as a NURBS surface for {name}");
            }
        }
    }
}
