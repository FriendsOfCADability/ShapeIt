using System.Globalization;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;

namespace CADability.Tests
{
    /// <summary>
    /// Regression tests for the triangulation of faces carrying a pole - the mantle of a cone whose apex
    /// lies on the axis, a sphere at its poles. See <see cref="CDTriangulation"/>.
    /// <para>
    /// These live in their own file rather than in TriangulationQualityTests, which the csproj excludes
    /// from compilation because it targets entry points that only exist in the prototype worktree.
    /// </para>
    /// </summary>
    [TestClass]
    public class PoleTriangulationTests
    {
        public TestContext TestContext { get; set; }

        /// <summary>
        /// The shape criterion of the refinement used to cascade at a pole: it measures the angle in 3d
        /// but computes its remedy, a circumcenter, in the per-axis arc length normalized uv space. That
        /// space cannot express the cone's polar parametrization - u is an angle, v the radius, so
        /// |dS/du| depends on v - and near the apex the inserted point does not improve the 3d angle, so
        /// the triangle is refined again. One half of this mantle produced 37368 triangles at this
        /// precision and stopped only at CDTriangulation's vertex cap; it needs a few hundred.
        /// </summary>
        [TestMethod]
        public void cone_mantle_with_apex_does_not_cascade()
        {
            Solid cone = MakeCone();

            const double precision = 0.01;
            // slant sqrt(10^2 + 24^2) = 26, so one half of the mantle is pi*10*26/2
            const double exactHalfMantleArea = Math.PI * 10.0 * 26.0 / 2.0;

            int mantleFaces = 0;
            foreach (Face face in cone.Shells[0].Faces)
            {
                if (!(face.Surface is ConicalSurface)) continue;
                ++mantleFaces;
                face.GetTriangulation(precision, out GeoPoint[] trianglePoint, out GeoPoint2D[] triangleUVPoint,
                    out int[] triangleIndex, out BoundingBox triangleExtent);
                int triangleCount = triangleIndex.Length / 3;
                double area = 0.0;
                for (int i = 0; i < triangleIndex.Length; i += 3)
                {
                    GeoVector a = trianglePoint[triangleIndex[i + 1]] - trianglePoint[triangleIndex[i]];
                    GeoVector b = trianglePoint[triangleIndex[i + 2]] - trianglePoint[triangleIndex[i]];
                    area += 0.5 * (a ^ b).Length;
                }
                TestContext.WriteLine($"cone mantle half: {triangleCount} triangles, area {area:F4} (exact {exactHalfMantleArea:F4})");

                Assert.IsTrue(triangleCount < 5000,
                    $"shape refinement cascaded at the apex: {triangleCount} triangles for a face that needs a few hundred");
                // bounding the refinement must not cost accuracy - that is the whole point of bounding it
                // where the refinement cannot help rather than bounding it everywhere
                Assert.AreEqual(exactHalfMantleArea, area, exactHalfMantleArea * 0.02,
                    "the bounded triangulation no longer represents the mantle");
            }
            Assert.AreEqual(2, mantleFaces, "expected the mantle to be split into two faces at the seam");
        }

        /// <summary>
        /// Every vertex handed to <see cref="IPaintTo3D.Triangle"/> must carry a usable normal, poles
        /// included. Face.PaintFaceTo3D used to pass a null vector at a pole, which the shader renderers
        /// survive - they normalize per fragment - while the fixed function OpenGL path lights the tip with
        /// ambient only and the STL export writes a facet normal that is not a unit vector.
        /// <para>
        /// Driven through PaintToSTL, which is a real IPaintTo3D and averages the three vertex normals into
        /// its facet normal: with a null vector at the apex that average came out at 2/3 of unit length.
        /// </para>
        /// </summary>
        [TestMethod]
        public void pole_vertices_get_a_usable_normal()
        {
            Solid cone = MakeCone();
            string file = System.IO.Path.Combine(System.IO.Path.GetTempPath(),
                "PoleTriangulationTests_" + Guid.NewGuid().ToString("N") + ".stl");
            try
            {
                using (PaintToSTL pstl = new PaintToSTL(file, 0.05))
                {
                    pstl.Init();
                    cone.Clone().PaintTo3D(pstl);
                }

                int facets = 0, degenerate = 0;
                double worst = 0.0;
                foreach (string line in File.ReadLines(file))
                {
                    string s = line.Trim();
                    if (!s.StartsWith("facet normal", StringComparison.Ordinal)) continue;
                    string[] parts = s.Substring("facet normal".Length)
                        .Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                    Assert.AreEqual(3, parts.Length, "malformed facet normal: " + s);
                    double x = double.Parse(parts[0], CultureInfo.InvariantCulture);
                    double y = double.Parse(parts[1], CultureInfo.InvariantCulture);
                    double z = double.Parse(parts[2], CultureInfo.InvariantCulture);
                    ++facets;
                    double len = Math.Sqrt(x * x + y * y + z * z);
                    if (len < 1e-6) { ++degenerate; continue; }
                    worst = Math.Max(worst, Math.Abs(len - 1.0));
                }

                TestContext.WriteLine($"{facets} facets, {degenerate} with a zero normal, worst |len-1| = {worst:F4}");
                Assert.IsTrue(facets > 0, "the STL export produced no facets");
                Assert.AreEqual(0, degenerate, "facets with a zero normal - a pole vertex had no normal");
                // the STL normal is the mean of three unit normals, so it is shorter than 1 by however much
                // they diverge; a null vector among them used to cost a full third
                Assert.IsTrue(worst < 0.2,
                    $"facet normals are not unit vectors, worst deviation {worst:F4} - a pole vertex is dragging the average down");
            }
            finally
            {
                if (File.Exists(file)) File.Delete(file);
            }
        }

        /// <summary>A cone of base radius 10 at z = 0 with its apex at (0,0,24), slant 26.</summary>
        private static Solid MakeCone()
        {
            Border profile = new Border(new ICurve2D[] {
                new Line2D(new GeoPoint2D(0, 24), new GeoPoint2D(10, 0)),
                new Line2D(new GeoPoint2D(10, 0), new GeoPoint2D(0, 0)),
                new Line2D(new GeoPoint2D(0, 0), new GeoPoint2D(0, 24)) });
            Face toRotate = Face.MakeFace(new PlaneSurface(Plane.XZPlane), new SimpleShape(profile));
            Solid cone = Make3D.Rotate(toRotate, new Axis(GeoPoint.Origin, GeoVector.ZAxis),
                SweepAngle.Deg(360), 0, null) as Solid;
            Assert.IsNotNull(cone, "the revolve did not produce a solid");
            return cone;
        }
    }
}
