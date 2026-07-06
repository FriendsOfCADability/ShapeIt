using CADability.GeoObject;

namespace CADability.Tests
{
    // Regression tests for Triangulation.cs: guard against sliver (very thin, acute) triangles
    // in the tessellation used for rendering. See Face.GetSimpleTriangulation.
    [TestClass]
    public class TriangulationQualityTests
    {
        public TestContext TestContext { get; set; }

        [TestMethod]
        [DeploymentItem(@"Files/Faces/ToroidalFace.json", nameof(triangulate_toroidal_face_has_no_sliver_triangles_at_moderate_precision))]
        public void triangulate_toroidal_face_has_no_sliver_triangles_at_moderate_precision()
        {
            // Before the quality-driven diagonal exchange fix in ExchangeDiagonalDist, this face
            // produced triangles with a minimum angle around 0.2 degrees (aspect ratio ~12) at
            // this precision. After that fix and the 3D-shape-aware ear-clipping priority: ~3.9
            // degrees / ~14.6. After adding SmoothInnerVertices (work-queue with a minimum
            // 5% improvement threshold per move, tuned for performance): ~7.3 degrees / ~7.8.
            AssertTriangulationQuality(nameof(triangulate_toroidal_face_has_no_sliver_triangles_at_moderate_precision),
                precision: 0.05, minAngleDegreesThreshold: 5.0, maxAspectRatioThreshold: 10.0);
        }

        [TestMethod]
        [DeploymentItem(@"Files/Faces/ToroidalFace.json", nameof(triangulate_toroidal_face_has_no_sliver_triangles_at_fine_precision))]
        public void triangulate_toroidal_face_has_no_sliver_triangles_at_fine_precision()
        {
            // Before the 3D-shape-aware ear-clipping priority fix in InsertEdge, this face
            // produced triangles with a minimum angle around 0.96 degrees (aspect ratio ~33, and
            // 2 triangles below 1 degree) at this precision. After that fix: ~1.7 / ~30, 0
            // triangles below 1 degree. After adding SmoothInnerVertices (work-queue with a
            // minimum 5% improvement threshold per move, tuned for performance): ~1.5 / ~30.
            AssertTriangulationQuality(nameof(triangulate_toroidal_face_has_no_sliver_triangles_at_fine_precision),
                precision: 0.01, minAngleDegreesThreshold: 1.0, maxAspectRatioThreshold: 35.0);
        }

        [TestMethod]
        [DeploymentItem(@"Files/Faces/ToroidalFace.json", nameof(triangulate_toroidal_face_thorough_smoothing_improves_quality_and_is_thread_scoped))]
        public void triangulate_toroidal_face_thorough_smoothing_improves_quality_and_is_thread_scoped()
        {
            var file = System.IO.Path.Combine(this.TestContext.DeploymentDirectory,
                nameof(triangulate_toroidal_face_thorough_smoothing_improves_quality_and_is_thread_scoped), "ToroidalFace.json");
            Assert.IsTrue(File.Exists(file));
            Face LoadFace()
            {
                using var stream = File.Open(file, FileMode.Open);
                return new JsonSerialize().FromStream(stream) as Face;
            }

            const double precision = 0.05;

            // The underlying smoothing is a greedy, order-dependent relaxation: the single worst
            // triangle's angle can occasionally end up marginally worse under "thorough" mode even
            // though the mesh as a whole improved (a different, but not everywhere-better, local
            // optimum). The average minimum angle across all triangles is a much more stable
            // measure of that and was confirmed better under "thorough" in every repeated run
            // during development, unlike the single worst-case triangle.
            double defaultAvgMinAngle = MeasureAvgMinAngle(LoadFace(), precision);

            double thoroughAvgMinAngle;
            using (TriangulationSmoothing.UseThoroughSmoothing())
            {
                thoroughAvgMinAngle = MeasureAvgMinAngle(LoadFace(), precision);
            }

            // UseThoroughSmoothing is scoped per thread (see its doc comment: STL export must not
            // race with concurrent background re-triangulation on other threads). After leaving
            // the using block, behavior on this thread must be back to the interactive default.
            double afterScopeAvgMinAngle = MeasureAvgMinAngle(LoadFace(), precision);

            Assert.IsTrue(thoroughAvgMinAngle > defaultAvgMinAngle,
                $"thorough smoothing should improve the average minimum angle at this precision: {thoroughAvgMinAngle:F3} vs default {defaultAvgMinAngle:F3}");
            Assert.AreEqual(defaultAvgMinAngle, afterScopeAvgMinAngle, 1e-9,
                "behavior should return to the default once the thorough-smoothing scope is disposed");
        }

        private static double MeasureAvgMinAngle(Face face, double precision)
        {
            face.GetSimpleTriangulation(precision, false, out GeoPoint[] trianglePoint, out GeoPoint2D[] triangleUVPoint,
                out int[] triangleIndex, out int[] edgeIndizes);
            double sum = 0;
            int triCount = triangleIndex.Length / 3;
            for (int i = 0; i < triangleIndex.Length; i += 3)
            {
                sum += MinTriangleAngleDegrees(
                    trianglePoint[triangleIndex[i]], trianglePoint[triangleIndex[i + 1]], trianglePoint[triangleIndex[i + 2]]);
            }
            return sum / triCount;
        }

        private void AssertTriangulationQuality(string testName, double precision, double minAngleDegreesThreshold, double maxAspectRatioThreshold)
        {
            var file = System.IO.Path.Combine(this.TestContext.DeploymentDirectory, testName, "ToroidalFace.json");
            Assert.IsTrue(File.Exists(file));

            Face face;
            using (var stream = File.Open(file, FileMode.Open))
            {
                var serializer = new JsonSerialize();
                face = serializer.FromStream(stream) as Face;
            }
            Assert.IsNotNull(face);

            face.GetSimpleTriangulation(precision, false, out GeoPoint[] trianglePoint, out GeoPoint2D[] triangleUVPoint,
                out int[] triangleIndex, out int[] edgeIndizes);

            Assert.IsTrue(triangleIndex.Length > 0, "triangulation produced no triangles");

            double minAngleDegrees = double.MaxValue;
            double worstAspectRatio = 0.0;
            for (int i = 0; i < triangleIndex.Length; i += 3)
            {
                GeoPoint p1 = trianglePoint[triangleIndex[i]];
                GeoPoint p2 = trianglePoint[triangleIndex[i + 1]];
                GeoPoint p3 = trianglePoint[triangleIndex[i + 2]];

                minAngleDegrees = Math.Min(minAngleDegrees, MinTriangleAngleDegrees(p1, p2, p3));
                worstAspectRatio = Math.Max(worstAspectRatio, TriangleAspectRatio(p1, p2, p3));
            }

            Assert.IsTrue(minAngleDegrees > minAngleDegreesThreshold, $"minimum triangle angle too small: {minAngleDegrees:F3} degrees");
            Assert.IsTrue(worstAspectRatio < maxAspectRatioThreshold, $"worst triangle aspect ratio too large: {worstAspectRatio:F1}");
        }

        private static double MinTriangleAngleDegrees(GeoPoint p1, GeoPoint p2, GeoPoint p3)
        {
            double a = p2 | p3;
            double b = p1 | p3;
            double c = p1 | p2;
            double angleA = Math.Acos(Clamp((b * b + c * c - a * a) / (2 * b * c)));
            double angleB = Math.Acos(Clamp((a * a + c * c - b * b) / (2 * a * c)));
            double angleC = Math.PI - angleA - angleB;
            return Math.Min(angleA, Math.Min(angleB, angleC)) * 180.0 / Math.PI;
        }

        private static double TriangleAspectRatio(GeoPoint p1, GeoPoint p2, GeoPoint p3)
        {
            double a = p2 | p3;
            double b = p1 | p3;
            double c = p1 | p2;
            return Math.Max(a, Math.Max(b, c)) / Math.Min(a, Math.Min(b, c));
        }

        private static double Clamp(double x) => Math.Max(-1.0, Math.Min(1.0, x));
    }
}
