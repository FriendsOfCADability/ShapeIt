using CADability.GeoObject;

namespace CADability.Tests
{
    // TEMPORARY diagnostic, not part of the regression suite - used to verify the
    // EarClippingRecursiveSplit/Approximate investigation. Safe to delete once done.
    [TestClass]
    public class RecursiveSplitDiagnosticTests
    {
        public TestContext TestContext { get; set; }

        private static readonly string[] Files = new[]
        {
            "ToroidalFace.json",
            "NurbsWithHoles.json",
            "CylinerConcaveWithHoles.json",
            "PlaneConcaveWithHole.json",
            "ConeSimple.json",
            "PlaneConcaveWithHoles.json",
            "SphereConcave.json",
        };

        [TestMethod]
        [DeploymentItem(@"Files/Faces/ToroidalFace.json")]
        [DeploymentItem(@"Files/Faces/NurbsWithHoles.json")]
        [DeploymentItem(@"Files/Faces/CylinerConcaveWithHoles.json")]
        [DeploymentItem(@"Files/Faces/PlaneConcaveWithHole.json")]
        [DeploymentItem(@"Files/Faces/ConeSimple.json")]
        [DeploymentItem(@"Files/Faces/PlaneConcaveWithHoles.json")]
        [DeploymentItem(@"Files/Faces/SphereConcave.json")]
        public void report_crossings_per_stage_for_all_test_faces()
        {
            var report = new System.Text.StringBuilder();
            foreach (var fileName in Files)
            {
                var path = System.IO.Path.Combine(this.TestContext.DeploymentDirectory, fileName);
                Face face;
                using (var stream = File.Open(path, FileMode.Open))
                {
                    face = new JsonSerialize().FromStream(stream) as Face;
                }
                report.AppendLine($"--- {fileName} ---");
                for (int stage = 0; stage <= 5; ++stage)
                {
                    try
                    {
                        face.GetSimpleTriangulationRecursiveSplit(0.05, stage, 5.0,
                            out GeoPoint[] trianglePoint, out _, out int[] triangleIndex, out System.Collections.Generic.List<string> crossings);
                        double minAngle = double.MaxValue, worstAspect = 0;
                        for (int i = 0; i < triangleIndex.Length; i += 3)
                        {
                            GeoPoint p1 = trianglePoint[triangleIndex[i]], p2 = trianglePoint[triangleIndex[i + 1]], p3 = trianglePoint[triangleIndex[i + 2]];
                            minAngle = System.Math.Min(minAngle, TriangulationQualityTests_MinAngle(p1, p2, p3));
                            worstAspect = System.Math.Max(worstAspect, TriangulationQualityTests_AspectRatio(p1, p2, p3));
                        }
                        report.AppendLine($"stage {stage}: triangles={triangleIndex.Length / 3}, crossings={crossings.Count}, minAngle={minAngle:F3}deg, worstAspect={worstAspect:F1}");
                        if (crossings.Count > 0)
                        {
                            foreach (var c in crossings) report.AppendLine("    " + c);
                        }
                    }
                    catch (System.Exception ex)
                    {
                        report.AppendLine($"stage {stage}: EXCEPTION {ex.GetType().Name}: {ex.Message}");
                    }
                }
            }
            TestContext.WriteLine(report.ToString());
            Assert.Fail(report.ToString()); // force output to be visible in test results
        }

        [TestMethod]
        [DeploymentItem(@"Files/Faces/PlaneConcaveWithHoles.json")]
        public void plane_concave_with_holes_bad_split_diagnostic()
        {
            var path = System.IO.Path.Combine(this.TestContext.DeploymentDirectory, "PlaneConcaveWithHoles.json");
            Face face;
            using (var stream = File.Open(path, FileMode.Open))
            {
                face = new JsonSerialize().FromStream(stream) as Face;
            }
            TriangulationDebugCounters.Log.Clear();
            var report = new System.Text.StringBuilder();
            try
            {
                face.GetSimpleTriangulationRecursiveSplit(0.05, 0, 5.0, out _, out _, out _, out _);
                report.AppendLine("no exception");
            }
            catch (System.Exception ex)
            {
                report.AppendLine($"EXCEPTION: {ex.Message}");
            }
            report.AppendLine($"log entries: {TriangulationDebugCounters.Log.Count}");
            foreach (var line in TriangulationDebugCounters.Log) report.AppendLine(line);
            Assert.Fail(report.ToString());
        }

        [TestMethod]
        [DeploymentItem(@"Files/Faces/ToroidalFace.json")]
        [DeploymentItem(@"Files/Faces/NurbsWithHoles.json")]
        [DeploymentItem(@"Files/Faces/SphereConcave.json")]
        public void stage5_retract_log_all_files()
        {
            var report = new System.Text.StringBuilder();
            foreach (var fileName in new[] { "ToroidalFace.json", "NurbsWithHoles.json", "SphereConcave.json" })
            {
                var path = System.IO.Path.Combine(this.TestContext.DeploymentDirectory, fileName);
                Face face;
                using (var stream = File.Open(path, FileMode.Open))
                {
                    face = new JsonSerialize().FromStream(stream) as Face;
                }
                using (var stream4 = File.Open(path, FileMode.Open))
                {
                    face = new JsonSerialize().FromStream(stream4) as Face;
                }
                face.GetSimpleTriangulationRecursiveSplit(0.05, 4, 5.0,
                    out _, out _, out int[] triangleIndex4, out System.Collections.Generic.List<string> crossings4);
                report.AppendLine($"--- {fileName} STAGE 4: triangles={triangleIndex4.Length / 3}, crossings={crossings4.Count} ---");

                TriangulationDebugCounters.Log.Clear();
                face.GetSimpleTriangulationRecursiveSplit(0.05, 5, 5.0,
                    out _, out _, out int[] triangleIndex, out System.Collections.Generic.List<string> crossings);
                report.AppendLine($"--- {fileName}: triangles={triangleIndex.Length / 3}, crossings={crossings.Count} ---");
                foreach (var c in crossings) report.AppendLine("XCROSS " + c);
                int accepted = 0, triggered = 0;
                foreach (var line in TriangulationDebugCounters.Log)
                {
                    if (line.StartsWith("RETRACT-TRIGGER")) triggered++;
                    if (line.StartsWith("RETRACT-ACCEPT")) accepted++;
                }
                report.AppendLine($"triggered={triggered} accepted={accepted}");
                foreach (var line in TriangulationDebugCounters.Log) report.AppendLine(line);
            }
            Assert.Fail(report.ToString());
        }

        [TestMethod]
        [DeploymentItem(@"Files/Faces/SphereConcave.json")]
        public void sphere_concave_stage1_nonconvex_quad_log()
        {
            var path = System.IO.Path.Combine(this.TestContext.DeploymentDirectory, "SphereConcave.json");
            Face face;
            using (var stream = File.Open(path, FileMode.Open))
            {
                face = new JsonSerialize().FromStream(stream) as Face;
            }
            TriangulationDebugCounters.Log.Clear();
            face.GetSimpleTriangulationRecursiveSplit(0.05, 1, 5.0,
                out _, out _, out int[] triangleIndex, out System.Collections.Generic.List<string> crossings);
            var report = new System.Text.StringBuilder();
            report.AppendLine($"triangles={triangleIndex.Length / 3}, crossings={crossings.Count}, nonconvex logged={TriangulationDebugCounters.Log.Count}, SplitCount={TriangulationDebugCounters.SplitCount}");
            foreach (var line in TriangulationDebugCounters.Log) report.AppendLine(line);
            Assert.Fail(report.ToString());
        }

        private static double TriangulationQualityTests_MinAngle(GeoPoint p1, GeoPoint p2, GeoPoint p3)
        {
            double a = p2 | p3, b = p1 | p3, c = p1 | p2;
            double angleA = System.Math.Acos(Clamp((b * b + c * c - a * a) / (2 * b * c)));
            double angleB = System.Math.Acos(Clamp((a * a + c * c - b * b) / (2 * a * c)));
            double angleC = System.Math.PI - angleA - angleB;
            return System.Math.Min(angleA, System.Math.Min(angleB, angleC)) * 180.0 / System.Math.PI;
        }
        private static double TriangulationQualityTests_AspectRatio(GeoPoint p1, GeoPoint p2, GeoPoint p3)
        {
            double a = p2 | p3, b = p1 | p3, c = p1 | p2;
            return System.Math.Max(a, System.Math.Max(b, c)) / System.Math.Min(a, System.Math.Min(b, c));
        }
        private static double Clamp(double x) => System.Math.Max(-1.0, System.Math.Min(1.0, x));

        [TestMethod]
        [DeploymentItem(@"Files/Faces/ToroidalFace.json")]
        [DeploymentItem(@"Files/Faces/NurbsWithHoles.json")]
        [DeploymentItem(@"Files/Faces/CylinerConcaveWithHoles.json")]
        [DeploymentItem(@"Files/Faces/PlaneConcaveWithHole.json")]
        [DeploymentItem(@"Files/Faces/ConeSimple.json")]
        [DeploymentItem(@"Files/Faces/PlaneConcaveWithHoles.json")]
        [DeploymentItem(@"Files/Faces/SphereConcave.json")]
        public void quality_and_timing_table_per_stage()
        {
            var report = new System.Text.StringBuilder();
            string[] phaseOrder = { "EarClippingRecursiveSplit", "Approximate(noFlip)", "Approximate(flip)", "RetractBoundaryTips", "CollapseShortEdges", "SmoothInnerVertices" };
            foreach (var fileName in Files)
            {
                var path = System.IO.Path.Combine(this.TestContext.DeploymentDirectory, fileName);
                report.AppendLine($"=== {fileName} ===");
                for (int stage = 0; stage <= 5; ++stage)
                {
                    Face face;
                    using (var stream = File.Open(path, FileMode.Open))
                    {
                        face = new JsonSerialize().FromStream(stream) as Face;
                    }
                    var sw = System.Diagnostics.Stopwatch.StartNew();
                    face.GetSimpleTriangulationRecursiveSplit(0.05, stage, 5.0,
                        out GeoPoint[] trianglePoint, out _, out int[] triangleIndex, out var crossings);
                    sw.Stop();

                    int triCount = triangleIndex.Length / 3;
                    double sumMinAngle = 0, worstMinAngle = double.MaxValue, worstAspect = 0;
                    for (int i = 0; i < triangleIndex.Length; i += 3)
                    {
                        GeoPoint p1 = trianglePoint[triangleIndex[i]], p2 = trianglePoint[triangleIndex[i + 1]], p3 = trianglePoint[triangleIndex[i + 2]];
                        double a = TriangulationQualityTests_MinAngle(p1, p2, p3);
                        sumMinAngle += a;
                        worstMinAngle = System.Math.Min(worstMinAngle, a);
                        worstAspect = System.Math.Max(worstAspect, TriangulationQualityTests_AspectRatio(p1, p2, p3));
                    }
                    double avgMinAngle = triCount > 0 ? sumMinAngle / triCount : 0;

                    report.AppendLine($"stage {stage}: triangles={triCount}, avgMinAngle={avgMinAngle:F2}deg, worstMinAngle={worstMinAngle:F3}deg, worstAspect={worstAspect:F1}, crossings={crossings.Count}, totalMs={sw.Elapsed.TotalMilliseconds:F1}");
                    foreach (var phase in phaseOrder)
                        if (TriangulationDebugCounters.PhaseMs.TryGetValue(phase, out double ms))
                            report.AppendLine($"    {phase}: {ms:F1} ms");
                }
            }
            Assert.Fail(report.ToString());
        }
    }
}
