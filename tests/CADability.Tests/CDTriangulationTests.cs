using CADability.GeoObject;

namespace CADability.Tests
{
    // Validation of the CDTriangulation prototype (constrained Delaunay triangulation with
    // 3d-criteria-driven refinement, see CADability/CDTriangulation.cs): structural mesh
    // invariants must hold for all test faces; quality and deflection metrics are reported
    // for comparison with the legacy Triangulation.
    [TestClass]
    public class CDTriangulationTests
    {
        public TestContext TestContext { get; set; }

        private static readonly string[] Files = new[]
        {
            "ToroidalFace.json",
            "NurbsWithHoles.json",
            "CylinerConcaveWithHoles.json",
            "PlaneConcaveWithHole.json",
            "ConeSimple.json",
            "ConeWithPole.json",
            "PlaneConcaveWithHoles.json",
            "SphereConcave.json",
        };

        [TestMethod]
        [DeploymentItem(@"Files/Faces/ToroidalFace.json")]
        [DeploymentItem(@"Files/Faces/NurbsWithHoles.json")]
        [DeploymentItem(@"Files/Faces/CylinerConcaveWithHoles.json")]
        [DeploymentItem(@"Files/Faces/PlaneConcaveWithHole.json")]
        [DeploymentItem(@"Files/Faces/ConeSimple.json")]
        [DeploymentItem(@"Files/Faces/ConeWithPole.json")]
        [DeploymentItem(@"Files/Faces/PlaneConcaveWithHoles.json")]
        [DeploymentItem(@"Files/Faces/SphereConcave.json")]
        public void cdt_meshes_are_structurally_valid_for_all_test_faces()
        {
            const double precision = 0.05;
            var report = new System.Text.StringBuilder();
            foreach (var fileName in Files)
            {
                var path = System.IO.Path.Combine(this.TestContext.DeploymentDirectory, fileName);
                Assert.IsTrue(File.Exists(path), fileName + " missing");
                Face face;
                using (var stream = File.Open(path, FileMode.Open))
                {
                    face = new JsonSerialize().FromStream(stream) as Face;
                }
                Assert.IsNotNull(face, fileName + ": could not load face");

                var sw = System.Diagnostics.Stopwatch.StartNew();
                face.GetSimpleTriangulationCDT(precision, 5.0,
                    out GeoPoint[] pts, out GeoPoint2D[] uvs, out int[] idx, out CDTriangulation cdt);
                sw.Stop();

                Assert.IsTrue(idx.Length > 0, fileName + ": no triangles produced");
                Assert.IsTrue(idx.Length % 3 == 0, fileName + ": triangle index count not divisible by 3");
                Assert.IsFalse(cdt.innerIntersection, fileName + ": input flagged as self intersecting");
                Assert.IsFalse(cdt.classificationConflict, fileName + ": inside/outside classification conflict");

                // uv extent as scale for orientation tolerance
                double uvScale = 0;
                for (int i = 0; i < idx.Length; ++i)
                {
                    uvScale = Math.Max(uvScale, Math.Abs(uvs[idx[i]].x) + Math.Abs(uvs[idx[i]].y));
                }

                var directed = new HashSet<long>();
                var boundaryOut = new Dictionary<int, int>();
                var boundaryIn = new Dictionary<int, int>();
                double minAngleDeg = double.MaxValue, worstAspect = 0, sumMinAngle = 0;
                for (int i = 0; i < idx.Length; i += 3)
                {
                    int a = idx[i], b = idx[i + 1], c = idx[i + 2];
                    Assert.IsTrue(a >= 0 && a < pts.Length && b >= 0 && b < pts.Length && c >= 0 && c < pts.Length,
                        fileName + ": vertex index out of range");
                    Assert.IsTrue(a != b && b != c && a != c, fileName + ": degenerate triangle (repeated vertex)");
                    // counterclockwise in uv (positive signed area; tiny tolerance for double rounding,
                    // the mesh itself was built with exact predicates)
                    double cross = (uvs[b].x - uvs[a].x) * (uvs[c].y - uvs[a].y)
                                 - (uvs[b].y - uvs[a].y) * (uvs[c].x - uvs[a].x);
                    Assert.IsTrue(cross > -1e-12 * uvScale * uvScale, fileName + ": clockwise/degenerate uv triangle at " + i);
                    // every directed edge may appear at most once in a valid triangulation
                    foreach (var (s, e) in new[] { (a, b), (b, c), (c, a) })
                    {
                        long key = ((long)(uint)s << 32) | (uint)e;
                        Assert.IsTrue(directed.Add(key), fileName + ": directed edge used twice - overlapping triangles");
                    }
                    minAngleDeg = Math.Min(minAngleDeg, MinAngleDeg(pts[a], pts[b], pts[c]));
                    sumMinAngle += MinAngleDeg(pts[a], pts[b], pts[c]);
                    worstAspect = Math.Max(worstAspect, AspectRatio(pts[a], pts[b], pts[c]));
                }
                // unpaired directed edges are boundary edges; they must form closed loops
                // (equal in/out degree at every vertex)
                foreach (long key in directed)
                {
                    int s = (int)(key >> 32), e = (int)(key & 0xffffffff);
                    long rev = ((long)(uint)e << 32) | (uint)s;
                    if (directed.Contains(rev)) continue;
                    boundaryOut.TryGetValue(s, out int od); boundaryOut[s] = od + 1;
                    boundaryIn.TryGetValue(e, out int idg); boundaryIn[e] = idg + 1;
                }
                foreach (var kv in boundaryOut)
                {
                    boundaryIn.TryGetValue(kv.Key, out int idg);
                    Assert.AreEqual(kv.Value, idg, fileName + ": boundary edges do not form closed loops at vertex " + kv.Key);
                }

                // deflection spot check at the uv centroid of each triangle (tolerance is
                // precision * 5.0, the same the production path uses for the interior)
                double tol = precision * 5.0;
                int deflViolations = 0;
                double worstDefl = 0;
                ISurface surface = face.Surface;
                for (int i = 0; i < idx.Length; i += 3)
                {
                    GeoPoint pa = pts[idx[i]], pb = pts[idx[i + 1]], pc = pts[idx[i + 2]];
                    GeoVector nrm = (pb - pa) ^ (pc - pa);
                    double area2 = nrm.Length;
                    double lmax = Math.Max(pa | pb, Math.Max(pb | pc, pc | pa));
                    if (lmax <= 0 || area2 < 1e-9 * lmax) continue; // degenerate, not meaningful
                    GeoPoint2D cuv = new GeoPoint2D(
                        (uvs[idx[i]].x + uvs[idx[i + 1]].x + uvs[idx[i + 2]].x) / 3.0,
                        (uvs[idx[i]].y + uvs[idx[i + 1]].y + uvs[idx[i + 2]].y) / 3.0);
                    double d = Math.Abs((surface.PointAt(cuv) - pa) * nrm) / area2;
                    worstDefl = Math.Max(worstDefl, d);
                    if (d > tol * 1.5) ++deflViolations;
                }

                report.AppendLine($"--- {fileName} ---");
                report.AppendLine($"triangles={idx.Length / 3}, vertices={pts.Length}, time={sw.ElapsedMilliseconds}ms");
                report.AppendLine($"minAngle={minAngleDeg:F2}deg, avgMinAngle={sumMinAngle / (idx.Length / 3):F2}deg, worstAspect={worstAspect:F1}");
                report.AppendLine($"worstCentroidDeflection={worstDefl:F4} (tol {tol:F2}), violations(>1.5tol)={deflViolations}");
                report.AppendLine($"steiner: edge={cdt.SteinerByEdgeDeflection}, face={cdt.SteinerByFaceDeflection}, quality={cdt.SteinerByQuality}, flips={cdt.FlipCount}");
                TestContext.WriteLine(report.ToString()); // also flushed per face so failures below still show the metrics
                report.Clear();

                Assert.IsTrue(minAngleDeg > 0.1, fileName + $": extreme sliver triangle, min angle {minAngleDeg:F4} deg");
            }
            TestContext.WriteLine(report.ToString());
        }

        private static double MinAngleDeg(GeoPoint p1, GeoPoint p2, GeoPoint p3)
        {
            double a = p2 | p3, b = p1 | p3, c = p1 | p2;
            if (a <= 0 || b <= 0 || c <= 0) return 0.0;
            double angleA = Math.Acos(Clamp((b * b + c * c - a * a) / (2 * b * c)));
            double angleB = Math.Acos(Clamp((a * a + c * c - b * b) / (2 * a * c)));
            double angleC = Math.PI - angleA - angleB;
            return Math.Min(angleA, Math.Min(angleB, angleC)) * 180.0 / Math.PI;
        }

        private static double AspectRatio(GeoPoint p1, GeoPoint p2, GeoPoint p3)
        {
            double a = p2 | p3, b = p1 | p3, c = p1 | p2;
            double mn = Math.Min(a, Math.Min(b, c));
            if (mn <= 0) return double.MaxValue;
            return Math.Max(a, Math.Max(b, c)) / mn;
        }

        private static double Clamp(double x) => Math.Max(-1.0, Math.Min(1.0, x));
    }
}
