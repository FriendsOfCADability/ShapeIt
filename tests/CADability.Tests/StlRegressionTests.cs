using CADability;
using CADability.GeoObject;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using Path = System.IO.Path;

namespace CADability.Tests
{
    /// <summary>
    /// Regression harness for <see cref="StlSurfaceReconstruction"/>: every *.stl in Files/STL is run through the
    /// recognition and its result is summarized (one canonical, sorted line per region). The summary is compared
    /// against a committed baseline &lt;file&gt;.txt next to the STL. A missing baseline is generated (review it and
    /// commit); a difference fails the test with a diff, so you can judge whether a change improved or worsened the
    /// result and, if it is an improvement, regenerate the baseline. Regenerate all baselines by setting the
    /// environment variable STL_REGEN=1.
    /// </summary>
    [TestClass]
    public class StlRegressionTests
    {
        private static bool Regenerate => Environment.GetEnvironmentVariable("STL_REGEN") == "1";

        private static string StlDir([CallerFilePath] string thisFile = "")
            => Path.Combine(Path.GetDirectoryName(thisFile), "Files", "STL");

        [TestMethod]
        public void AllStlFilesMatchBaseline()
        {
            string dir = StlDir();
            if (!Directory.Exists(dir)) Assert.Inconclusive($"STL directory not found: {dir}");
            string[] stlFiles = Directory.GetFiles(dir, "*.stl").OrderBy(f => f).ToArray();
            if (stlFiles.Length == 0) Assert.Inconclusive($"no STL files in {dir}");

            List<string> generated = new List<string>();
            List<string> mismatches = new List<string>();
            foreach (string stl in stlFiles)
            {
                ImportSTL import = new ImportSTL();
                import.BuildRawFaces = false; // only recognize surfaces; skip the (slow, fragile) stage 2 face building
                import.Read(stl);
                string summary = FormatSummary(import.Reconstruction);
                string baseline = Path.ChangeExtension(stl, ".txt");
                if (Regenerate || !File.Exists(baseline))
                {
                    File.WriteAllText(baseline, summary);
                    generated.Add(Path.GetFileName(baseline));
                    continue;
                }
                string expected = File.ReadAllText(baseline).Replace("\r\n", "\n").TrimEnd();
                string actual = summary.Replace("\r\n", "\n").TrimEnd();
                if (expected != actual) mismatches.Add(Path.GetFileName(stl) + "\n" + Diff(expected, actual));
            }
            if (mismatches.Count > 0)
                Assert.Fail($"{mismatches.Count} file(s) differ from baseline:\n\n" + string.Join("\n\n", mismatches));
            if (generated.Count > 0)
                Assert.Inconclusive("Generated baseline(s), please review and commit: " + string.Join(", ", generated));
        }

        private static string FormatSummary(StlSurfaceReconstruction rec)
        {
            List<string> lines = new List<string>();
            Dictionary<RecognizedSurfaceKind, int> counts = new Dictionary<RecognizedSurfaceKind, int>();
            int totalTris = 0;
            foreach (RecognizedRegion r in rec.Regions)
            {
                counts.TryGetValue(r.Kind, out int c);
                counts[r.Kind] = c + 1;
                totalTris += r.Triangles.Count;
                lines.Add(RegionLine(r));
            }
            lines.Sort(StringComparer.Ordinal);
            IEnumerable<string> hist = Enum.GetValues(typeof(RecognizedSurfaceKind)).Cast<RecognizedSurfaceKind>()
                .Where(k => counts.ContainsKey(k)).Select(k => $"{k}:{counts[k]}");
            StringBuilder sb = new StringBuilder();
            sb.AppendLine($"{string.Join(" ", hist)}  ({totalTris} triangles, {rec.Regions.Count} regions)");
            foreach (string l in lines) sb.AppendLine(l);
            return sb.ToString();
        }

        private static string RegionLine(RecognizedRegion r)
        {
            int n = r.Triangles.Count;
            switch (r.Surface)
            {
                case PlaneSurface p: return $"Plane loc{P(p.Location)} normal{D(p.Normal)} {n}";
                case CylindricalSurface cy: return $"Cylinder loc{P(cy.Location)} axis{D(cy.Axis)} r={N(cy.RadiusX)} {n}";
                case ConicalSurface co: return $"Cone apex{P(co.Location)} axis{D(co.Axis)} angle={N1(co.OpeningAngle.Radian * 180 / Math.PI)} {n}";
                case SphericalSurface s: return $"Sphere center{P(s.Location)} r={N(s.RadiusX)} {n}";
                case ToroidalSurface t: return $"Torus center{P(t.Location)} axis{D(t.Axis)} R={N(t.MajorRadius)} r={N(t.MinorRadius)} {n}";
                case NurbsSurface nu: return $"Nurbs poles={nu.Poles.GetLength(0)}x{nu.Poles.GetLength(1)} degree={nu.UDegree}x{nu.VDegree} {n}";
                default: return $"Unrecognized {n}";
            }
        }

        // stable, culture-invariant number formatting (no negative zero)
        private static string N(double d) => Fix(d, 2);
        private static string N1(double d) => Fix(d, 1);
        private static string Fix(double d, int dec)
        {
            double r = Math.Round(d, dec);
            if (r == 0.0) r = 0.0; // normalize -0
            return r.ToString("F" + dec, CultureInfo.InvariantCulture);
        }
        private static string P(GeoPoint p) => $"({N(p.x)},{N(p.y)},{N(p.z)})";
        private static string D(GeoVector v)
        {
            GeoVector n = v.Normalized;
            const double eps = 1e-6;
            // canonical sign: make the first significant component positive so axis/normal orientation is stable
            if (n.x < -eps || (Math.Abs(n.x) <= eps && n.y < -eps) || (Math.Abs(n.x) <= eps && Math.Abs(n.y) <= eps && n.z < 0))
                n = -n;
            return $"({Fix(n.x, 3)},{Fix(n.y, 3)},{Fix(n.z, 3)})";
        }

        private static string Diff(string expected, string actual)
        {
            HashSet<string> exp = new HashSet<string>(expected.Split('\n'));
            HashSet<string> act = new HashSet<string>(actual.Split('\n'));
            List<string> diff = new List<string>();
            foreach (string l in expected.Split('\n')) if (!act.Contains(l)) diff.Add("  - " + l);
            foreach (string l in actual.Split('\n')) if (!exp.Contains(l)) diff.Add("  + " + l);
            return string.Join("\n", diff);
        }
    }
}
