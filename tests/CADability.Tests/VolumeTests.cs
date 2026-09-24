using CADability.GeoObject;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using Path = System.IO.Path;

namespace CADability.Tests
{
    /// <summary>
    /// Accuracy check for <see cref="GeoObject.Shell.Volume(double)"/>. The test file Volumes.cdb.json contains three
    /// closed solids with exactly known volumes:
    /// <list type="bullet">
    /// <item>a hemisphere, radius 20</item>
    /// <item>a truncated cone (frustum), radii 20 and 10, height 20</item>
    /// <item>a toroidal segment, major radius 30, minor (ring) radius 10, swept 60 degrees</item>
    /// </list>
    /// The solids are identified by their curved surface type and the computed volume is compared to the analytic
    /// value at several triangulation precisions, so the convergence behaviour is visible in the test output.
    /// The torus reference value is derived from the sweep angle actually stored in the file (measured from the face
    /// parameter bounds), not from an assumed one, so the test stays honest even if the model is regenerated.
    /// </summary>
    [TestClass]
    public class VolumeTests
    {
        public TestContext TestContext { get; set; }

        // analytic reference volumes
        private const double HemisphereR = 20.0;
        private const double HemisphereVolume = 2.0 / 3.0 * Math.PI * HemisphereR * HemisphereR * HemisphereR; // 16755.1608

        private const double ConeR1 = 20.0, ConeR2 = 10.0, ConeH = 20.0;
        private const double ConeVolume = Math.PI * ConeH / 3.0 * (ConeR1 * ConeR1 + ConeR1 * ConeR2 + ConeR2 * ConeR2); // 14660.7657

        // triangulation precisions (chordal deviation, in model units): coarse -> fine
        private static readonly double[] Precisions = { 1.0, 0.5, 0.1, 0.05, 0.01 };
        // relative error required at the finest precision
        private const double Tolerance = 1e-3;
        /// <summary>
        /// How far apart the five meshes may leave the integrated volume of one solid. All three come out bit
        /// identical since the volume is a boundary integral; until 2026-09-24 the hemisphere sat about 6.5e-4 wide
        /// because the uv triangles partitioned its curved domain, see <see cref="IntegratedVolumeMatchesAnalyticOnEveryMesh"/>.
        /// </summary>
        private const double MaxMeshSpread = 1e-9;

        private static string CdbFile([CallerFilePath] string thisFile = "")
            => Path.Combine(Path.GetDirectoryName(thisFile), "Files", "CDB", "Volumes.cdb.json");

        [TestMethod]
        public void VolumesMatchAnalytic()
        {
            string file = CdbFile();
            Assert.IsTrue(System.IO.File.Exists(file), $"test file not found: {file}");

            Project project = Project.ReadFromFile(file, "cdb");
            Assert.IsNotNull(project, "could not read project");
            Model model = project.GetActiveModel();
            Assert.IsNotNull(model);

            List<Solid> solids = model.AllObjects.OfType<Solid>().ToList();
            Assert.AreEqual(3, solids.Count, "expected exactly three solids");

            Solid sphere = solids.Single(s => HasSurface<SphericalSurface>(s));
            Solid cone = solids.Single(s => HasSurface<ConicalSurface>(s));
            Solid torus = solids.Single(s => HasSurface<ToroidalSurface>(s));

            (string name, Solid solid, double expected)[] cases =
            {
                ("Hemisphere",   sphere, HemisphereVolume),
                ("Cone frustum", cone,   ConeVolume),
                ("Torus segment",torus,  TorusSegmentVolume(torus)),
            };

            StringBuilder report = new StringBuilder();
            report.AppendLine();
            report.AppendLine($"{"Solid",-14} {"expected",12} {"precision",10} {"computed",14} {"rel.err",12}");

            double worstFineErr = 0.0;
            double finest = Precisions.Min();
            foreach ((string name, Solid solid, double expected) in cases)
            {
                foreach (double prec in Precisions)
                {
                    double v = solid.Volume(prec);
                    double relErr = (v - expected) / expected;
                    if (prec == finest) worstFineErr = Math.Max(worstFineErr, Math.Abs(relErr));
                    report.AppendLine($"{name,-14} {expected,12:F4} {prec,10:F3} {v,14:F4} {relErr,12:P4}");
                }
            }
            report.AppendLine($"worst |rel.err| at precision {finest} = {worstFineErr:P4}");
            TestContext.WriteLine(report.ToString());
            System.Diagnostics.Trace.WriteLine(report.ToString());

            Assert.IsTrue(worstFineErr < Tolerance,
                $"volume error at finest precision too large: {worstFineErr:P4} (limit {Tolerance:P4})\n{report}");
        }

        /// <summary>
        /// The same three solids, measured with <see cref="ShapeIt.ShellMetrics.IntegratedVolume"/>, which integrates
        /// S.(Su x Sv)/3 over the parameter domain of each face instead of summing tetrahedra over its triangles.
        /// <para>
        /// Two things are asserted, and the second one is the point of the method. It has to hit the closed form at
        /// EVERY precision, not just the finest - the tetrahedron sum is 0.77 percent low on the hemisphere at
        /// precision 1.0 and would fail that on its own. And the spread across the meshes has to stay small, because
        /// that independence is the whole reason the method exists: the recorded volumes used to move when the
        /// triangulator was replaced, on inputs that had not changed at all.
        /// </para>
        /// <para>
        /// All three come out EXACT here, to eight digits and bit identical on all five meshes: the planar faces are
        /// closed form, and the curved ones are integrated over the boundary of their uv domain, which the mesh does
        /// not enter. The hemisphere used to be the exception - its spherical faces have curved uv outlines (domain
        /// 3.4647 against a bounding rectangle of 3.7011), and while the domain was partitioned by the uv triangles
        /// it stayed about 6e-4 wide over the five meshes.
        /// </para>
        /// </summary>
        [TestMethod]
        public void IntegratedVolumeMatchesAnalyticOnEveryMesh()
        {
            string file = CdbFile();
            Assert.IsTrue(System.IO.File.Exists(file), $"test file not found: {file}");
            Model model = Project.ReadFromFile(file, "cdb").GetActiveModel();
            List<Solid> solids = model.AllObjects.OfType<Solid>().ToList();

            (string name, Solid solid, double expected)[] cases =
            {
                ("Hemisphere",    solids.Single(s => HasSurface<SphericalSurface>(s)), HemisphereVolume),
                ("Cone frustum",  solids.Single(s => HasSurface<ConicalSurface>(s)),   ConeVolume),
                ("Torus segment", solids.Single(s => HasSurface<ToroidalSurface>(s)),  TorusSegmentVolume(solids.Single(s => HasSurface<ToroidalSurface>(s)))),
            };

            StringBuilder report = new StringBuilder().AppendLine();
            report.AppendLine($"{"Solid",-14} {"expected",14} {"precision",10} {"integrated",16} {"rel.err",11}   {"tetrahedra",14}");
            List<string> failures = new List<string>();

            foreach ((string name, Solid solid, double expected) in cases)
            {
                double lowest = double.MaxValue, highest = double.MinValue;
                foreach (double prec in Precisions)
                {
                    // a fresh copy per precision: Face.AssureTriangles keeps a mesh that is fine enough already,
                    // so without this the coarse mesh of the first round would be reused for all the others
                    Shell measured = (Shell)solid.Shells[0].Clone();
                    double integrated = ShapeIt.ShellMetrics.IntegratedVolume(measured, prec);
                    double overTriangles = measured.Volume(prec);
                    double relErr = (integrated - expected) / expected;
                    report.AppendLine($"{name,-14} {expected,14:F4} {prec,10:F3} {integrated,16:F6} {relErr,11:P4}   {overTriangles,14:F4}");
                    if (Math.Abs(relErr) > Tolerance)
                        failures.Add($"{name} at precision {prec}: {integrated:F6} against the closed form "
                            + $"{expected:F6}, off by {relErr:P4}");
                    lowest = Math.Min(lowest, integrated);
                    highest = Math.Max(highest, integrated);
                }
                double spread = (highest - lowest) / expected;
                report.AppendLine($"{name,-14} spread over the five meshes: {spread:P4}");
                if (spread > MaxMeshSpread)
                    failures.Add($"{name}: the five meshes give values {spread:P4} apart ({lowest:F6} to {highest:F6}), "
                        + $"limit {MaxMeshSpread:P4} - the mesh is still deciding the answer");
            }
            TestContext.WriteLine(report.ToString());
            System.Diagnostics.Trace.WriteLine(report.ToString());
            if (failures.Count > 0) Assert.Fail(string.Join("\n  ", failures) + "\n" + report);
        }

        // a pointed cone, away from the origin so that the reference point of the integral matters
        private const double PointedR = 20.0, PointedH = 30.0;
        private static readonly GeoPoint PointedBase = new GeoPoint(100.0, 50.0, 20.0);
        private const double BoxCut = 5.0;                                   // the box removes local x > BoxCut
        private const double BoreRadius = 3.0, BoreHeight = 10.0;            // a bore along y at this height
        // relative precisions of the mesh, the same range the regression tests live in and beyond
        private static readonly double[] RelativePrecisions = { 1e-3, 5e-4, 2.5e-4, 1.25e-4, 6.25e-5 };
        private const double PointedTolerance = 1e-6;

        /// <summary>
        /// A pointed cone whose mantle is TRIMMED but still contains the apex - the case that made the recorded volume
        /// of the regression tests jump. The triangulation drops the degenerate triangles next to the apex, and the
        /// old route through the uv triangles either extrapolated the uncovered part with a global factor or fell
        /// back to the tetrahedron sum beyond 2 percent, depending on the mesh: errors between -0.07 and -1.8 percent
        /// over the five meshes used here. The boundary integral of <see cref="ShapeIt.ShellMetrics.IntegratedVolume"/>
        /// has to hit the closed form on every one of them, without a single face falling back.
        /// <list type="bullet">
        /// <item>the untrimmed cone, whose mantle domain is the full rectangle;</item>
        /// <item>the cone with a box cut away beside the axis, so the mantle keeps the apex and gets a straight
        /// boundary that is a hyperbola in 3d and a curve in uv;</item>
        /// <item>the cone with a bore across it, which puts a HOLE into each half of the mantle.</item>
        /// </list>
        /// The reference values come from integrating the cross section over the height, which is elementary for all
        /// three.
        /// </summary>
        [TestMethod]
        public void IntegratedVolumeOfPointedConeIsMeshIndependent()
        {
            Solid Cone() => Make3D.MakeCone(PointedBase, GeoVector.XAxis, PointedH * GeoVector.ZAxis, PointedR, 0.0);
            Solid box = Make3D.MakeBox(PointedBase + new GeoVector(BoxCut, -50.0, -10.0),
                100.0 * GeoVector.XAxis, 100.0 * GeoVector.YAxis, 100.0 * GeoVector.ZAxis);
            Solid bore = Make3D.MakeCylinder(PointedBase + new GeoVector(0.0, -50.0, BoreHeight),
                BoreRadius * GeoVector.XAxis, 100.0 * GeoVector.YAxis);
            Solid[] cut = Solid.Subtract(Cone(), box);
            Solid[] bored = Solid.Subtract(Cone(), bore);
            Assert.AreEqual(1, cut.Length, "the box cut must leave one solid");
            Assert.AreEqual(1, bored.Length, "the bore must leave one solid");
            Assert.IsTrue(bored[0].Shell.Faces.Any(f => f.Surface is ConicalSurface && f.HoleCount > 0),
                "the bore is meant to put a hole into the mantle");

            (string name, Shell shell, double expected)[] cases =
            {
                ("Pointed cone", Cone().Shell,     Math.PI * PointedR * PointedR * PointedH / 3.0),
                ("Box cut",      cut[0].Shell,     PointedConeVolume(r => BoxCutSection(r))),
                ("Bored",        bored[0].Shell,   PointedConeVolume(r => 0.0, BoredSection)),
            };

            StringBuilder report = new StringBuilder().AppendLine();
            report.AppendLine($"{"Solid",-13} {"expected",14} {"rel.prec",9} {"integrated",16} {"rel.err",11}   {"tetrahedra",14}");
            List<string> failures = new List<string>();
            foreach ((string name, Shell shell, double expected) in cases)
            {
                foreach (double relative in RelativePrecisions)
                {
                    Shell measured = (Shell)shell.Clone();
                    double precision = ShapeIt.ShellMetrics.SizeOf(measured) * relative;
                    int fallbacksBefore = ShapeIt.ShellMetrics.VolumeFallbackCount;
                    double integrated = ShapeIt.ShellMetrics.IntegratedVolume(measured, precision);
                    int fallbacks = ShapeIt.ShellMetrics.VolumeFallbackCount - fallbacksBefore;
                    double relErr = (integrated - expected) / expected;
                    report.AppendLine($"{name,-13} {expected,14:F6} {relative,9:E2} {integrated,16:F6} {relErr,11:E2}   {measured.Volume(precision),14:F4}");
                    if (Math.Abs(relErr) > PointedTolerance)
                        failures.Add($"{name} at relative precision {relative:E2}: {integrated:F6} against {expected:F6}, off by {relErr:E2}");
                    if (fallbacks > 0)
                        failures.Add($"{name} at relative precision {relative:E2}: {fallbacks} face(s) fell back to the tetrahedron sum");
                }
            }
            TestContext.WriteLine(report.ToString());
            if (failures.Count > 0) Assert.Fail(string.Join("\n  ", failures) + "\n" + report);
        }

        /// <summary>
        /// The volume of the pointed cone minus what <paramref name="removedFromDisc"/> takes away from the disc of
        /// radius r, or what <paramref name="removedAtHeight"/> takes away at height z above the base, integrated
        /// over the height by the midpoint rule. The removed areas have square root edges, which limits the rule to
        /// about h^1.5 there; two million steps keep that far below the tolerance.
        /// </summary>
        private static double PointedConeVolume(Func<double, double> removedFromDisc, Func<double, double, double> removedAtHeight = null)
        {
            const int steps = 2000000;
            double h = PointedH / steps, sum = 0.0;
            for (int i = 0; i < steps; i++)
            {
                double z = (i + 0.5) * h;
                double r = PointedR * (1.0 - z / PointedH);
                double removed = removedAtHeight != null ? removedAtHeight(z, r) : removedFromDisc(r);
                sum += (Math.PI * r * r - removed) * h;
            }
            return sum;
        }

        /// <summary>The circular segment of the disc of radius r beyond x = BoxCut.</summary>
        private static double BoxCutSection(double r)
            => r <= BoxCut ? 0.0 : r * r * Math.Acos(BoxCut / r) - BoxCut * Math.Sqrt(r * r - BoxCut * BoxCut);

        /// <summary>
        /// The bore runs along y, so at height z it removes the band |x| &lt;= w of the disc, with w the half width
        /// of the bore at that height.
        /// </summary>
        private static double BoredSection(double z, double r)
        {
            double dz = z - BoreHeight;
            if (Math.Abs(dz) >= BoreRadius) return 0.0;
            double w = Math.Sqrt(BoreRadius * BoreRadius - dz * dz);
            if (w >= r) return Math.PI * r * r;
            return 2.0 * (w * Math.Sqrt(r * r - w * w) + r * r * Math.Asin(w / r));
        }

        /// <summary>
        /// Analytic volume of the toroidal segment, taking the sweep angle from the model itself (max u-range over the
        /// toroidal faces). A full torus has volume 2*pi^2*R*r^2; a segment of angle a is (a / 2pi) of that = a*pi*R*r^2.
        /// </summary>
        private static double TorusSegmentVolume(Solid torus)
        {
            ToroidalSurface ts = torus.Shells.SelectMany(sh => sh.Faces)
                .Select(f => f.Surface).OfType<ToroidalSurface>().First();
            double sweep = torus.Shells.SelectMany(sh => sh.Faces)
                .Where(f => f.Surface is ToroidalSurface)
                .Max(f => { BoundingRect uv = f.GetUVBounds(); return uv.Right - uv.Left; });
            return sweep * Math.PI * ts.MajorRadius * ts.MinorRadius * ts.MinorRadius;
        }

        private static bool HasSurface<T>(Solid solid) where T : ISurface
            => solid.Shells.Any(sh => sh.Faces.Any(f => f.Surface is T));
    }
}
