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
