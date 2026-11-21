using CADability.GeoObject;
using System.Runtime.CompilerServices;
using IOPath = System.IO.Path;

namespace CADability.Tests
{
    /// <summary>
    /// Export a solid to STEP and import it again. Files/Step/StepImportBug1.cdb.json is a bent channel: tori and
    /// cylinders spanning exactly half a turn, plus pipe surfaces and contact curves exported as NURBS. Many of its
    /// circular edges start exactly on the x-axis of their circle, where PositionOf of the start vertex returns 1
    /// instead of 0. The import trimmed these circles to the complementary arc (2*pi - alpha), so the faces looked as if
    /// they wrapped around the full period: some were split at a seam that does not exist, nine were not imported at all
    /// and the shell was open.
    /// </summary>
    [TestClass]
    public class StepImportRoundTripTests
    {
        private static string StepFile(string name, [CallerFilePath] string thisFile = "")
            => IOPath.Combine(IOPath.GetDirectoryName(thisFile)!, "Files", "Step", name);

        private static IEnumerable<Solid> Solids(GeoObjectList list)
        {
            foreach (IGeoObject go in list)
            {
                if (go is Solid s) yield return s;
                else if (go is Block b) foreach (Solid bs in Solids(b.Children)) yield return bs;
            }
        }

        [TestMethod]
        public void a_solid_with_half_turn_faces_survives_the_step_round_trip()
        {
            Project project = Project.ReadFromFile(StepFile("StepImportBug1.cdb.json"), "cdb");
            Solid original = Solids(project.GetActiveModel().AllObjects).Single();
            string tmp = IOPath.Combine(IOPath.GetTempPath(), "StepImportBug1_" + Guid.NewGuid().ToString("N") + ".stp");
            try
            {
                new ExportStep().WriteToFile(tmp, project);
                ImportStep import = new ImportStep();
                GeoObjectList imported = import.Read(tmp);
                Assert.AreEqual(0, import.Problems.Count, "import problems: " + string.Join("; ", import.Problems.Select(p => $"#{p.Key} {p.Value}")));

                Solid result = Solids(imported).Single();
                Assert.AreEqual(original.Shells.Sum(s => s.Faces.Length), result.Shells.Sum(s => s.Faces.Length), "number of faces");
                foreach (Shell shell in result.Shells)
                {
                    Assert.AreEqual(0, shell.OpenEdges.Length, "open edges");
                    Assert.IsTrue(shell.CheckConsistency(), "consistent shell");
                }
                double precision = original.GetExtent(0.0).Size * 1e-4;
                double v0 = original.Volume(precision);
                double v1 = result.Volume(precision);
                Assert.AreEqual(v0, v1, Math.Abs(v0) * 1e-3, "volume");
            }
            finally
            {
                if (File.Exists(tmp)) File.Delete(tmp);
            }
        }
    }
}
