using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Runtime.CompilerServices;
using CADability.Tests.BRep;
using ShapeIt;
using Path = System.IO.Path;

namespace CADability.Tests
{
    /// <summary>
    /// BooleanOperation.PrecalculateEdgeIntersections computes the intersections of edges with faces in parallel in the release
    /// version and one after the other in the debug version. It is built so that the result does not depend on that: every task
    /// works on private copies of the geometry. This holds it against a few cases with NURBS surfaces and curves of intersections,
    /// where the geometry has the most caches that are filled while it is used: the summary of the result must be exactly the
    /// same, not just within a tolerance.
    /// </summary>
    [TestClass]
    public class ParallelEdgeIntersectionTests
    {
        private static readonly FieldInfo precalculateInParallel = typeof(BooleanOperation).GetField("PrecalculateInParallel",
            BindingFlags.NonPublic | BindingFlags.Static)!;

        private static string BRepDir([CallerFilePath] string thisFile = "")
            => Path.Combine(Path.GetDirectoryName(thisFile)!, "Files", "BRep");

        [DataTestMethod]
        [DataRow("DifferenceTest4")]
        [DataRow("DifferenceBug13")]
        [DataRow("DifferenceBug14")]
        [DataRow("ChamferBug1")]
        public void ParallelAndSerialGiveTheSameResult(string caseName)
        {
            string file = Path.Combine(BRepDir(), caseName + ".cdb.json");
            CaseEntry entry = BRepCaseManifest.Load(Path.Combine(BRepDir(), "cases.json")).Get(caseName);
            bool original = (bool)precalculateInParallel.GetValue(null)!;
            try
            {
                List<string> summaries = new List<string>();
                foreach (bool parallel in new[] { false, true, true })
                {
                    precalculateInParallel.SetValue(null, parallel);
                    // read fresh: the operations modify their operands
                    BRepCase testCase = BRepCaseReader.Read(file, entry.Operation, entry.Parameter ?? double.NaN, entry.SecondaryParameter ?? double.NaN);
                    Assert.IsTrue(testCase.IsRunnable, string.Join("; ", testCase.Problems));
                    summaries.Add(BRepSummary.Describe(testCase, BRepRunner.Run(testCase, 120)).ToText());
                }
                Assert.AreEqual(summaries[0], summaries[1], "the parallel precalculation changed the result");
                Assert.AreEqual(summaries[0], summaries[2], "the parallel precalculation changed the result in the second run");
            }
            finally
            {
                precalculateInParallel.SetValue(null, original);
            }
        }
    }
}
