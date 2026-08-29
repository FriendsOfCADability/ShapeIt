using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using CADability.GeoObject;
using CADability.Tests.BRep;
using ShapeIt;
using Path = System.IO.Path;

namespace CADability.Tests
{
    /// <summary>
    /// Regression harness for the BRep operations (union / difference / intersection / round / chamfer).
    /// <para>
    /// Every *.cdb.json in Files/BRep is one case: the operands and the operation are marked inside the project
    /// itself (see <see cref="BRepCaseReader"/>), so the very same file can be opened in ShapeIt for debugging -
    /// with the same objects and the same hash codes, which is what makes conditional breakpoints usable.
    /// </para>
    /// <para>
    /// The result is not compared object by object (that would fail on every harmless re-parametrization) but
    /// through the invariants collected in <see cref="BRepSummary"/>, against a committed baseline next to the
    /// file. Files/BRep/cases.json says what is expected of each case; without that classification the suite
    /// would be permanently red, since most of these files exist because something is broken.
    /// </para>
    /// <para>Environment variables: <c>BREP_REGEN=1</c> rewrites all baselines, <c>BREP_CASE=&lt;name&gt;</c>
    /// restricts the run to a single case.</para>
    /// </summary>
    [TestClass]
    public class BRepRegressionTests
    {
        public TestContext TestContext { get; set; } = null!;

        // Both switches can be set in cases.json ("Run": { "Only": ..., "Regenerate": ... }), which is what makes
        // them usable from the Visual Studio test explorer, or as an environment variable for a command line run.
        private static bool Regenerate
            => Environment.GetEnvironmentVariable("BREP_REGEN") == "1" || manifest.Run.Regenerate;
        private static string? SingleCase
        {
            get
            {
                string? fromEnvironment = Environment.GetEnvironmentVariable("BREP_CASE");
                return string.IsNullOrWhiteSpace(fromEnvironment) ? manifest.Run.Only : fromEnvironment;
            }
        }

        private static string BRepDir([CallerFilePath] string thisFile = "")
            => Path.Combine(Path.GetDirectoryName(thisFile)!, "Files", "BRep");
        private static string BaselineDir() => Path.Combine(BRepDir(), "Baselines");
        private static string ManifestPath() => Path.Combine(BRepDir(), "cases.json");

        private static string[] CaseFiles()
        {
            string dir = BRepDir();
            if (!Directory.Exists(dir)) return Array.Empty<string>();
            IEnumerable<string> files = Directory.GetFiles(dir, "*.cdb.json").OrderBy(f => f, StringComparer.OrdinalIgnoreCase);
            if (!string.IsNullOrEmpty(SingleCase))
                files = files.Where(f => string.Equals(BRepCaseReader.CaseName(f), SingleCase, StringComparison.OrdinalIgnoreCase));
            return files.ToArray();
        }

        // Parsing all files takes a moment, so the two inspection tests share one pass. The regression test
        // re-reads every case right before running it, because the operations consume their operands.
        private static List<BRepCase>? parsedCases;
        private static BRepCaseManifest manifest = new BRepCaseManifest();

        [ClassInitialize]
        public static void ReadAllCases(TestContext context)
        {
            manifest = BRepCaseManifest.Load(ManifestPath());
            parsedCases = new List<BRepCase>();
            foreach (string file in CaseFiles())
            {
                CaseEntry entry = manifest.Get(BRepCaseReader.CaseName(file));
                parsedCases.Add(BRepCaseReader.Read(file, entry.Operation, entry.Parameter ?? double.NaN, entry.SecondaryParameter ?? double.NaN));
            }
        }

        /// <summary>
        /// The two manual switches in cases.json have to be off. Both change what the suite means - "Regenerate"
        /// makes it rewrite its own expectations instead of checking them, "Only" makes it look at a single case -
        /// so leaving one on by accident would turn a green run into a meaningless one. This test is the reminder
        /// and, since it is part of every run, also the safety net against committing them.
        /// </summary>
        [TestMethod]
        public void BaselineSwitchesAreTurnedOff()
        {
            List<string> active = new List<string>();
            if (manifest.Run.Regenerate)
                active.Add("\"Regenerate\": true - the baselines are being overwritten instead of checked. "
                    + "Review the changed files under Files/BRep/Baselines and set it back to false.");
            if (!string.IsNullOrWhiteSpace(manifest.Run.Only))
                active.Add($"\"Only\": \"{manifest.Run.Only}\" - all other cases are being skipped. "
                    + "Set it back to \"\" to check the whole suite again.");
            if (active.Count > 0)
                Assert.Fail("cases.json is still in manual mode:\n  " + string.Join("\n  ", active));
        }

        /// <summary>
        /// Every file must be understandable by the harness. A file that silently does nothing is worse than no
        /// file at all - that is exactly how the old "AutoDebug" mechanism could quietly do nothing on a typo.
        /// Files that do not follow the convention yet must be marked in cases.json.
        /// </summary>
        [TestMethod]
        public void AllFilesFollowTheConvention()
        {
            List<BRepCase> cases = RequireCases();
            StringBuilder report = new StringBuilder().AppendLine();
            List<string> unexpectedlyBroken = new List<string>();
            List<string> unexpectedlyFine = new List<string>();

            foreach (BRepCase testCase in cases)
            {
                CaseEntry entry = manifest.Get(testCase.Name);
                string parameter = double.IsNaN(testCase.Parameter) ? "" : " " + BRepSummary.Format(testCase.Parameter);
                report.AppendLine($"{testCase.Name,-22} {entry.Status,-12} {testCase.Operation}{parameter}"
                    + $"  operands={testCase.Operands.Count} markedEdges={testCase.MarkedEdges.Count}");
                foreach (string problem in testCase.Problems) report.AppendLine($"      problem: {problem}");
                foreach (string warning in testCase.Warnings) report.AppendLine($"      warning: {warning}");

                bool mayBeBroken = entry.Status == CaseStatus.NeedsFixup || entry.Status == CaseStatus.Skip;
                if (!testCase.IsRunnable && !mayBeBroken)
                    unexpectedlyBroken.Add($"{testCase.Name} [{entry.Status}]: {string.Join("; ", testCase.Problems)}");
                if (testCase.IsRunnable && entry.Status == CaseStatus.NeedsFixup)
                    unexpectedlyFine.Add(testCase.Name);
            }
            Write(report.ToString());

            List<string> failures = new List<string>();
            if (unexpectedlyBroken.Count > 0)
                failures.Add($"{unexpectedlyBroken.Count} file(s) do not follow the convention and are not marked "
                    + "\"NeedsFixup\" in cases.json:\n  " + string.Join("\n  ", unexpectedlyBroken));
            if (unexpectedlyFine.Count > 0)
                failures.Add($"{unexpectedlyFine.Count} file(s) marked \"NeedsFixup\" are fine now, please set their "
                    + "status in cases.json: " + string.Join(", ", unexpectedlyFine));
            if (failures.Count > 0) Assert.Fail(string.Join("\n\n", failures) + "\n" + report);
        }

        /// <summary>
        /// A case whose operands are already inconsistent is not a BRep regression test - it is an import or
        /// authoring bug. Such files are classified as "CorruptInput" so they do not poison the suite, but a
        /// newly broken input has to show up.
        /// </summary>
        [TestMethod]
        public void AllInputsAreConsistent()
        {
            List<BRepCase> cases = RequireCases();
            StringBuilder report = new StringBuilder().AppendLine();
            List<string> newlyCorrupt = new List<string>();
            List<string> noLongerCorrupt = new List<string>();

            foreach (BRepCase testCase in cases)
            {
                if (testCase.Operands.Count == 0) continue;
                CaseEntry entry = manifest.Get(testCase.Name);
                List<string> broken = new List<string>();
                for (int i = 0; i < testCase.Operands.Count; i++)
                {
                    bool consistent;
                    try { consistent = testCase.Operands[i].CheckConsistency(); }
                    catch (Exception e) { consistent = false; broken.Add($"operand{i + 1} threw {e.GetType().Name}"); continue; }
                    if (!consistent) broken.Add($"operand{i + 1} is inconsistent");
                }
                report.AppendLine($"{testCase.Name,-22} {(broken.Count == 0 ? "input ok" : string.Join(", ", broken))}");

                if (broken.Count > 0 && entry.Status != CaseStatus.CorruptInput && entry.Status != CaseStatus.Skip)
                    newlyCorrupt.Add($"{testCase.Name} [{entry.Status}]: {string.Join(", ", broken)}");
                if (broken.Count == 0 && entry.Status == CaseStatus.CorruptInput)
                    noLongerCorrupt.Add(testCase.Name);
            }
            Write(report.ToString());

            List<string> failures = new List<string>();
            if (newlyCorrupt.Count > 0)
                failures.Add($"{newlyCorrupt.Count} case(s) have inconsistent input geometry and are not marked "
                    + "\"CorruptInput\" in cases.json:\n  " + string.Join("\n  ", newlyCorrupt));
            if (noLongerCorrupt.Count > 0)
                failures.Add($"{noLongerCorrupt.Count} case(s) marked \"CorruptInput\" have consistent input now, "
                    + "please update cases.json: " + string.Join(", ", noLongerCorrupt));
            if (failures.Count > 0) Assert.Fail(string.Join("\n\n", failures) + "\n" + report);
        }

        /// <summary>
        /// The actual regression test: run every runnable case and compare its summary against the baseline.
        /// </summary>
        [TestMethod]
        public void AllCasesMatchBaseline()
        {
            string[] files = CaseFiles();
            if (files.Length == 0) Assert.Inconclusive($"no *.cdb.json in {BRepDir()}");
            Directory.CreateDirectory(BaselineDir());

            StringBuilder report = new StringBuilder().AppendLine();
            List<string> generated = new List<string>();
            List<string> mismatches = new List<string>();
            List<string> invalidResults = new List<string>();
            List<string> unexpectedSuccesses = new List<string>();
            List<string> unstable = new List<string>();
            List<string> verifiedMismatches = new List<string>();
            List<string> regeneratedVerified = new List<string>();

            foreach (string file in files)
            {
                string name = BRepCaseReader.CaseName(file);
                CaseEntry entry = manifest.Get(name);
                if (entry.Status == CaseStatus.Skip || entry.Status == CaseStatus.CorruptInput || entry.Status == CaseStatus.NeedsFixup)
                {
                    report.AppendLine($"{name,-22} skipped ({entry.Status})");
                    continue;
                }

                // read fresh: the operations modify their operands
                BRepCase testCase = BRepCaseReader.Read(file, entry.Operation, entry.Parameter ?? double.NaN, entry.SecondaryParameter ?? double.NaN);
                if (!testCase.IsRunnable)
                {
                    // AllFilesFollowTheConvention reports this in detail; do not fail twice
                    report.AppendLine($"{name,-22} not runnable: {string.Join("; ", testCase.Problems)}");
                    continue;
                }

                int timeout = entry.TimeoutSeconds ?? manifest.Defaults.TimeoutSeconds;
                double tolerance = entry.RelativeTolerance ?? manifest.Defaults.RelativeTolerance;
                BRepRunResult run = BRepRunner.Run(testCase, timeout);
                BRepSummary summary = BRepSummary.Describe(testCase, run);
                bool valid = run.IsValid;

                // "Repeat" runs the very same case again in the same process. Both the operation and the summary
                // are supposed to be a function of the input file alone, so a repeat has to produce the same
                // fingerprint. Where it does not, the case is not a regression test but a coin toss: comparing it
                // against a baseline then says nothing, and whichever outcome the baseline happened to catch will
                // look like a regression the next time. Worth setting on the cases that have been seen to move.
                int repeat = Math.Max(1, entry.Repeat ?? 1);
                List<string> unstableHere = new List<string>();
                // compare through the text form, so a repeat is held against exactly what would be written to the
                // baseline file - not against some internal state the file would never have carried
                Dictionary<string, string> firstRun = repeat > 1 ? BRepSummary.ParseText(summary.ToText()) : null!;
                for (int again = 1; again < repeat; again++)
                {   // read fresh again, the operation consumed the operands of the previous run
                    BRepCase repeated = BRepCaseReader.Read(file, entry.Operation, entry.Parameter ?? double.NaN, entry.SecondaryParameter ?? double.NaN);
                    BRepSummary repeatedSummary = BRepSummary.Describe(repeated, BRepRunner.Run(repeated, timeout));
                    foreach (string difference in repeatedSummary.DiffAgainst(firstRun, tolerance, repeated.Scale))
                        unstableHere.Add($"run {again + 1} of {repeat}: {difference}");
                }
                if (unstableHere.Count > 0)
                    unstable.Add($"{name}:\n    " + string.Join("\n    ", unstableHere));

                string baselinePath = Path.Combine(BaselineDir(), name + ".txt");
                string oldBaseline = File.Exists(baselinePath) ? File.ReadAllText(baselinePath) : "";
                bool isVerified = BRepSummary.IsVerified(oldBaseline);

                report.AppendLine($"{name,-22} {entry.Status,-10} {(isVerified ? "verified" : ""),-9} {run.Status,-9} "
                    + $"shells={run.Shells.Length} valid={valid} {run.ElapsedMilliseconds} ms"
                    + (repeat > 1 ? unstableHere.Count > 0 ? $"  UNSTABLE over {repeat} runs" : $"  stable over {repeat} runs" : "")
                    + (run.Error != null ? "  " + run.Error.GetType().Name + ": " + BRepSummary.FirstLine(run.Error.Message) : ""));

                if (Regenerate || oldBaseline.Length == 0)
                {
                    // A baseline taken from a case that does not reproduce would just freeze one of its outcomes
                    // and make the next run look like a regression, so refuse to write it. The instability is
                    // reported below and fails the run anyway.
                    if (unstableHere.Count == 0)
                    {
                        // keep the hand written comments - that is where the "# verified" note lives
                        File.WriteAllText(baselinePath, BRepSummary.ExtractComments(oldBaseline) + summary.ToText());
                        generated.Add(name);
                        if (isVerified) regeneratedVerified.Add(name);
                    }
                }
                else
                {
                    List<string> diff = summary.DiffAgainst(BRepSummary.ParseText(oldBaseline), tolerance, testCase.Scale);
                    if (diff.Count > 0)
                    {
                        string header = isVerified
                            ? $"{name} (this baseline is marked as verified, so a difference is a regression, not just a change):"
                            : $"{name}:";
                        (isVerified ? verifiedMismatches : mismatches).Add(header + "\n    " + string.Join("\n    ", diff));
                    }
                }

                if (entry.Status == CaseStatus.Ok && !valid)
                    invalidResults.Add($"{name}: {run.Describe()}");
                if (entry.Status == CaseStatus.KnownFail && valid)
                    unexpectedSuccesses.Add(name);
            }
            Write(report.ToString());

            List<string> failures = new List<string>();
            // instability first: for a case that does not reproduce, every other statement below - baseline
            // matched, baseline differs, result valid - is about one throw of the dice and means nothing
            if (unstable.Count > 0)
                failures.Add($"{unstable.Count} case(s) marked \"Repeat\" did not reproduce within this run. "
                    + "Their baselines are not written and not meaningful until this is fixed:\n  "
                    + string.Join("\n  ", unstable));
            // then the verified baselines: those results were judged correct by hand, so a difference there is
            // the most serious thing this test can report
            if (verifiedMismatches.Count > 0)
                failures.Add($"{verifiedMismatches.Count} VERIFIED baseline(s) changed:\n  "
                    + string.Join("\n  ", verifiedMismatches));
            if (invalidResults.Count > 0)
                failures.Add($"{invalidResults.Count} case(s) expected to work produced no valid result:\n  "
                    + string.Join("\n  ", invalidResults));
            if (unexpectedSuccesses.Count > 0)
                failures.Add($"{unexpectedSuccesses.Count} case(s) marked \"KnownFail\" succeed now - please promote "
                    + "them to \"Ok\" in cases.json: " + string.Join(", ", unexpectedSuccesses));
            if (mismatches.Count > 0)
                failures.Add($"{mismatches.Count} case(s) differ from their baseline (judge whether this is an "
                    + "improvement; if so, regenerate with BREP_REGEN=1):\n  " + string.Join("\n  ", mismatches));
            if (failures.Count > 0) Assert.Fail(string.Join("\n\n", failures) + "\n" + report);
            if (generated.Count > 0)
            {
                string warning = regeneratedVerified.Count == 0 ? "" :
                    $"\nCAUTION: {regeneratedVerified.Count} of them were marked as verified and have just been "
                    + "overwritten - check that the new values are still correct: " + string.Join(", ", regeneratedVerified);
                Assert.Inconclusive($"generated {generated.Count} baseline(s), please review and commit: "
                    + string.Join(", ", generated) + warning + "\n" + report);
            }
        }

        // ---- helpers ---------------------------------------------------------------------------------

        private List<BRepCase> RequireCases()
        {
            if (parsedCases == null || parsedCases.Count == 0) Assert.Inconclusive($"no *.cdb.json in {BRepDir()}");
            return parsedCases!;
        }

        private void Write(string text)
        {
            TestContext.WriteLine(text);
            System.Diagnostics.Trace.WriteLine(text);
        }
    }
}
