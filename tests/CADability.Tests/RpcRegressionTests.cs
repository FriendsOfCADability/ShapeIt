using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.Tests.Rpc;
using ShapeIt;
using Path = System.IO.Path;

namespace CADability.Tests
{
    /// <summary>
    /// Regression harness for the RPC case files in Files/RPC. Every *.json there except run.json is one case:
    /// a list of JSON-RPC calls as an MCP client would send them, plus the recorded invariants of the results.
    /// <para>
    /// Unlike <see cref="BRepRegressionTests"/> this is not limited to the boolean operations - anything the
    /// toolset can do is a case, and a real session that went wrong becomes a regression test by copying its
    /// calls into a file. See Files/RPC/readme.md for the format and the workflow.
    /// </para>
    /// <para>
    /// Each case is its own test, so the test explorer names the failing file and it can be re-run alone. The
    /// two switches for a manual run live in Files/RPC/run.json, or in the environment variables
    /// <c>RPC_CASE=&lt;name&gt;</c> and <c>RPC_REGEN=1</c>.
    /// </para>
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class RpcRegressionTests
    {
        public TestContext TestContext { get; set; } = null!;

        private const double DefaultRelativeTolerance = 1e-4;

        // The source directory, not the output directory: regenerating has to write the case files that are
        // under version control, the same way BRepRegressionTests writes its baselines.
        private static string RpcDir([CallerFilePath] string thisFile = "")
            => Path.Combine(Path.GetDirectoryName(thisFile)!, "Files", "RPC");
        private static string RunOptionsPath() => Path.Combine(RpcDir(), "run.json");

        private static bool Regenerate
            => Environment.GetEnvironmentVariable("RPC_REGEN") == "1" || RunSwitches().Regenerate;

        private static string SingleCase
        {
            get
            {
                string? fromEnvironment = Environment.GetEnvironmentVariable("RPC_CASE");
                return string.IsNullOrWhiteSpace(fromEnvironment) ? RunSwitches().Only : fromEnvironment;
            }
        }

        private sealed class RunOptions
        {
            public string Only { get; set; } = "";
            public bool Regenerate { get; set; }
        }

        private static RunOptions RunSwitches()
        {
            string path = RunOptionsPath();
            if (!File.Exists(path)) return new RunOptions();
            try
            {
                return JsonSerializer.Deserialize<RunOptions>(File.ReadAllText(path), new JsonSerializerOptions
                {
                    PropertyNameCaseInsensitive = true,
                    ReadCommentHandling = JsonCommentHandling.Skip,
                    AllowTrailingCommas = true
                }) ?? new RunOptions();
            }
            catch { return new RunOptions(); }
        }

        /// <summary>Every *.json except run.json. A file that is not a valid case is reported by
        /// <see cref="AllFilesAreUsable"/>, never silently skipped.</summary>
        private static string[] CaseFiles()
        {
            string dir = RpcDir();
            if (!Directory.Exists(dir)) return Array.Empty<string>();
            return Directory.GetFiles(dir, "*.json")
                .Where(f => !string.Equals(Path.GetFileName(f), "run.json", StringComparison.OrdinalIgnoreCase))
                .OrderBy(f => f, StringComparer.OrdinalIgnoreCase)
                .ToArray();
        }

        /// <summary>Feeds one test per case file. Enumerated at discovery time, so it must not depend on
        /// ClassInitialize.</summary>
        public static IEnumerable<object[]> CaseNames
            => CaseFiles().Select(f => new object[] { Path.GetFileNameWithoutExtension(f) });

        private void Write(string text) => TestContext.WriteLine(text);

        /// <summary>
        /// Leaving a switch on would quietly turn the suite into something else - a run that rewrites its own
        /// expectations, or one that looks at a single case. This test makes that impossible to forget and
        /// impossible to commit unnoticed.
        /// </summary>
        [TestMethod]
        public void RunSwitchesAreTurnedOff()
        {
            RunOptions switches = RunSwitches();
            List<string> active = new List<string>();
            if (switches.Regenerate)
                active.Add("\"Regenerate\": true - the baselines are being overwritten instead of checked. "
                    + "Review the changed case files and set it back to false.");
            if (!string.IsNullOrWhiteSpace(switches.Only))
                active.Add($"\"Only\": \"{switches.Only}\" - all other cases are being skipped. "
                    + "Set it back to \"\" to check the whole suite again.");
            if (active.Count > 0)
                Assert.Fail("Files/RPC/run.json is still in manual mode:\n  " + string.Join("\n  ", active));
        }

        /// <summary>
        /// Every file has to be a case the harness understands, and a case that is expected to pass has to have
        /// a recorded baseline. A file that runs but checks nothing is the failure mode this guards against.
        /// </summary>
        [TestMethod]
        public void AllFilesAreUsable()
        {
            string[] files = CaseFiles();
            if (files.Length == 0) Assert.Inconclusive($"no case files in {RpcDir()}");

            StringBuilder report = new StringBuilder().AppendLine();
            List<string> broken = new List<string>();
            List<string> withoutBaseline = new List<string>();

            foreach (string file in files)
            {
                RpcCase testCase = RpcCase.Read(file);
                report.AppendLine($"{testCase.Name,-34} {testCase.Status,-10} calls={testCase.CallCount,-3} "
                    + $"repeat={testCase.Repeat} verify={testCase.Verify.Length} baseline={testCase.Baseline.Count}");
                foreach (string problem in testCase.Problems) report.AppendLine($"      problem: {problem}");

                if (!testCase.IsRunnable && testCase.Status != RpcCaseStatus.Skip)
                    broken.Add($"{testCase.Name}: {string.Join("; ", testCase.Problems)}");
                if (testCase.IsRunnable && testCase.Status == RpcCaseStatus.Ok && testCase.Baseline.Count == 0)
                    withoutBaseline.Add(testCase.Name);
            }
            Write(report.ToString());

            List<string> failures = new List<string>();
            if (broken.Count > 0)
                failures.Add($"{broken.Count} file(s) are not usable as a case and are not marked \"Skip\":\n  "
                    + string.Join("\n  ", broken));
            if (withoutBaseline.Count > 0)
                failures.Add($"{withoutBaseline.Count} case(s) have status \"Ok\" but no recorded baseline, so they "
                    + "check nothing. Record one with \"Regenerate\": true, or set the status to \"KnownFail\": "
                    + string.Join(", ", withoutBaseline));
            if (failures.Count > 0) Assert.Fail(string.Join("\n\n", failures) + "\n" + report);
        }

        /// <summary>
        /// The actual regression test: run one case and compare its result against the recorded baseline.
        /// </summary>
        [DataTestMethod]
        [DynamicData(nameof(CaseNames))]
        public void CaseMatchesBaseline(string caseName)
        {
            if (!string.IsNullOrEmpty(SingleCase) && !string.Equals(caseName, SingleCase, StringComparison.OrdinalIgnoreCase))
                Assert.Inconclusive($"skipped: run.json restricts the run to \"{SingleCase}\"");

            string file = Path.Combine(RpcDir(), caseName + ".json");
            RpcCase testCase = RpcCase.Read(file);
            if (testCase.Status == RpcCaseStatus.Skip) Assert.Inconclusive("case is marked \"Skip\"");
            if (!testCase.IsRunnable) Assert.Fail($"{caseName} is not usable: {string.Join("; ", testCase.Problems)}");

            StringBuilder report = new StringBuilder().AppendLine();
            List<RpcRunResult> runs = new List<RpcRunResult>();
            for (int i = 0; i < testCase.Repeat; i++)
            {
                RpcRunResult run = RpcRunner.Run(testCase);
                runs.Add(run);
                report.AppendLine($"run {i + 1}/{testCase.Repeat}: {run.Calls.Count} calls, "
                    + $"{run.Objects.Count} object(s), {run.ElapsedMilliseconds} ms"
                    + (run.Crash != null ? $", CRASH {run.Crash.GetType().Name}: {BRepSummary.FirstLine(run.Crash.Message)}" : ""));
            }

            // Every repeat has to produce the same fingerprint. Without this a case that fails one run in five
            // would pass whenever it happened to be lucky - which is exactly how two defects in this corpus
            // stayed invisible until they were run repeatedly.
            List<string> unstable = UnstableFields(runs, testCase.RelativeTolerance ?? DefaultRelativeTolerance);
            foreach (string line in unstable) report.AppendLine("  unstable: " + line);

            List<string> problems = new List<string>();
            RpcRunResult first = runs[0];
            if (first.Crash != null) problems.Add($"the run threw {first.Crash.GetType().Name}: {first.Crash.Message}");
            foreach ((int id, string method, string error) in first.Failures)
                if (!RpcRunner.IsExpected(testCase, id, error))
                    problems.Add($"call id {id} ({method}) failed: {error}");
            foreach (ExpectedError expected in testCase.ExpectError)
                if (!first.Failures.Any(f => f.Id == expected.Id))
                    problems.Add($"call id {expected.Id} was declared in ExpectError but succeeded");

            if (Regenerate)
            {
                if (unstable.Count > 0)
                    Assert.Fail($"{caseName}: refusing to record a baseline, the case is not stable over "
                        + $"{testCase.Repeat} run(s):\n  " + string.Join("\n  ", unstable)
                        + "\nFix the instability first, or raise \"Repeat\" and look at what moves." + report);
                if (problems.Count > 0)
                    Assert.Fail($"{caseName}: refusing to record a baseline, the run is not clean:\n  "
                        + string.Join("\n  ", problems) + report);
                bool written = WriteBaseline(testCase, first, out string summary);
                Write(report.ToString());
                Assert.Inconclusive($"{caseName}: baseline {(written ? "written" : "unchanged")} - {summary}. "
                    + "Review the diff, add a \"verified\" note and set \"Regenerate\" back to false.");
            }

            problems.AddRange(unstable.Select(u => "unstable between runs: " + u));
            List<string> baselineDiffs = CompareAgainstBaseline(testCase, first, report);

            Write(report.ToString());
            if (testCase.Status == RpcCaseStatus.KnownFail)
            {
                // A known failure is pinned by its baseline like any other case - the baseline records the
                // WRONG result on purpose. So "it matches" does not mean the case passes, it means nothing
                // has changed; and a difference is the interesting event, whether it is the fix or a new
                // defect on top. Reporting a match as "passes now" would be exactly backwards.
                // A known failure without a baseline is not pinned to anything - that is the honest record for
                // a case whose result is not reproducible in the first place, where a baseline would only
                // produce noise. Nothing to compare, so nothing to report.
                if (testCase.Baseline.Count > 0 && baselineDiffs.Count > 0)
                    Assert.Fail($"{caseName} is marked \"KnownFail\" and its recorded result CHANGED. If this is "
                        + "the fix, record the new result with \"Regenerate\" and set \"CaseStatus\" to \"Ok\"; "
                        + "if not, something else moved:\n  " + string.Join("\n  ", baselineDiffs) + report);
                Assert.Inconclusive($"{caseName} is a known failure and is unchanged"
                    + (problems.Count > 0 ? $" ({problems.Count} problem(s)):\n  " + string.Join("\n  ", problems) : "."));
            }
            problems.AddRange(baselineDiffs);
            if (problems.Count > 0)
                Assert.Fail($"{caseName}:\n  " + string.Join("\n  ", problems) + report);
        }

        /// <summary>
        /// Fields that differ between the repeats of one case, compared with the same tolerance the baseline
        /// comparison will use. Not an exact comparison: volume and area come from a triangulation and are not
        /// bit-reproducible on curved faces - two runs of the same sphere differ by about 1e-7 relative. What
        /// this gate has to catch are the categorical flips (a part appearing or disappearing, a count or a
        /// surface histogram changing), and those are strings, compared exactly whatever the tolerance is.
        /// <para>
        /// Using the same tolerance is what makes the gate meaningful: whatever it accepts, the next normal run
        /// accepts too - it can neither write a baseline that immediately fails nor refuse a sound one.
        /// </para>
        /// </summary>
        private static List<string> UnstableFields(List<RpcRunResult> runs, double tolerance)
        {
            List<string> unstable = new List<string>();
            if (runs.Count < 2) return unstable;
            RpcRunResult first = runs[0];
            for (int i = 1; i < runs.Count; i++)
            {
                RpcRunResult other = runs[i];
                foreach (string name in first.Objects.Keys.Union(other.Objects.Keys, StringComparer.Ordinal))
                {
                    if (!first.Objects.TryGetValue(name, out RpcObjectSummary? a) || !other.Objects.TryGetValue(name, out RpcObjectSummary? b))
                    {
                        unstable.Add($"\"{name}\" exists in run 1 or run {i + 1} but not in the other");
                        continue;
                    }
                    foreach (string line in b.Summary.DiffAgainst(a.Values, tolerance, Math.Max(a.Scale, b.Scale)))
                        unstable.Add($"{name}.{line} (run 1 vs run {i + 1})");
                }
            }
            return unstable.Distinct(StringComparer.Ordinal).ToList();
        }

        private static List<string> CompareAgainstBaseline(RpcCase testCase, RpcRunResult run, StringBuilder report)
        {
            double tolerance = testCase.RelativeTolerance ?? DefaultRelativeTolerance;
            List<string> problems = new List<string>();
            List<string> verifiedProblems = new List<string>();

            foreach (string name in run.Objects.Keys.Union(testCase.Baseline.Keys, StringComparer.Ordinal).OrderBy(n => n, StringComparer.Ordinal))
            {
                bool hasResult = run.Objects.TryGetValue(name, out RpcObjectSummary? actual);
                bool hasBaseline = testCase.Baseline.TryGetValue(name, out Dictionary<string, string>? expected);
                if (!hasBaseline) { report.AppendLine($"  {name}: no baseline (new object)"); problems.Add($"\"{name}\" has no baseline entry"); continue; }
                if (!hasResult) { problems.Add($"\"{name}\" is in the baseline but was not produced"); continue; }

                List<string> diff = actual!.Summary.DiffAgainst(expected!, tolerance, actual.Scale);
                report.AppendLine($"  {name}: {(diff.Count == 0 ? "matches" : diff.Count + " difference(s)")}"
                    + (testCase.IsVerified(name) ? "  [verified]" : ""));
                foreach (string line in diff) report.AppendLine("      " + line);
                // A baseline that was judged correct by hand is more than a snapshot: a difference against it
                // is a regression, not just a change, and is reported first.
                if (diff.Count > 0) (testCase.IsVerified(name) ? verifiedProblems : problems)
                        .Add($"\"{name}\" differs from its {(testCase.IsVerified(name) ? "verified " : "")}baseline:\n      "
                            + string.Join("\n      ", diff));
            }
            verifiedProblems.AddRange(problems);
            return verifiedProblems;
        }

        /// <summary>
        /// Writes the recorded fingerprint into the case file. Everything else in the file is preserved: the
        /// prose fields are what make these files useful while a defect is open, and the "verified" notes are
        /// carried over so a hand written confirmation survives a regenerate.
        /// </summary>
        private static bool WriteBaseline(RpcCase testCase, RpcRunResult run, out string summary)
        {
            JsonObject baseline = new JsonObject();
            foreach (KeyValuePair<string, RpcObjectSummary> item in run.Objects)
            {
                JsonObject fields = new JsonObject();
                if (testCase.VerifiedNotes.TryGetValue(item.Key, out string? note)) fields[RpcCase.VerifiedKey] = note;
                foreach (KeyValuePair<string, string> entry in item.Value.Summary.Entries) fields[entry.Key] = entry.Value;
                baseline[item.Key] = fields;
            }

            JsonObject root = testCase.Root;
            string before = root["Baseline"]?.ToJsonString() ?? "";
            root["Baseline"] = baseline;
            string after = baseline.ToJsonString();
            summary = $"{run.Objects.Count} object(s): {string.Join(", ", run.Objects.Keys)}";
            if (before == after) return false;

            // RPCCalls last, so the recorded result stays readable at the top of the file.
            JsonObject ordered = new JsonObject();
            foreach (KeyValuePair<string, JsonNode?> property in root.ToList())
                if (property.Key != "RPCCalls") ordered[property.Key] = property.Value?.DeepClone();
            ordered["RPCCalls"] = root["RPCCalls"]?.DeepClone();
            File.WriteAllText(testCase.FilePath, ordered.ToJsonString(RpcCase.WriteOptions) + "\n");
            return true;
        }
    }
}
