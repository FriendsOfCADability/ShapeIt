using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading;
using CADability.Tests.Rpc;

namespace ShapeIt.Profiler
{
    /// <summary>
    /// Headless host for profiling RPC cases - one of them, or a whole set.
    /// <para>
    /// Profiling the application itself is close to useless: the message loop, the rendering and the idle time
    /// drown out the geometry, and the interesting work is spread over user interactions that are never
    /// repeated identically. This host runs the cases, no window and no message loop, as often as asked, so
    /// every sample a profiler takes belongs to the algorithm and two runs can be compared.
    /// </para>
    /// <para>
    /// One case gives the detailed report - spread over the iterations, allocation, GC, and the time per call
    /// of that case. A set of them answers where the time of the whole regression suite sits: which cases
    /// dominate it and which operation of the toolset they spend it in.
    /// </para>
    /// </summary>
    internal static class Program
    {
        [STAThread]
        private static int Main(string[] args)
        {
            Options? options = Options.Parse(args, out string parseError);
            if (options == null)
            {
                if (parseError.Length > 0) Console.Error.WriteLine(parseError);
                WriteUsage();
                return parseError.Length > 0 ? 2 : 0;
            }
            if (options.List) { ListCases(); return 0; }

            List<RpcCase> cases = new List<RpcCase>();
            foreach (string argument in options.All ? AllCaseFiles() : options.CaseArguments)
            {
                string? path = ResolveCase(argument);
                if (path == null)
                {
                    Console.Error.WriteLine($"no case \"{argument}\": not a file, and not found in a Files\\RPC directory above this executable.");
                    ListCases();
                    return 2;
                }

                RpcCase testCase = RpcCase.Read(path);
                if (!testCase.IsRunnable || testCase.Status == RpcCaseStatus.Skip)
                {
                    string reason = testCase.Status == RpcCaseStatus.Skip
                        ? "marked Skip" : string.Join("; ", testCase.Problems);
                    // Named explicitly it is an error; in a whole suite it is a line in the report, because one
                    // unusable file must not stop a run that takes minutes.
                    if (!options.All)
                    {
                        Console.Error.WriteLine($"{testCase.Name} cannot be run: {reason}");
                        return 3;
                    }
                    Console.WriteLine($"skipped   {testCase.Name}: {reason}");
                    continue;
                }
                cases.Add(testCase);
            }
            if (cases.Count == 0) { Console.Error.WriteLine("no runnable case named."); return 3; }

            // CADability's module initializer puts MathNet on a single thread for every host. It runs on the
            // first use of that assembly, not at process start, so it has to be forced before the value is
            // read or overridden here - otherwise this reads the runtime default and the override below would
            // be undone again as soon as the first CADability type is touched.
            RuntimeHelpers.RunModuleConstructor(typeof(CADability.Project).Module.ModuleHandle);
            if (options.MathThreads > 0) MathNet.Numerics.Control.MaxDegreeOfParallelism = options.MathThreads;
            Console.WriteLine($"math      MaxDegreeOfParallelism = {MathNet.Numerics.Control.MaxDegreeOfParallelism}"
                + (options.MathThreads > 0 ? "  (overridden)" : "  (as CADability configured it)"));

            // One case keeps the detailed report - spread over the iterations, GC, allocation, per call. A set
            // of cases answers a different question, where the time of the whole suite sits, and is measured
            // once through by default: five iterations of forty cases is an hour.
            if (cases.Count == 1) return Profile(cases[0], options);
            if (!options.IterationsSet) options.Iterations = 1;
            if (!options.WarmupSet) options.Warmup = 0;
            return ProfileSuite(cases, options);
        }

        private static int Profile(RpcCase testCase, Options options)
        {
            Process process = Process.GetCurrentProcess();
            string status = testCase.Status == RpcCaseStatus.Ok ? "" : $"  [{testCase.Status}]";

            Console.WriteLine($"case      {testCase.Name}{status}  {testCase.CallCount} call(s)");
            Console.WriteLine($"file      {testCase.FilePath}");
            Console.WriteLine($"host      .NET {Environment.Version} {(Environment.Is64BitProcess ? "x64" : "x86")}, " +
                              $"ServerGC={System.Runtime.GCSettings.IsServerGC}, " +
                              $"TieredCompilation={Environment.GetEnvironmentVariable("DOTNET_TieredCompilation") ?? "default"}, " +
                              $"pid {process.Id}");
            Console.WriteLine($"plan      {options.Warmup} warmup + {options.Iterations} measured iteration(s), " +
                              $"{options.StackMegabytes} MB stack per run");
            if (testCase.Status == RpcCaseStatus.KnownFail)
                Console.WriteLine("note      this case is marked KnownFail. What is measured here is a run that does not " +
                                  "produce the right result, and a wrong result can be reached on a very different path.");
            if (options.SolverTrace)
            {
                CADability.SolverTrace.Enabled = true;
                Console.WriteLine("note      solver tracing is on: every minimizer call walks the stack, so the times " +
                                  "below are inflated and only good for comparing calls with each other.");
            }
            Console.WriteLine();

            ProfilerEvents.Log.CaseStart(testCase.Name);
            try
            {
                for (int i = 1; i <= options.Warmup; i++)
                {
                    Measurement warmup = RunIteration(testCase, options, "warmup", i, process, null);
                    Console.WriteLine($"warmup {i,-3} {warmup.WallMilliseconds,8} ms");
                }

                if (options.Wait)
                {
                    Console.WriteLine();
                    Console.WriteLine($"attach the profiler to pid {process.Id} and start collecting, then press Enter.");
                    Console.ReadLine();
                }

                // Summed duration per top level call over the measured iterations. This is the breakdown that
                // says which operation of the case is expensive, before any profiler is involved.
                Dictionary<string, long> perCall = new Dictionary<string, long>(StringComparer.Ordinal);
                List<string> callOrder = new List<string>();
                List<Measurement> measurements = new List<Measurement>();

                for (int i = 1; i <= options.Iterations; i++)
                {
                    Measurement measurement = RunIteration(testCase, options, "measured", i, process,
                        (id, method, milliseconds) =>
                        {
                            string key = $"id {id,-3} {method}";
                            if (!perCall.ContainsKey(key)) { perCall[key] = 0; callOrder.Add(key); }
                            perCall[key] += milliseconds;
                        });
                    measurements.Add(measurement);
                    Console.WriteLine($"run    {i,-3} {measurement.WallMilliseconds,8} ms");
                }

                Report(measurements, perCall, callOrder, options.Iterations, process);
                if (options.SolverTrace)
                {
                    // Reset before every iteration, so what is reported here belongs to exactly one run.
                    Console.WriteLine();
                    Console.WriteLine("solver trace (last iteration)");
                    Console.WriteLine();
                    Console.WriteLine(CADability.SolverTrace.Report());
                }
                return measurements[measurements.Count - 1].Result.Crash != null ? 1 : 0;
            }
            finally
            {
                ProfilerEvents.Log.CaseStop(testCase.Name);
            }
        }

        /// <summary>
        /// Runs a set of cases in one process and reports where the time of the whole set sits: which cases
        /// dominate it, and which operation of the toolset they spend it in.
        /// <para>
        /// This is the run to point a sampler at. Profiling <c>dotnet test</c> instead measures the test host,
        /// the JIT of everything it loads, the file I/O of the case files and the baseline comparison along
        /// with the geometry; here the process does nothing else. The per case events of
        /// <see cref="ProfilerEvents"/> are on the timeline, so a trace can be cut down to one case afterwards.
        /// </para>
        /// </summary>
        private static int ProfileSuite(List<RpcCase> cases, Options options)
        {
            Process process = Process.GetCurrentProcess();
            Console.WriteLine($"suite     {cases.Count} case(s), {cases.Sum(c => c.CallCount)} call(s)");
            Console.WriteLine($"host      .NET {Environment.Version} {(Environment.Is64BitProcess ? "x64" : "x86")}, " +
                              $"ServerGC={System.Runtime.GCSettings.IsServerGC}, " +
                              $"TieredCompilation={Environment.GetEnvironmentVariable("DOTNET_TieredCompilation") ?? "default"}, " +
                              $"pid {process.Id}");
            Console.WriteLine($"plan      {options.Warmup} warmup + {options.Iterations} measured iteration(s) per case, " +
                              $"{options.StackMegabytes} MB stack per run");
            if (options.Warmup == 0)
                Console.WriteLine("note      no warmup, so the first case pays for the JIT as well. Read the table as a " +
                                  "ranking; for absolute times give it -w 1.");
            Console.WriteLine();

            // Summed over every case, keyed by the method alone rather than by call id: across a suite the
            // question is which operation is expensive, not which line of one case.
            Dictionary<string, long> perMethod = new Dictionary<string, long>(StringComparer.Ordinal);
            Dictionary<string, int> methodCalls = new Dictionary<string, int>(StringComparer.Ordinal);
            List<CaseResult> results = new List<CaseResult>();
            bool waited = false;

            foreach (RpcCase testCase in cases)
            {
                ProfilerEvents.Log.CaseStart(testCase.Name);
                try
                {
                    for (int i = 1; i <= options.Warmup; i++)
                        RunIteration(testCase, options, "warmup", i, process, null);

                    if (options.Wait && !waited)
                    {
                        Console.WriteLine($"attach the profiler to pid {process.Id} and start collecting, then press Enter.");
                        Console.WriteLine();
                        Console.ReadLine();
                        waited = true;
                    }

                    List<Measurement> measurements = new List<Measurement>();
                    for (int i = 1; i <= options.Iterations; i++)
                        measurements.Add(RunIteration(testCase, options, "measured", i, process,
                            (id, method, milliseconds) =>
                            {
                                perMethod.TryGetValue(method, out long sum);
                                perMethod[method] = sum + milliseconds;
                                methodCalls.TryGetValue(method, out int count);
                                methodCalls[method] = count + 1;
                            }));

                    RpcRunResult last = measurements[measurements.Count - 1].Result;
                    CaseResult result = new CaseResult
                    {
                        Name = testCase.Name,
                        WallMilliseconds = measurements.Average(m => (double)m.WallMilliseconds),
                        AllocatedBytes = measurements.Average(m => (double)m.AllocatedBytes),
                        CpuMilliseconds = measurements.Average(m => m.CpuMilliseconds),
                        Crashed = last.Crash != null,
                        // Only the unexpected ones: a case that declares ExpectError is doing what it is
                        // supposed to, and reporting those as failures would cry wolf on every run.
                        FailedCalls = last.Failures.Count(f => !RpcRunner.IsExpected(testCase, f.Id, f.Error))
                    };
                    results.Add(result);
                    Console.WriteLine(FormattableString.Invariant($"  {result.WallMilliseconds,8:F0} ms  {testCase.Name}")
                        + (result.Crashed ? "  CRASHED" : "")
                        + (result.FailedCalls > 0 ? $"  {result.FailedCalls} unexpected failed call(s)" : ""));
                }
                finally
                {
                    ProfilerEvents.Log.CaseStop(testCase.Name);
                }
            }

            ReportSuite(results, perMethod, methodCalls, options.Iterations, process);
            return results.Any(r => r.Crashed) ? 1 : 0;
        }

        /// <summary>
        /// The two tables that say where to look: the cases by their share of the total, with a running sum so
        /// that "the first n cases are half the suite" can be read off directly, and the toolset operations by
        /// the time spent in them across every case.
        /// </summary>
        private static void ReportSuite(List<CaseResult> results, Dictionary<string, long> perMethod,
            Dictionary<string, int> methodCalls, int iterations, Process process)
        {
            double totalWall = results.Sum(r => r.WallMilliseconds);
            double totalCpu = results.Sum(r => r.CpuMilliseconds);
            double cpuShare = totalWall > 0 ? totalCpu / totalWall * 100.0 : 0.0;
            // Above 100 percent means more than one core was busy: the process time counts every thread, and
            // the run itself is single threaded, so the surplus is the concurrent GC. That is not an error in
            // the measurement, it is the finding - it says the allocation is worth a look before the arithmetic.
            string verdict = cpuShare >= 110 ? "more than one core busy - the surplus is the concurrent GC"
                           : cpuShare >= 90 ? "CPU bound, a CPU profile is meaningful"
                           : "not CPU bound, look at GC and blocking first";

            Console.WriteLine();
            Console.WriteLine(FormattableString.Invariant(
                $"total     wall {totalWall / 1000.0:F1} s   cpu {totalCpu / 1000.0:F1} s ({cpuShare:F0}% - {verdict})"));
            Console.WriteLine($"alloc     {FormatBytes(results.Sum(r => r.AllocatedBytes))} over the whole set");
            process.Refresh();
            Console.WriteLine($"peak ws   {FormatBytes(process.PeakWorkingSet64)}");

            Console.WriteLine();
            Console.WriteLine($"per case (mean of {iterations} iteration(s), share and running sum of the total)");
            double running = 0.0;
            foreach (CaseResult result in results.OrderByDescending(r => r.WallMilliseconds))
            {
                double share = totalWall > 0 ? result.WallMilliseconds / totalWall * 100.0 : 0.0;
                running += share;
                Console.WriteLine(FormattableString.Invariant(
                    $"  {result.WallMilliseconds,8:F0} ms  {share,4:F1}%  {running,5:F1}%  {result.Name}"));
            }

            if (perMethod.Count > 0)
            {
                Console.WriteLine();
                Console.WriteLine($"per method (mean of {iterations} iteration(s))");
                // Every share here is of the same total as the table above, so the two can be read against each
                // other and the last line closes the gap to 100 percent instead of opening a second scale.
                foreach (KeyValuePair<string, long> entry in perMethod.OrderByDescending(e => e.Value))
                {
                    double mean = entry.Value / (double)iterations;
                    double share = totalWall > 0 ? mean / totalWall * 100.0 : 0.0;
                    Console.WriteLine(FormattableString.Invariant(
                        $"  {mean,8:F0} ms  {share,4:F1}%  {methodCalls[entry.Key] / (double)iterations,5:F0} call(s)  {entry.Key}"));
                }
                // What is left is what the harness does around the calls: a fresh project and server per run,
                // and the summaries taken from the result afterwards.
                double unattributed = totalWall - perMethod.Values.Sum() / (double)iterations;
                if (unattributed > 0.01 * totalWall)
                    Console.WriteLine(FormattableString.Invariant(
                        $"  {unattributed,8:F0} ms  {unattributed / totalWall * 100.0,4:F1}%            ")
                        + "project setup and collecting the result");
            }
        }

        /// <summary>
        /// One run, measured. The run itself happens on a thread of its own: the BRep code recurses deeply
        /// enough to overflow a default stack, which is why the regression harness gives it 32 MB as well, and
        /// the named thread is easy to find in a profiler that groups by thread.
        /// </summary>
        private static Measurement RunIteration(RpcCase testCase, Options options, string kind, int iteration,
            Process process, Action<int, string, long>? callObserver)
        {
            // Outside the measured window: without it, collecting what the previous iteration left behind
            // would be charged to this one and the iterations would not be comparable.
            GC.Collect();
            GC.WaitForPendingFinalizers();
            GC.Collect();
            // What the trace reports belongs to one run, not to the sum of all of them.
            if (CADability.SolverTrace.Enabled) CADability.SolverTrace.Reset();

            Measurement measurement = new Measurement();
            int gen0 = GC.CollectionCount(0), gen1 = GC.CollectionCount(1), gen2 = GC.CollectionCount(2);
            long allocated = GC.GetTotalAllocatedBytes(false);
            // Refresh before every read: Process caches part of its state, and a stale value here would show
            // up as a CPU share far below the truth - which is the one number the whole report hinges on.
            process.Refresh();
            TimeSpan cpu = process.TotalProcessorTime;

            ProfilerEvents.Log.IterationStart(kind, iteration);
            Stopwatch watch = Stopwatch.StartNew();
            RpcRunner.CallObserver = callObserver;
            try
            {
                measurement.Result = RunOnWorkerThread(testCase,
                    $"rpc {testCase.Name} {kind} {iteration}", options.StackMegabytes);
            }
            finally { RpcRunner.CallObserver = null; }
            watch.Stop();
            ProfilerEvents.Log.IterationStop(kind, iteration, watch.ElapsedMilliseconds);

            measurement.WallMilliseconds = watch.ElapsedMilliseconds;
            process.Refresh();
            measurement.CpuMilliseconds = (process.TotalProcessorTime - cpu).TotalMilliseconds;
            measurement.AllocatedBytes = GC.GetTotalAllocatedBytes(false) - allocated;
            measurement.Gen0 = GC.CollectionCount(0) - gen0;
            measurement.Gen1 = GC.CollectionCount(1) - gen1;
            measurement.Gen2 = GC.CollectionCount(2) - gen2;
            return measurement;
        }

        private static RpcRunResult RunOnWorkerThread(RpcCase testCase, string threadName, int stackMegabytes)
        {
            RpcRunResult? result = null;
            Exception? failure = null;
            Thread thread = new Thread(() =>
            {
                try { result = RpcRunner.Run(testCase); }
                catch (Exception e) { failure = e; }
            }, stackMegabytes * 1024 * 1024)
            { Name = threadName };
            // The same apartment as the application's main thread, so nothing behaves differently here.
            thread.SetApartmentState(ApartmentState.STA);
            thread.Start();
            // No timeout on purpose: a case that hangs is a finding, and abandoning the thread would hide
            // exactly where it hangs - which is what the profiler is attached for.
            thread.Join();
            if (failure != null) throw new InvalidOperationException("the run threw outside the harness", failure);
            return result!;
        }

        private static void Report(List<Measurement> measurements, Dictionary<string, long> perCall,
            List<string> callOrder, int iterations, Process process)
        {
            long[] wall = measurements.Select(m => m.WallMilliseconds).OrderBy(v => v).ToArray();
            double cpu = measurements.Average(m => m.CpuMilliseconds);
            double meanWall = measurements.Average(m => (double)m.WallMilliseconds);

            Console.WriteLine();
            Console.WriteLine($"wall ms   min {wall[0]}   median {Median(wall)}   mean {meanWall:F0}   max {wall[wall.Length - 1]}");
            // The number that decides whether a CPU profiler can answer the question at all: when the CPU time
            // is well below the wall clock time, the run is waiting - GC, locks, I/O - and a CPU sampler points
            // at whatever happened to be running instead of at the cause.
            double cpuShare = meanWall > 0 ? cpu / meanWall * 100.0 : 0.0;
            string verdict = cpuShare >= 90 ? "CPU bound, a CPU profile is meaningful"
                                            : "not CPU bound, look at GC and blocking first";
            Console.WriteLine($"cpu ms    mean {cpu:F0}  ({cpuShare:F0}% of wall - {verdict})");
            Console.WriteLine($"alloc     {FormatBytes(measurements.Average(m => (double)m.AllocatedBytes))} per iteration");
            Console.WriteLine($"gc        gen0 {measurements.Average(m => m.Gen0):F1}   gen1 {measurements.Average(m => m.Gen1):F1}   " +
                              $"gen2 {measurements.Average(m => m.Gen2):F1}   per iteration");
            process.Refresh();
            Console.WriteLine($"peak ws   {FormatBytes(process.PeakWorkingSet64)}");

            if (callOrder.Count > 0)
            {
                double total = perCall.Values.Sum() / (double)iterations;
                Console.WriteLine();
                Console.WriteLine($"per call (mean of {iterations} iteration(s))");
                foreach (string key in callOrder.OrderByDescending(k => perCall[k]))
                {
                    double mean = perCall[key] / (double)iterations;
                    double share = total > 0 ? mean / total * 100.0 : 0.0;
                    Console.WriteLine($"  {mean,8:F0} ms  {share,3:F0}%  {key}");
                }
            }

            RpcRunResult last = measurements[measurements.Count - 1].Result;
            if (last.Crash != null)
            {
                Console.WriteLine();
                Console.WriteLine("crashed   " + last.Crash.GetType().Name + ": " + last.Crash.Message);
            }
            (int Id, string Method, string Error)[] failures = last.Failures.ToArray();
            if (failures.Length > 0)
            {
                Console.WriteLine();
                Console.WriteLine("failed calls (last run)");
                foreach ((int id, string method, string error) in failures)
                    Console.WriteLine($"  id {id,-3} {method}  {error}");
            }
            if (last.Objects.Count > 0)
            {
                Console.WriteLine();
                Console.WriteLine("result (last run)");
                foreach (KeyValuePair<string, RpcObjectSummary> item in last.Objects)
                    Console.WriteLine($"  {item.Key}: " +
                        string.Join(", ", item.Value.Values.Select(v => v.Key + "=" + v.Value)));
            }
        }

        private static long Median(long[] sorted)
            => sorted.Length % 2 == 1 ? sorted[sorted.Length / 2]
                                      : (sorted[sorted.Length / 2 - 1] + sorted[sorted.Length / 2]) / 2;

        private static string FormatBytes(double bytes)
        {
            string[] units = { "B", "KB", "MB", "GB", "TB" };
            int unit = 0;
            while (bytes >= 1024.0 && unit < units.Length - 1) { bytes /= 1024.0; unit++; }
            return bytes.ToString(unit == 0 ? "F0" : "F2", CultureInfo.InvariantCulture) + " " + units[unit];
        }

        /// <summary>A case is named either by path or by its bare name, which is looked up in the Files\RPC
        /// directory of the repository this executable was built in.</summary>
        private static string? ResolveCase(string argument)
        {
            if (File.Exists(argument)) return Path.GetFullPath(argument);
            string name = argument.EndsWith(".json", StringComparison.OrdinalIgnoreCase) ? argument : argument + ".json";
            foreach (string directory in CaseDirectories())
            {
                string candidate = Path.Combine(directory, name);
                if (File.Exists(candidate)) return candidate;
            }
            return null;
        }

        private static IEnumerable<string> CaseDirectories()
        {
            HashSet<string> seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (string start in new[] { AppContext.BaseDirectory, Environment.CurrentDirectory })
            {
                DirectoryInfo? directory = new DirectoryInfo(start);
                while (directory != null)
                {
                    string candidate = Path.Combine(directory.FullName, "tests", "CADability.Tests", "Files", "RPC");
                    if (Directory.Exists(candidate) && seen.Add(candidate)) yield return candidate;
                    directory = directory.Parent;
                }
            }
        }

        /// <summary>Every case of the nearest Files\RPC directory, run.json excluded - what --all runs.</summary>
        private static IEnumerable<string> AllCaseFiles()
        {
            string? directory = CaseDirectories().FirstOrDefault();
            if (directory == null) return Array.Empty<string>();
            return Directory.GetFiles(directory, "*.json")
                            .Where(f => !string.Equals(Path.GetFileName(f), "run.json", StringComparison.OrdinalIgnoreCase))
                            .OrderBy(f => f, StringComparer.OrdinalIgnoreCase);
        }

        private static void ListCases()
        {
            foreach (string directory in CaseDirectories())
            {
                Console.WriteLine();
                Console.WriteLine(directory);
                foreach (string file in Directory.GetFiles(directory, "*.json")
                                                 .OrderBy(f => f, StringComparer.OrdinalIgnoreCase))
                    Console.WriteLine("  " + Path.GetFileNameWithoutExtension(file));
            }
        }

        private static void WriteUsage()
        {
            Console.WriteLine(@"ShapeIt.Profiler - runs RPC cases headless, as often as you want.

  ShapeIt.Profiler <case>... [-n <count>] [-w <count>] [-s <mb>] [--wait]
  ShapeIt.Profiler --all [-n <count>] [-w <count>] [-s <mb>] [--wait]
  ShapeIt.Profiler --list

  <case>        path to a *.json case, or the bare name of one in tests\CADability.Tests\Files\RPC.
                Name more than one and the report becomes a breakdown per case and per operation.
  --all         every case of that directory; Skip and unusable ones are reported and left out
  -n <count>    measured iterations, default 5 for one case and 1 for a set
  -w <count>    warmup iterations, excluded from the result, default 1 for one case and 0 for a set
  -s <mb>       stack size of the worker thread in MB, default 32
  --wait        run the warmup, then wait for Enter, so a profiler can be attached and started
                exactly around the measured iterations. For a set it waits once, before the first case.
  --solver-trace  count the calls into the numeric minimizers and report where they come from.
                Slows the run down considerably: every such call walks the stack.
  --math-threads <n>  override MathNet.Numerics.Control.MaxDegreeOfParallelism, which CADability
                sets to 1 for every host. Use it to measure what the parallelization costs.
  --list        list the cases that can be named without a path

  Examples:
    ShapeIt.Profiler DieWithPips -n 10 --wait
    ShapeIt.Profiler --all");
        }

        /// <summary>What one case contributed to a suite run, averaged over its iterations.</summary>
        private sealed class CaseResult
        {
            public string Name = "";
            public double WallMilliseconds;
            public double CpuMilliseconds;
            public double AllocatedBytes;
            public bool Crashed;
            public int FailedCalls;
        }

        private sealed class Measurement
        {
            public long WallMilliseconds;
            public double CpuMilliseconds;
            public long AllocatedBytes;
            public int Gen0;
            public int Gen1;
            public int Gen2;
            public RpcRunResult Result = null!;
        }

        private sealed class Options
        {
            public List<string> CaseArguments = new List<string>();
            public int Iterations = 5;
            public int Warmup = 1;
            public int StackMegabytes = 32;
            public bool Wait;
            public bool List;
            public bool All;
            public bool SolverTrace;
            public int MathThreads;
            // Whether the switch was given at all: a single case is measured five times, a whole set once, and
            // an explicit -n or -w has to win over both defaults.
            public bool IterationsSet;
            public bool WarmupSet;

            public static Options? Parse(string[] args, out string error)
            {
                error = "";
                Options options = new Options();
                for (int i = 0; i < args.Length; i++)
                {
                    string argument = args[i];
                    switch (argument)
                    {
                        case "-h":
                        case "-?":
                        case "--help":
                            return null;
                        case "--list":
                            options.List = true;
                            break;
                        case "--all":
                            options.All = true;
                            break;
                        case "--wait":
                            options.Wait = true;
                            break;
                        case "--solver-trace":
                            options.SolverTrace = true;
                            break;
                        case "--math-threads":
                            if (!TakeNumber(args, ref i, out options.MathThreads, 1, out error)) return null;
                            break;
                        case "-n":
                            if (!TakeNumber(args, ref i, out options.Iterations, 1, out error)) return null;
                            options.IterationsSet = true;
                            break;
                        case "-w":
                            if (!TakeNumber(args, ref i, out options.Warmup, 0, out error)) return null;
                            options.WarmupSet = true;
                            break;
                        case "-s":
                            if (!TakeNumber(args, ref i, out options.StackMegabytes, 1, out error)) return null;
                            break;
                        default:
                            if (argument.StartsWith("-", StringComparison.Ordinal))
                            {
                                error = "unknown option " + argument;
                                return null;
                            }
                            options.CaseArguments.Add(argument);
                            break;
                    }
                }
                if (options.List) return options;
                if (options.All && options.CaseArguments.Count > 0)
                {
                    error = "--all runs every case; do not name any in addition.";
                    return null;
                }
                if (!options.All && options.CaseArguments.Count == 0) { error = "no case named."; return null; }
                return options;
            }

            private static bool TakeNumber(string[] args, ref int i, out int value, int minimum, out string error)
            {
                error = "";
                value = 0;
                if (i + 1 >= args.Length) { error = args[i] + " needs a number."; return false; }
                if (!int.TryParse(args[i + 1], NumberStyles.Integer, CultureInfo.InvariantCulture, out value)
                    || value < minimum)
                {
                    error = $"{args[i]} needs a number >= {minimum}, not \"{args[i + 1]}\".";
                    return false;
                }
                i++;
                return true;
            }
        }
    }
}
