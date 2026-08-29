using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using CADability.Tests.Rpc;

namespace ShapeIt.Profiler
{
    /// <summary>
    /// Headless host for profiling a single RPC case.
    /// <para>
    /// Profiling the application itself is close to useless: the message loop, the rendering and the idle time
    /// drown out the geometry, and the interesting work is spread over user interactions that are never
    /// repeated identically. This host runs one case, no window and no message loop, as often as asked, so
    /// every sample a profiler takes belongs to the algorithm and two runs can be compared.
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

            string? path = ResolveCase(options.CaseArgument);
            if (path == null)
            {
                Console.Error.WriteLine($"no case \"{options.CaseArgument}\": not a file, and not found in a Files\\RPC directory above this executable.");
                ListCases();
                return 2;
            }

            RpcCase testCase = RpcCase.Read(path);
            if (!testCase.IsRunnable)
            {
                Console.Error.WriteLine($"{testCase.Name} cannot be run:");
                foreach (string problem in testCase.Problems) Console.Error.WriteLine("  " + problem);
                return 3;
            }

            return Profile(testCase, options);
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
            Console.WriteLine(@"ShapeIt.Profiler - runs one RPC case headless, as often as you want.

  ShapeIt.Profiler <case> [-n <count>] [-w <count>] [-s <mb>] [--wait]
  ShapeIt.Profiler --list

  <case>        path to a *.json case, or the bare name of one in tests\CADability.Tests\Files\RPC
  -n <count>    measured iterations, default 5
  -w <count>    warmup iterations, excluded from the result, default 1
  -s <mb>       stack size of the worker thread in MB, default 32
  --wait        run the warmup, then wait for Enter, so a profiler can be attached and started
                exactly around the measured iterations
  --solver-trace  count the calls into the numeric minimizers and report where they come from.
                Slows the run down considerably: every such call walks the stack.
  --list        list the cases that can be named without a path

  Example:
    ShapeIt.Profiler DieWithPips -n 10 --wait");
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
            public string CaseArgument = "";
            public int Iterations = 5;
            public int Warmup = 1;
            public int StackMegabytes = 32;
            public bool Wait;
            public bool List;
            public bool SolverTrace;

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
                        case "--wait":
                            options.Wait = true;
                            break;
                        case "--solver-trace":
                            options.SolverTrace = true;
                            break;
                        case "-n":
                            if (!TakeNumber(args, ref i, out options.Iterations, 1, out error)) return null;
                            break;
                        case "-w":
                            if (!TakeNumber(args, ref i, out options.Warmup, 0, out error)) return null;
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
                            if (options.CaseArgument.Length > 0)
                            {
                                error = "more than one case named; this host runs exactly one.";
                                return null;
                            }
                            options.CaseArgument = argument;
                            break;
                    }
                }
                if (options.List) return options;
                if (options.CaseArgument.Length == 0) { error = "no case named."; return null; }
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
