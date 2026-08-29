using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text.Json;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests.Rpc
{
    /// <summary>The recorded invariants of one workspace object.</summary>
    public sealed class RpcObjectSummary
    {
        public string Name { get; init; } = "";
        public BRepSummary Summary { get; init; } = new BRepSummary();
        /// <summary>Size of the object, used as the absolute floor when comparing floating point values.</summary>
        public double Scale { get; init; } = 1.0;

        public Dictionary<string, string> Values
            => Summary.Entries.ToDictionary(e => e.Key, e => e.Value, StringComparer.Ordinal);
    }

    /// <summary>What one execution of a case produced.</summary>
    public sealed class RpcRunResult
    {
        /// <summary>One entry per call: the id and, when it failed, the error as text.</summary>
        public List<(int Id, string Method, string? Error)> Calls { get; } = new List<(int, string, string?)>();
        /// <summary>The fingerprint, ordered by object name so that a diff is stable.</summary>
        public SortedDictionary<string, RpcObjectSummary> Objects { get; }
            = new SortedDictionary<string, RpcObjectSummary>(StringComparer.Ordinal);
        public Exception? Crash { get; set; }
        public long ElapsedMilliseconds { get; set; }

        public IEnumerable<(int Id, string Method, string Error)> Failures
            => Calls.Where(c => c.Error != null).Select(c => (c.Id, c.Method, c.Error!));
    }

    /// <summary>
    /// Executes an RPC case against a fresh <see cref="MCPServer"/> and collects the invariants of the results.
    /// <para>
    /// No HTTP and no MCP client: the server is constructed directly on a <see cref="Project.CreateSimpleProject"/>,
    /// which is the same path <c>MainForm.DebugRPC</c> takes, so a failing case can be stepped through in the
    /// application with <c>-r &lt;file&gt;</c> - same objects, same hash codes, usable conditional breakpoints.
    /// </para>
    /// </summary>
    public static class RpcRunner
    {
        /// <summary>Recorded for a name that does not exist after the run - the normal record for an operation
        /// whose result is legitimately empty, since nothing is stored under the name then.</summary>
        public const string ExistsKey = "exists";
        /// <summary>Number of parts of a result that is a list of solids.</summary>
        public const string SolidsKey = "solids";

        /// <summary>
        /// Diagnostic seam, called after every top level call with its id, its method and how long it took in
        /// milliseconds. The profiling host uses it to break a run down per call; the tests leave it null, and
        /// a null observer costs nothing but two timestamp reads.
        /// </summary>
        public static Action<int, string, long>? CallObserver;

        public static RpcRunResult Run(RpcCase testCase)
        {
            RpcRunResult result = new RpcRunResult();
            Stopwatch watch = Stopwatch.StartNew();

            // A fresh project and a fresh server for every run. Not an optimization to skip: BRep results
            // depend on what ran before in the same process, and cases have been seen to pass alone and fail
            // inside a batch.
            Project project = Project.CreateSimpleProject();
            // The IFrame is used in exactly one place, MCPServer.ReportError, and only when SuppressDialogs is
            // false. Unattended means: errors end up in the response instead of in a modal message box.
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };

            try
            {
                // The JsonElements handed to the server stay valid only while their document lives.
                using JsonDocument document = testCase.OpenCallDocument();
                JsonElement calls = document.RootElement.GetProperty("RPCCalls");
                foreach (JsonElement call in calls.EnumerateArray())
                {
                    int id = call.TryGetProperty("id", out JsonElement idElement) && idElement.ValueKind == JsonValueKind.Number
                        ? idElement.GetInt32() : 0;
                    string method = call.TryGetProperty("method", out JsonElement methodElement) && methodElement.ValueKind == JsonValueKind.String
                        ? methodElement.GetString() ?? "" : "";
                    JsonElement parameters = call.TryGetProperty("params", out JsonElement p) ? p : default;
                    if (method.Length == 0) { result.Calls.Add((id, "", "call has no method")); continue; }

                    long startTicks = Stopwatch.GetTimestamp();
                    string response = server.ProcessMethod(method, id, parameters);
                    CallObserver?.Invoke(id, method,
                        (Stopwatch.GetTimestamp() - startTicks) * 1000L / Stopwatch.Frequency);
                    result.Calls.Add((id, method, ErrorOf(response)));
                }
                Collect(testCase, server, project, result);
            }
            catch (Exception e)
            {
                result.Crash = e;
            }
            watch.Stop();
            result.ElapsedMilliseconds = watch.ElapsedMilliseconds;
            return result;
        }

        /// <summary>The error of a JSON-RPC response as text, null when the call succeeded.</summary>
        private static string? ErrorOf(string response)
        {
            try
            {
                using JsonDocument document = JsonDocument.Parse(response);
                if (!document.RootElement.TryGetProperty("error", out JsonElement error)) return null;
                string code = error.TryGetProperty("code", out JsonElement c) ? c.ToString() : "";
                string message = error.TryGetProperty("message", out JsonElement m) ? m.GetString() ?? "" : "";
                return $"[{code}] {BRepSummary.Truncate(BRepSummary.FirstLine(message), 300)}";
            }
            catch (Exception e) { return "unreadable response: " + e.Message; }
        }

        /// <summary>Does the error of this call match what the case declared with ExpectError?</summary>
        public static bool IsExpected(RpcCase testCase, int id, string error)
        {
            foreach (ExpectedError expected in testCase.ExpectError)
            {
                if (expected.Id != id) continue;
                if (expected.Code.Length == 0) return true;
                if (error.Contains(expected.Code, StringComparison.OrdinalIgnoreCase)) return true;
            }
            return false;
        }

        private static void Collect(RpcCase testCase, MCPServer server, Project project, RpcRunResult result)
        {
            string[] names = testCase.Verify.Length > 0 ? testCase.Verify : CommittedNames(project);
            foreach (string name in names)
            {
                object? item = null;
                if (!server.namedItems.TryGetValue(name, out item)) item = FindInModel(project, name);
                if (item == null)
                {
                    // Not an error: this is what an operation with an empty result looks like - it stores nothing.
                    Add(result, name, Single(ExistsKey, "false"), 1.0);
                    continue;
                }
                switch (item)
                {
                    case Solid solid:
                        AddShell(result, name, solid.Shell);
                        break;
                    case Shell shell:
                        AddShell(result, name, shell);
                        break;
                    case IReadOnlyList<Solid> solids:
                        AddParts(result, name, solids.Select(s => s.Shell));
                        break;
                    case System.Collections.IEnumerable list when list.Cast<object>().All(o => o is Solid):
                        AddParts(result, name, list.Cast<Solid>().Select(s => s.Shell));
                        break;
                    default:
                        Add(result, name, Single("type", item.GetType().Name), 1.0);
                        break;
                }
            }
        }

        private static void AddParts(RpcRunResult result, string name, IEnumerable<Shell> shells)
        {
            // Canonical order, never the order the operation happened to return: internal ordering has been
            // seen to vary between runs (a HashSet iteration order once made a case fail about one run in
            // five), and a baseline must not depend on it.
            Shell[] sorted = ShellMetrics.SortCanonically(shells);
            // A list of one behaves like the object itself - that is the toolset's own convention, and a
            // single result should not read differently just because it happens to be stored as a list.
            // A later split into two parts is still obvious in the diff: the fields move to "<name>#0".
            if (sorted.Length == 1) { AddShell(result, name, sorted[0]); return; }
            Add(result, name, Single(SolidsKey, sorted.Length.ToString(System.Globalization.CultureInfo.InvariantCulture)), 1.0);
            for (int i = 0; i < sorted.Length; i++) AddShell(result, $"{name}#{i}", sorted[i]);
        }

        private static void AddShell(RpcRunResult result, string name, Shell shell)
        {
            BRepSummary summary = new BRepSummary();
            ShellMetrics.Describe(summary, "", shell);
            // PrecisionFor is size/1000, so this is the size of the shell - the absolute floor for the
            // comparison of coordinates near zero.
            double scale = ShellMetrics.PrecisionFor(shell) * 1000.0;
            result.Objects[name] = new RpcObjectSummary { Name = name, Summary = summary, Scale = scale };
        }

        private static BRepSummary Single(string key, string value)
        {
            BRepSummary summary = new BRepSummary();
            summary.Add(key, value);
            return summary;
        }

        private static void Add(RpcRunResult result, string name, BRepSummary summary, double scale)
            => result.Objects[name] = new RpcObjectSummary { Name = name, Summary = summary, Scale = scale };

        private static string[] CommittedNames(Project project)
        {
            Model model = project.GetActiveModel();
            List<string> names = new List<string>();
            foreach (IGeoObject go in model.AllObjects)
            {
                string? name = (go as Solid)?.Name;
                if (!string.IsNullOrEmpty(name)) names.Add(name!);
            }
            return names.Distinct(StringComparer.Ordinal).OrderBy(n => n, StringComparer.Ordinal).ToArray();
        }

        private static object? FindInModel(Project project, string name)
        {
            Model model = project.GetActiveModel();
            foreach (IGeoObject go in model.AllObjects)
                if (go is Solid solid && string.Equals(solid.Name, name, StringComparison.Ordinal)) return solid;
            return null;
        }
    }
}
