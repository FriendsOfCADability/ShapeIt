using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using CADability;
using CADability.GeoObject;

namespace ShapeIt
{
    /// <summary>
    /// An ordered list of key/value pairs describing a case and its result. This is what the BRep regression
    /// tests write to their baseline files and compare - deliberately <b>not</b> the topology itself.
    /// <para>
    /// The reason: face order, split points, seam positions and surface parametrization legitimately change when
    /// the algorithms change, without the result being wrong. Comparing them would produce a stream of false
    /// alarms. Volume, area, face/edge/vertex counts, the Euler characteristic and the bounding box on the other
    /// hand catch the errors that matter (a missing face, the wrong intersection branch, a superfluous part)
    /// and are immune to harmless re-parametrization.
    /// </para>
    /// <para>
    /// Numbers are compared with a relative tolerance, integers and strings exactly. The text form stays readable
    /// and diffable, so a failing test tells you what changed at a glance.
    /// </para>
    /// </summary>
    public class BRepSummary
    {
        private readonly List<KeyValuePair<string, string>> entries = new List<KeyValuePair<string, string>>();

        public IReadOnlyList<KeyValuePair<string, string>> Entries => entries;

        public void Add(string key, string value) => entries.Add(new KeyValuePair<string, string>(key, value));
        public void Add(string key, int value) => Add(key, value.ToString(CultureInfo.InvariantCulture));
        public void Add(string key, bool value) => Add(key, value ? "true" : "false");
        public void Add(string key, double value) => Add(key, Format(value));
        public void Add(string key, IEnumerable<double> values) => Add(key, string.Join(", ", values.Select(Format)));

        /// <summary>
        /// Describes a case and the result of running it - the single place where both callers (the regression
        /// test and AutoDebug) turn a run into text, so that what you see in the app is what the baseline holds.
        /// </summary>
        public static BRepSummary Describe(BRepCase testCase, BRepRunResult run)
        {
            BRepSummary summary = new BRepSummary();
            summary.Add("case.operation", testCase.Operation.ToString());
            if (!double.IsNaN(testCase.Parameter)) summary.Add("case.parameter", testCase.Parameter);
            if (!double.IsNaN(testCase.SecondaryParameter)) summary.Add("case.parameter2", testCase.SecondaryParameter);
            summary.Add("case.markedEdges", testCase.MarkedEdges.Count);
            for (int i = 0; i < testCase.Operands.Count; i++)
                ShellMetrics.Describe(summary, $"in{i}.", testCase.Operands[i]);
            if (testCase.ExpectedResult != null) ShellMetrics.Describe(summary, "exp.", testCase.ExpectedResult);

            summary.Add("out.status", run.Status);
            if (run.Error != null)
            {
                summary.Add("out.exception", run.Error.GetType().FullName ?? run.Error.GetType().Name);
                // Debug.Assert failures all arrive as the same exception type, so the message is what
                // distinguishes them. Truncated, because some messages carry a whole stack trace.
                summary.Add("out.exceptionMessage", Truncate(FirstLine(run.Error.Message), 120));
            }
            summary.Add("out.shells", run.Shells.Length);
            Shell[] sorted = ShellMetrics.SortCanonically(run.Shells);
            for (int i = 0; i < sorted.Length; i++) ShellMetrics.Describe(summary, $"out{i}.", sorted[i]);
            return summary;
        }

        public static string Format(double value)
        {
            if (double.IsNaN(value)) return "NaN";
            if (double.IsPositiveInfinity(value)) return "+inf";
            if (double.IsNegativeInfinity(value)) return "-inf";
            return value.ToString("G12", CultureInfo.InvariantCulture);
        }

        public static string FirstLine(string text)
        {
            int nl = text.IndexOfAny(new[] { '\r', '\n' });
            return nl < 0 ? text : text.Substring(0, nl);
        }

        public static string Truncate(string text, int maxLength)
            => text.Length <= maxLength ? text : text.Substring(0, maxLength) + "...";

        public string ToText()
        {
            StringBuilder sb = new StringBuilder();
            int width = entries.Count == 0 ? 0 : entries.Max(e => e.Key.Length);
            foreach (KeyValuePair<string, string> entry in entries)
                sb.Append(entry.Key.PadRight(width)).Append(" = ").Append(entry.Value).Append('\n');
            return sb.ToString();
        }

        public static Dictionary<string, string> ParseText(string text)
        {
            Dictionary<string, string> result = new Dictionary<string, string>(StringComparer.Ordinal);
            foreach (string rawLine in text.Replace("\r\n", "\n").Split('\n'))
            {
                string line = rawLine.Trim();
                if (line.Length == 0 || line.StartsWith("#")) continue;
                int eq = line.IndexOf('=');
                if (eq < 0) continue;
                result[line.Substring(0, eq).Trim()] = line.Substring(eq + 1).Trim();
            }
            return result;
        }

        /// <summary>
        /// A comment line saying that the result of this case has been judged correct by hand, e.g.
        /// <c># verified 2026-08-01: subtraction leaves the two expected parts</c>.
        /// </summary>
        public const string VerifiedKeyword = "verified";

        /// <summary>
        /// All comment lines of a baseline file, in their original order. They are written back when the
        /// baseline is regenerated, so a hand written "# verified" note survives BREP_REGEN.
        /// </summary>
        public static string ExtractComments(string text)
        {
            StringBuilder sb = new StringBuilder();
            foreach (string rawLine in text.Replace("\r\n", "\n").Split('\n'))
            {
                string line = rawLine.Trim();
                if (line.StartsWith("#")) sb.Append(line).Append('\n');
            }
            return sb.ToString();
        }

        /// <summary>True when a comment line marks this baseline as manually verified.</summary>
        public static bool IsVerified(string text)
        {
            foreach (string rawLine in text.Replace("\r\n", "\n").Split('\n'))
            {
                string line = rawLine.Trim();
                if (!line.StartsWith("#")) continue;
                if (line.TrimStart('#').TrimStart().StartsWith(VerifiedKeyword, StringComparison.OrdinalIgnoreCase)) return true;
            }
            return false;
        }

        /// <summary>
        /// Compares this summary against a baseline. Integers, booleans and strings must match exactly, floating
        /// point numbers within <paramref name="relativeTolerance"/>, where <paramref name="scale"/> (the size of
        /// the input) provides the absolute floor so that coordinates near zero do not need an exact match.
        /// </summary>
        /// <returns>One line per difference; empty when the summaries agree.</returns>
        public List<string> DiffAgainst(Dictionary<string, string> baseline, double relativeTolerance, double scale)
        {
            List<string> diff = new List<string>();
            HashSet<string> seen = new HashSet<string>(StringComparer.Ordinal);
            foreach (KeyValuePair<string, string> entry in entries)
            {
                seen.Add(entry.Key);
                if (!baseline.TryGetValue(entry.Key, out string? expected))
                {
                    diff.Add($"+ {entry.Key} = {entry.Value}   (not in baseline)");
                    continue;
                }
                if (!ValuesAreEqual(expected, entry.Value, relativeTolerance, scale))
                    diff.Add($"! {entry.Key}: expected {expected}, got {entry.Value}");
            }
            foreach (KeyValuePair<string, string> entry in baseline)
                if (!seen.Contains(entry.Key)) diff.Add($"- {entry.Key} = {entry.Value}   (missing in result)");
            return diff;
        }

        private static bool ValuesAreEqual(string expected, string actual, double relativeTolerance, double scale)
        {
            if (string.Equals(expected, actual, StringComparison.Ordinal)) return true;
            double[]? a = TryParseNumbers(expected);
            double[]? b = TryParseNumbers(actual);
            if (a == null || b == null || a.Length != b.Length || a.Length == 0) return false;
            for (int i = 0; i < a.Length; i++)
            {
                if (double.IsNaN(a[i]) && double.IsNaN(b[i])) continue;
                double tolerance = relativeTolerance * Math.Max(Math.Max(Math.Abs(a[i]), Math.Abs(b[i])), scale);
                if (Math.Abs(a[i] - b[i]) > tolerance) return false;
            }
            return true;
        }

        private static double[]? TryParseNumbers(string value)
        {
            string[] parts = value.Split(',');
            double[] result = new double[parts.Length];
            for (int i = 0; i < parts.Length; i++)
            {
                string part = parts[i].Trim();
                if (part == "NaN") { result[i] = double.NaN; continue; }
                if (!double.TryParse(part, NumberStyles.Float, CultureInfo.InvariantCulture, out result[i])) return null;
            }
            return result;
        }
    }

    /// <summary>
    /// Computes the invariants of a shell. Every single value is guarded: these shells may be broken, and a
    /// summary that throws is useless - "error:&lt;type&gt;" is a perfectly good baseline value.
    /// </summary>
    public static class ShellMetrics
    {
        /// <summary>Precision passed to the triangulation; 0 means "let CADability choose", as elsewhere in the code.</summary>
        private const double TriangulationPrecision = 0.0;

        public static void Describe(BRepSummary summary, string prefix, Shell shell)
        {
            Add(summary, prefix + "consistent", () => shell.CheckConsistency() ? "true" : "false");
            Add(summary, prefix + "faces", () => shell.Faces.Length.ToString(CultureInfo.InvariantCulture));
            Add(summary, prefix + "edges", () => RealEdgeCount(shell).ToString(CultureInfo.InvariantCulture));
            Add(summary, prefix + "poleEdges", () => DescribePoleEdges(shell));
            Add(summary, prefix + "vertices", () => shell.Vertices.Length.ToString(CultureInfo.InvariantCulture));
            Add(summary, prefix + "holeLoops", () => HoleLoopCount(shell).ToString(CultureInfo.InvariantCulture));
            Add(summary, prefix + "euler", () => EulerCharacteristic(shell).ToString(CultureInfo.InvariantCulture));
            // Shell.IsClosed reports a pole edge as open, because it has no secondary face. For the same reason
            // pole edges do not count as edges, they must not make a shell count as open either.
            Add(summary, prefix + "closed", () => shell.OpenEdgesExceptPoles.Length == 0 ? "true" : "false");
            Add(summary, prefix + "openEdges", () => shell.OpenEdgesExceptPoles.Length.ToString(CultureInfo.InvariantCulture));
            Add(summary, prefix + "volume", () => BRepSummary.Format(shell.Volume(TriangulationPrecision)));
            Add(summary, prefix + "area", () => BRepSummary.Format(SurfaceArea(shell)));
            Add(summary, prefix + "edgeLength", () => BRepSummary.Format(TotalEdgeLength(shell)));
            Add(summary, prefix + "extent", () => FormatExtent(shell.GetExtent(TriangulationPrecision)));
            Add(summary, prefix + "surfaces", () => SurfaceHistogram(shell));
        }

        private static void Add(BRepSummary summary, string key, Func<string> compute)
        {
            try { summary.Add(key, compute()); }
            catch (Exception e) { summary.Add(key, "error:" + e.GetType().Name); }
        }

        /// <summary>
        /// A pole edge has no 3d curve and starts and ends at the same vertex - the degenerate boundary at the
        /// apex of a cone or at the pole of a sphere. It is a boundary in parameter space only and must not be
        /// counted as an edge of the solid.
        /// </summary>
        public static bool IsPoleEdge(Edge edge) => edge.Curve3D == null;

        /// <summary>Edges that really are edges of the solid, i.e. everything except the poles.</summary>
        public static int RealEdgeCount(Shell shell) => shell.Edges.Count(e => !IsPoleEdge(e));

        /// <summary>
        /// The number of pole edges. In CADability an edge without a 3d curve always starts and ends at the same
        /// vertex (there are no closed single edges), so both criteria have to agree - if they ever do not, that
        /// is broken data and it is spelled out here instead of being silently averaged away.
        /// </summary>
        public static string DescribePoleEdges(Shell shell)
        {
            int withoutCurve = 0;
            int withoutCurveButTwoVertices = 0;
            int closedWithCurve = 0;
            foreach (Edge edge in shell.Edges)
            {
                if (edge.Curve3D == null)
                {
                    ++withoutCurve;
                    if (edge.Vertex1 != edge.Vertex2) ++withoutCurveButTwoVertices;
                }
                else if (edge.Vertex1 == edge.Vertex2) ++closedWithCurve;
            }
            string result = withoutCurve.ToString(CultureInfo.InvariantCulture);
            if (withoutCurveButTwoVertices > 0) result += $" ({withoutCurveButTwoVertices} of them with two different vertices!)";
            if (closedWithCurve > 0) result += $" ({closedWithCurve} closed edge(s) with a 3d curve!)";
            return result;
        }

        /// <summary>The number of inner loops (holes) over all faces of the shell.</summary>
        public static int HoleLoopCount(Shell shell)
        {
            int count = 0;
            foreach (Face face in shell.Faces) count += face.HoleCount;
            return count;
        }

        /// <summary>
        /// V - E + F - R with E counting only real edges (no poles) and R the number of inner loops. Subtracting
        /// R is what makes the number meaningful for faces with holes, which the plain V - E + F does not handle:
        /// the Euler-Poincare formula for a b-rep requires every face to be a disk.
        /// <para>
        /// For a single closed shell the result is 2 - 2*genus: 2 for anything sphere-like, 0 with one through
        /// hole, -2 with two, and so on. It is a topology fingerprint, not an assertion - a wrong value in the
        /// baseline diff means faces, edges or loops got lost or duplicated.
        /// </para>
        /// </summary>
        public static int EulerCharacteristic(Shell shell)
            => shell.Vertices.Length - RealEdgeCount(shell) + shell.Faces.Length - HoleLoopCount(shell);

        /// <summary>The 3d surface area, summed over the triangulation of all faces (the same source Volume uses).</summary>
        public static double SurfaceArea(Shell shell)
        {
            double sum = 0.0;
            foreach (Face face in shell.Faces)
            {
                face.GetTriangulation(TriangulationPrecision, out GeoPoint[] points, out _, out int[] indices, out _);
                for (int i = 0; i < indices.Length; i += 3)
                {
                    GeoVector a = points[indices[i + 1]] - points[indices[i]];
                    GeoVector b = points[indices[i + 2]] - points[indices[i]];
                    sum += 0.5 * (a ^ b).Length;
                }
            }
            return sum;
        }

        public static double TotalEdgeLength(Shell shell)
        {
            double sum = 0.0;
            foreach (Edge edge in shell.Edges) if (edge.Curve3D != null) sum += edge.Curve3D.Length;
            return sum;
        }

        /// <summary>e.g. "ConicalSurface:2 CylindricalSurface:6 PlaneSurface:12" - sorted, so it is canonical.</summary>
        public static string SurfaceHistogram(Shell shell)
        {
            Dictionary<string, int> counts = new Dictionary<string, int>(StringComparer.Ordinal);
            foreach (Face face in shell.Faces)
            {
                string name = face.Surface == null ? "<null>" : face.Surface.GetType().Name;
                counts.TryGetValue(name, out int count);
                counts[name] = count + 1;
            }
            return string.Join(" ", counts.OrderBy(kv => kv.Key, StringComparer.Ordinal).Select(kv => kv.Key + ":" + kv.Value));
        }

        private static string FormatExtent(BoundingBox box)
        {
            double[] values = { box.Xmin, box.Ymin, box.Zmin, box.Xmax, box.Ymax, box.Zmax };
            return string.Join(", ", values.Select(BRepSummary.Format));
        }

        /// <summary>
        /// Canonical order for the resulting shells, so that a different order inside the algorithm does not
        /// show up as a difference: biggest volume first, ties broken by face count and area.
        /// </summary>
        public static Shell[] SortCanonically(IEnumerable<Shell> shells)
        {
            return shells.OrderByDescending(s => Safe(() => s.Volume(TriangulationPrecision)))
                         .ThenByDescending(s => s.Faces.Length)
                         .ThenByDescending(s => Safe(() => SurfaceArea(s)))
                         .ToArray();
        }

        private static double Safe(Func<double> compute)
        {
            try { return compute(); }
            catch (Exception) { return double.NegativeInfinity; }
        }
    }
}
