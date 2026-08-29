using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using CADability;
using CADability.GeoObject;
using CADability.Shapes;

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
        /// <summary>The triangulation precision, relative to the size of the shell.</summary>
        private const double RelativeTriangulationPrecision = 1e-3;

        /// <summary>
        /// The precision volume, area and extent are computed with. It must not be 0.0, which is what the rest of
        /// the code passes: <see cref="Face.AssureTriangles"/> then reuses whatever triangulation happens to exist
        /// - however coarse it was made - and invents "extent size / 10" when there is none.
        /// <para>
        /// The precision is derived from the exact geometry only - vertex positions and edge curves, never from a
        /// triangulation - so it is the same number in every run. Asking for an explicit precision is only half
        /// the story though, see <see cref="Describe"/> for why the measurement runs on a copy of the shell.
        /// </para>
        /// </summary>
        public static double PrecisionFor(Shell shell)
        {
            BoundingBox box = BoundingBox.EmptyBoundingBox;
            foreach (Vertex vertex in shell.Vertices) box.MinMax(vertex.Position);
            foreach (Edge edge in shell.Edges) if (edge.Curve3D != null) box.MinMax(edge.Curve3D.GetExtent());
            double size = box.IsEmpty ? 1.0 : box.Size;
            return Math.Max(size, 1e-6) * RelativeTriangulationPrecision;
        }

        /// <summary>
        /// Measures a shell. The triangulation dependent values - volume, area and extent - are taken from a
        /// CLONE of the shell, not from the shell itself.
        /// <para>
        /// The reason is the reuse rule in <see cref="Face.AssureTriangles"/>: "precision >= trianglePrecision
        /// / 2.0" keeps an existing mesh as long as it is at least half as fine as the one being asked for. A
        /// face that the boolean operation already triangulated at ext.Size * 1e-4, or that CheckConsistency
        /// meshed on the way past, is therefore never re-meshed for the summary - so the numbers described
        /// whatever mesh happened to be lying around, and that depends on what ran before in the same process.
        /// That is what made a case come out with a different area depending on whether it ran alone or in a
        /// full suite, and it is why regenerating the baselines under one test filter produced baselines that
        /// no longer matched under another.
        /// </para>
        /// <para>
        /// Face.Clone does not copy trianglePoint/triangleIndex/trianglePrecision, so the copy carries no mesh
        /// and is triangulated exactly once, at exactly <see cref="PrecisionFor"/>. The copy is created lazily
        /// and shared by all three values: the first of them meshes it, the other two reuse that mesh. A clone
        /// that throws makes those three values report "error:..." like any other guarded value, rather than
        /// silently falling back to the original shell and reintroducing the drift unnoticed.
        /// </para>
        /// <para>
        /// Everything else - the counts, the Euler characteristic, the surface histogram, the edge lengths - is
        /// read off the original shell: none of it touches a triangulation, and cloning for it would only cost
        /// time.
        /// </para>
        /// </summary>
        public static void Describe(BRepSummary summary, string prefix, Shell shell)
        {
            double precision = PrecisionFor(shell);
            Lazy<Shell> measured = new Lazy<Shell>(() => (Shell)shell.Clone());
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
            Add(summary, prefix + "volume", () => BRepSummary.Format(measured.Value.Volume(precision)));
            Add(summary, prefix + "area", () => BRepSummary.Format(SurfaceArea(measured.Value)));
            Add(summary, prefix + "edgeLength", () => BRepSummary.Format(TotalEdgeLength(shell)));
            Add(summary, prefix + "extent", () => FormatExtent(measured.Value.GetExtent(precision)));
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

        /// <summary>
        /// The 3d surface area. The triangulation is used only to PARTITION the parameter domain of each
        /// face; the area of a part is then integrated over its uv triangle as the surface integral of
        /// |Su x Sv|, with the three point rule (the edge midpoints of the uv triangle, weight 1/3 each),
        /// which is exact for quadratic integrands.
        /// <para>
        /// Summing the flat triangles instead - which is what this did until 2026-08-25 - underestimates a
        /// curved face by an amount that only falls with the square of the mesh size, and it makes the
        /// number depend on how finely the face happened to be triangulated. Measured on a band of a sphere
        /// against the exact area, flat sum versus this quadrature:
        /// </para>
        /// <para>
        /// 8x4 mesh: -7.622% / +0.00263%, 20x10: -1.260% / +0.00007%, 48x24: -0.220% / 0.00000%
        /// </para>
        /// <para>
        /// So the value is now essentially independent of the mesh, which also removes the mesh as a source
        /// of run to run scatter in this field.
        /// </para>
        /// </summary>
        public static double SurfaceArea(Shell shell)
        {
            double precision = PrecisionFor(shell);
            double sum = 0.0;
            foreach (Face face in shell.Faces)
            {
                face.GetTriangulation(precision, out GeoPoint[] points, out GeoPoint2D[] uvPoints, out int[] indices, out _);
                if (indices == null) continue;
                double flat = 0.0;
                for (int i = 0; i < indices.Length; i += 3)
                {
                    GeoVector a = points[indices[i + 1]] - points[indices[i]];
                    GeoVector b = points[indices[i + 2]] - points[indices[i]];
                    flat += 0.5 * (a ^ b).Length;
                }
                sum += IntegratedFaceArea(face, points, uvPoints, indices, flat);
            }
            return sum;
        }

        /// <summary>
        /// The area of one face: the integral of |Su x Sv| over its parameter domain. Which route is taken
        /// depends on what the domain looks like, because the triangulation is not always a usable partition
        /// of it.
        /// <list type="bullet">
        /// <item>A PLANAR face has a constant |Su x Sv|, so the area is that times the exact domain area -
        /// no quadrature and no mesh at all. This is what makes a trimmed disc come out as pi*r^2 instead of
        /// the inscribed polygon the mesh would give.</item>
        /// <item>A face whose domain is the full bounding RECTANGLE is integrated over that rectangle
        /// directly. This is the case for the mantle of a cone or cylinder and for a torus, and it is what
        /// closes the gap a pole leaves: the triangulation drops the triangles along a singular line because
        /// they are degenerate in 3d, which costs the flat sum nothing but leaves a hole in the parameter
        /// domain. Measured on the mantle of a cone the triangles cover only 87.6 percent of the rectangle.</item>
        /// <item>Otherwise the uv triangles are used as the partition. They form a polygon inscribed in the
        /// true domain, so they fall slightly short along a curved boundary; that is corrected by scaling
        /// with domain/covered, which is safe as long as the shortfall is a thin boundary strip. Beyond
        /// <c>maxDomainShortfall</c> the extrapolation is refused and the flat triangle sum is used - the
        /// old, slightly low value, but never a new error.</item>
        /// </list>
        /// </summary>
        private static double IntegratedFaceArea(Face face, GeoPoint[] points, GeoPoint2D[] uvPoints, int[] indices, double flat)
        {
            ISurface surface = face.Surface;
            if (surface == null) return flat;
            SimpleShape shape;
            double domain;
            BoundingRect rect;
            try
            {
                shape = face.Area;
                domain = shape.Area;
                rect = shape.GetExtent();
            }
            catch (Exception) { return flat; }
            if (!(domain > 0.0)) return flat;

            if (surface is PlaneSurface)
            {   // constant integrand: exact, and it needs neither the mesh nor a quadrature
                double scale = Jacobian(surface, rect.GetCenter());
                return scale > 0.0 ? scale * domain : flat;
            }

            if (Math.Abs(domain - rect.Width * rect.Height) <= 1e-6 * domain)
            {   // the domain IS the rectangle, so the mesh is not needed and the pole gap cannot bite
                double overRectangle = IntegrateOverRectangle(surface, rect);
                if (overRectangle > 0.0) return overRectangle;
                return flat;
            }

            if (uvPoints == null || uvPoints.Length != points.Length) return flat;
            double covered = 0.0, integrated = 0.0;
            for (int i = 0; i < indices.Length; i += 3)
            {
                GeoPoint2D uv1 = uvPoints[indices[i]], uv2 = uvPoints[indices[i + 1]], uv3 = uvPoints[indices[i + 2]];
                double duv = 0.5 * Math.Abs((uv2.x - uv1.x) * (uv3.y - uv1.y) - (uv3.x - uv1.x) * (uv2.y - uv1.y));
                if (duv <= 0.0) continue;
                double part = IntegrateArea(surface, uv1, uv2, uv3, duv);
                if (!(part > 0.0) || double.IsNaN(part) || double.IsInfinity(part)) return flat;
                covered += duv;
                integrated += part;
            }
            if (!(covered > 0.0)) return flat;
            double shortfall = (domain - covered) / domain;
            if (shortfall < -1e-6 || shortfall > maxDomainShortfall) return flat;
            return integrated * domain / covered;
        }

        /// <summary>How much of the parameter domain the uv triangles may leave uncovered before the
        /// correction by domain/covered is refused as an extrapolation.</summary>
        private const double maxDomainShortfall = 0.02;

        /// <summary>|Su x Sv| at a parameter point, 0 when the surface cannot be differentiated there.</summary>
        private static double Jacobian(ISurface surface, GeoPoint2D uv)
        {
            try
            {
                surface.DerivativeAt(uv, out GeoPoint location, out GeoVector du, out GeoVector dv);
                double res = (du ^ dv).Length;
                return double.IsNaN(res) || double.IsInfinity(res) ? 0.0 : res;
            }
            catch (Exception) { return 0.0; }
        }

        /// <summary>
        /// The integral of |Su x Sv| over a rectangle of the parameter plane, by a tensor product of the two
        /// point Gauss rule over a grid of cells, refined until the value settles. The integrand of a natural
        /// quadric is smooth and low order, so this converges in very few steps.
        /// </summary>
        private static double IntegrateOverRectangle(ISurface surface, BoundingRect rect)
        {
            const double g = 0.5773502691896257; // 1/sqrt(3), the two point Gauss node
            double previous = 0.0;
            for (int n = 2; n <= 64; n *= 2)
            {
                double du = rect.Width / n, dv = rect.Height / n;
                double sum = 0.0;
                for (int i = 0; i < n; i++)
                {
                    double uc = rect.Left + (i + 0.5) * du;
                    for (int j = 0; j < n; j++)
                    {
                        double vc = rect.Bottom + (j + 0.5) * dv;
                        for (int a = -1; a <= 1; a += 2)
                        {
                            for (int b = -1; b <= 1; b += 2)
                            {
                                sum += Jacobian(surface, new GeoPoint2D(uc + a * g * du / 2.0, vc + b * g * dv / 2.0));
                            }
                        }
                    }
                }
                sum *= du * dv / 4.0;
                if (n > 2 && Math.Abs(sum - previous) <= 1e-9 * Math.Abs(sum)) return sum;
                previous = sum;
            }
            return previous;
        }

        /// <summary>
        /// The area of the surface patch over one uv triangle: the integral of |Su x Sv| over that triangle,
        /// evaluated with the three point rule at the midpoints of its edges, which is exact for a quadratic
        /// integrand. Returns 0 when the surface cannot be differentiated at one of the three points.
        /// </summary>
        private static double IntegrateArea(ISurface surface, GeoPoint2D uv1, GeoPoint2D uv2, GeoPoint2D uv3, double duv)
        {
            if (duv <= 0.0) return 0.0;
            double acc = Jacobian(surface, new GeoPoint2D(uv1, uv2))
                       + Jacobian(surface, new GeoPoint2D(uv2, uv3))
                       + Jacobian(surface, new GeoPoint2D(uv3, uv1));
            return duv * acc / 3.0;
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
            return shells.OrderByDescending(s => Safe(() => s.Volume(PrecisionFor(s))))
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
