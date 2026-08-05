using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using CADability;
using CADability.Attribute;
using CADability.GeoObject;

namespace ShapeIt
{
    /// <summary>
    /// The BRep operation a test case asks for. Derived from the text object inside the project
    /// (or from an explicit override).
    /// </summary>
    public enum BRepOperationKind
    {
        Unknown,
        Union,
        Difference,
        Intersection,
        /// <summary>Unite all solids of the model, one after the other.</summary>
        UniteAll,
        RoundEdges,
        ChamferEdges
    }

    /// <summary>
    /// One BRep test case: the geometry read from a project plus the operation to perform on it.
    /// <para>
    /// Convention inside the project file:
    /// </para>
    /// <list type="bullet">
    /// <item>the two operands of a boolean operation carry the styles "Operand1" and "Operand2",</item>
    /// <item>the edges to round/chamfer are marked by curves with the style "EdgeMarker",</item>
    /// <item>a single text object names the operation, optionally with a parameter, e.g. "Difference",
    /// "Unite", "RoundEdges: 2.5", "ChamferEdges: 1" (parameters are always parsed with the invariant culture),</item>
    /// <item>an optional solid/shell with the style "Expected" holds the known good result.</item>
    /// </list>
    /// <para>
    /// This is used by two callers: the BRep regression tests (tests/CADability.Tests/BRepRegressionTests.cs) and
    /// the AutoDebug helper of the main form. Both take the same path on purpose - a failing test can then be
    /// reproduced in the app, with the objects having the same hash codes in every run, which is what makes
    /// conditional breakpoints usable.
    /// </para>
    /// <para>
    /// Anything that does not follow the convention is reported in <see cref="Problems"/> instead of being
    /// silently ignored - a case that quietly does nothing is worse than no case at all.
    /// </para>
    /// </summary>
    public class BRepCase
    {
        public string Name { get; set; } = "";
        public string FilePath { get; set; } = "";

        public BRepOperationKind Operation { get; set; } = BRepOperationKind.Unknown;
        /// <summary>Radius (RoundEdges) or first distance (ChamferEdges); NaN when the operation takes no parameter.</summary>
        public double Parameter { get; set; } = double.NaN;
        /// <summary>Second chamfer distance; NaN means "same as <see cref="Parameter"/>".</summary>
        public double SecondaryParameter { get; set; } = double.NaN;

        /// <summary>
        /// The operands in a well defined order: [0] is Operand1, [1..] are the Operand2 objects in the order
        /// they appear in the model. There may be more than one Operand2 - a difference then subtracts all of
        /// them from Operand1, a union adds all of them.
        /// </summary>
        public List<Shell> Operands { get; } = new List<Shell>();
        /// <summary>Edges of <c>Operands[0]</c> that the marker curves resolved to (round/chamfer only).</summary>
        public List<Edge> MarkedEdges { get; } = new List<Edge>();
        /// <summary>The known good result, if the project contains one (style "Expected"). Not required.</summary>
        public Shell? ExpectedResult { get; set; }

        /// <summary>Reasons why this case cannot be run. A case with problems is never executed.</summary>
        public List<string> Problems { get; } = new List<string>();
        /// <summary>Oddities that do not prevent execution but are worth reporting.</summary>
        public List<string> Warnings { get; } = new List<string>();

        public bool IsRunnable => Problems.Count == 0;

        /// <summary>The size of the input, used to derive absolute tolerances from relative ones.</summary>
        public double Scale
        {
            get
            {
                BoundingBox ext = BoundingBox.EmptyBoundingBox;
                foreach (Shell shell in Operands) ext.MinMax(shell.GetExtent(0.0));
                return ext.IsEmpty ? 1.0 : Math.Max(ext.Size, 1e-6);
            }
        }

        public override string ToString() => $"{Name} ({Operation})";
    }

    /// <summary>
    /// Builds a <see cref="BRepCase"/> from a .cdb.json project file or from an already opened model.
    /// </summary>
    public static class BRepCaseReader
    {
        public const string Operand1Style = "Operand1";
        public const string Operand2Style = "Operand2";
        public const string EdgeMarkerStyle = "EdgeMarker";
        public const string ExpectedStyle = "Expected";

        /// <summary>Tolerance for matching an edge marker curve against the edges of the operand.</summary>
        private const double MarkerTolerance = 0.1;

        /// <param name="operationOverride">Operation to use when the project contains no (or no unambiguous)
        /// text object. Null means "take it from the project".</param>
        public static BRepCase Read(string filePath, BRepOperationKind? operationOverride = null,
            double parameterOverride = double.NaN, double secondaryParameterOverride = double.NaN)
        {
            BRepCase result = new BRepCase { Name = CaseName(filePath), FilePath = filePath };
            Project project;
            try
            {
                project = Project.ReadFromFile(filePath, "cdb");
            }
            catch (Exception e)
            {
                result.Problems.Add($"cannot read project: {e.GetType().Name}: {e.Message}");
                return result;
            }
            if (project == null)
            {
                result.Problems.Add("Project.ReadFromFile returned null");
                return result;
            }
            Model? model = project.GetActiveModel();
            if (model == null)
            {
                result.Problems.Add("project has no active model");
                return result;
            }
            Fill(result, model, operationOverride, parameterOverride, secondaryParameterOverride);
            return result;
        }

        /// <summary>
        /// Same as <see cref="Read"/>, but on a model that is already open. AutoDebug uses this so that the
        /// operation runs on exactly those objects that are visible in the app.
        /// </summary>
        public static BRepCase FromModel(Model model, string name, BRepOperationKind? operationOverride = null,
            double parameterOverride = double.NaN, double secondaryParameterOverride = double.NaN)
        {
            BRepCase result = new BRepCase { Name = name };
            Fill(result, model, operationOverride, parameterOverride, secondaryParameterOverride);
            return result;
        }

        private static void Fill(BRepCase result, Model model, BRepOperationKind? operationOverride,
            double parameterOverride, double secondaryParameterOverride)
        {
            // --- collect what the model offers ---------------------------------------------------
            List<(Shell shell, string? style)> candidates = new List<(Shell, string?)>();
            List<ICurve> markerCurves = new List<ICurve>();
            List<string> commands = new List<string>();

            foreach (IGeoObject go in model.AllObjects)
            {
                string? styleName = StyleName(go);
                switch (go)
                {
                    case Solid solid:
                        if (solid.Shells.Length == 0) result.Warnings.Add("a solid without shells was ignored");
                        else
                        {
                            if (solid.Shells.Length > 1)
                                result.Warnings.Add($"a solid has {solid.Shells.Length} shells, only the first one is used");
                            candidates.Add((solid.Shells[0], styleName));
                        }
                        break;
                    case Shell shell:
                        candidates.Add((shell, styleName));
                        break;
                    case Text text:
                        if (!string.IsNullOrWhiteSpace(text.TextString)) commands.Add(text.TextString.Trim());
                        break;
                    case ICurve curve:
                        if (styleName == EdgeMarkerStyle) markerCurves.Add(curve);
                        break;
                }
            }

            // --- the expected result, if the project carries one ----------------------------------
            for (int i = candidates.Count - 1; i >= 0; --i)
            {
                if (candidates[i].style == ExpectedStyle)
                {
                    if (result.ExpectedResult != null) result.Warnings.Add("more than one object with style \"Expected\"");
                    result.ExpectedResult = candidates[i].shell;
                    candidates.RemoveAt(i);
                }
            }

            // --- the operation --------------------------------------------------------------------
            List<ParsedCommand> parsed = new List<ParsedCommand>();
            foreach (string command in commands)
            {
                if (TryParseCommand(command, out ParsedCommand pc)) parsed.Add(pc);
                else result.Warnings.Add($"text object \"{command}\" is not a known command");
            }
            List<BRepOperationKind> distinct = parsed.Select(p => p.Kind).Distinct().ToList();
            if (operationOverride.HasValue)
            {
                result.Operation = operationOverride.Value;
                result.Parameter = parameterOverride;
                result.SecondaryParameter = secondaryParameterOverride;
                if (parsed.Count > 0 && !distinct.Contains(operationOverride.Value))
                    result.Warnings.Add($"the operation is overridden to {operationOverride.Value}, "
                        + $"the project says {string.Join("/", distinct)}");
            }
            else if (parsed.Count == 0)
            {
                result.Problems.Add(commands.Count == 0
                    ? "no text object naming the operation"
                    : $"no recognizable command in {commands.Count} text object(s): {string.Join(", ", commands.Select(c => "\"" + c + "\""))}");
            }
            else if (distinct.Count > 1)
            {
                result.Problems.Add($"ambiguous: {parsed.Count} text objects name different operations "
                    + $"({string.Join(", ", parsed.Select(p => "\"" + p.Text + "\""))})");
            }
            else
            {
                result.Operation = parsed[0].Kind;
                result.Parameter = parsed[0].Parameter;
                result.SecondaryParameter = parsed[0].SecondaryParameter;
                if (parsed.Count > 1) result.Warnings.Add($"{parsed.Count} text objects, all naming {parsed[0].Kind}");
            }

            // --- the operands ---------------------------------------------------------------------
            // The order of the Operand2 objects is the order they appear in the model, which is the order the
            // operation is carried out in. It is stable across runs because it comes from the file.
            List<Shell> operand1 = candidates.Where(c => c.style == Operand1Style).Select(c => c.shell).ToList();
            List<Shell> operand2 = candidates.Where(c => c.style == Operand2Style).Select(c => c.shell).ToList();
            if (operand1.Count > 1) result.Problems.Add($"{operand1.Count} objects with style \"Operand1\", there can only be one");

            switch (LayoutOf(result.Operation))
            {
                case OperandLayout.FirstAndOthers:
                    if (operand1.Count != 1 || operand2.Count == 0)
                    {
                        result.Problems.Add("expected one \"Operand1\" and at least one \"Operand2\", found "
                            + DescribeCandidates(candidates));
                    }
                    else
                    {
                        result.Operands.Add(operand1[0]);
                        result.Operands.AddRange(operand2);
                        int unmarked = candidates.Count - 1 - operand2.Count;
                        if (unmarked > 0) result.Warnings.Add($"{unmarked} solid(s)/shell(s) without a marker are ignored");
                    }
                    break;
                case OperandLayout.Single:
                    Shell? single = operand1.Count == 1 ? operand1[0] : (candidates.Count == 1 ? candidates[0].shell : null);
                    if (single == null)
                        result.Problems.Add("expected exactly one operand (style \"Operand1\"), found " + DescribeCandidates(candidates));
                    else
                    {
                        result.Operands.Add(single);
                        if (operand2.Count > 0) result.Warnings.Add($"{operand2.Count} \"Operand2\" object(s) are ignored for this operation");
                    }
                    break;
                case OperandLayout.All:
                    // UniteAll takes every solid of the model, markers or not. Here the order does not come from
                    // the file, so it is derived from the geometry to stay reproducible: by the center of the
                    // extent, exactly as the old AutoDebug did.
                    if (candidates.Count < 2) result.Problems.Add("UniteAll needs at least two solids, found " + DescribeCandidates(candidates));
                    else result.Operands.AddRange(candidates.Select(c => c.shell).OrderBy(s => s.GetExtent(0.0).GetCenter(), CenterComparer.Instance));
                    break;
            }

            // --- the marked edges -----------------------------------------------------------------
            bool needsEdges = result.Operation == BRepOperationKind.RoundEdges || result.Operation == BRepOperationKind.ChamferEdges;
            if (needsEdges)
            {
                if (markerCurves.Count == 0) result.Problems.Add("no curve with style \"EdgeMarker\"");
                if (double.IsNaN(result.Parameter))
                    result.Problems.Add($"{result.Operation} needs a parameter, e.g. \"{result.Operation}: 2.5\"");
                if (result.Operands.Count == 1)
                {
                    foreach (ICurve marker in markerCurves)
                    {
                        List<Edge> hits = result.Operands[0].Edges
                            .Where(e => e.Curve3D != null && e.Curve3D.SameGeometry(marker, MarkerTolerance)).ToList();
                        if (hits.Count == 0) result.Problems.Add("an edge marker curve matches no edge of the operand");
                        else
                        {
                            if (hits.Count > 1) result.Warnings.Add($"an edge marker curve matches {hits.Count} edges, all of them are used");
                            foreach (Edge edge in hits) if (!result.MarkedEdges.Contains(edge)) result.MarkedEdges.Add(edge);
                        }
                    }
                }
            }
            else if (markerCurves.Count > 0)
            {
                result.Warnings.Add($"{markerCurves.Count} edge marker curve(s) are ignored for {result.Operation}");
            }
        }

        /// <summary>"DifferenceBug15.cdb.json" -> "DifferenceBug15"</summary>
        public static string CaseName(string filePath)
        {
            string name = System.IO.Path.GetFileName(filePath);
            if (name.EndsWith(".cdb.json", StringComparison.OrdinalIgnoreCase)) return name.Substring(0, name.Length - ".cdb.json".Length);
            return System.IO.Path.GetFileNameWithoutExtension(name);
        }

        /// <summary>How an operation expects its operands to be marked in the project.</summary>
        public enum OperandLayout
        {
            None,
            /// <summary>One shell, marked "Operand1" (or the only solid in the model).</summary>
            Single,
            /// <summary>One "Operand1" and one or more "Operand2", in the order they appear in the model.</summary>
            FirstAndOthers,
            /// <summary>Every solid of the model, markers or not.</summary>
            All
        }

        public static OperandLayout LayoutOf(BRepOperationKind kind)
        {
            switch (kind)
            {
                case BRepOperationKind.Union:
                case BRepOperationKind.Difference:
                case BRepOperationKind.Intersection: return OperandLayout.FirstAndOthers;
                case BRepOperationKind.RoundEdges:
                case BRepOperationKind.ChamferEdges: return OperandLayout.Single;
                case BRepOperationKind.UniteAll: return OperandLayout.All;
                default: return OperandLayout.None;
            }
        }

        private static string? StyleName(IGeoObject go)
        {
            Style? style = go.Style;
            return style?.Name;
        }

        private static string DescribeCandidates(List<(Shell shell, string? style)> candidates)
        {
            if (candidates.Count == 0) return "no solid/shell at all";
            return $"{candidates.Count} solid(s)/shell(s) with the styles ["
                + string.Join(", ", candidates.Select(c => c.style ?? "<none>")) + "]";
        }

        /// <summary>Orders points by y, then x, then z - a reproducible order for the UniteAll chain.</summary>
        private class CenterComparer : IComparer<GeoPoint>
        {
            public static readonly CenterComparer Instance = new CenterComparer();
            public int Compare(GeoPoint a, GeoPoint b)
            {
                int result = a.y.CompareTo(b.y);
                if (result != 0) return result;
                result = a.x.CompareTo(b.x);
                if (result != 0) return result;
                return a.z.CompareTo(b.z);
            }
        }

        private struct ParsedCommand
        {
            public string Text;
            public BRepOperationKind Kind;
            public double Parameter;
            public double SecondaryParameter;
        }

        /// <summary>
        /// Parses a text object like "Difference", "RoundEdges: 2.5" or "ChamferEdges: 1, 2".
        /// Numbers are always read with the invariant culture, so "2.5" is two and a half on a German machine too.
        /// </summary>
        private static bool TryParseCommand(string text, out ParsedCommand parsed)
        {
            parsed = new ParsedCommand { Text = text, Parameter = double.NaN, SecondaryParameter = double.NaN };
            string[] parts = text.Split(':');
            string verb = parts[0].Trim().ToLowerInvariant();
            switch (verb)
            {
                case "union":
                case "unite": parsed.Kind = BRepOperationKind.Union; break;
                case "uniteall":
                case "unionall": parsed.Kind = BRepOperationKind.UniteAll; break;
                case "difference":
                case "subtract": parsed.Kind = BRepOperationKind.Difference; break;
                case "intersect":
                case "intersection": parsed.Kind = BRepOperationKind.Intersection; break;
                case "roundedges":
                case "fillet": parsed.Kind = BRepOperationKind.RoundEdges; break;
                case "chamferedges":
                case "chamfer": parsed.Kind = BRepOperationKind.ChamferEdges; break;
                default: return false;
            }
            if (parts.Length > 1)
            {
                string[] numbers = parts[1].Split(',', ';');
                if (numbers.Length > 0 && double.TryParse(numbers[0].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out double p1))
                    parsed.Parameter = p1;
                if (numbers.Length > 1 && double.TryParse(numbers[1].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out double p2))
                    parsed.SecondaryParameter = p2;
            }
            return true;
        }
    }
}
