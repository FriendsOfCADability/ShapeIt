// Imports a STEP file with ImportStep (like Project.ReadFromFile does) and dumps
// - the problems reported by the import (ImportStep.Problems),
// - the resulting object hierarchy (type, name, bounding cube), compared by compare.py,
// - a geometric summary of every solid and shell (faces, edges, open edges, volume, surface types),
//   compared by geomcompare.py.
using System.Globalization;
using System.Text;
using CADability;
using CADability.GeoObject;

class Program
{
    static int Main(string[] args)
    {
        // usage: StepProbe <file.stp> <label> <outfile>
        // the label only names the run (runcorpus.py passes it), it doesn't change the import
        if (args.Length < 3)
        {
            Console.Error.WriteLine("usage: StepProbe <file.stp> <label> <outfile>");
            return 1;
        }
        string file = args[0];
        string outFile = args[2];
        SettingsGlobalFileName.neverSaveGlobalSettings = true;
        var sb = new StringBuilder();
        // a Debug build of CADability is much slower and runs into Debug.Assert: comparisons with it are meaningless
        var cadability = typeof(ImportStep).Assembly;
        var debuggable = cadability.GetCustomAttributes(typeof(System.Diagnostics.DebuggableAttribute), false).OfType<System.Diagnostics.DebuggableAttribute>().FirstOrDefault();
        bool optimized = debuggable == null || !debuggable.IsJITOptimizerDisabled;
        sb.AppendLine($"# cadability {cadability.GetName().Version} {(optimized ? "optimized" : "DEBUG")}");
        var sw = System.Diagnostics.Stopwatch.StartNew();
        try
        {
            ImportStep import = new ImportStep();
            import.HierarchyToBlocks = Settings.GlobalSettings.GetBoolValue("ImportStep.UseHierarchy", true);
            GeoObjectList all = import.Read(file) ?? new GeoObjectList();
            sb.AppendLine($"# time {F1(sw.Elapsed.TotalSeconds)}s, top level {all.Count}");
            BoundingBox ext = BoundingBox.EmptyBoundingBox;
            foreach (IGeoObject go in all) ext.MinMax(go.GetBoundingCube());
            sb.AppendLine("# extent " + Cube(ext));
            Dictionary<int, string> problems = import.Problems;
            sb.AppendLine($"# problems {problems.Count}");
            foreach (KeyValuePair<int, string> kv in problems.OrderBy(p => p.Key)) sb.AppendLine($"# problem #{kv.Key} {OneLine(kv.Value)}");
            foreach (IGeoObject go in all) Dump(go, "", sb);
            // the geometric summary may take some time (triangulation for the volume), so it is measured separately
            var gw = System.Diagnostics.Stopwatch.StartNew();
            foreach (IGeoObject go in all) Bodies(go, sb);
            sb.AppendLine($"# geometry time {F1(gw.Elapsed.TotalSeconds)}s");
        }
        catch (Exception e)
        {
            sb.AppendLine("# EXCEPTION " + e.GetType().Name + ": " + OneLine(e.Message));
            // set STEPPROBE_STACK to get the stack trace of the exception
            if (Environment.GetEnvironmentVariable("STEPPROBE_STACK") != null) sb.AppendLine("# STACK " + e.ToString().Replace("\n", "\n# STACK "));
        }
        File.WriteAllText(outFile, sb.ToString());
        return 0;
    }

    static string F(double d) => d.ToString("G6", CultureInfo.InvariantCulture);
    static string F1(double d) => d.ToString("F1", CultureInfo.InvariantCulture);
    static string OneLine(string s) => (s ?? "").Replace("\r", " ").Replace("\n", " ");
    static string Cube(BoundingBox c) => c.IsEmpty ? "empty" : $"[{F(c.Xmin)},{F(c.Ymin)},{F(c.Zmin)} .. {F(c.Xmax)},{F(c.Ymax)},{F(c.Zmax)}]";

    static void Dump(IGeoObject go, string indent, StringBuilder sb)
    {
        string name = go switch { Block b => b.Name, Solid s => s.Name, Shell s => s.Name, _ => null } ?? "";
        sb.AppendLine($"{indent}{go.GetType().Name} '{name}' {Cube(go.GetBoundingCube())}");
        if (go is Block blk)
        {
            for (int i = 0; i < blk.NumChildren; i++) Dump(blk.Child(i), indent + "  ", sb);
        }
    }

    /// <summary>
    /// One line per solid or shell:
    /// "# body Solid 'name' faces 12 edges 30 open 0 volume 1234.56 surfaces CylindricalSurface:4,PlaneSurface:8"
    /// The volume is computed with a triangulation precision of 1e-4 of the size of the body, it is "-" for shells and
    /// "err" when the computation fails. The order of the lines is not deterministic (parallel import).
    /// </summary>
    static void Bodies(IGeoObject go, StringBuilder sb)
    {
        if (go is Block blk)
        {
            for (int i = 0; i < blk.NumChildren; i++) Bodies(blk.Child(i), sb);
            return;
        }
        Shell[] shells;
        string volume = "-";
        if (go is Solid sld)
        {
            shells = sld.Shells;
            try
            {
                double size = sld.GetExtent(0.0).Size;
                volume = F(sld.Volume(size * 1e-4));
            }
            catch (Exception) { volume = "err"; }
        }
        else if (go is Shell shl) shells = new Shell[] { shl };
        else return;
        int faces = 0, edges = 0, open = 0;
        SortedDictionary<string, int> surfaces = new SortedDictionary<string, int>(StringComparer.Ordinal);
        foreach (Shell sh in shells)
        {
            faces += sh.Faces.Length;
            edges += sh.Edges.Length;
            try { open += sh.OpenEdgesExceptPoles.Length; } catch (Exception) { open = -1; }
            foreach (Face fc in sh.Faces)
            {
                string type = fc.Surface?.GetType().Name ?? "null";
                surfaces[type] = surfaces.TryGetValue(type, out int n) ? n + 1 : 1;
            }
        }
        string name = go is Solid s ? s.Name : (go as Shell).Name;
        sb.AppendLine($"# body {go.GetType().Name} '{OneLine(name)}' faces {faces} edges {edges} open {open} volume {volume} surfaces {string.Join(",", surfaces.Select(kv => kv.Key + ":" + kv.Value))}");
    }
}
