// Imports a STEP file with Project.ReadFromFile and dumps the resulting object hierarchy
// (type, name, bounding cube) so that different import variants can be compared.
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
        var sw = System.Diagnostics.Stopwatch.StartNew();
        try
        {
            Project pr = Project.ReadFromFile(file, "stp");
            GeoObjectList all = pr.GetActiveModel().AllObjects;
            sb.AppendLine($"# time {sw.Elapsed.TotalSeconds:F1}s, top level {all.Count}");
            BoundingBox ext = BoundingBox.EmptyBoundingBox;
            foreach (IGeoObject go in all) ext.MinMax(go.GetBoundingCube());
            sb.AppendLine("# extent " + Cube(ext));
            foreach (IGeoObject go in all) Dump(go, "", sb);
        }
        catch (Exception e)
        {
            sb.AppendLine("# EXCEPTION " + e.GetType().Name + ": " + e.Message);
        }
        File.WriteAllText(outFile, sb.ToString());
        return 0;
    }

    static string F(double d) => d.ToString("G6", CultureInfo.InvariantCulture);
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
}


