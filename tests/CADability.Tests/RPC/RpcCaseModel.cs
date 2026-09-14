using System;
using System.Collections.Generic;
using System.Linq;
using CADability.Attribute;
using CADability.GeoObject;
using ShapeIt;
using Path = System.IO.Path;

namespace CADability.Tests.Rpc
{
    /// <summary>
    /// Writes the bodies a case produced into a CADability project beside the case file, so that a recorded
    /// result can be looked at in ShapeIt instead of only being read as numbers.
    /// <para>
    /// One file per case, <c>&lt;case&gt;.cdb</c> next to <c>&lt;case&gt;.json</c>, written by a regenerate run
    /// so that the picture and the baseline always describe the same run. The bodies are the ones the baseline
    /// is taken from and they carry the baseline's names, so an entry in the file and a body in the project
    /// refer to the same thing - including the <c>name#0</c>, <c>name#1</c> of a result that is a list.
    /// </para>
    /// <para>
    /// The parts of a case are built at the same place and would sit inside one another, so every body is moved
    /// into its own slot in a row along x, in the order of the baseline. Only x changes: a body keeps the y and
    /// z it was modelled at, which is what makes two variants of the same shape comparable at a glance.
    /// </para>
    /// </summary>
    public static class RpcCaseModel
    {
        /// <summary>Gap between two bodies, relative to the size of the largest body of the case.</summary>
        private const double RelativeGap = 0.25;

        /// <summary>The project belonging to a case: the case file with a ".cdb" extension.</summary>
        public static string PathFor(RpcCase testCase) => Path.ChangeExtension(testCase.FilePath, ".cdb");

        /// <summary>
        /// Writes <see cref="PathFor"/> and returns the names it contains, or null when the run produced no
        /// geometry at all - a case whose result is legitimately empty has nothing to show, and an empty
        /// project would only be misleading.
        /// </summary>
        public static string? Write(RpcCase testCase, RpcRunResult run)
        {
            List<KeyValuePair<string, Shell>> bodies = run.Objects
                .Where(o => o.Value.Shell != null)
                .Select(o => new KeyValuePair<string, Shell>(o.Key, o.Value.Shell!))
                .ToList();
            if (bodies.Count == 0) return null;

            Project project = Project.CreateSimpleProject();
            Model model = project.GetActiveModel();
            Style style = project.StyleList.GetDefault(Style.EDefaultFor.Solids);
            double gap = bodies.Max(b => ShellMetrics.SizeOf(b.Value)) * RelativeGap;

            double cursor = 0.0;
            foreach (KeyValuePair<string, Shell> body in bodies)
            {
                // A copy: the shell belongs to the run that produced it, and moving the original would put the
                // body somewhere else than where its own summary was measured.
                Shell shell = (Shell)body.Value.Clone();
                BoundingBox box = shell.GetExtent(PrecisionOf(shell));
                Solid solid = Solid.MakeSolid(shell);
                solid.Name = body.Key;
                if (style != null) solid.Style = style;
                solid.Modify(ModOp.Translate(cursor - box.Xmin, 0.0, 0.0));
                model.Add(solid);
                cursor += box.XDiff + gap;
            }
            project.WriteToFile(PathFor(testCase));
            return string.Join(", ", bodies.Select(b => b.Key));
        }

        /// <summary>
        /// The mesh precision <see cref="ShellMetrics"/> derives from the size of a shell, guarded against a
        /// degenerate shell of size 0: GetExtent must not be called with a precision of 0.
        /// </summary>
        private static double PrecisionOf(Shell shell)
        {
            double precision = ShellMetrics.PrecisionFor(shell);
            return precision > 0.0 ? precision : 1e-6;
        }
    }
}
