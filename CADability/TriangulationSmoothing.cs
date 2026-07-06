using System;

namespace CADability
{
    // Lets a caller (e.g. STL export) request a more thorough - but slower - quality smoothing
    // pass than the interactive default, without affecting concurrently running triangulations
    // on other threads (e.g. background re-triangulation while the user zooms). This is a
    // per-thread setting, not a global one, precisely because such background recalculation runs
    // in parallel: a global setting would race with it. Usage:
    //   using (TriangulationSmoothing.UseThoroughSmoothing()) { project.ExportToSTL(fileName); }
    // Kept in its own file (originally part of Triangulation.cs) so it stays available while
    // Triangulation.cs is excluded from compilation during the CDTriangulation migration.
    public static class TriangulationSmoothing
    {
        [ThreadStatic] private static double? currentMinRelativeImprovement;
        internal static double CurrentOrDefault => currentMinRelativeImprovement ?? 0.05;
        // minRelativeImprovement: minimum fraction (0..1) the summed surface deviation of a
        // vertex's incident edges must shrink by for a smoothing move to be accepted. 0.05 (the
        // interactive default) stops early once further gains become marginal; use a lower value
        // (e.g. 0.0 for "accept any improvement") for a more uniform, but slower, result - useful
        // when exporting for a slicer, where the display speed does not matter.
        public static IDisposable UseThoroughSmoothing(double minRelativeImprovement = 0.0)
        {
            double? previous = currentMinRelativeImprovement;
            currentMinRelativeImprovement = minRelativeImprovement;
            return new RestoreAction(() => currentMinRelativeImprovement = previous);
        }
        private class RestoreAction : IDisposable
        {
            private readonly Action action;
            public RestoreAction(Action action) { this.action = action; }
            public void Dispose() => action();
        }
    }
}
