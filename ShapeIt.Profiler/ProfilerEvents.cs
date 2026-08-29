using System.Diagnostics.Tracing;

namespace ShapeIt.Profiler
{
    /// <summary>
    /// Marks the phases of a profiling run on the ETW timeline.
    /// <para>
    /// This is what makes a CPU profile readable: PerfView (and anything else reading the events, e.g.
    /// dotnet-trace) shows these markers next to the CPU stacks, so a time range can be restricted to
    /// "iteration 3" instead of being guessed from the shape of the CPU graph. Enable the provider with
    /// <c>/Providers=*ShapeIt-Profiler</c>; without a listener the events cost nothing.
    /// </para>
    /// </summary>
    [EventSource(Name = "ShapeIt-Profiler")]
    internal sealed class ProfilerEvents : EventSource
    {
        public static readonly ProfilerEvents Log = new ProfilerEvents();

        private ProfilerEvents() { }

        [Event(1, Level = EventLevel.Informational, Message = "case {0} start")]
        public void CaseStart(string name) => WriteEvent(1, name);

        [Event(2, Level = EventLevel.Informational, Message = "case {0} done")]
        public void CaseStop(string name) => WriteEvent(2, name);

        /// <summary><paramref name="kind"/> is "warmup" or "measured" - warmup iterations still contain JIT
        /// and first-touch costs and must not be read as part of the result.</summary>
        [Event(3, Level = EventLevel.Informational, Message = "{0} iteration {1} start")]
        public void IterationStart(string kind, int iteration) => WriteEvent(3, kind, iteration);

        [Event(4, Level = EventLevel.Informational, Message = "{0} iteration {1} done after {2} ms")]
        public void IterationStop(string kind, int iteration, long elapsedMilliseconds)
            => WriteEvent(4, kind, iteration, elapsedMilliseconds);
    }
}
