using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;

namespace ShapeIt
{
    /// <summary>
    /// Counts the steps of a running template, for a progress display like "17/25 (3/8)".
    /// <para>
    /// Every place that works through a list of steps - the template itself, a nested template.instantiate,
    /// an rpc.batch, the iterations of pattern.by_formula_solids - opens a level with <see cref="Level"/> and
    /// calls <see cref="Step"/> before each step. The levels nest the way the calls nest, the outermost one
    /// comes first in the text.
    /// </para>
    /// <para>
    /// Used on the thread that runs the template, under <see cref="MCPServer.GeometryLock"/>. The text is
    /// handed out through the callback and must be marshalled to the UI thread by the receiver.
    /// </para>
    /// </summary>
    internal sealed class TemplateProgress
    {
        private sealed class Counter
        {
            public int Index;      // the step in progress, 1-based; 0 before the first one
            public readonly int Total;
            public Counter(int total) { Total = total; }
        }

        private sealed class LevelScope : IDisposable
        {
            private readonly TemplateProgress owner;
            private readonly Counter counter;
            public LevelScope(TemplateProgress owner, Counter counter)
            {
                this.owner = owner;
                this.counter = counter;
            }
            public void Dispose()
            {
                // levels end in reverse order, an exception unwinds them the same way
                int i = owner.levels.LastIndexOf(counter);
                if (i >= 0) owner.levels.RemoveRange(i, owner.levels.Count - i);
            }
        }

        private readonly List<Counter> levels = new List<Counter>();
        private readonly CancellationToken token;
        private readonly Action<string> changed;

        /// <param name="token">checked at every step, so a cancellation also stops nested loops</param>
        /// <param name="changed">called with the new text after every step, on the template's thread</param>
        public TemplateProgress(CancellationToken token, Action<string> changed)
        {
            this.token = token;
            this.changed = changed;
        }

        /// <summary>
        /// Opens a level of <paramref name="total"/> steps; disposing the result closes it.
        /// </summary>
        public IDisposable Level(int total)
        {
            Counter counter = new Counter(total);
            levels.Add(counter);
            return new LevelScope(this, counter);
        }

        /// <summary>
        /// The next step of the innermost level begins.
        /// </summary>
        /// <exception cref="OperationCanceledException">the computation was cancelled</exception>
        public void Step()
        {
            token.ThrowIfCancellationRequested();
            if (levels.Count == 0) return;
            levels[levels.Count - 1].Index++;
            changed(Text);
        }

        /// <summary>
        /// "17/25" for one level, "17/25 (3/8)" for two, "17/25 (3/8 · 2/5)" for three. Deeper nestings show
        /// the outermost and the two innermost levels. Levels of a single step carry no information and are left out.
        /// </summary>
        public string Text
        {
            get
            {
                List<Counter> shown = levels.FindAll(c => c.Total > 1);
                if (shown.Count == 0) return "";
                StringBuilder sb = new StringBuilder();
                Append(sb, shown[0]);
                if (shown.Count > 1)
                {
                    sb.Append(" (");
                    int first = 1;
                    if (shown.Count > 3)
                    {
                        sb.Append("… ");
                        first = shown.Count - 2;
                    }
                    for (int i = first; i < shown.Count; i++)
                    {
                        if (i > first) sb.Append(" · ");
                        Append(sb, shown[i]);
                    }
                    sb.Append(')');
                }
                return sb.ToString();
            }
        }

        private static void Append(StringBuilder sb, Counter c)
        {
            sb.Append(Math.Max(c.Index, 1)).Append('/').Append(c.Total);
        }
    }
}
