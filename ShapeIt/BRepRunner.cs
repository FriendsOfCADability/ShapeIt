using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using CADability;
using CADability.GeoObject;

namespace ShapeIt
{
    public class BRepRunResult
    {
        public Shell[] Shells { get; set; } = Array.Empty<Shell>();
        public Exception? Error { get; set; }
        public bool TimedOut { get; set; }
        public long ElapsedMilliseconds { get; set; }

        public string Status => TimedOut ? "timeout" : Error != null ? "exception" : "ok";

        /// <summary>Tier 0 of the comparison: is this a usable result at all?</summary>
        public bool IsValid => Status == "ok" && Shells.Length > 0 && BRepRunner.AllShellsAreValid(Shells);

        public string Describe()
        {
            if (TimedOut) return "timed out";
            if (Error != null) return Error.GetType().Name + ": " + BRepSummary.FirstLine(Error.Message);
            if (Shells.Length == 0) return "no resulting shell";
            if (!BRepRunner.AllShellsAreValid(Shells)) return "the resulting shell(s) are not consistent/closed";
            return $"{Shells.Length} shell(s)";
        }
    }

    /// <summary>
    /// Executes a <see cref="BRepCase"/>.
    /// <para>
    /// <see cref="Execute"/> runs on the calling thread - that is what AutoDebug uses, so stepping and
    /// conditional breakpoints behave normally. <see cref="Run"/> wraps it in a background thread with a time
    /// budget for the regression tests, because a BRep bug quite often shows up as an endless loop and a single
    /// hanging case must not take the whole test run with it. A timed out thread cannot be killed in .NET, it is
    /// simply left behind as a background thread.
    /// </para>
    /// </summary>
    public static class BRepRunner
    {
        public static BRepRunResult Run(BRepCase testCase, int timeoutSeconds)
        {
            BRepRunResult result = new BRepRunResult();
            Stopwatch watch = Stopwatch.StartNew();
            Thread thread = new Thread(() =>
            {
                try { result.Shells = Execute(testCase); }
                catch (Exception e) { result.Error = e; }
            }, 32 * 1024 * 1024) // BRep code recurses deeply, give it a generous stack
            {
                IsBackground = true,
                Name = "BRep " + testCase.Name
            };
            thread.Start();
            if (!thread.Join(TimeSpan.FromSeconds(timeoutSeconds)))
            {
                result.TimedOut = true;
                result.Shells = Array.Empty<Shell>();
            }
            watch.Stop();
            result.ElapsedMilliseconds = watch.ElapsedMilliseconds;
            return result;
        }

        /// <summary>Performs the operation of the case on the calling thread.</summary>
        public static Shell[] Execute(BRepCase testCase)
        {
            switch (testCase.Operation)
            {
                case BRepOperationKind.Union:
                    return FoldUnion(testCase.Operands);
                case BRepOperationKind.Difference:
                    return Fold(testCase.Operands, BooleanOperation.Operation.difference);
                case BRepOperationKind.Intersection:
                    return Fold(testCase.Operands, BooleanOperation.Operation.intersection);
                case BRepOperationKind.UniteAll:
                    return UniteAll(testCase.Operands);
                case BRepOperationKind.RoundEdges:
                    {
                        RoundEdges round = new RoundEdges(testCase.Operands[0], testCase.MarkedEdges, testCase.Parameter);
                        Shell? shell = round.Execute();
                        return shell == null ? Array.Empty<Shell>() : new[] { shell };
                    }
                case BRepOperationKind.ChamferEdges:
                    {
                        double length2 = double.IsNaN(testCase.SecondaryParameter) ? testCase.Parameter : testCase.SecondaryParameter;
                        ChamferEdges chamfer = new ChamferEdges(testCase.Operands[0], testCase.MarkedEdges, testCase.Parameter, length2);
                        Shell? shell = chamfer.Execute();
                        return shell == null ? Array.Empty<Shell>() : new[] { shell };
                    }
                default:
                    throw new InvalidOperationException($"cannot execute operation {testCase.Operation}");
            }
        }

        public static bool AllShellsAreValid(IEnumerable<Shell> shells)
        {
            foreach (Shell shell in shells)
            {
                try
                {
                    if (!shell.CheckConsistency()) return false;
                    if (shell.OpenEdgesExceptPoles.Length > 0) return false;
                    // the same explicit precision the summary uses, so validity and baseline agree
                    if (shell.Volume(ShellMetrics.PrecisionFor(shell)) <= 0.0) return false;
                }
                catch (Exception) { return false; }
            }
            return true;
        }

        private static Shell[] Boolean(Shell first, Shell second, BooleanOperation.Operation operation)
        {
            BooleanOperation booleanOperation = new BooleanOperation();
            booleanOperation.SetShells(first, second, operation);
            return booleanOperation.Execute() ?? Array.Empty<Shell>();
        }

        /// <summary>
        /// Applies the operation to Operand1 and then, one after the other, to every further operand, in the
        /// order they appear in the model: A - B1 - B2 - ... A step may split its input, so every shell
        /// produced so far takes part in the next step. When nothing is left, the remaining operands are
        /// skipped - subtracting from nothing stays nothing.
        /// <para>With exactly two operands this is a single call, i.e. exactly what it did before.</para>
        /// </summary>
        private static Shell[] Fold(List<Shell> operands, BooleanOperation.Operation operation)
        {
            List<Shell> current = new List<Shell> { operands[0] };
            for (int i = 1; i < operands.Count && current.Count > 0; i++)
            {
                List<Shell> next = new List<Shell>();
                foreach (Shell shell in current) next.AddRange(Boolean(shell, operands[i], operation));
                current = next;
            }
            return current.ToArray();
        }

        /// <summary>
        /// Unites the operands one after the other, in the order they appear in the model. Unlike a difference
        /// the accumulated result has to stay a single shell, because only then can the next operand be united
        /// with it - uniting it with each part separately would duplicate that operand. When a step does not
        /// yield exactly one shell, its result is handed back as it is and the remaining operands are not
        /// applied; the summary then shows how many shells came out of it.
        /// </summary>
        private static Shell[] FoldUnion(List<Shell> operands)
        {
            Shell accumulated = operands[0];
            for (int i = 1; i < operands.Count; i++)
            {
                Shell[] united = Boolean(accumulated, operands[i], BooleanOperation.Operation.union);
                if (united.Length != 1) return united;
                accumulated = united[0];
            }
            return new[] { accumulated };
        }

        /// <summary>
        /// Unites all operands one after the other. A pair that does not unite (yet) goes back to the end of the
        /// queue, because a later union may make it touch the accumulated solid. When a full round through the
        /// queue makes no progress at all, the remaining solids cannot be united and that is reported.
        /// </summary>
        private static Shell[] UniteAll(List<Shell> operands)
        {
            Queue<Shell> queue = new Queue<Shell>(operands.Skip(1));
            Shell accumulated = operands[0];
            int failuresSinceProgress = 0;
            while (queue.Count > 0)
            {
                Shell next = queue.Dequeue();
                Shell[] united = Boolean(next, accumulated, BooleanOperation.Operation.union);
                if (united.Length == 1)
                {
                    accumulated = united[0];
                    failuresSinceProgress = 0;
                }
                else
                {
                    queue.Enqueue(next);
                    if (++failuresSinceProgress > queue.Count)
                        throw new InvalidOperationException($"{queue.Count} of {operands.Count} solids could not be united");
                }
            }
            return new[] { accumulated };
        }
    }
}
