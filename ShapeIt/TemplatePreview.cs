using System;
using System.Collections.Generic;
using System.Threading;
using CADability.GeoObject;

namespace ShapeIt
{
    /// <summary>What one preview computation produced.</summary>
    internal sealed class TemplatePreviewResult
    {
        /// <summary>The solids of the template result, empty when it produced none.</summary>
        public List<Solid> Solids { get; set; } = new List<Solid>();
        /// <summary>Why the template did not produce a result, null when it did. This is what a failed
        /// assert.check reports, i.e. the message the template author wrote for invalid parameters.</summary>
        public string? Error { get; set; }
        /// <summary>The parameter values this result belongs to.</summary>
        public Dictionary<string, object> Parameters { get; set; } = new Dictionary<string, object>();
    }

    /// <summary>
    /// Computes a template in the background while the user edits its parameters, so the action can show the
    /// current shape instead of only building it on OK.
    /// <para>
    /// One worker thread, one computation at a time, latest wins: a change while a computation runs cancels
    /// it and starts over with the new values once it has stopped. Serializing matters beyond CPU cost - the
    /// BRep code and the server's named items are not safe to touch from two threads at once, which is what
    /// <see cref="MCPServer.GeometryLock"/> is for.
    /// </para>
    /// <para>
    /// Cancellation is checked between the recorded steps of the template. A single long step, typically a
    /// boolean operation, still runs to its end; the BRep core cannot be interrupted. So the worst case
    /// latency after a keystroke is one boolean operation, not the whole template.
    /// </para>
    /// <para>
    /// The worker never touches the display. It hands finished solids to the UI thread, which puts them into
    /// the feedback list and repaints - so PaintTo3D and OpenGL stay single threaded. Triangulating the
    /// result here as well would be the next step; see the note at the end of Compute.
    /// </para>
    /// </summary>
    internal sealed class TemplatePreview : IDisposable
    {
        private readonly MCPServer mcpServer;
        private readonly string templateName;
        private readonly SynchronizationContext uiContext;
        private readonly Action<TemplatePreviewResult> onResult;

        private readonly object sync = new object();
        private Dictionary<string, object>? pending;    // values waiting to be computed, null when idle
        private bool workerRunning;
        private int generation;                          // bumped by every request, identifies the newest one
        private CancellationTokenSource? running;        // cancels the computation in flight
        private bool disposed;

        /// <param name="uiContext">captured on the UI thread; results are posted back through it</param>
        /// <param name="onResult">called on the UI thread, only for the most recent request</param>
        public TemplatePreview(MCPServer mcpServer, string templateName, SynchronizationContext uiContext,
            Action<TemplatePreviewResult> onResult)
        {
            this.mcpServer = mcpServer;
            this.templateName = templateName;
            this.uiContext = uiContext;
            this.onResult = onResult;
        }

        /// <summary>
        /// Asks for a preview of these values. Returns immediately. Superseding an earlier request that has
        /// not finished is the normal case, not an error - only the newest one is reported back.
        /// </summary>
        public void Request(Dictionary<string, object> parameterValues)
        {
            lock (sync)
            {
                if (disposed) return;
                generation++;
                pending = new Dictionary<string, object>(parameterValues); // copy: the caller keeps editing it
                running?.Cancel();
                if (workerRunning) return; // the worker will pick "pending" up when it comes round
                workerRunning = true;
            }
            // A generous stack, like BRepRunner uses: the BRep code recurses deeply.
            Thread thread = new Thread(Work, 32 * 1024 * 1024)
            {
                IsBackground = true,
                Name = "TemplatePreview " + templateName
            };
            thread.Start();
        }

        private void Work()
        {
            while (true)
            {
                Dictionary<string, object> values;
                int myGeneration;
                CancellationTokenSource cts;
                lock (sync)
                {
                    if (pending == null || disposed)
                    {
                        workerRunning = false;
                        return;
                    }
                    values = pending;
                    pending = null;
                    myGeneration = generation;
                    cts = new CancellationTokenSource();
                    running = cts;
                }

                TemplatePreviewResult result = Compute(values, cts.Token);

                lock (sync)
                {
                    if (running == cts) running = null;
                    cts.Dispose();
                    // Superseded while we were computing, or shut down: drop it and take the next round.
                    if (myGeneration != generation || disposed) continue;
                }
                uiContext.Post(_ => { if (!disposed) onResult(result); }, null);
            }
        }

        private TemplatePreviewResult Compute(Dictionary<string, object> values, CancellationToken token)
        {
            TemplatePreviewResult result = new TemplatePreviewResult { Parameters = values };
            try
            {
                object? produced;
                lock (mcpServer.GeometryLock)
                {
                    token.ThrowIfCancellationRequested();
                    bool oldSuppress = mcpServer.SuppressDialogs;
                    bool oldStop = mcpServer.stopExecution;
                    // No message boxes while the user is typing - a dialog per keystroke would be unusable,
                    // and it would pop up on the UI thread while this one holds the lock. The message goes
                    // to the input field instead; OnDone runs the template again with dialogs enabled.
                    mcpServer.SuppressDialogs = true;
                    mcpServer.stopExecution = false;
                    try
                    {
                        produced = mcpServer.ExecuteTemplate(templateName, values, token);
                    }
                    finally
                    {
                        mcpServer.SuppressDialogs = oldSuppress;
                        mcpServer.stopExecution = oldStop;
                    }
                }

                foreach (Solid solid in AsSolids(produced))
                {
                    token.ThrowIfCancellationRequested();
                    // The result is triangulated lazily by the first repaint, on the UI thread. Doing it here
                    // would be better - it is the one part of the work that still lands on the UI thread and
                    // it shows as a stutter on heavy templates - but Shell.PreCalcTriangulation is internal to
                    // CADability. Making it public is the one change needed to move it into this thread.
                    result.Solids.Add(solid);
                }
                if (result.Solids.Count == 0 && result.Error == null)
                    result.Error = "the template produced no solid";
            }
            catch (OperationCanceledException)
            {
                result.Error = null; // superseded; the caller drops this result anyway
            }
            catch (Exception e)
            {
                result.Solids.Clear();
                result.Error = ShortMessage(e.Message);
            }
            return result;
        }

        private static IEnumerable<Solid> AsSolids(object? produced)
        {
            switch (produced)
            {
                case Solid single: yield return single; break;
                case IEnumerable<Solid> many:
                    foreach (Solid s in many) yield return s;
                    break;
            }
        }

        /// <summary>
        /// A failing rpc.batch reports the whole batch protocol; an input field cannot show that. Keep the
        /// part that names what went wrong and drop the appended result dump.
        /// </summary>
        private static string ShortMessage(string message)
        {
            if (string.IsNullOrEmpty(message)) return "the template failed";
            int dump = message.IndexOf(" Results: ", StringComparison.Ordinal);
            if (dump > 0) message = message.Substring(0, dump);
            message = message.Replace("\r", " ").Replace("\n", " ").Trim();
            const int limit = 300;
            return message.Length <= limit ? message : message.Substring(0, limit) + "...";
        }

        /// <summary>
        /// Stops reporting and cancels what is in flight. A computation that is already inside a boolean
        /// operation cannot be stopped and runs to its end on its background thread; its result is dropped.
        /// </summary>
        public void Dispose()
        {
            lock (sync)
            {
                disposed = true;
                pending = null;
                running?.Cancel();
            }
        }
    }
}
