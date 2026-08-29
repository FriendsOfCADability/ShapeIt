using Avalonia.Threading;
using CADability;
using CADability.Avalonia;
using System;
using System.Threading;

namespace ShapeIt
{
    /// <summary>
    /// Avalonia counterpart of the WinForms <c>OffscreenPainter</c> (ShapeIt/OffscreenPainter.cs, excluded
    /// from this head): supplies the <see cref="PaintToOpenGL"/> used for offscreen rendering and runs the
    /// rendering where its OpenGL context is current.
    /// <para>
    /// The WinForms version can do two things this one cannot. There, the WGL context is made current on the
    /// user interface thread once and stays current, so any callback on that thread may issue GL calls; and a
    /// hidden <c>Control</c> with a context of its own can be created for a session that has no window at all.
    /// Avalonia owns the context inside <c>OpenGlControlBase</c> and makes it current only for the duration of
    /// <c>OnOpenGlInit</c> / <c>OnOpenGlRender</c>, and it offers no supported way to create a second one. So
    /// there is no headless fallback here, and the work has to be handed to the canvas' render pass.
    /// </para>
    /// <para>
    /// <see cref="ICanvas.OnPaintDone"/> is fired at the end of <c>OnOpenGlRender</c>, with the context
    /// current and the visible frame already drawn - exactly the window in which an offscreen framebuffer can
    /// be bound, filled and read back without disturbing what is on screen. A frame is requested through
    /// <see cref="ICanvas.Invalidate"/> and this class waits for that event.
    /// </para>
    /// </summary>
    internal static class OffscreenPainter
    {
        /// <summary>
        /// How long to wait for the render pass that does the offscreen drawing. A canvas that is not visible
        /// - a minimized window, a view that was never shown - is never rendered, and without a limit the
        /// call would never return.
        /// </summary>
        private static readonly TimeSpan RenderTimeout = TimeSpan.FromSeconds(10);

        /// <summary>
        /// Runs <paramref name="render"/> on a painter that can draw offscreen. Returns false when there is
        /// no such painter, with <paramref name="problem"/> saying why - a session without a 3-D view, or a
        /// machine without a usable OpenGL driver, is a normal outcome here and must not fail the caller.
        /// Exceptions thrown by <paramref name="render"/> itself are passed on: those are defects, not a
        /// missing capability.
        /// </summary>
        public static bool Run(IFrame? frame, Action<PaintToOpenGL> render, out string? problem)
        {
            problem = null;
            if (!TryCanvasPainter(frame, out PaintToOpenGL? painter, out ICanvas? canvas))
            {
                problem = "no offscreen OpenGL context: there is no OpenGL view to borrow one from";
                return false;
            }

            Exception? failure = null;
            bool executed = false;
            // Fired on the render thread of the canvas with the GL context current; see the class comment.
            Action<ICanvas> handler = null!;
            using CancellationTokenSource finished = new CancellationTokenSource();
            handler = _ =>
            {
                canvas!.OnPaintDone -= handler;
                try { render(painter!); }
                catch (Exception e) { failure = e; }
                finally
                {
                    executed = true;
                    // Ends the nested loop below, or releases the waiting background thread.
                    try { finished.Cancel(); } catch (ObjectDisposedException) { }
                }
            };
            canvas!.OnPaintDone += handler;

            try
            {
                canvas.Invalidate();
                Wait(finished.Token);
            }
            finally
            {
                canvas.OnPaintDone -= handler;
            }

            if (!executed)
            {
                problem = "no offscreen image: the OpenGL view was not rendered within " +
                          RenderTimeout.TotalSeconds.ToString(System.Globalization.CultureInfo.InvariantCulture) +
                          " s";
                return false;
            }
            // Rethrows what the rendering threw, keeping the original stack trace.
            if (failure != null) System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(failure).Throw();
            return true;
        }

        /// <summary>
        /// Waits until <paramref name="finished"/> is signalled, or the timeout expires.
        /// <para>
        /// MCP tool calls are marshalled to the user interface thread (see MCPHttpServer, which uses
        /// <c>SynchronizationContext.Send</c>), so this is normally called on that very thread - and blocking
        /// it would prevent the render pass we are waiting for. A nested dispatcher loop keeps the thread
        /// pumping until the render is done, the same async-to-sync bridge CadFrame.RunDialogSync and
        /// CadCanvas.DoDragDrop already use. When called from a background thread there is nothing to pump,
        /// and a plain wait is both sufficient and cheaper.
        /// </para>
        /// </summary>
        private static void Wait(CancellationToken finished)
        {
            if (!Dispatcher.UIThread.CheckAccess())
            {
                finished.WaitHandle.WaitOne(RenderTimeout);
                return;
            }

            using CancellationTokenSource timeout = new CancellationTokenSource(RenderTimeout);
            using CancellationTokenSource stop =
                CancellationTokenSource.CreateLinkedTokenSource(finished, timeout.Token);
            try { Dispatcher.UIThread.MainLoop(stop.Token); }
            catch (OperationCanceledException) { /* the expected way out of the nested loop */ }
        }

        /// <summary>The painter of the active view, when the application is running with a user interface.</summary>
        private static bool TryCanvasPainter(IFrame? frame, out PaintToOpenGL? painter, out ICanvas? canvas)
        {
            painter = null;
            canvas = null;
            if (frame == null) return false;
            try
            {
                ICanvas? active = frame.ActiveView?.Canvas;
                // It has to be PaintToOpenGL: the browser's WebGL painter and any 2-D painter cannot render
                // into a framebuffer object here.
                if (active?.PaintTo3D is not PaintToOpenGL canvasPainter) return false;
                painter = canvasPainter;
                canvas = active;
                return true;
            }
            catch (Exception)
            {
                // A frame without an active view does not necessarily return null, it may throw. Either way
                // there is no canvas painter, and there is no image.
                return false;
            }
        }
    }
}
