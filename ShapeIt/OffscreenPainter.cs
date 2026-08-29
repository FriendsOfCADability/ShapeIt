using CADability;
using CADability.Forms.NET8;
using System;
using System.Collections.Concurrent;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ShapeIt
{
    /// <summary>
    /// Supplies the <see cref="PaintToOpenGLModern"/> used for offscreen rendering, and runs the rendering on
    /// the thread that owns its OpenGL context.
    /// <para>
    /// With a user interface the painter of the active view's canvas is used, exactly as before: it already
    /// has a context, and sharing it keeps the display lists. Without one - the MCP server driven from the
    /// RPC harness or from ShapeIt.Profiler, where there is no frame at all - a window of this class' own is
    /// created instead. A window that is never shown still has a real handle, and a real handle is all the
    /// driver needs to hand out a hardware accelerated core profile context; the rendering itself goes into a
    /// framebuffer object, so neither the size nor the visibility of that window matters.
    /// </para>
    /// <para>
    /// A WGL context belongs to one thread. That is why nothing here hands the painter out: the caller passes
    /// in what it wants done, and it is executed where the context is current - through Control.Invoke on the
    /// user interface thread, or on this class' own render thread.
    /// </para>
    /// </summary>
    internal static class OffscreenPainter
    {
        private static HeadlessContext? headless;
        private static readonly object headlessLock = new object();

        /// <summary>
        /// Runs <paramref name="render"/> on a painter that can draw offscreen. Returns false when there is
        /// no such painter, with <paramref name="problem"/> saying why - a machine without a usable OpenGL
        /// driver, or a session without a desktop, is a normal outcome here and must not fail the caller.
        /// Exceptions thrown by <paramref name="render"/> itself are passed on: those are defects, not a
        /// missing capability.
        /// </summary>
        public static bool Run(IFrame? frame, Action<PaintToOpenGLModern> render, out string? problem)
        {
            problem = null;
            if (TryCanvasPainter(frame, out PaintToOpenGLModern? canvasPainter, out Control? canvasControl))
            {
                canvasControl!.Invoke((Action)(() => render(canvasPainter!)));
                return true;
            }

            HeadlessContext context = Headless();
            if (context.InitFailure != null)
            {
                problem = "no offscreen OpenGL context: " + context.InitFailure.GetType().Name + ": " +
                          BRepSummary.FirstLine(context.InitFailure.Message);
                return false;
            }
            context.Invoke(render);
            return true;
        }

        /// <summary>The painter of the active view, when the application is running with a user interface.</summary>
        private static bool TryCanvasPainter(IFrame? frame, out PaintToOpenGLModern? painter, out Control? control)
        {
            painter = null;
            control = null;
            if (frame == null) return false;
            try
            {
                ICanvas? canvas = frame.ActiveView?.Canvas;
                // It has to be PaintToOpenGLModern: GDI cannot do proper 3-D hidden line rendering.
                if (canvas?.PaintTo3D is not PaintToOpenGLModern canvasPainter) return false;
                // CadCanvas is a Windows Forms control, and its context belongs to the thread that owns it,
                // so the rendering has to be marshalled there with Control.Invoke.
                if (canvas is not Control owner || !owner.IsHandleCreated) return false;
                painter = canvasPainter;
                control = owner;
                return true;
            }
            catch (Exception)
            {
                // A frame without an active view does not necessarily return null, it may throw. Either way
                // there is no canvas painter, and the headless one takes over.
                return false;
            }
        }

        private static HeadlessContext Headless()
        {
            lock (headlessLock)
            {
                // Created once, kept forever - including a failed attempt. Retrying would create a window per
                // call on a machine where it cannot work anyway.
                return headless ??= new HeadlessContext();
            }
        }

        /// <summary>
        /// A thread of its own with a hidden window, an OpenGL context, and a painter on it. Everything that
        /// touches the context runs on that thread.
        /// </summary>
        private sealed class HeadlessContext
        {
            private readonly BlockingCollection<Action> queue = new BlockingCollection<Action>();
            private readonly ManualResetEventSlim initialized = new ManualResetEventSlim(false);
            private PaintToOpenGLModern? painter;

            /// <summary>Why there is no painter, null when there is one.</summary>
            public Exception? InitFailure { get; private set; }

            public HeadlessContext()
            {
                Thread thread = new Thread(Loop)
                {
                    IsBackground = true,
                    Name = "ShapeIt offscreen render"
                };
                // Creating a window requires a single threaded apartment.
                thread.SetApartmentState(ApartmentState.STA);
                thread.Start();
                initialized.Wait();
            }

            public void Invoke(Action<PaintToOpenGLModern> render)
            {
                TaskCompletionSource<bool> completed =
                    new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
                queue.Add(() =>
                {
                    try { render(painter!); completed.SetResult(true); }
                    catch (Exception e) { completed.SetException(e); }
                });
                // Rethrows what the rendering threw, with its original stack trace.
                completed.Task.GetAwaiter().GetResult();
            }

            private void Loop()
            {
                try
                {
                    // Touching Handle is what creates the window; it is never shown and never needs a
                    // message loop, because nothing is ever drawn to its surface.
                    Control control = new Control { Size = new Size(256, 256) };
                    _ = control.Handle;
                    PaintToOpenGLModern created = new PaintToOpenGLModern(1e-6);
                    // Fails on a machine or in a session where only the GDI software renderer is available:
                    // that one is OpenGL 1.1 and cannot compile the shaders this painter needs.
                    created.Init(control);
                    painter = created;
                }
                catch (Exception e)
                {
                    InitFailure = e;
                }
                finally
                {
                    initialized.Set();
                }

                if (painter == null) return;
                foreach (Action action in queue.GetConsumingEnumerable()) action();
            }
        }
    }
}
