using CADability;
using CADability.Forms.NET8;
using CADability.GeoObject;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Windows.Forms;

namespace ShapeIt
{
    /// <summary>
    /// Renders a collection of GeoObjects to a PNG bitmap using the existing
    /// PaintToOpenGLModern instance that is attached to the main CadCanvas.
    ///
    /// Each object is rendered with an explicit color override so the LLM can
    /// tell objects apart regardless of any stored color attribute.
    ///
    /// All OpenGL calls are marshalled to the UI thread via Control.Invoke so
    /// the method is safe to call from the MCP HTTP handler thread.
    /// </summary>
    internal static class WorkspaceRenderer
    {
        /// <summary>
        /// Renders <paramref name="coloredObjects"/> into a <paramref name="width"/> ×
        /// <paramref name="height"/> bitmap and returns it as a base-64 encoded PNG string,
        /// or <c>null</c> if no suitable painter is available.
        /// </summary>
        /// <param name="frame">The application frame (owns the active view and canvas).</param>
        /// <param name="coloredObjects">
        /// Geometry paired with the color to use for rendering. The color is applied as an
        /// override so it takes precedence over any color attribute stored on the object.
        /// </param>
        /// <param name="viewDirection">
        /// Parallel-projection view direction in world space.
        /// </param>
        /// <param name="width">Output bitmap width in pixels.</param>
        /// <param name="height">Output bitmap height in pixels.</param>
        public static string? RenderToPngBase64(
            IFrame frame,
            IEnumerable<(IGeoObject obj, CADability.Substitutes.Color color)> coloredObjects,
            GeoVector viewDirection,
            int width,
            int height)
        {
            // Resolve the painter from the active view's canvas.
            // Must be PaintToOpenGLModern – GDI cannot do proper 3-D hidden-line rendering.
            var canvas = frame.ActiveView?.Canvas;
            if (canvas?.PaintTo3D is not PaintToOpenGLModern painter)
                return null;

            // CadCanvas is a Windows Forms Control; we need it for Control.Invoke.
            if (canvas is not Control control)
                return null;

            System.Drawing.Bitmap? bitmap = null;

            control.Invoke((Action)(() =>
            {
                bitmap = RenderOnUIThread(painter, coloredObjects, viewDirection, width, height);
            }));

            if (bitmap == null) return null;

            using (bitmap)
            using (var ms = new MemoryStream())
            {
                bitmap.Save(ms, ImageFormat.Png);
                return Convert.ToBase64String(ms.ToArray());
            }
        }

        // ── called exclusively on the UI thread ──────────────────────────────

        private static System.Drawing.Bitmap? RenderOnUIThread(
            PaintToOpenGLModern painter,
            IEnumerable<(IGeoObject obj, CADability.Substitutes.Color color)> coloredObjects,
            GeoVector viewDirection,
            int width,
            int height)
        {
            var list = new List<(IGeoObject obj, CADability.Substitutes.Color color)>(coloredObjects);
            if (list.Count == 0) return null;

            // Combined 3-D bounding box for depth range and Z-fit.
            BoundingBox boundingCube = BoundingBox.EmptyBoundingBox;
            foreach (var (obj, _) in list)
                boundingCube.MinMax(obj.GetBoundingCube());
            if (boundingCube.IsEmpty) return null;

            // Build a parallel projection from the requested view direction.
            // Use world-Z as "up" hint; fall back to world-Y when nearly parallel to Z.
            GeoVector up = Math.Abs(viewDirection.z) < 0.9
                ? new GeoVector(0, 0, 1)
                : new GeoVector(0, 1, 0);
            var projection = new Projection(viewDirection, up);

            // Zoom-to-fit: project objects into 2-D, then fit the projection to the bitmap.
            var ext2D = BoundingRect.EmptyBoundingRect;
            foreach (var (obj, _) in list)
                ext2D.MinMax(obj.GetExtent(projection, ExtentPrecision.Raw));
            if (!ext2D.IsEmpty())
                ext2D *= 1.1; // 10 % margin
            projection.SetPlacement(new CADability.Substitutes.Rectangle(0, 0, width, height), ext2D);

            IPaintTo3D p = painter;

            painter.BeginOffscreen(width, height);
            try
            {
                p.MakeCurrent();
                p.Clear(CADability.Substitutes.Color.White);
                p.SetProjection(projection, boundingCube);
                p.UseZBuffer(true);

                // ── Pass 1: Faces ──────────────────────────────────────────────────
                // PaintMode.FacesOnly sets _paintEdges=false and translates faces
                // slightly along the view direction so they don't Z-fight with edges.
                p.PaintFaces(PaintTo3D.PaintMode.FacesOnly);
                foreach (var (obj, color) in list)
                {
                    p.SetColor(color, lockColor: 1);
                    obj.PaintTo3D(p);
                    p.SetColor(color, lockColor: -1);
                }

                // ── Pass 2: Edges ──────────────────────────────────────────────────
                // PaintMode.CurvesOnly sets _paintSurfaces=false, _paintEdges=true.
                // We lock the color to black so Shell.PaintTo3D cannot override it
                // with the object's own colorDef (which would make edges invisible
                // against the same-colored face).
                p.PaintFaces(PaintTo3D.PaintMode.CurvesOnly);
                foreach (var (obj, _) in list)
                {
                    p.SetColor(CADability.Substitutes.Color.Black, lockColor: 1);
                    obj.PaintTo3D(p);
                    p.SetColor(CADability.Substitutes.Color.Black, lockColor: -1);
                }

                p.FinishPaint();

                return painter.EndOffscreenAsBitmap();
            }
            catch
            {
                // Always clean up FBO and release color lock even on failure.
                try { p.SetColor(CADability.Substitutes.Color.White, lockColor: -1); } catch { }
                try { painter.EndOffscreenAsBitmap()?.Dispose(); } catch { }
                throw;
            }
        }
    }
}
