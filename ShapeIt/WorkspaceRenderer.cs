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
    /// Renders a collection of GeoObjects to a PNG bitmap with a PaintToOpenGLModern.
    ///
    /// Each object is rendered with an explicit color override so the LLM can
    /// tell objects apart regardless of any stored color attribute.
    ///
    /// Where the painter comes from and on which thread the OpenGL calls happen is
    /// <see cref="OffscreenPainter"/>'s business: the canvas of the active view when the application has a
    /// user interface, a hidden window of its own when it has not.
    /// </summary>
    internal static class WorkspaceRenderer
    {
        /// <summary>
        /// Renders <paramref name="coloredObjects"/> into a <paramref name="width"/> ×
        /// <paramref name="height"/> bitmap and returns it as a base-64 encoded PNG string,
        /// or <c>null</c> if no image could be produced.
        /// </summary>
        /// <param name="frame">
        /// The application frame, which owns the active view and its canvas. May be null: without a user
        /// interface there is no frame, and the painter is then created without one.
        /// </param>
        /// <param name="coloredObjects">
        /// Geometry paired with the color to use for rendering. The color is applied as an
        /// override so it takes precedence over any color attribute stored on the object.
        /// </param>
        /// <param name="viewDirection">
        /// Parallel-projection view direction in world space.
        /// </param>
        /// <param name="width">Output bitmap width in pixels.</param>
        /// <param name="height">Output bitmap height in pixels.</param>
        /// <param name="unavailableReason">
        /// Why no image was produced, null when one was. Worth reporting to the caller: "there is no
        /// picture" and "there is no picture because this machine has no usable OpenGL" are different
        /// answers, and only one of them is worth retrying.
        /// </param>
        public static string? RenderToPngBase64(
            IFrame? frame,
            IEnumerable<(IGeoObject obj, CADability.Substitutes.Color color)> coloredObjects,
            GeoVector viewDirection,
            int width,
            int height,
            out string? unavailableReason)
        {
            unavailableReason = null;
            var list = new List<(IGeoObject obj, CADability.Substitutes.Color color)>(coloredObjects);
            if (list.Count == 0)
            {
                unavailableReason = "nothing to render";
                return null;
            }

            BoundingBox boundingBox = BoundingBox.EmptyBoundingBox;
            foreach (var item in list)
            {
                boundingBox.MinMax(item.obj.GetExtent(0.0));
            }

            System.Drawing.Bitmap? bitmap = null;
            bool rendered = OffscreenPainter.Run(frame, painter =>
            {
                (painter as IPaintTo3D).Precision = boundingBox.Size / 1000.0;
                bitmap = RenderOnPainterThread(painter, list, viewDirection, width, height);
            }, out unavailableReason);

            if (!rendered) return null;
            if (bitmap == null)
            {
                unavailableReason = "the objects have no extent";
                return null;
            }

            using (bitmap)
            using (var ms = new MemoryStream())
            {
                bitmap.Save(ms, ImageFormat.Png);
                return Convert.ToBase64String(ms.ToArray());
            }
        }

        // ── called exclusively on the thread that owns the OpenGL context ────

        private static System.Drawing.Bitmap? RenderOnPainterThread(
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
            // The top direction rule is the same one CADability uses for its views
            // (see ProjectedModel.SetViewDirection and PaintToOpenGL.SetProjection):
            // world Z is the "up" hint, so the Z axis always points upwards in the image.
            // Only when looking along Z (top/bottom view), where that is impossible,
            // world Y is used instead. The test must be a direction comparison, not a
            // test on the z component, because viewDirection is not normalized.
            GeoVector up = Precision.SameDirection(viewDirection, GeoVector.ZAxis, false)
                ? GeoVector.YAxis
                : GeoVector.ZAxis;
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
