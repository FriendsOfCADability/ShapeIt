using System.Collections.Generic;
using CADability;
using CADability.GeoObject;

namespace ShapeIt
{
    /// <summary>
    /// Browser no-op stub for the desktop <c>WorkspaceRenderer</c>.
    /// <para>
    /// The real desktop implementation (ShapeIt/WorkspaceRenderer.cs, excluded from the
    /// browser compile) renders GeoObjects to a PNG via an offscreen
    /// <c>PaintToOpenGLModern</c> framebuffer and marshals the calls through
    /// <c>System.Windows.Forms.Control.Invoke</c>. Neither WinForms nor the offscreen FBO
    /// path is available inside Avalonia.Browser (WASM), so this stub simply returns
    /// <c>null</c> (no image) — exactly the "no suitable painter" fallback the caller in
    /// <c>MCPServer.InspectSceneImpl</c> already handles gracefully.
    /// </para>
    /// <para>
    /// This type exists only so the shared <c>ShapeIt/MCPServer.cs</c> compiles for the
    /// browser head. The signature mirrors the desktop method exactly.
    /// </para>
    /// </summary>
    internal static class WorkspaceRenderer
    {
        /// <summary>Always returns <c>null</c> in the browser: no offscreen rendering is available.</summary>
        public static string? RenderToPngBase64(
            IFrame frame,
            IEnumerable<(IGeoObject obj, CADability.Substitutes.Color color)> coloredObjects,
            GeoVector viewDirection,
            int width,
            int height)
        {
            return null;
        }
    }
}
