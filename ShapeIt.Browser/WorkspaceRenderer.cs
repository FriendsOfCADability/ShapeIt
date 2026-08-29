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
    /// <c>PaintToOpenGLModern</c> framebuffer, obtained through <c>OffscreenPainter</c> (likewise
    /// excluded). The browser head draws through <c>PaintToWebGL</c>, which talks to the canvas by
    /// JavaScript interop and has neither an offscreen framebuffer nor a pixel read-back, so this
    /// stub reports that there is no image — the same "no suitable painter" outcome the caller in
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
            IFrame? frame,
            IEnumerable<(IGeoObject obj, CADability.Substitutes.Color color)> coloredObjects,
            GeoVector viewDirection,
            int width,
            int height,
            out string? unavailableReason)
        {
            unavailableReason = "no offscreen rendering in the browser: the WebGL painter cannot read pixels back";
            return null;
        }
    }
}
