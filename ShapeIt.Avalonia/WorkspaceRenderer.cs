using CADability;
using CADability.GeoObject;
using CADability.UserInterface;
using System.Collections.Generic;

namespace ShapeIt
{
    /// <summary>
    /// Avalonia-Implementierung des WorkspaceRenderers.
    /// Ersetzt die WinForms-Version (Control.Invoke + System.Drawing.Bitmap)
    /// durch Avalonia-Dispatcher und Avalonia.Media.Imaging.
    /// </summary>
    internal static class WorkspaceRenderer
    {
        public static string? RenderToPngBase64(
            IFrame frame,
            IEnumerable<(IGeoObject obj, CADability.Substitutes.Color color)> coloredObjects,
            GeoVector viewDirection,
            int width,
            int height)
        {
            // TODO: Implementierung mit Avalonia-Dispatcher und OpenGL-Offscreen-Rendering.
            // Statt Control.Invoke → Avalonia.Threading.Dispatcher.UIThread.InvokeAsync
            // Statt System.Drawing.Bitmap → Avalonia.Media.Imaging.WriteableBitmap
            return null;
        }
    }
}
