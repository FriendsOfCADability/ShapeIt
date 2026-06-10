namespace CADability.Avalonia
{
    /// <summary>
    /// Exposes the last known keyboard-modifier state and screen pointer position
    /// captured from pointer events. Implemented by canvas controls (CadCanvas,
    /// Gl3DViewport) so that CadFrame can read modifier keys / mouse position on
    /// non-Windows platforms without taking a hard dependency on a concrete control.
    /// </summary>
    public interface IModifierKeyProvider
    {
        global::Avalonia.Input.KeyModifiers LastKeyModifiers { get; }
        global::Avalonia.PixelPoint LastScreenPosition { get; }
    }
}
