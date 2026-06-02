using Avalonia.Controls;
using CADability.UserInterface;

namespace CADability.Avalonia;

/// <summary>
/// Avalonia UserControl that hosts CadCanvas (left) and PropertiesExplorer (right),
/// connected through a CadFrame — the Avalonia equivalent of the WinForms CadControl.
/// </summary>
public partial class CadControl : UserControl, ICommandHandler
{
    private CadFrame? _cadFrame;

    public CadControl()
    {
        InitializeComponent();

        // Both cadCanvas and propertiesExplorer are created by InitializeComponent().
        // Wire them up through a CadFrame immediately; the ICommandHandler (this) is a
        // fallback — actual command routing happens inside FrameImpl / the action stack.
        _cadFrame = new CadFrame(propertiesExplorer, cadCanvas, this);

        // cadCanvas needs the Frame reference so Feedback.Attach can reach
        //   view.Canvas.Frame.GetIntSetting(…) without a NullReferenceException.
        cadCanvas.Frame = _cadFrame;
    }

    // ── Public surface (mirrors WinForms CadControl) ───────────────────────

    /// <summary>The frame that owns views, actions, and the control centre.</summary>
    public CadFrame CadFrame => _cadFrame!;

    /// <summary>The OpenGL canvas (left panel).</summary>
    public CadCanvas CadCanvas => cadCanvas;

    /// <summary>The properties explorer (right panel).</summary>
    public PropertiesExplorer PropertiesExplorer => propertiesExplorer;

    // ── ICommandHandler ────────────────────────────────────────────────────
    // Returns false for everything so CadFrame's own routing is not short-circuited.
    // Applications that need custom command handling can subscribe to CadFrame events.

    bool ICommandHandler.OnCommand(string MenuId) => false;
    bool ICommandHandler.OnUpdateCommand(string MenuId, CommandState CommandState) => false;
    void ICommandHandler.OnSelected(MenuWithHandler selectedMenuItem, bool selected) { }
}
