using System;

namespace ShapeIt
{
    /// <summary>
    /// Browser no-op stub for the desktop <c>MCPServerWindow</c>.
    /// <para>
    /// The real desktop window (ShapeIt.Avalonia/MCPServerWindow.cs) hosts the MCP RPC
    /// console and an embedded CAD workspace. The MCP HTTP server relies on
    /// <see cref="System.Net.HttpListener"/>, which is unavailable inside the
    /// Avalonia.Browser (WASM) single-view lifetime, so there is nothing to show here.
    /// </para>
    /// <para>
    /// This type exists only so the shared <c>ShapeIt/ModellingPropertyEntries.cs</c>
    /// compiles for the browser head. It deliberately does NOT derive from
    /// <c>Avalonia.Controls.Window</c> (a Window cannot be shown in the browser
    /// single-view lifetime). Every member is a safe no-op, so it does nothing harmful
    /// even if it is accidentally constructed or invoked at runtime.
    /// </para>
    /// </summary>
    public class MCPServerWindow
    {
        /// <summary>Matches the desktop ctor <c>MCPServerWindow(MCPServer)</c>; ignores the server.</summary>
        public MCPServerWindow(MCPServer server)
        {
            // No-op: the MCP console is not available in the browser.
        }

        /// <summary>Settable title; ignored in the browser stub.</summary>
        public string Title { get; set; } = "MCP Server";

        /// <summary>Never raised in the browser stub; present so <c>window.Closed += ...</c> compiles.</summary>
        public event EventHandler? Closed;

        /// <summary>No-op: nothing to show in the browser.</summary>
        public void Show()
        {
            // No-op.
        }

        /// <summary>No-op: nothing to activate in the browser.</summary>
        public void Activate()
        {
            // No-op.
        }

        /// <summary>No-op: RPC calls are not logged to any window in the browser.</summary>
        public void AppendRpcCall(string rpcJson)
        {
            // No-op.
        }
    }
}
