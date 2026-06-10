using System.Runtime.InteropServices.JavaScript;

namespace ShapeIt.Browser
{
    /// <summary>
    /// [JSImport] bridge to wwwroot/browserhost.js — small browser-lifecycle helpers
    /// that have no Avalonia single-view equivalent: the document/tab title and the
    /// beforeunload unsaved-changes guard. Mirrors the desktop's Window.Title and
    /// save-on-close prompt, browser-appropriate. Loaded in Program.Main via
    /// JSHost.ImportAsync("browserhost", "../browserhost.js").
    /// </summary>
    internal static partial class BrowserHostInterop
    {
        // Set the browser tab title (e.g. "ShapeIt — Part.cdb.json").
        [JSImport("setTitle", "browserhost")]
        public static partial void SetTitle(string title);

        // Arm/disarm the beforeunload prompt that warns about unsaved changes.
        [JSImport("setUnsavedGuard", "browserhost")]
        public static partial void SetUnsavedGuard(bool unsaved);

        // Save a file via a browser download (Blob). Works in all browsers, unlike the
        // Chromium-only SaveFilePickerAsync. `base64` is the file content.
        [JSImport("downloadFile", "browserhost")]
        public static partial void DownloadFile(string fileName, string base64);

        // True on touch-capable devices, so the touch helper bar can be shown by default.
        [JSImport("isTouchDevice", "browserhost")]
        public static partial bool IsTouchDevice();
    }
}
