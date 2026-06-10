using System.Runtime.InteropServices.JavaScript;
using System.Runtime.Versioning;
using System.Threading.Tasks;
using Avalonia;
using Avalonia.Browser;
using Avalonia.Logging;

[assembly: SupportedOSPlatform("browser")]

namespace ShapeIt.Browser
{
    internal sealed partial class Program
    {
        private static async Task Main(string[] args)
        {
            // Surface Avalonia rendering errors in the browser console.
            Logger.Sink = new ConsoleLogSink(LogEventLevel.Warning);

            // Load the WebGL module so [JSImport(...,"webgl")] resolves.
            // Path is relative to /_framework/, so step up to wwwroot root.
            await JSHost.ImportAsync("webgl", "../webgl.js");

            // Load the browser-lifecycle helper module so [JSImport(...,"browserhost")]
            // resolves (tab title + beforeunload unsaved-changes guard).
            await JSHost.ImportAsync("browserhost", "../browserhost.js");

            await BuildAvaloniaApp()
                .WithInterFont()
                .StartBrowserAppAsync("out");
        }

        public static AppBuilder BuildAvaloniaApp() =>
            AppBuilder.Configure<App>();
    }
}
