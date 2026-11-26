using Avalonia;
using System;

namespace ShapeIt
{
    internal sealed class Program
    {
        [STAThread]
        public static void Main(string[] args)
        {
            AppVersion.RegisterWithCADability(); // "Application" block in saved JSON files
            BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);
        }

        public static AppBuilder BuildAvaloniaApp() =>
            AppBuilder.Configure<App>()
                .UsePlatformDetect()
                .WithInterFont()
                .LogToTrace();
    }
}
