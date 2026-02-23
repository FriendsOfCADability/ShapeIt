#if AVALONIA
using Avalonia;
#endif
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
#if !AVALONIA
using System.Windows.Forms;
#endif


namespace ShapeIt
{
    internal static class Program
    {
#if AVALONIA
        /// <summary>
        ///  The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main(string[] args) => BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);

        public static AppBuilder BuildAvaloniaApp()
        => AppBuilder.Configure<ShapeIt>()
            .UsePlatformDetect()
            .WithInterFont()
            .LogToTrace();

#else
        /// <summary>
        ///  The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main(string[] args)
        {
            // To customize application configuration such as set high DPI settings or default font,
            // see https://aka.ms/applicationconfiguration.
            ApplicationConfiguration.Initialize();
            Application.Run(new MainForm(args));
        }
#endif
    }
}
