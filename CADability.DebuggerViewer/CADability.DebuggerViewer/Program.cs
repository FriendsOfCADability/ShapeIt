using CadDebuggerViewer;

namespace CADability.DebuggerViewer
{
    internal static class Program
    {
        private static Mutex? _singleInstanceMutex;

        /// <summary>
        ///  The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            // Single-Instance (optional, aber praktisch)
            bool createdNew;
            _singleInstanceMutex = new Mutex(true, "Cadability.DebuggerViewer_Running", out createdNew);
            if (!createdNew)
            {
                // schon eine Instanz aktiv → einfach beenden
                return;
            }

            ApplicationConfiguration.Initialize();
            Application.Run(new MainForm());
        }
    }
}