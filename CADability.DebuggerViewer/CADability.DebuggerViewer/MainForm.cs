using CADability;
using CADability.Forms.NET8;
using CADability.GeoObject;
using CADability.UserInterface;
using Microsoft.Win32;
using System;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace CadDebuggerViewer
{
    public class MainForm : CadForm
    {
        private readonly PipeServer _server;
        private readonly CancellationTokenSource _cts = new();

        public MainForm() : base([])
        {

            Text = "CADability Debugger Viewer";
            Width = 1200;
            Height = 800;

            // Pipe-Server starten
            _server = new PipeServer("cadability.visualizer.v1");
            _server.OnShow += Server_OnShow;
            //_server.OnCommand += Server_OnCommand;
            //_server.OnClientConnected += (_, __) => Ui(() => SetStatus("Verbunden"));
            //_server.OnClientDisconnected += (_, __) => Ui(() => SetStatus("Getrennt"));

            // start pipe server
            _ = _server.RunAsync(_cts.Token);

            // Falls du das Fenster beim Start maximiert willst:
            // WindowState = FormWindowState.Maximized;
            CadFrame.GenerateNewProject();
            StringTable.AddString("eng", "Debug.HashCode", StringTable.Category.label, "HashCode");
            StringTable.AddString("eng", "Debug.HashCode", StringTable.Category.info, "Original HashCode of object beeing debugged");
            StringTable.AddString("eng", "Debug.ListIndex", StringTable.Category.label, "Index");
            StringTable.AddString("eng", "Debug.ListIndex", StringTable.Category.info, "Original Index in list beeing debugged");
            StringTable.AddString("deu", "Debug.HashCode", StringTable.Category.label, "HashCode");
            StringTable.AddString("deu", "Debug.HashCode", StringTable.Category.info, "Original HashCodedes Objektes im Debugger");
            StringTable.AddString("deu", "Debug.ListIndex", StringTable.Category.label, "Index");
            StringTable.AddString("deu", "Debug.ListIndex", StringTable.Category.info, "Original Index des Objektes in der Liste");

            WriteLaunchInfo();
        }

        public static void WriteLaunchInfo()
        {
            // Robustester Weg zum aktuellen Prozesspfad in .NET 6+: 
            string? exePath = Environment.ProcessPath;
            if (string.IsNullOrEmpty(exePath))
                return;

            exePath = System.IO.Path.GetFullPath(exePath);

            using var key = Registry.CurrentUser.CreateSubKey(@"Software\CADability\DebuggerView");
            key!.SetValue("ExePath", exePath, RegistryValueKind.String);
            key.SetValue("Version", System.Reflection.Assembly.GetEntryAssembly()?
                                            .GetName().Version?.ToString() ?? "", RegistryValueKind.String);
            key.SetValue("WrittenUtc", DateTime.UtcNow.ToString("o"), RegistryValueKind.String);
        }
        protected override void OnFormClosed(FormClosedEventArgs e)
        {
            _cts.Cancel();
            base.OnFormClosed(e);
        }

        private void Ui(Action a)
        {
            if (IsHandleCreated)
                BeginInvoke(a);
            else
                a();
        }


        private void Server_OnShow(object? sender, string json)
        {
            Ui(() =>
            {
                try
                {
                    if (CadFrame.Project == null) CadFrame.GenerateNewProject();
                    object toShow = JsonSerialize.FromString(json);
                    if (toShow is IGeoObject go)
                    {
                        go.UpdateAttributes(CadFrame.Project);
                        CadFrame.Project?.GetActiveModel().Add(go);
                    }
                    else if (toShow is GeoObjectList l)
                    {
                        foreach (IGeoObject geoObject in l)
                        {
                            geoObject.UpdateAttributes(CadFrame.Project);
                        }
                        CadFrame.Project?.GetActiveModel().Add(l);
                    }
                    else return;
                    CadFrame.ActiveView.ZoomTotal(1.1);
                    this.WindowState = FormWindowState.Normal;  // if it was minimized
                    this.BringToFront();
                    this.Activate();
                }
                catch (Exception ex)
                {
                }
            });
        }

    }
}
