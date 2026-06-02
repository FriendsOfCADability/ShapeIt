using Avalonia.Threading;
using CADability;
using CADability.UserInterface;
using System;
using System.Diagnostics;
using System.IO;
using System.Net.Http;
using System.Reflection;
using System.Threading.Tasks;

namespace ShapeIt;

/// <summary>
/// Leichtgewichtige Update-Pruefung beim Programmstart.
///
/// Ablauf:
///   1. Eine kleine Textdatei auf dem Server (latest.txt) enthaelt die aktuell
///      veroeffentlichte Versionsnummer (z. B. "0.8.4").
///   2. Beim Start liest ShapeIt diese Datei, vergleicht sie mit der eigenen
///      eingebetteten Version (App.Version aus version.txt) und bietet bei einer
///      neueren Version an, die Download-Seite im Browser zu oeffnen.
///
/// Die Pruefung ist bewusst:
///   - nicht blockierend  (laeuft auf einem Hintergrund-Task),
///   - fehlertolerant      (kein Internet / Server nicht erreichbar -> still ignoriert),
///   - nur unter Windows   (Linux aktualisiert ueber das APT-Repository).
///
/// Anpassen: die beiden URL-Konstanten unten auf die eigene Server-Struktur setzen.
/// latest.txt wird von scripts\windows-packaging\1-build.ps1 automatisch miterzeugt.
/// </summary>
internal static class UpdateChecker
{
    // ── Konfiguration: Server-URLs hier anpassen ─────────────────────────────
    // Kleine Textdatei mit der aktuellen Versionsnummer (eine Zeile, z. B. "0.8.4").
    private const string LatestVersionUrl = "https://www.cadability.de/ShapeIt/latest.txt";
    // Seite, die im Browser geoeffnet wird (dort sind ZIP und Installer verlinkt).
    private const string DownloadPageUrl  = "https://www.cadability.de/ShapeIt/win-install.html";

    private static readonly HttpClient Http = new() { Timeout = TimeSpan.FromSeconds(5) };

    /// <summary>
    /// Startet die Update-Pruefung im Hintergrund. Kehrt sofort zurueck und stoert
    /// den Programmstart nie (alle Fehler werden verschluckt).
    /// </summary>
    public static void CheckInBackground(IFrame frame)
    {
        // Nur unter Windows pruefen; unter Linux uebernimmt das APT-Repository die Updates.
        if (!OperatingSystem.IsWindows()) return;

        _ = Task.Run(async () =>
        {
            try
            {
                string current = ReadCurrentVersion();
                string? latest = await FetchLatestVersionAsync().ConfigureAwait(false);
                if (latest == null || !IsNewer(latest, current)) return;

                // Dialog muss auf dem UI-Thread laufen.
                await Dispatcher.UIThread.InvokeAsync(() => Prompt(frame, latest));
            }
            catch { /* best-effort: Update-Pruefung darf den Start nie behindern */ }
        });
    }

    private static void Prompt(IFrame frame, string latest)
    {
        bool de = StringTable.ActiveLanguage?
            .StartsWith("deutsch", StringComparison.OrdinalIgnoreCase) ?? false;

        string msg = de
            ? $"Eine neue Version von ShapeIt ist verfügbar (Version {latest}).\n\n"
              + "Möchten Sie die Download-Seite öffnen?"
            : $"A new version of ShapeIt is available (version {latest}).\n\n"
              + "Would you like to open the download page?";

        var answer = frame.UIService.ShowMessageBox(
            msg, "ShapeIt", CADability.Substitutes.MessageBoxButtons.YesNo);

        if (answer == CADability.Substitutes.DialogResult.Yes)
        {
            try
            {
                Process.Start(new ProcessStartInfo { FileName = DownloadPageUrl, UseShellExecute = true });
            }
            catch { /* Browser konnte nicht geoeffnet werden */ }
        }
    }

    private static async Task<string?> FetchLatestVersionAsync()
    {
        string text = (await Http.GetStringAsync(LatestVersionUrl).ConfigureAwait(false)).Trim();
        return string.IsNullOrEmpty(text) ? null : text;
    }

    /// <summary>true, wenn <paramref name="latest"/> eine hoehere Version als <paramref name="current"/> ist.</summary>
    private static bool IsNewer(string latest, string current)
    {
        if (Version.TryParse(latest, out var lv) && Version.TryParse(current, out var cv))
            return lv > cv;
        return false; // bei nicht parsbaren Werten lieber nicht stoeren
    }

    /// <summary>Liest die eingebettete Version (App.Version / version.txt).</summary>
    private static string ReadCurrentVersion()
    {
        try
        {
            var asm = Assembly.GetExecutingAssembly();
            using Stream? s = asm.GetManifestResourceStream("App.Version");
            if (s != null)
            {
                using var sr = new StreamReader(s);
                string v = sr.ReadToEnd().Trim();
                if (!string.IsNullOrEmpty(v)) return v;
            }
        }
        catch { /* fall through */ }

        return Assembly.GetExecutingAssembly().GetName().Version?.ToString() ?? "0.0.0";
    }
}
