using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Markup.Xaml;
using CADability.UserInterface;
using System.Collections.Generic;
using System.IO;
using System.Reflection;

namespace ShapeIt;

public partial class AboutWindow : Window
{
    // One row in the third-party library list.
    private sealed record LibraryInfo(string Name, string License, string Copyright);

    public AboutWindow()
    {
        InitializeComponent();
        PopulateContent();
    }

    private void InitializeComponent() => AvaloniaXamlLoader.Load(this);

    private bool IsGerman =>
        StringTable.ActiveLanguage?.StartsWith("deutsch", System.StringComparison.OrdinalIgnoreCase) ?? false;

    private void PopulateContent()
    {
        bool de = IsGerman;
        string version = ReadVersion();

        Title = de ? "Über ShapeIt" : "About ShapeIt";

        var versionText = this.FindControl<TextBlock>("versionText")!;
        versionText.Text = (de ? "Version " : "Version ") + version;

        var basedOnText = this.FindControl<TextBlock>("basedOnText")!;
        basedOnText.Text = de
            ? "ShapeIt ist ein 3D-CAD-Programm, das auf der quelloffenen Bibliothek "
              + "CADability (MIT-Lizenz, © SOFA GmbH) aufbaut. ShapeIt selbst und CADability "
              + "sind freie Software."
            : "ShapeIt is a 3D CAD application built on top of the open-source library "
              + "CADability (MIT License, © SOFA GmbH). ShapeIt and CADability are free software.";

        this.FindControl<TextBlock>("moreInfoLabel")!.Text =
            de ? "Weitere Informationen:" : "More information:";

        this.FindControl<TextBlock>("librariesHeader")!.Text = de
            ? "Verwendete Bibliotheken und ihre Lizenzen:"
            : "Third-party libraries and their licenses:";

        this.FindControl<ItemsControl>("librariesList")!.ItemsSource = GetLibraries();

        this.FindControl<TextBlock>("copyrightText")!.Text = de
            ? "Alle genannten Bibliotheken werden gemäß ihren jeweiligen Lizenzen verwendet. "
              + "Die Marken- und Urheberrechte verbleiben bei den jeweiligen Inhabern."
            : "All listed libraries are used in accordance with their respective licenses. "
              + "Trademarks and copyrights remain the property of their respective owners.";
    }

    // Only libraries actually shipped in the ShapeIt.Avalonia output are listed
    // (verified against bin/.../net8.0/*.dll).
    private static List<LibraryInfo> GetLibraries() => new()
    {
        new("CADability",                   "MIT License",           "© SOFA GmbH — https://github.com/SOFAgh/CADability"),
        new("Avalonia UI",                  "MIT License",           "© AvaloniaUI OÜ and contributors"),
        new("SkiaSharp / HarfBuzzSharp",    "MIT License",           "© Microsoft Corporation"),
        new("MathNet.Numerics",             "MIT License",           "© Math.NET Project"),
        new("Silk.NET (OpenGL)",            "MIT License",           "© .NET Foundation and Contributors"),
        new("Svg.Skia",                     "MIT License",           "© Wiesław Šoltés"),
        new("StbImageSharp",                "Public Domain (MIT-0)", "Port by Roman Shapiro, based on stb by Sean Barrett"),
        new("System.Collections.Immutable", "MIT License",           "© .NET Foundation and Contributors"),
    };

    // Prefer the embedded "App.Version" (version.txt); fall back to the assembly version.
    private static string ReadVersion()
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

        return Assembly.GetExecutingAssembly().GetName().Version?.ToString() ?? "?";
    }

    private void OnCloseClick(object? sender, RoutedEventArgs e) => Close();
}
