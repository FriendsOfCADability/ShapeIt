using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Markup.Xaml;

namespace ShapeIt.Browser
{
    public partial class App : Application
    {
        public override void Initialize() => AvaloniaXamlLoader.Load(this);

        public override void OnFrameworkInitializationCompleted()
        {
            // Browser uses the single-view lifetime (no top-level windows).
            if (ApplicationLifetime is ISingleViewApplicationLifetime singleView)
                singleView.MainView = new MainView();
            // Desktop fallback so the same head can be smoke-tested on the dev machine.
            else if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
                desktop.MainWindow = new Window { Title = "ShapeIt (web spike)", Content = new MainView() };

            base.OnFrameworkInitializationCompleted();
        }
    }
}
