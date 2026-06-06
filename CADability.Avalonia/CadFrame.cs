using CADability;
using CADability.GeoObject;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.ExceptionServices;
using System.Runtime.InteropServices;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Platform.Storage;
using Avalonia.Threading;
using Av = global::Avalonia.Threading;
using SysPath = System.IO.Path;
using Substitutes = CADability.Substitutes;

namespace CADability.Avalonia;

public class CadFrame : FrameImpl, IUIService
{
    private readonly ICommandHandler? _commandHandler;
    private readonly ICanvas _icanvas;

    private event EventHandler? _applicationIdle;
    private Av.DispatcherTimer? _idleTimer;

    // Serialized JSON payload — used by the clipboard and as a cross-process drag fallback.
    internal const string ClipFormat = "CADability.GeoObjectList.Json";
    // In-process drag payload: the live GeoObjectList travels by reference (no round-trip).
    internal const string DragObjectFormat = "CADability.GeoObjectList.Object";
    private static readonly Dictionary<string, string> _directories = new();

    public Action<bool, double, string>? ProgressAction { get; set; }

    public CadFrame(IControlCenter cc, ICanvas canvas, ICommandHandler? commandHandler = null)
        : base(cc, canvas)
    {
        _commandHandler = commandHandler;
        _icanvas = canvas;
    }

    public CadFrame(ICanvas canvas, ICommandHandler? commandHandler = null)
        : base(canvas)
    {
        _commandHandler = commandHandler;
        _icanvas = canvas;
    }

    public override IUIService UIService => this;

    public override bool OnCommand(string MenuId)
    {
        if (_commandHandler != null && _commandHandler.OnCommand(MenuId)) return true;
        return base.OnCommand(MenuId);
    }

    public override bool OnUpdateCommand(string MenuId, CommandState CommandState)
    {
        if (_commandHandler != null && _commandHandler.OnUpdateCommand(MenuId, CommandState)) return true;
        return base.OnUpdateCommand(MenuId, CommandState);
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    private TopLevel? GetTopLevel()
        => _icanvas is Control ctrl ? TopLevel.GetTopLevel(ctrl) : null;

    // Bridges async Avalonia APIs to the synchronous IUIService interface.
    //
    // Background-thread path: InvokeAsync schedules on the UI thread and blocks the caller.
    //
    // UI-thread path: enters a nested Avalonia event loop (MainLoop) so that the OS message
    // pump keeps running while we wait — required for native file/color dialogs that need
    // Win32 messages to operate. RunJobs() alone is insufficient because it only drains the
    // Avalonia dispatcher queue and does not pump OS-level messages.
    private static T RunDialogSync<T>(Func<Task<T>> factory)
    {
        if (!Dispatcher.UIThread.CheckAccess())
        {
            return Dispatcher.UIThread.InvokeAsync(() => factory())
                .GetAwaiter()
                .GetResult();
        }

        var cts = new System.Threading.CancellationTokenSource();
        T result = default!;
        Exception? caught = null;

        Dispatcher.UIThread.Post(async () =>
        {
            try   { result = await factory(); }
            catch (Exception e) { caught = e; }
            finally { cts.Cancel(); }
        }, DispatcherPriority.Normal);

        Dispatcher.UIThread.MainLoop(cts.Token);

        if (caught != null) ExceptionDispatchInfo.Capture(caught).Throw();
        return result!;
    }

    // ── IUIService ─────────────────────────────────────────────────────────

    GeoObjectList IUIService.GetDataPresent(object data)
    {
        // During a drop, CADability passes us the Avalonia IDataObject from the DragEventArgs.
        if (data is not IDataObject ido) return null;

        // In-process drag (same application): the live object is available by reference.
        if (ido.Get(DragObjectFormat) is GeoObjectList live) return live;

        // Cross-process drag (another instance / app): fall back to the JSON payload.
        return ido.Get(ClipFormat) switch
        {
            byte[] bytes => DeserializeGeoObjectList(new MemoryStream(bytes)) as GeoObjectList,
            Stream s     => DeserializeGeoObjectList(s) as GeoObjectList,
            _            => null,
        };
    }

    Substitutes.Keys IUIService.ModifierKeys
    {
        get
        {
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                var mods = Substitutes.Keys.None;
                short shift   = NativeMethods.GetKeyState(0x10); // VK_SHIFT
                short control = NativeMethods.GetKeyState(0x11); // VK_CONTROL
                short alt     = NativeMethods.GetKeyState(0x12); // VK_MENU
                if ((shift   & 0x8000) != 0) mods |= Substitutes.Keys.Shift;
                if ((control & 0x8000) != 0) mods |= Substitutes.Keys.Control;
                if ((alt     & 0x8000) != 0) mods |= Substitutes.Keys.Alt;
                return mods;
            }
            // Nicht-Windows: letzten bekannten Zustand aus Pointer-Events lesen
            if (_icanvas is CadCanvas canvas)
            {
                var km   = canvas.LastKeyModifiers;
                var mods = Substitutes.Keys.None;
                if ((km & KeyModifiers.Shift)   != 0) mods |= Substitutes.Keys.Shift;
                if ((km & KeyModifiers.Control) != 0) mods |= Substitutes.Keys.Control;
                if ((km & KeyModifiers.Alt)     != 0) mods |= Substitutes.Keys.Alt;
                return mods;
            }
            return Substitutes.Keys.None;
        }
    }

    Substitutes.Point IUIService.CurrentMousePosition
    {
        get
        {
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                if (NativeMethods.GetCursorPos(out var pt))
                    return new Substitutes.Point(pt.X, pt.Y);
                return default;
            }
            // Nicht-Windows: zuletzt gespeicherte Screen-Position aus dem Canvas
            if (_icanvas is CadCanvas canvas)
            {
                var sp = canvas.LastScreenPosition;
                return new Substitutes.Point(sp.X, sp.Y);
            }
            return default;
        }
    }

    Substitutes.DialogResult IUIService.ShowOpenFileDlg(
        string id, string title, string filter, ref int filterIndex, out string fileName)
    {
        var topLevel = GetTopLevel();
        if (topLevel == null) { fileName = null; return Substitutes.DialogResult.Cancel; }

        var options = new FilePickerOpenOptions
        {
            Title          = string.IsNullOrWhiteSpace(title) ? null : title,
            AllowMultiple  = false,
            FileTypeFilter = BuildFileTypeFilters(filter),
        };

        if (!string.IsNullOrWhiteSpace(id) && _directories.TryGetValue(id, out var dir)
            && Uri.TryCreate(dir, UriKind.Absolute, out var dirUri))
        {
            options.SuggestedStartLocation =
                RunDialogSync(() => topLevel.StorageProvider.TryGetFolderFromPathAsync(dirUri));
        }

        var files = RunDialogSync(() => topLevel.StorageProvider.OpenFilePickerAsync(options));
        if (files?.Count > 0)
        {
            fileName = files[0].Path.LocalPath;
            filterIndex = InferFilterIndex(filter, fileName);
            if (!string.IsNullOrWhiteSpace(id))
                _directories[id] = SysPath.GetDirectoryName(fileName) ?? string.Empty;
            return Substitutes.DialogResult.OK;
        }
        fileName = null;
        return Substitutes.DialogResult.Cancel;
    }

    Substitutes.DialogResult IUIService.ShowSaveFileDlg(
        string id, string title, string filter, ref int filterIndex, ref string fileName)
    {
        var topLevel = GetTopLevel();
        if (topLevel == null) return Substitutes.DialogResult.Cancel;

        var options = new FilePickerSaveOptions
        {
            Title             = string.IsNullOrWhiteSpace(title) ? null : title,
            DefaultExtension  = GetDefaultExtensionFromFilter(filter),
            FileTypeChoices   = BuildFileTypeFilters(filter),
            SuggestedFileName = string.IsNullOrWhiteSpace(fileName) ? null : SysPath.GetFileName(fileName),
        };

        if (!string.IsNullOrWhiteSpace(id) && _directories.TryGetValue(id, out var dir)
            && Uri.TryCreate(dir, UriKind.Absolute, out var dirUri))
        {
            options.SuggestedStartLocation =
                RunDialogSync(() => topLevel.StorageProvider.TryGetFolderFromPathAsync(dirUri));
        }

        var file = RunDialogSync(() => topLevel.StorageProvider.SaveFilePickerAsync(options));
        if (file != null)
        {
            fileName = file.Path.LocalPath;
            filterIndex = InferFilterIndex(filter, fileName);
            if (!string.IsNullOrWhiteSpace(id))
                _directories[id] = SysPath.GetDirectoryName(fileName) ?? string.Empty;
            return Substitutes.DialogResult.OK;
        }
        return Substitutes.DialogResult.Cancel;
    }

    Substitutes.DialogResult IUIService.ShowMessageBox(
        string text, string caption, Substitutes.MessageBoxButtons buttons)
    {
        if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
        {
            uint mbType = buttons switch
            {
                Substitutes.MessageBoxButtons.OKCancel         => 0x01u,
                Substitutes.MessageBoxButtons.AbortRetryIgnore => 0x02u,
                Substitutes.MessageBoxButtons.YesNoCancel      => 0x03u,
                Substitutes.MessageBoxButtons.YesNo            => 0x04u,
                Substitutes.MessageBoxButtons.RetryCancel      => 0x05u,
                _                                              => 0x00u,
            };
            return NativeMethods.MessageBox(IntPtr.Zero, text, caption, mbType) switch
            {
                1 => Substitutes.DialogResult.OK,
                2 => Substitutes.DialogResult.Cancel,
                3 => Substitutes.DialogResult.Abort,
                4 => Substitutes.DialogResult.Retry,
                5 => Substitutes.DialogResult.Ignore,
                6 => Substitutes.DialogResult.Yes,
                7 => Substitutes.DialogResult.No,
                _ => Substitutes.DialogResult.Cancel,
            };
        }
        return RunDialogSync(() => ShowAvaloniaMessageBoxAsync(
            GetTopLevel() as Window, text, caption, buttons));
    }

    Substitutes.DialogResult IUIService.ShowColorDialog(ref Substitutes.Color color)
    {
        if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
        {
            int argb = color.ToArgb();
            uint colorRef = (uint)(
                ((argb >> 16) & 0xFF) |
                (((argb >>  8) & 0xFF) << 8) |
                ((argb & 0xFF) << 16));

            var custColors = new int[16];
            var pin = GCHandle.Alloc(custColors, GCHandleType.Pinned);
            try
            {
                var cc = new NativeMethods.CHOOSECOLOR
                {
                    lStructSize  = (uint)Marshal.SizeOf<NativeMethods.CHOOSECOLOR>(),
                    hwndOwner    = IntPtr.Zero,
                    rgbResult    = colorRef,
                    Flags        = NativeMethods.CC_RGBINIT | NativeMethods.CC_FULLOPEN,
                    lpCustColors = pin.AddrOfPinnedObject(),
                };

                if (!NativeMethods.ChooseColor(ref cc))
                    return Substitutes.DialogResult.Cancel;

                uint rgb = cc.rgbResult;
                color = Substitutes.Color.FromArgb(255,
                    (int)(rgb         & 0xFF),
                    (int)((rgb >>  8) & 0xFF),
                    (int)((rgb >> 16) & 0xFF));
            }
            finally { pin.Free(); }

            return Substitutes.DialogResult.OK;
        }

        // Nicht-Windows: plattformunabhängiger Avalonia-Farbwähler
        var initial = color;
        var (ok, picked) = RunDialogSync(() => ShowAvaloniaColorDialogAsync(
            GetTopLevel() as Window, initial));
        if (ok) color = picked;
        return ok ? Substitutes.DialogResult.OK : Substitutes.DialogResult.Cancel;
    }

    object IUIService.GetBitmap(string name) => null;

    event EventHandler IUIService.ApplicationIdle
    {
        add
        {
            _applicationIdle += value;
            if (_idleTimer == null)
            {
                _idleTimer = new Av.DispatcherTimer(
                    TimeSpan.FromMilliseconds(100),
                    Av.DispatcherPriority.Background,
                    (_, _) => _applicationIdle?.Invoke(this, EventArgs.Empty));
                _idleTimer.Start();
            }
        }
        remove
        {
            _applicationIdle -= value;
            if (_applicationIdle == null)
            {
                _idleTimer?.Stop();
                _idleTimer = null;
            }
        }
    }

    IPaintTo3D IUIService.CreatePaintInterface(object paintToBitmap, double precision) => null;

    Substitutes.DialogResult IUIService.ShowPageSetupDlg(
        object printDocument1, object pageSettings,
        out int width, out int height, out bool landscape)
    {
        width = height = 0;
        landscape = false;
        return Substitutes.DialogResult.Cancel;
    }

    Substitutes.DialogResult IUIService.ShowPrintDlg(object printDocument)
        => Substitutes.DialogResult.Cancel;

    void IUIService.SetClipboardData(GeoObjectList objects, bool copy)
    {
        var topLevel = GetTopLevel();
        if (topLevel?.Clipboard == null) return;

        byte[] payload;
        using (var ms = new MemoryStream())
        {
            var js = new JsonSerialize();
            js.ToStream(ms, objects, closeStream: false);
            payload = ms.ToArray();
        }

        var dataObject = new DataObject();
        dataObject.Set(ClipFormat, payload);
        dataObject.Set(DataFormats.Text, $"CADability GeoObjectList ({objects?.Count ?? 0} items)");

        RunDialogSync(async () =>
        {
            await topLevel.Clipboard.SetDataObjectAsync(dataObject);
            return true;
        });
    }

    object IUIService.GetClipboardData(Type typeOfData)
    {
        var topLevel = GetTopLevel();
        if (topLevel?.Clipboard == null) return null;
        try
        {
            if (typeOfData == typeof(string))
                return RunDialogSync(() => topLevel.Clipboard.GetTextAsync());

            var data = RunDialogSync(() => topLevel.Clipboard.GetDataAsync(ClipFormat));
            return data switch
            {
                byte[] bytes => DeserializeGeoObjectList(new MemoryStream(bytes)),
                Stream s     => DeserializeGeoObjectList(s),
                _            => null,
            };
        }
        catch { return null; }
    }

    bool IUIService.HasClipboardData(Type typeOfData)
    {
        var topLevel = GetTopLevel();
        if (topLevel?.Clipboard == null) return false;
        try
        {
            var formats = RunDialogSync(() => topLevel.Clipboard.GetFormatsAsync());
            return formats != null && Array.IndexOf(formats, ClipFormat) >= 0;
        }
        catch { return false; }
    }

    void IUIService.ShowProgressBar(bool show, double percent, string title)
    {
        if (ProgressAction == null) return;
        if (Dispatcher.UIThread.CheckAccess())
            ProgressAction(show, percent, title);
        else
            Dispatcher.UIThread.Post(() => ProgressAction?.Invoke(show, percent, title));
    }

    Substitutes.FontFamily IUIService.GetFontFamily(string fontFamilyName)
        => new FontFamilyImpl(fontFamilyName);

    string[] IUIService.GetFontFamilies()
    {
        try
        {
            var result = new List<string>();
            foreach (var family in FontManager.Current.SystemFonts)
                result.Add(family.Name);
            return result.ToArray();
        }
        catch { return Array.Empty<string>(); }
    }

    // ── Private helpers ─────────────────────────────────────────────────────

    private static object DeserializeGeoObjectList(Stream stream)
    {
        using var ms = new MemoryStream();
        stream.CopyTo(ms);
        ms.Position = 0;
        return new JsonSerialize().FromStream(ms);
    }

    // Converts WinForms-style filter ("Description|*.ext|...") to Avalonia FilePickerFileType list.
    private static IReadOnlyList<FilePickerFileType> BuildFileTypeFilters(string filter)
    {
        var result = new List<FilePickerFileType>();
        if (string.IsNullOrWhiteSpace(filter)) return result;
        var parts = filter.Split('|');
        for (int i = 0; i + 1 < parts.Length; i += 2)
            result.Add(new FilePickerFileType(parts[i]) { Patterns = parts[i + 1].Split(';') });
        return result;
    }

    private static string? GetDefaultExtensionFromFilter(string filter)
    {
        if (string.IsNullOrWhiteSpace(filter)) return null;
        var parts = filter.Split('|');
        if (parts.Length < 2) return null;
        var pattern = parts[1].Split(';')[0].TrimStart('*');
        return pattern.StartsWith('.') ? pattern : null;
    }

    // Maps a selected file's extension back to its 1-based filter index.
    // Filter format: "Desc1|*.ext1;*.ext2|Desc2|*.ext3|..."
    // Used to return the correct filterIndex to FrameImpl after the async dialog resolves.
    private static int InferFilterIndex(string filter, string filePath)
    {
        if (string.IsNullOrEmpty(filter) || string.IsNullOrEmpty(filePath)) return 1;
        string ext = SysPath.GetExtension(filePath).ToLowerInvariant();
        var parts = filter.Split('|');
        for (int i = 1; i < parts.Length; i += 2)
        {
            foreach (var pattern in parts[i].Split(';'))
            {
                if (pattern.Trim().EndsWith(ext, StringComparison.OrdinalIgnoreCase))
                    return (i + 1) / 2;
            }
        }
        return 1;
    }

    // ── P/Invoke (Windows only — TODO: replace with cross-platform Avalonia dialogs) ──

    // ── Plattformunabhängige Dialoge (Linux / macOS) ──────────────────────────

    private static async Task<Substitutes.DialogResult> ShowAvaloniaMessageBoxAsync(
        Window? owner, string text, string caption, Substitutes.MessageBoxButtons buttons)
    {
        var tcs = new TaskCompletionSource<Substitutes.DialogResult>();

        var dialog = new Window
        {
            Title                   = caption,
            SizeToContent           = SizeToContent.WidthAndHeight,
            WindowStartupLocation   = owner != null
                                        ? WindowStartupLocation.CenterOwner
                                        : WindowStartupLocation.CenterScreen,
            CanResize               = false,
            MinWidth                = 300,
            MaxWidth                = 600,
        };

        var btnPanel = new StackPanel
        {
            Orientation         = Orientation.Horizontal,
            HorizontalAlignment = HorizontalAlignment.Center,
            Spacing             = 8,
        };

        void Add(string label, Substitutes.DialogResult res)
        {
            var b = new Button { Content = label, MinWidth = 80 };
            b.Click += (_, _) => { tcs.TrySetResult(res); dialog.Close(); };
            btnPanel.Children.Add(b);
        }

        switch (buttons)
        {
            case Substitutes.MessageBoxButtons.OKCancel:
                Add("OK",          Substitutes.DialogResult.OK);
                Add("Abbrechen",   Substitutes.DialogResult.Cancel);
                break;
            case Substitutes.MessageBoxButtons.YesNo:
                Add("Ja",          Substitutes.DialogResult.Yes);
                Add("Nein",        Substitutes.DialogResult.No);
                break;
            case Substitutes.MessageBoxButtons.YesNoCancel:
                Add("Ja",          Substitutes.DialogResult.Yes);
                Add("Nein",        Substitutes.DialogResult.No);
                Add("Abbrechen",   Substitutes.DialogResult.Cancel);
                break;
            case Substitutes.MessageBoxButtons.AbortRetryIgnore:
                Add("Abbrechen",   Substitutes.DialogResult.Abort);
                Add("Wiederholen", Substitutes.DialogResult.Retry);
                Add("Ignorieren",  Substitutes.DialogResult.Ignore);
                break;
            case Substitutes.MessageBoxButtons.RetryCancel:
                Add("Wiederholen", Substitutes.DialogResult.Retry);
                Add("Abbrechen",   Substitutes.DialogResult.Cancel);
                break;
            default:
                Add("OK",          Substitutes.DialogResult.OK);
                break;
        }

        dialog.Content = new StackPanel
        {
            Margin   = new Thickness(20),
            Spacing  = 16,
            Children =
            {
                new TextBlock { Text = text, TextWrapping = TextWrapping.Wrap, MaxWidth = 560 },
                btnPanel,
            }
        };

        dialog.Closing += (_, _) => tcs.TrySetResult(Substitutes.DialogResult.Cancel);

        if (owner != null)
            await dialog.ShowDialog(owner);
        else
            dialog.Show();

        return await tcs.Task;
    }

    private static async Task<(bool ok, Substitutes.Color color)> ShowAvaloniaColorDialogAsync(
        Window? owner, Substitutes.Color initial)
    {
        var tcs = new TaskCompletionSource<(bool, Substitutes.Color)>();

        int initArgb = initial.ToArgb();
        var sliderR = new Slider { Minimum = 0, Maximum = 255, Value = (initArgb >> 16) & 0xFF, Width = 220 };
        var sliderG = new Slider { Minimum = 0, Maximum = 255, Value = (initArgb >>  8) & 0xFF, Width = 220 };
        var sliderB = new Slider { Minimum = 0, Maximum = 255, Value =  initArgb        & 0xFF,  Width = 220 };

        var preview = new Border
        {
            Width  = 220,
            Height = 40,
            Background = new SolidColorBrush(Color.FromRgb(
                (byte)sliderR.Value, (byte)sliderG.Value, (byte)sliderB.Value)),
        };

        void UpdatePreview()
            => preview.Background = new SolidColorBrush(Color.FromRgb(
                (byte)sliderR.Value, (byte)sliderG.Value, (byte)sliderB.Value));

        sliderR.PropertyChanged += (_, e) => { if (e.Property.Name == "Value") UpdatePreview(); };
        sliderG.PropertyChanged += (_, e) => { if (e.Property.Name == "Value") UpdatePreview(); };
        sliderB.PropertyChanged += (_, e) => { if (e.Property.Name == "Value") UpdatePreview(); };

        var okBtn     = new Button { Content = "OK",        MinWidth = 80 };
        var cancelBtn = new Button { Content = "Abbrechen", MinWidth = 80 };

        var dialog = new Window
        {
            Title                 = "Farbe auswählen",
            SizeToContent         = SizeToContent.WidthAndHeight,
            WindowStartupLocation = owner != null
                                        ? WindowStartupLocation.CenterOwner
                                        : WindowStartupLocation.CenterScreen,
            CanResize             = false,
        };

        okBtn.Click += (_, _) =>
        {
            tcs.TrySetResult((true, Substitutes.Color.FromArgb(
                255, (int)sliderR.Value, (int)sliderG.Value, (int)sliderB.Value)));
            dialog.Close();
        };
        cancelBtn.Click += (_, _) => { tcs.TrySetResult((false, initial)); dialog.Close(); };
        dialog.Closing  += (_, _) => tcs.TrySetResult((false, initial));

        dialog.Content = new StackPanel
        {
            Margin   = new Thickness(16),
            Spacing  = 6,
            Children =
            {
                preview,
                new TextBlock { Text = "Rot" },   sliderR,
                new TextBlock { Text = "Grün" },  sliderG,
                new TextBlock { Text = "Blau" },  sliderB,
                new StackPanel
                {
                    Orientation         = Orientation.Horizontal,
                    HorizontalAlignment = HorizontalAlignment.Right,
                    Spacing             = 8,
                    Children            = { okBtn, cancelBtn },
                },
            }
        };

        if (owner != null)
            await dialog.ShowDialog(owner);
        else
            dialog.Show();

        return await tcs.Task;
    }

    private static class NativeMethods
    {
        [DllImport("user32.dll", CharSet = CharSet.Unicode)]
        public static extern int MessageBox(IntPtr hWnd, string text, string caption, uint type);

        [DllImport("user32.dll")]
        public static extern short GetKeyState(int nVirtKey);

        [DllImport("user32.dll")]
        public static extern bool GetCursorPos(out POINT lpPoint);

        [StructLayout(LayoutKind.Sequential)]
        public struct POINT { public int X; public int Y; }

        [DllImport("comdlg32.dll", CharSet = CharSet.Unicode)]
        public static extern bool ChooseColor(ref CHOOSECOLOR lpcc);

        public const uint CC_RGBINIT  = 0x00000001;
        public const uint CC_FULLOPEN = 0x00000002;

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        public struct CHOOSECOLOR
        {
            public uint   lStructSize;
            public IntPtr hwndOwner;
            public IntPtr hInstance;
            public uint   rgbResult;
            public IntPtr lpCustColors;
            public uint   Flags;
            public IntPtr lCustData;
            public IntPtr lpfnHook;
            public IntPtr lpTemplateName;
        }
    }
}
