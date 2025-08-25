using System;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using Microsoft.Win32; // nur falls du SystemEvents nutzt

static class UiMetrics
{
    private const int SM_CXSMICON = 49;
    private const int SM_CYSMICON = 50;
    private const int SM_CXMENUCHECK = 71;
    private const int SM_CYMENUCHECK = 72;

    [DllImport("user32.dll")] static extern uint GetDpiForWindow(IntPtr hWnd);
    [DllImport("user32.dll")] static extern int GetSystemMetricsForDpi(int nIndex, uint dpi);

    public static Size GetToolbarImageSize(Control c)
    {
        uint dpi = GetDpiForWindow(c.Handle);
        int w = GetSystemMetricsForDpi(SM_CXSMICON, dpi);
        int h = GetSystemMetricsForDpi(SM_CYSMICON, dpi);
        return new Size(w, h);
    }

    public static Size GetMenuImageSize(Control c)
    {
        uint dpi = GetDpiForWindow(c.Handle);
        int w = GetSystemMetricsForDpi(SM_CXMENUCHECK, dpi);
        int h = GetSystemMetricsForDpi(SM_CYMENUCHECK, dpi);
        return new Size(w, h);
    }

    public static Font GetMenuFont() => SystemFonts.MenuFont;
}
