using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using Silk.NET.OpenGL;

namespace CADability.Forms.NET8
{
    /// <summary>
    /// Manages a WGL OpenGL 3.3 Core context for a Windows Forms control.
    ///
    /// Replaces the raw wglCreateContext / wglShareLists / PIXELFORMATDESCRIPTOR
    /// setup from the old PaintToOpenGL, but requests a Core Profile context
    /// via wglCreateContextAttribsARB so Silk.NET can load modern entry points.
    ///
    /// Usage:
    ///   var ctx = new WglContext();
    ///   ctx.Init(deviceContext, width, height, toBitmap);
    ///   ctx.MakeCurrent();
    ///   GL gl = ctx.CreateSilkBinding();   // pass to PaintToOpenGLModern
    ///   ...
    ///   ctx.Dispose();
    /// </summary>
    public sealed class WglContext : IDisposable
    {
        // ── Static shared-list chain (mirrors old MainRenderContext pattern) ─
        private static IntPtr s_mainDC      = IntPtr.Zero;
        private static IntPtr s_mainRC      = IntPtr.Zero;
        private static IntPtr s_lastRC      = IntPtr.Zero;
        private static readonly List<IntPtr> s_pendingDelete = new();
        private static readonly List<IntPtr> s_activeContexts = new();

        // ── Instance ─────────────────────────────────────────────────────────
        private IntPtr _dc = IntPtr.Zero;
        private IntPtr _rc = IntPtr.Zero;
        private IntPtr _controlHandle = IntPtr.Zero;
        private bool   _isBitmap;
        private bool   _disposed;

        // ── WGL constants ─────────────────────────────────────────────────────
        private const int WGL_CONTEXT_MAJOR_VERSION_ARB = 0x2091;
        private const int WGL_CONTEXT_MINOR_VERSION_ARB = 0x2092;
        private const int WGL_CONTEXT_PROFILE_MASK_ARB  = 0x9126;
        private const int WGL_CONTEXT_CORE_PROFILE_BIT_ARB = 0x0001;
        private const int WGL_CONTEXT_FLAGS_ARB         = 0x2094;
        private const int WGL_CONTEXT_DEBUG_BIT_ARB     = 0x0001;  // optional

        // ─────────────────────────────────────────────────────────────────────
        //  Public API
        // ─────────────────────────────────────────────────────────────────────

        /// <summary>
        /// Initialise from a Windows Forms control handle (normal window rendering).
        /// </summary>
        public void Init(Control ctrl)
        {
            if (ctrl.Handle == IntPtr.Zero)
                throw new InvalidOperationException("Control handle not yet created.");
            _controlHandle = ctrl.Handle;
            Init(GetDC(ctrl.Handle), ctrl.ClientSize.Width, ctrl.ClientSize.Height, toBitmap: false);
            ctrl.HandleDestroyed += OnHandleDestroyed;
        }

        /// <summary>
        /// Initialise from a raw device context (e.g. for bitmap rendering).
        /// Signature matches the old PaintToOpenGL.Init().
        /// </summary>
        public void Init(IntPtr deviceContext, int width, int height, bool toBitmap)
        {
            if (deviceContext == IntPtr.Zero)
                throw new ArgumentException("deviceContext must not be IntPtr.Zero");

            _dc       = deviceContext;
            _isBitmap = toBitmap;

            SetupPixelFormat(toBitmap);
            CreateContext(toBitmap);
            MakeCurrent();
            FlushPendingDeletes();
        }

        /// <summary>Makes this context current on the calling thread.</summary>
        public void MakeCurrent()
        {
            if (_rc != IntPtr.Zero && _dc != IntPtr.Zero)
                WglMakeCurrent(_dc, _rc);
        }

        /// <summary>
        /// Returns a Silk.NET GL instance bound to this context.
        /// Must be called after MakeCurrent().
        /// </summary>
        public GL CreateSilkBinding()
            => GL.GetApi(proc =>
            {
                nint addr = WglGetProcAddress(proc);
                if (addr == IntPtr.Zero)
                {
                    IntPtr lib = LoadLibrary("opengl32.dll");
                    if (lib != IntPtr.Zero)
                        addr = GetProcAddress(lib, proc);
                }
                return addr;
            });

        /// <summary>Swaps front/back buffer (double-buffered window rendering).</summary>
        public void SwapBuffers()
        {
            if (!_isBitmap && _dc != IntPtr.Zero)
                GdiSwapBuffers(_dc);
        }

        [DllImport("gdi32.dll", EntryPoint = "SwapBuffers")]
        private static extern bool GdiSwapBuffers(IntPtr hdc);

        // ─────────────────────────────────────────────────────────────────────
        //  Context creation
        // ─────────────────────────────────────────────────────────────────────

        private unsafe void SetupPixelFormat(bool toBitmap)
        {
            var pfd = new PIXELFORMATDESCRIPTOR
            {
                nSize        = (ushort)Marshal.SizeOf<PIXELFORMATDESCRIPTOR>(),
                nVersion     = 1,
                dwFlags      = toBitmap
                               ? PFD_SUPPORT_OPENGL | PFD_DRAW_TO_BITMAP
                               : PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
                iPixelType   = PFD_TYPE_RGBA,
                cColorBits   = 32,
                cDepthBits   = 24,
                cStencilBits = 8,
                iLayerType   = PFD_MAIN_PLANE
            };

            int fmt = ChoosePixelFormat(_dc, ref pfd);
            if (fmt == 0)
                throw new InvalidOperationException("ChoosePixelFormat failed.");

            if (!SetPixelFormat(_dc, fmt, ref pfd))
            {
                // SetPixelFormat may fail if already set – that is OK for shared contexts
                int err = Marshal.GetLastWin32Error();
                if (GetPixelFormat(_dc) == 0)
                    throw new InvalidOperationException(
                        $"SetPixelFormat failed (win32 error {err}).");
            }
        }

        private void CreateContext(bool toBitmap)
        {
            // Step 1: create a legacy context so we can query wglCreateContextAttribsARB
            IntPtr legacyRC = WglCreateContext(_dc);
            if (legacyRC == IntPtr.Zero)
                throw new InvalidOperationException("wglCreateContext (legacy) failed.");

            WglMakeCurrent(_dc, legacyRC);

            // Step 2: try to get wglCreateContextAttribsARB
            var createAttribs = GetWglProc<WglCreateContextAttribsARBDelegate>(
                "wglCreateContextAttribsARB");

            if (createAttribs != null)
            {
                // Request OpenGL 3.3 Core Profile
                int[] attribs = {
                    WGL_CONTEXT_MAJOR_VERSION_ARB, 3,
                    WGL_CONTEXT_MINOR_VERSION_ARB, 3,
                    WGL_CONTEXT_PROFILE_MASK_ARB,  WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
#if DEBUG
                    WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_DEBUG_BIT_ARB,
#endif
                    0
                };

                // Share with the main context so VAOs/VBOs are accessible across
                // multiple views (mirrors the old wglShareLists pattern)
                IntPtr shareWith = toBitmap ? IntPtr.Zero : s_mainRC;
                _rc = createAttribs(_dc, shareWith, attribs);

                // If sharing failed, try without
                if (_rc == IntPtr.Zero && shareWith != IntPtr.Zero)
                    _rc = createAttribs(_dc, IntPtr.Zero, attribs);
            }

            // Step 3: fall back to the legacy context if ARB extension unavailable
            if (_rc == IntPtr.Zero)
            {
                System.Diagnostics.Debug.WriteLine(
                    "wglCreateContextAttribsARB not available – falling back to legacy context. " +
                    "Shaders require at least OpenGL 3.3.");
                _rc = legacyRC;
                legacyRC = IntPtr.Zero; // don't delete it
            }

            // Delete the temporary legacy context (unless we kept it as fallback)
            if (legacyRC != IntPtr.Zero)
            {
                WglMakeCurrent(IntPtr.Zero, IntPtr.Zero);
                WglDeleteContext(legacyRC);
            }

            // Step 4: register in static chain (mirrors old MainRenderContext logic)
            s_activeContexts.Add(_rc);
            if (!toBitmap)
            {
                if (s_mainRC == IntPtr.Zero)
                {
                    s_mainRC = _rc;
                    s_lastRC = _rc;
                    s_mainDC = _dc;
                    Application.ApplicationExit += OnApplicationExit;
                }
                else
                {
                    s_lastRC = _rc;
                }
            }
        }

        // ─────────────────────────────────────────────────────────────────────
        //  Cleanup
        // ─────────────────────────────────────────────────────────────────────

        private void OnHandleDestroyed(object? sender, EventArgs e)
        {
            if (!_isBitmap && _rc != IntPtr.Zero && _rc != s_mainRC)
            {
                lock (s_pendingDelete) { s_pendingDelete.Add(_rc); }
                _rc = IntPtr.Zero;
            }
        }

        private static void OnApplicationExit(object? sender, EventArgs e)
        {
            if (s_mainRC != IntPtr.Zero)
            {
                WglMakeCurrent(IntPtr.Zero, IntPtr.Zero);
                WglDeleteContext(s_mainRC);
                s_mainRC = IntPtr.Zero;
            }
            foreach (var rc in s_activeContexts)
                if (rc != IntPtr.Zero) WglDeleteContext(rc);
            s_activeContexts.Clear();
        }

        private static void FlushPendingDeletes()
        {
            lock (s_pendingDelete)
            {
                foreach (var rc in s_pendingDelete)
                {
                    WglDeleteContext(rc);
                    s_activeContexts.Remove(rc);
                }
                s_pendingDelete.Clear();
            }
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            if (_rc != IntPtr.Zero && _rc != s_mainRC)
            {
                lock (s_pendingDelete) { s_pendingDelete.Add(_rc); }
                _rc = IntPtr.Zero;
            }

            if (_controlHandle != IntPtr.Zero && _dc != IntPtr.Zero)
            {
                ReleaseDC(_controlHandle, _dc);
                _dc = IntPtr.Zero;
            }
        }

        // ─────────────────────────────────────────────────────────────────────
        //  P/Invoke
        // ─────────────────────────────────────────────────────────────────────

        [UnmanagedFunctionPointer(CallingConvention.Winapi)]
        private delegate IntPtr WglCreateContextAttribsARBDelegate(
            IntPtr hDC, IntPtr hShareContext, int[] attribList);

        private static T? GetWglProc<T>(string name) where T : Delegate
        {
            IntPtr addr = WglGetProcAddress(name);
            return addr == IntPtr.Zero ? null
                : Marshal.GetDelegateForFunctionPointer<T>(addr);
        }

        // GDI / User32
        [DllImport("user32.dll")]             static extern IntPtr GetDC(IntPtr hwnd);
        [DllImport("user32.dll")]             static extern int    ReleaseDC(IntPtr hwnd, IntPtr hdc);

        // GDI32 pixel format
        [DllImport("gdi32.dll")]              static extern int  ChoosePixelFormat(IntPtr hdc, ref PIXELFORMATDESCRIPTOR ppfd);
        [DllImport("gdi32.dll")]              static extern bool SetPixelFormat(IntPtr hdc, int format, ref PIXELFORMATDESCRIPTOR ppfd);
        [DllImport("gdi32.dll")]              static extern int  GetPixelFormat(IntPtr hdc);

        // WGL
        [DllImport("opengl32.dll", EntryPoint = "wglCreateContext")] static extern IntPtr WglCreateContext(IntPtr hdc);
        [DllImport("opengl32.dll", EntryPoint = "wglDeleteContext")] static extern bool   WglDeleteContext(IntPtr hglrc);
        [DllImport("opengl32.dll", EntryPoint = "wglMakeCurrent")]  static extern bool   WglMakeCurrent(IntPtr hdc, IntPtr hglrc);
        [DllImport("opengl32.dll", EntryPoint = "wglGetProcAddress")]
                                              static extern nint   WglGetProcAddress(string proc);

        // Kernel32 (fallback for core OpenGL functions)
        [DllImport("kernel32.dll")]           static extern IntPtr LoadLibrary(string lpFileName);
        [DllImport("kernel32.dll")]           static extern IntPtr GetProcAddress(IntPtr hModule, string lpProcName);

        // ── PIXELFORMATDESCRIPTOR ─────────────────────────────────────────────
        private const uint PFD_DRAW_TO_WINDOW = 0x00000004;
        private const uint PFD_DRAW_TO_BITMAP = 0x00000008;
        private const uint PFD_SUPPORT_OPENGL = 0x00000020;
        private const uint PFD_DOUBLEBUFFER   = 0x00000001;
        private const byte PFD_TYPE_RGBA      = 0;
        private const byte PFD_MAIN_PLANE     = 0;

        [StructLayout(LayoutKind.Sequential)]
        private struct PIXELFORMATDESCRIPTOR
        {
            public ushort nSize, nVersion;
            public uint   dwFlags;
            public byte   iPixelType, cColorBits, cRedBits, cRedShift;
            public byte   cGreenBits, cGreenShift, cBlueBits, cBlueShift;
            public byte   cAlphaBits, cAlphaShift, cAccumBits;
            public byte   cAccumRedBits, cAccumGreenBits, cAccumBlueBits, cAccumAlphaBits;
            public byte   cDepthBits, cStencilBits, cAuxBuffers, iLayerType, bReserved;
            public uint   dwLayerMask, dwVisibleMask, dwDamageMask;
        }
    }
}
