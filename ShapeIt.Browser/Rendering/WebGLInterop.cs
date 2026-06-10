using System;
using System.Runtime.InteropServices.JavaScript;

namespace ShapeIt.Browser
{
    /// <summary>
    /// [JSImport] bridge to wwwroot/webgl.js — the WebGL2 display-list renderer that
    /// replaces the (browser-unsupported) Avalonia OpenGlControlBase path. Matrices are
    /// 16 floats in System.Numerics native order, uploaded with transpose=false.
    /// </summary>
    internal static partial class WebGLInterop
    {
        [JSImport("init", "webgl")]
        public static partial void Init(string canvasId);

        [JSImport("setSize", "webgl")]
        public static partial void SetSize(int w, int h);

        // Read the rendered RGBA8 frame (bottom-up) into a C# buffer via a fast MemoryView copy.
        [JSImport("readPixels", "webgl")]
        public static partial void ReadPixels(
            [JSMarshalAs<JSType.MemoryView>] ArraySegment<byte> buffer, int w, int h);

        [JSImport("beginFrame", "webgl")]
        public static partial void BeginFrame(double r, double g, double b);

        [JSImport("createList", "webgl")]
        public static partial int CreateList(
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] tri,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] line);

        [JSImport("deleteList", "webgl")]
        public static partial void DeleteList(int id);

        [JSImport("drawList", "webgl")]
        public static partial void DrawList(
            int id,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] mvp,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] model,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] lightDir,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] overrideColor);

        // ── Points ───────────────────────────────────────────────────────
        // Append a point bucket (one PointSymbol) to an existing display list.
        [JSImport("addPointsToList", "webgl")]
        public static partial void AddPointsToList(
            int id, int symbol,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] data);

        // Draw a point bucket immediately (no display list).
        [JSImport("drawPoints", "webgl")]
        public static partial void DrawPoints(
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] mvp,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] data,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] overrideColor,
            int symbol);

        // ── 2-D overlays / filled polylines (unlit immediate draws) ───────
        [JSImport("drawLines", "webgl")]
        public static partial void DrawLines(
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] mvp,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] data,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] overrideColor);

        [JSImport("drawTriangles", "webgl")]
        public static partial void DrawTriangles(
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] mvp,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] data,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] overrideColor);

        // ── Textures / textured quads (bitmaps + text) ───────────────────
        // Upload an RGBA texture (tightly packed, top-down) and return a texId.
        [JSImport("createTexture", "webgl")]
        public static partial int CreateTexture(
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] data, int w, int h);

        [JSImport("deleteTexture", "webgl")]
        public static partial void DeleteTexture(int texId);

        // Rasterize a string to an RGBA texture via an offscreen 2-D canvas.
        // Returns [texId, pixelWidth, pixelHeight, ascentFraction].
        [JSImport("rasterizeText", "webgl")]
        [return: JSMarshalAs<JSType.Array<JSType.Number>>]
        public static partial double[] RasterizeText(
            string text, string fontName, double fontPx,
            int r, int g, int b, int a);

        // Append a textured quad (4 corners = 12 floats) referencing texId to a list.
        [JSImport("addQuadToList", "webgl")]
        public static partial void AddQuadToList(
            int id,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] corners,
            int texId);

        // Draw a textured quad immediately.
        [JSImport("drawTexturedQuad", "webgl")]
        public static partial void DrawTexturedQuad(
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] mvp,
            [JSMarshalAs<JSType.Array<JSType.Number>>] double[] corners,
            int texId);

        // ── GL state ─────────────────────────────────────────────────────
        [JSImport("setDepthTest", "webgl")]
        public static partial void SetDepthTest(bool on);

        [JSImport("setBlend", "webgl")]
        public static partial void SetBlend(bool on);

        [JSImport("setLineWidth", "webgl")]
        public static partial void SetLineWidth(double width);
    }
}
