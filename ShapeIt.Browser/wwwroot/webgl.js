// WebGL2 display-list renderer, driven from C# (PaintToWebGL) via [JSImport].
// Mirrors CADability.Avalonia's PaintToOpenGL: a lit shader for triangle
// surfaces and an unlit shader for edges/lines. Shaders are the desktop
// GLSL 330 sources ported to GLSL ES 3.00 (WebGL2).
//
// Vertex format for every buffer: position(3) + normal(3) + color(4) = 10 floats.

let gl = null;
let lit = null;    // { prog, uMVP, uModel, uLightDir, uLightColor, uAmbient, uColorOverride }
let unlit = null;  // { prog, uMVP, uColorOverride }
let point = null;  // { prog, uMVP, uColorOverride, uPointSize, uPointSymbol }
let tex = null;    // { prog, uMVP, uTexture } — textured quad (bitmaps + text)
let canvas = null;

// Display lists hold triangle, line and per-symbol point buffers, plus textured quads.
// id -> { triVao, triCount, lineVao, lineCount, points:[{vao,count,symbol}], quads:[{vao,texture}] }
const lists = new Map();
let nextId = 1;

// Cached WebGL textures (bitmaps). id -> WebGLTexture
const textures = new Map();
let nextTexId = 1;

const PointSpritePixels = 11;

const FLOATS_PER_VERTEX = 10;
const STRIDE = FLOATS_PER_VERTEX * 4;

const LIT_VS = `#version 300 es
precision highp float;
layout(location=0) in vec3 aPosition;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec4 aColor;
uniform mat4 uMVP;
uniform mat4 uModel;
out vec3 vNormal;
out vec4 vColor;
void main() {
    vNormal = mat3(transpose(inverse(uModel))) * aNormal;
    vColor = aColor;
    gl_Position = uMVP * vec4(aPosition, 1.0);
}`;

const LIT_FS = `#version 300 es
precision highp float;
in vec3 vNormal;
in vec4 vColor;
uniform vec3 uLightDir;
uniform vec3 uLightColor;
uniform vec3 uAmbient;
uniform vec4 uColorOverride;
out vec4 FragColor;
void main() {
    vec4 baseColor = (uColorOverride.w > 0.5) ? vec4(uColorOverride.rgb, vColor.a) : vColor;
    vec3  n       = normalize(vNormal);
    float diff    = max(abs(dot(n, normalize(uLightDir))), 0.0);
    vec3  diffuse = diff * uLightColor;
    vec3  viewDir = vec3(0.0, 0.0, 1.0);
    vec3  halfDir = normalize(normalize(uLightDir) + viewDir);
    float spec    = pow(max(abs(dot(n, halfDir)), 0.0), 16.0);
    vec3  specular= spec * uLightColor * 0.3;
    vec3  result  = (uAmbient + diffuse + specular) * baseColor.rgb;
    FragColor = vec4(result, baseColor.a);
}`;

const UNLIT_VS = `#version 300 es
precision highp float;
layout(location=0) in vec3 aPosition;
layout(location=2) in vec4 aColor;
uniform mat4 uMVP;
out vec4 vColor;
void main() {
    vColor = aColor;
    gl_Position = uMVP * vec4(aPosition, 1.0);
}`;

const UNLIT_FS = `#version 300 es
precision highp float;
in vec4 vColor;
uniform vec4 uColorOverride;
out vec4 FragColor;
void main() {
    FragColor = (uColorOverride.w > 0.5) ? vec4(uColorOverride.rgb, vColor.a) : vColor;
}`;

// Point sprites — desktop PointVertexShader/PointFragmentShader ported to GLSL ES 3.00.
const POINT_VS = `#version 300 es
precision highp float;
layout(location=0) in vec3 aPosition;
layout(location=2) in vec4 aColor;
uniform mat4 uMVP;
uniform float uPointSize;
uniform vec4 uColorOverride;
out vec4 vColor;
void main() {
    gl_Position = uMVP * vec4(aPosition, 1.0);
    gl_PointSize = uPointSize;
    vColor = (uColorOverride.w > 0.5) ? vec4(uColorOverride.rgb, aColor.a) : aColor;
}`;

const POINT_FS = `#version 300 es
precision highp float;
in vec4 vColor;
uniform int uPointSymbol;
out vec4 FragColor;
void main() {
    vec2 uv = gl_PointCoord * 2.0 - 1.0;
    float r = length(uv);

    bool inside    = false;
    int  core      = uPointSymbol & 7;
    bool hasSquare = (uPointSymbol & 16) != 0;
    bool hasCircle = (uPointSymbol & 32) != 0;
    bool isSelect  = (uPointSymbol & 64) != 0;

    if (isSelect) {
        inside = max(abs(uv.x), abs(uv.y)) <= 1.0;
    } else {
        if      (core == 1) inside = r <= 0.3;
        else if (core == 2) inside = (abs(uv.x) <= 0.2 || abs(uv.y) <= 0.2) && r <= 1.0;
        else if (core == 3) inside = (abs(uv.x - uv.y) <= 0.3 || abs(uv.x + uv.y) <= 0.3) && r <= 1.0;
        else if (core == 4) inside = abs(uv.x) <= 0.2 && r <= 1.0;

        if (hasSquare) { float m = max(abs(uv.x), abs(uv.y)); inside = inside || (m > 0.6 && m <= 1.0); }
        if (hasCircle) inside = inside || (r > 0.6 && r <= 1.0);
    }

    if (!inside) discard;
    FragColor = vColor;
}`;

// Textured quad in world space (bitmaps + rasterized text). pos(3) + uv(2).
const TEX_VS = `#version 300 es
precision highp float;
layout(location=0) in vec3 aPos;
layout(location=1) in vec2 aUV;
uniform mat4 uMVP;
out vec2 vUV;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vUV = aUV;
}`;

const TEX_FS = `#version 300 es
precision highp float;
in vec2 vUV;
uniform sampler2D uTexture;
out vec4 FragColor;
void main() {
    vec4 c = texture(uTexture, vUV);
    if (c.a < 0.01) discard;   // skip fully-transparent texels (text background)
    FragColor = c;
}`;

function compile(type, src) {
    const s = gl.createShader(type);
    gl.shaderSource(s, src);
    gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS))
        throw new Error("shader compile: " + gl.getShaderInfoLog(s) + "\n" + src);
    return s;
}

function link(vsSrc, fsSrc) {
    const p = gl.createProgram();
    gl.attachShader(p, compile(gl.VERTEX_SHADER, vsSrc));
    gl.attachShader(p, compile(gl.FRAGMENT_SHADER, fsSrc));
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS))
        throw new Error("program link: " + gl.getProgramInfoLog(p));
    return p;
}

export function init(canvasId) {
    canvas = document.getElementById(canvasId);
    // preserveDrawingBuffer:true so readPixels() returns the rendered frame reliably
    // (the canvas is offscreen; C# reads the pixels into an Avalonia bitmap).
    gl = canvas.getContext("webgl2", { antialias: true, depth: true, stencil: true, preserveDrawingBuffer: true });
    if (!gl) throw new Error("WebGL2 not available");

    const litProg = link(LIT_VS, LIT_FS);
    lit = {
        prog: litProg,
        uMVP: gl.getUniformLocation(litProg, "uMVP"),
        uModel: gl.getUniformLocation(litProg, "uModel"),
        uLightDir: gl.getUniformLocation(litProg, "uLightDir"),
        uLightColor: gl.getUniformLocation(litProg, "uLightColor"),
        uAmbient: gl.getUniformLocation(litProg, "uAmbient"),
        uColorOverride: gl.getUniformLocation(litProg, "uColorOverride"),
    };

    const unlitProg = link(UNLIT_VS, UNLIT_FS);
    unlit = {
        prog: unlitProg,
        uMVP: gl.getUniformLocation(unlitProg, "uMVP"),
        uColorOverride: gl.getUniformLocation(unlitProg, "uColorOverride"),
    };

    const pointProg = link(POINT_VS, POINT_FS);
    point = {
        prog: pointProg,
        uMVP: gl.getUniformLocation(pointProg, "uMVP"),
        uColorOverride: gl.getUniformLocation(pointProg, "uColorOverride"),
        uPointSize: gl.getUniformLocation(pointProg, "uPointSize"),
        uPointSymbol: gl.getUniformLocation(pointProg, "uPointSymbol"),
    };

    const texProg = link(TEX_VS, TEX_FS);
    tex = {
        prog: texProg,
        uMVP: gl.getUniformLocation(texProg, "uMVP"),
        uTexture: gl.getUniformLocation(texProg, "uTexture"),
    };

    console.log("WebGL2 initialised:", gl.getParameter(gl.VERSION));
}

// Resize the offscreen render target.
export function setSize(w, h) {
    if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
}

// Read the rendered frame into the C# buffer (RGBA8, bottom-up).
// gl.readPixels needs a real TypedArray; the C# MemoryView is a wrapper, so read into a
// Uint8Array first, then copy into the MemoryView via its .set() method.
export function readPixels(buf, w, h) {
    const tmp = new Uint8Array(w * h * 4);
    gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, tmp);
    buf.set(tmp);
}

function makeVao(dataArray) {
    const data = new Float32Array(dataArray);
    const vao = gl.createVertexArray();
    const vbo = gl.createBuffer();
    gl.bindVertexArray(vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, STRIDE, 0);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 3, gl.FLOAT, false, STRIDE, 3 * 4);
    gl.enableVertexAttribArray(2);
    gl.vertexAttribPointer(2, 4, gl.FLOAT, false, STRIDE, 6 * 4);
    gl.bindVertexArray(null);
    return { vao, count: data.length / FLOATS_PER_VERTEX };
}

function newEntry() {
    return { triVao: null, triCount: 0, lineVao: null, lineCount: 0, points: [], quads: [] };
}

// tri / line: flat number arrays (10 floats per vertex), possibly empty.
export function createList(tri, line) {
    const entry = newEntry();
    if (tri && tri.length > 0) { const r = makeVao(tri); entry.triVao = r.vao; entry.triCount = r.count; }
    if (line && line.length > 0) { const r = makeVao(line); entry.lineVao = r.vao; entry.lineCount = r.count; }
    const id = nextId++;
    lists.set(id, entry);
    return id;
}

// Append a point bucket (one PointSymbol) to an existing list. data: 10 floats/vertex.
export function addPointsToList(id, symbol, data) {
    const e = lists.get(id);
    if (!e || !data || data.length === 0) return;
    const r = makeVao(data);
    e.points.push({ vao: r.vao, count: r.count, symbol });
}

// Append a textured quad (bitmap or text) to an existing list. corners: 18 floats
// (p0,p1,p2,p3 as xyz, only 12 used) — we pass the 4 corners as 12 floats [x,y,z]*4.
// texId is a texture handle from createTexture().
export function addQuadToList(id, corners, texId) {
    const e = lists.get(id);
    if (!e) return;
    const vao = makeQuadVao(corners);
    e.quads.push({ vao, texture: textures.get(texId) });
}

export function deleteList(id) {
    lists.delete(id);   // VAOs/VBOs collected with the context; fine for the PoC
}

// ── Textures ─────────────────────────────────────────────────────────────
// Build a textured-quad VAO. corners = [x0,y0,z0, x1,y1,z1, x2,y2,z2, x3,y3,z3]
// laid out as two triangles with UVs matching the desktop DrawTexturedQuad:
//   P0 (0,1) P1 (1,1) P2 (1,0) P3 (0,0)
function makeQuadVao(corners) {
    const c = corners;
    const verts = new Float32Array([
        c[0], c[1], c[2], 0, 1,
        c[3], c[4], c[5], 1, 1,
        c[6], c[7], c[8], 1, 0,
        c[0], c[1], c[2], 0, 1,
        c[6], c[7], c[8], 1, 0,
        c[9], c[10], c[11], 0, 0,
    ]);
    const vao = gl.createVertexArray();
    const vbo = gl.createBuffer();
    gl.bindVertexArray(vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
    gl.bufferData(gl.ARRAY_BUFFER, verts, gl.STATIC_DRAW);
    const s = 5 * 4;
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, s, 0);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 2, gl.FLOAT, false, s, 3 * 4);
    gl.bindVertexArray(null);
    return vao;
}

// Create an RGBA texture from raw bytes (tightly packed, top-down rows). Returns a texId.
export function createTexture(data, w, h) {
    const t = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, t);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1);
    const pixels = new Uint8Array(data);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
    gl.bindTexture(gl.TEXTURE_2D, null);
    const id = nextTexId++;
    textures.set(id, t);
    return id;
}

// Rasterize a string to an RGBA texture via an offscreen 2-D canvas.
// Returns [texId, pixelWidth, pixelHeight]. Color is 0..255 RGBA.
const _textCanvas = (typeof document !== "undefined") ? document.createElement("canvas") : null;
export function rasterizeText(text, fontName, fontPx, r, g, b, a) {
    const ctx = _textCanvas.getContext("2d");
    const font = `${Math.max(1, Math.round(fontPx))}px ${fontName || "sans-serif"}`;
    ctx.font = font;
    const metrics = ctx.measureText(text);
    let w = Math.max(1, Math.ceil(metrics.width));
    // Use font ascent/descent if available, else fall back to fontPx.
    let asc = metrics.actualBoundingBoxAscent || metrics.fontBoundingBoxAscent || fontPx * 0.8;
    let desc = metrics.actualBoundingBoxDescent || metrics.fontBoundingBoxDescent || fontPx * 0.2;
    let h = Math.max(1, Math.ceil(asc + desc));
    _textCanvas.width = w;
    _textCanvas.height = h;
    // measureText resets after resize; re-set font and draw.
    ctx.clearRect(0, 0, w, h);
    ctx.font = font;
    ctx.textBaseline = "alphabetic";
    ctx.fillStyle = `rgba(${r},${g},${b},${a / 255})`;
    ctx.fillText(text, 0, asc);
    const img = ctx.getImageData(0, 0, w, h);
    const id = createTexture(img.data, w, h);
    // Report the ascent so the caller can place the baseline; pack into the height slot
    // is not needed — caller scales by total height. Return ascent fraction via 4th value.
    return [id, w, h, asc / h];
}

export function deleteTexture(id) {
    const t = textures.get(id);
    if (t) { gl.deleteTexture(t); textures.delete(id); }
}

// ── Immediate-mode draws (no display list) ──────────────────────────────────
// Draw a point bucket immediately. data: 10 floats/vertex.
export function drawPoints(mvp, data, ovr, symbol) {
    if (!data || data.length === 0) return;
    const r = makeVao(data);
    gl.useProgram(point.prog);
    gl.uniformMatrix4fv(point.uMVP, false, new Float32Array(mvp));
    const o = (ovr && ovr.length >= 3) ? [ovr[0], ovr[1], ovr[2], 1.0] : [0, 0, 0, 0];
    gl.uniform4f(point.uColorOverride, o[0], o[1], o[2], o[3]);
    gl.uniform1f(point.uPointSize, PointSpritePixels);
    gl.uniform1i(point.uPointSymbol, symbol | 0);
    gl.bindVertexArray(r.vao);
    gl.drawArrays(gl.POINTS, 0, r.count);
    gl.bindVertexArray(null);
}

// Draw a list of immediate vertices with a chosen primitive (0=triangles,1=lines).
function drawImmediateVerts(mvp, data, ovr, prim) {
    if (!data || data.length === 0) return;
    const r = makeVao(data);
    const o = (ovr && ovr.length >= 3) ? [ovr[0], ovr[1], ovr[2], 1.0] : [0, 0, 0, 0];
    gl.useProgram(unlit.prog);
    gl.uniformMatrix4fv(unlit.uMVP, false, new Float32Array(mvp));
    gl.uniform4f(unlit.uColorOverride, o[0], o[1], o[2], o[3]);
    gl.bindVertexArray(r.vao);
    gl.drawArrays(prim === 1 ? gl.LINES : gl.TRIANGLES, 0, r.count);
    gl.bindVertexArray(null);
}

// Lines/triangles in unlit mode (used for 2-D overlays and filled polylines).
export function drawLines(mvp, data, ovr) { drawImmediateVerts(mvp, data, ovr, 1); }
export function drawTriangles(mvp, data, ovr) { drawImmediateVerts(mvp, data, ovr, 0); }

// Draw a textured quad immediately. corners = 12 floats, texId from createTexture.
export function drawTexturedQuad(mvp, corners, texId) {
    const t = textures.get(texId);
    if (!t) return;
    const vao = makeQuadVao(corners);
    gl.useProgram(tex.prog);
    gl.uniformMatrix4fv(tex.uMVP, false, new Float32Array(mvp));
    gl.uniform1i(tex.uTexture, 0);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, t);
    gl.bindVertexArray(vao);
    gl.drawArrays(gl.TRIANGLES, 0, 6);
    gl.bindVertexArray(null);
    gl.bindTexture(gl.TEXTURE_2D, null);
    gl.deleteVertexArray(vao);
}

// ── GL state ─────────────────────────────────────────────────────────────
export function setDepthTest(on) {
    if (on) gl.enable(gl.DEPTH_TEST); else gl.disable(gl.DEPTH_TEST);
}
export function setBlend(on) {
    if (on) { gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA); }
    else gl.disable(gl.BLEND);
}
export function setLineWidth(w) {
    // NOTE: WebGL/ANGLE clamps line width to 1.0 in virtually all browsers; the call is
    // issued for parity but has no visible effect beyond 1px.
    gl.lineWidth(w);
}

export function beginFrame(r, g, b) {
    gl.viewport(0, 0, canvas.width, canvas.height);
    gl.clearColor(r, g, b, 1.0);
    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LEQUAL);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
}

// mvp, model: 16 floats (System.Numerics native/row order, used with transpose=false).
// lightDir: 3 floats. ovr: [] for none, else [r,g,b].
export function drawList(id, mvp, model, lightDir, ovr) {
    const e = lists.get(id);
    if (!e) return;
    const mvpA = new Float32Array(mvp);
    const modelA = new Float32Array(model);
    const override = (ovr && ovr.length >= 3) ? [ovr[0], ovr[1], ovr[2], 1.0] : [0, 0, 0, 0];

    if (e.triCount > 0) {
        gl.useProgram(lit.prog);
        gl.uniformMatrix4fv(lit.uMVP, false, mvpA);
        gl.uniformMatrix4fv(lit.uModel, false, modelA);
        gl.uniform3f(lit.uLightDir, lightDir[0], lightDir[1], lightDir[2]);
        gl.uniform3f(lit.uLightColor, 1.0, 1.0, 1.0);
        gl.uniform3f(lit.uAmbient, 0.2, 0.2, 0.2);
        gl.uniform4f(lit.uColorOverride, override[0], override[1], override[2], override[3]);
        gl.bindVertexArray(e.triVao);
        gl.drawArrays(gl.TRIANGLES, 0, e.triCount);
    }
    if (e.lineCount > 0) {
        gl.useProgram(unlit.prog);
        gl.uniformMatrix4fv(unlit.uMVP, false, mvpA);
        gl.uniform4f(unlit.uColorOverride, override[0], override[1], override[2], override[3]);
        gl.bindVertexArray(e.lineVao);
        gl.drawArrays(gl.LINES, 0, e.lineCount);
    }
    if (e.points && e.points.length > 0) {
        gl.useProgram(point.prog);
        gl.uniformMatrix4fv(point.uMVP, false, mvpA);
        gl.uniform4f(point.uColorOverride, override[0], override[1], override[2], override[3]);
        gl.uniform1f(point.uPointSize, PointSpritePixels);
        for (const p of e.points) {
            gl.uniform1i(point.uPointSymbol, p.symbol | 0);
            gl.bindVertexArray(p.vao);
            gl.drawArrays(gl.POINTS, 0, p.count);
        }
    }
    if (e.quads && e.quads.length > 0) {
        gl.useProgram(tex.prog);
        gl.uniformMatrix4fv(tex.uMVP, false, mvpA);
        gl.uniform1i(tex.uTexture, 0);
        gl.activeTexture(gl.TEXTURE0);
        for (const q of e.quads) {
            gl.bindTexture(gl.TEXTURE_2D, q.texture);
            gl.bindVertexArray(q.vao);
            gl.drawArrays(gl.TRIANGLES, 0, 6);
        }
        gl.bindTexture(gl.TEXTURE_2D, null);
    }
    gl.bindVertexArray(null);
}
