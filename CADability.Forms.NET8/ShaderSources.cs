namespace CADability.Forms.NET8
{
    /// <summary>
    /// GLSL shader sources for the modern OpenGL painter.
    ///
    /// Two shader programs are used:
    ///   1. LitProgram   – for Triangle() calls (surfaces with Phong lighting)
    ///   2. UnlitProgram – for Polyline(), Points(), 2-D overlays (flat color)
    ///
    /// Both programs share the same vertex layout:
    ///   location 0 : vec3  position
    ///   location 1 : vec3  normal
    ///   location 2 : vec4  color
    ///
    /// Uniforms shared by both:
    ///   mat4 uMVP          – combined Model-View-Projection matrix
    ///   mat4 uModel        – model matrix (for normal transform in lit shader)
    ///   vec4 uColorOverride– when w > 0.5, overrides vertex color (select mode)
    /// </summary>
    internal static class ShaderSources
    {
        // ── Shared vertex shader ───────────────────────────────────────────
        // Used by both lit and unlit programs.
        public const string VertexShader = @"
#version 330 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aColor;

uniform mat4 uMVP;
uniform mat4 uModel;

out vec3 vFragPos;   // world-space position for lighting
out vec3 vNormal;    // world-space normal
out vec4 vColor;

void main()
{
    vec4 worldPos = uModel * vec4(aPosition, 1.0);
    vFragPos  = worldPos.xyz;
    // Normal matrix = transpose(inverse(uModel)) – acceptable for uniform scaling
    vNormal   = mat3(transpose(inverse(uModel))) * aNormal;
    vColor    = aColor;
    gl_Position = uMVP * vec4(aPosition, 1.0);
}
";

        // ── Lit fragment shader (Phong, for Triangle surfaces) ─────────────
        public const string LitFragmentShader = @"
#version 330 core

in  vec3 vFragPos;
in  vec3 vNormal;
in  vec4 vColor;

uniform vec3 uLightDir;       // normalised, world space, points TOWARD light
uniform vec3 uLightColor;     // usually (1,1,1)
uniform vec3 uAmbient;        // e.g. (0.2, 0.2, 0.2)
uniform vec4 uColorOverride;  // w>0.5 → use xyz as color (select highlight)

out vec4 FragColor;

void main()
{
    vec4 baseColor = (uColorOverride.w > 0.5)
                     ? vec4(uColorOverride.rgb, vColor.a)
                     : vColor;

    // Two-sided lighting: use abs(dot) so back-faces are also lit
    vec3  n       = normalize(vNormal);
    float diff    = max(abs(dot(n, normalize(uLightDir))), 0.0);
    vec3  diffuse = diff * uLightColor;

    // Simple specular (Blinn-Phong)
    vec3  viewDir  = vec3(0.0, 0.0, 1.0); // parallel projection: view along -Z
    vec3  halfDir  = normalize(normalize(uLightDir) + viewDir);
    float spec     = pow(max(abs(dot(n, halfDir)), 0.0), 16.0);
    vec3  specular = spec * uLightColor * 0.3;

    vec3 result = (uAmbient + diffuse + specular) * baseColor.rgb;
    FragColor   = vec4(result, baseColor.a);
}
";

        // ── Unlit fragment shader (for lines, points, 2-D overlays) ───────
        public const string UnlitFragmentShader = @"
#version 330 core

in  vec4 vColor;

uniform vec4 uColorOverride;  // w>0.5 → override color

out vec4 FragColor;

void main()
{
    FragColor = (uColorOverride.w > 0.5)
                ? vec4(uColorOverride.rgb, vColor.a)
                : vColor;
}
";

        // ── Point sprite vertex shader ────────────────────────────────────
        // Same vertex layout as lit/unlit; normal attribute is unused.
        // gl_PointSize is set from uPointSize so the symbol is zoom-independent.
        public const string PointVertexShader = @"
#version 330 core

layout(location = 0) in vec3 aPosition;
layout(location = 2) in vec4 aColor;

uniform mat4  uMVP;
uniform float uPointSize;
uniform vec4  uColorOverride;

out vec4 vColor;

void main()
{
    gl_Position  = uMVP * vec4(aPosition, 1.0);
    gl_PointSize = uPointSize;
    vColor = (uColorOverride.w > 0.5)
             ? vec4(uColorOverride.rgb, aColor.a)
             : aColor;
}
";

        // ── Point sprite fragment shader ──────────────────────────────────
        // Uses gl_PointCoord (0..1 across the sprite) to draw the symbol shape
        // selected by uPointSymbol (mirrors the PointSymbol flag enum).
        public const string PointFragmentShader = @"
#version 330 core

in  vec4 vColor;

uniform int uPointSymbol;

out vec4 FragColor;

void main()
{
    vec2  uv = gl_PointCoord * 2.0 - 1.0;   // remap 0..1 to -1..1
    float r  = length(uv);

    bool inside    = false;
    int  core      = uPointSymbol & 7;
    bool hasSquare = (uPointSymbol & 16) != 0;   // 0x10
    bool hasCircle = (uPointSymbol & 32) != 0;   // 0x20
    bool isSelect  = (uPointSymbol & 64) != 0;   // 0x40

    if (isSelect)
    {
        inside = max(abs(uv.x), abs(uv.y)) <= 1.0;   // filled square
    }
    else
    {
        if      (core == 1) inside = r <= 0.3;
        else if (core == 2) inside = (abs(uv.x) <= 0.2 || abs(uv.y) <= 0.2) && r <= 1.0;
        else if (core == 3) inside = (abs(uv.x - uv.y) <= 0.3 || abs(uv.x + uv.y) <= 0.3) && r <= 1.0;
        else if (core == 4) inside = abs(uv.x) <= 0.2 && r <= 1.0;

        if (hasSquare) { float m = max(abs(uv.x), abs(uv.y)); inside = inside || (m > 0.6 && m <= 1.0); }
        if (hasCircle) inside = inside || (r > 0.6 && r <= 1.0);
    }

    if (!inside) discard;
    FragColor = vColor;
}
";

        // ── Text billboard vertex shader (screen-space quads) ─────────────
        // Vertex positions are already in NDC (-1..1); no MVP multiplication.
        public const string TextVertexShader = @"
#version 330 core

layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aUV;

out vec2 vUV;

void main()
{
    gl_Position = vec4(aPos, 0.0, 1.0);
    vUV = aUV;
}
";

        // ── Text billboard fragment shader ────────────────────────────────
        // Samples the alpha channel of a white-on-transparent glyph texture
        // and multiplies by uColor so the caller controls the label colour.
        public const string TextFragmentShader = @"
#version 330 core

in  vec2 vUV;

uniform sampler2D uTexture;
uniform vec4      uColor;

out vec4 FragColor;

void main()
{
    float a   = texture(uTexture, vUV).a;
    FragColor = vec4(uColor.rgb, uColor.a * a);
}
";
    }
}
