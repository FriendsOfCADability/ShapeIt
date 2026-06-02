namespace CADability.Avalonia
{
    internal static class ShaderSources
    {
        public const string VertexShader = @"
#version 330 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aColor;

uniform mat4 uMVP;
uniform mat4 uModel;

out vec3 vFragPos;
out vec3 vNormal;
out vec4 vColor;

void main()
{
    vec4 worldPos = uModel * vec4(aPosition, 1.0);
    vFragPos  = worldPos.xyz;
    vNormal   = mat3(transpose(inverse(uModel))) * aNormal;
    vColor    = aColor;
    gl_Position = uMVP * vec4(aPosition, 1.0);
}
";

        public const string LitFragmentShader = @"
#version 330 core

in  vec3 vFragPos;
in  vec3 vNormal;
in  vec4 vColor;

uniform vec3 uLightDir;
uniform vec3 uLightColor;
uniform vec3 uAmbient;
uniform vec4 uColorOverride;

out vec4 FragColor;

void main()
{
    vec4 baseColor = (uColorOverride.w > 0.5)
                     ? vec4(uColorOverride.rgb, vColor.a)
                     : vColor;

    vec3  n       = normalize(vNormal);
    float diff    = max(abs(dot(n, normalize(uLightDir))), 0.0);
    vec3  diffuse = diff * uLightColor;

    vec3  viewDir  = vec3(0.0, 0.0, 1.0);
    vec3  halfDir  = normalize(normalize(uLightDir) + viewDir);
    float spec     = pow(max(abs(dot(n, halfDir)), 0.0), 16.0);
    vec3  specular = spec * uLightColor * 0.3;

    vec3 result = (uAmbient + diffuse + specular) * baseColor.rgb;
    FragColor   = vec4(result, baseColor.a);
}
";

        public const string UnlitFragmentShader = @"
#version 330 core

in  vec4 vColor;

uniform vec4 uColorOverride;

out vec4 FragColor;

void main()
{
    FragColor = (uColorOverride.w > 0.5)
                ? vec4(uColorOverride.rgb, vColor.a)
                : vColor;
}
";

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

        public const string PointFragmentShader = @"
#version 330 core

in  vec4 vColor;

uniform int uPointSymbol;

out vec4 FragColor;

void main()
{
    vec2  uv = gl_PointCoord * 2.0 - 1.0;
    float r  = length(uv);

    bool inside    = false;
    int  core      = uPointSymbol & 7;
    bool hasSquare = (uPointSymbol & 16) != 0;
    bool hasCircle = (uPointSymbol & 32) != 0;
    bool isSelect  = (uPointSymbol & 64) != 0;

    if (isSelect)
    {
        inside = max(abs(uv.x), abs(uv.y)) <= 1.0;
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

        // Textured quad in 3-D world space (RectangularBitmap).
        public const string TextureVertexShader = @"
#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUV;

uniform mat4 uMVP;

out vec2 vUV;

void main()
{
    gl_Position = uMVP * vec4(aPos, 1.0);
    vUV = aUV;
}
";

        public const string TextureFragmentShader = @"
#version 330 core

in  vec2 vUV;

uniform sampler2D uTexture;

out vec4 FragColor;

void main()
{
    vec4 c = texture(uTexture, vUV);
    if (c.a < 0.5) discard;   // match the old GL_ALPHA_TEST > 0.5 behaviour
    FragColor = c;
}
";
    }
}
