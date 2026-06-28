using System;
using Silk.NET.OpenGL;

namespace CADability.Avalonia
{
    internal sealed class ShaderProgram : IDisposable
    {
        public uint Handle { get; private set; }
        private readonly GL _gl;

        public ShaderProgram(GL gl, string vertexSrc, string fragmentSrc)
        {
            _gl = gl;
            uint vert = Compile(gl, ShaderType.VertexShader,   vertexSrc);
            uint frag = Compile(gl, ShaderType.FragmentShader, fragmentSrc);

            Handle = gl.CreateProgram();
            gl.AttachShader(Handle, vert);
            gl.AttachShader(Handle, frag);
            gl.LinkProgram(Handle);

            gl.GetProgram(Handle, ProgramPropertyARB.LinkStatus, out int status);
            if (status == 0)
            {
                string log = gl.GetProgramInfoLog(Handle);
                gl.DeleteProgram(Handle);
                throw new Exception($"Shader link error:\n{log}");
            }

            gl.DetachShader(Handle, vert);
            gl.DetachShader(Handle, frag);
            gl.DeleteShader(vert);
            gl.DeleteShader(frag);
        }

        private static unsafe uint Compile(GL gl, ShaderType type, string src)
        {
            uint shader = gl.CreateShader(type);
            // TrimStart: #version must be the first token; raw string literals often start with \n
            byte[] bytes = System.Text.Encoding.UTF8.GetBytes(src.TrimStart().Replace("\r\n", "\n").Replace("\r", "\n"));
            int length = bytes.Length;
            fixed (byte* ptr = bytes)
            {
                byte*[] ptrs = [ptr];
                fixed (byte** pptrs = ptrs)
                    gl.ShaderSource(shader, 1, pptrs, &length);
            }
            gl.CompileShader(shader);
            gl.GetShader(shader, ShaderParameterName.CompileStatus, out int ok);
            if (ok == 0)
            {
                string log = gl.GetShaderInfoLog(shader);
                gl.DeleteShader(shader);
                throw new Exception($"{type} compile error:\n{log}");
            }
            return shader;
        }

        public void Use() => _gl.UseProgram(Handle);

        public void SetMatrix4(string name, System.Numerics.Matrix4x4 m)
        {
            int loc = _gl.GetUniformLocation(Handle, name);
            if (loc < 0) return;
            unsafe
            {
                _gl.UniformMatrix4(loc, 1, false, (float*)&m);
            }
        }

        public void SetVec2(string name, System.Numerics.Vector2 v)
        {
            int loc = _gl.GetUniformLocation(Handle, name);
            if (loc >= 0) _gl.Uniform2(loc, v.X, v.Y);
        }

        public void SetVec3(string name, System.Numerics.Vector3 v)
        {
            int loc = _gl.GetUniformLocation(Handle, name);
            if (loc >= 0) _gl.Uniform3(loc, v.X, v.Y, v.Z);
        }

        public void SetVec4(string name, System.Numerics.Vector4 v)
        {
            int loc = _gl.GetUniformLocation(Handle, name);
            if (loc >= 0) _gl.Uniform4(loc, v.X, v.Y, v.Z, v.W);
        }

        public void SetInt(string name, int v)
        {
            int loc = _gl.GetUniformLocation(Handle, name);
            if (loc >= 0) _gl.Uniform1(loc, v);
        }

        public void SetFloat(string name, float v)
        {
            int loc = _gl.GetUniformLocation(Handle, name);
            if (loc >= 0) _gl.Uniform1(loc, v);
        }

        public void Dispose()
        {
            _gl.DeleteProgram(Handle);
            Handle = 0;
        }
    }
}
