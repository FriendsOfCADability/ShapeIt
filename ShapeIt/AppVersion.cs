using System.Reflection;

namespace ShapeIt
{
    /// <summary>
    /// The ShapeIt version number, shared by all heads (WinForms, Avalonia desktop, Browser).
    /// Its single source is the hand-maintained version.txt in the repository root, which
    /// ShapeIt.Version.props turns into the assembly's InformationalVersion at build time.
    /// </summary>
    internal static class AppVersion
    {
        public const string ApplicationName = "ShapeIt";

        /// <summary>The version as written in version.txt, e.g. "0.8.9".</summary>
        public static string Current { get; } = Read();

        /// <summary>"ShapeIt 0.8.9" or "ShapeIt 0.8.9 — fileName", for window and tab titles.</summary>
        public static string Title(string? fileName = null)
            => string.IsNullOrEmpty(fileName)
                ? $"{ApplicationName} {Current}"
                : $"{ApplicationName} {Current} — {fileName}";

        /// <summary>
        /// Lets CADability write an "Application" block (name and version) into the header of
        /// every saved JSON file. Call once at startup, before anything is saved.
        /// </summary>
        public static void RegisterWithCADability()
        {
            CADability.JsonSerialize.ApplicationName = ApplicationName;
            CADability.JsonSerialize.ApplicationVersion = Current;
        }

        private static string Read()
        {
            Assembly asm = typeof(AppVersion).Assembly; // the head assembly: this file is compiled into each head
            string? v = asm.GetCustomAttribute<AssemblyInformationalVersionAttribute>()?.InformationalVersion;
            if (!string.IsNullOrEmpty(v))
            {
                int plus = v!.IndexOf('+'); // strip a "+<source revision>" suffix, should one ever be added
                return plus >= 0 ? v.Substring(0, plus) : v;
            }
            return asm.GetName().Version?.ToString() ?? "0.0.0";
        }
    }
}
