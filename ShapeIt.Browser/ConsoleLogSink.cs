using System;
using System.Text;
using Avalonia.Logging;

namespace ShapeIt.Browser
{
    /// <summary>
    /// Minimal Avalonia log sink that forwards messages to the browser console
    /// (Console.WriteLine maps to console.log in the .NET wasm runtime). Used during
    /// the web spike to surface OpenGL / rendering errors that Avalonia otherwise
    /// only writes to its internal logger.
    /// </summary>
    internal sealed class ConsoleLogSink : ILogSink
    {
        private readonly LogEventLevel _min;
        public ConsoleLogSink(LogEventLevel min) => _min = min;

        public bool IsEnabled(LogEventLevel level, string area) => level >= _min;

        public void Log(LogEventLevel level, string area, object? source, string messageTemplate)
            => Console.WriteLine($"[Avalonia/{level}/{area}] {messageTemplate}");

        public void Log(LogEventLevel level, string area, object? source, string messageTemplate, params object?[] values)
        {
            var sb = new StringBuilder(messageTemplate);
            for (int i = 0; i < values.Length; i++)
                sb.Append("  {").Append(i).Append("}=").Append(values[i]);
            Console.WriteLine($"[Avalonia/{level}/{area}] {sb}");
        }
    }
}
