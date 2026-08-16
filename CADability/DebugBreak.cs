using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;

namespace CADability
{
    /// <summary>
    /// Central place for "break when object with hashCode N is created" debugging.
    /// <para>
    /// Edges, Faces and Vertices get a reproducible hashCode from a static counter, so the same
    /// sequence of operations always produces the same numbers. When a certain object turns out to be
    /// wrong, we want to stop in the debugger at the moment it is created. Instead of hard coding
    /// <c>if (hashCode == 29196) {}</c> and recompiling, the numbers to watch are configured at
    /// startup, e.g. from the command line ("-e:29196") or from the environment variable
    /// <see cref="EnvironmentVariable"/>.
    /// </para>
    /// <para>
    /// The <see cref="OnEdgeCreated"/>, <see cref="OnFaceCreated"/> and <see cref="OnVertexCreated"/>
    /// hooks are marked with <see cref="ConditionalAttribute"/>, so they disappear completely from a
    /// release build of CADability and cost nothing there.
    /// </para>
    /// </summary>
    public static class DebugBreak
    {
        /// <summary>
        /// Name of the environment variable which is evaluated in addition to the explicitly
        /// configured values. Same syntax as <see cref="Configure(string)"/>, e.g. "e:123,456;f:47".
        /// </summary>
        public const string EnvironmentVariable = "CADABILITY_DEBUG_BREAK";

        private static readonly HashSet<int> edges = new HashSet<int>();
        private static readonly HashSet<int> faces = new HashSet<int>();
        private static readonly HashSet<int> vertices = new HashSet<int>();
        private static readonly Dictionary<string, HashSet<int>> custom = new Dictionary<string, HashSet<int>>(StringComparer.OrdinalIgnoreCase);
        private static bool environmentRead;

        /// <summary>
        /// True, if at least one hashCode is being watched. Kept as a field, so the hooks are cheap
        /// when nothing is configured (which is the normal case).
        /// </summary>
        private static bool anyConfigured;

        /// <summary>
        /// Set to false to only write a trace line instead of actually stopping in the debugger.
        /// </summary>
        public static bool BreakIntoDebugger { get; set; } = true;

        /// <summary>
        /// Watch the specified edge hashCodes. Creation of such an edge stops in the debugger.
        /// </summary>
        public static void WatchEdges(params int[] hashCodes) => Add(edges, hashCodes);
        /// <summary>
        /// Watch the specified face hashCodes.
        /// </summary>
        public static void WatchFaces(params int[] hashCodes) => Add(faces, hashCodes);
        /// <summary>
        /// Watch the specified vertex hashCodes.
        /// </summary>
        public static void WatchVertices(params int[] hashCodes) => Add(vertices, hashCodes);
        /// <summary>
        /// Watch hashCodes of a freely named category, to be used with <see cref="Hit(string, int)"/>
        /// at an arbitrary place in the code.
        /// </summary>
        public static void Watch(string category, params int[] hashCodes)
        {
            if (!custom.TryGetValue(category, out HashSet<int> set)) custom[category] = set = new HashSet<int>();
            Add(set, hashCodes);
        }

        /// <summary>
        /// Removes all configured hashCodes (including the ones from the environment variable).
        /// </summary>
        public static void Clear()
        {
            edges.Clear();
            faces.Clear();
            vertices.Clear();
            custom.Clear();
            anyConfigured = false;
            environmentRead = true; // don't silently re-read the environment after an explicit Clear
        }

        /// <summary>
        /// Interprets the command line arguments and configures the hashCodes to watch. Recognized are
        /// "-e:&lt;list&gt;" for edges, "-f:&lt;list&gt;" for faces and "-v:&lt;list&gt;" for vertices, where
        /// &lt;list&gt; is a comma separated list of numbers and/or ranges, e.g. "-e:123,456,1000-1010".
        /// "-b:&lt;category&gt;:&lt;list&gt;" addresses a named category used with <see cref="Hit(string, int)"/>,
        /// e.g. "-b:Face.ModifySurface:406". All other arguments are ignored.
        /// </summary>
        public static void ParseCommandLine(string[] args)
        {
            if (args == null) return;
            for (int i = 0; i < args.Length; i++)
            {
                string arg = args[i];
                if (string.IsNullOrEmpty(arg) || arg.Length < 3) continue;
                if (arg[0] != '-' && arg[0] != '/') continue;
                if (arg[2] != ':' && arg[2] != '=') continue;
                if (char.ToLowerInvariant(arg[1]) == 'b')
                {
                    Configure(arg.Substring(3)); // "<category>:<list>"
                    continue;
                }
                HashSet<int> set = SetForPrefix(arg[1]);
                if (set == null) continue;
                AddList(set, arg.Substring(3));
            }
        }

        /// <summary>
        /// Configures the hashCodes to watch from a single string, e.g. "e:123,456;f:47;v:3435".
        /// The category is separated by ':' and the categories by ';' (or by whitespace).
        /// Without a category prefix the numbers are interpreted as edge hashCodes.
        /// </summary>
        public static void Configure(string spec)
        {
            if (string.IsNullOrWhiteSpace(spec)) return;
            string[] parts = spec.Split(new char[] { ';', ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            for (int i = 0; i < parts.Length; i++)
            {
                string part = parts[i];
                int colon = part.IndexOf(':');
                if (colon < 0)
                {
                    AddList(edges, part);
                }
                else
                {
                    string category = part.Substring(0, colon);
                    string list = part.Substring(colon + 1);
                    HashSet<int> set = category.Length == 1 ? SetForPrefix(category[0]) : null;
                    if (set == null)
                    {
                        if (!custom.TryGetValue(category, out set)) custom[category] = set = new HashSet<int>();
                    }
                    AddList(set, list);
                }
            }
        }

        /// <summary>
        /// Called from the Edge constructor. Stops in the debugger if this hashCode is being watched.
        /// Compiled away in a release build.
        /// </summary>
        [Conditional("DEBUG")]
        public static void OnEdgeCreated(int hashCode) => Check(edges, "Edge created", hashCode);
        /// <summary>
        /// Called from the Face constructor. Stops in the debugger if this hashCode is being watched.
        /// Compiled away in a release build.
        /// </summary>
        [Conditional("DEBUG")]
        public static void OnFaceCreated(int hashCode) => Check(faces, "Face created", hashCode);
        /// <summary>
        /// Called from the Vertex constructors. Stops in the debugger if this hashCode is being watched.
        /// Compiled away in a release build.
        /// </summary>
        [Conditional("DEBUG")]
        public static void OnVertexCreated(int hashCode) => Check(vertices, "Vertex created", hashCode);
        /// <summary>
        /// Stops in the debugger if the hashCode is watched for the given, freely chosen category.
        /// Can be placed anywhere in the code, e.g. DebugBreak.Hit("split", edge.GetHashCode()).
        /// Compiled away in a release build.
        /// </summary>
        [Conditional("DEBUG")]
        public static void Hit(string category, int hashCode)
        {
            if (!anyConfigured) EnsureEnvironment();
            if (custom.TryGetValue(category, out HashSet<int> set)) Check(set, category, hashCode);
        }

        private static HashSet<int> SetForPrefix(char prefix)
        {
            switch (char.ToLowerInvariant(prefix))
            {
                case 'e': return edges;
                case 'f': return faces;
                case 'v': return vertices;
                default: return null;
            }
        }

        private static void Add(HashSet<int> set, int[] hashCodes)
        {
            if (hashCodes == null) return;
            for (int i = 0; i < hashCodes.Length; i++) set.Add(hashCodes[i]);
            anyConfigured = anyConfigured || set.Count > 0;
        }

        /// <summary>
        /// Parses a comma separated list of numbers and ranges, e.g. "123,456,1000-1010".
        /// </summary>
        private static void AddList(HashSet<int> set, string list)
        {
            string[] items = list.Split(new char[] { ',' }, StringSplitOptions.RemoveEmptyEntries);
            for (int i = 0; i < items.Length; i++)
            {
                string item = items[i].Trim();
                int dash = item.IndexOf('-', 1); // a leading '-' would be a (nonsensical) negative number
                if (dash > 0)
                {
                    if (int.TryParse(item.Substring(0, dash), NumberStyles.Integer, CultureInfo.InvariantCulture, out int from)
                        && int.TryParse(item.Substring(dash + 1), NumberStyles.Integer, CultureInfo.InvariantCulture, out int to))
                    {
                        for (int hc = from; hc <= to; hc++) set.Add(hc);
                    }
                }
                else if (int.TryParse(item, NumberStyles.Integer, CultureInfo.InvariantCulture, out int single))
                {
                    set.Add(single);
                }
            }
            anyConfigured = anyConfigured || set.Count > 0;
        }

        private static void EnsureEnvironment()
        {
            if (environmentRead) return;
            environmentRead = true;
            try
            {
                Configure(Environment.GetEnvironmentVariable(EnvironmentVariable));
            }
            catch { } // reading the environment may fail in restricted hosts, that must not break anything
        }

        private static void Check(HashSet<int> set, string what, int hashCode)
        {
            if (!anyConfigured)
            {
                EnsureEnvironment();
                if (!anyConfigured) return;
            }
            if (!set.Contains(hashCode)) return;
            Trace.WriteLine("DebugBreak: " + what + ", hashCode " + hashCode.ToString(CultureInfo.InvariantCulture));
            if (BreakIntoDebugger && Debugger.IsAttached) Debugger.Break(); // step out once to see the caller
        }
    }
}
