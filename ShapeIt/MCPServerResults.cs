// Central result envelope for MCP tool calls.
//
// Most tool implementations (*Impl methods) only store their results in the namedItems
// dictionary and do not build an explicit JSON result. Before this mechanism existed the
// client received an empty object "{}" for almost every call. Now ProcessMethod records all
// changes to namedItems made during a call and appends them to the result:
//
//   {
//     "created":  [ { "name": "...", "type": "...", "summary": { ... } }, ... ],
//     "modified": [ ... same shape ... ],
//     "removed":  [ "name1", ... ],
//     "warnings": [ "..." ]
//   }
//
// Tools with their own output schema (inspect.*, rpc.batch, ...) keep their custom fields;
// the envelope properties are merged into the same result object. Nested calls (rpc.batch,
// template execution) each get their own envelope because ProcessMethod maintains a stack.
using CADability;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Nodes;

namespace ShapeIt
{
    public partial class MCPServer
    {
        /// <summary>
        /// Collects the named-item changes made during a single tool call.
        /// </summary>
        private sealed class CallChanges
        {
            public readonly Dictionary<string, object> Created = new(StringComparer.Ordinal);
            public readonly Dictionary<string, object> Modified = new(StringComparer.Ordinal);
            public readonly List<string> Removed = new();
            public readonly List<string> Warnings = new();
        }

        // One entry per nested ProcessMethod call; namedItems notifications are recorded in the
        // topmost entry only, so e.g. the calls inside rpc.batch report their changes themselves.
        private readonly Stack<CallChanges> callChangesStack = new();

        /// <summary>
        /// Subscribes the given dictionary to the change tracking of this server. Must be called
        /// for every NamedItemsDictionary instance the server works with (initial and clones).
        /// </summary>
        internal void AttachChangeTracking(NamedItemsDictionary items)
        {
            items.ItemSet = OnNamedItemSet;
            items.ItemRemoved = OnNamedItemRemoved;
        }

        // Monotonic counter incremented on every workspace change; reported by document.get_state
        // so clients can detect whether the workspace changed since they last looked.
        private int stateVersion;

        private void OnNamedItemSet(string name, object value, bool existedBefore)
        {
            stateVersion++;
            if (callChangesStack.Count == 0) return;
            CallChanges changes = callChangesStack.Peek();
            changes.Removed.Remove(name);
            if (changes.Created.ContainsKey(name))
            {   // still counts as created within this call, keep the latest value
                changes.Created[name] = value;
                return;
            }
            if (existedBefore) changes.Modified[name] = value;
            else changes.Created[name] = value;
        }

        /// <summary>
        /// Records that the item stored under <paramref name="name"/> was modified in place, i.e.
        /// without a write to namedItems. The transform tools mutate their input objects directly
        /// when no 'name' is given; without this the call would report an empty result and the
        /// missing-result check could not tell a successful transform from a silent no-op.
        /// </summary>
        internal void NoteModifiedInPlace(string name)
        {
            if (!namedItems.TryGetValue(name, out object? value) || value == null) return;
            stateVersion++;
            if (callChangesStack.Count == 0) return;
            CallChanges changes = callChangesStack.Peek();
            if (changes.Created.ContainsKey(name))
            {   // created earlier in this same call, so it stays a creation
                changes.Created[name] = value;
                return;
            }
            changes.Modified[name] = value;
        }

        private void OnNamedItemRemoved(string name)
        {
            stateVersion++;
            if (callChangesStack.Count == 0) return;
            CallChanges changes = callChangesStack.Peek();
            // an item that was created and removed within the same call (e.g. the temporary
            // "this" item used for expression evaluation) has no net effect
            if (changes.Created.Remove(name)) return;
            changes.Modified.Remove(name);
            if (!changes.Removed.Contains(name)) changes.Removed.Add(name);
        }

        /// <summary>
        /// Builds the JSON-RPC error object. Warnings collected before the failure are attached, so
        /// a hint such as "this batch is not rolled back on its own" is not lost exactly when the
        /// call fails - which is when it matters most.
        /// </summary>
        private static JsonObject MakeErrorObject(int code, string message, CallChanges changes)
        {
            JsonObject error = new JsonObject { ["code"] = code, ["message"] = message };
            if (changes.Warnings.Count > 0)
            {
                var warnings = new JsonArray();
                foreach (string warning in changes.Warnings) warnings.Add(warning);
                error["data"] = new JsonObject { ["warnings"] = warnings };
            }
            return error;
        }

        /// <summary>
        /// Adds a warning to the result envelope of the current tool call. No-op when called
        /// outside a tool call; identical messages are only reported once.
        /// </summary>
        private void AddCallWarning(string message)
        {
            if (callChangesStack.Count == 0) return;
            List<string> warnings = callChangesStack.Peek().Warnings;
            if (!warnings.Contains(message)) warnings.Add(message);
        }

        /// <summary>
        /// Damerau-Levenshtein distance (optimal string alignment), case-insensitive. Used for
        /// "did you mean" suggestions; transpositions count as one edit because swapped letters
        /// are the most common typo.
        /// </summary>
        private static int EditDistance(string a, string b)
        {
            a = a.ToLowerInvariant();
            b = b.ToLowerInvariant();
            int[,] d = new int[a.Length + 1, b.Length + 1];
            for (int i = 0; i <= a.Length; i++) d[i, 0] = i;
            for (int j = 0; j <= b.Length; j++) d[0, j] = j;
            for (int i = 1; i <= a.Length; i++)
            {
                for (int j = 1; j <= b.Length; j++)
                {
                    int cost = a[i - 1] == b[j - 1] ? 0 : 1;
                    d[i, j] = Math.Min(Math.Min(d[i - 1, j] + 1, d[i, j - 1] + 1), d[i - 1, j - 1] + cost);
                    if (i > 1 && j > 1 && a[i - 1] == b[j - 2] && a[i - 2] == b[j - 1])
                        d[i, j] = Math.Min(d[i, j], d[i - 2, j - 2] + 1);
                }
            }
            return d[a.Length, b.Length];
        }

        /// <summary>
        /// Returns the candidate most similar to the given (unknown) name, or null when none is
        /// plausibly a typo of it. Short names get a stricter threshold so we never suggest
        /// nonsense for e.g. two-letter names.
        /// </summary>
        private static string? FindSimilarName(string name, IEnumerable<string> candidates)
        {
            int maxDistance = name.Length <= 4 ? 1 : 2;
            string? best = null;
            int bestDistance = int.MaxValue;
            foreach (string candidate in candidates)
            {
                if (Math.Abs(candidate.Length - name.Length) > maxDistance) continue;
                int distance = EditDistance(name, candidate);
                if (distance < bestDistance) { bestDistance = distance; best = candidate; }
            }
            return bestDistance <= maxDistance ? best : null;
        }

        /// <summary>
        /// Warns (via the result envelope) about top-level parameters that are not part of the
        /// tool's schema. Such parameters were silently ignored before, which made typos in
        /// optional parameter names invisible. Called from the generated handlers with the
        /// literal parameter list of the tool.
        /// </summary>
        private void WarnUnknownParameters(JsonElement root, params string[] known)
        {
            if (root.ValueKind != JsonValueKind.Object) return;
            foreach (var prop in root.EnumerateObject())
            {
                string name = prop.Name;
                if (Array.IndexOf(known, name) >= 0) continue;
                if (known.Length == 0)
                {
                    AddCallWarning($"Unknown parameter '{name}' was ignored — this tool takes no parameters.");
                    continue;
                }
                string? caseMatch = null;
                foreach (string k in known)
                {
                    if (string.Equals(k, name, StringComparison.OrdinalIgnoreCase)) { caseMatch = k; break; }
                }
                if (caseMatch != null)
                {
                    AddCallWarning($"Unknown parameter '{name}' was ignored — parameter names are case-sensitive, did you mean '{caseMatch}'?");
                    continue;
                }
                string? similar = FindSimilarName(name, known);
                if (similar != null) AddCallWarning($"Unknown parameter '{name}' was ignored — did you mean '{similar}'?");
                else AddCallWarning($"Unknown parameter '{name}' was ignored. Valid parameters: {string.Join(", ", known)}.");
            }
        }

        /// <summary>
        /// Builds the exception for a workspace name that could not be resolved: suggests the
        /// most similar existing name and lists the available names (capped) so the client can
        /// re-orient without an extra round trip.
        /// </summary>
        private JsonRpcException NamedItemNotFound(string? name)
        {
            if (string.IsNullOrEmpty(name)) return new JsonRpcException("E_NOT_FOUND", "Named object not found: no name given.");
            var sb = new System.Text.StringBuilder($"Named object not found: '{name}'.");
            List<string> keys = namedItems.Keys.ToList();
            string? caseMatch = keys.FirstOrDefault(k => string.Equals(k, name, StringComparison.OrdinalIgnoreCase));
            if (caseMatch != null)
            {
                sb.Append($" Names are case-sensitive — did you mean '{caseMatch}'?");
            }
            else
            {
                string? similar = FindSimilarName(name, keys);
                if (similar != null) sb.Append($" Did you mean '{similar}'?");
            }
            if (keys.Count == 0)
            {
                sb.Append(" The workspace is empty.");
            }
            else
            {
                keys.Sort(StringComparer.Ordinal);
                const int maxListed = 20;
                sb.Append(" Available names: ").Append(string.Join(", ", keys.Take(maxListed)));
                if (keys.Count > maxListed) sb.Append($" … and {keys.Count - maxListed} more");
                sb.Append('.');
            }
            return new JsonRpcException("E_NOT_FOUND", sb.ToString());
        }

        // Tool families whose "name" parameter denotes the name under which the result is stored.
        private static readonly string[] resultNamePrefixes = { "solid.", "sketch.", "feature.", "pattern.", "surface.", "transform." };

        // Tools whose documented contract is that omitting "name" replaces or modifies the input.
        // Only for these does an empty change set prove that the call silently did nothing; the
        // remaining tools (sketch.add_*, solid.box, ...) legitimately store nothing without a name.
        private static readonly string[] inPlaceResultMethods =
            { "feature.", "transform.", "solid.boolean", "sketch.connect", "sketch.offset", "sketch.round_vertices" };

        private static bool StartsWithAny(string method, string[] prefixes)
        {
            foreach (string prefix in prefixes)
                if (method.StartsWith(prefix, StringComparison.Ordinal)) return true;
            return false;
        }

        /// <summary>
        /// Adds a warning when a creating tool produced no reachable result. With a 'name' that means
        /// nothing was stored under that name (or a suffixed variant of it); without a 'name' — where
        /// the convention is that the operation replaces or modifies its input — it means the call
        /// changed no workspace object at all. Both turn silent failures into a visible message.
        /// </summary>
        private void WarnWhenResultMissing(CallChanges changes, string method, JsonElement parameters, JsonNode? result)
        {
            if (parameters.ValueKind != JsonValueKind.Object) return;
            string? name = null;
            if (parameters.TryGetProperty("name", out JsonElement nameEl) && nameEl.ValueKind == JsonValueKind.String)
            {
                name = nameEl.GetString();
                if (string.IsNullOrEmpty(name)) name = null;
            }
            if (name == null)
            {
                if (!StartsWithAny(method, inPlaceResultMethods)) return;
                // A tool that returns its own payload reports the result itself and needs no check.
                if (result != null && (result is not JsonObject payload || payload.Count > 0)) return;
                if (changes.Created.Count > 0 || changes.Modified.Count > 0 || changes.Removed.Count > 0) return;
                changes.Warnings.Add($"'{method}' was called without a 'name', so it should have replaced or modified its input, but no workspace object changed. The operation probably yielded no result.");
                return;
            }
            if (!StartsWithAny(method, resultNamePrefixes)) return;
            if (namedItems.ContainsKey(name)) return;
            foreach (string created in changes.Created.Keys)
            {   // tools with suffix/nameWithSuffix store "name_0", "name_1", ...
                if (created.StartsWith(name, StringComparison.Ordinal)) return;
            }
            foreach (string modified in changes.Modified.Keys)
            {
                if (modified.StartsWith(name, StringComparison.Ordinal)) return;
            }
            changes.Warnings.Add($"The call completed but no workspace object was stored under the requested name '{name}'. The operation probably yielded no result.");
        }

        /// <summary>
        /// Appends the recorded changes (created/modified/removed/warnings) to the result object.
        /// Properties are only added when non-empty so unchanged tools stay compact.
        /// </summary>
        private void AppendCallChanges(JsonObject result, CallChanges changes)
        {
            if (changes.Created.Count > 0)
            {
                var created = new JsonArray();
                foreach (var item in changes.Created) created.Add(DescribeNamedItem(item.Key, item.Value));
                result["created"] = created;
            }
            if (changes.Modified.Count > 0)
            {
                var modified = new JsonArray();
                foreach (var item in changes.Modified) modified.Add(DescribeNamedItem(item.Key, item.Value));
                result["modified"] = modified;
            }
            if (changes.Removed.Count > 0)
            {
                var removed = new JsonArray();
                foreach (string name in changes.Removed) removed.Add(name);
                result["removed"] = removed;
            }
            if (changes.Warnings.Count > 0)
            {
                var warnings = new JsonArray();
                foreach (string warning in changes.Warnings) warnings.Add(warning);
                result["warnings"] = warnings;
            }
        }

        /// <summary>
        /// Builds a compact JSON description of a workspace item: name, type and — depending on
        /// the type — value, bounding box or element counts. Uses the same helpers as
        /// inspect.summary; unknown types degrade gracefully instead of failing the call.
        /// </summary>
        private static JsonObject DescribeNamedItem(string name, object item)
        {
            var description = new JsonObject { ["name"] = name };
            object unwrapped = UnwrapSingletonList(item);
            try
            {
                description["type"] = GetItemTypeName(unwrapped);
            }
            catch (Exception)
            {   // GetItemTypeName throws for types it does not know; report the CLR type instead
                switch (unwrapped)
                {
                    case string: description["type"] = "string"; break;
                    case bool: description["type"] = "boolean"; break;
                    case IList: description["type"] = "object[]"; break; // mixed-type selection result
                    default: description["type"] = unwrapped.GetType().Name; break;
                }
            }
            try
            {
                JsonObject summary;
                switch (unwrapped)
                {
                    case string s:
                        summary = new JsonObject { ["value"] = s };
                        break;
                    case bool b:
                        summary = new JsonObject { ["value"] = b };
                        break;
                    default:
                        summary = GetItemSummary(unwrapped);
                        if (summary.Count == 0 && unwrapped is ICollection coll) summary["count"] = coll.Count;
                        break;
                }
                if (summary.Count > 0) description["summary"] = summary;
            }
            catch (Exception ex)
            {   // never let a summary problem fail an otherwise successful call
                description["summaryError"] = ex.Message;
            }
            return description;
        }
    }
}
