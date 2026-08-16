using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Nodes;
using Path = System.IO.Path;

namespace CADability.Tests.Rpc
{
    /// <summary>
    /// What the harness expects from a case. A case is usually written while the defect is still open, so it
    /// has to be able to live in the suite before it is green - that is what <see cref="KnownFail"/> is for.
    /// </summary>
    public enum RpcCaseStatus
    {
        /// <summary>Every call must succeed and every baseline must match.</summary>
        Ok,
        /// <summary>Known to be broken. Must still match its baseline; if it passes, the test says so.</summary>
        KnownFail,
        /// <summary>Deliberately excluded. Reported, never run.</summary>
        Skip
    }

    /// <summary>A call that is allowed to fail, identified by its id in RPCCalls.</summary>
    public sealed class ExpectedError
    {
        public int Id { get; set; }
        /// <summary>Optional: the expected code, e.g. "E_INVALID_PARAMS" or "-32603". Empty accepts any error.</summary>
        public string Code { get; set; } = "";
    }

    /// <summary>
    /// One RPC regression case, read from a *.json file in Files/RPC. See the readme there for the format.
    /// <para>
    /// The whole file is kept as a <see cref="JsonNode"/> so that regenerating a baseline preserves everything
    /// the harness does not know about - the prose fields (Description, Expected, Actual, ...) are what makes
    /// these files useful while a defect is open, and they must survive a rewrite untouched.
    /// </para>
    /// </summary>
    public sealed class RpcCase
    {
        public string Name { get; private set; } = "";
        public string FilePath { get; private set; } = "";
        /// <summary>The file text, re-parsed for every run: the JsonElements handed to the server stay valid
        /// only as long as their JsonDocument lives, and every run needs its own.</summary>
        public string FileText { get; private set; } = "";
        public JsonObject Root { get; private set; } = new JsonObject();

        public RpcCaseStatus Status { get; private set; } = RpcCaseStatus.Ok;
        public int Repeat { get; private set; } = 1;
        public double? RelativeTolerance { get; private set; }
        public int? TimeoutSeconds { get; private set; }
        /// <summary>Workspace names whose result is recorded. Empty means: every object committed to the model.</summary>
        public string[] Verify { get; private set; } = Array.Empty<string>();
        public List<ExpectedError> ExpectError { get; private set; } = new List<ExpectedError>();
        /// <summary>The recorded result: object name -&gt; field name -&gt; value. Empty when none is recorded yet.</summary>
        public Dictionary<string, Dictionary<string, string>> Baseline { get; private set; }
            = new Dictionary<string, Dictionary<string, string>>(StringComparer.Ordinal);
        /// <summary>The "verified" note per object, kept out of the comparison and carried over on regenerate.</summary>
        public Dictionary<string, string> VerifiedNotes { get; private set; }
            = new Dictionary<string, string>(StringComparer.Ordinal);

        public int CallCount { get; private set; }
        /// <summary>Why this file cannot be used. A file the harness cannot understand is reported as a
        /// failure, never silently skipped - a case that quietly does nothing is worse than no case at all.</summary>
        public List<string> Problems { get; } = new List<string>();
        public bool IsRunnable => Problems.Count == 0;

        /// <summary>The key under which the manually confirmed note is stored inside a baseline entry.</summary>
        public const string VerifiedKey = "verified";

        private static readonly JsonDocumentOptions ParseOptions = new JsonDocumentOptions
        {
            CommentHandling = JsonCommentHandling.Skip,
            AllowTrailingCommas = true
        };

        public static readonly JsonSerializerOptions WriteOptions = new JsonSerializerOptions
        {
            WriteIndented = true,
            Encoder = System.Text.Encodings.Web.JavaScriptEncoder.UnsafeRelaxedJsonEscaping
        };

        public static RpcCase Read(string path)
        {
            RpcCase result = new RpcCase
            {
                FilePath = path,
                Name = Path.GetFileNameWithoutExtension(path)
            };
            try { result.FileText = File.ReadAllText(path); }
            catch (Exception e) { result.Problems.Add("cannot be read: " + e.Message); return result; }

            JsonNode? node;
            try { node = JsonNode.Parse(result.FileText, null, ParseOptions); }
            catch (Exception e) { result.Problems.Add("is not valid JSON: " + e.Message); return result; }
            if (node is not JsonObject root) { result.Problems.Add("is not a JSON object"); return result; }
            result.Root = root;

            if (root["RPCCalls"] is not JsonArray calls) { result.Problems.Add("has no array \"RPCCalls\""); return result; }
            result.CallCount = calls.Count;
            if (calls.Count == 0) result.Problems.Add("has an empty \"RPCCalls\" array");
            for (int i = 0; i < calls.Count; i++)
            {
                if (calls[i] is not JsonObject call)
                {
                    result.Problems.Add($"RPCCalls[{i}] is not an object");
                    continue;
                }
                if (call["method"] is null) result.Problems.Add($"RPCCalls[{i}] has no \"method\"");
                // Templates need MCPServer.ProcessMethod(JsonElement), which records the calls as a side effect
                // but discards the response - and the response is what tells the harness whether a call failed.
                string? method = call["method"]?.GetValue<string>();
                if (method != null && method.StartsWith("template.", StringComparison.OrdinalIgnoreCase))
                    result.Problems.Add($"RPCCalls[{i}] uses \"{method}\"; template recording is not supported by the harness yet");
            }

            result.Status = ReadStatus(root, result.Problems);
            result.Repeat = Math.Max(1, ReadInt(root, "Repeat", 1));
            if (root["RelativeTolerance"] is JsonValue tol && tol.TryGetValue(out double tolValue)) result.RelativeTolerance = tolValue;
            if (root["TimeoutSeconds"] is JsonValue tos && tos.TryGetValue(out int tosValue)) result.TimeoutSeconds = tosValue;
            result.Verify = ReadStringArray(root, "Verify");
            result.ExpectError = ReadExpectedErrors(root, result.Problems);
            ReadBaseline(root, result);
            return result;
        }

        /// <summary>
        /// The name is deliberately not "Status": the case files use that one for free prose - a whole history
        /// of a defect and its fixes - and a machine read enum next to it would be a trap, exactly like an
        /// "Expect" next to the prose "Expected" would be.
        /// </summary>
        public const string StatusKey = "CaseStatus";

        private static RpcCaseStatus ReadStatus(JsonObject root, List<string> problems)
        {
            string? text = root[StatusKey]?.GetValue<string>();
            if (text == null) return RpcCaseStatus.Ok;
            if (Enum.TryParse(text, true, out RpcCaseStatus status)) return status;
            problems.Add($"has an unknown \"{StatusKey}\": {text}");
            return RpcCaseStatus.Ok;
        }

        private static int ReadInt(JsonObject root, string key, int fallback)
            => root[key] is JsonValue value && value.TryGetValue(out int number) ? number : fallback;

        private static string[] ReadStringArray(JsonObject root, string key)
        {
            if (root[key] is not JsonArray array) return Array.Empty<string>();
            return array.Where(n => n != null).Select(n => n!.GetValue<string>()).ToArray();
        }

        private static List<ExpectedError> ReadExpectedErrors(JsonObject root, List<string> problems)
        {
            List<ExpectedError> result = new List<ExpectedError>();
            if (root["ExpectError"] is not JsonArray array) return result;
            foreach (JsonNode? entry in array)
            {
                if (entry is not JsonObject obj || obj["id"] is not JsonValue idValue || !idValue.TryGetValue(out int id))
                {
                    problems.Add("has an \"ExpectError\" entry without a numeric \"id\"");
                    continue;
                }
                result.Add(new ExpectedError { Id = id, Code = obj["code"]?.GetValue<string>() ?? "" });
            }
            return result;
        }

        private static void ReadBaseline(JsonObject root, RpcCase result)
        {
            if (root["Baseline"] is not JsonObject baseline) return;
            foreach (KeyValuePair<string, JsonNode?> item in baseline)
            {
                if (item.Value is not JsonObject fields)
                {
                    result.Problems.Add($"has a \"Baseline\" entry \"{item.Key}\" that is not an object");
                    continue;
                }
                Dictionary<string, string> values = new Dictionary<string, string>(StringComparer.Ordinal);
                foreach (KeyValuePair<string, JsonNode?> field in fields)
                {
                    string value = field.Value?.ToString() ?? "";
                    if (string.Equals(field.Key, VerifiedKey, StringComparison.OrdinalIgnoreCase))
                        result.VerifiedNotes[item.Key] = value;
                    else
                        values[field.Key] = value;
                }
                result.Baseline[item.Key] = values;
            }
        }

        public bool IsVerified(string objectName) => VerifiedNotes.ContainsKey(objectName);

        /// <summary>Parses the file freshly for one run. The document must stay alive while the calls run.</summary>
        public JsonDocument OpenCallDocument() => JsonDocument.Parse(FileText, ParseOptions);
    }
}
