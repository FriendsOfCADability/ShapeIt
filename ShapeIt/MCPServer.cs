// Combined from MCPServer.cs and MCPServerDispatcher.cs.
using CADability;
using CADability.Attribute;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.Substitutes;
using CdlToCSharp;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Reflection.Metadata.Ecma335;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading.Tasks;
using System.Xml.Linq;
using static ShapeIt.ShellExtensions;
using static System.Runtime.InteropServices.JavaScript.JSType;
using Plane = CADability.Plane;
using MathNet.Numerics.LinearAlgebra.Factorization;

namespace ShapeIt
{
    public partial class MCPServer
    {
        #region State, construction and named-item storage

        public IFrame frame;

        public readonly Project project;

        public class NamedItemsDictionary : IJsonSerialize
        {
            private readonly Dictionary<string, object> dict = new(StringComparer.Ordinal);
            public NamedItemsDictionary() { }
            public NamedItemsDictionary(NamedItemsDictionary other)
            {
                foreach (var item in other.dict)
                {
                    dict[item.Key] = item.Value;
                }
            }

            public IEnumerable<string> Keys => dict.Keys;
            public IEnumerable<object> Values => dict.Values;
            public IEnumerable<KeyValuePair<string, object>> Items => dict;

            public bool ContainsKey(string key) => dict.ContainsKey(key);
            public IEnumerator<KeyValuePair<string, object>> GetEnumerator() => dict.GetEnumerator();

            public Dictionary<string, object> Dict => dict;

            // Change notifications used by the MCPServer to build the result envelope of a tool call.
            // ItemSet is called after a value has been stored via the indexer (existedBefore tells whether
            // the name was already present), ItemRemoved after a name has been removed.
            internal Action<string, object, bool>? ItemSet;
            internal Action<string>? ItemRemoved;

            public object this[string key]
            {
                get => dict[key];
                set
                {
                    bool existedBefore = dict.ContainsKey(key);
                    dict[key] = value;
                    // Keep the document-side name of solids in sync with the workspace name.
                    // Operations like solid.boolean or workspace.select store their result as a
                    // list even when it contains a single solid, so lists are named here too:
                    // one solid gets the plain key, several get an index suffix following the
                    // "name_0", "name_1" convention of the suffix-generating tools. Document
                    // names need neither be unique nor non-null, so this is purely additive -
                    // but document.update_objects matches by name, and listing tools benefit
                    // from meaningful names.
                    if (value is Solid sld) sld.Name = key;
                    else if (value is IReadOnlyList<Solid> solids)
                    {
                        if (solids.Count == 1)
                        {
                            if (solids[0] != null) solids[0].Name = key;
                        }
                        else
                        {
                            for (int i = 0; i < solids.Count; i++)
                            {
                                if (solids[i] != null) solids[i].Name = $"{key}_{i}";
                            }
                        }
                    }
                    ItemSet?.Invoke(key, value, existedBefore);
                }
            }
            public bool TryGetValue(string key, out object? value) => dict.TryGetValue(key, out value);

            internal void Remove(string name)
            {
                if (dict.Remove(name)) ItemRemoved?.Invoke(name);
            }

            public void GetObjectData(IJsonWriteData data)
            {
                foreach (var item in dict) // it is currently not possible to save a List<Solid> as part of a dictionary, so we save the items expicitely
                {
                    data.AddProperty(item.Key, item.Value);
                }
            }
            public void SetObjectData(IJsonReadData data)
            {
                foreach (var item in data)
                {
                    if (item.Key.StartsWith("$") || item.Key.StartsWith("§")) continue; // skip internal properties
                    object val = item.Value; // convert to the correct typed List<>
                    if (val is List<object> lo) dict[item.Key] = MCPServer.MakeTypedList(lo);
                    else dict[item.Key] = item.Value;
                }
            }
        }

        // Named workspace items and created objects.
        // Names are chosen by the caller (LLM/client). 
        public NamedItemsDictionary namedItems = new();

        public Dictionary<string, List<JsonElement>> templates = [];

        private class NamedItemOverride : IDisposable
        {
            private object? oldNamedItem;
            private string name;
            NamedItemsDictionary namedItems;
            public NamedItemOverride(NamedItemsDictionary namedItems, object temp, string name = "this")
            {
                this.namedItems = namedItems;
                this.name = name;
                if (!namedItems.TryGetValue(name, out oldNamedItem)) oldNamedItem = null;
                object? wrappedItem = wrapForEvaluator(temp);
                if (wrappedItem != null) namedItems[name] = wrappedItem;
            }
            public void Dispose()
            {
                if (oldNamedItem == null) namedItems.Remove(name);
                else namedItems[name] = oldNamedItem;
            }
        }

        private class NamedItemClone : IDisposable
        {
            NamedItemsDictionary namedItems;
            MCPServer server;
            public NamedItemClone(MCPServer server)
            {
                this.server = server;
                namedItems = server.namedItems;
                server.namedItems = new NamedItemsDictionary(namedItems);
                server.AttachChangeTracking(server.namedItems);
                // this is a flat copy, so in theory we could change the values. But I cannot think of a way where values are changed
                // typically they are overwritten (in the new dictionary) with new values, which is not a problem here
            }

            public void Dispose()
            {
                server.namedItems = namedItems;
            }
        }

        private Stack<NamedItemClone> namedItemClones = new();

        private int nextId = 1;

        private int nextUndo = 1;

        public MCPServer(IFrame frame, Project project)
        {
            this.frame = frame;
            this.project = project;
            // we store the named items in the project user data, so we can save the session and proceed with executing RPC Code, which relies on the existing named items.
            // CADability has no concept of MCPServer
            if (false) // saving a string:object dictionary is not working yet //if (project.UserData.ContainsData("MCPServer.NamedItems"))
            {
                namedItems = new NamedItemsDictionary();
            }
            else
            {
                namedItems = new NamedItemsDictionary();
                project.UserData.Add("MCPServer.NamedItems", namedItems.Dict);
            }
            AttachChangeTracking(namedItems);
        }

        #endregion

        #region RPC processing and errors

        private List<JsonElement>? recordingTemplate = null;

        private string? currentTemplatName = null;

        public string currentRpcString;

        public bool stopExecution = false;

        /// <summary>
        /// Dispatches a JSON-RPC method call. The transport layer should parse JSON-RPC envelope and pass:
        /// - method: the method name
        /// - id: JSON-RPC id (already parsed)
        /// - parameters: the "params" object as JsonElement (may be undefined / null in the JSON)
        /// The return value is a JSON-RPC response string.
        /// </summary>
        // Maps MCP-facing tool names ("solid_box") back to JSON-RPC method names ("solid.box").
        // MCP tool names may not contain dots (see MCPHttpServer.BuildToolsList), so clients
        // call the underscore variant; internally everything keeps using the dotted names.
        private static Dictionary<string, string>? mcpToolNameToMethod;

        private static string NormalizeMethodName(string method)
        {
            if (method.IndexOf('.') >= 0) return method; // already a dotted RPC name
            if (mcpToolNameToMethod == null)
            {
                var map = new Dictionary<string, string>(StringComparer.Ordinal);
                var assembly = System.Reflection.Assembly.GetExecutingAssembly();
                using var stream = assembly.GetManifestResourceStream("ShapeIt.McpToolsetDefinition.json");
                if (stream != null)
                {
                    using var doc = JsonDocument.Parse(stream);
                    if (doc.RootElement.TryGetProperty("tools", out JsonElement tools) && tools.ValueKind == JsonValueKind.Array)
                    {
                        foreach (var tool in tools.EnumerateArray())
                        {
                            if (tool.TryGetProperty("name", out JsonElement nameEl) && nameEl.ValueKind == JsonValueKind.String)
                            {
                                string name = nameEl.GetString()!;
                                map[name.Replace('.', '_')] = name;
                            }
                        }
                    }
                }
                mcpToolNameToMethod = map;
            }
            return mcpToolNameToMethod.TryGetValue(method, out string? rpcName) ? rpcName : method;
        }

        public string ProcessMethod(string method, int id, JsonElement parameters)
        {
            method = NormalizeMethodName(method);
            // Only the outermost call goes into the protocol: the calls inside an rpc.batch or a
            // template are already part of that request and of its response.
            bool logToProtocol = rpcNestingDepth == 0;
            long startTimestamp = Stopwatch.GetTimestamp();
            if (logToProtocol) LogRpcRequest(method, id, parameters);
            rpcNestingDepth++;
            var response = new JsonObject
            {
                ["jsonrpc"] = "2.0",
                ["id"] = id
            };

            // Track all named-item changes made during this call so the client gets a meaningful
            // result (created/modified/removed items) even for tools whose implementation does not
            // build an explicit result object. Declared outside the try so that warnings collected
            // before a failure can still be attached to the error.
            CallChanges changes = new CallChanges();
            try
            {
                System.Diagnostics.Trace.WriteLine($"RPC: {method}");
                callChangesStack.Push(changes);
                JsonNode result;
                try
                {
                    result = DispatchGenerated(method, parameters);
                }
                finally
                {
                    callChangesStack.Pop();
                }
                WarnWhenResultMissing(changes, method, parameters, result);
                if (result is JsonObject resultObj)
                {
                    AppendCallChanges(resultObj, changes);
                    response["result"] = resultObj;
                }
                else if (result == null)
                {
                    var envelope = new JsonObject();
                    AppendCallChanges(envelope, changes);
                    response["result"] = envelope;
                }
                else
                {
                    response["result"] = result;
                }
            }
            catch (JsonRpcException jre)
            {
                response["error"] = MakeErrorObject(jre.Code, jre.Message, changes);
                if (!ReportError(jre.Message)) stopExecution = true;
            }
            catch (NotImplementedException nie)
            {
                // Explicit marker that the dispatcher knows the method but implementation isn't done yet.
                response["error"] = MakeErrorObject(-32601, nie.Message, changes);
                if (!ReportError(nie.Message)) stopExecution = true;
            }
            catch (Exception ex)
            {
                response["error"] = MakeErrorObject(-32603, ex.Message, changes);
                if (!ReportError(ex.Message)) stopExecution = true;
            }
            finally
            {
                rpcNestingDepth--;
            }

            string responseJson = response.ToJsonString();
            if (logToProtocol) LogRpcResponse(method, id, responseJson, (long)Stopwatch.GetElapsedTime(startTimestamp).TotalMilliseconds);
            return responseJson;
        }

        public void ProcessMethod(JsonElement root, bool executeTemplate = false)
        {
            string? method = null;
            int? id = null;
            JsonElement @params = default;
            currentRpcString = root.GetRawText(); // for error reporting, keep the original JSON string of the current method call

            if (root.TryGetProperty("method", out var m) && m.ValueKind == JsonValueKind.String)
            {
                method = m.GetString();
                // accept the MCP-facing underscore names here too, so the template.begin/commit
                // handling below works regardless of which spelling the client used
                if (method != null) method = NormalizeMethodName(method);
            }
            if (root.TryGetProperty("id", out var idEl))
            {
                if (idEl.ValueKind == JsonValueKind.Number) id = idEl.GetInt32();
                else if (idEl.ValueKind == JsonValueKind.Null) id = null;
            }

            if (root.TryGetProperty("params", out var p))
            {
                @params = p;         // JsonElement ist ein struct, aber Achtung: doc muss leben!
            }

            if (method != null)
            {
                ProcessMethod(method, id ?? 0, @params);
                if (method == "template.begin" && !executeTemplate)
                {
                    currentTemplatName = RequireString(@params, "name");
                    recordingTemplate = [root.Clone()];
                }
                else if (method == "template.commit" && !executeTemplate)
                {
                    if (recordingTemplate == null || currentTemplatName == null) throw new JsonRpcException("E_INVALID_METHOD", "'template.commit' was called with no 'template.begin' beeing called before.");
                    recordingTemplate.Add(root.Clone());
                    templates[currentTemplatName] = recordingTemplate;
                    recordingTemplate = null;
                    currentTemplatName = null;
                }
                else if (recordingTemplate != null)
                {
                    recordingTemplate.Add(root.Clone());
                }
            }

        }

        /// <summary>
        /// Return false, when further processing of RPC code should be canceled
        /// </summary>
        /// <param name="message"></param>
        /// <returns></returns>
        // When true (set by MCPHttpServer during HTTP calls), errors are returned in the
        // JSON-RPC response instead of showing a modal dialog.
        public bool SuppressDialogs { get; set; } = false;

        public bool ReportError(string message)
        {
            if (SuppressDialogs) return true;
            return frame.UIService.ShowMessageBox(currentRpcString + "\n" + message, "Error in MCPServer", CADability.Substitutes.MessageBoxButtons.OKCancel) == CADability.Substitutes.DialogResult.OK;
        }

        // ObjectRef: { "name": "..." } or { "id": "..." }

        // Selector : { "target": "..." }, { "name": "..." }, { "id": "..." }, {names: ["name": "n1", "id": "id1"]} }, {"query": "..."}, {"op": "..." }

        private static IEnumerable<string> ReadJsonObjects(string text)
        {
            var sb = new StringBuilder();

            int braceDepth = 0;
            bool inString = false;
            bool escape = false;

            foreach (char c in text)
            {
                sb.Append(c);

                if (escape)
                {
                    escape = false;
                    continue;
                }

                if (c == '\\')
                {
                    escape = true;
                    continue;
                }

                if (c == '"')
                {
                    inString = !inString;
                    continue;
                }

                if (!inString)
                {
                    if (c == '{')
                    {
                        braceDepth++;
                    }
                    else if (c == '}')
                    {
                        braceDepth--;

                        if (braceDepth == 0)
                        {
                            yield return sb.ToString();
                            sb.Clear();
                        }
                    }
                }
            }
        }

        public void ProcessText(string text)
        {
            foreach (var jsonBlock in ReadJsonObjects(text))
            {
                if (!TryParseRpcBlock(jsonBlock)) break;
            }
        }

        bool TryParseRpcBlock(string json)
        {
            if (string.IsNullOrWhiteSpace(json)) { return false; }
            try
            {
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;
                ProcessMethod(root);
                return !stopExecution;
            }
            catch (Exception ex) { return false; } // TODO: this exception must be integrated in the error result
        }

        /// <summary>
        /// Lightweight JSON-RPC exception used to return proper JSON-RPC error objects.
        /// </summary>
        internal sealed class JsonRpcException : Exception
        {
            public int Code { get; }
            public string CodeString;
            public JsonNode? Data { get; }

            public JsonRpcException(int code, string message, JsonNode? data = null) : base(message)
            {
                Code = code;
                CodeString = "E_UNKNOWN";
                Data = data;
            }
            public JsonRpcException(string errorCode, string message, JsonNode? data = null) : base(message)
            {
                if (!MCPServer.ErrorNumbers.TryGetValue(errorCode, out int code)) code = 9999;
                CodeString = errorCode;
                Code = code;
                Data = data;
            }
        }

        #endregion

        #region JSON parameter parsing

        // -------------------------
        // JSON helpers
        // -------------------------

        private BoundingBox ReadBoundingBox(JsonElement pointEl)
        {
            double xmin = RequireDouble(pointEl, "xmin");
            double ymin = RequireDouble(pointEl, "ymin");
            double zmin = RequireDouble(pointEl, "zmin");
            double xmax = RequireDouble(pointEl, "xmax");
            double ymax = RequireDouble(pointEl, "ymax");
            double zmax = RequireDouble(pointEl, "zmax");
            return new BoundingBox(xmin, xmax, ymin, ymax, zmin, zmax);
        }

        private static void AssertIsObject(JsonElement root)
        {
            if (root.ValueKind != JsonValueKind.Object)
                throw new JsonRpcException(-32602, $"Object expected");
        }

        private T RequireObjectRef<T>(JsonElement obj, string propName) where T : class
        {
            JsonElement el;
            if (propName != null) el = RequireProperty(obj, propName);
            else el = obj;
            var resolved = ResolveObjectRef(el);
            if (resolved is T t) return t;

            throw new JsonRpcException(1001, $"Object is not a {typeof(T).Name}: {propName}={el}");
        }

        private static JsonElement RequireProperty(JsonElement obj, string prop)
        {
            if (!obj.TryGetProperty(prop, out var el))
                throw new JsonRpcException(-32602, $"Invalid params: missing '{prop}'");
            return el;
        }

        private static string RequireString(JsonElement obj, string prop)
        {
            JsonElement el = obj;
            if (prop != null) el = RequireProperty(obj, prop);
            if (el.ValueKind != JsonValueKind.String) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be string");
            return el.GetString() ?? throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be string");
        }

        /// <summary>
        /// Accepts the strings "true" and "false" as boolean literals. Clients - LLMs in particular -
        /// sometimes send a stringified boolean. The expression evaluator does not know these
        /// literals, so without this the caller would get a puzzling "Unknown name 'false'" error.
        /// </summary>
        private static bool? BoolFromString(string? text)
        {
            string trimmed = text?.Trim() ?? "";
            if (string.Equals(trimmed, "true", StringComparison.OrdinalIgnoreCase)) return true;
            if (string.Equals(trimmed, "false", StringComparison.OrdinalIgnoreCase)) return false;
            return null;
        }

        private bool GetOptionalBool(JsonElement obj, string? prop, bool defaultValue)
        {
            JsonElement el = obj;
            if (prop != null) if (!obj.TryGetProperty(prop, out el)) return defaultValue;
            if (el.ValueKind == JsonValueKind.True) return true;
            if (el.ValueKind == JsonValueKind.False) return false;
            string? exprStr = null;
            if (el.ValueKind == JsonValueKind.Object && el.TryGetProperty("expr", out JsonElement expr) && expr.ValueKind == JsonValueKind.String)
            {
                exprStr = expr.GetString();
            }
            if (el.ValueKind == JsonValueKind.String) exprStr = el.GetString();
            if (exprStr != null)
            {
                bool? literal = BoolFromString(exprStr);
                if (literal.HasValue) return literal.Value;
                try
                {
                    object res = Evaluator.Evaluate(exprStr, namedItems.Dict);
                    if (res is bool b) return b;
                }
                catch (Exception ex) // exception of Evaluator could be more descriptive
                {
                    throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{exprStr}': {ex.Message}");
                }
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be boolean");
        }

        private bool RequireBool(JsonElement obj, string prop)
        {
            JsonElement el = obj;
            if (prop != null) if (!obj.TryGetProperty(prop, out el)) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be boolean");
            if (el.ValueKind == JsonValueKind.True) return true;
            if (el.ValueKind == JsonValueKind.False) return false;
            string? exprStr = null;
            if (el.ValueKind == JsonValueKind.String)
            {
                exprStr = el.GetString();
            }
            if (el.ValueKind == JsonValueKind.Object && el.TryGetProperty("expr", out JsonElement expr) && expr.ValueKind == JsonValueKind.String)
            {
                exprStr = expr.GetString();
            }
            if (exprStr != null)
            {
                bool? literal = BoolFromString(exprStr);
                if (literal.HasValue) return literal.Value;
                try
                {
                    object res = Evaluator.Evaluate(exprStr, namedItems.Dict);
                    if (res is bool b) return b;
                }
                catch (Exception ex) // exception of Evaluator could be more descriptive
                {
                    throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{exprStr}': {ex.Message}");
                }
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be boolean");
        }

        private static double RequireNumber(JsonElement obj, string prop)
        {
            var el = RequireProperty(obj, prop);
            if (el.ValueKind != JsonValueKind.Number) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be number");
            return el.GetDouble();
        }

        private int RequireInteger(JsonElement obj, string? prop = null)
        {
            JsonElement el = obj;
            if (prop != null) el = RequireProperty(obj, prop);
            if (el.ValueKind == JsonValueKind.String)
            {
                object res = Evaluator.Evaluate(el.GetString()!, namedItems.Dict);
                if (res is double d) return (int)d;
                if (res is int i) return i;
            }
            if (el.ValueKind == JsonValueKind.Number) return el.GetInt32();
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be number or expression");
        }

        /// <summary>
        /// Try to get an optional property, return default (undefined) if not found. 
        /// try with different property names to match AI variations
        /// </summary>
        /// <param name="obj"></param>
        /// <param name="prop"></param>
        /// <returns></returns>
        private static JsonElement GetOptional(JsonElement obj, params string[] prop)
        {
            for (int i = 0; i < prop.Length; i++)
            {
                if (obj.TryGetProperty(prop[i], out var el)) return el;
            }
            return default; // which is JsonElement undefined
        }

        private static string? GetOptionalString(JsonElement obj, string prop)
        {
            if (obj.ValueKind == JsonValueKind.Null || obj.ValueKind == JsonValueKind.Undefined) return null;
            JsonElement el = obj;
            if (prop != null && !obj.TryGetProperty(prop, out el)) return null;
            if (el.ValueKind == JsonValueKind.Null) return null;
            if (el.ValueKind != JsonValueKind.String) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be string");
            return el.GetString();
        }

        private object? GetOptionalObjectRef(JsonElement obj, string propName)
        {
            if (!obj.TryGetProperty(propName, out var el)) return null;
            if (el.ValueKind == JsonValueKind.Null) return null;
            if (el.ValueKind == JsonValueKind.Undefined) return null;
            var objRef = el;                                  // oder RequireObject(...) je nach Format
            return ResolveObjectRef(objRef);
        }

        private static double GetOptionalNumber(JsonElement obj, string prop, double def)
        {
            JsonElement el = obj;
            if (prop != null && !obj.TryGetProperty(prop, out el)) return def;
            if (el.ValueKind == JsonValueKind.Undefined) return def;
            if (el.ValueKind == JsonValueKind.Null) return def;
            if (el.ValueKind != JsonValueKind.Number) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be number");
            return el.GetDouble();
        }

        private int GetOptionalInteger(JsonElement obj, string prop, int def)
        {
            JsonElement el = obj;
            if (prop != null && !obj.TryGetProperty(prop, out el)) return def;
            if (el.ValueKind == JsonValueKind.Null) return def;

            if (el.ValueKind == JsonValueKind.Number || el.ValueKind == JsonValueKind.String) return RequireInteger(el);
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be integer");
        }

        private GeoVector GetOptionalVector3D(JsonElement obj, string? prop, GeoVector defaultValue)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else el = GetOptional(obj, prop);
            if (el.ValueKind == JsonValueKind.Undefined || el.ValueKind == JsonValueKind.Null) return defaultValue;
            try
            {
                return RequireVector3D(el, null);
            }
            catch (JsonRpcException)
            {
                return defaultValue;
            }
        }

        private GeoVector GetOptionalViewDirection(JsonElement el)
        {
            // Negated CADability isometric: StandardProjection.Isometric stores direction = xdir ^ ydir
            // = (1,1,2), which points from the scene towards the camera. The Projection(Direction, up)
            // constructor used by the renderer expects the opposite sense (camera towards scene).
            GeoVector defaultDir = new GeoVector(-1, -1, -2);
            if (el.ValueKind == JsonValueKind.Undefined || el.ValueKind == JsonValueKind.Null) return defaultDir;
            if (el.ValueKind == JsonValueKind.String)
            {
                return el.GetString() switch
                {
                    "top"       => new GeoVector(0,  0, -1),
                    "bottom"    => new GeoVector(0,  0,  1),
                    "front"     => new GeoVector(0,  1,  0),
                    "back"      => new GeoVector(0, -1,  0),
                    "left"      => new GeoVector(1,  0,  0),
                    "right"     => new GeoVector(-1, 0,  0),
                    "isometric" => new GeoVector(-1, -1, -2),
                    _           => defaultDir
                };
            }
            return RequireVector3D(el, null);
        }

        private GeoVector2D GetOptionalVector2D(JsonElement obj, string? prop, GeoVector2D defaultValue)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else el = GetOptional(obj, prop);
            if (el.ValueKind == JsonValueKind.Undefined || el.ValueKind == JsonValueKind.Null) return defaultValue;
            try
            {
                return RequireVector2D(el, null);
            }
            catch (JsonRpcException)
            {
                return defaultValue;
            }
        }

        private double RequireAngle(JsonElement obj, string? prop = null)
        {
            JsonElement angleEl = obj;
            if (prop != null) angleEl = RequireProperty(obj, prop);
            string? expr = null;
            if (angleEl.ValueKind == JsonValueKind.Object)
            {   // either "expr" or "full"
                if (angleEl.TryGetProperty("expr", out JsonElement exprEl))
                {
                    expr = exprEl.GetString();
                }
                else if (angleEl.TryGetProperty("full", out JsonElement fullEl) && fullEl.ValueKind == JsonValueKind.True)
                {
                    return 360;
                }
            }
            if (angleEl.ValueKind == JsonValueKind.String) expr = angleEl.GetString();
            if (expr != null)
            {
                try
                {
                    object res = Evaluator.Evaluate(expr, namedItems.Dict);
                    if (res is double) return (double)res;
                }
                catch (Exception ex) // exception of Evaluator could be more descriptive
                {
                    throw new JsonRpcException("E_InE_INVALID_PARAMS", $"Invalid params: '{prop}', error in expression '{expr}': {ex.Message}");
                }
            }
            if (angleEl.ValueKind == JsonValueKind.Number) return angleEl.GetDouble();

            throw new JsonRpcException("E_InE_INVALID_PARAMS", $"Invalid params: '{prop}' must be number, expression or named value");
        }

        private double GetOptionalAngle(JsonElement obj, string? prop, double def)
        {
            JsonElement axisEl = obj;
            if (prop != null && !obj.TryGetProperty(prop, out axisEl)) return def;
            if (axisEl.ValueKind != JsonValueKind.Undefined) return def; // maybe undefined obj
            return RequireAngle(obj, prop);
        }

        private Axis RequireAxis3D(JsonElement obj, string? prop = null)
        {
            JsonElement axisEl = obj;
            if (prop != null) axisEl = RequireProperty(obj, prop);
            GeoPoint org = RequirePoint3D(axisEl, "origin");
            GeoVector dir = RequireVector3D(axisEl, "direction");
            if (dir.IsNullVector()) throw new JsonRpcException("E_INVALID_PARAMS", "Axis direction cannot be null vector.");
            return new Axis(org, dir);
        }

        private Axis GetOptionalAxis3D(JsonElement obj, string? prop, Axis def)
        {
            JsonElement axisEl = obj;
            if (prop != null && !obj.TryGetProperty(prop, out axisEl)) return def;
            if (axisEl.ValueKind != JsonValueKind.Undefined) return def; // maybe undefined obj
            return RequireAxis3D(obj, prop);
        }

        private Axis2D RequireAxis2D(JsonElement obj, string? prop = null)
        {
            JsonElement axisEl = obj;
            if (prop != null) axisEl = RequireProperty(obj, prop);
            GeoPoint2D org = RequirePoint2D(axisEl, "origin");
            GeoVector2D dir = RequireVector2D(axisEl, "direction");
            if (dir.IsNullVector()) throw new JsonRpcException("E_INVALID_PARAMS", "Axis direction cannot be null vector.");
            return new Axis2D(org, dir);
        }

        private Axis2D GetOptionalAxis2D(JsonElement obj, string? prop, Axis2D def)
        {
            JsonElement axisEl = obj;
            if (prop != null && !obj.TryGetProperty(prop, out axisEl)) return def;
            if (axisEl.ValueKind == JsonValueKind.Undefined) return def; // maybe undefined obj
            return RequireAxis2D(obj, prop);
        }

        private Plane GetOptionalPlane(JsonElement obj, string? prop, Plane def)
        {
            JsonElement planeEl = obj;
            if (prop != null && !obj.TryGetProperty(prop, out planeEl)) return def;
            if (planeEl.ValueKind != JsonValueKind.Undefined) return def; // maybe undefined obj
            return RequirePlane(obj, prop);
        }

        private Plane RequirePlane(JsonElement obj, string? prop = null)
        {
            JsonElement planeEl = obj;
            if (prop != null) planeEl = RequireProperty(obj, prop);
            // PlaneRef can be either {standard:"XY"|"YZ"|"XZ"} or {origin:{x,y,z}, normal:{x,y,z}, xAxis?:{x,y,z}}
            if (planeEl.TryGetProperty("standard", out JsonElement stdEl) && stdEl.ValueKind == JsonValueKind.String)
            {
                string std = stdEl.GetString()?.ToUpper() ?? "XY";
                return std switch
                {
                    "XY" => Plane.XYPlane,
                    "YZ" => Plane.YZPlane,
                    "XZ" => Plane.XZPlane,
                    _ => throw new JsonRpcException("E_INVALID_PARAMS", "Unknown standard plane.")
                };
            }

            if (planeEl.TryGetProperty("origin", out JsonElement orgEl) && planeEl.TryGetProperty("xAxis", out JsonElement xEl))
            {
                GeoPoint org = RequirePoint3D(orgEl);
                GeoVector dirx = RequireVector3D(xEl);
                //if (dirx == null) throw new JsonRpcException("E_INVALID_PARAMS", "Invalid plane xAxis.");
                GeoVector diry = GeoVector.Invalid;
                if (planeEl.TryGetProperty("yAxis", out JsonElement yEl))
                {
                    diry = RequireVector3D(yEl);
                }
                else if (planeEl.TryGetProperty("normal", out JsonElement nEl))
                {
                    GeoVector normal = RequireVector3D(nEl);
                    diry = normal ^ dirx;
                    dirx = diry ^ normal;
                }
                try
                {
                    return new Plane(org, dirx, diry);
                }
                catch (PlaneException ex)
                {
                    throw new JsonRpcException("E_INVALID_PARAMS", "Invalid plane: " + ex.Message);
                }
            }
            if (planeEl.TryGetProperty("origin", out orgEl) && planeEl.TryGetProperty("normal", out JsonElement normalEl))
            {
                GeoPoint org = RequirePoint3D(orgEl);
                GeoVector normal = RequireVector3D(normalEl);
                return new Plane(org, normal);
            }
            throw new JsonRpcException("E_INVALID_PARAMS", "Invalid plane.");
        }

        private GeoPoint2D GetOptionalPoint2D(JsonElement obj, string? prop, GeoPoint2D defaultValue)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else el = GetOptional(obj, prop);
            if (el.ValueKind == JsonValueKind.Undefined || el.ValueKind == JsonValueKind.Null) return defaultValue;
            try
            {
                return RequirePoint2D(el, null);
            }
            catch (JsonRpcException)
            {
                return defaultValue;
            }
        }

        private GeoPoint2D RequirePoint2D(JsonElement obj, string? prop = null)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else el = RequireProperty(obj, prop);
            string? expr = null;

            if (el.ValueKind == JsonValueKind.Array)
            {
                List<double> coords = new List<double>();
                foreach (var a in el.EnumerateArray())
                {
                    coords.Add(RequireDouble(a, null));
                }
                if (coords.Count == 2) return new GeoPoint2D(coords[0], coords[1]);
            }
            if (el.ValueKind == JsonValueKind.String)
            {
                expr = el.GetString();
            }
            else if (el.ValueKind == JsonValueKind.Object)
            {
                if (el.TryGetProperty("name", out var pname))
                {
                    if (pname.ValueKind == JsonValueKind.String)
                    {
                        string? name = pname.GetString();
                        if (name != null && namedItems.TryGetValue(name, out object? o) && o is GeoPoint2D res) return res;
                    }
                }
                else if (el.TryGetProperty("expr", out var pexpr))
                {
                    if (pexpr.ValueKind == JsonValueKind.String)
                    {
                        expr = pexpr.GetString();
                    }
                }
                else if (el.TryGetProperty("x", out _) && el.TryGetProperty("y", out _))
                {
                    return new GeoPoint2D(RequireDouble(el, "x"), RequireDouble(el, "y"));
                }
            }
            if (expr != null)
            {
                try
                {
                    object res = Evaluator.Evaluate(expr, namedItems.Dict);
                    if (res is GeoPoint2D pres2) return pres2;
                }
                catch (Exception ex) // exception of Evaluator could be more descriptive
                {
                    throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{expr}': {ex.Message}");
                }
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be a 2d point");
        }

        private GeoPoint RequirePoint3D(JsonElement obj, string? prop = null)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else el = RequireProperty(obj, prop);
            string? expr = null;

            if (el.ValueKind == JsonValueKind.Array)
            {
                List<double> coords = new List<double>();
                foreach (var a in el.EnumerateArray())
                {
                    coords.Add(RequireDouble(a, null));
                }
                if (coords.Count == 3) return new GeoPoint(coords[0], coords[1], coords[2]);
            }
            if (el.ValueKind == JsonValueKind.String)
            {
                expr = el.GetString();
            }
            else if (el.ValueKind == JsonValueKind.Object)
            {
                if (el.TryGetProperty("name", out var pname))
                {
                    if (pname.ValueKind == JsonValueKind.String)
                    {
                        string? name = pname.GetString();
                        if (name != null && namedItems.TryGetValue(name, out object? o) && o is GeoPoint res) return res;
                    }
                }
                else if (el.TryGetProperty("expr", out var pexpr))
                {
                    if (pexpr.ValueKind == JsonValueKind.String)
                    {
                        expr = pexpr.GetString();
                    }
                }
                else if (el.TryGetProperty("x", out _) && el.TryGetProperty("y", out _) && el.TryGetProperty("z", out _))
                {
                    return new GeoPoint(RequireDouble(el, "x"), RequireDouble(el, "y"), RequireDouble(el, "z"));
                }
            }
            if (expr != null)
            {
                try
                {
                    object res = Evaluator.Evaluate(expr, namedItems.Dict);
                    if (res is GeoPoint pres3) return pres3;
                }
                catch (Exception ex) // exception of Evaluator could be more descriptive
                {
                    throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{expr}': {ex.Message}");
                }
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be a 3d point");
        }

        private GeoPoint GetOptionalPoint3D(JsonElement obj, string? prop, GeoPoint defaultValue)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else el = GetOptional(obj, prop);
            if (el.ValueKind == JsonValueKind.Undefined || el.ValueKind == JsonValueKind.Null) return defaultValue;
            try
            {
                return RequirePoint3D(el, null);
            }
            catch (JsonRpcException)
            {
                return defaultValue;
            }
        }

        private BoundingBox RequireBoundingBox(JsonElement obj, string? prop)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else el = RequireProperty(obj, prop);
            if (el.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be object");
            if (el.TryGetProperty("name", out var pname))
            {
                if (pname.ValueKind == JsonValueKind.String)
                {
                    string? name = pname.GetString();
                    if (name != null && namedItems.TryGetValue(name, out object? o) && o is BoundingBox res) return res;
                }
            }
            else if (el.TryGetProperty("xmin", out _) && el.TryGetProperty("ymin", out _) && el.TryGetProperty("zmin", out _)
                && el.TryGetProperty("xmax", out _) && el.TryGetProperty("ymax", out _) && el.TryGetProperty("zmax", out _))
            {
                return ReadBoundingBox(el);
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be a bounding box");
        }

        private GeoVector RequireVector3D(JsonElement obj, string? prop = null)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else el = RequireProperty(obj, prop);
            string? expr = null;

            if (el.ValueKind == JsonValueKind.Array)
            {
                List<double> coords = new List<double>();
                foreach (var a in el.EnumerateArray())
                {
                    coords.Add(RequireDouble(a, null));
                }
                if (coords.Count == 3) return new GeoVector(coords[0], coords[1], coords[2]);
            }
            if (el.ValueKind == JsonValueKind.String)
            {
                expr = el.GetString();
            }
            else if (el.ValueKind == JsonValueKind.Object)
            {
                if (el.TryGetProperty("name", out var pname))
                {
                    if (pname.ValueKind == JsonValueKind.String)
                    {
                        string? name = pname.GetString();
                        if (name != null && namedItems.TryGetValue(name, out object? o) && o is GeoVector res) return res;
                    }
                }
                else if (el.TryGetProperty("expr", out var pexpr))
                {
                    if (pexpr.ValueKind == JsonValueKind.String)
                    {
                        expr = pexpr.GetString();
                    }
                }
                else if (el.TryGetProperty("x", out _) && el.TryGetProperty("y", out _) && el.TryGetProperty("z", out _))
                {
                    return new GeoVector(RequireDouble(el, "x"), RequireDouble(el, "y"), RequireDouble(el, "z"));
                }
            }
            if (expr != null)
            {
                try
                {
                    object res = Evaluator.Evaluate(expr, namedItems.Dict);
                    if (res is GeoVector pres3) return pres3;
                }
                catch (Exception ex) // exception of Evaluator could be more descriptive
                {
                    throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{expr}': {ex.Message}");
                }
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be a 3d vector");
        }

        private GeoVector2D RequireVector2D(JsonElement obj, string? prop = null)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else el = RequireProperty(obj, prop);
            string? expr = null;

            if (el.ValueKind == JsonValueKind.Array)
            {
                List<double> coords = new List<double>();
                foreach (var a in el.EnumerateArray())
                {
                    coords.Add(RequireDouble(a, null));
                }
                if (coords.Count == 2) return new GeoVector2D(coords[0], coords[1]);
            }
            if (el.ValueKind == JsonValueKind.String)
            {
                expr = el.GetString();
            }
            else if (el.ValueKind == JsonValueKind.Object)
            {
                if (el.TryGetProperty("name", out var pname))
                {
                    if (pname.ValueKind == JsonValueKind.String)
                    {
                        string? name = pname.GetString();
                        if (name != null && namedItems.TryGetValue(name, out object? o) && o is GeoVector2D res) return res;
                    }
                }
                else if (el.TryGetProperty("expr", out var pexpr))
                {
                    if (pexpr.ValueKind == JsonValueKind.String)
                    {
                        expr = pexpr.GetString();
                    }
                }
                else if (el.TryGetProperty("x", out _) && el.TryGetProperty("y", out _))
                {
                    return new GeoVector2D(RequireDouble(el, "x"), RequireDouble(el, "y"));
                }
            }
            if (expr != null)
            {
                try
                {
                    object res = Evaluator.Evaluate(expr, namedItems.Dict);
                    if (res is GeoVector2D pres3) return pres3;
                }
                catch (Exception ex) // exception of Evaluator could be more descriptive
                {
                    throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{expr}': {ex.Message}");
                }
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be a 2d vector");
        }

        private double GetOptionalDouble(JsonElement obj, string? prop, double defaultValue)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else if (!obj.TryGetProperty(prop, out el)) return defaultValue;
            if (el.ValueKind == JsonValueKind.Undefined) return defaultValue;
            if (el.ValueKind == JsonValueKind.Number || el.ValueKind == JsonValueKind.String || (el.ValueKind == JsonValueKind.Object && el.TryGetProperty("expr", out var _)))
            {
                return RequireDouble(el, null);
            }
            else
            {
                return defaultValue;
            }
        }

        private double RequireDouble(JsonElement obj, string? prop)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else el = RequireProperty(obj, prop);
            string? expr = null;
            if (el.ValueKind == JsonValueKind.Number) { return el.GetDouble(); }
            if (el.ValueKind == JsonValueKind.Object)
            {
                if (el.TryGetProperty("name", out var pname))
                {
                    if (pname.ValueKind == JsonValueKind.String)
                    {
                        string? name = pname.GetString();
                        if (name != null && namedItems.TryGetValue(name, out object? o) && o is double res) return res;
                    }
                }
                else if (el.TryGetProperty("expr", out var pexpr))
                {
                    if (pexpr.ValueKind == JsonValueKind.String)
                    {
                        expr = pexpr.GetString();
                    }

                }
            }
            else if (el.ValueKind == JsonValueKind.String)
            {
                expr = el.GetString();
            }
            if (expr != null)
            {
                try
                {
                    object res = Evaluator.Evaluate(expr, namedItems.Dict);
                    if (res is double) return (double)res;
                }
                catch (Exception ex) // exception of Evaluator could be more descriptive
                {
                    throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{expr}': {ex.Message}");
                }
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be number, expression or named value");
        }

        #endregion

        #region Object references and selectors

        private string? ParseObjectRef(JsonElement objRef)
        {
            if (objRef.ValueKind == JsonValueKind.String) { return objRef.GetString(); }
            if (objRef.ValueKind == JsonValueKind.Object && objRef.TryGetProperty("name", out var nameEl)) return nameEl.GetString();
            throw new JsonRpcException(-32602, "Invalid params: ObjectRef must contain a string, or the property 'name'");
        }

        private object ResolveObjectRef(JsonElement objRef)
        {
            var name = ParseObjectRef(objRef);
            if (name != null)
            {
                if (namedItems.TryGetValue(name, out var o))
                {
                    if (o is IGeoObject go) go.UserData.Add("CADablity.MCP.Name", name);
                    return o;
                }
            }
            throw NamedItemNotFound(name);
        }

        private IEnumerable<object> ExpandResolved(JsonElement el)
        {
            var resolved = ResolveObjectRef(el);

            if (resolved is System.Collections.IEnumerable enumerable
                && resolved is not string)
            {
                foreach (var item in enumerable)
                    yield return item;
            }
            else
            {
                yield return resolved;
            }
        }

        private IEnumerable<object> IterateObjectRefs(JsonElement a)
        {
            if (a.ValueKind == JsonValueKind.Array)
            {
                foreach (var el in a.EnumerateArray())
                {
                    foreach (var resolved in ExpandResolved(el))
                        yield return resolved;
                }
            }
            else
            {
                foreach (var resolved in ExpandResolved(a))
                    yield return resolved;
            }
        }

        private IEnumerable<T> IterateSelector<T>(JsonElement selector) where T : class
        {
            if (selector.ValueKind == JsonValueKind.Undefined) yield break;
            if (selector.ValueKind == JsonValueKind.Array)
            {
                foreach (var el in selector.EnumerateArray())
                {
                    foreach (var t in IterateSelector<T>(el)) yield return t;
                }
                yield break;
            }
            if (selector.ValueKind == JsonValueKind.String)
            {
                string target = selector.GetString()!;
                if (namedItems.TryGetValue(target, out object? val))
                {
                    if (val is IEnumerable<T> seq) foreach (T item in seq) yield return item;
                    else if (val is T t) yield return t;
                }
                else throw NamedItemNotFound(target);
                yield break;
            }
            if (selector.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, "Invalid params: Selector must be an object");
            JsonElement je;
            if (selector.TryGetProperty("name", out je) && je.ValueKind == JsonValueKind.String)
            {
                if (namedItems.TryGetValue(je.GetString()!, out object? val))
                {   // check list first: when T is object, the whole list is returned as an item
                    if (val is IEnumerable<T> seq) foreach (T item in seq) yield return item;
                    else if (val is T t) yield return t;
                }
                else throw NamedItemNotFound(je.GetString());
            }
            else if (selector.TryGetProperty("names", out je) && je.ValueKind == JsonValueKind.Array)
            {   // array of ObjectRefs
                foreach (var t in IterateObjectRefs<T>(je)) yield return t;
            }
            else if (selector.TryGetProperty("op", out je))
            {   // a boolean operation, test before "items", because it also contains "items"
                string? op = null;
                if (je.ValueKind == JsonValueKind.String) op = je.GetString();
                if (op != null && selector.TryGetProperty("items", out var booleanItems) && booleanItems.ValueKind == JsonValueKind.Array)
                {
                    List<List<T>> items = new List<List<T>>();
                    foreach (var el in booleanItems.EnumerateArray())
                    {
                        List<T> item = IterateSelector<T>(el).ToList();
                        items.Add(item);
                    }
                    HashSet<T> result = [.. items[0]];
                    switch (op)
                    {
                        case "union":
                        case "unite":
                            for (int i = 1; i < items.Count; i++)
                            {
                                result.UnionWith(items[i]);
                            }
                            break;
                        case "difference":
                        case "subtract":
                        case "except":
                            for (int i = 1; i < items.Count; i++)
                            {
                                result.ExceptWith(items[i]);
                            }
                            break;
                        case "intersect":
                            for (int i = 1; i < items.Count; i++)
                            {
                                result.IntersectWith(items[i]);
                            }
                            break;
                        default:
                            throw new JsonRpcException(-32602, $"Unknown boolean operator '{op}'");
                    }
                    foreach (var item in result) yield return item;
                }
            }
            else if (selector.TryGetProperty("items", out je) && je.ValueKind == JsonValueKind.Array)
            {   // the same as names, sometimes AI calls it items although in the definition it should be called names
                foreach (var t in IterateObjectRefs<T>(je)) yield return t;
            }
            else if (selector.TryGetProperty("query", out je))
            {   // a query
                string target = RequireString(je, "target");
                switch (target)
                {

                    case "solids":
                        foreach (Solid t in IterateQuery<Solid>(je)) if (t is T tt) yield return tt;
                        break;
                    case "faces":
                        foreach (Face t in IterateQuery<Face>(je)) if (t is T tt) yield return tt;
                        break;
                    case "edges":
                        foreach (Edge t in IterateQuery<Edge>(je)) if (t is T tt) yield return tt;
                        break;
                    case "sketch_geometry":
                        foreach (ICurve2D t in IterateQuery<ICurve2D>(je)) if (t is T tt) yield return tt;
                        foreach (CompoundShape t in IterateQuery<CompoundShape>(je)) if (t is T tt) yield return tt;
                        break;
                    default: throw new JsonRpcException(-32602, $"Unknown query target '{target}'");
                }
            }
        }

        private IEnumerable<T> IterateQuery<T>(JsonElement query) where T : class
        {
            // from, filter
            if (query.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, "Invalid params: Query must be an object");
            JsonElement from = RequireProperty(query, "from");
            List<object> froms = IterateObjectRefs(from).ToList();
            List<T> fromsT = ExpandToType<T>(froms);
            JsonElement filter;
            if (!query.TryGetProperty("filter", out filter))
            {
                foreach (object obj in fromsT)
                {
                    if (obj is T t) yield return t;
                }
            }
            else
            {
                if (filter.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, "Invalid params: Filter must be an object");
                // there are different kinds of filters: edge, face, solid sketch geometry
                // we filter al properties and ignore those, which don't belong to type T
                JsonElement je;
                if (filter.TryGetProperty("extreme", out je))
                {   // here we are looking for the object with extreme coordinates. We must check all objects before
                    // yielding candidates
                    string axis = RequireString(je, "axis"); // x, y or z
                    string which = RequireString(je, "which"); // min or max
                    bool checkMin = which == "min";
                    double currentExtreme, currentMiddle = double.NaN;
                    if (checkMin) currentExtreme = double.MaxValue;
                    else currentExtreme = double.MinValue;
                    T? extremeObject = null;
                    foreach (T toTest in fromsT)
                    {
                        BoundingBox bb = BoundingBox.EmptyBoundingBox;
                        if (toTest is Face face) bb = face.GetBoundingCube();
                        if (toTest is Solid sld) bb = sld.GetBoundingCube();
                        if (toTest is Edge edge && edge.Curve3D is IGeoObject go) bb = go.GetBoundingCube();
                        switch (axis)
                        {
                            case "x":
                                if (checkMin)
                                {
                                    if (bb.Xmin < currentExtreme)
                                    {
                                        currentExtreme = bb.Xmin;
                                        currentMiddle = (bb.Xmin + bb.Xmax) / 2;
                                        extremeObject = toTest;
                                    }
                                    else if (bb.Xmin == currentExtreme)
                                    {
                                        double m = (bb.Xmin + bb.Xmax) / 2;
                                        if (double.IsNaN(currentMiddle) || m < currentMiddle)
                                        {
                                            currentMiddle = m;
                                            extremeObject = toTest;
                                        }
                                    }
                                }
                                else
                                {
                                    if (bb.Xmax > currentExtreme)
                                    {
                                        currentExtreme = bb.Xmax;
                                        currentMiddle = (bb.Xmin + bb.Xmax) / 2;
                                        extremeObject = toTest;
                                    }
                                    else if (bb.Xmax == currentExtreme)
                                    {
                                        double m = (bb.Xmin + bb.Xmax) / 2;
                                        if (double.IsNaN(currentMiddle) || m > currentMiddle)
                                        {
                                            currentMiddle = m;
                                            extremeObject = toTest;
                                        }
                                    }
                                }
                                break;
                            case "y":
                                if (checkMin)
                                {
                                    if (bb.Ymin < currentExtreme)
                                    {
                                        currentExtreme = bb.Ymin;
                                        currentMiddle = (bb.Ymin + bb.Ymax) / 2;
                                        extremeObject = toTest;
                                    }
                                    else if (bb.Ymin == currentExtreme)
                                    {
                                        double m = (bb.Ymin + bb.Ymax) / 2;
                                        if (double.IsNaN(currentMiddle) || m < currentMiddle)
                                        {
                                            currentMiddle = m;
                                            extremeObject = toTest;
                                        }
                                    }
                                }
                                else
                                {
                                    if (bb.Ymax > currentExtreme)
                                    {
                                        currentExtreme = bb.Ymax;
                                        currentMiddle = (bb.Ymin + bb.Ymax) / 2;
                                        extremeObject = toTest;
                                    }
                                    else if (bb.Ymax == currentExtreme)
                                    {
                                        double m = (bb.Ymin + bb.Ymax) / 2;
                                        if (double.IsNaN(currentMiddle) || m > currentMiddle)
                                        {
                                            currentMiddle = m;
                                            extremeObject = toTest;
                                        }
                                    }
                                }
                                break;

                            case "z":
                                if (checkMin)
                                {
                                    if (bb.Zmin < currentExtreme)
                                    {
                                        currentExtreme = bb.Zmin;
                                        currentMiddle = (bb.Zmin + bb.Zmax) / 2;
                                        extremeObject = toTest;
                                    }
                                    else if (bb.Zmin == currentExtreme)
                                    {
                                        double m = (bb.Zmin + bb.Zmax) / 2;
                                        if (double.IsNaN(currentMiddle) || m < currentMiddle)
                                        {
                                            currentMiddle = m;
                                            extremeObject = toTest;
                                        }
                                    }
                                }
                                else
                                {
                                    if (bb.Zmax > currentExtreme)
                                    {
                                        currentExtreme = bb.Zmax;
                                        currentMiddle = (bb.Zmin + bb.Zmax) / 2;
                                        extremeObject = toTest;
                                    }
                                    else if (bb.Zmax == currentExtreme)
                                    {
                                        double m = (bb.Zmin + bb.Zmax) / 2;
                                        if (double.IsNaN(currentMiddle) || m > currentMiddle)
                                        {
                                            currentMiddle = m;
                                            extremeObject = toTest;
                                        }
                                    }
                                }
                                break;
                        }
                    }
                    if (extremeObject != null) yield return extremeObject;
                }
                else
                {

                    // Filter properties that reference other objects are the same for every candidate,
                    // so resolve them once here instead of once per item.
                    List<Face>? onFaces = null, notOnFaces = null;
                    Face? sameSurfaceFace = null;
                    if (filter.TryGetProperty("onFace", out je)) onFaces = IterateSelector<Face>(je).ToList();
                    if (filter.TryGetProperty("notOnFace", out je)) notOnFaces = IterateSelector<Face>(je).ToList();
                    if (filter.TryGetProperty("sameSurface", out je)) sameSurfaceFace = IterateSelector<Face>(je).FirstOrDefault();

                    foreach (T toTest in fromsT)
                    {
                        if (toTest == null) continue;
                        if (filter.TryGetProperty("surfaceType", out je) && typeof(T) == typeof(Face))
                        {
                            if (je.ValueKind != JsonValueKind.String) throw new JsonRpcException(-32602, "Invalid params: SurfaceType must be a string");
                            if (!(toTest is Face face)) continue;
                            string? surfaceType = je.GetString();
                            {
                                switch (surfaceType!)
                                {
                                    case "planar": if (!(face.Surface is PlaneSurface)) continue; break;
                                    case "cylindrical": if (!(face.Surface is CylindricalSurface)) continue; break;
                                    case "conical": if (!(face.Surface is ConicalSurface)) continue; break;
                                    case "spherical": if (!(face.Surface is SphericalSurface)) continue; break;
                                    case "toroidal": if (!(face.Surface is ToroidalSurface)) continue; break;
                                    case "freeform": break;
                                    default: throw new JsonRpcException(-32602, $"Invalid params: 'surfaceType' = '{surfaceType}' must be one of planar, cylindrical, conical, spherical, toroidal or freeform");
                                }
                            }
                        }
                        if (filter.TryGetProperty("condition", out je))
                        {
                            string? expr = null;
                            if (je.ValueKind == JsonValueKind.String) expr = je.GetString();
                            else if (je.ValueKind == JsonValueKind.Object && je.TryGetProperty("expr", out var exprEl) && exprEl.ValueKind == JsonValueKind.String) expr = exprEl.GetString();
                            if (expr == null) throw new JsonRpcException("E_INVALID_PARAMETER", "condition not found");

                            using (new NamedItemOverride(namedItems, toTest))
                            {
                                object evalRes = Evaluator.Evaluate(expr, namedItems.Dict);
                                if (evalRes is bool b)
                                {
                                    if (!b) continue; // expression was false
                                }
                            }
                        }
                        if (filter.TryGetProperty("closeTo", out je))
                        {
                            GeoPoint p = RequirePoint3D(je, null);
                            BoundingBox pbox = new BoundingBox(p, Precision.eps);
                            if (toTest is Face face && Math.Abs(face.Distance(p)) > Precision.eps) continue;
                            if (toTest is Solid solid && !solid.HitTest(ref pbox, Precision.eps) && !solid.Shell.Contains(p)) continue;
                            if (toTest is Edge edge && edge.Curve3D is IGeoObject go && !go.HitTest(ref pbox, Precision.eps)) continue;
                        }
                        if (filter.TryGetProperty("inside", out je))
                        {
                            if (je.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, "Invalid params: 'inside' must be an object");
                            BoundingBox bbox = RequireBoundingBox(je, null);
                            if (toTest is Face face && !bbox.Contains(face.GetExtent(0.0))) continue;
                            if (toTest is Solid sld && !bbox.Contains(sld.GetExtent(0.0))) continue;
                            if (toTest is Edge edge && edge.Curve3D is IGeoObject go && !bbox.Contains(go.GetExtent(0.0))) continue;
                        }
                        if (filter.TryGetProperty("touchedBy", out je))
                        {
                            if (je.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, "Invalid params: 'touchedBy' must be an object");
                            BoundingBox bbox = RequireBoundingBox(je, null);
                            if (toTest is Face face && !face.HitTest(ref bbox, 0.0)) continue;
                            if (toTest is Solid sld && !sld.HitTest(ref bbox, 0.0)) continue;
                            if (toTest is Edge edge && edge.Curve3D is IGeoObject go && !go.HitTest(ref bbox, 0.0)) continue;
                        }
                        if (filter.TryGetProperty("contains", out je))
                        {
                            GeoPoint innerPoint = RequirePoint3D(je, null);
                            if (toTest is Solid sld && !sld.Shell.Contains(innerPoint)) continue;

                        }
                        if (filter.TryGetProperty("boundingBox", out je))
                        {
                            if (je.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, "Invalid params: 'boundingBox' must be an object");
                            double minValue = GetOptionalDouble(je, "minValue", double.MinValue);
                            double maxValue = GetOptionalDouble(je, "maxValue", double.MaxValue);
                            if (minValue != double.MinValue) minValue -= Precision.eps;
                            if (maxValue != double.MaxValue) maxValue += Precision.eps;
                            BoundingBox bb = BoundingBox.EmptyBoundingBox;
                            if (toTest is Face face) bb = face.GetBoundingCube();
                            if (toTest is Solid sld) bb = sld.GetBoundingCube();
                            if (toTest is Edge edge && edge.Curve3D is IGeoObject go) bb = go.GetBoundingCube();
                            string component = RequireString(je, "component");
                            switch (component.ToLower())
                            {   // face/solid style names (left, top, ...) and edge style names (xMin, zDiff, ...)
                                // are both accepted, so a filter written for one target works for the others too
                                case "left": case "xmin": if (bb.Xmin < minValue || bb.Xmin > maxValue) continue; break;
                                case "right": case "xmax": if (bb.Xmax < minValue || bb.Xmax > maxValue) continue; break;
                                case "front": case "ymin": if (bb.Ymin < minValue || bb.Ymin > maxValue) continue; break;
                                case "back": case "ymax": if (bb.Ymax < minValue || bb.Ymax > maxValue) continue; break;
                                case "bottom": case "zmin": if (bb.Zmin < minValue || bb.Zmin > maxValue) continue; break;
                                case "top": case "zmax": if (bb.Zmax < minValue || bb.Zmax > maxValue) continue; break;
                                case "centerx": if (bb.GetCenter().x < minValue || bb.GetCenter().x > maxValue) continue; break;
                                case "centery": if (bb.GetCenter().y < minValue || bb.GetCenter().y > maxValue) continue; break;
                                case "centerz": if (bb.GetCenter().z < minValue || bb.GetCenter().z > maxValue) continue; break;
                                case "xdiff": if (bb.XDiff < minValue || bb.XDiff > maxValue) continue; break;
                                case "ydiff": if (bb.YDiff < minValue || bb.YDiff > maxValue) continue; break;
                                case "zdiff": if (bb.ZDiff < minValue || bb.ZDiff > maxValue) continue; break;
                                default: throw new JsonRpcException(-32602, $"Invalid params: 'component' = '{component}' must be one of left/xMin, right/xMax, front/yMin, back/yMax, bottom/zMin, top/zMax, centerX, centerY, centerZ, xDiff, yDiff, zDiff");
                            }
                        }

                        // Edge specific filters (EdgeFilter in the toolset definition). They used to live
                        // in a separate FilterEdges path which became unreachable when the tools moved to
                        // NameRef + workspace.select; here they compose with the generic filters above.
                        if (toTest is Edge edgeToTest)
                        {
                            if (filter.TryGetProperty("isConvex", out je))
                            {   // tangential and same-surface edges are neither convex nor concave and
                                // are therefore rejected by both settings
                                AdjacencyType adjacency = edgeToTest.Adjacency();
                                bool wantConvex = je.ValueKind == JsonValueKind.True;
                                if (wantConvex && adjacency != AdjacencyType.Convex) continue;
                                if (!wantConvex && adjacency != AdjacencyType.Concave) continue;
                            }
                            if (onFaces != null && !onFaces.Any(f => edgeToTest.PrimaryFace == f || edgeToTest.SecondaryFace == f)) continue;
                            if (notOnFaces != null && notOnFaces.Any(f => edgeToTest.PrimaryFace == f || edgeToTest.SecondaryFace == f)) continue;
                            if (filter.TryGetProperty("length", out je))
                            {
                                double minLength = GetOptionalDouble(je, "minValue", double.MinValue);
                                double maxLength = GetOptionalDouble(je, "maxValue", double.MaxValue);
                                double length = edgeToTest.Curve3D != null ? edgeToTest.Curve3D.Length : 0.0;
                                if (length < minLength || length > maxLength) continue;
                            }
                        }
                        // Face specific filter: only faces lying on the same surface as a reference face
                        if (toTest is Face faceToTest && sameSurfaceFace != null)
                        {
                            if (faceToTest.Surface == null || sameSurfaceFace.Surface == null) continue;
                            if (!faceToTest.Surface.SameGeometry(faceToTest.Domain, sameSurfaceFace.Surface, sameSurfaceFace.Domain, Precision.eps, out ModOp2D _)) continue;
                        }

                        // when we arrive here, all conditions have been fullfilled
                        yield return toTest!;
                    }
                }
            }
        }

        private List<T> ExpandToType<T>(List<object> froms) where T : class
        {
            List<T> result = [];
            if (typeof(T) == typeof(Solid))
            {
                foreach (object obj in froms) if (obj is T t) { result.Add(t); }
                ;
            }
            else if (typeof(T) == typeof(Face))
            {
                foreach (object obj in froms)
                {
                    if (obj is T t) result.Add(t);
                    else if (obj is Solid sld) foreach (Face face in sld.Shells[0].Faces) result.Add(face as T);
                }
            }
            else if (typeof(T) == typeof(Edge))
            {
                foreach (object obj in froms)
                {
                    if (obj is T t) result.Add(t);
                    else if (obj is Solid sld)
                    {
                        foreach (Edge edge in sld.Shells[0].Edges) result.Add(edge as T);
                    }
                    else if (obj is Face face)
                    {
                        foreach (Edge edge in face.Edges) result.Add(edge as T);
                    }
                }
            }
            else
            {
                foreach (object obj in froms)
                {
                    if (obj is T t) result.Add(t);
                }
            }
            return result;
        }

        private IEnumerable<Face> FacesOf(List<object> objects)
        {
            foreach (object obj in objects)
            {
                if (obj is Face face) yield return face;
                if (obj is Solid sld)
                {
                    foreach (Face fc in sld.Shells[0].Faces) yield return fc;
                }
                if (obj is Shell shell)
                {
                    foreach (Face fc in shell.Faces) yield return fc;
                }
            }
        }

        private IEnumerable<T> IterateObjectRefs<T>(JsonElement a)
        {
            if (a.ValueKind == JsonValueKind.Array)
            {
                foreach (var el in a.EnumerateArray())
                    foreach (var resolved in ExpandResolved(el))
                        if (resolved is T t)
                            yield return t;
            }
            else
            {
                foreach (var resolved in ExpandResolved(a))
                    if (resolved is T t)
                        yield return t;
            }
        }

        #endregion

        #region Named-item binding and evaluator support

        private void StoreNamed(string name, object value)
        {
            namedItems[name] = value;
        }

        private void Rebind(Shell oldShell, Shell newShell)
        {
            foreach (var item in namedItems)
            {
                if (item.Value is Edge edge && edge.Owner.Owner == oldShell)
                {
                    Edge? newEdge = newShell.FindSimilarEdge(edge);
                    if (newEdge != null) namedItems[item.Key] = newEdge;
                    else AddCallWarning($"The edge '{item.Key}' could not be rebound to the modified topology and now references stale geometry; re-select it (workspace.select) before further use.");
                }
                if (item.Value is List<Edge> ledge)
                {
                    List<Edge> newList = [];
                    int affected = 0, rebound = 0;
                    for (int i = 0; i < ledge.Count; i++)
                    {
                        if (ledge[i].Owner.Owner == oldShell)
                        {
                            affected++;
                            Edge? newEdgel = newShell.FindSimilarEdge(ledge[i]);
                            if (newEdgel != null) { newList.Add(newEdgel); rebound++; }
                            else newList.Add(ledge[i]);
                        }
                        else newList.Add(ledge[i]);
                    }
                    // only re-store when something was actually rebound, so the result envelope
                    // reports 'modified' only for real rebinds, not for untouched lists
                    if (rebound > 0) namedItems[item.Key] = newList;
                    if (rebound < affected) AddCallWarning($"{affected - rebound} of {affected} edges in '{item.Key}' could not be rebound to the modified topology and now reference stale geometry; re-select them (workspace.select) before further use.");
                }
                if (item.Value is Face face && face.Owner == oldShell)
                {
                    Face? newFace = newShell.FindSimilarFace(face);
                    if (newFace != null) namedItems[item.Key] = newFace;
                    else AddCallWarning($"The face '{item.Key}' could not be rebound to the modified topology and now references stale geometry; re-select it (workspace.select) before further use.");
                }
                if (item.Value is List<Face> lface)
                {
                    List<Face> newList = [];
                    int affected = 0, rebound = 0;
                    for (int i = 0; i < lface.Count; i++)
                    {
                        if (lface[i].Owner == oldShell)
                        {
                            affected++;
                            Face? newFacel = newShell.FindSimilarFace(lface[i]);
                            if (newFacel != null) { newList.Add(newFacel); rebound++; }
                            else newList.Add(lface[i]);
                        }
                        else newList.Add(lface[i]);
                    }
                    if (rebound > 0) namedItems[item.Key] = newList;
                    if (rebound < affected) AddCallWarning($"{affected - rebound} of {affected} faces in '{item.Key}' could not be rebound to the modified topology and now reference stale geometry; re-select them (workspace.select) before further use.");
                }
            }
        }

        private void Rebind(Solid oldSolid, Solid[] newSolids)
        {
            foreach (Solid solid in newSolids)
            {
                Rebind(oldSolid.Shell, solid.Shell);
            }
        }

        private void Rebind(IEnumerable<Solid> oldSolids, IEnumerable<Solid> newSolids)
        {
            foreach (Solid solid1 in oldSolids)
            {
                foreach (Solid solid in newSolids)
                {
                    Rebind(solid1.Shell, solid.Shell);
                }
            }
        }

        private void Rebind(IEnumerable<Solid> oldSolids, Solid newSolid)
        {
            foreach (Solid solid1 in oldSolids)
            {
                Rebind(solid1.Shell, newSolid.Shell);
            }
        }

        private string? FindName(object entity)
        {
            foreach (var item in namedItems)
            {
                if (item.Value == entity) return item.Key;
            }
            return null;
        }

        /// <summary>
        /// Enumerates the workspace names of a NameRef parameter (a single name or an array of names).
        /// Yields nothing for shapes that carry no name, so callers decide whether that is an error.
        /// </summary>
        private IEnumerable<string> AllNames(JsonElement nameRef)
        {
            if (nameRef.ValueKind == JsonValueKind.Array)
            {
                foreach (var el in nameRef.EnumerateArray())
                    if (el.ValueKind == JsonValueKind.String && el.GetString() is string item) yield return item;
            }
            else if (nameRef.ValueKind == JsonValueKind.String)
            {
                if (nameRef.GetString() is string single) yield return single;
            }
            else if (nameRef.ValueKind == JsonValueKind.Object && nameRef.TryGetProperty("name", out var nameEl)
                     && nameEl.ValueKind == JsonValueKind.String && nameEl.GetString() is string named) yield return named;
        }

        private string? FirstName(JsonElement nameRef) => AllNames(nameRef).FirstOrDefault();

        /// <summary>
        /// Returns the single workspace name of a NameRef parameter. A NameRef may also be an array,
        /// but operations working on one shell cannot silently pick one entry out of several.
        /// </summary>
        private string RequireSingleName(JsonElement nameRef, string parameterName)
        {
            if (nameRef.ValueKind == JsonValueKind.Array)
            {
                List<string> names = [.. nameRef.EnumerateArray().Where(el => el.ValueKind == JsonValueKind.String).Select(el => el.GetString()!)];
                if (names.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", $"'{parameterName}' must reference exactly one named solid.");
                return names[0];
            }
            string? name = ParseObjectRef(nameRef);
            if (string.IsNullOrEmpty(name)) throw new JsonRpcException("E_INVALID_PARAMS", $"'{parameterName}' must reference exactly one named solid.");
            return name;
        }

        /// <summary>
        /// Verifies that the 'solid' parameter names the very solid the given shell belongs to and
        /// returns that name. Feature operations derive the shell from their edges and use 'solid'
        /// only for naming, so a mismatch would otherwise overwrite an unrelated workspace item.
        /// </summary>
        private string RequireSolidNameFor(JsonElement solidRef, Shell shell)
        {
            string solidName = RequireSingleName(solidRef, "solid");
            if (!namedItems.TryGetValue(solidName, out object? item) || item == null) throw NamedItemNotFound(solidName);
            if (UnwrapSingletonList(item) is not Solid sld || sld.Shells.Length == 0 || sld.Shells[0] != shell)
                throw new JsonRpcException("E_INVALID_PARAMS", $"'{solidName}' does not denote the single solid the given edges belong to.");
            return solidName;
        }

        private class FaceWrapperForEvaluator
        {
            Face face;
            public FaceWrapperForEvaluator(Face face)
            {
                this.face = face;
            }
            public string SurfaceType
            {
                get
                {
                    if (face.Surface is PlaneSurface) return "planar";
                    return "other";
                }
            }
            public int EdgeCount => face.AllEdges.Length;
            public BoundingBox bounds => face.GetExtent(0.0);
        }

        private class EdgeWrapperForEvaluator
        {
            Edge edge;
            public EdgeWrapperForEvaluator(Edge edge)
            {
                this.edge = edge;
            }
            public string CurveType
            {
                get
                {
                    if (edge.Curve3D is Line) return "line";
                    if (edge.Curve3D is Ellipse elli)
                    {
                        if (elli.IsCircle)
                        {
                            if (elli.IsClosed) return "circle";
                            else return "arc";
                        }
                        else
                        {
                            if (elli.IsClosed) return "ellipse";
                            else return "ellipse arc";
                        }
                    }
                    return "other";
                }
            }
            public GeoPoint startPoint => edge.Curve3D.StartPoint;
            public GeoPoint endPoint => edge.Curve3D.EndPoint;
            public GeoPoint pointAt(double u) => edge.Curve3D.PointAt(u);
            public GeoVector directionAt(double u) => edge.Curve3D.DirectionAt(u);
            public GeoVector startDirection => edge.Curve3D.StartDirection;
            public GeoVector endDirection => edge.Curve3D.EndDirection;
            public BoundingBox bounds => edge.Curve3D.GetExtent();
        }

        private class SolidWrapperForEvaluator
        {
            Solid solid;
            public SolidWrapperForEvaluator(Solid solid)
            {
                this.solid = solid;
            }

            public double Volume => solid.Shell.Volume(bounds.Size*1e-5);
            public BoundingBox bounds => solid.GetExtent(0.0);
        }

        private static object? wrapForEvaluator(object item)
        {
            // A named item holding exactly one object behaves like that object: a query result with a
            // single solid must answer 'this.Volume', not the properties of a List<Solid>. The rest of
            // the server (inspect.scene, workspace listing) already follows this convention.
            item = UnwrapSingletonList(item);
            if (item is Face fc) return new FaceWrapperForEvaluator(fc);
            if (item is Edge edg) return new EdgeWrapperForEvaluator(edg);
            if (item is Solid solid) return new SolidWrapperForEvaluator(solid);
            // TODO implement other wrappers
            return item;
        }

        #endregion

        #region Document, undo and workspace operations

        private JsonNode DocumentGetStateImpl()
        {
            // list all named workspace items so a client can (re-)orient itself mid-session
            var workspace = new JsonArray();
            foreach (var item in namedItems)
            {
                workspace.Add(DescribeNamedItem(item.Key, item.Value));
            }
            var templateNames = new JsonArray();
            foreach (string templateName in templates.Keys) templateNames.Add(templateName);
            // An undo frame left open swallows every later change - including changes the user makes
            // in the GUI - into that one step, so it must be visible here.
            JsonNode? openUndoFrame = currentUndoFrame == null ? null : new JsonObject
            {
                ["undoFrameId"] = currentUndoFrame.Id,
                ["label"] = currentUndoFrame.Label
            };
            return new JsonObject
            {
                ["openUndoFrame"] = openUndoFrame,
                ["docVersion"] = stateVersion,
                ["lengthUnit"] = "mm",
                ["workspace"] = workspace,
                ["templates"] = templateNames
            };
        }

        /// <summary>
        /// An undo frame opened by undo.begin. While it is open, every document change is collected
        /// into a single CADability undo step; the workspace snapshot allows undo.cancel to restore
        /// the named items as well, which the CADability undo system knows nothing about.
        /// </summary>
        private sealed class UndoFrameState
        {
            public readonly string Id;
            public readonly string Label;
            public readonly object Frame; // the object returned by UndoRedoSystem.OpenUndoFrame
            public readonly NamedItemsDictionary Workspace;
            /// <summary>True for a frame the server opened itself, e.g. for an atomic rpc.batch.</summary>
            public readonly bool IsInternal;
            public UndoFrameState(string id, string label, object frame, NamedItemsDictionary workspace, bool isInternal = false)
            {
                Id = id;
                Label = label;
                Frame = frame;
                Workspace = workspace;
                IsInternal = isInternal;
            }
        }

        // Undo frames are not nested here: CADability merges a nested frame into its parent, so a
        // cancel of the inner frame could no longer be rolled back on its own.
        private UndoFrameState? currentUndoFrame;

        private UndoRedoSystem RequireUndoSystem()
        {
            UndoRedoSystem? undo = project?.Undo;
            if (undo == null) throw new JsonRpcException("E_INTERNAL_ERROR", "Internal error: the project has no undo system.");
            return undo;
        }

        private UndoFrameState TakeUndoFrame(string undoFrameId)
        {
            if (currentUndoFrame == null)
                throw new JsonRpcException("E_INVALID_PARAMS", $"No undo frame is open, so '{undoFrameId}' cannot be closed. Open one with undo.begin first.");
            if (currentUndoFrame.Id != undoFrameId)
                throw new JsonRpcException("E_INVALID_PARAMS", $"The open undo frame is '{currentUndoFrame.Id}' (label '{currentUndoFrame.Label}'), not '{undoFrameId}'.");
            UndoFrameState frame = currentUndoFrame;
            currentUndoFrame = null;
            return frame;
        }

        /// <summary>
        /// Opens an undo frame and makes it the current one. The workspace snapshot is a flat copy:
        /// names are restored on a rollback, but objects modified in place rather than replaced keep
        /// their modified state.
        /// </summary>
        private UndoFrameState OpenUndoFrameState(string label, bool isInternal = false)
        {
            UndoRedoSystem undo = RequireUndoSystem();
            string id = "undo" + nextUndo++;
            UndoFrameState frame = new UndoFrameState(id, label, undo.OpenUndoFrame(), new NamedItemsDictionary(namedItems), isInternal);
            currentUndoFrame = frame;
            return frame;
        }

        /// <summary>
        /// Closes the frame and keeps its changes. Returns whether an undo step was created; an empty
        /// frame does not become one (see UndoRedoSystem.CloseUndoFrame).
        /// </summary>
        private bool CloseUndoFrameState(UndoFrameState frame)
        {
            bool undoStepCreated = frame.Frame is ArrayList al && al.Count > 0;
            RequireUndoSystem().CloseUndoFrame(frame.Frame);
            return undoStepCreated;
        }

        /// <summary>
        /// Closes the frame, undoes its document changes and restores the workspace snapshot.
        /// Returns whether document changes were actually undone.
        /// </summary>
        private bool RollBackUndoFrameState(UndoFrameState frame)
        {
            UndoRedoSystem undo = RequireUndoSystem();
            // Check for content BEFORE closing: only a non-empty frame is pushed onto the undo stack,
            // so undoing unconditionally would roll back an unrelated earlier step.
            bool hasDocumentChanges = frame.Frame is ArrayList al && al.Count > 0;
            undo.CloseUndoFrame(frame.Frame);
            bool documentChangesUndone = hasDocumentChanges && undo.UndoLastStep();
            if (hasDocumentChanges && !documentChangesUndone)
                AddCallWarning("The document changes could not be rolled back; the workspace was restored nevertheless.");
            // Restore the named items as they were when the frame was opened. Replacing the whole
            // dictionary fires no change notification, so bump the state counter by hand - a client
            // polling docVersion would otherwise miss the rollback.
            namedItems = frame.Workspace;
            AttachChangeTracking(namedItems);
            stateVersion++;
            return documentChangesUndone;
        }

        private JsonNode UndoBeginImpl(string label)
        {
            if (currentUndoFrame != null && currentUndoFrame.IsInternal)
                throw new JsonRpcException("E_INVALID_PARAMS", "The running rpc.batch is already a transaction, and undo frames cannot be nested. Drop the undo.begin/undo.end calls - the batch itself is rolled back on failure and becomes one undo step on success - or pass atomic=false to rpc.batch to manage the undo frame yourself.");
            if (currentUndoFrame != null)
                throw new JsonRpcException("E_INVALID_PARAMS", $"An undo frame is already open (undoFrameId '{currentUndoFrame.Id}', label '{currentUndoFrame.Label}'). Undo frames cannot be nested: close it with undo.end or undo.cancel first.");
            UndoFrameState frame = OpenUndoFrameState(label);
            return new JsonObject { ["undoFrameId"] = frame.Id, ["label"] = frame.Label };
        }

        private JsonNode UndoEndImpl(string undoFrameId)
        {
            UndoFrameState frame = TakeUndoFrame(undoFrameId);
            bool undoStepCreated = CloseUndoFrameState(frame);
            // the workspace snapshot is simply dropped: the current state is the intended result
            return new JsonObject
            {
                ["undoFrameId"] = frame.Id,
                ["label"] = frame.Label,
                ["undoStepCreated"] = undoStepCreated
            };
        }

        private JsonNode UndoCancelImpl(string undoFrameId)
        {
            UndoFrameState frame = TakeUndoFrame(undoFrameId);
            bool documentChangesUndone = RollBackUndoFrameState(frame);
            return new JsonObject
            {
                ["undoFrameId"] = frame.Id,
                ["label"] = frame.Label,
                ["documentChangesUndone"] = documentChangesUndone,
                ["workspaceRestored"] = true
            };
        }

        private void WorkspaceSetImpl(string name, JsonElement value, string? label, JsonElement input)
        {
            if (value.ValueKind == JsonValueKind.Number)
            {   // only get numbers here, expressions are evaluated below
                double d = GetOptionalDouble(value, null, double.NaN);
                if (!double.IsNaN(d))
                {
                    namedItems[name] = d;
                }
            }
            else if (value.ValueKind == JsonValueKind.String) // e.g. "v(1,2,3)" to define a vector
            {
                namedItems[name] = Evaluator.Evaluate(value.GetString(), namedItems.Dict);
            }
            else if (value.ValueKind == JsonValueKind.Object && value.TryGetProperty("expr", out var JeExpr) && JeExpr.ValueKind == JsonValueKind.String)
            {
                namedItems[name] = Evaluator.Evaluate(JeExpr.GetString(), namedItems.Dict);
            }
            else
            {
                List<object> selected = IterateSelector<object>(value).ToList();
                object? typedList = MakeTypedList(selected);
                if (typedList != null) namedItems[name] = typedList;


            }
        }

        private void WorkspaceSelectImpl(JsonElement selector, string name, string? type)
        {
            List<object> selected = IterateSelector<object>(selector).ToList();
            if (!string.IsNullOrEmpty(type))
            {   // An explicit type coerces the result and expands owners into their parts: a solid
                // yields its faces or edges, a face yields its edges. This makes it possible to name
                // a typed set without writing a query, and it fails fast when the selection contains
                // nothing of the requested type.
                selected = type switch
                {
                    "solids" => [.. ExpandToType<Solid>(selected)],
                    "faces" => [.. ExpandToType<Face>(selected)],
                    "edges" => [.. ExpandToType<Edge>(selected)],
                    "sketch_geometry" => [.. ExpandToType<ICurve2D>(selected), .. ExpandToType<CompoundShape>(selected).Cast<object>()],
                    _ => throw new JsonRpcException("E_INVALID_PARAMS", $"Unknown type '{type}'. Valid types are: solids, faces, edges, sketch_geometry.")
                };
                if (selected.Count == 0) throw new JsonRpcException("E_NOT_FOUND", $"The selection yielded no objects of type '{type}'. Nothing was stored.");
            }
            if (selected.Count > 1)
            {   // an edge or face reached through several owners must not appear twice
                HashSet<object> seen = new(ReferenceEqualityComparer.Instance);
                selected = selected.Where(seen.Add).ToList();
            }
            if (selected.Count == 0) throw new JsonRpcException("E_NOT_FOUND", "The selection yielded no objects. Nothing was stored.");
            if (selected.Count == 1)
            {   // store a single object directly so it can be used in expressions (e.g. 'e1.Length')
                namedItems[name] = selected[0];
            }
            else
            {   // store a typed list when all elements share a common type, otherwise the plain list
                namedItems[name] = MakeTypedList(selected) ?? selected;
            }
        }

        private void WorkspaceDeleteImpl(JsonElement objects)
        {
            List<string> names = [];
            if (objects.ValueKind == JsonValueKind.Array)
            {
                foreach (var el in objects.EnumerateArray())
                {
                    string? n = ParseObjectRef(el);
                    if (n != null) names.Add(n);
                }
            }
            else
            {
                string? n = ParseObjectRef(objects);
                if (n != null) names.Add(n);
            }
            foreach (string name in names)
            {
                // deleting is idempotent: a missing name is only a warning, not an error
                if (namedItems.ContainsKey(name)) namedItems.Remove(name);
                else AddCallWarning(NamedItemNotFound(name).Message + " Nothing was deleted for this name.");
            }
        }

        private JsonNode DocumentCommitObjectsImpl(JsonElement objects)
        {
            Project? project = FrameImpl.MainFrame?.Project; // TODO: project should be property of this
            if (project == null) throw new JsonRpcException("E_INTERNAL_ERROR", "Internal error: no active project.");
            Model model = project.GetActiveModel();
            Style style = project.StyleList.GetDefault(Style.EDefaultFor.Solids);
            // The result envelope only reports changes to namedItems, but committing touches the
            // document model instead. Report the committed names explicitly, otherwise the client
            // cannot tell a successful commit from a selector that resolved to nothing.
            JsonArray committed = new JsonArray();
            foreach (Solid sld in IterateSelector<Solid>(objects))
            {
                if (style != null) { sld.Style = style; }
                model.Add(sld);
                committed.Add(sld.Name);
            }
            if (committed.Count == 0) AddCallWarning("Nothing was committed: the selector did not resolve to any solid.");
            return new JsonObject { ["committed"] = committed };
        }

        private JsonNode DocumentUpdateObjectsImpl(JsonElement remove, JsonElement add)
        {
            Project? project = FrameImpl.MainFrame?.Project; // TODO: project should be property of this
            if (project == null) throw new JsonRpcException("E_INTERNAL_ERROR", "Internal error: no active project.");
            Model model = project.GetActiveModel();
            Style style = project.StyleList.GetDefault(Style.EDefaultFor.Solids);
            // Like document.commit_objects: report the affected names explicitly, because the
            // result envelope only covers namedItems and this tool changes the document model.
            JsonArray removedFromDocument = new JsonArray();
            JsonArray addedToDocument = new JsonArray();
            foreach (Solid sld in IterateSelector<Solid>(remove))
            {
                if (sld.Owner == model)
                {
                    model.Remove(sld);
                    removedFromDocument.Add(sld.Name);
                }
                else
                {
                    bool found = false;
                    foreach (IGeoObject go in model.AllObjects)
                    {
                        if (go is Solid sld2)
                        {
                            if (sld2.Name == sld.Name)
                            {
                                model.Remove(sld2);
                                removedFromDocument.Add(sld2.Name);
                                found = true;
                                break;
                            }
                        }
                    }
                    if (!found) AddCallWarning($"'{sld.Name}' was not removed: no matching object is present in the document.");
                }
            }
            foreach (Solid sld in IterateSelector<Solid>(add))
            {
                if (style != null) { sld.Style = style; }
                model.Add(sld);
                addedToDocument.Add(sld.Name);
            }
            return new JsonObject
            {
                ["removedFromDocument"] = removedFromDocument,
                ["addedToDocument"] = addedToDocument
            };
        }

        #endregion

        #region Sketch and profile operations

        private void SketchCreateImpl(Plane plane, string? name)
        {
            Sketch sketch = new Sketch(plane);
            if (name != null) namedItems[name] = sketch;
        }

        private void SketchCreateOnFaceImpl(JsonElement face, GeoPoint origin, GeoVector xAxis, string? name)
        {
            List<Face> lf = IterateSelector<Face>(face).ToList();
            if (lf.Count == 0) throw new JsonRpcException(-32602, "No face found for SketchCreateOnFace.");
            GeoVector xdir = xAxis;
            GeoPoint2D uv = lf[0].PositionOf(origin);
            GeoVector normal = lf[0].Surface.GetNormal(uv);
            GeoVector ydir = xdir ^ normal;
            Sketch? sketch = new Sketch(new Plane(origin, xdir, ydir));
            if (name != null) namedItems[name] = sketch;
        }

        private void SketchAddArcImpl(Sketch sketch, GeoPoint2D center, double radius, double startAngleDeg, double sweepAngleDeg, GeoPoint2D start, GeoPoint2D end, GeoPoint2D middle, bool ccw, string name)
        {
            ICurve2D? curve = null;
            if (center.IsValid && !double.IsNaN(radius) && !double.IsNaN(startAngleDeg) && !double.IsNaN(sweepAngleDeg))
            {
                curve = new Arc2D(center, radius, Angle.Deg(startAngleDeg), SweepAngle.Deg(sweepAngleDeg));
            }
            else if (center.IsValid && start.IsValid && end.IsValid)
            {
                curve = new Arc2D(center, center | start, start, end, ccw);
            }
            else if (middle.IsValid && start.IsValid && end.IsValid)
            {   // quick and dirty: use the existin 3d construction of an arc and project it back to 2d
                Ellipse tmp = Ellipse.Construct();
                Plane pln = sketch.Plane;
                tmp.SetArc3Points(pln.ToGlobal(start), pln.ToGlobal(middle), pln.ToGlobal(end), pln);
                curve = tmp.GetProjectedCurve(pln);
            }

            if (curve != null)
            {
                sketch.Add(curve);
                if (name != null) namedItems[name] = curve;
            }
            else throw new JsonRpcException(-32602, "Invalid parameters for sketch.add_arc (or not implemented).");
        }

        private void SketchAddCircleImpl(Sketch sketch, GeoPoint2D center, double radius, double diameter, string name)
        {
            if (double.IsNaN(radius) && double.IsNaN(diameter))
                throw new JsonRpcException(-32602, "Circle must have either radius or diameter.");
            if (double.IsNaN(radius)) radius = diameter / 2.0;
            ICurve2D curve = new Circle2D(center, radius);
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private object? GetSketchPointOrGeometry(JsonElement item)
        {
            string? expr = null;
            if (item.ValueKind == JsonValueKind.String) expr = item.GetString();
            if (item.ValueKind == JsonValueKind.Object)
            {
                if (item.TryGetProperty("name", out var nameEl))
                {
                    if (nameEl.ValueKind == JsonValueKind.String) expr = nameEl.GetString();
                }
                if (item.TryGetProperty("expr", out var exprEl))
                {
                    if (exprEl.ValueKind == JsonValueKind.String) expr = exprEl.GetString();
                }
                if (expr != null) return Evaluator.Evaluate(expr, namedItems.Dict);
                if (item.TryGetProperty("x", out var xEl) && item.TryGetProperty("y", out var yEl))
                {
                    return RequirePoint2D(item); // everything managed there
                }
            }
            if (item.ValueKind == JsonValueKind.Array) return RequirePoint2D(item); // everything managed there
            return null;
        }

        private void SketchAddCircleByConstraintsImpl(Sketch sketch, JsonElement constraints, double radius, GeoPoint2D center, GeoPoint2D preferredCenter, double tolerance, string name, JsonElement capture)
        {
            if (constraints.ValueKind != JsonValueKind.Array) throw new JsonRpcException("E_INVALID_PARAMETER", "constaints must be an array.");
            List<object> constaintObjects = [];
            foreach (var item in constraints.EnumerateArray())
            {
                object? constr = GetSketchPointOrGeometry(item);
                if (constr == null) throw new JsonRpcException("E_INVALID_PARAMETER", "object constraints not recognized.");
                constaintObjects.Add(constr);
            }
            Circle2D? circle = null;
            if (constaintObjects.Count == 3)
            {   // three tangents, may be curves or points
                List<ICurve2D> lc = [];
                List<GeoPoint2D> lp = [];
                List<GeoPoint2D> touchpoints = [];
                for (int i = 0; i < constaintObjects.Count; i++)
                {
                    if (constaintObjects[i] is ICurve2D c2d) lc.Add(c2d);
                    if (constaintObjects[i] is GeoPoint2D p2d) lp.Add(p2d);
                }
                if (lc.Count == 1)
                {
                    lc.Add(new Circle2D(lp[0], 0.0));
                    lc.Add(new Circle2D(lp[1], 0.0));
                }
                if (lc.Count == 2)
                {
                    lc.Add(new Circle2D(lp[0], 0.0));
                }
                if (lc.Count == 3)
                {   // tangential to 3 curves
                    GeoPoint2D[] circlePoints = Curves2D.TangentCircle(lc[0], lc[1], lc[2], GeoPoint2D.Invalid, GeoPoint2D.Invalid, GeoPoint2D.Invalid);
                    // the result is quadruples: center, point on first curve ...
                    int ind = -1;
                    if (preferredCenter.IsValid)
                    {
                        double minDist = double.MaxValue;
                        for (int j = 0; j < circlePoints.Length; j += 4)
                        {
                            double d = circlePoints[j] | preferredCenter;
                            if (d < minDist)
                            {
                                minDist = d;
                                ind = j;
                            }
                        }
                    }
                    else if (circlePoints.Length > 0) ind = 0;
                    circle = new Circle2D(circlePoints[ind], circlePoints[ind + 1] | circlePoints[ind]);
                    touchpoints.AddRange([circlePoints[ind + 1], circlePoints[ind + 2], circlePoints[ind + 3]]);
                }
                if (lp.Count == 3)
                {
                    if (Geometry.CircleFit(lp[0], lp[1], lp[2], out GeoPoint2D cnt, out double r))
                    {
                        circle = new Circle2D(cnt, r);
                    }
                    touchpoints.AddRange(lp); // 
                }
                if (circle == null) throw new JsonRpcException("E_OPERATION_FAILED", "could not construct circle from constraints.");
                sketch.Add(circle);
                if (name != null) namedItems[name] = circle;
                string? centerName = GetOptionalString(capture, "center");
                string? touch0Name = GetOptionalString(capture, "touch0");
                string? touch1Name = GetOptionalString(capture, "touch1");
                string? touch2Name = GetOptionalString(capture, "touch2");
                if (centerName != null) namedItems[centerName] = circle.Center;
                if (touch0Name != null) namedItems[touch0Name] = touchpoints[0];
                if (touch1Name != null) namedItems[touch1Name] = touchpoints[1];
                if (touch2Name != null) namedItems[touch2Name] = touchpoints[2];
            }
        }

        private void SketchAddLineByConstraintsImpl(Sketch sketch, GeoPoint2D start, JsonElement target0, JsonElement target1, GeoPoint2D preferredStart, GeoPoint2D preferredEnd, double tolerance, string name)
        {
            throw new NotImplementedException();
        }

        private void SketchSetCurveEndpointsImpl(Sketch sketch, JsonElement curve, GeoPoint2D start, GeoPoint2D end, string direction, bool projectToCurve, bool copy, string name, JsonElement capture)
        {
            List<ICurve2D> ca = IterateSelector<ICurve2D>(curve).ToList(); // should only be one
            if (ca.Count != 1) throw new JsonRpcException("E_INVALID_PARAMETER", "curve must contain exactely one curve.");
            ICurve2D crv = ca[0];
            if (copy || name != null) crv = crv.Clone();
            if (crv is Line2D l)
            {
                if (projectToCurve)
                {
                    if (start.IsValid)
                    {
                        GeoPoint2D sp = Geometry.DropPL(start, l.StartPoint, l.EndPoint);
                        l.StartPoint = sp;
                    }
                    if (end.IsValid)
                    {
                        GeoPoint2D ep = Geometry.DropPL(end, l.StartPoint, l.EndPoint);
                        l.EndPoint = ep;
                    }
                }
                else
                {
                    if (start.IsValid) l.StartPoint = start;
                    if (end.IsValid) l.EndPoint = end;
                }
            }
            if (crv is Circle2D c2d) // which is both circle and arc
            {
                if (!(c2d is Arc2D a2d))
                {
                    if (direction == null) direction = "shortest";
                    if (!start.IsValid || !end.IsValid) throw new JsonRpcException("E_INVALID_PARAMETER", "bot start and end must be set to make an arc from a circle.");
                    if ((direction == "ccw" && c2d.Sweep < 0) || (direction == "cw" && c2d.Sweep > 0)) c2d.Reverse();

                    a2d = new Arc2D(c2d.Center, c2d.Radius, start, end, c2d.Sweep > 0);
                }
                else
                {
                    if (!start.IsValid) start = a2d.StartPoint;
                    if (!end.IsValid) end = a2d.EndPoint;
                    a2d = new Arc2D(c2d.Center, c2d.Radius, start, end, direction == "cw");
                }
                if (direction == "shortest")
                {
                    if (Math.Abs(a2d.SweepAngle) > Math.PI) a2d.Complement();
                }
                if (direction == "longest")
                {
                    if (Math.Abs(a2d.SweepAngle) < Math.PI) a2d.Complement();
                }
                crv = a2d;
            }
            if (!sketch.Curves.Contains(crv)) sketch.Add(crv);
            if (name != null) namedItems[name] = crv;
        }

        private void SketchAddEllipseImpl(Sketch sketch, GeoPoint2D center, double radiusMajor, double radiusMinor, double rotationDeg, string name)
        {
            double major = radiusMajor;
            double minor = radiusMajor;
            double rotation = rotationDeg;
            ModOp2D rot = ModOp2D.Rotate(center, SweepAngle.Deg(rotation));
            GeoVector2D majAxis = rot * (major * GeoVector2D.XAxis);
            GeoVector2D minAxis = rot * (minor * GeoVector2D.YAxis);
            ICurve2D curve = new Ellipse2D(center, majAxis, minAxis);
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private void SketchAddEllipseArcImpl(Sketch sketch, GeoPoint2D center, double radiusMajor, double radiusMinor, double rotationDeg, double startAngleDeg, double sweepAngleDeg, string name)
        {
            throw new NotImplementedException();
        }

        private void SketchAddLineImpl(Sketch sketch, GeoPoint2D start, GeoPoint2D end, string name)
        {
            ICurve2D curve = new Line2D(start, end);
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private void SketchAddNurbsImpl(Sketch sketch, int degree, JsonElement controlPointsEl, JsonElement throughPointsEl, JsonElement knotsEl, JsonElement weightsEl, bool isPeriodic, JsonElement approximation, string name)
        {
            BSpline2D? curve = null;
            if (approximation.ValueKind == JsonValueKind.Object && approximation.TryGetProperty("parameter", out var _) && approximation.TryGetProperty("pointExpr", out var _))
            {
                string parameter = RequireString(approximation, "parameter");
                JsonElement pointExpr = RequireProperty(approximation, "pointExpr");
                string? xExpr = null, yExpr = null;
                if (pointExpr.ValueKind == JsonValueKind.Array)
                {
                    foreach (var xy in pointExpr.EnumerateArray())
                    {
                        if (xy.ValueKind == JsonValueKind.String)
                        {
                            if (xExpr == null) xExpr = xy.GetString();
                            else yExpr = xy.GetString();
                        }
                    }
                }
                else if (pointExpr.ValueKind == JsonValueKind.Object)
                {
                    xExpr = RequireString(pointExpr, "x");
                    yExpr = RequireString(pointExpr, "y");
                }
                if (xExpr == null || yExpr == null) throw new JsonRpcException("E_INVALID_PARAMS", $"Point must be defined as Expression depending on {parameter}");
                double tMin = RequireDouble(approximation, "tMin");
                double tMax = RequireDouble(approximation, "tMax");
                int minSamples = GetOptionalInteger(approximation, "minSamples", 4);
                int maxSamples = GetOptionalInteger(approximation, "maxSamples", 40);
                double tolerance = GetOptionalDouble(approximation, "tolerance", Precision.eps);
                namedItems.TryGetValue(parameter, out var oldValue);
                Func<double, GeoPoint2D> crv = (d) =>
                {
                    namedItems[parameter] = d;
                    double x = (double)Evaluator.Evaluate(xExpr, namedItems.Dict);
                    double y = (double)Evaluator.Evaluate(yExpr, namedItems.Dict);
                    return new GeoPoint2D(x, y);
                };
                // curve = BSpline2D.Approximate(crv, tolerance, tMin, tMax, maxSamples);
                // a problemhere: Approximate uses the parameters and the points. When crv has a different speed at the beginning and end
                // the resulting curve has bad conditions. We need an approximate without parameter synchronisation
                GeoPoint2D[] pnts = new GeoPoint2D[20];
                for (int i = 0; i < pnts.Length; i++)
                {
                    pnts[i] = crv(tMin + i * (tMax - tMin) / (pnts.Length - 1));
                }
                curve = new BSpline2D(pnts, 3, isPeriodic);
                if (oldValue == null) namedItems.Remove(parameter);
                else namedItems[parameter] = oldValue;
            }
            else if (throughPointsEl.ValueKind == JsonValueKind.Array)
            {
                List<GeoPoint2D> throughPoints = [];
                foreach (var pnt in throughPointsEl.EnumerateArray())
                {
                    throughPoints.Add(RequirePoint2D(pnt, null));
                }
                curve = new BSpline2D(throughPoints.ToArray(), degree, isPeriodic);
            }
            else
            {
                throw new NotImplementedException();
            }
            if (curve != null)
            {
                sketch.Add(curve);
                if (name != null) namedItems[name] = curve;
            }
        }

        private void SketchAddPolycurveImpl(Sketch sketch, JsonElement verticesEl, bool isClosed, string name)
        {
            if (verticesEl.ValueKind != JsonValueKind.Array)
                throw new JsonRpcException(-32602, "vertices must be array");

            List<(double x, double y, double b)> vertices = [];

            foreach (var seg in verticesEl.EnumerateArray())
            {
                double vx = RequireDouble(seg, "x");
                double vy = RequireDouble(seg, "y");
                double vb = GetOptionalDouble(seg, "b", 0.0);
                vertices.Add((vx, vy, vb));
            }

            List<ICurve2D> curves = [];
            for (int i = 0; i < vertices.Count; i++)
            {
                GeoPoint2D pn;

                if (i == vertices.Count - 1)
                {
                    if (!isClosed) continue;
                    pn = new GeoPoint2D(vertices[0].x, vertices[0].y);
                }
                else pn = new GeoPoint2D(vertices[i + 1].x, vertices[i + 1].y);
                GeoPoint2D pi = new GeoPoint2D(vertices[i].x, vertices[i].y);
                if (vertices[i].b == 0.0)
                {
                    curves.Add(new Line2D(pi, pn));
                }
                else
                {
                    GeoVector2D d = pn - pi;
                    double b = vertices[i].b;
                    double c = d.Length;
                    double r = Math.Abs(c * (1 + b * b) / (4 * b));
                    double h = c * (1 - b * b) / (4 * b);
                    GeoVector2D n = d.ToLeft();
                    GeoPoint2D center = new GeoPoint2D(pi, pn) + h / c * n;
                    curves.Add(new Arc2D(center, r, pi, pn, h >= 0));
                }
            }
            ICurve2D curve = new Path2D(curves.ToArray());
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private void SketchAddRectangleImpl(Sketch sketch, double width, double height, double cornerRadius, GeoPoint2D center, double rotationDeg, string name)
        {
            ICurve2D curve;
            if (cornerRadius == 0)
                curve = Polyline2D.MakeRectangle(center, width, height, SweepAngle.Deg(rotationDeg));
            else
                curve = Path2D.CreateRoundedRectangle(center, width, height, cornerRadius, SweepAngle.Deg(rotationDeg));
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private void SketchAddRegularPolygonImpl(Sketch sketch, GeoPoint2D center, double innerRadius, double outerRadius, int sides, double rotationDeg, string name)
        {
            if (double.IsNaN(outerRadius) || outerRadius == 0.0) outerRadius = innerRadius / Math.Cos(Math.PI / sides);
            ICurve2D curve = Polyline2D.MakeRegularPolygon(center, outerRadius, rotationDeg * Math.PI / 180.0, sides);
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private void SketchAddSlotImpl(Sketch sketch, GeoPoint2D center, double length, double width, double rotationDeg, string name)
        {
            GeoVector2D dir = GeoVector2D.XAxis;
            if (!double.IsNaN(rotationDeg)) dir = GeoVector2D.FromAngle(Angle.Deg(rotationDeg));
            double radius = width / 2;
            GeoPoint2D startPoint = center - (length / 2 - radius) * dir;
            GeoPoint2D endPoint = center + (length / 2 - radius) * dir;
            Border lh = Border.MakeLongHole(startPoint, endPoint, radius, radius);
            Path2D curve = lh.AsPath();
            sketch.Add(curve);
            if (name != null) namedItems[name] = curve;
        }

        private void SketchAddSolidSectionImpl(Sketch sketch, JsonElement solid, bool merge, string name)
        {
            List<Solid> solids = IterateSelector<Solid>(solid).ToList(); // should only be one
            List<ICurve> curves = [];
            foreach (Solid s in solids)
            {
                curves.AddRange(s.Shell.GetPlaneIntersection(new PlaneSurface(sketch.Plane)));
            }
            List<ICurve2D> curves2d = curves.Select(c => c.GetProjectedCurve(sketch.Plane)).ToList();
            CompoundShape cs = CompoundShape.CreateFromList(curves2d.ToArray(), Precision.eps, true, out _);
            sketch.Add(cs);
            if (name != null) namedItems[name] = cs;
        }

        private void SketchAddTextImpl(Sketch sketch, string text, GeoPoint2D location, double height, JsonElement font, JsonElement horizontalAlign, JsonElement verticalAlign, double characterSpacing, double wordSpacing, string name)
        {
            string fontFamily = RequireString(font, "family");
            bool bold = GetOptionalBool(font, "bold", false);
            bool italic = GetOptionalBool(font, "italic", false);
            bool underline = GetOptionalBool(font, "underline", false);
            bool strikeout = GetOptionalBool(font, "strikeout", false);


            Text goText = Text.Construct();
            goText.TextString = text;
            goText.Font = fontFamily;
            goText.Location = GeoPoint.Origin;
            goText.LineDirection = GeoVector.XAxis;
            goText.GlyphDirection = GeoVector.YAxis;

            CompoundShape[] shapes = goText.GetShapes();
            List<CompoundShape> res = [];
            for (int i = 0; i < shapes.Length; i++)
            {
                CompoundShape cs = shapes[i].GetModified(ModOp2D.Translate(location.x, location.y) * ModOp2D.Scale(height));
                res.Add(cs);
                sketch.Add(cs);
            }
            if (name != null) namedItems[name] = res;

        }

        private void SketchBooleanImpl(Sketch sketch, string op, JsonElement inputs, JsonElement subtract, string? name)
        {
            CompoundShape? result = null;
            List<CompoundShape> inputshapes = IterateSelector<CompoundShape>(inputs).ToList();
            List<ICurve2D> inputcurves = IterateSelector<ICurve2D>(inputs).ToList();
            for (int i = 0; i < inputcurves.Count; i++)
            {
                if (inputcurves[i].IsClosed)
                {
                    inputshapes.Add(new CompoundShape(new SimpleShape(new Border(inputcurves[i]))));
                }
            }
            List<CompoundShape> subtractshapes = [];
            if (subtract.ValueKind != JsonValueKind.Undefined)
            {
                subtractshapes = IterateSelector<CompoundShape>(subtract).ToList();
                List<ICurve2D> subtractcurves = IterateSelector<ICurve2D>(subtract).ToList();
                for (int i = 0; i < subtractcurves.Count; i++)
                {
                    if (subtractcurves[i].IsClosed)
                    {
                        subtractshapes.Add(new CompoundShape(new SimpleShape(new Border(subtractcurves[i]))));
                    }
                }
            }
            switch (op)
            {
                case "union":
                case "unite":
                    {
                        if (inputshapes.Count < 2) throw new JsonRpcException(-32602, "We need at least 2 shapes for union.");
                        result = inputshapes[0];
                        for (int i = 1; i < inputshapes.Count; i++)
                        {
                            result = CompoundShape.Union(result, inputshapes[i]);
                        }
                    }
                    break;
                case "intersect":
                    {
                        if (inputshapes.Count < 2) throw new JsonRpcException(-32602, "We need at least 2 shapes for intersection.");
                        result = inputshapes[0];
                        for (int i = 1; i < inputshapes.Count; i++)
                        {
                            result = CompoundShape.Intersection(result, inputshapes[i]);
                        }
                    }
                    break;
                case "subtract":
                case "difference":
                    {
                        if (inputshapes.Count != 1) throw new JsonRpcException(-32602, "We need at least 1 shape to subtract from.");
                        if (subtractshapes.Count < 1) throw new JsonRpcException(-32602, "We need at least 1 shape to subtract with.");
                        result = inputshapes[0];
                        for (int i = 0; i < subtractshapes.Count; i++)
                        {
                            result = CompoundShape.Difference(result, subtractshapes[i]);
                        }
                    }
                    break;
                default:
                    throw new JsonRpcException(-32601, $"Unknown sketch boolean operation '{op}'");
            }

            if (name != null && result != null)
            {
                result.UserData.Add("MCPServer.Sketch", sketch);
                StoreNamed(name, result);
            }

        }

        private ICurve2D ConnectToSinglePath(HashSet<ICurve2D> curves, double eps = 1e-8)
        {
            if (curves == null) throw new ArgumentNullException(nameof(curves));
            if (curves.Count == 0) throw new ArgumentException("Keine Kurven vorhanden.", nameof(curves));

            // Nicht direkt im HashSet arbeiten, falls Reverse() den Hash beeinflusst
            List<ICurve2D> work = curves.ToList();
            curves.Clear();

            while (work.Count > 1)
            {
                double bestDist = double.MaxValue;
                int bestI = -1;
                int bestJ = -1;
                int bestMode = -1;

                for (int i = 0; i < work.Count; i++)
                {
                    for (int j = i + 1; j < work.Count; j++)
                    {
                        ICurve2D a = work[i];
                        ICurve2D b = work[j];

                        Check(a.EndPoint, b.StartPoint, 0); // a -> b
                        Check(a.EndPoint, b.EndPoint, 1); // a -> reverse(b)
                        Check(a.StartPoint, b.StartPoint, 2); // reverse(a) -> b
                        Check(a.StartPoint, b.EndPoint, 3); // reverse(a) -> reverse(b)

                        void Check(GeoPoint2D p1, GeoPoint2D p2, int mode)
                        {
                            double d = p1 | p2;
                            if (d < bestDist)
                            {
                                bestDist = d;
                                bestI = i;
                                bestJ = j;
                                bestMode = mode;
                            }
                        }
                    }
                }

                ICurve2D first = work[bestI];
                ICurve2D second = work[bestJ];

                switch (bestMode)
                {
                    case 0:
                        // first.End -> second.Start
                        break;

                    case 1:
                        // first.End -> second.End
                        second.Reverse();
                        break;

                    case 2:
                        // first.Start -> second.Start
                        first.Reverse();
                        break;

                    case 3:
                        // first.Start -> second.End
                        first.Reverse();
                        second.Reverse();
                        break;
                }

                List<ICurve2D> parts = new List<ICurve2D>();
                parts.Add(first);

                if ((first.EndPoint | second.StartPoint) > eps)
                {
                    parts.Add(new Line2D(first.EndPoint, second.StartPoint));
                }

                parts.Add(second);

                ICurve2D combined = new Path2D(parts.ToArray());

                // Wichtig: höheren Index zuerst entfernen
                if (bestI > bestJ)
                {
                    work.RemoveAt(bestI);
                    work.RemoveAt(bestJ);
                }
                else
                {
                    work.RemoveAt(bestJ);
                    work.RemoveAt(bestI);
                }

                work.Add(combined);
            }

            curves.Add(work[0]);
            return work[0];
        }

        private void SketchConnectImpl(Sketch sketch, JsonElement entities, double precision, bool closeGaps, string name)
        {
            List<ICurve2D> toConnect = IterateSelector<ICurve2D>(entities).ToList();
            if (toConnect.Count < 2) throw new JsonRpcException(-32602, "We need at least 2 curves to connect.");
            Reduce2D r2d = new Reduce2D();
            r2d.OutputMode = Reduce2D.Mode.Paths;
            r2d.Add(toConnect.ToArray());
            ICurve2D[] reduced = r2d.Reduced;
            if (closeGaps)
            {
                ICurve2D c2d = ConnectToSinglePath(new HashSet<ICurve2D>(reduced), precision);
                if (!c2d.IsClosed)
                {
                    c2d = new Path2D(new ICurve2D[] { c2d, new Line2D(c2d.EndPoint, c2d.StartPoint) });
                }
                reduced = new ICurve2D[] { c2d };
            }
            if (name != null) namedItems[name] = reduced.ToList();
            if (sketch != null)
            {
                foreach (ICurve2D c in reduced)
                {
                    sketch.Add(c);
                }
            }
        }

        private void ProfileFromRegionsImpl(JsonElement regions, string? name)
        {   // regins are for extracting. There is no difference between CompoundShapes from a sketch and a region.
            // Maybe we should connect open curves.
            List<CompoundShape> inputshapes = new List<CompoundShape>();
            if (regions.ValueKind == JsonValueKind.Array)
            {
                foreach (var inp in regions.EnumerateArray())
                {
                    object obj = ResolveObjectRef(inp);
                    if (obj != null)
                    {
                        if (obj is ICurve2D c2d && c2d.IsClosed)
                        {
                            Border bdr = new Border(c2d);
                            CompoundShape cs = new CompoundShape(new SimpleShape(new Border(c2d)));
                            cs.UserData.Add("MCPServer.Sketch", c2d.UserData["MCPServer.Sketch"]);
                            inputshapes.Add(cs);
                        }
                        else if (obj is CompoundShape cs)
                        {
                            inputshapes.Add(cs);
                        }
                        else throw new JsonRpcException(-32602, "All regions must be closed shapes.");
                    }
                    else throw new JsonRpcException(-32602, "All regions must be sketch shapes.");
                }
            }
            if (name != null) namedItems[name] = inputshapes;
        }

        private List<SimpleShape> GetProfiles(JsonElement profile, out Sketch? sketch)
        {
            List<object> profiles = IterateSelector<object>(profile).ToList(); // should return a single sketch or a compoundShape or a closed curve
            sketch = null;
            List<SimpleShape> simpleShapes = new List<SimpleShape>();
            foreach (object obj in profiles)
            {
                if (obj is Sketch ps)
                {
                    sketch = ps;
                    CompoundShape? pcs = sketch.GetCompoundShape();
                    if (pcs != null) simpleShapes.AddRange(pcs.SimpleShapes);
                }
                else if (obj is CompoundShape cs)
                {
                    sketch = cs.UserData["MCPServer.Sketch"] as Sketch;
                    simpleShapes.AddRange(cs.SimpleShapes);
                }
                else if (obj is List<CompoundShape> cslist)
                {
                    for (int i = 0; i < cslist.Count; i++)
                    {
                        if (sketch == null) sketch = cslist[i].UserData["MCPServer.Sketch"] as Sketch;
                        foreach (SimpleShape simpleShape in cslist[i].SimpleShapes)
                        {
                            simpleShapes.Add(simpleShape);
                        }
                    }
                }
                else if (obj is ICurve2D c2d && c2d.IsClosed)
                {
                    if (sketch == null) sketch = c2d.UserData["MCPServer.Sketch"] as Sketch;
                    simpleShapes.Add(new SimpleShape(new Border(c2d)));
                }
                else throw new JsonRpcException(-32602, "Profile must be a sketch shape.");
            }
            return simpleShapes;
        }

        List<ICurve> GetSketchCurves(JsonElement selector)
        {
            List<CompoundShape> lcs = IterateSelector<CompoundShape>(selector).ToList();
            List<ICurve2D> lc2 = IterateSelector<ICurve2D>(selector).ToList();
            List<ICurve> res = [];
            if (lc2.Count > 0)
            {
                Sketch? sketch = lc2[0].UserData["MCPServer.Sketch"] as Sketch;
                if (sketch == null) throw new JsonRpcException("E_INTERNAL_ERROR", "No sketch assoziated with curve.");

                Reduce2D r2d = new Reduce2D();
                r2d.Add(lc2.ToArray());
                r2d.OutputMode = Reduce2D.Mode.Paths;
                foreach (ICurve2D curve2D in r2d.Reduced)
                {
                    ICurve? toAdd = curve2D.MakeGeoObject(sketch.Plane) as ICurve;
                    if (toAdd != null) res.Add(toAdd);
                }
            }
            if (lcs.Count > 0)
            {
                Sketch? sketch = lcs[0].UserData["MCPServer.Sketch"] as Sketch;
                if (sketch == null) throw new JsonRpcException("E_INTERNAL_ERROR", "No sketch assoziated with profile.");
                foreach (CompoundShape cs in lcs)
                {
                    res.AddRange(cs.MakePaths(sketch.Plane));
                }
            }
            return res;
        }

        private void PatternCircularSketchImpl(Sketch sketch, JsonElement entities, GeoPoint2D center, int count, double angle, bool merge, string name, bool nameWithSuffix)
        {
            // only implemented for closed shapes for now, which we convert to CompoundShape for easier boolean operations. We can add support for open curves later if needed.
            List<CompoundShape> inputshapes = new List<CompoundShape>();
            if (entities.ValueKind == JsonValueKind.Array)
            {
                foreach (var inp in entities.EnumerateArray())
                {
                    object obj = ResolveObjectRef(inp);
                    if (obj != null)
                    {
                        foreach (object oo in IterateListOrSingleObject(obj))
                        {
                            if (oo is ICurve2D c2d && c2d.IsClosed)
                            {
                                Border bdr = new Border(c2d);
                                inputshapes.Add(new CompoundShape(new SimpleShape(new Border(c2d))));
                            }
                            else if (oo is CompoundShape cs)
                            {
                                cs.UserData.Add("MCPServer.Sketch", sketch);
                                inputshapes.Add(cs);
                            }
                            else throw new JsonRpcException(-32602, "All inputs must be closed shapes.");
                        }
                    }
                    else throw new JsonRpcException(-32602, "All inputs must be sketch shapes.");
                }
            }
            List<CompoundShape> resultshapes = new List<CompoundShape>();
            if (double.IsNaN(angle)) angle = 360;
            SweepAngle angleStep = SweepAngle.Deg(angle / count);
            ModOp2D rot = ModOp2D.Rotate(center, angleStep);
            resultshapes.AddRange(inputshapes); // first not rotatet
            for (int i = 1; i < count; i++)
            {
                List<CompoundShape> newshapes = new List<CompoundShape>();
                foreach (var s in inputshapes)
                {
                    CompoundShape copy = s.GetModified(rot);
                    newshapes.Add(copy);
                }
                inputshapes = newshapes; // already rotated for next iteration
                resultshapes.AddRange(newshapes);
            }
            if (merge)
            {
                CompoundShape? merged = null;
                foreach (var s in resultshapes)
                {
                    if (merged == null) merged = s;
                    else merged = CompoundShape.Union(merged, s);
                }
                if (name != null && merged != null) StoreNamed(name, merged);
                if (merged != null) sketch.Add(merged);
            }
            else
            {
                foreach (var s in resultshapes) sketch.Add(s);
                if (name != null) StoreNamed(name, resultshapes); // a List of CompoundShape
            }
        }

        private void SketchRoundVerticesImpl(Sketch? sketch, JsonElement entity, double radius, JsonElement nearPoints, JsonElement indices, double tolerance, string name)
        {
            if (nearPoints.ValueKind == JsonValueKind.Undefined || indices.ValueKind == JsonValueKind.Undefined)
            {
                throw new NotImplementedException("Vertex selection for sketch.round_vertices not implemented.");
            }
            List<object> entities = IterateObjectRefs(entity).ToList();
            if (entities.Count == 0 && sketch != null)
            {
                entities.Add(sketch.Curves);
                entities.Add(sketch.Shapes);
            }
            if (entities.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "No entities provided for rounding edges.");
            for (int i = 0; i < entities.Count; i++)
            {
                Path2D? p2d = null;
                CompoundShape? cs = null;
                string? currentName = null;
                if (entities[i] is Path2D ep2d)
                {
                    if (sketch == null) sketch = ep2d.UserData["MCPServer.Sketch"] as Sketch;
                    currentName = FindName(ep2d);
                    p2d = ep2d;
                }
                else if (entities[i] is ICurve2D c2d)
                {
                    if (sketch == null) sketch = c2d.UserData["MCPServer.Sketch"] as Sketch;
                    currentName = FindName(c2d);
                    p2d = new Path2D([c2d]);
                }
                else if (entities[i] is CompoundShape ecs)
                {
                    if (sketch == null) sketch = ecs.UserData["MCPServer.Sketch"] as Sketch;
                    currentName = FindName(ecs);
                    throw new NotImplementedException("round vertices of sketch shape");
                }
                if (p2d != null)
                {
                    Path2D rounded = p2d.RoundVertices(radius);
                    if (rounded != null)
                    {
                        rounded.UserData.Add("MCPServer.Sketch", sketch);
                        if (string.IsNullOrEmpty(name) && currentName != null) namedItems[currentName] = rounded;
                        else if (name != null) namedItems[name] = rounded;
                    }
                }
                if (cs != null)
                {
                    CompoundShape rounded = cs.RoundVertices(radius);
                    if (rounded != null)
                    {
                        rounded.UserData.Add("MCPServer.Sketch", sketch);
                        if (string.IsNullOrEmpty(name) && currentName != null) namedItems[currentName] = rounded;
                        else if (name != null) namedItems[name] = rounded;
                    }
                }
            }

        }

        private void PatternGridSketchImpl(Sketch sketch, JsonElement entities, int countX, int countY, JsonElement stepX, JsonElement stepY, bool merge, string name, bool nameWithSuffix)
        {
            throw new NotImplementedException();
        }

        private void SketchOffsetImpl(Sketch sketch, JsonElement sketchGeometry, double distance, string joinType, double miterLimit, bool makeRegion, string capType, string name)
        {
            object? pathOrShape = null;
            if (sketchGeometry.ValueKind == JsonValueKind.Object)
            {
                List<CompoundShape> inputshapes = IterateSelector<CompoundShape>(sketchGeometry).ToList();
                List<ICurve2D> inputcurves = IterateSelector<ICurve2D>(sketchGeometry).ToList();
                if (inputshapes.Count > 0)
                {
                    pathOrShape = inputshapes[0];
                }
                else if (inputcurves.Count > 0) pathOrShape = inputcurves[0];
            }
            else
            {   // from sketch
                if (sketch.Shapes.Count > 0) { pathOrShape = sketch.Shapes[0]; }
                else if (sketch.Curves.Count > 0)
                {
                    if (sketch.Curves.Count == 1) pathOrShape = sketch.Curves[0];
                }
                else if (sketch.Curves.Count > 1)
                {
                    // combine all curves to a path?
                }
            }
            if (pathOrShape == null) throw new JsonRpcException("E_INVALID_PARAMS", "No input found to offset.");
            if (double.IsNaN(miterLimit)) miterLimit = Math.PI;
            object? result = null;
            if (pathOrShape is ICurve2D c2d)
            {
                result = c2d.Parallel(distance, true, Precision.eps, miterLimit);
                if (makeRegion && !c2d.IsClosed && result is ICurve2D rc2d)
                {
                    rc2d.Reverse();
                    Border bdr = new Border([c2d, new Line2D(c2d.EndPoint, rc2d.StartPoint), rc2d, new Line2D(rc2d.EndPoint, c2d.StartPoint)], true, true);
                    result = new CompoundShape(new SimpleShape(bdr));
                }
            }
            else if (pathOrShape is CompoundShape cs)
            {
                if (distance > 0) result = cs.Expand(distance);
                else result = cs.Shrink(distance);
            }
            if (result == null) throw new JsonRpcException("E_OPERATION_FAILED", "Failed to calculate offset.");
            if (name != null) namedItems[name] = result;
            if (result != null)
            {
                if (result is ICurve2D ic2d) sketch.Add(ic2d);
                if (result is CompoundShape cs) sketch.Add(cs);
            }
        }

        private void SketchGetVerticesImpl(Sketch sketch, JsonElement sketchGeometry, string name, bool suffix, bool includeEndpoints, bool unique, double tolerance)
        {
            throw new NotImplementedException();
        }

        private void SketchPerpendicularThroughImpl(Sketch sketch, JsonElement curve, GeoPoint2D point, double length, string name)
        {
            List<ICurve2D> curves = IterateSelector<ICurve2D>(curve).ToList(); // should only be one
            if (curves.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be exactly one curve in 'curve'.");
            GeoPoint2D[] ftpts = curves[0].PerpendicularFoot(point);
            if (ftpts.Length == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Failed to find foot point.");
            GeoPoint2D ftpt = ftpts.MinBy(f => f | point); // in case there are multiple foot points, we take the one closest to the given point);
            GeoVector2D dir = ftpt - point;
            if (dir.IsNullVector()) dir = curves[0].DirectionAt(curves[0].PositionOf(ftpt)).Normalized.ToLeft();
            Line2D nl = new Line2D(ftpt, ftpt + length * dir.Normalized);
            sketch.Add(nl);
            if (name != null) namedItems[name] = nl;
        }

        private void SketchFootPointOnCurveImpl(Sketch sketch, GeoPoint2D point, JsonElement curve, string mode, bool clamp, string name, JsonElement captured)
        {
            List<ICurve2D> c = IterateSelector<ICurve2D>(curve).ToList(); // should only be one
            if (c.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be exactly one curve in 'curve'.");
            GeoPoint2D[] ftpts = c[0].PerpendicularFoot(point);
            if (ftpts.Length == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Failed to find foot point.");
            GeoPoint2D ftpt = ftpts.MinBy(f => f | point); // in case there are multiple foot points, we take the one closest to the given point);
            if (name != null) namedItems[name] = ftpt;
        }

        private void SketchIntersectionsImpl(Sketch sketch, JsonElement a, JsonElement b, string mode, double tolerance, string name, bool suffix)
        {
            List<ICurve2D> ca = IterateSelector<ICurve2D>(a).ToList(); // should only be one
            List<ICurve2D> cb = IterateSelector<ICurve2D>(b).ToList(); // should only be one
            if (ca.Count != 1 || cb.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be exactly one curve in 'a' and one curve in 'b'.");
            GeoPoint2DWithParameter[] ips = ca[0].Intersect(cb[0]);
            if (ips == null || ips.Length == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Failed to find intersection points.");
            List<GeoPoint2D> points = ips.Select(ip => ip.p).ToList();
            if (name != null)
            {
                if (points.Count == 1)
                {
                    namedItems[name] = points[0];
                }
                else
                {
                    if (suffix)
                    {
                        for (int i = 0; i < points.Count; i++)
                        {
                            namedItems[$"{name}_{i}"] = points[i];
                        }
                    }
                    namedItems[name] = points;
                }
            }
        }

        private void SketchAngleBisectorImpl(Sketch sketch, JsonElement a, JsonElement b, string which, GeoPoint2D at, double length, string name)
        {
            List<Line2D> linea = IterateSelector<Line2D>(a).ToList(); // should only be one
            List<Line2D> lineb = IterateSelector<Line2D>(b).ToList(); // should only be one
            if (linea.Count != 1 || lineb.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be one line in a and one line in b.");
            if (!Geometry.IntersectLL(linea[0].StartPoint, linea[0].StartDirection, lineb[0].StartPoint, lineb[0].StartDirection, out GeoPoint2D intersectionPoint)) throw new JsonRpcException("E_INVALID_PARAMS", "The lines don't intersect.");

            GeoVector2D dir1 = linea[0].StartDirection.Normalized + lineb[0].StartDirection.Normalized;
            GeoVector2D dir2 = linea[0].StartDirection.Normalized - lineb[0].StartDirection.Normalized;
            GeoVector2D dir = GeoVector2D.NullVector;
            if (which != null)
            {
                switch (which)
                {
                    case "acute":
                    case "inner":
                        dir = dir1.Length < dir2.Length ? dir2.Normalized : dir1.Normalized;
                        break;
                    case "obtuse":
                    case "outer":
                        dir = dir1.Length > dir2.Length ? dir2.Normalized : dir1.Normalized;
                        break;
                    case "ccw":
                        dir = GeoVector2D.Orientation(dir, dir2) > 0 ? dir1.Normalized : dir2.Normalized;
                        break;
                    case "cw":
                        dir = GeoVector2D.Orientation(dir, dir2) < 0 ? dir1.Normalized : dir2.Normalized;
                        break;
                }
            }

            Line2D res = new Line2D(intersectionPoint, intersectionPoint + length * dir);
            sketch.Add(res);
            if (name != null) namedItems[name] = res;

        }

        public class Sketch : IJsonSerialize
        {
            Plane plane;
            List<ICurve2D> curves = [];
            List<CompoundShape> shapes = [];

            public Sketch(Plane plane)
            {
                this.plane = plane;
            }
            public void Add(ICurve2D curve)
            {
                curve.UserData.Add("MCPServer.Sketch", this);
                curves.Add(curve);
            }

            public void Add(CompoundShape shape)
            {
                shape.UserData.Add("MCPServer.Sketch", this);
                shapes.Add(shape);
            }

            internal CompoundShape? GetCompoundShape()
            {
                if (curves.Count == 0 && shapes.Count == 1) return shapes[0];
                if (curves.Count == 0 && shapes.Count == 0) return null;
                if (shapes.Count > 0)
                {   // we must somhow combine the compound shapes 
                    CompoundShape? shape = shapes[0];
                    for (int i = 1; i < shapes.Count; i++)
                    {
                        shape = CompoundShape.Union(shape, shapes[i]);
                    }
                    shape.UserData.Add("MCPServer.Sketch", this);
                    return shape;
                }
                if (curves.Count == 1 && curves[0].IsClosed && shapes.Count == 0) return new CompoundShape(new SimpleShape(new Border(curves[0])));
                if (curves.Count > 1)
                {
                    List<SimpleShape> simpleShapes = new List<SimpleShape>();
                    for (int i = 0; i < curves.Count; i++)
                    {
                        if (curves[i].IsClosed) simpleShapes.Add(new SimpleShape(new Border(curves[i])));
                    }
                    // we should check all SimpleShapes against each other.
                    // but for now, quick and dirty
                    simpleShapes.Sort((a, b) => b.Area.CompareTo(a.Area));
                    CompoundShape? res = null;
                    for (int i = 0; i < simpleShapes.Count; i++)
                    {
                        if (simpleShapes[i] == null) continue;
                        CompoundShape cs = new CompoundShape(simpleShapes[i]);
                        for (int j = i + 1; j < simpleShapes.Count; j++)
                        {
                            if (simpleShapes[j] == null) continue;
                            if (SimpleShape.GetPosition(simpleShapes[i], simpleShapes[j]) == SimpleShape.Position.firstcontainscecond)
                            {
                                cs = CompoundShape.Difference(cs, new CompoundShape(simpleShapes[j]));
                                simpleShapes[j] = null; // mark as used
                            }
                        }
                        if (res == null) res = cs;
                        else res = CompoundShape.Union(res, cs);
                    }
                    res.UserData.Add("MCPServer.Sketch", this);
                    return res;
                }
                return null;
            }

            protected Sketch() { } // for IJsonSerialize
            public void GetObjectData(IJsonWriteData data)
            {
                data.AddProperty("Plane", plane);
                data.AddProperty("Curves", curves);
                data.AddProperty("Shapes", shapes);
            }

            public void SetObjectData(IJsonReadData data)
            {
                plane = data.GetProperty<Plane>("Plane");
                curves = data.GetProperty<List<ICurve2D>>("Curves");
                shapes = data.GetProperty<List<CompoundShape>>("Shapes");
            }

            public Plane Plane => plane;
            public List<ICurve2D> Curves => curves;
            public List<CompoundShape> Shapes => shapes;
        }

        #endregion

        #region Solid, surface and feature operations

        private void SolidExtrudeImpl(JsonElement profile, double length, GeoVector direction, double offset, string? name, JsonElement capture)
        {
            List<SimpleShape> simpleShapes = GetProfiles(profile, out Sketch? sketch);

            if (sketch != null)
            {
                string? startEdges = GetOptionalString(capture, "startEdges");
                string? endEdges = GetOptionalString(capture, "endEdges");
                string? startFace = GetOptionalString(capture, "startFace");
                string? endFace = GetOptionalString(capture, "endFace");
                List<Solid> solids = new List<Solid>();
                PlaneSurface ps = new PlaneSurface(sketch.Plane);
                GeoVector dir = direction.IsValid() ? direction : ps.Normal.Normalized;
                dir.Length = length;
                for (int i = 0; i < simpleShapes.Count; i++)
                {
                    Face face = Face.MakeFace(ps, simpleShapes[i]);
                    if (face != null)
                    {
                        if (offset != 0.0) face.Modify(ModOp.Translate(offset * dir.Normalized));
                        Solid? sld = Make3D.Extrude(face, dir, null) as Solid;
                        if (sld != null)
                        {
                            if (startEdges != null || startFace != null)
                            {
                                GeoPoint2D point2dOnFace = face.Area.GetSomeInnerPoint();
                                GeoPoint point3dOnFace = face.Surface.PointAt(point2dOnFace);
                                Face startFaceOfExtrusion = sld.FindFace(point3dOnFace);
                                if (!string.IsNullOrEmpty(startFace))
                                {
                                    namedItems[startFace] = startFaceOfExtrusion;
                                }
                                if (!string.IsNullOrEmpty(startEdges))
                                {
                                    namedItems[startEdges] = new List<Edge>(startFaceOfExtrusion.Edges); ;
                                }
                            }
                            if (endEdges != null || endFace != null)
                            {
                                GeoPoint2D point2dOnFace = face.Area.GetSomeInnerPoint();
                                GeoPoint point3dOnFace = face.Surface.PointAt(point2dOnFace) + dir;
                                Face endFaceOfExtrusion = sld.FindFace(point3dOnFace);
                                if (!string.IsNullOrEmpty(endFace))
                                {
                                    namedItems[endFace] = endFaceOfExtrusion;
                                }
                                if (!string.IsNullOrEmpty(endEdges))
                                {
                                    namedItems[endEdges] = new List<Edge>(endFaceOfExtrusion.Edges); ;
                                }
                            }
                            solids.Add(sld);
                        }
                    }
                }
                if (name != null) namedItems[name] = solids;
            }
        }

        private void SolidHelicalExtrudeImpl(JsonElement profile, Axis axis, double angle, double offset, double pitch, string name, JsonElement capture)
        {
            List<SimpleShape> simpleShapes = GetProfiles(profile, out Sketch? sketch);

            if (sketch != null)
            {
                string? startEdges = GetOptionalString(capture, "startEdges");
                string? endEdges = GetOptionalString(capture, "endEdges");
                string? startFace = GetOptionalString(capture, "startFace");
                string? endFace = GetOptionalString(capture, "endFace");
                List<Solid> solids = new List<Solid>();
                PlaneSurface ps = new PlaneSurface(sketch.Plane);
                for (int i = 0; i < simpleShapes.Count; i++)
                {
                    Face face = Face.MakeFace(ps, simpleShapes[i]);
                    if (face != null)
                    {
                        Shell shl = Make3D.MakeHelicalSolid(face, axis, pitch, pitch * angle / 360, 0.0, true);
                        if (shl != null)
                        {
                            Solid sld = Solid.MakeSolid(shl);
                            if (sld != null)
                            {
                                if (startEdges != null || startFace != null)
                                {
                                    GeoPoint2D point2dOnFace = face.Area.GetSomeInnerPoint();
                                    GeoPoint point3dOnFace = face.Surface.PointAt(point2dOnFace);
                                    Face startFaceOfExtrusion = sld.FindFace(point3dOnFace);
                                    if (!string.IsNullOrEmpty(startFace))
                                    {
                                        namedItems[startFace] = startFaceOfExtrusion;
                                    }
                                    if (!string.IsNullOrEmpty(startEdges))
                                    {
                                        namedItems[startEdges] = new List<Edge>(startFaceOfExtrusion.Edges);
                                    }
                                }
                                if (endEdges != null || endFace != null)
                                {
                                }
                                solids.Add(sld);
                            }
                        }
                    }
                }
                if (name != null) namedItems[name] = solids;
            }
        }

        private void SolidBoxImpl(GeoPoint origin, GeoVector axisX, GeoVector axisY, double sizeX, double sizeY, double sizeZ, string name)
        {
            GeoVector axisZ;
            if (axisX.IsValid() && axisY.IsValid())
            {
                axisZ = axisX ^ axisY;
                axisX.Norm();
                axisY.Norm();
                axisZ.Norm();
            }
            else
            {
                axisX = GeoVector.XAxis;
                axisY = GeoVector.YAxis;
                axisZ = GeoVector.ZAxis;
            }
            Solid res = Make3D.MakeBox(origin, sizeX * axisX, sizeY * axisY, sizeZ * axisZ);
            if (name != null) namedItems[name] = res;
        }

        private void SolidCapsuleImpl(GeoPoint start, GeoPoint end, double radius, string cap, double coneTipDistance, string name)
        {
            Solid? res = null;
            double l = end | start; // the length of the capsule
            Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
            Plane profilePlane = new Plane(start, end - start, pln.DirectionX); // in this plane we construct a profile for rotation along the x-axis of the plane
            GeoVector dirx = radius * pln.ToGlobal(GeoVector2D.XAxis);
            Solid cylinder = Make3D.MakeCylinder(start, dirx, end - start);
            if (double.IsNaN(coneTipDistance))
            {
                // sphericalTips
                Arc2D arcstart = new Arc2D(GeoPoint2D.Origin, radius, Angle.Deg(180), SweepAngle.Deg(90));
                Arc2D arcend = new Arc2D(new GeoPoint2D(l, 0), radius, Angle.Deg(270), SweepAngle.Deg(90));
                Line2D line1 = new Line2D(arcstart.EndPoint, arcend.StartPoint);
                Line2D line2 = new Line2D(arcend.EndPoint, arcstart.StartPoint);
                Path2D profile = new Path2D(new ICurve2D[] { arcstart, line1, arcend, line2 });
                Path? profile3D = profile.MakeGeoObject(profilePlane) as Path;
                var rotated = Make3D.Rotate(profile3D, new Axis(start, end), SweepAngle.Full, 0.0, null);
                if (rotated is Solid sld) res = sld;
            }
            else
            {
                Solid cone1 = Make3D.MakeCone(start, dirx, coneTipDistance * (start - end).Normalized, radius, 0.0);
                Solid cone2 = Make3D.MakeCone(end, dirx, coneTipDistance * (end - start).Normalized, radius, 0.0);
                res = BooleanOperation.Unite(cone1, cylinder);
                res = BooleanOperation.Unite(cone2, res);
            }
            if (res != null && name != null) namedItems[name] = res;
        }

        private void SolidConeImpl(GeoPoint start, GeoPoint end, double radiusStart, double radiusEnd, string name)
        {
            Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
            GeoVector dirx = pln.ToGlobal(GeoVector2D.XAxis);
            Solid res = Make3D.MakeCone(start, dirx, end - start, radiusStart, radiusEnd);
            if (res != null && name != null) namedItems[name] = res;
        }

        private void SolidCylinderImpl(GeoPoint start, GeoPoint end, double radius, string name)
        {
            Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
            GeoVector dirx = radius * pln.ToGlobal(GeoVector2D.XAxis);
            Solid res = Make3D.MakeCylinder(start, dirx, end - start);
            if (res != null && name != null) namedItems[name] = res;
        }

        private void SolidPipeImpl(GeoPoint start, GeoPoint end, double outerRadius, double innerRadius, string name)
        {
            if (innerRadius == 0)
            {
                SolidCylinderImpl(start, end, outerRadius, name);
                return;
            }
            Solid? res = null;
            Plane pln = new Plane(start, end - start); // to use the arbitrary axis algorithm
            GeoVector dirx = outerRadius * pln.ToGlobal(GeoVector2D.XAxis);
            Solid cylinder1 = Make3D.MakeCylinder(start, dirx, end - start);
            dirx = innerRadius * pln.ToGlobal(GeoVector2D.XAxis);
            Solid cylinder2 = Make3D.MakeCylinder(start, dirx, end - start);
            Solid[] diff = BooleanOperation.Subtract(cylinder1, cylinder2);
            if (diff != null && diff.Length == 1) res = diff[0];
            if (res != null && name != null) namedItems[name] = res;
        }

        private void SolidSphereImpl(GeoPoint center, double radius, JsonElement points, string name)
        {
            Solid? res = Make3D.MakeSphere(center, radius);
            if (res != null && name != null) namedItems[name] = res;
        }

        private void SolidTorusImpl(GeoPoint center, GeoVector axis, double majorRadius, double minorRadius, string name)
        {
            Plane pln = new Plane(center, axis); // to use the arbitrary axis algorithm
            Solid? res = Make3D.MakeTorus(center, axis, majorRadius, minorRadius);
            if (res != null && name != null) namedItems[name] = res;
        }

        private void SurfaceNurbsImpl(int degreeU, int degreeV, JsonElement controlPoints, JsonElement uKnots, JsonElement vKnots, JsonElement weights, bool uPeriodic, bool vPeriodic, string name)
        {
            throw new NotImplementedException();
        }

        private void SurfaceParametricImpl(int degreeU, int degreeV, JsonElement approximation, string name)
        {
            int minSamplesU = RequireInteger(approximation, "minSamplesU");
            int minSamplesV = RequireInteger(approximation, "minSamplesV");

            string uParameter = RequireString(approximation, "uParameter");
            string vParameter = RequireString(approximation, "vParameter");
            string xExpr = RequireString(approximation, "xExpr");
            string yExpr = RequireString(approximation, "yExpr");
            string zExpr = RequireString(approximation, "zExpr");
            double uMin = RequireDouble(approximation, "uMin");
            double uMax = RequireDouble(approximation, "uMax");
            double vMin = RequireDouble(approximation, "vMin");
            double vMax = RequireDouble(approximation, "vMax");
            double tolerance = RequireDouble(approximation, "tolerance");
            bool uPeriodic = RequireBool(approximation, "uPeriodic");
            bool vPeriodic = RequireBool(approximation, "vPeriodic");

            GeoPoint[,] throughPoints = new GeoPoint[minSamplesU, minSamplesV];
            double du = (uMax - uMin) / (minSamplesU - 1);
            double dv = (vMax - vMin) / (minSamplesV - 1);
            for (int i = 0; i < minSamplesU; i++)
            {
                for (int j = 0; j < minSamplesV; ++j)
                {
                    using var uu = new NamedItemOverride(namedItems, uMin + i * du, uParameter);
                    using var vv = new NamedItemOverride(namedItems, vMin + j * dv, vParameter);
                    double x = (double)Evaluator.Evaluate(xExpr, namedItems.Dict);
                    double y = (double)Evaluator.Evaluate(yExpr, namedItems.Dict);
                    double z = (double)Evaluator.Evaluate(zExpr, namedItems.Dict);
                    throughPoints[i, j] = new GeoPoint(x, y, z);
                }
            }
            NurbsSurface ns = new NurbsSurface(throughPoints, degreeU, degreeV, uPeriodic, vPeriodic);
            BoundingRect ext = new BoundingRect(ns.UKnots.First(), ns.VKnots.First(), ns.UKnots.Last(), ns.VKnots.Last());
            ns.SetBounds(ext);
            namedItems[name] = ns;
        }

        private void SolidRuledImpl(JsonElement profile1, JsonElement profile2, string synchronization, string alignment, JsonElement matchPoints1, JsonElement matchPoints2, string name)
        {
            Path? path1 = null, path2 = null;
            List<ICurve2D> curves1 = IterateSelector<ICurve2D>(profile1).ToList();
            if (curves1.Count > 0)
            {
                if (curves1.Count > 1) throw new JsonRpcException("E_INVALID_PARAMETER", $"we need exactely one closed profile in profile1.");
                Sketch? sketch = curves1[0].UserData.GetData("MCPServer.Sketch") as Sketch;
                if (sketch == null) throw new JsonRpcException("E_INTERNAL_ERROR", "Sketch not found for profile1.");
                IGeoObject go = curves1[0].MakeGeoObject(sketch.Plane);
                if (go is Path p) path1 = p;
                else if (go is ICurve crv)
                {
                    path1 = Path.FromSegments(new ICurve[] { crv }, true);
                    path1.Flatten();
                }
                else throw new JsonRpcException("E_INVALID_PARAMETER", $"profile1 must be closed.");
            }
            List<ICurve2D> curves2 = IterateSelector<ICurve2D>(profile2).ToList();
            if (curves2.Count > 0)
            {
                if (curves2.Count > 1) throw new JsonRpcException("E_INVALID_PARAMETER", $"we need exactely one closed profile in profile2.");
                Sketch? sketch = curves2[0].UserData.GetData("MCPServer.Sketch") as Sketch;
                if (sketch == null) throw new JsonRpcException("E_INTERNAL_ERROR", "Sketch not found for profile2.");
                IGeoObject go = curves2[0].MakeGeoObject(sketch.Plane);
                if (go is Path p) path2 = p;
                else if (go is ICurve crv)
                {
                    path2 = Path.FromSegments(new ICurve[] { crv }, true);
                }
                else throw new JsonRpcException("E_INVALID_PARAMETER", $"profile2 must be closed.");
            }
            if (path1 != null && path2 != null)
            {
                Solid sld = Make3D.MakeRuledSolid(path1, path2, null);
                if (sld != null)
                {
                    if (name != null) namedItems[name] = sld;
                }
                else throw new JsonRpcException("E_OPERATION_FAILED", "Could not create ruled solid.");
            }
        }

        private void SolidSweepImpl(JsonElement profile, JsonElement path, string? orientation, string? name, JsonElement capture)
        {
            List<CompoundShape> profiles = GetProfiles(profile);
            List<ICurve> paths = GetSketchCurves(path);
            if (profiles.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be exactely one profile.");
            if (paths.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be exactely one path.");
            Sketch? sketch = profiles[0].UserData["MCPServer.Sketch"] as Sketch;
            if (sketch == null) throw new JsonRpcException("E_INTERNAL_ERROR", "No sketch assoziated with profile.");

            Face toSweep = Face.MakeFace(new PlaneSurface(sketch.Plane), profiles[0].SimpleShapes[0]); // the profile should not consist of multiple SimpleShapes
            if (!(paths[0] is Path)) paths[0] = Path.FromSegments(paths)[0]; // there must be at least one!
            Path? p = paths[0] as Path;
            if (p != null)
            {
                IGeoObject sweptSolid = Make3D.MakePipe(toSweep, p, null);
                if (sweptSolid is Solid sld)
                {
                    if (name != null) namedItems[name] = sld;
                    if (capture.ValueKind != JsonValueKind.Undefined)
                    {
                        string? startEdgesName = GetOptionalString(capture, "startEdges");
                        string? endEdgesName = GetOptionalString(capture, "endEdges");
                        string? startFaceName = GetOptionalString(capture, "startFace");
                        string? endFaceName = GetOptionalString(capture, "endFace");

                        Face endFace = (toSweep.Clone() as Face)!;
                        endFace.Modify(ModOp.Fit(p.StartPoint, [p.StartDirection], p.EndPoint, [p.EndDirection]));
                        Face? startingFace = sld.Shell.FindSimilarFace(toSweep);
                        Face? endingFace = sld.Shell.FindSimilarFace(endFace);
                        if (startingFace != null)
                        {
                            if (startFaceName != null) namedItems[startFaceName] = startingFace; // there should only be one
                            if (startEdgesName != null) namedItems[startEdgesName] = startingFace.AllEdges.ToList();
                        }
                        if (endingFace != null)
                        {
                            if (endFaceName != null) namedItems[endFaceName] = endingFace; // there should only be one
                            if (endEdgesName != null) namedItems[endEdgesName] = endingFace.AllEdges.ToList();
                        }
                    }
                }
            }
        }

        private void SolidRotateImpl(JsonElement profile, Axis axis, double angle, string? name, JsonElement capture)
        {
            List<CompoundShape> profiles = GetProfiles(profile);
            List<Solid> res = [];
            for (int i = 0; i < profiles.Count; i++)
            {
                Sketch? sketch = profiles[i].UserData["MCPServer.Sketch"] as Sketch;
                if (sketch == null) throw new JsonRpcException("E_OPERATION_FAILED", "No suitable sketch found for profile");
                for (int j = 0; j < profiles[i].SimpleShapes.Length; j++)
                {
                    Face toRotate = Face.MakeFace(new PlaneSurface(sketch.Plane), profiles[i].SimpleShapes[j]);
                    IGeoObject go = Make3D.Rotate(toRotate, axis, SweepAngle.Deg(angle), 0, null);
                    if (go is Solid sld) res.Add(sld);
                }
            }
            if (profiles.Count == 0)
            {
                List<ICurve> crvs = GetSketchCurves(profile);
                for (int i = 0; i < crvs.Count; i++)
                {
                    // crvs[i] is not closed here, otherwise it would have been a profile in profiles
                    Line l1 = Line.TwoPoints(crvs[i].EndPoint, Geometry.DropPL(crvs[i].EndPoint, axis.Location, axis.Direction));
                    Line l3 = Line.TwoPoints(Geometry.DropPL(crvs[i].StartPoint, axis.Location, axis.Direction), crvs[i].StartPoint);
                    Line l2 = Line.TwoPoints(l1.EndPoint, l3.StartPoint);
                    Face toRotate = Face.MakeFace(new GeoObjectList(crvs[i] as IGeoObject, l1, l2, l3));
                    if (toRotate != null)
                    {
                        IGeoObject go = Make3D.Rotate(toRotate, axis, SweepAngle.Deg(angle), 0, null);
                        if (go is Solid sld) res.Add(sld);
                    }
                }
            }
            if (name != null) namedItems[name] = res;
        }

        private IEnumerable<object> IterateListOrSingleObject(object obj)
        {
            if (obj is IEnumerable<object> list)
            {
                foreach (var item in list) yield return item;
            }
            else
            {
                yield return obj;
            }
        }

        private Solid UniteWithMany(Solid a, List<Solid> b, out List<Solid> unused)
        {
            Solid accumulate = a;
            HashSet<Solid> bb = [.. b];
            while (bb.Count > 0)
            {
                bool united = false;
                foreach (Solid sld in bb.Clone())
                {
                    Solid tmp = NewBooleanOperation.Unite(sld, accumulate);
                    if (tmp != null)
                    {
                        accumulate = tmp;
                        bb.Remove(sld);
                        united = true;
                    }
                }
                if (!united) break; // there is nothing we could unite with, so we are done
            }
            unused = bb.ToList();
            return accumulate;
        }

        private List<Solid> SubtractMany(List<Solid> toSubtractFrom, List<Solid> subtractItems)
        {
            List<Solid> fragments = new List<Solid>();
            fragments.AddRange(toSubtractFrom);
            foreach (Solid sld in subtractItems)
            {
                List<Solid> newfragments = new List<Solid>();
                for (int i = 0; i < fragments.Count; i++)
                {
                    Solid[] res = NewBooleanOperation.Subtract(fragments[i], sld);
                    if (res != null && res.Length > 0)
                    {
                        newfragments.AddRange(res);
                    }
                    else
                    {
                        newfragments.Add(fragments[i]);
                    }
                }
                fragments = newfragments;
            }
            return fragments;
        }

        Solid[] IntersectMany(Solid solid, List<Solid> other)
        {
            List<Solid> fragments = new List<Solid>();
            fragments.Add(solid);
            foreach (Solid sld in other)
            {
                List<Solid> newfragments = new List<Solid>();
                for (int i = 0; i < fragments.Count; i++)
                {
                    Solid[] res = NewBooleanOperation.Intersect(fragments[i], sld);
                    if (res != null && res.Length > 0)
                    {
                        newfragments.AddRange(res);
                    }
                    else
                    {
                        newfragments.Add(fragments[i]);
                    }
                }
                fragments = newfragments;
            }
            return fragments.ToArray();
        }

        private void SolidBooleanImpl(string op, JsonElement a, JsonElement b, string? name, bool rebind, JsonElement rebindTargets)
        {
            List<Solid> slda = IterateSelector<Solid>(a).ToList();
            List<Solid> sldb = IterateSelector<Solid>(b).ToList();
            if (slda.Count == 0 || sldb.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "Boolean operations require at least one solid 'a' and at least one other solid 'b' to operate with.");
            // Without a name the result inherits the name of operand 'a'. Take it from the parameter:
            // the name stashed in UserData is only written on the ResolveObjectRef path (IterateSelector
            // never sets it) and it goes stale as soon as the workspace entry is reassigned.
            if (name == null) name = FirstName(a);
            object? res = null;
            Solid s1 = slda[0];
            List<Solid> s2 = [.. sldb, .. slda.Skip(1)];
            // if there are more than one solid in a, we also treat them as solids to operate with, so we add them to the list of b solids. This is a bit unintuitive but it allows for more complex operations without needing to call boolean multiple times. For example, if you want to unite 3 solids, you can just put them all in a and leave b empty.
            var sw = Stopwatch.StartNew();
            switch (op.ToLower())
            {
                case "union":
                case "unite":
                    {
                        Solid sld = UniteWithMany(s1, s2, out List<Solid> unused);
                        if (sld != null && unused.Count > 0)
                        {
                            // we were not able to unite with all solids, so we return the result as a list of solids (the united one and the ones we could not unite with)
                            List<Solid> resList = [sld];
                            while (unused.Count > 1)
                            {
                                s1 = unused[0];
                                s2 = [.. unused.Skip(1)];
                                sld = UniteWithMany(s1, s2, out unused);
                                resList.Add(sld);
                            }
                            resList.AddRange(unused);
                            res = resList;
                        }
                        else res = sld;
                    }
                    break;
                case "difference":
                case "subtract":
                    {
                        res = SubtractMany(slda, sldb);
                    }
                    break;
                case "intersect":
                    {
                        res = new List<Solid>(IntersectMany(s1, s2));
                    }
                    break;
                default: throw new JsonRpcException("E_INVALID_PARAMS", $"'solid.boolean' unknown operator {op}");
            }
            sw.Stop();
            if (res != null && name != null) namedItems[name] = res;
            if (rebind)
            {
                if (res is Solid sres)
                {
                    Rebind(slda, sres);
                    Rebind(sldb, sres);
                }
                else if (res is List<Solid> lres)
                {
                    Rebind(slda, lres);
                    Rebind(sldb, lres);
                }
            }
        }

        private void PatternCircularSolidsImpl(JsonElement objects, Axis axis, int count, double angle, bool copy, string name, bool suffix)
        {

            List<Solid> list = IterateObjectRefs<Solid>(objects).ToList(); // all objects assiziated via userdat by name
            double rotationAngle = double.IsNaN(angle) ? 360 : angle;
            if (rotationAngle <= Precision.eps) throw new JsonRpcException("E_INVALID_PARAMS", "Angle must be greater than 0.");
            double stepAngle = rotationAngle / count;
            List<Solid> current = new List<Solid>();
            foreach (Solid s in list) current.Add(s.Clone() as Solid);
            List<Solid> total = new List<Solid>(current);
            ModOp rot = ModOp.Rotate(axis.Location, axis.Direction, SweepAngle.Deg(stepAngle));
            for (int i = 1; i < count; i++)
            {
                List<Solid> next = new List<Solid>();
                foreach (Solid s in current)
                {
                    Solid clone = s.Clone() as Solid;
                    if (clone != null)
                    {
                        clone.Modify(rot);
                        next.Add(clone);
                    }
                }
                // One entry per rotation step. When several solids are patterned at once, the
                // whole step is stored as a list (the indexer then names the members
                // "<name>_<i>_0", "<name>_<i>_1", ...); a single solid is stored directly so it
                // stays usable in expressions.
                if (suffix && name != null && next.Count > 0)
                {
                    namedItems[$"{name}_{i}"] = next.Count == 1 ? (object)next[0] : next;
                }
                total.AddRange(next);
                current = next;
            }
            if (name != null) namedItems[name] = total;
        }

        private void PatternGridSolidsImpl(JsonElement objects, int countX, int countY, int countXNegative, int countYNegative, GeoVector stepX, GeoVector stepY, bool copy, string? name, bool suffix)
        {
            GeoVector sx = stepX;
            if (sx.Length <= Precision.eps) throw new JsonRpcException("E_INVALID_PARAMS", "stepX must be a non-zero vector.");
            GeoVector sy = stepY.IsValid() ? stepY : GeoVector.NullVector;
            if (countXNegative > 0) countXNegative = -countXNegative; // make sure countXNegative is negative or zero (usually provided positiv)
            if (countYNegative > 0) countYNegative = -countYNegative; // make sure countYNegative is negative or zero (usually provided positiv)
            if (countY - countYNegative > 0 && sy.Length <= Precision.eps) throw new JsonRpcException("E_INVALID_PARAMS", "stepY must be a non-zero vector when y counts are provided");
            List<Solid> list = IterateObjectRefs<Solid>(objects).ToList(); // all objects to pattern
            List<Solid> total = new List<Solid>();
            for (int ix = countXNegative; ix <= countX; ix++)
            {
                for (int iy = countYNegative; iy <= countY; iy++)
                {
                    GeoVector moveVec = ix * sx;
                    if (iy != 0) moveVec += iy * sy;
                    ModOp move = ModOp.Translate(moveVec);
                    List<Solid> subList = [];
                    List<Solid> unnamedSources = [];
                    foreach (Solid s in list)
                    {
                        Solid? clone = s.Clone() as Solid;
                        if (clone != null)
                        {
                            clone.Modify(move);
                            subList.Add(clone);
                            if (suffix)
                            {
                                string? sn = FindName(s);
                                if (sn != null) namedItems[$"{sn}_{ix}_{iy}"] = clone;
                                else if (name != null) unnamedSources.Add(clone);
                            }
                        }
                    }
                    // Clones of source solids without an own name share one entry per grid cell.
                    if (unnamedSources.Count > 0)
                    {
                        namedItems[$"{name}_{ix}_{iy}"] = unnamedSources.Count == 1 ? (object)unnamedSources[0] : unnamedSources;
                    }
                    total.AddRange(subList);
                }
            }
            if (name != null) namedItems[name] = total;
        }

        private void PatternByFormulaSolidsImpl(JsonElement solids, string template, JsonElement variables, JsonElement formulas, string? condition, JsonElement arguments, string transform, bool includeSource, bool copy, string? name, bool suffix, string? indexName, bool skipInvalidInstances)
        {
            List<Solid> solidsToInsert = [];
            solidsToInsert = IterateSelector<Solid>(solids).ToList();
            if (solidsToInsert.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "No solids for the pattern found.");
            List<(string name, double start, double step, int count)> loopVariables = [];
            if (variables.ValueKind != JsonValueKind.Array) throw new JsonRpcException("E_INVALID_PARAMS", "'variables' must be an array.");
            foreach (var variable in variables.EnumerateArray())
            {
                if (!(variable.TryGetProperty("name", out var nameEl) && nameEl.ValueKind == JsonValueKind.String))
                    throw new JsonRpcException("E_INVALID_PARAMS", "'variable' must have a 'name' proerty.");
                string varname = nameEl.GetString()!;
                if (!(variable.TryGetProperty("start", out var startEl)))
                    throw new JsonRpcException("E_INVALID_PARAMS", $"'variable' {name} must have a 'start' proerty.");
                double start = double.NaN;
                if (startEl.ValueKind == JsonValueKind.Number) start = startEl.GetDouble();
                if (startEl.ValueKind == JsonValueKind.String) start = (double)Evaluator.Evaluate(startEl.GetString(), namedItems.Dict);
                if (double.IsNaN(start)) throw new JsonRpcException("E_INVALID_PARAMS", $"could not evaluate start value of variable {name}.");
                if (!(variable.TryGetProperty("step", out var stepEl)))
                    throw new JsonRpcException("E_INVALID_PARAMS", $"'variable' {name} must have a 'step' proerty.");
                double step = double.NaN;
                if (stepEl.ValueKind == JsonValueKind.Number) step = stepEl.GetDouble();
                if (stepEl.ValueKind == JsonValueKind.String) step = (double)Evaluator.Evaluate(stepEl.GetString(), namedItems.Dict);
                if (double.IsNaN(step)) throw new JsonRpcException("E_INVALID_PARAMS", $"could not evaluate step value of variable {name}.");
                if (!(variable.TryGetProperty("count", out var countEl)))
                    throw new JsonRpcException("E_INVALID_PARAMS", $"'variable' {name} must have a 'count' proerty.");
                int count = 0;
                if (countEl.ValueKind == JsonValueKind.Number) count = countEl.GetInt32();
                if (countEl.ValueKind == JsonValueKind.String)
                {
                    object eva = Evaluator.Evaluate(countEl.GetString(), namedItems.Dict);
                    if (eva is double d) count = (int)d;
                    if (eva is int i) count = i;
                }
                if (count <= 0) throw new JsonRpcException("E_INVALID_PARAMS", $"could not evaluate count value of variable {name}.");
                loopVariables.Add((varname, start, step, count));
            }
            List<Solid> result = [];
            IterateLoops(loopVariables, () =>
            {
                if (formulas.ValueKind == JsonValueKind.Object)
                {
                    foreach (var item in formulas.EnumerateObject())
                    {
                        if (item.Value.ValueKind == JsonValueKind.String)
                        {
                            namedItems[item.Name] = Evaluator.Evaluate(item.Value.GetString(), namedItems.Dict);
                        }
                    }
                }
                if (condition != null)
                {
                    if (Evaluator.Evaluate(condition, namedItems.Dict) is bool ok)
                    {
                        if (!ok) return;
                    }
                }
                ModOp transforModOp = ModOp.Identity;
                if (Evaluator.Evaluate(transform, namedItems.Dict) is ModOp t) transforModOp = t;
                if (template != null)
                {
                    object? templInst = TemplateInstantiateImpl(template, arguments, transform, null, false);
                    if (templInst is Solid sld)
                    {
                        sld.Modify(transforModOp);
                        result.Add(sld);
                    }
                    else if (templInst is List<Solid> lsld)
                    {
                        foreach (Solid solid in lsld) solid.Modify(transforModOp);
                        result.AddRange(lsld);

                    }
                }
                else if (solidsToInsert.Count > 0)
                {
                    for (int i = 0; i < solidsToInsert.Count; i++)
                    {
                        var clone = solidsToInsert[i].Clone();
                        clone.Modify(transforModOp);
                        result.Add((clone as Solid)!);
                    }
                }
            });
            if (name != null) namedItems[name] = result;
            if (suffix)
            {
                string? n = indexName != null ? indexName : name;
                if (n != null)
                {
                    for (int i = 0; i < result.Count; i++)
                    {
                        namedItems[n + "_" + i.ToString()] = result[i];
                    }
                }
            }
        }

        private void FeatureHoleImpl(JsonElement solid, JsonElement face, JsonElement centerOnFace, GeoPoint center, double diameter, bool through, double depth, string? name, JsonElement capture, bool rebind, JsonElement rebindTargets)
        {
            List<Solid> solids = IterateSelector<Solid>(solid).ToList(); // should only be one
            List<Face> faces = IterateSelector<Face>(face).ToList(); // should only be one
            if (solids.Count != 1 && faces.Count != 1) throw new JsonRpcException("E_INVALID_PARAMS", "There must be one solid and one face in 'feature.hole'.");
            Face onFace = faces[0];
            Solid onSolid = solids[0];
            GeoPoint holeCenter = center;
            GeoPoint2D uv = onFace.PositionOf(holeCenter);
            holeCenter = onFace.Surface.PointAt(uv);
            GeoVector holeDir = -onFace.Surface.GetNormal(uv).Normalized;
            double holeDepth = 0.0;
            if (through)
            {
                GeoPoint[] ips = onSolid.Shells[0].GetLineIntersection(holeCenter, holeDir);
                // now the length of the cylinder to remove may be longer than the first intersection
                // the best result would be to intersect a long cylinder (bbox) with the solid and find the part
                // which is closest to holeCenter.
                double minLength = double.MaxValue;
                foreach (GeoPoint ip in ips)
                {
                    double par = Geometry.LinePar(holeCenter, holeDir, ip);
                    if (par > Precision.eps && par < minLength) minLength = par;
                }
                if (minLength != double.MaxValue) holeDepth = minLength;
                else holeDepth = onSolid.GetBoundingCube().DiagonalLength;
            }
            else
            {
                holeDepth = depth;
            }
            double radius = diameter / 2;
            Plane pln = new Plane(holeCenter, holeDir); // arbitrary axis
            Solid cyl = Make3D.MakeCylinder(holeCenter, radius * pln.DirectionX, holeDepth * holeDir);
            if (cyl != null)
            {
                // Without a name the original is replaced. Take that name from the 'solid' parameter:
                // a reverse lookup of onSolid fails whenever the workspace item is a list of solids.
                string resName = string.IsNullOrEmpty(name) ? RequireSingleName(solid, "solid") : name;
                Solid[] res = NewBooleanOperation.Subtract(onSolid, cyl);
                if (res != null && res.Length > 0)
                {
                    if (res.Length == 1)
                    {
                        namedItems[resName] = res[0];
                    }
                    else
                    {
                        namedItems[resName] = new List<Solid>(res);
                    }
                    if (capture.ValueKind != JsonValueKind.Undefined)
                    {
                        string? entryEdgesName = null, exitEdgesName = null, wallFacesName = null;
                        if (capture.TryGetProperty("entryEdges", out var entryEl)) entryEdgesName = entryEl.GetString();
                        if (capture.TryGetProperty("exitEdges", out var exitEl)) exitEdgesName = exitEl.GetString();
                        if (capture.TryGetProperty("wallFaces", out var wallEl)) wallFacesName = wallEl.GetString();

                        List<Face> cylindricalFaces = [];
                        foreach (Face fc in cyl.Shells[0].Faces)
                        {
                            if (fc.Surface is ICylinder) cylindricalFaces.Add(fc);
                        }
                        if (entryEdgesName != null || exitEdgesName != null)
                        {
                            List<Edge> entryEdges = [];
                            List<Edge> exitEdges = [];
                            foreach (Edge edge in res[0].Edges)
                            {
                                foreach (Face fc in cylindricalFaces)
                                {
                                    if (Precision.IsNull(fc.Distance(edge.Vertex1.Position))
                                        && Precision.IsNull(fc.Distance(edge.Vertex2.Position))
                                        && fc.Surface.IsCurveOnSurface(edge.Curve3D))
                                    {   // either entry or exit
                                        if (Precision.IsNull(onFace.Distance(edge.Vertex1.Position))
                                        && Precision.IsNull(onFace.Distance(edge.Vertex2.Position))
                                        && onFace.Surface.IsCurveOnSurface(edge.Curve3D))
                                            entryEdges.Add(edge);
                                        else exitEdges.Add(edge);
                                        break;
                                    }
                                }
                            }
                            if (entryEdgesName != null) namedItems[entryEdgesName] = entryEdges;
                            if (exitEdgesName != null) namedItems[exitEdgesName] = exitEdges;
                        }
                        if (wallFacesName != null)
                        {
                            List<Face> wallFaces = [];
                            foreach (Face fc in cylindricalFaces)
                            {
                                foreach (Face fc1 in res[0].Shells[0].Faces)
                                {
                                    if (fc.SameSurface(fc1)) wallFaces.Add(fc1);
                                }
                            }
                            namedItems[wallFacesName] = wallFaces;
                        }
                    }
                    if (rebind) Rebind(onSolid, res);
                }
            }
        }

        private void FeatureSplitImpl(JsonElement solid, JsonElement splitBy, string nameInner, string nameOuter, bool rebind, JsonElement rebindTargets)
        {
            List<Solid> solids = IterateSelector<Solid>(solid).ToList(); // should only be one
            Plane pln = Plane.Invalid;
            Shell? shell = null;
            if (splitBy.TryGetProperty("standard", out var _) || splitBy.TryGetProperty("origin", out var _))
            {
                pln = RequirePlane(splitBy);
            }
            else
            {
                List<ISurface> surfaces = IterateSelector<ISurface>(splitBy).ToList(); // should only be one
                                                                                       // make a shell from this surface
                List<Face> faces = [];
                for (int i = 0; i < surfaces.Count; i++)
                {
                    BoundingRect ext = surfaces[i].GetBounds();
                    // TODO: both u and v are periodic!
                    if (surfaces[i].IsUPeriodic && ext.Width > surfaces[i].UPeriod * 0.9)
                    {
                        BoundingRect extl = new BoundingRect(ext);
                        extl.Right = ext.Left + ext.Width / 2;
                        BoundingRect extr = new BoundingRect(ext);
                        extr.Left = ext.Left + ext.Width / 2;
                        faces.Add(Face.MakeFace(surfaces[i].Clone(), extl));
                        faces.Add(Face.MakeFace(surfaces[i].Clone(), extr));
                    }
                    else if (surfaces[i].IsVPeriodic && ext.Height > surfaces[i].VPeriod * 0.9)
                    {
                        BoundingRect extb = new BoundingRect(ext);
                        extb.Top = ext.Bottom + ext.Height / 2;
                        BoundingRect extt = new BoundingRect(ext);
                        extt.Bottom = ext.Bottom + ext.Height / 2;
                        faces.Add(Face.MakeFace(surfaces[i].Clone(), extb));
                        faces.Add(Face.MakeFace(surfaces[i].Clone(), extt));
                    }
                    else
                    {
                        faces.Add(Face.MakeFace(surfaces[i].Clone(), ext));
                    }
                }
                Shell[] shells = Make3D.SewFaces(faces.ToArray());
                if (shells.Length > 0) shell = shells[0]; // should only be one
            }
            if (nameOuter != null)
            {
                List<Solid> res = [];
                for (int i = 0; i < solids.Count; i++)
                {
                    if (pln.IsValid()) res.AddRange(BooleanOperation.SplitSolidByPlane(solids[i], pln, true));
                    else if (shell != null) res.AddRange(BooleanOperation.SplitSolidByShell(solids[i], shell, true));
                }
                if (res.Count == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Splitting reveald no outer part.");
                namedItems[nameOuter] = res;
            }
            if (nameInner != null)
            {
                List<Solid> res = [];
                if (pln.IsValid()) pln.Reverse();
                else if (shell != null) shell.ReverseOrientation();
                for (int i = 0; i < solids.Count; i++)
                {
                    if (pln.IsValid()) res.AddRange(BooleanOperation.SplitSolidByPlane(solids[i], pln, true));
                    else if (shell != null) res.AddRange(BooleanOperation.SplitSolidByShell(solids[i], shell, true));
                }
                if (res.Count == 0) throw new JsonRpcException("E_OPERATION_FAILED", "Splitting reveald no inner part.");
                namedItems[nameInner] = res;
            }
        }

        private void FeatureChamferImpl(JsonElement solid, JsonElement edges, double distance, JsonElement primaryFace, double secondaryDistance, string? name, bool rebind, JsonElement rebindTargets)
        {
            List<Edge> edgesToRound = IterateSelector<Edge>(edges).ToList();
            if (edgesToRound.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "No edges found to fillet.");
            Shell? shell = edgesToRound.First().Owner.Owner as Shell;
            if (shell == null) throw new JsonRpcException("E_INVALID_PARAMS", "Edge is not part of a solid.");
            // The geometry comes from the edges, 'solid' only names the workspace item to replace when
            // no new name is given. Check it before doing the work so a mismatch fails fast.
            string solidName = RequireSolidNameFor(solid, shell);
            if (double.IsNaN(secondaryDistance)) secondaryDistance = distance;
            // maybe flip distances
            ChamferEdges ce = new ChamferEdges(shell, edgesToRound, distance, secondaryDistance);
            Shell? rounded = ce.Execute();
            if (rounded == null) throw new JsonRpcException("E_OPERATION_FAILED", "Filletting failed.");
            namedItems[string.IsNullOrEmpty(name) ? solidName : name] = Solid.MakeSolid(rounded);
            if (rebind) Rebind(shell, rounded);
        }

        private void FeatureFilletImpl(JsonElement solid, JsonElement edges, double radius, string? name, bool rebind, JsonElement rebindTargets)
        {
            List<Edge> edgesToRound = IterateSelector<Edge>(edges).ToList();
            if (edgesToRound.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "No edges found to fillet.");
            Shell? shell = edgesToRound.First().Owner.Owner as Shell;
            if (shell == null) throw new JsonRpcException("E_INVALID_PARAMS", "Edge is not part of a solid.");
            // The geometry comes from the edges, 'solid' only names the workspace item to replace when
            // no new name is given. Check it before doing the work so a mismatch fails fast.
            string solidName = RequireSolidNameFor(solid, shell);
            RoundEdges re = new RoundEdges(shell, edgesToRound, radius);
            Shell? rounded = re.Execute();
            if (rounded == null) throw new JsonRpcException("E_OPERATION_FAILED", "Filletting failed.");
            namedItems[string.IsNullOrEmpty(name) ? solidName : name] = Solid.MakeSolid(rounded);
            if (rebind) Rebind(shell, rounded);
        }

        #endregion

        #region Transform operations

        private void TransformScaleImpl(JsonElement objectsEl, GeoPoint center, double factor, JsonElement factorsEl, string name, string copySuffix)
        {
            throw new NotImplementedException();
        }

        private void TransformReflectImpl(JsonElement objectsEl, Plane plane, Axis axis3d, Axis2D axis2d, string name, string copySuffix)
        {
            List<object> objects = IterateObjectRefs<object>(objectsEl).ToList();
            if (objects.Count == 0) throw new JsonRpcException("E_INVALID_PARAMS", "No objects to reflect.");
            if (objects[0] is Solid)
            {   // 3d reflection

            }
            else
            {   // 2d reflection
                if (axis2d.IsValid)
                {
                    ModOp2D reflect = ModOp2D.Reflect(axis2d.Location, axis2d.Direction);
                    foreach (object obj in objects)
                    {
                        if (obj is ICurve2D c2d)
                        {
                            ICurve2D? clone = c2d.GetModified(reflect) as ICurve2D;
                            if (clone != null)
                            {
                                if (!string.IsNullOrEmpty(name)) namedItems[name] = clone;
                                else
                                {
                                    string? currentName = FindName(c2d);
                                    if (currentName != null) namedItems[currentName] = clone;
                                }
                            }
                        }
                        else if (obj is CompoundShape cs)
                        {
                            CompoundShape? clone = cs.GetModified(reflect) as CompoundShape;
                            if (clone != null)
                            {
                                if (!string.IsNullOrEmpty(name)) namedItems[name] = clone;
                                else
                                {
                                    string? currentName = FindName(cs);
                                    if (currentName != null) namedItems[currentName] = clone;
                                }
                            }
                        }
                    }
                }
                else throw new JsonRpcException("E_INVALID_PARAMS", "For 2D reflection, a valid axis2d must be provided.");
            }
        }

        private void TransformRotateImpl(JsonElement objectsEl, Axis axis, double angle, string name, string copySuffix)
        {
            List<Solid> objects = IterateObjectRefs<Solid>(objectsEl).ToList();
            GeoPoint origin = axis.Location;
            GeoVector direction = axis.Direction;
            ModOp rot = ModOp.Rotate(origin, direction, SweepAngle.Deg(angle));
            bool copy = !string.IsNullOrEmpty(name);
            List<Solid> modified = [];
            foreach (Solid s in objects)
            {
                Solid? clone;
                if (copy) clone = s.Clone() as Solid;
                else clone = s;
                if (clone != null)
                {
#if DEBUG
                    clone.Shell.CheckConsistency();
#endif
                    clone.Modify(rot);
#if DEBUG
                    clone.Shell.CheckConsistency();
#endif
                    modified.Add(clone);
                    if (copy && !string.IsNullOrEmpty(copySuffix))
                    {
                        string newName = name + copySuffix;
                        namedItems[newName] = clone;
                    }
                }
            }
            if (copy) namedItems[name] = modified;
            // Without a name the originals were rotated in place, so nothing was written to
            // namedItems; report the affected items explicitly.
            else foreach (string rotatedName in AllNames(objectsEl)) NoteModifiedInPlace(rotatedName);
        }

        private void TransformMoveImpl(JsonElement objects, GeoVector delta, string name, string copySuffix)
        {
            List<Solid> toMove = IterateSelector<Solid>(objects).ToList();
            List<Solid> modified = [];
            ModOp move = ModOp.Translate(delta);
            for (int i = 0; i < toMove.Count; i++)
            {
                if (name == null) toMove[i].Modify(move);
                else
                {
                    Solid clone = toMove[i].Clone() as Solid;
                    modified.Add(clone);
                    clone.Modify(move);
                    if (name != null && !string.IsNullOrEmpty(copySuffix))
                    {
                        string newName = name + copySuffix;
                        namedItems[newName] = clone;
                    }
                }
            }
            if (name != null) namedItems[name] = modified;
            // Without a name the originals were moved in place, so nothing was written to
            // namedItems; report the affected items explicitly.
            else foreach (string movedName in AllNames(objects)) NoteModifiedInPlace(movedName);
        }

        #endregion

        #region Inspection and assertions

        private void AssertCheckImpl(JsonElement objects, string? condition, int minCount, int maxCount, string message, string name)
        {
            List<object> selected = IterateSelector<object>(objects).ToList();
            // "this.", FaceWrapperForEval with properties
            if (minCount >= 0) // default: -1
            {
                if (selected.Count < minCount) throw new JsonRpcException("E_ASSERTION_FAILED", $"Assertion failed. Count={selected.Count}. {message}");
            }
            if (maxCount >= 0)
            {
                if (selected.Count > maxCount) throw new JsonRpcException("E_ASSERTION_FAILED", $"Assertion failed. Count={selected.Count}. {message}");
            }
            if (condition != null)
            {
                object? oldValue = null;
                try
                {
                    namedItems.TryGetValue("this", out oldValue);
                    foreach (var item in selected)
                    {
                        object? wrappedItem = wrapForEvaluator(item);
                        if (wrappedItem != null)
                        {
                            namedItems["this"] = wrappedItem;
                            object evalRes = Evaluator.Evaluate(condition, namedItems.Dict);
                            if (evalRes is bool b)
                            {
                                if (!b) throw new JsonRpcException("E_ASSERTION_FAILED", $"Assertion failed. {message}");
                            }
                        }
                        else throw new NotImplementedException("assert.check not yet fully implemented");
                    }
                    if (selected.Count == 0)
                    {   // a condition without objects
                        object evalRes = Evaluator.Evaluate(condition, namedItems.Dict);
                        if (evalRes is bool b)
                        {
                            if (!b) throw new JsonRpcException("E_ASSERTION_FAILED", $"Assertion failed. {message}");
                        }
                    }
                }
                finally
                {
                    if (oldValue != null) namedItems["this"] = oldValue;
                    else namedItems.Remove("this");
                }
            }
        }

        private JsonNode InspectPropertiesImpl(string target, JsonElement properties)
        {
            if (!namedItems.TryGetValue(target, out object? item) || item == null) throw NamedItemNotFound(target);
            if (properties.ValueKind != JsonValueKind.Array) throw new JsonRpcException(-32602, "Invalid params: 'properties' must be an array of property names");
            var values = new JsonObject();
            using (new NamedItemOverride(namedItems, item))
            {   // evaluate each property as 'this.<property>' in the expression evaluator;
                // a failing property yields an error entry instead of failing the whole call
                foreach (var propEl in properties.EnumerateArray())
                {
                    if (propEl.ValueKind != JsonValueKind.String) throw new JsonRpcException(-32602, "Invalid params: 'properties' must contain strings");
                    string prop = propEl.GetString()!;
                    try
                    {
                        object result = Evaluator.Evaluate("this." + prop, namedItems.Dict);
                        values[prop] = EvalResultToJson(result);
                    }
                    catch (Exception ex)
                    {
                        values[prop] = new JsonObject { ["error"] = ex.Message };
                    }
                }
            }
            return new JsonObject { ["values"] = values };
        }

        private static JsonNode? EvalResultToJson(object? result) => result switch
        {
            null => null,
            double d => JsonValue.Create(d),
            int i => JsonValue.Create(i),
            bool b => JsonValue.Create(b),
            string s => JsonValue.Create(s),
            GeoPoint p => new JsonObject { ["x"] = p.x, ["y"] = p.y, ["z"] = p.z },
            GeoVector v => new JsonObject { ["x"] = v.x, ["y"] = v.y, ["z"] = v.z },
            GeoPoint2D p2 => new JsonObject { ["x"] = p2.x, ["y"] = p2.y },
            GeoVector2D v2 => new JsonObject { ["x"] = v2.x, ["y"] = v2.y },
            BoundingBox bb => BoundingBoxToJson(bb),
            BoundingRect br => BoundingRectToJson(br),
            _ => JsonValue.Create(result.ToString())
        };

        // Visually distinct colours for up to 8 named targets.
        // Cycles when there are more than 8 targets.
        private static readonly Color[] ScenePalette =
        [
            Color.FromArgb(204,  60,  60),  // red
            Color.FromArgb( 60, 100, 200),  // blue
            Color.FromArgb( 50, 160,  50),  // green
            Color.FromArgb(210, 130,  20),  // orange
            Color.FromArgb(130,  60, 200),  // purple
            Color.FromArgb( 20, 170, 170),  // cyan
            Color.FromArgb(180, 180,  20),  // yellow
            Color.FromArgb(200,  60, 160),  // pink
        ];

        private static string ColorToHex(Color c)
            => $"#{c.R:X2}{c.G:X2}{c.B:X2}";

        private JsonNode InspectSceneImpl(JsonElement targets, bool includeBoundingBoxes, string? geometryFormat, bool includeImage, JsonElement imageSize, JsonElement viewDirectionEl)
        {
            // imageSize and viewDirection arrive as raw JSON because their schema types (object
            // resp. string|Vec3 union) have no direct parameter mapping in the generator
            int imageWidth = imageSize.ValueKind == JsonValueKind.Object ? GetOptionalInteger(imageSize, "width", 512) : 512;
            int imageHeight = imageSize.ValueKind == JsonValueKind.Object ? GetOptionalInteger(imageSize, "height", 512) : 512;
            // cap the image size: rendered images travel through the client's context window
            imageWidth = Math.Clamp(imageWidth, 16, 1024);
            imageHeight = Math.Clamp(imageHeight, 16, 1024);
            GeoVector viewDirection = GetOptionalViewDirection(viewDirectionEl);
            var names = new List<string>();
            if (targets.ValueKind == JsonValueKind.String)
                names.Add(targets.GetString()!);
            else if (targets.ValueKind == JsonValueKind.Array)
                foreach (var el in targets.EnumerateArray())
                    if (el.ValueKind == JsonValueKind.String && el.GetString() is string s)
                        names.Add(s);

            var objects  = new JsonArray();
            var geoObjs  = new List<(IGeoObject obj, Color color)>();
            var sceneBB  = BoundingBox.EmptyBoundingBox;
            int colorIdx = 0;

            foreach (string name in names)
            {
                if (!namedItems.TryGetValue(name, out object? item) || item == null)
                    throw new JsonRpcException("E_NOT_FOUND", $"Workspace item '{name}' not found.");

                // Unwrap only singleton lists so multi-element lists keep their type.
                object unwrapped = UnwrapSingletonList(item);

                Color color = ScenePalette[colorIdx++ % ScenePalette.Length];

                var entry = new JsonObject { ["name"] = name };
                entry["type"]  = GetItemTypeName(unwrapped);
                entry["color"] = ColorToHex(color);

                var itemGeos = ItemToRenderableGeoObjects(unwrapped).ToList();
                if (itemGeos.Count > 1)
                    entry["count"] = itemGeos.Count;

                foreach (var (go, col) in itemGeos.Select(g => (g, color)))
                    geoObjs.Add((go, col));

                var itemBB = BoundingBox.EmptyBoundingBox;
                foreach (var (go, _) in geoObjs.TakeLast(itemGeos.Count))
                    itemBB.MinMax(go.GetBoundingCube());
                if (!itemBB.IsEmpty)
                {
                    entry["boundingBox"] = BoundingBoxToJson(itemBB);
                    sceneBB.MinMax(itemBB);
                }

                objects.Add(entry);
            }

            string? imageBase64 = geoObjs.Count > 0
                ? WorkspaceRenderer.RenderToPngBase64(frame, geoObjs, viewDirection, imageWidth, imageHeight)
                : null;

            var result = new JsonObject();
            result["objects"]          = objects;
            result["sceneBoundingBox"] = sceneBB.IsEmpty ? null : BoundingBoxToJson(sceneBB);
            result["image"]            = imageBase64;
            if (imageBase64 != null)
            {
                try
                {   // debug convenience: also write the PNG to the temp directory so it can be
                    // viewed when working with the RPC debug window instead of an MCP client
                    string dir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ShapeIt");
                    System.IO.Directory.CreateDirectory(dir);
                    string file = System.IO.Path.Combine(dir, "inspect_scene.png");
                    System.IO.File.WriteAllBytes(file, Convert.FromBase64String(imageBase64));
                    result["imageFile"] = file;
                }
                catch (Exception) { } // never fail the call over the debug dump
            }
            return result;
        }

        private static IEnumerable<IGeoObject> ItemToRenderableGeoObjects(object item) => item switch
        {
            IGeoObject go => new[] { go },
            List<Solid> sl => sl.Cast<IGeoObject>(),
            List<Face> fl => fl.Cast<IGeoObject>(),
            List<Edge> el => el.Select(e => e.Curve3D as IGeoObject).OfType<IGeoObject>(),
            List<ICurve> cl => cl.OfType<IGeoObject>(),
            Sketch sk => SketchToGeoObjects(sk),
            ICurve2D c2d => Curve2DToGeoObject(c2d) is IGeoObject go2 ? new[] { go2 } : Array.Empty<IGeoObject>(),
            List<ICurve2D> cl2 => cl2.Select(Curve2DToGeoObject).OfType<IGeoObject>(),
            CompoundShape cs => ShapeToGeoObjects(cs),
            List<CompoundShape> csl => csl.SelectMany(ShapeToGeoObjects),
            _ => Enumerable.Empty<IGeoObject>()
        };

        private static IEnumerable<IGeoObject> SketchToGeoObjects(Sketch sk)
        {
            var result = new List<IGeoObject>();
            if (sk.Curves != null)
                result.AddRange(sk.Curves.Select(c => c.MakeGeoObject(sk.Plane)).OfType<IGeoObject>());
            if (sk.Shapes != null)
                result.AddRange(sk.Shapes.SelectMany(sh => sh.MakePaths(sk.Plane)));
            return result;
        }

        private static IGeoObject? Curve2DToGeoObject(ICurve2D c2d)
        {
            Sketch? sk = c2d.UserData["MCPServer.Sketch"] as Sketch;
            return c2d.MakeGeoObject(sk?.Plane ?? Plane.XYPlane);
        }

        private static IEnumerable<IGeoObject> ShapeToGeoObjects(CompoundShape cs)
        {
            Sketch? sk = cs.UserData["MCPServer.Sketch"] as Sketch;
            return cs.MakePaths(sk?.Plane ?? Plane.XYPlane);
        }

        private JsonNode InspectSummaryImpl(JsonElement targets)
        {
            var names = new List<string>();
            if (targets.ValueKind == JsonValueKind.String)
                names.Add(targets.GetString()!);
            else if (targets.ValueKind == JsonValueKind.Array)
                foreach (var el in targets.EnumerateArray())
                    if (el.ValueKind == JsonValueKind.String && el.GetString() is string s)
                        names.Add(s);

            var objects = new JsonArray();
            foreach (string name in names)
            {
                if (!namedItems.TryGetValue(name, out object? item) || item == null)
                    throw new JsonRpcException("E_NOT_FOUND", $"Workspace item '{name}' not found.");

                item = UnwrapSingletonList(item);

                var entry = new JsonObject { ["name"] = name };
                entry["type"] = GetItemTypeName(item);
                entry["summary"] = GetItemSummary(item);
                objects.Add(entry);
            }
            return new JsonObject { ["objects"] = objects };
        }

        private static object UnwrapSingletonList(object item) => item switch
        {
            List<Solid> sl when sl.Count == 1 => sl[0],
            List<Face> fl when fl.Count == 1 => fl[0],
            List<Edge> el when el.Count == 1 => el[0],
            List<ICurve> cl when cl.Count == 1 => cl[0],
            List<ICurve2D> cl2 when cl2.Count == 1 => cl2[0],
            List<CompoundShape> csl when csl.Count == 1 => csl[0],
            // workspace.select falls back to an untyped list when MakeTypedList finds no common type
            List<object> ol when ol.Count == 1 => ol[0],
            _ => item
        };

        private static string GetItemTypeName(object item) => item switch
        {
            double => "number",
            int => "integer",
            GeoPoint => "point3",
            GeoVector => "vector3",
            GeoPoint2D => "point2",
            GeoVector2D => "vector2",
            Solid => "solid",
            Sketch => "sketch",
            Face => "face",
            Edge => "edge",
            ICurve => "curve",
            ICurve2D => "curve2d",
            CompoundShape => "shape",
            List<Solid> => "solid[]",
            List<Face> => "face[]",
            List<Edge> => "edge[]",
            List<ICurve> => "curve[]",
            List<ICurve2D> => "curve2d[]",
            List<CompoundShape> => "shape[]",
            _ => throw new JsonRpcException("E_NOT_FOUND", $"Unsupported workspace item type: {item.GetType().Name}")
        };

        private static JsonObject GetItemSummary(object item)
        {
            var s = new JsonObject();
            switch (item)
            {
                case double d:
                    s["value"] = d;
                    break;
                case int i:
                    s["value"] = i;
                    break;
                case GeoPoint p:
                    s["x"] = p.x; s["y"] = p.y; s["z"] = p.z;
                    break;
                case GeoVector v:
                    s["x"] = v.x; s["y"] = v.y; s["z"] = v.z;
                    s["length"] = v.Length;
                    break;
                case GeoPoint2D p2:
                    s["x"] = p2.x; s["y"] = p2.y;
                    break;
                case GeoVector2D v2:
                    s["x"] = v2.x; s["y"] = v2.y;
                    s["length"] = v2.Length;
                    break;
                case Solid sld:
                    Shell shell = sld.Shells[0];
                    s["faceCount"] = shell.Faces.Length;
                    s["edgeCount"] = shell.Edges.Length;
                    s["boundingBox"] = BoundingBoxToJson(sld.GetBoundingCube());
                    break;
                case Sketch sk:
                    s["curveCount"] = sk.Curves.Count;
                    s["shapeCount"] = sk.Shapes.Count;
                    break;
                case Face fc:
                    s["boundingBox"] = BoundingBoxToJson(fc.GetBoundingCube());
                    break;
                case List<Solid> sl:
                    s["count"] = sl.Count;
                    s["boundingBox"] = BoundingBoxToJson(CombineBoundingBoxes(sl, x => x.GetBoundingCube()));
                    break;
                case List<Face> fl:
                    s["count"] = fl.Count;
                    s["boundingBox"] = BoundingBoxToJson(CombineBoundingBoxes(fl, x => x.GetBoundingCube()));
                    break;
                case List<Edge> el:
                    s["count"] = el.Count;
                    s["boundingBox"] = BoundingBoxToJson(CombineBoundingBoxes(el, x =>
                        x.Curve3D is IGeoObject go ? go.GetBoundingCube() : BoundingBox.EmptyBoundingBox));
                    break;
                case List<ICurve> cl:
                    s["count"] = cl.Count;
                    s["boundingBox"] = BoundingBoxToJson(CombineBoundingBoxes(cl, x =>
                        x is IGeoObject go ? go.GetBoundingCube() : BoundingBox.EmptyBoundingBox));
                    break;
                case List<ICurve2D> cl2:
                    s["count"] = cl2.Count;
                    s["boundingRect"] = BoundingRectToJson(CombineBoundingRects(cl2, x => x.GetExtent()));
                    break;
                case List<CompoundShape> csl:
                    s["count"] = csl.Count;
                    s["boundingRect"] = BoundingRectToJson(CombineBoundingRects(csl,
                        x => CombineBoundingRects(x.SimpleShapes, ss => ss.GetExtent())));
                    break;
            }
            return s;
        }

        private static JsonObject BoundingBoxToJson(BoundingBox bb) => new JsonObject
        {
            ["minX"] = bb.Xmin,
            ["minY"] = bb.Ymin,
            ["minZ"] = bb.Zmin,
            ["maxX"] = bb.Xmax,
            ["maxY"] = bb.Ymax,
            ["maxZ"] = bb.Zmax,
            ["sizeX"] = bb.Xmax - bb.Xmin,
            ["sizeY"] = bb.Ymax - bb.Ymin,
            ["sizeZ"] = bb.Zmax - bb.Zmin
        };

        private static JsonObject BoundingRectToJson(BoundingRect r) => new JsonObject
        {
            ["left"] = r.Left,
            ["bottom"] = r.Bottom,
            ["right"] = r.Right,
            ["top"] = r.Top,
            ["width"] = r.Width,
            ["height"] = r.Height
        };

        private static BoundingBox CombineBoundingBoxes<T>(IEnumerable<T> items, Func<T, BoundingBox> getBB)
        {
            var bb = BoundingBox.EmptyBoundingBox;
            foreach (var item in items)
                bb.MinMax(getBB(item));
            return bb;
        }

        private static BoundingRect CombineBoundingRects<T>(IEnumerable<T> items, Func<T, BoundingRect> getRect)
        {
            var rect = BoundingRect.EmptyBoundingRect;
            foreach (var item in items)
                rect.MinMax(getRect(item));
            return rect;
        }

        #endregion

        #region Templates and system metadata

        private static JsonObject? cachedToolsetInfo;

        private JsonNode SystemGetInfoImpl()
        {
            if (cachedToolsetInfo == null)
            {
                cachedToolsetInfo = new JsonObject();
                var assembly = System.Reflection.Assembly.GetExecutingAssembly();
                using var stream = assembly.GetManifestResourceStream("ShapeIt.McpToolsetDefinition.json");
                if (stream != null)
                {
                    using var doc = JsonDocument.Parse(stream);
                    foreach (string prop in new[] { "toolsetId", "toolsetVersion", "schemaVersion" })
                    {
                        if (doc.RootElement.TryGetProperty(prop, out JsonElement el) && el.ValueKind == JsonValueKind.String)
                            cachedToolsetInfo[prop] = el.GetString();
                    }
                }
            }
            return cachedToolsetInfo.DeepClone();
        }

        private void TemplateBeginImpl(string name, string label, string description, string category, JsonElement tags, JsonElement parameters, bool allowDocumentCommit)
        {
            namedItemClones.Push(new NamedItemClone(this));
            if (parameters.ValueKind != JsonValueKind.Array) { throw new JsonRpcException("E_INVALID_PARAMETER", $"'parameters must be an array'."); }
            foreach (var item in parameters.EnumerateArray())
            {
                string parName = RequireString(item, "name");
                if (!item.TryGetProperty("value", out JsonElement parValue)) { throw new JsonRpcException("E_INVALID_PARAMETER", $"No value found for {parName}."); }
                string? parLabel = GetOptionalString(item, "label");
                item.TryGetProperty("input", out var parInput);
                WorkspaceSetImpl(parName, parValue, parLabel, parInput);
            }
        }

        private object? TemplateCommitImpl(JsonElement result, string? resultKind, bool suffixInternalNames)
        {
            List<object> resultingObjects = IterateSelector<object>(result).ToList();
            namedItemClones.Pop().Dispose();
            return MakeTypedList(resultingObjects);
        }

        private object? TemplateInstantiateImpl(string template, JsonElement arguments, string transform, string? name, bool explodeResult)
        {

            if (!templates.TryGetValue(template, out var jsons)) throw new JsonRpcException("E_INVALID_PARAMETER", $"Template '{template}' not found.");
            object? res = null; // the result
            {   // use a clone of the named items dictionary during evaluation of the template
                foreach (var element in jsons)
                {
                    string methodName = RequireString(element, "method");
                    if (methodName == "template.commit")
                    {
                        if (!element.TryGetProperty("params", out var parameters)) throw new JsonRpcException("E_INTERNAL_ERROR", $"Template '{template}' has invalid commit method.");
                        JsonElement result = RequireProperty(parameters, "result");
                        var resultKind = GetOptionalString(parameters, "resultKind");
                        var suffixInternalNames = GetOptionalBool(parameters, "suffixInternalNames", true);

                        res = TemplateCommitImpl(result, resultKind, suffixInternalNames);
                        // template.commit restored the old named items
                    }
                    else
                    {
                        ProcessMethod(element, true);
                        if (stopExecution) return null;
                        if (methodName == "template.begin")
                        {   // here we overwrite the workspace values of the parameters
                            // template.begin createt a new copy of the named items, so we can safely overwrite values here without affecting the outside
                            if (arguments.ValueKind == JsonValueKind.Object)
                            {
                                foreach (var item in arguments.EnumerateObject())
                                {
                                    string parName = item.Name;
                                    if (item.Value.ValueKind == JsonValueKind.Number) namedItems[parName] = item.Value.GetDouble();
                                    else if (item.Value.ValueKind == JsonValueKind.String) namedItems[parName] = Evaluator.Evaluate(item.Value.GetString(), namedItems.Dict);
                                }
                            }
                        }
                    }
                }
            }
            // now apply the transformation if there is one
            // (here the named items contain the result of the template, so the transform can refer to it)
            if (!string.IsNullOrEmpty(transform))
            {
                object m = Evaluator.Evaluate(transform, namedItems.Dict);
                if (m is ModOp mop)
                {
                    if (res is List<Solid> solids)
                    {
                        foreach (Solid s in solids)
                        {
                            s.Modify(mop);
                        }
                    }
                    else if (res is IGeoObject go)
                    {
                        go.Modify(mop);
                    }
                }
                else
                {
                    throw new JsonRpcException("E_INVALID_PARAMETER", $"Transform expression did not evaluate to a transformation.");
                }
            }

            // now save in the original named items dictionary
            if (res != null && name != null) namedItems[name] = res;
            return res;
        }

        public static readonly Dictionary<string, int> ErrorNumbers = new()
        {
            ["E_INVALID_PARAMS"] = 1001,
            ["E_NOT_FOUND"] = 1002,
            ["E_DOC_CHANGED"] = 1003,
            ["E_CENTER_OUTSIDE_FACE"] = 1203,
            ["E_REF_GONE"] = 1302,
            ["E_BOOLEAN_FAIL"] = 1401
        };

        public struct ParameterInfo
        {
            public string label;
            public object? defaultValue;
            public string? kind;
            public string? description;
            public string? group;
            public int order;
        }

        internal string? GetTemplateLabel(string key)
        {
            if (templates.TryGetValue(key, out var jsons))
            {
                if (jsons.Count > 0 && jsons[0].ValueKind == JsonValueKind.Object)
                {
                    if (jsons[0].TryGetProperty("params", out var parameters) && parameters.ValueKind == JsonValueKind.Object)
                    {
                        if (parameters.TryGetProperty("label", out var label) && label.ValueKind == JsonValueKind.String)
                            return label.GetString();
                    }
                }
            }
            return null;
        }

        internal string? GetTemplateDescription(string key)
        {
            if (templates.TryGetValue(key, out var jsons))
            {
                if (jsons.Count > 0 && jsons[0].ValueKind == JsonValueKind.Object)
                {
                    if (jsons[0].TryGetProperty("params", out var parameters) && parameters.ValueKind == JsonValueKind.Object)
                    {
                        if (parameters.TryGetProperty("description", out var description) && description.ValueKind == JsonValueKind.String)
                            return description.GetString();
                    }
                }
            }
            return null;
        }

        public ParameterInfo GetTemplateParameterInfo(string templateName, string parameterName)
        {
            ParameterInfo res = new ParameterInfo();
            if (templates.TryGetValue(templateName, out var jsons))
            {
                if (jsons.Count > 0 && jsons[0].ValueKind == JsonValueKind.Object)
                {
                    using (new NamedItemClone(this))
                    {
                        ProcessMethod(jsons[0], false); // now we should find the values of the parameters in the (temporary) namedItems
                        if (stopExecution) return res;
                        if (jsons[0].TryGetProperty("params", out var prms) && prms.ValueKind == JsonValueKind.Object)
                        {
                            if (prms.TryGetProperty("parameters", out var parameters) && parameters.ValueKind == JsonValueKind.Array)
                            {
                                foreach (var item in parameters.EnumerateArray())
                                {
                                    if (item.ValueKind == JsonValueKind.Object)
                                    {
                                        if (item.TryGetProperty("name", out var propName) && propName.ValueKind == JsonValueKind.String)
                                        {
                                            if (propName.GetString() == parameterName)
                                            {
                                                if (item.TryGetProperty("label", out var label) && label.ValueKind == JsonValueKind.String)
                                                    res.label = label.GetString()!;
                                                namedItems.TryGetValue(parameterName, out res.defaultValue);
                                                if (item.TryGetProperty("input", out var input) && input.ValueKind == JsonValueKind.Object)
                                                {
                                                    if (input.TryGetProperty("kind", out var kind) && kind.ValueKind == JsonValueKind.String)
                                                        res.kind = kind.GetString();
                                                    if (input.TryGetProperty("description", out var description) && description.ValueKind == JsonValueKind.String)
                                                        res.description = description.GetString();
                                                    if (input.TryGetProperty("group", out var group) && group.ValueKind == JsonValueKind.String)
                                                        res.group = group.GetString();
                                                    if (input.TryGetProperty("order", out var order) && order.ValueKind == JsonValueKind.Number)
                                                        res.order = order.GetInt32();
                                                }
                                            }

                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return res;
        }

        internal List<string> GetTemplateParameters(string templateName)
        {
            List<string> res = [];
            if (templates.TryGetValue(templateName, out var jsons))
            {
                if (jsons.Count > 0 && jsons[0].ValueKind == JsonValueKind.Object)
                {
                    if (jsons[0].TryGetProperty("params", out var prms) && prms.ValueKind == JsonValueKind.Object)
                    {
                        if (prms.TryGetProperty("parameters", out var parameters) && parameters.ValueKind == JsonValueKind.Array)
                        {
                            foreach (var item in parameters.EnumerateArray())
                            {
                                if (item.ValueKind == JsonValueKind.Object)
                                {
                                    if (item.TryGetProperty("name", out var propName) && propName.ValueKind == JsonValueKind.String)
                                    {
                                        res.Add(propName.GetString()!);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return res;
        }

        public object? ExecuteTemplate(string template, Dictionary<string, object> parameterValues)
        {
            try
            {
                if (templates.TryGetValue(template, out var jsons))
                {
                    using (new NamedItemClone(this))
                    {
                        foreach (var element in jsons)
                        {
                            string methodName = RequireString(element, "method");
                            if (methodName == "template.commit")
                            {
                                if (!element.TryGetProperty("params", out var parameters)) throw new JsonRpcException("E_INTERNAL_ERROR", $"Template '{template}' has invalid commit method.");
                                JsonElement result = RequireProperty(parameters, "result");
                                var resultKind = GetOptionalString(parameters, "resultKind");
                                var suffixInternalNames = GetOptionalBool(parameters, "suffixInternalNames", true);

                                object? res = TemplateCommitImpl(result, resultKind, suffixInternalNames);
                                return res;

                            }
                            else
                            {
                                ProcessMethod(element, true);
                                if (stopExecution) return null;
                                if (methodName == "template.begin")
                                {   // here we overwrite the workspace values of the parameters
                                    foreach (var item in parameterValues)
                                    {
                                        namedItems[item.Key] = item.Value;
                                    }
                                }
                            }
                        }
                    }
                }
                return null;
            }
            catch (Exception ex)
            {
                return null;
            }
        }

        #endregion

        #region Geometry utility methods

        private Axis AxisFromJson(JsonElement axisRef)
        {
            // AxisRef can be either {standard:"X"|"Y"|"Z"} or {origin:{x,y,z}, direction:{x,y,z}}
            if (axisRef.TryGetProperty("standard", out JsonElement stdEl) && stdEl.ValueKind == JsonValueKind.String)
            {
                string std = stdEl.GetString()?.ToUpper() ?? "Z";
                return std switch
                {
                    "X" => new Axis(GeoPoint.Origin, GeoVector.XAxis),
                    "Y" => new Axis(GeoPoint.Origin, GeoVector.YAxis),
                    "Z" => new Axis(GeoPoint.Origin, GeoVector.ZAxis),
                    _ => throw new JsonRpcException("E_INVALID_PARAMS", "Unknown standard axis.")
                };
            }
            if (axisRef.TryGetProperty("origin", out JsonElement orgEl) && axisRef.TryGetProperty("direction", out JsonElement dirEl))
            {
                GeoPoint org = RequirePoint3D(orgEl, null);
                GeoVector dir = RequireVector3D(dirEl);
                try
                {
                    return new Axis(org, dir);
                }
                catch (ArgumentException ex)
                {
                    throw new JsonRpcException("E_INVALID_PARAMS", "Invalid axis: " + ex.Message);
                }
            }
            throw new JsonRpcException("E_INVALID_PARAMS", "Invalid AxisRef.");
        }

        #endregion

        #region Nested model types

        List<CompoundShape> GetProfiles(JsonElement selector)
        {
            List<CompoundShape> lcs = IterateSelector<CompoundShape>(selector).ToList();
            List<ICurve2D> lc2 = IterateSelector<ICurve2D>(selector).ToList();
            List<ICurve2D> remainingCurves = [];
            for (int i = 0; i < lc2.Count; i++)
            {
                if (lc2[i].IsClosed || Precision.IsEqual(lc2[i].StartPoint, lc2[i].EndPoint))
                {
                    Border bdr = new Border(lc2[i]);
                    CompoundShape cs = new CompoundShape(new SimpleShape(bdr));
                    lcs.Add(cs);
                    cs.UserData.Add("MCPServer.Sketch", lc2[i].UserData["MCPServer.Sketch"]);
                }
                else
                {
                    remainingCurves.Add(lc2[i]);
                }
            }
            if (remainingCurves.Count > 0)
            {
                Reduce2D r2d = new Reduce2D();
                r2d.Add(remainingCurves.ToArray());
                r2d.OutputMode = Reduce2D.Mode.Paths;
                ICurve2D[] r = r2d.Reduced;
                for (int j = 0; j < r.Length; j++)
                {
                    if (r[j].IsClosed || Precision.IsEqual(r[j].StartPoint, r[j].EndPoint))
                    {
                        Border bdr = new Border(r[j]);
                        CompoundShape cs = new CompoundShape(new SimpleShape(bdr));
                        lcs.Add(cs);
                        cs.UserData.Add("MCPServer.Sketch", remainingCurves[0].UserData["MCPServer.Sketch"]);
                    }
                }
            }
            return lcs;
        }

        // Placeholder type for "profile" objects created from sketches.
        // Replace with the real CADability/ShapeIt type when you wire it up.
        internal sealed class Profile
        {
            public string? Name { get; set; }
        }

        #endregion

        #region Miscellaneous implementation helpers

        static private object? MakeTypedList(List<object> selected)
        {
            if (selected.Count == 0) return null;
            // use is-checks so that different concrete implementations of the same interface
            // (e.g. Line2D and Arc2D as ICurve2D) still yield a common typed list
            if (selected.All(x => x is Edge)) return selected.Cast<Edge>().ToList();
            if (selected.All(x => x is Face)) return selected.Cast<Face>().ToList();
            if (selected.All(x => x is Solid)) return selected.Cast<Solid>().ToList();
            if (selected.All(x => x is CompoundShape)) return selected.Cast<CompoundShape>().ToList();
            if (selected.All(x => x is ICurve2D)) return selected.Cast<ICurve2D>().ToList();
            if (selected.All(x => x is ICurve)) return selected.Cast<ICurve>().ToList();
            return null;
        }

        void IterateLoops(List<(string name, double start, double step, int count)> loopVariables, Action body)
        {
            int n = loopVariables.Count;
            int[] indices = new int[n];

            while (true)
            {
                // Aktuelle Werte setzen
                for (int i = 0; i < n; i++)
                {
                    var (name, start, step, count) = loopVariables[i];
                    double val = start + indices[i] * step;
                    namedItems[name] = val;
                }

                // Das eigentliche "Innere" der Schleife
                body();

                // "Zähler erhöhen" (wie bei verschachtelten Schleifen)
                int k = n - 1;
                while (k >= 0)
                {
                    indices[k]++;
                    if (indices[k] < loopVariables[k].count)
                        break;

                    indices[k] = 0;
                    k--;
                }

                // Wenn wir über die erste Schleife hinaus sind: fertig
                if (k < 0)
                    break;
            }
        }

        #endregion

        #region rpc.batch

        private JsonNode RpcBatchImpl(JsonElement calls, bool atomic)
        {
            if (calls.ValueKind != JsonValueKind.Array || calls.GetArrayLength() == 0)
                throw new JsonRpcException(-32602, "Missing or empty 'calls' array");

            JsonElement[] callsArr = calls.EnumerateArray().ToArray();
            int total = callsArr.Length;
            JsonArray results = new JsonArray();

            // An atomic batch runs in its own undo frame: on failure everything it did is rolled
            // back, on success it becomes a single undo step. Undo frames cannot be nested, so when
            // the caller opened one itself the batch runs inside that frame and shares its fate.
            UndoFrameState? transaction = null;
            if (atomic)
            {
                if (currentUndoFrame == null) transaction = OpenUndoFrameState("rpc.batch", isInternal: true);
                else AddCallWarning($"The batch runs inside the undo frame '{currentUndoFrame.Id}' opened with undo.begin, so a failing call is not rolled back on its own. Close that frame with undo.end or undo.cancel for a self-contained transaction.");
            }
            bool frameClosed = transaction == null;

            try
            {
                bool hasError = RunBatchCalls(callsArr, results);
                int executed = results.Count;
                string summary = hasError
                    ? $"Stopped at call {executed - 1} of {total} " +
                      $"({results[executed - 1]!["method"]?.GetValue<string>()}): " +
                      $"{results[executed - 1]!["error"]?.GetValue<string>()}"
                    : $"Completed all {executed} of {total} calls successfully.";

                if (!hasError)
                {
                    if (transaction != null)
                    {
                        currentUndoFrame = null;
                        CloseUndoFrameState(transaction);
                        frameClosed = true;
                    }
                    return new JsonObject
                    {
                        ["summary"] = summary,
                        ["results"] = results
                    };
                }

                string rollbackNote = "";
                if (transaction != null)
                {
                    currentUndoFrame = null;
                    bool documentChangesUndone = RollBackUndoFrameState(transaction);
                    frameClosed = true;
                    rollbackNote = documentChangesUndone
                        ? " All changes made by this batch were rolled back."
                        : " The batch had made no document changes to roll back; the workspace was restored.";
                }
                throw new JsonRpcException(-32000, summary + rollbackNote + " Results: " + results.ToJsonString());
            }
            finally
            {
                // Safety net for an unexpected exception: an undo frame left open would collect every
                // later change into one step, including edits the user makes in the application.
                if (transaction != null && !frameClosed)
                {
                    currentUndoFrame = null;
                    try { RollBackUndoFrameState(transaction); } catch (Exception) { }
                }
            }
        }

        /// <summary>
        /// Runs the calls of a batch in order and collects one result entry per call. Stops at the
        /// first failing call and returns true in that case.
        /// </summary>
        private bool RunBatchCalls(JsonElement[] callsArr, JsonArray results)
        {
            for (int i = 0; i < callsArr.Length; i++)
            {
                JsonElement callEl = callsArr[i];

                if (!callEl.TryGetProperty("method", out var methodEl) || methodEl.ValueKind != JsonValueKind.String)
                {
                    results.Add(new JsonObject
                    {
                        ["index"] = i,
                        ["method"] = "(unknown)",
                        ["error"] = "Missing 'method' field"
                    });
                    return true;
                }

                string method = methodEl.GetString()!;
                int callId = callEl.TryGetProperty("id", out var idEl) && idEl.TryGetInt32(out int idInt) ? idInt : i;
                JsonElement paramsEl = callEl.TryGetProperty("params", out var p) ? p : default;

                string rpcResponse = ProcessMethod(method, callId, paramsEl);

                JsonNode? rpcDoc = null;
                try { rpcDoc = JsonNode.Parse(rpcResponse); } catch { }

                if (rpcDoc?["error"] is JsonNode errNode)
                {
                    results.Add(new JsonObject
                    {
                        ["index"] = i,
                        ["method"] = method,
                        ["error"] = errNode["message"]?.GetValue<string>() ?? "Unknown error"
                    });
                    return true;
                }

                results.Add(new JsonObject
                {
                    ["index"] = i,
                    ["method"] = method,
                    ["result"] = rpcDoc?["result"]?.DeepClone() ?? new JsonObject()
                });
            }
            return false;
        }

        #endregion

    }
}
