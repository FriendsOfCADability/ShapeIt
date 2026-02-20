// Auto-generated dispatcher skeleton for MCP tools (based on McpToolsetDefinition_updated3.json)
// NOTE: All CAD operations are intentionally left unimplemented (throw NotImplementedException).
// Comments are in English by request.

using CADability;
using System;
using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace ShapeIt
{
    internal partial class MCPServer
    {


        /// <summary>
        /// Dispatches a JSON-RPC method call. The transport layer should parse JSON-RPC envelope and pass:
        /// - method: the method name
        /// - id: JSON-RPC id (already parsed)
        /// - parameters: the "params" object as JsonElement (may be undefined / null in the JSON)
        /// The return value is a JSON-RPC response string.
        /// </summary>
        public string ProcessMethod(string method, int id, JsonElement parameters)
        {
            var response = new JsonObject
            {
                ["jsonrpc"] = "2.0",
                ["id"] = id
            };

            try
            {
                JsonNode result = DispatchGenerated(method, parameters);

                response["result"] = result ?? new JsonObject();
            }
            catch (JsonRpcException jre)
            {
                response.Remove("result");
                response["error"] = new JsonObject
                {
                    ["code"] = jre.Code,
                    ["message"] = jre.Message,
                    ["data"] = jre.Data
                };
            }
            catch (NotImplementedException)
            {
                // Explicit marker that the dispatcher knows the method but implementation isn't done yet.
                response.Remove("result");
                response["error"] = new JsonObject
                {
                    ["code"] = 9901,
                    ["message"] = "Not implemented"
                };
            }
            catch (Exception ex)
            {
                response.Remove("result");
                response["error"] = new JsonObject
                {
                    ["code"] = 9999,
                    ["message"] = "Internal error",
                    ["data"] = new JsonObject
                    {
                        ["exceptionType"] = ex.GetType().FullName,
                        ["exceptionMessage"] = ex.Message
                    }
                };
            }

            return response.ToJsonString();
        }

        // -------------------------
        // JSON helpers
        // -------------------------

        private GeoPoint ReadPoint3(JsonElement pointEl)
        {
            double x = RequireLength(pointEl, "x");
            double y = RequireLength(pointEl, "y");
            double z = RequireLength(pointEl, "z");
            return new GeoPoint(x, y, z);
        }
        private GeoVector ReadVec3(JsonElement pointEl)
        {
            double x = RequireLength(pointEl, "x");
            double y = RequireLength(pointEl, "y");
            double z = RequireLength(pointEl, "z");
            return new GeoVector(x, y, z);
        }
        private GeoPoint2D ReadPoint2(JsonElement pointEl)
        {
            double x = RequireLength(pointEl, "x");
            double y = RequireLength(pointEl, "y");
            return new GeoPoint2D(x, y);
        }
        private GeoVector2D ReadVec(JsonElement pointEl)
        {
            double x = RequireLength(pointEl, "x");
            double y = RequireLength(pointEl, "y");
            double z = RequireLength(pointEl, "z");
            return new GeoVector2D(x, y);
        }
        private static JsonElement RequireObject(JsonElement root, string nameForError)
        {
            if (root.ValueKind != JsonValueKind.Object)
                throw new JsonRpcException(-32602, $"Invalid params: expected object for {nameForError}");
            return root;
        }
        private T RequireObjectRef<T>(JsonElement obj, string propName) where T : class
        {
            var el = RequireProperty(obj, propName);          // liefert JsonElement des props
            var objRef = el;                                  // oder RequireObject(...) je nach Format
            var resolved = ResolveObjectRef(objRef);
            if (resolved is T t) return t;

            throw new JsonRpcException(1001, $"Object is not a {typeof(T).Name}: {propName}={objRef}");
        }
        private static JsonElement RequireProperty(JsonElement obj, string prop)
        {
            if (!obj.TryGetProperty(prop, out var el))
                throw new JsonRpcException(-32602, $"Invalid params: missing '{prop}'");
            return el;
        }

        private static string RequireString(JsonElement obj, string prop)
        {
            var el = RequireProperty(obj, prop);
            if (el.ValueKind != JsonValueKind.String) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be string");
            return el.GetString() ?? throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be string");
        }

        private static bool GetBool(JsonElement obj, string prop, bool defaultValue)
        {
            if (!obj.TryGetProperty(prop, out var el)) return defaultValue;
            if (el.ValueKind == JsonValueKind.True) return true;
            if (el.ValueKind == JsonValueKind.False) return false;
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be boolean");
        }

        private static double RequireNumber(JsonElement obj, string prop)
        {
            var el = RequireProperty(obj, prop);
            if (el.ValueKind != JsonValueKind.Number) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be number");
            return el.GetDouble();
        }
        private static int RequireInteger(JsonElement obj, string prop)
        {
            var el = RequireProperty(obj, prop);
            if (el.ValueKind != JsonValueKind.Number) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be number");
            return el.GetInt32();
        }

        private static JsonElement GetOptional(JsonElement obj, string prop)
            => obj.TryGetProperty(prop, out var el) ? el : default;

        private static string? GetOptionalString(JsonElement obj, string prop)
        {
            if (!obj.TryGetProperty(prop, out var el)) return null;
            if (el.ValueKind == JsonValueKind.Null) return null;
            if (el.ValueKind != JsonValueKind.String) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be string");
            return el.GetString();
        }
        private static bool GetOptionalBool(JsonElement obj, string prop, bool def)
        {
            if (!obj.TryGetProperty(prop, out var el)) return def;
            if (el.ValueKind == JsonValueKind.Null) return def;
            if (el.ValueKind != JsonValueKind.True && el.ValueKind != JsonValueKind.False) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be boolean");
            return el.GetBoolean();
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
            if (!obj.TryGetProperty(prop, out var el)) return def;
            if (el.ValueKind == JsonValueKind.Null) return def;
            if (el.ValueKind != JsonValueKind.Number) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be number");
            return el.GetDouble();
        }
        private static int GetOptionalInteger(JsonElement obj, string prop, int def)
        {
            if (!obj.TryGetProperty(prop, out var el)) return def;
            if (el.ValueKind == JsonValueKind.Null) return def;
            if (el.ValueKind != JsonValueKind.Number) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be integer");
            return el.GetInt32();
        }
        private  GeoVector GetOptionalVector3D(JsonElement obj, string prop, GeoVector def)
        {
            JsonElement el = obj;
            if (prop!=null && !obj.TryGetProperty(prop, out el)) return def;
            if (el.ValueKind != JsonValueKind.Object) return def; // maybe undefined obj
            GeoPoint res = RequirePoint3D(el, null);
            return res.ToVector();
        }
        private GeoPoint2D RequirePoint2D(JsonElement obj, string? prop)
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
                    if (name != null && namedItems.TryGetValue(name, out object? o) && o is GeoPoint2D res) return res;
                }
            }
            else if (el.TryGetProperty("expr", out var pexpr))
            {
                if (pexpr.ValueKind == JsonValueKind.String)
                {
                    string? expr = pexpr.GetString();
                    if (expr != null)
                    {
                        try
                        {
                            object res = Evaluator.Evaluate(expr, namedItems);
                            if (res is GeoPoint2D pres) return pres;
                            if (res is GeoPoint pres3) return pres3.To2D();
                        }
                        catch (Exception ex) // exception of Evaluator could be more descriptive
                        {
                            throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{expr}': {ex.Message}");
                        }
                    }
                }
            }
            else if (el.TryGetProperty("x", out _) && el.TryGetProperty("y", out _))
            {
                return ReadPoint2(el);
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be a 2d point");
        }
        private GeoPoint RequirePoint3D(JsonElement obj, string? prop)
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
                    if (name != null && namedItems.TryGetValue(name, out object? o) && o is GeoPoint res) return res;
                }
            }
            else if (el.TryGetProperty("expr", out var pexpr))
            {
                if (pexpr.ValueKind == JsonValueKind.String)
                {
                    string? expr = pexpr.GetString();
                    if (expr != null)
                    {
                        try
                        {
                            object res = Evaluator.Evaluate(expr, namedItems);
                            if (res is GeoPoint pres3) return pres3;
                        }
                        catch (Exception ex) // exception of Evaluator could be more descriptive
                        {
                            throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{expr}': {ex.Message}");
                        }
                    }
                }
            }
            else if (el.TryGetProperty("x", out _) && el.TryGetProperty("y", out _) && el.TryGetProperty("z", out _))
            {
                return ReadPoint3(el);
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be a 3d point");
        }

        private GeoVector RequireVector3D(JsonElement obj, string? prop)
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
                    if (name != null && namedItems.TryGetValue(name, out object? o) && o is GeoVector res) return res;
                }
            }
            else if (el.TryGetProperty("expr", out var pexpr))
            {
                if (pexpr.ValueKind == JsonValueKind.String)
                {
                    string? expr = pexpr.GetString();
                    if (expr != null)
                    {
                        try
                        {
                            object res = Evaluator.Evaluate(expr, namedItems);
                            if (res is GeoVector pres3) return pres3;
                        }
                        catch (Exception ex) // exception of Evaluator could be more descriptive
                        {
                            throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{expr}': {ex.Message}");
                        }
                    }
                }
            }
            else if (el.TryGetProperty("x", out _) && el.TryGetProperty("y", out _) && el.TryGetProperty("z", out _))
            {
                return ReadPoint3(el).ToVector();
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be a 3d vector");
        }

        private double GetOptionalLength(JsonElement obj, string prop, double defaultValue)
        {
            if (!obj.TryGetProperty(prop, out var el)) return defaultValue;

            if (el.ValueKind == JsonValueKind.Number) return el.GetDouble();

            return RequireLength(obj, prop);
        }
        private double RequireLength(JsonElement obj, string? prop)
        {
            JsonElement el;
            if (string.IsNullOrEmpty(prop)) el = obj; // the element is already resolved
            else el = RequireProperty(obj, prop);
            if (el.ValueKind == JsonValueKind.Number) { return el.GetDouble(); }
            if (el.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be number, expression or a named value");
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
                    string? expr = pexpr.GetString();
                    if (expr != null)
                    {
                        try
                        {
                            object res = Evaluator.Evaluate(expr, namedItems);
                            if (res is double) return (double)res;
                        }
                        catch (Exception ex) // exception of Evaluator could be more descriptive
                        {
                            throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{expr}': {ex.Message}");
                        }
                    }
                }

            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be number, expression or named value");
        }


        // ObjectRef: { "name": "..." } or { "id": "..." }
        private (string Kind, string Value) ParseObjectRef(JsonElement objRef)
        {
            if (objRef.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, "Invalid params: ObjectRef must be an object");
            if (objRef.TryGetProperty("name", out var n) && n.ValueKind == JsonValueKind.String)
                return ("name", n.GetString()!);
            if (objRef.TryGetProperty("id", out var i) && i.ValueKind == JsonValueKind.String)
                return ("id", i.GetString()!);
            throw new JsonRpcException(-32602, "Invalid params: ObjectRef must have 'id' or 'name'");
        }

        private object ResolveObjectRef(JsonElement objRef)
        {
            var (kind, value) = ParseObjectRef(objRef);
            if (kind == "name")
            {
                if (namedItems.TryGetValue(value, out var o)) return o;
                throw new JsonRpcException(1001, $"Named object not found: {value}");
            }
            else
            {
                if (idItems.TryGetValue(value, out var o)) return o;
                throw new JsonRpcException(1001, $"Object id not found: {value}");
            }
        }


        string GetNextId(string? name)
        {
            int id = nextId++;
            string res = "id_" + id.ToString();
            if (name != null) idItems[res] = name;
            return res;
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

}
