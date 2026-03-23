// Auto-generated dispatcher skeleton for MCP tools (based on McpToolsetDefinition_updated3.json)
// NOTE: All CAD operations are intentionally left unimplemented (throw NotImplementedException).
// Comments are in English by request.

using CADability;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using System;
using System.Collections.Generic;
using System.DirectoryServices.ActiveDirectory;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml.Linq;
using static System.ComponentModel.Design.ObjectSelectorEditor;

namespace ShapeIt
{
    public partial class MCPServer
    {
        private List<JsonElement>? recordingTemplate = null;
        private string? currentTemplatName = null;

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
                System.Diagnostics.Trace.WriteLine($"RPC: {method}");
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
                    ["data"] = jre.Data,
                    ["id"] = id
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

        public void ProcessMethod(JsonElement root, bool executeTemplate = false)
        {
            string? method = null;
            int? id = null;
            JsonElement @params = default;

            if (root.TryGetProperty("method", out var m) && m.ValueKind == JsonValueKind.String)
            {
                method = m.GetString();
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
                try
                {
                    object res = Evaluator.Evaluate(exprStr, namedItems);
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
                try
                {
                    object res = Evaluator.Evaluate(exprStr, namedItems);
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
                object res = Evaluator.Evaluate(el.GetString()!, namedItems);
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
                    object res = Evaluator.Evaluate(expr, namedItems);
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
                    object res = Evaluator.Evaluate(expr, namedItems);
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
                    object res = Evaluator.Evaluate(expr, namedItems);
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
                    object res = Evaluator.Evaluate(expr, namedItems);
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
                    object res = Evaluator.Evaluate(expr, namedItems);
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
                    object res = Evaluator.Evaluate(expr, namedItems);
                    if (res is double) return (double)res;
                }
                catch (Exception ex) // exception of Evaluator could be more descriptive
                {
                    throw new JsonRpcException(-32602, $"Invalid params: '{prop}', error in expression '{expr}': {ex.Message}");
                }
            }
            throw new JsonRpcException(-32602, $"Invalid params: '{prop}' must be number, expression or named value");
        }

        // ObjectRef: { "name": "..." } or { "id": "..." }
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
            throw new JsonRpcException(1001, $"Named object not found: {name}");
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
        // Selector : { "target": "..." }, { "name": "..." }, { "id": "..." }, {names: ["name": "n1", "id": "id1"]} }, {"query": "..."}, {"op": "..." }
        private IEnumerable<T> IterateSelector<T>(JsonElement selector) where T : class
        {
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
                                object evalRes = Evaluator.Evaluate(expr, namedItems);
                                if (evalRes is bool b)
                                {
                                    if (!b) continue; // expression was false
                                }
                            }
                        }
                        if (filter.TryGetProperty("closeTo", out je))
                        {
                            if (je.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, "Invalid params: 'surfaceType' must be a string");
                            GeoPoint p = RequirePoint3D(je, null);
                            BoundingBox pbox = new BoundingBox(p, Precision.eps);
                            if (toTest is Face face && Math.Abs(face.Distance(p)) > Precision.eps) continue;
                            if (toTest is Solid solid && solid.HitTest(ref pbox, Precision.eps)) continue;
                            if (toTest is Edge edge && edge.Curve3D is IGeoObject go && !go.HitTest(ref pbox, Precision.eps)) continue;
                        }
                        if (filter.TryGetProperty("inside", out je))
                        {
                            if (je.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, "Invalid params: 'inside' must be a string");
                            BoundingBox bbox = RequireBoundingBox(je, null);
                            if (toTest is Face face && !bbox.Contains(face.GetExtent(0.0))) continue;
                            if (toTest is Solid sld && !bbox.Contains(sld.GetExtent(0.0))) continue;
                            if (toTest is Edge edge && edge.Curve3D is IGeoObject go && !bbox.Contains(go.GetExtent(0.0))) continue;
                        }
                        if (filter.TryGetProperty("touchedBy", out je))
                        {
                            if (je.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, "Invalid params: 'touchedBy' must be a string");
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
                            if (je.ValueKind != JsonValueKind.Object) throw new JsonRpcException(-32602, "Invalid params: 'surfaceType' must be a string");
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
                            {
                                case "left": if (bb.Xmin < minValue || bb.Xmin > maxValue) continue; break;
                                case "right": if (bb.Xmax < minValue || bb.Xmax > maxValue) continue; break;
                                case "bottom": if (bb.Zmin < minValue || bb.Zmin > maxValue) continue; break;
                                case "top": if (bb.Zmax < minValue || bb.Zmax > maxValue) continue; break;
                                case "front": if (bb.Ymin < minValue || bb.Ymin > maxValue) continue; break;
                                case "back": if (bb.Ymax < minValue || bb.Ymax > maxValue) continue; break;
                                case "centerx": if (bb.GetCenter().x < minValue || bb.GetCenter().x > maxValue) continue; break;
                                case "centery": if (bb.GetCenter().y < minValue || bb.GetCenter().y > maxValue) continue; break;
                                case "centerz": if (bb.GetCenter().z < minValue || bb.GetCenter().z > maxValue) continue; break;
                                default: throw new JsonRpcException(-32602, $"Invalid params: 'component' = '{component}' must be one of left,right,bottom,top,front,back,centerX,centerY,centerZ");
                            }
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
