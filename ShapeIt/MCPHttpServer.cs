using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Reflection;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading;

namespace ShapeIt
{
    internal sealed class MCPHttpServer : IDisposable
    {
        private readonly SynchronizationContext uiContext;
        private readonly HttpListener listener;
        private readonly JsonArray toolsList;
        private Thread? listenerThread;
        private volatile bool running;

        // Updated when a new project opens
        internal MCPServer Server { get; set; }

        // When set, called on the UI thread with the JSON-RPC block for every tools/call request.
        public Action<string>? RpcCallLogger { get; set; }

        public int Port { get; }

        // Directory containing *_example.md files served as resources/examples.
        // Defaults to ~/Documents/MCPDoku; can be overridden before Start().
        public string ExamplesDirectory { get; set; } =
            Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "MCPDoku");

        internal MCPHttpServer(MCPServer server, SynchronizationContext uiContext, int port = 3001)
        {
            Server = server;
            this.uiContext = uiContext;
            Port = port;
            listener = new HttpListener();
            listener.Prefixes.Add($"http://localhost:{port}/");
            toolsList = BuildToolsList();
        }

        // -----------------------------------------------------------------------------------------
        // tools/list: read the embedded definition and convert to MCP format

        private static JsonArray BuildToolsList()
        {
            var assembly = Assembly.GetExecutingAssembly();
            const string resourceName = "ShapeIt.McpToolsetDefinition.json";
            using var stream = assembly.GetManifestResourceStream(resourceName);
            if (stream == null)
            {
                Trace($"Embedded resource not found: {resourceName}");
                return new JsonArray();
            }

            var definition = JsonNode.Parse(stream)!;
            var typesObj = definition["types"]?.AsObject();
            var toolsArr = definition["tools"]?.AsArray();
            if (toolsArr == null) return new JsonArray();

            // The shared type definitions are kept as-is and referenced from each tool via
            // standard JSON Schema "$defs" + "$ref". They are never mutated here; every copy
            // placed into a tool's "$defs" is cloned first.
            var types = new Dictionary<string, JsonNode>(StringComparer.Ordinal);
            if (typesObj != null)
                foreach (var kv in typesObj)
                    if (kv.Value != null)
                        types[kv.Key] = kv.Value;

            var result = new JsonArray();
            foreach (var toolNode in toolsArr)
            {
                if (toolNode == null) continue;
                var tool = toolNode.DeepClone().AsObject();

                if (tool["inputSchema"] is JsonObject schema)
                {
                    // MCP clients validate every tool strictly and reject the WHOLE tools/list
                    // when a single inputSchema lacks type:"object" — guard against that here
                    if (schema["type"] is not JsonValue typeVal || !typeVal.TryGetValue<string>(out string? schemaType) || schemaType != "object")
                    {
                        Trace($"Tool '{tool["name"]}' inputSchema has no type:\"object\" — fixed up for tools/list");
                        schema["type"] = "object";
                    }
                    // Rewrite "#/types/X" refs to "#/$defs/X" and gather the transitive closure
                    // of referenced types. Each tool then carries a compact, self-contained
                    // "$defs" section (one copy per needed type) instead of the fully inlined
                    // definitions, which duplicated large recursive types (Selector -> Query ->
                    // filters -> expressions) at every ref site and bloated the payload ~16x.
                    var needed = new HashSet<string>(StringComparer.Ordinal);
                    CollectRefs(schema, needed);
                    RewriteRefsInPlace(schema);

                    var closure = new HashSet<string>(StringComparer.Ordinal);
                    var queue = new Queue<string>(needed);
                    while (queue.Count > 0)
                    {
                        string name = queue.Dequeue();
                        if (!closure.Add(name)) continue;
                        if (!types.TryGetValue(name, out var def)) continue;
                        CollectRefs(def, needed); // reuse buffer; enqueue any not yet closed
                        foreach (var r in needed)
                            if (!closure.Contains(r)) queue.Enqueue(r);
                    }

                    if (closure.Count > 0)
                    {
                        var defs = new JsonObject();
                        foreach (var name in closure)
                        {
                            if (!types.TryGetValue(name, out var def)) continue;
                            var clone = def.DeepClone();
                            RewriteRefsInPlace(clone);
                            defs[name] = clone;
                        }
                        schema["$defs"] = defs;
                    }
                }

                StripInternalFields(tool);

                // MCP tool names must match ^[a-zA-Z0-9_-]{1,128}$ (Anthropic API requirement;
                // clients drop tools with invalid names). Our JSON-RPC methods use dots
                // ("solid.box"), so serve an underscore variant ("solid_box"); MCPServer
                // translates it back on dispatch (see MCPServer.NormalizeMethodName).
                if (tool["name"] is JsonValue nameVal && nameVal.TryGetValue<string>(out string? toolName) && toolName != null)
                {
                    tool["name"] = toolName.Replace('.', '_');
                }
                result.Add(tool);
            }

            return result;
        }

        // Collect the names of all "#/types/X" $ref targets found in the subtree (read-only).
        private static void CollectRefs(JsonNode? node, HashSet<string> found)
        {
            if (node is JsonObject obj)
            {
                if (obj["$ref"] is JsonValue refVal && refVal.TryGetValue<string>(out var refStr)
                    && refStr.StartsWith("#/types/", StringComparison.Ordinal))
                    found.Add(refStr["#/types/".Length..]);
                foreach (var prop in obj)
                    CollectRefs(prop.Value, found);
            }
            else if (node is JsonArray arr)
            {
                foreach (var item in arr)
                    CollectRefs(item, found);
            }
        }

        // Rewrite every "#/types/X" $ref to "#/$defs/X" in place, so refs resolve against the
        // tool's own "$defs" section. Sibling keywords next to $ref (e.g. a "description"
        // override) are preserved as-is (valid under JSON Schema 2020-12).
        private static void RewriteRefsInPlace(JsonNode? node)
        {
            if (node is JsonObject obj)
            {
                if (obj["$ref"] is JsonValue refVal && refVal.TryGetValue<string>(out var refStr)
                    && refStr.StartsWith("#/types/", StringComparison.Ordinal))
                    obj["$ref"] = "#/$defs/" + refStr["#/types/".Length..];
                foreach (var prop in obj)
                    RewriteRefsInPlace(prop.Value);
            }
            else if (node is JsonArray arr)
            {
                foreach (var item in arr)
                    RewriteRefsInPlace(item);
            }
        }

        // Remove fields that are internal tooling annotations and not part of JSON Schema
        private static void StripInternalFields(JsonNode? node)
        {
            if (node is JsonObject obj)
            {
                obj.Remove("__comment__");
                foreach (var kv in obj)
                    StripInternalFields(kv.Value);
            }
            else if (node is JsonArray arr)
            {
                foreach (var item in arr)
                    StripInternalFields(item);
            }
        }

        // -----------------------------------------------------------------------------------------
        // Lifecycle

        public void Start()
        {
            listener.Start();
            running = true;
            listenerThread = new Thread(ListenLoop) { IsBackground = true, Name = "MCPHttpServer" };
            listenerThread.Start();
            Trace($"Listening on http://localhost:{Port}/  —  {toolsList.Count} tools available");
        }

        public void Stop()
        {
            running = false;
            try { listener.Stop(); } catch { }
        }

        public void Dispose()
        {
            Stop();
            try { listener.Close(); } catch { }
        }

        // -----------------------------------------------------------------------------------------
        // Request loop

        private void ListenLoop()
        {
            while (running)
            {
                try
                {
                    var ctx = listener.GetContext();
                    ThreadPool.QueueUserWorkItem(_ => HandleRequest(ctx));
                }
                catch (HttpListenerException) { break; }
                catch (ObjectDisposedException) { break; }
                catch { }
            }
        }

        private void HandleRequest(HttpListenerContext ctx)
        {
            try
            {
                ctx.Response.AddHeader("Access-Control-Allow-Origin", "*");
                ctx.Response.AddHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
                ctx.Response.AddHeader("Access-Control-Allow-Headers", "Content-Type, Accept, Mcp-Session-Id");

                if (ctx.Request.HttpMethod == "OPTIONS")
                {
                    ctx.Response.StatusCode = 204;
                    ctx.Response.Close();
                    return;
                }

                if (ctx.Request.HttpMethod != "POST")
                {
                    ctx.Response.StatusCode = 405;
                    ctx.Response.Close();
                    return;
                }

                string body;
                using (var sr = new StreamReader(ctx.Request.InputStream, Encoding.UTF8))
                    body = sr.ReadToEnd();

                JsonNode? request;
                try { request = JsonNode.Parse(body); }
                catch
                {
                    WriteJson(ctx, 400, MakeError(null, -32700, "Parse error"));
                    return;
                }

                if (request == null)
                {
                    WriteJson(ctx, 400, MakeError(null, -32700, "Empty body"));
                    return;
                }

                // Notifications have no "id" — acknowledge without a response body
                bool isNotification = request["id"] == null;
                string? method = request["method"]?.GetValue<string>();

                if (method == null)
                {
                    WriteJson(ctx, 400, MakeError(null, -32600, "Invalid request: missing method"));
                    return;
                }

                if (isNotification)
                {
                    ctx.Response.StatusCode = 202;
                    ctx.Response.Close();
                    return;
                }

                int id = request["id"] is JsonValue idVal && idVal.TryGetValue<int>(out var idInt) ? idInt : 0;

                string responseJson = method switch
                {
                    "initialize"      => HandleInitialize(id),
                    "ping"            => MakeResult(id, new JsonObject()),
                    "tools/list"      => HandleToolsList(id),
                    "tools/call"      => HandleToolsCall(id, request["params"]),
                    "resources/list"  => HandleResourcesList(id),
                    "resources/read"  => HandleResourcesRead(id, request["params"]),
                    _                 => MakeError(id, -32601, $"Method not found: {method}")
                };

                // tools/call is written to the protocol by MCPServer.ProcessMethod with its full
                // request and response. The protocol level methods only get a one line note: a
                // tools/list response is ~190 KB of schema and would swamp the protocol.
                string? note = method switch
                {
                    "initialize"     => "MCP initialize - a client connected",
                    "tools/list"     => $"MCP tools/list - {toolsList.Count} tools served",
                    "resources/list" => "MCP resources/list",
                    "resources/read" => $"MCP resources/read {request["params"]?["uri"]}",
                    "ping"           => null, // keepalive, not worth an entry
                    "tools/call"     => null, // logged in full further down the call chain
                    _                => $"MCP {method}"
                };
                if (note != null) Server.LogProtocolNote(note);

                WriteJson(ctx, 200, responseJson);
            }
            catch (Exception ex)
            {
                Trace($"Unhandled error in HandleRequest: {ex.Message}");
                try { ctx.Response.StatusCode = 500; ctx.Response.Close(); } catch { }
            }
        }

        // -----------------------------------------------------------------------------------------
        // MCP method handlers

        private static string HandleInitialize(int id) =>
            MakeResult(id, new JsonObject
            {
                ["protocolVersion"] = "2024-11-05",
                ["capabilities"]    = new JsonObject
                {
                    ["tools"]     = new JsonObject(),
                    ["resources"] = new JsonObject()
                },
                ["serverInfo"] = new JsonObject { ["name"] = "ShapeIt", ["version"] = "1.0" }
            });

        private string HandleToolsList(int id) =>
            MakeResult(id, new JsonObject { ["tools"] = toolsList.DeepClone() });

        // -----------------------------------------------------------------------------------------
        // resources/list  &  resources/read

        private string HandleResourcesList(int id)
        {
            var resources = new JsonArray();

            // Embedded chapter docs (ShapeIt.Docs.chapter_*.md)
            var assembly = Assembly.GetExecutingAssembly();
            foreach (var resName in assembly.GetManifestResourceNames()
                .Where(n => n.StartsWith("ShapeIt.Docs.chapter_", StringComparison.Ordinal) && n.EndsWith(".md", StringComparison.Ordinal))
                .OrderBy(n => n))
            {
                string stem = resName["ShapeIt.Docs.".Length..^".md".Length]; // e.g. "chapter_1_introduction"
                resources.Add(new JsonObject
                {
                    ["uri"]      = $"shapeit://docs/{stem}",
                    ["name"]     = FormatChapterTitle(stem),
                    ["mimeType"] = "text/markdown"
                });
            }

            // File-system examples (*_example.md from ExamplesDirectory)
            if (!string.IsNullOrEmpty(ExamplesDirectory) && Directory.Exists(ExamplesDirectory))
            {
                foreach (var file in Directory.GetFiles(ExamplesDirectory, "*_example.md").OrderBy(f => f))
                {
                    string stem = Path.GetFileNameWithoutExtension(file);
                    resources.Add(new JsonObject
                    {
                        ["uri"]      = $"shapeit://examples/{stem}",
                        ["name"]     = stem.Replace('_', ' '),
                        ["mimeType"] = "text/markdown"
                    });
                }
            }

            return MakeResult(id, new JsonObject { ["resources"] = resources });
        }

        private string HandleResourcesRead(int id, JsonNode? paramsNode)
        {
            string? uri = paramsNode?["uri"]?.GetValue<string>();
            if (uri == null)
                return MakeError(id, -32602, "Missing parameter: uri");

            string? content = null;

            if (uri.StartsWith("shapeit://docs/", StringComparison.Ordinal))
            {
                string stem = uri["shapeit://docs/".Length..];
                string resName = $"ShapeIt.Docs.{stem}.md";
                var assembly = Assembly.GetExecutingAssembly();
                using var stream = assembly.GetManifestResourceStream(resName);
                if (stream != null)
                    using (var sr = new StreamReader(stream, Encoding.UTF8))
                        content = sr.ReadToEnd();
            }
            else if (uri.StartsWith("shapeit://examples/", StringComparison.Ordinal))
            {
                string stem = uri["shapeit://examples/".Length..];
                if (!string.IsNullOrEmpty(ExamplesDirectory))
                {
                    string filePath = Path.Combine(ExamplesDirectory, stem + ".md");
                    if (File.Exists(filePath))
                        content = File.ReadAllText(filePath, Encoding.UTF8);
                }
            }

            if (content == null)
                return MakeError(id, -32002, $"Resource not found: {uri}");

            return MakeResult(id, new JsonObject
            {
                ["contents"] = new JsonArray
                {
                    new JsonObject
                    {
                        ["uri"]      = uri,
                        ["mimeType"] = "text/markdown",
                        ["text"]     = content
                    }
                }
            });
        }

        private static string FormatChapterTitle(string stem)
        {
            // "chapter_1_introduction" -> "Chapter 1: Introduction"
            var parts = stem.Split('_');
            if (parts.Length >= 3 && parts[0] == "chapter")
            {
                string rest = string.Join(" ", parts.Skip(2));
                return $"Chapter {parts[1]}: {char.ToUpper(rest[0])}{rest[1..]}";
            }
            return stem.Replace('_', ' ');
        }

        private string HandleToolsCall(int id, JsonNode? paramsNode)
        {
            string? toolName = paramsNode?["name"]?.GetValue<string>();
            if (toolName == null)
                return MakeError(id, -32602, "Missing parameter: name");

            string argumentsJson = paramsNode?["arguments"]?.ToJsonString() ?? "{}";

            if (RpcCallLogger != null)
            {
                var rpcBlock = new JsonObject
                {
                    ["method"] = toolName,
                    ["id"]     = id,
                    ["params"] = JsonNode.Parse(argumentsJson)
                };
                string rpcJson = rpcBlock.ToJsonString(new JsonSerializerOptions { WriteIndented = true });
                uiContext.Post(_ => RpcCallLogger(rpcJson), null);
            }
            string rpcResponse = "";
            Exception? invocationError = null;

            // All CAD operations must run on the UI thread
            uiContext.Send(_ =>
            {
                try
                {
                    Server.SuppressDialogs = true;
                    var paramsElement = JsonDocument.Parse(argumentsJson).RootElement;
                    rpcResponse = Server.ProcessMethod(toolName, id, paramsElement);
                }
                catch (Exception ex)
                {
                    invocationError = ex;
                }
                finally
                {
                    Server.SuppressDialogs = false;
                }
            }, null);

            if (invocationError != null)
                return MakeError(id, -32603, invocationError.Message);

            // Repackage the JSON-RPC result as MCP tools/call content
            try
            {
                var rpcDoc = JsonNode.Parse(rpcResponse);

                if (rpcDoc?["error"] is JsonNode errNode)
                {
                    string msg = errNode["message"]?.GetValue<string>() ?? "Unknown error";
                    // Warnings collected before the failure travel in error.data.warnings; append them
                    // to the text, because a tools/call result carries no structured error payload.
                    if (errNode["data"]?["warnings"] is JsonArray warnings)
                    {
                        foreach (JsonNode? warning in warnings)
                        {
                            if (warning != null) msg += Environment.NewLine + "Warning: " + warning.GetValue<string>();
                        }
                    }
                    return MakeResult(id, new JsonObject
                    {
                        ["content"] = new JsonArray { new JsonObject { ["type"] = "text", ["text"] = msg } },
                        ["isError"] = true
                    });
                }

                // When the tool result carries a base64 PNG under "image", move it out of the
                // JSON text into a proper MCP image content item: clients then pass it to the
                // model as a real image instead of an unreadable base64 string.
                string? imageBase64 = null;
                if (rpcDoc?["result"] is JsonObject resultObj
                    && resultObj["image"] is JsonValue imgVal
                    && imgVal.TryGetValue<string>(out string? imgStr) && !string.IsNullOrEmpty(imgStr))
                {
                    imageBase64 = imgStr;
                    resultObj["image"] = "(attached as separate image content item)";
                }

                string resultText = rpcDoc?["result"]?.ToJsonString() ?? "{}";
                var content = new JsonArray { new JsonObject { ["type"] = "text", ["text"] = resultText } };
                if (imageBase64 != null)
                {
                    content.Add(new JsonObject
                    {
                        ["type"] = "image",
                        ["data"] = imageBase64,
                        ["mimeType"] = "image/png"
                    });
                }
                return MakeResult(id, new JsonObject
                {
                    ["content"] = content,
                    ["isError"] = false
                });
            }
            catch
            {
                return MakeResult(id, new JsonObject
                {
                    ["content"] = new JsonArray { new JsonObject { ["type"] = "text", ["text"] = rpcResponse } },
                    ["isError"] = false
                });
            }
        }

        // -----------------------------------------------------------------------------------------
        // Helpers

        private static void WriteJson(HttpListenerContext ctx, int statusCode, string json)
        {
            byte[] bytes = Encoding.UTF8.GetBytes(json);
            ctx.Response.StatusCode = statusCode;
            ctx.Response.ContentType = "application/json";
            ctx.Response.ContentLength64 = bytes.Length;
            ctx.Response.OutputStream.Write(bytes, 0, bytes.Length);
            ctx.Response.Close();
        }

        private static string MakeResult(int id, JsonNode result) =>
            new JsonObject { ["jsonrpc"] = "2.0", ["id"] = id, ["result"] = result }.ToJsonString();

        private static string MakeError(int? id, int code, string message)
        {
            var r = new JsonObject
            {
                ["jsonrpc"] = "2.0",
                ["error"]   = new JsonObject { ["code"] = code, ["message"] = message }
            };
            if (id.HasValue) r["id"] = id.Value;
            return r.ToJsonString();
        }

        private static void Trace(string msg) =>
            System.Diagnostics.Trace.WriteLine($"[MCPHttpServer] {msg}");
    }
}
