// Protocol of the RPC traffic, shown in the "Protokoll" tab of the MCP server window.
//
// Entries are kept as structured objects rather than as preformatted text, because the protocol
// serves two purposes at once: it is read as a log, and its requests can be copied back into the
// "RPC Code" tab to replay a run. Rebuilding the requests from rendered text would be guesswork,
// so the original request payload is kept as a JsonNode and the display text is derived from it.
//
// Every outermost RPC call is recorded with both its request and its response. Nested calls - the
// individual calls of an rpc.batch or the recorded calls of a template - are deliberately not
// recorded on their own: they are already contained in the enclosing request and in its response.
//
// Base64 image data is moved out of the payload into a separate table when an entry is added. The
// display then shows a short placeholder (which the window turns into a preview on double click),
// and the log stays readable instead of being buried under a few hundred kilobytes of base64.
using System;
using System.Collections.Generic;
using System.Text;
using System.Text.Encodings.Web;
using System.Text.Json;
using System.Text.Json.Nodes;

namespace ShapeIt
{
    public enum ProtocolEntryKind
    {
        Request,
        Response,
        Note
    }

    /// <summary>
    /// One entry of the RPC protocol. <see cref="Payload"/> is only kept for requests, where it is
    /// needed to rebuild replayable RPC code; for responses the rendered text is all that remains.
    /// </summary>
    public sealed class ProtocolEntry
    {
        public DateTime Timestamp { get; init; }
        public ProtocolEntryKind Kind { get; init; }
        public string Method { get; init; } = "";
        public int Id { get; init; }
        public long ElapsedMilliseconds { get; init; }
        public bool IsError { get; init; }
        public string? Note { get; init; }

        /// <summary>The original request parameters, with base64 images replaced by placeholders.</summary>
        internal JsonNode? Payload { get; set; }

        /// <summary>The entry as it appears in the protocol view, including its header line.</summary>
        internal string RenderedText { get; set; } = "";

        /// <summary>Numbers of the images extracted from this entry, used to release them again.</summary>
        internal List<int>? ImageNumbers { get; set; }
    }

    public partial class MCPServer
    {
        private readonly List<ProtocolEntry> protocolEntries = new();
        private readonly Dictionary<int, string> protocolImages = new();
        private readonly object protocolLock = new();
        private int protocolVersion;
        private int nextProtocolImageNumber = 1;

        // Nesting depth of ProcessMethod; only calls at depth 0 are written to the protocol.
        private int rpcNestingDepth;

        private const int MaxProtocolEntries = 1000;
        private const int MaxRetainedImages = 25;
        private const int MaxRenderedStringLength = 2000;

        /// <summary>Marks an elided image in the rendered text; the number identifies it in the image table.</summary>
        private const string ImagePlaceholderPrefix = "<Bild #";

        // The default encoder escapes '<', '>', '&' and every non-ASCII character as a numeric
        // escape sequence. That would hide the angle brackets of the image placeholder (so the view
        // could no longer recognize it) and would render umlauts unreadable. This text goes into a
        // text box and onto the clipboard, never into HTML, so the relaxed encoder is safe here.
        private static readonly JsonSerializerOptions IndentedJson = new()
        {
            WriteIndented = true,
            Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping
        };

        /// <summary>
        /// Raised after an entry has been added or the protocol has been cleared. May be raised on
        /// any thread, so a view has to marshal to its own thread before reading <see cref="Protocol"/>.
        /// </summary>
        public event Action? ProtocolChanged;

        /// <summary>The whole protocol as displayable text.</summary>
        public string Protocol
        {
            get
            {
                lock (protocolLock)
                {
                    StringBuilder text = new();
                    foreach (ProtocolEntry entry in protocolEntries)
                    {
                        if (text.Length > 0) text.AppendLine();
                        text.Append(entry.RenderedText);
                    }
                    return text.ToString();
                }
            }
        }

        /// <summary>
        /// The recorded requests as JSON-RPC blocks, in the format the "RPC Code" tab expects. Pasting
        /// this text there and running it repeats the same sequence of calls.
        /// </summary>
        public string ProtocolCalls
        {
            get
            {
                lock (protocolLock)
                {
                    StringBuilder text = new();
                    foreach (ProtocolEntry entry in protocolEntries)
                    {
                        if (entry.Kind != ProtocolEntryKind.Request) continue;
                        if (text.Length > 0) text.AppendLine().AppendLine();
                        JsonObject block = new()
                        {
                            ["jsonrpc"] = "2.0",
                            ["id"] = entry.Id,
                            ["method"] = entry.Method,
                            ["params"] = entry.Payload?.DeepClone() ?? new JsonObject()
                        };
                        text.Append(block.ToJsonString(IndentedJson));
                    }
                    return text.ToString();
                }
            }
        }

        /// <summary>
        /// Counter incremented with every change. A view can compare it against the value it last
        /// rendered instead of rebuilding its text on every notification.
        /// </summary>
        public int ProtocolVersion
        {
            get { lock (protocolLock) return protocolVersion; }
        }

        /// <summary>Returns the base64 data of an elided image, or null when it is no longer retained.</summary>
        public string? GetProtocolImage(int imageNumber)
        {
            lock (protocolLock) return protocolImages.TryGetValue(imageNumber, out string? data) ? data : null;
        }

        /// <summary>
        /// Extracts the image number from a line of the protocol text, or -1 when the line holds none.
        /// Used by the protocol view to turn a double click into an image preview.
        /// </summary>
        public static int FindImageNumberInLine(string? line)
        {
            if (string.IsNullOrEmpty(line)) return -1;
            // search without the leading angle bracket, so the lookup also works if the text was
            // written by an encoder that escapes '<'
            const string marker = "Bild #";
            int start = line.IndexOf(marker, StringComparison.Ordinal);
            if (start < 0) return -1;
            start += marker.Length;
            int end = start;
            while (end < line.Length && char.IsDigit(line[end])) end++;
            return end > start && int.TryParse(line.Substring(start, end - start), out int number) ? number : -1;
        }

        public void ClearProtocol()
        {
            lock (protocolLock)
            {
                protocolEntries.Clear();
                protocolImages.Clear();
                protocolVersion++;
            }
            ProtocolChanged?.Invoke();
        }

        internal void LogRpcRequest(string method, int id, JsonElement parameters)
        {
            JsonNode? payload = ParsePayload(parameters.ValueKind == JsonValueKind.Undefined ? null : parameters.GetRawText());
            AddProtocolEntry(new ProtocolEntry
            {
                Timestamp = DateTime.Now,
                Kind = ProtocolEntryKind.Request,
                Method = method,
                Id = id,
                Payload = payload
            });
        }

        internal void LogRpcResponse(string method, int id, string responseJson, long elapsedMilliseconds)
        {
            JsonNode? payload = ParsePayload(responseJson);
            bool isError = payload is JsonObject obj && obj.ContainsKey("error");
            AddProtocolEntry(new ProtocolEntry
            {
                Timestamp = DateTime.Now,
                Kind = ProtocolEntryKind.Response,
                Method = method,
                Id = id,
                ElapsedMilliseconds = elapsedMilliseconds,
                IsError = isError,
                Payload = payload
            }, keepPayload: false);
        }

        /// <summary>
        /// Records an MCP protocol level event (connect, tool list, resource read). These never reach
        /// ProcessMethod, but they show when a client attached and what it asked for.
        /// </summary>
        internal void LogProtocolNote(string text)
        {
            AddProtocolEntry(new ProtocolEntry
            {
                Timestamp = DateTime.Now,
                Kind = ProtocolEntryKind.Note,
                Note = text
            });
        }

        private static JsonNode? ParsePayload(string? rawJson)
        {
            if (string.IsNullOrWhiteSpace(rawJson)) return null;
            try
            {
                return JsonNode.Parse(rawJson);
            }
            catch (Exception)
            {   // not valid JSON: keep it as a string so nothing is lost
                return JsonValue.Create(rawJson);
            }
        }

        /// <summary>
        /// Adds an entry: extracts its images, renders its display text and applies the size limits.
        /// Responses drop their payload afterwards - only requests need to stay replayable.
        /// </summary>
        private void AddProtocolEntry(ProtocolEntry entry, bool keepPayload = true)
        {
            lock (protocolLock)
            {
                List<int> imageNumbers = new();
                ExtractImages(entry.Payload, imageNumbers);
                if (imageNumbers.Count > 0) entry.ImageNumbers = imageNumbers;
                entry.RenderedText = RenderEntry(entry);
                if (!keepPayload) entry.Payload = null;
                protocolEntries.Add(entry);
                TrimProtocol();
                protocolVersion++;
            }
            ProtocolChanged?.Invoke();
        }

        private void TrimProtocol()
        {
            while (protocolEntries.Count > MaxProtocolEntries)
            {
                ProtocolEntry dropped = protocolEntries[0];
                protocolEntries.RemoveAt(0);
                if (dropped.ImageNumbers != null)
                {
                    foreach (int number in dropped.ImageNumbers) protocolImages.Remove(number);
                }
            }
            // Images dominate the memory of the protocol, so keep only the most recent ones. Their
            // placeholders stay in the text; the view reports that the data is no longer available.
            while (protocolImages.Count > MaxRetainedImages)
            {
                int oldest = int.MaxValue;
                foreach (int number in protocolImages.Keys) if (number < oldest) oldest = number;
                if (oldest == int.MaxValue) break;
                protocolImages.Remove(oldest);
            }
        }

        private static string RenderEntry(ProtocolEntry entry)
        {
            string time = entry.Timestamp.ToString("HH:mm:ss.fff");
            StringBuilder text = new();
            switch (entry.Kind)
            {
                case ProtocolEntryKind.Request:
                    text.AppendLine($"=== {time}  REQUEST   {entry.Method}  (id {entry.Id})");
                    text.AppendLine(RenderPayload(entry.Payload));
                    break;
                case ProtocolEntryKind.Response:
                    string marker = entry.IsError ? "  ERROR" : "";
                    text.AppendLine($"=== {time}  RESPONSE  {entry.Method}  (id {entry.Id})  [{entry.ElapsedMilliseconds} ms]{marker}");
                    text.AppendLine(RenderPayload(entry.Payload));
                    break;
                default:
                    text.AppendLine($"=== {time}  {entry.Note}");
                    break;
            }
            return text.ToString();
        }

        private static string RenderPayload(JsonNode? payload)
        {
            if (payload == null) return "{}";
            JsonNode? forDisplay = payload.DeepClone();
            ShortenLongStrings(forDisplay);
            return forDisplay?.ToJsonString(IndentedJson) ?? "{}";
        }

        /// <summary>
        /// Moves base64 image payloads into the image table and replaces them by a placeholder, so the
        /// stored entry stays small and the view can offer a preview.
        /// </summary>
        private void ExtractImages(JsonNode? node, List<int> imageNumbers)
        {
            if (node is JsonObject obj)
            {
                List<KeyValuePair<string, string>> replacements = new();
                foreach (KeyValuePair<string, JsonNode?> item in obj)
                {
                    if (IsImagePayload(item.Key, item.Value, out string? data))
                    {
                        int number = nextProtocolImageNumber++;
                        protocolImages[number] = data!;
                        imageNumbers.Add(number);
                        replacements.Add(new KeyValuePair<string, string>(item.Key,
                            $"{ImagePlaceholderPrefix}{number}, {data!.Length} Zeichen base64 - Doppelklick zeigt das Bild>"));
                    }
                    else ExtractImages(item.Value, imageNumbers);
                }
                foreach (KeyValuePair<string, string> replacement in replacements) obj[replacement.Key] = replacement.Value;
            }
            else if (node is JsonArray array)
            {
                foreach (JsonNode? item in array) ExtractImages(item, imageNumbers);
            }
        }

        private static bool IsImagePayload(string propertyName, JsonNode? node, out string? data)
        {
            data = null;
            if (propertyName is not ("image" or "data" or "imageBase64")) return false;
            if (node is not JsonValue value || !value.TryGetValue(out string? text) || text == null) return false;
            if (text.Length <= 100) return false;
            data = text;
            return true;
        }

        /// <summary>
        /// Caps overlong strings for the display only. The stored payload keeps its original values so
        /// copied requests stay replayable.
        /// </summary>
        private static void ShortenLongStrings(JsonNode? node)
        {
            if (node is JsonObject obj)
            {
                List<KeyValuePair<string, string>> replacements = new();
                foreach (KeyValuePair<string, JsonNode?> item in obj)
                {
                    if (item.Value is JsonValue value && value.TryGetValue(out string? text) && text != null)
                    {
                        if (text.Length > MaxRenderedStringLength) replacements.Add(new KeyValuePair<string, string>(item.Key, Shorten(text)));
                    }
                    else ShortenLongStrings(item.Value);
                }
                foreach (KeyValuePair<string, string> replacement in replacements) obj[replacement.Key] = replacement.Value;
            }
            else if (node is JsonArray array)
            {
                for (int i = 0; i < array.Count; i++)
                {
                    if (array[i] is JsonValue value && value.TryGetValue(out string? text) && text != null)
                    {
                        if (text.Length > MaxRenderedStringLength) array[i] = Shorten(text);
                    }
                    else ShortenLongStrings(array[i]);
                }
            }
        }

        private static string Shorten(string text) =>
            text.Substring(0, MaxRenderedStringLength) + $"... <{text.Length - MaxRenderedStringLength} weitere Zeichen>";
    }
}
