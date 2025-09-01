using System;
using System.Buffers.Binary;
using System.IO;
using System.IO.Pipes;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Text.Unicode;
using System.Threading;
using System.Threading.Tasks;

namespace CadDebuggerViewer
{
    public sealed class PipeServer
    {

        private const string logFileName = "C:\\Temp\\DebViewer.log";
        public event EventHandler? OnClientConnected;
        public event EventHandler? OnClientDisconnected;
        public event EventHandler<string>? OnShow;
        public event EventHandler<CommandPayload>? OnCommand;

        private readonly string _pipeName;

        public PipeServer(string pipeName) => _pipeName = pipeName;

        public async Task RunAsync(CancellationToken ct)
        {
            // Simple single-client variant. If client disconnects → wait again.
            while (!ct.IsCancellationRequested)
            {
                using var server = new NamedPipeServerStream(
                    _pipeName, PipeDirection.InOut, 1, PipeTransmissionMode.Byte, PipeOptions.Asynchronous);

                try
                {
                    await server.WaitForConnectionAsync(ct).ConfigureAwait(false);
                }
                catch (OperationCanceledException) { break; }

                OnClientConnected?.Invoke(this, EventArgs.Empty);

                try
                {
                    await HandleClientAsync(server, ct).ConfigureAwait(false);
                }
                catch { /* Logging optional */ }
                finally
                {
                    try { server.Disconnect(); } catch { }
                    OnClientDisconnected?.Invoke(this, EventArgs.Empty);
                }
            }
        }

        private async Task HandleClientAsync(NamedPipeServerStream pipe, CancellationToken ct)
        {
            var lenBuf = new byte[4];

            // 0) Handshake – within e.g. 5s a Hello must arrive
            using (var helloCts = CancellationTokenSource.CreateLinkedTokenSource(ct))
            {
                helloCts.CancelAfter(TimeSpan.FromSeconds(5));
                var hello = await ReadEnvelopeAsync(pipe, helloCts.Token).ConfigureAwait(false);
                if (!string.Equals(hello?.type, "Hello", StringComparison.OrdinalIgnoreCase) || hello!.ver != 1)
                    throw new InvalidDataException("Protocol: expected Hello v1");

                // Ack back
                await WriteEnvelopeAsync(pipe, new Envelope { ver = 1, type = "HelloAck" }, ct).ConfigureAwait(false);
            }
            File.AppendAllText(logFileName, "HandleClientAsync Stelle 1" + Environment.NewLine);

            while (pipe.IsConnected && !ct.IsCancellationRequested)
            {
                // 1) Read length
                await ReadExactlyAsync(pipe, lenBuf, 4, ct).ConfigureAwait(false);
                int len = BinaryPrimitives.ReadInt32LittleEndian(lenBuf);
                if (len <= 0 || len > 16 * 1024 * 1024) throw new InvalidDataException("Frame size");
                File.AppendAllText(logFileName, "HandleClientAsync Stelle 2" + Environment.NewLine);

                // 2) Read payload
                var buf = new byte[len];
                await ReadExactlyAsync(pipe, buf, len, ct).ConfigureAwait(false);
                File.AppendAllText(logFileName, "HandleClientAsync Stelle 3" + Environment.NewLine);

                // 3) JSON → Envelope
                string dbg = Encoding.UTF8.GetString(buf);
                var env = JsonSerializer.Deserialize<Envelope>(buf);
                if (env == null) continue;
                //if (env.payload == null) continue;
                string payload = env.payload.ToString();
                File.AppendAllText(logFileName, "HandleClientAsync Stelle 4 "+ env.type?.ToString() + Environment.NewLine);

                // 4) Dispatch
                switch (env.type?.ToLowerInvariant())
                {
                    case "show":
                        {
                            if (!string.IsNullOrEmpty(payload)) OnShow?.Invoke(this, payload);
                        }
                        break;
                    case "command":
                        {
                            //var cp = env.payload.Deserialize<CommandPayload>();
                            //if (cp != null) OnCommand?.Invoke(this, cp);
                        }
                        break;
                    case "ping":
                        // optional: write ack
                        break;
                    default:
                        // unknown → ignore/log
                        break;
                }
            }
        }

        private static async Task ReadExactlyAsync(Stream s, byte[] buffer, int count, CancellationToken ct)
        {
            int off = 0;
            while (count > 0)
            {
                int r = await s.ReadAsync(buffer, off, count, ct).ConfigureAwait(false);
                if (r == 0) throw new EndOfStreamException();
                off += r;
                count -= r;
            }
        }
        // above: shared serializer options (Case-insensitive helps if client writes differently)
        private static readonly System.Text.Json.JsonSerializerOptions JsonOpts = new()
        {
            PropertyNameCaseInsensitive = true
        };

        // Helper methods for framing
        private static async Task<Envelope?> ReadEnvelopeAsync(Stream s, CancellationToken ct)
        {
            byte[] lenBuf = new byte[4];
            await ReadExactlyAsync(s, lenBuf, 4, ct).ConfigureAwait(false);
            int len = System.Buffers.Binary.BinaryPrimitives.ReadInt32LittleEndian(lenBuf);
            if (len <= 0 || len > 16 * 1024 * 1024) throw new InvalidDataException("Frame size");

            byte[] buf = new byte[len];
            await ReadExactlyAsync(s, buf, len, ct).ConfigureAwait(false);
            string dbg = Encoding.UTF8.GetString(buf);
            return System.Text.Json.JsonSerializer.Deserialize<Envelope>(buf, JsonOpts);
        }

        private static async Task WriteEnvelopeAsync(Stream s, Envelope env, CancellationToken ct)
        {
            byte[] payload = System.Text.Json.JsonSerializer.SerializeToUtf8Bytes(env, JsonOpts);
            byte[] lenBuf = new byte[4];
            System.Buffers.Binary.BinaryPrimitives.WriteInt32LittleEndian(lenBuf, payload.Length);
            await s.WriteAsync(lenBuf, ct).ConfigureAwait(false);
            await s.WriteAsync(payload, ct).ConfigureAwait(false);
            await s.FlushAsync(ct).ConfigureAwait(false);
        }
    }

    // Protocol types
    // Attention: Your client serializes in camelCase.
    // For deserialization I set JsonOptions.PropertyNameCaseInsensitive = true below.

    public sealed class Envelope
    {
        public int ver { get; set; }
        public string type { get; set; } = "";
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingDefault)] 
        public JsonElement payload { get; set; } 
    }

    public sealed class ShowPayload
    {
        public string? title { get; set; }           // comes from client: title ?? "CADability Object"
        public string? objectKind { get; set; }      // currently null
        public string cadJson { get; set; } = "";    // <-- your CAD-JSON
        public DisplayOptions? display { get; set; } // e.g. { fitAll: true }
    }

    public sealed class DisplayOptions
    {
        public bool fitAll { get; set; } = false;
    }
    public sealed class CommandPayload
    {
        public string Name { get; set; } = "";
        public JsonElement? Args { get; set; }
    }
}
