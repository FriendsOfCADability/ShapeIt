using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Windows.Forms;

namespace ShapeIt
{
    public class TestMCP : Form
    {
        private TextBox textBox;
        private Button okButton;
        private MCPServer server;


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
        public TestMCP()
        {
            InitializeComponents();
            server = new MCPServer();
        }

        private void InitializeComponents()
        {
            // Bildschirmgröße bestimmen
            Rectangle screen = Screen.PrimaryScreen.WorkingArea;

            // Dialoggröße = halber Bildschirm
            this.Size = new Size(screen.Width / 2, screen.Height / 2);

            // Zentriert anzeigen
            this.StartPosition = FormStartPosition.CenterScreen;

            this.Text = "TestMCP";
            this.FormBorderStyle = FormBorderStyle.Sizable;
            this.MinimizeBox = true;
            this.MaximizeBox = true;

            // TextBox
            textBox = new TextBox();
            textBox.Multiline = true;
            textBox.Dock = DockStyle.Fill;
            textBox.ScrollBars = ScrollBars.Both;
            textBox.AcceptsReturn = true;
            textBox.AcceptsTab = true;

            // OK Button
            okButton = new Button();
            okButton.Text = "OK";
            okButton.Dock = DockStyle.Bottom;
            okButton.Height = 40;
            okButton.Click += OkButton_Click;

            this.Controls.Add(textBox);
            this.Controls.Add(okButton);

            this.AcceptButton = okButton;
        }

        private void OkButton_Click(object? sender, EventArgs e)
        {
            ProcessText(textBox.Text);
            this.DialogResult = DialogResult.OK;
            this.Close();
        }

        private void ProcessText(string text)
        {
            foreach (var jsonBlock in ReadJsonObjects(text))
            {
                TryParseRpcBlock(jsonBlock);
            }
        }

        private void ProcessLine(string line)
        {
            bool ok = TryParseRpcBlock(line);
        }
        bool TryParseRpcBlock(string json)
        {
            string? method = null;
            int? id = null;
            JsonElement @params = default;
            bool hasParams = false;
            string? direction = null;
            if (string.IsNullOrWhiteSpace(json)) { return false; }
            if (json.StartsWith('#')) return false;
            if (json.StartsWith("<--")) return false;
            if (json.StartsWith("-->"))
            {
                direction = "client->server";
                json = json.Substring(3);
            }
            try
            {
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;

                if (root.TryGetProperty("method", out var m) && m.ValueKind == JsonValueKind.String)
                {
                    method = m.GetString();
                    if (direction == null) direction = "client->server";
                }

                if (root.TryGetProperty("direction", out var dir) && dir.ValueKind == JsonValueKind.String)
                    direction = dir.GetString();

                if (root.TryGetProperty("id", out var idEl))
                {
                    if (idEl.ValueKind == JsonValueKind.Number) id = idEl.GetInt32();
                    else if (idEl.ValueKind == JsonValueKind.Null) id = null;
                }

                if (root.TryGetProperty("params", out var p))
                {
                    @params = p;         // JsonElement ist ein struct, aber Achtung: doc muss leben!
                    hasParams = true;
                }

                if (direction == "client->server" && method != null) server.ProcessMethod(method, id ?? 0, @params);

                return method != null;
            }
            catch (Exception ex) { return false; }
        }

        private void ProcessMethod(string? method, int? id, JsonElement parameters)
        {
            System.Diagnostics.Trace.WriteLine("MCP method: " + method);
        }
    }
}