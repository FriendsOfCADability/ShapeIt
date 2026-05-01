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
        public class JsonFilteringTextBox : RichTextBox
        {
            protected override void OnKeyDown(KeyEventArgs e)
            {
                if (e.Control && e.KeyCode == Keys.V)
                {
                    e.SuppressKeyPress = true;

                    string text = Clipboard.GetText();

                    var fragments = ExtractBracedFragments(text);

                    string filtered = string.Join(
                        Environment.NewLine + Environment.NewLine,
                        fragments);

                    SelectedText = filtered;
                    return;
                }

                base.OnKeyDown(e);
            }

            protected override void WndProc(ref Message m)
            {
                const int WM_PASTE = 0x0302;

                if (m.Msg == WM_PASTE)
                {
                    string text = Clipboard.GetText();

                    var fragments = ExtractBracedFragments(text);

                    string filtered = string.Join(
                        Environment.NewLine + Environment.NewLine,
                        fragments);

                    SelectedText = filtered;
                    return;
                }

                base.WndProc(ref m);
            }
        }

        private JsonFilteringTextBox textBox;
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
        public TestMCP(MCPServer server)
        {
            InitializeComponents();
            this.server = server;
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
            textBox = new JsonFilteringTextBox();
            textBox.Multiline = true;
            textBox.Dock = DockStyle.Fill;
            textBox.ScrollBars = RichTextBoxScrollBars.Both;
            // textBox.AcceptsReturn = true;
            textBox.AcceptsTab = true;
            textBox.MaxLength = 1000000;

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

        bool TryParseRpcBlock(string json)
        {
            if (string.IsNullOrWhiteSpace(json)) { return false; }
            try
            {
                server.currentRpcString = json;
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;
                server.ProcessMethod(root);
                return true;
            }
            catch (Exception ex)
            {
                server.ReportError(ex.Message);
                return false;
            }
        }

        public static List<string> ExtractBracedFragments(string text)
        {
            var result = new List<string>();

            int depth = 0;
            int start = -1;
            bool inString = false;
            bool escape = false;

            for (int i = 0; i < text.Length; i++)
            {
                char ch = text[i];

                if (inString)
                {
                    if (escape)
                    {
                        escape = false;
                    }
                    else if (ch == '\\')
                    {
                        escape = true;
                    }
                    else if (ch == '"')
                    {
                        inString = false;
                    }

                    continue;
                }

                // außerhalb eines String-Literals
                if (ch == '"')
                {
                    inString = true;
                }
                else if (ch == '{')
                {
                    if (depth == 0)
                        start = i;

                    depth++;
                }
                else if (ch == '}')
                {
                    if (depth > 0)
                    {
                        depth--;

                        if (depth == 0 && start >= 0)
                        {
                            result.Add(text.Substring(start, i - start + 1));
                            start = -1;
                        }
                    }
                }
            }

            return result;
        }

    }
}