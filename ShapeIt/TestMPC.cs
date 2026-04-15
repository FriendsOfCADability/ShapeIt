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
            textBox = new TextBox();
            textBox.Multiline = true;
            textBox.Dock = DockStyle.Fill;
            textBox.ScrollBars = ScrollBars.Both;
            textBox.AcceptsReturn = true;
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
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;
                server.ProcessMethod(root);
                return true;
            }
            catch (Exception ex) { return false; }
        }

    }
}