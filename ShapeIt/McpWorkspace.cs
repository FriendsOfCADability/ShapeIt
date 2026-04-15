using CADability;
using CADability.Forms.NET8;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ShapeIt
{
    internal class McpWorkspace : Form
    {
        CadControl cadControl;
        MCPServer server;
        Project project;
        public McpWorkspace(MCPServer server)
        {
            this.server = server;
            this.Text = "McpWorkspace";
            this.Size = new System.Drawing.Size(800, 600);
            cadControl = new CadControl();
            cadControl.Dock = DockStyle.Fill;
            this.Controls.Add(cadControl);
            cadControl.CadFrame.Project = Project.CreateSimpleProject();
        }

        public override void Refresh()
        {
            base.Refresh();
        }
    }
}
