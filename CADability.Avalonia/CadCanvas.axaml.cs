using System;
using System.Numerics;
using Avalonia.Controls;
using Avalonia.OpenGL;
using Avalonia.OpenGL.Controls;
using static Avalonia.OpenGL.GlConsts;

namespace CADability.Avalonia
{
    public partial class CadCanvas: UserControl
    {
        public CadCanvas()
        {
            InitializeComponent();
            // CadCanvasControl canvasControl = new CadCanvasControl();
            PaintToOpenGL openGlControl = new PaintToOpenGL();
            this.Content = openGlControl;
        }
    }
}
