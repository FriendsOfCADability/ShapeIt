using System.Text.Json;
using System.Text.Json.Nodes;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// The 'icon' of a template: plain SVG is accepted, anything that runs code or reaches outside the icon is
    /// refused, and the icons that come with ShapeIt render.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class TemplateIconTests
    {
        private const string Plain = "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 64 64\"><defs><linearGradient id=\"g\"/></defs>"
            + "<circle cx=\"32\" cy=\"32\" r=\"20\" fill=\"url(#g)\"/><use href=\"#g\"/></svg>";

        [TestMethod]
        public void PlainSvgIsAccepted()
        {
            Assert.IsNull(TemplateIcon.Problem(Plain));
            Assert.IsNull(TemplateIcon.Problem(TemplateIcon.Generic));
        }

        [DataTestMethod]
        [DataRow("<svg xmlns=\"http://www.w3.org/2000/svg\"><script>alert(1)</script></svg>", "<script>")]
        [DataRow("<svg xmlns=\"http://www.w3.org/2000/svg\"><circle r=\"3\" onclick=\"x()\"/></svg>", "onclick")]
        [DataRow("<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\"><use xlink:href=\"file:///c:/x.svg#a\"/></svg>", "file:///c:/x.svg#a")]
        [DataRow("<svg xmlns=\"http://www.w3.org/2000/svg\"><rect fill=\"url(http://example.com/p)\"/></svg>", "fill")]
        [DataRow("<svg xmlns=\"http://www.w3.org/2000/svg\"><image href=\"#a\"/></svg>", "<image>")]
        [DataRow("<svg xmlns=\"http://www.w3.org/2000/svg\"><style>@import url(x.css);</style></svg>", "<style>")]
        [DataRow("<!DOCTYPE svg [<!ENTITY x SYSTEM \"file:///c:/secret\">]><svg xmlns=\"http://www.w3.org/2000/svg\">&x;</svg>", "well-formed")]
        [DataRow("<svg><circle r=\"3\"/></svg>", "namespace")]
        [DataRow("<svg xmlns=\"http://www.w3.org/2000/svg\"><circle r=\"3\"", "well-formed")]
        public void UnsafeOrBrokenSvgIsRefused(string svg, string expected)
        {
            string? problem = TemplateIcon.Problem(svg);
            Assert.IsNotNull(problem, svg);
            StringAssert.Contains(problem, expected);
        }

        [TestMethod]
        public void TooLongIconIsRefused()
        {
            string svg = "<svg xmlns=\"http://www.w3.org/2000/svg\">" + new string(' ', TemplateIcon.MaxLength) + "</svg>";
            StringAssert.Contains(TemplateIcon.Problem(svg), "at most");
        }

        [TestMethod]
        public void TemplateBeginRefusesAnUnsafeIcon()
        {
            MCPServer server = new MCPServer(null!, Project.CreateSimpleProject()) { SuppressDialogs = true };
            string icon = JsonSerializer.Serialize("<svg xmlns=\"http://www.w3.org/2000/svg\"><script/></svg>");
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"template.begin\",\"params\":{{\"name\":\"t\",\"parameters\":[],\"icon\":{icon}}}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            string? error = JsonNode.Parse(server.ProcessMethod(document.RootElement))?["error"]?["message"]?.GetValue<string>();
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "<script>");
        }

        private static string GearIcon([System.Runtime.CompilerServices.CallerFilePath] string thisFile = "")
        {
            string file = System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile)!, "..", "..", "..", "ShapeIt", "Templates", "StirnradTemplate.json");
            using JsonDocument document = JsonDocument.Parse(System.IO.File.ReadAllText(file));
            return document.RootElement.GetProperty("templates")[0].GetProperty("items")[0].GetProperty("params").GetProperty("icon").GetString()!;
        }

        [TestMethod]
        public void ShippedIconsRender()
        {
            foreach ((string name, string svg) in new[] { ("generic", TemplateIcon.Generic), ("gear", GearIcon()) })
            {
                Assert.IsNull(TemplateIcon.Problem(svg), name);
                // the renderer of the WinForms front end, at the small toolbar size
                using System.Drawing.Bitmap bmp = Svg.SvgDocument.FromSvg<Svg.SvgDocument>(svg).Draw(16, 16);
                int opaque = 0;
                for (int x = 0; x < bmp.Width; x++)
                    for (int y = 0; y < bmp.Height; y++)
                        if (bmp.GetPixel(x, y).A > 128) opaque++;
                Assert.IsTrue(opaque > 16 * 16 / 3, $"{name}: only {opaque} opaque pixels");
                string? dir = System.Environment.GetEnvironmentVariable("ICON_PREVIEW_DIR");
                if (dir != null)
                {   // for looking at it: a large rendering
                    using System.Drawing.Bitmap large = Svg.SvgDocument.FromSvg<Svg.SvgDocument>(svg).Draw(128, 128);
                    large.Save(System.IO.Path.Combine(dir, name + ".png"));
                    bmp.Save(System.IO.Path.Combine(dir, name + "16.png"));
                }
            }
        }
    }
}
