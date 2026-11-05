using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// The "rechteckbox" template of BoxTemplate.json: a rectangular box open at the top with rounded vertical edges.
    /// The volume is compared with the closed formula (outer prism minus cavity, each corner reduces the area by
    /// (4 - π)·r²), and the rules of the template are checked.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class BoxTemplateTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        private static string TemplateFile([System.Runtime.CompilerServices.CallerFilePath] string thisFile = "")
            => System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile)!, "..", "..", "..", "ShapeIt", "Templates", "BoxTemplate.json");

        /// <summary>Loads the templates of the template file the way the application does.</summary>
        private static MCPServer ServerWithTemplates()
        {
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };
            using JsonDocument document = JsonDocument.Parse(System.IO.File.ReadAllText(TemplateFile()));
            foreach (JsonElement template in document.RootElement.GetProperty("templates").EnumerateArray())
            {
                List<JsonElement> items = new List<JsonElement>();
                foreach (JsonElement item in template.GetProperty("items").EnumerateArray()) items.Add(item.Clone());
                server.templates[template.GetProperty("name").GetString()!] = items;
            }
            return server;
        }

        private sealed record Box(double L, double B, double H, double t, double R)
        {
            public string Arguments => FormattableString.Invariant(
                $"{{\"L\":{L},\"B\":{B},\"H\":{H},\"t\":{t},\"R\":{R}}}");
        }

        private static void AssertBox(Box box)
        {
            MCPServer server = ServerWithTemplates();
            string response = Call(server, "template.instantiate", $"{{\"template\":\"rechteckbox\",\"arguments\":{box.Arguments},\"name\":\"box\"}}");
            Assert.IsNull(Error(response), response);
            object result = server.namedItems["box"];
            Solid solid = result as Solid ?? ((List<Solid>)result)[0];

            double ri = Math.Max(box.R - box.t, 0);
            double outer = box.L * box.B - (4 - Math.PI) * box.R * box.R;
            double inner = (box.L - 2 * box.t) * (box.B - 2 * box.t) - (4 - Math.PI) * ri * ri;
            double expected = box.H * outer - (box.H - box.t) * inner;
            Assert.AreEqual(expected, solid.Volume(1e-4), 1e-6 * expected, $"volume of {box}");

            BoundingBox ext = solid.GetExtent(0.0);
            Assert.AreEqual(-box.L / 2, ext.Xmin, 1e-6, "length");
            Assert.AreEqual(box.L / 2, ext.Xmax, 1e-6, "length");
            Assert.AreEqual(-box.B / 2, ext.Ymin, 1e-6, "width");
            Assert.AreEqual(box.B / 2, ext.Ymax, 1e-6, "width");
            Assert.AreEqual(0, ext.Zmin, 1e-6, "bottom");
            Assert.AreEqual(box.H, ext.Zmax, 1e-6, "height");

            double d = 0.01 * box.t, zMid = box.H / 2;
            Assert.IsTrue(solid.Shells[0].IsInside(new GeoPoint(0, 0, box.t - d)), "the bottom is material");
            Assert.IsFalse(solid.Shells[0].IsInside(new GeoPoint(0, 0, box.t + d)), "above the bottom is the cavity");
            Assert.IsTrue(solid.Shells[0].IsInside(new GeoPoint(box.L / 2 - box.t + d, 0, zMid)), "the wall");
            Assert.IsFalse(solid.Shells[0].IsInside(new GeoPoint(box.L / 2 - box.t - d, 0, zMid)), "inside the wall");
            // on the diagonal of a corner: the outer rounding cuts it off, the inner one leaves material
            double outerCorner = box.R * (1 - 1 / Math.Sqrt(2));
            Assert.IsFalse(solid.Shells[0].IsInside(new GeoPoint(box.L / 2 - outerCorner + d, box.B / 2 - outerCorner + d, zMid)), "outside the outer rounding");
            Assert.IsTrue(solid.Shells[0].IsInside(new GeoPoint(box.L / 2 - outerCorner - d, box.B / 2 - outerCorner - d, zMid)), "inside the outer rounding");
        }

        [TestMethod]
        public void DefaultBoxWithInnerRounding()
        {
            AssertBox(new Box(100, 60, 40, 2, 5));
        }

        [TestMethod]
        public void SharpCorners()
        {
            AssertBox(new Box(80, 50, 30, 3, 0));
        }

        [TestMethod]
        public void RadiusSmallerThanWallGivesSharpInnerCorners()
        {
            AssertBox(new Box(50, 40, 20, 4, 3));
        }

        private sealed record Bosses(Box Box, double dBoss, double dHole, double holeDepth, double bossInset)
        {
            public string Arguments => FormattableString.Invariant(
                $"{{\"L\":{Box.L},\"B\":{Box.B},\"H\":{Box.H},\"t\":{Box.t},\"R\":{Box.R},\"bosses\":\"corners\",\"dBoss\":{dBoss},\"dHole\":{dHole},\"holeDepth\":{holeDepth},\"bossInset\":{bossInset}}}");
        }

        /// <summary>
        /// A box with screw bosses: the boss center lies rb - t/4 from both inner walls. Per corner the cavity loses the
        /// square between boss center and corner (less the inner rounding), the three quarters of the disc outside
        /// that square, less the two half segments of height t/4 that sink into the walls.
        /// </summary>
        private static void AssertBosses(Bosses b)
        {
            Box box = b.Box;
            MCPServer server = ServerWithTemplates();
            string response = Call(server, "template.instantiate", $"{{\"template\":\"rechteckbox\",\"arguments\":{b.Arguments},\"name\":\"box\"}}");
            Assert.IsNull(Error(response), response);
            object result = server.namedItems["box"];
            Solid solid = result as Solid ?? ((List<Solid>)result)[0];

            double ri = Math.Max(box.R - box.t, 0), rb = b.dBoss / 2, o = box.t / 4, rh = b.dHole / 2;
            Assert.IsTrue(ri <= rb - o, "the formula below needs the inner rounding inside the square");
            double segment = rb * rb * Math.Acos((rb - o) / rb) - (rb - o) * Math.Sqrt(2 * rb * o - o * o);
            double cornerArea = (rb - o) * (rb - o) - (1 - Math.PI / 4) * ri * ri + 0.75 * Math.PI * rb * rb - segment;
            double outer = box.L * box.B - (4 - Math.PI) * box.R * box.R;
            double inner = (box.L - 2 * box.t) * (box.B - 2 * box.t) - (4 - Math.PI) * ri * ri;
            double expected = box.H * outer - (box.H - box.t) * inner
                + 4 * cornerArea * (box.H - b.bossInset - box.t) - 4 * Math.PI * rh * rh * b.holeDepth;
            Assert.AreEqual(expected, solid.Volume(1e-4), 1e-6 * expected, $"volume of {b}");

            BoundingBox ext = solid.GetExtent(0.0);
            Assert.AreEqual(box.L, ext.Xmax - ext.Xmin, 1e-6, "length");
            Assert.AreEqual(box.H, ext.Zmax - ext.Zmin, 1e-6, "height");

            double ux = box.L / 2 - box.t - rb + o, uy = box.B / 2 - box.t - rb + o, d = 0.01 * box.t;
            double top = box.H - b.bossInset, bottom = top - b.holeDepth;
            for (int k = 0; k < 4; k++)
            {
                double sx = k == 0 || k == 3 ? 1 : -1, sy = k < 2 ? 1 : -1;
                Assert.IsFalse(solid.Shells[0].IsInside(new GeoPoint(sx * ux, sy * uy, (top + bottom) / 2)), $"hole {k}");
                Assert.IsTrue(solid.Shells[0].IsInside(new GeoPoint(sx * ux, sy * uy, bottom - d)), $"below the hole {k}");
                Assert.IsTrue(solid.Shells[0].IsInside(new GeoPoint(sx * (ux - (rh + rb) / 2), sy * uy, (top + bottom) / 2)), $"boss wall {k}");
                Assert.IsFalse(solid.Shells[0].IsInside(new GeoPoint(sx * (ux + rh + d), sy * uy, top + d)), $"above the boss {k}");
                Assert.IsFalse(solid.Shells[0].IsInside(new GeoPoint(sx * (ux - rb - d), sy * uy, top - d)), $"beside the boss {k}");
                // between the boss and the corner is filled
                double gap = (rb - o) * 0.9;
                Assert.IsTrue(solid.Shells[0].IsInside(new GeoPoint(sx * (ux + gap), sy * (uy + gap), top - d)), $"corner filled {k}");
            }
        }

        [TestMethod]
        public void BossesFlushWithSharpInnerCorners()
        {
            AssertBosses(new Bosses(new Box(100, 60, 40, 2, 2), 8, 2.5, 10, 0));
        }

        [TestMethod]
        public void BossesBelowRimWithInnerRounding()
        {
            AssertBosses(new Bosses(new Box(100, 60, 40, 2, 5), 8, 3, 12, 3));
        }

        [TestMethod]
        public void BossesInSharpBox()
        {
            AssertBosses(new Bosses(new Box(80, 50, 30, 3, 0), 10, 4, 8, 2));
        }

        [TestMethod]
        public void HoleTooLargeIsRefused()
        {
            string? error = Error(Call(ServerWithTemplates(), "template.instantiate", $"{{\"template\":\"rechteckbox\",\"arguments\":{new Bosses(new Box(100, 60, 40, 2, 5), 8, 7.5, 10, 0).Arguments},\"name\":\"b\"}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "The hole diameter (7.5) must be smaller than 7");
        }

        [TestMethod]
        public void HoleThroughTheBottomIsRefused()
        {
            string? error = Error(Call(ServerWithTemplates(), "template.instantiate", $"{{\"template\":\"rechteckbox\",\"arguments\":{new Bosses(new Box(100, 60, 40, 2, 5), 8, 3, 38, 2).Arguments},\"name\":\"b\"}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "The hole depth (38) must be smaller than the height of the bosses (38)");
        }

        [TestMethod]
        public void WallTooThickIsRefused()
        {
            string? error = Error(Call(ServerWithTemplates(), "template.instantiate", $"{{\"template\":\"rechteckbox\",\"arguments\":{new Box(40, 20, 20, 10, 0).Arguments},\"name\":\"b\"}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "The wall thickness (10) must be smaller than half the length (20) and half the width (10).");
        }

        [TestMethod]
        public void RadiusTooLargeIsRefused()
        {
            string? error = Error(Call(ServerWithTemplates(), "template.instantiate", $"{{\"template\":\"rechteckbox\",\"arguments\":{new Box(40, 20, 20, 2, 10).Arguments},\"name\":\"b\"}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "The corner radius (10)");
        }

        [TestMethod]
        public void TheFileIsOneGroup()
        {
            Dictionary<string, List<JsonElement>> templates = new();
            TemplateFile file = TemplateLibrary.ReadFile(TemplateFile(), templates);
            Assert.AreEqual("Boxes", file.Label);
            Assert.IsNotNull(file.Icon, "the icon of the drop-down button is accepted");
            CollectionAssert.AreEqual(new[] { "rechteckbox" }, (System.Collections.ICollection)file.Names);
            foreach (List<JsonElement> items in templates.Values)
            {
                JsonElement begin = items[0].GetProperty("params");
                Assert.IsNull(TemplateIcon.Problem(begin.GetProperty("icon").GetString()!), begin.GetProperty("name").GetString());
                TemplateParameterRules.ValidateDefinition(begin.GetProperty("parameters"), default, begin.GetProperty("checks"));
            }
        }
    }
}
