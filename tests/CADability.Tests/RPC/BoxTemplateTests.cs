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

        private sealed record Lid(Box Box, string lidType, double d = 2, double gap = 0.2, double overhang = 2, double skirtDepth = 5, double rimHeight = 3, double rimWidth = 1.5)
        {
            public string Arguments => FormattableString.Invariant(
                $"{{\"body\":\"box\",\"lidType\":\"{lidType}\",\"d\":{d},\"gap\":{gap},\"overhang\":{overhang},\"skirtDepth\":{skirtDepth},\"rimHeight\":{rimHeight},\"rimWidth\":{rimWidth}}}");
        }

        /// <summary>Area of a rectangle with rounded corners.</summary>
        private static double RoundedRect(double l, double b, double r) => l * b - (4 - Math.PI) * r * r;

        /// <summary>
        /// A lid on a box made with the "rechteckbox" template. Every region of the lid is a rounded rectangle: the
        /// outline or the opening of the box offset by the overhang, the gap or the rim thickness, the corner radius
        /// growing or shrinking with the offset (at least 0).
        /// </summary>
        private static void AssertLid(Lid lid)
        {
            Box box = lid.Box;
            MCPServer server = ServerWithTemplates();
            string response = Call(server, "template.instantiate", $"{{\"template\":\"rechteckbox\",\"arguments\":{box.Arguments},\"name\":\"box\"}}");
            Assert.IsNull(Error(response), response);
            response = Call(server, "template.instantiate", $"{{\"template\":\"deckel\",\"arguments\":{lid.Arguments},\"name\":\"lid\"}}");
            Assert.IsNull(Error(response), response);
            object result = server.namedItems["lid"];
            Solid solid = result as Solid ?? ((List<Solid>)result)[0];

            bool skirt = lid.lidType == "skirt" || lid.lidType == "both", rim = lid.lidType == "inner" || lid.lidType == "both";
            double o = lid.overhang, g = lid.gap, w = lid.rimWidth;
            double li = box.L - 2 * box.t, bi = box.B - 2 * box.t;
            double expected = ExpectedLidVolume(lid);
            Assert.AreEqual(expected, solid.Volume(1e-4), 1e-6 * expected, $"volume of {lid}");

            BoundingBox ext = solid.GetExtent(0.0);
            double reach = Math.Max(skirt ? lid.skirtDepth : 0, rim ? lid.rimHeight : 0);
            Assert.AreEqual(box.H + lid.d, ext.Zmax, 1e-6, "the plate lies on the rim");
            Assert.AreEqual(box.H - reach, ext.Zmin, 1e-6, "skirt or inner rim reach down");
            Assert.AreEqual(box.L / 2 + (skirt ? o : 0), ext.Xmax, 1e-6, "overhang");
            Assert.AreEqual(box.B / 2 + (skirt ? o : 0), ext.Ymax, 1e-6, "overhang");

            Shell shell = solid.Shells[0];
            Assert.IsTrue(shell.IsInside(new GeoPoint(0, 0, box.H + lid.d / 2)), "plate");
            if (skirt)
            {
                double z = box.H - lid.skirtDepth / 2;
                Assert.IsTrue(shell.IsInside(new GeoPoint(box.L / 2 + (g + o) / 2, 0, z)), "skirt");
                if (g > 0) Assert.IsFalse(shell.IsInside(new GeoPoint(box.L / 2 + g / 2, 0, z)), "gap between skirt and wall");
            }
            if (rim)
            {
                double z = box.H - lid.rimHeight / 2;
                if (w > 0)
                {
                    Assert.IsTrue(shell.IsInside(new GeoPoint(li / 2 - g - w / 2, 0, z)), "inner rim");
                    Assert.IsFalse(shell.IsInside(new GeoPoint(0, 0, z)), "inside the inner rim");
                }
                else Assert.IsTrue(shell.IsInside(new GeoPoint(0, 0, z)), "thickened inside");
                if (g > 0) Assert.IsFalse(shell.IsInside(new GeoPoint(li / 2 - g / 2, 0, z)), "gap between inner rim and wall");
            }
        }

        /// <summary>The volume of a lid without holes on a box without screw bosses.</summary>
        private static double ExpectedLidVolume(Lid lid)
        {
            Box box = lid.Box;
            bool skirt = lid.lidType == "skirt" || lid.lidType == "both", rim = lid.lidType == "inner" || lid.lidType == "both";
            double o = lid.overhang, g = lid.gap, w = lid.rimWidth;
            double li = box.L - 2 * box.t, bi = box.B - 2 * box.t, ri = Math.Max(box.R - box.t, 0);
            // the opening less the gap: the inset plate and the outside of the inner rim
            double inner = RoundedRect(li - 2 * g, bi - 2 * g, Math.Max(ri - g, 0));
            if (lid.lidType == "inset") return lid.d * inner;
            // an outward offset keeps sharp corners sharp (mitered), a rounded corner grows with the offset
            double Grown(double e) => box.R > 0 ? box.R + e : 0;
            double expected = lid.d * (skirt ? RoundedRect(box.L + 2 * o, box.B + 2 * o, Grown(o)) : RoundedRect(box.L, box.B, box.R));
            if (skirt) expected += lid.skirtDepth * (RoundedRect(box.L + 2 * o, box.B + 2 * o, Grown(o)) - RoundedRect(box.L + 2 * g, box.B + 2 * g, Grown(g)));
            if (rim)
            {
                double inside = w > 0 ? RoundedRect(li - 2 * g - 2 * w, bi - 2 * g - 2 * w, Math.Max(ri - g - w, 0)) : 0;
                expected += lid.rimHeight * (inner - inside);
            }
            return expected;
        }

        /// <summary>
        /// A lid on a box with screw bosses: the holes of the bosses (below the rim or flush with it) go through the
        /// plate of the lid, widened by the hole allowance; the opening is not cut. Skirt and inner rim are not used
        /// here, an inner rim would run into the bosses.
        /// </summary>
        private static void AssertLidOnBosses(Bosses bosses, Lid lid, string holes = "through", double holeAllowance = 0.9)
        {
            MCPServer server = ServerWithTemplates();
            string response = Call(server, "template.instantiate", $"{{\"template\":\"rechteckbox\",\"arguments\":{bosses.Arguments},\"name\":\"box\"}}");
            Assert.IsNull(Error(response), response);
            string arguments = lid.Arguments.TrimEnd('}') + FormattableString.Invariant($",\"holes\":\"{holes}\",\"holeAllowance\":{holeAllowance}}}");
            response = Call(server, "template.instantiate", $"{{\"template\":\"deckel\",\"arguments\":{arguments},\"name\":\"lid\"}}");
            Assert.IsNull(Error(response), response);
            object result = server.namedItems["lid"];
            Solid solid = result as Solid ?? ((List<Solid>)result)[0];

            Box box = lid.Box;
            double rb = bosses.dBoss / 2, o = box.t / 4, rHole = (bosses.dHole + holeAllowance) / 2;
            double ux = box.L / 2 - box.t - rb + o, uy = box.B / 2 - box.t - rb + o; // centers of the bosses
            bool cut = holes == "through";
            double expected = ExpectedLidVolume(lid) - (cut ? 4 * Math.PI * rHole * rHole * lid.d : 0);
            Assert.AreEqual(expected, solid.Volume(1e-4), 1e-6 * expected, $"volume of {lid} on {bosses}");

            double zPlate = lid.lidType == "inset" ? box.H - lid.d / 2 : box.H + lid.d / 2;
            Shell shell = solid.Shells[0];
            for (int k = 0; k < 4; k++)
            {
                double sx = k == 0 || k == 3 ? 1 : -1, sy = k < 2 ? 1 : -1;
                Assert.AreEqual(!cut, shell.IsInside(new GeoPoint(sx * ux, sy * uy, zPlate)), $"hole {k}");
                Assert.IsTrue(shell.IsInside(new GeoPoint(sx * (ux - rHole - 0.1), sy * uy, zPlate)), $"beside hole {k}");
            }
            Assert.IsTrue(shell.IsInside(new GeoPoint(0, 0, zPlate)), "the opening is not cut");
        }

        [TestMethod]
        public void InsetPlateOnBossesBelowTheRim()
        {
            Box box = new Box(100, 60, 40, 2, 5);
            AssertLidOnBosses(new Bosses(box, 8, 2.5, 10, 2), new Lid(box, "inset"));
        }

        [TestMethod]
        public void SkirtLidWithHolesOfBossesBelowTheRim()
        {
            Box box = new Box(100, 60, 40, 2, 5);
            AssertLidOnBosses(new Bosses(box, 8, 2.5, 10, 3), new Lid(box, "skirt"));
        }

        [TestMethod]
        public void PlainLidWithHolesOfFlushBosses()
        {
            Box box = new Box(80, 50, 30, 3, 0);
            AssertLidOnBosses(new Bosses(box, 10, 4, 8, 0), new Lid(box, "plain", d: 3), holeAllowance: 0);
        }

        [TestMethod]
        public void LidWithoutHoles()
        {
            Box box = new Box(100, 60, 40, 2, 5);
            AssertLidOnBosses(new Bosses(box, 8, 2.5, 10, 0), new Lid(box, "skirt"), holes: "none");
        }

        [TestMethod]
        public void PlainLid()
        {
            AssertLid(new Lid(new Box(100, 60, 40, 2, 5), "plain"));
        }

        [TestMethod]
        public void LidWithSkirt()
        {
            AssertLid(new Lid(new Box(100, 60, 40, 2, 5), "skirt"));
        }

        [TestMethod]
        public void LidWithInnerRim()
        {
            AssertLid(new Lid(new Box(100, 60, 40, 2, 5), "inner"));
        }

        [TestMethod]
        public void LidWithSkirtAndFilledInnerRimOnSharpBox()
        {
            AssertLid(new Lid(new Box(80, 50, 30, 3, 0), "both", d: 3, gap: 0.3, overhang: 2.5, skirtDepth: 4, rimHeight: 2, rimWidth: 0));
        }

        [TestMethod]
        public void LidWithInnerRimWithoutGap()
        {
            AssertLid(new Lid(new Box(70, 40, 25, 2, 0), "inner", gap: 0, rimWidth: 1));
        }

        [TestMethod]
        public void LidWithSkirtWithoutGap()
        {
            AssertLid(new Lid(new Box(70, 40, 25, 2, 6), "skirt", gap: 0, overhang: 1.5, skirtDepth: 3));
        }

        [TestMethod]
        public void SkirtThinnerThanTheGapIsRefused()
        {
            MCPServer server = ServerWithTemplates();
            Call(server, "template.instantiate", $"{{\"template\":\"rechteckbox\",\"arguments\":{new Box(100, 60, 40, 2, 5).Arguments},\"name\":\"box\"}}");
            string? error = Error(Call(server, "template.instantiate", $"{{\"template\":\"deckel\",\"arguments\":{new Lid(new Box(100, 60, 40, 2, 5), "skirt", gap: 1, overhang: 1).Arguments},\"name\":\"l\"}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "The overhang (1) must be larger than the gap (1)");
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
            CollectionAssert.AreEqual(new[] { "rechteckbox", "deckel" }, (System.Collections.ICollection)file.Names);
            foreach (List<JsonElement> items in templates.Values)
            {
                JsonElement begin = items[0].GetProperty("params");
                Assert.IsNull(TemplateIcon.Problem(begin.GetProperty("icon").GetString()!), begin.GetProperty("name").GetString());
                TemplateParameterRules.ValidateDefinition(begin.GetProperty("parameters"), default, begin.GetProperty("checks"));
            }
        }
    }
}
