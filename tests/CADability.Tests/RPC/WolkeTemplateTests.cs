using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// The "cloudRelief" template of WolkeTemplate.json: a box on the x/y plane split by a NURBS surface that
    /// interpolates a grid of random heights; only the part below the surface and above z=0 remains.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class WolkeTemplateTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        private static string TemplateFile([System.Runtime.CompilerServices.CallerFilePath] string thisFile = "")
            => System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile)!, "..", "Files", "Templates", "WolkeTemplate.json");

        /// <summary>Loads the templates of a template file the way the application does.</summary>
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

        private static List<Solid> Instantiate(MCPServer server, string name, string arguments = "", string template = "cloudRelief")
        {
            string response = Call(server, "template.instantiate", $"{{\"template\":\"{template}\",\"arguments\":{{{arguments}}},\"name\":\"{name}\"}}");
            Assert.IsNull(Error(response), response);
            object item = server.namedItems[name];
            return item is Solid solid ? new List<Solid> { solid } : ((List<Solid>)item).ToList();
        }

        private static double Volume(List<Solid> solids) => solids.Sum(s => s.Volume(1e-4));

        private static BoundingBox Extent(List<Solid> solids)
        {
            BoundingBox ext = BoundingBox.EmptyBoundingBox;
            foreach (Solid s in solids) ext.MinMax(s.GetExtent(0.0));
            return ext;
        }

        [TestMethod]
        public void ReliefStandsOnTheXYPlaneInsideTheRectangle()
        {
            MCPServer server = ServerWithTemplates();
            List<Solid> cloud = Instantiate(server, "cloud1");
            Assert.IsTrue(cloud.Count > 0);
            double volume = Volume(cloud);
            Assert.IsTrue(volume > 0, $"volume {volume} must be positive");
            BoundingBox ext = Extent(cloud);
            Assert.AreEqual(0.0, ext.Zmin, 1e-6, "the relief stands on the x/y plane");
            Assert.IsTrue(ext.Zmax > 0 && ext.Zmax < 20, $"top {ext.Zmax} must lie between 0 and about the highest value");
            Assert.IsTrue(ext.Xmin > -60 && ext.Xmax < 60 && ext.Ymin > -40 && ext.Ymax < 40, $"extent {ext} must lie inside the rectangle");
            // the relief is clearly smaller than the box below the highest value
            Assert.IsTrue(volume < 120 * 80 * 10, $"volume {volume} too large");
        }

        [TestMethod]
        public void SameSeedSameReliefOtherSeedOtherRelief()
        {
            MCPServer server = ServerWithTemplates();
            double v1 = Volume(Instantiate(server, "a", "\"seed\":3"));
            double v2 = Volume(Instantiate(server, "b", "\"seed\":3"));
            double v3 = Volume(Instantiate(server, "c", "\"seed\":4"));
            Assert.AreEqual(v1, v2, v1 * 1e-9, "the same seed must give the same relief");
            Assert.AreNotEqual(v1, v3, v1 * 1e-3, "another seed must give another relief");
        }

        [DataTestMethod]
        [DataRow(1, 6)]
        [DataRow(2, 8)]
        [DataRow(3, 9)]
        [DataRow(4, 7)]
        [DataRow(5, 8)]
        [DataRow(6, 10)]
        [DataRow(7, 8)]
        [DataRow(8, 8)]
        [Timeout(120000)]
        public void SeveralSeedsAndGridSizesBuild(int seed, int gridPoints)
        {
            MCPServer server = ServerWithTemplates();
            var watch = System.Diagnostics.Stopwatch.StartNew();
            List<Solid> cloud = Instantiate(server, "s", $"\"seed\":{seed},\"gridPoints\":{gridPoints}");
            Console.WriteLine($"seed {seed}, grid {gridPoints}: {cloud.Count} part(s), {watch.ElapsedMilliseconds} ms");
            Assert.IsTrue(Volume(cloud) > 0, $"seed {seed}");
            foreach (Solid s in cloud) Assert.IsTrue(s.Volume(1e-4) > 0, $"seed {seed}: every part must be a proper solid");
        }
            /// <summary>
        /// The volume of the trunk along a straight polyline: the cross section is about a circular segment, the
        /// part of a circle above a chord sunk by sink * radius, and the radius grows linearly.
        /// </summary>
        [TestMethod]
        public void StraightTrunkIsAboutASunkHalfCone()
        {
            MCPServer server = ServerWithTemplates();
            const double sink = 0.25;
            List<Solid> trunk = Instantiate(server, "t", "\"path\":[[0,0,0],[0,40,0],[0,80,0]],\"startDiameter\":4,\"endDiameter\":12,\"deviation\":0,\"sink\":0.25", "trunkRelief");
            Assert.AreEqual(1, trunk.Count);
            BoundingBox ext = Extent(trunk);
            Console.WriteLine($"extent {ext}, volume {Volume(trunk)}");
            Assert.AreEqual(0.0, ext.Zmin, 1e-6, "the trunk lies on the x/y plane");
            Assert.AreEqual(0.0, ext.Ymin, 1e-3, "ends in the first half circle");
            Assert.AreEqual(80.0, ext.Ymax, 1e-3, "ends in the last half circle");
            Assert.AreEqual(6.0 * (1 - sink), ext.Zmax, 0.2, "the height at the end is the sunk end radius");
            Assert.AreEqual(6.0 * Math.Sqrt(1 - sink * sink), ext.Xmax, 0.2, "the half width at the end is the chord at the x/y plane");
            // area of the segment above the chord: R² (acos(s) - s sqrt(1 - s²)); the integral of R² along the axis
            double segment = Math.Acos(sink) - sink * Math.Sqrt(1 - sink * sink);
            double expected = segment * 80 / 3 * (2 * 2 + 2 * 6 + 6 * 6);
            Assert.AreEqual(expected, Volume(trunk), expected * 0.06, "about a sunk half cone frustum (5 interpolated points are a bit less than a circle)");
        }

        [DataTestMethod]
        [DataRow(1, 5, 0.15)]
        [DataRow(2, 5, 0.3)]
        [DataRow(3, 7, 0.2)]
        [DataRow(4, 3, 0.1)]
        [Timeout(120000)]
        public void BentTrunkBuilds(int seed, int profilePoints, double deviation)
        {
            MCPServer server = ServerWithTemplates();
            var watch = System.Diagnostics.Stopwatch.StartNew();
            List<Solid> trunk = Instantiate(server, "t", FormattableString.Invariant($"\"seed\":{seed},\"profilePoints\":{profilePoints},\"deviation\":{deviation}"), "trunkRelief");
            long buildMs = watch.ElapsedMilliseconds;
            BoundingBox ext = Extent(trunk);
            Console.WriteLine($"seed {seed}: {trunk.Count} part(s), extent {ext}, volume {Volume(trunk)}, built in {buildMs} ms");
            Assert.AreEqual(1, trunk.Count, "one trunk");
            Assert.AreEqual(0.0, ext.Zmin, 1e-6, "the trunk lies on the x/y plane");
            // the ends are cut perpendicular to the end segments, which are not parallel to the y axis
            Assert.IsTrue(ext.Ymin < 0.5 && ext.Ymin > -7.5, $"cut off near the thick end at y=0: {ext.Ymin}");
            Assert.IsTrue(ext.Ymax > 109.5 && ext.Ymax < 111.5, $"cut off near the thin end at y=110: {ext.Ymax}");
            Assert.IsTrue(ext.Zmax < 7.5 * (1 + deviation) + 1, $"not higher than the dented end radius: {ext.Zmax}");
            Assert.IsTrue(Volume(trunk) > 0);
        }

        [TestMethod]
        public void TrunkTakesThePolylineFromTheWorkspace()
        {
            MCPServer server = ServerWithTemplates();
            server.namedItems["axis"] = Polyline.Construct();
            ((Polyline)server.namedItems["axis"]).SetPoints(new[] { new GeoPoint(0, 0, 0), new GeoPoint(30, 10, 0), new GeoPoint(60, 0, 0) }, false);
            List<Solid> trunk = Instantiate(server, "t", "\"path\":\"axis\"", "trunkRelief");
            BoundingBox ext = Extent(trunk);
            Assert.AreEqual(1, trunk.Count);
            Assert.IsTrue(ext.Xmin > -8 && ext.Xmax < 68 && ext.Ymax < 18, $"extent {ext}");
        }

        /// <summary>An arc as axis: approximated by a polyline, the trunk follows the quarter circle.</summary>
        [TestMethod]
        public void TrunkAlongAnArc()
        {
            MCPServer server = ServerWithTemplates();
            Ellipse arc = Ellipse.Construct();
            arc.SetArcPlaneCenterRadiusAngles(Plane.XYPlane, GeoPoint.Origin, 80, 0, Math.PI / 2);
            server.namedItems["axis"] = arc;
            List<Solid> trunk = Instantiate(server, "t", "\"path\":\"axis\",\"deviation\":0", "trunkRelief");
            BoundingBox ext = Extent(trunk);
            Console.WriteLine($"extent {ext}");
            Assert.AreEqual(1, trunk.Count);
            Assert.AreEqual(0.0, ext.Zmin, 1e-6);
            Assert.IsTrue(ext.Xmax < 80 + 7.6 && ext.Ymax < 80 + 7.6 && ext.Xmin > -8 && ext.Ymin > -8, $"extent {ext}");
            Assert.IsTrue(Volume(trunk) > 0);
        }

        /// <summary>Several connected lines, given out of order, are made into one path first.</summary>
        [TestMethod]
        public void TrunkAlongSeveralLines()
        {
            MCPServer server = ServerWithTemplates();
            List<Solid> trunk = Instantiate(server, "t", "\"path\":[[[40,30,0],[40,70,0]],[[0,0,0],[40,30,0]]]", "trunkRelief");
            BoundingBox ext = Extent(trunk);
            Assert.AreEqual(1, trunk.Count);
            Assert.IsTrue(ext.Ymax > 69 && ext.Ymax < 71 && ext.Xmin < 2, $"extent {ext}");
        }

        [TestMethod]
        public void TrunkRefusesAnAxisOutsideTheXYPlane()
        {
            MCPServer server = ServerWithTemplates();
            string response = Call(server, "template.instantiate", "{\"template\":\"trunkRelief\",\"arguments\":{\"path\":[[0,0,0],[0,50,10]]},\"name\":\"t\"}");
            StringAssert.Contains(Error(response) ?? "", "x/y");
        }
    }
}
