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

        private static List<Solid> Instantiate(MCPServer server, string name, string arguments = "")
        {
            string response = Call(server, "template.instantiate", $"{{\"template\":\"cloudRelief\",\"arguments\":{{{arguments}}},\"name\":\"{name}\"}}");
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
    }
}
