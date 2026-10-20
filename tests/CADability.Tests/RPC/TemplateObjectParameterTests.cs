using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// Templates with object parameters: a required 'profile' and an optional 'solid', and steps made
    /// conditional with 'when' and defined("name").
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class TemplateObjectParameterTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static Solid FirstSolid(object item) => item as Solid ?? ((List<Solid>)item)[0];
        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        /// <summary>Records the "rays" template: rods from the origin along a spiral, subtracted from the optional hull.</summary>
        private static MCPServer ServerWithRaysTemplate()
        {
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };
            Assert.IsNull(Error(Call(server, "sketch.create", "{\"plane\":{\"standard\":\"XY\"},\"name\":\"prof\"}")));
            Assert.IsNull(Error(Call(server, "sketch.add_circle", "{\"sketch\":\"prof\",\"center\":[0,0],\"radius\":3}")));
            // a hollow ball like a lamp shade: the rods overlap only near the origin, inside the cavity
            Assert.IsNull(Error(Call(server, "solid.sphere", "{\"center\":[0,0,0],\"radius\":80,\"name\":\"outer\"}")));
            Assert.IsNull(Error(Call(server, "solid.sphere", "{\"center\":[0,0,0],\"radius\":75,\"name\":\"inner\"}")));
            Assert.IsNull(Error(Call(server, "solid.boolean", "{\"op\":\"difference\",\"a\":\"outer\",\"b\":\"inner\",\"name\":\"ball\"}")));

            string begin = @"{""name"":""rays"",""label"":""Rays"",""parameters"":[
                {""name"":""profile"",""value"":""prof"",""label"":""Profile"",""input"":{""kind"":""profile""}},
                {""name"":""hull"",""value"":""ball"",""label"":""Hull"",""input"":{""kind"":""solid"",""optional"":true,""replace"":true}},
                {""name"":""n"",""value"":5,""label"":""Count"",""input"":{""kind"":""integer""}}]}";
            Assert.IsNull(Error(Call(server, "template.begin", begin)));
            string batch = @"{""calls"":[
                {""jsonrpc"":""2.0"",""id"":1,""method"":""solid.extrude"",""params"":{""profile"":""profile"",""length"":200,""direction"":[0,0,1],""name"":""rod""}},
                {""jsonrpc"":""2.0"",""id"":2,""method"":""pattern.by_formula_solids"",""params"":{""solids"":""rod"",""variables"":[{""name"":""i"",""start"":0,""step"":1,""count"":""n""}],
                    ""formulas"":{""lat"":""75 - 150*i/(n-1)"",""azi"":""i*137.50776""},
                    ""transform"":""rotate(p(0,0,0), v(0,0,1), azi*pi/180) * rotate(p(0,0,0), v(0,1,0), (90-lat)*pi/180)"",""name"":""beams""}},
                {""jsonrpc"":""2.0"",""id"":3,""when"":""defined(\""hull\"")"",""method"":""solid.boolean"",""params"":{""op"":""difference"",""a"":""hull"",""b"":""beams"",""name"":""result""}},
                {""jsonrpc"":""2.0"",""id"":4,""when"":""!defined(\""hull\"")"",""method"":""workspace.select"",""params"":{""selector"":""beams"",""name"":""result""}}]}";
            string? batchError = Error(Call(server, "rpc.batch", batch));
            Assert.IsNull(batchError, "the recording itself must build: " + batchError);
            Assert.IsNull(Error(Call(server, "template.commit", "{\"result\":\"result\"}")));
            return server;
        }

        [TestMethod]
        public void OptionalSolidLeftOutYieldsTheTools()
        {
            MCPServer server = ServerWithRaysTemplate();
            string response = Call(server, "template.instantiate", "{\"template\":\"rays\",\"arguments\":{\"profile\":\"prof\",\"n\":7},\"name\":\"onlyRays\"}");
            Assert.IsNull(Error(response), response);
            Assert.IsTrue(server.namedItems.TryGetValue("onlyRays", out object? value));
            Assert.IsInstanceOfType(value, typeof(List<Solid>));
            Assert.AreEqual(7, ((List<Solid>)value!).Count, "without hull the result are the rods");
        }

        [TestMethod]
        public void OptionalSolidGivenGetsTheHoles()
        {
            MCPServer server = ServerWithRaysTemplate();
            double ballVolume = FirstSolid(server.namedItems["ball"]).Volume(0.01);
            string response = Call(server, "template.instantiate", "{\"template\":\"rays\",\"arguments\":{\"profile\":\"prof\",\"hull\":\"ball\"},\"name\":\"perforated\"}");
            Assert.IsNull(Error(response), response);
            object perforated = server.namedItems["perforated"];
            Solid solid = FirstSolid(perforated);
            Assert.IsTrue(solid.Volume(0.01) < ballVolume - 100, "the holes must have removed material");
            Assert.AreEqual(ballVolume, FirstSolid(server.namedItems["ball"]).Volume(0.01), 1e-6, "the argument itself must stay untouched");
        }

        [TestMethod]
        public void RequiredProfileMissingIsReported()
        {
            MCPServer server = ServerWithRaysTemplate();
            string? error = Error(Call(server, "template.instantiate", "{\"template\":\"rays\",\"arguments\":{\"n\":3},\"name\":\"noProfile\"}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "profile");
            Assert.IsFalse(server.namedItems.ContainsKey("noProfile"));
        }

        [TestMethod]
        public void OptionalSolidLeftOutIsNotTakenFromTheWorkspace()
        {
            // "ball" exists in the outer workspace, and so would "hull" if somebody had stored it there:
            // an object parameter left out must not pick up a workspace item of the same name
            MCPServer server = ServerWithRaysTemplate();
            server.namedItems["hull"] = server.namedItems["ball"];
            string response = Call(server, "template.instantiate", "{\"template\":\"rays\",\"arguments\":{\"profile\":\"prof\",\"n\":3},\"name\":\"rays3\"}");
            Assert.IsNull(Error(response), response);
            Assert.IsInstanceOfType(server.namedItems["rays3"], typeof(List<Solid>));
        }

        private static string TemplateFile([System.Runtime.CompilerServices.CallerFilePath] string thisFile = "")
            => System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile)!, "..", "Files", "Templates", "SupereiTemplate.json");

        /// <summary>Loads the templates of a template file the way the application does.</summary>
        private static void LoadTemplates(MCPServer server, string file)
        {
            using JsonDocument document = JsonDocument.Parse(System.IO.File.ReadAllText(file));
            foreach (JsonElement template in document.RootElement.GetProperty("templates").EnumerateArray())
            {
                List<JsonElement> items = new List<JsonElement>();
                foreach (JsonElement item in template.GetProperty("items").EnumerateArray()) items.Add(item.Clone());
                server.templates[template.GetProperty("name").GetString()!] = items;
            }
        }

        [TestMethod]
        public void SavedRaindropsTemplateStaysBetweenTheLatitudes()
        {
            // the "raindrops" template of the template file: rays from the origin (no shade), spread with the golden
            // angle between latTop and latBottom - whatever the count, no ray may leave that band
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };
            LoadTemplates(server, TemplateFile());
            Assert.IsNull(Error(Call(server, "sketch.create", "{\"plane\":{\"standard\":\"XY\"},\"name\":\"prof\"}")));
            Assert.IsNull(Error(Call(server, "sketch.add_circle", "{\"sketch\":\"prof\",\"center\":[0,0],\"radius\":3}")));
            foreach ((int n, double top, double bottom) in new[] { (25, 75.0, -75.0), (120, 75.0, -75.0), (9, 60.0, 10.0) })
            {
                string response = Call(server, "template.instantiate",
                    $"{{\"template\":\"raindrops\",\"arguments\":{{\"profile\":\"prof\",\"n\":\"{n}\",\"latTop\":\"{top}\",\"latBottom\":\"{bottom}\"}},\"name\":\"rays{n}\"}}");
                Assert.IsNull(Error(response), response);
                List<Solid> rays = (List<Solid>)server.namedItems[$"rays{n}"];
                Assert.AreEqual(n, rays.Count);
                BoundingBox ext = BoundingBox.EmptyBoundingBox;
                foreach (Solid ray in rays) ext.MinMax(ray.GetExtent(0.0));
                // rays of length 200 start at the origin; 3 is the radius of the profile
                Assert.IsTrue(ext.Zmax <= 200 * Math.Sin(top * Math.PI / 180) + 3 + 1e-6, $"n={n}: a ray is above {top}°");
                Assert.IsTrue(ext.Zmin >= Math.Min(0.0, 200 * Math.Sin(bottom * Math.PI / 180)) - 3 - 1e-6, $"n={n}: a ray is below {bottom}°");
            }
        }

        [TestMethod]
        public void SavedValueTemplatesStillRun()
        {
            // superEgg, and lampShade which instantiates superEgg twice: arguments are now evaluated in the caller's
            // workspace before template.begin, which must not change these templates
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };
            LoadTemplates(server, TemplateFile());
            foreach (string template in new[] { "superEgg", "lampShade" })
            {
                string response = Call(server, "template.instantiate", $"{{\"template\":\"{template}\",\"name\":\"{template}Instance\"}}");
                Assert.IsNull(Error(response), response);
                Assert.IsTrue(server.namedItems.ContainsKey(template + "Instance"), template + " produced no result");
            }
        }

        [TestMethod]
        public void SavedRaindropsTemplatePerforatesTheShade()
        {
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };
            LoadTemplates(server, TemplateFile());
            Assert.IsNull(Error(Call(server, "sketch.create", "{\"plane\":{\"standard\":\"XY\"},\"name\":\"prof\"}")));
            Assert.IsNull(Error(Call(server, "sketch.add_circle", "{\"sketch\":\"prof\",\"center\":[0,0],\"radius\":3}")));
            // a hollow box around (0,0,100) as shade: the rays start in the center of its bounding box
            Assert.IsNull(Error(Call(server, "solid.box", "{\"name\":\"outer\",\"origin\":[-100,-100,0],\"sizeX\":200,\"sizeY\":200,\"sizeZ\":200}")));
            Assert.IsNull(Error(Call(server, "solid.box", "{\"name\":\"inner\",\"origin\":[-95,-95,5],\"sizeX\":190,\"sizeY\":190,\"sizeZ\":190}")));
            Assert.IsNull(Error(Call(server, "solid.boolean", "{\"op\":\"difference\",\"a\":\"outer\",\"b\":\"inner\",\"name\":\"shade\"}")));
            int shadeFaces = FirstSolid(server.namedItems["shade"]).Shells[0].Faces.Length;
            string response = Call(server, "template.instantiate", "{\"template\":\"raindrops\",\"arguments\":{\"profile\":\"prof\",\"shade\":\"shade\",\"n\":\"10\"},\"name\":\"holed\"}");
            Assert.IsNull(Error(response), response);
            Solid holed = FirstSolid(server.namedItems["holed"]);
            Assert.IsTrue(holed.Shells[0].Faces.Length > shadeFaces, "the rays must have made holes into the shade");
        }
    }
}
