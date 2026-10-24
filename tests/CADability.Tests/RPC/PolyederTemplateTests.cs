using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// The "polyeder" template of PolyederTemplate.json: a tetrahedron whose farthest vertex from the insphere center
    /// is cut off n times by a tangent plane of the insphere, in an rpc.batch with 'repeat'.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class PolyederTemplateTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        private static string TemplateFile([System.Runtime.CompilerServices.CallerFilePath] string thisFile = "")
            => System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile)!, "..", "Files", "Templates", "PolyederTemplate.json");

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

        // the default corners of the template
        private static readonly GeoPoint A = new GeoPoint(0, 0, 0), B = new GeoPoint(100, 0, 0), C = new GeoPoint(20, 70, 0), D = new GeoPoint(30, 20, 90);

        private static (GeoPoint center, double radius) Insphere()
        {
            double sA = ((C - B) ^ (D - B)).Length / 2, sB = ((C - A) ^ (D - A)).Length / 2;
            double sC = ((B - A) ^ (D - A)).Length / 2, sD = ((B - A) ^ (C - A)).Length / 2;
            double sum = sA + sB + sC + sD;
            GeoPoint center = A + (sB * (B - A) + sC * (C - A) + sD * (D - A)) / sum;
            double det = Math.Abs(((B - A) ^ (C - A)) * (D - A));
            return (center, det / (2 * sum));
        }

        private static Solid Instantiate(MCPServer server, int n)
        {
            string response = Call(server, "template.instantiate", $"{{\"template\":\"polyeder\",\"arguments\":{{\"n\":{n}}},\"name\":\"p{n}\"}}");
            Assert.IsNull(Error(response), response);
            object item = server.namedItems[$"p{n}"];
            return item as Solid ?? ((List<Solid>)item)[0];
        }

        [TestMethod]
        public void WithoutCutsTheTetrahedronIsReturned()
        {
            MCPServer server = ServerWithTemplates();
            Solid tetrahedron = Instantiate(server, 0);
            Assert.AreEqual(4, tetrahedron.Shells[0].Faces.Length);
            double volume = Math.Abs(((B - A) ^ (C - A)) * (D - A)) / 6;
            Assert.AreEqual(volume, tetrahedron.Volume(1e-6), volume * 1e-6);
        }

        [TestMethod]
        public void CornersArePointParameters()
        {
            MCPServer server = ServerWithTemplates();
            string response = Call(server, "template.instantiate",
                "{\"template\":\"polyeder\",\"arguments\":{\"A\":\"p(0,0,0)\",\"B\":\"p(50,0,0)\",\"C\":\"p(0,50,0)\",\"D\":\"p(0,0,50)\",\"n\":0},\"name\":\"corner\"}");
            Assert.IsNull(Error(response), response);
            object item = server.namedItems["corner"];
            Solid tetrahedron = item as Solid ?? ((List<Solid>)item)[0];
            Assert.AreEqual(50.0 * 50 * 50 / 6, tetrahedron.Volume(1e-6), 1e-3);
        }

        [TestMethod]
        public void EveryCutAddsAFaceTangentToTheInsphere()
        {
            MCPServer server = ServerWithTemplates();
            (GeoPoint center, double radius) = Insphere();
            const int n = 12;
            Solid polyeder = Instantiate(server, n);
            Face[] faces = polyeder.Shells[0].Faces;
            // a tangent plane never removes another tangent face completely: its touching point stays inside
            Assert.AreEqual(4 + n, faces.Length);
            foreach (Face face in faces)
            {
                Assert.IsInstanceOfType(face.Surface, typeof(PlaneSurface));
                double distance = Math.Abs(((PlaneSurface)face.Surface).Plane.Distance(center));
                Assert.AreEqual(radius, distance, radius * 1e-8, "every face must touch the insphere");
            }
            double sphereVolume = 4.0 / 3.0 * Math.PI * radius * radius * radius;
            double tetrahedronVolume = Math.Abs(((B - A) ^ (C - A)) * (D - A)) / 6;
            double volume = polyeder.Volume(1e-6);
            Assert.IsTrue(volume > sphereVolume && volume < tetrahedronVolume, $"volume {volume} must lie between sphere {sphereVolume} and tetrahedron {tetrahedronVolume}");
        }
    }
}
