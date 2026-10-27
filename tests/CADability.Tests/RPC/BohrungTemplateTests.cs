using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// The "sacklochbohrung" template of BohrungTemplate.json: a blind hole perpendicular to a face, with a sharp,
    /// chamfered or rounded entry and a flat, conical or spherical bottom. The removed volume is compared with the
    /// volume of the hole computed here from its shape.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class BohrungTemplateTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();
        private static Solid FirstSolid(object item) => item as Solid ?? ((List<Solid>)item)[0];

        private static string TemplateFile([System.Runtime.CompilerServices.CallerFilePath] string thisFile = "")
            => System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile)!, "..", "..", "..", "ShapeIt", "Templates", "BohrungTemplate.json");

        /// <summary>Loads the templates of the file the way the application does, and a 20x20x20 block with its faces.</summary>
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
            Assert.IsNull(Error(Call(server, "solid.box", "{\"name\":\"block\",\"origin\":[0,0,0],\"sizeX\":20,\"sizeY\":20,\"sizeZ\":20}")));
            Solid block = FirstSolid(server.namedItems["block"]);
            server.namedItems["top"] = PlanarFace(block, 2);
            server.namedItems["side"] = PlanarFace(block, 0);
            return server;
        }

        /// <summary>The face of the block lying in the plane where the given coordinate (0: x, 2: z) is 20.</summary>
        private static Face PlanarFace(Solid solid, int axis)
        {
            return solid.Shells[0].Faces.Single(f =>
            {
                BoundingBox box = f.GetBoundingCube();
                double min = axis == 0 ? box.Xmin : box.Zmin, max = axis == 0 ? box.Xmax : box.Zmax;
                return Math.Abs(min - 20) < 1e-9 && Math.Abs(max - 20) < 1e-9;
            });
        }

        private const double R = 4, Depth = 15; // d = 8 as in the template
        private static readonly double tipHalf = 59 * Math.PI / 180;

        /// <summary>Instantiates the template on the given face and returns the volume taken out of the block.</summary>
        private static double RemovedVolume(MCPServer server, string arguments)
        {
            string response = Call(server, "template.instantiate", $"{{\"template\":\"sacklochbohrung\",\"arguments\":{arguments},\"name\":\"drilled\"}}");
            Assert.IsNull(Error(response), response);
            Solid drilled = FirstSolid(server.namedItems["drilled"]);
            Assert.AreEqual(8000, FirstSolid(server.namedItems["block"]).Volume(0.001), 1e-6, "the block itself must stay untouched");
            return 8000 - drilled.Volume(0.0001);
        }

        private static double Cylinder => Math.PI * R * R * Depth;

        [TestMethod]
        public void FlatBottomSharpEntry()
        {
            MCPServer server = ServerWithTemplates();
            Assert.AreEqual(Cylinder, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\"}"), 1e-3);
        }

        [TestMethod]
        public void PointedConeBottom()
        {
            MCPServer server = ServerWithTemplates();
            double h = R / Math.Tan(tipHalf);
            Assert.AreEqual(Cylinder + Math.PI * R * R * h / 3, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"bottom\":\"cone\"}"), 1e-3);
        }

        [TestMethod]
        public void TruncatedConeBottom()
        {
            MCPServer server = ServerWithTemplates();
            double rb = 1.5, h = (R - rb) / Math.Tan(tipHalf);
            double frustum = Math.PI * h * (R * R + R * rb + rb * rb) / 3;
            Assert.AreEqual(Cylinder + frustum, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"bottom\":\"cone\",\"bottomDiameter\":3}"), 1e-3);
        }

        [TestMethod]
        public void SphericalBottom()
        {
            MCPServer server = ServerWithTemplates();
            Assert.AreEqual(Cylinder + 2.0 / 3.0 * Math.PI * R * R * R, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"bottom\":\"sphere\"}"), 1e-3);
        }

        /// <summary>A 45° chamfer of width c: a triangle with legs c revolved around the axis.</summary>
        [TestMethod]
        public void ChamferedEntry()
        {
            MCPServer server = ServerWithTemplates();
            double c = 1;
            double chamfer = 2 * Math.PI * (R + c / 3) * c * c / 2;
            Assert.AreEqual(Cylinder + chamfer, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"entry\":\"chamfer\",\"entrySize\":1}"), 1e-3);
        }

        /// <summary>A fillet of radius rho: the square rho x rho without the quarter circle, revolved (Pappus).</summary>
        [TestMethod]
        public void RoundedEntryWithSphericalBottom()
        {
            MCPServer server = ServerWithTemplates();
            double rho = 1.5;
            double area = rho * rho * (1 - Math.PI / 4);
            double centroid = rho * (10 - 3 * Math.PI) / (12 - 3 * Math.PI); // distance of the centroid from the edge
            double fillet = 2 * Math.PI * (R + centroid) * area;
            double expected = Cylinder + fillet + 2.0 / 3.0 * Math.PI * R * R * R;
            Assert.AreEqual(expected, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"entry\":\"round\",\"entrySize\":1.5,\"bottom\":\"sphere\"}"), 1e-2);
        }

        /// <summary>A face whose normal is the x-axis takes the other branch for the radial direction of the profile;
        /// the position is given off the face and put onto it.</summary>
        [TestMethod]
        public void SideFaceWithPointOffTheFace()
        {
            MCPServer server = ServerWithTemplates();
            double h = R / Math.Tan(tipHalf);
            Assert.AreEqual(Cylinder + Math.PI * R * R * h / 3,
                RemovedVolume(server, "{\"face\":\"side\",\"pos\":\"p(35,10,8)\",\"bottom\":\"cone\",\"entry\":\"chamfer\",\"entrySize\":0.5}")
                    - 2 * Math.PI * (R + 0.5 / 3) * 0.125, 1e-3);
        }

        /// <summary>A template loaded from a file was never recorded here: check its definition and icon as template.begin would.</summary>
        [TestMethod]
        public void DefinitionIsValid()
        {
            using JsonDocument document = JsonDocument.Parse(System.IO.File.ReadAllText(TemplateFile()));
            foreach (JsonElement template in document.RootElement.GetProperty("templates").EnumerateArray())
            {
                JsonElement begin = template.GetProperty("items")[0].GetProperty("params");
                begin.TryGetProperty("presets", out JsonElement presets);
                begin.TryGetProperty("checks", out JsonElement checks);
                TemplateParameterRules.ValidateDefinition(begin.GetProperty("parameters"), presets, checks);
                if (begin.TryGetProperty("icon", out JsonElement icon)) Assert.IsNull(TemplateIcon.Problem(icon.GetString()!));
            }
            Assert.IsNull(TemplateIcon.Problem(document.RootElement.GetProperty("icon").GetString()!));
        }

        [TestMethod]
        public void BottomDiameterTooLargeIsRefused()
        {
            MCPServer server = ServerWithTemplates();
            string? error = Error(Call(server, "template.instantiate", "{\"template\":\"sacklochbohrung\",\"arguments\":{\"face\":\"top\",\"bottom\":\"cone\",\"bottomDiameter\":8},\"name\":\"x\"}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "bottom diameter");
        }
    }
}
