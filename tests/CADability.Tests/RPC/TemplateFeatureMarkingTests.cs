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
    /// After a template run the faces it made carry a feature name and the template feature (template, id, parameter
    /// values); the faces of the input solid it only trimmed or split keep what they had.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class TemplateFeatureMarkingTests
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

        /// <summary>The hole templates and a 20x20x20 block with its top face.</summary>
        private static MCPServer ServerWithBlock()
        {
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };
            using JsonDocument document = JsonDocument.Parse(System.IO.File.ReadAllText(TemplateFile()));
            foreach (JsonElement template in document.RootElement.GetProperty("templates").EnumerateArray())
                server.templates[template.GetProperty("name").GetString()!] = template.GetProperty("items").EnumerateArray().Select(i => i.Clone()).ToList();
            Assert.IsNull(Error(Call(server, "solid.box", "{\"name\":\"block\",\"origin\":[0,0,0],\"sizeX\":20,\"sizeY\":20,\"sizeZ\":20}")));
            server.namedItems["top"] = PlaneAt(FirstSolid(server.namedItems["block"]), 2, 20);
            return server;
        }

        /// <summary>The face of a solid lying in the plane where the given coordinate (0: x, 1: y, 2: z) has the value.</summary>
        private static Face PlaneAt(Solid solid, int axis, double value)
        {
            return solid.Shells[0].Faces.Single(f =>
            {
                BoundingBox b = f.GetBoundingCube();
                (double min, double max) = axis switch { 0 => (b.Xmin, b.Xmax), 1 => (b.Ymin, b.Ymax), _ => (b.Zmin, b.Zmax) };
                return f.Surface is PlaneSurface && Math.Abs(min - value) < 1e-9 && Math.Abs(max - value) < 1e-9;
            });
        }

        private static Solid Instantiate(MCPServer server, string template, string arguments, string name)
        {
            string response = Call(server, "template.instantiate", $"{{\"template\":\"{template}\",\"arguments\":{arguments},\"name\":\"{name}\"}}");
            Assert.IsNull(Error(response), response);
            return FirstSolid(server.namedItems[name]);
        }

        [TestMethod]
        public void TheNewFacesOfAHoleAreOneFeature()
        {
            MCPServer server = ServerWithBlock();
            Solid drilled = Instantiate(server, "durchgangsbohrung", "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"entry\":\"chamfer\"}", "drilled");
            Shell shell = drilled.Shells[0];
            Dictionary<string, HashSet<Face>> features = shell.GetFeatures();
            Assert.AreEqual(1, features.Count, string.Join(", ", features.Keys));
            (string name, HashSet<Face> faces) = (features.Keys.Single(), features.Values.Single());
            Assert.AreEqual("Through hole 1", name);
            // the wall (one or two faces) and the chamfer, nothing of the block
            Assert.IsTrue(faces.All(f => !(f.Surface is PlaneSurface)), "only the cylinder and the cone of the chamfer are new");
            Assert.IsTrue(faces.Any(f => f.Surface is ConicalSurface), "the chamfer belongs to the hole");
            Assert.AreEqual(6, shell.Faces.Count(f => f.Surface is PlaneSurface && TemplateFeatures.Of(f).Count == 0), "the six faces of the block stay unmarked");
            List<TemplateFeatureInfo> infos = faces.Select(f => TemplateFeatures.Of(f).Single()).ToList();
            Assert.AreEqual(1, infos.Select(i => i.Id).Distinct().Count(), "one id for the whole feature");
            Assert.AreEqual("durchgangsbohrung", infos[0].Template);
            Assert.AreEqual(name, infos[0].Name);
            using JsonDocument parameters = JsonDocument.Parse(infos[0].Parameters);
            Assert.AreEqual(8, parameters.RootElement.GetProperty("d").GetDouble());
            Assert.AreEqual("chamfer", parameters.RootElement.GetProperty("entry").GetString());
            Assert.AreEqual(20, parameters.RootElement.GetProperty("pos")[2].GetDouble());
            Assert.IsFalse(parameters.RootElement.TryGetProperty("body", out _), "objects are input, not values");
            CollectionAssert.AreEquivalent(faces.ToList(), TemplateFeatures.FacesOf(shell, infos[0].Id));
        }

        /// <summary>A second hole across the first: the split wall of the first keeps its mark, the second is a feature of its own.</summary>
        [TestMethod]
        public void ACrossingHoleKeepsTheFirstFeature()
        {
            MCPServer server = ServerWithBlock();
            Solid first = Instantiate(server, "durchgangsbohrung", "{\"face\":\"top\",\"pos\":\"p(10,10,20)\"}", "first");
            server.namedItems["side"] = PlaneAt(first, 0, 20);
            Solid second = Instantiate(server, "durchgangsbohrung", "{\"body\":\"first\",\"face\":\"side\",\"pos\":\"p(20,10,10)\",\"d\":6}", "second");
            Dictionary<string, HashSet<Face>> features = second.Shells[0].GetFeatures();
            CollectionAssert.AreEquivalent(new[] { "Through hole 1", "Through hole 2" }, features.Keys.ToArray());
            Assert.IsTrue(features["Through hole 1"].Count >= 2, "the wall of the first hole is split by the second one");
            Assert.IsTrue(features["Through hole 1"].All(f => f.Surface is CylindricalSurface cs && Math.Abs(cs.RadiusX - 4) < 1e-9), "the parts of the first wall");
            Assert.IsTrue(features["Through hole 2"].All(f => f.Surface is CylindricalSurface cs && Math.Abs(cs.RadiusX - 3) < 1e-9), "the second wall only");
            Assert.IsFalse(features["Through hole 1"].Overlaps(features["Through hole 2"]));
            foreach (Face face in features["Through hole 1"]) Assert.AreEqual("Through hole 1", TemplateFeatures.Of(face).Single().Name);
        }

        /// <summary>
        /// A face of the input that carries a feature hands its attributes to a fillet made next to it: the fillet of the
        /// new hole must not take over that feature.
        /// </summary>
        [TestMethod]
        public void ANewFaceDropsInheritedMarks()
        {
            MCPServer server = ServerWithBlock();
            Solid block = FirstSolid(server.namedItems["block"]);
            Face top = (Face)server.namedItems["top"];
            block.Shells[0].AddFeature("Earlier 1", new[] { top });
            TemplateFeatureList earlier = new TemplateFeatureList();
            earlier.Features.Add(new TemplateFeatureInfo("Earlier 1", "earlier", "earlier-id", "{}"));
            top.UserData[TemplateFeatures.Key] = earlier;
            Solid drilled = Instantiate(server, "sacklochbohrung", "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"entry\":\"round\"}", "drilled");
            Dictionary<string, HashSet<Face>> features = drilled.Shells[0].GetFeatures();
            Assert.AreEqual(1, features["Earlier 1"].Count, "only the top face keeps the earlier feature");
            Assert.IsTrue(features["Earlier 1"].Single().Surface is PlaneSurface);
            HashSet<Face> hole = features["Blind hole 1"];
            Assert.IsTrue(hole.Any(f => !(f.Surface is PlaneSurface) && !(f.Surface is CylindricalSurface)), "the fillet is part of the hole");
            foreach (Face face in hole) Assert.AreEqual("sacklochbohrung", TemplateFeatures.Of(face).Single().Template);
        }

        [TestMethod]
        public void TheMarksAreSavedWithTheSolid()
        {
            MCPServer server = ServerWithBlock();
            Solid drilled = Instantiate(server, "kugelmulde", "{\"face\":\"top\",\"pos\":\"p(10,10,20)\"}", "drilled");
            Solid read = (Solid)JsonSerialize.FromString(JsonSerialize.ToString(drilled));
            Dictionary<string, HashSet<Face>> features = read.Shells[0].GetFeatures();
            Assert.AreEqual("Spherical dimple 1", features.Keys.Single());
            TemplateFeatureInfo info = TemplateFeatures.Of(features.Values.Single().First()).Single();
            Assert.AreEqual("kugelmulde", info.Template);
            Assert.AreEqual(TemplateFeatures.Of(drilled.Shells[0].Faces.First(f => TemplateFeatures.Of(f).Count > 0)).Single().Id, info.Id);
            StringAssert.Contains(info.Parameters, "\"depth\":1.5");
        }

        /// <summary>The application runs the template with the values of its inputs: the result is marked the same way.</summary>
        [TestMethod]
        public void TheApplicationPathMarksToo()
        {
            MCPServer server = ServerWithBlock();
            (Face face, Solid? copy) = MCPServer.CopyWithSolid((Face)server.namedItems["top"]);
            Dictionary<string, object> values = new Dictionary<string, object> { ["face"] = face, ["pos"] = new GeoPoint(5, 5, 20), ["d"] = 4.0 };
            object? result = server.ExecuteTemplate("durchgangsbohrung", values, System.Threading.CancellationToken.None);
            Assert.IsNotNull(result);
            Assert.AreEqual("Through hole 1", FirstSolid(result).Shells[0].GetFeatures().Keys.Single());
        }
    }
}
