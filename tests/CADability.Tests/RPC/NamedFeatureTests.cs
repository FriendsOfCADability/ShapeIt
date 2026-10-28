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
    /// <see cref="Shell.FeatureOfFace"/>: from one face of a named feature (here made by the hole templates) to all its
    /// faces and the caps that close it, so that it can be shown, copied or removed as a whole.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class NamedFeatureTests
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

        /// <summary>The feature as a solid, made the way GetFeatureProperties of the application does it.</summary>
        private static Solid FeatureSolid(List<Face> featureFaces, List<Face> caps)
        {
            Dictionary<Edge, Edge> clonedEdges = new Dictionary<Edge, Edge>();
            Dictionary<Vertex, Vertex> clonedVertices = new Dictionary<Vertex, Vertex>();
            List<Face> faces = featureFaces.Select(f => f.Clone(clonedEdges, clonedVertices)).ToList();
            foreach (Face cap in caps)
            {
                Face reversed = (Face)cap.Clone();
                reversed.ReverseOrientation();
                faces.Add(reversed);
            }
            BoundingBox ext = BoundingBox.EmptyBoundingBox;
            ext.MinMax(faces);
            Shell.ConnectFaces(faces.ToArray(), Math.Max(Precision.eps, ext.Size * 1e-6));
            Shell feature = Shell.FromFaces(faces.ToArray());
            feature.AssertOutwardOrientation();
            Assert.AreEqual(0, feature.OpenEdgesExceptPoles.Length, "the caps close the feature");
            return Solid.MakeSolid(feature);
        }

        private static Face MarkedFace(Solid solid, string featureName) => solid.Shells[0].Faces.First(f => solid.Shells[0].GetFeatures()[featureName].Contains(f));

        [TestMethod]
        public void AThroughHoleIsClosedByTwoDiscs()
        {
            MCPServer server = ServerWithBlock();
            Solid drilled = Instantiate(server, "durchgangsbohrung", "{\"face\":\"top\",\"pos\":\"p(10,10,20)\"}", "drilled");
            Shell shell = drilled.Shells[0];
            Assert.IsTrue(shell.FeatureOfFace(MarkedFace(drilled, "Through hole 1"), out string name, out List<Face> featureFaces, out List<Face> caps, out bool isGap));
            Assert.AreEqual("Through hole 1", name);
            Assert.IsTrue(isGap, "a hole is a gap");
            Assert.AreEqual(2, caps.Count);
            Assert.IsTrue(caps.All(c => c.Surface is PlaneSurface));
            // against the faces around them: the top disc points down, the bottom disc up
            foreach (Face cap in caps)
            {
                GeoPoint2D uv = cap.Area.GetSomeInnerPoint();
                double z = cap.Surface.PointAt(uv).z, nz = cap.Surface.GetNormal(uv).Normalized.z;
                Assert.AreEqual(z > 10 ? -1 : 1, nz, 1e-9, $"cap at z = {z}");
            }
            Assert.AreEqual(Math.PI * 16 * 20, FeatureSolid(featureFaces, caps).Volume(0.0001), 1e-3, "the feature is the plug of the hole");
        }

        /// <summary>
        /// Removing the feature the way the application does (the caps reversed close the rest), then drilling again at
        /// another position: the rest is a valid solid.
        /// </summary>
        [TestMethod]
        public void AHoleIsMovedByRemovingItsFeature()
        {
            MCPServer server = ServerWithBlock();
            Solid drilled = Instantiate(server, "durchgangsbohrung", "{\"face\":\"top\",\"pos\":\"p(10,10,20)\"}", "drilled");
            Shell shell = drilled.Shells[0];
            Assert.IsTrue(shell.FeatureOfFace(MarkedFace(drilled, "Through hole 1"), out _, out List<Face> featureFaces, out List<Face> caps, out _));
            foreach (Face cap in caps) cap.ReverseOrientation();
            Assert.IsTrue(shell.AddAndRemoveFaces(caps, featureFaces), "the hole can be removed");
            Assert.AreEqual(8000, drilled.Volume(0.0001), 1e-6, "the block is whole again");
            foreach (Face face in shell.Faces.Where(f => f.Surface is PlaneSurface))
            {   // every face points out of the block
                GeoPoint2D uv = face.Area.GetSomeInnerPoint();
                GeoPoint p = face.Surface.PointAt(uv);
                GeoVector n = face.Surface.GetNormal(uv).Normalized;
                Assert.IsTrue((p + n - new GeoPoint(10, 10, 10)).Length > (p - new GeoPoint(10, 10, 10)).Length, $"face at {p} points inwards");
            }
            server.namedItems["healed"] = drilled;
            server.namedItems["healedTop"] = shell.Faces.First(f => f.Surface is PlaneSurface && Math.Abs(f.GetBoundingCube().Zmin - 20) < 1e-9 && f.GetBoundingCube().Xmax - f.GetBoundingCube().Xmin > 19);
            Solid moved = Instantiate(server, "durchgangsbohrung", "{\"body\":\"healed\",\"face\":\"healedTop\",\"pos\":\"p(6,14,20)\"}", "moved");
            Assert.AreEqual(8000 - Math.PI * 16 * 20, moved.Volume(0.0001), 1e-3);
        }

        [TestMethod]
        public void ABlindHoleWithAFilletIsClosedByOneDisc()
        {
            MCPServer server = ServerWithBlock();
            Solid drilled = Instantiate(server, "sacklochbohrung", "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"entry\":\"round\",\"bottom\":\"cone\"}", "drilled");
            Assert.IsTrue(drilled.Shells[0].FeatureOfFace(MarkedFace(drilled, "Blind hole 1"), out _, out List<Face> featureFaces, out List<Face> caps, out bool isGap));
            Assert.IsTrue(isGap);
            Assert.AreEqual(1, caps.Count, "only the top is open");
            Assert.AreEqual(8000 - drilled.Volume(0.0001), FeatureSolid(featureFaces, caps).Volume(0.0001), 1e-3, "the feature is what was drilled out");
        }

        /// <summary>A slot along a shaft: the cap is the part of the mantle inside the outline of the slot.</summary>
        [TestMethod]
        public void ASlotInAShaftIsClosedByAPieceOfTheMantle()
        {
            MCPServer server = ServerWithBlock();
            Assert.IsNull(Error(Call(server, "solid.cylinder", "{\"name\":\"shaft\",\"start\":[0,0,0],\"end\":[0,0,60],\"radius\":20}")));
            server.namedItems["mantle"] = FirstSolid(server.namedItems["shaft"]).Shells[0].Faces.Single(f => f.Surface is CylindricalSurface
                && f.GetBoundingCube().Xmin < -1 && f.GetBoundingCube().Xmax > 1 && f.GetBoundingCube().Ymax > 19);
            Solid slotted = Instantiate(server, "langloch", "{\"face\":\"mantle\",\"p1\":\"p(0,20,10)\",\"p2\":\"p(0,20,40)\",\"d\":6,\"depth\":5}", "slotted");
            Assert.IsTrue(slotted.Shells[0].FeatureOfFace(MarkedFace(slotted, "Slot 1"), out _, out List<Face> featureFaces, out List<Face> caps, out bool isGap));
            Assert.IsTrue(isGap);
            Assert.AreEqual(1, caps.Count);
            Assert.IsTrue(caps[0].Surface is CylindricalSurface, "the cap lies on the mantle");
            // the removed volume, integrated across the slot as in BohrungTemplateTests
            double R = 20, r = 3, bottom = R - 5, expected = 0;
            const int n = 20000;
            for (int i = 0; i < n; i++)
            {
                double y = -r + (i + 0.5) * 2 * r / n, height = Math.Sqrt(R * R - y * y) - bottom;
                expected += (30 + 2 * Math.Sqrt(r * r - y * y)) * height * 2 * r / n;
            }
            Assert.AreEqual(expected, FeatureSolid(featureFaces, caps).Volume(0.0001), 1e-2);
        }

        /// <summary>
        /// The first of two crossing holes: besides the discs at top and bottom its feature has a window where the second
        /// hole ends in it (a through hole from the side ends where it reaches the first one). The loop of the window winds
        /// around the wall of the second hole, so the window is closed by the wall of the first one, continued across it.
        /// </summary>
        [TestMethod]
        public void ACrossedHoleIsClosedAcrossItsWindow()
        {
            MCPServer server = ServerWithBlock();
            Solid first = Instantiate(server, "durchgangsbohrung", "{\"face\":\"top\",\"pos\":\"p(10,10,20)\"}", "first");
            server.namedItems["side"] = PlaneAt(first, 0, 20);
            Solid second = Instantiate(server, "durchgangsbohrung", "{\"body\":\"first\",\"face\":\"side\",\"pos\":\"p(20,10,10)\",\"d\":6}", "second");
            Assert.IsTrue(second.Shells[0].FeatureOfFace(MarkedFace(second, "Through hole 1"), out _, out List<Face> featureFaces, out List<Face> caps, out bool isGap));
            Assert.IsTrue(isGap);
            Assert.AreEqual(2, caps.Count(c => c.Surface is PlaneSurface), "top and bottom");
            Assert.AreEqual(1, caps.Count(c => c.Surface is CylindricalSurface cs && Math.Abs(cs.RadiusX - 4) < 1e-9), "the window, on the wall of the first hole");
            Assert.AreEqual(3, caps.Count);
            double volume = FeatureSolid(featureFaces, caps).Volume(0.0001);
            Assert.AreEqual(Math.PI * 16 * 20, volume, 1e-2, "the plug of the first hole is whole");
        }

        /// <summary>
        /// Removing the first of two crossing holes: the second one, which ended in it, now ends at the window, a piece of
        /// the former wall of the first. The solid gains exactly the plug of the first hole.
        /// </summary>
        [TestMethod]
        public void RemovingACrossedHoleLeavesTheOtherBlind()
        {
            MCPServer server = ServerWithBlock();
            Solid first = Instantiate(server, "durchgangsbohrung", "{\"face\":\"top\",\"pos\":\"p(10,10,20)\"}", "first");
            server.namedItems["side"] = PlaneAt(first, 0, 20);
            Solid second = Instantiate(server, "durchgangsbohrung", "{\"body\":\"first\",\"face\":\"side\",\"pos\":\"p(20,10,10)\",\"d\":6}", "second");
            double before = second.Volume(0.0001);
            Shell shell = second.Shells[0];
            Assert.IsTrue(shell.FeatureOfFace(MarkedFace(second, "Through hole 1"), out _, out List<Face> featureFaces, out List<Face> caps, out _));
            foreach (Face cap in caps) cap.ReverseOrientation();
            Assert.IsTrue(shell.AddAndRemoveFaces(caps, featureFaces));
            Assert.AreEqual(0, shell.OpenEdgesExceptPoles.Length);
            Assert.AreEqual(before + Math.PI * 16 * 20, second.Volume(0.0001), 1e-2);
            Assert.IsTrue(shell.GetFeatures().ContainsKey("Through hole 2") && !shell.GetFeatures().ContainsKey("Through hole 1"));
        }

        [TestMethod]
        public void AFaceOfNoFeatureHasNone()
        {
            MCPServer server = ServerWithBlock();
            Solid drilled = Instantiate(server, "durchgangsbohrung", "{\"face\":\"top\",\"pos\":\"p(10,10,20)\"}", "drilled");
            Face side = PlaneAt(drilled, 0, 20);
            Assert.IsFalse(drilled.Shells[0].FeatureOfFace(side, out _, out _, out _, out _));
        }
    }
}
