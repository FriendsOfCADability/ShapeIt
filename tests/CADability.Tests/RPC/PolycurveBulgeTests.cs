using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// sketch.add_polycurve with DXF bulges: bulge = tan(sweep/4), so |b| > 1 means an arc of more than 180°.
    /// The profile is the square [6,12]² with three quarters of the circle around (6,6), radius 5, attached
    /// (area 36 + 0.75·π·25), extruded by 20.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class PolycurveBulgeTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        private static void AssertMajorArcProfile(string vertices)
        {
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };
            Assert.IsNull(Error(Call(server, "sketch.create", "{\"plane\":{\"standard\":\"XY\"},\"name\":\"sk\"}")));
            string response = Call(server, "sketch.add_polycurve", $"{{\"sketch\":\"sk\",\"vertices\":{vertices},\"isClosed\":true}}");
            Assert.IsNull(Error(response), response);
            response = Call(server, "solid.extrude", "{\"profile\":\"sk\",\"length\":20,\"direction\":[0,0,1],\"name\":\"s\"}");
            Assert.IsNull(Error(response), response);
            object item = server.namedItems["s"];
            Solid solid = item as Solid ?? ((List<Solid>)item)[0];

            double expected = 20 * (36 + 0.75 * Math.PI * 25);
            Assert.AreEqual(expected, Math.Abs(solid.Volume(1e-4)), 1e-6 * expected, "volume");

            BoundingBox ext = solid.GetExtent(0.0);
            Assert.AreEqual(1, ext.Xmin, 1e-6, "xmin");
            Assert.AreEqual(12, ext.Xmax, 1e-6, "xmax");
            Assert.AreEqual(1, ext.Ymin, 1e-6, "ymin");
            Assert.AreEqual(12, ext.Ymax, 1e-6, "ymax");
            Assert.AreEqual(0, ext.Zmin, 1e-6, "zmin");
            Assert.AreEqual(20, ext.Zmax, 1e-6, "zmax");
        }

        [TestMethod]
        public void PositiveBulgeGivesCounterclockwiseMajorArc()
        {
            string b = Math.Tan(3 * Math.PI / 8).ToString("R", System.Globalization.CultureInfo.InvariantCulture);
            AssertMajorArcProfile($"[{{\"x\":12,\"y\":12}},{{\"x\":6,\"y\":12}},{{\"x\":6,\"y\":11,\"b\":{b}}},{{\"x\":11,\"y\":6}},{{\"x\":12,\"y\":6}}]");
        }

        [TestMethod]
        public void NegativeBulgeGivesClockwiseMajorArc()
        {
            // the same contour traversed the other way round
            string b = (-Math.Tan(3 * Math.PI / 8)).ToString("R", System.Globalization.CultureInfo.InvariantCulture);
            AssertMajorArcProfile($"[{{\"x\":12,\"y\":6}},{{\"x\":11,\"y\":6,\"b\":{b}}},{{\"x\":6,\"y\":11}},{{\"x\":6,\"y\":12}},{{\"x\":12,\"y\":12}}]");
        }
    }
}
