using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// solid.ruled and solid.rotate take the name of a sketch as profile, as solid.extrude does: the closed curves of
    /// the sketch are the profile. Before, a sketch name gave no ruled solid at all and an empty list of rotated solids.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class SketchProfileByNameTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            string response = server.ProcessMethod(document.RootElement);
            return response;
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        private static void Ok(MCPServer server, string method, string parameters)
        {
            string response = Call(server, method, parameters);
            Assert.IsNull(Error(response), $"{method}: {response}");
        }

        private static MCPServer NewServer() => new MCPServer(null!, Project.CreateSimpleProject()) { SuppressDialogs = true };

        private static void Sketch(MCPServer server, string name, double z, double[] normal)
            => Ok(server, "sketch.create", FormattableString.Invariant(
                $"{{\"name\":\"{name}\",\"plane\":{{\"origin\":[0,0,{z}],\"xAxis\":[1,0,0],\"normal\":[{normal[0]},{normal[1]},{normal[2]}]}}}}"));

        /// <summary>A square of side a around the origin of the sketch, as four lines joined by sketch.connect.</summary>
        private static void ConnectedSquare(MCPServer server, string sketch, double a)
        {
            double h = a / 2;
            double[][] corners = { new[] { -h, -h }, new[] { h, -h }, new[] { h, h }, new[] { -h, h } };
            for (int i = 0; i < 4; i++)
            {
                double[] s = corners[i], e = corners[(i + 1) % 4];
                Ok(server, "sketch.add_line", FormattableString.Invariant($"{{\"sketch\":\"{sketch}\",\"start\":[{s[0]},{s[1]}],\"end\":[{e[0]},{e[1]}]}}"));
            }
            Ok(server, "sketch.connect", $"{{\"sketch\":\"{sketch}\",\"precision\":0.000001}}"); // without a name: the closed path goes into the sketch
        }

        private static Solid Single(MCPServer server, string name)
        {
            object result = server.namedItems[name];
            Solid? solid = result as Solid ?? (result as System.Collections.Generic.List<Solid>)?.FirstOrDefault();
            Assert.IsNotNull(solid, $"'{name}' holds no solid");
            return solid;
        }

        [TestMethod]
        public void RuledSolidBetweenTwoSketches()
        {
            MCPServer server = NewServer();
            Sketch(server, "bottom", 0, new double[] { 0, 0, 1 });
            Sketch(server, "top", 6, new double[] { 0, 0, 1 });
            ConnectedSquare(server, "bottom", 10);
            ConnectedSquare(server, "top", 4);
            Ok(server, "solid.ruled", "{\"profile1\":\"bottom\",\"profile2\":\"top\",\"name\":\"frustum\"}");
            Solid frustum = Single(server, "frustum");
            // a square frustum: h/3*(A1 + A2 + sqrt(A1*A2))
            Assert.AreEqual(6.0 / 3 * (100 + 16 + 40), frustum.Volume(1e-4), 1e-6);
            Assert.AreEqual(6, frustum.Shells[0].Faces.Length);
        }

        [TestMethod]
        public void RuledSolidBetweenSketchAndRectangle()
        {
            // a rectangle of sketch.add_rectangle is a closed curve of its own in the sketch, without sketch.connect
            MCPServer server = NewServer();
            Sketch(server, "bottom", 0, new double[] { 0, 0, 1 });
            Sketch(server, "top", 5, new double[] { 0, 0, 1 });
            Ok(server, "sketch.add_rectangle", "{\"sketch\":\"bottom\",\"center\":[0,0],\"width\":8,\"height\":8}");
            Ok(server, "sketch.add_rectangle", "{\"sketch\":\"top\",\"center\":[0,0],\"width\":8,\"height\":8}");
            Ok(server, "solid.ruled", "{\"profile1\":\"bottom\",\"profile2\":\"top\",\"name\":\"prism\"}");
            Assert.AreEqual(8 * 8 * 5, Single(server, "prism").Volume(1e-4), 1e-6);
        }

        [TestMethod]
        public void RuledRefusesASketchWithTwoProfiles()
        {
            MCPServer server = NewServer();
            Sketch(server, "bottom", 0, new double[] { 0, 0, 1 });
            Sketch(server, "top", 5, new double[] { 0, 0, 1 });
            Ok(server, "sketch.add_rectangle", "{\"sketch\":\"bottom\",\"center\":[-10,0],\"width\":4,\"height\":4}");
            Ok(server, "sketch.add_rectangle", "{\"sketch\":\"bottom\",\"center\":[10,0],\"width\":4,\"height\":4}");
            Ok(server, "sketch.add_rectangle", "{\"sketch\":\"top\",\"center\":[0,0],\"width\":4,\"height\":4}");
            string? error = Error(Call(server, "solid.ruled", "{\"profile1\":\"bottom\",\"profile2\":\"top\",\"name\":\"x\"}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "'profile1' must contain exactly one closed profile");
        }

        [TestMethod]
        public void RuledStillTakesTheNameOfAConnectedCurve()
        {
            MCPServer server = NewServer();
            Sketch(server, "bottom", 0, new double[] { 0, 0, 1 });
            Sketch(server, "top", 6, new double[] { 0, 0, 1 });
            ConnectedSquare(server, "bottom", 10);
            ConnectedSquare(server, "top", 4);
            Ok(server, "sketch.connect", "{\"sketch\":\"bottom\",\"precision\":0.000001,\"name\":\"bottomProfile\"}");
            Ok(server, "sketch.connect", "{\"sketch\":\"top\",\"precision\":0.000001,\"name\":\"topProfile\"}");
            Ok(server, "solid.ruled", "{\"profile1\":\"bottomProfile\",\"profile2\":\"topProfile\",\"name\":\"frustum\"}");
            Assert.AreEqual(6.0 / 3 * (100 + 16 + 40), Single(server, "frustum").Volume(1e-4), 1e-6);
        }

        [TestMethod]
        public void RotateTakesASketchName()
        {
            // the sketch lies in the xz-plane (x = radius, y = height); a 3x3 square from radius 2 to 5 revolved
            // around the z-axis is a ring of volume pi*(5² - 2²)*3
            MCPServer server = NewServer();
            Sketch(server, "section", 0, new double[] { 0, -1, 0 });
            Ok(server, "sketch.add_rectangle", "{\"sketch\":\"section\",\"center\":[3.5,1.5],\"width\":3,\"height\":3}");
            Ok(server, "solid.rotate", "{\"profile\":\"section\",\"axis\":{\"origin\":[0,0,0],\"direction\":[0,0,1]},\"angle\":360,\"name\":\"ring\"}");
            Solid ring = Single(server, "ring");
            Assert.AreEqual(Math.PI * 21 * 3, ring.Volume(1e-4), 1e-6 * Math.PI * 63);
            BoundingBox ext = ring.GetExtent(0.0);
            Assert.AreEqual(0, ext.Zmin, 1e-9, "the sketch y-axis is the z-axis");
            Assert.AreEqual(3, ext.Zmax, 1e-9);
            Assert.AreEqual(5, ext.Xmax, 1e-9);
        }
    }
}
