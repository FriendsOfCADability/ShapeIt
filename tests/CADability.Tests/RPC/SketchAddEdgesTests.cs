using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.Shapes;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// sketch.add_edges: the edges of holes of a face, reached in a loop over the holes (f.HoleCount, f.HoleEdges(h)),
    /// are projected into a sketch and joined into closed profiles, whatever their shape.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class SketchAddEdgesTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        private static void Ok(MCPServer server, string method, string parameters)
        {
            string response = Call(server, method, parameters);
            Assert.IsNull(Error(response), response);
        }

        [TestMethod]
        public void HolesOfAFaceOfAnyShapeBecomeProfiles()
        {
            MCPServer server = new MCPServer(null!, Project.CreateSimpleProject()) { SuppressDialogs = true };
            // a plate 60 x 40 x 10 with a slot (overall length 20, width 8) and a rectangular pocket 10 x 6, both 5 deep
            Ok(server, "solid.box", "{\"origin\":[0,0,0],\"sizeX\":60,\"sizeY\":40,\"sizeZ\":10,\"name\":\"plate\"}");
            Ok(server, "sketch.create", "{\"plane\":{\"origin\":[0,0,5],\"xAxis\":[1,0,0],\"normal\":[0,0,1]},\"name\":\"tools\"}");
            Ok(server, "sketch.add_slot", "{\"sketch\":\"tools\",\"center\":[20,20],\"length\":20,\"width\":8,\"rotationDeg\":0,\"name\":\"slot\"}");
            Ok(server, "sketch.add_rectangle", "{\"sketch\":\"tools\",\"center\":[48,20],\"width\":10,\"height\":6,\"name\":\"pocket\"}");
            Ok(server, "solid.extrude", "{\"profile\":\"slot\",\"length\":10,\"direction\":[0,0,1],\"name\":\"slotTool\"}");
            Ok(server, "solid.extrude", "{\"profile\":\"pocket\",\"length\":10,\"direction\":[0,0,1],\"name\":\"pocketTool\"}");
            Ok(server, "solid.boolean", "{\"op\":\"difference\",\"a\":\"plate\",\"b\":[\"slotTool\",\"pocketTool\"],\"name\":\"plate\"}");
            Ok(server, "workspace.select", "{\"selector\":{\"query\":{\"target\":\"faces\",\"from\":\"plate\",\"filter\":{\"extreme\":{\"axis\":\"z\",\"which\":\"max\"}}}},\"name\":\"top\"}");
            Ok(server, "workspace.set", "{\"name\":\"top\",\"value\":\"top\"}");
            Ok(server, "sketch.create", "{\"plane\":{\"origin\":[0,0,20],\"xAxis\":[1,0,0],\"normal\":[0,0,1]},\"name\":\"sk\"}");
            Ok(server, "workspace.set", "{\"name\":\"total\",\"value\":0}");
            Ok(server, "workspace.set", "{\"name\":\"count\",\"value\":0}");
            string loop = "{\"repeat\":{\"count\":\"top.HoleCount\",\"variable\":\"h\"},\"calls\":["
                + "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"workspace.set\",\"params\":{\"name\":\"holeEdges\",\"value\":\"top.HoleEdges(h)\"}},"
                + "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"sketch.add_edges\",\"params\":{\"sketch\":\"sk\",\"edges\":\"holeEdges\",\"name\":\"holeShape\"}},"
                + "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"workspace.set\",\"params\":{\"name\":\"total\",\"value\":\"total + holeShape.Area\"}},"
                + "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"workspace.set\",\"params\":{\"name\":\"count\",\"value\":\"count + 1\"}}]}";
            Ok(server, "rpc.batch", loop);
            Assert.AreEqual(2.0, Convert.ToDouble(server.namedItems["count"]), "two holes");
            double slot = (20 - 8) * 8 + Math.PI * 16, pocket = 10 * 6;
            Assert.AreEqual(slot + pocket, Convert.ToDouble(server.namedItems["total"]), 1e-6, "slot and pocket");
            // the last profile can be extruded
            Ok(server, "solid.extrude", "{\"profile\":\"holeShape\",\"length\":3,\"direction\":[0,0,1],\"name\":\"copy\"}");
        }

        [TestMethod]
        public void OpenEdgesWithoutConnectAreAddedAsCurves()
        {
            MCPServer server = new MCPServer(null!, Project.CreateSimpleProject()) { SuppressDialogs = true };
            Ok(server, "solid.box", "{\"origin\":[0,0,0],\"sizeX\":10,\"sizeY\":10,\"sizeZ\":10,\"name\":\"cube\"}");
            Ok(server, "workspace.select", "{\"selector\":{\"query\":{\"target\":\"edges\",\"from\":\"cube\",\"filter\":{\"boundingBox\":{\"component\":\"zDiff\",\"minValue\":5}}}},\"name\":\"vertical\"}");
            Ok(server, "sketch.create", "{\"plane\":{\"origin\":[0,0,0],\"xAxis\":[1,0,0],\"normal\":[0,1,0]},\"name\":\"side\"}");
            Ok(server, "sketch.add_edges", "{\"sketch\":\"side\",\"edges\":\"vertical\",\"connect\":false,\"name\":\"lines\"}");
            Assert.AreEqual(4, ((System.Collections.ICollection)server.namedItems["lines"]).Count);
            // vertical edges alone do not close a contour
            Assert.IsNotNull(Error(Call(server, "sketch.add_edges", "{\"sketch\":\"side\",\"edges\":\"vertical\",\"name\":\"x\"}")));
        }
    }
}
