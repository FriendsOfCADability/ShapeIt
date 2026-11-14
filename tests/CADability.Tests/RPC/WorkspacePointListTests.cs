using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Nodes;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// Lists of points, as the formula function vertices() yields them, are workspace items of their own:
    /// they must be described by workspace.set and inspect.summary instead of failing as an unknown type.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class WorkspacePointListTests
    {
        private static JsonNode Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            JsonNode response = JsonNode.Parse(server.ProcessMethod(document.RootElement))!;
            Assert.IsNull(response["error"], response.ToJsonString());
            return response["result"]!;
        }

        private static MCPServer ServerWithPointLists()
        {
            MCPServer server = new MCPServer(null!, Project.CreateSimpleProject()) { SuppressDialogs = true };
            server.namedItems["pts3"] = new List<GeoPoint> { new GeoPoint(0, 0, 0), new GeoPoint(10, 0, 0), new GeoPoint(10, 5, 2) };
            server.namedItems["pts2"] = new List<GeoPoint2D> { new GeoPoint2D(-1, 0), new GeoPoint2D(3, 4) };
            return server;
        }

        [TestMethod]
        public void InspectSummaryDescribesPointLists()
        {
            MCPServer server = ServerWithPointLists();
            JsonArray objects = Call(server, "inspect.summary", "{\"objects\":[\"pts3\",\"pts2\"]}")["objects"]!.AsArray();

            Assert.AreEqual("point3[]", objects[0]!["type"]!.GetValue<string>());
            Assert.AreEqual(3, objects[0]!["summary"]!["count"]!.GetValue<int>());
            Assert.IsNotNull(objects[0]!["summary"]!["boundingBox"]);

            Assert.AreEqual("point2[]", objects[1]!["type"]!.GetValue<string>());
            Assert.AreEqual(2, objects[1]!["summary"]!["count"]!.GetValue<int>());
            Assert.IsNotNull(objects[1]!["summary"]!["boundingRect"]);
        }

        [TestMethod]
        public void WorkspaceSetReportsPointListType()
        {
            MCPServer server = ServerWithPointLists();
            string response = Call(server, "workspace.set", "{\"name\":\"copy\",\"value\":\"pts3\"}").ToJsonString();
            StringAssert.Contains(response, "\"point3[]\"");
        }
    }
}
