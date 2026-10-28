using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// rpc.batch with 'repeat': the calls run once per iteration, with a loop variable, a count and/or a 'while' condition.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class RpcBatchRepeatTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();
        private static JsonNode? Result(string response) => JsonNode.Parse(response)?["result"];

        private static MCPServer NewServer()
        {
            Project project = Project.CreateSimpleProject();
            return new MCPServer(null!, project) { SuppressDialogs = true };
        }

        private const string AddToSum = @"[{""jsonrpc"":""2.0"",""id"":1,""method"":""workspace.set"",""params"":{""name"":""sum"",""value"":""sum + i""}}]";

        [TestMethod]
        public void CountWithLoopVariable()
        {
            MCPServer server = NewServer();
            Assert.IsNull(Error(Call(server, "workspace.set", "{\"name\":\"sum\",\"value\":0}")));
            string response = Call(server, "rpc.batch", $"{{\"repeat\":{{\"variable\":\"i\",\"start\":1,\"step\":2,\"count\":\"2+2\"}},\"calls\":{AddToSum}}}");
            Assert.IsNull(Error(response), response);
            Assert.AreEqual(1.0 + 3 + 5 + 7, (double)server.namedItems["sum"]);
            Assert.AreEqual(4, Result(response)!["iterations"]!.GetValue<int>());
            Assert.AreEqual(1, Result(response)!["results"]!.AsArray().Count, "only the results of the last iteration");
        }

        [TestMethod]
        public void WhileEndsTheLoop()
        {
            MCPServer server = NewServer();
            Assert.IsNull(Error(Call(server, "workspace.set", "{\"name\":\"sum\",\"value\":0}")));
            string response = Call(server, "rpc.batch", $"{{\"repeat\":{{\"variable\":\"i\",\"while\":\"sum < 10\"}},\"calls\":{AddToSum}}}");
            Assert.IsNull(Error(response), response);
            // 0+1+2+3+4 = 10 stops the loop before i = 5
            Assert.AreEqual(10.0, (double)server.namedItems["sum"]);
            Assert.AreEqual(5, Result(response)!["iterations"]!.GetValue<int>());
        }

        [TestMethod]
        public void ZeroCountRunsNothing()
        {
            MCPServer server = NewServer();
            Assert.IsNull(Error(Call(server, "workspace.set", "{\"name\":\"sum\",\"value\":0}")));
            string response = Call(server, "rpc.batch", $"{{\"repeat\":{{\"variable\":\"i\",\"count\":0}},\"calls\":{AddToSum}}}");
            Assert.IsNull(Error(response), response);
            Assert.AreEqual(0.0, (double)server.namedItems["sum"]);
            Assert.AreEqual(0, Result(response)!["iterations"]!.GetValue<int>());
        }

        [TestMethod]
        public void FailingIterationIsNamed()
        {
            MCPServer server = NewServer();
            Assert.IsNull(Error(Call(server, "workspace.set", "{\"name\":\"sum\",\"value\":0}")));
            string calls = @"[{""jsonrpc"":""2.0"",""id"":1,""method"":""assert.check"",""params"":{""condition"":""i < 3"",""message"":""too far""}}]";
            string? error = Error(Call(server, "rpc.batch", $"{{\"repeat\":{{\"variable\":\"i\",\"count\":10}},\"calls\":{calls}}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "iteration 4");
        }

        [TestMethod]
        public void RepeatNeedsCountOrWhile()
        {
            MCPServer server = NewServer();
            string? error = Error(Call(server, "rpc.batch", $"{{\"repeat\":{{\"variable\":\"i\"}},\"calls\":{AddToSum}}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "count");
        }

        private static void Boxes(MCPServer server)
        {
            for (int k = 0; k < 3; k++)
                Assert.IsNull(Error(Call(server, "solid.box", $"{{\"name\":\"b{k}\",\"origin\":[{30 * k},0,0],\"sizeX\":10,\"sizeY\":10,\"sizeZ\":10}}")));
        }

        [TestMethod]
        public void OverFacesOfANamedList()
        {
            MCPServer server = NewServer();
            Boxes(server);
            Assert.IsNull(Error(Call(server, "workspace.select", "{\"selector\":\"b0\",\"type\":\"faces\",\"name\":\"faces\"}")));
            Assert.IsNull(Error(Call(server, "workspace.set", "{\"name\":\"n\",\"value\":0}")));
            Assert.IsNull(Error(Call(server, "workspace.set", "{\"name\":\"sumIndex\",\"value\":0}")));
            string calls = @"[{""method"":""workspace.set"",""params"":{""name"":""n"",""value"":""n + 1""}},
                              {""method"":""workspace.set"",""params"":{""name"":""sumIndex"",""value"":""sumIndex + i""}},
                              {""method"":""assert.check"",""params"":{""condition"":""f == faces[i]"",""message"":""element and index disagree""}}]";
            string response = Call(server, "rpc.batch", $"{{\"repeat\":{{\"over\":\"faces\",\"variable\":\"f\",\"index\":\"i\"}},\"calls\":{calls}}}");
            Assert.IsNull(Error(response), response);
            Assert.AreEqual(6, Result(response)!["iterations"]!.GetValue<int>());
            Assert.AreEqual(6.0, (double)server.namedItems["n"]);
            Assert.AreEqual(0.0 + 1 + 2 + 3 + 4 + 5, (double)server.namedItems["sumIndex"]);
            Assert.IsInstanceOfType(server.namedItems["f"], typeof(Face), "the variable keeps the last element");
        }

        [TestMethod]
        public void OverSolidsUsesTheElementAsObjectReference()
        {
            MCPServer server = NewServer();
            Boxes(server);
            // an array of names is a selector: its elements are collected before the loop
            string calls = @"[{""method"":""transform.move"",""params"":{""objects"":""s"",""delta"":[0,0,5]}}]";
            string response = Call(server, "rpc.batch", $"{{\"repeat\":{{\"over\":[\"b0\",\"b1\",\"b2\"],\"variable\":\"s\"}},\"calls\":{calls}}}");
            Assert.IsNull(Error(response), response);
            for (int k = 0; k < 3; k++)
            {
                Solid box = (Solid)server.namedItems[$"b{k}"];
                Assert.AreEqual(5.0, box.GetExtent(0.0).Zmin, 1e-9, $"b{k} was moved");
                Assert.AreEqual($"b{k}", box.Name, "the loop variable must not rename the solid");
            }
        }

        [TestMethod]
        public void OverAnEmptySelectionRunsNothing()
        {
            MCPServer server = NewServer();
            Assert.IsNull(Error(Call(server, "workspace.set", "{\"name\":\"n\",\"value\":0}")));
            string calls = @"[{""method"":""workspace.set"",""params"":{""name"":""n"",""value"":""n + 1""}}]";
            string response = Call(server, "rpc.batch", $"{{\"repeat\":{{\"over\":{{\"query\":{{\"target\":\"solids\",\"from\":[]}}}},\"variable\":\"s\"}},\"calls\":{calls}}}");
            Assert.IsNull(Error(response), response);
            Assert.AreEqual(0, Result(response)!["iterations"]!.GetValue<int>());
            Assert.AreEqual(0.0, (double)server.namedItems["n"]);
        }

        [TestMethod]
        public void OverAListWhoseLengthChangesFails()
        {
            MCPServer server = NewServer();
            Boxes(server);
            Assert.IsNull(Error(Call(server, "workspace.select", "{\"selector\":[\"b0\",\"b1\"],\"name\":\"list\"}")));
            string calls = @"[{""method"":""workspace.select"",""params"":{""selector"":""b2"",""name"":""list""}}]";
            string? error = Error(Call(server, "rpc.batch", $"{{\"repeat\":{{\"over\":\"list\",\"variable\":\"s\"}},\"calls\":{calls}}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "iteration 2");
            StringAssert.Contains(error, "length");
        }

        [TestMethod]
        public void OverRejectsCountingParameters()
        {
            MCPServer server = NewServer();
            Boxes(server);
            string? error = Error(Call(server, "rpc.batch", $"{{\"repeat\":{{\"over\":\"b0\",\"variable\":\"s\",\"count\":3}},\"calls\":{AddToSum}}}"));
            StringAssert.Contains(error, "'repeat.count' cannot be combined with 'over'");
            error = Error(Call(server, "rpc.batch", $"{{\"repeat\":{{\"over\":\"b0\"}},\"calls\":{AddToSum}}}"));
            StringAssert.Contains(error, "needs 'variable'");
            error = Error(Call(server, "rpc.batch", $"{{\"repeat\":{{\"variable\":\"i\",\"index\":\"k\",\"count\":2}},\"calls\":{AddToSum}}}"));
            StringAssert.Contains(error, "only valid with 'over'");
        }
    }
}
