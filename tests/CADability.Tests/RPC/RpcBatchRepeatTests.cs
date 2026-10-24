using System.Text.Json;
using System.Text.Json.Nodes;
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
    }
}
