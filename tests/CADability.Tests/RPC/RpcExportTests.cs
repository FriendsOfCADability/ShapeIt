using System;
using System.IO;
using System.Text.Json;
using CADability.Tests.Rpc;
using ShapeIt;
using Path = System.IO.Path;

namespace CADability.Tests
{
    /// <summary>
    /// The export in the MCP server window has to produce a file this harness can read - otherwise the
    /// workflow "session goes wrong, export it, debug it, record it" breaks at its first step, and it would
    /// break silently: an unreadable file is only noticed when someone tries to use it.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class RpcExportTests
    {
        [TestMethod]
        public void ExportedSessionIsAReadableCase()
        {
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };

            // A short session, as a client would send it.
            string calls = @"[
              {""jsonrpc"":""2.0"",""id"":1,""method"":""solid.box"",""params"":{""name"":""blockA"",""origin"":[0,0,0],""sizeX"":10,""sizeY"":10,""sizeZ"":10}},
              {""jsonrpc"":""2.0"",""id"":2,""method"":""solid.box"",""params"":{""name"":""blockB"",""origin"":[5,5,5],""sizeX"":10,""sizeY"":10,""sizeZ"":10}},
              {""jsonrpc"":""2.0"",""id"":3,""method"":""solid.boolean"",""params"":{""op"":""union"",""a"":""blockA"",""b"":""blockB"",""name"":""blockUnion"",""rebind"":false}}]";
            using (JsonDocument document = JsonDocument.Parse(calls))
                foreach (JsonElement call in document.RootElement.EnumerateArray())
                    server.ProcessMethod(call);

            Assert.AreEqual(3, server.ProtocolCallCount, "the protocol should hold the three requests");

            string path = Path.Combine(Path.GetTempPath(), "RpcExportTest_" + Guid.NewGuid().ToString("N") + ".json");
            try
            {
                File.WriteAllText(path, server.BuildRpcCaseFile("union of two overlapping blocks"));
                RpcCase exported = RpcCase.Read(path);

                Assert.IsTrue(exported.IsRunnable, "the exported file is not a usable case: " + string.Join("; ", exported.Problems));
                Assert.AreEqual(3, exported.CallCount, "all three calls must be exported");
                Assert.AreEqual(RpcCaseStatus.KnownFail, exported.Status,
                    "a fresh case has no baseline yet, so it must not claim to be Ok");
                Assert.AreEqual(0, exported.Baseline.Count,
                    "the export must not write a result - that is recorded later, after it has been judged correct");
                CollectionAssert.Contains(exported.Verify, "blockUnion", "Verify should offer the solids of the session");

                // And the exported case has to run.
                RpcRunResult run = RpcRunner.Run(exported);
                Assert.IsNull(run.Crash, "the exported case threw: " + run.Crash);
                foreach ((int id, string method, string error) in run.Failures)
                    Assert.Fail($"call id {id} ({method}) failed in the exported case: {error}");
                Assert.IsTrue(run.Objects.ContainsKey("blockUnion"), "the union should be in the fingerprint");
            }
            finally
            {
                if (File.Exists(path)) File.Delete(path);
            }
        }
    }
}
