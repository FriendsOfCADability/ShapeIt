using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// Templates with object parameters: a required 'profile' and an optional 'solid', and steps made
    /// conditional with 'when' and defined("name").
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class TemplateObjectParameterTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static Solid FirstSolid(object item) => item as Solid ?? ((List<Solid>)item)[0];
        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        /// <summary>Records the "rays" template: rods from the origin along a spiral, subtracted from the optional hull.</summary>
        private static MCPServer ServerWithRaysTemplate()
        {
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };
            Assert.IsNull(Error(Call(server, "sketch.create", "{\"plane\":{\"standard\":\"XY\"},\"name\":\"prof\"}")));
            Assert.IsNull(Error(Call(server, "sketch.add_circle", "{\"sketch\":\"prof\",\"center\":[0,0],\"radius\":3}")));
            // a hollow ball like a lamp shade: the rods overlap only near the origin, inside the cavity
            Assert.IsNull(Error(Call(server, "solid.sphere", "{\"center\":[0,0,0],\"radius\":80,\"name\":\"outer\"}")));
            Assert.IsNull(Error(Call(server, "solid.sphere", "{\"center\":[0,0,0],\"radius\":75,\"name\":\"inner\"}")));
            Assert.IsNull(Error(Call(server, "solid.boolean", "{\"op\":\"difference\",\"a\":\"outer\",\"b\":\"inner\",\"name\":\"ball\"}")));

            string begin = @"{""name"":""rays"",""label"":""Rays"",""parameters"":[
                {""name"":""profile"",""value"":""prof"",""label"":""Profile"",""input"":{""kind"":""profile""}},
                {""name"":""hull"",""value"":""ball"",""label"":""Hull"",""input"":{""kind"":""solid"",""optional"":true,""replace"":true}},
                {""name"":""n"",""value"":5,""label"":""Count"",""input"":{""kind"":""integer""}}]}";
            Assert.IsNull(Error(Call(server, "template.begin", begin)));
            string batch = @"{""calls"":[
                {""jsonrpc"":""2.0"",""id"":1,""method"":""solid.extrude"",""params"":{""profile"":""profile"",""length"":200,""direction"":[0,0,1],""name"":""rod""}},
                {""jsonrpc"":""2.0"",""id"":2,""method"":""pattern.by_formula_solids"",""params"":{""solids"":""rod"",""variables"":[{""name"":""i"",""start"":0,""step"":1,""count"":""n""}],
                    ""formulas"":{""lat"":""75 - 150*i/(n-1)"",""azi"":""i*137.50776""},
                    ""transform"":""rotate(p(0,0,0), v(0,0,1), azi*pi/180) * rotate(p(0,0,0), v(0,1,0), (90-lat)*pi/180)"",""name"":""beams""}},
                {""jsonrpc"":""2.0"",""id"":3,""when"":""defined(\""hull\"")"",""method"":""solid.boolean"",""params"":{""op"":""difference"",""a"":""hull"",""b"":""beams"",""name"":""result""}},
                {""jsonrpc"":""2.0"",""id"":4,""when"":""!defined(\""hull\"")"",""method"":""workspace.select"",""params"":{""selector"":""beams"",""name"":""result""}}]}";
            string? batchError = Error(Call(server, "rpc.batch", batch));
            Assert.IsNull(batchError, "the recording itself must build: " + batchError);
            Assert.IsNull(Error(Call(server, "template.commit", "{\"result\":\"result\"}")));
            return server;
        }

        [TestMethod]
        public void OptionalSolidLeftOutYieldsTheTools()
        {
            MCPServer server = ServerWithRaysTemplate();
            string response = Call(server, "template.instantiate", "{\"template\":\"rays\",\"arguments\":{\"profile\":\"prof\",\"n\":7},\"name\":\"onlyRays\"}");
            Assert.IsNull(Error(response), response);
            Assert.IsTrue(server.namedItems.TryGetValue("onlyRays", out object? value));
            Assert.IsInstanceOfType(value, typeof(List<Solid>));
            Assert.AreEqual(7, ((List<Solid>)value!).Count, "without hull the result are the rods");
        }

        [TestMethod]
        public void OptionalSolidGivenGetsTheHoles()
        {
            MCPServer server = ServerWithRaysTemplate();
            double ballVolume = FirstSolid(server.namedItems["ball"]).Volume(0.01);
            string response = Call(server, "template.instantiate", "{\"template\":\"rays\",\"arguments\":{\"profile\":\"prof\",\"hull\":\"ball\"},\"name\":\"perforated\"}");
            Assert.IsNull(Error(response), response);
            object perforated = server.namedItems["perforated"];
            Solid solid = FirstSolid(perforated);
            Assert.IsTrue(solid.Volume(0.01) < ballVolume - 100, "the holes must have removed material");
            Assert.AreEqual(ballVolume, FirstSolid(server.namedItems["ball"]).Volume(0.01), 1e-6, "the argument itself must stay untouched");
        }

        [TestMethod]
        public void RequiredProfileMissingIsReported()
        {
            MCPServer server = ServerWithRaysTemplate();
            string? error = Error(Call(server, "template.instantiate", "{\"template\":\"rays\",\"arguments\":{\"n\":3},\"name\":\"noProfile\"}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "profile");
            Assert.IsFalse(server.namedItems.ContainsKey("noProfile"));
        }

        [TestMethod]
        public void OptionalSolidLeftOutIsNotTakenFromTheWorkspace()
        {
            // "ball" exists in the outer workspace, and so would "hull" if somebody had stored it there:
            // an object parameter left out must not pick up a workspace item of the same name
            MCPServer server = ServerWithRaysTemplate();
            server.namedItems["hull"] = server.namedItems["ball"];
            string response = Call(server, "template.instantiate", "{\"template\":\"rays\",\"arguments\":{\"profile\":\"prof\",\"n\":3},\"name\":\"rays3\"}");
            Assert.IsNull(Error(response), response);
            Assert.IsInstanceOfType(server.namedItems["rays3"], typeof(List<Solid>));
        }
    }
}
