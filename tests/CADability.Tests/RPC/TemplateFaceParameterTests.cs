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
    /// Templates with a 'face' parameter linked to a 'solid' parameter, a point put onto the face with 'onFace',
    /// and an 'enum' parameter whose value selects steps with 'when'.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class TemplateFaceParameterTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static Solid FirstSolid(object item) => item as Solid ?? ((List<Solid>)item)[0];
        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        /// <summary>The planar face of the solid whose center has the given z.</summary>
        private static Face FaceAtZ(Solid solid, double z)
        {
            return solid.Shells[0].Faces.Single(f => f.Surface is PlaneSurface && Math.Abs(f.GetBoundingCube().GetCenter().z - z) < 1e-9
                && f.GetBoundingCube().Zmax - f.GetBoundingCube().Zmin < 1e-9);
        }

        private const string HoleParameters = @"[
                {""name"":""body"",""value"":""block"",""label"":""Body"",""input"":{""kind"":""solid"",""replace"":true}},
                {""name"":""face"",""value"":""top"",""label"":""Face"",""input"":{""kind"":""face"",""solid"":""body""}},
                {""name"":""pos"",""value"":""p(5,5,10)"",""label"":""Position"",""input"":{""kind"":""point3"",""onFace"":""face""}},
                {""name"":""d"",""value"":2,""label"":""Diameter"",""input"":{""kind"":""length"",""min"":0.1}},
                {""name"":""entry"",""value"":""sharp"",""label"":""Entry"",""input"":{""kind"":""enum"",""options"":[
                    {""value"":""sharp"",""label"":""Sharp""},{""value"":""chamfer"",""label"":""Chamfered""},""round""]}}]";

        /// <summary>
        /// Records the "hole" template: a through hole perpendicular to the picked face at the given position. The
        /// enum only chooses the radius here, which is enough to see that the right step ran.
        /// </summary>
        private static MCPServer ServerWithHoleTemplate()
        {
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };
            Assert.IsNull(Error(Call(server, "solid.box", "{\"name\":\"block\",\"origin\":[0,0,0],\"sizeX\":10,\"sizeY\":10,\"sizeZ\":10}")));
            Assert.IsNull(Error(Call(server, "solid.box", "{\"name\":\"other\",\"origin\":[20,0,0],\"sizeX\":10,\"sizeY\":10,\"sizeZ\":10}")));
            server.namedItems["top"] = FaceAtZ(FirstSolid(server.namedItems["block"]), 10);
            server.namedItems["otherTop"] = FaceAtZ(FirstSolid(server.namedItems["other"]), 10);

            Assert.IsNull(Error(Call(server, "template.begin", $"{{\"name\":\"hole\",\"label\":\"Hole\",\"parameters\":{HoleParameters}}}")));
            string batch = @"{""calls"":[
                {""jsonrpc"":""2.0"",""id"":1,""method"":""workspace.set"",""params"":{""name"":""n"",""value"":""normalize(face.Surface.GetNormal(face.Surface.PositionOf(pos)))""}},
                {""jsonrpc"":""2.0"",""id"":2,""when"":""entry == \""sharp\"""",""method"":""workspace.set"",""params"":{""name"":""r"",""value"":""d/2""}},
                {""jsonrpc"":""2.0"",""id"":3,""when"":""entry != \""sharp\"""",""method"":""workspace.set"",""params"":{""name"":""r"",""value"":""d""}},
                {""jsonrpc"":""2.0"",""id"":4,""method"":""solid.cylinder"",""params"":{""start"":""pos + n"",""end"":""pos - n*20"",""radius"":""r"",""name"":""drill""}},
                {""jsonrpc"":""2.0"",""id"":5,""method"":""solid.boolean"",""params"":{""op"":""difference"",""a"":""body"",""b"":""drill"",""name"":""result""}}]}";
            string? batchError = Error(Call(server, "rpc.batch", batch));
            Assert.IsNull(batchError, "the recording itself must build: " + batchError);
            Assert.IsNull(Error(Call(server, "template.commit", "{\"result\":\"result\"}")));
            return server;
        }

        [TestMethod]
        public void FaceAloneGivesItsSolidAndThePointLandsOnTheFace()
        {
            MCPServer server = ServerWithHoleTemplate();
            // the point is far above the face: it is projected onto it, so the hole still goes through the block
            string response = Call(server, "template.instantiate", "{\"template\":\"hole\",\"arguments\":{\"face\":\"top\",\"pos\":\"p(3,4,50)\"},\"name\":\"drilled\"}");
            Assert.IsNull(Error(response), response);
            Solid drilled = FirstSolid(server.namedItems["drilled"]);
            Assert.AreEqual(1000 - Math.PI * 10, drilled.Volume(0.001), 0.05, "a through hole of diameter 2");
            Assert.AreEqual(1000, FirstSolid(server.namedItems["block"]).Volume(0.001), 1e-6, "the argument itself must stay untouched");
        }

        [TestMethod]
        public void EnumOptionSelectsTheSteps()
        {
            MCPServer server = ServerWithHoleTemplate();
            string response = Call(server, "template.instantiate", "{\"template\":\"hole\",\"arguments\":{\"face\":\"top\",\"entry\":\"chamfer\"},\"name\":\"wide\"}");
            Assert.IsNull(Error(response), response);
            Assert.AreEqual(1000 - Math.PI * 4 * 10, FirstSolid(server.namedItems["wide"]).Volume(0.001), 0.1, "'chamfer' doubles the radius");
            // an option given without a label, and as an expression
            response = Call(server, "template.instantiate", "{\"template\":\"hole\",\"arguments\":{\"face\":\"top\",\"entry\":\"round\"},\"name\":\"round\"}");
            Assert.IsNull(Error(response), response);
            response = Call(server, "template.instantiate", "{\"template\":\"hole\",\"arguments\":{\"face\":\"top\",\"entry\":\"\\\"sharp\\\"\"},\"name\":\"sharp\"}");
            Assert.IsNull(Error(response), response);
            Assert.AreEqual(1000 - Math.PI * 10, FirstSolid(server.namedItems["sharp"]).Volume(0.001), 0.05);
        }

        [TestMethod]
        public void ValueThatIsNoOptionIsRefused()
        {
            MCPServer server = ServerWithHoleTemplate();
            server.namedItems["bogus"] = "bogus";
            string? error = Error(Call(server, "template.instantiate", "{\"template\":\"hole\",\"arguments\":{\"face\":\"top\",\"entry\":\"bogus\"},\"name\":\"x\"}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "Entry must be one of sharp, chamfer, round");
        }

        [TestMethod]
        public void FaceOfAnotherSolidIsRefused()
        {
            MCPServer server = ServerWithHoleTemplate();
            string? error = Error(Call(server, "template.instantiate", "{\"template\":\"hole\",\"arguments\":{\"body\":\"block\",\"face\":\"otherTop\"},\"name\":\"x\"}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "not a face of the solid 'body'");
            // with its own solid it is fine
            string response = Call(server, "template.instantiate", "{\"template\":\"hole\",\"arguments\":{\"body\":\"other\",\"face\":\"otherTop\",\"pos\":\"p(25,5,10)\"},\"name\":\"y\"}");
            Assert.IsNull(Error(response), response);
            Assert.AreEqual(1000 - Math.PI * 10, FirstSolid(server.namedItems["y"]).Volume(0.001), 0.05);
        }

        [TestMethod]
        public void MissingFaceIsReported()
        {
            MCPServer server = ServerWithHoleTemplate();
            string? error = Error(Call(server, "template.instantiate", "{\"template\":\"hole\",\"arguments\":{},\"name\":\"x\"}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "face parameter 'face'");
        }

        /// <summary>The application passes the picked face of a copy of the solid, the way RpcTemplateAction does.</summary>
        [TestMethod]
        public void ApplicationValuesWithFaceOnly()
        {
            MCPServer server = ServerWithHoleTemplate();
            (Face face, Solid? copy) = MCPServer.CopyWithSolid(FaceAtZ(FirstSolid(server.namedItems["block"]), 10));
            Assert.IsNotNull(copy);
            Dictionary<string, object> values = new Dictionary<string, object>
            {
                ["face"] = face,
                ["pos"] = MCPServer.PointOnFace(face, new GeoPoint(2, 2, 30)),
                ["d"] = 2.0,
                ["entry"] = "round",
            };
            object? result = server.ExecuteTemplate("hole", values, System.Threading.CancellationToken.None);
            Assert.IsNotNull(result);
            Assert.AreEqual(1000 - Math.PI * 4 * 10, FirstSolid(result).Volume(0.001), 0.1);
        }

        [TestMethod]
        public void DefinitionMistakesAreReported()
        {
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };
            string? error = Error(Call(server, "template.begin", @"{""name"":""t"",""parameters"":[
                {""name"":""e"",""value"":""a"",""input"":{""kind"":""enum""}}]}"));
            StringAssert.Contains(error, "needs 'options'");
            error = Error(Call(server, "template.begin", @"{""name"":""t"",""parameters"":[
                {""name"":""d"",""value"":1,""input"":{""kind"":""length""}},
                {""name"":""q"",""value"":""p(0,0,0)"",""input"":{""kind"":""point3"",""onFace"":""d""}}]}"));
            StringAssert.Contains(error, "no 'face' parameter");
            error = Error(Call(server, "template.begin", @"{""name"":""t"",""parameters"":[
                {""name"":""f"",""value"":""x"",""input"":{""kind"":""face"",""solid"":""nobody""}}]}"));
            StringAssert.Contains(error, "no 'solid' parameter");
        }
    }
}
