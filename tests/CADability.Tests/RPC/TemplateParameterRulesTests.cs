using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Nodes;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// The rules a template states about its values - 'min'/'max' of the inputs and 'checks' of template.begin -
    /// and its 'presets': refused definitions, refused instantiations with messages that name the values, and
    /// the evaluation the application uses for its drop-down lists.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class TemplateParameterRulesTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        private static MCPServer NewServer() => new MCPServer(null!, Project.CreateSimpleProject()) { SuppressDialogs = true };

        private const string Parameters = @"[
            {""name"":""a"",""value"":30,""label"":""Länge"",""input"":{""kind"":""length"",""min"":1,""max"":""10*b""}},
            {""name"":""b"",""value"":20,""label"":""Breite"",""input"":{""kind"":""length""}},
            {""name"":""n"",""value"":3,""label"":""Anzahl"",""input"":{""kind"":""integer"",""min"":2}}]";

        private const string Checks = @"[
            {""condition"":""a > b"",""message"":""Die Länge ({a}) muss größer als die Breite ({b}) sein, mindestens {b+1}."",""parameters"":[""a"",""b""]}]";

        private const string Presets = @"[
            {""name"":""sizes"",""label"":""Größe"",""options"":[
                {""label"":""klein"",""values"":{""a"":20,""b"":10}},
                {""label"":""doppelt"",""values"":{""a"":""2*b""}}]}]";

        /// <summary>Records the "plate" template: a box a × b × n with rules on its values.</summary>
        private static MCPServer ServerWithPlateTemplate()
        {
            MCPServer server = NewServer();
            string begin = $"{{\"name\":\"plate\",\"label\":\"Platte\",\"parameters\":{Parameters},\"checks\":{Checks},\"presets\":{Presets}}}";
            string response = Call(server, "template.begin", begin);
            Assert.IsNull(Error(response), response);
            Assert.IsNull(Error(Call(server, "solid.box", "{\"origin\":[0,0,0],\"sizeX\":\"a\",\"sizeY\":\"b\",\"sizeZ\":\"n\",\"name\":\"plate\"}")));
            Assert.IsNull(Error(Call(server, "template.commit", "{\"result\":\"plate\"}")));
            return server;
        }

        [TestMethod]
        public void ValidArgumentsBuildTheTemplate()
        {
            MCPServer server = ServerWithPlateTemplate();
            string response = Call(server, "template.instantiate", "{\"template\":\"plate\",\"arguments\":{\"a\":50,\"b\":10,\"n\":4},\"name\":\"p1\"}");
            Assert.IsNull(Error(response), response);
            Assert.IsTrue(server.namedItems.ContainsKey("p1"));
        }

        [TestMethod]
        public void FailedCheckIsReportedWithItsValues()
        {
            MCPServer server = ServerWithPlateTemplate();
            string? error = Error(Call(server, "template.instantiate", "{\"template\":\"plate\",\"arguments\":{\"a\":15,\"b\":20},\"name\":\"p2\"}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "Die Länge (15) muss größer als die Breite (20) sein, mindestens 21.");
            Assert.IsFalse(server.namedItems.ContainsKey("p2"));
        }

        [TestMethod]
        public void MinAndMaxAreChecked()
        {
            MCPServer server = ServerWithPlateTemplate();
            // n below its minimum, a above its maximum 10*b: both are reported at once
            string? error = Error(Call(server, "template.instantiate", "{\"template\":\"plate\",\"arguments\":{\"a\":300,\"b\":20,\"n\":1},\"name\":\"p3\"}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "Anzahl must be at least 2.");
            StringAssert.Contains(error, "Länge must be at most 200.");
        }

        [TestMethod]
        public void TheWorkspaceIsRestoredAfterAFailedCheck()
        {
            MCPServer server = ServerWithPlateTemplate();
            server.namedItems["a"] = 7.0;
            Assert.IsNotNull(Error(Call(server, "template.instantiate", "{\"template\":\"plate\",\"arguments\":{\"a\":1,\"b\":20},\"name\":\"p4\"}")));
            Assert.AreEqual(7.0, server.namedItems["a"], "the template's values must not leak into the caller's workspace");
            Assert.IsNull(Error(Call(server, "template.instantiate", "{\"template\":\"plate\",\"name\":\"p5\"}")), "the defaults still build");
        }

        [TestMethod]
        public void PresetSettingAnUnknownParameterIsRefused()
        {
            MCPServer server = NewServer();
            string presets = @"[{""name"":""sizes"",""label"":""Größe"",""options"":[{""label"":""x"",""values"":{""c"":1}}]}]";
            string? error = Error(Call(server, "template.begin", $"{{\"name\":\"bad\",\"parameters\":{Parameters},\"presets\":{presets}}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "'c'");
            // the failed template.begin started no recording: a commit has nothing to end
            string? commitError;
            try { commitError = Error(Call(server, "template.commit", "{\"result\":\"nothing\"}")); }
            catch (System.Exception ex) { commitError = ex.Message; }
            Assert.IsNotNull(commitError);
            StringAssert.Contains(commitError, "no 'template.begin'");
            Assert.IsFalse(server.templates.ContainsKey("bad"));
        }

        [TestMethod]
        public void CheckNamingAnUnknownParameterIsRefused()
        {
            MCPServer server = NewServer();
            string checks = @"[{""condition"":""a > 0"",""message"":""m"",""parameters"":[""z""]}]";
            string? error = Error(Call(server, "template.begin", $"{{\"name\":\"bad\",\"parameters\":{Parameters},\"checks\":{checks}}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "'z'");
        }

        [TestMethod]
        public void DefaultsViolatingAChecksAreAWarning()
        {
            MCPServer server = NewServer();
            string checks = @"[{""condition"":""a < b"",""message"":""a muss kleiner als b sein""}]";
            string response = Call(server, "template.begin", $"{{\"name\":\"warned\",\"parameters\":{Parameters},\"checks\":{checks}}}");
            Assert.IsNull(Error(response), response);
            StringAssert.Contains(response, "a muss kleiner als b sein");
            Assert.IsNull(Error(Call(server, "solid.box", "{\"origin\":[0,0,0],\"sizeX\":\"a\",\"sizeY\":\"b\",\"sizeZ\":\"n\",\"name\":\"plate\"}")));
            Assert.IsNull(Error(Call(server, "template.commit", "{\"result\":\"plate\"}")));
            Assert.IsTrue(server.templates.ContainsKey("warned"));
        }

        [TestMethod]
        public void ProblemsNameTheirParameters()
        {
            using JsonDocument parameters = JsonDocument.Parse(Parameters);
            using JsonDocument checks = JsonDocument.Parse(Checks);
            Dictionary<string, object> values = new() { ["a"] = 10.0, ["b"] = 20.0, ["n"] = 1 };
            List<ParameterProblem> problems = TemplateParameterRules.Check(parameters.RootElement, checks.RootElement, values,
                (label, isMinimum, limit) => $"{label} {(isMinimum ? ">=" : "<=")} {limit}");
            Assert.AreEqual(2, problems.Count);
            Assert.AreEqual("Anzahl >= 2", problems[0].Message, "an integer value is compared like a number");
            CollectionAssert.AreEqual(new[] { "n" }, (System.Collections.ICollection)problems[0].Parameters);
            CollectionAssert.AreEqual(new[] { "a", "b" }, (System.Collections.ICollection)problems[1].Parameters);
        }

        [TestMethod]
        public void UnknownNamesAreIgnoredWhileObjectsAreMissing()
        {
            using JsonDocument parameters = JsonDocument.Parse("[]");
            using JsonDocument checks = JsonDocument.Parse(@"[{""condition"":""hull.Volume > 0"",""message"":""m""}]");
            Dictionary<string, object> values = new();
            Assert.AreEqual(0, TemplateParameterRules.Check(parameters.RootElement, checks.RootElement, values, ignoreEvaluationErrors: true).Count);
            List<ParameterProblem> problems = TemplateParameterRules.Check(parameters.RootElement, checks.RootElement, values);
            Assert.AreEqual(1, problems.Count);
            StringAssert.Contains(problems[0].Message, "cannot be evaluated");
        }

        [TestMethod]
        public void PresetsAreReadAndEvaluated()
        {
            using JsonDocument presets = JsonDocument.Parse(Presets);
            List<TemplatePreset> read = TemplateParameterRules.ReadPresets(presets.RootElement);
            Assert.AreEqual(1, read.Count);
            Assert.AreEqual("Größe", read[0].Label);
            Assert.AreEqual(2, read[0].Options.Count);
            Assert.AreEqual("klein", read[0].Options[0].Label);
            Assert.AreEqual(2, read[0].Options[0].Values.Count);
            Dictionary<string, object> values = new() { ["a"] = 30.0, ["b"] = 12.5 };
            Assert.AreEqual(25.0, TemplateParameterRules.EvaluateValue(read[0].Options[1].Values[0].Value, values), "an expression uses the current values");
            Assert.AreEqual(20.0, TemplateParameterRules.EvaluateValue(read[0].Options[0].Values[0].Value, values));
        }
    }
}
