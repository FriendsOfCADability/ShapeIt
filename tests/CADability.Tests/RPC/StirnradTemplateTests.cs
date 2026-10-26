using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// The "stirnrad" template of StirnradTemplate.json: a spur gear with involute flanks. The volume is compared with
    /// the cross section computed here independently in polar form (involute function, not the Cartesian form of
    /// the template), and the rules of the template are checked: undercut, pointed teeth, bore too large.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class StirnradTemplateTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        private static string TemplateFile([System.Runtime.CompilerServices.CallerFilePath] string thisFile = "")
            => System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile)!, "..", "Files", "Templates", "StirnradTemplate.json");

        /// <summary>Loads the templates of a template file the way the application does.</summary>
        private static MCPServer ServerWithTemplates()
        {
            Project project = Project.CreateSimpleProject();
            MCPServer server = new MCPServer(null!, project) { SuppressDialogs = true };
            using JsonDocument document = JsonDocument.Parse(System.IO.File.ReadAllText(TemplateFile()));
            foreach (JsonElement template in document.RootElement.GetProperty("templates").EnumerateArray())
            {
                List<JsonElement> items = new List<JsonElement>();
                foreach (JsonElement item in template.GetProperty("items").EnumerateArray()) items.Add(item.Clone());
                server.templates[template.GetProperty("name").GetString()!] = items;
            }
            return server;
        }

        private sealed record Gear(double m, int z, double x = 0, double alpha = 20, double haP = 1, double hfP = 1.25, double Asn = 0, double b = 20, double dBore = 0)
        {
            public string Arguments => FormattableString.Invariant(
                $"{{\"m\":{m},\"z\":{z},\"x\":{x},\"alpha\":{alpha},\"haP\":{haP},\"hfP\":{hfP},\"Asn\":{Asn},\"b\":{b},\"dBore\":{dBore}}}");
        }

        private static double Inv(double a) => Math.Tan(a) - a;

        /// <summary>
        /// The area of the cross section: per tooth the radial line below the base circle, the involute, the tip
        /// arc, the other involute, the other radial line and the root arc, densely sampled in polar coordinates.
        /// </summary>
        private static double CrossSectionArea(Gear g, out double ra, out double rf)
        {
            double a = g.alpha * Math.PI / 180;
            double r = g.m * g.z / 2, rb = r * Math.Cos(a);
            ra = r + g.m * (g.haP + g.x);
            rf = r - g.m * (g.hfP - g.x);
            double s = g.m * (Math.PI / 2 + 2 * g.x * Math.Tan(a)) + g.Asn; // tooth thickness at the reference circle
            double psib = s / (2 * r) + Inv(a);                            // half tooth angle at the base circle
            double psia = psib - Inv(Math.Acos(rb / ra));
            double psir = rf < rb ? psib : psib - Inv(Math.Acos(rb / rf));
            double tStart = Math.Sqrt(Math.Pow(Math.Max(rf, rb) / rb, 2) - 1), tEnd = Math.Sqrt(Math.Pow(ra / rb, 2) - 1);
            const int n = 400;
            List<(double radius, double angle)> polar = new List<(double, double)>();
            double rfLocal = rf, raLocal = ra;
            for (int k = 0; k < g.z; k++)
            {
                double phi = k * 2 * Math.PI / g.z;
                if (rf < rb) { polar.Add((rfLocal, phi - psib)); }
                for (int i = 0; i <= n; i++)
                {   // the flank below the tooth axis, from the root to the tip: the polar angle of the involute is inv
                    double t = tStart + i * (tEnd - tStart) / n;
                    polar.Add((rb * Math.Sqrt(1 + t * t), phi - psib + (t - Math.Atan(t))));
                }
                for (int i = 1; i < n; i++) polar.Add((raLocal, phi - psia + i * 2 * psia / n));
                for (int i = n; i >= 0; i--)
                {
                    double t = tStart + i * (tEnd - tStart) / n;
                    polar.Add((rb * Math.Sqrt(1 + t * t), phi + psib - (t - Math.Atan(t))));
                }
                if (rf < rb) { polar.Add((rfLocal, phi + psib)); }
                for (int i = 1; i < n; i++) polar.Add((rfLocal, phi + psir + i * (2 * Math.PI / g.z - 2 * psir) / n));
            }
            double area = 0;
            for (int i = 0; i < polar.Count; i++)
            {
                (double r1, double a1) = polar[i];
                (double r2, double a2) = polar[(i + 1) % polar.Count];
                area += r1 * r2 * Math.Sin(a2 - a1) / 2; // shoelace in polar form
            }
            return area;
        }

        private static Solid Instantiate(MCPServer server, Gear gear, string name)
        {
            string response = Call(server, "template.instantiate", $"{{\"template\":\"stirnrad\",\"arguments\":{gear.Arguments},\"name\":\"{name}\"}}");
            Assert.IsNull(Error(response), response);
            object result = server.namedItems[name];
            return result as Solid ?? ((List<Solid>)result)[0];
        }

        private static void AssertGear(Gear gear)
        {
            MCPServer server = ServerWithTemplates();
            Solid solid = Instantiate(server, gear, "gear");
            double area = CrossSectionArea(gear, out double ra, out double rf);
            double expected = gear.b * (area - Math.PI * gear.dBore * gear.dBore / 4);
            // measured: 3e-7 relative; a finer precision only costs time (1e-6·m: half a minute per gear)
            double volume = solid.Volume(1e-3 * gear.m);
            Assert.AreEqual(expected, volume, 1e-5 * expected, $"volume of {gear}");
            if (gear.dBore == 0)
            {   // per tooth two flanks, tip and root, below the base circle two radial faces; the two end faces
                bool radial = rf < gear.m * gear.z / 2 * Math.Cos(gear.alpha * Math.PI / 180);
                Assert.AreEqual(gear.z * (radial ? 6 : 4) + 2, solid.Shells[0].Faces.Length, "faces");
            }
            BoundingBox ext = solid.GetExtent(0.0);
            Assert.AreEqual(gear.b, ext.Zmax - ext.Zmin, 1e-6, "face width");
            // the tip circle bounds the gear; a tooth on the x-axis reaches it exactly
            Assert.IsTrue(ext.Xmax <= ra + 1e-6 && ext.Xmax > ra - 1e-3 * gear.m, $"tip circle {ra}, extent {ext.Xmax}");
            // the root circle: the gap centered on angle 180°/z lies on it
            double half = Math.PI / gear.z;
            GeoPoint inGap = new GeoPoint((rf - 0.01 * gear.m) * Math.Cos(half), (rf - 0.01 * gear.m) * Math.Sin(half), gear.b / 2);
            GeoPoint outsideGap = new GeoPoint((rf + 0.01 * gear.m) * Math.Cos(half), (rf + 0.01 * gear.m) * Math.Sin(half), gear.b / 2);
            Assert.IsTrue(solid.Shells[0].IsInside(inGap), "below the root circle is material");
            Assert.IsFalse(solid.Shells[0].IsInside(outsideGap), "above the root circle in the middle of the gap is none");
        }

        [TestMethod]
        public void DefaultGearWithRadialFlankBelowTheBaseCircle()
        {
            // z = 20: the root circle (17.5) lies inside the base circle (18.79)
            AssertGear(new Gear(2, 20));
        }

        [TestMethod]
        public void ManyTeethFlankStartsAtTheRootCircle()
        {
            // z = 60: the root circle (57.5) lies outside the base circle (56.38), there is no radial piece
            AssertGear(new Gear(1.5, 60, b: 8));
        }

        [TestMethod]
        public void ProfileShiftBacklashAndBore()
        {
            AssertGear(new Gear(3, 12, x: 0.4, Asn: -0.1, b: 15, dBore: 12));
        }

        [TestMethod]
        public void UndercutIsRefusedWithTheShiftNeeded()
        {
            MCPServer server = ServerWithTemplates();
            string? error = Error(Call(server, "template.instantiate", $"{{\"template\":\"stirnrad\",\"arguments\":{new Gear(2, 10).Arguments},\"name\":\"g\"}}"));
            Assert.IsNotNull(error);
            // x >= 1 - 10*sin²(20°)/2 = 0.4151, or z >= 2/sin²(20°) = 17.1
            StringAssert.Contains(error, "Unterschnitt: Bei z = 10 und α = 20° braucht das Rad eine Profilverschiebung x ≥ 0.4151 oder mindestens 18 Zähne.");
        }

        [TestMethod]
        public void PointedTeethAreRefused()
        {
            MCPServer server = ServerWithTemplates();
            string? error = Error(Call(server, "template.instantiate", $"{{\"template\":\"stirnrad\",\"arguments\":{new Gear(2, 10, x: 1.2).Arguments},\"name\":\"g\"}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "Die Zähne werden spitz");
            Assert.IsFalse(error!.Contains("Unterschnitt"), "the shift avoids undercut: " + error);
        }

        [TestMethod]
        public void BoreTooLargeIsRefused()
        {
            MCPServer server = ServerWithTemplates();
            // root circle 35, at least one module of material: at most 31
            string? error = Error(Call(server, "template.instantiate", $"{{\"template\":\"stirnrad\",\"arguments\":{new Gear(2, 20, dBore: 32).Arguments},\"name\":\"g\"}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "die Bohrung darf höchstens ⌀ 31 mm haben");
        }

        [TestMethod]
        public void ChecksWorkWithIntegerToothCount()
        {
            // the application passes the tooth count as int, an instantiation as double
            using JsonDocument document = JsonDocument.Parse(System.IO.File.ReadAllText(TemplateFile()));
            JsonElement begin = document.RootElement.GetProperty("templates")[0].GetProperty("items")[0].GetProperty("params");
            Dictionary<string, object> values = new()
            {
                ["center"] = GeoPoint.Origin, ["m"] = 2.0, ["z"] = 10, ["b"] = 20.0, ["x"] = 0.0, ["alpha"] = 20.0,
                ["haP"] = 1.0, ["hfP"] = 1.25, ["Asn"] = 0.0, ["dBore"] = 0.0
            };
            List<ParameterProblem> problems = TemplateParameterRules.Check(begin.GetProperty("parameters"), begin.GetProperty("checks"), values);
            Assert.AreEqual(1, problems.Count, string.Join(" | ", problems.ConvertAll(p => p.Message)));
            StringAssert.Contains(problems[0].Message, "Unterschnitt");
            values["z"] = 20;
            problems = TemplateParameterRules.Check(begin.GetProperty("parameters"), begin.GetProperty("checks"), values);
            Assert.AreEqual(0, problems.Count, string.Join(" | ", problems.ConvertAll(p => p.Message)));
        }

        [TestMethod]
        public void PresetsSetDeclaredParameters()
        {
            using JsonDocument document = JsonDocument.Parse(System.IO.File.ReadAllText(TemplateFile()));
            JsonElement begin = document.RootElement.GetProperty("templates")[0].GetProperty("items")[0].GetProperty("params");
            // the definition passes the checks template.begin applies when it is recorded
            TemplateParameterRules.ValidateDefinition(begin.GetProperty("parameters"), begin.GetProperty("presets"), begin.GetProperty("checks"));
            List<TemplatePreset> presets = TemplateParameterRules.ReadPresets(begin.GetProperty("presets"));
            Assert.AreEqual("Modulreihe I (DIN 780)", presets[0].Label);
            Assert.IsTrue(System.Linq.Enumerable.Any(presets[0].Options, o => o.Label == "1,25"));
        }
    }
}
