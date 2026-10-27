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
            => System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile)!, "..", "..", "..", "ShapeIt", "Templates", "StirnradTemplate.json");

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

        private static Solid InstantiateTemplate(MCPServer server, string template, string arguments)
        {
            string response = Call(server, "template.instantiate", $"{{\"template\":\"{template}\",\"arguments\":{arguments},\"name\":\"result\"}}");
            Assert.IsNull(Error(response), response);
            object result = server.namedItems["result"];
            return result as Solid ?? ((List<Solid>)result)[0];
        }

        private sealed record Rack(double m, int z, double b = 20, double H = 20, double alpha = 20, double haP = 1, double hfP = 1.25, double Asn = 0)
        {
            public string Arguments => FormattableString.Invariant(
                $"{{\"m\":{m},\"z\":{z},\"b\":{b},\"H\":{H},\"alpha\":{alpha},\"haP\":{haP},\"hfP\":{hfP},\"Asn\":{Asn}}}");
        }

        private static void AssertRack(Rack rack)
        {
            Solid solid = InstantiateTemplate(ServerWithTemplates(), "zahnstange", rack.Arguments);
            double p = Math.PI * rack.m, ha = rack.haP * rack.m, hf = rack.hfP * rack.m;
            double s = p / 2 + rack.Asn; // tooth thickness at the reference line
            double tipAndRoot = 2 * s + 2 * (hf - ha) * Math.Tan(rack.alpha * Math.PI / 180); // tip width plus root width of a tooth
            // the body below the root line and z trapezoidal teeth
            double expected = rack.b * (rack.z * p * (rack.H - ha - hf) + rack.z * tipAndRoot / 2 * (ha + hf));
            Assert.AreEqual(expected, solid.Volume(1e-3 * rack.m), 1e-6 * expected, $"volume of {rack}");
            // per tooth two flanks, the tip and the root behind it, the root before the first tooth, the two ends and
            // the back, and the two side faces
            Assert.AreEqual(4 * rack.z + 4 + 2, solid.Shells[0].Faces.Length, "faces");
            BoundingBox ext = solid.GetExtent(0.0);
            Assert.AreEqual(0, ext.Xmin, 1e-6);
            Assert.AreEqual(rack.z * p, ext.Xmax, 1e-6, "z pitches long");
            Assert.AreEqual(ha, ext.Ymax, 1e-6, "tip line");
            Assert.AreEqual(ha - rack.H, ext.Ymin, 1e-6, "back");
            Assert.AreEqual(rack.b, ext.Zmax - ext.Zmin, 1e-6, "face width");
            // the middle of the first tooth reaches the tip line, the middle of the gap between the first two teeth the root line
            Assert.IsTrue(solid.Shells[0].IsInside(new GeoPoint(p / 2, ha - 0.01 * rack.m, rack.b / 2)), "tooth");
            Assert.IsFalse(solid.Shells[0].IsInside(new GeoPoint(p, -hf + 0.01 * rack.m, rack.b / 2)), "gap");
            Assert.IsTrue(solid.Shells[0].IsInside(new GeoPoint(p, -hf - 0.01 * rack.m, rack.b / 2)), "below the gap");
        }

        [TestMethod]
        public void DefaultRack()
        {
            AssertRack(new Rack(2, 10));
        }

        [TestMethod]
        public void RackWithBacklashAndOtherBasicRack()
        {
            AssertRack(new Rack(1.25, 7, b: 12, H: 9, alpha: 25, haP: 0.8, hfP: 1, Asn: -0.05));
        }

        [TestMethod]
        public void RackTooLowIsRefused()
        {
            // 2.25 modules of teeth and one module of body: at least 6.5
            string? error = Error(Call(ServerWithTemplates(), "template.instantiate", $"{{\"template\":\"zahnstange\",\"arguments\":{new Rack(2, 10, H: 6).Arguments},\"name\":\"r\"}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "die Höhe muss mindestens 6.5 mm sein");
        }

        private sealed record Ring(double m, int z, double dOut, double b = 20, double alpha = 20, double haP = 1, double hfP = 1.25, double Asn = 0)
        {
            public string Arguments => FormattableString.Invariant(
                $"{{\"m\":{m},\"z\":{z},\"dOut\":{dOut},\"b\":{b},\"alpha\":{alpha},\"haP\":{haP},\"hfP\":{hfP},\"Asn\":{Asn}}}");
        }

        /// <summary>
        /// The area inside the teeth of an internal gear: per gap the involute from the tip to the root circle, the root
        /// arc, the other involute and the tip arc of the next tooth, densely sampled in polar coordinates.
        /// </summary>
        private static double HoleArea(Ring g, out double rai, out double rfi)
        {
            double a = g.alpha * Math.PI / 180;
            double r = g.m * g.z / 2, rb = r * Math.Cos(a);
            rai = Math.Max(r - g.haP * g.m, rb); // the tip circle is enlarged to the base circle where the involute ends
            rfi = r + g.hfP * g.m;
            double psib = (Math.PI / 2 - g.Asn / g.m) / g.z + Inv(a); // half gap angle at the base circle
            double psia = psib - Inv(Math.Acos(rb / rai));
            double psir = psib - Inv(Math.Acos(rb / rfi));
            double tStart = Math.Sqrt(Math.Pow(rai / rb, 2) - 1), tEnd = Math.Sqrt(Math.Pow(rfi / rb, 2) - 1);
            const int n = 400;
            List<(double radius, double angle)> polar = new List<(double, double)>();
            for (int k = 0; k < g.z; k++)
            {
                double phi = k * 2 * Math.PI / g.z;
                for (int i = 0; i <= n; i++)
                {
                    double t = tStart + i * (tEnd - tStart) / n;
                    polar.Add((rb * Math.Sqrt(1 + t * t), phi - psib + (t - Math.Atan(t))));
                }
                for (int i = 1; i < n; i++) polar.Add((rfi, phi - psir + i * 2 * psir / n));
                for (int i = n; i >= 0; i--)
                {
                    double t = tStart + i * (tEnd - tStart) / n;
                    polar.Add((rb * Math.Sqrt(1 + t * t), phi + psib - (t - Math.Atan(t))));
                }
                for (int i = 1; i < n; i++) polar.Add((rai, phi + psia + i * (2 * Math.PI / g.z - 2 * psia) / n));
            }
            double area = 0;
            for (int i = 0; i < polar.Count; i++)
            {
                (double r1, double a1) = polar[i];
                (double r2, double a2) = polar[(i + 1) % polar.Count];
                area += r1 * r2 * Math.Sin(a2 - a1) / 2;
            }
            return area;
        }

        private static void AssertRing(Ring ring)
        {
            Solid solid = InstantiateTemplate(ServerWithTemplates(), "hohlrad", ring.Arguments);
            double expected = ring.b * (Math.PI * ring.dOut * ring.dOut / 4 - HoleArea(ring, out double rai, out double rfi));
            double volume = solid.Volume(1e-3 * ring.m);
            Assert.AreEqual(expected, volume, 1e-5 * expected, $"volume of {ring}");
            BoundingBox ext = solid.GetExtent(0.0);
            Assert.AreEqual(ring.b, ext.Zmax - ext.Zmin, 1e-6, "face width");
            Assert.AreEqual(ring.dOut / 2, ext.Xmax, 1e-6, "outer diameter");
            // a gap is centered on the x-axis, a tooth on the angle 180°/z
            double half = Math.PI / ring.z, d = 0.01 * ring.m, zMid = ring.b / 2;
            Assert.IsFalse(solid.Shells[0].IsInside(new GeoPoint(rfi - d, 0, zMid)), "inside the root circle in the gap is none");
            Assert.IsTrue(solid.Shells[0].IsInside(new GeoPoint(rfi + d, 0, zMid)), "outside the root circle is material");
            Assert.IsTrue(solid.Shells[0].IsInside(new GeoPoint((rai + d) * Math.Cos(half), (rai + d) * Math.Sin(half), zMid)), "the tooth reaches the tip circle");
            Assert.IsFalse(solid.Shells[0].IsInside(new GeoPoint((rai - d) * Math.Cos(half), (rai - d) * Math.Sin(half), zMid)), "inside the tip circle is none");
        }

        [TestMethod]
        public void DefaultInternalGear()
        {
            AssertRing(new Ring(2, 40, 100));
        }

        [TestMethod]
        public void InternalGearWithTipCircleEnlargedToTheBaseCircle()
        {
            // z = 24: the tip circle (22) would lie inside the base circle (22.55)
            AssertRing(new Ring(2, 24, 60, b: 10, Asn: -0.1));
        }

        [TestMethod]
        public void InternalGearOuterDiameterTooSmallIsRefused()
        {
            // root circle 85, at least one module of material: at least 89
            string? error = Error(Call(ServerWithTemplates(), "template.instantiate", $"{{\"template\":\"hohlrad\",\"arguments\":{new Ring(2, 40, 88).Arguments},\"name\":\"r\"}}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "der Außendurchmesser muss mindestens ⌀ 89 mm sein");
        }

        [TestMethod]
        public void TheFileIsOneGroupOfThreeTemplates()
        {
            Dictionary<string, List<JsonElement>> templates = new();
            TemplateFile file = TemplateLibrary.ReadFile(TemplateFile(), templates);
            Assert.AreEqual("Zahnräder", file.Label);
            Assert.IsNotNull(file.Icon, "the icon of the drop-down button is accepted");
            CollectionAssert.AreEqual(new[] { "stirnrad", "zahnstange", "hohlrad" }, (System.Collections.ICollection)file.Names);
            List<TemplateToolbarEntry> entries = TemplateLibrary.ToolbarEntries(templates.Keys, new[] { file }, name => name);
            Assert.AreEqual(1, entries.Count, "one drop-down button");
            Assert.AreEqual(3, entries[0].Templates.Count);
            foreach (List<JsonElement> items in templates.Values)
            {
                JsonElement begin = items[0].GetProperty("params");
                Assert.IsNull(TemplateIcon.Problem(begin.GetProperty("icon").GetString()!), begin.GetProperty("name").GetString());
                TemplateParameterRules.ValidateDefinition(begin.GetProperty("parameters"), begin.GetProperty("presets"), begin.GetProperty("checks"));
            }
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
