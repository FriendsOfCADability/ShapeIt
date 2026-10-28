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
    /// The "sacklochbohrung" template of BohrungTemplate.json: a blind hole perpendicular to a face, with a sharp,
    /// chamfered or rounded entry and a flat, conical or spherical bottom. The removed volume is compared with the
    /// volume of the hole computed here from its shape.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class BohrungTemplateTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();
        private static Solid FirstSolid(object item) => item as Solid ?? ((List<Solid>)item)[0];

        private static string TemplateFile([System.Runtime.CompilerServices.CallerFilePath] string thisFile = "")
            => System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile)!, "..", "..", "..", "ShapeIt", "Templates", "BohrungTemplate.json");

        /// <summary>Loads the templates of the file the way the application does, and a 20x20x20 block with its faces.</summary>
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
            Assert.IsNull(Error(Call(server, "solid.box", "{\"name\":\"block\",\"origin\":[0,0,0],\"sizeX\":20,\"sizeY\":20,\"sizeZ\":20}")));
            Solid block = FirstSolid(server.namedItems["block"]);
            server.namedItems["top"] = PlanarFace(block, 2);
            server.namedItems["side"] = PlanarFace(block, 0);
            return server;
        }

        /// <summary>The face of the block lying in the plane where the given coordinate (0: x, 2: z) is 20.</summary>
        private static Face PlanarFace(Solid solid, int axis)
        {
            return solid.Shells[0].Faces.Single(f =>
            {
                BoundingBox box = f.GetBoundingCube();
                double min = axis == 0 ? box.Xmin : box.Zmin, max = axis == 0 ? box.Xmax : box.Zmax;
                return Math.Abs(min - 20) < 1e-9 && Math.Abs(max - 20) < 1e-9;
            });
        }

        private const double R = 4, Depth = 15; // d = 8 as in the template
        private static readonly double tipHalf = 59 * Math.PI / 180;

        /// <summary>Instantiates the template on the given face and returns the volume taken out of the solid.</summary>
        private static double RemovedVolume(MCPServer server, string arguments, string template = "sacklochbohrung", string solid = "block", bool measureCutOut = false)
        {
            double before = FirstSolid(server.namedItems[solid]).Volume(0.0001);
            string response = Call(server, "template.instantiate", $"{{\"template\":\"{template}\",\"arguments\":{arguments},\"name\":\"drilled\"}}");
            Assert.IsNull(Error(response), response);
            Solid drilled = FirstSolid(server.namedItems["drilled"]);
            Assert.AreEqual(before, FirstSolid(server.namedItems[solid]).Volume(0.0001), 1e-6, "the solid itself must stay untouched");
            if (!measureCutOut) return before - drilled.Volume(0.0001);
            // the difference of two large volumes integrated on curved faces is too coarse for an exact comparison:
            // measure the cut out piece
            string removedName = "removed" + (++removedCount); // a fresh name: a failed boolean must not find an older piece
            Assert.IsNull(Error(Call(server, "solid.boolean", $"{{\"op\":\"difference\",\"a\":\"{solid}\",\"b\":\"drilled\",\"name\":\"{removedName}\"}}")));
            Assert.IsTrue(server.namedItems.TryGetValue(removedName, out object? removed), "the cut out piece");
            return removed is List<Solid> pieces ? pieces.Sum(p => p.Volume(0.0001)) : ((Solid)removed).Volume(0.0001);
        }

        private static int removedCount;

        private static double ChamferRing(double c) => 2 * Math.PI * (R + c / 3) * c * c / 2;

        private static double FilletRing(double rho)
        {
            double centroid = rho * (10 - 3 * Math.PI) / (12 - 3 * Math.PI); // distance of the centroid from the edge
            return 2 * Math.PI * (R + centroid) * rho * rho * (1 - Math.PI / 4);
        }

        private static double Cylinder => Math.PI * R * R * Depth;

        [TestMethod]
        public void FlatBottomSharpEntry()
        {
            MCPServer server = ServerWithTemplates();
            Assert.AreEqual(Cylinder, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\"}"), 1e-3);
        }

        [TestMethod]
        public void PointedConeBottom()
        {
            MCPServer server = ServerWithTemplates();
            double h = R / Math.Tan(tipHalf);
            Assert.AreEqual(Cylinder + Math.PI * R * R * h / 3, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"bottom\":\"cone\"}"), 1e-3);
        }

        [TestMethod]
        public void TruncatedConeBottom()
        {
            MCPServer server = ServerWithTemplates();
            double rb = 1.5, h = (R - rb) / Math.Tan(tipHalf);
            double frustum = Math.PI * h * (R * R + R * rb + rb * rb) / 3;
            Assert.AreEqual(Cylinder + frustum, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"bottom\":\"cone\",\"bottomDiameter\":3}"), 1e-3);
        }

        [TestMethod]
        public void SphericalBottom()
        {
            MCPServer server = ServerWithTemplates();
            Assert.AreEqual(Cylinder + 2.0 / 3.0 * Math.PI * R * R * R, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"bottom\":\"sphere\"}"), 1e-3);
        }

        /// <summary>A 45° chamfer of width c: a triangle with legs c revolved around the axis.</summary>
        [TestMethod]
        public void ChamferedEntry()
        {
            MCPServer server = ServerWithTemplates();
            double c = 1;
            double chamfer = 2 * Math.PI * (R + c / 3) * c * c / 2;
            Assert.AreEqual(Cylinder + chamfer, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"entry\":\"chamfer\",\"entrySize\":1}"), 1e-3);
        }

        /// <summary>A fillet of radius rho: the square rho x rho without the quarter circle, revolved (Pappus).</summary>
        [TestMethod]
        public void RoundedEntryWithSphericalBottom()
        {
            MCPServer server = ServerWithTemplates();
            double rho = 1.5;
            double area = rho * rho * (1 - Math.PI / 4);
            double centroid = rho * (10 - 3 * Math.PI) / (12 - 3 * Math.PI); // distance of the centroid from the edge
            double fillet = 2 * Math.PI * (R + centroid) * area;
            double expected = Cylinder + fillet + 2.0 / 3.0 * Math.PI * R * R * R;
            Assert.AreEqual(expected, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"entry\":\"round\",\"entrySize\":1.5,\"bottom\":\"sphere\"}"), 1e-2);
        }

        /// <summary>A face whose normal is the x-axis takes the other branch for the radial direction of the profile;
        /// the position is given off the face and put onto it.</summary>
        [TestMethod]
        public void SideFaceWithPointOffTheFace()
        {
            MCPServer server = ServerWithTemplates();
            double h = R / Math.Tan(tipHalf);
            Assert.AreEqual(Cylinder + Math.PI * R * R * h / 3,
                RemovedVolume(server, "{\"face\":\"side\",\"pos\":\"p(35,10,8)\",\"bottom\":\"cone\",\"entry\":\"chamfer\",\"entrySize\":0.5}")
                    - 2 * Math.PI * (R + 0.5 / 3) * 0.125, 1e-3);
        }

        /// <summary>A template loaded from a file was never recorded here: check its definition and icon as template.begin would.</summary>
        [TestMethod]
        public void DefinitionIsValid()
        {
            using JsonDocument document = JsonDocument.Parse(System.IO.File.ReadAllText(TemplateFile()));
            foreach (JsonElement template in document.RootElement.GetProperty("templates").EnumerateArray())
            {
                JsonElement begin = template.GetProperty("items")[0].GetProperty("params");
                begin.TryGetProperty("presets", out JsonElement presets);
                begin.TryGetProperty("checks", out JsonElement checks);
                TemplateParameterRules.ValidateDefinition(begin.GetProperty("parameters"), presets, checks);
                if (begin.TryGetProperty("icon", out JsonElement icon)) Assert.IsNull(TemplateIcon.Problem(icon.GetString()!));
            }
            Assert.IsNull(TemplateIcon.Problem(document.RootElement.GetProperty("icon").GetString()!));
        }

        [TestMethod]
        public void BottomDiameterTooLargeIsRefused()
        {
            MCPServer server = ServerWithTemplates();
            string? error = Error(Call(server, "template.instantiate", "{\"template\":\"sacklochbohrung\",\"arguments\":{\"face\":\"top\",\"bottom\":\"cone\",\"bottomDiameter\":8},\"name\":\"x\"}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "bottom diameter");
        }

        [TestMethod]
        public void ThroughHoleSharp()
        {
            MCPServer server = ServerWithTemplates();
            Assert.AreEqual(Math.PI * R * R * 20, RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\"}", "durchgangsbohrung"), 1e-3);
        }

        [TestMethod]
        public void ThroughHoleChamferedOnBothEnds()
        {
            MCPServer server = ServerWithTemplates();
            Assert.AreEqual(Math.PI * R * R * 20 + 2 * ChamferRing(1),
                RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"entry\":\"chamfer\",\"exit\":\"chamfer\"}", "durchgangsbohrung"), 1e-3);
        }

        [TestMethod]
        public void ThroughHoleRoundedEntryChamferedExitOnTheSide()
        {
            MCPServer server = ServerWithTemplates();
            Assert.AreEqual(Math.PI * R * R * 20 + FilletRing(1.5) + ChamferRing(0.5),
                RemovedVolume(server, "{\"face\":\"side\",\"pos\":\"p(35,10,8)\",\"entry\":\"round\",\"entrySize\":1.5,\"exit\":\"chamfer\",\"exitSize\":0.5}", "durchgangsbohrung"), 1e-2);
        }

        private const double TaperDepth = 12;

        /// <summary>The conical part of a tapered hole from radius r1 at the face to r2 at the depth.</summary>
        private static double Frustum(double r1, double r2, double h) => Math.PI * h * (r1 * r1 + r1 * r2 + r2 * r2) / 3;

        /// <summary>The spherical bottom tangent to the wall: the part of the sphere below the end of the wall.</summary>
        private static double TangentSphereBottom(double r1, double r2, double h)
        {
            double slope = (r2 - r1) / h, rho = r2 * Math.Sqrt(1 + slope * slope);
            double cap = rho + r2 * slope; // height of the part below the plane of the end of the wall
            return Math.PI * cap * cap * (3 * rho - cap) / 3;
        }

        private static double Taper(MCPServer server, string arguments)
            => RemovedVolume(server, "{\"face\":\"top\",\"pos\":\"p(10,10,20)\",\"depth\":12," + arguments + "}", "kegelbohrung");

        [TestMethod]
        public void TaperNarrowingFlat()
        {
            MCPServer server = ServerWithTemplates();
            Assert.AreEqual(Frustum(4, 2.5, TaperDepth), Taper(server, "\"dEntry\":8,\"dEnd\":5"), 1e-3);
        }

        [TestMethod]
        public void TaperWideningWithPointedCone()
        {
            MCPServer server = ServerWithTemplates();
            double h = 5 / Math.Tan(tipHalf);
            // 1e-2: the volume is integrated with a relative precision of about 1e-6
            Assert.AreEqual(Frustum(3, 5, TaperDepth) + Math.PI * 25 * h / 3, Taper(server, "\"dEntry\":6,\"dEnd\":10,\"bottom\":\"cone\""), 1e-2);
        }

        [TestMethod]
        public void TaperNarrowingWithTruncatedCone()
        {
            MCPServer server = ServerWithTemplates();
            double rb = 1, h = (3 - rb) / Math.Tan(tipHalf);
            Assert.AreEqual(Frustum(4, 3, TaperDepth) + Frustum(3, rb, h),
                Taper(server, "\"dEntry\":8,\"dEnd\":6,\"bottom\":\"cone\",\"flatDiameter\":2"), 1e-3);
        }

        [TestMethod]
        public void TaperSphericalBottomNarrowingAndWidening()
        {
            MCPServer server = ServerWithTemplates();
            Assert.AreEqual(Frustum(4, 2.5, TaperDepth) + TangentSphereBottom(4, 2.5, TaperDepth),
                Taper(server, "\"dEntry\":8,\"dEnd\":5,\"bottom\":\"sphere\""), 1e-3);
            server = ServerWithTemplates();
            Assert.AreEqual(Frustum(2.5, 4, TaperDepth) + TangentSphereBottom(2.5, 4, TaperDepth),
                Taper(server, "\"dEntry\":5,\"dEnd\":8,\"bottom\":\"sphere\""), 1e-3);
        }

        /// <summary>
        /// A chamfer of width c on both faces of the entry edge: the triangle between the edge, the point c out on the
        /// face and the point c down the inclined wall, revolved (Pappus).
        /// </summary>
        [TestMethod]
        public void TaperChamferedEntry()
        {
            MCPServer server = ServerWithTemplates();
            double r1 = 4, r2 = 2.5, c = 1, slope = (r2 - r1) / TaperDepth, w = Math.Sqrt(1 + slope * slope);
            (double x, double y) e = (r1, 0), f = (r1 + c, 0), g = (r1 + c * slope / w, -c / w);
            double area = Math.Abs((f.x - e.x) * (g.y - e.y) - (g.x - e.x) * (f.y - e.y)) / 2;
            double chamfer = 2 * Math.PI * (e.x + f.x + g.x) / 3 * area;
            Assert.AreEqual(Frustum(r1, r2, TaperDepth) + chamfer, Taper(server, "\"dEntry\":8,\"dEnd\":5,\"entry\":\"chamfer\""), 1e-3);
        }

        /// <summary>
        /// A fillet of radius rho at the entry: the cross section between the edge, the two tangent points and the
        /// arc, densely sampled and revolved (Pappus). The material at the edge of a narrowing hole has an angle above 90°.
        /// </summary>
        [TestMethod]
        public void TaperRoundedEntry()
        {
            foreach ((double d1, double d2) in new[] { (8.0, 5.0), (5.0, 8.0) })
            {
                MCPServer server = ServerWithTemplates();
                double r1 = d1 / 2, r2 = d2 / 2, rho = 1, slope = (r2 - r1) / TaperDepth, w = Math.Sqrt(1 + slope * slope);
                double phi = Math.Acos(slope / w), reach = rho / Math.Tan(phi / 2);
                (double x, double y) center = (r1 + reach, -rho);
                double startAngle = Math.PI / 2, sweep = Math.PI - phi; // from the tangent point on the face, counterclockwise
                List<(double x, double y)> polygon = new List<(double x, double y)> { (r1, 0) };
                const int n = 2000;
                for (int i = n; i >= 0; i--) // from the wall back to the face, so that the polygon runs clockwise like E, T, arc, S
                {
                    double a = startAngle + sweep * i / n;
                    polygon.Add((center.x + rho * Math.Cos(a), center.y + rho * Math.Sin(a)));
                }
                double area = 0, moment = 0; // moment: integral of x dA
                for (int i = 0; i < polygon.Count; i++)
                {
                    (double x0, double y0) = polygon[i];
                    (double x1, double y1) = polygon[(i + 1) % polygon.Count];
                    double cross = x0 * y1 - x1 * y0;
                    area += cross / 2;
                    moment += (x0 + x1) * cross / 6;
                }
                double fillet = 2 * Math.PI * Math.Abs(moment);
                Assert.AreEqual(Frustum(r1, r2, TaperDepth) + fillet,
                    Taper(server, FormattableString.Invariant($"\"dEntry\":{d1},\"dEnd\":{d2},\"entry\":\"round\",\"entrySize\":1")), 1e-3, $"{d1} -> {d2}");
            }
        }

        private const double SlotR = 3, SlotDepth = 5, SlotLength = 10; // d = 6, centers 10 apart

        private static double SlotArea => SlotLength * 2 * SlotR + Math.PI * SlotR * SlotR;

        private static string Slot(string arguments) => "{\"face\":\"top\",\"p1\":\"p(5,10,20)\",\"p2\":\"p(15,10,20)\",\"d\":6,\"depth\":5" + arguments + "}";

        [TestMethod]
        public void SlotOnAPlane()
        {
            MCPServer server = ServerWithTemplates();
            Assert.AreEqual(SlotArea * SlotDepth, RemovedVolume(server, Slot(""), "langloch"), 1e-3);
        }

        /// <summary>The chamfer along the two straight edges and around the two half circles, which make one ring.</summary>
        [TestMethod]
        public void SlotOnAPlaneChamfered()
        {
            MCPServer server = ServerWithTemplates();
            double c = 1;
            double chamfer = 2 * SlotLength * c * c / 2 + 2 * Math.PI * (SlotR + c / 3) * c * c / 2;
            Assert.AreEqual(SlotArea * SlotDepth + chamfer, RemovedVolume(server, Slot(",\"entry\":\"chamfer\""), "langloch"), 1e-3);
        }

        [TestMethod]
        public void SlotOnAPlaneRounded()
        {
            MCPServer server = ServerWithTemplates();
            double rho = 1, area = rho * rho * (1 - Math.PI / 4);
            double centroid = rho * (10 - 3 * Math.PI) / (12 - 3 * Math.PI);
            double fillet = 2 * SlotLength * area + 2 * Math.PI * (SlotR + centroid) * area;
            Assert.AreEqual(SlotArea * SlotDepth + fillet, RemovedVolume(server, Slot(",\"entry\":\"round\""), "langloch"), 1e-3);
        }

        /// <summary>A cylinder of radius 20 along z with the half of its mantle around (0, 20, z), for slots along the axis.</summary>
        private static MCPServer ServerWithShaft()
        {
            MCPServer server = ServerWithTemplates();
            Assert.IsNull(Error(Call(server, "solid.cylinder", "{\"name\":\"shaft\",\"start\":[0,0,0],\"end\":[0,0,60],\"radius\":20}")));
            // the mantle consists of two halves: the one around (0, 20, z)
            server.namedItems["mantle"] = FirstSolid(server.namedItems["shaft"]).Shells[0].Faces.Single(f => f.Surface is CylindricalSurface
                && f.GetBoundingCube().Xmin < -1 && f.GetBoundingCube().Xmax > 1 && f.GetBoundingCube().Ymax > 19);
            return server;
        }

        /// <summary>
        /// The slot along the axis of a shaft: below every point of the outline in the tangent plane the material from
        /// the bottom plane up to the mantle is removed, integrated numerically across the slot.
        /// </summary>
        [TestMethod]
        public void SlotAlongAShaft()
        {
            MCPServer server = ServerWithShaft();
            double R = 20, bottom = R - SlotDepth;
            Func<double, double> height = y => Math.Sqrt(R * R - y * y) - bottom;
            double straight = 0, ends = 0;
            const int n = 20000;
            for (int i = 0; i < n; i++)
            {
                double y = -SlotR + (i + 0.5) * 2 * SlotR / n, dy = 2 * SlotR / n;
                straight += 30 * height(y) * dy; // the centers lie 30 apart along the axis
                ends += 2 * Math.Sqrt(SlotR * SlotR - y * y) * height(y) * dy; // the two half circles make one disk
            }
            Assert.AreEqual(straight + ends,
                RemovedVolume(server, "{\"face\":\"mantle\",\"p1\":\"p(0,20,10)\",\"p2\":\"p(0,25,40)\",\"d\":6,\"depth\":5}", "langloch", "shaft", measureCutOut: true), 1e-2,
                "the second center lies off the face and is put onto it");
        }

        [TestMethod]
        public void SlotAlongAShaftChamferedAndRounded()
        {
            MCPServer server = ServerWithShaft();
            double sharp = RemovedVolume(server, "{\"face\":\"mantle\",\"p1\":\"p(0,20,10)\",\"p2\":\"p(0,20,40)\",\"d\":6,\"depth\":5}", "langloch", "shaft");
            foreach (string entry in new[] { "chamfer", "round" })
            {
                double blended = RemovedVolume(server, "{\"face\":\"mantle\",\"p1\":\"p(0,20,10)\",\"p2\":\"p(0,20,40)\",\"d\":6,\"depth\":5,\"entry\":\"" + entry + "\"}", "langloch", "shaft");
                // about the blend of a straight slot of the same length, a bit more since the mantle falls away
                double perLength = entry == "chamfer" ? 1.0 / 2 : 1 - Math.PI / 4;
                Assert.IsTrue(blended - sharp > 0.8 * 2 * 30 * perLength && blended - sharp < 2 * 2 * 30 * perLength, $"{entry}: {blended - sharp}");
            }
        }

        [TestMethod]
        public void SlotAcrossAShaftIsRefused()
        {
            MCPServer server = ServerWithShaft();
            string? error = Error(Call(server, "template.instantiate", "{\"template\":\"langloch\",\"arguments\":{\"face\":\"mantle\",\"p1\":\"p(0,20,10)\",\"p2\":\"p(14,14,40)\"},\"name\":\"x\"}"));
            Assert.IsNotNull(error);
            StringAssert.Contains(error, "same line along its axis");
        }

        /// <summary>
        /// A block with a slot across it: the axis leaves the solid at the slot and enters it again below. Only the
        /// part above the slot is drilled, the material below stays.
        /// </summary>
        [TestMethod]
        public void ThroughHoleStopsAtAGap()
        {
            MCPServer server = ServerWithTemplates();
            Assert.IsNull(Error(Call(server, "solid.box", "{\"name\":\"outer\",\"origin\":[0,0,0],\"sizeX\":40,\"sizeY\":20,\"sizeZ\":30}")));
            Assert.IsNull(Error(Call(server, "solid.box", "{\"name\":\"slot\",\"origin\":[10,-1,10],\"sizeX\":20,\"sizeY\":22,\"sizeZ\":10}")));
            Assert.IsNull(Error(Call(server, "solid.boolean", "{\"op\":\"difference\",\"a\":\"outer\",\"b\":\"slot\",\"name\":\"slotted\"}")));
            Solid slotted = FirstSolid(server.namedItems["slotted"]);
            server.namedItems["slottedTop"] = slotted.Shells[0].Faces.Single(f => Math.Abs(f.GetBoundingCube().Zmin - 30) < 1e-9 && Math.Abs(f.GetBoundingCube().Zmax - 30) < 1e-9);
            double removed = RemovedVolume(server, "{\"face\":\"slottedTop\",\"pos\":\"p(20,10,30)\",\"exit\":\"round\",\"exitSize\":1}", "durchgangsbohrung", "slotted");
            Assert.AreEqual(Math.PI * R * R * 10 + FilletRing(1), removed, 1e-2, "only down to the slot, with the rounded exit at its ceiling");
        }
    }
}
