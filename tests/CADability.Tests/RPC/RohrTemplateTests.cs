using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// The "rohradapter" template of RohrTemplate.json: two straight pipe stubs connected by a ruled transition, the
    /// pipes given by two circles in one plane, possibly eccentric. With the wall thickness measured in the planes of
    /// the circles, every section parallel to these planes is an annulus of known radii, so the volume follows from
    /// Cavalieri's principle, also for eccentric pipes. With the wall thickness measured perpendicular to the wall
    /// (concentric pipes only) the adapter is a solid of revolution, its volume is computed from the profile here.
    /// In both cases points just inside and just outside the wall are tested.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class RohrTemplateTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        private static string TemplateFile([System.Runtime.CompilerServices.CallerFilePath] string thisFile = "")
            => System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile)!, "..", "..", "..", "ShapeIt", "Templates", "RohrTemplate.json");

        /// <summary>Loads the templates of the template file the way the application does.</summary>
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

        /// <summary>A full circle, given to the template from the workspace: the curve parameters have no default.</summary>
        private static Ellipse Circle(GeoPoint center, double radius)
        {
            Ellipse circle = Ellipse.Construct();
            circle.SetCirclePlaneCenterRadius(new Plane(center, GeoVector.ZAxis), center, radius);
            return circle;
        }

        /// <summary>
        /// An adapter in the x/y plane: the first circle around the origin, the second around (x2, y2), both with the
        /// normal +z, so the adapter runs from z = -L1 to z = Lt + L2.
        /// </summary>
        private sealed record Adapter(double r1, double x2, double y2, double r2, string diameter, string wall, double L1, double Lt, double L2, double t)
        {
            public string Arguments => FormattableString.Invariant(
                $"{{\"c1\":\"k1\",\"c2\":\"k2\",\"diameter\":\"{diameter}\",\"wall\":\"{wall}\",\"L1\":{L1},\"Lt\":{Lt},\"L2\":{L2},\"t\":{t}}}");
            public bool Outer => diameter == "outer";
            public bool Perpendicular => wall == "perpendicular";
        }

        private static Solid Instantiate(Adapter a)
        {
            MCPServer server = ServerWithTemplates();
            server.namedItems["k1"] = Circle(GeoPoint.Origin, a.r1);
            server.namedItems["k2"] = Circle(new GeoPoint(a.x2, a.y2, 0), a.r2);
            string response = Call(server, "template.instantiate", $"{{\"template\":\"rohradapter\",\"arguments\":{a.Arguments},\"name\":\"adapter\"}}");
            Assert.IsNull(Error(response), response);
            object result = server.namedItems["adapter"];
            if (result is List<Solid> list)
            {   // a template result may come as a list
                Assert.AreEqual(1, list.Count, "the adapter is a single solid");
                result = list[0];
            }
            Assert.IsInstanceOfType(result, typeof(Solid), "the adapter is a single solid");
            return (Solid)result;
        }

        /// <summary>
        /// The wall along lines through it: in both stubs, and in the middle of the transition, along the normal of the
        /// oblique cone or in the plane of the section, depending on how the wall is measured. Each line starts on the
        /// surface given by the circles and runs into the wall, the shell must be hit there and at the wall thickness,
        /// and nowhere in between.
        /// </summary>
        private static void AssertWall(Solid solid, Adapter a)
        {
            // the circles the template got: the outside with outer diameters, the cavity with inner diameters.
            // 'wall' is the signed direction from these surfaces into the wall
            double wall = a.Outer ? -1 : 1;
            GeoPoint c1 = new GeoPoint(0, 0, 0), c2 = new GeoPoint(a.x2, a.y2, a.Lt);
            for (int i = 0; i < 8; i++)
            {
                double phi = (i + 0.3) * Math.PI / 4; // not on the seams of the circles, which are split at 0 and 180 degrees
                string at = FormattableString.Invariant($" at {(i + 0.3) * 45}°");
                GeoVector radial = new GeoVector(Math.Cos(phi), Math.Sin(phi), 0);
                AssertWallAlong(solid, c1 + a.r1 * radial + new GeoVector(0, 0, -a.L1 / 2), wall * radial, a.t, "first stub" + at);
                AssertWallAlong(solid, c2 + a.r2 * radial + new GeoVector(0, 0, a.L2 / 2), wall * radial, a.t, "second stub" + at);
                // the transition: S(phi, s) = (1-s)·P1(phi) + s·P2(phi), the normal points away from the axis
                GeoPoint p1 = c1 + a.r1 * radial, p2 = c2 + a.r2 * radial;
                GeoVector tangent = new GeoVector(-Math.Sin(phi), Math.Cos(phi), 0);
                GeoVector normal = ((0.5 * (a.r1 + a.r2)) * tangent ^ (p2 - p1)).Normalized;
                if (normal * radial < 0) normal = -normal;
                if (!a.Perpendicular) normal = radial; // the section at half the transition length
                AssertWallAlong(solid, new GeoPoint(p1, p2), wall * normal, a.t, "transition" + at);
            }
            BoundingBox ext = solid.GetExtent(0.0);
            Assert.AreEqual(-a.L1, ext.Zmin, 1e-6, "open end of the first stub");
            Assert.AreEqual(a.Lt + a.L2, ext.Zmax, 1e-6, "open end of the second stub");
        }

        /// <summary>
        /// The line from <paramref name="start"/> in the unit direction <paramref name="into"/> hits the shell at the
        /// start and at <paramref name="thickness"/>, and nowhere in between.
        /// </summary>
        private static void AssertWallAlong(Solid solid, GeoPoint start, GeoVector into, double thickness, string what)
        {
            double tol = 1e-6 * thickness;
            List<double> hits = [.. solid.GetLineIntersection(start, into).Select(p => Geometry.LinePar(start, into, p))];
            Assert.IsTrue(hits.Any(h => Math.Abs(h) < tol), $"{what}: the surface of the circles, hits {string.Join(", ", hits)}");
            Assert.IsTrue(hits.Any(h => Math.Abs(h - thickness) < tol), $"{what}: the other side of the wall, hits {string.Join(", ", hits)}");
            Assert.IsFalse(hits.Any(h => h > tol && h < thickness - tol), $"{what}: nothing inside the wall, hits {string.Join(", ", hits)}");
        }

        /// <summary>
        /// The volume of a concentric adapter: π·∫(ro² - ri²)dz over the length, where the radius of the given side
        /// follows the profile and the other side is the set of points at distance t from it (the wall measured
        /// perpendicular, with a rounding of radius t where the wall bends towards the other side).
        /// </summary>
        private static double ConcentricVolume(Adapter a)
        {
            double e = a.Outer ? 0 : 2 * a.t; // the cavity runs beyond the ends, so its offset is not rounded there
            (double r, double z)[] profile = [(a.r1, -a.L1 - e), (a.r1, 0), (a.r2, a.Lt), (a.r2, a.Lt + a.L2 + e)];
            double Given(double z) => z <= 0 ? a.r1 : z >= a.Lt ? a.r2 : a.r1 + (a.r2 - a.r1) * z / a.Lt;
            double Distance(double r, double z)
            {
                double min = double.MaxValue;
                for (int i = 0; i < profile.Length - 1; i++)
                {
                    double dr = profile[i + 1].r - profile[i].r, dz = profile[i + 1].z - profile[i].z;
                    double u = Math.Clamp(((r - profile[i].r) * dr + (z - profile[i].z) * dz) / (dr * dr + dz * dz), 0, 1);
                    min = Math.Min(min, Math.Sqrt(Math.Pow(r - profile[i].r - u * dr, 2) + Math.Pow(z - profile[i].z - u * dz, 2)));
                }
                return min;
            }
            double Other(double z)
            {   // bisection between the given radius and t beyond it for the radius at distance t
                double g = Given(z), sign = a.Outer ? -1 : 1, lo = 0, hi = 1.5 * a.t;
                for (int k = 0; k < 60; k++)
                {
                    double m = (lo + hi) / 2;
                    if (Distance(g + sign * m, z) < a.t) lo = m; else hi = m;
                }
                return g + sign * (lo + hi) / 2;
            }
            int n = 200000;
            double z0 = -a.L1, z1 = a.Lt + a.L2, h = (z1 - z0) / n, sum = 0;
            for (int i = 0; i < n; i++)
            {
                double z = z0 + (i + 0.5) * h, g = Given(z), o = Other(z);
                sum += Math.Abs(g * g - o * o);
            }
            return Math.PI * sum * h;
        }

        /// <summary>
        /// The volume with the wall measured in the planes of the circles: each section is an annulus, the
        /// transition is the difference of two oblique cone frustums, which have the volume of the right ones.
        /// </summary>
        private static double PlanesVolume(Adapter a)
        {
            double o1 = a.Outer ? a.r1 : a.r1 + a.t, o2 = a.Outer ? a.r2 : a.r2 + a.t;
            double c1 = o1 - a.t, c2 = o2 - a.t;
            double stubs = Math.PI * ((o1 * o1 - c1 * c1) * a.L1 + (o2 * o2 - c2 * c2) * a.L2);
            double transition = Math.PI * a.Lt / 3 * ((o1 * o1 + o1 * o2 + o2 * o2) - (c1 * c1 + c1 * c2 + c2 * c2));
            return stubs + transition;
        }

        private static void AssertAdapter(Adapter a)
        {
            Solid solid = Instantiate(a);
            AssertWall(solid, a);
            double expected = a.Perpendicular ? ConcentricVolume(a) : PlanesVolume(a);
            Assert.AreEqual(expected, solid.Volume(1e-4), 1e-4 * expected, $"volume of {a}");
        }

        [TestMethod]
        public void PerpendicularReducerOuterDiameters()
        {
            AssertAdapter(new Adapter(25, 0, 0, 15, "outer", "perpendicular", 30, 40, 30, 2));
        }

        [TestMethod]
        public void PerpendicularExpanderOuterDiameters()
        {
            AssertAdapter(new Adapter(15, 0, 0, 25, "outer", "perpendicular", 20, 30, 25, 3));
        }

        [TestMethod]
        public void PerpendicularReducerInnerDiameters()
        {
            AssertAdapter(new Adapter(25, 0, 0, 15, "inner", "perpendicular", 30, 40, 30, 2));
        }

        [TestMethod]
        public void PlanesConcentricReducer()
        {
            AssertAdapter(new Adapter(25, 0, 0, 15, "outer", "planes", 30, 40, 30, 2));
        }

        [TestMethod]
        public void PlanesEccentricOuterDiameters()
        {
            AssertAdapter(new Adapter(25, 10, 0, 15, "outer", "planes", 30, 40, 30, 2));
        }

        [TestMethod]
        public void PlanesEccentricInnerDiameters()
        {
            AssertAdapter(new Adapter(20, 6, -8, 12, "inner", "planes", 25, 35, 20, 2.5));
        }

        [TestMethod]
        public void PlanesEccentricExpander()
        {
            AssertAdapter(new Adapter(12, -5, 7, 22, "outer", "planes", 15, 25, 30, 1.5));
        }

        [TestMethod]
        public void CirclesAreRequired()
        {
            MCPServer server = ServerWithTemplates();
            server.namedItems["k1"] = Circle(GeoPoint.Origin, 25);
            string response = Call(server, "template.instantiate", "{\"template\":\"rohradapter\",\"arguments\":{\"c1\":\"k1\"},\"name\":\"adapter\"}");
            StringAssert.Contains(Error(response) ?? "", "needs the curve parameter 'c2'");
        }

        [TestMethod]
        public void OuterWorkspaceItemDoesNotStandInForAMissingCircle()
        {
            MCPServer server = ServerWithTemplates();
            server.namedItems["c1"] = Circle(GeoPoint.Origin, 25); // same name as the parameter, but not passed
            server.namedItems["k2"] = Circle(GeoPoint.Origin, 15);
            string response = Call(server, "template.instantiate", "{\"template\":\"rohradapter\",\"arguments\":{\"c2\":\"k2\"},\"name\":\"adapter\"}");
            StringAssert.Contains(Error(response) ?? "", "needs the curve parameter 'c1'");
        }

        private static string Rejected(string arguments)
        {
            MCPServer server = ServerWithTemplates();
            server.namedItems["k1"] = Circle(GeoPoint.Origin, 25);
            server.namedItems["k2"] = Circle(new GeoPoint(10, 0, 0), 15);
            server.namedItems["k3"] = Circle(new GeoPoint(10, 0, 5), 15);
            return Error(Call(server, "template.instantiate", "{\"template\":\"rohradapter\",\"arguments\":" + arguments + ",\"name\":\"adapter\"}")) ?? "";
        }

        [TestMethod]
        public void CirclesInDifferentPlanesAreRejected()
        {
            StringAssert.Contains(Rejected("{\"c1\":\"k1\",\"c2\":\"k3\"}"), "one plane");
        }

        [TestMethod]
        public void PerpendicularEccentricIsRejected()
        {
            StringAssert.Contains(Rejected("{\"c1\":\"k1\",\"c2\":\"k2\",\"wall\":\"perpendicular\"}"), "concentric");
        }

        [TestMethod]
        public void OuterWallThickerThanRadiusIsRejected()
        {
            StringAssert.Contains(Rejected("{\"c1\":\"k1\",\"c2\":\"k2\",\"t\":16}"), "wall thickness");
        }
    }
}
