using System.IO;
using System.Text.Json;
using System.Text.Json.Nodes;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// The offset of a ruled surface is an <see cref="OffsetSurface"/>, whose iso curves are OffsetSurface.FixedCurve.
    /// An oblique cone, ruled between two circles with different axes, is the simplest body which needs it.
    /// </summary>
    [TestClass]
    public class OffsetRuledSurfaceTests
    {
        private static string Call(MCPServer server, string method, string parameters)
        {
            string json = $"{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"{method}\",\"params\":{parameters}}}";
            using JsonDocument document = JsonDocument.Parse(json);
            return server.ProcessMethod(document.RootElement);
        }

        private static string? Error(string response) => JsonNode.Parse(response)?["error"]?["message"]?.GetValue<string>();

        private static void Ok(MCPServer server, string method, string parameters)
        {
            string response = Call(server, method, parameters);
            Assert.IsNull(Error(response), $"{method}: {response}");
        }

        private static Solid Single(MCPServer server, string name)
        {
            object result = server.namedItems[name];
            Solid? solid = result as Solid ?? (result as System.Collections.Generic.List<Solid>)?.FirstOrDefault();
            Assert.IsNotNull(solid, $"'{name}' holds no solid");
            return solid;
        }

        private static MCPServer ObliqueCone()
        {
            MCPServer server = new MCPServer(null!, Project.CreateSimpleProject()) { SuppressDialogs = true };
            Ok(server, "sketch.create", "{\"name\":\"bottom\",\"plane\":{\"origin\":[0,0,0],\"xAxis\":[1,0,0],\"normal\":[0,0,1]}}");
            Ok(server, "sketch.create", "{\"name\":\"top\",\"plane\":{\"origin\":[0,0,20],\"xAxis\":[1,0,0],\"normal\":[0,0,1]}}");
            Ok(server, "sketch.add_circle", "{\"sketch\":\"bottom\",\"center\":[0,0],\"radius\":10}");
            Ok(server, "sketch.add_circle", "{\"sketch\":\"top\",\"center\":[6,0],\"radius\":4}");
            Ok(server, "solid.ruled", "{\"profile1\":\"bottom\",\"profile2\":\"top\",\"name\":\"cone\"}");
            return server;
        }

        /// <summary>
        /// The iso curves of the offset surface: DirectionAt must be the derivative of PointAt, also after Reverse,
        /// Split and Trim, and the pieces must stay on the offset surface (they used to be pieces of the base curve).
        /// </summary>
        [TestMethod]
        public void FixedCurveIsConsistent()
        {
            Solid cone = Single(ObliqueCone(), "cone");
            Face lateral = cone.Shells[0].Faces.First(f => f.Surface is RuledSurface);
            OffsetSurface os = new OffsetSurface(lateral.Surface, 1.0);
            BoundingRect d = lateral.Domain;
            const double h = 1e-6;
            void CheckDerivative(ICurve c)
            {
                for (int i = 0; i <= 10; i++)
                {
                    double t = Math.Clamp(i / 10.0, h, 1 - h);
                    GeoVector numeric = (c.PointAt(t + h) - c.PointAt(t - h)) / (2 * h);
                    Assert.AreEqual(0.0, (numeric - c.DirectionAt(t)).Length, 1e-6 * numeric.Length, $"DirectionAt({t})");
                }
            }
            void CheckOnSurface(ICurve c)
            {
                for (int i = 0; i <= 10; i++)
                {
                    GeoPoint p = c.PointAt(i / 10.0);
                    Assert.AreEqual(0.0, p | os.PointAt(os.PositionOf(p)), 1e-8);
                }
            }
            ICurve[] curves = {
                os.FixedU(d.Left + 0.3 * d.Width, d.Bottom, d.Top),
                os.FixedV(d.Bottom, d.Left, d.Right),
                os.FixedV(d.Top, d.Right, d.Left) }; // the reversed one
            foreach (ICurve c in curves)
            {
                CheckDerivative(c);
                GeoPoint sp = c.StartPoint, ep = c.EndPoint;
                ICurve r = c.Clone();
                r.Reverse();
                Assert.AreEqual(0.0, r.StartPoint | ep, 1e-10);
                Assert.AreEqual(0.0, r.EndPoint | sp, 1e-10);
                CheckDerivative(r);
                ICurve[] parts = c.Split(0.4);
                Assert.AreEqual(2, parts.Length);
                Assert.AreEqual(0.0, parts[0].EndPoint | c.PointAt(0.4), 1e-10);
                Assert.AreEqual(0.0, parts[1].StartPoint | c.PointAt(0.4), 1e-10);
                Assert.AreEqual(c.Length, parts[0].Length + parts[1].Length, 1e-8 * c.Length);
                foreach (ICurve part in parts) { CheckDerivative(part); CheckOnSurface(part); }
                ICurve tr = c.Clone();
                tr.Trim(0.2, 0.7);
                Assert.AreEqual(0.0, tr.StartPoint | c.PointAt(0.2), 1e-10);
                Assert.AreEqual(0.0, tr.EndPoint | c.PointAt(0.7), 1e-10);
                CheckDerivative(tr);
                CheckOnSurface(tr);
            }
        }

        /// <summary>
        /// The offset body, whose edges are iso curves of the offset surfaces, survives a JSON round trip.
        /// </summary>
        [TestMethod]
        public void OffsetBodyJsonRoundTrip()
        {
            MCPServer server = ObliqueCone();
            Ok(server, "solid.offset", "{\"solid\":\"cone\",\"distance\":1,\"name\":\"grown\"}");
            Solid grown = Single(server, "grown");
            Assert.IsTrue(grown.Shells[0].Edges.Any(e => e.Curve3D?.GetType().Name == "FixedCurve"), "the case should have an iso curve of an offset surface as an edge");
            Solid read;
            using (MemoryStream stream = new MemoryStream())
            {
                new JsonSerialize().ToStream(stream, grown, false);
                stream.Position = 0;
                read = new JsonSerialize().FromStream(stream) as Solid;
            }
            Assert.IsNotNull(read);
            Assert.AreEqual(grown.Volume(1e-4), read.Volume(1e-4), 1e-6);
            Edge[] e1 = grown.Shells[0].Edges, e2 = read.Shells[0].Edges;
            Assert.AreEqual(e1.Length, e2.Length);
            for (int i = 0; i < e1.Length; i++)
            {
                Assert.AreEqual(e1[i].Curve3D.GetType(), e2[i].Curve3D.GetType());
                for (int j = 0; j <= 4; j++) Assert.AreEqual(0.0, e1[i].Curve3D.PointAt(j / 4.0) | e2[i].Curve3D.PointAt(j / 4.0), 1e-10);
            }
        }

        [TestMethod]
        public void ObliqueConeOffset()
        {
            MCPServer server = ObliqueCone();
            Solid cone = Single(server, "cone");
            double v0 = cone.Volume(1e-4);
            foreach (double d in new double[] { 1, -1 })
            {
                string name = d > 0 ? "grown" : "shrunk";
                Ok(server, "solid.offset", FormattableString.Invariant($"{{\"solid\":\"cone\",\"distance\":{d},\"name\":\"{name}\"}}"));
                Solid res = Single(server, name);
                double v = res.Volume(1e-4);
                Console.WriteLine($"{name}: faces {res.Shells[0].Faces.Length}, volume {v} (cone {v0}), open edges {res.Shells[0].OpenEdges.Length}");
                foreach (Face f in res.Shells[0].Faces) Console.WriteLine("  " + f.Surface.GetType().Name);
                Assert.AreEqual(0, res.Shells[0].OpenEdges.Length);
                if (d > 0) Assert.IsTrue(v > v0);
                else Assert.IsTrue(v < v0);
                // every point of the offset body is at the distance |d| from the cone
                double maxErr = 0.0;
                foreach (Face f in res.Shells[0].Faces)
                {
                    foreach (Edge e in f.AllEdges)
                    {
                        for (int i = 0; i <= 10; i++) maxErr = Math.Max(maxErr, Math.Abs(Math.Abs(cone.Shells[0].Distance(e.Curve3D.PointAt(i / 10.0))) - Math.Abs(d)));
                    }
                    BoundingRect dom = f.Domain;
                    for (int i = 1; i < 10; i++) for (int j = 1; j < 10; j++)
                        {
                            GeoPoint2D uv = new GeoPoint2D(dom.Left + i / 10.0 * dom.Width, dom.Bottom + j / 10.0 * dom.Height);
                            if (!f.Contains(ref uv, true)) continue;
                            maxErr = Math.Max(maxErr, Math.Abs(Math.Abs(cone.Shells[0].Distance(f.Surface.PointAt(uv))) - Math.Abs(d)));
                        }
                }
                Console.WriteLine($"  max distance error {maxErr}");
                Assert.AreEqual(0.0, maxErr, 1e-6);
            }
        }
    }
}
