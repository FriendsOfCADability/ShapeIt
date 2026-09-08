namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="PolygonRegion"/>, which splits polygons that intersect themselves or
    /// each other into non intersecting outline/hole groups. The reference is the winding number:
    /// a point belongs to the region described by the input when its winding number is 1 or more,
    /// and the result has to cover exactly those points, exactly once.
    /// </summary>
    [TestClass]
    public class PolygonRegionTests
    {
        /// <summary>counterclockwise (ccw = false: clockwise) regular polygon</summary>
        private static GeoPoint2D[] Circle(double r, int n, double startAngle = 0.0, double cx = 0.0, double cy = 0.0, bool ccw = true)
        {
            GeoPoint2D[] res = new GeoPoint2D[n];
            for (int i = 0; i < n; ++i)
            {
                double a = startAngle + 2.0 * Math.PI * (ccw ? i : n - i) / n;
                res[i] = new GeoPoint2D(cx + r * Math.Cos(a), cy + r * Math.Sin(a));
            }
            return res;
        }

        private static int Winding(GeoPoint2D p, params GeoPoint2D[][] loops)
        {
            int w = 0;
            foreach (GeoPoint2D[] lp in loops)
            {
                for (int i = 0; i < lp.Length; ++i)
                {
                    GeoPoint2D a = lp[i], b = lp[(i + 1) % lp.Length];
                    if (a.y <= p.y)
                    {
                        if (b.y > p.y && (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y) > 0) ++w;
                    }
                    else
                    {
                        if (b.y <= p.y && (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y) < 0) --w;
                    }
                }
            }
            return w;
        }

        private static double DistanceTo(GeoPoint2D p, IEnumerable<GeoPoint2D[]> loops)
        {
            double best = double.MaxValue;
            foreach (GeoPoint2D[] lp in loops)
            {
                for (int i = 0; i < lp.Length; ++i)
                {
                    GeoPoint2D a = lp[i], b = lp[(i + 1) % lp.Length];
                    double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
                    double t = l2 == 0.0 ? 0.0 : Math.Max(0.0, Math.Min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2));
                    best = Math.Min(best, Math.Sqrt((p.x - a.x - t * dx) * (p.x - a.x - t * dx) + (p.y - a.y - t * dy) * (p.y - a.y - t * dy)));
                }
            }
            return best;
        }

        /// <summary>number of proper crossings of a closed polygon with itself</summary>
        private static int SelfIntersections(GeoPoint2D[] poly)
        {
            int count = 0;
            for (int i = 0; i < poly.Length; ++i)
            {
                GeoPoint2D a = poly[i], b = poly[(i + 1) % poly.Length];
                for (int j = i + 1; j < poly.Length; ++j)
                {
                    if ((j + 1) % poly.Length == i || (i + 1) % poly.Length == j) continue;
                    GeoPoint2D c = poly[j], d = poly[(j + 1) % poly.Length];
                    double rx = b.x - a.x, ry = b.y - a.y, sx = d.x - c.x, sy = d.y - c.y;
                    double den = rx * sy - ry * sx;
                    if (den == 0.0) continue;
                    double t = ((c.x - a.x) * sy - (c.y - a.y) * sx) / den;
                    double u = ((c.x - a.x) * ry - (c.y - a.y) * rx) / den;
                    if (t > 1e-12 && t < 1.0 - 1e-12 && u > 1e-12 && u < 1.0 - 1e-12) ++count;
                }
            }
            return count;
        }

        /// <summary>
        /// The result must consist of counterclockwise outlines with clockwise holes, none of the
        /// loops may intersect itself, and the covered area must be exactly the set of points with
        /// a winding number of 1 or more with respect to the input - covered once, not twice.
        /// </summary>
        private static GeoPoint2D[][][] AssertSameRegion(string what, GeoPoint2D[][] input, double minArea = 1e-9, int samples = 3000)
        {
            GeoPoint2D[][][] res = PolygonRegion.Subdivide(input, minArea);
            List<GeoPoint2D[]> all = new List<GeoPoint2D[]>();
            foreach (GeoPoint2D[][] group in res)
            {
                Assert.IsTrue(GeoPoint2D.Area(group[0]) > 0.0, what + ": the outline of a group is not counterclockwise");
                for (int i = 1; i < group.Length; ++i)
                    Assert.IsTrue(GeoPoint2D.Area(group[i]) < 0.0, what + ": a hole is not clockwise");
                foreach (GeoPoint2D[] loop in group)
                {
                    Assert.AreEqual(0, SelfIntersections(loop), what + ": a resulting loop intersects itself");
                    all.Add(loop);
                }
            }
            BoundingRect ext = BoundingRect.EmptyBoundingRect;
            foreach (GeoPoint2D[] lp in input) foreach (GeoPoint2D p in lp) ext.MinMax(p);
            Random rnd = new Random(4711);
            int tested = 0;
            for (int k = 0; k < 200 * samples && tested < samples; ++k)
            {
                GeoPoint2D p = new GeoPoint2D(ext.Left + rnd.NextDouble() * ext.Width, ext.Bottom + rnd.NextDouble() * ext.Height);
                // points on one of the boundaries cannot be classified, skip them
                double eps = ext.Size * 1e-7;
                if (DistanceTo(p, input) < eps) continue;
                if (all.Count > 0 && DistanceTo(p, all) < eps) continue;
                ++tested;
                bool inside = Winding(p, input) >= 1;
                int covered = 0;
                foreach (GeoPoint2D[][] group in res)
                {
                    if (Winding(p, group[0]) == 0) continue;
                    bool inHole = false;
                    for (int i = 1; i < group.Length && !inHole; ++i) inHole = Winding(p, group[i]) != 0;
                    if (!inHole) ++covered;
                }
                Assert.AreEqual(inside ? 1 : 0, covered,
                    what + ": point (" + p.x.ToString("F6") + ", " + p.y.ToString("F6") + ") is covered " + covered + " times");
            }
            Assert.IsTrue(tested > samples / 2, what + ": too few points could be classified");
            return res;
        }

        [TestMethod]
        public void plain_outline_stays_as_it_is()
        {
            GeoPoint2D[][][] res = AssertSameRegion("circle", new[] { Circle(10, 24) });
            Assert.AreEqual(1, res.Length);
            Assert.AreEqual(1, res[0].Length);
            Assert.AreEqual(GeoPoint2D.Area(Circle(10, 24)), GeoPoint2D.Area(res[0][0]), 1e-9);
        }

        [TestMethod]
        public void holes_that_touch_nothing_are_kept()
        {
            GeoPoint2D[][][] res = AssertSameRegion("two holes",
                new[] { Circle(10, 24), Circle(3, 12, 0.0, 4, 4, false), Circle(2, 11, 0.2, -4, -3, false) });
            Assert.AreEqual(1, res.Length, "the holes do not touch the outline, so there is one group");
            Assert.AreEqual(3, res[0].Length, "the outline and both holes");
        }

        /// <summary>a hole inside a hole is filled again, it is part of the face</summary>
        [TestMethod]
        public void island_inside_a_hole()
        {
            GeoPoint2D[][][] res = AssertSameRegion("island",
                new[] { Circle(10, 24), Circle(6, 16, 0.0, 0, 0, false), Circle(3, 12, 0.1) });
            Assert.AreEqual(2, res.Length, "the ring and the island are separate");
        }

        [TestMethod]
        public void hole_crossing_the_outline()
        {
            AssertSameRegion("crossing hole", new[] { Circle(10, 24), Circle(4, 13, 0.1, 8, 0, false) });
        }

        [TestMethod]
        public void hole_covering_the_whole_outline_leaves_nothing()
        {
            GeoPoint2D[][][] res = PolygonRegion.Subdivide(new[] { Circle(10, 24), Circle(14, 20, 0.1, 0, 0, false) }, 1e-9);
            Assert.AreEqual(0, res.Length);
        }

        /// <summary>
        /// A ring so thin that the chords of the outer polygon reach inside the inner one: the two
        /// loops cross each other twice per pair of vertices. This is the situation
        /// Face.Triangulate runs into with a thin cylindrical face.
        /// </summary>
        [TestMethod]
        public void thin_ring_falls_apart_into_pieces()
        {
            foreach (double innerRadius in new[] { 9.99, 9.999 })
            {
                GeoPoint2D[][][] res = AssertSameRegion("thin ring " + innerRadius,
                    new[] { Circle(10, 32), Circle(innerRadius, 33, 0.3, 0, 0, false) });
                Assert.IsTrue(res.Length > 1, "the ring has to fall apart into several pieces");
                double sum = 0.0;
                foreach (GeoPoint2D[][] group in res) foreach (GeoPoint2D[] loop in group) sum += GeoPoint2D.Area(loop);
                Assert.IsTrue(sum < 0.05 * Math.PI * 100.0, "the hole of the ring was filled: " + sum.ToString("F6"));
            }
        }

        /// <summary>the two halves of a bowtie run in opposite directions, only the ccw one counts</summary>
        [TestMethod]
        public void bowtie_keeps_only_the_counterclockwise_half()
        {
            GeoPoint2D[][][] res = AssertSameRegion("bowtie", new[]
            {
                new[] { new GeoPoint2D(0, 0), new GeoPoint2D(10, 10), new GeoPoint2D(10, 0), new GeoPoint2D(0, 10) }
            });
            Assert.AreEqual(1, res.Length);
            Assert.AreEqual(25.0, GeoPoint2D.Area(res[0][0]), 1e-9);
        }

        /// <summary>a zero width slit does not cut the face apart, the two sides cancel each other</summary>
        [TestMethod]
        public void slit_is_closed_again()
        {
            GeoPoint2D[][][] res = AssertSameRegion("slit", new[]
            {
                new[] { new GeoPoint2D(0, 0), new GeoPoint2D(10, 0), new GeoPoint2D(10, 10), new GeoPoint2D(5, 10),
                        new GeoPoint2D(5, 4), new GeoPoint2D(5, 10), new GeoPoint2D(0, 10) }
            });
            Assert.AreEqual(1, res.Length);
            Assert.AreEqual(1, res[0].Length);
            Assert.AreEqual(100.0, GeoPoint2D.Area(res[0][0]), 1e-9);
        }

        /// <summary>an outline that overlaps itself is not counted twice</summary>
        [TestMethod]
        public void overlapping_outline_is_covered_once()
        {
            AssertSameRegion("self overlapping", new[]
            {
                new[] { new GeoPoint2D(0, 0), new GeoPoint2D(10, 0), new GeoPoint2D(10, 10), new GeoPoint2D(0, 10),
                        new GeoPoint2D(0, 2), new GeoPoint2D(8, 2), new GeoPoint2D(8, 8), new GeoPoint2D(2, 8),
                        new GeoPoint2D(2, 4), new GeoPoint2D(12, 4), new GeoPoint2D(12, -2), new GeoPoint2D(0, -2) }
            });
        }

        [TestMethod]
        public void two_separate_outlines_with_a_hole()
        {
            GeoPoint2D[][][] res = AssertSameRegion("two areas",
                new[] { Circle(5, 16), Circle(2, 12, 0.1, 0, 0, false), Circle(5, 16, 0.1, 0, 30) });
            Assert.AreEqual(2, res.Length);
        }

        /// <summary>
        /// The situation of TriangulationBug1.cdb.json boiled down: a circle and a spiral running
        /// tangentially along it from the inside, so that the spiral pokes through the chords of
        /// the circle's polygon in a few places.
        /// </summary>
        [TestMethod]
        public void spiral_touching_a_circle_tangentially()
        {
            List<GeoPoint2D> outline = new List<GeoPoint2D>();
            for (int i = 0; i <= 40; ++i)
            {   // spiral from radius 8 to radius 12, ending where the circle starts
                double a = Math.PI / 2.0 + i * Math.PI / 40.0;
                double r = 8.0 + 4.0 * i / 40.0;
                outline.Add(new GeoPoint2D(r * Math.Cos(a), r * Math.Sin(a)));
            }
            outline.RemoveAt(outline.Count - 1);
            for (int i = 0; i < 26; ++i)
            {
                double a = 3.0 * Math.PI / 2.0 + i * 2.0 * Math.PI / 26.0;
                outline.Add(new GeoPoint2D(12.0 * Math.Cos(a), 12.0 * Math.Sin(a)));
            }
            AssertSameRegion("spiral", new[] { outline.ToArray() });
        }
    }
}
