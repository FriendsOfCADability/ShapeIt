using CADability.GeoObject;

namespace CADability.Tests
{
    // Tests for the static bounding volume hierarchy (BVHTree / BVH<T>): every query type is
    // verified against a brute force reference implementation on random input, plus the intended
    // first real use case, a BVH over the triangles of a face triangulation.
    [TestClass]
    public class BVHTests
    {
        public TestContext TestContext { get; set; }

        private static BoundingBox RandomBox(Random rnd, double range, double maxSize)
        {
            double cx = (rnd.NextDouble() - 0.5) * 2 * range;
            double cy = (rnd.NextDouble() - 0.5) * 2 * range;
            double cz = (rnd.NextDouble() - 0.5) * 2 * range;
            double sx = rnd.NextDouble() * maxSize;
            double sy = rnd.NextDouble() * maxSize;
            double sz = rnd.NextDouble() * maxSize;
            return new BoundingBox(new GeoPoint(cx - sx, cy - sy, cz - sz), new GeoPoint(cx + sx, cy + sy, cz + sz));
        }

        private static BoundingBox[] RandomBoxes(int seed, int count, double range = 50.0, double maxSize = 5.0)
        {
            Random rnd = new Random(seed);
            BoundingBox[] boxes = new BoundingBox[count];
            for (int i = 0; i < count; i++) boxes[i] = RandomBox(rnd, range, maxSize);
            return boxes;
        }

        [TestMethod]
        public void overlapping_box_query_matches_brute_force()
        {
            BoundingBox[] boxes = RandomBoxes(seed: 4711, count: 500);
            BVHTree tree = new BVHTree(boxes);
            Assert.AreEqual(boxes.Length, tree.Count);

            Random rnd = new Random(815);
            for (int q = 0; q < 25; q++)
            {
                BoundingBox query = RandomBox(rnd, 50.0, 15.0);
                var expected = new HashSet<int>();
                for (int i = 0; i < boxes.Length; i++)
                    if (boxes[i].Interferes(query)) expected.Add(i);
                var actual = new HashSet<int>(tree.Overlapping(query));
                Assert.IsTrue(expected.SetEquals(actual),
                    $"query {q}: expected {expected.Count} hits, got {actual.Count}");
            }
        }

        [TestMethod]
        public void traverse_with_ray_predicate_matches_brute_force()
        {
            BoundingBox[] boxes = RandomBoxes(seed: 42, count: 400);
            BVHTree tree = new BVHTree(boxes);

            Random rnd = new Random(43);
            for (int q = 0; q < 25; q++)
            {
                GeoPoint start = new GeoPoint((rnd.NextDouble() - 0.5) * 100, (rnd.NextDouble() - 0.5) * 100, (rnd.NextDouble() - 0.5) * 100);
                GeoVector dir = new GeoVector(rnd.NextDouble() - 0.5, rnd.NextDouble() - 0.5, rnd.NextDouble() - 0.5);
                if (dir.Length < 1e-6) continue;

                bool RayHits(BoundingBox b) => b.Interferes(start, dir, double.MaxValue, false);

                var expected = new HashSet<int>();
                for (int i = 0; i < boxes.Length; i++)
                    if (RayHits(boxes[i])) expected.Add(i);
                var actual = new HashSet<int>(tree.Traverse(RayHits));
                Assert.IsTrue(expected.SetEquals(actual),
                    $"ray {q}: expected {expected.Count} hits, got {actual.Count}");
            }
        }

        [TestMethod]
        public void dual_traversal_matches_brute_force()
        {
            // two overlapping clouds of boxes, so there is a substantial number of candidate pairs
            BoundingBox[] boxesA = RandomBoxes(seed: 1, count: 300, range: 40.0, maxSize: 4.0);
            BoundingBox[] boxesB = RandomBoxes(seed: 2, count: 350, range: 40.0, maxSize: 4.0);
            BVHTree treeA = new BVHTree(boxesA);
            BVHTree treeB = new BVHTree(boxesB);

            var expected = new HashSet<(int, int)>();
            for (int i = 0; i < boxesA.Length; i++)
                for (int j = 0; j < boxesB.Length; j++)
                    if (boxesA[i].Interferes(boxesB[j])) expected.Add((i, j));

            var actual = new List<(int, int)>(treeA.Overlapping(treeB));
            Assert.AreEqual(actual.Count, new HashSet<(int, int)>(actual).Count, "pairs must be reported exactly once");
            Assert.IsTrue(expected.SetEquals(actual),
                $"expected {expected.Count} pairs, got {actual.Count}");
            Assert.IsTrue(expected.Count > 0, "test setup should produce at least some overlapping pairs");
        }

        [TestMethod]
        public void self_overlapping_pairs_match_brute_force()
        {
            BoundingBox[] boxes = RandomBoxes(seed: 7, count: 300, range: 30.0, maxSize: 4.0);
            BVHTree tree = new BVHTree(boxes);

            var expected = new HashSet<(int, int)>();
            for (int i = 0; i < boxes.Length; i++)
                for (int j = i + 1; j < boxes.Length; j++)
                    if (boxes[i].Interferes(boxes[j])) expected.Add((i, j));

            var actual = new List<(int, int)>(tree.SelfOverlappingPairs());
            foreach ((int i, int j) in actual) Assert.IsTrue(i < j, "pairs must be ordered (i < j)");
            Assert.AreEqual(actual.Count, new HashSet<(int, int)>(actual).Count, "pairs must be reported exactly once");
            Assert.IsTrue(expected.SetEquals(actual),
                $"expected {expected.Count} pairs, got {actual.Count}");
            Assert.IsTrue(expected.Count > 0, "test setup should produce at least some overlapping pairs");
        }

        [TestMethod]
        public void median_split_strategy_matches_brute_force()
        {
            // the default build strategy is BinnedSAH (covered by the tests above); this keeps
            // the MedianSplit strategy (which is also the SAH fallback path) covered as well
            BoundingBox[] boxes = RandomBoxes(seed: 4711, count: 500);
            BVHTree tree = new BVHTree(boxes, BVHTree.DefaultLeafSize, BVHBuildStrategy.MedianSplit);

            Random rnd = new Random(815);
            for (int q = 0; q < 10; q++)
            {
                BoundingBox query = RandomBox(rnd, 50.0, 15.0);
                var expected = new HashSet<int>();
                for (int i = 0; i < boxes.Length; i++)
                    if (boxes[i].Interferes(query)) expected.Add(i);
                Assert.IsTrue(expected.SetEquals(new HashSet<int>(tree.Overlapping(query))), $"query {q}");
            }

            var expectedPairs = new HashSet<(int, int)>();
            for (int i = 0; i < boxes.Length; i++)
                for (int j = i + 1; j < boxes.Length; j++)
                    if (boxes[i].Interferes(boxes[j])) expectedPairs.Add((i, j));
            Assert.IsTrue(expectedPairs.SetEquals(new HashSet<(int, int)>(tree.SelfOverlappingPairs())), "self pairs");
        }

        [TestMethod]
        [DeploymentItem(@"Files/Faces/ToroidalFace.json", nameof(binned_sah_needs_no_more_box_tests_than_median_split))]
        public void binned_sah_needs_no_more_box_tests_than_median_split()
        {
            // compares the traversal cost (number of box tests, i.e. predicate invocations) of the
            // two build strategies on a real triangulation; both must return identical results
            var file = System.IO.Path.Combine(TestContext.DeploymentDirectory,
                nameof(binned_sah_needs_no_more_box_tests_than_median_split), "ToroidalFace.json");
            Face face;
            using (var stream = File.Open(file, FileMode.Open))
            {
                face = new JsonSerialize().FromStream(stream) as Face;
            }
            face.GetTriangulation(0.02, out GeoPoint[] trianglePoint, out GeoPoint2D[] _, out int[] triangleIndex, out BoundingBox extent);

            BVHTree sah = BVHTree.FromTriangles(trianglePoint, triangleIndex, BVHTree.DefaultLeafSize, BVHBuildStrategy.BinnedSAH);
            BVHTree median = BVHTree.FromTriangles(trianglePoint, triangleIndex, BVHTree.DefaultLeafSize, BVHBuildStrategy.MedianSplit);

            (long tests, HashSet<int> hits) Run(BVHTree tree, BoundingBox query)
            {
                long c = 0;
                var hits = new HashSet<int>(tree.Traverse(b => { c++; return b.Interferes(query); }));
                return (c, hits);
            }

            Random rnd = new Random(4711);
            long sahTotal = 0, medianTotal = 0;
            for (int q = 0; q < 200; q++)
            {
                // small query boxes randomly placed within the extent of the face
                GeoPoint c = new GeoPoint(
                    extent.Xmin + rnd.NextDouble() * extent.XDiff,
                    extent.Ymin + rnd.NextDouble() * extent.YDiff,
                    extent.Zmin + rnd.NextDouble() * extent.ZDiff);
                BoundingBox query = new BoundingBox(c, 0.05 * extent.Size);
                (long sahTests, HashSet<int> sahHits) = Run(sah, query);
                (long medianTests, HashSet<int> medianHits) = Run(median, query);
                Assert.IsTrue(sahHits.SetEquals(medianHits), $"query {q}: strategies must agree on the result");
                sahTotal += sahTests;
                medianTotal += medianTests;
            }
            TestContext.WriteLine($"box tests over 200 queries: SAH {sahTotal}, median split {medianTotal} " +
                $"({(double)sahTotal / medianTotal:P1} of median)");
            Assert.IsTrue(sahTotal <= medianTotal,
                $"binned SAH should not need more box tests than median split (SAH {sahTotal} vs median {medianTotal})");
        }

        [TestMethod]
        public void empty_and_single_item_trees_work()
        {
            BVHTree empty = new BVHTree(new BoundingBox[0]);
            Assert.AreEqual(0, empty.Count);
            Assert.AreEqual(0, new List<int>(empty.Overlapping(new BoundingBox(GeoPoint.Origin, 100))).Count);
            Assert.AreEqual(0, new List<(int, int)>(empty.SelfOverlappingPairs()).Count);

            BoundingBox single = new BoundingBox(new GeoPoint(1, 1, 1), new GeoPoint(2, 2, 2));
            BVHTree one = new BVHTree(new[] { single });
            Assert.AreEqual(1, one.Count);
            var hits = new List<int>(one.Overlapping(new BoundingBox(GeoPoint.Origin, 10)));
            Assert.AreEqual(1, hits.Count);
            Assert.AreEqual(0, hits[0]);
            Assert.AreEqual(0, new List<int>(one.Overlapping(new BoundingBox(new GeoPoint(5, 5, 5), new GeoPoint(6, 6, 6)))).Count);
            Assert.AreEqual(0, new List<(int, int)>(one.SelfOverlappingPairs()).Count);

            // dual traversal with an empty side yields nothing
            Assert.AreEqual(0, new List<(int, int)>(one.Overlapping(empty)).Count);
            Assert.AreEqual(0, new List<(int, int)>(empty.Overlapping(one)).Count);
        }

        [TestMethod]
        public void identical_boxes_do_not_break_the_build()
        {
            // all centroids coincide: no split axis separates the items, the build must
            // terminate with a (possibly oversized) leaf instead of recursing forever
            BoundingBox box = new BoundingBox(new GeoPoint(-1, -1, -1), new GeoPoint(1, 1, 1));
            BoundingBox[] boxes = new BoundingBox[100];
            for (int i = 0; i < boxes.Length; i++) boxes[i] = box;
            BVHTree tree = new BVHTree(boxes);
            Assert.AreEqual(100, new HashSet<int>(tree.Overlapping(box)).Count);
            Assert.AreEqual(100 * 99 / 2, new List<(int, int)>(tree.SelfOverlappingPairs()).Count);
        }

        [TestMethod]
        public void generic_wrapper_returns_items()
        {
            // items are (name, box) tuples; the wrapper must return the items themselves
            var items = new List<(string name, BoundingBox box)>
            {
                ("a", new BoundingBox(new GeoPoint(0, 0, 0), new GeoPoint(1, 1, 1))),
                ("b", new BoundingBox(new GeoPoint(10, 10, 10), new GeoPoint(11, 11, 11))),
                ("c", new BoundingBox(new GeoPoint(0.5, 0.5, 0.5), new GeoPoint(2, 2, 2))),
            };
            var bvh = new BVH<(string name, BoundingBox box)>(items, it => it.box);
            var hits = new List<string>();
            foreach (var it in bvh.Overlapping(new BoundingBox(GeoPoint.Origin, 3))) hits.Add(it.name);
            hits.Sort();
            CollectionAssert.AreEqual(new[] { "a", "c" }, hits);

            var other = new BVH<string>(new[] { "x" }, _ => new BoundingBox(new GeoPoint(0.9, 0.9, 0.9), new GeoPoint(1.2, 1.2, 1.2)));
            var pairs = new List<(( string, BoundingBox), string)>(bvh.Overlapping(other));
            Assert.AreEqual(2, pairs.Count); // "a" and "c" touch the box of "x"
        }

        [TestMethod]
        [DeploymentItem(@"Files/Faces/ToroidalFace.json", nameof(triangle_bvh_over_face_triangulation_matches_brute_force))]
        public void triangle_bvh_over_face_triangulation_matches_brute_force()
        {
            // the intended first use case: a BVH over the triangles of a triangulated face
            var file = System.IO.Path.Combine(TestContext.DeploymentDirectory,
                nameof(triangle_bvh_over_face_triangulation_matches_brute_force), "ToroidalFace.json");
            Assert.IsTrue(File.Exists(file));
            Face face;
            using (var stream = File.Open(file, FileMode.Open))
            {
                face = new JsonSerialize().FromStream(stream) as Face;
            }
            Assert.IsNotNull(face);

            face.GetTriangulation(0.05, out GeoPoint[] trianglePoint, out GeoPoint2D[] _, out int[] triangleIndex, out BoundingBox extent);
            int triangleCount = triangleIndex.Length / 3;
            Assert.IsTrue(triangleCount > 100, "expecting a non-trivial triangulation for this test");

            BVHTree tree = BVHTree.FromTriangles(trianglePoint, triangleIndex);
            Assert.AreEqual(triangleCount, tree.Count);

            // reference boxes, computed independently of the factory
            BoundingBox[] boxes = new BoundingBox[triangleCount];
            for (int i = 0; i < triangleCount; i++)
            {
                BoundingBox b = BoundingBox.EmptyBoundingBox;
                b.MinMax(trianglePoint[triangleIndex[3 * i]]);
                b.MinMax(trianglePoint[triangleIndex[3 * i + 1]]);
                b.MinMax(trianglePoint[triangleIndex[3 * i + 2]]);
                boxes[i] = b;
            }

            // box query around the center of the face extent
            GeoPoint center = extent.GetCenter();
            BoundingBox query = new BoundingBox(
                new GeoPoint(center.x - extent.XDiff / 4, center.y - extent.YDiff / 4, center.z - extent.ZDiff / 4),
                new GeoPoint(center.x + extent.XDiff / 4, center.y + extent.YDiff / 4, center.z + extent.ZDiff / 4));
            var expected = new HashSet<int>();
            for (int i = 0; i < triangleCount; i++)
                if (boxes[i].Interferes(query)) expected.Add(i);
            var actual = new HashSet<int>(tree.Overlapping(query));
            Assert.IsTrue(expected.Count > 0, "the query box should hit some triangles");
            Assert.IsTrue(expected.SetEquals(actual),
                $"expected {expected.Count} triangles, got {actual.Count}");

            // self pairs: adjacent triangles share vertices, so their boxes interfere; the BVH
            // must find exactly the same candidate set as the brute force n^2 loop
            var expectedPairs = new HashSet<(int, int)>();
            for (int i = 0; i < triangleCount; i++)
                for (int j = i + 1; j < triangleCount; j++)
                    if (boxes[i].Interferes(boxes[j])) expectedPairs.Add((i, j));
            var actualPairs = new HashSet<(int, int)>(tree.SelfOverlappingPairs());
            Assert.IsTrue(expectedPairs.SetEquals(actualPairs),
                $"expected {expectedPairs.Count} pairs, got {actualPairs.Count}");
        }
    }
}
