using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// Pins down <see cref="GapInserter.FillLargestGaps"/>. The class moved from ShapeIt into CADability,
    /// and its priority queue had to be replaced on the way: <c>PriorityQueue&lt;,&gt;</c> only exists from
    /// .NET 6 on, while CADability targets netstandard2.0. These tests check the behaviour the replacement
    /// has to reproduce - repeatedly splitting the currently widest gap at its midpoint.
    /// </summary>
    [TestClass]
    public class GapInserterTests
    {
        private const double Eps = 1e-9;

        private static void AssertSorted(List<double> xs)
        {
            for (int i = 1; i < xs.Count; i++)
            {
                Assert.IsTrue(xs[i - 1] <= xs[i], $"result is not sorted at index {i}: {xs[i - 1]} > {xs[i]}");
            }
        }

        [TestMethod]
        public void FillLargestGaps_SplitsTheWidestGapFirst()
        {
            List<double> xs = new List<double> { 0.0, 1.0, 10.0 };

            GapInserter.FillLargestGaps(xs, 4);

            // 1..10 is the widest gap, so its midpoint 5.5 is the one that gets inserted.
            CollectionAssert.AreEqual(new[] { 0.0, 1.0, 5.5, 10.0 }, xs, Comparer<double>.Create(
                (a, b) => Math.Abs(a - b) <= Eps ? 0 : a.CompareTo(b)));
        }

        [TestMethod]
        public void FillLargestGaps_KeepsSplittingTheCurrentlyWidest()
        {
            // Deliberately chosen so that the second step is unambiguous: splitting a gap always leaves
            // two equal halves, so the widths have to be picked such that an untouched gap is strictly
            // the widest afterwards. 0..100 splits into two 50s, which leaves 100..160 (60) on top.
            List<double> xs = new List<double> { 0.0, 100.0, 160.0 };

            GapInserter.FillLargestGaps(xs, 5);

            CollectionAssert.AreEqual(new[] { 0.0, 50.0, 100.0, 130.0, 160.0 }, xs, Comparer<double>.Create(
                (a, b) => Math.Abs(a - b) <= Eps ? 0 : a.CompareTo(b)));
        }

        [TestMethod]
        public void FillLargestGaps_ReachesTheRequestedCountAndStaysSorted()
        {
            List<double> xs = new List<double> { 0.0, 0.25, 3.0, 12.0, 12.5 };

            GapInserter.FillLargestGaps(xs, 20);

            Assert.AreEqual(20, xs.Count, "the requested number of values must be reached");
            AssertSorted(xs);
            Assert.AreEqual(0.0, xs[0], Eps, "the original bounds must not move");
            Assert.AreEqual(12.5, xs[xs.Count - 1], Eps, "the original bounds must not move");
        }

        [TestMethod]
        public void FillLargestGaps_DistributesEvenlyOnAUniformStart()
        {
            // Splitting each of the four unit gaps once gives eight steps of 0.5.
            List<double> xs = new List<double> { 0.0, 1.0, 2.0, 3.0, 4.0 };

            GapInserter.FillLargestGaps(xs, 9);

            Assert.AreEqual(9, xs.Count);
            AssertSorted(xs);
            for (int i = 0; i < xs.Count; i++)
            {
                Assert.AreEqual(i * 0.5, xs[i], Eps, $"value at index {i}");
            }
        }

        [TestMethod]
        public void FillLargestGaps_DoesNothingWhenTheCountIsAlreadyReached()
        {
            List<double> xs = new List<double> { 0.0, 1.0, 10.0 };

            GapInserter.FillLargestGaps(xs, 3);
            CollectionAssert.AreEqual(new[] { 0.0, 1.0, 10.0 }, xs);

            GapInserter.FillLargestGaps(xs, 2);
            CollectionAssert.AreEqual(new[] { 0.0, 1.0, 10.0 }, xs);
        }

        [TestMethod]
        public void FillLargestGaps_StopsOnDegenerateInput()
        {
            // All values equal: there is no gap to split, so the list cannot grow and must not hang.
            List<double> xs = new List<double> { 2.0, 2.0, 2.0 };

            GapInserter.FillLargestGaps(xs, 10);

            Assert.AreEqual(3, xs.Count, "without a gap nothing can be inserted");
        }
    }
}
