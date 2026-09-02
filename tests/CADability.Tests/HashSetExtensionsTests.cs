namespace CADability.Tests
{
    /// <summary>
    /// Pins down the behaviour of the <see cref="HashSetExtensions"/> shim that replaces
    /// Wintellect.PowerCollections' <c>Set&lt;T&gt;</c>. The interesting cases are the ones where a
    /// naive implementation would differ from the PowerCollections original: the empty-set behaviour
    /// of GetAny, and passing a set to itself in AddMany/RemoveMany.
    /// </summary>
    [TestClass]
    public class HashSetExtensionsTests
    {
        [TestMethod]
        public void AddMany_AddsAllItems()
        {
            HashSet<int> set = new HashSet<int> { 1, 2 };
            set.AddMany(new[] { 2, 3, 4 });
            CollectionAssert.AreEquivalent(new[] { 1, 2, 3, 4 }, set.ToArray());
        }

        [TestMethod]
        public void AddMany_WithItself_LeavesSetUnchanged()
        {
            // PowerCollections returned early here; enumerating while adding would otherwise throw.
            HashSet<int> set = new HashSet<int> { 1, 2, 3 };
            set.AddMany(set);
            CollectionAssert.AreEquivalent(new[] { 1, 2, 3 }, set.ToArray());
        }

        [TestMethod]
        public void RemoveMany_ReturnsNumberOfItemsActuallyRemoved()
        {
            HashSet<int> set = new HashSet<int> { 1, 2, 3 };
            // 2 and 3 are present, 99 is not, so only two removals count.
            Assert.AreEqual(2, set.RemoveMany(new[] { 2, 3, 99 }));
            CollectionAssert.AreEquivalent(new[] { 1 }, set.ToArray());
        }

        [TestMethod]
        public void RemoveMany_WithItself_ClearsSetAndReturnsFormerCount()
        {
            HashSet<int> set = new HashSet<int> { 1, 2, 3 };
            Assert.AreEqual(3, set.RemoveMany(set));
            Assert.AreEqual(0, set.Count);
        }

        [TestMethod]
        public void ContainsAll_IsTrueOnlyIfEveryItemIsPresent()
        {
            HashSet<int> set = new HashSet<int> { 1, 2, 3 };
            Assert.IsTrue(set.ContainsAll(new[] { 1, 3 }));
            Assert.IsTrue(set.ContainsAll(new int[0]));
            Assert.IsFalse(set.ContainsAll(new[] { 1, 4 }));
        }

        [TestMethod]
        public void IsEmpty_ReflectsCount()
        {
            Assert.IsTrue(new HashSet<int>().IsEmpty());
            Assert.IsFalse(new HashSet<int> { 1 }.IsEmpty());
        }

        [TestMethod]
        public void GetAny_OnEmptySet_ReturnsDefaultInsteadOfThrowing()
        {
            // This is why GetAny cannot simply be replaced by Enumerable.First.
            Assert.AreEqual(0, new HashSet<int>().GetAny());
            Assert.IsNull(new HashSet<string>().GetAny());
        }

        [TestMethod]
        public void GetAny_ReturnsAContainedItemWithoutRemovingIt()
        {
            HashSet<int> set = new HashSet<int> { 7, 8 };
            int any = set.GetAny();
            Assert.IsTrue(set.Contains(any));
            Assert.AreEqual(2, set.Count);
        }

        [TestMethod]
        public void GetAndRemoveAny_ReturnsAndRemovesOneItem()
        {
            HashSet<int> set = new HashSet<int> { 7, 8 };
            int any = set.GetAndRemoveAny();
            Assert.IsTrue(any == 7 || any == 8);
            Assert.IsFalse(set.Contains(any));
            Assert.AreEqual(1, set.Count);
        }

        [TestMethod]
        public void GetAndRemoveAny_OnEmptySet_ReturnsDefault()
        {
            HashSet<string> set = new HashSet<string>();
            Assert.IsNull(set.GetAndRemoveAny());
            Assert.AreEqual(0, set.Count);
        }

        [TestMethod]
        public void IsEqualTo_ComparesContentNotOrder()
        {
            Assert.IsTrue(new HashSet<int> { 1, 2, 3 }.IsEqualTo(new HashSet<int> { 3, 2, 1 }));
            Assert.IsFalse(new HashSet<int> { 1, 2 }.IsEqualTo(new HashSet<int> { 1, 2, 3 }));
        }

        [TestMethod]
        public void Intersection_Difference_SymmetricDifference_DoNotModifyOperands()
        {
            HashSet<int> a = new HashSet<int> { 1, 2, 3 };
            HashSet<int> b = new HashSet<int> { 3, 4 };

            CollectionAssert.AreEquivalent(new[] { 3 }, a.Intersection(b).ToArray());
            CollectionAssert.AreEquivalent(new[] { 1, 2 }, a.Difference(b).ToArray());
            CollectionAssert.AreEquivalent(new[] { 1, 2, 4 }, a.SymmetricDifference(b).ToArray());

            // The PowerCollections versions returned a new set and left both operands alone.
            CollectionAssert.AreEquivalent(new[] { 1, 2, 3 }, a.ToArray());
            CollectionAssert.AreEquivalent(new[] { 3, 4 }, b.ToArray());
        }

        [TestMethod]
        public void SetOperations_KeepTheEqualityComparerOfTheLeftOperand()
        {
            HashSet<string> a = new HashSet<string>(StringComparer.OrdinalIgnoreCase) { "ALPHA", "BETA" };
            HashSet<string> b = new HashSet<string>(StringComparer.OrdinalIgnoreCase) { "beta" };

            // Without carrying the comparer over, "beta" would not match "BETA" here.
            CollectionAssert.AreEquivalent(new[] { "BETA" }, a.Intersection(b).ToArray());
            CollectionAssert.AreEquivalent(new[] { "ALPHA" }, a.Difference(b).ToArray());
            Assert.IsTrue(a.Difference(b).Contains("alpha"));
        }
    }
}
