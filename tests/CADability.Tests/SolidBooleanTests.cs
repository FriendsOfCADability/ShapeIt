using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// Covers <see cref="Solid.Unite"/>, <see cref="Solid.Intersect"/>, <see cref="Solid.Subtract"/> and
    /// <see cref="Solid.SplitByPlane"/> against volumes that can be worked out by hand.
    /// <para>
    /// These four used to run on BRepOperation (BRepIntersection.cs) and were reachable from no test at
    /// all, although Make3D.Union/Difference/Intersection and therefore a good deal of the public API sit
    /// on top of them. They now use BooleanOperation, the same engine the BRep regression suite exercises.
    /// </para>
    /// </summary>
    [TestClass]
    public class SolidBooleanTests
    {
        private const double Tolerance = 1e-6;

        /// <summary>An axis aligned box with one corner at <paramref name="corner"/>.</summary>
        private static Solid Box(GeoPoint corner, double dx, double dy, double dz)
        {
            return Make3D.MakeBox(corner, dx * GeoVector.XAxis, dy * GeoVector.YAxis, dz * GeoVector.ZAxis);
        }

        private static double VolumeOf(Solid solid)
        {
            return solid.Shells[0].Volume(solid.Shells[0].GetExtent(0.0).Size / 1000.0);
        }

        private static void AssertVolume(double expected, double actual, string what)
        {
            Assert.AreEqual(expected, actual, Math.Abs(expected) * Tolerance + Tolerance, what);
        }

        // Two 10-cubes sharing a 10 x 10 x 5 slab: union 1500, intersection 500, difference 500.
        private static Solid LowerBox() => Box(new GeoPoint(0, 0, 0), 10, 10, 10);
        private static Solid UpperBox() => Box(new GeoPoint(0, 0, 5), 10, 10, 10);

        [TestMethod]
        public void Unite_OverlappingBoxes_YieldsCombinedVolume()
        {
            Solid united = Solid.Unite(LowerBox(), UpperBox());

            Assert.IsNotNull(united, "the two boxes overlap, so the union must be a single solid");
            AssertVolume(1500.0, VolumeOf(united), "volume of the union");
        }

        [TestMethod]
        public void Unite_DisjointBoxes_ReturnsNull()
        {
            // Documented contract: null when the result is not exactly one solid.
            Solid united = Solid.Unite(LowerBox(), Box(new GeoPoint(100, 100, 100), 10, 10, 10));

            Assert.IsNull(united, "disjoint solids cannot be united into a single solid");
        }

        [TestMethod]
        public void Intersect_OverlappingBoxes_YieldsTheCommonSlab()
        {
            Solid[] common = Solid.Intersect(LowerBox(), UpperBox());

            Assert.AreEqual(1, common.Length, "the overlap is a single slab");
            AssertVolume(500.0, VolumeOf(common[0]), "volume of the intersection");
        }

        [TestMethod]
        public void Intersect_DisjointBoxes_YieldsNothing()
        {
            Solid[] common = Solid.Intersect(LowerBox(), Box(new GeoPoint(100, 100, 100), 10, 10, 10));

            Assert.AreEqual(0, common.Length, "disjoint solids have no common part");
        }

        [TestMethod]
        public void Subtract_OverlappingBoxes_RemovesTheCommonSlab()
        {
            Solid[] rest = Solid.Subtract(LowerBox(), UpperBox());

            Assert.AreEqual(1, rest.Length);
            AssertVolume(500.0, VolumeOf(rest[0]), "volume of the difference");
        }

        [TestMethod]
        public void Subtract_DisjointBoxes_LeavesTheFirstSolidUnchanged()
        {
            Solid[] rest = Solid.Subtract(LowerBox(), Box(new GeoPoint(100, 100, 100), 10, 10, 10));

            Assert.AreEqual(1, rest.Length, "nothing is removed, so the first solid survives");
            AssertVolume(1000.0, VolumeOf(rest[0]), "volume of the untouched solid");
        }

        [TestMethod]
        public void SplitByPlane_ThroughTheMiddle_YieldsTwoHalves()
        {
            Solid[] parts = LowerBox().SplitByPlane(new Plane(new GeoPoint(0, 0, 5), GeoVector.ZAxis));

            Assert.AreEqual(2, parts.Length, "a plane through the middle cuts the box in two");
            foreach (Solid part in parts)
            {
                AssertVolume(500.0, VolumeOf(part), "volume of a half");
            }
            AssertVolume(1000.0, parts.Sum(p => VolumeOf(p)), "the halves must add up to the whole box");
        }
    }
}
