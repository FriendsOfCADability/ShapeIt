using CADability.GeoObject;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Linq;

namespace CADability.Tests
{
    /// <summary>
    /// Boolean operations on solids with cavities. The hollow shells are assembled manually from a hull and inverted inner shells,
    /// so a failing test points at the boolean operation and not at the way the operands were built.
    /// </summary>
    [TestClass]
    public class BooleanOperationCavityTests
    {
        private const double TriangulationPrecision = 0.01;
        private const double Tolerance = 1e-3; // relative error of the volume, the cavities are approximated by triangles

        private static Shell Box(GeoPoint location, double size)
            => Make3D.MakeBox(location, size * GeoVector.XAxis, size * GeoVector.YAxis, size * GeoVector.ZAxis).Shells[0];

        /// <summary>
        /// Creates a shell consisting of the faces of <paramref name="hull"/> and the inverted faces of the <paramref name="cavities"/>,
        /// which must be located inside the hull.
        /// </summary>
        private static Shell MakeHollowShell(Shell hull, params Shell[] cavities)
        {
            foreach (Shell cavity in cavities)
            {
                cavity.ReverseOrientation(); // the normals of a cavity point into the cavity
                hull.AddInnerHole(cavity.Faces);
            }
            return hull;
        }

        private static Shell[] Execute(Shell first, Shell second, BooleanOperation.Operation operation)
        {
            BooleanOperation booleanOperation = new BooleanOperation();
            booleanOperation.SetShells(first, second, operation); // SetShells clones, the operands stay intact
            return booleanOperation.Execute() ?? Array.Empty<Shell>();
        }

        private static void AssertVolume(double expected, Shell shell, string message)
            => Assert.AreEqual(expected, shell.Volume(TriangulationPrecision), Math.Abs(expected) * Tolerance, message);

        [TestMethod]
        public void DifferenceWithEnclosedSolidCreatesCavity()
        {
            // the classic case: a sphere completely inside a box is subtracted, the result is a box with a spherical cavity
            Shell box = Box(new GeoPoint(0.0, 0.0, 0.0), 40.0);
            Shell sphere = Make3D.MakeSphere(new GeoPoint(20.0, 20.0, 20.0), 8.0).Shells[0];

            Shell[] result = Execute(box, sphere, BooleanOperation.Operation.difference);

            Assert.AreEqual(1, result.Length, "the result is a single solid");
            (HashSet<Face> hull, HashSet<Face>[] holes) = result[0].GetHullAndHoles();
            Assert.AreEqual(6, hull.Count, "the hull is the untouched box");
            Assert.AreEqual(1, holes.Length, "the sphere has become a cavity");
            AssertVolume(40.0 * 40.0 * 40.0 - 4.0 / 3.0 * Math.PI * 512.0, result[0], "the volume of the cavity must be missing");
        }

        [TestMethod]
        public void IntersectionKeepsTheCavityOfTheEnclosedSolid()
        {
            // a hollow box completely inside a bigger box: the intersection is the hollow box, the cavity has to survive
            Shell hollow = MakeHollowShell(Box(new GeoPoint(0.0, 0.0, 0.0), 40.0), Make3D.MakeSphere(new GeoPoint(20.0, 20.0, 20.0), 8.0).Shells[0]);
            double hollowVolume = 40.0 * 40.0 * 40.0 - 4.0 / 3.0 * Math.PI * 512.0;
            Shell bigBox = Box(new GeoPoint(-20.0, -20.0, -20.0), 100.0);

            Shell[] result = Execute(hollow, bigBox, BooleanOperation.Operation.intersection);

            Assert.AreEqual(1, result.Length, "the result is a single solid");
            (HashSet<Face> hull, HashSet<Face>[] holes) = result[0].GetHullAndHoles();
            Assert.AreEqual(6, hull.Count, "the hull is the untouched box");
            Assert.AreEqual(1, holes.Length, "the cavity must survive");
            AssertVolume(hollowVolume, result[0], "the result is the hollow box");
        }

        [TestMethod]
        public void UnionWithDisjointSolidKeepsTheCavity()
        {
            // a hollow box and a disjoint box: the result are two solids, the cavity of the first one has to survive
            Shell hollow = MakeHollowShell(Box(new GeoPoint(0.0, 0.0, 0.0), 40.0), Make3D.MakeSphere(new GeoPoint(20.0, 20.0, 20.0), 8.0).Shells[0]);
            double hollowVolume = 40.0 * 40.0 * 40.0 - 4.0 / 3.0 * Math.PI * 512.0;
            Shell disjoint = Box(new GeoPoint(100.0, 0.0, 0.0), 10.0);

            Shell[] result = Execute(hollow, disjoint, BooleanOperation.Operation.union);

            Assert.AreEqual(2, result.Length, "the two solids are disjoint, the union returns both");
            Shell withCavity = result.OrderByDescending(sh => sh.GetExtent(0.0).Volume).First();
            Shell separate = result.OrderByDescending(sh => sh.GetExtent(0.0).Volume).Last();
            Assert.AreEqual(1, withCavity.GetHullAndHoles().holes.Length, "the cavity must survive");
            Assert.AreEqual(0, separate.GetHullAndHoles().holes.Length, "the disjoint box has no cavity");
            AssertVolume(hollowVolume, withCavity, "the hollow box is unchanged");
            AssertVolume(1000.0, separate, "the disjoint box is unchanged");
        }

        [TestMethod]
        public void UnionWithSolidInsideTheCavityReturnsTwoSolids()
        {
            // a smaller sphere floating inside the cavity of a hollow box: it is not a hole of the hollow box but a separate solid
            Shell hollow = MakeHollowShell(Box(new GeoPoint(0.0, 0.0, 0.0), 40.0), Make3D.MakeSphere(new GeoPoint(20.0, 20.0, 20.0), 8.0).Shells[0]);
            double hollowVolume = 40.0 * 40.0 * 40.0 - 4.0 / 3.0 * Math.PI * 512.0;
            Shell inner = Make3D.MakeSphere(new GeoPoint(20.0, 20.0, 20.0), 4.0).Shells[0];

            Shell[] result = Execute(hollow, inner, BooleanOperation.Operation.union);

            Assert.AreEqual(2, result.Length, "the sphere inside the cavity is a separate solid");
            Shell withCavity = result.OrderByDescending(sh => sh.GetExtent(0.0).Volume).First();
            Shell floating = result.OrderByDescending(sh => sh.GetExtent(0.0).Volume).Last();
            Assert.AreEqual(1, withCavity.GetHullAndHoles().holes.Length, "the cavity must survive");
            AssertVolume(hollowVolume, withCavity, "the hollow box is unchanged");
            AssertVolume(4.0 / 3.0 * Math.PI * 64.0, floating, "the inner sphere is unchanged");
        }

        [TestMethod]
        public void UnionFillingTheCavityRemovesIt()
        {
            // a solid which fills the cavity and is connected to the hull: the cavity disappears
            Shell hollow = MakeHollowShell(Box(new GeoPoint(0.0, 0.0, 0.0), 40.0), Make3D.MakeSphere(new GeoPoint(20.0, 20.0, 20.0), 8.0).Shells[0]);
            Shell filling = Make3D.MakeCylinder(new GeoPoint(20.0, 20.0, -10.0), 9.0 * GeoVector.XAxis, 60.0 * GeoVector.ZAxis).Shells[0];

            Shell[] result = Execute(hollow, filling, BooleanOperation.Operation.union);

            Assert.AreEqual(1, result.Length, "the result is a single solid");
            Assert.AreEqual(0, result[0].GetHullAndHoles().holes.Length, "the cavity has been filled and connected to the outside");
        }

        [TestMethod]
        public void DifferenceOfTwoHollowSolidsKeepsTheUntouchedCavity()
        {
            // a hollow box minus a smaller box which touches neither the hull nor the cavity of the first one: the untouched cavity
            // has to survive, the subtracted box becomes a second cavity
            Shell hollow = MakeHollowShell(Box(new GeoPoint(0.0, 0.0, 0.0), 40.0), Make3D.MakeSphere(new GeoPoint(10.0, 10.0, 10.0), 5.0).Shells[0]);
            Shell toSubtract = Box(new GeoPoint(28.0, 28.0, 28.0), 6.0);

            Shell[] result = Execute(hollow, toSubtract, BooleanOperation.Operation.difference);

            Assert.AreEqual(1, result.Length, "the result is a single solid");
            (HashSet<Face> hull, HashSet<Face>[] holes) = result[0].GetHullAndHoles();
            Assert.AreEqual(6, hull.Count, "the hull is the untouched box");
            Assert.AreEqual(2, holes.Length, "the original cavity and the subtracted box");
            AssertVolume(40.0 * 40.0 * 40.0 - 4.0 / 3.0 * Math.PI * 125.0 - 6.0 * 6.0 * 6.0, result[0], "both cavities are missing");
        }
    }
}
