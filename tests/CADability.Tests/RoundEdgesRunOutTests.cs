using System;
using System.Collections.Generic;
using System.Linq;
using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// Rounding edges which run out tangentially: a box 40 x 60 x 20 whose bottom edges are rounded by 8. The four
    /// cylinders meet in miter edges, which are convex but end tangentially in the bottom face (the two cylinders have
    /// the same normal there). Rounding a vertical edge together with the miter edge below it by 5 makes a fillet which
    /// ends in a point at the bottom.
    /// </summary>
    [TestClass]
    public class RoundEdgesRunOutTests
    {
        private const double r = 5.0;

        private static Shell BoxWithRoundedBottom()
        {
            Solid box = Make3D.MakeBox(GeoPoint.Origin, 40 * GeoVector.XAxis, 60 * GeoVector.YAxis, 20 * GeoVector.ZAxis);
            Edge[] bottom = box.Shells[0].Edges.Where(e => Math.Abs(e.Curve3D.StartPoint.z) < 1e-6 && Math.Abs(e.Curve3D.EndPoint.z) < 1e-6).ToArray();
            Shell? shell = box.Shells[0].RoundEdges(bottom, 8.0);
            Assert.IsNotNull(shell, "the bottom edges could not be rounded");
            return shell;
        }

        private static Edge[] Vertical(Shell shell) => shell.Edges.Where(e => e.Curve3D is Line l && Math.Abs(l.StartDirection.Normalized.z) > 0.99).ToArray();

        private static Edge[] Miters(Shell shell) => shell.Edges.Where(e => e.PrimaryFace.Surface is CylindricalSurface && e.SecondaryFace?.Surface is CylindricalSurface).ToArray();

        /// <summary>
        /// The removed volume of a fillet of radius r along a straight edge of length l between two perpendicular planes.
        /// </summary>
        private static double StraightFilletVolume(double length) => length * (r * r - Math.PI * r * r / 4);

        private static void AssertValid(Shell shell)
        {
            Assert.AreEqual(0, shell.OpenEdgesExceptPoles.Length, "the shell has open edges");
            Assert.IsTrue(shell.CheckConsistency(), "the shell is not consistent");
        }

        [TestMethod]
        public void miter_edge_is_classified_as_convex()
        {
            Shell shell = BoxWithRoundedBottom();
            Edge[] miters = Miters(shell);
            Assert.AreEqual(4, miters.Length);
            RoundEdges re = new RoundEdges(shell, miters, r);
            Assert.AreEqual(4, re.convexEdges.Count(), "the miter edges are convex, although they are tangential at their lower end");
            Assert.AreEqual(0, re.concaveEdges.Count());
        }

        [TestMethod]
        public void vertical_and_miter_edge_of_one_corner()
        {
            Shell shell = BoxWithRoundedBottom();
            double before = shell.Volume(0.01);
            Edge vertical = Vertical(shell).Single(e => (new GeoPoint(e.Curve3D.StartPoint.x, e.Curve3D.StartPoint.y, 0) | GeoPoint.Origin) < 1e-6);
            Edge miter = Miters(shell).Single(e => e.Curve3D.PointAt(0.5).x < 10 && e.Curve3D.PointAt(0.5).y < 10);
            RoundEdges re = new RoundEdges(shell, new[] { vertical, miter }, r);
            Assert.IsNull(re.CheckSelection());
            Shell? res = re.Execute();
            Assert.IsNotNull(res, "rounding failed");
            AssertValid(res);
            Assert.AreEqual(shell.Faces.Length + 2, res.Faces.Length, "one fillet face for each edge");
            Assert.AreEqual(1, res.Faces.Count(f => f.Surface is SweptCircleSurface));
            double removed = before - res.Volume(0.01);
            // the vertical part is exact, the part along the miter is less than a straight fillet along the chord of the miter
            double vertical12 = StraightFilletVolume(12.0);
            Assert.IsTrue(removed > vertical12 + 1.0, $"too little removed: {removed}");
            Assert.IsTrue(removed < vertical12 + StraightFilletVolume(8 * Math.Sqrt(3)), $"too much removed: {removed}");
        }

        [TestMethod]
        public void all_vertical_and_miter_edges()
        {
            Shell shell = BoxWithRoundedBottom();
            double before = shell.Volume(0.01);
            Edge[] edges = Vertical(shell).Concat(Miters(shell)).ToArray();
            Assert.AreEqual(8, edges.Length);
            Shell? res = new RoundEdges(shell, edges, r).Execute();
            Assert.IsNotNull(res, "rounding failed");
            AssertValid(res);
            Assert.AreEqual(shell.Faces.Length + 8, res.Faces.Length, "one fillet face for each edge");
            double removed = before - res.Volume(0.01);
            double vertical12 = StraightFilletVolume(12.0);
            Assert.IsTrue(removed > 4 * (vertical12 + 1.0), $"too little removed: {removed}");
            Assert.IsTrue(removed < 4 * (vertical12 + StraightFilletVolume(8 * Math.Sqrt(3))), $"too much removed: {removed}");
        }
    }
}
