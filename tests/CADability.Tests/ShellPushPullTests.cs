using System;
using System.Collections.Generic;
using System.Linq;
using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="ShellExtensions.PushPull"/> at tangential edges. The body is a prism 40 x 30 x 20 whose vertical
    /// edge at x = 40, y = 30 is rounded with the radius 5: the cylinder is tangential to the side faces x = 40 and y = 30.
    /// </summary>
    [TestClass]
    public class ShellPushPullTests
    {
        private const double r = 5.0;
        private const double d = 3.0;

        private static Shell RoundedPrism()
        {
            Solid box = Make3D.MakeBox(GeoPoint.Origin, 40 * GeoVector.XAxis, 30 * GeoVector.YAxis, 20 * GeoVector.ZAxis);
            Edge vertical = box.Shells[0].Edges.Single(e => e.Curve3D is Line line && Math.Abs(line.StartDirection.z) > 0.5 && (line.StartPoint | new GeoPoint(40, 30, line.StartPoint.z)) < 1e-6);
            Shell shell = box.Shells[0].RoundEdges(new[] { vertical }, r);
            Assert.IsNotNull(shell, "the edge could not be rounded");
            Assert.AreEqual(7, shell.Faces.Length);
            Assert.AreEqual(1200 - r * r + Math.PI * r * r / 4, shell.Volume(0.01) / 20, 1e-3, "wrong volume of the prism");
            return shell;
        }

        private static IEnumerable<Face> FacesWhere(Shell shell, Func<Face, bool> predicate) => shell.Faces.Where(predicate).ToArray();

        private static bool IsTop(Face face) => face.Surface is PlaneSurface && face.GetExtent(0.0).Zmin > 19.0;

        /// <summary>the cylinder and the two side faces tangential to it</summary>
        private static bool IsRoundedSide(Face face) => face.Surface is CylindricalSurface
            || face.Surface is PlaneSurface && (face.GetExtent(0.0).Xmin > 39.0 || face.GetExtent(0.0).Ymin > 29.0);

        private static void AssertValid(Shell shell)
        {
            Assert.AreEqual(0, shell.OpenEdgesExceptPoles.Length, "the shell has open edges");
            Assert.IsTrue(shell.CheckConsistency(), "the shell is not consistent");
            Assert.AreEqual(7, shell.Faces.Length, "the topology has changed");
        }

        /// <summary>
        /// Pulling the top face: at the ends of the arc the moved vertex slides along the tangential edges between the
        /// cylinder and the side faces, which are not moved (the third edge at the vertex is tangential).
        /// </summary>
        [TestMethod]
        public void pulling_a_face_slides_along_an_unmoved_tangential_edge()
        {
            Shell shell = RoundedPrism();
            Assert.IsTrue(shell.PushPull(FacesWhere(shell, IsTop), d));
            AssertValid(shell);
            double expected = (1200 - r * r + Math.PI * r * r / 4) * (20 + d);
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-6);
        }

        /// <summary>
        /// Pulling the cylinder together with both tangential side faces: the tangential edges between them are offset
        /// and the radius grows to r + d around the same axis.
        /// </summary>
        [TestMethod]
        public void pulling_a_tangential_chain_offsets_the_tangential_edges()
        {
            Shell shell = RoundedPrism();
            Assert.IsTrue(shell.PushPull(FacesWhere(shell, IsRoundedSide), d));
            AssertValid(shell);
            double R = r + d;
            double expected = ((40 + d) * (30 + d) - R * R + Math.PI * R * R / 4) * 20;
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-6);
        }

        /// <summary>
        /// Both at once: the offset tangential edges between the moved side faces and the cylinder end in the moved top face.
        /// </summary>
        [TestMethod]
        public void pulling_a_tangential_chain_and_the_top_face()
        {
            Shell shell = RoundedPrism();
            Assert.IsTrue(shell.PushPull(FacesWhere(shell, f => IsTop(f) || IsRoundedSide(f)), d));
            AssertValid(shell);
            double R = r + d;
            double expected = ((40 + d) * (30 + d) - R * R + Math.PI * R * R / 4) * (20 + d);
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-6);
        }

        /// <summary>
        /// Pushing the chain inwards: the radius shrinks to r - d.
        /// </summary>
        [TestMethod]
        public void pushing_a_tangential_chain_shrinks_the_fillet()
        {
            Shell shell = RoundedPrism();
            Assert.IsTrue(shell.PushPull(FacesWhere(shell, IsRoundedSide), -d));
            AssertValid(shell);
            double R = r - d;
            double expected = ((40 - d) * (30 - d) - R * R + Math.PI * R * R / 4) * 20;
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-6);
        }
    }
}
