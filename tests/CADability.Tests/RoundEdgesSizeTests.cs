using System;
using System.Collections.Generic;
using System.Linq;
using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="RoundEdges.CheckSelection"/> with radii which do not fit the curvature of the faces: a box
    /// 40 x 60 x 20 with the four vertical edges rounded by 5. The bottom outline (four lines and four arcs) can only be
    /// rounded with a radius smaller than 5, because the ball cannot touch the cylinders of radius 5 tangentially otherwise.
    /// </summary>
    [TestClass]
    public class RoundEdgesSizeTests
    {
        private static Shell BoxWithRoundedVerticalEdges()
        {
            Solid box = Make3D.MakeBox(GeoPoint.Origin, 40 * GeoVector.XAxis, 60 * GeoVector.YAxis, 20 * GeoVector.ZAxis);
            Edge[] vertical = box.Shells[0].Edges.Where(e => e.Curve3D is Line line && Math.Abs(line.StartDirection.z) > 0.5).ToArray();
            Assert.AreEqual(4, vertical.Length);
            Shell? shell = box.Shells[0].RoundEdges(vertical, 5.0);
            Assert.IsNotNull(shell, "the vertical edges could not be rounded");
            return shell;
        }

        private static Edge[] BottomEdges(Shell shell)
        {
            Edge[] bottom = shell.Edges.Where(e => e.Curve3D != null && Math.Abs(e.Curve3D.StartPoint.z) < 1e-6 && Math.Abs(e.Curve3D.EndPoint.z) < 1e-6
                && Math.Abs(e.Curve3D.PointAt(0.5).z) < 1e-6).ToArray();
            Assert.AreEqual(8, bottom.Length, "four lines and four arcs expected");
            return bottom;
        }

        [TestMethod]
        public void radius_bigger_than_the_corner_radius_is_rejected()
        {
            Shell shell = BoxWithRoundedVerticalEdges();
            RoundEdges re = new RoundEdges(shell, BottomEdges(shell), 8.0);
            string? problem = re.CheckSelection();
            Assert.IsNotNull(problem, "a radius of 8 on a corner of radius 5 must be rejected");
            StringAssert.Contains(problem, "too large");
            StringAssert.Contains(problem, "radius of 5");
            Assert.IsNull(re.Execute());
        }

        [TestMethod]
        public void radius_equal_to_the_corner_radius_is_rejected()
        {
            Shell shell = BoxWithRoundedVerticalEdges();
            Assert.IsNotNull(new RoundEdges(shell, BottomEdges(shell), 5.0).CheckSelection());
        }

        [TestMethod]
        public void radius_smaller_than_the_corner_radius_is_accepted()
        {
            Shell shell = BoxWithRoundedVerticalEdges();
            RoundEdges re = new RoundEdges(shell, BottomEdges(shell), 3.0);
            Assert.IsNull(re.CheckSelection());
            Shell? res = re.Execute();
            Assert.IsNotNull(res, "rounding with radius 3 failed");
            Assert.AreEqual(0, res.OpenEdgesExceptPoles.Length, "the shell has open edges");
        }

        /// <summary>
        /// The curvature of the cylinder is along the arc, not across it: rounding the rim of a cylinder of radius 10 with
        /// radius 12 must be rejected, while the planar top face does not limit the radius.
        /// </summary>
        [TestMethod]
        public void cylinder_rim_bigger_than_the_cylinder_is_rejected()
        {
            Solid cylinder = Make3D.MakeCylinder(GeoPoint.Origin, 10 * GeoVector.XAxis, 30 * GeoVector.ZAxis);
            Shell shell = cylinder.Shells[0];
            Edge[] rim = shell.Edges.Where(e => e.Curve3D != null && e.Curve3D.StartPoint.z > 29.0 && e.Curve3D.PointAt(0.5).z > 29.0).ToArray();
            Assert.IsTrue(rim.Length > 0);
            Assert.IsNotNull(new RoundEdges(shell, rim, 12.0).CheckSelection());
            Assert.IsNull(new RoundEdges(shell, rim, 2.0).CheckSelection());
        }
    }
}
