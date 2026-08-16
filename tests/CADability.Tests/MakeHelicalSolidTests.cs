using CADability.GeoObject;
using CADability.Curve2D;
using CADability.Shapes;
using System;

namespace CADability.Tests
{
    [TestClass]
    public class MakeHelicalSolidTests
    {
        /// <summary>
        /// A triangle in the xz plane (the plane of the axis) whose edge (10,0,0)-(10,0,4) is parallel to the axis,
        /// so the sweep produces both HelicalSweepSurface and CylindricalSurface faces.
        /// </summary>
        private static Face MakeTriangle()
        {
            Plane pln = new Plane(GeoPoint.Origin, GeoVector.XAxis, GeoVector.ZAxis);
            Polyline2D p2d = new Polyline2D([new GeoPoint2D(10, 0), new GeoPoint2D(10, 4), new GeoPoint2D(14, 2), new GeoPoint2D(10, 0)]);
            return Face.MakeFace(new PlaneSurface(pln), new SimpleShape(new Border(p2d)));
        }

        /// <summary>
        /// Every cylindrical face must be the sweep of the axis parallel edge. The centre of its uv domain is
        /// therefore a point of that sweep: at the angle a the height must be within the range which the profile
        /// edge covers, i.e. [pitch*a/(2*PI), pitch*a/(2*PI)+4]. When the periodicity of the cylinder is handled
        /// wrongly, the patch ends up on the opposite side of the cylinder and this check fails by a lot.
        /// </summary>
        private static void AssertCylindricalFacesAreOnTheSweep(Shell shell, Axis axis, double pitch, int expectedCylFaces)
        {
            int cylFaces = 0;
            foreach (Face f in shell.Faces)
            {
                if (!(f.Surface is CylindricalSurface cs)) continue;
                cylFaces++;
                GeoPoint mid = cs.PointAt(f.Area.GetExtent().GetCenter());
                GeoVector fromAxis = mid - axis.Location;
                double height = fromAxis * axis.Direction.Normalized;
                double angle = Math.Atan2(fromAxis * GeoVector.YAxis, fromAxis * GeoVector.XAxis);
                double best = double.MaxValue;
                for (double a = angle - 8 * Math.PI; a < angle + 8 * Math.PI + 1e-9; a += 2 * Math.PI)
                {
                    double low = pitch * a / (2 * Math.PI);
                    best = Math.Min(best, Math.Max(0.0, Math.Max(low - height, height - (low + 4.0))));
                }
                Assert.IsTrue(best < 1e-6, $"a cylindrical face is not on the sweep, off by {best}");
            }
            Assert.AreEqual(expectedCylFaces, cylFaces, "number of cylindrical faces");
        }

        private static void AssertSweep(double pitch, double extrHeight, int expectedCylFaces)
        {
            Axis axis = new Axis(GeoPoint.Origin, GeoVector.ZAxis);
            Shell shell = Make3D.MakeHelicalSolid(MakeTriangle(), axis, pitch, extrHeight, 0.0, true);
            Assert.IsNotNull(shell, "the faces must sew into a single shell");
            Assert.AreEqual(0, shell.OpenEdges.Length, "the shell must be closed");
            AssertCylindricalFacesAreOnTheSweep(shell, axis, pitch, expectedCylFaces);
        }

        [TestMethod]
        public void OneTurn_IsClosedAndOnTheSweep()
        {
            AssertSweep(6.0, 6.0, 2);
        }

        [TestMethod]
        public void TwoTurns_IsClosedAndOnTheSweep()
        {
            AssertSweep(6.0, 12.0, 4);
        }

        [TestMethod]
        public void HalfTurn_IsClosedAndOnTheSweep()
        {
            AssertSweep(6.0, 3.0, 1);
        }

        [TestMethod]
        public void OneAndAHalfTurns_IsClosedAndOnTheSweep()
        {
            AssertSweep(2.0, 3.0, 3);
        }

        [TestMethod]
        public void NegativePitch_IsClosedAndOnTheSweep()
        {
            Axis axis = new Axis(GeoPoint.Origin, GeoVector.ZAxis);
            Shell shell = Make3D.MakeHelicalSolid(MakeTriangle(), axis, -6.0, -6.0, 0.0, true);
            Assert.IsNotNull(shell);
            Assert.AreEqual(0, shell.OpenEdges.Length);
            AssertCylindricalFacesAreOnTheSweep(shell, axis, -6.0, 2);
        }

        [TestMethod]
        public void AxisLocationOffOrigin_IsClosedAndOnTheSweep()
        {   // the axis line is the same, only its location is different
            Axis axis = new Axis(new GeoPoint(0, 0, 5), GeoVector.ZAxis);
            Shell shell = Make3D.MakeHelicalSolid(MakeTriangle(), axis, 6.0, 6.0, 0.0, true);
            Assert.IsNotNull(shell);
            Assert.AreEqual(0, shell.OpenEdges.Length);
        }

        [TestMethod]
        public void WithAxialOffset_IsClosedAndOnTheSweep()
        {   // extrOffset != 0 used to produce no shell at all: lid0 is already rotated by vmin and lifted by
            // extrOffset, so the sweep has to start at 0 and not at vmin
            Axis axis = new Axis(GeoPoint.Origin, GeoVector.ZAxis);
            Shell shell = Make3D.MakeHelicalSolid(MakeTriangle(), axis, 6.0, 6.0, 2.5, true);
            Assert.IsNotNull(shell, "extrOffset != 0 must also produce a closed shell");
            Assert.AreEqual(0, shell.OpenEdges.Length, "the shell must be closed");
            AssertCylindricalFacesAreOnTheSweep(shell, axis, 6.0, 2);
            // the profile covers z in [0,4], the sweep starts at the height 2.5 and rises by 6
            BoundingBox bb = shell.GetExtent(0.001);
            Assert.AreEqual(2.5, bb.Zmin, 0.05);
            Assert.AreEqual(12.5, bb.Zmax, 0.05);
        }

        [TestMethod]
        public void AxialOffset_DoesNotChangeTheVolume()
        {   // an offset is a pure screw motion of the same body
            Axis axis = new Axis(GeoPoint.Origin, GeoVector.ZAxis);
            double reference = double.NaN;
            foreach (double extrOffset in new double[] { 0.0, 1.5, 3.0, 4.5 })
            {
                Shell shell = Make3D.MakeHelicalSolid(MakeTriangle(), axis, 6.0, 6.0, extrOffset, true);
                Assert.IsNotNull(shell, "extrOffset " + extrOffset);
                Assert.AreEqual(0, shell.OpenEdges.Length, "extrOffset " + extrOffset);
                double volume = shell.Volume(0.05);
                if (double.IsNaN(reference)) reference = volume;
                else Assert.AreEqual(reference, volume, 0.001 * reference, "extrOffset " + extrOffset);
            }
        }

        [TestMethod]
        public void NonNormalizedAxisDirection_GivesTheSameSolid()
        {   // extrOffset and extrHeight are distances along the axis, so the length of the direction must not matter
            foreach (double length in new double[] { 1.0, 2.0, 0.25 })
            {
                Axis axis = new Axis(GeoPoint.Origin, length * GeoVector.ZAxis);
                Shell shell = Make3D.MakeHelicalSolid(MakeTriangle(), axis, 6.0, 6.0, 2.5, true);
                Assert.IsNotNull(shell, "|direction| = " + length);
                Assert.AreEqual(0, shell.OpenEdges.Length, "|direction| = " + length);
                BoundingBox bb = shell.GetExtent(0.001);
                Assert.AreEqual(2.5, bb.Zmin, 0.05, "|direction| = " + length);
                Assert.AreEqual(12.5, bb.Zmax, 0.05, "|direction| = " + length);
            }
        }

        [TestMethod]
        public void OneTurn_VolumeMatchesPappus()
        {   // the axial part of a screw motion lies inside the profile plane and adds no volume, so
            // V = area * radius of the centroid * swept angle
            Axis axis = new Axis(GeoPoint.Origin, GeoVector.ZAxis);
            Shell shell = Make3D.MakeHelicalSolid(MakeTriangle(), axis, 6.0, 6.0, 0.0, true);
            Assert.IsNotNull(shell);
            double expected = 8.0 * (34.0 / 3.0) * 2 * Math.PI;
            Assert.AreEqual(expected, shell.Volume(0.05), 0.01 * expected);
        }
    }
}
