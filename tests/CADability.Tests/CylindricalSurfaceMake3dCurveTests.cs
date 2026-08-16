using CADability.Curve2D;
using CADability.GeoObject;
using System;

namespace CADability.Tests
{
    [TestClass]
    public class CylindricalSurfaceMake3dCurveTests
    {
        private static CylindricalSurface Circular()
        {
            return new CylindricalSurface(new GeoPoint(1, 2, 3), 5 * GeoVector.XAxis, 5 * GeoVector.YAxis, GeoVector.ZAxis);
        }
        private static CylindricalSurface Elliptical()
        {
            return new CylindricalSurface(GeoPoint.Origin, 5 * GeoVector.XAxis, 2 * GeoVector.YAxis, GeoVector.ZAxis);
        }
        private static CylindricalSurface Tilted()
        {   // a tilted cylinder whose axis is additionally scaled by 2
            GeoVector d = new GeoVector(1, 1, 1);
            d.Norm();
            Plane pl = new Plane(new GeoPoint(-2, 4, 1), d);
            return new CylindricalSurface(pl.Location, 3 * pl.DirectionX, 3 * pl.DirectionY, 2 * d);
        }
        private static CylindricalSurface LeftHanded()
        {
            return new CylindricalSurface(GeoPoint.Origin, 4 * GeoVector.XAxis, 4 * GeoVector.YAxis, -GeoVector.ZAxis);
        }

        /// <summary>
        /// The 3d curve must be the image of the 2d line under the surface, with the same parametrization.
        /// </summary>
        private static ICurve AssertMatchesSurface(CylindricalSurface cyl, GeoPoint2D from, GeoPoint2D to)
        {
            Line2D l2d = new Line2D(from, to);
            ICurve c3d = cyl.Make3dCurve(l2d);
            Assert.IsNotNull(c3d);
            for (int i = 0; i <= 20; ++i)
            {
                double t = i / 20.0;
                GeoPoint onSurface = cyl.PointAt(l2d.PointAt(t));
                GeoPoint onCurve = c3d.PointAt(t);
                Assert.IsTrue((onSurface | onCurve) < 1e-8, $"at t={t}: distance {(onSurface | onCurve)}");
            }
            return c3d;
        }

        [TestMethod]
        public void VerticalLine_StillYieldsALine()
        {
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), new GeoPoint2D(0.4, -1), new GeoPoint2D(0.4, 7)), typeof(Line));
        }

        [TestMethod]
        public void HorizontalLine_StillYieldsAnEllipse()
        {
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), new GeoPoint2D(0.4, 2), new GeoPoint2D(3.1, 2)), typeof(Ellipse));
            Assert.IsInstanceOfType(AssertMatchesSurface(Elliptical(), new GeoPoint2D(0.4, 2), new GeoPoint2D(3.1, 2)), typeof(Ellipse));
        }

        [TestMethod]
        public void SlantedLine_YieldsAHelicalCurve()
        {
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), new GeoPoint2D(0.4, 1), new GeoPoint2D(0.4 + 4 * Math.PI, 9)), typeof(HelicalCurve));
            // downwards
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), new GeoPoint2D(0.4, 9), new GeoPoint2D(0.4 + 4 * Math.PI, 1)), typeof(HelicalCurve));
            // negative u sweep
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), new GeoPoint2D(2.0, 1), new GeoPoint2D(2.0 - 3 * Math.PI, 6)), typeof(HelicalCurve));
            // starting at u == 0 and at negative v
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), new GeoPoint2D(0.0, 0), new GeoPoint2D(2 * Math.PI, 5)), typeof(HelicalCurve));
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), new GeoPoint2D(-1.5, -4), new GeoPoint2D(3.5, -9)), typeof(HelicalCurve));
            // almost vertical, i.e. a very large pitch
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), new GeoPoint2D(1.0, 0), new GeoPoint2D(1.0 + 1e-3, 10)), typeof(HelicalCurve));
        }

        [TestMethod]
        public void SlantedLine_TiltedAndScaledAxis()
        {
            Assert.IsInstanceOfType(AssertMatchesSurface(Tilted(), new GeoPoint2D(0.7, 2), new GeoPoint2D(0.7 + 4 * Math.PI, 8)), typeof(HelicalCurve));
        }

        [TestMethod]
        public void SlantedLine_LeftHandedCylinder()
        {
            Assert.IsInstanceOfType(AssertMatchesSurface(LeftHanded(), new GeoPoint2D(0.3, 1), new GeoPoint2D(0.3 + 3 * Math.PI, 7)), typeof(HelicalCurve));
        }

        [TestMethod]
        public void SlantedLine_OnAnEllipticalCylinderFallsBack()
        {   // the image of a helix on an elliptical cylinder is not a helix, so the base implementation must be used
            ICurve c3d = Elliptical().Make3dCurve(new Line2D(new GeoPoint2D(0.4, 1), new GeoPoint2D(0.4 + 4 * Math.PI, 9)));
            Assert.IsNotNull(c3d);
            Assert.IsFalse(c3d is HelicalCurve);
        }

        [TestMethod]
        public void SlantedLine_BackProjectionYieldsTheOriginalLine()
        {
            CylindricalSurface cyl = Circular();
            Line2D l2d = new Line2D(new GeoPoint2D(0.4, 1), new GeoPoint2D(0.4 + 4 * Math.PI, 9));
            ICurve c3d = cyl.Make3dCurve(l2d);
            for (int i = 0; i <= 20; ++i)
            {
                double t = i / 20.0;
                GeoPoint2D uv = cyl.PositionOf(c3d.PointAt(t));
                GeoPoint2D expected = l2d.PointAt(t);
                double du = uv.x - expected.x;
                du -= 2 * Math.PI * Math.Round(du / (2 * Math.PI)); // u is periodic
                Assert.AreEqual(0.0, du, 1e-8, "u at " + t);
                Assert.AreEqual(expected.y, uv.y, 1e-8, "v at " + t);
            }
        }

        [TestMethod]
        public void ToBSpline_KeepsTheRequestedPrecision()
        {
            HelicalCurve h = HelicalCurve.FromAxisStartPoint(GeoPoint.Origin, GeoVector.ZAxis, new GeoPoint(20, 0, 0), 8.0, 3.0);
            foreach (double precision in new double[] { 1e-2, 1e-4, 1e-6 })
            {
                ICurve bsp = h.ToBSpline(precision);
                Assert.IsTrue((bsp.StartPoint | h.StartPoint) < 1e-9);
                Assert.IsTrue((bsp.EndPoint | h.EndPoint) < 1e-9);
                for (int i = 0; i <= 200; ++i)
                {
                    double d = h.DistanceTo(bsp.PointAt(i / 200.0));
                    Assert.IsTrue(d < precision, $"precision {precision}: deviation {d}");
                }
            }
        }
    }
}
