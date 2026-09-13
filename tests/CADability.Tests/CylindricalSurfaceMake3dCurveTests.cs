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
        /// Creates the 2d curve v = a*sin(u-phi)+c, restricted to u from ustart to ustart+usweep, the way the
        /// intersection of a cylinder with a plane provides it.
        /// </summary>
        private static SineCurve2D Sine(double a, double phi, double c, double ustart, double usweep)
        {
            return new SineCurve2D(ustart - phi, usweep, new ModOp2D(1, 0, phi, 0, a, c));
        }

        /// <summary>
        /// The 3d curve must be the image of the 2d curve under the surface, with the same parametrization.
        /// </summary>
        private static ICurve AssertMatchesSurface(CylindricalSurface cyl, ICurve2D curve2d)
        {
            ICurve c3d = cyl.Make3dCurve(curve2d);
            Assert.IsNotNull(c3d);
            for (int i = 0; i <= 20; ++i)
            {
                double t = i / 20.0;
                GeoPoint onSurface = cyl.PointAt(curve2d.PointAt(t));
                GeoPoint onCurve = c3d.PointAt(t);
                Assert.IsTrue((onSurface | onCurve) < 1e-8, $"at t={t}: distance {(onSurface | onCurve)}");
            }
            return c3d;
        }
        private static ICurve AssertMatchesSurface(CylindricalSurface cyl, GeoPoint2D from, GeoPoint2D to)
        {
            return AssertMatchesSurface(cyl, new Line2D(from, to));
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
        public void SineCurve_YieldsAnEllipse()
        {   // an arc of the intersection with a slanted plane
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), Sine(1.7, 0.6, 3.0, 0.4, 2.2)), typeof(Ellipse));
            // the full intersection curve
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), Sine(1.7, 0.6, 3.0, 0.0, 2 * Math.PI)), typeof(Ellipse));
            // reversed, i.e. a negative sweep
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), Sine(1.7, 0.6, 3.0, 2.6, -2.2)), typeof(Ellipse));
            // a negative amplitude and a phase beyond 2*pi
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), Sine(-4.0, 7.5, -2.0, -1.0, 5.0)), typeof(Ellipse));
            // an almost horizontal plane, i.e. almost a circle
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), Sine(1e-4, 0.6, 3.0, 0.4, 2.2)), typeof(Ellipse));
        }

        [TestMethod]
        public void SineCurve_MirroredParametrization()
        {   // fromUnit may also reverse the u direction (slope -1), which only reverses the resulting ellipse
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), new SineCurve2D(0.4, 2.2, new ModOp2D(-1, 0, 0.6, 0, 1.7, 3.0))), typeof(Ellipse));
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), new SineCurve2D(0.4, -2.2, new ModOp2D(-1, 0, 0.6, 0, -1.7, 3.0))), typeof(Ellipse));
            Assert.IsInstanceOfType(AssertMatchesSurface(Circular(), new SineCurve2D(0.0, 2 * Math.PI, new ModOp2D(-1, 0, 0.6, 0, 1.7, 3.0))), typeof(Ellipse));
            Assert.IsInstanceOfType(AssertMatchesSurface(Elliptical(), new SineCurve2D(0.4, 2.2, new ModOp2D(-1, 0, 0.6, 0, 1.7, 3.0))), typeof(Ellipse));
            Assert.IsInstanceOfType(AssertMatchesSurface(Tilted(), new SineCurve2D(0.4, 2.2, new ModOp2D(-1, 0, 0.6, 0, 1.7, 3.0))), typeof(Ellipse));
        }

        [TestMethod]
        public void SineCurve_OnDistortedCylinders()
        {   // an elliptical and a sheared/scaled cylinder distort the ellipse, but it stays an ellipse
            Assert.IsInstanceOfType(AssertMatchesSurface(Elliptical(), Sine(1.7, 0.6, 3.0, 0.4, 2.2)), typeof(Ellipse));
            Assert.IsInstanceOfType(AssertMatchesSurface(Elliptical(), Sine(1.7, 0.6, 3.0, 0.0, 2 * Math.PI)), typeof(Ellipse));
            Assert.IsInstanceOfType(AssertMatchesSurface(Tilted(), Sine(2.5, -1.2, 1.0, 0.3, 4.0)), typeof(Ellipse));
            Assert.IsInstanceOfType(AssertMatchesSurface(LeftHanded(), Sine(2.5, -1.2, 1.0, 0.3, 4.0)), typeof(Ellipse));
        }

        [TestMethod]
        public void SineCurve_AxesArePerpendicular()
        {   // the major axis must be the longer one and perpendicular to the minor axis, also on a distorted cylinder
            foreach (CylindricalSurface cyl in new CylindricalSurface[] { Circular(), Elliptical(), Tilted(), LeftHanded() })
            {
                Ellipse elli = (Ellipse)AssertMatchesSurface(cyl, Sine(1.7, 0.6, 3.0, 0.4, 2.2));
                Assert.IsTrue(elli.MajorAxis.Length >= elli.MinorAxis.Length);
                Assert.AreEqual(0.0, elli.MajorAxis.Normalized * elli.MinorAxis.Normalized, 1e-10);
            }
        }

        [TestMethod]
        public void SineCurve_WithAnotherPeriodFallsBack()
        {   // only the period 2*pi yields a planar curve
            ICurve c3d = Circular().Make3dCurve(new SineCurve2D(0.0, 2 * Math.PI, new ModOp2D(2, 0, 0.5, 0, 1.7, 3.0)));
            Assert.IsNotNull(c3d);
            Assert.IsFalse(c3d is Ellipse);
            // a sine curve which is slanted in v is not planar either
            c3d = Circular().Make3dCurve(new SineCurve2D(0.0, 2 * Math.PI, new ModOp2D(1, 0, 0.5, 0.3, 1.7, 3.0)));
            Assert.IsNotNull(c3d);
            Assert.IsFalse(c3d is Ellipse);
        }

        [TestMethod]
        public void SineCurve_MatchesThePlaneIntersection()
        {   // the sine curve provided by GetPlaneIntersection must yield the ellipse in which the plane cuts the cylinder
            CylindricalSurface cyl = Circular();
            PlaneSurface pls = new PlaneSurface(new Plane(new GeoPoint(1, 2, 4), new GeoVector(0.3, 0.2, 1)));
            IDualSurfaceCurve[] dsc = cyl.GetPlaneIntersection(pls, 0, 2 * Math.PI, -10, 10, 0.0);
            Assert.AreEqual(1, dsc.Length);
            Assert.AreEqual(cyl, dsc[0].Surface1);
            ICurve2D c2d = dsc[0].Curve2D1; // GetCurveOnSurface would wrap it into a Curve2DAspect
            Assert.IsInstanceOfType(c2d, typeof(SineCurve2D));
            Ellipse elli = (Ellipse)AssertMatchesSurface(cyl, c2d);
            for (int i = 0; i <= 20; ++i)
            {   // the whole curve lies in the plane
                Assert.AreEqual(0.0, pls.Plane.Distance(elli.PointAt(i / 20.0)), 1e-8, "in the plane at " + i / 20.0);
            }
            // it is the same ellipse as the one GetPlaneIntersection provides (which starts at a different parameter)
            Ellipse expected = (Ellipse)dsc[0].Curve3D;
            Assert.IsTrue((elli.Center | expected.Center) < 1e-8);
            Assert.AreEqual(expected.MajorAxis.Length, elli.MajorAxis.Length, 1e-8);
            Assert.AreEqual(expected.MinorAxis.Length, elli.MinorAxis.Length, 1e-8);
        }

        [TestMethod]
        public void SineCurve_BackProjectionYieldsTheOriginalCurve()
        {
            CylindricalSurface cyl = Circular();
            SineCurve2D sc = Sine(1.7, 0.6, 3.0, 0.4, 2.2);
            ICurve c3d = cyl.Make3dCurve(sc);
            for (int i = 0; i <= 20; ++i)
            {
                double t = i / 20.0;
                GeoPoint2D uv = cyl.PositionOf(c3d.PointAt(t));
                GeoPoint2D expected = sc.PointAt(t);
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
