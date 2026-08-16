using CADability.GeoObject;
using System;

namespace CADability.Tests
{
    [TestClass]
    public class HelicalSweepSurfaceTests
    {
        /// <summary>
        /// A helical sweep around the z-axis with a profile which is a line from (10,0,zOffset) to (20,0,zOffset).
        /// With zOffset != 0 the profile does not lie in the plane of the axis location.
        /// </summary>
        private static HelicalSweepSurface MakeSurface(double zOffset, double pitch)
        {
            Line profile = Line.TwoPoints(new GeoPoint(10, 0, zOffset), new GeoPoint(20, 0, zOffset));
            return new HelicalSweepSurface(profile, pitch, new Axis(GeoPoint.Origin, GeoVector.ZAxis));
        }

        private static void AssertFixedUMatchesSurface(HelicalSweepSurface surface, double u, double vmin, double vmax)
        {
            ICurve fu = surface.FixedU(u, vmin, vmax);
            for (int i = 0; i <= 10; ++i)
            {
                double t = i / 10.0;
                double v = vmin + t * (vmax - vmin);
                GeoPoint onSurface = surface.PointAt(new GeoPoint2D(u, v));
                GeoPoint onCurve = fu.PointAt(t);
                Assert.IsTrue((onSurface | onCurve) < 1e-8,
                    $"u={u} v={v}: surface {onSurface} != curve {onCurve} (distance {(onSurface | onCurve)})");
            }
        }

        [TestMethod]
        public void FixedU_ProfileInAxisPlane()
        {
            HelicalSweepSurface s = MakeSurface(0.0, 5.0);
            AssertFixedUMatchesSurface(s, 0.0, 0.0, 4 * Math.PI);
            AssertFixedUMatchesSurface(s, 0.5, 0.0, 4 * Math.PI);
        }

        [TestMethod]
        public void FixedU_ProfileOffsetAlongTheAxis()
        {   // the profile is 2.5 (half a pitch) above the axis location
            HelicalSweepSurface s = MakeSurface(2.5, 5.0);
            AssertFixedUMatchesSurface(s, 0.0, 0.0, 4 * Math.PI);
            AssertFixedUMatchesSurface(s, 0.5, 0.0, 4 * Math.PI);
        }

        [TestMethod]
        public void FixedU_StartingAtANonZeroV()
        {
            HelicalSweepSurface s = MakeSurface(2.5, 5.0);
            AssertFixedUMatchesSurface(s, 0.3, 1.0, 1.0 + 4 * Math.PI);
            AssertFixedUMatchesSurface(s, 0.3, -2.0, 3.0);
        }

        [TestMethod]
        public void FixedU_NegativePitch()
        {
            HelicalSweepSurface s = MakeSurface(1.0, -5.0);
            AssertFixedUMatchesSurface(s, 0.7, 0.5, 0.5 + 3 * Math.PI);
        }

        [TestMethod]
        public void FixedU_TiltedAxis()
        {
            Line profile = Line.TwoPoints(new GeoPoint(10, 0, 3), new GeoPoint(20, 0, 3));
            HelicalSweepSurface s = new HelicalSweepSurface(profile, 5.0, new Axis(new GeoPoint(1, 2, 3), new GeoVector(1, 1, 1)));
            AssertFixedUMatchesSurface(s, 0.4, 0.7, 0.7 + 4 * Math.PI);
        }
    }
}
