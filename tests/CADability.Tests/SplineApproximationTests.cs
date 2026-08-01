using CADability.Curve2D;
using CADability.GeoObject;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="BSpline2D.Approximate(Func{double, GeoPoint2D}, double, double, double, int)"/> and
    /// <see cref="BSpline.Approximate(Func{double, GeoPoint}, double, double, double, int)"/>. Both methods used to
    /// pass a curve parameter where a normalized position (0...1) is expected, so with a parameter range other than
    /// 0...1 the measured deviation was meaningless and the approximation never converged: it subdivided until the
    /// maximum number of points was reached (1000 poles and seconds of calculation time instead of a few dozen poles).
    /// The parameter range is only a reparametrization, so it must not influence the result at all.
    /// </summary>
    [TestClass]
    public class SplineApproximationTests
    {
        public TestContext TestContext { get; set; }

        // a curve which is not a spline: the shape of a plane/cone intersection in the parameter space of a cone
        private static GeoPoint2D Curve2d(double u) => new GeoPoint2D(u, 50.0 / (0.8 * Math.Cos(u - 0.3) + 1.0));
        private static GeoPoint Curve3d(double u) => new GeoPoint(20.0 * Math.Cos(u), 20.0 * Math.Sin(u), 50.0 / (0.8 * Math.Cos(u - 0.3) + 1.0));
        private const double MinPar = 1.0, MaxPar = 5.0; // the parameter range of the curves above

        [TestMethod]
        public void Approximate2dDoesNotDependOnTheParameterRange()
        {
            const double precision = 1e-4;
            // the same curve, once parameterized from 0 to 1 and once with its own parameter range
            BSpline2D unitRange = BSpline2D.Approximate(t => Curve2d(MinPar + t * (MaxPar - MinPar)), precision);
            BSpline2D ownRange = BSpline2D.Approximate(Curve2d, precision, MinPar, MaxPar);
            Assert.IsNotNull(unitRange);
            Assert.IsNotNull(ownRange);
            double deviation1 = MaxDeviation(t => unitRange.PointAt(t));
            double deviation2 = MaxDeviation(t => ownRange.PointAt(t));
            TestContext?.WriteLine($"poles: {unitRange.Poles.Length} (0...1), {ownRange.Poles.Length} ({MinPar}...{MaxPar}), deviation: {deviation1}, {deviation2}");
            Assert.AreEqual(unitRange.Poles.Length, ownRange.Poles.Length, "the parameter range must not influence the number of poles");
            Assert.AreEqual(deviation1, deviation2, precision * 1e-3, "the parameter range must not influence the resulting curve");
            Assert.IsTrue(deviation1 < 0.01, "the approximation deviates far too much from the curve");
        }

        [TestMethod]
        public void Approximate3dDoesNotDependOnTheParameterRange()
        {
            const double precision = 1e-4;
            BSpline unitRange = BSpline.Approximate(t => Curve3d(MinPar + t * (MaxPar - MinPar)), precision);
            BSpline ownRange = BSpline.Approximate(Curve3d, precision, MinPar, MaxPar);
            Assert.IsNotNull(unitRange);
            Assert.IsNotNull(ownRange);
            double deviation1 = MaxDeviation(t => (unitRange as ICurve).PointAt(t).To2D());
            double deviation2 = MaxDeviation(t => (ownRange as ICurve).PointAt(t).To2D());
            TestContext?.WriteLine($"poles: {unitRange.PoleCount} (0...1), {ownRange.PoleCount} ({MinPar}...{MaxPar})");
            Assert.AreEqual(unitRange.PoleCount, ownRange.PoleCount, "the parameter range must not influence the number of poles");
            for (int i = 0; i <= 50; i++)
            {
                double t = i / 50.0;
                Assert.AreEqual(0.0, (unitRange as ICurve).PointAt(t) | (ownRange as ICurve).PointAt(t), precision * 1e-3, "the parameter range must not influence the resulting curve");
                Assert.IsTrue(((unitRange as ICurve).PointAt(t) | Curve3d(MinPar + t * (MaxPar - MinPar))) < 0.01, "the approximation deviates far too much from the curve");
            }
        }

        [TestMethod]
        public void ApproximateRejectsEmptyParameterRange()
        {
            // an empty or reversed range must not loop forever
            Assert.IsNull(BSpline2D.Approximate(Curve2d, 1e-4, 1.0, 1.0));
            Assert.IsNull(BSpline2D.Approximate(Curve2d, 1e-4, 5.0, 1.0));
            Assert.IsNull(BSpline.Approximate(Curve3d, 1e-4, 1.0, 1.0));
            Assert.IsNull(BSpline.Approximate(Curve3d, 1e-4, 5.0, 1.0));
        }

        /// <summary>
        /// The maximum distance between the approximation (at the normalized position 0...1) and the exact 2d curve.
        /// </summary>
        private static double MaxDeviation(Func<double, GeoPoint2D> approximation)
        {
            double res = 0.0;
            for (int i = 0; i <= 200; i++)
            {
                double t = i / 200.0;
                res = Math.Max(res, approximation(t) | Curve2d(MinPar + t * (MaxPar - MinPar)));
            }
            return res;
        }
    }
}
