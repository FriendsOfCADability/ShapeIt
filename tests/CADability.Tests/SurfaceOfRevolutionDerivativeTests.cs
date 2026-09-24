using CADability.Curve2D;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;

namespace CADability.Tests
{
    /// <summary>
    /// The derivatives of <see cref="SurfaceOfRevolution"/> must be the derivatives of its
    /// <see cref="SurfaceOfRevolution.PointAt"/>. The v parameter is the natural parameter of the profile curve,
    /// while <see cref="ICurve.DirectionAt"/> differentiates by the normalized position, so VDirection needs the
    /// factor d(position)/d(parameter). It used to be missing: invisible for a line, whose parameter is its
    /// position, but wrong by the length of the knot range for a BSpline and by the sweep for an arc. Every
    /// Newton iteration and every surface integral built on these derivatives inherited the error
    /// (SurfaceOfRevolution1 of the BRep regression cases: 24 percent).
    /// </summary>
    [TestClass]
    public class SurfaceOfRevolutionDerivativeTests
    {
        private static readonly GeoPoint AxisLocation = new GeoPoint(3, -2, 7);
        // deliberately not normalized: the regression file carries an axis of length 56
        private static readonly GeoVector AxisDirection = 4.0 * new GeoVector(0.3, -1.7, 0.5);

        /// <summary>
        /// Profile curves with the different relations between parameter and position.
        /// </summary>
        private static IEnumerable<(string what, ICurve curve)> Profiles()
        {
            Plane meridian = new Plane(AxisLocation, AxisDirection.Normalized ^ GeoVector.XAxis); // contains the axis
            GeoVector radial = meridian.Normal ^ AxisDirection.Normalized;

            Line line = Line.TwoPoints(AxisLocation + 5 * radial, AxisLocation + 9 * radial + 0.5 * AxisDirection);
            yield return ("line: parameter == position", line);

            BSpline spline = BSpline.Construct();
            spline.SetData(3, new GeoPoint[]
            {
                AxisLocation + 5 * radial, AxisLocation + 8 * radial + 0.2 * AxisDirection,
                AxisLocation + 6 * radial + 0.5 * AxisDirection, AxisLocation + 9 * radial + 0.7 * AxisDirection,
                AxisLocation + 7 * radial + AxisDirection
            }, null, new double[] { 2, 3.5, 5 }, new int[] { 4, 1, 4 }, false);
            yield return ("BSpline with knots from 2 to 5", spline);

            BSpline circle = RationalCircle(new Plane(AxisLocation + 12 * radial, radial, AxisDirection.Normalized), 4.0);
            yield return ("rational circle with knots from 0 to 4", circle);

            Ellipse arc = Ellipse.Construct();
            arc.SetArcPlaneCenterRadiusAngles(new Plane(AxisLocation, radial, AxisDirection.Normalized),
                AxisLocation + 10 * radial, 3.0, 0.4, 2.5);
            yield return ("circular arc, sweep 2.5", arc);

            Ellipse reversedArc = Ellipse.Construct();
            reversedArc.SetArcPlaneCenterRadiusAngles(new Plane(AxisLocation, radial, AxisDirection.Normalized),
                AxisLocation + 10 * radial, 3.0, 2.9, -2.5);
            yield return ("circular arc, sweep -2.5", reversedArc);

            BSpline skew = BSpline.Construct(); // not in a plane with the axis
            skew.SetData(3, new GeoPoint[]
            {
                new GeoPoint(10, 1, 2), new GeoPoint(12, 5, 3), new GeoPoint(9, 8, 6), new GeoPoint(13, 11, 4)
            }, null, new double[] { -1, 0.7 }, new int[] { 4, 4 }, false);
            yield return ("3d BSpline with knots from -1 to 0.7", skew);
        }

        private static BSpline RationalCircle(Plane plane, double r)
        {
            GeoPoint2D[] poles2d =
            {
                new GeoPoint2D(r, 0), new GeoPoint2D(r, r), new GeoPoint2D(0, r), new GeoPoint2D(-r, r),
                new GeoPoint2D(-r, 0), new GeoPoint2D(-r, -r), new GeoPoint2D(0, -r), new GeoPoint2D(r, -r),
                new GeoPoint2D(r, 0)
            };
            GeoPoint[] poles = poles2d.Select(p => plane.ToGlobal(p)).ToArray();
            double w = Math.Sqrt(2.0) / 2.0;
            BSpline spline = BSpline.Construct();
            spline.SetData(2, poles, new double[] { 1, w, 1, w, 1, w, 1, w, 1 },
                           new double[] { 0, 1, 2, 3, 4 }, new int[] { 3, 2, 2, 2, 3 }, false);
            return spline;
        }

        /// <summary>
        /// Sample points strictly inside the v range (central differences must not leave the curve) and away
        /// from the knots (a cubic is only C2 there, which is fine for the first derivative but not for a
        /// clean difference of the second).
        /// </summary>
        private static IEnumerable<GeoPoint2D> Samples(double vmin, double vmax)
        {
            double[] us = { -2.0, 0.0, 0.7, 2.1, 4.4 };
            double[] positions = { 0.13, 0.37, 0.61, 0.88 };
            foreach (double u in us)
                foreach (double pos in positions)
                    yield return new GeoPoint2D(u, vmin + pos * (vmax - vmin));
        }

        private static (GeoVector du, GeoVector dv) Differenced(ISurface surface, GeoPoint2D uv, double hu, double hv)
        {
            GeoVector du = (1.0 / (2 * hu)) * (surface.PointAt(new GeoPoint2D(uv.x + hu, uv.y)) - surface.PointAt(new GeoPoint2D(uv.x - hu, uv.y)));
            GeoVector dv = (1.0 / (2 * hv)) * (surface.PointAt(new GeoPoint2D(uv.x, uv.y + hv)) - surface.PointAt(new GeoPoint2D(uv.x, uv.y - hv)));
            return (du, dv);
        }

        private static void AssertClose(GeoVector expected, GeoVector actual, double relTol, string message)
        {
            double err = (expected - actual).Length / Math.Max(expected.Length, 1e-12);
            Assert.IsTrue(err < relTol, $"{message}: expected {expected}, got {actual}, relative error {err:E2}");
        }

        /// <summary>
        /// Compares UDirection, VDirection and DerivativeAt with central differences of PointAt, and, if asked,
        /// Derivative2At with central differences of DerivativeAt.
        /// </summary>
        private static void CheckDerivatives(string what, ISurface surface, double vmin, double vmax, bool secondDerivatives)
        {
            double hu = 1e-5;
            double hv = 1e-5 * (vmax - vmin);
            foreach (GeoPoint2D uv in Samples(vmin, vmax))
            {
                (GeoVector fdu, GeoVector fdv) = Differenced(surface, uv, hu, hv);
                string at = $"{what} at {uv}";
                AssertClose(fdu, surface.UDirection(uv), 1e-7, $"UDirection, {at}");
                AssertClose(fdv, surface.VDirection(uv), 1e-7, $"VDirection, {at}");
                surface.DerivativeAt(uv, out GeoPoint location, out GeoVector du, out GeoVector dv);
                Assert.IsTrue((location | surface.PointAt(uv)) < 1e-10, $"DerivativeAt location, {at}");
                AssertClose(fdu, du, 1e-7, $"DerivativeAt du, {at}");
                AssertClose(fdv, dv, 1e-7, $"DerivativeAt dv, {at}");
                if (!secondDerivatives) continue;

                surface.Derivative2At(uv, out GeoPoint location2, out GeoVector du2, out GeoVector dv2,
                    out GeoVector duu, out GeoVector dvv, out GeoVector duv);
                Assert.IsTrue((location2 | surface.PointAt(uv)) < 1e-9, $"Derivative2At location, {at}");
                AssertClose(fdu, du2, 1e-7, $"Derivative2At du, {at}");
                AssertClose(fdv, dv2, 1e-7, $"Derivative2At dv, {at}");
                surface.DerivativeAt(new GeoPoint2D(uv.x + hu, uv.y), out _, out GeoVector duP, out GeoVector dvuP);
                surface.DerivativeAt(new GeoPoint2D(uv.x - hu, uv.y), out _, out GeoVector duM, out GeoVector dvuM);
                surface.DerivativeAt(new GeoPoint2D(uv.x, uv.y + hv), out _, out _, out GeoVector dvP);
                surface.DerivativeAt(new GeoPoint2D(uv.x, uv.y - hv), out _, out _, out GeoVector dvM);
                AssertClose((1.0 / (2 * hu)) * (duP - duM), duu, 1e-6, $"Derivative2At duu, {at}");
                AssertClose((1.0 / (2 * hv)) * (dvP - dvM), dvv, 1e-5, $"Derivative2At dvv, {at}");
                AssertClose((1.0 / (2 * hu)) * (dvuP - dvuM), duv, 1e-6, $"Derivative2At duv, {at}");
            }
        }

        [TestMethod]
        public void the_derivatives_are_the_derivatives_of_PointAt()
        {
            foreach ((string what, ICurve curve) in Profiles())
            {
                SurfaceOfRevolution surface = new SurfaceOfRevolution(curve, AxisLocation, AxisDirection);
                CheckDerivatives(what, surface, curve.PositionToParameter(0.0), curve.PositionToParameter(1.0), true);
            }
        }

        [TestMethod]
        public void the_derivatives_survive_Modify_and_Clone()
        {
            ModOp m = ModOp.Translate(1, 2, 3) * ModOp.Rotate(new GeoVector(1, 1, 0).Normalized, SweepAngle.Deg(33)) * ModOp.Scale(1.7);
            foreach ((string what, ICurve curve) in Profiles())
            {
                ISurface surface = new SurfaceOfRevolution(curve, AxisLocation, AxisDirection).GetModified(m).Clone();
                CheckDerivatives(what + ", modified", surface, curve.PositionToParameter(0.0), curve.PositionToParameter(1.0), true);
            }
        }

        /// <summary>
        /// The old implementation: a 2d curve in the xy plane, rotated about the y axis and placed by toSurface,
        /// v running from curveStartParameter to curveEndParameter. The internal constructor always builds
        /// curveToRotate too, so the old branch is reached only with curveToRotate removed.
        /// </summary>
        private static SurfaceOfRevolution OldImplementation(ICurve2D basisCurve2D, ModOp toSurface, double vStart, double vEnd)
        {
            ConstructorInfo ctor = typeof(SurfaceOfRevolution).GetConstructor(BindingFlags.Instance | BindingFlags.NonPublic, null,
                new Type[] { typeof(ICurve2D), typeof(ModOp), typeof(double), typeof(double), typeof(double) }, null);
            Assert.IsNotNull(ctor, "internal constructor of SurfaceOfRevolution not found");
            SurfaceOfRevolution res = (SurfaceOfRevolution)ctor.Invoke(new object[] { basisCurve2D, toSurface, vStart, vEnd, 0.0 });
            FieldInfo curveToRotate = typeof(SurfaceOfRevolution).GetField("curveToRotate", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.IsNotNull(curveToRotate, "field curveToRotate not found");
            curveToRotate.SetValue(res, null);
            return res;
        }

        [TestMethod]
        public void the_old_implementation_has_consistent_derivatives_too()
        {
            ModOp toSurface = ModOp.Translate(3, -2, 7) * ModOp.Rotate(new GeoVector(1, 2, 3).Normalized, SweepAngle.Deg(37));
            (string what, ICurve2D curve)[] profiles =
            {
                ("Line2D", new Line2D(new GeoPoint2D(5, 0), new GeoPoint2D(9, 3))),
                ("Arc2D", new Arc2D(new GeoPoint2D(10, 2), 3.0, 0.4, 2.5)),
                ("BSpline2D", new BSpline2D(new GeoPoint2D[] { new GeoPoint2D(5, 0), new GeoPoint2D(8, 1), new GeoPoint2D(6, 3), new GeoPoint2D(9, 5) }, 3, false)),
            };
            foreach ((string what, ICurve2D curve) in profiles)
            {
                foreach ((double vStart, double vEnd) in new[] { (0.0, 1.0), (2.0, 5.0), (-0.3, 0.2) })
                {
                    SurfaceOfRevolution surface = OldImplementation(curve, toSurface, vStart, vEnd);
                    CheckDerivatives($"old implementation, {what}, v from {vStart} to {vEnd}", surface, vStart, vEnd, false);
                }
            }
        }

        /// <summary>
        /// Derivative2At of a surface with an arc profile relies on <see cref="ICurve.TryPointDeriv2At"/> of the
        /// Ellipse, whose second derivative carried the sweep only once instead of squared (the public overload in
        /// the minor axis component only).
        /// </summary>
        [TestMethod]
        public void the_second_derivative_of_an_elliptical_arc_is_the_derivative_of_its_first()
        {
            Ellipse arc = Ellipse.Construct();
            arc.SetEllipseArcCenterAxis(new GeoPoint(1, 2, 3), new GeoVector(5, 1, 0), 2.0 * new GeoVector(-1, 5, 2).Normalized, 0.4, 2.5);
            double h = 1e-6;
            foreach (double pos in new double[] { 0.1, 0.45, 0.8 })
            {
                (arc as ICurve).TryPointDeriv2At(pos, out GeoPoint point, out GeoVector d1, out GeoVector d2);
                arc.TryPointDeriv2At(pos, out GeoPoint pointV, out GeoVector d1V, out GeoVector d2V);
                GeoVector fd1 = (1.0 / (2 * h)) * (arc.PointAt(pos + h) - arc.PointAt(pos - h));
                GeoVector fd2 = (1.0 / (2 * h)) * (arc.DirectionAt(pos + h) - arc.DirectionAt(pos - h));
                Assert.IsTrue((point | arc.PointAt(pos)) < 1e-10 && (pointV | arc.PointAt(pos)) < 1e-10, $"point at {pos}");
                AssertClose(fd1, d1, 1e-7, $"ICurve.TryPointDeriv2At first derivative at {pos}");
                AssertClose(fd1, d1V, 1e-7, $"Ellipse.TryPointDeriv2At first derivative at {pos}");
                AssertClose(fd2, d2, 1e-6, $"ICurve.TryPointDeriv2At second derivative at {pos}");
                AssertClose(fd2, d2V, 1e-6, $"Ellipse.TryPointDeriv2At second derivative at {pos}");
            }
        }

        private static string BRepFile(string name, [CallerFilePath] string thisFile = "")
            => System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile), "Files", "BRep", name);

        /// <summary>
        /// The face that exposed the bug: a BSpline profile whose knots do not run from 0 to 1.
        /// </summary>
        [TestMethod]
        public void the_regression_case_SurfaceOfRevolution1_is_consistent()
        {
            string file = BRepFile("SurfaceOfRevolution1.cdb.json");
            Assert.IsTrue(File.Exists(file), $"test file not found: {file}");
            Project project = Project.ReadFromFile(file, "cdb");
            Assert.IsNotNull(project, "could not read project");
            List<Face> faces = project.GetActiveModel().AllObjects.OfType<Solid>()
                .SelectMany(s => s.Shells).SelectMany(sh => sh.Faces)
                .Where(f => f.Surface is SurfaceOfRevolution).ToList();
            Assert.IsTrue(faces.Count > 0, "no SurfaceOfRevolution face in the regression case");
            foreach (Face face in faces)
            {
                BoundingRect domain = face.Domain;
                double hu = 1e-5 * domain.Width;
                double hv = 1e-5 * domain.Height;
                for (int i = 1; i < 6; i++)
                {
                    for (int j = 1; j < 6; j++)
                    {
                        GeoPoint2D uv = new GeoPoint2D(domain.Left + i * domain.Width / 6, domain.Bottom + j * domain.Height / 6);
                        (GeoVector fdu, GeoVector fdv) = Differenced(face.Surface, uv, hu, hv);
                        face.Surface.DerivativeAt(uv, out _, out GeoVector du, out GeoVector dv);
                        AssertClose(fdu, du, 1e-6, $"du of face {face.GetHashCode()} at {uv}");
                        AssertClose(fdv, dv, 1e-6, $"dv of face {face.GetHashCode()} at {uv}");
                    }
                }
            }
        }
    }
}
