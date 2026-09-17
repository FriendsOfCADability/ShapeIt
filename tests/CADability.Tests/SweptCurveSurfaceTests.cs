using CADability;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Linq;

namespace CADability.Tests
{
    /// <summary>
    /// The general swept surface. Two kinds of test: against a body whose equation is known - a circle swept
    /// along a circle is a torus, a circle swept along a line is a cylinder, and both can be checked to the
    /// last digit - and against central differences, which is what catches a derivative that looks plausible
    /// but is not the derivative of PointAt.
    /// <para>
    /// These are deliberately unit tests on the surface and not the RPC cases: a sweep through solid.sweep
    /// runs over eight stages and a failure anywhere looks the same. When one of these fails, the surface is
    /// wrong.
    /// </para>
    /// </summary>
    [TestClass]
    public class SweptCurveSurfaceTests
    {
        private const double MajorRadius = 30.0;
        private const double MinorRadius = 8.0;

        // ------------------------------------------------------------------------------------- fixtures --

        /// <summary>The spine of the torus: a circle of radius 30 in the xy plane.</summary>
        private static Ellipse MajorCircle()
        {
            Ellipse circle = Ellipse.Construct();
            circle.SetCirclePlaneCenterRadius(Plane.XYPlane, GeoPoint.Origin, MajorRadius);
            return circle;
        }

        /// <summary>
        /// The profile: a circle of radius 8 around the start of the spine, in the plane through it that
        /// contains the axis - the xz plane here. That is where a torus wants its generating circle.
        /// </summary>
        private static Ellipse MinorCircle()
        {
            Ellipse circle = Ellipse.Construct();
            Plane plane = new Plane(new GeoPoint(MajorRadius, 0.0, 0.0), GeoVector.XAxis, GeoVector.ZAxis);
            circle.SetCirclePlaneCenterRadius(plane, new GeoPoint(MajorRadius, 0.0, 0.0), MinorRadius);
            return circle;
        }

        private static SweptCurveSurface Torus() => new SweptCurveSurface(MinorCircle(), MajorCircle());

        /// <summary>The distance of a point from the circle of radius 30 in the xy plane - 8 on the torus.</summary>
        private static double DistanceFromTubeAxis(GeoPoint p)
        {
            double rho = Math.Sqrt(p.x * p.x + p.y * p.y);
            return Math.Sqrt((rho - MajorRadius) * (rho - MajorRadius) + p.z * p.z);
        }

        /// <summary>A profile and a spine that are both splines, so nothing can be exact by accident.</summary>
        private static SweptCurveSurface AwkwardSweep()
        {
            BSpline profile = BSpline.Construct();
            profile.ThroughPoints(new GeoPoint[]
            {
                new GeoPoint(0.0, -4.0, 0.0), new GeoPoint(0.0, -1.0, 3.0), new GeoPoint(0.0, 2.0, 1.0),
                new GeoPoint(0.0, 4.0, 4.0)
            }, 3, false);
            BSpline spine = BSpline.Construct();
            spine.ThroughPoints(new GeoPoint[]
            {
                new GeoPoint(0.0, 0.0, 0.0), new GeoPoint(10.0, 3.0, 1.0), new GeoPoint(20.0, -2.0, 4.0),
                new GeoPoint(30.0, 4.0, 2.0), new GeoPoint(40.0, 0.0, 5.0)
            }, 3, false);
            return new SweptCurveSurface(profile, spine);
        }

        // --------------------------------------------------------------------- against the known bodies --

        [TestMethod]
        public void a_circle_swept_along_a_circle_is_the_exact_torus()
        {
            SweptCurveSurface torus = Torus();
            double worst = 0.0;
            for (int i = 0; i <= 24; i++)
            {
                for (int j = 0; j <= 24; j++)
                {
                    GeoPoint2D uv = new GeoPoint2D(i / 24.0, j / 24.0);
                    worst = Math.Max(worst, Math.Abs(DistanceFromTubeAxis(torus.PointAt(uv)) - MinorRadius));
                }
            }
            Assert.IsTrue(worst < 1e-9, $"the surface is {worst:E3} away from the exact torus");
        }

        [TestMethod]
        public void the_normal_of_the_torus_points_away_from_the_tube_axis()
        {
            SweptCurveSurface torus = Torus();
            for (int i = 0; i <= 8; i++)
            {
                for (int j = 0; j <= 8; j++)
                {
                    GeoPoint2D uv = new GeoPoint2D(i / 8.0, j / 8.0);
                    GeoPoint p = torus.PointAt(uv);
                    // the point of the tube axis this one belongs to
                    double rho = Math.Sqrt(p.x * p.x + p.y * p.y);
                    GeoPoint onAxis = new GeoPoint(MajorRadius * p.x / rho, MajorRadius * p.y / rho, 0.0);
                    GeoVector expected = (p - onAxis).Normalized;
                    GeoVector normal = torus.GetNormal(uv).Normalized;
                    Assert.IsTrue(Math.Abs(Math.Abs(normal * expected) - 1.0) < 1e-8,
                        $"at {uv} the normal {normal} is not radial, expected {expected}");
                }
            }
        }

        [TestMethod]
        public void a_circle_swept_along_a_line_is_the_exact_cylinder()
        {
            Ellipse profile = Ellipse.Construct();
            profile.SetCirclePlaneCenterRadius(Plane.XYPlane, GeoPoint.Origin, 5.0);
            Line spine = Line.TwoPoints(GeoPoint.Origin, new GeoPoint(0.0, 0.0, 20.0));
            SweptCurveSurface cylinder = new SweptCurveSurface(profile, spine);
            for (int i = 0; i <= 12; i++)
            {
                for (int j = 0; j <= 12; j++)
                {
                    GeoPoint2D uv = new GeoPoint2D(i / 12.0, j / 12.0);
                    GeoPoint p = cylinder.PointAt(uv);
                    Assert.AreEqual(5.0, Math.Sqrt(p.x * p.x + p.y * p.y), 1e-9, $"radius at {uv}");
                    Assert.AreEqual(20.0 * uv.y, p.z, 1e-9, $"height at {uv}");
                }
            }
        }

        // ------------------------------------------------------------------- the derivatives are exact --

        [TestMethod]
        public void the_first_derivatives_match_central_differences()
        {
            foreach ((string what, SweptCurveSurface surface) in EverySurface())
                ForEveryInnerParameter(surface, (uv) => AssertFirstDerivatives(what, surface, uv, 1e-6));
        }

        [TestMethod]
        public void the_second_derivatives_match_central_differences()
        {
            foreach ((string what, SweptCurveSurface surface) in EverySurface())
                ForEveryInnerParameter(surface, (uv) => AssertSecondDerivatives(what, surface, uv, 1e-4));
        }

        [TestMethod]
        public void derivative_at_agrees_with_the_single_accessors()
        {
            foreach ((string what, SweptCurveSurface surface) in EverySurface())
            {
                ForEveryInnerParameter(surface, (uv) =>
                {
                    surface.Derivative2At(uv, out GeoPoint location, out GeoVector du, out GeoVector dv,
                                          out GeoVector _, out GeoVector _, out GeoVector _);
                    Assert.IsTrue((location | surface.PointAt(uv)) < 1e-9, $"{what} at {uv}: location");
                    AssertSameVector($"{what} at {uv}: du", surface.UDirection(uv), du, 1e-9);
                    AssertSameVector($"{what} at {uv}: dv", surface.VDirection(uv), dv, 1e-9);
                });
            }
        }

        // ------------------------------------------------------------------------------ the fixed curves --

        [TestMethod]
        public void fixed_u_and_fixed_v_lie_on_the_surface()
        {
            foreach ((string what, SweptCurveSurface surface) in EverySurface())
            {
                for (int i = 1; i < 5; i++)
                {
                    double at = i / 5.0;
                    ICurve fixedU = surface.FixedU(at, 0.0, 1.0);
                    ICurve fixedV = surface.FixedV(at, 0.0, 1.0);
                    for (int j = 0; j <= 10; j++)
                    {
                        double t = j / 10.0;
                        Assert.IsTrue((fixedU.PointAt(t) | surface.PointAt(new GeoPoint2D(at, t))) < 1e-8,
                            $"{what}: FixedU({at}) at {t} is not on the surface");
                        Assert.IsTrue((fixedV.PointAt(t) | surface.PointAt(new GeoPoint2D(t, at))) < 1e-8,
                            $"{what}: FixedV({at}) at {t} is not on the surface");
                    }
                }
            }
        }

        [TestMethod]
        public void the_fixed_u_curve_has_the_direction_of_the_surface()
        {
            SweptCurveSurface surface = AwkwardSweep();
            ICurve fixedU = surface.FixedU(0.4, 0.0, 1.0);
            for (int j = 1; j < 10; j++)
            {
                double t = j / 10.0;
                AssertSameDirection($"FixedU direction at {t}",
                    surface.VDirection(new GeoPoint2D(0.4, t)), fixedU.DirectionAt(t));
            }
        }

        // ----------------------------------------------------------------------------------- PositionOf --

        [TestMethod]
        public void position_of_finds_back_to_the_parameters()
        {
            foreach ((string what, SweptCurveSurface surface) in EverySurface())
            {
                ForEveryInnerParameter(surface, (uv) =>
                {
                    GeoPoint p = surface.PointAt(uv);
                    GeoPoint2D found = surface.PositionOf(p);
                    Assert.IsTrue((surface.PointAt(found) | p) < 1e-6,
                        $"{what}: PositionOf({uv}) gave {found}, which is {(surface.PointAt(found) | p):E3} away");
                });
            }
        }

        // ---------------------------------------------------------------------------------- periodicity --

        [TestMethod]
        public void periodicity_is_decided_by_the_geometry_and_not_by_a_flag()
        {
            // The trap this pins: BSpline.IsClosed returns the "periodic" FLAG. A clamped nine pole NURBS
            // circle - the classic exact circle, last pole repeating the first - answers false to it although
            // it closes perfectly. Asking the flag would make the torus below non periodic in both directions.
            BSpline profile = NurbsCircle(new Plane(new GeoPoint(MajorRadius, 0, 0), GeoVector.XAxis, GeoVector.ZAxis), MinorRadius);
            BSpline spine = NurbsCircle(Plane.XYPlane, MajorRadius);
            Assert.IsFalse(((ICurve)profile).IsClosed, "the premise of this test: the flag says not closed");
            Assert.IsFalse(((ICurve)spine).IsClosed);
            Assert.IsTrue((((ICurve)profile).StartPoint | ((ICurve)profile).EndPoint) < Precision.eps, "...while the geometry closes");

            SweptCurveSurface torus = new SweptCurveSurface(profile, spine);
            Assert.IsTrue(torus.IsUPeriodic, "the profile closes, so u is periodic");
            Assert.IsTrue(torus.IsVPeriodic, "the spine closes and the frame comes back, so v is periodic");
            Assert.AreEqual(1.0, torus.UPeriod, 1e-12);
            Assert.AreEqual(1.0, torus.VPeriod, 1e-12);
        }

        [TestMethod]
        public void an_open_sweep_is_periodic_in_neither_direction()
        {
            SweptCurveSurface surface = AwkwardSweep();
            Assert.IsFalse(surface.IsUPeriodic);
            Assert.IsFalse(surface.IsVPeriodic);
            Assert.AreEqual(0.0, surface.UPeriod, 1e-12);
            Assert.AreEqual(0.0, surface.VPeriod, 1e-12);
        }

        [TestMethod]
        public void a_closed_spine_alone_does_not_make_the_surface_periodic()
        {
            // The spine closes, but "fixed" keeps the profile pointing the same way all along, so the surface
            // does meet itself - while a twisting frame would not. This is the case the geometric test has to
            // get right rather than just looking at the spine.
            SweptCurveSurface following = new SweptCurveSurface(MinorCircle(), MajorCircle(), SweepOrientation.Follow);
            SweptCurveSurface fixedFrame = new SweptCurveSurface(MinorCircle(), MajorCircle(), SweepOrientation.Fixed);
            Assert.IsTrue(following.IsVPeriodic);
            Assert.IsTrue(fixedFrame.IsVPeriodic, "a frame that never rotates comes back to itself trivially");
            // and they are different surfaces: "fixed" only translates the profile round the circle. Taken
            // over the whole patch rather than at one parameter - at u = 0.25 the profile point happens to sit
            // exactly over the spine, where rotating and translating give the same answer.
            double biggest = 0.0;
            for (int i = 0; i <= 8; i++)
                for (int j = 1; j <= 8; j++)
                {
                    GeoPoint2D uv = new GeoPoint2D(i / 8.0, j / 8.0);
                    biggest = Math.Max(biggest, following.PointAt(uv) | fixedFrame.PointAt(uv));
                }
            Assert.IsTrue(biggest > 1.0,
                $"follow and fixed must not produce the same surface on a bent spine, but they differ by at "
                + $"most {biggest:F6}");
        }

        [TestMethod]
        public void fixed_and_follow_agree_at_the_start_of_the_sweep()
        {
            SweptCurveSurface following = new SweptCurveSurface(MinorCircle(), MajorCircle(), SweepOrientation.Follow);
            SweptCurveSurface fixedFrame = new SweptCurveSurface(MinorCircle(), MajorCircle(), SweepOrientation.Fixed);
            for (int i = 0; i <= 8; i++)
            {
                GeoPoint2D uv = new GeoPoint2D(i / 8.0, 0.0);
                Assert.IsTrue((following.PointAt(uv) | fixedFrame.PointAt(uv)) < 1e-9,
                    $"at v = 0 both laws have to place the profile identically, they differ at {uv}");
            }
        }

        [TestMethod]
        public void a_fixed_u_curve_does_not_move_when_the_surface_is_changed_afterwards()
        {
            // ISurfaceImpl lets a surface be changed in place, and by the time that happens the curves it
            // produced have long become edges of a face. A curve that read the surface it came from would move
            // with it: measured before FixedU started copying the surface, ReverseOrientation displaced the
            // already built edges by the extent of the profile, and the resulting shell had edges that no
            // longer met their own vertices - which is exactly what Face.CheckConsistency looks for.
            SweptCurveSurface surface = AwkwardSweep();
            ICurve fixedU = surface.FixedU(0.3, 0.0, 1.0);
            GeoPoint[] before = Enumerable.Range(0, 5).Select(i => fixedU.PointAt(i / 4.0)).ToArray();

            surface.ReverseOrientation();
            for (int i = 0; i < before.Length; i++)
                Assert.IsTrue((fixedU.PointAt(i / 4.0) | before[i]) < 1e-12,
                    $"ReverseOrientation moved the curve at {i / 4.0}");

            surface.Modify(ModOp.Translate(new GeoVector(100.0, 0.0, 0.0)));
            for (int i = 0; i < before.Length; i++)
                Assert.IsTrue((fixedU.PointAt(i / 4.0) | before[i]) < 1e-12,
                    $"Modify moved the curve at {i / 4.0}");
        }

        // -------------------------------------------------------------------- copying and serialization --

        [TestMethod]
        public void clone_and_get_modified_keep_the_geometry()
        {
            SweptCurveSurface surface = AwkwardSweep();
            ISurface clone = surface.Clone();
            ModOp move = ModOp.Translate(new GeoVector(1.0, 2.0, 3.0))
                       * ModOp.Rotate(GeoPoint.Origin, GeoVector.ZAxis, new SweepAngle(0.7));
            ISurface moved = surface.GetModified(move);
            ForEveryInnerParameter(surface, (uv) =>
            {
                Assert.IsTrue((clone.PointAt(uv) | surface.PointAt(uv)) < 1e-9, $"clone differs at {uv}");
                Assert.IsTrue((moved.PointAt(uv) | move * surface.PointAt(uv)) < 1e-9, $"modified differs at {uv}");
            });
        }

        [TestMethod]
        public void the_surface_survives_a_json_round_trip()
        {
            SweptCurveSurface surface = AwkwardSweep();
            object read = JsonSerialize.FromString(JsonSerialize.ToString(surface));
            Assert.IsInstanceOfType(read, typeof(SweptCurveSurface));
            SweptCurveSurface restored = (SweptCurveSurface)read;
            ForEveryInnerParameter(surface, (uv) =>
            {
                Assert.IsTrue((restored.PointAt(uv) | surface.PointAt(uv)) < 1e-9, $"differs at {uv} after the round trip");
                AssertSameVector($"u direction at {uv}", surface.UDirection(uv), restored.UDirection(uv), 1e-9);
                AssertSameVector($"v direction at {uv}", surface.VDirection(uv), restored.VDirection(uv), 1e-9);
            });
        }

        [TestMethod]
        public void the_fixed_u_curve_survives_a_json_round_trip()
        {
            // FixedUCurve is written on its own whenever it has become an edge of a face, so it has to come
            // back from a file by itself - together with the surface it reads its points from.
            SweptCurveSurface surface = AwkwardSweep();
            ICurve curve = surface.FixedU(0.3, 0.2, 0.8);
            Assert.IsInstanceOfType(curve, typeof(SweptCurveSurface.FixedUCurve),
                "the test needs the curve class itself, not something FixedU simplified into");

            object read = JsonSerialize.FromString(JsonSerialize.ToString(curve));
            Assert.IsInstanceOfType(read, typeof(SweptCurveSurface.FixedUCurve),
                "reading has to produce the curve again. A JsonProxyType or an exception here means the "
                + "class is being serialized through a route it cannot be read back from.");

            ICurve restored = (ICurve)read;
            for (int i = 0; i <= 8; i++)
            {
                double position = i / 8.0;
                Assert.IsTrue((restored.PointAt(position) | curve.PointAt(position)) < 1e-9,
                    $"the curve differs at {position} after the round trip");
                AssertSameVector($"direction at {position}", curve.DirectionAt(position),
                                 restored.DirectionAt(position), 1e-9);
            }
        }

        [TestMethod]
        public void reverse_orientation_turns_the_normal_round_and_relabels_u()
        {
            SweptCurveSurface surface = AwkwardSweep();
            GeoPoint2D uv = new GeoPoint2D(0.3, 0.6);
            GeoPoint before = surface.PointAt(uv);
            GeoVector normalBefore = surface.GetNormal(uv).Normalized;

            ModOp2D toNew = surface.ReverseOrientation();
            GeoPoint2D moved = toNew * uv;
            Assert.IsTrue((surface.PointAt(moved) | before) < 1e-8,
                "the same point of space has to be reachable under the new parametrisation");
            AssertSameVector("the normal has to point the other way",
                             -normalBefore, surface.GetNormal(moved).Normalized, 1e-7);
        }

        // ------------------------------------------------------------------------------------- helpers --

        private static IEnumerable<(string, SweptCurveSurface)> EverySurface()
        {
            yield return ("torus", Torus());
            yield return ("awkward sweep", AwkwardSweep());
            yield return ("fixed orientation", new SweptCurveSurface(MinorCircle(), MajorCircle(), SweepOrientation.Fixed));
        }

        /// <summary>
        /// The nine pole rational quadratic circle: exact, and clamped rather than periodic, which is what
        /// makes it the interesting case for the periodicity test.
        /// </summary>
        private static BSpline NurbsCircle(Plane plane, double r)
        {
            GeoPoint2D[] poles2d =
            {
                new GeoPoint2D(r, 0), new GeoPoint2D(r, r), new GeoPoint2D(0, r), new GeoPoint2D(-r, r),
                new GeoPoint2D(-r, 0), new GeoPoint2D(-r, -r), new GeoPoint2D(0, -r), new GeoPoint2D(r, -r),
                new GeoPoint2D(r, 0)
            };
            GeoPoint[] poles = new GeoPoint[poles2d.Length];
            for (int i = 0; i < poles.Length; i++) poles[i] = plane.ToGlobal(poles2d[i]);
            double w = Math.Sqrt(2.0) / 2.0;
            BSpline spline = BSpline.Construct();
            spline.SetData(2, poles, new double[] { 1, w, 1, w, 1, w, 1, w, 1 },
                           new double[] { 0, 1, 2, 3, 4 }, new int[] { 3, 2, 2, 2, 3 }, false);
            return spline;
        }

        private static void ForEveryInnerParameter(SweptCurveSurface surface, Action<GeoPoint2D> check)
        {
            for (int i = 1; i < 8; i++)
                for (int j = 1; j < 8; j++)
                    check(new GeoPoint2D(i / 8.0, j / 8.0));
        }

        private static void AssertFirstDerivatives(string what, SweptCurveSurface surface, GeoPoint2D uv, double tolerance)
        {
            const double h = 1e-6;
            GeoVector du = (surface.PointAt(new GeoPoint2D(uv.x + h, uv.y))
                          - surface.PointAt(new GeoPoint2D(uv.x - h, uv.y))) / (2.0 * h);
            GeoVector dv = (surface.PointAt(new GeoPoint2D(uv.x, uv.y + h))
                          - surface.PointAt(new GeoPoint2D(uv.x, uv.y - h))) / (2.0 * h);
            AssertClose($"{what} at {uv}: UDirection", du, surface.UDirection(uv), tolerance);
            AssertClose($"{what} at {uv}: VDirection", dv, surface.VDirection(uv), tolerance);
        }

        private static void AssertSecondDerivatives(string what, SweptCurveSurface surface, GeoPoint2D uv, double tolerance)
        {
            const double h = 1e-5;
            surface.Derivative2At(uv, out GeoPoint _, out GeoVector _, out GeoVector _,
                                  out GeoVector duu, out GeoVector dvv, out GeoVector duv);
            GeoVector duuNumeric = (surface.UDirection(new GeoPoint2D(uv.x + h, uv.y))
                                  - surface.UDirection(new GeoPoint2D(uv.x - h, uv.y))) / (2.0 * h);
            GeoVector dvvNumeric = (surface.VDirection(new GeoPoint2D(uv.x, uv.y + h))
                                  - surface.VDirection(new GeoPoint2D(uv.x, uv.y - h))) / (2.0 * h);
            GeoVector duvNumeric = (surface.UDirection(new GeoPoint2D(uv.x, uv.y + h))
                                  - surface.UDirection(new GeoPoint2D(uv.x, uv.y - h))) / (2.0 * h);
            AssertClose($"{what} at {uv}: duu", duuNumeric, duu, tolerance);
            AssertClose($"{what} at {uv}: dvv", dvvNumeric, dvv, tolerance);
            AssertClose($"{what} at {uv}: duv", duvNumeric, duv, tolerance);
        }

        private static void AssertClose(string what, GeoVector expected, GeoVector actual, double tolerance)
        {
            double scale = Math.Max(1.0, expected.Length);
            double error = (expected - actual).Length / scale;
            Assert.IsTrue(error < tolerance,
                $"{what}: expected about {expected}, got {actual} (relative difference {error:E3})");
        }

        private static void AssertSameVector(string what, GeoVector expected, GeoVector actual, double tolerance)
        {
            Assert.IsTrue((expected - actual).Length < tolerance,
                $"{what}: expected {expected}, got {actual} (difference {(expected - actual).Length:E3})");
        }

        private static void AssertSameDirection(string what, GeoVector expected, GeoVector actual)
        {
            Assert.IsTrue(Math.Abs(expected.Normalized * actual.Normalized - 1.0) < 1e-7,
                $"{what}: expected the direction of {expected}, got {actual}");
        }
    }
}
