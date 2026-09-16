using CADability;
using CADability.GeoObject;
using System;
using System.Collections.Generic;

namespace CADability.Tests
{
    /// <summary>
    /// The moving coordinate system a swept surface is built on. Two things have to hold and nothing else
    /// matters: the three axes must be an orthonormal right handed system at every parameter, and the
    /// derivatives must really be the derivatives - which is checked against central differences of the frame
    /// itself, so a wrong formula cannot hide behind a plausible looking vector.
    /// <para>
    /// The spines used here are mostly a line, a circle and a helix, because those three deliver their
    /// derivatives in closed form; a BSpline produces everything beyond the second one numerically. The one
    /// place a BSpline is needed anyway is the second derivative of the Frenet frame: it contains a term with
    /// the derivative of the TORSION, and the torsion of a line, a circle and a helix is constant, so none of
    /// those three would ever touch that term.
    /// </para>
    /// </summary>
    [TestClass]
    public class SweepFrameTests
    {
        private const double StepForDifferences = 1e-5;

        // ------------------------------------------------------------------------------------- spines --

        private static Line StraightSpine()
            => Line.TwoPoints(new GeoPoint(1.0, 2.0, 3.0), new GeoPoint(11.0, -4.0, 8.0));

        /// <summary>A circle of radius 30 in the xy plane - a planar, non straight spine.</summary>
        private static Ellipse CircularSpine()
        {
            Ellipse circle = Ellipse.Construct();
            circle.SetCirclePlaneCenterRadius(Plane.XYPlane, GeoPoint.Origin, 30.0);
            return circle;
        }

        /// <summary>
        /// A helix of radius <paramref name="radius"/> and the given pitch, two turns. Its curvature and its
        /// torsion are constant and known: with h = pitch/(2*pi), k = r/(r^2+h^2) and w = h/(r^2+h^2).
        /// </summary>
        private static HelicalCurve HelicalSpine(double radius = 10.0, double pitch = 8.0)
        {
            HelicalCurve helix = HelicalCurve.Construct();
            helix.SetHelix(Plane.XYPlane, radius, pitch, 0.0, 4.0 * Math.PI);
            return helix;
        }

        /// <summary>
        /// A non planar BSpline whose torsion varies along the curve - measured between 5.7e-3 and 1.9e-1, a
        /// factor of 33. The only spine here that exercises the derivative of the torsion.
        /// </summary>
        private static BSpline TwistedSpine()
        {
            GeoPoint[] through =
            {
                new GeoPoint(0.0, 0.0, 0.0), new GeoPoint(10.0, 4.0, 1.0), new GeoPoint(20.0, -3.0, 5.0),
                new GeoPoint(30.0, 6.0, 2.0), new GeoPoint(40.0, -2.0, 9.0), new GeoPoint(50.0, 3.0, 3.0)
            };
            BSpline spline = BSpline.Construct();
            spline.ThroughPoints(through, 3, false);
            return spline;
        }

        private static double AxialFactor(double pitch) => pitch / (2.0 * Math.PI);

        private static double TorsionOf(ICurve spine, double u)
        {
            IReadOnlyList<GeoVector> c = spine.PointAndDerivativesAt(u, 3);
            GeoVector crossVA = c[1] ^ c[2];
            return (crossVA * c[3]) / (crossVA * crossVA);
        }

        // --------------------------------------------------------------------------- the frame itself --

        [TestMethod]
        public void axes_are_orthonormal_and_right_handed()
        {
            foreach ((string what, SweepFrame frame) in EveryFrame())
                for (int i = 0; i <= 10; i++)
                    AssertOrthonormal(what, frame, i / 10.0);
        }

        [TestMethod]
        public void first_derivative_matches_a_central_difference()
        {
            foreach ((string what, SweepFrame frame) in EveryFrame())
                for (int i = 1; i < 10; i++)
                    AssertDerivativeMatchesDifference(what, frame, i / 10.0, order: 1, tolerance: 1e-6);
        }

        [TestMethod]
        public void second_derivative_matches_a_central_difference()
        {
            foreach ((string what, SweepFrame frame) in EveryFrame())
                for (int i = 1; i < 10; i++)
                    AssertDerivativeMatchesDifference(what, frame, i / 10.0, order: 2, tolerance: 1e-4);
        }

        [TestMethod]
        public void the_location_is_the_spine_and_its_derivatives_are_the_spines()
        {
            foreach ((string what, SweepFrame frame) in EveryFrame())
            {
                for (int i = 0; i <= 10; i++)
                {
                    double u = i / 10.0;
                    IReadOnlyList<SweepAxes> axes = frame.AxesAndDerivativesAt(u, 2);
                    IReadOnlyList<GeoVector> spine = frame.Spine.PointAndDerivativesAt(u, 2);
                    for (int order = 0; order <= 2; order++)
                        AssertSameVector($"{what} at u={u}, location derivative {order}",
                                         spine[order], axes[order].Location, 1e-9);
                }
            }
        }

        // ------------------------------------------------------------------------------- the two laws --

        [TestMethod]
        public void follow_keeps_the_profile_perpendicular_to_the_spine()
        {
            foreach ((string what, SweepFrame frame) in EveryFollowingFrame())
            {
                for (int i = 0; i <= 10; i++)
                {
                    double u = i / 10.0;
                    SweepAxes axes = frame.AxesAt(u);
                    GeoVector tangent = frame.Spine.DirectionAt(u).Normalized;
                    Assert.IsTrue(Math.Abs(axes.Z * tangent - 1.0) < 1e-9,
                        $"{what} at u={u}: the z-axis {axes.Z} is not the tangent {tangent}");
                }
            }
        }

        [TestMethod]
        public void fixed_reference_keeps_the_reference_direction()
        {
            // The x-axis is the reference, projected into the plane perpendicular to the tangent. So it has to
            // stay in the plane spanned by the reference and the tangent, and on the same side of it.
            GeoVector reference = new GeoVector(0.3, -0.5, 1.0).Normalized;
            SweepFrame frame = SweepFrame.Create(HelicalSpine(), SweepOrientation.Follow, reference);
            for (int i = 0; i <= 10; i++)
            {
                double u = i / 10.0;
                SweepAxes axes = frame.AxesAt(u);
                Assert.IsTrue(Math.Abs(axes.Y * reference) < 1e-9,
                    $"at u={u} the y-axis {axes.Y} is not perpendicular to the reference {reference}");
                Assert.IsTrue(axes.X * reference > 0.0,
                    $"at u={u} the x-axis {axes.X} points away from the reference {reference}");
            }
        }

        [TestMethod]
        public void follow_on_a_planar_spine_uses_the_plane_normal()
        {
            // A circle in the xy plane: the plane normal is z, so the x-axis - the reference projected into the
            // normal plane of the tangent - must BE z everywhere, and the frame must not twist at all.
            SweepFrame frame = SweepFrame.Create(CircularSpine(), SweepOrientation.Follow);
            for (int i = 0; i <= 10; i++)
            {
                double u = i / 10.0;
                SweepAxes axes = frame.AxesAt(u);
                AssertSameVector($"x-axis at u={u}", GeoVector.ZAxis, axes.X, 1e-9);
                // and the y-axis points at the centre of the circle, which is the origin here
                GeoVector toCentre = (GeoPoint.Origin - frame.Spine.PointAt(u)).Normalized;
                Assert.IsTrue(Math.Abs(Math.Abs(axes.Y * toCentre) - 1.0) < 1e-9,
                    $"at u={u} the y-axis {axes.Y} is not radial");
            }
        }

        [TestMethod]
        public void fixed_orientation_does_not_rotate()
        {
            SweepFrame frame = SweepFrame.Create(HelicalSpine(), SweepOrientation.Fixed);
            SweepAxes start = frame.AxesAt(0.0);
            for (int i = 0; i <= 10; i++)
            {
                double u = i / 10.0;
                IReadOnlyList<SweepAxes> axes = frame.AxesAndDerivativesAt(u, 2);
                AssertSameVector($"x-axis at u={u}", start.X, axes[0].X, 1e-12);
                AssertSameVector($"y-axis at u={u}", start.Y, axes[0].Y, 1e-12);
                AssertSameVector($"z-axis at u={u}", start.Z, axes[0].Z, 1e-12);
                for (int order = 1; order <= 2; order++)
                {
                    AssertSameVector($"x-axis derivative {order} at u={u}", GeoVector.NullVector, axes[order].X, 1e-12);
                    AssertSameVector($"y-axis derivative {order} at u={u}", GeoVector.NullVector, axes[order].Y, 1e-12);
                    AssertSameVector($"z-axis derivative {order} at u={u}", GeoVector.NullVector, axes[order].Z, 1e-12);
                }
            }
        }

        [TestMethod]
        public void fixed_and_follow_agree_at_the_start_of_the_sweep()
        {
            // Both laws have to place the profile identically at u=0 and only differ further along, otherwise
            // switching the orientation would move the body.
            ICurve spine = HelicalSpine();
            SweepAxes atFixed = SweepFrame.Create(spine, SweepOrientation.Fixed).AxesAt(0.0);
            SweepAxes atFollow = SweepFrame.Create(spine, SweepOrientation.Follow).AxesAt(0.0);
            AssertSameVector("x-axis", atFollow.X, atFixed.X, 1e-12);
            AssertSameVector("y-axis", atFollow.Y, atFixed.Y, 1e-12);
            AssertSameVector("z-axis", atFollow.Z, atFixed.Z, 1e-12);
        }

        // -------------------------------------------------------------------------------- Frenet frame --

        [TestMethod]
        public void the_frenet_frame_of_a_helix_has_its_known_curvature_and_torsion()
        {
            // The regression guard for a defect this code was written to avoid: SweptCircleSurface computes the
            // torsion as c'.(c' x c'''), which is identically zero because a vector is perpendicular to any
            // cross product it appears in. A frame built that way treats every spine as if it were planar, and
            // the error is invisible on a planar one. A helix has a constant, non zero torsion, so it shows.
            double radius = 10.0, pitch = 8.0;
            double h = AxialFactor(pitch);
            double expectedCurvature = radius / (radius * radius + h * h);
            double expectedTorsion = h / (radius * radius + h * h);
            Assert.IsTrue(expectedTorsion > 1e-3, "the test helix has to have a torsion worth measuring");

            SweepFrame frame = FrenetFrameOf(HelicalSpine(radius, pitch));
            for (int i = 1; i < 10; i++)
            {
                double u = i / 10.0;
                IReadOnlyList<SweepAxes> axes = frame.AxesAndDerivativesAt(u, 1);
                double speed = frame.Spine.DirectionAt(u).Length;

                // Frenet-Serret, scaled by the speed because these are derivatives by the curve parameter:
                // T' = k*s*N and B' = -w*s*N, so the two constants can be read straight off the frame.
                double curvature = (axes[1].Z * axes[0].X) / speed;   // T'.N / s
                double torsion = -(axes[1].Y * axes[0].X) / speed;    // -B'.N / s
                Assert.IsTrue(Math.Abs(curvature - expectedCurvature) < 1e-8,
                    $"at u={u}: curvature {curvature} instead of {expectedCurvature}");
                Assert.IsTrue(Math.Abs(torsion - expectedTorsion) < 1e-6,
                    $"at u={u}: torsion {torsion} instead of {expectedTorsion}. A torsion of zero means the "
                    + "frame is using c'.(c' x c''') instead of (c' x c'').c'''");
            }
        }

        [TestMethod]
        public void the_frenet_second_derivative_holds_where_the_torsion_varies()
        {
            // The second derivative of the Frenet frame contains the derivative of the torsion, and that is the
            // one quantity on this path which is not closed form: it would need the fourth derivative of the
            // spine, so it is taken as a central difference instead. A line, a circle and a helix all have a
            // constant torsion and would leave the term at zero, which is why this one case needs a BSpline.
            BSpline spine = TwistedSpine();
            double smallest = double.MaxValue, largest = 0.0;
            for (int i = 1; i < 10; i++)
            {
                double torsion = Math.Abs(TorsionOf(spine, i / 10.0));
                smallest = Math.Min(smallest, torsion);
                largest = Math.Max(largest, torsion);
            }
            Assert.IsTrue(largest > 10.0 * smallest,
                $"the spine has to have a torsion that really varies, but it only ranges from {smallest:E3} "
                + $"to {largest:E3} - this test would then be checking nothing");

            SweepFrame frame = SweepFrame.CreateFrenet(spine);
            for (int i = 1; i < 10; i++)
                AssertDerivativeMatchesDifference("frenet along a twisted spline", frame, i / 10.0,
                                                  order: 2, tolerance: 1e-5);
        }

        [TestMethod]
        public void the_frenet_frame_refuses_a_straight_spine_instead_of_returning_nonsense()
        {
            SweepFrame frame = FrenetFrameOf(StraightSpine());
            Assert.ThrowsException<SweepFrameException>(() => frame.AxesAt(0.5));
        }

        [TestMethod]
        public void a_reference_parallel_to_the_tangent_is_refused()
        {
            // The reference direction may not lie along the spine: there is no rotation it could define then.
            Line spine = StraightSpine();
            SweepFrame frame = SweepFrame.Create(spine, SweepOrientation.Follow, spine.StartDirection);
            Assert.ThrowsException<SweepFrameException>(() => frame.AxesAt(0.5));
        }

        [TestMethod]
        public void create_prefers_a_reference_direction_over_frenet()
        {
            // Which law comes out is not a detail: the Frenet frame flips its normal at an inflection point of
            // the spine, so it has to stay the last resort. A line, a circle and even a helix must all end up
            // with a fixed reference.
            Assert.AreEqual(SweepLaw.FixedReference, SweepFrame.Create(StraightSpine(), SweepOrientation.Follow).Law);
            Assert.AreEqual(SweepLaw.FixedReference, SweepFrame.Create(CircularSpine(), SweepOrientation.Follow).Law);
            Assert.AreEqual(SweepLaw.FixedReference, SweepFrame.Create(HelicalSpine(), SweepOrientation.Follow).Law,
                "a helix is not planar, but its tangents all make the same angle with the axis, so "
                + "FindSweepNormal finds that axis and no Frenet frame is needed");
            Assert.AreEqual(SweepLaw.NoRotation, SweepFrame.Create(HelicalSpine(), SweepOrientation.Fixed).Law);
        }

        // ------------------------------------------------------------------------------------ Between --

        [TestMethod]
        public void between_maps_one_frame_onto_the_other()
        {
            foreach ((string what, SweepFrame frame) in EveryFrame())
            {
                ModOp toEnd = frame.Between(0.0, 1.0);
                SweepAxes start = frame.AxesAt(0.0), end = frame.AxesAt(1.0);
                AssertSameVector($"{what}: x-axis", end.X, toEnd * start.X, 1e-9);
                AssertSameVector($"{what}: y-axis", end.Y, toEnd * start.Y, 1e-9);
                AssertSameVector($"{what}: z-axis", end.Z, toEnd * start.Z, 1e-9);
                GeoPoint startLocation = GeoPoint.Origin + start.Location;
                GeoPoint endLocation = GeoPoint.Origin + end.Location;
                Assert.IsTrue((toEnd * startLocation | endLocation) < 1e-9,
                    $"{what}: the origin is not mapped onto the end of the spine");
            }
        }

        // ------------------------------------------------------------------------------------ helpers --

        /// <summary>Every combination of spine and law that is meant to work, for the blanket tests.</summary>
        private static IEnumerable<(string, SweepFrame)> EveryFrame()
        {
            foreach ((string what, SweepFrame frame) in EveryFollowingFrame()) yield return (what, frame);
            yield return ("fixed along a helix", SweepFrame.Create(HelicalSpine(), SweepOrientation.Fixed));
            yield return ("fixed along a circle", SweepFrame.Create(CircularSpine(), SweepOrientation.Fixed));
        }

        private static IEnumerable<(string, SweepFrame)> EveryFollowingFrame()
        {
            yield return ("follow along a line", SweepFrame.Create(StraightSpine(), SweepOrientation.Follow));
            yield return ("follow along a circle", SweepFrame.Create(CircularSpine(), SweepOrientation.Follow));
            yield return ("follow along a helix", SweepFrame.Create(HelicalSpine(), SweepOrientation.Follow));
            yield return ("follow along a helix, explicit reference",
                SweepFrame.Create(HelicalSpine(), SweepOrientation.Follow, new GeoVector(0.3, -0.5, 1.0)));
            yield return ("frenet along a helix", FrenetFrameOf(HelicalSpine()));
        }

        /// <summary>
        /// The Frenet frame, asked for explicitly. Create would not give it: on a helix FindSweepNormal does
        /// find a reference direction - see <see cref="create_prefers_a_reference_direction_over_frenet"/> -
        /// so going through Create here would silently test the fixed reference frame instead.
        /// </summary>
        private static SweepFrame FrenetFrameOf(ICurve spine) => SweepFrame.CreateFrenet(spine);

        private static void AssertOrthonormal(string what, SweepFrame frame, double u)
        {
            SweepAxes axes = frame.AxesAt(u);
            Assert.IsTrue(Math.Abs(axes.X.Length - 1.0) < 1e-9, $"{what} at u={u}: |x| = {axes.X.Length}");
            Assert.IsTrue(Math.Abs(axes.Y.Length - 1.0) < 1e-9, $"{what} at u={u}: |y| = {axes.Y.Length}");
            Assert.IsTrue(Math.Abs(axes.Z.Length - 1.0) < 1e-9, $"{what} at u={u}: |z| = {axes.Z.Length}");
            Assert.IsTrue(Math.Abs(axes.X * axes.Y) < 1e-9, $"{what} at u={u}: x and y are not perpendicular");
            Assert.IsTrue(Math.Abs(axes.X * axes.Z) < 1e-9, $"{what} at u={u}: x and z are not perpendicular");
            Assert.IsTrue(Math.Abs(axes.Y * axes.Z) < 1e-9, $"{what} at u={u}: y and z are not perpendicular");
            AssertSameVector($"{what} at u={u}: x^y must be z", axes.Z, axes.X ^ axes.Y, 1e-9);
        }

        /// <summary>
        /// Compares the derivative of the given order against a central difference of the order below it. The
        /// tolerance is relative to the size of what is being compared, because the location of a spine far
        /// from the origin carries much larger numbers than the unit axes do.
        /// </summary>
        private static void AssertDerivativeMatchesDifference(string what, SweepFrame frame, double u, int order, double tolerance)
        {
            IReadOnlyList<SweepAxes> exact = frame.AxesAndDerivativesAt(u, order);
            IReadOnlyList<SweepAxes> before = frame.AxesAndDerivativesAt(u - StepForDifferences, order - 1);
            IReadOnlyList<SweepAxes> after = frame.AxesAndDerivativesAt(u + StepForDifferences, order - 1);
            double f = 1.0 / (2.0 * StepForDifferences);

            AssertClose($"{what} at u={u}: location derivative {order}",
                f * (after[order - 1].Location - before[order - 1].Location), exact[order].Location, tolerance);
            AssertClose($"{what} at u={u}: x-axis derivative {order}",
                f * (after[order - 1].X - before[order - 1].X), exact[order].X, tolerance);
            AssertClose($"{what} at u={u}: y-axis derivative {order}",
                f * (after[order - 1].Y - before[order - 1].Y), exact[order].Y, tolerance);
            AssertClose($"{what} at u={u}: z-axis derivative {order}",
                f * (after[order - 1].Z - before[order - 1].Z), exact[order].Z, tolerance);
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
    }
}
