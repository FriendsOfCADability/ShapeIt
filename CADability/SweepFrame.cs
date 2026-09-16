using System;
using System.Collections.Generic;
using System.Linq;
using static System.Math;

namespace CADability.GeoObject
{
    /// <summary>
    /// How a profile is oriented while it travels along a spine curve. This is the only degree of freedom a
    /// sweep has: the origin of the moving system is fixed to the spine and one of its axes is the tangent,
    /// so all that is left to decide is how the system is rotated about that tangent.
    /// </summary>
    public enum SweepOrientation
    {
        /// <summary>
        /// The profile keeps its orientation in space and is only translated along the spine. The moving system
        /// does not rotate at all, so a profile which is perpendicular to the spine at the start will be oblique
        /// to it wherever the spine bends away.
        /// </summary>
        Fixed,
        /// <summary>
        /// The profile follows the spine: the z-axis of the moving system is the tangent, so the profile stays
        /// perpendicular to the spine. Which rotation about that tangent is used is decided by
        /// <see cref="SweepFrame.Create(ICurve, SweepOrientation, GeoVector)"/>.
        /// </summary>
        Follow
    }

    /// <summary>
    /// Which rule a <see cref="SweepFrame"/> actually ended up using. <see cref="SweepFrame.Create"/> picks one
    /// by looking at the spine, and this says which - worth knowing, because the three behave quite differently
    /// where the spine is awkward.
    /// </summary>
    public enum SweepLaw
    {
        /// <summary>The system does not rotate at all, see <see cref="SweepOrientation.Fixed"/>.</summary>
        NoRotation,
        /// <summary>The x-axis is a constant direction, projected perpendicular to the tangent. The usual
        /// case, and the well behaved one: no twist, and nothing to go wrong at an inflection point.</summary>
        FixedReference,
        /// <summary>Tangent, principal normal and binormal. The last resort, for a spine whose tangents fill
        /// space so evenly that no constant reference direction works.</summary>
        Frenet
    }

    /// <summary>
    /// The moving coordinate system at one parameter of the spine: its origin and its three axes. The same
    /// structure carries the derivatives of those four vectors, which is why the origin is a
    /// <see cref="GeoVector"/> and not a <see cref="GeoPoint"/> - the derivative of a point is not a point.
    /// This mirrors <see cref="ICurve.PointAndDerivativesAt(double, int)"/>, where entry 0 is the point as a
    /// vector from the origin.
    /// </summary>
    public readonly struct SweepAxes
    {
        /// <summary>The origin of the system, which is the point on the spine.</summary>
        public GeoVector Location { get; }
        /// <summary>The x-axis, a unit vector. For <see cref="SweepOrientation.Follow"/> it is the reference
        /// direction, projected into the plane perpendicular to the tangent.</summary>
        public GeoVector X { get; }
        /// <summary>The y-axis, a unit vector, perpendicular to <see cref="X"/> and <see cref="Z"/>.</summary>
        public GeoVector Y { get; }
        /// <summary>The z-axis, a unit vector. For <see cref="SweepOrientation.Follow"/> it is the tangent of
        /// the spine.</summary>
        public GeoVector Z { get; }

        public SweepAxes(GeoVector location, GeoVector x, GeoVector y, GeoVector z)
        {
            Location = location;
            X = x;
            Y = y;
            Z = z;
        }

        /// <summary>
        /// The linear combination Location + local.x*X + local.y*Y + local.z*Z, i.e. the point whose
        /// coordinates in this system are <paramref name="local"/>.
        /// <para>
        /// Applied to the n-th derivative of the system this is the n-th derivative of that point: the
        /// combination is linear and <paramref name="local"/> does not depend on the spine parameter, so
        /// differentiating it means differentiating the four vectors and nothing else. That is what makes the
        /// derivatives of a swept surface fall out of the derivatives of its frame.
        /// </para>
        /// </summary>
        public GeoVector Evaluate(GeoPoint local) => Location + local.x * X + local.y * Y + local.z * Z;
    }

    /// <summary>
    /// A coordinate system travelling along a curve - the "spine" - which is what a swept surface is built on:
    /// the profile is expressed once in the system at the start and then carried along by it.
    /// <para>
    /// This is the part <see cref="SweptCircleSurface"/> and a general swept surface have in common. Where the system
    /// stands at a parameter is the same question for both of them; only what is carried along differs - a
    /// circle of a given radius there, an arbitrary curve here.
    /// </para>
    /// <para>
    /// Accuracy: the derivatives are closed form, not difference quotients, but the second derivative of the
    /// frame needs the THIRD derivative of the spine, and
    /// <see cref="GeneralCurve.PointAndDerivativesAt(ICurve, double, int)"/> produces anything beyond the
    /// second one numerically for most curves. So the second derivative of the frame is exact in its formula
    /// and only as good as that numerical third derivative in its value.
    /// </para>
    /// </summary>
    public abstract class SweepFrame
    {
        /// <summary>The curve the system travels along.</summary>
        public ICurve Spine { get; }

        /// <summary>Which of the three rules this frame follows.</summary>
        public abstract SweepLaw Law { get; }

        protected SweepFrame(ICurve spine)
        {
            Spine = spine ?? throw new ArgumentNullException(nameof(spine));
        }

        /// <summary>
        /// The system at <paramref name="u"/> and, when asked for, its derivatives with respect to u. Entry 0
        /// is the system itself, entry 1 its first derivative, entry 2 its second - the same layout
        /// <see cref="ICurve.PointAndDerivativesAt(double, int)"/> uses.
        /// </summary>
        /// <param name="u">the parameter on the spine</param>
        /// <param name="derivations">0, 1 or 2</param>
        public abstract IReadOnlyList<SweepAxes> AxesAndDerivativesAt(double u, int derivations);

        /// <summary>The system at <paramref name="u"/>, without derivatives.</summary>
        public SweepAxes AxesAt(double u) => AxesAndDerivativesAt(u, 0)[0];

        /// <summary>
        /// The transformation which takes the system at <paramref name="from"/> onto the system at
        /// <paramref name="to"/>. Applied to a profile placed in the first one it moves it to the second, which
        /// is how a fixed-v curve of a swept surface is built.
        /// </summary>
        public ModOp Between(double from, double to)
        {
            SweepAxes start = AxesAt(from);
            SweepAxes end = AxesAt(to);
            return ModOp.Translate(end.Location) * ModOp.Fit(new GeoVector[] { start.X, start.Y, start.Z },
                                                             new GeoVector[] { end.X, end.Y, end.Z })
                                                 * ModOp.Translate(-start.Location);
        }

        /// <summary>
        /// Chooses the law. With <see cref="SweepOrientation.Fixed"/> the system never rotates. Otherwise the
        /// tangent becomes the z-axis and the rotation about it is settled in this order:
        /// <list type="bullet">
        /// <item>an explicit <paramref name="reference"/> direction, if one is given,</item>
        /// <item>the normal of the plane the spine lies in, if it is planar,</item>
        /// <item>any direction perpendicular to a straight spine,</item>
        /// <item>the direction <see cref="SweptCircleSurface.FindSweepNormal"/> finds by a principal component
        /// analysis of the tangents, for a spine which is nearly planar,</item>
        /// <item>and only when none of that works, the Frenet frame - which is the least pleasant of the five
        /// because its normal flips at an inflection point of the spine.</item>
        /// </list>
        /// This is the order <see cref="SweptCircleSurface"/> has been using, spelled out.
        /// </summary>
        /// <param name="spine">the curve to travel along</param>
        /// <param name="orientation">whether the system follows the spine or stays as it is</param>
        /// <param name="reference">the direction the x-axis is derived from, or the null vector to let this
        /// method find one. It must not be parallel to the tangent anywhere on the spine.</param>
        public static SweepFrame Create(ICurve spine, SweepOrientation orientation, GeoVector reference = default)
        {
            SweepFrame following = CreateFollowing(spine, reference);
            if (orientation == SweepOrientation.Follow) return following;
            // "Fixed" freezes the axes the following frame has at the start, so that both laws place the
            // profile identically at the beginning of the sweep and only differ further along.
            SweepAxes start = following.AxesAt(0.0);
            return new NoRotation(spine, start.X, start.Y, start.Z);
        }

        /// <summary>
        /// The Frenet frame, explicitly. <see cref="Create"/> only falls back to it when nothing else works, so
        /// this is the way to ask for it on a spine where a reference direction would have been found. It is
        /// undefined wherever the spine is straight.
        /// </summary>
        public static SweepFrame CreateFrenet(ICurve spine) => new Frenet(spine);

        private static SweepFrame CreateFollowing(ICurve spine, GeoVector reference)
        {
            if (!reference.IsNullVector()) return new FixedReference(spine, reference);
            switch (spine.GetPlanarState())
            {
                case PlanarState.Planar:
                    return new FixedReference(spine, spine.GetPlane().Normal);
                case PlanarState.UnderDetermined: // a straight line: any perpendicular direction will do
                    spine.StartDirection.ArbitraryNormals(out GeoVector perpendicular, out GeoVector _);
                    return new FixedReference(spine, perpendicular);
                default:
                    const int samples = 25;
                    double[] at = Enumerable.Range(0, samples + 1).Select(i => i / (double)samples).ToArray();
                    GeoVector found = SweptCircleSurface.FindSweepNormal(spine, at);
                    if (!found.IsNullVector()) return new FixedReference(spine, found);
                    return new Frenet(spine);
            }
        }

        /// <summary>
        /// Normalizes <paramref name="w"/> and carries its derivatives along. For n = w/|w| and m = |w|:
        /// m' = (w.w')/m, n' = w'/m - w*m'/m^2, and with m'' = ((w'.w') + (w.w''))/m - m'^2/m,
        /// n'' = w''/m - 2*w'*m'/m^2 - w*m''/m^2 + 2*w*m'^2/m^3.
        /// </summary>
        protected static void Normalize(int derivations, GeoVector w, GeoVector w1, GeoVector w2,
                                        out GeoVector n, out GeoVector n1, out GeoVector n2)
        {
            double m = w.Length;
            if (m == 0.0) throw new SweepFrameException("the spine has no direction at this parameter");
            n = w / m;
            n1 = n2 = GeoVector.NullVector;
            if (derivations < 1) return;
            double m1 = (w * w1) / m;
            n1 = w1 / m - (m1 / (m * m)) * w;
            if (derivations < 2) return;
            double m2 = ((w1 * w1) + (w * w2)) / m - (m1 * m1) / m;
            n2 = w2 / m - (2.0 * m1 / (m * m)) * w1 - (m2 / (m * m)) * w + (2.0 * m1 * m1 / (m * m * m)) * w;
        }

        /// <summary>
        /// The system never rotates: the axes are constant and only the origin moves along the spine. Their
        /// derivatives are therefore the null vector, and the derivatives of the origin are those of the spine.
        /// </summary>
        private sealed class NoRotation : SweepFrame
        {
            private readonly GeoVector x, y, z;

            public override SweepLaw Law => SweepLaw.NoRotation;

            public NoRotation(ICurve spine, GeoVector x, GeoVector y, GeoVector z) : base(spine)
            {
                this.x = x;
                this.y = y;
                this.z = z;
            }

            public override IReadOnlyList<SweepAxes> AxesAndDerivativesAt(double u, int derivations)
            {
                IReadOnlyList<GeoVector> c = Spine.PointAndDerivativesAt(u, derivations);
                SweepAxes[] result = new SweepAxes[derivations + 1];
                result[0] = new SweepAxes(c[0], x, y, z);
                for (int i = 1; i <= derivations; i++)
                    result[i] = new SweepAxes(c[i], GeoVector.NullVector, GeoVector.NullVector, GeoVector.NullVector);
                return result;
            }
        }

        /// <summary>
        /// The z-axis is the tangent and the x-axis is a constant reference direction, projected into the plane
        /// perpendicular to it - ISO 10303 calls this a fixed reference swept surface. Everything here is
        /// closed form and needs the spine only up to its third derivative.
        /// </summary>
        private sealed class FixedReference : SweepFrame
        {
            private readonly GeoVector reference;

            public override SweepLaw Law => SweepLaw.FixedReference;

            public FixedReference(ICurve spine, GeoVector reference) : base(spine)
            {
                this.reference = reference.Normalized;
            }

            public override IReadOnlyList<SweepAxes> AxesAndDerivativesAt(double u, int derivations)
            {
                // the z-axis needs c', its first derivative c'', its second c''' - one more than is asked for
                IReadOnlyList<GeoVector> c = Spine.PointAndDerivativesAt(u, derivations + 1);
                GeoVector a = derivations >= 1 ? c[2] : GeoVector.NullVector;
                GeoVector j = derivations >= 2 ? c[3] : GeoVector.NullVector;
                Normalize(derivations, c[1], a, j, out GeoVector z, out GeoVector z1, out GeoVector z2);

                // y is perpendicular to both the reference and the tangent, so x = y^z is the reference itself,
                // projected into the plane perpendicular to the tangent and normalized - which is exactly what
                // "the reference direction is kept" means.
                GeoVector w = z ^ reference, w1 = z1 ^ reference, w2 = z2 ^ reference;
                if (w.Length < 1e-12)
                    throw new SweepFrameException("the reference direction is parallel to the tangent of the spine "
                        + $"at parameter {u}, so it cannot define the orientation of the profile there");
                Normalize(derivations, w, w1, w2, out GeoVector y, out GeoVector y1, out GeoVector y2);

                SweepAxes[] result = new SweepAxes[derivations + 1];
                result[0] = new SweepAxes(c[0], y ^ z, y, z);
                if (derivations >= 1) result[1] = new SweepAxes(c[1], (y1 ^ z) + (y ^ z1), y1, z1);
                if (derivations >= 2) result[2] = new SweepAxes(c[2], (y2 ^ z) + 2.0 * (y1 ^ z1) + (y ^ z2), y2, z2);
                return result;
            }
        }

        /// <summary>
        /// The Frenet frame: x is the principal normal, y the binormal, z the tangent. The last resort, because
        /// the principal normal is undefined where the spine is straight and flips where it has an inflection
        /// point. Use it only when no reference direction can be found.
        /// </summary>
        private sealed class Frenet : SweepFrame
        {
            public override SweepLaw Law => SweepLaw.Frenet;

            public Frenet(ICurve spine) : base(spine) { }

            public override IReadOnlyList<SweepAxes> AxesAndDerivativesAt(double u, int derivations)
            {
                // the principal normal needs c'', its derivatives need c''' - and no more than that, because
                // the derivative of the torsion is taken as a difference rather than in closed form below
                IReadOnlyList<GeoVector> c = Spine.PointAndDerivativesAt(u, derivations == 0 ? 2 : 3);
                GeoVector v = c[1], a = c[2];
                GeoVector j = derivations >= 1 ? c[3] : GeoVector.NullVector;

                double s = v.Length;
                if (s == 0.0) throw new SweepFrameException("the spine has no direction at this parameter");
                GeoVector t = v / s;
                GeoVector crossVA = v ^ a;
                if (crossVA.Length < 1e-12)
                    throw new SweepFrameException($"the spine is straight at parameter {u}, so it has no principal "
                        + "normal and the Frenet frame is undefined there. A reference direction is needed.");

                GeoVector normal = (a - (a * t) * t).Normalized;
                GeoVector binormal = t ^ normal;

                SweepAxes[] result = new SweepAxes[derivations + 1];
                result[0] = new SweepAxes(c[0], normal, binormal, t);
                if (derivations < 1) return result;

                // Frenet-Serret, scaled by s = |c'| because the derivatives are with respect to the curve
                // parameter and not to the arc length: T' = k*s*N, N' = -k*s*T + w*s*B, B' = -w*s*N.
                double s1 = (v * a) / s;
                double curvature = crossVA.Length / Pow(s, 3);
                // The torsion is ((c' x c'').c''') / |c' x c''|^2. Note that (c'.(c' x c''')) - which would be
                // the obvious looking way to write it - is identically zero, so it is not that.
                double torsion = (crossVA * j) / (crossVA * crossVA);

                GeoVector t1 = (curvature * s) * normal;
                GeoVector normal1 = (-curvature * s) * t + (torsion * s) * binormal;
                GeoVector binormal1 = (-torsion * s) * normal;
                result[1] = new SweepAxes(v, normal1, binormal1, t1);
                if (derivations < 2) return result;

                GeoVector crossVJ = v ^ j;
                double s2 = ((a * a) + (v * j)) / s - (s1 * s1) / s;
                double curvature1 = (crossVA * crossVJ) / (crossVA.Length * Pow(s, 3)) - 3.0 * curvature * s1 / s;
                // The derivative of the torsion is the one quantity here which would need the FOURTH derivative
                // of the spine, and that one is two numerical differentiations away from what the curve can
                // actually deliver. A central difference of the torsion itself - which is closed form - is both
                // cheaper and better conditioned. It is the only value on this path that is not closed form.
                double torsion1 = TorsionDerivativeAt(u);

                GeoVector t2 = (curvature1 * s + curvature * s1) * normal + (curvature * s) * normal1;
                GeoVector normal2 = (-curvature1 * s - curvature * s1) * t + (-curvature * s) * t1
                                  + (torsion1 * s + torsion * s1) * binormal + (torsion * s) * binormal1;
                GeoVector binormal2 = (-torsion1 * s - torsion * s1) * normal + (-torsion * s) * normal1;
                result[2] = new SweepAxes(a, normal2, binormal2, t2);
                return result;
            }

            /// <summary>The torsion of the spine at <paramref name="u"/>.</summary>
            private double TorsionAt(double u)
            {
                IReadOnlyList<GeoVector> c = Spine.PointAndDerivativesAt(u, 3);
                GeoVector crossVA = c[1] ^ c[2];
                double squared = crossVA * crossVA;
                if (squared < 1e-24) return 0.0;
                return (crossVA * c[3]) / squared;
            }

            /// <summary>
            /// The derivative of the torsion, by a central difference which stays inside [0, 1] - the spine is
            /// not defined outside its own parameter range, so at the ends the difference becomes one sided.
            /// </summary>
            private double TorsionDerivativeAt(double u)
            {
                const double step = 1e-5;
                double left = Max(0.0, u - step);
                double right = Min(1.0, u + step);
                if (right - left < 1e-12) return 0.0;
                return (TorsionAt(right) - TorsionAt(left)) / (right - left);
            }
        }
    }

    /// <summary>
    /// The moving system cannot be computed at some parameter of the spine - because the spine has no
    /// direction there, or because the reference direction is parallel to its tangent.
    /// </summary>
    public class SweepFrameException : ApplicationException
    {
        public SweepFrameException(string message) : base(message) { }
    }
}
