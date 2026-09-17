using CADability.Curve2D;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Runtime.Serialization;

namespace CADability.GeoObject
{
    /// <summary>
    /// A surface made by sweeping one curve along another: the profile <c>ToSweep</c> travels along the spine
    /// <c>Along</c>, carried by a <see cref="SweepFrame"/>.
    /// <para>
    /// Parameter u runs along the profile, parameter v along the spine, both from 0 to 1. The profile is
    /// expressed once in the coordinates of the frame at v = 0 and from then on is only ever carried: a point
    /// of the surface is the frame at v applied to the profile point at u. Because that mapping is linear in
    /// the profile point and the profile does not depend on v, EVERY derivative of the surface is the matching
    /// derivative of the frame applied to the matching derivative of the profile - which is where
    /// <see cref="SweepAxes.Evaluate(GeoPoint)"/> comes in and why there is not a difference quotient in this
    /// class.
    /// </para>
    /// <para>
    /// The special case of a circular profile is <see cref="SweptCircleSurface"/>, which knows its radius and
    /// can do things this class cannot - an exact PositionOf, self intersection, the fillet machinery. Where a
    /// line or a circular arc is swept along a line or a circular arc, the result is a quadric and neither
    /// class is needed.
    /// </para>
    /// </summary>
    [Serializable]
    public class SweptCurveSurface : ISurfaceImpl, ISerializable, IJsonSerialize, IJsonSerializeDone, ISurfaceOfExtrusion
    {
        private ICurve toSweep;              // the profile, u runs along it
        private ICurve along;                // the spine, v runs along it
        private SweepOrientation orientation;
        private GeoVector reference;         // the null vector when the frame is to find one itself

        // secondary, rebuilt by Init
        private SweepFrame frame;
        private SweepAxes startAxes;         // the frame at v = 0, the system the profile is expressed in

        /// <summary>
        /// Creates the surface.
        /// </summary>
        /// <param name="toSweep">the profile. It does not have to be planar or perpendicular to the spine, it
        /// is taken exactly where it is and carried along from there.</param>
        /// <param name="along">the spine</param>
        /// <param name="orientation">whether the profile follows the spine or keeps its orientation</param>
        /// <param name="reference">the direction the profile is kept aligned to, or the null vector to let
        /// <see cref="SweepFrame.Create"/> find one. It must not be parallel to the tangent of the spine.</param>
        public SweptCurveSurface(ICurve toSweep, ICurve along,
                                 SweepOrientation orientation = SweepOrientation.Follow,
                                 GeoVector reference = default, BoundingRect? domain = null) : base(domain)
        {
            this.toSweep = toSweep ?? throw new ArgumentNullException(nameof(toSweep));
            this.along = along ?? throw new ArgumentNullException(nameof(along));
            this.orientation = orientation;
            this.reference = reference;
            Init();
        }

        private void Init()
        {
            frame = SweepFrame.Create(along, orientation, reference);
            startAxes = frame.AxesAt(0.0);
        }

        /// <summary>The curve that is being swept. u runs along it.</summary>
        public ICurve Profile => toSweep;
        /// <summary>The curve it is swept along. v runs along it.</summary>
        public ICurve Spine => along;
        /// <summary>The moving coordinate system that carries the profile.</summary>
        public SweepFrame Frame => frame;

        // ------------------------------------------------------- the profile in the coordinates of the frame --

        private GeoPoint ProfilePointAt(double u) => startAxes.LocalOf(toSweep.PointAt(u));
        private GeoVector ProfileDirectionAt(double u) => startAxes.LocalOf(toSweep.DirectionAt(u));

        private GeoVector ProfileSecondDerivativeAt(double u)
        {
            IReadOnlyList<GeoVector> d = toSweep.PointAndDerivativesAt(u, 2);
            return startAxes.LocalOf(d[2]);
        }

        // --------------------------------------------------------------------------------- ISurfaceImpl --

        public override GeoPoint PointAt(GeoPoint2D uv)
            => GeoPoint.Origin + frame.AxesAt(uv.y).Evaluate(ProfilePointAt(uv.x));

        public override GeoVector UDirection(GeoPoint2D uv)
            => frame.AxesAt(uv.y).Evaluate(ProfileDirectionAt(uv.x));

        public override GeoVector VDirection(GeoPoint2D uv)
            => frame.AxesAndDerivativesAt(uv.y, 1)[1].Evaluate(ProfilePointAt(uv.x));

        public override void DerivativeAt(GeoPoint2D uv, out GeoPoint location, out GeoVector du, out GeoVector dv)
        {
            IReadOnlyList<SweepAxes> axes = frame.AxesAndDerivativesAt(uv.y, 1);
            GeoPoint profile = ProfilePointAt(uv.x);
            location = GeoPoint.Origin + axes[0].Evaluate(profile);
            du = axes[0].Evaluate(ProfileDirectionAt(uv.x));
            dv = axes[1].Evaluate(profile);
        }

        public override void Derivative2At(GeoPoint2D uv, out GeoPoint location, out GeoVector du, out GeoVector dv,
                                           out GeoVector duu, out GeoVector dvv, out GeoVector duv)
        {
            // S(u,v) = F(v) applied to p(u). Differentiating by u touches only p, by v only F, and the
            // combination is linear - so all six values are one product of the two sides each.
            IReadOnlyList<SweepAxes> axes = frame.AxesAndDerivativesAt(uv.y, 2);
            GeoPoint profile = ProfilePointAt(uv.x);
            GeoVector profile1 = ProfileDirectionAt(uv.x);
            GeoVector profile2 = ProfileSecondDerivativeAt(uv.x);

            location = GeoPoint.Origin + axes[0].Evaluate(profile);
            du = axes[0].Evaluate(profile1);
            dv = axes[1].Evaluate(profile);
            duu = axes[0].Evaluate(profile2);
            dvv = axes[2].Evaluate(profile);
            duv = axes[1].Evaluate(profile1);
        }

        public override void GetNaturalBounds(out double umin, out double umax, out double vmin, out double vmax)
        {
            umin = 0.0;
            umax = 1.0;
            vmin = 0.0;
            vmax = 1.0;
        }

        /// <summary>
        /// Periodic in u when the profile closes on itself. This is a GEOMETRIC test on purpose: BSpline.IsClosed
        /// returns the "periodic" flag, and a clamped NURBS circle - nine poles, the last one repeating the first -
        /// answers false to it although it is as closed as a curve can be.
        /// </summary>
        public override bool IsUPeriodic => (toSweep.StartPoint | toSweep.EndPoint) < Precision.eps;

        /// <summary>
        /// Periodic in v when the spine closes AND the frame comes back to where it started. A closed spine is
        /// not enough by itself: the frame may have twisted round it, and then the surface does not meet itself
        /// at the seam even though the spine does.
        /// </summary>
        public override bool IsVPeriodic
        {
            get
            {
                if ((along.StartPoint | along.EndPoint) > Precision.eps) return false;
                SweepAxes end = frame.AxesAt(1.0);
                return Precision.SameDirection(startAxes.X, end.X, false)
                    && Precision.SameDirection(startAxes.Y, end.Y, false)
                    && Precision.SameDirection(startAxes.Z, end.Z, false);
            }
        }

        public override double UPeriod => IsUPeriodic ? 1.0 : 0.0;
        public override double VPeriod => IsVPeriodic ? 1.0 : 0.0;

        public override ICurve FixedV(double v, double umin, double umax)
        {
            ICurve res = toSweep.CloneModified(frame.Between(0.0, v));
            if (umin > umax)
            {
                res.Trim(umax, umin);
                res.Reverse();
            }
            else res.Trim(umin, umax);
            return res;
        }

        public override ICurve FixedU(double u, double vmin, double vmax) => new FixedUCurve(this, u, vmin, vmax);

        public override ISurface Clone()
        {
            SweptCurveSurface res = new SweptCurveSurface(toSweep.Clone(), along.Clone(), orientation, reference);
            res.Domain = Domain;
            return res;
        }

        public override ISurface GetModified(ModOp m)
        {
            SweptCurveSurface res = new SweptCurveSurface(toSweep.CloneModified(m), along.CloneModified(m),
                                                          orientation, m * reference);
            res.Domain = Domain;
            return res;
        }

        public override void Modify(ModOp m)
        {
            toSweep = toSweep.CloneModified(m);
            along = along.CloneModified(m);
            reference = m * reference;
            parallelepipedHull = null;
            Init();
        }

        public override void CopyData(ISurface copyFrom)
        {
            if (copyFrom is SweptCurveSurface other)
            {
                toSweep = other.toSweep;
                along = other.along;
                orientation = other.orientation;
                reference = other.reference;
                parallelepipedHull = null;
                Init();
            }
        }

        /// <summary>
        /// Reverses the direction of the profile, which turns the normal round. The spine is left alone: it is
        /// what defines the frame, and reversing it would move every point of the surface instead of only
        /// relabelling it.
        /// </summary>
        public override ModOp2D ReverseOrientation()
        {
            toSweep.Reverse();
            parallelepipedHull = null;
            return ModOp2D.Translate(1, 0) * ModOp2D.Scale(-1, 1);
        }

        /// <summary>
        /// The steps follow the two curves: where the profile bends, the surface bends in u, and where the
        /// spine bends it bends in v.
        /// </summary>
        public override void GetSafeParameterSteps(double umin, double umax, double vmin, double vmax,
                                                   out double[] intu, out double[] intv)
        {
            intu = StepsAlong(toSweep, umin, umax);
            intv = StepsAlong(along, vmin, vmax);
        }

        /// <summary>
        /// Where the curve bends, refined until no single step turns its tangent by more than 45 degrees.
        /// <para>
        /// The bare save positions are not enough, and it is worth saying why: they are where a spline has its
        /// knots, and a spline through four points has four of them however far it curls in between. These
        /// steps are what the <see cref="ParallelepipedHull"/> is built from, and PositionOf searches that
        /// hull - measured on a sweep of a spline along a spline, the unrefined grid sent one lookup in ten to
        /// the wrong sheet of the surface, up to 3 units away from the point it was asked about.
        /// </para>
        /// </summary>
        private static double[] StepsAlong(ICurve curve, double from, double to)
        {
            SortedSet<double> steps = new SortedSet<double> { from, to };
            double[] savePositions = curve.GetSavePositions();
            if (savePositions != null)
                foreach (double position in savePositions)
                    if (position > from && position < to) steps.Add(position);

            const double maxTurn = Math.PI / 4.0; // 45 degrees per step
            const int maxRefinements = 6;         // so one interval turns into at most 64
            for (int round = 0; round < maxRefinements; round++)
            {
                double[] current = new double[steps.Count];
                steps.CopyTo(current);
                List<double> toInsert = new List<double>();
                for (int i = 1; i < current.Length; i++)
                {
                    GeoVector before = curve.DirectionAt(current[i - 1]);
                    GeoVector after = curve.DirectionAt(current[i]);
                    if (before.IsNullVector() || after.IsNullVector()) continue;
                    if (new Angle(before, after) > maxTurn) toInsert.Add((current[i - 1] + current[i]) / 2.0);
                }
                if (toInsert.Count == 0) break;
                foreach (double position in toInsert) steps.Add(position);
            }

            if (steps.Count < 3) steps.Add((from + to) / 2.0);
            double[] res = new double[steps.Count];
            steps.CopyTo(res);
            return res;
        }

        // PositionOf is deliberately NOT overridden. The obvious seed - the closest point of the spine gives
        // v, the profile position of the point pulled back into the frame there gives u - is only exact while
        // the profile is perpendicular to the spine, and on a wavy sweep the minimizer started from it
        // converges to the wrong sheet: measured 1.5 units away from the point it was asked about. The base
        // implementation searches the ParallelepipedHull and is robust, which matters more here than speed.
        // RuledSurface does the same. Should this ever show up in a profile, a seeded version needs to VALIDATE
        // its result against the hull rather than trust it.

        public override ICurve Make3dCurve(ICurve2D curve2d)
        {
            if (curve2d is Line2D line2d)
            {
                if (Math.Abs(line2d.StartPoint.y - line2d.EndPoint.y) < Precision.eps)
                    return FixedV(line2d.StartPoint.y, line2d.StartPoint.x, line2d.EndPoint.x);
                if (Math.Abs(line2d.StartPoint.x - line2d.EndPoint.x) < Precision.eps)
                    return FixedU(line2d.StartPoint.x, line2d.StartPoint.y, line2d.EndPoint.y);
            }
            if (curve2d is ProjectedCurve projected && projected.Surface is SweptCurveSurface)
            {
                BoundingRect otherBounds = new BoundingRect(PositionOf(projected.Surface.PointAt(projected.StartPoint)),
                                                            PositionOf(projected.Surface.PointAt(projected.EndPoint)));
                if (projected.Surface.SameGeometry(projected.GetExtent(), this, otherBounds, Precision.eps, out ModOp2D _))
                    return projected.Curve3DFromParams; // still correct when it was trimmed or reversed
            }
            return base.Make3dCurve(curve2d);
        }

        public override bool SameGeometry(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds,
                                          double precision, out ModOp2D firstToSecond)
        {
            if (other is SweptCurveSurface sweptOther
                && toSweep.SameGeometry(sweptOther.toSweep, precision)
                && along.SameGeometry(sweptOther.along, precision)
                && orientation == sweptOther.orientation)
            {
                firstToSecond = ModOp2D.Identity;
                return true;
            }
            firstToSecond = ModOp2D.Null;
            return false;
        }

        /// <summary>
        /// Returns the projected curve for a curve which runs in the direction of v, i.e. which has one u for
        /// every v. It starts at v = 0 and ends at v = 1, or the other way round. The profile of this surface
        /// must be planar for this: the curve is intersected with the plane of the profile at a few v, and the
        /// position on the profile there is the u.
        /// </summary>
        internal ICurve2D GetProjectedCurveAlongV(ICurve toProject)
        {
            const int n = 8;
            List<GeoPoint2D> pnts = new List<GeoPoint2D>();
            for (int i = 1; i < n - 1; i++)
            {
                double v = (double)i / (n - 1);
                ICurve swept = FixedV(v, 0.0, 1.0);
                double[] ips = toProject.GetPlaneIntersection(swept.GetPlane());
                // there should always be exactly one intersection
                for (int j = 0; j < ips.Length; j++)
                {
                    if (ips[j] > -1e-6 && ips[j] < 1 + 1e-6)
                    {
                        pnts.Add(new GeoPoint2D(swept.PositionOf(toProject.PointAt(ips[j])), v));
                        break;
                    }
                }
            }
            ICurve atStart = FixedV(0.0, 0.0, 1.0);
            Plane pln = atStart.GetPlane();
            bool forward = pln.Distance(toProject.StartPoint) < pln.Distance(toProject.EndPoint);
            pnts.Insert(0, new GeoPoint2D(atStart.PositionOf(forward ? toProject.StartPoint : toProject.EndPoint), 0));
            ICurve atEnd = FixedV(1.0, 0.0, 1.0);
            pnts.Add(new GeoPoint2D(atEnd.PositionOf(forward ? toProject.EndPoint : toProject.StartPoint), 1));

            BSpline2D res = new BSpline2D(pnts.ToArray(), 3, false);
            if ((PointAt(res.StartPoint) | toProject.StartPoint) + (PointAt(res.EndPoint) | toProject.EndPoint)
              > (PointAt(res.StartPoint) | toProject.EndPoint) + (PointAt(res.EndPoint) | toProject.StartPoint))
            {
                res.Reverse();
            }
            return res;
        }

        public override IPropertyEntry GetPropertyEntry(IFrame frame) => new GroupProperty("SweptCurveSurface", new IPropertyEntry[0]);

        // ---------------------------------------------------------------------------- ISurfaceOfExtrusion --

        ICurve ISurfaceOfExtrusion.Axis(BoundingRect domain)
        {
            ICurve res = along.Clone();
            res.Trim(domain.Bottom, domain.Top);
            return res;
        }
        IOrientation ISurfaceOfExtrusion.Orientation => null;
        ICurve ISurfaceOfExtrusion.ExtrudedCurve => FixedV(0.0, 0.0, 1.0);
        bool ISurfaceOfExtrusion.ExtrusionDirectionIsV => true;
        bool ISurfaceOfExtrusion.ModifyAxis(GeoPoint throughPoint) => false; // the spine is a curve, it cannot simply be shifted

        // ------------------------------------------------------------------------------------ FixedUCurve --

        /// <summary>
        /// The path one point of the profile takes along the spine. Its direction comes from the frame, so it
        /// is exact rather than approximated.
        /// </summary>
        public class FixedUCurve : GeneralCurve, IJsonSerialize
        {
            private SweptCurveSurface parent;
            private double u;
            private double vmin, vmax;

            public FixedUCurve(SweptCurveSurface parent, double u, double vmin, double vmax)
            {
                this.parent = parent;
                this.u = u;
                this.vmin = vmin;
                this.vmax = vmax;
            }

            private double ParameterAt(double position) => vmin + position * (vmax - vmin);

            public override GeoPoint PointAt(double position) => parent.PointAt(new GeoPoint2D(u, ParameterAt(position)));

            public override GeoVector DirectionAt(double position)
                => (vmax - vmin) * parent.VDirection(new GeoPoint2D(u, ParameterAt(position)));

            public override IGeoObject Clone() => new FixedUCurve(parent, u, vmin, vmax);

            public override void CopyGeometry(IGeoObject toCopyFrom)
            {
                if (toCopyFrom is FixedUCurve other)
                {
                    parent = other.parent;
                    u = other.u;
                    vmin = other.vmin;
                    vmax = other.vmax;
                }
            }

            public override void Modify(ModOp m)
            {
                parent = (SweptCurveSurface)parent.GetModified(m);
                InvalidateSecondaryData();
            }

            public override void Reverse()
            {
                double tmp = vmax;
                vmax = vmin;
                vmin = tmp;
                InvalidateSecondaryData();
            }

            public override ICurve[] Split(double position)
            {
                if (position <= 0.0 || position >= 1.0) return new ICurve[] { Clone() as ICurve };
                double at = ParameterAt(position);
                return new ICurve[] { new FixedUCurve(parent, u, vmin, at), new FixedUCurve(parent, u, at, vmax) };
            }

            public override void Trim(double startPos, double endPos)
            {
                double from = ParameterAt(startPos), to = ParameterAt(endPos);
                vmin = from;
                vmax = to;
                InvalidateSecondaryData();
            }

            protected override double[] GetBasePoints()
            {
                return StepsAlong(parent.along, 0.0, 1.0);
            }

            // ---------------------------------------------------------------------------- serialization --
            // Only IJsonSerialize: the base class already implements IJsonSerialize (via IGeoObjectImpl), so
            // JsonSerialize takes that route for this type and never looks at ISerializable. That route needs
            // a parameterless constructor, which is why the old ISerializable implementation was unusable.

            protected FixedUCurve() { } // for IJsonSerialize

            public void GetObjectData(IJsonWriteData data)
            {
                data.AddProperty("Parent", parent);
                data.AddProperty("U", u);
                data.AddProperty("Vmin", vmin);
                data.AddProperty("Vmax", vmax);
            }

            public void SetObjectData(IJsonReadData data)
            {
                parent = data.GetProperty<SweptCurveSurface>("Parent");
                u = data.GetProperty<double>("U");
                vmin = data.GetProperty<double>("Vmin");
                vmax = data.GetProperty<double>("Vmax");
            }
        }

        // --------------------------------------------------------------------------------- serialization --

        protected SweptCurveSurface(SerializationInfo info, StreamingContext context)
        {
            toSweep = (ICurve)info.GetValue("ToSweep", typeof(ICurve));
            along = (ICurve)info.GetValue("Along", typeof(ICurve));
            orientation = (SweepOrientation)info.GetValue("Orientation", typeof(SweepOrientation));
            reference = (GeoVector)info.GetValue("Reference", typeof(GeoVector));
            Init();
        }

        void ISerializable.GetObjectData(SerializationInfo info, StreamingContext context)
        {
            info.AddValue("ToSweep", toSweep, toSweep.GetType());
            info.AddValue("Along", along, along.GetType());
            info.AddValue("Orientation", orientation, typeof(SweepOrientation));
            info.AddValue("Reference", reference, typeof(GeoVector));
        }

        protected SweptCurveSurface() { } // for IJsonSerialize

        public void GetObjectData(IJsonWriteData data)
        {
            data.AddProperty("ToSweep", toSweep);
            data.AddProperty("Along", along);
            data.AddProperty("Orientation", orientation);
            data.AddProperty("Reference", reference);
        }

        public void SetObjectData(IJsonReadData data)
        {
            toSweep = data.GetProperty<ICurve>("ToSweep");
            along = data.GetProperty<ICurve>("Along");
            orientation = data.GetProperty<SweepOrientation>("Orientation");
            reference = data.GetProperty<GeoVector>("Reference");
            data.RegisterForSerializationDoneCallback(this);
        }

        void IJsonSerializeDone.SerializationDone(JsonSerialize jsonSerialize) => Init();
    }
}
