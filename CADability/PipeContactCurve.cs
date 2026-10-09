using CADability.Curve2D;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;

namespace CADability.GeoObject
{
    /// <summary>
    /// The geometry of the curve along which a pipe (a <see cref="SweptCircleSurface"/>) touches a surface, as a fillet
    /// touches the faces it connects (in the literature this is called a "spring curve" of a rolling ball blend).
    /// <para>
    /// The spine of such a pipe lies on the offset of the touched surface at the distance of the radius. So every spine
    /// point S(t) is F(uv) + side * r * n(uv) for some uv on the touched surface F, and F(uv) is the point of contact:
    /// it lies on the normal of F, which is perpendicular to the spine, so it is on the circle of the pipe at t. The
    /// curve is defined by this uv(t): its points are exactly on F, and on the pipe as precisely as the spine is known.
    /// The tangential intersection of the two surfaces, which is ill conditioned, is never solved.
    /// </para>
    /// <para>
    /// uv(t) is found by a Gauss-Newton iteration on the offset of F (whose derivatives follow from the first and second
    /// derivatives of F, the offset surface itself is not needed), started at the closest of some precalculated
    /// samples. The derivative uv'(t) follows from the derivative of the spine: S'(t) = O_u u' + O_v v' with the
    /// derivatives O_u, O_v of the offset, solved in the least squares sense.
    /// </para>
    /// This object is immutable: it is shared by <see cref="PipeContactCurve"/> and its 2d curves
    /// <see cref="PipeContactCurve2D"/>, which only add a parameter range (and a transformation of the uv space).
    /// </summary>
    [Serializable]
    internal class PipeContact : ISerializable, IJsonSerialize
    {
        private ISurface surface; // the touched surface, a private clone
        private SweptCircleSurface pipe; // the pipe, a private clone
        private double tA, tB; // the range of the spine parameter, which is covered by the samples
        private GeoPoint2D uvHint; // the uv position on surface at tA, a start value and the anchor for periodic surfaces
        // The ends of a curve of contact are usually vertices, which are also calculated otherwise (e.g. as the foot point of
        // the end of the spine). The curve ends exactly there: the solved uv positions are corrected by a vector, which is
        // linear in t and which is (exactly) the difference at the spine parameters endA and endB, see SetEnds.
        private double endA, endB;
        private GeoVector2D correctionA, correctionB;

        // secondary data, calculated when needed
        private readonly object samplesLock = new object();
        private double side; // +1 or -1: the spine point is at F(uv) + side * radius * n(uv), n is the normalized du^dv
        private double[] sampleT; // the spine parameters of the samples
        private GeoPoint2D[] sampleUvSurface, sampleUvPipe; // continuous along the curve
        private GeoPoint[] samplePoint;

        /// <summary>
        /// Creates the contact of <paramref name="pipe"/> with <paramref name="surface"/> between the spine parameters
        /// <paramref name="tStart"/> and <paramref name="tEnd"/> (which may be in descending order and, for a closed
        /// spine, outside [0,1]). <paramref name="uvStart"/> is (close to) the uv position on <paramref name="surface"/>
        /// of the contact point at <paramref name="tStart"/>, it defines the period on periodic surfaces.
        /// </summary>
        public PipeContact(ISurface surface, SweptCircleSurface pipe, double tStart, double tEnd, GeoPoint2D uvStart)
        {
            // clones, because the surfaces of faces are sometimes modified (e.g. their orientation reversed), but these
            // must keep their parametrization
            this.surface = surface.Clone();
            _ = pipe.PreciseSpine; // expensive, the clone shares it
            this.pipe = pipe.Clone() as SweptCircleSurface;
            tA = tStart;
            tB = tEnd;
            uvHint = uvStart;
        }
        protected PipeContact() { } // for the JSON serialization

        public ISurface Surface => surface;
        public SweptCircleSurface Pipe => pipe;
        private double Radius => Math.Abs(pipe.Radius);
        private bool ClosedSpine => pipe.Spine.IsClosed;
        private double SpineParameter(double t) => ClosedSpine ? t - Math.Floor(t) : t;
        private GeoPoint SpinePoint(double t) => pipe.PreciseSpine.PointAt(SpineParameter(t));
        private GeoVector SpineDirection(double t) => pipe.PreciseSpine.DirectionAt(SpineParameter(t));

        /// <summary>
        /// The contact with both surfaces modified by <paramref name="m"/>. The uv positions don't change.
        /// </summary>
        public PipeContact GetModified(ModOp m)
        {
            PipeContact res = new PipeContact();
            res.surface = surface.GetModified(m);
            res.pipe = pipe.GetModified(m) as SweptCircleSurface;
            res.tA = tA;
            res.tB = tB;
            res.uvHint = uvHint;
            res.endA = endA;
            res.endB = endB;
            res.correctionA = correctionA;
            res.correctionB = correctionB;
            return res;
        }

        /// <summary>
        /// Makes the curve end exactly at <paramref name="pointA"/> at the spine parameter <paramref name="tPointA"/> and at
        /// <paramref name="pointB"/> at <paramref name="tPointB"/>. Both points must be (almost) points of this curve. Only
        /// before the object is used by any curve.
        /// </summary>
        public void SetEnds(double tPointA, GeoPoint pointA, double tPointB, GeoPoint pointB)
        {
            correctionA = correctionB = GeoVector2D.NullVector; // evaluate without correction
            GeoPoint onA = Evaluate(tPointA, out GeoPoint2D solvedA, out _), onB = Evaluate(tPointB, out GeoPoint2D solvedB, out _);
            // only a small correction, otherwise the points are not on this curve
            if ((onA | pointA) > 100 * Precision.eps || (onB | pointB) > 100 * Precision.eps || tPointA == tPointB) return;
            endA = tPointA;
            endB = tPointB;
            correctionA = NearestPeriod(surface, surface.PositionOf(pointA), solvedA) - solvedA;
            correctionB = NearestPeriod(surface, surface.PositionOf(pointB), solvedB) - solvedB;
        }

        /// <summary>
        /// The correction of the solved uv positions at the spine parameter <paramref name="t"/>, see <see cref="endA"/>.
        /// </summary>
        private GeoVector2D Correction(double t)
        {
            if (endA == endB) return GeoVector2D.NullVector;
            double f = (t - endA) / (endB - endA);
            return correctionA + f * (correctionB - correctionA);
        }
        private GeoVector2D CorrectionDerivative => endA == endB ? GeoVector2D.NullVector : (1.0 / (endB - endA)) * (correctionB - correctionA);

        /// <summary>
        /// The point of <paramref name="s"/> at <paramref name="uv"/> moved by full periods to be closest to <paramref name="reference"/>.
        /// </summary>
        private static GeoPoint2D NearestPeriod(ISurface s, GeoPoint2D uv, GeoPoint2D reference)
        {
            if (s.IsUPeriodic && s.UPeriod > 0.0) uv.x += s.UPeriod * Math.Round((reference.x - uv.x) / s.UPeriod);
            if (s.IsVPeriodic && s.VPeriod > 0.0) uv.y += s.VPeriod * Math.Round((reference.y - uv.y) / s.VPeriod);
            return uv;
        }

        /// <summary>
        /// The point, the first derivatives and the normal of the touched surface, and the first derivatives of its offset
        /// at the distance <see cref="side"/> * radius.
        /// </summary>
        private bool Frame(GeoPoint2D uv, out GeoPoint location, out GeoVector du, out GeoVector dv, out GeoVector normal, out GeoVector ou, out GeoVector ov)
        {
            surface.Derivative2At(uv, out location, out du, out dv, out GeoVector duu, out GeoVector dvv, out GeoVector duv);
            GeoVector n = du ^ dv;
            double length = n.Length;
            if (length == 0.0 || double.IsNaN(length))
            {
                normal = ou = ov = GeoVector.NullVector;
                return false;
            }
            normal = (1.0 / length) * n;
            // derivatives of the normalized normal: the part of the derivative of du^dv perpendicular to the normal
            GeoVector nu = (duu ^ dv) + (du ^ duv);
            GeoVector nv = (duv ^ dv) + (du ^ dvv);
            nu = (1.0 / length) * (nu - (nu * normal) * normal);
            nv = (1.0 / length) * (nv - (nv * normal) * normal);
            double offset = side * Radius;
            ou = du + offset * nu;
            ov = dv + offset * nv;
            return true;
        }

        /// <summary>
        /// Solves the 2x2 normal equations of the least squares problem a*x + b*y = r.
        /// </summary>
        private static bool LeastSquares(GeoVector a, GeoVector b, GeoVector r, out double x, out double y)
        {
            double a11 = a * a, a12 = a * b, a22 = b * b;
            double r1 = a * r, r2 = b * r;
            double det = a11 * a22 - a12 * a12;
            if (det <= 0.0 || double.IsNaN(det) || det < 1e-24 * a11 * a22)
            {
                x = y = 0.0;
                return false;
            }
            x = (r1 * a22 - r2 * a12) / det;
            y = (a11 * r2 - a12 * r1) / det;
            return true;
        }

        /// <summary>
        /// The uv position on the touched surface, whose offset is the spine point at <paramref name="t"/>, by Gauss-Newton
        /// starting at <paramref name="uv"/>. Returns false, if it doesn't converge to a point of the offset.
        /// </summary>
        private bool Solve(double t, ref GeoPoint2D uv)
        {
            GeoPoint s = SpinePoint(t);
            double scale = Radius + Math.Abs(s.x) + Math.Abs(s.y) + Math.Abs(s.z);
            GeoPoint2D current = uv;
            for (int i = 0; i < 30; i++)
            {
                if (!Frame(current, out GeoPoint location, out GeoVector du, out GeoVector dv, out GeoVector normal, out GeoVector ou, out GeoVector ov)) return false;
                GeoVector residual = s - (location + side * Radius * normal);
                if (!LeastSquares(ou, ov, residual, out double dx, out double dy)) return false;
                current = new GeoPoint2D(current.x + dx, current.y + dy);
                double step = (dx * du + dy * dv).Length;
                if (double.IsNaN(step)) return false;
                if (step < 1e-15 * scale || (residual.Length < 1e-15 * scale && step < 1e-12 * scale))
                {
                    break;
                }
            }
            if (!Frame(current, out GeoPoint loc, out _, out _, out GeoVector n, out _, out _)) return false;
            if ((s - (loc + side * Radius * n)).Length > 1e-6 * Radius + Precision.eps) return false;
            uv = current;
            return true;
        }

        private int SampleIndex(double t)
        {
            int n = sampleT.Length - 1;
            if (tB == tA) return 0;
            int i = (int)Math.Round((t - tA) / (tB - tA) * n);
            return Math.Max(0, Math.Min(n, i));
        }

        /// <summary>
        /// Calculates the samples (once): uniformly distributed over the spine parameter, each one started at its predecessor,
        /// so that the uv positions are continuous also on periodic surfaces.
        /// </summary>
        private void EnsureSamples()
        {
            if (sampleT != null) return;
            lock (samplesLock)
            {
                if (sampleT != null) return;
                // the side of the offset, which contains the spine
                surface.DerivativeAt(uvHint, out GeoPoint hintLocation, out GeoVector hdu, out GeoVector hdv);
                side = ((SpinePoint(tA) - hintLocation) * (hdu ^ hdv)) < 0.0 ? -1.0 : 1.0;
                int count = Math.Max(32, 2 * pipe.Spine.GetSavePositions().Length);
                double[] ts = new double[count + 1];
                GeoPoint2D[] uvs = new GeoPoint2D[count + 1];
                GeoPoint2D[] uvp = new GeoPoint2D[count + 1];
                GeoPoint[] pts = new GeoPoint[count + 1];
                GeoPoint2D uv = uvHint;
                if (!Solve(tA, ref uv)) uv = uvHint;
                for (int i = 0; i <= count; i++)
                {
                    ts[i] = tA + i * (tB - tA) / count;
                    if (i > 0)
                    {
                        GeoPoint2D next = uv;
                        if (!Solve(ts[i], ref next))
                        {   // smaller steps from the predecessor
                            next = uv;
                            const int substeps = 8;
                            for (int j = 1; j <= substeps; j++)
                            {
                                GeoPoint2D sub = next;
                                if (Solve(ts[i - 1] + j * (ts[i] - ts[i - 1]) / substeps, ref sub)) next = sub;
                            }
                        }
                        uv = NearestPeriod(surface, next, uvs[i - 1]);
                    }
                    uvs[i] = uv;
                    pts[i] = surface.PointAt(uv);
                    GeoPoint2D onPipe = pipe.CirclePosition(pts[i], SpineParameter(ts[i]));
                    // the spine parameter of the pipe follows t (also beyond the period of a closed spine), the angle is continuous
                    onPipe.x += ts[i] - SpineParameter(ts[i]);
                    if (i > 0) onPipe = NearestPeriod(pipe, onPipe, uvp[i - 1]);
                    uvp[i] = onPipe;
                }
                sampleUvSurface = uvs;
                sampleUvPipe = uvp;
                samplePoint = pts;
                sampleT = ts; // last, it signals that the samples are ready
            }
        }

        /// <summary>
        /// The spine parameters of the samples, see <see cref="PipeContactCurve.GetSavePositions"/>.
        /// </summary>
        public double[] SampleParameters
        {
            get
            {
                EnsureSamples();
                return sampleT;
            }
        }

        /// <summary>
        /// The contact point at the spine parameter <paramref name="t"/> with its uv positions on both surfaces.
        /// </summary>
        public GeoPoint Evaluate(double t, out GeoPoint2D uvSurface, out GeoPoint2D uvPipe)
        {
            EnsureSamples();
            int i = SampleIndex(t);
            GeoPoint2D uv = sampleUvSurface[i];
            if (!Solve(t, ref uv))
            {   // should not happen, maybe a better start value helps
                GeoPoint2D fromPosition = NearestPeriod(surface, surface.PositionOf(SpinePoint(t)), sampleUvSurface[i]);
                uv = fromPosition;
                if (!Solve(t, ref uv)) uv = fromPosition;
            }
            uvSurface = NearestPeriod(surface, uv, sampleUvSurface[i]) + Correction(t);
            GeoPoint p = surface.PointAt(uvSurface);
            uvPipe = pipe.CirclePosition(p, SpineParameter(t));
            uvPipe.x += t - SpineParameter(t);
            uvPipe = NearestPeriod(pipe, uvPipe, sampleUvPipe[i]);
            return p;
        }

        /// <summary>
        /// Like <see cref="Evaluate"/>, together with the derivatives with respect to the spine parameter.
        /// </summary>
        public GeoPoint Derivatives(double t, out GeoPoint2D uvSurface, out GeoPoint2D uvPipe, out GeoVector derivative, out GeoVector2D derivativeSurface, out GeoVector2D derivativePipe)
        {
            GeoPoint p = Evaluate(t, out uvSurface, out uvPipe);
            derivative = GeoVector.NullVector;
            derivativeSurface = derivativePipe = GeoVector2D.NullVector;
            if (Frame(uvSurface, out _, out GeoVector du, out GeoVector dv, out _, out GeoVector ou, out GeoVector ov))
            {   // S'(t) = O_u u'(t) + O_v v'(t)
                if (LeastSquares(ou, ov, SpineDirection(t), out double u1, out double v1))
                {
                    derivativeSurface = new GeoVector2D(u1, v1) + CorrectionDerivative;
                    derivative = derivativeSurface.x * du + derivativeSurface.y * dv;
                }
            }
            GeoVector pu = pipe.UDirection(uvPipe), pv = pipe.VDirection(uvPipe);
            if (LeastSquares(pu, pv, derivative, out double up, out double vp)) derivativePipe = new GeoVector2D(up, vp);
            return p;
        }

        /// <summary>
        /// The spine parameter of the point of this curve closest to <paramref name="p"/>, by Newton, started at the closest sample.
        /// </summary>
        public double ParameterOf(GeoPoint p)
        {
            EnsureSamples();
            int best = 0;
            double minDist = double.MaxValue;
            for (int i = 0; i < samplePoint.Length; i++)
            {
                double d = samplePoint[i] | p;
                if (d < minDist)
                {
                    minDist = d;
                    best = i;
                }
            }
            double t = sampleT[best];
            double dt = sampleT.Length > 1 ? Math.Abs(sampleT[1] - sampleT[0]) : 1.0;
            for (int i = 0; i < 30; i++)
            {
                GeoPoint c = Derivatives(t, out _, out _, out GeoVector d, out _, out _);
                double dd = d * d;
                if (dd == 0.0 || double.IsNaN(dd)) break;
                double step = ((p - c) * d) / dd;
                if (Math.Abs(step) > 2 * dt) step = Math.Sign(step) * 2 * dt; // stay in the region of the samples
                t += step;
                if (Math.Abs(step) < 1e-15 * (1.0 + Math.Abs(t))) break;
            }
            return t;
        }

        #region ISerializable, IJsonSerialize
        protected PipeContact(SerializationInfo info, StreamingContext context)
        {
            surface = info.GetValue("Surface", typeof(ISurface)) as ISurface;
            pipe = info.GetValue("Pipe", typeof(SweptCircleSurface)) as SweptCircleSurface;
            tA = info.GetDouble("TA");
            tB = info.GetDouble("TB");
            uvHint = (GeoPoint2D)info.GetValue("UvHint", typeof(GeoPoint2D));
            endA = info.GetDouble("EndA");
            endB = info.GetDouble("EndB");
            correctionA = (GeoVector2D)info.GetValue("CorrectionA", typeof(GeoVector2D));
            correctionB = (GeoVector2D)info.GetValue("CorrectionB", typeof(GeoVector2D));
        }
        public void GetObjectData(SerializationInfo info, StreamingContext context)
        {
            info.AddValue("Surface", surface);
            info.AddValue("Pipe", pipe);
            info.AddValue("TA", tA);
            info.AddValue("TB", tB);
            info.AddValue("UvHint", uvHint);
            info.AddValue("EndA", endA);
            info.AddValue("EndB", endB);
            info.AddValue("CorrectionA", correctionA);
            info.AddValue("CorrectionB", correctionB);
        }
        public void GetObjectData(IJsonWriteData data)
        {
            data.AddProperty("Surface", surface);
            data.AddProperty("Pipe", pipe);
            data.AddProperty("TA", tA);
            data.AddProperty("TB", tB);
            data.AddProperty("UvHint", uvHint);
            data.AddProperty("EndA", endA);
            data.AddProperty("EndB", endB);
            data.AddProperty("CorrectionA", new double[] { correctionA.x, correctionA.y });
            data.AddProperty("CorrectionB", new double[] { correctionB.x, correctionB.y });
        }
        public void SetObjectData(IJsonReadData data)
        {   // the surfaces may still be empty here, the samples are calculated on demand
            surface = data.GetProperty<ISurface>("Surface");
            pipe = data.GetProperty<SweptCircleSurface>("Pipe");
            tA = data.GetProperty<double>("TA");
            tB = data.GetProperty<double>("TB");
            uvHint = data.GetProperty<GeoPoint2D>("UvHint");
            endA = data.GetProperty<double>("EndA");
            endB = data.GetProperty<double>("EndB");
            double[] a = data.GetProperty<double[]>("CorrectionA"), b = data.GetProperty<double[]>("CorrectionB");
            correctionA = new GeoVector2D(a[0], a[1]);
            correctionB = new GeoVector2D(b[0], b[1]);
        }
        #endregion
    }

    /// <summary>
    /// The curve along which a pipe (a <see cref="SweptCircleSurface"/>) touches a surface, e.g. the edge between a
    /// fillet and a face it connects. See <see cref="PipeContact"/> for the definition. The curve is parametrized
    /// linearly by the spine parameter of the pipe between <see cref="t0"/> and <see cref="t1"/>. Its 2d curves on the
    /// touched surface and on the pipe are the exact <see cref="PipeContactCurve2D"/>, see <see cref="CurveOnSurface"/>.
    /// </summary>
    [Serializable]
    internal class PipeContactCurve : GeneralCurve, ISerializable, IJsonSerialize
    {
        private PipeContact contact;
        private double t0, t1; // the spine parameters at the start and end point
        private double length = -1.0; // cached, see Length

        /// <summary>
        /// The curve along which <paramref name="pipe"/> touches <paramref name="surface"/> from <paramref name="startPoint"/>
        /// to <paramref name="endPoint"/>. Both points must be (close to) points of contact, i.e. on <paramref name="surface"/>
        /// and on the circle of <paramref name="pipe"/> at <paramref name="tStart"/> and <paramref name="tEnd"/> respectively.
        /// </summary>
        public static PipeContactCurve Create(ISurface surface, SweptCircleSurface pipe, GeoPoint startPoint, double tStart, GeoPoint endPoint, double tEnd)
        {
            PipeContact contact = new PipeContact(surface, pipe, tStart, tEnd, surface.PositionOf(startPoint));
            // the spine parameters of the end points may be a little imprecise: the closest points of the curve
            double s = contact.ParameterOf(startPoint);
            double e = contact.ParameterOf(endPoint);
            if (double.IsNaN(s) || double.IsNaN(e) || Math.Sign(e - s) != Math.Sign(tEnd - tStart)) return new PipeContactCurve(contact, tStart, tEnd);
            // and the curve passes exactly through the end points
            contact.SetEnds(s, startPoint, e, endPoint);
            return new PipeContactCurve(contact, s, e);
        }
        private PipeContactCurve(PipeContact contact, double t0, double t1)
        {
            this.contact = contact;
            this.t0 = t0;
            this.t1 = t1;
        }
        protected PipeContactCurve() { } // for the JSON serialization

        internal PipeContact Contact => contact;
        private double SpineParameter(double position) => t0 + position * (t1 - t0);
        private double Position(double t) => (t - t0) / (t1 - t0);

        /// <summary>
        /// The exact 2d curve of this curve on <paramref name="onThis"/>, when it is the touched surface or the pipe (or has
        /// the same parametrization), otherwise null.
        /// </summary>
        public ICurve2D CurveOnSurface(ISurface onThis)
        {
            foreach (bool onPipe in new bool[] { false, true })
            {
                ISurface s = onPipe ? contact.Pipe : contact.Surface;
                if (onThis.GetType() != s.GetType()) continue;
                bool same = true;
                foreach (double pos in new double[] { 0.0, 0.25, 0.5, 0.75, 1.0 })
                {
                    GeoPoint p = contact.Evaluate(SpineParameter(pos), out GeoPoint2D uvSurface, out GeoPoint2D uvPipe);
                    if ((onThis.PointAt(onPipe ? uvPipe : uvSurface) | p) > 10 * Precision.eps)
                    {
                        same = false;
                        break;
                    }
                }
                if (same) return new PipeContactCurve2D(contact, onPipe, t0, t1, ModOp2D.Identity);
            }
            return null;
        }

        public override IGeoObject Clone()
        {
            PipeContactCurve res = new PipeContactCurve(contact, t0, t1);
            res.CopyAttributes(this);
            return res;
        }
        public override void CopyGeometry(IGeoObject ToCopyFrom)
        {
            if (ToCopyFrom is PipeContactCurve other)
            {
                contact = other.contact;
                t0 = other.t0;
                t1 = other.t1;
                InvalidateSecondaryData();
            }
        }
        public override void Modify(ModOp m)
        {
            contact = contact.GetModified(m);
            InvalidateSecondaryData();
        }
        public override GeoPoint PointAt(double Position)
        {
            return contact.Evaluate(SpineParameter(Position), out _, out _);
        }
        public override GeoVector DirectionAt(double Position)
        {
            contact.Derivatives(SpineParameter(Position), out _, out _, out GeoVector derivative, out _, out _);
            return (t1 - t0) * derivative;
        }
        public override GeoPoint StartPoint
        {
            get => PointAt(0.0);
            set
            {   // move the start along the curve
                t0 = contact.ParameterOf(value);
                InvalidateSecondaryData();
            }
        }
        public override GeoPoint EndPoint
        {
            get => PointAt(1.0);
            set
            {
                t1 = contact.ParameterOf(value);
                InvalidateSecondaryData();
            }
        }
        public override double PositionOf(GeoPoint p)
        {
            double t = contact.ParameterOf(p);
            if (double.IsNaN(t)) return base.PositionOf(p);
            return Position(t);
        }
        public override bool IsClosed => Precision.IsEqual(StartPoint, EndPoint);
        /// <summary>
        /// The length, integrated by Gauss-Legendre (5 points) between the base points.
        /// </summary>
        public override double Length
        {
            get
            {
                if (length < 0.0)
                {
                    double[] nodes = { -0.906179845938664, -0.538469310105683, 0.0, 0.538469310105683, 0.906179845938664 };
                    double[] weights = { 0.236926885056189, 0.478628670499366, 0.568888888888889, 0.478628670499366, 0.236926885056189 };
                    double[] positions = GetBasePoints();
                    double sum = 0.0;
                    for (int i = 0; i < positions.Length - 1; i++)
                    {
                        double half = (positions[i + 1] - positions[i]) / 2.0, middle = (positions[i + 1] + positions[i]) / 2.0;
                        for (int j = 0; j < nodes.Length; j++) sum += weights[j] * half * DirectionAt(middle + half * nodes[j]).Length;
                    }
                    length = sum;
                }
                return length;
            }
        }
        protected override void InvalidateSecondaryData()
        {
            base.InvalidateSecondaryData();
            length = -1.0;
        }
        public override void Reverse()
        {
            (t0, t1) = (t1, t0);
            InvalidateSecondaryData();
        }
        public override void Trim(double StartPos, double EndPos)
        {
            double s = SpineParameter(StartPos), e = SpineParameter(EndPos);
            t0 = s;
            t1 = e;
            InvalidateSecondaryData();
        }
        public override ICurve[] Split(double Position)
        {
            double t = SpineParameter(Position);
            return new ICurve[] { new PipeContactCurve(contact, t0, t), new PipeContactCurve(contact, t, t1) };
        }
        public override bool SameGeometry(ICurve other, double precision)
        {
            if (other is PipeContactCurve pcc && pcc.contact == contact)
            {
                if ((pcc.t0 == t0 && pcc.t1 == t1) || (pcc.t0 == t1 && pcc.t1 == t0)) return true;
            }
            return base.SameGeometry(other, precision);
        }
        protected override double[] GetBasePoints()
        {
            List<double> res = new List<double> { 0.0, 0.25, 0.5, 0.75, 1.0 };
            foreach (double t in contact.SampleParameters)
            {
                double pos = Position(t);
                if (pos > 0.0 && pos < 1.0) res.Add(pos);
            }
            res.Sort();
            res.RemoveDuplicatesWithTolerance(1e-6);
            return res.ToArray();
        }

        #region ISerializable, IJsonSerialize
        protected PipeContactCurve(SerializationInfo info, StreamingContext context) : base(info, context)
        {
            contact = info.GetValue("Contact", typeof(PipeContact)) as PipeContact;
            t0 = info.GetDouble("T0");
            t1 = info.GetDouble("T1");
        }
        public override void GetObjectData(SerializationInfo info, StreamingContext context)
        {
            base.GetObjectData(info, context);
            info.AddValue("Contact", contact);
            info.AddValue("T0", t0);
            info.AddValue("T1", t1);
        }
        public void GetObjectData(IJsonWriteData data)
        {
            data.AddProperty("Contact", contact);
            data.AddProperty("T0", t0);
            data.AddProperty("T1", t1);
        }
        public void SetObjectData(IJsonReadData data)
        {
            contact = data.GetProperty<PipeContact>("Contact");
            t0 = data.GetProperty<double>("T0");
            t1 = data.GetProperty<double>("T1");
        }
        #endregion
    }

    /// <summary>
    /// The 2d curve of a <see cref="PipeContactCurve"/> on the touched surface or on the pipe: the uv positions of
    /// <see cref="PipeContact"/>, transformed by <see cref="modOp"/> (when the surface of the face changes its uv system,
    /// or the curve is moved by a period).
    /// </summary>
    [Serializable]
    internal class PipeContactCurve2D : GeneralCurve2D, ISerializable, IJsonSerialize
    {
        private PipeContact contact;
        private bool onPipe; // the curve on the pipe, otherwise on the touched surface
        private double t0, t1; // the spine parameters at the start and end point
        private ModOp2D modOp;
        private GeoPoint2D? startPointSet, endPointSet; // explicitly set end points (the edges sometimes adjust them to the vertices)
        private BSpline2D approximation; // see Approximation

        public PipeContactCurve2D(PipeContact contact, bool onPipe, double t0, double t1, ModOp2D modOp)
        {
            this.contact = contact;
            this.onPipe = onPipe;
            this.t0 = t0;
            this.t1 = t1;
            this.modOp = modOp;
        }
        protected PipeContactCurve2D() { } // for the JSON serialization

        private double SpineParameter(double position) => t0 + position * (t1 - t0);

        /// <summary>
        /// Clears the cached data of this object and of the base class (points and directions of the triangulation).
        /// </summary>
        private void Invalidate()
        {
            ClearTriangulation();
            approximation = null;
        }

        /// <summary>
        /// A BSpline, which approximates this curve to a fraction of its length in space (like <see cref="ProjectedCurve.ApproxBSpline2D"/>).
        /// The area, the extent and the sweep angle are taken from it: the default implementation uses a rough approximation by arcs.
        /// </summary>
        private BSpline2D Approximation
        {
            get
            {
                if (approximation == null)
                {
                    ISurface surface = onPipe ? contact.Pipe : contact.Surface;
                    double length = 0.0, scale = 0.0;
                    GeoPoint last = GeoPoint.Origin;
                    for (int i = 0; i <= 16; i++)
                    {
                        GeoPoint2D uv = PointAt(i / 16.0);
                        GeoPoint p = surface.PointAt(modOp.GetInverse() * uv);
                        if (i > 0) length += p | last;
                        last = p;
                        surface.DerivativeAt(modOp.GetInverse() * uv, out _, out GeoVector du, out GeoVector dv);
                        scale = Math.Max(scale, Math.Max(du.Length, dv.Length));
                    }
                    double precision = Math.Max(length * 1e-6, Precision.eps) / Math.Max(scale, 1e-12);
                    approximation = BSpline2D.Approximate(pos => PointAt(pos), precision);
                }
                return approximation;
            }
        }
        public override double GetArea() => Approximation.GetArea();
        public override double GetAreaFromPoint(GeoPoint2D p) => Approximation.GetAreaFromPoint(p);
        public override BoundingRect GetExtent() => Approximation.GetExtent();
        public override double Sweep => Approximation.Sweep;
        public override bool IsClosed => false; // like a curve of an intersection, see ProjectedCurve.IsClosed

        public override GeoPoint2D PointAt(double Position)
        {
            contact.Evaluate(SpineParameter(Position), out GeoPoint2D uvSurface, out GeoPoint2D uvPipe);
            return modOp * (onPipe ? uvPipe : uvSurface);
        }
        public override GeoVector2D DirectionAt(double Position)
        {
            contact.Derivatives(SpineParameter(Position), out _, out _, out _, out GeoVector2D derivativeSurface, out GeoVector2D derivativePipe);
            return (t1 - t0) * (modOp * (onPipe ? derivativePipe : derivativeSurface));
        }
        public override bool TryPointDeriv2At(double position, out GeoPoint2D point, out GeoVector2D deriv1, out GeoVector2D deriv2)
        {
            const double h = 1e-6;
            point = PointAt(position);
            deriv1 = DirectionAt(position);
            deriv2 = (1.0 / (2.0 * h)) * (DirectionAt(position + h) - DirectionAt(position - h));
            return true;
        }
        public override GeoPoint2D StartPoint
        {
            get => startPointSet ?? PointAt(0.0);
            set
            {
                startPointSet = value;
                Invalidate();
            }
        }
        public override GeoPoint2D EndPoint
        {
            get => endPointSet ?? PointAt(1.0);
            set
            {
                endPointSet = value;
                Invalidate();
            }
        }
        public override GeoVector2D StartDirection => DirectionAt(0.0);
        public override GeoVector2D EndDirection => DirectionAt(1.0);
        public override double PositionOf(GeoPoint2D p)
        {   // through the 3d point, which is not affected by periods
            GeoPoint2D uv = modOp.GetInverse() * p;
            GeoPoint p3d = onPipe ? contact.Pipe.PointAt(uv) : contact.Surface.PointAt(uv);
            double t = contact.ParameterOf(p3d);
            if (double.IsNaN(t)) return base.PositionOf(p);
            return (t - t0) / (t1 - t0);
        }
        internal override void GetTriangulationPoints(out GeoPoint2D[] interpol, out double[] interparam)
        {
            List<double> positions = new List<double> { 0.0, 0.25, 0.5, 0.75, 1.0 };
            foreach (double t in contact.SampleParameters)
            {
                double pos = (t - t0) / (t1 - t0);
                if (pos > 0.0 && pos < 1.0) positions.Add(pos);
            }
            positions.Sort();
            positions.RemoveDuplicatesWithTolerance(1e-6);
            interparam = positions.ToArray();
            interpol = new GeoPoint2D[interparam.Length];
            for (int i = 0; i < interparam.Length; i++) interpol[i] = PointAt(interparam[i]);
            if (startPointSet.HasValue) interpol[0] = startPointSet.Value;
            if (endPointSet.HasValue) interpol[interpol.Length - 1] = endPointSet.Value;
        }
        public override void Reverse()
        {
            (t0, t1) = (t1, t0);
            (startPointSet, endPointSet) = (endPointSet, startPointSet);
            Invalidate();
        }
        public override ICurve2D Clone()
        {
            PipeContactCurve2D res = new PipeContactCurve2D(contact, onPipe, t0, t1, modOp);
            res.startPointSet = startPointSet;
            res.endPointSet = endPointSet;
            return res;
        }
        public override void Copy(ICurve2D toCopyFrom)
        {
            if (toCopyFrom is PipeContactCurve2D other)
            {
                contact = other.contact;
                onPipe = other.onPipe;
                t0 = other.t0;
                t1 = other.t1;
                modOp = other.modOp;
                startPointSet = other.startPointSet;
                endPointSet = other.endPointSet;
                Invalidate();
            }
        }
        public override ICurve2D Trim(double StartPos, double EndPos)
        {
            PipeContactCurve2D res = new PipeContactCurve2D(contact, onPipe, SpineParameter(StartPos), SpineParameter(EndPos), modOp);
            if (StartPos == 0.0) res.startPointSet = startPointSet;
            if (EndPos == 1.0) res.endPointSet = endPointSet;
            return res;
        }
        public override ICurve2D[] Split(double Position)
        {
            double t = SpineParameter(Position);
            PipeContactCurve2D first = new PipeContactCurve2D(contact, onPipe, t0, t, modOp);
            PipeContactCurve2D second = new PipeContactCurve2D(contact, onPipe, t, t1, modOp);
            first.startPointSet = startPointSet;
            second.endPointSet = endPointSet;
            return new ICurve2D[] { first, second };
        }
        public override void Move(double x, double y)
        {
            modOp = ModOp2D.Translate(x, y) * modOp;
            GeoVector2D offset = new GeoVector2D(x, y);
            if (startPointSet.HasValue) startPointSet = startPointSet.Value + offset;
            if (endPointSet.HasValue) endPointSet = endPointSet.Value + offset;
            Invalidate();
        }
        public override ICurve2D GetModified(ModOp2D m)
        {
            PipeContactCurve2D res = new PipeContactCurve2D(contact, onPipe, t0, t1, m * modOp);
            if (startPointSet.HasValue) res.startPointSet = m * startPointSet.Value;
            if (endPointSet.HasValue) res.endPointSet = m * endPointSet.Value;
            return res;
        }

        #region ISerializable, IJsonSerialize
        protected PipeContactCurve2D(SerializationInfo info, StreamingContext context) : base(info, context)
        {
            contact = info.GetValue("Contact", typeof(PipeContact)) as PipeContact;
            onPipe = info.GetBoolean("OnPipe");
            t0 = info.GetDouble("T0");
            t1 = info.GetDouble("T1");
            modOp = (ModOp2D)info.GetValue("ModOp", typeof(ModOp2D));
        }
        public override void GetObjectData(SerializationInfo info, StreamingContext context)
        {
            base.GetObjectData(info, context);
            info.AddValue("Contact", contact);
            info.AddValue("OnPipe", onPipe);
            info.AddValue("T0", t0);
            info.AddValue("T1", t1);
            info.AddValue("ModOp", modOp);
        }
        public void GetObjectData(IJsonWriteData data)
        {
            JSonGetObjectData(data);
            data.AddProperty("Contact", contact);
            data.AddProperty("OnPipe", onPipe);
            data.AddProperty("T0", t0);
            data.AddProperty("T1", t1);
            data.AddProperty("ModOp", new double[] { modOp[0, 0], modOp[0, 1], modOp[0, 2], modOp[1, 0], modOp[1, 1], modOp[1, 2] });
        }
        public void SetObjectData(IJsonReadData data)
        {
            JSonSetObjectData(data);
            contact = data.GetProperty<PipeContact>("Contact");
            onPipe = data.GetProperty<bool>("OnPipe");
            t0 = data.GetProperty<double>("T0");
            t1 = data.GetProperty<double>("T1");
            double[] m = data.GetProperty<double[]>("ModOp");
            modOp = new ModOp2D(m[0], m[1], m[2], m[3], m[4], m[5]);
        }
        #endregion
    }
}
