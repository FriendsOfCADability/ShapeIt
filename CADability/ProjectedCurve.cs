using CADability.Curve2D;
using CADability.GeoObject;
using System;
using System.Runtime.Serialization;

namespace CADability
{
    /// <summary>
    /// A 2d curve as a projection of a 3d curve onto a surface. Sometimes it is easier to calculate points in 3d than in 2d. Then we use this as a more
    /// exact for of the curve than we get, when we approximate the curve in 2d.
    /// <para>
    /// On a periodic surface the uv values of a point are only defined up to whole periods. The curve runs on
    /// continuously from an anchor: the uv value it has at one parameter of the 3d curve, fixed when the curve is made.
    /// Every other point is PositionOf of its 3d point, moved by whole periods next to its neighbours on the curve.
    /// The anchor is absolute, so it stays valid when the domain of the surface is set or changed later, which changes
    /// what PositionOf returns. <see cref="Move"/> moves the anchor.
    /// </para>
    /// </summary>
    [Serializable()]
    public class ProjectedCurve : GeneralCurve2D, ISerializable
    {
        private double startParam; // start parameter on the 3d curve, together wit endParam also specifies the orientation
        private double endParam; // on the 3d curve
        ICurve curve3D;
        ISurface surface;
        GeoPoint2D anchorUv; // the uv value of this curve at the parameter anchor3d of the 3d curve
        double anchor3d;
        bool hasAnchor; // false only for a curve read from a file written before the anchor existed
        BoundingRect windowOfAnOlderFile; // such a file has the domain the curve used, it gives the anchor on first use
        /// <summary>The number of points of the chain which carries the anchor along the curve, see <see cref="ContinuousChain"/>.</summary>
        private const int chainLength = 17;
#if DEBUG
        static int debugCounter = 0;
        private int debugCount; // to identify instance when debugging
#endif
        private BSpline2D approxBSpline2D = null;
        public BSpline2D ApproxBSpline2D
        {
            get
            {
                if (approxBSpline2D == null)
                {
                    double prec = Math.Max(curve3D.Length * 1e-5, Precision.eps);
                    GeoPoint2D[] chain = AnchoredChain();
                    approxBSpline2D = BSpline2D.Approximate(pos => ContinuousUv(pos, chain), prec);
                }
                return approxBSpline2D;
            }
        }
        /// <summary>
        /// The projection of <paramref name="curve3D"/> onto <paramref name="surface"/>. On a periodic surface, where the curve
        /// may lie in any period, <paramref name="domain"/> tells which: the curve is moved by whole periods as close as
        /// possible to it. An empty domain leaves it where PositionOf puts its start, in the domain of the surface.
        /// <paramref name="precision"/> is not used.
        /// </summary>
        public ProjectedCurve(ICurve curve3D, ISurface surface, bool forward, BoundingRect domain, double precision = 0.0)
        {
#if DEBUG
            debugCount = debugCounter++;
#endif
            this.curve3D = curve3D; // keep in mind, the curve is not cloned, curve3D should not be modified after this
            this.surface = surface;
            if (forward)
            {
                startParam = 0.0;
                endParam = 1.0;
            }
            else
            {
                startParam = 1.0;
                endParam = 0.0;
            }
            SetAnchor(domain);
#if DEBUG
            this.MakeTriangulation();
#endif
        }
        /// <summary>
        /// The projection of the part from <paramref name="startParam"/> to <paramref name="endParam"/> of
        /// <paramref name="curve3D"/>, moved by whole periods as close as possible to <paramref name="domain"/>, see the
        /// other constructor.
        /// </summary>
        public ProjectedCurve(ICurve curve3D, ISurface surface, double startParam, double endParam, BoundingRect domain)
        {
#if DEBUG
            debugCount = debugCounter++;
#endif
            this.curve3D = curve3D;
            this.surface = surface;
            this.startParam = startParam;
            this.endParam = endParam;
            SetAnchor(domain);
        }
        /// <summary>A copy or a part of another projected curve, with an anchor taken from it, so that both lie in the same periods.</summary>
        private ProjectedCurve(ICurve curve3D, ISurface surface, double startParam, double endParam, double anchor3d, GeoPoint2D anchorUv)
        {
#if DEBUG
            debugCount = debugCounter++;
#endif
            this.curve3D = curve3D;
            this.surface = surface;
            this.startParam = startParam;
            this.endParam = endParam;
            this.anchor3d = anchor3d;
            this.anchorUv = anchorUv;
            hasAnchor = true;
        }
        /// <summary>
        /// Fixes the anchor: at the first point of the chain which is not at a pole, with the whole chain moved by periods
        /// as close as possible to <paramref name="window"/>, as SurfaceHelper.UnwrapPeriodic does.
        /// </summary>
        private void SetAnchor(BoundingRect window)
        {
            GeoPoint2D[] chain = ContinuousChain(out int first);
            first = Math.Max(first, 0);
            anchor3d = Get3dParameter(first / (double)(chain.Length - 1));
            anchorUv = chain[first] + WindowShift(chain, window);
            hasAnchor = true;
        }
        private void EnsureAnchor()
        {
            if (!hasAnchor) SetAnchor(windowOfAnOlderFile);
        }
        /// <summary>
        /// The uv values of the curve at <see cref="chainLength"/> parameters: from PositionOf, which honours the domain of
        /// the surface, each moved by whole periods next to its predecessor - the curve as a continuous row in the
        /// parameter plane. A point at a pole, where the periodic parameter may take any value, takes the value of its
        /// neighbour and does not anchor the next one. <paramref name="first"/> is the first point which is not at a pole,
        /// -1 when there is none.
        /// </summary>
        private GeoPoint2D[] ContinuousChain(out int first)
        {
            GeoPoint2D[] res = new GeoPoint2D[chainLength];
            bool[] pole = new bool[chainLength];
            for (int i = 0; i < chainLength; i++)
            {
                res[i] = surface.PositionOf(curve3D.PointAt(Get3dParameter(i / (double)(chainLength - 1))));
                pole[i] = IsAtPole(res[i]);
            }
            first = Array.IndexOf(pole, false);
            if (first < 0) return res; // nothing but a pole
            for (int i = first + 1; i < chainLength; i++)
            {
                if (pole[i]) res[i] = res[i - 1];
                else MoveNextTo(ref res[i], res[i - 1]);
            }
            for (int i = first - 1; i >= 0; i--) res[i] = res[i + 1];
            return res;
        }
        /// <summary>The <see cref="ContinuousChain"/> moved by whole periods to the anchor.</summary>
        private GeoPoint2D[] AnchoredChain()
        {
            EnsureAnchor();
            GeoPoint2D[] chain = ContinuousChain(out _);
            int k = Math.Max(0, Math.Min(chain.Length - 1, (int)Math.Round(Get2dParameter(anchor3d) * (chain.Length - 1))));
            GeoPoint2D anchored = chain[k];
            MoveNextTo(ref anchored, anchorUv);
            GeoVector2D shift = anchored - chain[k];
            if (shift.x != 0.0 || shift.y != 0.0)
            {
                for (int i = 0; i < chain.Length; i++) chain[i] += shift;
            }
            return chain;
        }
        /// <summary>At a pole a periodic parameter may take any value without moving the point.</summary>
        private bool IsAtPole(GeoPoint2D uv)
        {
            GeoPoint p = surface.PointAt(uv);
            if (surface.IsUPeriodic && (surface.PointAt(new GeoPoint2D(uv.x + surface.UPeriod / 4, uv.y)) | p) < Precision.eps) return true;
            if (surface.IsVPeriodic && (surface.PointAt(new GeoPoint2D(uv.x, uv.y + surface.VPeriod / 4)) | p) < Precision.eps) return true;
            return false;
        }
        /// <summary>Moves <paramref name="uv"/> by whole periods next to <paramref name="next"/>.</summary>
        private void MoveNextTo(ref GeoPoint2D uv, GeoPoint2D next)
        {
            if (surface.IsUPeriodic && surface.UPeriod > 0.0) uv.x -= Math.Round((uv.x - next.x) / surface.UPeriod) * surface.UPeriod;
            if (surface.IsVPeriodic && surface.VPeriod > 0.0) uv.y -= Math.Round((uv.y - next.y) / surface.VPeriod) * surface.VPeriod;
        }
        /// <summary>The uv value at the 2d parameter <paramref name="pos"/>, continuous with the anchored <paramref name="chain"/>.</summary>
        private GeoPoint2D ContinuousUv(double pos, GeoPoint2D[] chain)
        {
            // Get3dParameter also reflects orientation
            GeoPoint2D uv = surface.PositionOf(curve3D.PointAt(Get3dParameter(pos)));
            int k = Math.Max(0, Math.Min(chain.Length - 1, (int)Math.Round(pos * (chain.Length - 1))));
            MoveNextTo(ref uv, chain[k]);
            return uv;
        }
        /// <summary>
        /// The whole periods which move the <paramref name="chain"/> as close as possible to <paramref name="window"/>: the
        /// center of its extent next to the center of the window. Nothing for an empty or unbounded window.
        /// </summary>
        private GeoVector2D WindowShift(GeoPoint2D[] chain, BoundingRect window)
        {
            if (window.IsEmpty()) return GeoVector2D.NullVector;
            BoundingRect ext = BoundingRect.EmptyBoundingRect;
            foreach (GeoPoint2D uv in chain) ext.MinMax(uv);
            double du = 0.0, dv = 0.0;
            if (surface.IsUPeriodic && surface.UPeriod > 0.0 && window.Left > double.MinValue && window.Right < double.MaxValue)
                du = -Math.Round(((ext.Left + ext.Right) / 2 - (window.Left + window.Right) / 2) / surface.UPeriod) * surface.UPeriod;
            if (surface.IsVPeriodic && surface.VPeriod > 0.0 && window.Bottom > double.MinValue && window.Top < double.MaxValue)
                dv = -Math.Round(((ext.Bottom + ext.Top) / 2 - (window.Bottom + window.Top) / 2) / surface.VPeriod) * surface.VPeriod;
            return new GeoVector2D(du, dv);
        }
        internal override void GetTriangulationPoints(out GeoPoint2D[] interpol, out double[] interparam)
        {
            ApproxBSpline2D.GetTriangulationPoints(out interpol, out interparam);
        }
        public override double[] GetInflectionPoints()
        {
            return ApproxBSpline2D.GetInflectionPoints();
        }
#if DEBUG
        public void DebugTest()
        {
            GeoPoint2D[] points;
            GeoVector2D[] directions;
            double[] parameters;
            GetTriangulationBasis(out points, out directions, out parameters);
            GeoPoint2D sp = this.StartPoint;
            GeoPoint2D ep = this.EndPoint;
        }
#endif
        public ICurve Curve3DFromParams
        {
            get
            {
                ICurve res;
                res = curve3D.Clone();
                if (IsReverse)
                {
                    res.Reverse();
                    if (startParam == 0.0 && endParam == 1.0) return res;
                    else
                    {
                        res.Trim(1 - startParam, 1 - endParam);
                        return res;
                    }
                }
                else
                {
                    if (startParam == 0.0 && endParam == 1.0) return res;
                    else
                    {
                        res.Trim(startParam, endParam);
                        return res;
                    }
                }
            }
        }
        public ICurve Curve3D
        {
            get
            {
                return curve3D;
            }
        }
        public ISurface Surface
        {
            get
            {
                return surface;
            }
        }
        public bool IsReverse
        {
            get => endParam < startParam;
            internal set
            {
                if (value != IsReverse)
                {
                    double tmp = startParam;
                    startParam = endParam;
                    endParam = tmp;
                    InvalidateSecondaryData();
                }
            }
        }
        #region ICurve2D Members
        private double Get3dParameter(double par)
        {
            return startParam + par * (endParam - startParam);
        }
        private double Get2dParameter(double pos)
        {
            return (pos - startParam) / (endParam - startParam);
        }
        private void PointDirAt(double pos, out GeoPoint2D uv, out GeoVector2D dir)
        {
            ApproxBSpline2D.PointDirAt(pos, out uv, out dir); // ApproxBSpline2D is always parametrized from 0 to 1
        }

        internal void ReflectModification(ISurface surface, ICurve curve3d)
        {   // a face has been modified, in 2d there are no changes, but the surface and the 3d curve must be adopted
            this.surface = surface;
            this.curve3D = curve3d;
        }

        public override GeoVector2D DirectionAt(double Position)
        {
            GeoPoint2D loc;
            GeoVector2D res;
            PointDirAt(Position, out loc, out res);
            return res;
        }
        public override GeoPoint2D PointAt(double Position)
        {
            return ApproxBSpline2D.PointAt(Position); // it is synchronous with parameters too!
        }
        /// <summary>
        /// The position of <paramref name="p"/> on this curve, found on the 3d curve: on a periodic surface a point may
        /// be given in another period than the one this curve runs through, e.g. a point in the domain of a face when
        /// the curve crosses the seam of that domain.
        /// </summary>
        public override double PositionOf(GeoPoint2D p)
        {
            return Get2dParameter(curve3D.PositionOf(surface.PointAt(p)));
        }
        public override void Reverse()
        {
            (startParam, endParam) = (endParam, startParam);
            InvalidateSecondaryData();
        }
        public override ICurve2D Clone()
        {
            EnsureAnchor();
            ICurve2D res = new ProjectedCurve(curve3D, surface, startParam, endParam, anchor3d, anchorUv);
            res.UserData.CloneFrom(this.UserData);
            return res;
        }
        public override void Copy(ICurve2D toCopyFrom)
        {
            ProjectedCurve pc = toCopyFrom as ProjectedCurve;
            if (pc != null)
            {
                pc.EnsureAnchor();
                startParam = pc.startParam;
                endParam = pc.endParam;
                curve3D = pc.curve3D;
                surface = pc.surface;
                anchor3d = pc.anchor3d;
                anchorUv = pc.anchorUv;
                hasAnchor = true;
                InvalidateSecondaryData();
            }
        }
        /// <summary>
        /// The parts keep the periods of this curve: each gets an anchor from the middle of this curve's piece, which is
        /// never at a pole unless the whole piece is.
        /// </summary>
        public override ICurve2D Trim(double StartPos, double EndPos)
        {
            if (StartPos < EndPos)
            {
                double sp3d = Get3dParameter(StartPos);
                double ep3d = Get3dParameter(EndPos);
                double middle = (StartPos + EndPos) / 2.0;
                return new ProjectedCurve(curve3D, surface, sp3d, ep3d, Get3dParameter(middle), PointAt(middle));
            }
            else
            {
                // es geht bei einer geschlossenen Kurve über den Nahtpunkt
                double sp3d = Get3dParameter(StartPos);
                double ep3d = Get3dParameter(EndPos);
                ICurve c3d = curve3D.Clone();
                c3d.Trim(sp3d, ep3d);
                double middle = (StartPos + (EndPos + 1.0 - StartPos) / 2.0) % 1.0; // the middle of the piece across the seam
                return new ProjectedCurve(c3d, surface, 0, 1, 0.5, PointAt(middle));
            }
        }
        public override ICurve2D[] Split(double Position)
        {
            double sp3d = Get3dParameter(Position);
            if (Math.Abs(sp3d - startParam) < 1e-6 || Math.Abs(sp3d - endParam) < 1e-6) return new ICurve2D[] { Clone() };
            double middle1 = Position / 2.0, middle2 = (Position + 1.0) / 2.0;
            return new ICurve2D[] {
                new ProjectedCurve(curve3D, surface, startParam, sp3d, Get3dParameter(middle1), PointAt(middle1)),
                new ProjectedCurve(curve3D, surface, sp3d, endParam, Get3dParameter(middle2), PointAt(middle2)) };
        }
        public override void Move(double x, double y)
        {
            bool ok = true; // move the anchor by the period
            if (surface.IsUPeriodic)
            {
                double dx = x / surface.UPeriod;
                ok &= (Math.Abs(dx - Math.Round(dx)) < 1e-10);
            }
            else
            {
                ok &= x == 0.0;
            }
            if (surface.IsVPeriodic)
            {
                double dy = y / surface.VPeriod;
                ok &= (Math.Abs(dy - Math.Round(dy)) < 1e-10);
            }
            else
            {
                ok &= y == 0.0;
            }
            if (ok)
            {
                EnsureAnchor();
                anchorUv = anchorUv + new GeoVector2D(x, y);
                InvalidateSecondaryData();
            }
            else throw new ApplicationException("cannot move ProjectedCurve");
        }
        public override ICurve2D GetModified(ModOp2D m)
        {
            // actually we cannot modify a projected curve, because it relies on the 3d curve and the surface
            if (m.IsIdentity) return Clone();
            GeoPoint2D sp = m * StartPoint;
            GeoPoint2D ep = m * EndPoint;
            if (Precision.IsEqual(sp, EndPoint) && Precision.IsEqual(ep, StartPoint))
            {
                return this.CloneReverse(true);
            }
            return base.GetModified(m);
        }
        #endregion
        #region ISerializable Members
        protected ProjectedCurve(SerializationInfo info, StreamingContext context)
            : base(info, context)
        {
#if DEBUG
            debugCount = debugCounter++;
#endif
            curve3D = info.GetValue("Curve3D", typeof(ICurve)) as ICurve;
            surface = info.GetValue("Surface", typeof(ISurface)) as ISurface;
            startParam = info.GetDouble("StartParam");
            endParam = info.GetDouble("EndParam");
            try
            {
                anchorUv = (GeoPoint2D)info.GetValue("AnchorUv", typeof(GeoPoint2D));
                anchor3d = info.GetDouble("Anchor3d");
                hasAnchor = true;
            }
            catch (SerializationException)
            {   // written before the anchor existed: the anchor comes from the domain the curve was written with, on first
                // use - the 3d curve and the surface may not be complete while they are being read
                try
                {
                    windowOfAnOlderFile = (BoundingRect)info.GetValue("PeriodicDomain", typeof(BoundingRect));
                }
                catch (SerializationException)
                {
                    windowOfAnOlderFile = BoundingRect.EmptyBoundingRect;
                }
            }
        }
        /// <summary>
        /// Implements <see cref="ISerializable.GetObjectData"/>
        /// </summary>
        /// <param name="info">The <see cref="System.Runtime.Serialization.SerializationInfo"/> to populate with data.</param>
        /// <param name="context">The destination (<see cref="System.Runtime.Serialization.StreamingContext"/>) for this serialization.</param>
        public override void GetObjectData(SerializationInfo info, StreamingContext context)
        {
            base.GetObjectData(info, context);
            info.AddValue("Curve3D", curve3D);
            info.AddValue("Surface", surface);
            info.AddValue("StartParam", startParam);
            info.AddValue("EndParam", endParam);
            EnsureAnchor();
            info.AddValue("AnchorUv", anchorUv);
            info.AddValue("Anchor3d", anchor3d);
            // older versions move every point next to the center of this domain; the extent of the curve puts it where it is
            info.AddValue("PeriodicDomain", GetExtent());
        }

        public override bool TryPointDeriv2At(double position, out GeoPoint2D point, out GeoVector2D deriv, out GeoVector2D deriv2)
        {
            point = GeoPoint2D.Origin;
            deriv = deriv2 = GeoVector2D.NullVector;
            return false;
        }

        internal void InvalidateSecondaryData()
        {
            approxBSpline2D = null;
            ClearTriangulation();
        }

        #endregion
#if DEBUG
        public Polyline DebugPolyLine
        {
            get
            {
                GeoPoint[] pnts = new GeoPoint[100];
                for (int i = 0; i < pnts.Length; i++)
                {
                    pnts[i] = Plane.XYPlane.ToGlobal(PointAt(i / 99.0));

                }
                Polyline res = Polyline.Construct();
                res.SetPoints(pnts, false);
                return res;
            }
        }
        public ICurve DebugReprojectedCurve3D
        {
            get
            {
                GeoPoint[] pnts = new GeoPoint[100];
                for (int i = 0; i < pnts.Length; i++)
                {
                    pnts[i] = surface.PointAt(PointAt(i / 99.0));

                }
                Polyline res = Polyline.Construct();
                res.SetPoints(pnts, false);
                return res;
            }
        }
#endif
    }
}
