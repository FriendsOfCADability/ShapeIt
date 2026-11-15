using CADability.Curve2D;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Runtime.Serialization;

namespace CADability.GeoObject
{
    [Serializable()]
    public class OffsetSurface : ISurfaceImpl, ISerializable, IExportStep, IJsonSerialize
    {
        private ISurface baseSurface;
        private double offset;
        double umin, umax, vmin, vmax; // nur zur Kommunikation mit OCas nötig
        public OffsetSurface(ISurface baseSurface, double offset, BoundingRect? domain = null) : base(domain)
        {
            this.baseSurface = baseSurface;
            this.offset = offset;
            baseSurface.GetNaturalBounds(out umin, out umax, out vmin, out vmax);
        }
        public ISurface BaseSurface
        {
            get
            {
                return baseSurface;
            }
        }
        public double Offset
        {
            get
            {
                return offset;
            }
        }
        /// <summary>
        /// The iso curve of an <see cref="OffsetSurface"/>: the curve at a fixed u (or fixed v) parameter. The curve parameter
        /// 0..1 runs linear from pstart to pend on the free surface parameter; pstart is greater than pend when the curve
        /// is reversed. Point and derivative are taken from the offset surface itself, not from the iso curve of the base
        /// surface: the latter is the curve this one is parallel to, but its points and its direction are different.
        /// The offset surface is a copy of its own, which is never changed in place (see <see cref="Modify"/>): the surface
        /// of a face is changed in place, e.g. <see cref="ReverseOrientation"/> of a ruled surface inverts u, and the
        /// edges of the face keep their 3d curves. A curve sharing that surface would then describe a different curve.
        /// </summary>
        [Serializable]
        class FixedCurve : GeneralCurve, ISerializable, IJsonSerialize
        {
            OffsetSurface offsetSurface;
            bool fixedU; // true: u is fixed and the curve runs along v, false: v is fixed and the curve runs along u
            double param; // the fixed parameter
            double pstart, pend; // the free parameter at the start and at the end of the curve
            public FixedCurve(OffsetSurface offsetSurface, double param, double pstart, double pend, bool fixedU)
            {
                this.fixedU = fixedU;
                this.offsetSurface = offsetSurface;
                this.param = param;
                this.pstart = pstart;
                this.pend = pend;
            }
            private GeoPoint2D UvAt(double position)
            {
                double p = pstart + position * (pend - pstart);
                if (fixedU) return new GeoPoint2D(param, p);
                else return new GeoPoint2D(p, param);
            }
            protected override double[] GetBasePoints()
            {   // the iso curve of the base surface knows where its curvature changes (e.g. the knots of a spline),
                // the offset curve changes at the same parameters. The base curve runs from the smaller to the greater
                // parameter, linear in the free surface parameter like this curve.
                double pmin = Math.Min(pstart, pend), pmax = Math.Max(pstart, pend);
                List<double> res = new List<double>();
                if (pmax > pmin)
                {
                    try
                    {
                        ICurve baseCurve = fixedU ? offsetSurface.baseSurface.FixedU(param, pmin, pmax) : offsetSurface.baseSurface.FixedV(param, pmin, pmax);
                        double[] sp = baseCurve?.GetSavePositions();
                        if (sp != null)
                        {
                            foreach (double s in sp)
                            {
                                if (s > 0.0 && s < 1.0) res.Add(pstart < pend ? s : 1.0 - s);
                            }
                        }
                    }
                    catch (NotImplementedException) { }
                    catch (ApplicationException) { }
                }
                res.Sort();
                res.Insert(0, 0.0);
                res.Add(1.0);
                for (int i = res.Count - 2; i > 0; --i)
                {   // keep 0 and 1, drop inner positions too close to their neighbour
                    if (res[i] - res[i - 1] < 1e-3 || res[i + 1] - res[i] < 1e-3) res.RemoveAt(i);
                }
                while (res.Count < 5)
                {   // refine the longest interval: start and end point alone are too few for the tetrahedron hull
                    int longest = 0;
                    for (int i = 1; i < res.Count - 1; i++) if (res[i + 1] - res[i] > res[longest + 1] - res[longest]) longest = i;
                    res.Insert(longest + 1, (res[longest] + res[longest + 1]) / 2.0);
                }
                return res.ToArray();
            }
            public override GeoVector DirectionAt(double Position)
            {   // d/dPosition of offsetSurface.PointAt(UvAt(Position)): the partial derivative of the offset surface
                // in the free direction, scaled by the length of the parameter interval
                GeoPoint2D uv = UvAt(Position);
                if (fixedU) return (pend - pstart) * offsetSurface.VDirection(uv);
                else return (pend - pstart) * offsetSurface.UDirection(uv);
            }
            public override GeoPoint PointAt(double Position)
            {
                return offsetSurface.PointAt(UvAt(Position));
            }
            public override double Length
            {
                get
                {   // integrate the speed, the TetraederHull estimate of GeneralCurve is too short
                    if (pend == pstart) return 0.0;
                    return ArcLength.FromSpeed(t => DirectionAt(t).Length, 0.0, 1.0, GetBasePoints());
                }
            }
            public override void Reverse()
            {
                double tmp = pstart;
                pstart = pend;
                pend = tmp;
                InvalidateSecondaryData();
            }
            public override ICurve[] Split(double Position)
            {
                if (Position <= 0.0 || Position >= 1.0) return new ICurve[] { Clone() as ICurve };
                double p = pstart + Position * (pend - pstart);
                return new ICurve[] { new FixedCurve(offsetSurface, param, pstart, p, fixedU), new FixedCurve(offsetSurface, param, p, pend, fixedU) };
            }
            public override void Trim(double StartPos, double EndPos)
            {
                double p1 = pstart + StartPos * (pend - pstart);
                double p2 = pstart + EndPos * (pend - pstart);
                pstart = p1;
                pend = p2;
                InvalidateSecondaryData();
            }
            /// <summary>
            /// Overrides <see cref="CADability.GeoObject.IGeoObjectImpl.Modify (ModOp)"/>
            /// </summary>
            /// <param name="m"></param>
            public override void Modify(ModOp m)
            {   // the surface is replaced, not changed in place: it may be shared with clones and pieces of this curve
                offsetSurface = offsetSurface.GetModified(m) as OffsetSurface;
                InvalidateSecondaryData();
            }
            /// <summary>
            /// Overrides <see cref="CADability.GeoObject.IGeoObjectImpl.Clone ()"/>
            /// </summary>
            /// <returns></returns>
            public override IGeoObject Clone()
            {
                return new FixedCurve(offsetSurface, param, pstart, pend, fixedU);
            }
            /// <summary>
            /// Overrides <see cref="CADability.GeoObject.IGeoObjectImpl.CopyGeometry (IGeoObject)"/>
            /// </summary>
            /// <param name="ToCopyFrom"></param>
            public override void CopyGeometry(IGeoObject ToCopyFrom)
            {
                FixedCurve other = ToCopyFrom as FixedCurve;
                offsetSurface = other.offsetSurface;
                fixedU = other.fixedU;
                param = other.param;
                pstart = other.pstart;
                pend = other.pend;
                InvalidateSecondaryData();
            }
            /// <summary>
            /// The free surface parameter of the point p, in the period closest to <paramref name="near"/>.
            /// </summary>
            private double FreeParameter(GeoPoint p, double near)
            {
                GeoPoint2D uv = offsetSurface.PositionOf(p);
                double res = fixedU ? uv.y : uv.x;
                bool periodic = fixedU ? offsetSurface.IsVPeriodic : offsetSurface.IsUPeriodic;
                if (periodic)
                {
                    double period = fixedU ? offsetSurface.VPeriod : offsetSurface.UPeriod;
                    if (period > 0.0) res += Math.Round((near - res) / period) * period;
                }
                return res;
            }
            public override GeoPoint EndPoint
            {
                get
                {
                    return base.EndPoint;
                }
                set
                {
                    pend = FreeParameter(value, pend);
                    InvalidateSecondaryData();
                }
            }
            public override GeoPoint StartPoint
            {
                get
                {
                    return base.StartPoint;
                }
                set
                {
                    pstart = FreeParameter(value, pstart);
                    InvalidateSecondaryData();
                }
            }

            #region ISerializable Members
            protected FixedCurve(SerializationInfo info, StreamingContext context)
            {   // older files also contain "FixedCurve", the iso curve of the base surface, which is no longer needed
                fixedU = info.GetBoolean("FixedU");
                offsetSurface = info.GetValue("OffsetSurface", typeof(OffsetSurface)) as OffsetSurface;
                param = info.GetDouble("Param");
                pstart = info.GetDouble("Pmin");
                pend = info.GetDouble("Pmax");
            }
            void ISerializable.GetObjectData(SerializationInfo info, StreamingContext context)
            {
                info.AddValue("FixedU", fixedU);
                info.AddValue("OffsetSurface", offsetSurface);
                info.AddValue("Param", param);
                info.AddValue("Pmin", pstart);
                info.AddValue("Pmax", pend);
            }
            #endregion
            #region IJsonSerialize
            protected FixedCurve() { } // for IJsonSerialize
            public void GetObjectData(IJsonWriteData data)
            {
                data.AddProperty("OffsetSurface", offsetSurface);
                data.AddProperty("FixedU", fixedU);
                data.AddProperty("Param", param);
                data.AddProperty("PStart", pstart);
                data.AddProperty("PEnd", pend);
            }
            public void SetObjectData(IJsonReadData data)
            {
                offsetSurface = data.GetProperty<OffsetSurface>("OffsetSurface");
                fixedU = data.GetProperty<bool>("FixedU");
                param = data.GetProperty<double>("Param");
                pstart = data.GetProperty<double>("PStart");
                pend = data.GetProperty<double>("PEnd");
            }
            #endregion
        }
        #region ISurfaceImpl overrides
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.GetNaturalBounds (out double, out double, out double, out double)"/>
        /// </summary>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        public override void GetNaturalBounds(out double umin, out double umax, out double vmin, out double vmax)
        {
            base.GetNaturalBounds(out umin, out umax, out vmin, out vmax);
            if (umin == double.MinValue || vmin == double.MinValue)
            {
                umin = this.umin; // diese Werte existieren immer (siehe Konstruktoren) Das Problem ist die BoxedSurface, die domain braucht
                umax = this.umax;
                vmin = this.vmin;
                vmax = this.vmax;
                // baseSurface.GetNaturalBounds(out umin, out umax, out vmin, out vmax);
            }
        }
        public override bool SameGeometry(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, double precision, out ModOp2D firstToSecond)
        {
            if (other is OffsetSurface)
            {
                OffsetSurface oos = (other as OffsetSurface);
                return baseSurface.SameGeometry(thisBounds, oos.baseSurface, otherBounds, precision, out firstToSecond) && oos.offset == offset;
            }
            return base.SameGeometry(thisBounds, other, otherBounds, precision, out firstToSecond);
        }

        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.PointAt (GeoPoint2D)"/>
        /// </summary>
        /// <param name="uv"></param>
        /// <returns></returns>
        public override GeoPoint PointAt(GeoPoint2D uv)
        {
            return baseSurface.PointAt(uv) + offset * baseSurface.GetNormal(uv).Normalized;
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.UDirection (GeoPoint2D)"/>
        /// </summary>
        /// <param name="uv"></param>
        /// <returns></returns>
        public override GeoVector UDirection(GeoPoint2D uv)
        {
            /* Formula from maxima, syntax:
             * load(vect);
             * f(x,y) := depends([f1,f2,f3],[x,y]);
             * g(x,y) := f(x,y)+c*(diff(f(x,y),x)~diff(f(x,y),y))/((diff(f(x,y),x)~diff(f(x,y),y)).(diff(f(x,y),x)~diff(f(x,y),y)))^0.5;
             * Note: "." means scalarproduct, "~" is crossprodukt, note that x~y~z is x~(y~z) not (x~y)~z
             * diff(g(x,y),x);
             * stringout("file",%);
             * With appropriate substitution you get the formula below.
             */

            GeoVector du, dv, duu, dvv, duv;
            GeoPoint loc;
            baseSurface.Derivative2At(uv, out loc, out du, out dv, out duu, out dvv, out duv);
            GeoVector crop;
            crop = (dv ^ (du ^ dv));
            return -0.5 * offset * (duu * crop + (du * (dv ^ ((duu ^ dv) + (du ^ duv)))) + (du * (duv ^ (du ^ dv)))) / System.Math.Pow((du * crop), 1.5) * (du ^ dv) + offset * ((duu ^ dv) + (du ^ duv)) / System.Math.Pow((du * crop), 0.5) + du;
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.VDirection (GeoPoint2D)"/>
        /// </summary>
        /// <param name="uv"></param>
        /// <returns></returns>
        public override GeoVector VDirection(GeoPoint2D uv)
        {
            /* Formula from maxima, syntax:
             * load(vect);
             * f(x,y) := depends([f1,f2,f3],[x,y]);
             * g(x,y) := f(x,y)+c*(diff(f(x,y),x)~diff(f(x,y),y))/((diff(f(x,y),x)~diff(f(x,y),y)).(diff(f(x,y),x)~diff(f(x,y),y)))^0.5;
             * Note: "." means scalarproduct, "~" is crossprodukt, note that x~y~z is x~(y~z) not (x~y)~z
             * diff(g(x,y),y);
             * With appropriate substitution you get the formula below.
             */
            GeoVector du, dv, duu, dvv, duv;
            GeoPoint loc;
            baseSurface.Derivative2At(uv, out loc, out du, out dv, out duu, out dvv, out duv);
            GeoVector crop = dv ^ (du ^ dv);
            return (-0.5 * offset * ((duv * crop) + (du * (dvv ^ (du ^ dv))) + (du * (dv ^ ((duv ^ dv) + (du ^ dvv))))) / System.Math.Pow(du * crop, 1.5) * (du ^ dv)) + (offset / System.Math.Pow(du * crop, 0.5) * ((duv ^ dv) + (du ^ dvv))) + dv;
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.GetNormal (GeoPoint2D)"/>
        /// </summary>
        /// <param name="uv"></param>
        /// <returns></returns>
        public override GeoVector GetNormal(GeoPoint2D uv)
        {
            return baseSurface.GetNormal(uv); // the normal should be the same, and we would have to handle singularities here
            // return this.UDirection(uv) ^ this.VDirection(uv);
        }
        public override double[] GetUSingularities()
        {
            return baseSurface.GetUSingularities();
        }
        public override double[] GetVSingularities()
        {
            return baseSurface.GetVSingularities();
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.GetZMinMax (Projection, double, double, double, double, ref double, ref double)"/>
        /// </summary>
        /// <param name="p"></param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <param name="zMin"></param>
        /// <param name="zMax"></param>
        public override void GetZMinMax(Projection p, double umin, double umax, double vmin, double vmax, ref double zMin, ref double zMax)
        {
            throw new NotImplementedException("GetZMinMax not implemented");
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.Make3dCurve (ICurve2D)"/>
        /// </summary>
        /// <param name="curve2d"></param>
        /// <returns></returns>
        public override ICurve Make3dCurve(ICurve2D curve2d)
        {
            if (curve2d is Curve2DAspect)
            {
                ICurve res = (curve2d as Curve2DAspect).Get3DCurve(this);
                if (res != null) return res;
            }
            if (Curve3dOfProjected(curve2d) is ICurve onThisSurface) return onThisSurface;
            return base.Make3dCurve(curve2d);
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.GetPlaneIntersection (PlaneSurface, double, double, double, double, double)"/>
        /// </summary>
        /// <param name="pl"></param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <param name="precision"></param>
        /// <returns></returns>
        public override IDualSurfaceCurve[] GetPlaneIntersection(PlaneSurface pl, double umin, double umax, double vmin, double vmax, double precision)
        {
            return base.GetPlaneIntersection(pl, umin, umax, vmin, vmax, precision);
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.Clone ()"/>
        /// </summary>
        /// <returns></returns>
        public override ISurface Clone()
        {
            OffsetSurface res = new OffsetSurface(baseSurface.Clone(), offset);
            res.umin = umin;
            res.umax = umax;
            res.vmin = vmin;
            res.vmax = vmax;
            res.domain = domain;
            return res;
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.Modify (ModOp)"/>
        /// </summary>
        /// <param name="m"></param>
        public override void Modify(ModOp m)
        {   // genaugenommen geht eine Modifikation hier nicht, wird aber z.Z. nur zur Translation verwendet
            parallelepipedHull = null;
            if (m.IsOrthogonal)
            {
                baseSurface = baseSurface.GetModified(m);
                offset = m * offset;
            }
            else throw new NotImplementedException("Modify OffsetSurface with non ortogonal Matrix");
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.GetModified (ModOp)"/>
        /// </summary>
        /// <param name="m"></param>
        /// <returns></returns>
        public override ISurface GetModified(ModOp m)
        {
            if (m.IsIsogonal)
            {
                OffsetSurface res = new OffsetSurface(baseSurface.GetModified(m), offset * m.Factor, domain);
                return res;
            }
            else throw new NotImplementedException("Modify OffsetSurface with non ortogonal Matrix");
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.CopyData (ISurface)"/>
        /// </summary>
        /// <param name="CopyFrom"></param>
        public override void CopyData(ISurface CopyFrom)
        {
            OffsetSurface cc = CopyFrom as OffsetSurface;
            if (cc != null)
            {
                this.baseSurface = cc.baseSurface.Clone();
                this.offset = cc.offset;
            }
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.GetLineIntersection (GeoPoint, GeoVector)"/>
        /// </summary>
        /// <param name="startPoint"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        public override GeoPoint2D[] GetLineIntersection(GeoPoint startPoint, GeoVector direction)
        {
            return base.GetLineIntersection(startPoint, direction);
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.GetOffsetSurface (double)"/>
        /// </summary>
        /// <param name="offset"></param>
        /// <returns></returns>
        public override ISurface GetOffsetSurface(double offset)
        {
            return new OffsetSurface(baseSurface.Clone(), this.offset + offset);
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.FixedU (double, double, double)"/>
        /// </summary>
        /// <param name="u"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <returns></returns>
        public override ICurve FixedU(double u, double vmin, double vmax)
        {
            return new FixedCurve(Clone() as OffsetSurface, u, vmin, vmax, true); // vmin > vmax gives the reversed curve
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.ISurfaceImpl.FixedV (double, double, double)"/>
        /// </summary>
        /// <param name="u"></param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <returns></returns>
        public override ICurve FixedV(double u, double umin, double umax)
        {
            return new FixedCurve(Clone() as OffsetSurface, u, umin, umax, false); // umin > umax gives the reversed curve
        }
        public override bool IsUPeriodic
        {
            get
            {
                return baseSurface.IsUPeriodic;
            }
        }
        public override bool IsVPeriodic
        {
            get
            {
                return baseSurface.IsVPeriodic;
            }
        }
        public override double UPeriod => baseSurface.UPeriod;
        public override double VPeriod => baseSurface.VPeriod;
        public override RuledSurfaceMode IsRuled
        {
            get
            {
                return baseSurface.IsRuled;
            }
        }
        public override ModOp2D ReverseOrientation()
        {
            parallelepipedHull = null;
            offset = -offset;
            return baseSurface.ReverseOrientation();
        }
        #endregion
        public override IPropertyEntry GetPropertyEntry(IFrame frame)
        {
            // to implement:
            return new GroupProperty("OffsetSurface", new IPropertyEntry[0]);
        }
        #region ISerializable Members
        protected OffsetSurface(SerializationInfo info, StreamingContext context)
        {
            baseSurface = info.GetValue("BaseSurface", typeof(ISurface)) as ISurface;
            offset = info.GetDouble("Offset");
            umin = info.GetDouble("Umin");
            umax = info.GetDouble("Umax");
            vmin = info.GetDouble("Vmin");
            vmax = info.GetDouble("Vmax");
        }
        void ISerializable.GetObjectData(SerializationInfo info, StreamingContext context)
        {
            info.AddValue("BaseSurface", baseSurface);
            info.AddValue("Offset", offset);
            info.AddValue("Umin", umin);
            info.AddValue("Umax", umax);
            info.AddValue("Vmin", vmin);
            info.AddValue("Vmax", vmax);
        }

        protected OffsetSurface() { } // for IJsonSerialize
        public void GetObjectData(IJsonWriteData data)
        {
            data.AddProperty("BaseSurface", baseSurface);
            data.AddProperty("Offset", offset);
            data.AddProperty("Umin", umin);
            data.AddProperty("Umax", umax);
            data.AddProperty("Vmin", vmin);
            data.AddProperty("Vmax", vmax);
        }
        public void SetObjectData(IJsonReadData data)
        {
            baseSurface = data.GetProperty<ISurface>("BaseSurface");
            offset = data.GetProperty<double>("Offset");
            umin = data.GetProperty<double>("Umin");
            umax = data.GetProperty<double>("Umax");
            vmin = data.GetProperty<double>("Vmin");
            vmax = data.GetProperty<double>("Vmax");
        }

        int IExportStep.Export(ExportStep export, bool topLevel)
        {
            int n = (baseSurface as IExportStep).Export(export, false);
            return export.WriteDefinition("OFFSET_SURFACE('',#" + n.ToString() + "," + export.ToString(offset) + ",.F.)");
        }
        #endregion
    }
}
