using CADability.Attribute;
using CADability.Curve2D;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Text;

namespace CADability.GeoObject
{
    /// <summary>
    /// A helical swept surface, defined by a curve to be swept, the pitch and the axis
    /// Parameter u : along the curve
    /// Parameter v : along the helix
    /// </summary>
    public class HelicalSweepSurface : ISurfaceImpl, IJsonSerialize
    {
        private ICurve curve;
        private double pitch;
        private Axis axis;

        public HelicalSweepSurface(ICurve curve, double pitch, Axis axis)
        {
            this.curve = curve;
            this.pitch = pitch;
            this.axis = axis.Normalized;
        }
        protected HelicalSweepSurface() { } // for IJsonSerialize
        private double K => pitch / (2.0 * Math.PI);  // per radiant
        private static GeoVector RotateVectorAroundUnitAxis(GeoVector x, GeoVector aUnit, double cos, double sin)
        {
            // Rodrigues: x' = x*cos + (a×x)*sin + a*(a·x)*(1-cos)
            GeoVector ax = aUnit ^ x;            // Cross
            double adx = aUnit * x;              // Dot (oder aUnit.Dot(x))
            return cos * x + sin * ax + (adx * (1.0 - cos)) * aUnit;
        }

        public override ICurve FixedU(double u, double vmin, double vmax)
        {
            Line2D l2d = new Line2D(new GeoPoint2D(u, vmin), new GeoPoint2D(u, vmax));
            return CurveOnSurface.Construct(l2d, this);
        }

        public override ICurve FixedV(double v, double umin, double umax)
        {
            GeoVector offset = (K * v) * axis.Direction;

            ModOp rot = ModOp.Rotate(axis.Location, axis.Direction, new SweepAngle(v));
            ModOp tra = ModOp.Translate(offset);

            ICurve fv = curve.CloneModified(tra * rot);
            fv.Trim(umin, umax);
            return fv;
        }

        public override ISurface GetModified(ModOp m)
        {
            return new HelicalSweepSurface(curve.CloneModified(m), pitch, (m * axis).Normalized);
        }
        public override ModOp2D ReverseOrientation()
        {
            curve.Reverse();
            return new ModOp2D(-1, 0, 0, 0, 1, 0);
        }
        public override ISurface Clone()
        {
            HelicalSweepSurface res = new HelicalSweepSurface(curve.Clone(), pitch, axis);
            res.SetBounds(this.GetBounds());
            return res;
        }

        public override IPropertyEntry GetPropertyEntry(IFrame frame)
        {
            return new GroupProperty("HelicalSweptSurface", new IPropertyEntry[0]);
        }

        public override GeoPoint PointAt(GeoPoint2D uv)
        {
            double u = uv.x;
            double v = uv.y;

            double cos = Math.Cos(v);
            double sin = Math.Sin(v);

            // Kurve
            GeoPoint cu = curve.PointAt(u);

            // r = C(u) - A
            GeoVector r = cu - axis.Location;

            // rRot = Rv(r)
            GeoVector rRot = RotateVectorAroundUnitAxis(r, axis.Direction, cos, sin);

            // location = A + rRot + k*v*a
            return axis.Location + rRot + (K * v) * axis.Direction;
        }

        public override GeoVector UDirection(GeoPoint2D uv)
        {
            double u = uv.x;
            double v = uv.y;

            double cos = Math.Cos(v);
            double sin = Math.Sin(v);

            // Kurve
            GeoPoint cu = curve.PointAt(u);
            GeoVector c1 = curve.DirectionAt(u);

            // du = Rv(C'(u))
            return RotateVectorAroundUnitAxis(c1, axis.Direction, cos, sin);
        }

        public override GeoVector VDirection(GeoPoint2D uv)
        {
            double u = uv.x;
            double v = uv.y;

            double cos = Math.Cos(v);
            double sin = Math.Sin(v);

            // Kurve
            GeoPoint cu = curve.PointAt(u);

            // r = C(u) - A
            GeoVector r = cu - axis.Location;

            // rRot = Rv(r)
            GeoVector rRot = RotateVectorAroundUnitAxis(r, axis.Direction, cos, sin);

            // dv = a × rRot + k*a
            return (axis.Direction ^ rRot) + K * axis.Direction;

        }

        public override void DerivationAt(GeoPoint2D uv, out GeoPoint location, out GeoVector du, out GeoVector dv)
        {
            double u = uv.x;
            double v = uv.y;

            double cos = Math.Cos(v);
            double sin = Math.Sin(v);

            // Kurve
            GeoPoint cu = curve.PointAt(u);
            GeoVector c1 = curve.DirectionAt(u);

            // r = C(u) - A
            GeoVector r = cu - axis.Location; // falls GeoPoint-GeoPoint => GeoVector

            // rRot = Rv(r)
            GeoVector rRot = RotateVectorAroundUnitAxis(r, axis.Direction, cos, sin);

            // location = A + rRot + k*v*a
            location = axis.Location + rRot + (K * v) * axis.Direction;

            // du = Rv(C'(u))
            du = RotateVectorAroundUnitAxis(c1, axis.Direction, cos, sin);

            // dv = a × rRot + k*a
            dv = (axis.Direction ^ rRot) + K * axis.Direction;
        }

        public override void Derivation2At(GeoPoint2D uv,
            out GeoPoint location,
            out GeoVector du, out GeoVector dv,
            out GeoVector duu, out GeoVector dvv, out GeoVector duv)
        {
            double u = uv.x;
            double v = uv.y;

            double cos = Math.Cos(v);
            double sin = Math.Sin(v);

            // Kurve + Ableitungen
            GeoPoint cu;
            GeoVector c1, c2;
            if (!curve.TryPointDeriv2At(u, out cu, out c1, out c2))
            {
                // Fallback: wenn 2. Ableitung nicht verfügbar ist
                cu = curve.PointAt(u);
                c1 = curve.DirectionAt(u);
                c2 = GeoVector.NullVector; // oder 0
            }

            GeoVector r = cu - axis.Location;
            GeoVector rRot = RotateVectorAroundUnitAxis(r, axis.Direction, cos, sin);

            location = axis.Location + rRot + (K * v) * axis.Direction;

            du = RotateVectorAroundUnitAxis(c1, axis.Direction, cos, sin);
            dv = (axis.Direction ^ rRot) + K * axis.Direction;

            duu = RotateVectorAroundUnitAxis(c2, axis.Direction, cos, sin);

            // dvv = a × (a × rRot)
            dvv = axis.Direction ^ (axis.Direction ^ rRot);

            // duv = a × du
            duv = axis.Direction ^ du;
        }
        public override bool IsUPeriodic => false;
        public override bool IsVPeriodic => false;

        public override bool SameGeometry(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, double precision, out ModOp2D firstToSecond)
        {
            if (other is HelicalSweepSurface hs)
            {
                if (Precision.SameAxis(axis, hs.axis) && Math.Abs(pitch - hs.pitch) < precision)
                {
                    if (curve.SameGeometry(hs.curve, precision))
                    {
                        firstToSecond = ModOp2D.Identity;
                        return true;
                    }
                    // there is still the possibility that the surfaces are identical, and the curves are helically shifted
                }
            }
            firstToSecond = ModOp2D.Null;
            return false;
        }
        public override GeoPoint2D PositionOf(GeoPoint p)
        {   // when p is close to the surface, we can intersect the curve with the cylinder around the axis and through p
            // then we know the u-parameter
            GeoPoint pa = Geometry.DropPL(p, axis.Location, axis.Direction);
            GeoVector dirx = (p - pa);
            GeoVector diry = dirx ^ axis.Direction;
            diry.Length = dirx.Length;

            CylindricalSurface cs = new CylindricalSurface(axis.Location, dirx, diry, axis.Direction);
            cs.Intersect(curve, usedArea, out GeoPoint[] ips, out GeoPoint2D[] uv, out double[] u);
            if (ips.Length > 0)
            {
                // now find the point with the right v-parameter
                double bestDist = double.MaxValue;
                int bestIndex = -1;
                GeoPoint2D bestuv = GeoPoint2D.Origin;
                Plane pln = new Plane(axis.Location, axis.Direction);
                for (int i = 0; i < ips.Length; i++)
                {
                    double dv = pln.Distance(p) - pln.Distance(ips[i]);
                    double v = (dv / pitch) * 2.0 * Math.PI;
                    double d = PointAt(new GeoPoint2D(u[i], v)) | p;
                    if (d< bestDist)
                    {
                        bestDist = d;
                        bestIndex = i;
                        bestuv = new GeoPoint2D(u[i], v);
                    }
                }
                if (bestIndex >= 0)
                {
                    if (BoxedSurfaceExtension.PositionOfLM(this, p, ref bestuv, out double minDist))
                    {
                        return bestuv;
                    }
                }
            }

            return base.PositionOf(p);
        }
        public void GetObjectData(IJsonWriteData data)
        {
            data.AddProperty("Curve", curve);
            data.AddProperty("Pitch", pitch);
            data.AddProperty("Axis", axis);
        }
        public void SetObjectData(IJsonReadData data)
        {
            curve = data.GetProperty<ICurve>("Curve");
            pitch = data.GetDoubleProperty("Pitch");
            axis = data.GetProperty<Axis>("Axis");
        }

    }
}
