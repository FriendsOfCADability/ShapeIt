using CADability;
using CADability.GeoObject;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics;
using MathNet.Numerics.Differentiation;
using static System.Math;
using CADability.Attribute;
using MathNet.Numerics.RootFinding;
using CADability.Curve2D;
using CADability.Shapes;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Financial;

namespace CADability.GeoObject
{
    public static class CurveExtensions
    {
        /// <summary>
        /// Computes the curvature circle at a given parameter on a 3D curve.
        /// </summary>
        /// <param name="curve">The curve to evaluate.</param>
        /// <param name="u">The curve parameter.</param>
        /// <returns>
        /// A tuple containing:
        /// - center: the center point of the osculating circle,
        /// - normal: the normal vector of the osculating plane,
        /// - radius: the curvature radius (1 / curvature).
        /// </returns>
        public static (GeoPoint center, GeoVector normal, double radius) CurvatureAt(this ICurve curve, double u)
        {
            IReadOnlyList<GeoVector> ders = curve.PointAndDerivativesAt(u, 2);

            double deriv1Length = ders[1].Length;
            if (ders[1].Length < 1e-12)
            {
                throw new ArgumentException("First derivative is too small to determine curvature.");
            }

            // Tangent vector
            GeoVector T = ders[1] / deriv1Length;

            // Normal component of second derivative
            GeoVector proj = (ders[2] * T) * T;
            GeoVector normalComponent = ders[2] - proj;
            GeoPoint point = GeoPoint.Origin + ders[0];
            double normalLength = normalComponent.Length;
            if (normalLength < 1e-12)
            {
                // Curve is locally straight (e.g. line)
                return (point, GeoVector.NullVector, double.PositiveInfinity);
            }

            // Unit normal vector (direction to curvature center)
            GeoVector N = normalComponent / normalLength;

            // Curvature and radius
            double curvature = normalLength / (deriv1Length * deriv1Length);
            double radius = 1.0 / curvature;

            // Center of curvature
            GeoPoint center = point + radius * N;

            return (center, (ders[1] ^ ders[2]).Normalized, radius);
        }

        public static double RadiusAt(this ICurve curve, double u)
        {
            if (!curve.TryPointDeriv2At(u, out GeoPoint p, out GeoVector d1, out GeoVector d2))
            {
                throw new ArgumentException("Curve does not support second derivative at the given parameter.");
            }
            double d1l = d1.Length;
            return d1l * d1l * d1l / (d1 ^ d2).Length;
        }
        /// <summary>
        /// Returns the positions, where the curve has maximal and minimal curature
        /// </summary>
        /// <param name="curve"></param>
        /// <returns>Array of maximal curvature, array of minimal curvature</returns>
        public static (double[] max, double[] min) CurvatureExtrema(this ICurve curve)
        {
            List<double> minima = new List<double>();
            List<double> maxima = new List<double>();
            if (curve is Line || (curve is Ellipse e && e.IsCircle) || curve is Polyline) { } // no minima and maxima
            if (curve is Ellipse ellipse)
            {
                double pos = curve.ParameterToPosition(0.0);
                if (pos >= 0.0 && pos <= 1.0) maxima.Add(pos);
                pos = curve.ParameterToPosition(Math.PI);
                if (pos >= 0.0 && pos <= 1.0) maxima.Add(pos);
                pos = curve.ParameterToPosition(Math.PI / 2.0);
                if (pos >= 0.0 && pos <= 1.0) minima.Add(pos);
                pos = curve.ParameterToPosition(3.0 * Math.PI / 2.0);
                if (pos >= 0.0 && pos <= 1.0) minima.Add(pos);
            }
            else
            {
                // TODO
                double[] pp = curve.GetSavePositions();
            }
            return (minima.ToArray(), maxima.ToArray());
        }
    }


    [Serializable()]
    public class SweptCircle : ISurfaceImpl, ISerializable, IJsonSerialize, ISurfaceOfExtrusion
    {
        private ICurve spine; // spine curve for the pipe
        private double radius; // radius of the pipe, when negative, the normal of the surface points towwards the spine curve
        private GeoVector normal; // when spine curve is planar, this is the normal vector to the plane. When n is the nullvector we use the Frenet frame
        private double[] criticalPositions; // the u parameters, where the spines curvature changes from greater than radius to smaller than radius

        /// <summary>
        /// create a surface which is defined by a curve along which a circle is beeing moved.
        /// The result may be a cylindrical surface, a toroidal surface or a SweptCircle
        /// </summary>
        /// <param name="along"></param>
        /// <param name="radius"></param>
        /// <param name="seam">points to the 0°, 360° seam</param>
        /// <returns></returns>
        public static ISurface MakePipeSurface(ICurve along, double radius, GeoVector seam)
        {   // erzeugt ein Rohr mit gegebenem Radius entland der Kurve als Mittelachse und der Nahtstelle in Richtung seam
            if (along is Line)
            {
                GeoVector dirz = along.StartDirection.Normalized;
                GeoVector diry = radius * (seam ^ dirz).Normalized;
                GeoVector dirx = radius * (dirz ^ diry).Normalized;
                CylindricalSurface cs = new CylindricalSurface(along.StartPoint, dirx, diry, dirz);
                return cs;
            }
            if (along is Ellipse && (along as Ellipse).IsCircle)
            {
                Ellipse e = (along as Ellipse);
                ToroidalSurface ts = new ToroidalSurface(e.Center, e.Plane.DirectionX, e.Plane.DirectionY, e.Plane.Normal, e.Radius, radius);
                return ts;
            }
            else
            {
                return new SweptCircle(along, radius);
            }
        }

        public SweptCircle(ICurve spine, double radius, BoundingRect? usedArea = null): base(usedArea)
        {
            this.spine = spine;
            this.radius = radius;
            switch (spine.GetPlanarState())
            {
                case PlanarState.Planar:
                    // a planar curve
                    normal = spine.GetPlane().Normal;
                    break;
                case PlanarState.UnderDetermined:
                    // a line, we can choose an arbitrary normal vector
                    spine.StartDirection.ArbitraryNormals(out normal, out GeoVector _);
                    break;
                default:
                    int n = 25;
                    double[] values = Enumerable.Range(0, n + 1).Select(i => i / (double)n).ToArray();
                    normal = FindSweepNormal(spine, values); // if normal==null, we use the Frenet frame
                    break;
            }
        }

        public ICurve Spine
        {
            get { return spine; }
        }
        public double Radius
        {
            get { return radius; }
        }

        /// <summary>
        /// Computes a constant sweep normal for a (possibly non-planar) curve
        /// by PCA on its tangent directions.
        /// Returns GeoVector.NullVector if no clear normal exists.
        /// </summary>
        public static GeoVector FindSweepNormal(ICurve curve, double[] parameters)
        {
            // Need at least 3 samples to define a direction
            if (parameters == null || parameters.Length < 3)
                return GeoVector.NullVector;

            // 1) Build covariance matrix C = sum(t_i * t_i^T)
            var C = Matrix<double>.Build.Dense(3, 3, 0.0);
            foreach (double u in parameters)
            {
                // get tangent and normalize
                GeoVector tg = curve.DirectionAt(u);
                Vector<double> t = Vector<double>.Build.DenseOfArray(new[] { tg.x, tg.y, tg.z });
                double norm = t.L2Norm();
                if (norm < 1e-8)
                    continue;
                t = t.Divide(norm);

                // outer product
                C += t.ToColumnMatrix() * t.ToRowMatrix();
            }

            // 2) Eigen-Decomposition
            var evd = C.Evd();
            var eValues = evd.EigenValues;
            var eVectors = evd.EigenVectors;

            // 3) Find smallest and largest real eigenvalue
            int minIdx = 0, maxIdx = 0;
            double minVal = eValues[0].Real, maxVal = eValues[0].Real;
            for (int i = 1; i < 3; i++)
            {
                double val = eValues[i].Real;
                if (val < minVal) { minVal = val; minIdx = i; }
                if (val > maxVal) { maxVal = val; maxIdx = i; }
            }

            // 4) Critical condition: no clear “weakest” direction
            if (maxVal <= 0)
                return GeoVector.NullVector;

            // If the smallest variance direction is not much smaller than the
            // largest, the tangents are too isotropic -> fallback to Frenet
            const double ratioThreshold = 0.8;
            if (minVal / maxVal > ratioThreshold)
                return GeoVector.NullVector;

            // 5) Extract the eigenvector for the smallest eigenvalue
            Vector<double> nVec = eVectors.Column(minIdx);
            var normal = new GeoVector(nVec[0], nVec[1], nVec[2]);

            // Normalize final result
            double length = Math.Sqrt(normal * normal);
            if (length < 1e-8)
                return GeoVector.NullVector;
            return normal.Normalized;
        }

        private double[] CriticalPositions
        {
            get
            {
                if (criticalPositions == null)
                {
                    Func<double, double> f = u => spine.CurvatureAt(u).radius - radius; // radius may be negative here. Is this still correct then?
                    List<double> roots = AdaptiveRootFinder.FindRootsAdaptive(f, 0, 1);
                    criticalPositions = roots.ToArray();
                }
                return criticalPositions;
            }
        }

        [Serializable()]
        public class FixedVCurve : GeneralCurve, ISerializable, IJsonSerialize
        {
            SweptCircle sweptCircle;
            double v0; // the sweptCircle v position, which is fixed for this curve
            double umin;
            double umax;
            public FixedVCurve(SweptCircle sweptCircle, double v0, double umin, double umax)
            {
                this.sweptCircle = sweptCircle;
                this.v0 = v0;
                this.umin = umin;
                this.umax = umax;
            }
            private double posToParam(double u)
            {   // v is in the range [0,1], where 0 is vmin and 1 is vmax
                return umin + u * (umax - umin);
            }
            private double paramToPos(double p)
            {   // inverse to posToParam
                return (p - umin) / (umax - umin);
            }

            public override IGeoObject Clone()
            {
                return new FixedVCurve(sweptCircle, v0, umin, umax);
            }

            public override void CopyGeometry(IGeoObject ToCopyFrom)
            {
                if (ToCopyFrom is FixedVCurve fixedV)
                {
                    sweptCircle = fixedV.sweptCircle;
                    v0 = fixedV.v0;
                    umin = fixedV.umin;
                    umax = fixedV.umax;
                }
            }

            public override GeoVector DirectionAt(double Position)
            {
                return (umax - umin) * sweptCircle.UDirection(new GeoPoint2D(posToParam(Position), v0));
            }

            public override void Modify(ModOp m)
            {
                throw new NotSupportedException("SweptCircle.FixedVCurve is immutable");
            }

            public override GeoPoint PointAt(double Position)
            {
                return sweptCircle.PointAt(new GeoPoint2D(posToParam(Position), v0));
            }
            public override double PositionOf(GeoPoint p)
            {
                return base.PositionOf(p); // ?? can we do better?
            }

            public override void Reverse()
            {
                throw new NotSupportedException("SweptCircle.FixedVCurve is immutable");
            }

            public override ICurve[] Split(double Position)
            {
                throw new NotSupportedException("SweptCircle.FixedVCurve is immutable");
            }

            public override void Trim(double StartPos, double EndPos)
            {
                double umi = posToParam(StartPos);
                double uma = posToParam(EndPos);
                umin = umi;
                umax = uma;
            }
            public override bool TryPointDeriv2At(double position, out GeoPoint point, out GeoVector deriv, out GeoVector deriv2)
            {
                sweptCircle.Derivative2At(new GeoPoint2D(posToParam(position), v0), out GeoPoint location, out GeoVector du, out GeoVector dv, out GeoVector duu, out GeoVector dvv, out GeoVector duv);
                point = location;
                deriv = (umax - umin) * du;
                deriv2 = (umax - umin) * (umax - umin) * duu;
                return true;
            }

            protected override double[] GetBasePoints()
            {
                List<double> parPositions = new List<double>(sweptCircle.spine.GetSavePositions()); // maybe we must do better here, because the curve on the inside of the curvature may become wild!
                parPositions.AddRange(sweptCircle.CriticalPositions);
                for (int i = 0; i < parPositions.Count; i++)
                {
                    parPositions[i] = paramToPos(parPositions[i]);
                }
                parPositions.RemoveAll(x => x < 0);
                parPositions.RemoveAll(x => x > 1);
                parPositions.Add(0.0);
                parPositions.Add(1.0);
                parPositions.Sort();
                parPositions.RemoveDuplicatesWithTolerance(1e-6);
                return parPositions.ToArray();
            }
            #region ISerializable
            protected FixedVCurve(SerializationInfo info, StreamingContext context)
                   : base(info, context)
            {
                sweptCircle = (SweptCircle)info.GetValue("SweptCircle", typeof(SweptCircle));
                v0 = (double)info.GetValue("V0", typeof(double));
                umin = (double)info.GetValue("Umin", typeof(double));
                umax = (double)info.GetValue("Umax", typeof(double));
            }
            public override void GetObjectData(SerializationInfo info, StreamingContext context)
            {
                base.GetObjectData(info, context);
                info.AddValue("SweptCircle", sweptCircle, typeof(SweptCircle));
                info.AddValue("V0", v0, typeof(double));
                info.AddValue("Umin", umin, typeof(double));
                info.AddValue("Umax", umax, typeof(double));
            }
            #endregion
            #region IJsonSerialize
            protected FixedVCurve() { } // we need this for JsonSerialisation
            public void GetObjectData(IJsonWriteData data)
            {
                data.AddProperty("SweptCircle", sweptCircle);
                data.AddProperty("V0", v0);
                data.AddProperty("Umin", umin);
                data.AddProperty("Umax", umax);
            }

            public void SetObjectData(IJsonReadData data)
            {
                sweptCircle = data.GetProperty<SweptCircle>("SweptCircle");
                v0 = data.GetProperty<double>("V0");
                umin = data.GetProperty<double>("Umin");
                umax = data.GetProperty<double>("Umax");
            }

            #endregion
        }

        public override ICurve FixedU(double u, double vmin, double vmax)
        {
            Plane circlePlane = new Plane(spine.PointAt(u), spine.DirectionAt(u));
            GeoPoint spinePoint = spine.PointAt(u);
            GeoVector tangent = spine.DirectionAt(u).Normalized;
            GeoVector yAxis = (normal ^ tangent).Normalized;
            GeoVector xAxis = Sign(radius) * tangent ^ yAxis;
            Plane plane = new Plane(spinePoint, xAxis, yAxis);
            Ellipse circularArc = Ellipse.Construct();
            circularArc.SetArcPlaneCenterStartEndPoint(plane, GeoPoint2D.Origin, plane.Project(PointAt(new GeoPoint2D(u, vmin))), plane.Project(PointAt(new GeoPoint2D(u, vmax))), plane, vmin < vmax);
            if (Math.Abs(circularArc.SweepParameter) < 1e-12)
            {   // a full circle
                if (vmin < vmax) circularArc.SweepParameter = 2 * PI;
                else circularArc.SweepParameter = -2 * PI;
            }
            double pos = circularArc.PositionOf(PointAt(new GeoPoint2D(u, (vmin + vmax) / 2.0)));
            if (Math.Abs(0.5 - pos) > 0.5)
            {   // this is the 
                circularArc.Complement();
            }
            return circularArc; // no need for FixedUCurve!
        }

        public override ICurve FixedV(double v, double umin, double umax)
        {
            return new FixedVCurve(this, v, umin, umax);
        }

        public override ISurface GetModified(ModOp m)
        {
            if (m.IsIsogonal) return new SweptCircle(spine.CloneModified(m), m.Factor * radius);
            else throw new NotImplementedException();
        }

        public override IPropertyEntry GetPropertyEntry(IFrame frame)
        {
            List<IPropertyEntry> se = new List<IPropertyEntry>();
            IPropertyEntry spineProperty = (spine as IGeoObject).GetShowProperties(frame);
            spineProperty.ReadOnly = true;
            se.Add(spineProperty);
            LengthProperty radiusProperty = new LengthProperty(frame, "SweptCircle.Radius");
            radiusProperty.ReadOnly = true;
            radiusProperty.OnGetValue = () => radius;
            se.Add(radiusProperty);
            return new GroupProperty("SweptCircleSurface", se.ToArray());
        }

        public override bool IsUPeriodic => spine.IsClosed;
        public override double UPeriod => spine.IsClosed ? 1.0 : 0.0;
        public override bool IsVPeriodic => true;
        public override double VPeriod => 2 * PI;

        IOrientation ISurfaceOfExtrusion.Orientation => null; // do we need this?

        public ICurve ExtrudedCurve => FixedU(0.0, 0.0, 2 * Math.PI);

        public bool ExtrusionDirectionIsV => false;

        public override GeoPoint PointAt(GeoPoint2D uv)
        {
            double u = uv.x;
            double v = uv.y;
            if (normal != GeoVector.NullVector)
            {
                GeoPoint spinePoint = spine.PointAt(u);
                GeoVector tangent = spine.DirectionAt(u).Normalized;
                GeoVector yAxis = (normal ^ tangent).Normalized;
                GeoVector xAxis = Sign(radius) * tangent ^ yAxis;
                double sinV = Sin(v);
                double cosV = Cos(v);
                return spinePoint + radius * (cosV * xAxis + sinV * yAxis);
            }
            else
            {
                var deriv = spine.PointAndDerivativesAt(u, 2).ToArray();
                GeoPoint spinePoint = GeoPoint.Origin + deriv[0];
                GeoVector vel = deriv[1];
                GeoVector acc = deriv[2];
                GeoVector T = vel.Normalized;
                // Frenet-Frame 
                GeoVector N = (acc - (acc * T) * T).Normalized;   // Hauptnormalen­vektor
                GeoVector B = Sign(radius) * T ^ N;                              // Binormale
                double sinV = Sin(v);
                double cosV = Cos(v);
                return spinePoint + radius * (cosV * N + sinV * B);
            }
        }
        public override GeoPoint2D PositionOf(GeoPoint p)
        {
            double u;
            if (spine is Ellipse)
            {   // Ellipse.PositionOf is wrong implemented for points outside the ellipse
                TetraederHull tetraederHull = new TetraederHull(spine);
                u = tetraederHull.PositionOf(p);
            }
            else u = spine.PositionOf(p);
            if (normal != GeoVector.NullVector)
            {
#if DEBUG
                // GeoObjectList dbgl = this.DebugGrid;
#endif
                GeoPoint spinePoint = spine.PointAt(u);
                GeoVector tangent = spine.DirectionAt(u).Normalized;
                GeoVector yAxis = (normal ^ tangent).Normalized;
                GeoVector xAxis = Sign(radius) * tangent ^ yAxis;
                double v = Atan2((p - spinePoint) * yAxis, (p - spinePoint) * xAxis);
                if (radius < 0) v = PI + v;
                GeoPoint2D uv = new GeoPoint2D(u, v);
#if DEBUG
                // DebuggerContainer dc = this.ParallelepipedHull.Debug;
#endif
                // commented out, because it too often throws exceptions
                //if (BoxedSurfaceExtension.PositionOfMN(this, p, ref uv, out double dist)) return uv;
                //uv = new GeoPoint2D(u, v);
                if (!usedArea.IsEmpty()) SurfaceHelper.AdjustPeriodic(this, usedArea, ref uv); // must be adjusted to usedArea
                if (BoxedSurfaceExtension.PositionOfLM(this, p, ref uv, out double dist)) return uv;
                return new GeoPoint2D(u, v);
            }
            else
            {
                var deriv = spine.PointAndDerivativesAt(u, 2).ToArray();
                GeoPoint spinePoint = GeoPoint.Origin + deriv[0];
                GeoVector vel = deriv[1];
                GeoVector acc = deriv[2];
                GeoVector T = vel.Normalized;
                // Frenet-Frame 
                GeoVector N = (acc - (acc * T) * T).Normalized;   // Hauptnormalen­vektor
                GeoVector B = Sign(radius) * T ^ N;                              // Binormale
                double v = Atan2((p - spinePoint) * B, (p - spinePoint) * N);
                GeoPoint2D res = new GeoPoint2D(u, v);
                if (!usedArea.IsEmpty()) SurfaceHelper.AdjustPeriodic(this, usedArea, ref res); // must be adjusted to usedArea
                return res;
            }
        }

        public override GeoVector UDirection(GeoPoint2D uv)
        {
            double u = uv.x;
            double v = uv.y;

            if (normal != GeoVector.NullVector)
            {
                // curve derivatives
                var deriv = spine.PointAndDerivativesAt(u, 2);
                GeoVector vel = deriv[1]; // C'(u)
                GeoVector acc = deriv[2]; // C''(u)

                double speed = vel.Length;
                if (speed == 0.0) return GeoVector.NullVector;

                GeoVector T = vel / speed;

                // T' = acc/|vel| - vel*(vel·acc)/|vel|^3
                double velDotAcc = vel * acc; 
                GeoVector Tp = (acc / speed) - (velDotAcc / (speed * speed * speed)) * vel;

                // w = normal x T
                GeoVector w = normal ^ T;
                double wlen = w.Length;
                if (wlen == 0.0)
                {
                    return vel; // best-effort fallback
                }

                GeoVector y = w / wlen;

                // w' = normal x T'
                GeoVector wp = normal ^ Tp;

                // y' = wp/|w| - w*(w·wp)/|w|^3
                double wDotWp = w * wp;
                GeoVector yp = (wp / wlen) - (wDotWp / (wlen * wlen * wlen))*w;

                // x = sign(radius) * (T x y)
                double sr = Math.Sign(radius);
                GeoVector x = sr * (T ^ y);

                // x' = sr * (T' x y + T x y')
                GeoVector xp = sr * ((Tp ^ y) + (T ^ yp));

                double sinV = Math.Sin(v);
                double cosV = Math.Cos(v);

                // Pu = vel + r*(cos v * x' + sin v * y')
                return vel + radius * (cosV * xp + sinV * yp);
            }
            else
            {
                var deriv = spine.PointAndDerivativesAt(u, 3).ToArray();

                GeoVector vel = deriv[1];                    // c′
                GeoVector acc = deriv[2];                    // c″
                GeoVector jerk = deriv[3];         // 3rd derivative   c'''(u)

                double speed = vel.Length;             // |c′|
                GeoVector T = vel / speed;            // Frenet-Tangent

                //  curvature & torsion (+ derivatives)
                GeoVector crossVA = vel ^ acc;              // c′ × c″
                double curvature = crossVA.Length / Pow(speed, 3);     // κ

                GeoVector crossVB = vel ^ jerk;             // c′ × c‴
                double torsion = (vel * crossVB) / Pow(crossVA.Length, 2); // τ

                // Frenet-Frame 
                GeoVector N = (acc - (acc * T) * T).Normalized;   // Hauptnormalen­vektor
                GeoVector B = T ^ N;                              // Binormale

                // Derivatives of the frame
                // scaling with s = |c′|
                double s = speed;
                GeoVector N_u = (-curvature * s) * T + torsion * s * B;
                GeoVector B_u = (-torsion * s) * N;


                // final results
                double sinV = Sin(v);
                double cosV = Cos(v);
                return vel + radius * (cosV * N_u + sinV * B_u);
            }
        }

        public  GeoVector UDirectionOld(GeoPoint2D uv)
        {
            double u = uv.x;
            double v = uv.y;

            if (normal != GeoVector.NullVector)
            {
                // Derivatives of the spine curve
                var deriv = spine.PointAndDerivativesAt(u, 2);
                var derivdbg = SurfaceIntersectionSolvers.NumericalPointAndDerivativesAt(spine, u, 2);
                GeoVector vel = deriv[1];         // 1st  derivative  c'(u)
                GeoVector acc = deriv[2];         // 2nd derivative   c''(u)

                // scalar helpers
                double speed = vel.Length;                              // |c'|
                double curvature = (vel ^ acc).Length / Pow(speed, 3);      // κ

                // frame vectors
                GeoVector tangent = vel / speed;                             // T

                // final results
                double sinV = Sin(v);

                return speed * (1 - radius * curvature * sinV) * tangent;
            }
            else
            {
                var deriv = spine.PointAndDerivativesAt(u, 3).ToArray();

                GeoVector vel = deriv[1];                    // c′
                GeoVector acc = deriv[2];                    // c″
                GeoVector jerk = deriv[3];         // 3rd derivative   c'''(u)

                double speed = vel.Length;             // |c′|
                GeoVector T = vel / speed;            // Frenet-Tangent

                //  curvature & torsion (+ derivatives)
                GeoVector crossVA = vel ^ acc;              // c′ × c″
                double curvature = crossVA.Length / Pow(speed, 3);     // κ

                GeoVector crossVB = vel ^ jerk;             // c′ × c‴
                double torsion = (vel * crossVB) / Pow(crossVA.Length, 2); // τ

                // Frenet-Frame 
                GeoVector N = (acc - (acc * T) * T).Normalized;   // Hauptnormalen­vektor
                GeoVector B = T ^ N;                              // Binormale

                // Derivatives of the frame
                // scaling with s = |c′|
                double s = speed;
                GeoVector N_u = (-curvature * s) * T + torsion * s * B;
                GeoVector B_u = (-torsion * s) * N;


                // final results
                double sinV = Sin(v);
                double cosV = Cos(v);
                return vel + radius * (cosV * N_u + sinV * B_u);
            }
        }
        public override GeoVector VDirection(GeoPoint2D uv)
        {
            double u = uv.x;
            double v = uv.y;

            if (normal != GeoVector.NullVector)
            {
                // Derivatives of the spine curve
                GeoVector vel = spine.DirectionAt(u);

                // scalar helpers
                double speed = vel.Length;                              // |c'|

                // frame vectors
                GeoVector tangent = vel / speed;                             // T
                GeoVector yAxis = (normal ^ tangent).Normalized;
                GeoVector xAxis = Sign(radius) * tangent ^ yAxis;

                // final results
                double sinV = Sin(v);
                double cosV = Cos(v);

                return -radius * sinV * xAxis + radius * cosV * yAxis;
            }
            else
            {
                var deriv = spine.PointAndDerivativesAt(u, 2).ToArray();

                GeoVector vel = deriv[1];                    // c′
                GeoVector acc = deriv[2];                    // c″

                double speed = vel.Length;             // |c′|
                GeoVector T = vel / speed;            // Frenet-Tangent

                // Frenet-Frame 
                GeoVector N = (acc - (acc * T) * T).Normalized;   // Hauptnormalen­vektor
                GeoVector B = Sign(radius) * T ^ N;                              // Binormale

                // final results
                double sinV = Sin(v);
                double cosV = Cos(v);
                return -radius * sinV * N + radius * cosV * B;
            }
        }
        public override ICurve2D GetProjectedCurve(ICurve curve, double precision)
        {
            if (curve is Ellipse e && e.IsCircle && Precision.IsEqual(e.Radius, Math.Abs(Radius)))
            {   // the circle might be the fixedU curve of this surface
                double cu = spine.PositionOf(e.Center);
                if (Precision.IsEqual(spine.PointAt(cu), e.Center))
                {
                    GeoPoint2D sp = PositionOf(e.StartPoint);
                    GeoPoint2D ep = PositionOf(e.EndPoint);
                    return new Line2D(sp, ep);
                }
            }
            return base.GetProjectedCurve(curve, precision);
        }
        public override void Derivative2At(GeoPoint2D uv, out GeoPoint location, out GeoVector du, out GeoVector dv, out GeoVector duu, out GeoVector dvv, out GeoVector duv)
        {
            double u = uv.x;
            double v = uv.y;

            if (normal != GeoVector.NullVector)
            {
                // Derivatives of the spine curve
                var deriv = spine.PointAndDerivativesAt(u, 3);
                GeoPoint spinePoint = GeoPoint.Origin + deriv[0];

                GeoVector vel = deriv[1];         // 1st  derivative  c'(u)
                GeoVector acc = deriv[2];         // 2nd derivative   c''(u)
                GeoVector jerk = deriv[3];         // 3rd derivative   c'''(u)

                // scalar helpers
                double speed = vel.Length;                              // |c'|
                double curvature = (vel ^ acc).Length / Pow(speed, 3);      // κ
                GeoVector crossVA = vel ^ acc;                               // w
                GeoVector crossVB = vel ^ jerk;                              // w'
                double curvatureDash = (crossVA * crossVB) /
                                        (crossVA.Length * Pow(speed, 3))
                                        - 3.0 * (vel * acc) * curvature /
                                        (speed * speed);                         // κ'

                double speedDash = (vel * acc) / speed;                     // s'

                // frame vectors
                GeoVector tangent = vel / speed;                             // T
                GeoVector yAxis = (normal ^ tangent).Normalized;                        // P
                GeoVector xAxis = Sign(radius) * tangent ^ yAxis;
                GeoVector tangent2nd = (curvatureDash * speed + curvature * speedDash)
                                        * yAxis
                                        - curvature * curvature * speed * speed * tangent;

                // final results
                double sinV = Sin(v);
                double cosV = Cos(v);

                location = spinePoint + radius * (cosV * xAxis + sinV * yAxis);
                du = speed * (1 - radius * curvature * sinV) * tangent;
                dv = -radius * sinV * xAxis + radius * cosV * yAxis;
                duu = acc + radius * sinV * (xAxis ^ tangent2nd);
                duv = -radius * curvature * speed * cosV * tangent;
                dvv = -(location - spinePoint);
                base.Derivative2At(uv, out GeoPoint _location, out GeoVector _du, out GeoVector _dv, out GeoVector _duu, out GeoVector _dvv, out GeoVector _duv); // for debug only
                GeoPoint __location = PointAt(uv); // for debug only
            }
            else
            {
                var deriv = spine.PointAndDerivativesAt(u, 3).ToArray();
                GeoPoint spinePoint = GeoPoint.Origin + deriv[0];

                GeoVector vel = deriv[1];                    // c′
                GeoVector acc = deriv[2];                    // c″
                GeoVector jerk = deriv[3];                    // c‴

                double speed = vel.Length;             // |c′|
                GeoVector T = vel / speed;            // Frenet-Tangent

                //  curvature & torsion (+ derivatives)
                GeoVector crossVA = vel ^ acc;              // c′ × c″
                double curvature = crossVA.Length / Pow(speed, 3);     // κ

                GeoVector crossVB = vel ^ jerk;             // c′ × c‴
                double torsion = (vel * crossVB) / Pow(crossVA.Length, 2); // τ

                // κ′ und τ′ we will need only for S_uu (T″)
                double curvatureDash = (crossVA * crossVB) / (crossVA.Length * Pow(speed, 3))
                                     - 3.0 * (vel * acc) * curvature / (speed * speed);

                double speedDash = (vel * acc) / speed;

                // Torsion-Ableitung
                double torsionDash =
                    (jerk * crossVB + vel * (acc ^ jerk)) / Pow(crossVA.Length, 2)
                    - 2.0 * torsion * (crossVA * crossVB) / Pow(crossVA.Length, 2);

                // Frenet-Frame 
                GeoVector N = (acc - (acc * T) * T).Normalized;   // Hauptnormalen­vektor
                GeoVector B = Sign(radius) * T ^ N;                              // Binormale

                // Derivatives of the frame
                // scaling with s = |c′|
                double s = speed;
                GeoVector T_u = curvature * s * N;                // T′
                GeoVector N_u = (-curvature * s) * T + torsion * s * B;
                GeoVector B_u = (-torsion * s) * N;

                // T″ = (κ′ s + κ s′) N + κ s           N′ + …
                GeoVector T_uu =
                    (curvatureDash * s + curvature * speedDash) * N
                  + curvature * s * N_u;                              // κ s N′-Anteil

                GeoVector N_uu =
                    (-curvatureDash * s - curvature * speedDash) * T
                  + (-curvature * s) * T_u
                  + torsionDash * s * B
                  + torsion * s * B_u;

                GeoVector B_uu =
                    (-torsionDash * s - torsion * speedDash) * N
                  + (-torsion * s) * N_u;

                // final results
                double sinV = Sin(v);
                double cosV = Cos(v);
                location = spinePoint + radius * (cosV * N + sinV * B);
                du = vel + radius * (cosV * N_u + sinV * B_u);
                dv = -radius * sinV * N + radius * cosV * B;
                duu = acc + radius * (cosV * N_uu + sinV * B_uu);
                duv = -radius * sinV * N_u + radius * cosV * B_u;
                dvv = -radius * cosV * N - radius * sinV * B;   // = –(location–spinePoint)
            }
        }
        public override IDualSurfaceCurve[] GetDualSurfaceCurves(BoundingRect thisBounds, ISurface otherSurface, BoundingRect otherBounds, List<GeoPoint> seeds, List<Tuple<double, double, double, double>> extremePositions)
        {   // test, whether it is a tangential intersection, e.g. when rounding edges
            double ds = otherSurface.GetDistance(spine.StartPoint);
            if (Abs(Abs(ds) - radius) < Precision.eps)
            {
                double de = otherSurface.GetDistance(spine.EndPoint);
                if (Abs(Abs(de) - radius) < Precision.eps)
                {
                    ISurface offsetSurface;
                    if (ds < 0) offsetSurface = otherSurface.GetOffsetSurface(radius);
                    else offsetSurface = otherSurface.GetOffsetSurface(-radius);
                    if (offsetSurface.IsCurveOnSurface(spine)) // Make sure, all surfaces support this as a quick check
                    {
                        ICurve2D spineOnOffset = offsetSurface.GetProjectedCurve(spine, 0.0);
                        ICurve res = otherSurface.Make3dCurve(spineOnOffset); // this is the perpendicular projection of the spine onto otherSurface for most surfaces
                        bool ok = true;
                        for (int i = 0; i < seeds.Count; i++)
                        {
                            ok &= res.DistanceTo(seeds[i]) < Precision.eps;
                            if (!ok) break;
                        }
                        if (ok)
                        {
                            GeoPoint2D sp2d = PositionOf(res.StartPoint);
                            GeoPoint2D ep2d = PositionOf(res.EndPoint);
                            SurfaceHelper.AdjustPeriodic(this, thisBounds, ref sp2d);
                            SurfaceHelper.AdjustPeriodic(this, thisBounds, ref ep2d);
                            Line2D l2d = new Line2D(sp2d, ep2d);
                            return [new DualSurfaceCurve(res, this, l2d, otherSurface, spineOnOffset)]; // it is not required to clip the curve
                        }
                    }
                }

            }
            return base.GetDualSurfaceCurves(thisBounds, otherSurface, otherBounds, seeds, extremePositions);
        }
        public override ISurface Clone()
        {
            return new SweptCircle(spine, radius);
        }
        public override void Modify(ModOp m)
        {
            if (m.IsIsogonal)
            {
                spine = spine.CloneModified(m);
                radius = m.Factor * radius;
                normal = m * normal;
            }
            else
            {
                throw new NotImplementedException("Modify not implemented for non isogonal matrices");
            }
        }
        public override ModOp2D ReverseOrientation()
        {
            // reversing the radius also afects the orientation of the circle
            radius = -radius;
            return new ModOp2D(1, 0, 0, 0, -1, 0);
        }
        public override void CopyData(ISurface CopyFrom)
        {
            SweptCircle cc = CopyFrom as SweptCircle;
            if (cc != null)
            {
                this.spine = cc.spine;
                this.radius = cc.radius;
                this.normal = cc.normal;
            }
        }
        public override bool SameGeometry(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, double precision, out ModOp2D firstToSecond)
        {
            if (other is SweptCircle sc)
            {
                if (sc.spine.SameGeometry(spine, Precision.eps))
                {
                    if (Math.Abs(sc.radius - radius) < Precision.eps)
                    {
                        firstToSecond = ModOp2D.Null; // we have to implement this fully
                        return true;
                    }
                }
            }
            firstToSecond = ModOp2D.Null;
            return false;
        }
        public override void GetNaturalBounds(out double umin, out double umax, out double vmin, out double vmax)
        {
            umin = 0.0; umax = 1.0;
            vmin = 0.0; vmax = 2 * PI;
        }

        /// <summary>
        /// Liefert den v-Parameter zum Punkt p, für gegebenes u.
        /// </summary>
        private double InverseV(double u, GeoPoint p)
        {
            GeoPoint spinePoint = spine.PointAt(u);
            GeoVector tangent = spine.DirectionAt(u).Normalized;
            GeoVector yAxis = (normal ^ tangent).Normalized;
            GeoVector xAxis = Sign(radius) * tangent ^ yAxis;
            GeoVector d = (p - spinePoint) / radius;

            double x = d * xAxis;
            double y = d * yAxis;
            double v = Math.Atan2(y, x);
            if (v < 0.0) v += 2.0 * Math.PI;
            return v;
        }

        /// <summary>
        /// Maps a (maybe unwrapped) spine parameter into the parameter range of the spine. Only a closed spine is
        /// periodic, for an open spine the parameter is returned unchanged.
        /// </summary>
        private double NormalizedSpineParameter(double u)
        {
            if (!spine.IsClosed) return u;
            return u - Floor(u);
        }

        /// <summary>
        /// The signed curvature of the spine at <paramref name="u"/>, measured with respect to the "yAxis"
        /// (normal ^ tangent) of this surface. Only meaningful when the spine is planar and
        /// <paramref name="unitNormal"/> is the normalized normal vector of its plane.
        /// </summary>
        private double SpineCurvature(double u, GeoVector unitNormal)
        {
            IReadOnlyList<GeoVector> deriv = spine.PointAndDerivativesAt(NormalizedSpineParameter(u), 2);
            double l = deriv[1].Length;
            if (l < 1e-13) return 0.0;
            return ((deriv[1] ^ deriv[2]) * unitNormal) / (l * l * l);
        }

        /// <summary>
        /// Point and derivative of the parallel (offset) curve of the spine at the signed distance
        /// <paramref name="dist"/>, measured in the direction of the "yAxis" of this surface and expressed in the
        /// coordinate system of <paramref name="pln"/>. The intersection of this surface with a plane parallel to the
        /// plane of the spine is exactly such an offset curve (with dist == radius*sin(v) at the height
        /// |radius|*cos(v)), which is why the self intersection of the surface can be reduced to the self
        /// intersection of these 2d curves.
        /// </summary>
        private void OffsetCurveAt(double u, double dist, GeoVector unitNormal, Plane pln, out GeoPoint2D point, out GeoVector2D dir)
        {
            IReadOnlyList<GeoVector> deriv = spine.PointAndDerivativesAt(NormalizedSpineParameter(u), 2);
            double l = deriv[1].Length;
            double curvature = ((deriv[1] ^ deriv[2]) * unitNormal) / (l * l * l);
            GeoVector yAxis = (unitNormal ^ deriv[1]).Normalized;
            point = pln.Project(GeoPoint.Origin + deriv[0] + dist * yAxis);
            // the offset curve has the direction of the spine, scaled by (1-dist*curvature), which vanishes at its cusps
            dir = pln.Project((1.0 - dist * curvature) * deriv[1]);
        }

        /// <summary>
        /// Newton iteration for a double point of the offset curve at the distance <paramref name="dist"/>: two
        /// different spine parameters which yield the same point on that offset curve. <paramref name="u1"/> and
        /// <paramref name="u2"/> must already be close to the solution, they are only modified on success.
        /// </summary>
        private bool RefineDoublePoint(double dist, GeoVector unitNormal, Plane pln, ref double u1, ref double u2)
        {
            double p1 = u1, p2 = u2;
            for (int i = 0; i < 30; i++)
            {
                OffsetCurveAt(p1, dist, unitNormal, pln, out GeoPoint2D pnt1, out GeoVector2D dir1);
                OffsetCurveAt(p2, dist, unitNormal, pln, out GeoPoint2D pnt2, out GeoVector2D dir2);
                GeoVector2D err = pnt1 - pnt2;
                if (err.Length < Precision.eps * 1e-3)
                {
                    u1 = p1;
                    u2 = p2;
                    return true;
                }
                // solve dir1*du1 - dir2*du2 == -err
                double det = dir2.x * dir1.y - dir1.x * dir2.y;
                if (Abs(det) < 1e-13) return false; // the two sheets are tangential here, Newton does not work
                p1 += (err.x * dir2.y - dir2.x * err.y) / det;
                p2 += (err.x * dir1.y - dir1.x * err.y) / det;
                if (p2 - p1 < 1e-8) return false; // the two branches collapsed into a single point
                if (!spine.IsClosed && (p1 < 0.0 || p1 > 1.0 || p2 < 0.0 || p2 > 1.0)) return false;
            }
            return false;
        }

        /// <summary>
        /// Searches a double point of the offset curve at the distance <paramref name="dist"/> by intersecting a
        /// polygonal approximation of that offset curve with itself. Of all double points which enclose
        /// <paramref name="uVertex"/> the one with the smallest parameter range is returned: this is the one caused
        /// by the fold around <paramref name="uVertex"/> and not by a global self penetration of the pipe.
        /// </summary>
        private bool FindDoublePoint(double dist, double uVertex, GeoVector unitNormal, Plane pln, out double u1, out double u2)
        {
            const int samples = 400;
            GeoPoint2D[] pnts = new GeoPoint2D[samples + 1];
            for (int i = 0; i <= samples; i++) OffsetCurveAt(i / (double)samples, dist, unitNormal, pln, out pnts[i], out GeoVector2D _);
            u1 = u2 = 0.0;
            bool found = false;
            for (int i = 0; i < samples; i++)
            {
                for (int j = i + 3; j < samples; j++)
                {   // neighboring segments are skipped: close to a cusp of the offset curve they intersect each other
                    if (spine.IsClosed && samples - (j - i) < 3) continue;
                    GeoVector2D dir1 = pnts[i + 1] - pnts[i];
                    GeoVector2D dir2 = pnts[j + 1] - pnts[j];
                    double det = dir1.x * dir2.y - dir1.y * dir2.x;
                    if (Abs(det) < 1e-30) continue;
                    GeoVector2D off = pnts[j] - pnts[i];
                    double s = (off.x * dir2.y - off.y * dir2.x) / det;
                    double t = (off.x * dir1.y - off.y * dir1.x) / det;
                    if (s < 0.0 || s >= 1.0 || t < 0.0 || t >= 1.0) continue;
                    double p1 = (i + s) / samples;
                    double p2 = (j + t) / samples;
                    if (p1 >= uVertex || p2 <= uVertex) continue; // the double point must enclose the fold
                    if (found && p2 - p1 >= u2 - u1) continue;
                    u1 = p1;
                    u2 = p2;
                    found = true;
                }
            }
            if (!found) return false;
            return RefineDoublePoint(dist, unitNormal, pln, ref u1, ref u2);
        }

        /// <summary>
        /// The angle by which the double curve is sampled: it runs from -halfWidth (the first swallowtail point) at
        /// index 0 over 0 (the plane of the spine) at index steps to +halfWidth at index 2*steps. The samples are
        /// dense at both ends, where the two branches separate like the square root of the distance to the
        /// swallowtail point, so that the steps in u stay about equal.
        /// </summary>
        private static double DoubleCurveAngle(int index, int steps, double halfWidth)
        {
            double x = (index - steps) / (double)steps;
            return Sign(x) * halfWidth * Cos(PI / 2.0 * (1.0 - Abs(x)));
        }

        /// <summary>
        /// Bisects between an angle where the double curve exists inside the domain of the spine
        /// (<paramref name="good"/>, with the corresponding parameters in <paramref name="u1"/> and
        /// <paramref name="u2"/>) and an angle where it does not (<paramref name="bad"/>). Returns the last angle
        /// where it exists and updates the parameters accordingly: this is where the double curve leaves the domain.
        /// </summary>
        private double BisectDoubleCurveEnd(double good, double bad, double dist0, GeoVector unitNormal, Plane pln,
            ref double u1, ref double u2)
        {
            for (int i = 0; i < 30; i++)
            {
                double mid = (good + bad) / 2.0;
                double p1 = u1, p2 = u2;
                if (RefineDoublePoint(dist0 * Cos(mid), unitNormal, pln, ref p1, ref p2))
                {
                    good = mid;
                    u1 = p1;
                    u2 = p2;
                }
                else bad = mid;
            }
            return good;
        }

        /// <summary>
        /// Follows the double curve with Newton from the angle at index <paramref name="fromIndex"/> to the end of
        /// its v range in the given direction and appends the points to the two branches in marching order. The last
        /// point is the swallowtail point, where the two branches meet, or, when the double curve leaves the domain
        /// of the spine before, the point where it does so.
        /// </summary>
        private void MarchDoubleCurve(int direction, int fromIndex, int steps, double halfWidth, double vCenter,
            double dist0, double uVertex, double u1, double u2, GeoVector unitNormal, Plane pln,
            List<GeoPoint2D> branch1, List<GeoPoint2D> branch2)
        {
            double lastAngle = DoubleCurveAngle(fromIndex, steps, halfWidth);
            for (int i = fromIndex + direction; i >= 1 && i <= 2 * steps - 1; i += direction)
            {
                double angle = DoubleCurveAngle(i, steps, halfWidth);
                if (!RefineDoublePoint(dist0 * Cos(angle), unitNormal, pln, ref u1, ref u2))
                {   // the double curve leaves the domain of the spine between the last two angles
                    double end = BisectDoubleCurveEnd(lastAngle, angle, dist0, unitNormal, pln, ref u1, ref u2);
                    branch1.Add(new GeoPoint2D(u1, vCenter + end));
                    branch2.Add(new GeoPoint2D(u2, vCenter + end));
                    return;
                }
                branch1.Add(new GeoPoint2D(u1, vCenter + angle));
                branch2.Add(new GeoPoint2D(u2, vCenter + angle));
                lastAngle = angle;
            }
            // the swallowtail point, where the two branches meet
            branch1.Add(new GeoPoint2D(uVertex, vCenter + direction * halfWidth));
            branch2.Add(new GeoPoint2D(uVertex, vCenter + direction * halfWidth));
        }

        /// <summary>
        /// Moves a pair of branches into the given domain (v is periodic with 2*pi, u with 1 when the spine is
        /// closed) and adds it to <paramref name="res"/>: first the branch with the bigger u with ascending v, then
        /// the other one with descending v, so that both together enclose the hidden part of the surface.
        /// </summary>
        private void AddSelfIntersectionPair(List<GeoPoint2D> branch1, List<GeoPoint2D> branch2, BoundingRect bounds, List<ICurve2D> res)
        {
            if (branch1.Count < 2 || branch2.Count < 2) return;
            BoundingRect ext = new BoundingRect(branch1.ToArray());
            ext.MinMax(new BoundingRect(branch2.ToArray()));
            double shiftV = Round(((bounds.Bottom + bounds.Top) / 2.0 - (ext.Bottom + ext.Top) / 2.0) / (2 * PI)) * 2 * PI;
            double shiftU = 0.0;
            if (spine.IsClosed) shiftU = Round((bounds.Left + bounds.Right) / 2.0 - (ext.Left + ext.Right) / 2.0);
            if (shiftU != 0.0 || shiftV != 0.0)
            {
                GeoVector2D shift = new GeoVector2D(shiftU, shiftV);
                for (int i = 0; i < branch1.Count; i++) branch1[i] = branch1[i] + shift;
                for (int i = 0; i < branch2.Count; i++) branch2[i] = branch2[i] + shift;
                ext.Move(shift);
            }
            if (!ext.Interferes(ref bounds)) return;
            ICurve2D ascending = MakeCurve2D(branch2); // the branch with the bigger u, ascending v
            ICurve2D descending = MakeCurve2D(branch1); // the branch with the smaller u, to be reversed
            if (ascending == null || descending == null) return;
            descending.Reverse();
            res.Add(ascending);
            res.Add(descending);
        }

        /// <summary>
        /// The position of the maximum of the absolute curvature of the spine in the given interval, i.e. the vertex
        /// of the spine. This is where the two branches of the self intersection meet in a swallowtail point.
        /// </summary>
        private double MaxCurvaturePosition(double from, double to, GeoVector unitNormal)
        {
            const int samples = 32;
            double best = from, bestValue = double.MinValue;
            for (int i = 0; i <= samples; i++)
            {
                double u = from + (to - from) * i / samples;
                double c = Abs(SpineCurvature(u, unitNormal));
                if (c > bestValue) { bestValue = c; best = u; }
            }
            double a = Max(from, best - (to - from) / samples);
            double b = Min(to, best + (to - from) / samples);
            const double gr = 0.6180339887498949; // golden section
            double x1 = b - gr * (b - a), x2 = a + gr * (b - a);
            double f1 = Abs(SpineCurvature(x1, unitNormal)), f2 = Abs(SpineCurvature(x2, unitNormal));
            for (int i = 0; i < 60 && b - a > 1e-12; i++)
            {
                if (f1 > f2)
                {
                    b = x2; x2 = x1; f2 = f1;
                    x1 = b - gr * (b - a); f1 = Abs(SpineCurvature(x1, unitNormal));
                }
                else
                {
                    a = x1; x1 = x2; f1 = f2;
                    x2 = a + gr * (b - a); f2 = Abs(SpineCurvature(x2, unitNormal));
                }
            }
            return (a + b) / 2.0;
        }

        /// <summary>
        /// A 2d curve through the provided points: a cubic spline when there are enough points, a polyline otherwise.
        /// Returns null when too few points remain after removing duplicates.
        /// </summary>
        private static ICurve2D MakeCurve2D(List<GeoPoint2D> points)
        {
            List<GeoPoint2D> pnts = new List<GeoPoint2D>(points.Count);
            for (int i = 0; i < points.Count; i++)
            {
                if (pnts.Count > 0 && (points[i] | pnts[pnts.Count - 1]) < 1e-9) continue;
                pnts.Add(points[i]);
            }
            if (pnts.Count < 2) return null;
            if (pnts.Count < 4) return new Polyline2D(pnts.ToArray());
            return new BSpline2D(pnts.ToArray(), 3, false);
        }

        /// <summary>
        /// The curves in the (u,v) system where this surface penetrates itself. When the curvature radius of the
        /// spine falls below the radius of the swept circle, the surface folds over and two of its sheets cross each
        /// other. This is the pipe analogon of the spindle torus, but since the curvature of the spine is not
        /// constant, the two poles of the spindle torus unfold into two swallowtail points (at the vertex of the
        /// spine, i.e. where its curvature is maximal) which are connected by a real double curve.
        /// The result is a list of pairs: the curves 2*i and 2*i+1 are the two branches of one and the same 3d curve,
        /// running in opposite directions, so that both together enclose the part of the surface which is hidden
        /// inside the pipe (positively oriented, the hidden part is on the left hand side of both branches).
        /// Note that this is not the same as the curve where the surface normal vanishes: that one is
        /// sin(v) == curvatureRadius(u)/radius and lies completely inside the area enclosed by the double curve.
        /// Only implemented for a planar spine, for a non planar one null is returned. Where the double curve leaves
        /// the domain of the spine, it is clipped; should it leave and enter again, only the piece which is found
        /// first is returned. Not covered is a global self penetration, where the spine comes closer to itself than
        /// twice the radius.
        /// </summary>
        /// <param name="bounds">only self intersections within this domain are returned</param>
        /// <returns>pairs of 2d curves or null, when the surface does not intersect itself</returns>
        public override ICurve2D[] GetSelfIntersections(BoundingRect bounds)
        {
            if (spine.GetPlanarState() != PlanarState.Planar) return null; // only implemented for a planar spine
            if (normal.IsNullVector()) return null;
            double absRadius = Abs(radius);
            if (absRadius < Precision.eps) return null;
            GeoVector unitNormal = normal.Normalized;
            Plane pln = new Plane(spine.PointAt(0.0), unitNormal); // to express the offset curves of the spine in 2d

            // the surface is folded where the curvature radius of the spine is smaller than the radius of the circle.
            // A fold may be narrow, so the initial grid must be fine enough to see it at all.
            Func<double, double> f = u => Abs(absRadius * SpineCurvature(u, unitNormal)) - 1.0;
            List<double> limits = new List<double>();
            limits.Add(0.0);
            limits.AddRange(AdaptiveRootFinder.FindRootsAdaptive(f, 0.0, 1.0, 200));
            limits.Add(1.0);

            List<ICurve2D> res = new List<ICurve2D>();
            for (int i = 0; i < limits.Count - 1; i++)
            {
                if (limits[i + 1] - limits[i] < 1e-6) continue;
                // between two consecutive roots the sign does not change, so the midpoint decides whether the
                // surface is folded in this interval or not
                if (Abs(absRadius * SpineCurvature((limits[i] + limits[i + 1]) / 2.0, unitNormal)) <= 1.0) continue;
                double uVertex = MaxCurvaturePosition(limits[i], limits[i + 1], unitNormal);
                double curvature = SpineCurvature(uVertex, unitNormal);
                double sinMin = 1.0 / (absRadius * Abs(curvature)); // curvatureRadius_min/|radius|, less than 1 in a fold
                if (!(sinMin < 1.0)) continue; // not folded in this interval (also catches a curvature of 0)
                // the v values of the double curve are symmetric to vCenter, the half width of its v range is halfWidth
                double vCenter = Sign(radius) * Sign(curvature) * PI / 2.0;
                double halfWidth = Acos(sinMin);
                double dist0 = absRadius * Sign(curvature); // the offset distance in the plane of the spine (v == vCenter)

                // find a starting point: in the plane of the spine (angle 0) the two branches are farthest apart and
                // therefore easiest to find. Only when the double curve is outside the domain of the spine there, we
                // try closer to the swallowtail points, where it moves towards the vertex.
                const int steps = 32;
                int seed = -1;
                double u1 = 0.0, u2 = 0.0;
                for (int k = 0; k < steps && seed < 0; k += 2)
                {
                    for (int s = -1; s <= 1 && seed < 0; s += 2)
                    {
                        if (k == 0 && s > 0) continue; // the middle is tried only once
                        double angle = DoubleCurveAngle(steps + s * k, steps, halfWidth);
                        if (FindDoublePoint(dist0 * Cos(angle), uVertex, unitNormal, pln, out u1, out u2)) seed = steps + s * k;
                    }
                }
                if (seed < 0) continue; // the double curve is not inside the domain of the spine

                // from there follow it with Newton in both directions, which also works close to the swallowtail
                // points, where the two branches come arbitrarily close to each other
                List<GeoPoint2D> branch1 = new List<GeoPoint2D>(), branch2 = new List<GeoPoint2D>(); // ascending v
                MarchDoubleCurve(-1, seed, steps, halfWidth, vCenter, dist0, uVertex, u1, u2, unitNormal, pln, branch1, branch2);
                branch1.Reverse();
                branch2.Reverse();
                double seedAngle = DoubleCurveAngle(seed, steps, halfWidth);
                branch1.Add(new GeoPoint2D(u1, vCenter + seedAngle));
                branch2.Add(new GeoPoint2D(u2, vCenter + seedAngle));
                MarchDoubleCurve(1, seed, steps, halfWidth, vCenter, dist0, uVertex, u1, u2, unitNormal, pln, branch1, branch2);
                AddSelfIntersectionPair(branch1, branch2, bounds, res);
            }
            if (res.Count == 0) return null;
            return res.ToArray();
        }

        /// <summary>
        /// Appends a line to an outline, skipping it when start and end point coincide.
        /// </summary>
        private static void AddLine2D(List<ICurve2D> outline, GeoPoint2D from, GeoPoint2D to)
        {
            if ((from | to) > 1e-9) outline.Add(new Line2D(from, to));
        }

        /// <summary>
        /// The outer boundary of the pipe as faces on this surface. Without a self intersection this is a single
        /// face over the whole domain. Where the surface penetrates itself, the part hidden inside the double curve
        /// (see <see cref="GetSelfIntersections(BoundingRect)"/>) is cut away and the surface is split at the vertex
        /// of the spine, so that two neighboring faces meet along the double curve. They share up to three edges:
        /// the split line below the fold, the double curve itself (tangential contact in the two swallowtail points,
        /// transversal in between) and the split line above the fold.
        /// </summary>
        /// <param name="vmin">lower bound of the v range</param>
        /// <param name="vmax">upper bound of the v range</param>
        /// <returns>the faces of the outer shell, or null when the self intersection cannot be resolved</returns>
        public Face[] OuterShell(double vmin, double vmax)
        {
            if (vmax <= vmin) vmax += 2 * PI;
            BoundingRect domain = new BoundingRect(0.0, vmin, 1.0, vmax);
            ICurve2D[] selfIntersections = GetSelfIntersections(domain);
            if (selfIntersections == null || selfIntersections.Length == 0) return new Face[] { Face.MakeFace(this, domain) };

            // one pair of branches per fold: [2*i] is the branch with the bigger u and ascends in v, [2*i+1] the one
            // with the smaller u and descends. Both meet in the two swallowtail points, which lie at the vertex of
            // the spine, and that is where the surface is split.
            List<(double uVertex, ICurve2D ascending, ICurve2D descending)> folds = new List<(double, ICurve2D, ICurve2D)>();
            for (int i = 0; i + 1 < selfIntersections.Length; i += 2)
            {
                ICurve2D ascending = selfIntersections[i], descending = selfIntersections[i + 1];
                double uVertex = ascending.StartPoint.x;
                // only a double curve which is a complete loop well inside the domain can be used to split here
                if (Abs(ascending.EndPoint.x - uVertex) > 1e-6) return null;
                if (ascending.StartPoint.y < vmin || ascending.EndPoint.y > vmax) return null;
                BoundingRect ext = ascending.GetExtent();
                ext.MinMax(descending.GetExtent());
                if (ext.Left <= 0.0 || ext.Right >= 1.0) return null;
                folds.Add((uVertex, ascending, descending));
            }
            if (folds.Count == 0) return null;
            folds.Sort((a, b) => a.uVertex.CompareTo(b.uVertex));
            for (int i = 1; i < folds.Count; i++)
            {   // two folds must stay apart, otherwise the outlines would run into each other
                if (folds[i - 1].ascending.GetExtent().Right >= folds[i].descending.GetExtent().Left) return null;
            }

            Face[] res = new Face[folds.Count + 1];
            for (int i = 0; i < res.Length; i++)
            {
                double leftU = i == 0 ? 0.0 : folds[i - 1].uVertex;
                double rightU = i == folds.Count ? 1.0 : folds[i].uVertex;
                List<ICurve2D> outline = new List<ICurve2D>();
                AddLine2D(outline, new GeoPoint2D(leftU, vmin), new GeoPoint2D(rightU, vmin));
                if (i < folds.Count)
                {   // on the right hand side go around the branch of the fold which bulges into this part
                    ICurve2D bulge = folds[i].descending.CloneReverse(true); // the smaller u, now ascending
                    AddLine2D(outline, new GeoPoint2D(rightU, vmin), bulge.StartPoint);
                    outline.Add(bulge);
                    AddLine2D(outline, bulge.EndPoint, new GeoPoint2D(rightU, vmax));
                }
                else AddLine2D(outline, new GeoPoint2D(rightU, vmin), new GeoPoint2D(rightU, vmax));
                AddLine2D(outline, new GeoPoint2D(rightU, vmax), new GeoPoint2D(leftU, vmax));
                if (i > 0)
                {   // the same on the left hand side, where the outline runs downwards
                    ICurve2D bulge = folds[i - 1].ascending.CloneReverse(true); // the bigger u, now descending
                    AddLine2D(outline, new GeoPoint2D(leftU, vmax), bulge.StartPoint);
                    outline.Add(bulge);
                    AddLine2D(outline, bulge.EndPoint, new GeoPoint2D(leftU, vmin));
                }
                else AddLine2D(outline, new GeoPoint2D(leftU, vmax), new GeoPoint2D(leftU, vmin));

                SweptCircle part = Clone() as SweptCircle; // every face gets its own surface with its own domain
                BoundingRect ext = BoundingRect.EmptyBoundingRect;
                for (int j = 0; j < outline.Count; j++) ext.MinMax(outline[j].GetExtent());
                part.SetBounds(ext);
                res[i] = Face.MakeFace(part, new SimpleShape(new Border(outline.ToArray())));
            }
            return res;
        }
        public List<GeoPoint2D> SelfIntParams(int samples = 100)
        {

            Func<double, double> f = u => spine.CurvatureAt(u).radius - radius;
            double lastu = 0.0;
            List<double> roots = new List<double>();
            for (int i = 0; i <= samples; ++i)
            {
                double u = (double)i / (double)samples;
                double fu = f(u);
                if (i > 0 && Sign(f(u)) != Sign(f(lastu)))
                {
                    double root = Brent.FindRoot(f, lastu, u);
                    roots.Add(root);
                }
                lastu = u;
            }
            BoundingRect ext1 = new BoundingRect(0, 0, roots[0], PI);
            BoundingRect ext2 = new BoundingRect(roots[1], 0, 0.8, PI);
            Face fc1 = Face.MakeFace(Clone(), ext1);
            Face fc2 = Face.MakeFace(Clone(), ext2);
            if (roots.Count == 2)
            {
                ICurve c1 = FixedU(roots[0], 0, 2 * PI);
                ICurve c2 = FixedU(roots[1], 0, 2 * PI);
                Curves.Intersect(c1, c2, out double[] _, out double[] _, out GeoPoint[] intersectionPoints);
                GeoPoint2D uvs1, uve1, uvs2, uve2;
                if (intersectionPoints.Length == 2)
                {
                    double v0 = InverseV(roots[0], intersectionPoints[0]);
                    double v1 = InverseV(roots[1], intersectionPoints[0]);
                    double v = (v0 + v1) / 2;
                    uvs1 = new GeoPoint2D(roots[0], v0);
                    uvs2 = new GeoPoint2D(roots[1], v1);
                    v0 = InverseV(roots[0], intersectionPoints[1]);
                    v1 = InverseV(roots[1], intersectionPoints[1]);
                    v = (v0 + v1) / 2;
                    uve1 = new GeoPoint2D(roots[0], v0);
                    uve2 = new GeoPoint2D(roots[1], v1);
                    ICurve cc = FixedV((uvs1.y + uve1.y) / 2, 0, roots[0]);
                    ICurve cca = cc.Approximate(true, 0.1);
                    SweptCircle clone = Clone() as SweptCircle;
                    clone.SetBounds(new BoundingRect(roots[1], 0, 1, 2 * PI));
                    clone.Intersect(cc, ext2, out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve3Ds);
                }
            }
            return new List<GeoPoint2D>();
            //var result = new List<GeoPoint2D>();

            //// grobes Gitter
            //for (int i = 0; i <= samples; ++i)
            //{
            //    double u = (double)i / (double)samples;
            //    var d = spine.PointAndDerivativesAt(u, 2);
            //    GeoVector v1 = d[1], v2 = d[2];

            //    double speed = v1.Length;
            //    double curvature = (v1 ^ v2).Length / Pow(speed, 3);
            //    double cr = 1 / curvature;
            //    double rk = radius * curvature;

            //    if (rk > 1.0 + 1e-9)          // Toleranz
            //    {
            //        double phi = Acos(1.0 / rk);
            //        result.Add(new GeoPoint2D(u, phi + PI / 2.0));    // obere Linie
            //        result.Insert(0, new GeoPoint2D(u, -phi + PI / 2.0));    // untere Linie (symmetrisch)
            //    }
            //}
            //return result;
        }


        #region ISerializable
        protected SweptCircle(SerializationInfo info, StreamingContext context)
        {
            spine = (ICurve)info.GetValue("Spine", typeof(ICurve));
            radius = (double)info.GetValue("Radius", typeof(double));
            normal = (GeoVector)info.GetValue("Normal", typeof(GeoVector));
        }
        public void GetObjectData(SerializationInfo info, StreamingContext context)
        {
            info.AddValue("Spine", spine, typeof(ICurve));
            info.AddValue("Radius", radius, typeof(double));
            info.AddValue("Normal", normal, typeof(GeoVector));
        }

        #endregion
        #region IJsonSerialize
        protected SweptCircle() { } // we need this for JsonSerialisation
        public void GetObjectData(IJsonWriteData data)
        {
            data.AddProperty("Spine", spine);
            data.AddProperty("Radius", radius);
            data.AddProperty("Normal", normal);

        }

        public void SetObjectData(IJsonReadData data)
        {
            spine = data.GetProperty<ICurve>("Spine");
            radius = data.GetProperty<double>("Radius");
            normal = data.GetProperty<GeoVector>("Normal");
        }

        public ICurve Axis(BoundingRect domain)
        {
            return spine;
        }

        public bool ModifyAxis(GeoPoint throughPoint)
        {
            return false;
        }

        #endregion
    }

}
