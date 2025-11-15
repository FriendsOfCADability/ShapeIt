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
        // with help from https://chatgpt.com/c/686bffb7-52b0-8013-955c-8331d5ce2ad2
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

        public SweptCircle(ICurve spine, double radius)
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
        private static GeoVector FindSweepNormal(ICurve curve, double[] parameters)
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
            const double ratioThreshold = 0.5;
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
                sweptCircle.Derivation2At(new GeoPoint2D(posToParam(position), v0), out GeoPoint location, out GeoVector du, out GeoVector dv, out GeoVector duu, out GeoVector dvv, out GeoVector duv);
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
            public override void GetObjectData(IJsonWriteData data)
            {
                base.GetObjectData(data);
                data.AddProperty("SweptCircle", sweptCircle);
                data.AddProperty("V0", v0);
                data.AddProperty("Umin", umin);
                data.AddProperty("Umax", umax);
            }

            public override void SetObjectData(IJsonReadData data)
            {
                base.SetObjectData(data);
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
                // DebuggerContainer dc = this.BoxedSurfaceEx.Debug;
#endif
                // commented out, because it too often throws exceptions
                //if (BoxedSurfaceExtension.PositionOfMN(this, p, ref uv, out double dist)) return uv;
                //uv = new GeoPoint2D(u, v);
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
                return new GeoPoint2D(u, v);
            }
        }
        public override GeoVector UDirection(GeoPoint2D uv)
        {
            double u = uv.x;
            double v = uv.y;

            if (normal != GeoVector.NullVector)
            {
                // Derivatives of the spine curve
                var deriv = spine.PointAndDerivativesAt(u, 2);

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

        public override void Derivation2At(GeoPoint2D uv, out GeoPoint location, out GeoVector du, out GeoVector dv, out GeoVector duu, out GeoVector dvv, out GeoVector duv)
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

        public Face[] OuterShell(double vmin, double vmax)
        {
            if (CriticalPositions.Length == 0)
            {
                // there is no self intersection
                BoundingRect b = new BoundingRect(0, vmin, 1, vmax);
                return new Face[] { Face.MakeFace(this, b) };
            }
            else if (CriticalPositions.Length == 2)
            {
                if (spine.CurvatureAt((CriticalPositions[0] + CriticalPositions[1]) / 2.0).radius < radius)
                {   // exclude the part between the two critical positions
                    ICurve c1 = FixedU(CriticalPositions[0], 0, 2 * PI);
                    ICurve c2 = FixedU(CriticalPositions[1], 0, 2 * PI);
                    Curves.Intersect(c1, c2, out double[] _, out double[] _, out GeoPoint[] intersectionPoints);
                    // the two circles should intersect at two points
                    if (intersectionPoints.Length == 2)
                    {
                        GeoPoint cusp0, cusp1;
                        double v0 = InverseV(CriticalPositions[0], intersectionPoints[0]);
                        double v1 = InverseV(CriticalPositions[1], intersectionPoints[0]);
                        double vm0 = (v0 + v1) / 2;
                        // v0 and v1 should be almost identical
                        double v2 = InverseV(CriticalPositions[0], intersectionPoints[1]);
                        double v3 = InverseV(CriticalPositions[1], intersectionPoints[1]);
                        double vm1 = (v2 + v3) / 2;
                        double ivmin = Min(vm0, vm1);
                        double ivmax = Max(vm0, vm1);
                        if (vmin > vmax) vmax += 2 * PI;
                        // vmin and vmax define the domain, when both ivmin and ivmax are outside, there is no folding
                        if (ivmin < vmin) ivmin += 2 * PI;
                        if (ivmin > vmax) ivmin -= 2 * PI;
                        if (ivmax < vmin) ivmax += 2 * PI;
                        if (ivmax > vmax) ivmax -= 2 * PI;
                        // calculate the two cusp points. both PointAt should return the same 3d point
                        cusp0 = new GeoPoint(PointAt(new GeoPoint2D(CriticalPositions[0], vm0)), PointAt(new GeoPoint2D(CriticalPositions[1], vm0)));
                        cusp1 = new GeoPoint(PointAt(new GeoPoint2D(CriticalPositions[0], vm1)), PointAt(new GeoPoint2D(CriticalPositions[1], vm1)));
                        // use v-values between lo and high and also ivmin and ivmax when inside lo to hi
                        // v-values should be about pi/10 (18°) but also contain ivmin and ivmax
                        List<double> vValues = new List<double>(); // v-values for the grid
                        int cusp0Index = int.MaxValue, cusp1Index = int.MinValue;
                        double lastv = vmin;
                        if (ivmin > vmin && ivmin < vmax)
                        {   // from vmin to first cusp
                            int n = (int)Max(3, Ceiling((ivmin - lastv) / (PI / 10)));
                            vValues.AddRange(Enumerable.Range(0, n - 1).Select(i => lastv + (ivmin - lastv) * i / (n - 1)));
                            lastv = ivmin;
                            cusp0Index = vValues.Count;
                        }
                        if (ivmax > vmin && ivmax < vmax)
                        {   // from first cusp to second cusp
                            int n = (int)Max(3, Ceiling((ivmax - lastv) / (PI / 10)));
                            vValues.AddRange(Enumerable.Range(0, n - 1).Select(i => lastv + (ivmax - lastv) * i / (n - 1)));
                            lastv = ivmax;
                            cusp1Index = vValues.Count;
                        }
                        {   // from second cusp to vmax
                            int n = (int)Max(3, Ceiling((vmax - lastv) / (PI / 10)));
                            vValues.AddRange(Enumerable.Range(0, n).Select(i => lastv + (vmax - lastv) * i / (n - 1)));
                        }
                        double umiddle = (CriticalPositions[0] + CriticalPositions[1]) / 2.0;
                        double[] sp = spine.GetSavePositions();
                        int minu = 5;
                        List<double> u0Values = new List<double>(); // uvalues for the first part
                        if (sp.Count(x => x < umiddle) >= minu) u0Values.AddRange(sp.Where(x => x < umiddle));
                        else u0Values.AddRange(Enumerable.Range(0, minu).Select(i => i * umiddle / (minu - 1)));
                        List<double> u1Values = new List<double>(); // u-values for the second part
                        if (sp.Count(x => x > umiddle) >= minu) u1Values.AddRange(sp.Where(x => x > umiddle));
                        else u1Values.AddRange(Enumerable.Range(0, minu).Select(i => umiddle + i * (1 - umiddle) / (minu - 1)));
                        GeoPoint[,] points0 = new GeoPoint[u0Values.Count, vValues.Count];
                        GeoVector[,] du0 = new GeoVector[u0Values.Count, vValues.Count];
                        GeoVector[,] dv0 = new GeoVector[u0Values.Count, vValues.Count];
                        GeoVector[,] duv0 = new GeoVector[u0Values.Count, vValues.Count];
                        GeoPoint[,] points1 = new GeoPoint[u1Values.Count, vValues.Count];
                        GeoVector[,] du1 = new GeoVector[u1Values.Count, vValues.Count];
                        GeoVector[,] dv1 = new GeoVector[u1Values.Count, vValues.Count];
                        GeoVector[,] duv1 = new GeoVector[u1Values.Count, vValues.Count];

                        SweptCircle swc0 = new SweptCircle(spine, radius);
                        BoundingRect bounds0 = new BoundingRect(0, vmin, CriticalPositions[0], vmax);
                        swc0.SetBounds(bounds0);
                        SweptCircle swc1 = new SweptCircle(spine, radius);
                        BoundingRect bounds1 = new BoundingRect(criticalPositions[1], vmin, 1, vmax);
                        swc1.SetBounds(bounds1);
                        for (int i = 0; i < vValues.Count; i++)
                        {
                            double v = vValues[i];
                            {   // the case for the first half
                                List<double> uvalues = u0Values;
                                if (i >= cusp0Index && i <= cusp1Index)
                                {   // find the intersection point
                                    double uintsect;
                                    if (i == cusp0Index || i == cusp1Index) uintsect = CriticalPositions[0]; // here we know the u-value, intersection is unsafe here
                                    else
                                    {
                                        ICurve cv1 = FixedV(v, CriticalPositions[1], 1); // intersect this curve with the other half of the surface
                                        swc0.Intersect(cv1, bounds0, out GeoPoint[] ips0, out GeoPoint2D[] uvOnFaces0, out double[] uOnCurve3Ds0);
                                        if (ips0.Length == 1) // which it should be
                                        {
                                            uintsect = uvOnFaces0[0].x; // end here with u
                                        }
                                        else
                                        {   // should not happen!
                                            uintsect = CriticalPositions[0];
                                        }
                                    }
                                    uvalues = new List<double>(Enumerable.Range(0, u0Values.Count).Select(j => j * (uintsect) / (u0Values.Count - 1)));
                                }
                                for (int j = 0; j < uvalues.Count; j++)
                                {
                                    swc0.Derivation2At(new GeoPoint2D(uvalues[j], v), out GeoPoint location, out GeoVector ddu, out GeoVector ddv, out GeoVector duu, out GeoVector dvv, out GeoVector duv);
                                    points0[j, i] = location;
                                    du0[j, i] = ddu;
                                    dv0[j, i] = ddv;
                                    duv0[j, i] = duv;
                                }
                            }
                            {   // the case for the second half
                                List<double> uvalues = u1Values;
                                if (i >= cusp0Index && i <= cusp1Index)
                                {   // find the intersection point
                                    double uintsect;
                                    if (i == cusp0Index || i == cusp1Index) uintsect = CriticalPositions[1]; // here we know the u-value, intersection is unsafe here
                                    else
                                    {
                                        ICurve cv0 = FixedV(v, 0, CriticalPositions[0]); // intersect this curve with the other half of the surface
                                        swc1.Intersect(cv0, bounds1, out GeoPoint[] ips0, out GeoPoint2D[] uvOnFaces0, out double[] uOnCurve3Ds0);
                                        if (ips0.Length == 1) // which it should be
                                        {
                                            uintsect = uvOnFaces0[0].x; // end here with u
                                        }
                                        else
                                        {   // should not happen!
                                            uintsect = CriticalPositions[0];
                                        }
                                    }
                                    uvalues = new List<double>(Enumerable.Range(0, u1Values.Count).Select(j => uintsect + j * (1 - uintsect) / (u1Values.Count - 1)));
                                }
                                for (int j = 0; j < uvalues.Count; j++)
                                {
                                    swc1.Derivation2At(new GeoPoint2D(uvalues[j], v), out GeoPoint location, out GeoVector ddu, out GeoVector ddv, out GeoVector duu, out GeoVector dvv, out GeoVector duv);
                                    points1[j, i] = location;
                                    du1[j, i] = ddu;
                                    dv1[j, i] = ddv;
                                    duv1[j, i] = duv;
                                }
                            }
                        }
                        GeoObjectList dbgl = new GeoObjectList();
                        for (int k = 0; k < points1.GetLength(0); k++)
                        {
                            List<GeoPoint> p1 = new List<GeoPoint>();
                            for (int l = 0; l < points1.GetLength(1); l++)
                            {
                                p1.Add(points1[k, l]);
                            }
                            dbgl.Add(Polyline.FromPoints(p1.ToArray()));
                        }
                        // it would be better to have a NURBS interpolator, which respects the derivations. But I dont have one
                        NurbsSurface ns0 = new NurbsSurface(points0, 3, 3, false, false);
                        NurbsSurface ns1 = new NurbsSurface(points1, 3, 3, false, false);
                        //NurbsSurface ns00 = NurbsSurfaceInterpolator.BuildNurbsSurfaceOfDegree3(points0, du0, dv0, duv0);
                        //NurbsSurface ns11 = NurbsSurfaceInterpolator.BuildNurbsSurfaceOfDegree3(points1, du1, dv1, duv1);
                        GeoPoint2D cusp0uv = ns0.PositionOf(cusp0);
                        GeoPoint2D cusp1uv = ns0.PositionOf(cusp1);
                        ns0.GetNaturalBounds(out double left, out double right, out double bottom, out double top);
                        BoundingRect br = new BoundingRect(left, bottom, right, top);
                        // now this bounding rect should be 0 to 1. We could make a face with it directely, but on the right hand side we want to plit the line in parts outside the cusp and inside
                        double vstart = bottom;
                        double lo = Min(cusp0uv.y, cusp1uv.y);
                        double hi = Max(cusp0uv.y, cusp1uv.y);
                        List<ICurve2D> edges2d = new List<ICurve2D>();
                        if (bottom < lo && lo < top)
                        {
                            edges2d.Add(new Line2D(new GeoPoint2D(right, vstart), new GeoPoint2D(right, lo)));
                            vstart = lo;
                        }
                        if (hi < top)
                        {
                            edges2d.Add(new Line2D(new GeoPoint2D(right, vstart), new GeoPoint2D(right, hi)));
                            vstart = hi;
                        }
                        edges2d.Add(new Line2D(new GeoPoint2D(right, vstart), br.GetUpperRight()));
                        edges2d.Add(new Line2D(br.GetUpperRight(), br.GetUpperLeft()));
                        edges2d.Add(new Line2D(br.GetUpperLeft(), br.GetLowerLeft()));
                        edges2d.Add(new Line2D(br.GetLowerLeft(), br.GetLowerRight()));
                        Face fc0 = Face.MakeFace(ns0, new SimpleShape(new Border(edges2d.ToArray())));
                        // the same with the second part
                        cusp0uv = ns1.PositionOf(cusp0);
                        cusp1uv = ns1.PositionOf(cusp1);
                        ns1.GetNaturalBounds(out left, out right, out bottom, out top);
                        br = new BoundingRect(left, bottom, right, top);
                        // the left hand side of the rectangle: here we must create lines from top to bottom
                        vstart = top;
                        lo = Min(cusp0uv.y, cusp1uv.y);
                        hi = Max(cusp0uv.y, cusp1uv.y);
                        edges2d = new List<ICurve2D>();
                        if (bottom < hi && hi < top)
                        {
                            edges2d.Add(new Line2D(new GeoPoint2D(left, vstart), new GeoPoint2D(left, hi)));
                            vstart = hi;
                        }
                        if (lo > bottom)
                        {
                            edges2d.Add(new Line2D(new GeoPoint2D(left, vstart), new GeoPoint2D(left, lo)));
                            vstart = lo;
                        }
                        edges2d.Add(new Line2D(new GeoPoint2D(left, vstart), br.GetLowerLeft()));
                        edges2d.Add(new Line2D(br.GetLowerLeft(), br.GetLowerRight()));
                        edges2d.Add(new Line2D(br.GetLowerRight(), br.GetUpperRight()));
                        edges2d.Add(new Line2D(br.GetUpperRight(), br.GetUpperLeft()));
                        Face fc1 = Face.MakeFace(ns1, new SimpleShape(new Border(edges2d.ToArray())));
                        // the two faces connect at the self intersection of the SweptCircle: the connection is on up to three edges. The outer edges are tangential, the inner edge is a real non tangential intersection
                        return new Face[] { fc0, fc1 };
                    }
                }
            }
            return null;
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
