﻿using CADability.GeoObject;
using CADability;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.CompilerServices;
using CADability.Curve2D;
using CADability.Shapes;
using Wintellect.PowerCollections;
using MathNet.Numerics.RootFinding;
using MathNet.Numerics.LinearAlgebra;

namespace ShapeIt
{

    /// <summary>
    /// A reference to a edge. This is mainly used as a UserData: the Clone method does not clone the edge but returns this object, so you won't have an infinite loop
    /// when the userdata of a edge refers to itself.
    /// </summary>
    public class EdgeReference : ICloneable
    {
        public EdgeReference(Edge edge) { Edge = edge; }
        public Edge Edge { get; }
        public object Clone()
        {
            return this; // don't clone the edge, this would result in an infinite loop
        }
    }


    public static class EllipseDistanceSolver
    {
        /// <summary>
        /// Finds a point on the line G(t) = P0 + t*v, whose minimal distance to the ellipse
        /// E(θ) = (a*cos(θ), b*sin(θ), 0) equals a given value d.
        /// </summary>
        public static bool FindPointOnLineAtDistanceToEllipse(
            GeoPoint P0, GeoVector v,
            Ellipse elli,
            double d,
            out GeoPoint pointOnLine,
            out GeoPoint closestPointOnEllipse)
        {
            pointOnLine = default;
            closestPointOnEllipse = default;

            // Function F(θ) = distance between E(θ) and G(t(θ)) - d
            Func<double, double> F = theta =>
            {
                // Ellipse point
                GeoPoint E = elli.PointAt(theta);

                // Tangent vector of the ellipse at θ (treated as normal to plane)
                GeoVector tangent = elli.DirectionAt(theta);

                // Plane through E with tangent as normal
                Plane plane = new Plane(E, tangent);

                // Intersect line with tangent plane
                if (!plane.Intersect(P0, v, out GeoPoint G))
                    return double.NaN; // Degenerate case: line parallel to plane

                // Distance between ellipse point and line point
                return (G | E) - d;
            };

            // Search interval in θ (full ellipse)
            double thetaMin = 0;
            double thetaMax = 1;

            // Discretely sample to find a sign change (zero crossing)
            int samples = 36;
            while (thetaMax - thetaMin > 0.001)
            {
                double prevTheta = thetaMin;
                double prevValue = F(prevTheta);

                List<(double, double)> vals = new List<(double, double)>();
                vals.Add((prevTheta, prevValue));
                for (int i = 1; i <= samples; i++)
                {
                    double theta = thetaMin + i * (thetaMax - thetaMin) / samples;
                    double value = F(theta);

                    if (double.IsNaN(prevValue) || double.IsNaN(value))
                    {
                        prevTheta = theta;
                        prevValue = value;
                        continue;
                    }
                    vals.Add((theta, value));

                    if (prevValue * value < 0)
                    {
                        // Found sign change → root between prevTheta and theta
                        var result = Brent.FindRoot(F, prevTheta, theta, accuracy: 1e-10, maxIterations: 100);

                        // Evaluate result
                        double thetaRoot = result;
                        GeoPoint E = elli.PointAt(thetaRoot);
                        GeoVector tangent = elli.DirectionAt(thetaRoot);
                        Plane plane = new Plane(E, tangent);

                        if (!plane.Intersect(P0, v, out GeoPoint G))
                            return false;

                        pointOnLine = G;
                        closestPointOnEllipse = E;
                        return true;
                    }

                    prevTheta = theta;
                    prevValue = value;
                }
                int mini = -1;
                double minval = double.MaxValue;
                for (int i = 0; i < vals.Count; i++)
                {
                    if (Math.Abs(vals[i].Item2) < minval)
                    {
                        minval = vals[i].Item2;
                        mini = i;
                    }
                }
                thetaMin = vals[Math.Max(mini - 1, 0)].Item1;
                thetaMax = vals[Math.Min(mini + 1, vals.Count - 1)].Item1;
            }
            // No zero crossing found
            return false;
        }
    }

    internal static class CurveExtensions
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
            if (!curve.TryPointDeriv2At(u, out GeoPoint point, out GeoVector deriv1, out GeoVector deriv2))
            {
                throw new ArgumentException("Curve does not support second derivative at the given parameter.");
            }

            double deriv1Length = deriv1.Length;
            if (deriv1Length < 1e-12)
            {
                throw new ArgumentException("First derivative is too small to determine curvature.");
            }

            // Tangent vector
            GeoVector T = deriv1 / deriv1Length;

            // Normal component of second derivative
            GeoVector proj = (deriv2 * T) * T;
            GeoVector normalComponent = deriv2 - proj;

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

            return (center, (deriv1 ^ deriv2).Normalized, radius);
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
                pos = curve.ParameterToPosition(Math.PI/2.0);
                if (pos >= 0.0 && pos <= 1.0) minima.Add(pos);
                pos = curve.ParameterToPosition(3.0*Math.PI / 2.0);
                if (pos >= 0.0 && pos <= 1.0) minima.Add(pos);
            }
            else
            {
                // TODO
                double[] pp = curve.GetSavePositions();
            }
            return (minima.ToArray(),maxima.ToArray());
        }
    }
    internal static class ShellExtensions
    {
        public static int GetFaceDistances(this Shell shell, Face distanceFrom, GeoPoint touchingPoint, out List<Face> distanceTo, out List<double> distance, out List<GeoPoint> pointsFrom, out List<GeoPoint> pointsTo)
        {
            distanceTo = new List<Face>();
            distance = new List<double>();
            pointsFrom = new List<GeoPoint>();
            pointsTo = new List<GeoPoint>();
            foreach (Face face in shell.Faces)
            {
                if (face == distanceFrom) continue;
                if (Surfaces.ParallelDistance(distanceFrom.Surface, distanceFrom.Domain, face.Surface, face.Domain, touchingPoint, out GeoPoint2D uv1, out GeoPoint2D uv2))
                {
                    GeoPoint pFrom = distanceFrom.Surface.PointAt(uv1);
                    GeoPoint pTo = face.Surface.PointAt(uv2);
                    double dist = pFrom | pTo;
                    if (dist > Precision.eps)
                    {
                        distanceTo.Add(face);
                        distance.Add(dist);
                        pointsFrom.Add(pFrom);
                        pointsTo.Add(pTo);
                    }
                }
            }
            return distanceTo.Count;
        }
        public enum AdjacencyType
        {
            Unknown,
            Open,
            SameSurface,
            Tangent,
            Convex,
            Concave,
            Mixed
        }
        public static AdjacencyType Adjacency(this Edge edge)
        {
            if (edge.SecondaryFace != null)
            {
                if (edge.PrimaryFace.Surface.SameGeometry(edge.PrimaryFace.Domain, edge.SecondaryFace.Surface, edge.SecondaryFace.Domain, Precision.eps, out ModOp2D _)) return AdjacencyType.SameSurface;
                if (edge.IsTangentialEdge()) return AdjacencyType.Tangent;
                // there should be a test for "mixed"
                Vertex v1 = edge.StartVertex(edge.PrimaryFace);
                GeoPoint2D uvp = v1.GetPositionOnFace(edge.PrimaryFace);
                GeoPoint2D uvs = v1.GetPositionOnFace(edge.SecondaryFace);
                GeoVector curveDir;
                if (edge.Forward(edge.PrimaryFace)) curveDir = edge.Curve3D.StartDirection;
                else curveDir = -edge.Curve3D.EndDirection;
                double orientation = curveDir * (edge.PrimaryFace.Surface.GetNormal(uvp) ^ edge.SecondaryFace.Surface.GetNormal(uvs));
                if (orientation > 0) return AdjacencyType.Convex;
                else return AdjacencyType.Concave;
            }
            else
            {
                return AdjacencyType.Open;
            }
        }
        public static bool AllEdgesAreConvex(this Face face, bool reverse)
        {
            AdjacencyType toFollow = reverse ? AdjacencyType.Concave : AdjacencyType.Convex;
            foreach (Edge edge in face.AllEdges)
            {
                if (edge.Adjacency() != AdjacencyType.Tangent && edge.Adjacency() != toFollow) return false;
            }
            return true;
        }
        public static List<List<Face>> GetConvexParts(IEnumerable<Face> faces, bool reverse)
        {
            // HashSet<Face> faces = new HashSet<Face>(shell.Faces);
            var visited = new HashSet<Face>();
            var result = new List<List<Face>>();

            AdjacencyType toFollow = reverse ? AdjacencyType.Concave : AdjacencyType.Convex;

            foreach (var face in faces)
            {
                if (visited.Contains(face))
                    continue;

                // start a new region
                var region = new List<Face>();
                var queue = new Queue<Face>();
                queue.Enqueue(face);
                visited.Add(face);

                while (queue.Count > 0)
                {
                    var current = queue.Dequeue();
                    region.Add(current);

                    foreach (var edge in current.Edges)
                    {
                        // only respect convex or tangential connections
                        if (edge.Adjacency() != toFollow &&
                            edge.Adjacency() != AdjacencyType.Tangent)
                            continue;

                        Face neighbor = edge.OtherFace(current);

                        if (neighbor == null || visited.Contains(neighbor))
                            continue;

                        if (faces.Contains(neighbor))
                        {
                            queue.Enqueue(neighbor);
                            visited.Add(neighbor);
                        }
                    }
                }

                result.Add(region);
            }

            return result;
        }

        /// <summary>
        /// Finds the (u,v) on <paramref name="surface"/> that minimises
        /// the distance to <paramref name="target"/>.
        /// </summary>
        /// <param name="surface">Parametric surface f(u,v).</param>
        /// <param name="target">3-D point P.</param>
        /// <param name="startValue">Initial guess (u0,v0).</param>
        /// <param name="tol">Tolerance in world units.</param>
        /// <param name="maxIter">Maximum number of Newton steps.</param>
        public static GeoPoint2D PositionOf(
            this ISurface surface,
            GeoPoint target,
            GeoPoint2D startValue,
            double tol = 1e-8,
            int maxIter = 30)
        {
            var uv = new GeoPoint2D(startValue.x, startValue.y);

            for (int iter = 0; iter < maxIter; iter++)
            {
                // First- and second-order surface data
                surface.Derivation2At(
                    uv, out GeoPoint loc, out GeoVector du, out GeoVector dv,
                    out GeoVector duu, out GeoVector dvv, out GeoVector duv);

                GeoVector r = loc - target;               // residual f-P
                double resLen = r.Length;
                if (resLen < tol) return uv;              // converged

                // Gradient of g
                var g = Vector<double>.Build.DenseOfArray(new[]
                {
                r * du,                               // dot product
                r * dv
            });

                // Hessian of g
                var H = Matrix<double>.Build.DenseOfArray(new[,]
                {
                { du * du + r * duu,  du * dv + r * duv },
                { du * dv + r * duv,  dv * dv + r * dvv }
            });

                // Solve H Δ = -∇g  (fallback to damped GN if necessary)
                Vector<double> delta;
                try
                {
                    delta = H.Solve(-g);
                }
                catch (Exception)                   // singular Hessian
                {
                    // Levenberg-Marquardt fallback: (H + λI) Δ = -∇g
                    double lambda = 1e-4 * H.Diagonal().Maximum();
                    delta = (H + lambda * Matrix<double>.Build.DenseIdentity(2)).Solve(-g);
                }

                // Update parameters
                uv.x += delta[0];
                uv.y += delta[1];

                if (delta.L2Norm() < tol) return uv;      // small step → done
            }

            // If we get here we did not converge
            return GeoPoint2D.Invalid;
        }

        public static (double from, double upto) EllipticalPipeSelfIntersectionInterval(Ellipse ellipticalArc, double r)
        {
            double a = ellipticalArc.MajorRadius;
            double b = ellipticalArc.MinorRadius;
            double C = Math.Pow((a * b * r), (2.0 / 3.0));
            double sin2u = (C - b * b) / (a * a - b * b);
            double u0 = Math.Asin(Math.Sqrt(sin2u));
            double pos1 = (ellipticalArc as ICurve).ParameterToPosition(u0);
            double pos2 = (ellipticalArc as ICurve).ParameterToPosition(-u0);
            if ((pos1 >= 0 && pos1 <= 1) || (pos2 >= 0 && pos2 <= 1))
            {   // the elliptical arc contains the point at the major axis
                return (Math.Min(pos2, pos1), Math.Max(pos2, pos1));
            }
            double pos3 = (ellipticalArc as ICurve).ParameterToPosition(Math.PI + u0);
            double pos4 = (ellipticalArc as ICurve).ParameterToPosition(Math.PI - u0);
            if ((pos3 >= 0 && pos3 <= 1) || (pos4 >= 0 && pos4 <= 1))
            {   // the elliptical arc contains the point at the negative major axis
                return (Math.Min(pos4, pos3), Math.Max(pos4, pos3));
            }
            return (double.MaxValue, double.MaxValue);
        }

        /// <summary>
        /// On the elliptical arc there is a self intersection for the swept circle with radius <paramref name="r"/> at the returned parameter (radian) of the circle.
        /// The circle 0 position ist in the plane of the ellipse outside of the ellipse.
        /// </summary>
        /// <param name="ellipticalArc"></param>
        /// <param name="r"></param>
        /// <param name="earcPos"></param>
        /// <returns>Parameters on the circle of the self intersection</returns>
        public static (double, double) EllipticalPipeSelfIntersection(Ellipse ellipticalArc, double r, double earcPos)
        {
            ModOp toUnit = ellipticalArc.ToUnitCircle;
            double a = ellipticalArc.MajorRadius;
            double b = ellipticalArc.MinorRadius;
            Func<double, double> rho = (double u) =>
            {
                double su = Math.Sin(u);
                double cu = Math.Cos(u);
                double s = a * a * su * su + b * b * cu * cu;
                return Math.Pow(s, 1.5) / (a * b);
            };
            double C = Math.Pow((a * b * r), (2.0 / 3.0));
            double sin2u = (C - b * b) / (a * a - b * b);
            double earcPar = (ellipticalArc as ICurve).PositionToParameter(earcPos);
            double delta = Math.Acos(rho(earcPar) / r);
            double v = Math.PI - delta;
            return (Math.PI - delta, Math.PI + delta);
        }

        public static void EllipticalPipe(Ellipse ellipse, double r, List<GeoPoint> selfInt3, List<GeoPoint2D> selfInt2)
        {
            ModOp toUnit = ellipse.ToUnitCircle;
            double a = (toUnit * ellipse.MajorAxis).Length;
            double b = (toUnit * ellipse.MinorAxis).Length;
            Func<double, double> rho = (double u) =>
                {
                    double su = Math.Sin(u);
                    double cu = Math.Cos(u);
                    double s = a * a * su * su + b * b * cu * cu;
                    return Math.Pow(s, 1.5) / (a * b);
                };
            double C = Math.Pow((a * b * r), (2.0 / 3.0));
            double sin2u = (C - b * b) / (a * a - b * b);
            double u0 = Math.Asin(Math.Sqrt(sin2u));
            //double u1 = Math.PI - u0;
            //double u2 = Math.PI + u0;
            double u1 = -u0;
            double u2 = +u0;
            int n = 100;
            GeoPoint2D[] p2d = new GeoPoint2D[n];
            GeoPoint[] p3d = new GeoPoint[n];
            for (int i = 0; i < n; i++)
            {
                double u = u1 + i * (u2 - u1) / (n - 1);
                double delta = Math.Acos(rho(u) / r);
                double v = Math.PI - delta;
                if (u < 0) u += 2.0 * Math.PI;
                if (u > 2 * Math.PI) u -= 2.0 * Math.PI;
                double eu = (ellipse as ICurve).ParameterToPosition(u);
                if (eu < -0.5) eu = (ellipse as ICurve).ParameterToPosition(u + 2 * Math.PI);
                p2d[i] = new GeoPoint2D(v, eu);
                GeoPoint ep = ellipse.PointAt(eu);
                GeoVector edir = ellipse.DirectionAt(eu);
                Plane cPlane = new Plane(ep, edir ^ ellipse.Plane.Normal, ellipse.Plane.Normal);
                GeoPoint2D cp = new GeoPoint2D(r * Math.Cos(v), r * Math.Sin(v));
                p3d[i] = cPlane.ToGlobal(cp);
            }
            selfInt2.AddRange(p2d);
            selfInt3.AddRange(p3d);
        }
        /// <summary>
        /// Make a fillet like face, which fills the gap between two offset faces at the provided <paramref name="axis"/>
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="radius"></param>
        /// <param name="forward"></param>
        /// <param name="backward"></param>
        /// <returns></returns>
        public static Face MakeOffsetFillet(Edge axis, double radius, Edge forward, Edge backward, bool dontUseForward, bool dontUseBackward)
        {
            ISurface surface = null;
            GeoVector toInside;
            if (axis.Curve3D is Line line)
            {
                if (axis.Forward(axis.PrimaryFace))
                {
                    Vertex v = axis.StartVertex(axis.PrimaryFace);
                    GeoPoint2D uvp = v.GetPositionOnFace(axis.PrimaryFace);
                    GeoPoint2D uvs = v.GetPositionOnFace(axis.SecondaryFace);
                    toInside = -(axis.PrimaryFace.Surface.GetNormal(uvp).Normalized + axis.SecondaryFace.Surface.GetNormal(uvs).Normalized); // points away from the edge
                    GeoVector dir = line.StartDirection;
                    GeoVector majorAxis = radius * toInside.Normalized;
                    GeoVector minorAxis = radius * (dir ^ majorAxis).Normalized;
                    surface = new CylindricalSurface(v.Position, majorAxis, minorAxis, dir);
                }
                else
                {
                    Vertex v = axis.EndVertex(axis.PrimaryFace);
                    GeoPoint2D uvp = v.GetPositionOnFace(axis.PrimaryFace);
                    GeoPoint2D uvs = v.GetPositionOnFace(axis.SecondaryFace);
                    toInside = -(axis.PrimaryFace.Surface.GetNormal(uvp).Normalized + axis.SecondaryFace.Surface.GetNormal(uvs).Normalized); // points away from the edge
                    GeoVector dir = -line.EndDirection;
                    GeoVector majorAxis = radius * toInside.Normalized;
                    GeoVector minorAxis = radius * (dir ^ majorAxis).Normalized;
                    surface = new CylindricalSurface(v.Position, majorAxis, minorAxis, dir);
                }
            }
            else if (axis.Curve3D is Ellipse bigCircle && bigCircle.IsCircle)
            {
                if (axis.Forward(axis.PrimaryFace))
                {
                    Vertex v = axis.StartVertex(axis.PrimaryFace);
                    GeoVector dirx = (v.Position - bigCircle.Center).Normalized;
                    GeoVector dirz = (bigCircle.SweepParameter > 0) ? bigCircle.Plane.Normal : -bigCircle.Plane.Normal;
                    GeoVector diry = dirz ^ dirx; // still to test
                    surface = new ToroidalSurface(bigCircle.Center, dirx, diry, dirz, bigCircle.MajorRadius, radius);
                }
                else
                {
                    Vertex v = axis.EndVertex(axis.PrimaryFace);
                    GeoVector dirx = -(v.Position - bigCircle.Center).Normalized;
                    GeoVector dirz = (bigCircle.SweepParameter > 0) ? bigCircle.Plane.Normal : -bigCircle.Plane.Normal;
                    GeoVector diry = dirz ^ dirx; // still to test
                    surface = new ToroidalSurface(bigCircle.Center, dirx, diry, dirz, bigCircle.MajorRadius, radius);
                }
            }
            else
            {
                // make a NURBS surface defined by a certain number of circles
                // the curvature of the axis plays a critical role:
                // when it is always greater than the radius: we can simply make a NURBS from the circles
                // when it is both greater and smaller than the radius: there is a folding of the surface
                // when it is always smaller than the radius: we dont't have to produce a fillet here
                if (axis.Curve3D.GetPlanarState() == PlanarState.Planar) { }

                int n = 20; // number of intermediate points, need some adaptive algorithm
                List<Ellipse> throughEllipses = new List<Ellipse>(n);
                // we need only a half pipe at maximum: (not sure, wether this is correct for non planar curves)
                (GeoPoint center, GeoVector normal, double radius) middleCurvature = axis.Curve3D.CurvatureAt(0.5);
                GeoPoint middlePoint = axis.Curve3D.PointAt(0.5);
                GeoPoint2D uvp = axis.PrimaryFace.Surface.PositionOf(middlePoint);
                GeoPoint2D uvs = axis.SecondaryFace.Surface.PositionOf(middlePoint);
                GeoVector toOutside = axis.PrimaryFace.Surface.GetNormal(uvp).Normalized + axis.SecondaryFace.Surface.GetNormal(uvs).Normalized;
                Plane middlePlane = new Plane(middlePoint, axis.Curve3D.DirectionAt(0.5) ^ middleCurvature.normal, middleCurvature.normal);
                Ellipse middleElli = Ellipse.Construct();
                middleElli.SetCirclePlaneCenterRadius(middlePlane, middlePoint, Math.Abs(radius));
                double mpos = middleElli.PositionOf(middlePoint + radius * toOutside.Normalized);
                middleElli.Trim(mpos - 0.25, mpos + 0.25);
                double startParameter = middleElli.StartParameter;
                double sweepParameter = middleElli.SweepParameter;
                double[] knots = new double[n];
                for (int i = 0; i < n; i++)
                {
                    double par = (double)i / (n - 1);
                    knots[i] = par;
                    GeoVector dir = axis.Curve3D.DirectionAt(par);
                    GeoPoint pos = axis.Curve3D.PointAt(par);
                    (GeoPoint center, GeoVector normal, double radius) curvature = axis.Curve3D.CurvatureAt(0.0);
                    Plane epln = new Plane(pos, dir ^ curvature.normal, curvature.normal);
                    // the 0°-point of the circle is to the outside of the curvature, so self intersection can only occur between 90° and 270°
                    Ellipse elli = Ellipse.Construct();
                    elli.SetCirclePlaneCenterRadius(epln, pos, Math.Abs(radius));
                    elli.StartParameter = startParameter;
                    elli.SweepParameter = sweepParameter;
                    throughEllipses.Add(elli);
                }
                surface = new NurbsSurface(throughEllipses.ToArray(), knots);
                // the u-parameter of the NurbsSurface is the circular arc. It spans 180°, so the u parameter goes from 0 to 0.5 (the whole circle is from 0 to 1)
                // the v-parameter goes from 0 to 1. It is not linear synchronous to the axis.Curve3d parameter, but to the length of the segments
                if (axis.Curve3D is Ellipse ellipse)
                {
                    (GeoPoint center, GeoVector normal, double radius) c1 = ellipse.CurvatureAt(0.0);
                    (GeoPoint center, GeoVector normal, double radius) c2 = ellipse.CurvatureAt(1.0);
                    if (Math.Sign(c1.radius - Math.Abs(radius)) != Math.Sign(c2.radius - Math.Abs(radius)))
                    {
                        (double vmin, double vmax) vInterval = EllipticalPipeSelfIntersectionInterval(ellipse, Math.Abs(radius));
                        if (vInterval.vmin < 0 && vInterval.vmax > 0)
                        {   // self intersection at the beginning

                        }
                        if (vInterval.vmin < 1 && vInterval.vmax > 1)
                        {   // self intersection at the end
                            (double v1, double v2) = EllipticalPipeSelfIntersection(ellipse, Math.Abs(radius), 1.0);
                        }
                    }
                    //    List<GeoPoint2D> uvCurve = new List<GeoPoint2D>();
                    //List<GeoPoint2D> uvCurveOrg = new List<GeoPoint2D>();
                    //for (int i = 0; i < selfInt3.Count; i++)
                    //{
                    //    GeoPoint pdbg = surface.PointAt(new GeoPoint2D(0.5, 0.5));
                    //    GeoPoint2D uvstart = new GeoPoint2D((middleElli as ICurve).ParameterToPosition(selfInt2[i].x) / 2, selfInt2[i].y);
                    //    pdbg = surface.PointAt(uvstart);
                    //    uvCurveOrg.Add(uvstart);
                    //    GeoPoint2D uv = surface.PositionOf(selfInt3[i], uvstart);
                    //    if (uv.IsValid) uvCurve.Add(uv); else uvCurve.Add(uvstart);
                    //}
                    //BoundingRect outline = new BoundingRect(0, 0, 0.5, 1.0);

                }
            }
            if (surface is ToroidalSurface toroidalSurface)
            {
                BoundingRect domain = new BoundingRect(surface.PositionOf(forward.Curve3D.PointAt(0.5))); // toroidal fillets may sweep over 180°. so it would be ambiguous which part to use
                GeoPoint2D uv = surface.PositionOf(forward.Vertex2.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);
                uv = surface.PositionOf(forward.Vertex1.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);
                uv = surface.PositionOf(backward.Vertex1.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);
                uv = surface.PositionOf(backward.Vertex2.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);

                ICurve2D c1 = surface.GetProjectedCurve(forward.Curve3D, Precision.eps);
                ICurve2D c3 = surface.GetProjectedCurve(backward.Curve3D, Precision.eps);
                SurfaceHelper.AdjustPeriodic(toroidalSurface, domain, c1);
                SurfaceHelper.AdjustPeriodic(toroidalSurface, domain, c3);
                if (forward.Forward(forward.PrimaryFace)) c1.Reverse(); // then it is backward on secondary face
                if (backward.Forward(backward.PrimaryFace)) c3.Reverse();
                ICurve2D c2 = new Line2D(c1.EndPoint, c3.StartPoint);
                ICurve2D c4 = new Line2D(c3.EndPoint, c1.StartPoint);
                Border bdr = new Border(new ICurve2D[] { c1, c2, c3, c4 });
                double[] vs = toroidalSurface.GetVSingularities(); // this are the poles of the torus, when minor radius > major radius
                double vToSplit = double.MaxValue;
                double vPeriod = toroidalSurface.VPeriod; // is Math.PI*2.0
                for (int i = 0; i < vs.Length; i++)
                {
                    while (vs[i] > domain.Top) vs[i] -= vPeriod;
                    while (vs[i] < domain.Bottom) vs[i] += vPeriod;
                    if (vs[i] > domain.Bottom && vs[i] < domain.Top) vToSplit = vs[i];
                }
                if (vToSplit < double.MaxValue)
                {
                    // one of the two provided edges must be invalid

                    if (dontUseBackward)
                    {
                        c2.EndPoint = new GeoPoint2D(c1.EndPoint.x, vToSplit);
                        c4.StartPoint = new GeoPoint2D(c1.StartPoint.x, vToSplit);
                        bdr = new Border(new ICurve2D[] { c1, c2, new Line2D(c2.EndPoint, c4.StartPoint), c4 });
                    }
                    else
                    {
                        System.Diagnostics.Debug.Assert(dontUseForward); // this must be the case here
                        c2.StartPoint = new GeoPoint2D(c3.StartPoint.x, vToSplit);
                        c4.EndPoint = new GeoPoint2D(c3.EndPoint.x, vToSplit);
                        bdr = new Border(new ICurve2D[] { c2, c3, c4, new Line2D(c4.EndPoint, c2.StartPoint) });
                    }
                    Face res = Face.MakeFace(toroidalSurface, new SimpleShape(bdr));
                    if (!dontUseForward) res.UseEdge(forward);
                    if (!dontUseBackward) res.UseEdge(backward);
                    return res;
                }
                else
                {
                    Face res = Face.MakeFace(toroidalSurface, new SimpleShape(bdr));
                    res.UseEdge(forward);
                    res.UseEdge(backward);
                    return res;
                }
            }
            else if (surface != null)
            {
                BoundingRect domain = new BoundingRect(surface.PositionOf(forward.Curve3D.PointAt(0.5))); // toroidal fillets may sweep over 180°. so it would be ambiguous which part to use
                GeoPoint2D uv = surface.PositionOf(forward.Vertex2.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);
                uv = surface.PositionOf(forward.Vertex1.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);
                uv = surface.PositionOf(backward.Vertex1.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);
                uv = surface.PositionOf(backward.Vertex2.Position);
                SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                domain.MinMax(uv);

                // we need 4 edges to make this face. Two edges are already provided as parameters, the other two edges are arcs at the start- and endpoint of the axis
                // maybe the two provided edges meet in a single point, then the ellipses are null and we only have 3 edges for the face
                // is there a case with both ellipses==null?
                GeoPoint center = axis.Vertex1.Position; // this is the startpoint of axis.Curve3D
                GeoVector toOutside = (axis.PrimaryFace.Surface.GetNormal(axis.Vertex1.GetPositionOnFace(axis.PrimaryFace)).Normalized +
                             axis.SecondaryFace.Surface.GetNormal(axis.Vertex1.GetPositionOnFace(axis.SecondaryFace)).Normalized);
                Plane pln = new Plane(center, axis.Curve3D.StartDirection);
                Ellipse elli1 = null;
                double d1 = (center | forward.Vertex1.Position) + (center | backward.Vertex2.Position);
                double d2 = (center | forward.Vertex2.Position) + (center | backward.Vertex1.Position);
                Vertex v1 = d1 < d2 ? forward.Vertex1 : forward.Vertex2;
                Vertex v2 = d1 < d2 ? backward.Vertex2 : backward.Vertex1;
                if (!Precision.IsEqual(v1.Position, v2.Position))
                {
                    elli1 = Ellipse.Construct();
                    elli1.SetArc3Points(v1.Position, center + radius * toOutside.Normalized, v2.Position, pln);
                    if (Math.Abs(elli1.SweepParameter) > Math.PI) elli1.SetArc3Points(v1.Position, center - radius * toOutside.Normalized, v2.Position, pln);
                }

                center = axis.Vertex2.Position; // this is the endpoint of axis.Curve3D
                toOutside = (axis.PrimaryFace.Surface.GetNormal(axis.Vertex2.GetPositionOnFace(axis.PrimaryFace)).Normalized +
                             axis.SecondaryFace.Surface.GetNormal(axis.Vertex2.GetPositionOnFace(axis.SecondaryFace)).Normalized);
                pln = new Plane(center, axis.Curve3D.EndDirection);
                Ellipse elli2 = null;
                v1 = forward.Vertex1 == v1 ? forward.Vertex2 : forward.Vertex1;
                v2 = backward.Vertex1 == v2 ? backward.Vertex2 : backward.Vertex1;
                if (!Precision.IsEqual(v1.Position, v2.Position))
                {
                    elli2 = Ellipse.Construct();
                    elli2.SetArc3Points(v1.Position, center + radius * toOutside.Normalized, v2.Position, pln);
                    if (Math.Abs(elli2.SweepParameter) > Math.PI) elli2.SetArc3Points(v1.Position, center - radius * toOutside.Normalized, v2.Position, pln);
                }

                List<ICurve2D> curve2Ds = new List<ICurve2D>();

                curve2Ds.Add(surface.GetProjectedCurve(forward.Curve3D, Precision.eps));
                if (elli1 != null) curve2Ds.Add(surface.GetProjectedCurve(elli1, Precision.eps));
                curve2Ds.Add(surface.GetProjectedCurve(backward.Curve3D, Precision.eps));
                if (elli2 != null) curve2Ds.Add(surface.GetProjectedCurve(elli2, Precision.eps));
                for (int i = 0; i < curve2Ds.Count; ++i) SurfaceHelper.AdjustPeriodic(surface, domain, curve2Ds[i]);
                SimpleShape ss = new SimpleShape(Border.FromUnorientedList(curve2Ds.ToArray(), true));
                Face res = Face.MakeFace(surface, ss);
                res.UseEdge(forward);
                res.UseEdge(backward);
                ICurve2D[] selfIntersection = surface.GetSelfIntersections(res.Domain);

                return res;
            }
            return null;
        }
        static List<Face> ConnectedList(Face startWith, HashSet<Face> visited = null, List<Face> result = null)
        {
            if (visited == null)
            {
                visited = new HashSet<Face>();
                result = new List<Face>();
            }
            result.Add(startWith);
            visited.Add(startWith);
            foreach (Edge edg in startWith.AllEdges)
            {
                Face other = edg.OtherFace(startWith);
                if (visited.Contains(other)) continue;
                visited.Add(other);
                ConnectedList(other, visited, result);
            }
            return result;
        }
        public static Shell[] GetOffsetNew(this Shell shell, double offset)
        {
            Dictionary<(Face, Edge), Edge> faceEdgeToParallelEdge = new Dictionary<(Face, Edge), Edge>(); // the parallel edges to the original edges, also depend on the face
            Dictionary<Vertex, List<Edge>> vertexToArcs = new Dictionary<Vertex, List<Edge>>(); // for each vertex there are the sides of the wedges, which build spherical wedges
            Dictionary<Face, Face> faceToOffsetFace = new Dictionary<Face, Face>(); // for each face of the original shell we have a face with the required offset here
            Dictionary<Edge, Face> edgeToFillet = new Dictionary<Edge, Face>(); // for each convex edge we create a "fillet" face
            HashSet<Face> inverseFaces = new HashSet<Face>();
            List<Face> sphercalWegdes = new List<Face>(); // the sphreical faces fill the gaps between the fillets
                                                          // for the brep operation we need edge-face pairs, which should not be tested for intersection, because they connect adjacent parts like the offset faces with the fillets
                                                          // or the fillets with the spherical faces. Since each breop operation creates new faces and edges, we attach this information to the user data of the original parts
                                                          // and retrieve them after the brep operation is done.
            HashSet<(Edge, Face)> dontIntersect = new HashSet<(Edge, Face)>(); // NOT USED ANY MORE, these pairs will be connected and there is no need to calculate the intersection
                                                                               // set UserData with the original face and edge references to 
            foreach (Face face in shell.Faces)
            {   // makeparallel faces with the provided offset
                ISurface offsetSurface = face.Surface.GetOffsetSurface(offset);
                if (offsetSurface == null) continue; // a sphere, cylinder or torus shrinking to 0
                GeoPoint2D cnt = face.Domain.GetCenter();
                // if the orentation is reversed (e.g. a cylinder will have a negativ radius) or the surface disappears, don't use it
                if (offsetSurface != null) // 
                {
                    List<ICurve2D> outline = new List<ICurve2D>();
                    foreach (Edge edge in face.OutlineEdges)
                    {
                        if (offsetSurface is ConicalSurface) // or any other surface, where the u/v system of the offset differs from the u/v system of the original (which are those?)
                        {
                            ICurve2D c2d = offsetSurface.GetProjectedCurve(edge.Curve3D, Precision.eps);
                            // what about orientation?
                            if (!edge.Forward(face)) c2d.Reverse(); // not tested!
                            if (c2d != null)
                            {
                                SurfaceHelper.AdjustPeriodic(offsetSurface, face.Domain, c2d);
                                outline.Add(c2d);
                                c2d.UserData.Add("CADability.CurveToEdge", edge);
                            }
                        }
                        else
                        {   // surfaces and offset surfaces usually have the same u/v system, i.e. 2d curves on both are parallel with the distance "offset"
                            ICurve2D c2d = edge.Curve2D(face).Clone();
                            if (c2d is InterpolatedDualSurfaceCurve.ProjectedCurve pc) c2d = pc.ToBSpline(0.0);
                            if (c2d is Path2D)
                            {
                                c2d = face.Surface.GetProjectedCurve(edge.Curve3D, 0.0); // sine curve is not maintained (14.6.25) but was converted to Path2D
                                if (!edge.Forward(face)) c2d.Reverse();
                            }
                            outline.Add(c2d);
                            c2d.UserData.Add("CADability.CurveToEdge", edge);
                        }
                    }
                    Border outlineBorder = new Border(outline.ToArray(), true);
                    List<Border> holes = new List<Border>();
                    for (int i = 0; i < face.HoleCount; i++)
                    {
                        List<ICurve2D> hole = new List<ICurve2D>();
                        foreach (Edge edge in face.HoleEdges(i))
                        {
                            if (offsetSurface is ConicalSurface) // or any other surface, where the u/v system of the offset differs from the u/v system of the original (which are those?)
                            {
                                ICurve2D c2d = offsetSurface.GetProjectedCurve(edge.Curve3D, Precision.eps);
                                if (c2d != null)
                                {
                                    SurfaceHelper.AdjustPeriodic(offsetSurface, face.Domain, c2d);
                                    hole.Add(c2d);
                                    c2d.UserData.Add("CADability.CurveToEdge", edge);
                                }
                            }
                            else
                            {   // surfaces and offset surfaces usually have the same u/v system, i.e. 2d curves on both are parallel with the distance "offset"
                                ICurve2D c2d = edge.Curve2D(face).Clone();
                                hole.Add(c2d);
                                c2d.UserData.Add("CADability.CurveToEdge", edge);
                            }
                        }
                        Border holeBorder = new Border(hole.ToArray(), true);
                        holes.Add(holeBorder);
                    }
                    SimpleShape ss = new SimpleShape(outlineBorder, holes.ToArray());
                    Face offsetFace = Face.MakeFace(offsetSurface, ss);
                    offsetFace.UserData.Add("ShapeIt.OriginalFace", new FaceReference(offsetFace));
#if DEBUG
                    DebuggerContainer dc = new DebuggerContainer();
                    dc.Add(face);
                    dc.Add(offsetFace);
                    foreach (Vertex vtx in face.Vertices)
                    {
                        Line l = Line.TwoPoints(vtx.Position, vtx.Position + offset * face.Surface.GetNormal(vtx.GetPositionOnFace(face)).Normalized);
                        dc.Add(l);
                    }
#endif

                    // we need a reference from edges of the original face to the edges of the paralell face.
                    foreach (Edge edg in offsetFace.Edges)
                    {
                        Edge originalEdge = edg.Curve2D(offsetFace).UserData["CADability.CurveToEdge"] as Edge;
                        if (originalEdge != null) faceEdgeToParallelEdge[(face, originalEdge)] = edg;
                        edg.Curve2D(offsetFace).UserData.Remove("CADability.CurveToEdge"); // no longer needed
                        if (edg.Curve3D is IGeoObject go)
                        {
                            go.UserData.Add("ShapeIt.OriginalEdge", new EdgeReference(edg));
                        }
                    }
                    faceToOffsetFace[face] = offsetFace;
                    if (offsetSurface.GetNormal(cnt) * face.Surface.GetNormal(cnt) < 0)
                    {   // this face is reversed, e.g. a cylinder, which now has negative radius
                        // we still need to create it, because we need the edges for the fillets, but we do not use it in the result
                        inverseFaces.Add(offsetFace);
                    }
                }
            }

            foreach (Edge sedge in shell.Edges)
            {
                AdjacencyType toCheckFor = offset > 0 ? AdjacencyType.Convex : AdjacencyType.Concave;
                if (sedge.Adjacency() == toCheckFor && faceEdgeToParallelEdge.TryGetValue((sedge.PrimaryFace, sedge), out Edge e1) && faceEdgeToParallelEdge.TryGetValue((sedge.SecondaryFace, sedge), out Edge e2))
                {
                    Face fillet = MakeOffsetFillet(sedge, offset, e1, e2, inverseFaces.Contains(e1.PrimaryFace), inverseFaces.Contains(e2.PrimaryFace));
                    if (fillet != null)
                    {
                        fillet.UserData.Add("ShapeIt.OriginalFace", new FaceReference(fillet));
                        dontIntersect.Add((e1, fillet));
                        dontIntersect.Add((e2, fillet));
                        edgeToFillet[sedge] = fillet;
                        Face primaryOffset = faceToOffsetFace[sedge.PrimaryFace];
                        Face secondaryOffset = faceToOffsetFace[sedge.SecondaryFace]; // must both exist!
                                                                                      // look for the arcs in the fillet and add them to vertexToArcs
                        foreach (Edge edg in fillet.AllEdges)
                        {
                            if (edg.Curve3D is IGeoObject go)
                            {
                                go.UserData.Add("ShapeIt.OriginalEdge", new EdgeReference(edg));
                            }
                            if (edg.Curve3D is Ellipse elli && elli.IsCircle && Math.Abs(elli.Radius - Math.Abs(offset)) < Precision.eps)
                            {
                                if (Precision.IsEqual(elli.Center, sedge.Vertex1.Position))
                                {
                                    if (!vertexToArcs.TryGetValue(sedge.Vertex1, out List<Edge> arcs))
                                    {
                                        vertexToArcs[sedge.Vertex1] = arcs = new List<Edge>();
                                    }
                                    arcs.Add(edg);
                                }
                                if (Precision.IsEqual(elli.Center, sedge.Vertex2.Position))
                                {
                                    if (!vertexToArcs.TryGetValue(sedge.Vertex2, out List<Edge> arcs))
                                    {
                                        vertexToArcs[sedge.Vertex2] = arcs = new List<Edge>();
                                    }
                                    arcs.Add(edg);
                                }
                            }
                            if (Precision.IsEqual(edg.Vertex1.Position, e1.Vertex2.Position) && Precision.IsEqual(edg.Vertex2.Position, e1.Vertex1.Position)) // edges must be reverse oriented, i.e. we can use Vertex1, Vertex2
                            {
                                dontIntersect.Add((edg, primaryOffset));
                            }
                            if (Precision.IsEqual(edg.Vertex1.Position, e2.Vertex2.Position) && Precision.IsEqual(edg.Vertex2.Position, e2.Vertex1.Position))
                            {
                                dontIntersect.Add((edg, secondaryOffset));
                            }
                        }
                    }
                }
            }
            //#if DEBUG
            //            foreach ((Edge, Face) item in dontIntersect)
            //            {
            //                DebuggerContainer dc = new DebuggerContainer();
            //                dc.Add(item.Item1.Curve3D as IGeoObject);
            //                dc.Add(item.Item2);
            //                System.Diagnostics.Trace.WriteLine(item.Item1.GetHashCode().ToString() + " - - " + item.Item2.GetHashCode().ToString());
            //            }
            //#endif

            foreach (Vertex vtx in shell.Vertices)
            {
                if (!vertexToArcs.TryGetValue(vtx, out List<Edge> arcs)) continue;
                for (int i = 0; i < arcs.Count - 1; i++)
                {
                    for (int j = i + 1; j < arcs.Count; j++)
                    {
                        if (Curves.GetCommonPlane(new ICurve[] { arcs[i].Curve3D, arcs[j].Curve3D }, out Plane _))
                        {   // two tangential edges in a vertex create the same arc edge and should be connected
                            dontIntersect.Add((arcs[i], arcs[j].PrimaryFace));
                            dontIntersect.Add((arcs[j], arcs[i].PrimaryFace));
                        }
                    }
                }
                if (arcs.Count < 3) continue; // this is not a valid spherical face
                GeoVector toOutside = GeoVector.NullVector;
                foreach (Face face in vtx.Faces)
                {
                    GeoVector n = face.Surface.GetNormal(vtx.GetPositionOnFace(face));
                    toOutside += n.Normalized;
                }
                toOutside.ArbitraryNormals(out GeoVector dirx, out GeoVector diry);
                SphericalSurface sphericalSurface = new SphericalSurface(vtx.Position, offset * toOutside.Normalized, offset * dirx.Normalized, offset * diry.Normalized);
                List<ICurve2D> curvesOnSphere = new List<ICurve2D>();
                BoundingRect domain = BoundingRect.EmptyBoundingRect;
                for (int i = 0; i < arcs.Count; i++)
                {
                    ICurve2D c2d = sphericalSurface.GetProjectedCurve(arcs[i].Curve3D, Precision.eps);
                    if (domain.IsEmpty()) domain.MinMax(c2d.GetExtent());
                    else
                    {
                        SurfaceHelper.AdjustPeriodic(sphericalSurface, domain, c2d);
                        domain.MinMax(c2d.GetExtent());
                    }
                    curvesOnSphere.Add(c2d);
                }
                Border bdr = Border.FromUnorientedList(curvesOnSphere.ToArray(), true); // we would need to sort the curves when more than 3
                Face sphere = Face.MakeFace(sphericalSurface, new SimpleShape(bdr));
                for (int i = 0; i < arcs.Count; i++) sphere.UseEdge(arcs[i]);

                //sphere.ReverseOrientation();
                sphercalWegdes.Add(sphere);
                foreach (Edge edg in sphere.Edges)
                {
                    for (int i = 0; i < arcs.Count; i++)
                    {
                        if (Curves.GetCommonPlane(edg.Curve3D, arcs[i].Curve3D, out Plane _))
                        {
                            dontIntersect.Add((edg, arcs[i].PrimaryFace));
                            dontIntersect.Add((arcs[i], sphere));
                        }
                    }
                }
            }
            HashSet<Face> faces = new HashSet<Face>(faceToOffsetFace.Values); // the raw offset faces
            faces.ExceptWith(inverseFaces); // these faces will not appear in the result
            faces.UnionWith(edgeToFillet.Values); // the fillets on the edges
            faces.UnionWith(sphercalWegdes); // the spherical faces on the vertices
                                             // faces contains all the faces for the offset shell, the edges are properly connected but some parts are standing out
            Shell.ConnectFaces(faces.ToArray(), Precision.eps);
            BRepOperation bo = new BRepOperation(faces);
            Shell[] res = bo.Result();
            return res;
        }
    }
}
