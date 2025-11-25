using CADability;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.RootFinding;
using netDxf.Tables;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms.VisualStyles;
using Wintellect.PowerCollections;

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
                            ICurve dbg = offsetSurface.Make3dCurve(c2d);
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
        public static Shell RoundEdgesOld(this Shell shell, IEnumerable<Edge> edges, double radius)
        {
            if (radius < 0) radius = -radius; // radius always >0
            List<Shell> shellsToSubtract = new List<Shell>();
            List<Shell> shellsToAdd = new List<Shell>();
            Dictionary<Face, Edge> tangentialEdges = new Dictionary<Face, Edge>(); // edges of the fillet which are tangential to a face of the original shell
            bool isConvex = false;
            foreach (Edge edgeToRound in edges)
            {
                double dist = 0.0;
                switch (Adjacency(edgeToRound))
                {
                    case AdjacencyType.Convex:
                        isConvex = true;
                        dist = -radius;
                        break; // the fillet will be subtracted
                    case AdjacencyType.Concave:
                        isConvex = false;
                        dist = radius;
                        break; // the fillet will be added
                    default: continue; // tangential or mixed: not possible to round
                }
                // for documentation we assume the following:
                // the edge edgeToRound is going from left to right. We call one of the faces top, the other bottom.
                // we create a body here which we call fillet. The fillet may be removed from the shell or added to the shell depending on whether the edge is convex or concav
                // the fillet is bounded by 5 surfaces/faces: the swept circle, the bottom and top surface (part of bottom and top faces) and the two lids on the left and right side.
                ISurface topSurface = edgeToRound.PrimaryFace.Surface; // for previty
                ISurface bottomSurface = edgeToRound.SecondaryFace.Surface;
                ICurve leadingEdge = edgeToRound.Curve3D.Clone();
                if (!edgeToRound.Forward(edgeToRound.PrimaryFace)) leadingEdge.Reverse(); // always forward on topSurface
                // we are looking for the three vertices on each side of the "negative fillet" (the part we want to remove from the shell
                // this "negative fillet" is bounded by the two offset surfaces, the fillet surface (swept circle) and the two sides. For the sides we can either
                // use the third face of the vertex of the edge e (if there is exactely a third face - which is in most cases) or a plane perpendicular ti the start or endpoint of the edge
                ISurface topOffset = topSurface.GetOffsetSurface(dist);
                ISurface bottomOffset = bottomSurface.GetOffsetSurface(dist);
                if (topOffset == null || bottomOffset == null) continue; // e.g. a sphere shrinking to a point
                topOffset.SetBounds(edgeToRound.PrimaryFace.Domain);
                bottomOffset.SetBounds(edgeToRound.SecondaryFace.Domain);
                PlaneSurface leftPlane = new PlaneSurface(new Plane(leadingEdge.StartPoint, -leadingEdge.StartDirection));
                PlaneSurface rightPlane = new PlaneSurface(new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection));
                GeoPoint filletAxisLeft = leadingEdge.StartPoint; // a first guess for the intersection, typically a good start
                GeoPoint filletAxisRight = leadingEdge.EndPoint; // the start and endpoint of the axis (spine) of the swept circle
                BoundingRect plnBounds = new BoundingRect(GeoPoint2D.Origin, radius, radius);
                BoundingRect topDomain = edgeToRound.PrimaryFace.Domain; // the domains of the top and bottom surfaces may be a little bit bigger than the original domains
                BoundingRect bottomDomain = edgeToRound.SecondaryFace.Domain;
                if (!CADability.GeoObject.Surfaces.IntersectThreeSurfaces(topOffset, edgeToRound.PrimaryFace.Domain, bottomOffset, edgeToRound.SecondaryFace.Domain, leftPlane, plnBounds,
                    ref filletAxisLeft, out GeoPoint2D uv11, out GeoPoint2D uv12, out GeoPoint2D uv13)) return null;
                SurfaceHelper.AdjustPeriodic(topSurface, topDomain, ref uv11);
                SurfaceHelper.AdjustPeriodic(bottomSurface, bottomDomain, ref uv12);
                topDomain.MinMax(uv11);
                bottomDomain.MinMax(uv12);
                if (!CADability.GeoObject.Surfaces.IntersectThreeSurfaces(topOffset, edgeToRound.PrimaryFace.Domain, bottomOffset, edgeToRound.SecondaryFace.Domain, rightPlane, plnBounds,
                    ref filletAxisRight, out GeoPoint2D uv21, out GeoPoint2D uv22, out GeoPoint2D uv23)) return null;
                SurfaceHelper.AdjustPeriodic(topSurface, topDomain, ref uv21);
                SurfaceHelper.AdjustPeriodic(bottomSurface, bottomDomain, ref uv22);
                topDomain.MinMax(uv21);
                bottomDomain.MinMax(uv22);
                IDualSurfaceCurve filletAxisCurve = topOffset.GetDualSurfaceCurves(topDomain, bottomOffset, bottomDomain, new List<GeoPoint>([filletAxisLeft, filletAxisRight]))
                    .MinBy(dsc => dsc.Curve3D.DistanceTo(filletAxisLeft) + dsc.Curve3D.DistanceTo(filletAxisRight));
                if (filletAxisCurve == null) return null;
                Vertex lt, lb, rb, rt; // the four vertices, left plane (lid) with bottom face lb, etc.
                // we calculate the four vertices as intersection of the swept circle with the left and right plane and the top and bottom surface
                SweptCircle swc = new SweptCircle(filletAxisCurve.Curve3D, radius);
                GeoPoint2D uvswc, uvs;
                if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(leftPlane.Location, leftPlane.Normal, swc, bottomSurface, out uvswc, out uvs))
                {
                    GeoPoint ipswc = swc.PointAt(uvswc);
                    GeoPoint ips = bottomSurface.PointAt(uvs);
                    lb = new Vertex(new GeoPoint(ipswc, ips)); // both points should be almost identical
                }
                else return null;
                if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(leftPlane.Location, leftPlane.Normal, swc, topSurface, out uvswc, out uvs))
                {
                    GeoPoint ipswc = swc.PointAt(uvswc);
                    GeoPoint ips = topSurface.PointAt(uvs);
                    lt = new Vertex(new GeoPoint(ipswc, ips)); // both points should be almost identical
                }
                else return null;
                if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(rightPlane.Location, rightPlane.Normal, swc, bottomSurface, out uvswc, out uvs))
                {
                    GeoPoint ipswc = swc.PointAt(uvswc);
                    GeoPoint ips = bottomSurface.PointAt(uvs);
                    rb = new Vertex(new GeoPoint(ipswc, ips)); // both points should be almost identical
                }
                else return null;
                if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(rightPlane.Location, rightPlane.Normal, swc, topSurface, out uvswc, out uvs))
                {
                    GeoPoint ipswc = swc.PointAt(uvswc);
                    GeoPoint ips = topSurface.PointAt(uvs);
                    rt = new Vertex(new GeoPoint(ipswc, ips)); // both points should be almost identical
                }
                else return null;
                filletAxisCurve.Trim(filletAxisLeft, filletAxisRight); // the axis for the fillet
                // construct the left lid of the "negative fillet"
                // now lets construct the lid as a face
                IDualSurfaceCurve lid1crv1 = topSurface.GetDualSurfaceCurves(edgeToRound.PrimaryFace.Domain, leftPlane, new BoundingRect(GeoPoint2D.Origin, radius, radius),
                    new List<GeoPoint>([lt.Position, leadingEdge.StartPoint])).MinBy(dsc => dsc.Curve3D.DistanceTo(lt.Position));
                if (lid1crv1 == null) return null;
                lid1crv1.Trim(leadingEdge.StartPoint, lt.Position); // lt.Location
                IDualSurfaceCurve lid1crv2 = bottomSurface.GetDualSurfaceCurves(edgeToRound.SecondaryFace.Domain, leftPlane, new BoundingRect(GeoPoint2D.Origin, radius, radius),
                    new List<GeoPoint>([lb.Position, leadingEdge.StartPoint])).MinBy(dsc => dsc.Curve3D.DistanceTo(lb.Position));
                if (lid1crv2 == null) return null;
                lid1crv2.Trim(lb.Position, leadingEdge.StartPoint);
                Arc2D arc2DOnLeftPlane = new Arc2D(leftPlane.PositionOf(filletAxisLeft), radius, leftPlane.PositionOf(lt.Position), leftPlane.PositionOf(lb.Position), false);
                if (!isConvex) arc2DOnLeftPlane.Complement();
                ICurve lid1crv3 = leftPlane.Make3dCurve(arc2DOnLeftPlane);
                lid1crv3.StartPoint = lt.Position; // for better precision, should be almost equal
                lid1crv3.EndPoint = lb.Position;
                Edge le1;
                Edge le2;
                Edge le3;
                Face leftLid;
                if (isConvex)
                {
                    le1 = new Edge(null, lid1crv1.Curve3D, null, lid1crv1.Curve2D2, true);
                    le1.UseVertices(lb, lt);
                    le2 = new Edge(null, lid1crv3, null, arc2DOnLeftPlane, true);
                    le2.UseVertices(lb, lt);
                    le3 = new Edge(null, lid1crv2.Curve3D, null, lid1crv2.Curve2D2, true);
                    le3.UseVertices(lb, lt);
                    leftLid = Face.MakeFace(leftPlane, new Edge[] { le1, le2, le3 });
                }
                else
                {
                    lid1crv1.Reverse();
                    lid1crv3.Reverse();
                    lid1crv2.Reverse();
                    arc2DOnLeftPlane.Reverse();
                    le1 = new Edge(null, lid1crv1.Curve3D, null, lid1crv1.Curve2D2, true);
                    le1.UseVertices(lb, lt);
                    le2 = new Edge(null, lid1crv3, null, arc2DOnLeftPlane, true);
                    le2.UseVertices(lb, lt);
                    le3 = new Edge(null, lid1crv2.Curve3D, null, lid1crv2.Curve2D2, true);
                    le3.UseVertices(lb, lt);
                    leftLid = Face.MakeFace(leftPlane, new Edge[] { le3, le2, le1 });
                }
#if DEBUG
                for (int i = 0; i < leftLid.OutlineEdges.Length; i++)
                {
                    int j = (i + 1) % leftLid.OutlineEdges.Length;
                    double d = leftLid.OutlineEdges[i].EndVertex(leftLid).Position | leftLid.OutlineEdges[j].StartVertex(leftLid).Position;
                }
                leftLid.CheckConsistency();
#endif

                // construct the right lid of the "negative fillet"
                // now lets construct the lid as a face
                IDualSurfaceCurve lid2crv1 = topSurface.GetDualSurfaceCurves(edgeToRound.PrimaryFace.Domain, rightPlane, new BoundingRect(GeoPoint2D.Origin, radius, radius),
                    new List<GeoPoint>([rt.Position, leadingEdge.EndPoint])).MinBy(dsc => dsc.Curve3D.DistanceTo(rt.Position));
                if (lid2crv1 == null) return null;
                lid2crv1.Trim(rt.Position, leadingEdge.EndPoint);
                IDualSurfaceCurve lid2crv2 = bottomSurface.GetDualSurfaceCurves(edgeToRound.SecondaryFace.Domain, rightPlane, new BoundingRect(GeoPoint2D.Origin, radius, radius),
                    new List<GeoPoint>([rb.Position, leadingEdge.EndPoint])).MinBy(dsc => dsc.Curve3D.DistanceTo(rb.Position));
                if (lid2crv2 == null) return null;
                lid2crv2.Trim(leadingEdge.EndPoint, rb.Position);
                Arc2D arc2DOnRightPlane = new Arc2D(rightPlane.PositionOf(filletAxisRight), radius, rightPlane.PositionOf(rb.Position), rightPlane.PositionOf(rt.Position), false);
                if (!isConvex) arc2DOnRightPlane.Complement();
                ICurve lid2crv3 = rightPlane.Make3dCurve(arc2DOnRightPlane);
                lid2crv3.StartPoint = rb.Position;
                lid2crv3.EndPoint = rt.Position;
                Edge re1;
                Edge re2;
                Edge re3;
                Face rightLid;
                if (isConvex)
                {
                    re1 = new Edge(null, lid2crv1.Curve3D, null, lid2crv1.Curve2D2, true);
                    re1.UseVertices(rb, rt);
                    re2 = new Edge(null, lid2crv3, null, arc2DOnRightPlane, true);
                    re2.UseVertices(rb, rt);
                    re3 = new Edge(null, lid2crv2.Curve3D, null, lid2crv2.Curve2D2, true);
                    re3.UseVertices(rb, rt);
                    rightLid = Face.MakeFace(rightPlane, new Edge[] { re1, re2, re3 });
                }
                else
                {
                    lid2crv1.Reverse();
                    lid2crv3.Reverse();
                    lid2crv2.Reverse();
                    arc2DOnRightPlane.Reverse();
                    re1 = new Edge(null, lid2crv1.Curve3D, null, lid2crv1.Curve2D2, true);
                    re1.UseVertices(rb, rt);
                    re2 = new Edge(null, lid2crv3, null, arc2DOnRightPlane, true);
                    re2.UseVertices(rb, rt);
                    re3 = new Edge(null, lid2crv2.Curve3D, null, lid2crv2.Curve2D2, true);
                    re3.UseVertices(rb, rt);
                    rightLid = Face.MakeFace(rightPlane, new Edge[] { re3, re2, re1 });
                }
#if DEBUG
                for (int i = 0; i < rightLid.OutlineEdges.Length; i++)
                {
                    int j = (i + 1) % rightLid.OutlineEdges.Length;
                    double d = rightLid.OutlineEdges[i].EndVertex(rightLid).Position | rightLid.OutlineEdges[j].StartVertex(rightLid).Position;
                }
                rightLid.CheckConsistency();
#endif

                // now we need the tangential edges between the swept arc and the top rsp. bottom face
                // we could do an intersection, which is currently numerical instable with tangential intersections
                // or we get the perpendicular projection of the filletAxisCurve on both surfaces
                ICurve topTangentialCurve = topSurface.Make3dCurve(filletAxisCurve.Curve2D1);
                topTangentialCurve.StartPoint = lt.Position; // adjust the curve to the already calculated vertices
                topTangentialCurve.EndPoint = rt.Position;
                ICurve bottomTangentialCurve = bottomSurface.Make3dCurve(filletAxisCurve.Curve2D2);
                bottomTangentialCurve.StartPoint = lb.Position; // adjust the curve to the already calculated vertices
                bottomTangentialCurve.EndPoint = rb.Position;
                ICurve edgeCurve = leadingEdge.Clone();
                // le1 and re1 are two edges we need for the top face, the orientation of the topTangentialCurve and edgeCurve and the order must be tested
                // the filletAxisCurve and the leadingEdge have the same orientation.
                // on the top face, the edgCurve must have the same orientation as the edgeToRound
                Edge[] topEdges = new Edge[4];
                Face topFace;
                if (isConvex)
                {
                    topTangentialCurve.Reverse();
                    topEdges[0] = new Edge(null, topTangentialCurve, null, filletAxisCurve.Curve2D1.CloneReverse(true), true);
                    topEdges[1] = le1; // we know the 2d curve but cannot provide it
                    topEdges[2] = new Edge(null, edgeCurve, null, edgeToRound.Curve2D(edgeToRound.PrimaryFace), true);
                    topEdges[3] = re1;
                    topFace = Face.MakeFace(topSurface.Clone(), topEdges);
                }
                else
                {
                    ISurface surface = topSurface.Clone();
                    ModOp2D rev = surface.ReverseOrientation();
                    ICurve2D tan = filletAxisCurve.Curve2D1.GetModified(rev);
                    topEdges[3] = new Edge(null, topTangentialCurve, null, tan, true);
                    topEdges[2] = le1; // we know the 2d curve but cannot provide it
                    ICurve2D lead = edgeToRound.Curve2D(edgeToRound.PrimaryFace).GetModified(rev);
                    lead.Reverse();
                    edgeCurve.Reverse();
                    topEdges[1] = new Edge(null, edgeCurve, null, lead, true);
                    topEdges[0] = re1;
                    topFace = Face.MakeFace(surface, topEdges);
                }
#if DEBUG
                topFace.CheckConsistency();
#endif

                Edge[] bottomEdges = new Edge[4];
                Face bottomFace;
                if (isConvex)
                {
                    bottomEdges[3] = new Edge(null, bottomTangentialCurve, null, filletAxisCurve.Curve2D2.CloneReverse(edgeToRound.Forward(edgeToRound.SecondaryFace)), true);
                    bottomEdges[2] = le3; // we know the 2d curve but cannot provide it
                    bottomEdges[1] = topEdges[2];
                    bottomEdges[0] = re3;
                    bottomFace = Face.MakeFace(bottomSurface.Clone(), bottomEdges);
                }
                else
                {
                    ISurface surface = bottomSurface.Clone();
                    ModOp2D rev = surface.ReverseOrientation();
                    bottomTangentialCurve.Reverse();
                    ICurve2D tan = filletAxisCurve.Curve2D2.GetModified(rev);
                    bottomEdges[0] = new Edge(null, bottomTangentialCurve, null, tan, true);
                    bottomEdges[1] = le3; // we know the 2d curve but cannot provide it
                    bottomEdges[2] = topEdges[1];
                    bottomEdges[3] = re3;
                    bottomFace = Face.MakeFace(surface.Clone(), bottomEdges);
                }
#if DEBUG
                bottomFace.CheckConsistency();
#endif

                ISurface sweptCircle;
                sweptCircle = SweptCircle.MakePipeSurface(filletAxisCurve.Curve3D, radius, filletAxisCurve.Curve3D.PointAt(0.5) - leadingEdge.PointAt(0.5));
                //GeoPoint2D dbguv = new GeoPoint2D(0, 2 * Math.PI - Math.PI / 4);
                //GeoPoint dbg3d = sweptCircle.PointAt(dbguv);
                //GeoVector dbgudir = sweptCircle.UDirection(dbguv);
                //GeoVector dbgvdir = sweptCircle.VDirection(dbguv);
                //GeoVector dbgdir = sweptCircle.GetNormal(dbguv);
                //ModOp2D revo = sweptCircle.ReverseOrientation();
                //GeoPoint dbg3d1 = sweptCircle.PointAt(revo * dbguv);
                //GeoPoint2D uvxxx = sweptCircle.PositionOf(dbg3d);
                //dbg3d1 = sweptCircle.PointAt(uvxxx);
                //GeoVector dbgdir1 = sweptCircle.GetNormal(revo * dbguv);
                //GeoVector dbgudir1 = sweptCircle.UDirection(revo * dbguv);
                //GeoVector dbgvdir1 = sweptCircle.VDirection(revo * dbguv);

                // we need bounds for sweptCircle to enable Makeface to use BoxedSurface methods
                sweptCircle.SetBounds(new BoundingRect(0, Math.PI / 2, 1, 3 * Math.PI / 2));
                //sweptCircle.PointAt(GeoPoint2D.Origin);
                //Face dbgfc = Face.MakeFace(sweptCircle, new BoundingRect(0.0, Math.PI / 2, 1.0, 3 * Math.PI / 2));
                //Face dbgfc1 = Face.MakeFace(sweptCircle, new BoundingRect(0.0, 3 * Math.PI / 2, 1.0, 5 * Math.PI / 2));
                //SimpleShape dbgss = dbgfc.Area;
                //sweptCircle.Intersect(Line.TwoPoints(GeoPoint.Origin, new GeoPoint(100, 100, 100)), new BoundingRect(0.1, 0, 0.2, 0.5), out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve3Ds);
                Face sweptFace;
                // how to test for correct orientation of the sweptCircle?
                // The normal of the swept circle must point towards the filletAxisCurve, it must be a concave surface in both cases
                // In the covex-edge case the fillet shell is removed, the result will be a convex swept surface
                // In the concave-edge case the fillet shell is added, the result will be a concave swept surface
                GeoVector testNormal = sweptCircle.GetNormal(new GeoPoint2D(0.5, Math.PI));
                GeoPoint testPoint = sweptCircle.PointAt(new GeoPoint2D(0.5, Math.PI));
                Face dbgfc0 = Face.MakeFace(sweptCircle, SimpleShape.MakeCircle(new GeoPoint2D(0.5, Math.PI), 0.5));
                if (testNormal * (filletAxisCurve.Curve3D.PointAt(0.5) - testPoint) < 0) sweptCircle.ReverseOrientation();
                GeoPoint dbgp = sweptCircle.PointAt(new GeoPoint2D(0.3, 1.5));
                GeoPoint2D dbg2d = sweptCircle.PositionOf(dbgp);
                Face dbgfc = Face.MakeFace(sweptCircle, SimpleShape.MakeCircle(new GeoPoint2D(0.5, Math.PI), 0.5));
                dbgfc.GetTriangulation(0.01, out GeoPoint[] trianglePoint, out GeoPoint2D[] triangleUVPoint, out int[] triangleIndex, out BoundingCube triangleExtent);
                GeoObjectList dbgtr = new GeoObjectList();
                for (int i = 0; i < triangleIndex.Length; i += 3)
                {
                    dbgtr.Add(Line.MakeLine(trianglePoint[triangleIndex[i]], trianglePoint[triangleIndex[i + 1]]));
                    dbgtr.Add(Line.MakeLine(trianglePoint[triangleIndex[i + 1]], trianglePoint[triangleIndex[i + 2]]));
                    dbgtr.Add(Line.MakeLine(trianglePoint[triangleIndex[i + 2]], trianglePoint[triangleIndex[i]]));
                }
                sweptCircle.SetBounds(BoundingRect.EmptyBoundingRect);
                if (isConvex)
                {
                    sweptFace = Face.MakeFace(sweptCircle, new Edge[] { topEdges[0], re2, bottomEdges[3], le2 });
                }
                else
                {
                    sweptFace = Face.MakeFace(sweptCircle, new Edge[] { topEdges[3], le2, bottomEdges[0], re2 });
                    sweptFace.RecalcVertices();
                    // sweptFace = Face.MakeFace(sweptCircle, new Edge[] { re2, bottomEdges[0], le2, topEdges[3] });
                    for (int i = 0; i < sweptFace.OutlineEdges.Length; i++)
                    {
                        int j = (i + 1) % sweptFace.OutlineEdges.Length;
                        double d = sweptFace.OutlineEdges[i].EndVertex(sweptFace).Position | sweptFace.OutlineEdges[j].StartVertex(sweptFace).Position;
                    }
                }
                GeoVector testNormal1 = sweptCircle.GetNormal(new GeoPoint2D(0.5, Math.PI));
                GeoPoint testPoint1 = sweptCircle.PointAt(new GeoPoint2D(0.5, Math.PI));
                bool oksw = sweptFace.CheckConsistency();
                SimpleShape dbgss = sweptFace.Area;
                Shell filletShell = Shell.FromFaces(sweptFace, bottomFace, topFace, rightLid, leftLid);
                List<Edge> te = [.. sweptFace.Edges.Intersect(bottomFace.Edges)];
                if (te.Count == 1) tangentialEdges[edgeToRound.SecondaryFace] = te[0];
                te = [.. sweptFace.Edges.Intersect(topFace.Edges)];
                if (te.Count == 1) tangentialEdges[edgeToRound.PrimaryFace] = te[0];
#if DEBUG
                bool ok = filletShell.CheckConsistency();
#endif
                if (isConvex) shellsToSubtract.Add(filletShell);
                else shellsToAdd.Add(filletShell);
            }

            Dictionary<Edge, Edge> clonedEdges = new Dictionary<Edge, Edge>();
            Dictionary<Vertex, Vertex> clonedVertices = new Dictionary<Vertex, Vertex>();
            Dictionary<Face, Face> clonedFaces = new Dictionary<Face, Face>();
            Shell toOperateOn = shell.Clone(clonedEdges, clonedVertices, clonedFaces) as Shell;
            bool success = false;
            for (int i = 0; i < shellsToSubtract.Count; i++)
            {
                BooleanOperation bo = new BooleanOperation();
                List<Face> sweptFaces = new List<Face>(shellsToSubtract[i].Faces.Where(f => !(f.Surface is PlaneSurface)));
                Shell dbg = Shell.FromFaces(sweptFaces.ToArray());
                bo.SetShells(toOperateOn, dbg, BooleanOperation.Operation.difference);
                // bo.SetShells(toOperateOn, shellsToSubtract[i], BooleanOperation.Operation.difference);
                tangentialEdges = tangentialEdges.ToDictionary(kv => clonedFaces[kv.Key], kv => kv.Value);
                Shell[] bores = bo.Execute();
                if (bores.Length == 1)
                {
                    toOperateOn = bores[0];
                    success = true; // at least one rounding succeeded
                }
            }
            for (int i = 0; i < shellsToAdd.Count; i++)
            {
                BooleanOperation bo = new BooleanOperation();
                bo.SetShells(toOperateOn, shellsToAdd[i], BooleanOperation.Operation.union);
                tangentialEdges = tangentialEdges.ToDictionary(kv => clonedFaces[kv.Key], kv => kv.Value);
                Shell[] bores = bo.Execute();
                if (bores.Length == 1)
                {
                    toOperateOn = bores[0];
                    success = true; // at least one rounding succeeded
                }
            }
            if (success) return toOperateOn;
            else return null;
        }

        public static Shell? MakeConvexFilletShell(Edge edgeToRound, double radius)
        {
            ISurface topSurface = edgeToRound.PrimaryFace.Surface; // for previty
            ISurface bottomSurface = edgeToRound.SecondaryFace.Surface;
            ICurve leadingEdge = edgeToRound.Curve3D.Clone();
            if (!edgeToRound.Forward(edgeToRound.PrimaryFace)) leadingEdge.Reverse(); // always forward on topSurface
            ISurface topOffset = topSurface.GetOffsetSurface(-radius);
            ISurface bottomOffset = bottomSurface.GetOffsetSurface(-radius);
            if (topOffset == null || bottomOffset == null) return null; // e.g. a sphere shrinking to a point
            topOffset.SetBounds(edgeToRound.PrimaryFace.Domain);
            bottomOffset.SetBounds(edgeToRound.SecondaryFace.Domain);
            // construct the two planes at the front and end side of the fillet
            // we did move them a little bit outwards but rejected this solution again, because we need it at the exact endposition sometimes
            PlaneSurface leftPlane = new PlaneSurface(new Plane(leadingEdge.StartPoint, -leadingEdge.StartDirection));
            PlaneSurface rightPlane = new PlaneSurface(new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection));
            GeoPoint filletAxisLeft = leadingEdge.StartPoint; // a first guess for the intersection, typically a good start
            GeoPoint filletAxisRight = leadingEdge.EndPoint; // the start and endpoint of the axis (spine) of the swept circle
            BoundingRect plnBounds = new BoundingRect(GeoPoint2D.Origin, radius, radius);
            BoundingRect topDomain = edgeToRound.PrimaryFace.Domain; // the domains of the top and bottom surfaces may be a little bit bigger than the original domains
            BoundingRect bottomDomain = edgeToRound.SecondaryFace.Domain;
            if (!CADability.GeoObject.Surfaces.IntersectThreeSurfaces(topOffset, edgeToRound.PrimaryFace.Domain, bottomOffset, edgeToRound.SecondaryFace.Domain, leftPlane, plnBounds,
                ref filletAxisLeft, out GeoPoint2D uv11, out GeoPoint2D uv12, out GeoPoint2D uv13)) return null;
            SurfaceHelper.AdjustPeriodic(topSurface, topDomain, ref uv11);
            SurfaceHelper.AdjustPeriodic(bottomSurface, bottomDomain, ref uv12);
            topDomain.MinMax(uv11);
            bottomDomain.MinMax(uv12);
            if (!CADability.GeoObject.Surfaces.IntersectThreeSurfaces(topOffset, edgeToRound.PrimaryFace.Domain, bottomOffset, edgeToRound.SecondaryFace.Domain, rightPlane, plnBounds,
                ref filletAxisRight, out GeoPoint2D uv21, out GeoPoint2D uv22, out GeoPoint2D uv23)) return null;
            SurfaceHelper.AdjustPeriodic(topSurface, topDomain, ref uv21);
            SurfaceHelper.AdjustPeriodic(bottomSurface, bottomDomain, ref uv22);
            topDomain.MinMax(uv21);
            bottomDomain.MinMax(uv22);
            IDualSurfaceCurve? filletAxisCurve = topOffset.GetDualSurfaceCurves(topDomain, bottomOffset, bottomDomain, new List<GeoPoint>([filletAxisLeft, filletAxisRight]))
                .MinBy(dsc => dsc.Curve3D.DistanceTo(filletAxisLeft) + dsc.Curve3D.DistanceTo(filletAxisRight));
            if (filletAxisCurve == null) return null;
            filletAxisCurve.Trim(filletAxisLeft, filletAxisRight);
            ISurface sweptCircle;
            sweptCircle = SweptCircle.MakePipeSurface(filletAxisCurve.Curve3D, radius, filletAxisCurve.Curve3D.PointAt(0.5) - leadingEdge.PointAt(0.5));

            // we need bounds for sweptCircle to enable Makeface to use BoxedSurface methods
            sweptCircle.SetBounds(new BoundingRect(0, Math.PI / 2, 1, 3 * Math.PI / 2));
#if DEBUG
            GeoObjectList dbgsws = (sweptCircle as ISurfaceImpl).DebugGrid;
            GeoObjectList dbgswd = (sweptCircle as ISurfaceImpl).DebugDirectionsGrid;
#endif
            Face sweptFace;
            // how to test for correct orientation of the sweptCircle?
            // The normal of the swept circle must point towards the filletAxisCurve, it must be a concave surface in both cases
            // In the covex-edge case the fillet shell is removed, the result will be a convex swept surface
            // In the concave-edge case the fillet shell is added, the result will be a concave swept surface

            GeoVector testNormal = sweptCircle.GetNormal(new GeoPoint2D(0.5, Math.PI));
            GeoPoint testPoint = sweptCircle.PointAt(new GeoPoint2D(0.5, Math.PI));
            if (testNormal * (filletAxisCurve.Curve3D.PointAt(0.5) - testPoint) < 0) sweptCircle.ReverseOrientation();
            // find the tangential curves at the "long" sides of the sweptSurface

            GeoPoint2D uvswc, uvs;
            GeoPoint rb, rt;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(rightPlane.Location, rightPlane.Normal, sweptCircle, bottomSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = bottomSurface.PointAt(uvs);
                rb = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(rightPlane.Location, rightPlane.Normal, sweptCircle, topSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = topSurface.PointAt(uvs);
                rt = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            Arc2D arc2DOnRightPlane = new Arc2D(rightPlane.PositionOf(filletAxisRight), radius, rightPlane.PositionOf(rb), rightPlane.PositionOf(rt), false);
            ICurve lid2crv3 = rightPlane.Make3dCurve(arc2DOnRightPlane);
            lid2crv3.StartPoint = rb;
            lid2crv3.EndPoint = rt;
            lid2crv3.Reverse();
            Edge e2 = new Edge(null, lid2crv3, null, sweptCircle.GetProjectedCurve(lid2crv3, 0.0), true);


            GeoPoint lb, lt;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(leftPlane.Location, leftPlane.Normal, sweptCircle, bottomSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = bottomSurface.PointAt(uvs);
                lb = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(leftPlane.Location, leftPlane.Normal, sweptCircle, topSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = topSurface.PointAt(uvs);
                lt = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            Arc2D arc2DOnLeftPlane = new Arc2D(leftPlane.PositionOf(filletAxisLeft), radius, leftPlane.PositionOf(lt), leftPlane.PositionOf(lb), false);
            ICurve lid1crv3 = leftPlane.Make3dCurve(arc2DOnLeftPlane);
            lid1crv3.StartPoint = lt; // for better precision, should be almost equal
            lid1crv3.EndPoint = lb;
            lid1crv3.Reverse();
            Edge e4 = new Edge(null, lid1crv3, null, sweptCircle.GetProjectedCurve(lid1crv3, 0.0), true);

            BoundingRect sweptBounds = new BoundingRect(sweptCircle.PositionOf(rt), sweptCircle.PositionOf(rb), sweptCircle.PositionOf(lt), sweptCircle.PositionOf(lb));
            IDualSurfaceCurve[] tcCandidates = topSurface.GetDualSurfaceCurves(edgeToRound.PrimaryFace.Domain, sweptCircle, sweptBounds, new List<GeoPoint>([lt, rt]));
            if (tcCandidates == null || tcCandidates.Length == 0) return null;
            IDualSurfaceCurve[] bcCandidates = bottomSurface.GetDualSurfaceCurves(edgeToRound.SecondaryFace.Domain, sweptCircle, sweptBounds, new List<GeoPoint>([rb, lb]));
            if (bcCandidates == null || bcCandidates.Length == 0) return null;
            if (tcCandidates.Length > 1)
            {
                // select best solution here
            }
            if (bcCandidates.Length > 1)
            {
                // select best solution here
            }
            if ((bcCandidates[0].Curve3D.StartPoint | rb) + (bcCandidates[0].Curve3D.EndPoint | lb) > (bcCandidates[0].Curve3D.StartPoint | lb) + (bcCandidates[0].Curve3D.EndPoint | rb))
            {
                bcCandidates[0].Reverse();
            }
            if ((tcCandidates[0].Curve3D.StartPoint | lt) + (tcCandidates[0].Curve3D.EndPoint | rt) > (tcCandidates[0].Curve3D.StartPoint | rt) + (tcCandidates[0].Curve3D.EndPoint | lt))
            {
                tcCandidates[0].Reverse();
            }
            Edge e1 = new Edge(null, tcCandidates[0].Curve3D, null, tcCandidates[0].Curve2D2, true); // this is primary face of the edge with swept circle face
            Edge e3 = new Edge(null, bcCandidates[0].Curve3D, null, bcCandidates[0].Curve2D2, true); // this is secondary face of the edge with swept circle face


            sweptFace = Face.MakeFace(sweptCircle, [e1, e2, e3, e4]);

            // when the leading edge goes from left to right, we look from top onto the primary face. the leading edge is the lower outline of the primary face. The e2 is on the right side.
            // Now we want to construct the right end face of the rounding wedge. All edges are forward on the swept circle, i.e. Vertex1 and Vertex2 are start and endpoints.
            // The swept circle points to the outward.
            ICurve? topRight = null, topLeft = null, bottomRight = null, bottomLeft = null;
            IDualSurfaceCurve[] dsctr = rightPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.PrimaryFace.Surface, edgeToRound.PrimaryFace.Domain, [leadingEdge.EndPoint, e2.Vertex1.Position], null);
            topRight = dsctr.MinBy(dsc => { dsc.Trim(leadingEdge.EndPoint, e2.Vertex1.Position);  return dsc.Curve3D.Length; })?.Curve3D; // when there are more, take the shortest
            IDualSurfaceCurve[] dsctl = leftPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.PrimaryFace.Surface, edgeToRound.PrimaryFace.Domain, [e4.Vertex2.Position, leadingEdge.StartPoint], null);
            topLeft = dsctl.MinBy(dsc => { dsc.Trim(e4.Vertex2.Position, leadingEdge.StartPoint); return dsc.Curve3D.Length; })?.Curve3D; // when there are more, take the shortest

            Face topFace = Face.Construct();
            topSurface = edgeToRound.PrimaryFace.Surface.Clone();
            Edge etr = new Edge(topFace, topRight, topFace, topSurface.GetProjectedCurve(topRight, 0.0), true);
            e1.SetFace(topFace, topSurface.GetProjectedCurve(e1.Curve3D, 0.0), false); // the tangential edge on the primary face, but reverse (SetFace with forward==false reverses the 2d curve)
            Edge etl = new Edge(topFace, topLeft, topFace, topSurface.GetProjectedCurve(topLeft, 0.0), true);
            Edge lde = new Edge(topFace, leadingEdge, topFace, topSurface.GetProjectedCurve(leadingEdge, 0.0), true); // leading edge, which is identical to the edgeToRound
            topFace.Set(topSurface, [[etr, e1, etl, lde]]);

            IDualSurfaceCurve[] dscbr = rightPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.SecondaryFace.Surface, edgeToRound.SecondaryFace.Domain, [e2.Vertex2.Position, leadingEdge.EndPoint], null);
            bottomRight = dscbr.MinBy(dsc => { dsc.Trim(e2.Vertex2.Position, leadingEdge.EndPoint); return dsc.Curve3D.Length; })?.Curve3D; // when there are more, take the shortest
            IDualSurfaceCurve[] dscbl = leftPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.SecondaryFace.Surface, edgeToRound.SecondaryFace.Domain, [leadingEdge.StartPoint, e4.Vertex1.Position], null);
            bottomLeft = dscbl.MinBy(dsc => { dsc.Trim(leadingEdge.StartPoint, e4.Vertex1.Position); return dsc.Curve3D.Length; })?.Curve3D; // when there are more, take the shortest

            Face bottomFace = Face.Construct();
            bottomSurface = edgeToRound.SecondaryFace.Surface.Clone();
            Edge ebr = new Edge(bottomFace, bottomRight, bottomFace, bottomSurface.GetProjectedCurve(bottomRight, 0.0), true);
            e3.SetFace(bottomFace, bottomSurface.GetProjectedCurve(e3.Curve3D, 0.0), false); // the tangential edge on the secondary face, but reverse (SetFace with forward==false reverses the 2d curve)
            Edge ebl = new Edge(bottomFace, bottomLeft, bottomFace, bottomSurface.GetProjectedCurve(bottomLeft, 0.0), true);
            lde.SetFace(bottomFace, bottomSurface.GetProjectedCurve(leadingEdge, 0.0), false); // leading edge, which is identical to the edgeToRound, now in reverse direction
            bottomFace.Set(bottomSurface, [[lde, ebl, e3, ebr]]);

            Face rightEndFace = Face.Construct();
            e2.SetFace(rightEndFace, rightPlane.GetProjectedCurve(e2.Curve3D, 0.0), false); // the arc of the right hand side
            etr.SetFace(rightEndFace, rightPlane.GetProjectedCurve(etr.Curve3D, 0.0), false); // the top bound
            ebr.SetFace(rightEndFace, rightPlane.GetProjectedCurve(ebr.Curve3D, 0.0), false);
            rightEndFace.Set(rightPlane, [[e2, etr, ebr]]);
            Face leftEndFace = Face.Construct();
            e4.SetFace(leftEndFace, leftPlane.GetProjectedCurve(e4.Curve3D, 0.0), false);
            ebl.SetFace(leftEndFace, leftPlane.GetProjectedCurve(ebl.Curve3D, 0.0), false);
            etl.SetFace(leftEndFace, leftPlane.GetProjectedCurve(etl.Curve3D, 0.0), false);
            leftEndFace.Set(leftPlane, [[e4, ebl, etl]]);
            Shell res = Shell.FromFaces(sweptFace, topFace, bottomFace, rightEndFace, leftEndFace);
            res.RecalcVertices();
            rightEndFace.UserData.Add("CADability.Fillet.EndFace", "endface"); // categorize faces
            leftEndFace.UserData.Add("CADability.Fillet.EndFace", "endface");
            sweptFace.UserData.Add("CADability.Fillet.SweptFace", "sweptface");
#if DEBUG
            bool ok = res.CheckConsistency();
#endif
            return res;
        }

        public static Face? MakeConvexFilletFace(Edge edgeToRound, double radius, out Edge[]? frontEnd, out Edge[]? tangential)
        {
            frontEnd = null;
            tangential = null;
            ISurface topSurface = edgeToRound.PrimaryFace.Surface; // for previty
            ISurface bottomSurface = edgeToRound.SecondaryFace.Surface;
            ICurve leadingEdge = edgeToRound.Curve3D.Clone();
            if (!edgeToRound.Forward(edgeToRound.PrimaryFace)) leadingEdge.Reverse(); // always forward on topSurface
            ISurface topOffset = topSurface.GetOffsetSurface(-radius);
            ISurface bottomOffset = bottomSurface.GetOffsetSurface(-radius);
            if (topOffset == null || bottomOffset == null) return null; // e.g. a sphere shrinking to a point
            topOffset.SetBounds(edgeToRound.PrimaryFace.Domain);
            bottomOffset.SetBounds(edgeToRound.SecondaryFace.Domain);
            // construct the two planes at the front and end side of the fillet
            // we did move them a little bit outwards but rejected this solution again, because we need it at the exact endposition sometimes
            PlaneSurface leftPlane = new PlaneSurface(new Plane(leadingEdge.StartPoint, -leadingEdge.StartDirection));
            PlaneSurface rightPlane = new PlaneSurface(new Plane(leadingEdge.EndPoint, leadingEdge.EndDirection));
            GeoPoint filletAxisLeft = leadingEdge.StartPoint; // a first guess for the intersection, typically a good start
            GeoPoint filletAxisRight = leadingEdge.EndPoint; // the start and endpoint of the axis (spine) of the swept circle
            BoundingRect plnBounds = new BoundingRect(GeoPoint2D.Origin, radius, radius);
            BoundingRect topDomain = edgeToRound.PrimaryFace.Domain; // the domains of the top and bottom surfaces may be a little bit bigger than the original domains
            BoundingRect bottomDomain = edgeToRound.SecondaryFace.Domain;
            if (!CADability.GeoObject.Surfaces.IntersectThreeSurfaces(topOffset, edgeToRound.PrimaryFace.Domain, bottomOffset, edgeToRound.SecondaryFace.Domain, leftPlane, plnBounds,
                ref filletAxisLeft, out GeoPoint2D uv11, out GeoPoint2D uv12, out GeoPoint2D uv13)) return null;
            SurfaceHelper.AdjustPeriodic(topSurface, topDomain, ref uv11);
            SurfaceHelper.AdjustPeriodic(bottomSurface, bottomDomain, ref uv12);
            topDomain.MinMax(uv11);
            bottomDomain.MinMax(uv12);
            if (!CADability.GeoObject.Surfaces.IntersectThreeSurfaces(topOffset, edgeToRound.PrimaryFace.Domain, bottomOffset, edgeToRound.SecondaryFace.Domain, rightPlane, plnBounds,
                ref filletAxisRight, out GeoPoint2D uv21, out GeoPoint2D uv22, out GeoPoint2D uv23)) return null;
            SurfaceHelper.AdjustPeriodic(topSurface, topDomain, ref uv21);
            SurfaceHelper.AdjustPeriodic(bottomSurface, bottomDomain, ref uv22);
            topDomain.MinMax(uv21);
            bottomDomain.MinMax(uv22);
            IDualSurfaceCurve? filletAxisCurve = topOffset.GetDualSurfaceCurves(topDomain, bottomOffset, bottomDomain, new List<GeoPoint>([filletAxisLeft, filletAxisRight]))
                .MinBy(dsc => dsc.Curve3D.DistanceTo(filletAxisLeft) + dsc.Curve3D.DistanceTo(filletAxisRight));
            if (filletAxisCurve == null) return null;
            filletAxisCurve.Trim(filletAxisLeft, filletAxisRight);
            ISurface sweptCircle;
            sweptCircle = SweptCircle.MakePipeSurface(filletAxisCurve.Curve3D, radius, filletAxisCurve.Curve3D.PointAt(0.5) - leadingEdge.PointAt(0.5));

            // we need bounds for sweptCircle to enable Makeface to use BoxedSurface methods
            sweptCircle.SetBounds(new BoundingRect(0, Math.PI / 2, 1, 3 * Math.PI / 2));
#if DEBUG
            GeoObjectList dbgsws = (sweptCircle as ISurfaceImpl).DebugGrid;
            GeoObjectList dbgswd = (sweptCircle as ISurfaceImpl).DebugDirectionsGrid;
#endif
            Face sweptFace;
            // how to test for correct orientation of the sweptCircle?
            // The normal of the swept circle must point towards the filletAxisCurve, it must be a concave surface in both cases
            // In the covex-edge case the fillet shell is removed, the result will be a convex swept surface
            // In the concave-edge case the fillet shell is added, the result will be a concave swept surface

            GeoVector testNormal = sweptCircle.GetNormal(new GeoPoint2D(0.5, Math.PI));
            GeoPoint testPoint = sweptCircle.PointAt(new GeoPoint2D(0.5, Math.PI));
            if (testNormal * (filletAxisCurve.Curve3D.PointAt(0.5) - testPoint) < 0) sweptCircle.ReverseOrientation();
            // find the tangential curves at the "long" sides of the sweptSurface

            GeoPoint2D uvswc, uvs;
            GeoPoint rb, rt;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(rightPlane.Location, rightPlane.Normal, sweptCircle, bottomSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = bottomSurface.PointAt(uvs);
                rb = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(rightPlane.Location, rightPlane.Normal, sweptCircle, topSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = topSurface.PointAt(uvs);
                rt = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            Arc2D arc2DOnRightPlane = new Arc2D(rightPlane.PositionOf(filletAxisRight), radius, rightPlane.PositionOf(rb), rightPlane.PositionOf(rt), false);
            ICurve lid2crv3 = rightPlane.Make3dCurve(arc2DOnRightPlane);
            lid2crv3.StartPoint = rb;
            lid2crv3.EndPoint = rt;
            lid2crv3.Reverse();
            Edge e2 = new Edge(null, lid2crv3, null, sweptCircle.GetProjectedCurve(lid2crv3, 0.0), true);


            GeoPoint lb, lt;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(leftPlane.Location, leftPlane.Normal, sweptCircle, bottomSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = bottomSurface.PointAt(uvs);
                lb = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(leftPlane.Location, leftPlane.Normal, sweptCircle, topSurface, out uvswc, out uvs))
            {
                GeoPoint ipswc = sweptCircle.PointAt(uvswc);
                GeoPoint ips = topSurface.PointAt(uvs);
                lt = new GeoPoint(ipswc, ips); // both points should be almost identical
            }
            else return null;
            Arc2D arc2DOnLeftPlane = new Arc2D(leftPlane.PositionOf(filletAxisLeft), radius, leftPlane.PositionOf(lt), leftPlane.PositionOf(lb), false);
            ICurve lid1crv3 = leftPlane.Make3dCurve(arc2DOnLeftPlane);
            lid1crv3.StartPoint = lt; // for better precision, should be almost equal
            lid1crv3.EndPoint = lb;
            lid1crv3.Reverse();
            Edge e4 = new Edge(null, lid1crv3, null, sweptCircle.GetProjectedCurve(lid1crv3, 0.0), true);

            BoundingRect sweptBounds = new BoundingRect(sweptCircle.PositionOf(rt), sweptCircle.PositionOf(rb), sweptCircle.PositionOf(lt), sweptCircle.PositionOf(lb));
            IDualSurfaceCurve[] tcCandidates = topSurface.GetDualSurfaceCurves(edgeToRound.PrimaryFace.Domain, sweptCircle, sweptBounds, new List<GeoPoint>([lt, rt]));
            if (tcCandidates == null || tcCandidates.Length == 0) return null;
            IDualSurfaceCurve[] bcCandidates = bottomSurface.GetDualSurfaceCurves(edgeToRound.SecondaryFace.Domain, sweptCircle, sweptBounds, new List<GeoPoint>([rb, lb]));
            if (bcCandidates == null || bcCandidates.Length == 0) return null;
            if (tcCandidates.Length > 1)
            {
                // select best solution here
            }
            if (bcCandidates.Length > 1)
            {
                // select best solution here
            }
            if ((bcCandidates[0].Curve3D.StartPoint | rb) + (bcCandidates[0].Curve3D.EndPoint | lb) > (bcCandidates[0].Curve3D.StartPoint | lb) + (bcCandidates[0].Curve3D.EndPoint | rb))
            {
                bcCandidates[0].Reverse();
            }
            if ((tcCandidates[0].Curve3D.StartPoint | lt) + (tcCandidates[0].Curve3D.EndPoint | rt) > (tcCandidates[0].Curve3D.StartPoint | rt) + (tcCandidates[0].Curve3D.EndPoint | lt))
            {
                tcCandidates[0].Reverse();
            }
            Edge e1 = new Edge(null, tcCandidates[0].Curve3D, null, tcCandidates[0].Curve2D2, true); // this is primary face of the edge with swept circle face
            Edge e3 = new Edge(null, bcCandidates[0].Curve3D, null, bcCandidates[0].Curve2D2, true); // this is secondary face of the edge with swept circle face


            sweptFace = Face.MakeFace(sweptCircle, [e1, e2, e3, e4]);
#if DEBUG
            bool ok = sweptFace.CheckConsistency();
#endif

            // when the leading edge goes from left to right, we look from top onto the primary face. the leading edge is the lower outline of the primary face. The e2 is on the right side.
            // Now we want to construct the right end face of the rounding wedge. All edges are forward on the swept circle, i.e. Vertex1 and Vertex2 are start and endpoints.
            // The swept circle points to the outward.

            IDualSurfaceCurve[] dsctr = rightPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.PrimaryFace.Surface, edgeToRound.PrimaryFace.Domain, [leadingEdge.EndPoint, e2.Vertex1.Position], null);
            dsctr[0].Trim(leadingEdge.EndPoint, e2.Vertex1.Position);
            IDualSurfaceCurve[] dsctl = leftPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.PrimaryFace.Surface, edgeToRound.PrimaryFace.Domain, [e4.Vertex2.Position, leadingEdge.StartPoint], null);
            dsctl[0].Trim(e4.Vertex2.Position, leadingEdge.StartPoint);

            Face topFace = Face.Construct();
            topSurface = edgeToRound.PrimaryFace.Surface.Clone();
            Edge etr = new Edge(topFace, dsctr[0].Curve3D, topFace, topSurface.GetProjectedCurve(dsctr[0].Curve3D, 0.0), true);
            e1.SetFace(topFace, topSurface.GetProjectedCurve(e1.Curve3D, 0.0), false); // the tangential edge on the primary face, but reverse (SetFace with forward==false reverses the 2d curve)
            Edge etl = new Edge(topFace, dsctl[0].Curve3D, topFace, topSurface.GetProjectedCurve(dsctl[0].Curve3D, 0.0), true);
            Edge lde = new Edge(topFace, leadingEdge, topFace, topSurface.GetProjectedCurve(leadingEdge, 0.0), true); // leading edge, which is identical to the edgeToRound
            topFace.Set(topSurface, [[etr, e1, etl, lde]]);
#if DEBUG
            //            ok = topFace.CheckConsistency();
#endif

            IDualSurfaceCurve[] dscbr = rightPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.SecondaryFace.Surface, edgeToRound.SecondaryFace.Domain, [e2.Vertex2.Position, leadingEdge.EndPoint], null);
            dscbr[0].Trim(e2.Vertex2.Position, leadingEdge.EndPoint);
            IDualSurfaceCurve[] dscbl = leftPlane.GetDualSurfaceCurves(plnBounds, edgeToRound.SecondaryFace.Surface, edgeToRound.SecondaryFace.Domain, [leadingEdge.StartPoint, e4.Vertex1.Position], null);
            dscbl[0].Trim(leadingEdge.StartPoint, e4.Vertex1.Position);

            Face bottomFace = Face.Construct();
            bottomSurface = edgeToRound.SecondaryFace.Surface.Clone();
            Edge ebr = new Edge(bottomFace, dscbr[0].Curve3D, bottomFace, bottomSurface.GetProjectedCurve(dscbr[0].Curve3D, 0.0), true);
            e3.SetFace(bottomFace, bottomSurface.GetProjectedCurve(e3.Curve3D, 0.0), false); // the tangential edge on the secondary face, but reverse (SetFace with forward==false reverses the 2d curve)
            Edge ebl = new Edge(bottomFace, dscbl[0].Curve3D, bottomFace, bottomSurface.GetProjectedCurve(dscbl[0].Curve3D, 0.0), true);
            lde.SetFace(bottomFace, bottomSurface.GetProjectedCurve(leadingEdge, 0.0), false); // leading edge, which is identical to the edgeToRound, now in reverse direction
            bottomFace.Set(bottomSurface, [[lde, ebl, e3, ebr]]);
#if DEBUG
            //            ok = bottomFace.CheckConsistency();
#endif

            Face rightEndFace = Face.Construct();
            e2.SetFace(rightEndFace, rightPlane.GetProjectedCurve(e2.Curve3D, 0.0), false); // the arc of the right hand side
            etr.SetFace(rightEndFace, rightPlane.GetProjectedCurve(etr.Curve3D, 0.0), false); // the top bound
            ebr.SetFace(rightEndFace, rightPlane.GetProjectedCurve(ebr.Curve3D, 0.0), false);
            rightEndFace.Set(rightPlane, [[e2, etr, ebr]]);
            Face leftEndFace = Face.Construct();
            e4.SetFace(leftEndFace, leftPlane.GetProjectedCurve(e4.Curve3D, 0.0), false);
            ebl.SetFace(leftEndFace, leftPlane.GetProjectedCurve(ebl.Curve3D, 0.0), false);
            etl.SetFace(leftEndFace, leftPlane.GetProjectedCurve(etl.Curve3D, 0.0), false);
            leftEndFace.Set(leftPlane, [[e4, ebl, etl]]);
            Shell res = Shell.FromFaces(sweptFace, topFace, bottomFace, rightEndFace, leftEndFace);
            res.RecalcVertices();
#if DEBUG
            ok = res.CheckConsistency();
#endif


            frontEnd = [e2, e4];
            tangential = [e1, e3];


            return sweptFace;
        }
        enum EdgeConnection { Convex, Concave, Tangential };
        public static Shell RoundEdges(this Shell shell, IEnumerable<Edge> edges, double radius)
        {
            RoundEdges re = new RoundEdges(shell, edges, radius);
            return re.Execute();
        }
        public static Shell RoundEdgesX(this Shell shell, IEnumerable<Edge> edges, double radius)
        {
            if (radius < 0) radius = -radius; // radius always >0

            // cluster the edges into connected groups
            List<List<Edge>> clusteredEdges = [];
            while (edges.Any())
            {
                Edge startEdge = edges.First();
                edges = edges.Except(new List<Edge> { startEdge });
                List<Edge> cluster = new List<Edge> { startEdge };
                Queue<Edge> toProcess = new Queue<Edge>();
                toProcess.Enqueue(startEdge);
                while (toProcess.Count > 0)
                {
                    Edge current = toProcess.Dequeue();
                    List<Edge> connected = edges.Where(e => e.IsConnected(current)).ToList();
                    foreach (Edge ce in connected)
                    {
                        cluster.Add(ce);
                        toProcess.Enqueue(ce);
                    }
                    edges = edges.Except(connected);
                }
                clusteredEdges.Add(cluster);
            }

            List<Shell> shellsForRounding = new List<Shell>();
            HashSet<(Face, Edge)> tangentialEdges = new HashSet<(Face, Edge)>();
            foreach (List<Edge> edgeCluster in clusteredEdges)
            {
                // we must differentiate between vertices at the open end of the cluster and vertices inside the cluster
                // inside the cluster, vertices may be connected to two edges or three (or more) edges, where more than three is currently not supported
                Dictionary<Vertex, List<Edge>> connectedVertices = new Dictionary<Vertex, List<Edge>>(); // how many and which edges of this cluster are connected to the vertex
                foreach (Edge edgeToRound in edgeCluster)
                {
                    foreach (Vertex vtx in edgeToRound.Vertices)
                    {
                        if (!connectedVertices.TryGetValue(vtx, out List<Edge>? edgesAtVertex)) connectedVertices[vtx] = edgesAtVertex = new List<Edge>();
                        edgesAtVertex.Add(edgeToRound);
                    }
                }
                // patchToFillets helps to make shells from the fillets and the patches (torus or pipe segments) later
                Dictionary<Face, List<Face>> patchToFillets = new Dictionary<Face, List<Face>>();
                // edgeToFillet collects infos regarding the fillet, which was created for this edge:
                // - fillet: the face the bounds of the edge
                // - frontEnd: the two arcs at the end of the fillet face
                // - tangent: the edges tangential to the primary face [0] and secondary face [1]
                Dictionary<Edge, (Face fillet, Edge[]? frontEnd, Edge[]? tangent)> edgeToFillet = new Dictionary<Edge, (Face fillet, Edge[]? frontEnd, Edge[]? tangent)>();
                foreach (Edge edgeToRound in edgeCluster)
                {
                    if (Adjacency(edgeToRound) == AdjacencyType.Convex)
                    {
                        Face? filletFace = MakeConvexFilletFace(edgeToRound, radius, out Edge[]? frontEnd, out Edge[]? tangential);
                        if (filletFace != null)
                        {
                            edgeToFillet[edgeToRound] = (filletFace, frontEnd, tangential);
                        }
                    }
                }
                foreach (var item in connectedVertices)
                {
                    if (item.Value.Count == 1)
                    {   // this is an open end
                        Edge? openEdge = edgeToFillet[item.Value[0]].frontEnd?.MinBy(edg => edg.Curve3D.DistanceTo(item.Key.Position));
                        Ellipse? ellipse = openEdge?.Curve3D as Ellipse;
                        // we need to check whether we need a face extension on the third (impact) face here
                        if (ellipse == null) continue; // should not happen
                        GeoPoint cnt = ellipse.Center;
                        GeoVector edgeDirAtTheEnd; // the direction at the end of the fillet, at the position of this vertex, poiting outward in the extension of the fillet
                        if (item.Key == item.Value[0].Vertex1) edgeDirAtTheEnd = -item.Value[0].Curve3D.StartDirection;
                        else if (item.Key == item.Value[0].Vertex2) edgeDirAtTheEnd = item.Value[0].Curve3D.EndDirection;
                        else continue; // this should not happen
                        IEnumerable<Face> thirdFaces = item.Key.Faces.Except([item.Value[0].PrimaryFace, item.Value[0].SecondaryFace]);
                        GeoVector dirOfImpactFaces = GeoVector.NullVector;
                        foreach (Face fc in thirdFaces)
                        {
                            dirOfImpactFaces += fc.Surface.GetNormal(item.Key.GetPositionOnFace(fc)).Normalized;
                        }
                        if (dirOfImpactFaces.IsNullVector()) continue; // no impact faces?
                        // we have to decide how to extent the fillet in this direction, there is no simple, definite and obvious solution
                        // first check, whether the two tangents of the fillet intersect the shell
                        if (edgeToFillet[item.Value[0]].tangent == null || edgeToFillet[item.Value[0]].tangent.Length != 2) continue; // there must be two tangents where the fillet touches the shell
                        bool shortExtensionIsOk = true;
                        foreach (GeoPoint startPoint in new List<GeoPoint>([ellipse.PointAt(0.05), ellipse.PointAt(0.5), ellipse.PointAt(0.95)]))
                        {
                            // lets see, whether the extension intersects the shell close to the point
                            if (shell.IsInside(startPoint)) // this point on the fillet ellipse is inside, check whether we can savely expand it
                            {
                                GeoPoint[] extIntsect = shell.GetLineIntersection(startPoint, edgeDirAtTheEnd);
                                bool ok = false;
                                for (int i = 0; i < extIntsect.Length; i++)
                                {
                                    if ((extIntsect[i] | startPoint) < 2 * radius)
                                    {
                                        ok = true;
                                        break;
                                    }
                                }
                                if (!ok)
                                {
                                    shortExtensionIsOk = false;
                                    break;
                                }
                            }
                        }
                        double impact = dirOfImpactFaces.Normalized * edgeDirAtTheEnd.Normalized;
                        // if the impact faces point in the same direction as the fillet, they are probably trimmed by the fillet
                        // if it points the other way, it may have to be extended
                        if (!shortExtensionIsOk && impact < 0)
                        {
                            Face impactFace = item.Key.Faces.Except([item.Value[0].PrimaryFace, item.Value[0].SecondaryFace]).First(); // how to deal with multiple?
                            GeoPoint2D[] impInts = impactFace.Surface.GetLineIntersection(ellipse.StartPoint, edgeDirAtTheEnd);
                            if (impInts.Length > 0)
                            {
                                GeoPoint2D firstPoint = impInts.MinBy(uv =>
                                {
                                    double par = Geometry.LinePar(ellipse.StartPoint, edgeDirAtTheEnd, impactFace.Surface.PointAt(uv));
                                    if (par < 0) return double.MaxValue;
                                    else return par;
                                });
                                GeoPoint ip = impactFace.Surface.PointAt(firstPoint);
                            }
                        }
                        // following is the case where the impact face is truncated by the fillet if the fillet was extented egnough
                        // we construct a torus extension that bends the fillet outwards
                        GeoVector torusXAxis = (item.Key.Position - cnt).Normalized;
                        ToroidalSurface ts = new ToroidalSurface(item.Key.Position, torusXAxis, edgeDirAtTheEnd.Normalized, -(torusXAxis ^ edgeDirAtTheEnd).Normalized, item.Key.Position | cnt, radius);
                        // the ellipse lies on the torus surface, but we need to define a domain that covers the ellipse
                        GeoPoint2D uvsp = ts.PositionOf(ellipse.StartPoint);
                        GeoPoint2D uvep = ts.PositionOf(ellipse.EndPoint);
                        BoundingRect torusDomain = new BoundingRect(uvep, uvsp); // this should work without periodic adjustments, because we are at the inside of the torus surface
                        torusDomain.Left = torusDomain.Right - Math.PI; // a 180° segment is sufficient
                        Face torusFace = Face.MakeFace(ts, torusDomain);
                        torusFace.ReverseOrientation();
                        if (!patchToFillets.TryGetValue(torusFace, out List<Face>? patchFillets))
                        {
                            patchToFillets[torusFace] = patchFillets = new List<Face>();
                        }
                        patchFillets.Add(edgeToFillet[item.Value[0]].fillet);
                    }
                    else if (item.Value.Count == 2)
                    {
                        // this is the connection of two fillets which meet at the vertex
                        // try to make a torus-like segment to fill the gap
                        Face? commonFace = Edge.CommonFace(item.Value[0], item.Value[1]);
                        Edge? thirdEdge = item.Key.AllEdges.Except(item.Value).TheOnlyOrDefault();
                        // in most cases we have a vertex with three faces meeting, so one common face and one third edge
                        // if there are more than three faces meeting at the vertex, we cannot handle this currently
                        if (commonFace == null || thirdEdge == null) continue;
                        GeoVector normalCommon = commonFace.Surface.GetNormal(item.Key.GetPositionOnFace(commonFace)).Normalized;
                        // for brevity:
                        Vertex vtx = item.Key;
                        Face fillet1 = edgeToFillet[item.Value[0]].fillet;
                        Face fillet2 = edgeToFillet[item.Value[1]].fillet;
                        Face touchedByFillet1 = Edge.CommonFace(item.Value[0], thirdEdge);
                        Face touchedByFillet2 = Edge.CommonFace(item.Value[1], thirdEdge);
                        GeoVector normal1 = touchedByFillet1.Surface.GetNormal(item.Key.GetPositionOnFace(touchedByFillet1)).Normalized;
                        GeoVector normal2 = touchedByFillet2.Surface.GetNormal(item.Key.GetPositionOnFace(touchedByFillet2)).Normalized;
                        bool forwardConnected;
                        if (item.Value[0].EndVertex(commonFace) == item.Value[1].StartVertex(commonFace)) forwardConnected = true;
                        else if (item.Value[0].StartVertex(commonFace) == item.Value[1].EndVertex(commonFace)) forwardConnected = false;
                        else continue; // edges are not connected properly
                        double orientation = (forwardConnected ? 1 : -1) * normalCommon * (normal1 ^ normal2); // >0: convex, <0: concave, ==0: tangential
                        EdgeConnection edgeConnection;
                        if (orientation > 1e-6) edgeConnection = EdgeConnection.Convex;
                        else if (orientation < -1e-6) edgeConnection = EdgeConnection.Concave;
                        else edgeConnection = EdgeConnection.Tangential;
                        // now we have to differentiate between the cases: the third edge is convex or concave or tangential
                        ICurve? spine1 = (fillet1.Surface as ISurfaceOfExtrusion)?.Axis(fillet1.Domain);
                        ICurve? spine2 = (fillet2.Surface as ISurfaceOfExtrusion)?.Axis(fillet2.Domain);
                        // the fillets "overshoot" the leading edge, which they are rounding, so we should find an intersection with the third edge.
                        // the fillets should stop ehere and a toroidal surface should connect the two fillets
                        // it is a tangential intersection here, there should be only one intersection point
                        fillet1.Surface.Intersect(thirdEdge.Curve3D, fillet1.Domain, out GeoPoint[] ips1, out GeoPoint2D[] uvs1, out double[] uOnCurve1);
                        fillet2.Surface.Intersect(thirdEdge.Curve3D, fillet2.Domain, out GeoPoint[] ips2, out GeoPoint2D[] uvs2, out double[] uOnCurve2);
                        if (ips1 == null || ips2 == null || ips1.Length != 1 || ips2.Length != 1) continue; // no intersection found, should not happen
                        ISurface aroundThirdEdge = SweptCircle.MakePipeSurface(thirdEdge.Curve3D, radius, -(fillet1.Surface.GetNormal(uvs1[0]).Normalized + fillet2.Surface.GetNormal(uvs2[0]).Normalized));
                        BoundingRect aroundThirdEdgeDomain;
                        if ((aroundThirdEdge as ISurfaceOfExtrusion)!.ExtrusionDirectionIsV) aroundThirdEdgeDomain = new BoundingRect(0, 0, 2 * Math.PI, 1);
                        else aroundThirdEdgeDomain = new BoundingRect(0, 0, 1, 2 * Math.PI);
                        // two points (start and endpoint) on the toroidal spine:
                        GeoPoint tor1 = ips1[0] - radius * fillet1.Surface.GetNormal(uvs1[0]).Normalized;
                        GeoPoint tor2 = ips2[0] - radius * fillet2.Surface.GetNormal(uvs2[0]).Normalized;
#if DEBUG
                        Face dbgAround = Face.MakeFace(aroundThirdEdge, aroundThirdEdgeDomain);
#endif

                        ISurface offset = commonFace.Surface.GetOffsetSurface(-radius);
                        if (offset != null)
                        {
                            IDualSurfaceCurve[] dsc = offset.GetDualSurfaceCurves(commonFace.Domain, aroundThirdEdge, aroundThirdEdgeDomain, [ips1[0], ips2[0]]);
                            ICurve? toroidalSpine = null;
                            double minLength = double.MaxValue;
                            for (int i = 0; i < dsc.Length; i++)
                            {
                                if (dsc[i].Curve3D.IsClosed)
                                {   // we might cross to 1/0 bound
                                    // but it should not cross as we have choosen the seam of aroundThirdEdge accordingly
                                }
                                ICurve ts = dsc[i].Curve3D.Clone();
                                double pos1 = ts.PositionOf(tor1);
                                double pos2 = ts.PositionOf(tor2);
                                if (Math.Abs(pos2 - pos1) > 0.5 && ts.IsClosed)
                                {
                                    ts.Trim(Math.Max(pos1, pos2), Math.Min(pos1, pos2));
                                    if ((ts.StartPoint | tor1) + (ts.EndPoint | tor2) > (ts.StartPoint | tor2) + (ts.EndPoint | tor1)) ts.Reverse();
                                }
                                else
                                {
                                    if (pos1 > pos2)
                                    {
                                        ts.Reverse();
                                        pos1 = 1 - pos1;
                                        pos2 = 1 - pos2;
                                    }
                                    ts.Trim(pos1, pos2);
                                }
                                if (ts.Length < minLength)
                                {
                                    minLength = ts.Length;
                                    toroidalSpine = ts;
                                }
                            }
                            if (toroidalSpine != null)
                            {
                                ISurface connectingToroid = SweptCircle.MakePipeSurface(toroidalSpine, radius, commonFace.Surface.GetNormal(commonFace.Surface.PositionOf(item.Key.Position)));
                                PlaneSurface pln1 = new PlaneSurface(new Plane(toroidalSpine.StartPoint, -toroidalSpine.StartDirection));
                                Face toClipWith1 = Face.MakeFace(pln1, new BoundingRect(GeoPoint2D.Origin, radius * 1.1, radius * 1.1));
                                PlaneSurface pln2 = new PlaneSurface(new Plane(toroidalSpine.EndPoint, toroidalSpine.EndDirection));
                                Face toClipWith2 = Face.MakeFace(pln2, new BoundingRect(GeoPoint2D.Origin, radius * 1.1, radius * 1.1));
                                Face fillet1Clipped = BooleanOperation.ClipFace(fillet1, toClipWith1).TheOnlyOrDefault();
                                Face fillet2Clipped = BooleanOperation.ClipFace(fillet2, toClipWith2).TheOnlyOrDefault();
                                bool tangentialEdgeGoesFrom1to2 = false; // the tangential edge on the common face connects fillet1 and fillet2, but in which direction?
                                if (fillet1Clipped != null && fillet2Clipped != null)
                                {
                                    // create the torus face, which is not a real torus but a segment of a swept circle surface
                                    // the two fillet faces have two edges of the torus face a third edge is the (tangential) intersection with the common face, the fourth
                                    // edge is the segment of the third edge
                                    // find the edges
                                    Edge? arcEdgeOn1 = null, arcEdgeOn2 = null, tangentialEdge = null, thirdEdgeSegment = null;
                                    GeoPoint tangentialp1 = GeoPoint.Invalid, tangentialp2 = GeoPoint.Invalid; // points on the tangential edge
                                    GeoPoint thirdEdgep1 = GeoPoint.Invalid, thirdEdgep2 = GeoPoint.Invalid; // points on the third edge
                                    Face torusFace = Face.Construct();
                                    foreach (Edge edge in fillet1Clipped.AllEdges)
                                    {
                                        if (Math.Abs(pln1.GetDistance(edge.Vertex1.Position)) < Precision.eps && Math.Abs(pln1.GetDistance(edge.Vertex2.Position)) < Precision.eps)
                                        {
                                            ICurve2D onct = connectingToroid.GetProjectedCurve(edge.Curve3D, 0.0);
                                            if (edge.Forward(fillet1Clipped))
                                            {
                                                onct.Reverse();
                                                edge.SetFace(torusFace, onct, false);
                                            }
                                            else
                                            {
                                                edge.SetFace(torusFace, onct, true);
                                            }
                                            arcEdgeOn1 = edge;
                                            if (Math.Abs(commonFace.Surface.GetDistance(edge.StartVertex(fillet1Clipped).Position)) < Precision.eps)
                                            {
                                                tangentialp1 = edge.StartVertex(fillet1Clipped).Position;
                                                tangentialEdgeGoesFrom1to2 = false;
                                            }
                                            else thirdEdgep1 = edge.StartVertex(fillet1Clipped).Position;
                                            if (Math.Abs(commonFace.Surface.GetDistance(edge.EndVertex(fillet1Clipped).Position)) < Precision.eps)
                                            {
                                                tangentialp1 = edge.EndVertex(fillet1Clipped).Position;
                                                tangentialEdgeGoesFrom1to2 = true;
                                            }
                                            else thirdEdgep1 = edge.EndVertex(fillet1Clipped).Position;
                                        }
                                    }
                                    foreach (Edge edge in fillet2Clipped.AllEdges)
                                    {
                                        if (Math.Abs(pln2.GetDistance(edge.Vertex1.Position)) < Precision.eps && Math.Abs(pln2.GetDistance(edge.Vertex2.Position)) < Precision.eps)
                                        {
                                            ICurve2D onct = connectingToroid.GetProjectedCurve(edge.Curve3D, 0.0);
                                            if (edge.Forward(fillet2Clipped))
                                            {
                                                onct.Reverse();
                                                edge.SetFace(torusFace, onct, false);
                                            }
                                            else
                                            {
                                                edge.SetFace(torusFace, onct, true);
                                            }
                                            arcEdgeOn2 = edge;
                                            if (Math.Abs(commonFace.Surface.GetDistance(edge.Vertex1.Position)) < Precision.eps) tangentialp2 = edge.Vertex1.Position;
                                            else thirdEdgep2 = edge.Vertex1.Position;
                                            if (Math.Abs(commonFace.Surface.GetDistance(edge.Vertex2.Position)) < Precision.eps) tangentialp2 = edge.Vertex2.Position;
                                            else thirdEdgep2 = edge.Vertex2.Position;
                                        }
                                    }
                                    if (arcEdgeOn1 == null || arcEdgeOn2 == null) continue; // no edges found, should not happen
                                    if (!tangentialp1.IsValid || !tangentialp2.IsValid) continue; // we need two points to make the tangential edge
                                    // find a domain on connectingToroid. it is in both directions (u and v) less than 180°
                                    // so we can adjust the uv points close to each other
                                    BoundingRect toroidDomain = BoundingRect.EmptyBoundingRect;
                                    foreach (Edge edg in (List<Edge>)[arcEdgeOn1, arcEdgeOn2])
                                    {
                                        GeoPoint2D uv = connectingToroid.PositionOf(edg.Vertex1.Position);
                                        if (!toroidDomain.IsEmpty()) SurfaceHelper.AdjustPeriodic(connectingToroid, toroidDomain, ref uv);
                                        toroidDomain.MinMax(uv);
                                        uv = connectingToroid.PositionOf(edg.Vertex2.Position);
                                        SurfaceHelper.AdjustPeriodic(connectingToroid, toroidDomain, ref uv);
                                        toroidDomain.MinMax(uv);
                                    }
                                    // we must have two points on the common face here and two curves (arcs)

                                    // construct the edge segment on the third edge between the two intersection points
                                    double pos1 = thirdEdge.Curve3D.PositionOf(ips1[0]);
                                    double pos2 = thirdEdge.Curve3D.PositionOf(ips2[0]);
                                    if (pos1 > pos2) (pos1, pos2) = (pos2, pos1);
                                    ICurve tes = thirdEdge.Curve3D.Clone();
                                    tes.Trim(pos1, pos2); // this is the connection between the two arcs on thirdEdge
                                    if (tangentialEdgeGoesFrom1to2)
                                    {
                                        if ((tes.StartPoint | thirdEdgep1) + (tes.EndPoint | thirdEdgep2) > (tes.StartPoint | thirdEdgep2) + (tes.EndPoint | thirdEdgep1)) tes.Reverse();
                                    }
                                    else
                                    {
                                        if ((tes.StartPoint | thirdEdgep2) + (tes.EndPoint | thirdEdgep1) > (tes.StartPoint | thirdEdgep1) + (tes.EndPoint | thirdEdgep2)) tes.Reverse();
                                    }
                                    thirdEdgeSegment = new Edge(torusFace, tes, torusFace, connectingToroid.GetProjectedCurve(tes, 0.0), true);

                                    // construct the tangential edge on the common face
                                    IDualSurfaceCurve? onCommonFace = connectingToroid.GetDualSurfaceCurves(toroidDomain, commonFace.Surface, commonFace.Domain, [tangentialp1, tangentialp2]).TheOnlyOrDefault();
                                    if (onCommonFace == null)
                                    {
                                        onCommonFace = new InterpolatedDualSurfaceCurve(commonFace.Surface, commonFace.Domain, connectingToroid, toroidDomain, tangentialp1, tangentialp2, true);
                                    }
                                    if (onCommonFace == null) continue; // there must be a tangential intersection between the connecting toroid and the common face
                                    if (tangentialEdgeGoesFrom1to2) onCommonFace.Trim(tangentialp2, tangentialp1);
                                    else onCommonFace.Trim(tangentialp1, tangentialp2);
                                    tangentialEdge = new Edge(torusFace, onCommonFace.Curve3D, torusFace, onCommonFace.Curve2D1, true);
#if DEBUG
                                    DebuggerContainer dbg3d = new DebuggerContainer();
                                    dbg3d.Add(arcEdgeOn1.Curve3D as IGeoObject, arcEdgeOn1.GetHashCode());
                                    dbg3d.Add(arcEdgeOn2.Curve3D as IGeoObject, arcEdgeOn2.GetHashCode());
                                    dbg3d.Add(tangentialEdge.Curve3D as IGeoObject, tangentialEdge.GetHashCode());
                                    dbg3d.Add(thirdEdgeSegment.Curve3D as IGeoObject, thirdEdgeSegment.GetHashCode());
                                    DebuggerContainer dbg2d = new DebuggerContainer();
                                    dbg2d.Add(arcEdgeOn1.Curve2D(torusFace), System.Drawing.Color.Red, arcEdgeOn1.GetHashCode());
                                    dbg2d.Add(arcEdgeOn2.Curve2D(torusFace), System.Drawing.Color.Red, arcEdgeOn2.GetHashCode());
                                    dbg2d.Add(tangentialEdge.Curve2D(torusFace), System.Drawing.Color.Red, tangentialEdge.GetHashCode());
                                    dbg2d.Add(thirdEdgeSegment.Curve2D(torusFace), System.Drawing.Color.Red, thirdEdgeSegment.GetHashCode());
#endif

                                    // construct the "torus" face
                                    if (tangentialEdgeGoesFrom1to2) torusFace.Set(connectingToroid, [[arcEdgeOn2, tangentialEdge, arcEdgeOn1, thirdEdgeSegment]], false);
                                    else torusFace.Set(connectingToroid, [[arcEdgeOn1, tangentialEdge, arcEdgeOn2, thirdEdgeSegment]], false);
                                    // now we substitute fillet1 and fillet2 by fillet1Clipped and fillet2Clipped
                                    // we must do this in edgeToFillet and patchToFillets
                                    ReplaceFace(item.Value[0], edgeToFillet, fillet1, fillet1Clipped);
                                    ReplaceFace(item.Value[1], edgeToFillet, fillet2, fillet2Clipped);
                                    ReplaceFace(patchToFillets, fillet1, fillet1Clipped);
                                    ReplaceFace(patchToFillets, fillet2, fillet2Clipped);
                                    // connect the patch with the two fillets here
                                    if (!patchToFillets.TryGetValue(torusFace, out List<Face>? fillets1))
                                    {
                                        patchToFillets[torusFace] = fillets1 = new List<Face>();
                                    }
                                    fillets1.Add(fillet1Clipped);
                                    fillets1.Add(fillet2Clipped);
                                }
                            }
                        }
                    }
                    else if (item.Value.Count == 3)
                    {
                        // vertex connected to three edges, make a sphere segment to fill the gap
                        // all edges must be convex
                        List<Face> offsetFaces = new(item.Key.InvolvedFaces);
                        List<Edge> vertexEdges = new(item.Key.AllEdges);
                        List<ISurface> offsetSurfaces = [];
                        for (int i = 0; i < offsetFaces.Count; i++)
                        {
                            offsetSurfaces.AddIfNotNull(offsetFaces[i].Surface.GetOffsetSurface(-radius));
                        }
                        if (offsetSurfaces.Count == 3 && offsetFaces.Count == 3 && vertexEdges.Count == 3)
                        {
                            GeoPoint ip = item.Key.Position;
                            if (CADability.GeoObject.Surfaces.IntersectThreeSurfaces(offsetSurfaces[0], offsetFaces[0].Domain, offsetSurfaces[1], offsetFaces[1].Domain,
                                offsetSurfaces[2], offsetFaces[2].Domain, ref ip, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint2D uv3))
                            {
                                // create sphere segment here. The spherical surface connects the three fillets
                                Solid dbgsph = Make3D.MakeSphere(ip, radius);
                                List<Ellipse> ellipseArcs = [];
                                List<Face> clippedFillets = [];
                                foreach (var edge in vertexEdges)
                                {
                                    var fillet = edgeToFillet[edge].fillet;
                                    if (fillet.Surface is not ISurfaceOfExtrusion ex) continue; // sollte nie passieren

                                    ICurve axis = ex.Axis(fillet.Domain);
                                    double pos = axis.PositionOf(ip);
                                    var dir = axis.DirectionAt(pos);
                                    if (pos > 0.5) dir = -dir;
                                    var plane = new Plane(ip, dir);
                                    var planeSurface = new PlaneSurface(plane);
                                    var toClipWith = Face.MakeFace(planeSurface, new BoundingRect(GeoPoint2D.Origin, radius * 1.1, radius * 1.1));

                                    var filletClipped = BooleanOperation.ClipFace(fillet, toClipWith).TheOnlyOrDefault();
                                    clippedFillets.Add(filletClipped);
                                    foreach (var edg in filletClipped.AllEdges.Where(e => e.Curve3D is Ellipse))
                                    {
                                        if (Math.Abs(planeSurface.GetDistance(edg.Vertex1.Position)) < Precision.eps && Math.Abs(planeSurface.GetDistance(edg.Vertex2.Position)) < Precision.eps)
                                        {
                                            var ellipse = edg.Curve3D as Ellipse;
                                            if (ellipse != null)
                                            {
                                                ellipseArcs.Add(ellipse);
                                            }
                                        }
                                    }

                                    ReplaceFace(edge, edgeToFillet, fillet, filletClipped);
                                    ReplaceFace(patchToFillets, fillet, filletClipped);
                                }
                                if (ellipseArcs.Count == 3)
                                {
                                    for (int i = 0; i < 3; i++) ellipseArcs[i] = ellipseArcs[i].Clone() as Ellipse;
                                    Face sphericalPatch = Face.MakeNonPolarSphere(ellipseArcs[0], ellipseArcs[1], ellipseArcs[2]);
                                    patchToFillets[sphericalPatch] = clippedFillets;
#if DEBUG
                                    DebuggerContainer dc = new DebuggerContainer();
                                    dc.Add(sphericalPatch, System.Drawing.Color.Green, sphericalPatch.GetHashCode());
                                    dc.Add(clippedFillets[0], System.Drawing.Color.Red, clippedFillets[0].GetHashCode());
                                    dc.Add(clippedFillets[1], System.Drawing.Color.Red, clippedFillets[1].GetHashCode());
                                    dc.Add(clippedFillets[2], System.Drawing.Color.Red, clippedFillets[2].GetHashCode());
#endif
                                }
                            }
                        }
                    }
                }
                HashSet<Face> allFillets = new HashSet<Face>(edgeToFillet.Values.Select(v => v.fillet));
                while (patchToFillets.Any())
                {
                    Face ptf = patchToFillets.First().Key;
                    HashSet<Face> shellAroundPatch = new([ptf]);
                    shellAroundPatch.UnionWith(patchToFillets[ptf]);
                    List<Face> patchesToRemove = [ptf];
                    foreach (var ptf1 in patchToFillets)
                    {
                        if (ptf1.Key != ptf)
                        {
                            if (shellAroundPatch.Overlaps(ptf1.Value))
                            {
                                shellAroundPatch.Add(ptf1.Key);
                                shellAroundPatch.UnionWith(ptf1.Value);
                                patchesToRemove.Add(ptf1.Key);
                            }
                        }
                    }
                    // shellAroundPatch contains all patches an their fillets which are connected
                    Face[] s = shellAroundPatch.ToArray();
                    Shell.ConnectFaces(s, Precision.eps);
                    shellsForRounding.AddIfNotNull(Shell.FromFaces(s));
                    allFillets.ExceptWith(shellAroundPatch);
                    foreach (var patch in patchesToRemove) patchToFillets.Remove(patch);
                }
                // the remaining fillets (which are not connected to patches) are used as singl-face shells
                foreach (Face fillet in allFillets) shellsForRounding.AddIfNotNull(Shell.FromFaces(fillet));
            }

            // here we have the convex rounded edges as shells, which have to be subtracted from the shell on which the edges are to be rounded
            Shell toOperateOn = shell;
            for (int i = 0; i < shellsForRounding.Count; i++)
            {
                BooleanOperation bo = new BooleanOperation();
                bo.SetShells(toOperateOn, shellsForRounding[i], BooleanOperation.Operation.intersection);
                bo.SetClosedShells(true, false);
                bo.SetTangentialEdges(tangentialEdges);
                Shell[] roundedShells = bo.Execute();
                if (roundedShells != null && roundedShells.Length == 1) toOperateOn = roundedShells[0];
            }

            return toOperateOn;
        }

        private static void ReplaceFace(Dictionary<Face, List<Face>> patchToFillets, Face fillet1, Face fillet1Clipped)
        {
            foreach (var ptf in patchToFillets)
            {
                for (int i = 0; i < ptf.Value.Count; i++)
                {
                    if (ptf.Value[i] == fillet1) ptf.Value[i] = fillet1Clipped;
                }
            }
        }

        private static void ReplaceFace(Edge key, Dictionary<Edge, (Face fillet, Edge[]? frontEnd, Edge[]? tangent)> edgeToFillet, Face fillet1, Face fillet1Clipped)
        {
            edgeToFillet[key] = (fillet1Clipped, edgeToFillet[key].frontEnd, edgeToFillet[key].tangent);
        }
    }
}
