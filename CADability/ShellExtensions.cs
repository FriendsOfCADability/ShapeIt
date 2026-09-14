using CADability;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.Substitutes;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.RootFinding;
using netDxf.Tables;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;

namespace CADability.GeoObject
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
    public static class ShellExtensions
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
                surface.Derivative2At(
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
        public static Face[] MakeOffsetFillet(Edge axis, double radius, Edge forward, Edge backward, bool dontUseForward, bool dontUseBackward)
        {
            return MakeOffsetFillet(new (Edge, Edge, Edge)[] { (axis, forward, backward) }, radius, dontUseForward, dontUseBackward);
        }

        /// <summary>
        /// Make the fillet like faces which fill the gap between the offset faces along a chain of edges. The edges
        /// of the chain must be tangential continuations of each other, so that all their fillets are parts of one
        /// and the same pipe surface: only then a self intersection of that pipe - which occurs where the curvature
        /// radius of the chain falls below <paramref name="radius"/> - can be resolved, because both sheets which
        /// penetrate each other belong to the same surface. The chain may be closed, e.g. the elliptical edge of a
        /// slanted bore, which is usually split into two edges.
        /// </summary>
        /// <param name="chain">the edges to make the fillet for, in consecutive order, each one together with the
        /// two edges of the offset faces which the fillet has to connect</param>
        /// <param name="radius">the offset, negative for an inner offset</param>
        /// <returns>the faces of the fillet: more than one where the pipe folds over itself</returns>
        public static Face[] MakeOffsetFillet(IReadOnlyList<(Edge axis, Edge forward, Edge backward)> chain, double radius,
            bool dontUseForward = false, bool dontUseBackward = false)
        {
            Edge axis = chain[0].axis, forward = chain[0].forward, backward = chain[0].backward;
            ISurface surface = null;
            GeoVector toInside;
            if (chain.Count > 1)
            {
                surface = ChainPipeSurface(chain, radius);
                if (surface == null)
                {   // the edges cannot be joined into a single spine, so every edge gets its own fillet. A self
                    // intersection which runs across such a joint cannot be resolved then
                    List<Face> single = new List<Face>();
                    for (int i = 0; i < chain.Count; i++)
                    {
                        single.AddRange(MakeOffsetFillet(chain[i].axis, radius, chain[i].forward, chain[i].backward, dontUseForward, dontUseBackward));
                    }
                    return single.ToArray();
                }
            }
            else if (axis.Curve3D is Line line)
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
            {   // the general case: a pipe around the edge. Where the curvature radius of the edge falls below
                // the radius of the pipe, the surface folds over and penetrates itself; that is resolved below
                surface = new SweptCircle(axis.Curve3D, Math.Abs(radius));
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
                    // orientation for the toroidalSurface is difficult, so we fix it here
                    res.Surface.DerivativeAt(bdr.Extent.GetCenter(), out GeoPoint loc, out GeoVector du, out GeoVector dv);
                    GeoPoint locOnAxis = axis.Curve3D.PointAt(axis.Curve3D.PositionOf(loc));
                    double dd = (du ^ dv) * (locOnAxis - loc);
                    if (Math.Sign(dd) == Math.Sign(radius)) res.ReverseOrientation();
                    if (!dontUseForward) res.UseEdge(forward);
                    if (!dontUseBackward) res.UseEdge(backward);
                    return new Face[] { res };
                }
                else
                {
                    Face res = Face.MakeFace(toroidalSurface, new SimpleShape(bdr));
                    res.Surface.DerivativeAt(bdr.Extent.GetCenter(), out GeoPoint loc, out GeoVector du, out GeoVector dv);
                    GeoPoint locOnAxis = axis.Curve3D.PointAt(axis.Curve3D.PositionOf(loc));
                    double dd = (du ^ dv) * (locOnAxis - loc);
                    if (Math.Sign(dd) == Math.Sign(radius)) res.ReverseOrientation();
                    res.UseEdge(forward);
                    res.UseEdge(backward);
                    return new Face[] { res };
                }
            }
            if (surface == null) return new Face[0];
            // the fillet covers the area between the two curves where the pipe touches the offset faces
            SimpleShape area = FilletArea(chain, surface, radius, out double uSeam);
            if (area == null) return new Face[0];
            List<Face> filletFaces = new List<Face>();
            List<Edge> offsetEdges = new List<Edge>(); // the edges of the offset faces the fillet is connected to
            for (int i = 0; i < chain.Count; i++) { offsetEdges.Add(chain[i].forward); offsetEdges.Add(chain[i].backward); }
            // a pipe which folds over itself is split into several faces, everything else stays a single face
            if (surface is SweptCircle sweptCircle)
            {
                // a closed chain is additionally split where its edges meet: a face which covers a whole period
                // would be glued to itself along the seam, and such a face is hard to handle further on. The
                // vertices of the chain cost nothing, the offset faces have their edges ending there anyway
                List<double> atJunctions = double.IsNaN(uSeam) ? null : ChainJunctions(chain, sweptCircle, uSeam);
                filletFaces.AddRange(sweptCircle.OuterShell(area, atJunctions, out double[] splitPositions));
                List<double> splits = new List<double>(splitPositions);
                if (!double.IsNaN(uSeam)) splits.Add(uSeam); // a closed chain is cut open at the seam
                offsetEdges = SplitOffsetEdges(chain, surface, splits);
            }
            else filletFaces.Add(Face.MakeFace(surface, area));
            // the fillet uses the edges of the offset faces themselves, so that the two are really connected and
            // not only close enough to be connected afterwards
            for (int i = 0; i < filletFaces.Count; i++)
            {
                for (int j = 0; j < offsetEdges.Count; j++) filletFaces[i].UseEdge(offsetEdges[j]);
            }
            for (int i = filletFaces.Count - 1; i >= 0; --i)
            {
                if (filletFaces[i] == null) { filletFaces.RemoveAt(i); continue; }
                foreach (Edge edg in filletFaces[i].Edges)
                {   // where an edge is the projection of one of the curves the area was built from, the exact 3d
                    // curve is used instead of the approximation MakeFace created from the 2d curve: only then it
                    // is close enough to the edge of the offset face to be connected with it
                    if (edg.PrimaryCurve2D is ProjectedCurve pc && pc.Surface == surface && edg.Curve3D != null)
                    {
                        ICurve exact = pc.Curve3DFromParams;
                        if (exact != null && exact.Length > Precision.eps)
                        {
                            edg.Curve3D = exact;
                            edg.Orient();
                        }
                    }
                }
            }
            return filletFaces.ToArray();
        }
        /// <summary>
        /// Groups the edges which get a fillet into chains: two edges belong to the same chain when they meet in a
        /// vertex where no other edge gets a fillet, when they continue each other tangentially there and when the
        /// offset edges on both sides meet there as well. The fillets of such a chain are parts of one and the same
        /// pipe surface. Every other edge forms a chain of its own.
        /// </summary>
        private static List<List<Edge>> FilletChains(Dictionary<Edge, (Edge forward, Edge backward)> filletEdges)
        {
            Dictionary<Vertex, List<Edge>> atVertex = new Dictionary<Vertex, List<Edge>>();
            foreach (Edge edge in filletEdges.Keys)
            {
                foreach (Vertex vtx in new Vertex[] { edge.Vertex1, edge.Vertex2 })
                {
                    if (!atVertex.TryGetValue(vtx, out List<Edge> edges)) atVertex[vtx] = edges = new List<Edge>();
                    if (!edges.Contains(edge)) edges.Add(edge);
                }
            }
            Dictionary<Vertex, (Edge first, Edge second)> connections = new Dictionary<Vertex, (Edge, Edge)>();
            foreach (KeyValuePair<Vertex, List<Edge>> item in atVertex)
            {
                if (item.Value.Count != 2) continue; // a vertex where more edges meet needs a spherical face
                if (ContinuesTangentially(item.Value[0], item.Value[1], item.Key, filletEdges)) connections[item.Key] = (item.Value[0], item.Value[1]);
            }
            HashSet<Edge> used = new HashSet<Edge>();
            List<List<Edge>> res = new List<List<Edge>>();
            foreach (Edge edge in filletEdges.Keys)
            {
                if (used.Contains(edge)) continue;
                List<Edge> chain = new List<Edge> { edge };
                used.Add(edge);
                Vertex at = edge.Vertex2;
                while (connections.TryGetValue(at, out (Edge first, Edge second) pair))
                {
                    Edge next = pair.first == chain[chain.Count - 1] ? pair.second : pair.first;
                    if (used.Contains(next)) break; // the chain is closed
                    chain.Add(next);
                    used.Add(next);
                    at = next.Vertex1 == at ? next.Vertex2 : next.Vertex1;
                }
                at = edge.Vertex1;
                while (connections.TryGetValue(at, out (Edge first, Edge second) pair))
                {
                    Edge previous = pair.first == chain[0] ? pair.second : pair.first;
                    if (used.Contains(previous)) break;
                    chain.Insert(0, previous);
                    used.Add(previous);
                    at = previous.Vertex1 == at ? previous.Vertex2 : previous.Vertex1;
                }
                res.Add(chain);
            }
            return res;
        }

        /// <summary>
        /// Whether the two edges continue each other smoothly in the vertex <paramref name="vtx"/> and their offset
        /// edges meet there: only then their fillets are two parts of one and the same pipe, which can be made in
        /// one piece.
        /// </summary>
        private static bool ContinuesTangentially(Edge e1, Edge e2, Vertex vtx, Dictionary<Edge, (Edge forward, Edge backward)> filletEdges)
        {
            if (e1 == e2 || e1.Curve3D == null || e2.Curve3D == null) return false;
            GeoVector d1 = vtx == e1.Vertex1 ? e1.Curve3D.StartDirection : e1.Curve3D.EndDirection;
            GeoVector d2 = vtx == e2.Vertex1 ? e2.Curve3D.StartDirection : e2.Curve3D.EndDirection;
            if (!Precision.SameDirection(d1, d2, false)) return false;
            // the two offset edges of the one fillet must end where those of the other one start, otherwise the
            // outline of the common fillet would have a gap at this vertex
            (Edge forward, Edge backward) o1 = filletEdges[e1], o2 = filletEdges[e2];
            GeoPoint p11 = CloserEndPoint(o1.forward.Curve3D, vtx.Position), p12 = CloserEndPoint(o1.backward.Curve3D, vtx.Position);
            GeoPoint p21 = CloserEndPoint(o2.forward.Curve3D, vtx.Position), p22 = CloserEndPoint(o2.backward.Curve3D, vtx.Position);
            return (Precision.IsEqual(p11, p21) && Precision.IsEqual(p12, p22))
                || (Precision.IsEqual(p11, p22) && Precision.IsEqual(p12, p21));
        }

        /// <summary>
        /// The pipe surface for a chain of edges: all their curves must be arcs of one and the same ellipse, then
        /// the circle is swept along that ellipse, which is closed when the arcs cover it completely. Returns null
        /// when the edges do not fit together this way; every edge needs its own fillet then.
        /// </summary>
        private static SweptCircle ChainPipeSurface(IReadOnlyList<(Edge axis, Edge forward, Edge backward)> chain, double radius)
        {
            if (!(chain[0].axis.Curve3D is Ellipse first)) return null;
            // a chain of circular arcs is left alone: its fillet is a torus, and the fold of a torus whose minor
            // radius exceeds the major one is a pole, which the toroidal branch resolves on its own
            if (first.IsCircle) return null;
            double covered = 0.0;
            for (int i = 0; i < chain.Count; i++)
            {
                if (!(chain[i].axis.Curve3D is Ellipse elli)) return null;
                if (!Precision.IsEqual(elli.Center, first.Center)) return null;
                if (Math.Abs(elli.MajorRadius - first.MajorRadius) > Precision.eps) return null;
                if (Math.Abs(elli.MinorRadius - first.MinorRadius) > Precision.eps) return null;
                if (!Precision.SameDirection(elli.Plane.Normal, first.Plane.Normal, false)) return null;
                if (!Precision.SameDirection(elli.MajorAxis, first.MajorAxis, false)) return null;
                covered += Math.Abs(elli.SweepParameter);
            }
            Ellipse spine = first.Clone() as Ellipse;
            spine.SweepParameter = Math.Sign(first.SweepParameter) * 2.0 * Math.PI; // the whole ellipse first
            if (covered < 2.0 * Math.PI - 1e-6)
            {   // an open chain: it starts at the vertex of the first edge which the second one does not share and
                // ends at the corresponding vertex of the last edge
                Vertex startVertex = ChainEndVertex(chain, true);
                Vertex endVertex = ChainEndVertex(chain, false);
                spine.StartParameter = spine.StartParameter + spine.PositionOf(startVertex.Position) * spine.SweepParameter;
                if (spine.PositionOf(chain[0].axis.Curve3D.PointAt(0.5)) > 0.5)
                {   // the chain runs against the direction of the first edge
                    spine.SweepParameter = -spine.SweepParameter;
                }
                double endPosition = spine.PositionOf(endVertex.Position);
                if (endPosition < 1e-6 || endPosition > 1.0 - 1e-6) return null; // cannot tell where the chain ends
                spine.SweepParameter = endPosition * spine.SweepParameter;
            }
            return new SweptCircle(spine, Math.Abs(radius));
        }

        /// <summary>
        /// Whether the chain forms a closed loop, i.e. whether every one of its vertices is shared by two of its
        /// edges. A closed chain has no ends where the fillet would be closed by an arc.
        /// </summary>
        private static bool ChainIsClosed(IReadOnlyList<(Edge axis, Edge forward, Edge backward)> chain)
        {
            Dictionary<Vertex, int> count = new Dictionary<Vertex, int>();
            for (int i = 0; i < chain.Count; i++)
            {
                foreach (Vertex vtx in new Vertex[] { chain[i].axis.Vertex1, chain[i].axis.Vertex2 })
                {
                    count.TryGetValue(vtx, out int n);
                    count[vtx] = n + 1;
                }
            }
            foreach (KeyValuePair<Vertex, int> item in count) if (item.Value != 2) return false;
            return true;
        }

        /// <summary>The vertex where the chain starts or ends, i.e. the one which the neighbouring edge does not share.</summary>
        private static Vertex ChainEndVertex(IReadOnlyList<(Edge axis, Edge forward, Edge backward)> chain, bool atStart)
        {
            Edge edge = atStart ? chain[0].axis : chain[chain.Count - 1].axis;
            if (chain.Count == 1) return atStart ? edge.Vertex1 : edge.Vertex2;
            Edge neighbour = atStart ? chain[1].axis : chain[chain.Count - 2].axis;
            if (edge.Vertex1 == neighbour.Vertex1 || edge.Vertex1 == neighbour.Vertex2) return edge.Vertex2;
            return edge.Vertex1;
        }

        /// <summary>
        /// The area in the (u,v) system of <paramref name="surface"/> which the fillet covers: between the two
        /// curves where the pipe touches the offset faces, limited by the arcs around the two ends of the chain.
        /// A closed chain has no ends, its area is cut open at a fold of the surface - where it is split anyway -
        /// and the two sides of the cut become a seam.
        /// </summary>
        private static SimpleShape FilletArea(IReadOnlyList<(Edge axis, Edge forward, Edge backward)> chain, ISurface surface,
            double radius, out double uSeam)
        {
            uSeam = double.NaN;
            BoundingRect domain = BoundingRect.EmptyBoundingRect;
            List<ICurve2D> curves = new List<ICurve2D>();
            void AddProjected(ICurve curve3d, BoundingRect window)
            {
                if (curve3d == null) return;
                ICurve2D c2d;
                if (window.IsEmpty())
                {   // a cylinder or a torus: the curves are brought into a common period as usual
                    c2d = surface.GetProjectedCurve(curve3d, Precision.eps);
                    if (c2d != null && !domain.IsEmpty()) SurfaceHelper.AdjustPeriodic(surface, domain, c2d);
                }
                else
                {   // a pipe: the projection window already places the curve at the right u, which is the parameter
                    // of the spine and unique along the whole chain. Only v is adjusted here, a periodic adjustment
                    // of u would move the curve into the wrong period of a closed spine
                    c2d = new ProjectedCurve(curve3d, surface, true, window);
                    if (c2d != null && !domain.IsEmpty() && surface.IsVPeriodic)
                    {
                        double vm = (domain.Bottom + domain.Top) / 2.0, vPeriod = surface.VPeriod, dv = 0.0;
                        double mv = c2d.PointAt(0.5).y;
                        while (Math.Abs(mv + dv - vm) > Math.Abs(mv + dv - vPeriod - vm)) dv -= vPeriod;
                        while (Math.Abs(mv + dv - vm) > Math.Abs(mv + dv + vPeriod - vm)) dv += vPeriod;
                        if (dv != 0.0) c2d.Move(0.0, dv);
                    }
                }
                if (c2d == null) return;
                if (domain.IsEmpty()) domain = c2d.GetExtent();
                else domain.MinMax(c2d.GetExtent());
                curves.Add(c2d);
            }
            if (!(surface is SweptCircle))
            {   // the domain of a cylindrical or toroidal fillet is fixed by the four corners of the area: the
                // surface needs it to project the following curves into the right period
                domain = new BoundingRect(surface.PositionOf(chain[0].forward.Curve3D.PointAt(0.5)));
                foreach (Edge offsetEdge in new Edge[] { chain[0].forward, chain[0].backward })
                {
                    foreach (GeoPoint p in new GeoPoint[] { offsetEdge.Curve3D.StartPoint, offsetEdge.Curve3D.EndPoint })
                    {
                        GeoPoint2D uv = surface.PositionOf(p);
                        SurfaceHelper.AdjustPeriodic(surface, domain, ref uv);
                        domain.MinMax(uv);
                    }
                }
                surface.Domain = domain;
            }
            for (int i = 0; i < chain.Count; i++)
            {
                AddProjected(chain[i].forward.Curve3D, ProjectionWindow(surface, chain[i].axis, chain[i].forward.Curve3D));
                AddProjected(chain[i].backward.Curve3D, ProjectionWindow(surface, chain[i].axis, chain[i].backward.Curve3D));
            }
            if (!ChainIsClosed(chain))
            {
                Ellipse startArc = EndArc(chain[0], ChainEndVertex(chain, true), radius);
                Ellipse endArc = EndArc(chain[chain.Count - 1], ChainEndVertex(chain, false), radius);
                AddProjected(startArc, ProjectionWindow(surface, chain[0].axis, startArc));
                AddProjected(endArc, ProjectionWindow(surface, chain[chain.Count - 1].axis, endArc));
                return MakeSimpleShape(curves);
            }
            // the chain is closed: everything is moved into one period [uSeam, uSeam+1]. The seam is put at a fold
            // of the surface, because there the surface is split anyway, and only then the two sides of the double
            // curve end up on two different faces, which can be sewn together along it
            uSeam = 0.0;
            if (surface is SweptCircle sweptCircle)
            {
                (double uVertex, ICurve2D ascending, ICurve2D descending)[] branches = sweptCircle.GetSelfIntersectionBranches(domain);
                if (branches.Length > 0) uSeam = branches[0].uVertex;
            }
            List<ICurve2D> moved = new List<ICurve2D>();
            for (int i = 0; i < curves.Count; i++) PlaceInPeriod(curves[i], uSeam, true, moved);
            // the two ends of the area are closed with the circle of the pipe at the seam
            List<GeoPoint2D> atSeam = new List<GeoPoint2D>(), atPeriod = new List<GeoPoint2D>();
            for (int i = 0; i < moved.Count; i++)
            {
                CollectSeamPoint(moved[i].StartPoint, uSeam, atSeam, atPeriod);
                CollectSeamPoint(moved[i].EndPoint, uSeam, atSeam, atPeriod);
            }
            if (atSeam.Count != 2 || atPeriod.Count != 2) return null; // the curves do not cover the period as expected
            moved.Add(new Line2D(atSeam[0], atSeam[1]));
            moved.Add(new Line2D(atPeriod[0], atPeriod[1]));
            return MakeSimpleShape(moved);
        }

        /// <summary>
        /// The parameters where the edges of a closed chain meet, expressed in the period which starts at
        /// <paramref name="uSeam"/>. The seam itself is not among them, it is the border of the area anyway.
        /// </summary>
        private static List<double> ChainJunctions(IReadOnlyList<(Edge axis, Edge forward, Edge backward)> chain,
            SweptCircle surface, double uSeam)
        {
            List<double> res = new List<double>();
            double period = surface.UPeriod;
            for (int i = 0; i < chain.Count; i++)
            {
                if (chain[i].axis.Curve3D == null) continue;
                double u = surface.Spine.PositionOf(chain[i].axis.Curve3D.StartPoint);
                u -= Math.Floor((u - uSeam) / period) * period; // into the period which starts at the seam
                if (u > uSeam + 1e-6 && u < uSeam + period - 1e-6) res.Add(u);
            }
            return res;
        }

        /// <summary>
        /// Splits the edges of the offset faces where the fillet has been split at a fold of the pipe (and at the
        /// seam of a closed chain): the fillet is connected to those faces along its whole length, so wherever it
        /// ends, its neighbour has to end as well, otherwise the two cannot be sewn together.
        /// </summary>
        private static List<Edge> SplitOffsetEdges(IReadOnlyList<(Edge axis, Edge forward, Edge backward)> chain, ISurface surface,
            List<double> splitPositions)
        {
            List<Edge> res = new List<Edge>();
            for (int i = 0; i < chain.Count; i++) { res.Add(chain[i].forward); res.Add(chain[i].backward); }
            if (splitPositions.Count == 0) return res;
            res.Clear();
            for (int i = 0; i < chain.Count; i++)
            {
                foreach (Edge offsetEdge in new Edge[] { chain[i].forward, chain[i].backward })
                {
                    res.Add(offsetEdge);
                    if (offsetEdge.Curve3D == null) continue;
                    BoundingRect window = ProjectionWindow(surface, chain[i].axis, offsetEdge.Curve3D);
                    ICurve2D c2d = window.IsEmpty() ? surface.GetProjectedCurve(offsetEdge.Curve3D, Precision.eps)
                        : new ProjectedCurve(offsetEdge.Curve3D, surface, true, window);
                    if (c2d == null) continue;
                    BoundingRect ext = c2d.GetExtent();
                    SortedList<double, Vertex> splitHere = new SortedList<double, Vertex>();
                    for (int j = 0; j < splitPositions.Count; j++)
                    {
                        for (int period = -1; period <= 1; ++period)
                        {
                            if (period != 0 && !surface.IsUPeriodic) continue;
                            double u = splitPositions[j] + period * (surface.IsUPeriodic ? surface.UPeriod : 0.0);
                            if (u <= ext.Left + 1e-6 || u >= ext.Right - 1e-6) continue;
                            // the point where the fillet ends, expressed on the edge of the offset face
                            GeoPoint p = surface.PointAt(c2d.PointAt(ParameterAtU(c2d, u)));
                            double position = offsetEdge.Curve3D.PositionOf(p);
                            if (position <= 1e-6 || position >= 1.0 - 1e-6) continue;
                            splitHere[position] = new Vertex(offsetEdge.Curve3D.PointAt(position));
                        }
                    }
                    if (splitHere.Count > 0)
                    {
                        res.RemoveAt(res.Count - 1);
                        res.AddRange(offsetEdge.Split(splitHere, Precision.eps));
                    }
                }
            }
            return res;
        }

        /// <summary>
        /// The part of the (u,v) system where the curves along one edge of the chain are expected. On a closed
        /// spine the projection of a curve which comes close to the seam of the parameter range would otherwise
        /// jump from one period into the other and the resulting 2d curve would be useless. Empty (i.e. no hint
        /// needed) when the surface is not periodic in u.
        /// </summary>
        private static BoundingRect ProjectionWindow(ISurface surface, Edge axis, ICurve toProject)
        {
            if (!surface.IsUPeriodic || !(surface is SweptCircle sweptCircle) || axis.Curve3D == null || toProject == null)
            {
                return BoundingRect.EmptyBoundingRect;
            }
            double length = sweptCircle.Spine.Length;
            if (length < Precision.eps) return BoundingRect.EmptyBoundingRect;
            // the middle of the edge is far away from the seam, so its parameter can be determined safely. Only the
            // center of the window is relevant: the projection maps every point into the period closest to it
            double period = surface.UPeriod;
            double uMiddle = sweptCircle.Spine.PositionOf(axis.Curve3D.PointAt(0.5));
            uMiddle -= Math.Floor(uMiddle / period) * period;
            double halfWidth = 0.5 * axis.Curve3D.Length / length * period + 0.01 * period;
            double vMiddle = surface.PositionOf(toProject.PointAt(0.5)).y;
            return new BoundingRect(uMiddle - halfWidth, vMiddle - 0.24 * surface.VPeriod,
                uMiddle + halfWidth, vMiddle + 0.24 * surface.VPeriod);
        }

        private static SimpleShape MakeSimpleShape(List<ICurve2D> curves)
        {
            if (curves.Count < 3) return null;
            List<ICurve2D> loop = SortToLoop(curves);
            if (loop == null) return null;
            Border bdr = Border.FromUnorientedList(loop.ToArray(), true);
            if (bdr == null || bdr.Area < Precision.eps) return null;
            return new SimpleShape(bdr);
        }

        /// <summary>
        /// Brings the curves into the order in which they form a closed loop, reversing them where necessary:
        /// starting with the first one, the curve whose end point is closest is appended. Returns null when the
        /// loop does not close, i.e. when the curves do not describe a single closed outline.
        /// </summary>
        private static List<ICurve2D> SortToLoop(List<ICurve2D> curves)
        {
            BoundingRect ext = BoundingRect.EmptyBoundingRect;
            for (int i = 0; i < curves.Count; i++) ext.MinMax(curves[i].GetExtent());
            double maxGap = 1e-4 * Math.Max(ext.Width, ext.Height);
            List<ICurve2D> sorted = new List<ICurve2D>(curves.Count) { curves[0] };
            List<ICurve2D> rest = new List<ICurve2D>(curves.GetRange(1, curves.Count - 1));
            while (rest.Count > 0)
            {
                GeoPoint2D at = sorted[sorted.Count - 1].EndPoint;
                int best = -1;
                bool reverse = false;
                double bestDistance = double.MaxValue;
                for (int i = 0; i < rest.Count; i++)
                {
                    if ((at | rest[i].StartPoint) < bestDistance) { bestDistance = at | rest[i].StartPoint; best = i; reverse = false; }
                    if ((at | rest[i].EndPoint) < bestDistance) { bestDistance = at | rest[i].EndPoint; best = i; reverse = true; }
                }
                ICurve2D next = rest[best];
                rest.RemoveAt(best);
                if (reverse) next.Reverse();
                sorted.Add(next);
            }
            if ((sorted[sorted.Count - 1].EndPoint | sorted[0].StartPoint) > maxGap) return null;
            return sorted;
        }

        /// <summary>
        /// Moves a 2d curve into the period [uSeam, uSeam+1] of a closed pipe surface, splitting it when it
        /// contains the seam.
        /// </summary>
        private static void PlaceInPeriod(ICurve2D curve, double uSeam, bool maySplit, List<ICurve2D> res)
        {
            BoundingRect ext = curve.GetExtent();
            if (ext.Right <= uSeam + 1e-6) res.Add(MovedByPeriod(curve, 1.0));
            else if (ext.Left >= uSeam - 1e-6 || !maySplit) res.Add(curve);
            else
            {
                ICurve2D[] parts = curve.Split(ParameterAtU(curve, uSeam));
                if (parts == null || parts.Length != 2) res.Add(curve);
                else
                {
                    PlaceInPeriod(parts[0], uSeam, false, res);
                    PlaceInPeriod(parts[1], uSeam, false, res);
                }
            }
        }

        /// <summary>
        /// Moves a 2d curve by whole periods in u. Move is used and not GetModified, because a
        /// <see cref="ProjectedCurve"/> can only be moved by whole periods (it stays a projected curve then),
        /// whereas GetModified would replace it by a polygonal approximation, which would show up as a lot of tiny
        /// edges in the resulting face.
        /// </summary>
        private static ICurve2D MovedByPeriod(ICurve2D curve, double periods)
        {
            ICurve2D res = curve.Clone();
            try
            {
                res.Move(periods, 0.0);
            }
            catch (ApplicationException)
            {
                res = curve.GetModified(ModOp2D.Translate(periods, 0.0));
            }
            return res;
        }

        /// <summary>Collects the (unique) points at the two ends of the period, they are connected by the seam.</summary>
        private static void CollectSeamPoint(GeoPoint2D p, double uSeam, List<GeoPoint2D> atSeam, List<GeoPoint2D> atPeriod)
        {
            List<GeoPoint2D> list = null;
            if (Math.Abs(p.x - uSeam) < 1e-6) list = atSeam;
            else if (Math.Abs(p.x - uSeam - 1.0) < 1e-6) list = atPeriod;
            if (list == null) return;
            for (int i = 0; i < list.Count; i++) if ((p | list[i]) < 1e-6) return;
            list.Add(p);
        }

        /// <summary>The parameter of a 2d curve which is monotonous in u at the given u value.</summary>
        private static double ParameterAtU(ICurve2D curve, double u)
        {
            double lo = 0.0, hi = 1.0;
            bool ascending = curve.EndPoint.x > curve.StartPoint.x;
            for (int i = 0; i < 60; i++)
            {
                double m = (lo + hi) / 2.0;
                if (curve.PointAt(m).x < u == ascending) lo = m;
                else hi = m;
            }
            return (lo + hi) / 2.0;
        }

        /// <summary>
        /// The arc of the pipe around the end of the chain: it connects the ends of the two offset edges and is
        /// also a part of the spherical face which fills the gap at this vertex.
        /// </summary>
        private static Ellipse EndArc((Edge axis, Edge forward, Edge backward) item, Vertex vtx, double radius)
        {
            Edge axis = item.axis;
            GeoPoint center = vtx.Position;
            GeoVector toOutside = axis.PrimaryFace.Surface.GetNormal(vtx.GetPositionOnFace(axis.PrimaryFace)).Normalized +
                                  axis.SecondaryFace.Surface.GetNormal(vtx.GetPositionOnFace(axis.SecondaryFace)).Normalized;
            Plane pln = new Plane(center, vtx == axis.Vertex1 ? axis.Curve3D.StartDirection : axis.Curve3D.EndDirection);
            GeoPoint p1 = CloserEndPoint(item.forward.Curve3D, center);
            GeoPoint p2 = CloserEndPoint(item.backward.Curve3D, center);
            if (Precision.IsEqual(p1, p2)) return null; // the two offset edges meet in a single point
            Ellipse arc = Ellipse.Construct();
            arc.SetArc3Points(p1, center + radius * toOutside.Normalized, p2, pln);
            if (Math.Abs(arc.SweepParameter) > Math.PI) arc.SetArc3Points(p1, center - radius * toOutside.Normalized, p2, pln);
            return arc;
        }

        private static GeoPoint CloserEndPoint(ICurve curve, GeoPoint p)
        {
            return (curve.StartPoint | p) < (curve.EndPoint | p) ? curve.StartPoint : curve.EndPoint;
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
        /// <summary>
        /// The 2d curve of <paramref name="edge"/> for the offset of <paramref name="face"/>: the curve it has on
        /// the original face, moved into the (u,v) system of the offset surface. For every surface except the cone
        /// that system is the same one and <paramref name="toOffsetUv"/> is the identity.
        /// </summary>
        private static ICurve2D OffsetCurve2D(Edge edge, Face face, ModOp2D toOffsetUv)
        {
            ICurve2D c2d = edge.Curve2D(face).Clone();
            if (c2d is InterpolatedDualSurfaceCurve.ProjectedCurve pc) c2d = pc.ToBSpline(0.0);
            if (c2d is Path2D)
            {   // sine curve is not maintained (14.6.25) but was converted to Path2D
                c2d = face.Surface.GetProjectedCurve(edge.Curve3D, 0.0);
                if (!edge.Forward(face)) c2d.Reverse();
            }
            if (!toOffsetUv.IsIdentity) c2d = c2d.GetModified(toOffsetUv);
            c2d.UserData.Add("CADability.CurveToEdge", edge);
            return c2d;
        }
        public static Shell[] GetOffset(this Shell shell, double offset)
        {
            Face[] faces = shell.GetOffsetParts(offset, out bool allEdgesAreConnected);
            if (allEdgesAreConnected)
            {
                Shell s = Shell.MakeShell(faces);
                if (!s.HasOpenEdgesExceptPoles())
                    return new Shell[] { s }; // the offset is a perfectly closed shell, no need to do a boolean operation
            }
            BooleanOperation bo = new BooleanOperation();
            bo.SetFaces(faces, offset > 0);
            return bo.Execute();
        }

        /// <summary>
        /// The faces the offset of this shell is made of: the offset of every face of the shell, a fillet along
        /// every convex edge (concave for a negative offset) and a spherical patch at every vertex where three or
        /// more of those fillets meet. The faces are already connected where they share an edge, but parts which
        /// stand out still have to be trimmed against each other, which is what <see cref="GetOffset"/> does with a
        /// boolean operation afterwards. Exposed separately because it is the part of the offset which can be
        /// tested and looked at on its own.
        /// </summary>
        /// <param name="offset">the distance, positive to the outside</param>
        /// <param name="allEdgesAreConnected">true when every edge of the shell got a fillet or needs none, i.e.
        /// when the faces should already form a closed shell</param>
        public static Face[] GetOffsetParts(this Shell shell, double offset, out bool allEdgesAreConnected)
        {
            Dictionary<(Face, Edge), Edge> faceEdgeToParallelEdge = new Dictionary<(Face, Edge), Edge>(); // the parallel edges to the original edges, also depend on the face
            Dictionary<Vertex, List<Edge>> vertexToArcs = new Dictionary<Vertex, List<Edge>>(); // for each vertex there are the sides of the wedges, which build spherical wedges
            Dictionary<Face, Face> faceToOffsetFace = new Dictionary<Face, Face>(); // for each face of the original shell we have a face with the required offset here
            List<Face> filletFaces = new List<Face>(); // the "fillet" faces along the convex edges
            HashSet<Face> inverseFaces = new HashSet<Face>();
            List<Face> sphercalWegdes = new List<Face>(); // the sphreical faces fill the gaps between the fillets
                                                          // for the brep operation we need edge-face pairs, which should not be tested for intersection, because they connect adjacent parts like the offset faces with the fillets
                                                          // or the fillets with the spherical faces. Since each breop operation creates new faces and edges, we attach this information to the user data of the original parts
                                                          // and retrieve them after the brep operation is done.
            HashSet<(Edge, Face)> dontIntersect = new HashSet<(Edge, Face)>(); // NOT USED ANY MORE, these pairs will be connected and there is no need to calculate the intersection
                                                                               // set UserData with the original face and edge references to 
            DebuggerContainer dc = new DebuggerContainer();
            foreach (Face face in shell.Faces)
            {   // makeparallel faces with the provided offset
                // The offset surface usually uses the same (u,v) system as the original one, so the 2d curves of
                // the face can be taken over unchanged. The cone is the exception: its v is shifted by the offset,
                // and GetOffsetSurface tells by how much. Same pattern as Face.GetOffsetFace.
                ModOp2D toOffsetUv = ModOp2D.Identity;
                ISurface offsetSurface;
                if (face.Surface is ConicalSurface conicalSurface) offsetSurface = conicalSurface.GetOffsetSurface(offset, out toOffsetUv);
                else offsetSurface = face.Surface.GetOffsetSurface(offset);
                if (offsetSurface == null) continue; // a sphere, cylinder or torus shrinking to 0
                GeoPoint2D cnt = face.Domain.GetCenter();
                // if the orentation is reversed (e.g. a cylinder will have a negativ radius) or the surface disappears, don't use it
                if (offsetSurface != null) // 
                {
                    List<ICurve2D> outline = new List<ICurve2D>();
                    foreach (Edge edge in face.OutlineEdges) outline.Add(OffsetCurve2D(edge, face, toOffsetUv));
                    Border outlineBorder = new Border(outline.ToArray(), true);
                    List<Border> holes = new List<Border>();
                    for (int i = 0; i < face.HoleCount; i++)
                    {
                        List<ICurve2D> hole = new List<ICurve2D>();
                        foreach (Edge edge in face.HoleEdges(i)) hole.Add(OffsetCurve2D(edge, face, toOffsetUv));
                        Border holeBorder = new Border(hole.ToArray(), true);
                        holes.Add(holeBorder);
                    }
                    SimpleShape ss = new SimpleShape(outlineBorder, holes.ToArray());
                    Face offsetFace = Face.MakeFace(offsetSurface, ss);
                    offsetFace.UserData.Add("ShapeIt.OriginalFace", new FaceReference(offsetFace));
#if DEBUG
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
                    if (offsetSurface.GetNormal(toOffsetUv * cnt) * face.Surface.GetNormal(cnt) < 0)
                    {   // this face is reversed, e.g. a cylinder, which now has negative radius
                        // we still need to create it, because we need the edges for the fillets, but we do not use it in the result
                        inverseFaces.Add(offsetFace);
                    }
                }
            }

            // the edges which need a fillet: convex edges (concave for an inner offset) whose two offset faces exist
            allEdgesAreConnected = true;
            Dictionary<Edge, (Edge forward, Edge backward)> filletEdges = new Dictionary<Edge, (Edge, Edge)>();
            foreach (Edge sedge in shell.Edges)
            {
                AdjacencyType toCheckFor = offset > 0 ? AdjacencyType.Convex : AdjacencyType.Concave;
                if (sedge.Adjacency() == toCheckFor && faceEdgeToParallelEdge.TryGetValue((sedge.PrimaryFace, sedge), out Edge e1)
                    && faceEdgeToParallelEdge.TryGetValue((sedge.SecondaryFace, sedge), out Edge e2))
                {
                    filletEdges[sedge] = (e1, e2);
                }
                else if (sedge.Adjacency() != AdjacencyType.SameSurface && sedge.Adjacency() != AdjacencyType.Tangent) allEdgesAreConnected = false;
            }
            // edges which continue each other tangentially share a single fillet: only then the pipe which folds
            // over itself at a tightly curved edge can be resolved, because both sheets belong to the same surface
            foreach (List<Edge> chain in FilletChains(filletEdges))
            {
                List<(Edge axis, Edge forward, Edge backward)> withOffsetEdges = new List<(Edge, Edge, Edge)>(chain.Count);
                for (int i = 0; i < chain.Count; i++) withOffsetEdges.Add((chain[i], filletEdges[chain[i]].forward, filletEdges[chain[i]].backward));
                Face[] fillets = MakeOffsetFillet(withOffsetEdges, offset,
                    inverseFaces.Contains(filletEdges[chain[0]].forward.PrimaryFace), inverseFaces.Contains(filletEdges[chain[0]].backward.PrimaryFace));
                if (fillets == null) continue;
                // only where the chain ends several fillets meet in a vertex and a spherical patch is needed: at
                // the junctions inside a chain the fillet simply continues, and a closed chain has no ends at all
                HashSet<Vertex> chainVertices = new HashSet<Vertex>();
                if (!ChainIsClosed(withOffsetEdges))
                {
                    chainVertices.Add(ChainEndVertex(withOffsetEdges, true));
                    chainVertices.Add(ChainEndVertex(withOffsetEdges, false));
                }
                foreach (Face fillet in fillets)
                {
                    if (fillet == null) continue;
                    fillet.UserData.Add("ShapeIt.OriginalFace", new FaceReference(fillet));
                    filletFaces.Add(fillet);
                    foreach (Edge edg in fillet.AllEdges)
                    {
                        if (edg.Curve3D is IGeoObject go && !go.UserData.ContainsData("ShapeIt.OriginalEdge"))
                        {
                            go.UserData.Add("ShapeIt.OriginalEdge", new EdgeReference(edg));
                        }
                        // the arcs at the ends of the chain are the sides of the spherical faces at those vertices
                        if (edg.Curve3D is Ellipse elli && elli.IsCircle && Math.Abs(elli.Radius - Math.Abs(offset)) < Precision.eps)
                        {
                            foreach (Vertex vtx in chainVertices)
                            {
                                if (!Precision.IsEqual(elli.Center, vtx.Position)) continue;
                                if (!vertexToArcs.TryGetValue(vtx, out List<Edge> arcs)) vertexToArcs[vtx] = arcs = new List<Edge>();
                                if (!arcs.Contains(edg)) arcs.Add(edg);
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
                foreach (Face face in vtx.Faces.Intersect(shell.Faces)) // only use faces of this shell
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
            faces.UnionWith(filletFaces); // the fillets on the edges
            faces.UnionWith(sphercalWegdes); // the spherical faces on the vertices
                                             // faces contains all the faces for the offset shell, the edges are properly connected but some parts are standing out
            Face[] res = faces.ToArray();
            // the parts are built independently and only meet exactly in theory: where a fillet has been split at a
            // fold, the 2d operations which cut it leave the common point a few 1e-6 apart, so the tolerance for
            // connecting them has to be relative to the size of the shell
            BoundingBox ext = shell.GetExtent(0.0);
            Shell.ConnectFaces(res, Math.Max(Precision.eps, ext.Size * 1e-6));
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
                topOffset.Domain = edgeToRound.PrimaryFace.Domain;
                bottomOffset.Domain = edgeToRound.SecondaryFace.Domain;
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
                sweptCircle.Domain = new BoundingRect(0, Math.PI / 2, 1, 3 * Math.PI / 2);
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
                dbgfc.GetTriangulation(0.01, out GeoPoint[] trianglePoint, out GeoPoint2D[] triangleUVPoint, out int[] triangleIndex, out BoundingBox triangleExtent);
                GeoObjectList dbgtr = new GeoObjectList();
                for (int i = 0; i < triangleIndex.Length; i += 3)
                {
                    dbgtr.Add(Line.MakeLine(trianglePoint[triangleIndex[i]], trianglePoint[triangleIndex[i + 1]]));
                    dbgtr.Add(Line.MakeLine(trianglePoint[triangleIndex[i + 1]], trianglePoint[triangleIndex[i + 2]]));
                    dbgtr.Add(Line.MakeLine(trianglePoint[triangleIndex[i + 2]], trianglePoint[triangleIndex[i]]));
                }
                // end of the provisional phase, see the remarks on ISurface.Domain
                sweptCircle.Domain = BoundingRect.EmptyBoundingRect;
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
            topOffset.Domain = edgeToRound.PrimaryFace.Domain;
            bottomOffset.Domain = edgeToRound.SecondaryFace.Domain;
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
            sweptCircle.Domain = new BoundingRect(0, Math.PI / 2, 1, 3 * Math.PI / 2);
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
        public enum EdgeConnection { Convex, Concave, Tangential };
        public static Shell? RoundEdges(this Shell shell, IEnumerable<Edge> edges, double radius)
        {
            RoundEdges re = new RoundEdges(shell, edges, radius);
            return re.Execute();
        }
        public static Shell? ChamferEdges(this Shell shell, IEnumerable<Edge> edges, double length1, double length2)
        {
            ChamferEdges re = new ChamferEdges(shell, edges, length1, length2);
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
                    foreach (Vertex vtx in new List<Vertex>([edgeToRound.Vertex1, edgeToRound.Vertex1]))
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
                                    dbg2d.Add(arcEdgeOn1.Curve2D(torusFace), Color.Red, arcEdgeOn1.GetHashCode());
                                    dbg2d.Add(arcEdgeOn2.Curve2D(torusFace), Color.Red, arcEdgeOn2.GetHashCode());
                                    dbg2d.Add(tangentialEdge.Curve2D(torusFace), Color.Red, tangentialEdge.GetHashCode());
                                    dbg2d.Add(thirdEdgeSegment.Curve2D(torusFace), Color.Red, thirdEdgeSegment.GetHashCode());
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
                                    dc.Add(sphericalPatch, Color.Green, sphericalPatch.GetHashCode());
                                    dc.Add(clippedFillets[0], Color.Red, clippedFillets[0].GetHashCode());
                                    dc.Add(clippedFillets[1], Color.Red, clippedFillets[1].GetHashCode());
                                    dc.Add(clippedFillets[2], Color.Red, clippedFillets[2].GetHashCode());
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

        public static IEnumerable<Face> AllFacesWithUserData(this Shell shell, string userDataKey)
        {
            return shell.Faces.Where(f => f.UserData.ContainsData(userDataKey));
        }
    }
}
