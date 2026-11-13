using CADability.GeoObject;
using System;

namespace CADability
{
    /// <summary>
    /// Newton's method for the common points of surfaces: of a plane and two surfaces, the basic step of tracing an
    /// intersection curve (marching, <see cref="InterpolatedDualSurfaceCurve"/>), and of three surfaces, the vertex where
    /// an intersection curve meets a third surface.
    /// <para>
    /// The unknowns are the uv parameters on the surfaces, the equations say that the surface points coincide (and lie
    /// in the plane): square systems of 4 or 6 equations. The Levenberg-Marquardt solvers used so far treated a plane as
    /// a surface with parameters of its own, had more residuals than unknowns (9 for 6) and allocate vectors, matrices
    /// and delegates on every call. Here everything is on the stack, a good start value converges in two or three steps
    /// and a failure is a return value, not an exception.
    /// </para>
    /// <para>
    /// Where surfaces touch the Jacobian is (almost) singular and Newton's method is not reliable: then it gives up, and
    /// the caller should use the damped Levenberg-Marquardt solver.
    /// </para>
    /// </summary>
    public static class SurfaceIntersectionNewton
    {
        /// <summary>
        /// The point where <paramref name="plane"/> intersects the curve of intersection of <paramref name="surface1"/>
        /// and <paramref name="surface2"/>, starting from <paramref name="uv1"/> and <paramref name="uv2"/>.
        /// </summary>
        /// <param name="plane">the plane, usually perpendicular to the intersection curve</param>
        /// <param name="surface1">the first surface</param>
        /// <param name="surface2">the second surface</param>
        /// <param name="uv1">start value and result on the first surface</param>
        /// <param name="uv2">start value and result on the second surface</param>
        /// <param name="ip">the intersection point, the middle of the points of the two surfaces</param>
        /// <param name="maxIterations">the maximum number of Newton steps</param>
        /// <returns>true, when the point has been found; the uv values are unchanged otherwise</returns>
        public static bool PlaneAndSurfaces(Plane plane, ISurface surface1, ISurface surface2,
            ref GeoPoint2D uv1, ref GeoPoint2D uv2, out GeoPoint ip, int maxIterations = 12)
        {
            ip = GeoPoint.Origin;
            GeoVector n = plane.Normal;
            if (n.IsNullVector()) return false;
            n.Norm();
            PlaneAndTwoSurfaces system = new PlaneAndTwoSurfaces(plane.Location, n, surface1, surface2);
            Span<double> x = stackalloc double[4] { uv1.x, uv1.y, uv2.x, uv2.y };
            if (!Solve(ref system, x, 4, Scale(plane.Location), maxIterations)) return false;
            uv1 = new GeoPoint2D(x[0], x[1]);
            uv2 = new GeoPoint2D(x[2], x[3]);
            ip = new GeoPoint(surface1.PointAt(uv1), surface2.PointAt(uv2));
            return true;
        }

        /// <summary>
        /// The common point of <paramref name="surface1"/>, <paramref name="surface2"/> and <paramref name="surface3"/>,
        /// starting from <paramref name="uv1"/>, <paramref name="uv2"/> and <paramref name="uv3"/>.
        /// </summary>
        /// <param name="ip">the intersection point, the center of the points of the three surfaces</param>
        /// <param name="maxIterations">the maximum number of Newton steps</param>
        /// <returns>true, when the point has been found; the uv values are unchanged otherwise</returns>
        public static bool ThreeSurfaces(ISurface surface1, ISurface surface2, ISurface surface3,
            ref GeoPoint2D uv1, ref GeoPoint2D uv2, ref GeoPoint2D uv3, out GeoPoint ip, int maxIterations = 12)
        {
            ip = GeoPoint.Origin;
            ThreeSurfacesSystem system = new ThreeSurfacesSystem(surface1, surface2, surface3);
            Span<double> x = stackalloc double[6] { uv1.x, uv1.y, uv2.x, uv2.y, uv3.x, uv3.y };
            if (!Solve(ref system, x, 6, Scale(surface1.PointAt(uv1)), maxIterations)) return false;
            uv1 = new GeoPoint2D(x[0], x[1]);
            uv2 = new GeoPoint2D(x[2], x[3]);
            uv3 = new GeoPoint2D(x[4], x[5]);
            ip = new GeoPoint(surface1.PointAt(uv1), surface2.PointAt(uv2), surface3.PointAt(uv3));
            return true;
        }

        /// <summary>
        /// <see cref="ThreeSurfaces"/>, and where it gives up the Levenberg-Marquardt solver
        /// <see cref="BoxedSurfaceExtension.SurfacesIntersectionLM(ISurface, ISurface, ISurface, ref GeoPoint2D, ref GeoPoint2D, ref GeoPoint2D, ref GeoPoint)"/>,
        /// with the same parameters as that.
        /// </summary>
        public static bool ThreeSurfacesOrLM(ISurface surface1, ISurface surface2, ISurface surface3,
            ref GeoPoint2D uv1, ref GeoPoint2D uv2, ref GeoPoint2D uv3, ref GeoPoint ip)
        {
            if (ThreeSurfaces(surface1, surface2, surface3, ref uv1, ref uv2, ref uv3, out GeoPoint found))
            {
                ip = found;
                return true;
            }
            return BoxedSurfaceExtension.SurfacesIntersectionLM(surface1, surface2, surface3, ref uv1, ref uv2, ref uv3, ref ip);
        }

        /// <summary>The achievable precision is limited by the magnitude of the coordinates.</summary>
        private static double Scale(GeoPoint p)
        {
            return 1.0 + Math.Max(Math.Abs(p.x), Math.Max(Math.Abs(p.y), Math.Abs(p.z)));
        }

        /// <summary>
        /// A square system of nonlinear equations: the residual and its Jacobian at a point.
        /// </summary>
        private interface ISystem
        {
            /// <summary>
            /// The residual <paramref name="f"/> and its Jacobian <paramref name="jac"/> (row major) at <paramref name="x"/>,
            /// returns the norm of the residual (NaN, when it cannot be evaluated there).
            /// </summary>
            double Evaluate(ReadOnlySpan<double> x, Span<double> f, Span<double> jac);
        }

        private readonly struct PlaneAndTwoSurfaces : ISystem
        {
            private readonly GeoPoint location;
            private readonly GeoVector normal;
            private readonly ISurface surface1, surface2;
            public PlaneAndTwoSurfaces(GeoPoint location, GeoVector normal, ISurface surface1, ISurface surface2)
            {
                this.location = location;
                this.normal = normal;
                this.surface1 = surface1;
                this.surface2 = surface2;
            }
            public double Evaluate(ReadOnlySpan<double> x, Span<double> f, Span<double> jac)
            {
                surface1.DerivativeAt(new GeoPoint2D(x[0], x[1]), out GeoPoint p1, out GeoVector du1, out GeoVector dv1);
                surface2.DerivativeAt(new GeoPoint2D(x[2], x[3]), out GeoPoint p2, out GeoVector du2, out GeoVector dv2);
                GeoVector n = normal;
                // S1 - S2 = 0 and the middle of both points in the plane
                f[0] = p1.x - p2.x;
                f[1] = p1.y - p2.y;
                f[2] = p1.z - p2.z;
                f[3] = 0.5 * (n * (p1 - location) + n * (p2 - location));
                jac[0] = du1.x; jac[1] = dv1.x; jac[2] = -du2.x; jac[3] = -dv2.x;
                jac[4] = du1.y; jac[5] = dv1.y; jac[6] = -du2.y; jac[7] = -dv2.y;
                jac[8] = du1.z; jac[9] = dv1.z; jac[10] = -du2.z; jac[11] = -dv2.z;
                jac[12] = 0.5 * (n * du1); jac[13] = 0.5 * (n * dv1); jac[14] = 0.5 * (n * du2); jac[15] = 0.5 * (n * dv2);
                return Norm(f);
            }
        }

        private readonly struct ThreeSurfacesSystem : ISystem
        {
            private readonly ISurface surface1, surface2, surface3;
            public ThreeSurfacesSystem(ISurface surface1, ISurface surface2, ISurface surface3)
            {
                this.surface1 = surface1;
                this.surface2 = surface2;
                this.surface3 = surface3;
            }
            public double Evaluate(ReadOnlySpan<double> x, Span<double> f, Span<double> jac)
            {
                surface1.DerivativeAt(new GeoPoint2D(x[0], x[1]), out GeoPoint p1, out GeoVector du1, out GeoVector dv1);
                surface2.DerivativeAt(new GeoPoint2D(x[2], x[3]), out GeoPoint p2, out GeoVector du2, out GeoVector dv2);
                surface3.DerivativeAt(new GeoPoint2D(x[4], x[5]), out GeoPoint p3, out GeoVector du3, out GeoVector dv3);
                // S1 - S2 = 0 and S2 - S3 = 0
                f[0] = p1.x - p2.x;
                f[1] = p1.y - p2.y;
                f[2] = p1.z - p2.z;
                f[3] = p2.x - p3.x;
                f[4] = p2.y - p3.y;
                f[5] = p2.z - p3.z;
                jac.Clear();
                jac[0] = du1.x; jac[1] = dv1.x; jac[2] = -du2.x; jac[3] = -dv2.x;
                jac[6] = du1.y; jac[7] = dv1.y; jac[8] = -du2.y; jac[9] = -dv2.y;
                jac[12] = du1.z; jac[13] = dv1.z; jac[14] = -du2.z; jac[15] = -dv2.z;
                jac[20] = du2.x; jac[21] = dv2.x; jac[22] = -du3.x; jac[23] = -dv3.x;
                jac[26] = du2.y; jac[27] = dv2.y; jac[28] = -du3.y; jac[29] = -dv3.y;
                jac[32] = du2.z; jac[33] = dv2.z; jac[34] = -du3.z; jac[35] = -dv3.z;
                return Norm(f);
            }
        }

        private static double Norm(ReadOnlySpan<double> f)
        {
            double sum = 0.0;
            for (int i = 0; i < f.Length; i++) sum += f[i] * f[i];
            double norm = Math.Sqrt(sum);
            return double.IsNaN(norm) || double.IsInfinity(norm) ? double.NaN : norm; // netstandard2.0 has no IsFinite
        }

        /// <summary>
        /// Damped Newton iterations for the <paramref name="n"/> x <paramref name="n"/> <paramref name="system"/> from
        /// <paramref name="x"/>, which becomes the solution. A step is halved until the residual decreases. Converged,
        /// when the residual is below 1e-12 * <paramref name="scale"/>; accepted when it is below 1e-9 * scale and no
        /// further progress is possible. False, when the Jacobian is (almost) singular or the residual stays too large;
        /// <paramref name="x"/> is undefined then.
        /// </summary>
        private static bool Solve<T>(ref T system, Span<double> x, int n, double scale, int maxIterations) where T : struct, ISystem
        {
            double tight = 1e-12 * scale; // converged
            double loose = 1e-9 * scale; // good enough, when no further progress is possible
            Span<double> f = stackalloc double[n];
            Span<double> jac = stackalloc double[n * n];
            Span<double> delta = stackalloc double[n];
            Span<double> trial = stackalloc double[n];
            Span<double> trialF = stackalloc double[n];
            Span<double> trialJac = stackalloc double[n * n];
            Span<double> work = stackalloc double[n * n];

            double norm = system.Evaluate(x, f, jac);
            if (double.IsNaN(norm)) return false;
            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                if (norm <= tight) break;
                // the Newton step: jac * delta = -f
                for (int i = 0; i < n; i++) delta[i] = -f[i];
                jac.CopyTo(work);
                if (!SolveLinear(work, delta, n)) return false; // (almost) singular: the surfaces touch here
                double lambda = 1.0;
                bool improved = false;
                for (int halving = 0; halving < 10; halving++)
                {
                    for (int i = 0; i < n; i++) trial[i] = x[i] + lambda * delta[i];
                    double trialNorm = system.Evaluate(trial, trialF, trialJac);
                    if (trialNorm < norm)
                    {
                        trial.CopyTo(x);
                        trialF.CopyTo(f);
                        trialJac.CopyTo(jac);
                        norm = trialNorm;
                        improved = true;
                        break;
                    }
                    lambda *= 0.5;
                }
                if (!improved) break; // no more progress, decided below by the residual
                double stepSize = 0.0, size = 0.0;
                for (int i = 0; i < n; i++)
                {
                    stepSize = Math.Max(stepSize, Math.Abs(lambda * delta[i]));
                    size = Math.Max(size, Math.Abs(x[i]));
                }
                if (stepSize <= 1e-15 * (1.0 + size)) break; // the parameters don't change any more
            }
            return norm <= loose;
        }

        /// <summary>
        /// Solves a * x = b for the <paramref name="n"/> x <paramref name="n"/> matrix <paramref name="a"/> (row major, it is
        /// overwritten) by Gaussian elimination with partial pivoting; <paramref name="b"/> becomes the solution. False,
        /// when the matrix is (almost) singular.
        /// </summary>
        private static bool SolveLinear(Span<double> a, Span<double> b, int n)
        {
            double maxAbs = 0.0;
            for (int i = 0; i < n * n; i++) maxAbs = Math.Max(maxAbs, Math.Abs(a[i]));
            if (maxAbs == 0.0) return false;
            double singular = 1e-12 * maxAbs;
            for (int col = 0; col < n; col++)
            {
                int pivot = col;
                for (int row = col + 1; row < n; row++)
                {
                    if (Math.Abs(a[row * n + col]) > Math.Abs(a[pivot * n + col])) pivot = row;
                }
                if (Math.Abs(a[pivot * n + col]) <= singular) return false;
                if (pivot != col)
                {
                    for (int k = 0; k < n; k++) (a[col * n + k], a[pivot * n + k]) = (a[pivot * n + k], a[col * n + k]);
                    (b[col], b[pivot]) = (b[pivot], b[col]);
                }
                for (int row = col + 1; row < n; row++)
                {
                    double factor = a[row * n + col] / a[col * n + col];
                    if (factor == 0.0) continue;
                    for (int k = col; k < n; k++) a[row * n + k] -= factor * a[col * n + k];
                    b[row] -= factor * b[col];
                }
            }
            for (int row = n - 1; row >= 0; row--)
            {
                double sum = b[row];
                for (int k = row + 1; k < n; k++) sum -= a[row * n + k] * b[k];
                b[row] = sum / a[row * n + row];
            }
            return true;
        }
    }
}
