using CADability.GeoObject;
using System;

namespace CADability
{
    /// <summary>
    /// Newton's method for the common point of a plane and two surfaces, the basic step of tracing an intersection
    /// curve (marching, <see cref="InterpolatedDualSurfaceCurve"/>).
    /// <para>
    /// The unknowns are the parameters (u1, v1) on the first and (u2, v2) on the second surface, the equations
    /// S1(u1, v1) - S2(u2, v2) = 0 (three) and n * ((S1 + S2) / 2 - location) = 0 (the plane): a square 4x4 system.
    /// The Levenberg-Marquardt solvers used so far treated the plane as a third surface with parameters of its own
    /// (6 unknowns, 9 residuals) and allocate vectors, matrices and delegates on every call. Here everything is on
    /// the stack, a good start value converges in two or three steps and a failure is a return value, not an
    /// exception.
    /// </para>
    /// <para>
    /// Where the surfaces touch (the curve is tangential) the Jacobian is (almost) singular and Newton's method is
    /// not reliable: then it gives up, and the caller should use the damped Levenberg-Marquardt solver.
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
            GeoPoint location = plane.Location;
            // The achievable precision is limited by the magnitude of the coordinates
            double scale = 1.0 + Math.Max(Math.Abs(location.x), Math.Max(Math.Abs(location.y), Math.Abs(location.z)));
            double tight = 1e-12 * scale; // converged
            double loose = 1e-9 * scale; // good enough, when no further progress is possible

            Span<double> x = stackalloc double[4] { uv1.x, uv1.y, uv2.x, uv2.y };
            Span<double> f = stackalloc double[4];
            Span<double> jac = stackalloc double[16];
            Span<double> delta = stackalloc double[4];
            Span<double> trial = stackalloc double[4];
            Span<double> trialF = stackalloc double[4];
            Span<double> trialJac = stackalloc double[16];

            double norm = Evaluate(plane.Location, n, surface1, surface2, x, f, jac, out GeoPoint p1, out GeoPoint p2);
            if (double.IsNaN(norm)) return false;
            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                if (norm <= tight) break;
                // the Newton step: jac * delta = -f
                for (int i = 0; i < 4; i++) delta[i] = -f[i];
                if (!Solve4(jac, delta)) return false; // (almost) singular: the surfaces touch here
                // damped: halve the step until the residual decreases
                double lambda = 1.0;
                bool improved = false;
                for (int halving = 0; halving < 10; halving++)
                {
                    for (int i = 0; i < 4; i++) trial[i] = x[i] + lambda * delta[i];
                    double trialNorm = Evaluate(location, n, surface1, surface2, trial, trialF, trialJac, out GeoPoint tp1, out GeoPoint tp2);
                    if (trialNorm < norm)
                    {
                        trial.CopyTo(x);
                        trialF.CopyTo(f);
                        trialJac.CopyTo(jac);
                        p1 = tp1;
                        p2 = tp2;
                        norm = trialNorm;
                        improved = true;
                        break;
                    }
                    lambda *= 0.5;
                }
                if (!improved) break; // no more progress, decided below by the residual
                double stepSize = 0.0, size = 0.0;
                for (int i = 0; i < 4; i++)
                {
                    stepSize = Math.Max(stepSize, Math.Abs(lambda * delta[i]));
                    size = Math.Max(size, Math.Abs(x[i]));
                }
                if (stepSize <= 1e-15 * (1.0 + size)) break; // the parameters don't change any more
            }
            if (!(norm <= loose)) return false;
            uv1 = new GeoPoint2D(x[0], x[1]);
            uv2 = new GeoPoint2D(x[2], x[3]);
            ip = new GeoPoint(p1, p2);
            return true;
        }

        /// <summary>
        /// The residual <paramref name="f"/> and its Jacobian <paramref name="jac"/> (row major) at <paramref name="x"/>,
        /// returns the norm of the residual (NaN, when a surface cannot be evaluated there).
        /// </summary>
        private static double Evaluate(GeoPoint location, GeoVector n, ISurface surface1, ISurface surface2,
            ReadOnlySpan<double> x, Span<double> f, Span<double> jac, out GeoPoint p1, out GeoPoint p2)
        {
            surface1.DerivativeAt(new GeoPoint2D(x[0], x[1]), out p1, out GeoVector du1, out GeoVector dv1);
            surface2.DerivativeAt(new GeoPoint2D(x[2], x[3]), out p2, out GeoVector du2, out GeoVector dv2);
            f[0] = p1.x - p2.x;
            f[1] = p1.y - p2.y;
            f[2] = p1.z - p2.z;
            f[3] = 0.5 * (n * (p1 - location) + n * (p2 - location));
            // rows: d(S1 - S2)/d(u1, v1, u2, v2) in x, y, z, then d(plane equation)
            jac[0] = du1.x; jac[1] = dv1.x; jac[2] = -du2.x; jac[3] = -dv2.x;
            jac[4] = du1.y; jac[5] = dv1.y; jac[6] = -du2.y; jac[7] = -dv2.y;
            jac[8] = du1.z; jac[9] = dv1.z; jac[10] = -du2.z; jac[11] = -dv2.z;
            jac[12] = 0.5 * (n * du1); jac[13] = 0.5 * (n * dv1); jac[14] = 0.5 * (n * du2); jac[15] = 0.5 * (n * dv2);
            double norm = Math.Sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2] + f[3] * f[3]);
            return double.IsNaN(norm) || double.IsInfinity(norm) ? double.NaN : norm; // netstandard2.0 has no IsFinite
        }

        /// <summary>
        /// Solves a * x = b for a 4x4 matrix <paramref name="a"/> (row major, not modified) by Gaussian elimination with
        /// partial pivoting; <paramref name="b"/> becomes the solution. False, when the matrix is (almost) singular.
        /// </summary>
        private static bool Solve4(Span<double> a, Span<double> b)
        {
            Span<double> m = stackalloc double[16];
            a.CopyTo(m);
            double maxAbs = 0.0;
            for (int i = 0; i < 16; i++) maxAbs = Math.Max(maxAbs, Math.Abs(m[i]));
            if (maxAbs == 0.0) return false;
            double singular = 1e-12 * maxAbs;
            for (int col = 0; col < 4; col++)
            {
                int pivot = col;
                for (int row = col + 1; row < 4; row++)
                {
                    if (Math.Abs(m[row * 4 + col]) > Math.Abs(m[pivot * 4 + col])) pivot = row;
                }
                if (Math.Abs(m[pivot * 4 + col]) <= singular) return false;
                if (pivot != col)
                {
                    for (int k = 0; k < 4; k++) (m[col * 4 + k], m[pivot * 4 + k]) = (m[pivot * 4 + k], m[col * 4 + k]);
                    (b[col], b[pivot]) = (b[pivot], b[col]);
                }
                for (int row = col + 1; row < 4; row++)
                {
                    double factor = m[row * 4 + col] / m[col * 4 + col];
                    if (factor == 0.0) continue;
                    for (int k = col; k < 4; k++) m[row * 4 + k] -= factor * m[col * 4 + k];
                    b[row] -= factor * b[col];
                }
            }
            for (int row = 3; row >= 0; row--)
            {
                double sum = b[row];
                for (int k = row + 1; k < 4; k++) sum -= m[row * 4 + k] * b[k];
                b[row] = sum / m[row * 4 + row];
            }
            return true;
        }
    }
}
