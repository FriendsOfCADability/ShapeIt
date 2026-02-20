using CADability;
using CADability.GeoObject;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using System;
using System.Collections.Generic;

public static class SurfaceIntersectionFootPoint
{
    /// <summary>
    /// Finds the point f on the intersection curve of s1 and s2 such that
    /// the vector (f - p) is orthogonal to the intersection curve tangent.
    /// The initial uv1/uv2 parameters must already be close to the intersection.
    /// Returns true iff LM converged.
    /// </summary>
    public static bool TryFootPointOnIntersectionCurveLM(
        ISurface s1,
        ISurface s2,
        GeoPoint p,
        ref GeoPoint2D uv1,
        ref GeoPoint2D uv2,
        out GeoPoint f,
        int maxIterations = 50,
        double initialMu = 1e-3,
        double gradientTolerance = 1e-14,
        double stepTolerance = 1e-16,
        double functionTolerance = 1e-14,
        double orthWeight = 1.0,
        double fdStep = 1e-6)
    {
        // Parameter vector: [u, v, a, b]
        var V = Vector<double>.Build;

        Vector<double> initialGuess =
            V.DenseOfArray(new[] { uv1.x, uv1.y, uv2.x, uv2.y });

        // Observations: we want F(p) = 0 (dummy "x", typical for this API)
        Vector<double> observedX = V.Dense(4, 0.0);
        Vector<double> observedY = V.Dense(4, 0.0);

        // Weights per residual (can scale the orthogonality equation)
        Vector<double> weight = V.Dense(4, 1.0);
        weight[3] = orthWeight;

        Vector<double> Residual(Vector<double> par)
        {
            double u = par[0], v = par[1], a = par[2], b = par[3];

            // We only need location + first derivatives here;
            // Derivative2At is used because it's available in your interface.
            s1.Derivative2At(new GeoPoint2D(u, v),
                out GeoPoint s1p, out GeoVector s1u, out GeoVector s1v,
                out _, out _, out _);

            s2.Derivative2At(new GeoPoint2D(a, b),
                out GeoPoint s2p, out GeoVector s2u, out GeoVector s2v,
                out _, out _, out _);

            // g = s1 - s2 (3 equations)
            double r0 = s1p.x - s2p.x;
            double r1 = s1p.y - s2p.y;
            double r2 = s1p.z - s2p.z;

            // h = (s1 - p) · t, where t is the normalized intersection tangent direction proxy
            // t = normalize( (n1 × n2) ), n1 = Su×Sv, n2 = Sa×Sb
            GeoVector n1 = s1u ^ s1v;
            GeoVector n2 = s2u ^ s2v;
            GeoVector w = n1 ^ n2;

            double wl = w.Length;
            GeoVector t = (wl > 0.0) ? (1.0 / wl) * w : w; // tangential case ignored per your note

            GeoVector q = new GeoVector(
                s1p.x - p.x,
                s1p.y - p.y,
                s1p.z - p.z);

            double r3 = q * t;

            return V.DenseOfArray(new[] { r0, r1, r2, r3 });
        }

        Vector<double> Model(Vector<double> par, Vector<double> x)
        {
            // Return "predicted y" for given parameters (here: residual vector)
            // observedY is zero -> we minimize ||Residual(par) - 0||^2
            return Residual(par);
        }

        Matrix<double> Jacobian(Vector<double> par, Vector<double> x)
        {
            double u = par[0], v = par[1], a = par[2], b = par[3];

            s1.Derivative2At(new GeoPoint2D(u, v),
                out _, out GeoVector s1u, out GeoVector s1v,
                out _, out _, out _);

            s2.Derivative2At(new GeoPoint2D(a, b),
                out _, out GeoVector s2u, out GeoVector s2v,
                out _, out _, out _);

            // Jacobian matrix (4x4): rows = residuals r0..r3, cols = u,v,a,b
            var J = Matrix<double>.Build.Dense(4, 4);

            // Analytic part for g = s1 - s2
            // dr/du, dr/dv
            J[0, 0] = s1u.x; J[1, 0] = s1u.y; J[2, 0] = s1u.z;
            J[0, 1] = s1v.x; J[1, 1] = s1v.y; J[2, 1] = s1v.z;

            // dr/da, dr/db
            J[0, 2] = -s2u.x; J[1, 2] = -s2u.y; J[2, 2] = -s2u.z;
            J[0, 3] = -s2v.x; J[1, 3] = -s2v.y; J[2, 3] = -s2v.z;

            // Numeric row for h using central finite differences
            // dh/dθ ≈ (h(θ+ε) - h(θ-ε)) / (2ε)
            double epsU = StepFor(par[0], fdStep);
            double epsV = StepFor(par[1], fdStep);
            double epsA = StepFor(par[2], fdStep);
            double epsB = StepFor(par[3], fdStep);

            J[3, 0] = CentralDiffH(par, 0, epsU);
            J[3, 1] = CentralDiffH(par, 1, epsV);
            J[3, 2] = CentralDiffH(par, 2, epsA);
            J[3, 3] = CentralDiffH(par, 3, epsB);

            return J;

            double CentralDiffH(Vector<double> p0, int idx, double eps)
            {
                var pPlus = p0.Clone();
                var pMinus = p0.Clone();
                pPlus[idx] += eps;
                pMinus[idx] -= eps;

                // Only need the 4th residual component (h)
                double hPlus = Residual(pPlus)[3];
                double hMinus = Residual(pMinus)[3];
                return (hPlus - hMinus) / (2.0 * eps);
            }
        }

        static double StepFor(double value, double baseStep)
        {
            // Relative step helps when parameters are not near 1.0
            // Avoid going too tiny when value is ~0
            double s = baseStep * (1.0 + Math.Abs(value));
            return (s > 0.0) ? s : baseStep;
        }

        var objective = ObjectiveFunction.NonlinearModel(Model, Jacobian, observedX, observedY, weight);

        var lm = new LevenbergMarquardtMinimizer(
            initialMu, gradientTolerance, stepTolerance, functionTolerance, maxIterations);

        NonlinearMinimizationResult result;
        try
        {
            result = lm.FindMinimum(objective, initialGuess);
        }
        catch
        {
            f = default;
            return false;
        }

        bool converged = result.ReasonForExit == ExitCondition.Converged;

        var sol = result.MinimizingPoint;
        uv1 = new GeoPoint2D(sol[0], sol[1]);
        uv2 = new GeoPoint2D(sol[2], sol[3]);

        // Final foot point on the intersection curve (use s1)
        f = s1.PointAt(uv1);

        return converged;
    }
}
