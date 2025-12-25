using CADability;
using CADability.GeoObject;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.Optimization;
using System;
using System.Collections.Generic;

namespace CADability
{

    public static class SurfaceIntersectionSolvers
    {
        // -----------------------------
        // Shared residual evaluators
        // -----------------------------

        // 9 residuals: (p1-p2), (p2-p3), (p3-p1) in x,y,z
        private static Vector<double> Residual9(
            ISurface s1, ISurface s2, ISurface s3, Vector<double> vd)
        {
            GeoPoint p1 = s1.PointAt(new GeoPoint2D(vd[0], vd[1]));
            GeoPoint p2 = s2.PointAt(new GeoPoint2D(vd[2], vd[3]));
            GeoPoint p3 = s3.PointAt(new GeoPoint2D(vd[4], vd[5]));

            return new DenseVector(new double[]
            {
            p1.x - p2.x, p1.y - p2.y, p1.z - p2.z,
            p2.x - p3.x, p2.y - p3.y, p2.z - p3.z,
            p3.x - p1.x, p3.y - p1.y, p3.z - p1.z
            });
        }

        // 6 residuals: (p1-p2), (p2-p3) in x,y,z
        private static Vector<double> Residual6(
            ISurface s1, ISurface s2, ISurface s3, Vector<double> vd)
        {
            GeoPoint p1 = s1.PointAt(new GeoPoint2D(vd[0], vd[1]));
            GeoPoint p2 = s2.PointAt(new GeoPoint2D(vd[2], vd[3]));
            GeoPoint p3 = s3.PointAt(new GeoPoint2D(vd[4], vd[5]));

            return new DenseVector(new double[]
            {
            p1.x - p2.x, p1.y - p2.y, p1.z - p2.z,
            p2.x - p3.x, p2.y - p3.y, p2.z - p3.z
            });
        }

        // Jacobian for Residual9 with analytic DerivationAt
        private static Matrix<double> Jacobian9_Analytic(
            ISurface s1, ISurface s2, ISurface s3, Vector<double> vd)
        {
            // DerivationAt returns loc + tangents du/dv
            s1.DerivationAt(new GeoPoint2D(vd[0], vd[1]), out GeoPoint loc1, out GeoVector du1, out GeoVector dv1);
            s2.DerivationAt(new GeoPoint2D(vd[2], vd[3]), out GeoPoint loc2, out GeoVector du2, out GeoVector dv2);
            s3.DerivationAt(new GeoPoint2D(vd[4], vd[5]), out GeoPoint loc3, out GeoVector du3, out GeoVector dv3);

            var J = new DenseMatrix(9, 6);

            // rows 0..2: p1 - p2
            J[0, 0] = du1.x; J[0, 1] = dv1.x; J[0, 2] = -du2.x; J[0, 3] = -dv2.x;
            J[1, 0] = du1.y; J[1, 1] = dv1.y; J[1, 2] = -du2.y; J[1, 3] = -dv2.y;
            J[2, 0] = du1.z; J[2, 1] = dv1.z; J[2, 2] = -du2.z; J[2, 3] = -dv2.z;

            // rows 3..5: p2 - p3
            J[3, 2] = du2.x; J[3, 3] = dv2.x; J[3, 4] = -du3.x; J[3, 5] = -dv3.x;
            J[4, 2] = du2.y; J[4, 3] = dv2.y; J[4, 4] = -du3.y; J[4, 5] = -dv3.y;
            J[5, 2] = du2.z; J[5, 3] = dv2.z; J[5, 4] = -du3.z; J[5, 5] = -dv3.z;

            // rows 6..8: p3 - p1
            J[6, 0] = -du1.x; J[6, 1] = -dv1.x; J[6, 4] = du3.x; J[6, 5] = dv3.x;
            J[7, 0] = -du1.y; J[7, 1] = -dv1.y; J[7, 4] = du3.y; J[7, 5] = dv3.y;
            J[8, 0] = -du1.z; J[8, 1] = -dv1.z; J[8, 4] = du3.z; J[8, 5] = dv3.z;

            return J;
        }

        // Jacobian for Residual6 with analytic DerivationAt
        private static Matrix<double> Jacobian6_Analytic(
            ISurface s1, ISurface s2, ISurface s3, Vector<double> vd)
        {
            s1.DerivationAt(new GeoPoint2D(vd[0], vd[1]), out GeoPoint loc1, out GeoVector du1, out GeoVector dv1);
            s2.DerivationAt(new GeoPoint2D(vd[2], vd[3]), out GeoPoint loc2, out GeoVector du2, out GeoVector dv2);
            s3.DerivationAt(new GeoPoint2D(vd[4], vd[5]), out GeoPoint loc3, out GeoVector du3, out GeoVector dv3);

            var J = new DenseMatrix(6, 6);

            // rows 0..2: p1 - p2
            J[0, 0] = du1.x; J[0, 1] = dv1.x; J[0, 2] = -du2.x; J[0, 3] = -dv2.x;
            J[1, 0] = du1.y; J[1, 1] = dv1.y; J[1, 2] = -du2.y; J[1, 3] = -dv2.y;
            J[2, 0] = du1.z; J[2, 1] = dv1.z; J[2, 2] = -du2.z; J[2, 3] = -dv2.z;

            // rows 3..5: p2 - p3
            J[3, 2] = du2.x; J[3, 3] = dv2.x; J[3, 4] = -du3.x; J[3, 5] = -dv3.x;
            J[4, 2] = du2.y; J[4, 3] = dv2.y; J[4, 4] = -du3.y; J[4, 5] = -dv3.y;
            J[5, 2] = du2.z; J[5, 3] = dv2.z; J[5, 4] = -du3.z; J[5, 5] = -dv3.z;

            return J;
        }

        // Numerical Jacobian for Residual9 (central differences)
        private static Matrix<double> Jacobian9_CentralFD(
            ISurface s1, ISurface s2, ISurface s3, Vector<double> vd,
            double relStep = 1e-6)
        {
            var J = new DenseMatrix(9, 6);

            // Evaluate columns
            for (int i = 0; i < 6; i++)
            {
                double hi = relStep * Math.Max(1.0, Math.Abs(vd[i]));

                var xp = vd.Clone();
                var xm = vd.Clone();
                xp[i] += hi;
                xm[i] -= hi;

                Vector<double> fp = Residual9(s1, s2, s3, xp);
                Vector<double> fm = Residual9(s1, s2, s3, xm);

                double inv2h = 1.0 / (2.0 * hi);
                for (int r = 0; r < 9; r++)
                    J[r, i] = (fp[r] - fm[r]) * inv2h;
            }
            return J;
        }

        private static bool IsGoodExit(ExitCondition reason)
        {
            return reason == ExitCondition.Converged
                || reason == ExitCondition.RelativeGradient
                || reason == ExitCondition.RelativePoints;
        }

        private static void UpdateOutputsFromResult(
            ISurface s1, ISurface s2, ISurface s3,
            NonlinearMinimizationResult mres,
            ref GeoPoint2D uv1, ref GeoPoint2D uv2, ref GeoPoint2D uv3, ref GeoPoint ip)
        {
            uv1 = new GeoPoint2D(mres.MinimizingPoint[0], mres.MinimizingPoint[1]);
            uv2 = new GeoPoint2D(mres.MinimizingPoint[2], mres.MinimizingPoint[3]);
            uv3 = new GeoPoint2D(mres.MinimizingPoint[4], mres.MinimizingPoint[5]);

            ip = new GeoPoint(
                s1.PointAt(uv1),
                s2.PointAt(uv2),
                s3.PointAt(uv3));
        }

        // ============================================================
        // Variante A: 9 Residuen + analytischer Jacobian (dein Status quo)
        // ============================================================
        public static bool SurfacesIntersectionLM_Analytic9(
            ISurface surface1, ISurface surface2, ISurface surface3,
            ref GeoPoint2D uv1, ref GeoPoint2D uv2, ref GeoPoint2D uv3, ref GeoPoint ip,
            int maxIterations = 20)
        {
            Vector<double> observedX = new DenseVector(9);
            Vector<double> observedY = new DenseVector(9); // all zeros
            var lm = new LevenbergMarquardtMinimizer(maximumIterations: maxIterations);

            IObjectiveModel iom = ObjectiveFunction.NonlinearModel(
                (vd, ox) => Residual9(surface1, surface2, surface3, vd),
                (vd, ox) => Jacobian9_Analytic(surface1, surface2, surface3, vd),
                observedX, observedY);

            try
            {
                var start = new DenseVector(new double[] { uv1.x, uv1.y, uv2.x, uv2.y, uv3.x, uv3.y });
                NonlinearMinimizationResult mres = lm.FindMinimum(iom, start);

                if (!IsGoodExit(mres.ReasonForExit)) return false;

                UpdateOutputsFromResult(surface1, surface2, surface3, mres, ref uv1, ref uv2, ref uv3, ref ip);
                return true;
            }
            catch
            {
                ip = GeoPoint.Origin;
                return false;
            }
        }

        // ============================================================
        // Variante B: 9 Residuen + numerischer Jacobian (central differences)
        // ============================================================
        public static bool SurfacesIntersectionLM_CentralFD9(
            ISurface surface1, ISurface surface2, ISurface surface3,
            ref GeoPoint2D uv1, ref GeoPoint2D uv2, ref GeoPoint2D uv3, ref GeoPoint ip,
            int maxIterations = 20,
            double relStep = 1e-6)
        {
            Vector<double> observedX = new DenseVector(9);
            Vector<double> observedY = new DenseVector(9); // all zeros
            var lm = new LevenbergMarquardtMinimizer(maximumIterations: maxIterations);

            IObjectiveModel iom = ObjectiveFunction.NonlinearModel(
                (vd, ox) => Residual9(surface1, surface2, surface3, vd),
                (vd, ox) => Jacobian9_CentralFD(surface1, surface2, surface3, vd, relStep),
                observedX, observedY);

            try
            {
                var start = new DenseVector(new double[] { uv1.x, uv1.y, uv2.x, uv2.y, uv3.x, uv3.y });
                NonlinearMinimizationResult mres = lm.FindMinimum(iom, start);

                if (!IsGoodExit(mres.ReasonForExit)) return false;

                UpdateOutputsFromResult(surface1, surface2, surface3, mres, ref uv1, ref uv2, ref uv3, ref ip);
                return true;
            }
            catch
            {
                ip = GeoPoint.Origin;
                return false;
            }
        }

        // ============================================================
        // Variante C: 6 Residuen (redundanzfrei) + analytischer Jacobian
        // ============================================================
        public static bool SurfacesIntersectionLM_Analytic6(
            ISurface surface1, ISurface surface2, ISurface surface3,
            ref GeoPoint2D uv1, ref GeoPoint2D uv2, ref GeoPoint2D uv3, ref GeoPoint ip,
            int maxIterations = 20)
        {
            Vector<double> observedX = new DenseVector(6);
            Vector<double> observedY = new DenseVector(6); // all zeros
            var lm = new LevenbergMarquardtMinimizer(maximumIterations: maxIterations);

            IObjectiveModel iom = ObjectiveFunction.NonlinearModel(
                (vd, ox) => Residual6(surface1, surface2, surface3, vd),
                (vd, ox) => Jacobian6_Analytic(surface1, surface2, surface3, vd),
                observedX, observedY);

            try
            {
                var start = new DenseVector(new double[] { uv1.x, uv1.y, uv2.x, uv2.y, uv3.x, uv3.y });
                NonlinearMinimizationResult mres = lm.FindMinimum(iom, start);

                if (!IsGoodExit(mres.ReasonForExit)) return false;

                UpdateOutputsFromResult(surface1, surface2, surface3, mres, ref uv1, ref uv2, ref uv3, ref ip);
                return true;
            }
            catch
            {
                ip = GeoPoint.Origin;
                return false;
            }
        }
        
        /// <summary>
        /// Numerical approximation of ISurface.DerivationAt using central differences of PointAt.
        /// Intended for debugging/validation of analytic DerivationAt.
        /// </summary>
        public static void NumericalDerivationAt(
            ISurface surface,
            GeoPoint2D uv,
            out GeoPoint location,
            out GeoVector du,
            out GeoVector dv,
            double relStep = 1e-6,
            double minAbsStep = 1e-9)
        {
            location = surface.PointAt(uv);

            // Step sizes (scaled)
            double hu = Math.Max(minAbsStep, relStep * Math.Max(1.0, Math.Abs(uv.x)));
            double hv = Math.Max(minAbsStep, relStep * Math.Max(1.0, Math.Abs(uv.y)));

            // Helpers
            GeoPoint Eval(double u, double v) => surface.PointAt(new GeoPoint2D(u, v));

            // --- du ---
            // Try central difference first
            bool duCentralOk = true;
            GeoPoint puPlus = default, puMinus = default;

            try { puPlus = Eval(uv.x + hu, uv.y); }
            catch { duCentralOk = false; }

            try { puMinus = Eval(uv.x - hu, uv.y); }
            catch { duCentralOk = false; }

            if (duCentralOk)
            {
                // (p(u+h)-p(u-h)) / (2h)
                du = (1.0 / (2.0 * hu) * (puPlus - puMinus));
            }
            else
            {
                // Fallback: forward or backward difference
                bool forwardOk = true;
                GeoPoint pu0 = location;
                GeoPoint puF = default;

                try { puF = Eval(uv.x + hu, uv.y); }
                catch { forwardOk = false; }

                if (forwardOk)
                {
                    du = (1.0 / hu) * (puF - pu0);
                }
                else
                {
                    // backward
                    GeoPoint puB = Eval(uv.x - hu, uv.y); // if this throws too, let it bubble up
                    du = (1.0 / hu) * (pu0 - puB);
                }
            }

            // --- dv ---
            bool dvCentralOk = true;
            GeoPoint pvPlus = default, pvMinus = default;

            try { pvPlus = Eval(uv.x, uv.y + hv); }
            catch { dvCentralOk = false; }

            try { pvMinus = Eval(uv.x, uv.y - hv); }
            catch { dvCentralOk = false; }

            if (dvCentralOk)
            {
                dv = (1.0 / (2.0 * hv)) * (pvPlus - pvMinus);
            }
            else
            {
                bool forwardOk = true;
                GeoPoint pv0 = location;
                GeoPoint pvF = default;

                try { pvF = Eval(uv.x, uv.y + hv); }
                catch { forwardOk = false; }

                if (forwardOk)
                {
                    dv = (1.0 / hv) * (pvF - pv0);
                }
                else
                {
                    GeoPoint pvB = Eval(uv.x, uv.y - hv);
                    dv = (1.0 / hv) * (pv0 - pvB);
                }
            }
        }

        // Optional: quick comparison helper (prints relative errors etc.)
        public static void CompareDerivationAt(
            ISurface surface, GeoPoint2D uv,
            double relStep = 1e-6)
        {
            surface.DerivationAt(uv, out GeoPoint locA, out GeoVector duA, out GeoVector dvA);
            NumericalDerivationAt(surface, uv, out GeoPoint locN, out GeoVector duN, out GeoVector dvN, relStep);

            // location should match exactly (both from PointAt)
            // If your DerivationAt returns a loc that differs, that's already a red flag.
            double locDist = (locA - locN).Length;

            double duAbs = (duA - duN).Length;
            double dvAbs = (dvA - dvN).Length;

            double duRel = duAbs / Math.Max(1e-30, duN.Length);
            double dvRel = dvAbs / Math.Max(1e-30, dvN.Length);

            System.Diagnostics.Debug.WriteLine($"uv={uv.x},{uv.y}");
            System.Diagnostics.Debug.WriteLine($"locDist={locDist}");
            System.Diagnostics.Debug.WriteLine($"duAbs={duAbs}, duRel={duRel}");
            System.Diagnostics.Debug.WriteLine($"dvAbs={dvAbs}, dvRel={dvRel}");
        }

        /// <summary>
        /// Numerical approximation of PointAndDerivativesAt using only curve.PointAt(u).
        /// Returns a list: [P(u), P'(u), P''(u), ...] as GeoVector (P is returned as vector from origin, like your code).
        /// Intended for debugging / validation of analytic derivatives.
        /// </summary>
        public static IReadOnlyList<GeoVector> NumericalPointAndDerivativesAt(
            ICurve curve, double u, int grad,
            double relStep = 1e-6,
            double minAbsStep = 1e-9)
        {
            if (grad < 0) throw new ArgumentOutOfRangeException(nameof(grad));

            var res = new List<GeoVector>(grad + 1);

            GeoPoint p0 = curve.PointAt(u);
            res.Add(p0.ToVector());

            if (grad == 0) return res;

            // Step size scaled to parameter magnitude
            double h = Math.Max(minAbsStep, relStep * Math.Max(1.0, Math.Abs(u)));

            // Helper to evaluate points
            GeoPoint Eval(double t) => curve.PointAt(t);

            // We'll build derivatives iteratively using central differences of the previous "function".
            // For grad=1, this is classic central difference.
            // For grad=2, it becomes the standard 3-point second derivative, etc.
            //
            // Specifically:
            // D1(u) ≈ (P(u+h)-P(u-h))/(2h)
            // D2(u) ≈ (P(u+h)-2P(u)+P(u-h))/h^2
            //
            // For grad>2, the recursion uses finite-difference of the previous derivative, which is okay for debugging
            // but noise grows quickly; for grad=2 it's excellent.

            // Precompute points used by 1st and 2nd derivatives
            GeoPoint pPlus = Eval(u + h);
            GeoPoint pMinus = Eval(u - h);

            // 1st derivative
            GeoVector d1 = (1.0 / (2.0 * h)*(pPlus - pMinus));
            res.Add(d1);
            if (grad == 1) return res;

            // 2nd derivative
            GeoVector d2 = (1.0 / (h * h))*(pPlus.ToVector() - (2.0 * p0.ToVector()) + pMinus.ToVector()) ;
            res.Add(d2);
            if (grad == 2) return res;

            // For higher derivatives, use central difference on the previous derivative function:
            // Dk(u) ≈ (D(k-1)(u+h) - D(k-1)(u-h)) / (2h)
            // This requires computing D(k-1) at u±h, which in turn needs points at wider offsets.
            //
            // For debugging, we can do this by recursively calling a helper that computes derivative order m at t.
            // It's not the most efficient, but it's straightforward and good for validating analytic implementations.

            for (int k = 3; k <= grad; k++)
            {
                GeoVector dk = CentralDerivativeOfOrder(curve, u, k, h);
                res.Add(dk);
            }

            return res;
        }

        /// <summary>
        /// Central finite difference approximation for derivative of order n of curve.PointAt(u).
        /// Uses a simple recursive scheme (good for debugging, not for production high-order derivatives).
        /// </summary>
        private static GeoVector CentralDerivativeOfOrder(ICurve curve, double u, int n, double h)
        {
            if (n == 1)
            {
                GeoPoint pPlus = curve.PointAt(u + h);
                GeoPoint pMinus = curve.PointAt(u - h);
                return (1.0 / (2.0 * h) * (pPlus.ToVector() - pMinus.ToVector()));
            }
            if (n == 2)
            {
                GeoPoint p0 = curve.PointAt(u);
                GeoPoint pPlus = curve.PointAt(u + h);
                GeoPoint pMinus = curve.PointAt(u - h);
                return (1.0 / (h * h))*(pPlus.ToVector() - (2.0 * p0.ToVector()) + pMinus.ToVector());
            }

            // Recurrence: D^n f(u) ≈ (D^(n-1) f(u+h) - D^(n-1) f(u-h)) / (2h)
            GeoVector dPrevPlus = CentralDerivativeOfOrder(curve, u + h, n - 1, h);
            GeoVector dPrevMinus = CentralDerivativeOfOrder(curve, u - h, n - 1, h);
            return (1.0 / (2.0 * h) * (dPrevPlus - dPrevMinus));
        }
    }


}
