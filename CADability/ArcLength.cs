using System;
using System.Collections.Generic;

namespace CADability
{
    /// <summary>
    /// Arc length of a curve by numerical integration of its speed, i.e. the length of the unnormalized
    /// tangent |C'(t)|. The length is the integral of that speed over the parameter interval, which is what
    /// a curve whose parametrization is not proportional to the arc length - a spline, a rational spline
    /// above all - needs: measuring it by the polygon through a handful of interpolation points is short by
    /// whatever the curve bulges out between them. The exact rational quadratic circle is the extreme case,
    /// with one span per quadrant: its chords are 10% shorter than the circle.
    /// </summary>
    internal static class ArcLength
    {
        // 10 point Gauss-Legendre on [-1,1], positive half only - the rule is symmetric, so each node counts
        // twice. Two properties are what make it the right rule here: it is exact up to polynomial degree 19,
        // so a smooth span is finished in one evaluation, and it never samples the ends of the interval,
        // where the speed of a spline with a degenerate first or last span is singular.
        private static readonly double[] nodes = {
            0.1488743389816312, 0.4333953941292472, 0.6794095682990244, 0.8650633666889845, 0.9739065285171717 };
        private static readonly double[] weights = {
            0.2955242247147529, 0.2692667193099963, 0.2190863625159820, 0.1494513491505806, 0.0666713443086881 };

        private static double Gauss(Func<double, double> speed, double from, double to)
        {
            double middle = (from + to) / 2.0;
            double half = (to - from) / 2.0;
            double sum = 0.0;
            for (int i = 0; i < nodes.Length; ++i)
            {
                double d = half * nodes[i];
                sum += weights[i] * (speed(middle - d) + speed(middle + d));
            }
            return sum * half;
        }

        /// <summary>
        /// Bisects until the halved interval no longer changes the result by more than <paramref name="tolerance"/>.
        /// The tolerance is halved along with the interval, so the errors of all parts add up to at most the
        /// tolerance this was started with.
        /// </summary>
        private static double Refine(Func<double, double> speed, double from, double to, double coarse, double tolerance, int depth)
        {
            double middle = (from + to) / 2.0;
            double left = Gauss(speed, from, middle);
            double right = Gauss(speed, middle, to);
            double fine = left + right;
            if (depth <= 0 || Math.Abs(fine - coarse) <= tolerance) return fine;
            return Refine(speed, from, middle, left, tolerance / 2.0, depth - 1)
                 + Refine(speed, middle, to, right, tolerance / 2.0, depth - 1);
        }

        /// <summary>
        /// The arc length over [<paramref name="from"/>, <paramref name="to"/>] of a curve whose speed |C'(t)|
        /// is given by <paramref name="speed"/>. Always positive, also for from &gt; to.
        /// </summary>
        /// <param name="speed">length of the unnormalized tangent at a parameter</param>
        /// <param name="from">start of the parameter interval</param>
        /// <param name="to">end of the parameter interval</param>
        /// <param name="breakPoints">parameters where the speed is not smooth - the knots of a spline. Values
        /// outside the interval are ignored, the order does not matter</param>
        /// <param name="relativeTolerance">accuracy relative to the total length</param>
        /// <param name="maxDepth">gives up bisecting after this many levels, so a curve with a cusp - where
        /// the speed drops to zero and the integrand is no longer smooth - terminates too</param>
        public static double FromSpeed(Func<double, double> speed, double from, double to,
            IEnumerable<double> breakPoints = null, double relativeTolerance = 1e-9, int maxDepth = 12)
        {
            if (from > to)
            {
                double t = from; from = to; to = t;
            }
            if (!(to > from)) return 0.0; // empty interval, or NaN ends

            // the knots cut the curve into its spans: inside a span the speed is smooth and the Gauss rule
            // converges fast, across a knot it is only continuous and a rule spanning the kink would not
            List<double> parts = new List<double>();
            parts.Add(from);
            if (breakPoints != null)
            {
                foreach (double b in breakPoints)
                {
                    if (b > from && b < to) parts.Add(b);
                }
                parts.Sort();
            }
            parts.Add(to);

            double[] coarse = new double[parts.Count - 1];
            double estimate = 0.0;
            for (int i = 0; i < coarse.Length; ++i)
            {
                coarse[i] = Gauss(speed, parts[i], parts[i + 1]);
                estimate += coarse[i];
            }
            if (!(estimate > 0.0)) return estimate; // zero length, or NaN - nothing to refine

            double tolerance = relativeTolerance * estimate / coarse.Length;
            double res = 0.0;
            for (int i = 0; i < coarse.Length; ++i)
            {
                res += Refine(speed, parts[i], parts[i + 1], coarse[i], tolerance, maxDepth);
            }
            return res;
        }
    }
}
