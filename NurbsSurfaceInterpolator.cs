using System;
using System.Collections.Generic;
using System.Linq;
using CADability;
using CADability.GeoObject;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace ShapeIt
{
    public static class NurbsSurfaceInterpolator
    {
        /// <summary>
        /// Represents a bicubic Hermite patch defined by corner positions, tangents, and twist vectors.
        /// </summary>
        public class HermitePatch
        {
            // Corner points
            public GeoPoint P00, P10, P01, P11;

            // Tangents in u direction at corners
            public GeoVector M00, M10, M01, M11;

            // Tangents in v direction at corners
            public GeoVector N00, N10, N01, N11;

            // Twist vectors at corners (mixed second derivatives)
            public GeoVector T00, T10, T01, T11;

            GeoPoint P(int i, int j)
            {
                if (i == 0)
                {
                    if (j == 0) return P00;
                    else return P01;
                }
                else
                {
                    if (j == 0) return P10;
                    else return P11;
                }
            }
            GeoVector M(int i, int j)
            {
                if (i == 0)
                {
                    if (j == 0) return M00;
                    else return M01;
                }
                else
                {
                    if (j == 0) return M10;
                    else return M11;
                }
            }
            GeoVector N(int i, int j)
            {
                if (i == 0)
                {
                    if (j == 0) return N00;
                    else return N01;
                }
                else
                {
                    if (j == 0) return N10;
                    else return N11;
                }
            }
            GeoVector T(int i, int j)
            {
                if (i == 0)
                {
                    if (j == 0) return T00;
                    else return T01;
                }
                else
                {
                    if (j == 0) return T10;
                    else return T11;
                }
            }
            public GeoPoint PointAt(GeoPoint2D uv)
            {
                Func<double, double>[] H = new Func<double, double>[] {
                    u => 2 * u * u * u - 3 * u * u + 1,
                    u => -2 * u * u * u + 3 * u * u,
                    u => u * u * u - 2 * u * u + u,
                    u => u * u * u - u * u };
                GeoPoint[] tmp = new GeoPoint[4];
                GeoVector p = GeoVector.NullVector;
                for (int i = 0; i <= 1; i++)
                {
                    for (int j = 0; j <= 1; j++)
                    {
                        p += H[i](uv.x) * H[j](uv.y) * P(i, j).ToVector() + H[i + 2](uv.x) * H[j](uv.y) * M(i, j) + H[i](uv.x) * H[j + 2](uv.y) * N(i, j) + H[i + 2](uv.x) * H[j + 2](uv.y) * T(i, j);
                    }
                }
                return GeoPoint.Origin + p;
            }
            public Face Debug
            {
                get
                {
                    GeoPoint[,] pnts = new GeoPoint[10, 10];
                    for (int i = 0; i < 10; i++)
                    {
                        for (int j = 0; j < 10; j++)
                        {
                            pnts[i, j] = PointAt(new GeoPoint2D(i / 9.0, j / 9.0));
                        }
                    }
                    NurbsSurface ns = new NurbsSurface(pnts, 3, 3, false, false);
                    return Face.MakeFace(ns, BoundingRect.UnitBoundingRect);
                }
            }

            /// <summary>
            /// Converts this Hermite patch into a cubic NURBS patch control grid (4x4)
            /// aligned to the knot spans [uIndex, uIndex+1] and [vIndex, vIndex+1].
            /// </summary>
            /// <returns>4x4 array of GeoPoint representing the NURBS control points.</returns>
            public GeoPoint[,] ToNurbs()
            {
                var C = new GeoPoint[4, 4];
                // unit span factors 1/3 and 1/9
                // Row 0
                C[0, 0] = P00;
                C[0, 1] = new GeoPoint(P00.x + N00.x / 3.0, P00.y + N00.y / 3.0, P00.z + N00.z / 3.0);
                C[0, 2] = new GeoPoint(P01.x - N01.x / 3.0, P01.y - N01.y / 3.0, P01.z - N01.z / 3.0);
                C[0, 3] = P01;
                // Row 1
                C[1, 0] = new GeoPoint(P00.x + M00.x / 3.0, P00.y + M00.y / 3.0, P00.z + M00.z / 3.0);
                C[1, 1] = new GeoPoint(
                    P00.x + M00.x / 3.0 + N00.x / 3.0 + T00.x / 9.0,
                    P00.y + M00.y / 3.0 + N00.y / 3.0 + T00.y / 9.0,
                    P00.z + M00.z / 3.0 + N00.z / 3.0 + T00.z / 9.0);
                C[1, 2] = new GeoPoint(
                    P01.x + M01.x / 3.0 - N01.x / 3.0 - T01.x / 9.0,
                    P01.y + M01.y / 3.0 - N01.y / 3.0 - T01.y / 9.0,
                    P01.z + M01.z / 3.0 - N01.z / 3.0 - T01.z / 9.0);
                C[1, 3] = new GeoPoint(P01.x + M01.x / 3.0, P01.y + M01.y / 3.0, P01.z + M01.z / 3.0);
                // Row 2
                C[2, 0] = new GeoPoint(P10.x - M10.x / 3.0, P10.y - M10.y / 3.0, P10.z - M10.z / 3.0);
                C[2, 1] = new GeoPoint(
                    P10.x - M10.x / 3.0 + N10.x / 3.0 - T10.x / 9.0,
                    P10.y - M10.y / 3.0 + N10.y / 3.0 - T10.y / 9.0,
                    P10.z - M10.z / 3.0 + N10.z / 3.0 - T10.z / 9.0);
                C[2, 2] = new GeoPoint(
                    P11.x - M11.x / 3.0 - N11.x / 3.0 + T11.x / 9.0,
                    P11.y - M11.y / 3.0 - N11.y / 3.0 + T11.y / 9.0,
                    P11.z - M11.z / 3.0 - N11.z / 3.0 + T11.z / 9.0);
                C[2, 3] = new GeoPoint(P11.x - M11.x / 3.0, P11.y - M11.y / 3.0, P11.z - M11.z / 3.0);
                // Row 3
                C[3, 0] = P10;
                C[3, 1] = new GeoPoint(P10.x + N10.x / 3.0, P10.y + N10.y / 3.0, P10.z + N10.z / 3.0);
                C[3, 2] = new GeoPoint(P11.x - N11.x / 3.0, P11.y - N11.y / 3.0, P11.z - N11.z / 3.0);
                C[3, 3] = P11;
                return C;
            }
        }

        /// <summary>
        /// Connects a grid of Hermite patches into one NURBS control grid.
        /// Expects patches[,] sized [pu, pv], each contributing a 4x4 net.
        /// Returns the merged control points after knot removal.
        /// </summary>
        public static GeoPoint[,] ConnectHermitePatches(HermitePatch[,] patches)
        {
            int pu = patches.GetLength(0);
            int pv = patches.GetLength(1);
            int p = 3;
            // initial control-point count: pu*3+1, pv*3+1
            int nu = pu * p + 1;
            int nv = pv * p + 1;
            // initial grid
            GeoPoint[,] P = new GeoPoint[nu, nv];
            // populate with local ToNurbs at correct offsets
            for (int i = 0; i < pu; i++)
                for (int j = 0; j < pv; j++)
                {
                    var local = patches[i, j].ToNurbs();
                    NurbsSurface dbg = new NurbsSurface(local, null, new double[] { 0, 0, 0, 0, 1, 1, 1, 1 }, new double[] { 0, 0, 0, 0, 1, 1, 1, 1 }, 3, 3, false, false);
                    for (int di = 0; di <= p; di++)
                        for (int dj = 0; dj <= p; dj++)
                            P[i * p + di, j * p + dj] = local[di, dj];
                }
            // build initial knot vectors
            double[] U = BuildOpenUniformKnotVector(pu);
            double[] V = BuildOpenUniformKnotVector(pv);
            // remove internal knots in u-direction
            for (int k = 1; k < pu; k++)
                for (int r = 0; r < p; r++)
                {
                    int rem = k * (p + 1);
                    // new grid has one less row
                    var Q = new GeoPoint[P.GetLength(0) - 1, P.GetLength(1)];
                    for (int col = 0; col < P.GetLength(1); col++)
                    {
                        // extract column
                        var colPts = new GeoPoint[P.GetLength(0)];
                        for (int row = 0; row < P.GetLength(0); row++) colPts[row] = P[row, col];
                        var newCol = RemoveKnotCurve(colPts, p, rem, U);
                        for (int row = 0; row < newCol.Length; row++) Q[row, col] = newCol[row];
                    }
                    P = Q;
                    U = RemoveKnotFromVector(U, rem);
                }
            // remove internal knots in v-direction
            for (int k = 1; k < pv; k++)
                for (int r = 0; r < p; r++)
                {
                    int rem = k * (p + 1);
                    var Q = new GeoPoint[P.GetLength(0), P.GetLength(1) - 1];
                    for (int row = 0; row < P.GetLength(0); row++)
                    {
                        var rowPts = new GeoPoint[P.GetLength(1)];
                        for (int col = 0; col < P.GetLength(1); col++) rowPts[col] = P[row, col];
                        var newRow = RemoveKnotCurve(rowPts, p, rem, V);
                        for (int col = 0; col < newRow.Length; col++) Q[row, col] = newRow[col];
                    }
                    P = Q;
                    V = RemoveKnotFromVector(V, rem);
                }
            return P;
        }

        // builds open uniform knot vector [0^4,1^4,...,segments^4]
        private static double[] BuildOpenUniformKnotVector(int segments)
        {
            int p = 3;
            var U = new double[(segments + 1) * (p + 1)];
            int idx = 0;
            for (int i = 0; i <= segments; i++)
                for (int j = 0; j <= p; j++)
                    U[idx++] = i;
            return U;
        }

        // removes entry at index k from knot vector
        private static double[] RemoveKnotFromVector(double[] U, int k)
        {
            var V = new double[U.Length - 1];
            for (int i = 0; i < k; i++) V[i] = U[i];
            for (int i = k + 1; i < U.Length; i++) V[i - 1] = U[i];
            return V;
        }

        // removes a knot at index k from a curve's control points P
        private static GeoPoint[] RemoveKnotCurve(GeoPoint[] P, int p, int k, double[] U)
        {
            int n = P.Length - 1;
            int first = k - p;
            int last = k - 1;
            var Q = new GeoPoint[n];
            // copy unaffected
            for (int i = 0; i < first; i++) Q[i] = P[i];
            for (int i = last + 1; i <= n; i++) Q[i - 1] = P[i];
            // update affected
            for (int i = first; i <= last; i++)
            {
                double alpha = (U[k] - U[i]) / (U[i + p + 1] - U[i]);
                var prev = Q[i - 1];
                Q[i] = new GeoPoint(
                    (P[i].x - (1 - alpha) * prev.x) / alpha,
                    (P[i].y - (1 - alpha) * prev.y) / alpha,
                    (P[i].z - (1 - alpha) * prev.z) / alpha
                );
            }
            return Q;
        }
        public static NurbsSurface BuildNurbsSurfaceOfDegree3(GeoPoint[,] throughPoints, GeoVector[,] uDerivatives, GeoVector[,] vDerivatives, GeoVector[,] uvDerivatives)
        {
            HermitePatch[,] hermitePatches = new HermitePatch[throughPoints.GetLength(0) - 1, throughPoints.GetLength(1) - 1];

            for (int i = 1; i < throughPoints.GetLength(0); i++)
            {
                for (int j = 1; j < throughPoints.GetLength(1); j++)
                {
                    HermitePatch hp = new HermitePatch();
                    hp.P00 = throughPoints[i - 1, j - 1];
                    hp.P10 = throughPoints[i, j - 1];
                    hp.P01 = throughPoints[i - 1, j];
                    hp.P11 = throughPoints[i, j];

                    hp.M00 = uDerivatives[i - 1, j - 1];
                    hp.M10 = uDerivatives[i, j - 1];
                    hp.M01 = uDerivatives[i - 1, j];
                    hp.M11 = uDerivatives[i, j];

                    hp.N00 = vDerivatives[i - 1, j - 1];
                    hp.N10 = vDerivatives[i, j - 1];
                    hp.N01 = vDerivatives[i - 1, j];
                    hp.N11 = vDerivatives[i, j];

                    hp.T00 = uvDerivatives[i - 1, j - 1];
                    hp.T10 = uvDerivatives[i, j - 1];
                    hp.T01 = uvDerivatives[i - 1, j];
                    hp.T11 = uvDerivatives[i, j];

                    hermitePatches[i - 1, j - 1] = hp;
                }
            }

            GeoPoint[,] poles = ConnectHermitePatches(hermitePatches);
            List<double> uknotes = new List<double>(Enumerable.Range(0, throughPoints.GetLength(0)).Select(x => (double)(x)));
            List<double> vknotes = new List<double>(Enumerable.Range(0, throughPoints.GetLength(0)).Select(x => (double)(x)));
            for (int i = 0; i < 3; i++)
            {
                uknotes.Insert(0, 0.0);
                uknotes.Add(uknotes.Last());
                vknotes.Insert(0, 0.0);
                vknotes.Add(uknotes.Last());
            }

            return new NurbsSurface(poles, null, uknotes.ToArray(), vknotes.ToArray(), 3, 3, false, false);
        }
    }
}