using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CADability
{
    public class Symmetry2D
    {
        static public GeoPoint2D FindSymmetryAxes2D(GeoPoint2D[] points, out List<GeoVector2D> axis)
        {
            GeoPoint2D centroid = new GeoPoint2D(points);
            double[] dist = new double[points.Length];
            for (int i = 0; i < points.Length; i++) dist[i] = points[i] | centroid;
            int[] sortedIndices = Enumerable.Range(0, points.Length).ToArray();
            Array.Sort(sortedIndices, (a, b) => dist[a].CompareTo(dist[b]));
            axis = new List<GeoVector2D>();
            for (int i = 0; i < sortedIndices.Length - 1; i++)
            {
                for (int l=i+1; l < sortedIndices.Length; ++l)
                {
                    if (dist[sortedIndices[l]] - dist[sortedIndices[i]] < Precision.eps)
                    {
                        // found a pair of points with the same distance to the centroid
                        GeoVector2D v = points[sortedIndices[l]] - points[sortedIndices[i]];
                        if (v.Length < Precision.eps) continue; // both points are identical
                        GeoVector2D axisCandidate = new GeoVector2D(-v.y, v.x);
                        axisCandidate.Norm();
                        bool alreadyFound = false;
                        for (int j = 0; j < axis.Count; j++)
                        {
                            if (Precision.SameDirection(axis[j], axisCandidate, true))
                            {
                                alreadyFound = true;
                                break;
                            }
                        }
                        if (alreadyFound) continue;
                        // check if this axis is a symmetry axis
                        HashSet<int> toTest = Enumerable.Range(0, points.Length).ToHashSet();
                        toTest.Remove(sortedIndices[i]);
                        toTest.Remove(sortedIndices[l]);
                        for (int j = 0; j < sortedIndices.Length; j++)
                        {
                            if (!toTest.Contains(sortedIndices[j])) continue; // already tested
                            if (Math.Abs(Geometry.DistPL(points[sortedIndices[j]], centroid, axisCandidate)) < Precision.eps)
                            {
                                // point lies on the axis
                                toTest.Remove(sortedIndices[j]);
                                continue;
                            }
                            for (int k = j + 1; k < points.Length; k++)
                            {
                                if (dist[sortedIndices[k]] - dist[sortedIndices[j]] < Precision.eps)
                                {
                                    double s = (points[sortedIndices[j]] - points[sortedIndices[k]]) * axisCandidate;
                                    if (Math.Abs(s) < Precision.eps) // connection is perpendicular to axisCandidate
                                    { // the two points are symmetric to each other
                                        toTest.Remove(sortedIndices[j]);
                                        toTest.Remove(sortedIndices[k]);
                                        break;
                                    }
                                }
                                else break; // no more points with the same distance
                            }
                        }
                        if (!toTest.Any())
                        {
                            axis.Add(axisCandidate);
                        }
                    }
                }
            }
            return centroid;
        }
        public static void test()
        {
            GeoPoint2D[] points = { new GeoPoint2D(10, 10), new GeoPoint2D(10, -10), new GeoPoint2D(-10, 10), new GeoPoint2D(-10, -10) };
            GeoPoint2D cntroid = FindSymmetryAxes2D(points, out List<GeoVector2D> axes);
        }
    }
}
