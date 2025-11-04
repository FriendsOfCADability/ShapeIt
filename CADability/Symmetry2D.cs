using CADability.Curve2D;
using CADability.GeoObject;
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
                for (int l = i + 1; l < sortedIndices.Length; ++l)
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
        public static GeoPoint2D FindSymmetryAxes2D(Path2D path, out List<GeoVector2D> axis)
        {
            List<GeoPoint2D> samples = new List<GeoPoint2D>();
            foreach (ICurve2D c2d in path.SubCurves)
            {
                samples.Add(c2d.EndPoint);
            }
            if (samples.Count <= 2)
            {
                foreach (ICurve2D c2d in path.SubCurves)
                {
                    samples.Add(c2d.PointAt(0.5));
                }
            }
            GeoPoint2D centroid = FindSymmetryAxes2D(samples.ToArray(), out axis);
            BoundingRect ext = path.GetExtent();
            for (int i = axis.Count - 1; i >= 0; --i)
            {
                GeoPoint2DWithParameter[] ips = path.Intersect(centroid - ext.Size * axis[i], centroid + ext.Size * axis[i]);
                List<GeoPoint2DWithParameter> lips = new List<GeoPoint2DWithParameter>();
                for (int j = 0; j < ips.Length; j++)
                {
                    if (ips[j].par1 < 0.0 || ips[j].par1 > 1.0 || ips[j].par2 < 0.0 || ips[j].par2 > 1.0) continue;
                    if (lips.Count > 0 && Precision.IsEqual(ips[j].p, lips.Last().p)) continue;
                    lips.Add(ips[j]);
                }
                if (lips.Count == 0)
                {
                    axis.RemoveAt(i);
                    continue;
                }
                else if (lips.Count == 2)
                {
                    ModOp2D reflect = ModOp2D.Reflect(centroid, axis[i]);
                    double[] pars = new double[ips.Length];
                    for (int j = 0; j < lips.Count; j++) pars[j] = lips[j].par1;
                    List<ICurve2D> parts = new List<ICurve2D>(path.Split(new double[] { lips[0].par1, lips[1].par1 }));
                    if (path.IsClosed && Precision.IsEqual(parts.First().StartPoint, parts.Last().EndPoint))
                    {
                        parts[0] = new Path2D(new ICurve2D[] { parts.Last(), parts.First() });
                        (parts[0] as Path2D).Flatten();
                        parts.RemoveAt(parts.Count - 1);
                    }
                    bool ok = false;
                    if (parts.Count % 2 == 0)
                    {
                        ok = true;
                        for (int j = 0; j < parts.Count / 2; j++)
                        {
                            (parts[parts.Count - 1 - j] as Path2D).Flatten();
                            (parts[j] as Path2D).Flatten();
                            ICurve2D c = parts[parts.Count - 1 - j].GetModified(reflect);
                            c.Reverse();
                            if (!(parts[j].MakeGeoObject(Plane.XYPlane) as ICurve).SameGeometry(c.MakeGeoObject(Plane.XYPlane) as ICurve, Precision.eps))
                            {
                                ok = false;
                                break;
                            }
                        }
                    }
                    if (!ok)
                    {
                        axis.RemoveAt(i);
                        continue;
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
