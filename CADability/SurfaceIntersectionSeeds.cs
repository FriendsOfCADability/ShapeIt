using System;
using System.Collections.Generic;

namespace CADability.GeoObject
{
    public partial class Surfaces
    {
        #region seeds for intersection loops by subdivision

        /// <summary>The normals of a patch are sampled on a grid of this many points in each direction.</summary>
        private const int coneSamples = 5;
        /// <summary>Upper limit for the number of patch pairs examined, a safety net against endless subdivision.</summary>
        private const int maxSeedPairs = 20000;

        /// <summary>
        /// A uv patch of one of the two surfaces with its (cached) 3d extent and normal cone.
        /// </summary>
        private class SeedPatch
        {
            public BoundingRect uv;
            public BoundingBox extent;
            public GeoVector coneAxis; // unit vector, null vector when there is no useful cone
            public double coneAngle; // half opening angle, Math.PI when the normals point in all directions
            public SeedPatch(ISurface surface, BoundingRect uv)
            {
                this.uv = uv;
                extent = surface.GetPatchExtent(uv, true);
                NormalCone(surface, uv, out coneAxis, out coneAngle);
            }
            public SeedPatch[] Split(ISurface surface)
            {
                GeoPoint2D c = uv.GetCenter();
                return new SeedPatch[]
                {
                    new SeedPatch(surface, new BoundingRect(uv.Left, uv.Bottom, c.x, c.y)),
                    new SeedPatch(surface, new BoundingRect(c.x, uv.Bottom, uv.Right, c.y)),
                    new SeedPatch(surface, new BoundingRect(uv.Left, c.y, c.x, uv.Top)),
                    new SeedPatch(surface, new BoundingRect(c.x, c.y, uv.Right, uv.Top))
                };
            }
        }

        /// <summary>
        /// Finds points on all intersection curves of two surfaces inside the given domains, at least one per curve, as
        /// seeds for <see cref="GetIntersectionLoops"/>. Works for any kind of surfaces and needs no positions where the
        /// normals are parallel (see <see cref="ISurface.GetExtremePositions"/>), which are hard to find for two NURBS
        /// surfaces.
        /// <para>
        /// Pairs of patches of the two surfaces are subdivided as long as their extents overlap. A pair is simple, when no
        /// normal of the one patch is parallel to a normal of the other patch (the normal cones and the reversed normal
        /// cones are separated). Then the intersection inside this pair cannot contain a closed loop (Sederberg/Meyers,
        /// Hohmeyer): every intersection curve inside the pair crosses the border of one of the two patches. So the
        /// intersections of the four border curves of each patch with the other patch are points on all curves of this pair.
        /// </para>
        /// <para>
        /// The normal cones are estimated from a grid of sampled normals plus a margin for the variation between the
        /// samples, they are not strictly guaranteed. Where the surfaces touch, the cones never separate: when the
        /// subdivision reaches a very small pair there, <paramref name="complete"/> is false, and the result may miss
        /// a curve. <paramref name="complete"/> is also false when the safety limit of examined pairs is reached.
        /// </para>
        /// </summary>
        /// <param name="surface1">the first surface</param>
        /// <param name="bounds1">the domain of interest on the first surface</param>
        /// <param name="surface2">the second surface</param>
        /// <param name="bounds2">the domain of interest on the second surface</param>
        /// <param name="complete">true, when every pair was decided, so an empty result means there is no intersection</param>
        /// <returns>points on (or very close to) the intersection curves</returns>
        public static List<GeoPoint> IntersectionSeedsBySubdivision(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, out bool complete)
        {
            complete = true;
            List<GeoPoint> seeds = new List<GeoPoint>();
            SeedPatch root1 = new SeedPatch(surface1, bounds1);
            SeedPatch root2 = new SeedPatch(surface2, bounds2);
            double size = Math.Max(root1.extent.Size, root2.extent.Size);
            if (!(size > 0.0)) return seeds;
            double minSize = size * 1e-4; // a pair this small is not split any more
            double extentTolerance = size * 1e-9;
            Stack<(SeedPatch, SeedPatch)> pairs = new Stack<(SeedPatch, SeedPatch)>();
            pairs.Push((root1, root2));
            int examined = 0;
            while (pairs.Count > 0)
            {
                (SeedPatch p1, SeedPatch p2) = pairs.Pop();
                if (++examined > maxSeedPairs)
                {
                    complete = false;
                    break;
                }
                if (!Overlap(p1.extent, p2.extent, extentTolerance)) continue; // no intersection in this pair
                if (ConesSeparated(p1, p2))
                {   // no closed loop inside this pair: the curves cross the borders
                    BorderIntersections(surface1, p1.uv, surface2, p2.uv, seeds);
                    BorderIntersections(surface2, p2.uv, surface1, p1.uv, seeds);
                    continue;
                }
                bool split1 = p1.extent.Size > minSize, split2 = p2.extent.Size > minSize;
                if (!split1 && !split2)
                {   // the surfaces (almost) touch here: the cones cannot be separated
                    complete = false;
                    continue;
                }
                // split the bigger patch, so that the two patches of a pair keep a similar size
                if (split1 && (!split2 || p1.extent.Size >= p2.extent.Size))
                {
                    foreach (SeedPatch sub in p1.Split(surface1)) pairs.Push((sub, p2));
                }
                else
                {
                    foreach (SeedPatch sub in p2.Split(surface2)) pairs.Push((p1, sub));
                }
            }
            // neighbouring pairs share their borders: remove the duplicates
            List<GeoPoint> res = new List<GeoPoint>();
            double eps = Math.Max(Precision.eps, size * 1e-9);
            for (int i = 0; i < seeds.Count; i++)
            {
                bool duplicate = false;
                for (int j = 0; j < res.Count; j++)
                {
                    if ((res[j] | seeds[i]) < eps)
                    {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) res.Add(seeds[i]);
            }
            return res;
        }

        private static bool Overlap(BoundingBox b1, BoundingBox b2, double tolerance)
        {
            if (b1.IsEmpty || b2.IsEmpty) return false;
            return b1.Xmin <= b2.Xmax + tolerance && b2.Xmin <= b1.Xmax + tolerance &&
                b1.Ymin <= b2.Ymax + tolerance && b2.Ymin <= b1.Ymax + tolerance &&
                b1.Zmin <= b2.Zmax + tolerance && b2.Zmin <= b1.Zmax + tolerance;
        }

        /// <summary>
        /// True, when no normal of the first patch can be parallel or antiparallel to a normal of the second patch.
        /// </summary>
        private static bool ConesSeparated(SeedPatch p1, SeedPatch p2)
        {
            if (p1.coneAngle >= Math.PI / 2.0 || p2.coneAngle >= Math.PI / 2.0) return false;
            double angle = new Angle(p1.coneAxis, p2.coneAxis).Radian; // 0..pi
            double sum = p1.coneAngle + p2.coneAngle;
            return angle > sum && Math.PI - angle > sum;
        }

        /// <summary>
        /// Estimates the cone of the normals of the patch: the axis is the mean of sampled normals, the half angle is the
        /// biggest deviation of a sample plus half of the biggest angle between neighbouring samples, which accounts for
        /// the variation between the samples.
        /// </summary>
        private static void NormalCone(ISurface surface, BoundingRect uv, out GeoVector axis, out double halfAngle)
        {
            GeoVector[,] normals = new GeoVector[coneSamples, coneSamples];
            GeoVector sum = GeoVector.NullVector;
            for (int i = 0; i < coneSamples; i++)
            {
                for (int j = 0; j < coneSamples; j++)
                {
                    GeoPoint2D p = new GeoPoint2D(uv.Left + uv.Width * i / (coneSamples - 1), uv.Bottom + uv.Height * j / (coneSamples - 1));
                    GeoVector n = surface.GetNormal(p);
                    if (n.IsNullVector() || double.IsNaN(n.x))
                    {   // a singular point (a pole): no statement about the normals of this patch
                        axis = GeoVector.NullVector;
                        halfAngle = Math.PI;
                        return;
                    }
                    n.Norm();
                    normals[i, j] = n;
                    sum += n;
                }
            }
            if (sum.Length < 1e-6)
            {
                axis = GeoVector.NullVector;
                halfAngle = Math.PI;
                return;
            }
            axis = sum.Normalized;
            double maxDeviation = 0.0, maxNeighbour = 0.0;
            for (int i = 0; i < coneSamples; i++)
            {
                for (int j = 0; j < coneSamples; j++)
                {
                    maxDeviation = Math.Max(maxDeviation, new Angle(axis, normals[i, j]).Radian);
                    if (i > 0) maxNeighbour = Math.Max(maxNeighbour, new Angle(normals[i - 1, j], normals[i, j]).Radian);
                    if (j > 0) maxNeighbour = Math.Max(maxNeighbour, new Angle(normals[i, j - 1], normals[i, j]).Radian);
                    if (i > 0 && j > 0) maxNeighbour = Math.Max(maxNeighbour, new Angle(normals[i - 1, j - 1], normals[i, j]).Radian);
                }
            }
            halfAngle = maxDeviation + 0.5 * maxNeighbour;
        }

        /// <summary>
        /// Adds the intersection points of the four border curves of the patch <paramref name="uv1"/> of
        /// <paramref name="surface1"/> with the patch <paramref name="uv2"/> of <paramref name="surface2"/> to
        /// <paramref name="seeds"/>.
        /// </summary>
        private static void BorderIntersections(ISurface surface1, BoundingRect uv1, ISurface surface2, BoundingRect uv2, List<GeoPoint> seeds)
        {
            ICurve[] borders = new ICurve[]
            {
                surface1.FixedU(uv1.Left, uv1.Bottom, uv1.Top),
                surface1.FixedU(uv1.Right, uv1.Bottom, uv1.Top),
                surface1.FixedV(uv1.Bottom, uv1.Left, uv1.Right),
                surface1.FixedV(uv1.Top, uv1.Left, uv1.Right)
            };
            BoundingRect inside = uv2;
            inside.Inflate(uv2.Width * 1e-6, uv2.Height * 1e-6);
            for (int i = 0; i < borders.Length; i++)
            {
                if (borders[i] == null) continue;
                surface2.Intersect(borders[i], uv2, out GeoPoint[] ips, out GeoPoint2D[] uvOn2, out double[] uOnCurve);
                for (int j = 0; j < ips.Length; j++)
                {
                    if (uOnCurve[j] < -1e-6 || uOnCurve[j] > 1.0 + 1e-6) continue;
                    GeoPoint2D uv = uvOn2[j];
                    SurfaceHelper.AdjustPeriodic(surface2, uv2, ref uv);
                    if (inside.Contains(uv)) seeds.Add(ips[j]);
                }
            }
        }
        #endregion
    }
}
