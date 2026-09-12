using CADability.Curve2D;
using CADability.GeoObject;
using System;
using System.Collections.Generic;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="SweptCircle.GetSelfIntersections(BoundingRect)"/>: where the curvature radius of the
    /// spine falls below the radius of the swept circle, the surface folds over and penetrates itself.
    /// </summary>
    [TestClass]
    public class SweptCircleSelfIntersectionTests
    {
        private const double radius = 1.5;
        private static readonly BoundingRect fullDomain = new BoundingRect(0, 0, 1, 2 * Math.PI);

        /// <summary>
        /// A planar spine in the z==0 plane along y == -cos(t). Its curvature radius is 1 at t == 0 and at
        /// t == +-pi (bending to the other side there) and grows in between, so a circle radius above 1 makes the
        /// surface fold. Which v the fold belongs to depends on the orientation of the plane of the spine, so the
        /// tests must not assume it.
        /// </summary>
        private static SweptCircle MakeSurface(double from, double to, double r)
        {
            int n = 40;
            GeoPoint[] pnts = new GeoPoint[n + 1];
            for (int i = 0; i <= n; i++)
            {
                double t = from + (to - from) * i / n;
                pnts[i] = new GeoPoint(t, -Math.Cos(t), 0.0);
            }
            BSpline bsp = BSpline.Construct();
            Assert.IsTrue(bsp.ThroughPoints(pnts, 3, false));
            return new SweptCircle(bsp, r);
        }

        /// <summary>
        /// The point on a branch of the double curve which has the given v. The branches are monotonous in v.
        /// </summary>
        private static GeoPoint2D PointAtV(ICurve2D curve, double v)
        {
            double lo = 0.0, hi = 1.0;
            bool ascending = curve.EndPoint.y > curve.StartPoint.y;
            for (int i = 0; i < 60; i++)
            {
                double m = (lo + hi) / 2.0;
                if (curve.PointAt(m).y < v == ascending) lo = m;
                else hi = m;
            }
            return curve.PointAt((lo + hi) / 2.0);
        }

        /// <summary>
        /// The two branches of a pair must describe the same 3d curve: at the same v they must yield the same 3d
        /// point, at clearly different u values.
        /// </summary>
        private static void AssertIsDoubleCurve(SweptCircle surface, ICurve2D branch1, ICurve2D branch2)
        {
            double v1 = Math.Min(branch1.StartPoint.y, branch1.EndPoint.y);
            double v2 = Math.Max(branch1.StartPoint.y, branch1.EndPoint.y);
            Assert.IsTrue(v2 - v1 > 1e-3, "the v range of the double curve is empty");
            for (int i = 1; i < 20; i++)
            {
                double v = v1 + (v2 - v1) * i / 20.0;
                GeoPoint2D uv1 = PointAtV(branch1, v);
                GeoPoint2D uv2 = PointAtV(branch2, v);
                GeoPoint p1 = surface.PointAt(uv1);
                GeoPoint p2 = surface.PointAt(uv2);
                Assert.IsTrue((p1 | p2) < 1e-4, $"v={v}: {uv1} and {uv2} are not the same 3d point ({p1} != {p2}, distance {p1 | p2})");
                Assert.IsTrue(Math.Abs(uv1.x - uv2.x) > 1e-3, $"v={v}: the two branches did not separate ({uv1.x} vs {uv2.x})");
            }
        }

        /// <summary>The smallest curvature radius of the spine and the position where it occurs.</summary>
        private static (double radius, double position) MinCurvatureRadius(ICurve spine, double from, double to)
        {
            double best = double.MaxValue, at = from;
            for (int i = 0; i <= 2000; i++)
            {
                double u = from + (to - from) * i / 2000.0;
                double cr = Math.Abs(spine.CurvatureAt(u).radius);
                if (cr < best) { best = cr; at = u; }
            }
            return (best, at);
        }

        [TestMethod]
        public void single_fold_yields_one_pair_of_branches()
        {
            SweptCircle sc = MakeSurface(-1.3, 1.3, radius);
            ICurve2D[] si = sc.GetSelfIntersections(fullDomain);
            Assert.IsNotNull(si, "the surface folds over, so it must intersect itself");
            Assert.AreEqual(2, si.Length, "one fold must yield exactly one pair of branches");
            AssertIsDoubleCurve(sc, si[0], si[1]);
        }

        [TestMethod]
        public void branches_meet_in_the_two_swallowtail_points()
        {
            SweptCircle sc = MakeSurface(-1.3, 1.3, radius);
            ICurve2D[] si = sc.GetSelfIntersections(fullDomain);
            // the branch with the bigger u ascends, the other one descends, so start and end points are exchanged
            Assert.IsTrue((si[0].StartPoint | si[1].EndPoint) < 1e-6, "the branches do not meet in the lower swallowtail point");
            Assert.IsTrue((si[0].EndPoint | si[1].StartPoint) < 1e-6, "the branches do not meet in the upper swallowtail point");
            double vlow = si[0].StartPoint.y, vhigh = si[0].EndPoint.y;
            // the double curve is symmetric to the v which points to the center of curvature, i.e. to +-pi/2
            double vCenter = (vlow + vhigh) / 2.0;
            Assert.IsTrue(Math.Abs(Math.Abs(Math.Sin(vCenter)) - 1.0) < 1e-6, $"the double curve is not centered at +-pi/2: {vCenter}");
            // both swallowtail points sit at the vertex of the spine, where sin(v) == minCurvatureRadius/radius
            (double minRadius, double uVertex) = MinCurvatureRadius(sc.Spine, 0.0, 1.0);
            Assert.IsTrue(Math.Abs(si[0].StartPoint.x - uVertex) < 1e-3, $"swallowtail point not at the vertex of the spine: {si[0].StartPoint.x} != {uVertex}");
            Assert.IsTrue(Math.Abs(si[0].EndPoint.x - uVertex) < 1e-3, $"swallowtail point not at the vertex of the spine: {si[0].EndPoint.x} != {uVertex}");
            Assert.IsTrue(Math.Abs(Math.Abs(Math.Sin(vlow)) - minRadius / radius) < 1e-4, $"wrong v of the swallowtail point: {vlow}");
            Assert.IsTrue(Math.Abs(Math.Abs(Math.Sin(vhigh)) - minRadius / radius) < 1e-4, $"wrong v of the swallowtail point: {vhigh}");
        }

        [TestMethod]
        public void double_curve_encloses_the_fold_curve()
        {
            // the curve where the surface normal vanishes is sin(v) == curvatureRadius(u)/radius. At the center of
            // the double curve it reaches its widest u range, and the double curve must be outside of that.
            SweptCircle sc = MakeSurface(-1.3, 1.3, radius);
            ICurve2D[] si = sc.GetSelfIntersections(fullDomain);
            double vCenter = (si[0].StartPoint.y + si[0].EndPoint.y) / 2.0;
            GeoPoint2D uv1 = PointAtV(si[0], vCenter);
            GeoPoint2D uv2 = PointAtV(si[1], vCenter);
            double ulow = Math.Min(uv1.x, uv2.x), uhigh = Math.Max(uv1.x, uv2.x);
            List<double> fold = new List<double>();
            for (int i = 0; i < 2000; i++)
            {   // the positions where the curvature radius of the spine equals the radius of the circle
                bool before = Math.Abs(sc.Spine.CurvatureAt(i / 2000.0).radius) < radius;
                bool after = Math.Abs(sc.Spine.CurvatureAt((i + 1) / 2000.0).radius) < radius;
                if (before != after) fold.Add((i + 0.5) / 2000.0);
            }
            Assert.AreEqual(2, fold.Count, "there should be two positions where the curvature radius equals the radius");
            Assert.IsTrue(ulow < fold[0] && fold[1] < uhigh,
                $"the double curve [{ulow},{uhigh}] does not enclose the fold curve [{fold[0]},{fold[1]}]");
        }

        [TestMethod]
        public void three_folds_with_alternating_curvature_sign()
        {
            // -cos(t) has its vertices at t == -pi, 0, pi, the outer ones bend to the other side, so their double
            // curves are centered around the opposite v
            SweptCircle sc = MakeSurface(-4.5, 4.5, radius);
            ICurve2D[] si = sc.GetSelfIntersections(fullDomain);
            Assert.IsNotNull(si);
            Assert.AreEqual(6, si.Length, "three folds must yield three pairs of branches");
            List<double> centers = new List<double>();
            for (int i = 0; i < si.Length; i += 2)
            {
                AssertIsDoubleCurve(sc, si[i], si[i + 1]);
                double vm = (si[i].StartPoint.y + si[i].EndPoint.y) / 2.0;
                Assert.IsTrue(Math.Abs(Math.Abs(Math.Sin(vm)) - 1.0) < 1e-6, $"the double curve is not centered at +-pi/2: {vm}");
                Assert.IsTrue(vm >= 0.0 && vm <= 2 * Math.PI, $"the double curve was not moved into the domain: {vm}");
                centers.Add(vm);
            }
            centers.Sort();
            Assert.IsTrue(Math.Abs(centers[2] - centers[0] - Math.PI) < 1e-6,
                $"the middle fold must be centered at the opposite side of the other two: {centers[0]}, {centers[1]}, {centers[2]}");
            Assert.IsTrue(Math.Abs(centers[1] - centers[0]) < 1e-6 || Math.Abs(centers[2] - centers[1]) < 1e-6,
                "two of the three folds bend to the same side, so two of the centers must be equal");
        }

        [TestMethod]
        public void no_self_intersection_when_the_radius_is_small_enough()
        {
            SweptCircle sc = MakeSurface(-1.3, 1.3, 0.5); // the curvature radius of the spine stays above 1
            Assert.IsNull(sc.GetSelfIntersections(fullDomain));
        }

        [TestMethod]
        public void nothing_is_returned_outside_the_given_bounds()
        {
            SweptCircle sc = MakeSurface(-1.3, 1.3, radius);
            ICurve2D[] si = sc.GetSelfIntersections(fullDomain);
            double vCenter = (si[0].StartPoint.y + si[0].EndPoint.y) / 2.0;
            double halfWidth = (si[0].EndPoint.y - si[0].StartPoint.y) / 2.0;
            // a domain which starts above the double curve, but well below the next period
            BoundingRect beside = new BoundingRect(0, vCenter + halfWidth + 0.1, 1, vCenter + halfWidth + 0.5);
            Assert.IsNull(sc.GetSelfIntersections(beside));
        }
        [TestMethod]
        public void outer_shell_without_a_fold_is_a_single_face()
        {
            SweptCircle sc = MakeSurface(-1.3, 1.3, 0.5);
            Face[] faces = sc.OuterShell(0.0, 2 * Math.PI);
            Assert.IsNotNull(faces);
            Assert.AreEqual(1, faces.Length, "without a self intersection nothing has to be split");
            Assert.IsTrue(Math.Abs(faces[0].Area.Area - 2 * Math.PI) < 1e-6, "the single face must cover the whole domain");
        }

        [TestMethod]
        public void outer_shell_of_a_single_fold_is_split_into_two_faces()
        {
            SweptCircle sc = MakeSurface(-1.3, 1.3, radius);
            ICurve2D[] si = sc.GetSelfIntersections(fullDomain);
            double uVertex = si[0].StartPoint.x;
            double vlow = si[0].StartPoint.y, vhigh = si[0].EndPoint.y;
            Face[] faces = sc.OuterShell(0.0, 2 * Math.PI);
            Assert.IsNotNull(faces);
            Assert.AreEqual(2, faces.Length, "a single fold splits the surface into two faces");
            // the hidden part inside the double curve must not belong to any face
            foreach (Face f in faces)
            {
                Assert.IsNotNull(f);
                Assert.IsFalse(f.Area.Contains(new GeoPoint2D(uVertex, (vlow + vhigh) / 2.0), false), "the hidden part is still covered");
            }
            // and exactly the area enclosed by the double curve must be missing
            double lens = 0.0;
            int n = 2000;
            for (int i = 0; i < n; i++)
            {
                double v = vlow + (vhigh - vlow) * (i + 0.5) / n;
                lens += (PointAtV(si[0], v).x - PointAtV(si[1], v).x) * (vhigh - vlow) / n;
            }
            double covered = faces[0].Area.Area + faces[1].Area.Area;
            Assert.IsTrue(lens > 0.01, "the enclosed area is suspiciously small");
            Assert.IsTrue(Math.Abs(2 * Math.PI - covered - lens) < 1e-3,
                $"cut away {2 * Math.PI - covered}, but the double curve encloses {lens}");
        }

        [TestMethod]
        public void outer_shell_faces_do_not_contain_the_fold_curve()
        {
            // inside the double curve the surface normal is flipped, that part must not be part of any face
            SweptCircle sc = MakeSurface(-1.3, 1.3, radius);
            Face[] faces = sc.OuterShell(0.0, 2 * Math.PI);
            int tested = 0;
            foreach (Face f in faces)
            {
                BoundingRect ext = f.Area.GetExtent();
                for (int i = 1; i < 40; i++)
                {
                    for (int j = 1; j < 40; j++)
                    {
                        GeoPoint2D uv = new GeoPoint2D(ext.Left + ext.Width * i / 40.0, ext.Bottom + ext.Height * j / 40.0);
                        if (!f.Area.Contains(uv, false)) continue;
                        ++tested;
                        Assert.IsTrue(sc.UDirection(uv) * sc.Spine.DirectionAt(uv.x) > 0.0,
                            $"the surface normal is flipped at {uv}, so the fold lies inside a face");
                    }
                }
            }
            Assert.IsTrue(tested > 500, $"only {tested} points inside the faces were tested");
        }

        [TestMethod]
        public void outer_shell_of_three_folds_has_four_faces()
        {
            SweptCircle sc = MakeSurface(-4.5, 4.5, radius);
            Face[] faces = sc.OuterShell(0.0, 2 * Math.PI);
            Assert.IsNotNull(faces);
            Assert.AreEqual(4, faces.Length, "three folds split the surface into four faces");
            double covered = 0.0;
            foreach (Face f in faces)
            {
                Assert.IsNotNull(f);
                covered += f.Area.Area;
            }
            Assert.IsTrue(covered < 2 * Math.PI - 0.05, "nothing was cut away");
        }
    }
}
