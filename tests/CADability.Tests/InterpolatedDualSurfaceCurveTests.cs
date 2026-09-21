using CADability.Curve2D;
using CADability.GeoObject;
using System.Reflection;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for single defects of <see cref="InterpolatedDualSurfaceCurve"/> and its nested ProjectedCurve. The
    /// curves are made from exact points of two simple intersections: an ellipse, where a plane cuts a cylinder
    /// obliquely, and a line, where a plane touches a cylinder.
    /// </summary>
    [TestClass]
    public class InterpolatedDualSurfaceCurveTests
    {
        private const double radius = 10.0;

        /// <summary>The cylinder around the z-axis with <see cref="radius"/>.</summary>
        private static CylindricalSurface Cylinder()
        {
            return new CylindricalSurface(GeoPoint.Origin, radius * GeoVector.XAxis, radius * GeoVector.YAxis, GeoVector.ZAxis);
        }

        private static BoundingRect FullCylinder => new BoundingRect(0, -20, 2 * System.Math.PI, 20);
        private static BoundingRect Wide => new BoundingRect(-30, -30, 30, 30);

        /// <summary>The plane z = x/2, which cuts the cylinder in an ellipse.</summary>
        private static PlaneSurface ObliquePlane()
        {
            return new PlaneSurface(new Plane(GeoPoint.Origin, new GeoVector(-0.5, 0.0, 1.0)));
        }

        /// <summary>The point of the ellipse at the angle <paramref name="t"/> around the axis.</summary>
        private static GeoPoint OnEllipse(double t)
        {
            return new GeoPoint(radius * System.Math.Cos(t), radius * System.Math.Sin(t), 0.5 * radius * System.Math.Cos(t));
        }

        /// <summary>
        /// An arc of the ellipse from exact base points, which are spaced very unevenly on purpose: dense at the
        /// start, wide at the end. So the parameter of a base point is far from i/(n-1).
        /// </summary>
        private static InterpolatedDualSurfaceCurve Ellipse(bool isTangential = false)
        {
            double[] fractions = { 0.0, 0.02, 0.05, 0.1, 0.17, 0.27, 0.42, 0.65, 1.0 };
            GeoPoint[] points = new GeoPoint[fractions.Length];
            for (int i = 0; i < fractions.Length; i++) points[i] = OnEllipse(0.3 + 2.5 * fractions[i]);
            return new InterpolatedDualSurfaceCurve(Cylinder(), FullCylinder, ObliquePlane(), Wide, points, null, null, isTangential);
        }

        /// <summary>The line x = radius, y = 0, where the plane x = radius touches the cylinder: a tangential intersection.</summary>
        private static InterpolatedDualSurfaceCurve TangentialLine()
        {
            PlaneSurface touching = new PlaneSurface(new Plane(new GeoPoint(radius, 0, 0), GeoVector.XAxis));
            GeoPoint[] points = { new GeoPoint(radius, 0, -5), new GeoPoint(radius, 0, -1), new GeoPoint(radius, 0, 2), new GeoPoint(radius, 0, 5) };
            return new InterpolatedDualSurfaceCurve(Cylinder(), FullCylinder, touching, Wide, points, null, null, true);
        }

        private static void AssertClose(GeoPoint expected, GeoPoint actual, double precision, string message)
        {
            Assert.AreEqual(0.0, expected | actual, precision, message + ": expected " + expected.ToString() + ", was " + actual.ToString());
        }

        /// <summary>Calls the internal CloneTrimmed, which Edge.Split uses for an edge whose 2d curves belong to its 3d curve.</summary>
        private static InterpolatedDualSurfaceCurve CloneTrimmed(InterpolatedDualSurfaceCurve curve, double startPos, double endPos)
        {
            MethodInfo method = typeof(InterpolatedDualSurfaceCurve).GetMethod("CloneTrimmed", BindingFlags.NonPublic | BindingFlags.Instance);
            object[] args = { startPos, endPos, curve.CurveOnSurface1, curve.CurveOnSurface2, null, null };
            return (InterpolatedDualSurfaceCurve)method.Invoke(curve, args);
        }

        /// <summary>
        /// CloneTrimmed handed forwardOriented to a constructor whose fourth parameter is isTangential, so the
        /// pieces of a split edge got a flag that depended on the orientation instead of the geometry.
        /// </summary>
        [TestMethod]
        public void clone_trimmed_keeps_the_tangential_flag()
        {
            foreach (bool reverse in new[] { false, true })
            {   // n1 x n2 runs along the curve in one of the two orientations and against it in the other
                InterpolatedDualSurfaceCurve ellipse = Ellipse();
                if (reverse) ellipse.Reverse();
                InterpolatedDualSurfaceCurve trimmed = CloneTrimmed(ellipse, 0.2, 0.7);
                Assert.IsFalse(trimmed.IsTangential, "a transversal intersection stays transversal, reversed: " + reverse);
                AssertClose(ellipse.PointAt(0.2), trimmed.StartPoint, 1e-4, "start of the trimmed curve");
                AssertClose(ellipse.PointAt(0.7), trimmed.EndPoint, 1e-4, "end of the trimmed curve");

                InterpolatedDualSurfaceCurve line = TangentialLine();
                if (reverse) line.Reverse();
                Assert.IsTrue(CloneTrimmed(line, 0.2, 0.7).IsTangential, "a tangential intersection stays tangential, reversed: " + reverse);
            }
        }

        /// <summary>
        /// Files written while CloneTrimmed had its defect contain transversal curves marked as tangential. Such a
        /// curve computes each of its points with the solver for touching surfaces, which fails, and silently falls
        /// back to the unrefined point. Reading the file corrects the flag.
        /// </summary>
        [TestMethod]
        public void a_wrong_tangential_flag_is_corrected_when_read()
        {
            InterpolatedDualSurfaceCurve wrong = Ellipse(isTangential: true);
            InterpolatedDualSurfaceCurve read = JsonSerialize.FromString(JsonSerialize.ToString(wrong)) as InterpolatedDualSurfaceCurve;
            Assert.IsNotNull(read);
            Assert.IsFalse(read.IsTangential, "the surfaces intersect transversally, the flag is corrected");

            read = JsonSerialize.FromString(JsonSerialize.ToString(TangentialLine())) as InterpolatedDualSurfaceCurve;
            Assert.IsNotNull(read);
            Assert.IsTrue(read.IsTangential, "a tangential curve keeps its flag");
        }

        /// <summary>
        /// Split assumed that base point i lies at the parameter i/(n-1). With unevenly spaced base points the new
        /// point was inserted between the wrong base points, and the second part ran back over the first.
        /// </summary>
        [TestMethod]
        public void split_inserts_the_new_point_where_it_belongs()
        {
            InterpolatedDualSurfaceCurve curve = Ellipse();
            foreach (double at in new[] { 0.3, 0.6 })
            {
                ICurve[] parts = curve.Split(at);
                Assert.AreEqual(2, parts.Length);
                AssertClose(curve.StartPoint, parts[0].StartPoint, 1e-9, "start of the first part");
                AssertClose(curve.EndPoint, parts[1].EndPoint, 1e-9, "end of the second part");
                AssertClose(parts[0].EndPoint, parts[1].StartPoint, 1e-9, "the parts meet");
                AssertClose(curve.PointAt(at), parts[0].EndPoint, 1e-4, "they meet at the split position");
                for (int k = 0; k < 2; k++)
                {
                    Assert.IsFalse((parts[k] as InterpolatedDualSurfaceCurve).IsTangential, "the flag is kept");
                    double from = k == 0 ? 0.0 : at, to = k == 0 ? at : 1.0;
                    double previous = from;
                    for (int i = 0; i <= 20; i++)
                    {   // every point of a part lies on its own section of the curve, and the part runs forward
                        GeoPoint p = parts[k].PointAt(i / 20.0);
                        double t = curve.PositionOf(p);
                        AssertClose(curve.PointAt(t), p, 1e-4, "point " + i + " of part " + k + " lies on the curve");
                        Assert.IsTrue(t >= from - 1e-3 && t <= to + 1e-3, "point " + i + " of part " + k + " lies in [" + from + ", " + to + "], but at " + t);
                        Assert.IsTrue(t >= previous - 1e-3, "part " + k + " runs forward: " + previous + " then " + t);
                        previous = t;
                    }
                }
            }
        }

        /// <summary>
        /// PositionOf with a plane threw "not implemented", and FindSnapPoint calls it to snap to a point on the curve.
        /// </summary>
        [TestMethod]
        public void position_of_a_point_seen_through_a_plane()
        {
            InterpolatedDualSurfaceCurve curve = Ellipse();
            foreach (double t in new[] { 0.1, 0.37, 0.8 })
            {   // a point above the curve, seen from above
                GeoPoint above = curve.PointAt(t) + 3.0 * GeoVector.ZAxis;
                Assert.AreEqual(t, curve.PositionOf(above, Plane.XYPlane), 1e-6, "seen through the xy-plane");
                Assert.AreEqual(t, curve.PositionOf(curve.PointAt(t), 0.5), 1e-6, "with a preferred position");
            }
        }

        /// <summary>
        /// The extent was taken from the base points only, but the curve bulges out between them. The octree and
        /// hit tests rely on the extent containing the whole curve.
        /// </summary>
        [TestMethod]
        public void the_extent_contains_the_whole_curve()
        {
            InterpolatedDualSurfaceCurve curve = Ellipse();
            BoundingBox extent = curve.GetExtent(0.0);
            BoundingBox cube = curve.GetBoundingCube();
            for (int i = 0; i <= 1000; i++)
            {
                GeoPoint p = curve.PointAt(i / 1000.0);
                Assert.IsTrue(extent.Contains(p, 1e-9), "GetExtent contains the point at " + (i / 1000.0) + ": " + p.ToString());
                Assert.IsTrue(cube.Contains(p, 1e-9), "GetBoundingCube contains the point at " + (i / 1000.0) + ": " + p.ToString());
            }
        }

        /// <summary>
        /// Moving the 2d curve by a period shifted the uv values stored in the 3d curve, but not the approximation, which
        /// is computed from the 3d points: afterwards StartPoint and EndPoint were a period away from PointAt, and the
        /// 3d curve, which is shared with the edge and the other 2d curve, had been changed.
        /// Shell.CombineConnectedFaces measures the extent of a moved outline this way.
        /// </summary>
        [TestMethod]
        public void moving_a_2d_curve_by_a_period_moves_all_of_it()
        {
            InterpolatedDualSurfaceCurve curve = Ellipse();
            ICurve2D c2d = curve.CurveOnSurface1; // on the cylinder, u is periodic
            GeoPoint2D start = c2d.StartPoint, end = c2d.EndPoint, middle = c2d.PointAt(0.5);
            GeoVector2D period = new GeoVector2D(2 * System.Math.PI, 0.0);
            c2d.Move(period.x, period.y);
            Assert.AreEqual(0.0, (start + period) | c2d.StartPoint, 1e-9, "the start point is moved");
            Assert.AreEqual(0.0, (end + period) | c2d.EndPoint, 1e-9, "the end point is moved");
            Assert.AreEqual(0.0, (middle + period) | c2d.PointAt(0.5), 1e-6, "the curve is moved");
            Assert.AreEqual(0.0, c2d.StartPoint | c2d.PointAt(0.0), 1e-6, "start point and curve agree");
            Assert.IsTrue(c2d.GetExtent().Left > start.x + 0.5 * period.x, "the extent is moved");
            // the 3d curve is shared with the edge and the other 2d curve, it is not changed
            Assert.AreEqual(0.0, start | curve.CurveOnSurface1.StartPoint, 1e-9, "a new 2d curve of the 3d curve is where the old one was");
            ICurve2D reversed = c2d.CloneReverse(true);
            Assert.AreEqual(0.0, (end + period) | reversed.StartPoint, 1e-9, "a copy keeps the move");
        }

        /// <summary>
        /// TryPointDeriv2At of the 2d curve ignored that the curve may be reversed and returned the point of the
        /// opposite position.
        /// </summary>
        [TestMethod]
        public void a_reversed_2d_curve_derives_where_it_is()
        {
            foreach (bool onSurface1 in new[] { true, false })
            {
                InterpolatedDualSurfaceCurve curve = Ellipse();
                ICurve2D c2d = onSurface1 ? curve.CurveOnSurface1 : curve.CurveOnSurface2;
                c2d.Reverse();
                foreach (double t in new[] { 0.2, 0.5, 0.9 })
                {
                    Assert.IsTrue(c2d.TryPointDeriv2At(t, out GeoPoint2D point, out GeoVector2D deriv, out GeoVector2D _));
                    GeoPoint2D expected = c2d.PointAt(t);
                    Assert.AreEqual(0.0, point | expected, 1e-9, "point at " + t + " on surface " + (onSurface1 ? 1 : 2));
                    GeoVector2D direction = c2d.DirectionAt(t);
                    Assert.AreEqual(0.0, (deriv - direction).Length, 1e-6 * direction.Length, "derivative at " + t + " on surface " + (onSurface1 ? 1 : 2));
                }
            }
        }

        /// <summary>The tangent of the ellipse at the angle <paramref name="t"/>, in the direction of growing t.</summary>
        private static GeoVector EllipseTangent(double t)
        {
            return new GeoVector(-radius * System.Math.Sin(t), radius * System.Math.Cos(t), -0.5 * radius * System.Math.Sin(t));
        }

        private static void AssertSameDirection(GeoVector expected, GeoVector actual, string message)
        {
            Assert.IsTrue(expected * actual > 0.0, message + ": runs the other way");
            Assert.AreEqual(0.0, (expected.Normalized ^ actual.Normalized).Length, 1e-9, message + ": not parallel");
        }

        /// <summary>
        /// StartDirection and EndDirection are n1 x n2 at the end points, the exact tangent up to its sign. The sign was
        /// a stored flag, which Reverse, SwapSurfaces and every change of a surface orientation had to turn round. Now
        /// the approximating spline decides it.
        /// </summary>
        [TestMethod]
        public void start_and_end_direction_run_along_the_curve()
        {
            foreach (bool reverse in new[] { false, true })
            {
                foreach (bool swap in new[] { false, true })
                {
                    InterpolatedDualSurfaceCurve curve = Ellipse();
                    if (reverse) curve.Reverse();
                    if (swap) (curve as IDualSurfaceCurve).SwapSurfaces();
                    string what = "reversed: " + reverse + ", surfaces swapped: " + swap;
                    // the ellipse runs from the angle 0.3 to 2.8
                    AssertSameDirection(reverse ? -EllipseTangent(2.8) : EllipseTangent(0.3), curve.StartDirection, "start direction, " + what);
                    AssertSameDirection(reverse ? -EllipseTangent(0.3) : EllipseTangent(2.8), curve.EndDirection, "end direction, " + what);
                }
            }
        }

        /// <summary>
        /// Older versions need "ForwardOriented" to read a file. The curve no longer keeps it but computes it when
        /// writing, so it has to follow the orientation of the curve and the order of the surfaces.
        /// </summary>
        [TestMethod]
        public void the_orientation_written_for_older_versions_follows_the_curve()
        {
            static bool Written(string json)
            {
                System.Text.RegularExpressions.Match m = System.Text.RegularExpressions.Regex.Match(json, "\"ForwardOriented\":(true|false)");
                Assert.IsTrue(m.Success, "ForwardOriented is written");
                return m.Groups[1].Value == "true";
            }
            InterpolatedDualSurfaceCurve curve = Ellipse();
            bool forward = Written(JsonSerialize.ToString(curve));
            curve.Reverse();
            Assert.AreEqual(!forward, Written(JsonSerialize.ToString(curve)), "the reversed curve");
            (curve as IDualSurfaceCurve).SwapSurfaces();
            Assert.AreEqual(forward, Written(JsonSerialize.ToString(curve)), "the reversed curve with swapped surfaces");
        }
    }
}
