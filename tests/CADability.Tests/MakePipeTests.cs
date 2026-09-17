using CADability;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using System;
using System.Linq;
using Path = CADability.GeoObject.Path;

namespace CADability.Tests
{
    /// <summary>
    /// <see cref="Make3D.MakePipe(IGeoObject, Path, Project)"/> on paths that are neither a straight line nor a
    /// circular arc - the case that used to come back as null, because Make3D.ExtrudeCurveToFace gave up on
    /// anything else. It now builds a <see cref="SweptCurveSurface"/> for those.
    /// <para>
    /// The profile here is always the exact nine pole NURBS circle rather than an Ellipse, because an Ellipse
    /// would take one of the older branches and the test would be about a cylinder or a torus instead.
    /// </para>
    /// </summary>
    [TestClass]
    public class MakePipeTests
    {
        private static BSpline2D NurbsCircle(double r)
        {
            GeoPoint2D[] poles =
            {
                new GeoPoint2D(r, 0), new GeoPoint2D(r, r), new GeoPoint2D(0, r), new GeoPoint2D(-r, r),
                new GeoPoint2D(-r, 0), new GeoPoint2D(-r, -r), new GeoPoint2D(0, -r), new GeoPoint2D(r, -r),
                new GeoPoint2D(r, 0)
            };
            double w = Math.Sqrt(2.0) / 2.0;
            return new BSpline2D(poles, new double[] { 1, w, 1, w, 1, w, 1, w, 1 },
                                 new double[] { 0, 1, 2, 3, 4 }, new int[] { 3, 2, 2, 2, 3 }, 2, false, 0, 4);
        }

        private static Face CircularProfile(Plane plane, double r)
            => Face.MakeFace(new PlaneSurface(plane), new SimpleShape(new Border(NurbsCircle(r))));

        private static Path PathOf(params ICurve[] curves)
        {
            Path path = Path.FromSegments(curves, true);
            path.Flatten();
            return path;
        }

        /// <summary>Everything a swept solid has to satisfy whatever its shape, plus its volume.</summary>
        private static void AssertSoundSolid(string what, IGeoObject result, double expectedVolume, double tolerance)
        {
            Assert.IsInstanceOfType(result, typeof(Solid), $"{what}: MakePipe produced no solid");
            Shell shell = ((Solid)result).Shell;
            Assert.IsTrue(shell.CheckConsistency(), $"{what}: the shell is not consistent");
            Assert.AreEqual(0, shell.OpenEdges.Length, $"{what}: the shell is not closed");
            Assert.AreEqual(0, shell.Edges.Count(e => e.Curve3D != null && e.Vertex1 == e.Vertex2),
                $"{what}: no edge may be closed on itself - that is what SplitSingleOutlines is for");
            double volume = ((Solid)result).Volume(shell.GetExtent(0.0).Size / 4000.0);
            Assert.IsTrue(volume > 0.0, $"{what}: the volume is {volume}, so the shell is inside out");
            Assert.AreEqual(expectedVolume, volume, Math.Abs(expectedVolume) * tolerance,
                $"{what}: volume {volume} instead of {expectedVolume}");
        }

        [TestMethod]
        public void a_nurbs_profile_along_a_straight_line_is_a_cylinder()
        {
            IGeoObject pipe = Make3D.MakePipe(CircularProfile(Plane.XYPlane, 5.0),
                PathOf(Line.TwoPoints(GeoPoint.Origin, new GeoPoint(0.0, 0.0, 20.0))), null);
            AssertSoundSolid("circle along a line", pipe, Math.PI * 25.0 * 20.0, 1e-4);
        }

        [TestMethod]
        public void a_nurbs_profile_along_half_a_nurbs_circle_is_half_a_torus()
        {
            Plane profilePlane = new Plane(new GeoPoint(30.0, 0.0, 0.0), GeoVector.XAxis, GeoVector.ZAxis);
            ICurve half = ((ICurve)NurbsCircle(30.0).MakeGeoObject(Plane.XYPlane)).Split(0.5)[0];
            IGeoObject pipe = Make3D.MakePipe(CircularProfile(profilePlane, 8.0), PathOf(half), null);
            AssertSoundSolid("circle along half a circle", pipe, Math.PI * Math.PI * 30.0 * 64.0, 1e-4);
        }

        [TestMethod]
        public void a_nurbs_profile_along_a_closed_nurbs_circle_is_a_torus()
        {
            // The case the whole exercise is about: both curves are exact NURBS circles, nothing in it is a
            // primitive, and the path is CLOSED and consists of a single curve - which MakePipe has to split.
            Plane profilePlane = new Plane(new GeoPoint(30.0, 0.0, 0.0), GeoVector.XAxis, GeoVector.ZAxis);
            ICurve major = (ICurve)NurbsCircle(30.0).MakeGeoObject(Plane.XYPlane);
            Path path = PathOf(major);
            Assert.AreEqual(1, path.CurveCount, "the premise: the path is one single closed curve");

            IGeoObject pipe = Make3D.MakePipe(CircularProfile(profilePlane, 8.0), path, null);
            AssertSoundSolid("torus", pipe, 2.0 * Math.PI * Math.PI * 30.0 * 64.0, 1e-4);

            Shell shell = ((Solid)pipe).Shell;
            // A torus has no caps: the path closes and comes back with the same direction, so the start and
            // end faces fall away. Four patches, and the Euler characteristic of a torus is zero.
            Assert.IsTrue(shell.Faces.All(f => f.Surface is SweptCurveSurface),
                "a closed path leaves no start or end face, so every face is a swept one");
            Assert.AreEqual(0, shell.Vertices.Length - shell.Edges.Length + shell.Faces.Length,
                "V - E + F has to be 0 for a torus");
        }

        // ------------------------------------------------------------------------------- the two laws --

        /// <summary>
        /// A spine that bends but never tightly enough for the pipe to run into itself: its smallest radius of
        /// curvature is 5.99 against a profile radius of 3. It runs from y = 0 to y = 40 and wanders in x and z.
        /// </summary>
        private static ICurve WavySpine()
        {
            BSpline spine = BSpline.Construct();
            spine.ThroughPoints(new GeoPoint[]
            {
                new GeoPoint(0.0, 0.0, 0.0), new GeoPoint(2.0, 10.0, 1.0), new GeoPoint(-2.0, 20.0, -1.0),
                new GeoPoint(1.0, 30.0, 2.0), new GeoPoint(0.0, 40.0, 0.0)
            }, 3, false);
            return spine;
        }

        /// <summary>The arc length of a curve, sampled finely - ICurve.Length of a spline is a chord polygon.</summary>
        private static double ArcLengthOf(ICurve curve)
        {
            double length = 0.0;
            GeoPoint previous = curve.PointAt(0.0);
            for (int i = 1; i <= 20000; i++)
            {
                GeoPoint next = curve.PointAt(i / 20000.0);
                length += next | previous;
                previous = next;
            }
            return length;
        }

        [TestMethod]
        public void following_the_path_sweeps_the_profile_over_the_whole_arc_length()
        {
            // Pappus: a profile that stands perpendicular to the path and is carried along perpendicular
            // sweeps exactly its area times the length of the path. That pins the law and the parametrisation
            // at once - a frame that lagged or ran ahead would not come out at the arc length.
            ICurve spine = WavySpine();
            spine.StartDirection.Normalized.ArbitraryNormals(out GeoVector dx, out GeoVector dy);
            Face profile = CircularProfile(new Plane(spine.StartPoint, dx, dy), 3.0);

            IGeoObject pipe = Make3D.MakePipe(profile, PathOf(spine), null, SweepOrientation.Follow);
            AssertSoundSolid("follow along a wavy spine", pipe, Math.PI * 9.0 * ArcLengthOf(spine), 1e-3);
        }

        [TestMethod]
        public void a_fixed_profile_is_only_carried_along_and_never_turned()
        {
            // The profile lies in the xz plane and stays there, so the solid reaches exactly as far in y as the
            // path does - a profile that turned would lean out of that range - and the volume is its area times
            // the distance travelled in y, whatever detour the path takes in x and z on the way.
            ICurve spine = WavySpine();
            Face profile = CircularProfile(new Plane(GeoPoint.Origin, GeoVector.XAxis, GeoVector.ZAxis), 3.0);

            IGeoObject pipe = Make3D.MakePipe(profile, PathOf(spine), null, SweepOrientation.Fixed);
            AssertSoundSolid("fixed along a wavy spine", pipe, Math.PI * 9.0 * 40.0, 1e-3);

            BoundingBox box = ((Solid)pipe).Shell.GetExtent(((Solid)pipe).Shell.GetExtent(0.0).Size / 4000.0);
            Assert.AreEqual(0.0, box.Ymin, 1e-3, "a profile that is never turned cannot reach below the path");
            Assert.AreEqual(40.0, box.Ymax, 1e-3, "...nor beyond its end");
        }

        [TestMethod]
        public void the_two_laws_do_not_produce_the_same_solid()
        {
            // Guards against the orientation being accepted and then quietly ignored, which is what used to
            // happen: solid.sweep declared it and SolidSweepImpl never read it.
            ICurve spine = WavySpine();
            spine.StartDirection.Normalized.ArbitraryNormals(out GeoVector dx, out GeoVector dy);
            Plane perpendicular = new Plane(spine.StartPoint, dx, dy);

            Solid following = (Solid)Make3D.MakePipe(CircularProfile(perpendicular, 3.0), PathOf(spine), null,
                                                     SweepOrientation.Follow);
            Solid carried = (Solid)Make3D.MakePipe(CircularProfile(perpendicular, 3.0), PathOf(spine), null,
                                                   SweepOrientation.Fixed);
            double a = following.Volume(following.Shell.GetExtent(0.0).Size / 4000.0);
            double b = carried.Volume(carried.Shell.GetExtent(0.0).Size / 4000.0);
            Assert.IsTrue(Math.Abs(a - b) > 0.1 * Math.Max(a, b),
                $"follow and fixed have to differ on a bent path, but gave {a} and {b}");
        }

        [TestMethod]
        public void the_extent_of_the_torus_is_exact()
        {
            Plane profilePlane = new Plane(new GeoPoint(30.0, 0.0, 0.0), GeoVector.XAxis, GeoVector.ZAxis);
            Path path = PathOf((ICurve)NurbsCircle(30.0).MakeGeoObject(Plane.XYPlane));
            Solid torus = (Solid)Make3D.MakePipe(CircularProfile(profilePlane, 8.0), path, null);
            BoundingBox box = torus.Shell.GetExtent(torus.Shell.GetExtent(0.0).Size / 4000.0);
            foreach ((string what, double actual, double expected) in new (string, double, double)[]
            {
                ("xmin", box.Xmin, -38.0), ("xmax", box.Xmax, 38.0),
                ("ymin", box.Ymin, -38.0), ("ymax", box.Ymax, 38.0),
                ("zmin", box.Zmin, -8.0), ("zmax", box.Zmax, 8.0)
            })
            {
                Assert.AreEqual(expected, actual, 1e-3, $"{what} of the torus");
            }
        }
    }
}
