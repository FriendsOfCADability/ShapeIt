using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;

namespace CADability.Tests
{
    /// <summary>
    /// Regression tests for faces whose triangulation basis polygons are degenerate: loops that
    /// touch each other in a single point, and loops that intersect each other because two curves
    /// were approximated with a different number of points. See <see cref="CDTriangulation"/> and
    /// Face.Triangulate.
    /// </summary>
    [TestClass]
    public class TriangulationRegressionTests
    {
        public TestContext TestContext { get; set; }

        /// <summary>signed area of the triangulation in uv, counterclockwise triangles counting positive</summary>
        private static double MeshArea2d(GeoPoint2D[] uv, int[] idx)
        {
            double a = 0.0;
            for (int i = 0; i < idx.Length; i += 3)
            {
                a += 0.5 * ((uv[idx[i + 1]].x - uv[idx[i]].x) * (uv[idx[i + 2]].y - uv[idx[i]].y)
                          - (uv[idx[i + 1]].y - uv[idx[i]].y) * (uv[idx[i + 2]].x - uv[idx[i]].x));
            }
            return a;
        }

        /// <summary>no directed edge twice (no overlapping triangles) and boundary edges form closed loops</summary>
        private static void AssertStructurallyValid(GeoPoint2D[] uv, int[] idx, string what)
        {
            HashSet<long> directed = new HashSet<long>();
            for (int i = 0; i < idx.Length; i += 3)
            {
                int a = idx[i], b = idx[i + 1], c = idx[i + 2];
                Assert.IsTrue(a != b && b != c && a != c, what + ": degenerate triangle");
                foreach ((int s, int e) in new[] { (a, b), (b, c), (c, a) })
                {
                    Assert.IsTrue(directed.Add(((long)(uint)s << 32) | (uint)e),
                        what + ": directed edge used twice - overlapping triangles");
                }
            }
            Dictionary<int, int> outDeg = new Dictionary<int, int>(), inDeg = new Dictionary<int, int>();
            foreach (long key in directed)
            {
                int s = (int)(key >> 32), e = (int)(key & 0xffffffff);
                if (directed.Contains(((long)(uint)e << 32) | (uint)s)) continue;
                outDeg.TryGetValue(s, out int od); outDeg[s] = od + 1;
                inDeg.TryGetValue(e, out int id); inDeg[e] = id + 1;
            }
            foreach (KeyValuePair<int, int> kv in outDeg)
            {
                inDeg.TryGetValue(kv.Key, out int id);
                Assert.AreEqual(kv.Value, id, what + ": boundary is not closed at vertex " + kv.Key);
            }
        }

        /// <summary>
        /// Two holes of this face touch in a single point. Both loops carry that point, but computed
        /// from two different curves it differs by 3.9e-14, and the tip of the lower hole ends up
        /// above the tip of the upper one - to the exact predicates the two holes overlap and their
        /// loops cross. Before CDTriangulation merged coincident input vertices, one of the crossing
        /// constraints was dropped (innerIntersection) and the gap let the inside/outside flood fill
        /// leak: triangles outside the face survived and triangles inside it were removed.
        /// </summary>
        [TestMethod]
        [DeploymentItem(@"Files/Faces/TriangulationBug.cdb.json")]
        public void holes_touching_in_one_point_do_not_leak()
        {
            string path = System.IO.Path.Combine(this.TestContext.DeploymentDirectory, "TriangulationBug.cdb.json");
            Assert.IsTrue(File.Exists(path), "TriangulationBug.cdb.json missing");
            Project pr;
            using (FileStream stream = File.Open(path, FileMode.Open)) pr = new JsonSerialize().FromStream(stream) as Project;
            Assert.IsNotNull(pr, "could not load the project");

            List<Face> faces = new List<Face>();
            foreach (IGeoObject go in pr.GetActiveModel())
            {
                if (go is Face f) faces.Add(f);
                else if (go is Shell sh) faces.AddRange(sh.Faces);
                else if (go is Solid so) faces.AddRange(so.Shells[0].Faces);
            }
            Assert.AreNotEqual(0, faces.Count, "no face in the project");

            const double precision = 0.076973493975904292;
            foreach (Face face in faces)
            {
                face.GetTriangulation(precision, out GeoPoint[] p3d, out GeoPoint2D[] uv, out int[] idx, out BoundingBox bb);
                Assert.IsTrue(idx.Length > 0, "no triangles produced");
                AssertStructurallyValid(uv, idx, "touching holes");
                // the mesh is the polygonal approximation of a face bounded by arcs, so it stays
                // slightly below the exact area - but it must never exceed it
                double mesh = MeshArea2d(uv, idx);
                double exact = face.Area.Area;
                Assert.IsTrue(mesh <= exact,
                    "mesh covers area outside the face: " + mesh.ToString("F6") + " > " + exact.ToString("F6"));
                Assert.IsTrue(mesh > exact * 0.99,
                    "mesh is missing parts of the face: " + mesh.ToString("F6") + " vs " + exact.ToString("F6"));
            }
        }

        /// <summary>
        /// A ring so thin that the polygons approximating its two circles intersect each other (the
        /// chords of the outer circle reach inside the inner one). Face.Triangulate detects this
        /// (innerIntersection) and retriangulates the pieces the outline and the holes split into.
        /// Without that the flood fill filled the whole disc - 312 instead of 0.63 square units.
        /// </summary>
        [TestMethod]
        public void thin_ring_is_not_filled_completely()
        {
            const double outerRadius = 10.0;
            foreach (double innerRadius in new[] { 9.99, 9.999 })
            {
                Border outer = new Border(new Arc2D(GeoPoint2D.Origin, outerRadius, 0.0, Math.PI * 2.0));
                // a different start angle, so the two polygons do not share their vertex angles
                Border inner = new Border(new Arc2D(GeoPoint2D.Origin, innerRadius, 0.3, Math.PI * 2.0));
                Face face = Face.MakeFace(new PlaneSurface(Plane.XYPlane), new SimpleShape(outer, inner));

                face.GetTriangulation(0.05, out GeoPoint[] p3d, out GeoPoint2D[] uv, out int[] idx, out BoundingBox bb);
                Assert.IsTrue(idx.Length > 0, "no triangles produced");
                AssertStructurallyValid(uv, idx, "thin ring r=" + innerRadius);

                double mesh = MeshArea2d(uv, idx);
                double disc = Math.PI * outerRadius * outerRadius;
                Assert.IsTrue(mesh < 0.05 * disc,
                    "the hole of the ring was filled: mesh area " + mesh.ToString("F6") + " of " + disc.ToString("F6"));
            }
        }
    }
}
