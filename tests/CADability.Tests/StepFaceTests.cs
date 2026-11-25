using CADability.GeoObject;
using System.Runtime.CompilerServices;
using IOPath = System.IO.Path;

namespace CADability.Tests
{
    /// <summary>
    /// Creation of single faces in the STEP import (Face.MakeFacesFromStepAdvancedFace).
    /// </summary>
    [TestClass]
    public class StepFaceTests
    {
        private static string StepFile(string name, [CallerFilePath] string thisFile = "")
            => IOPath.Combine(IOPath.GetDirectoryName(thisFile)!, "Files", "Step", name);

        private static IEnumerable<Face> Faces(GeoObjectList list)
        {
            foreach (IGeoObject go in list)
            {
                if (go is Face f) yield return f;
                else if (go is Shell s) foreach (Face sf in s.Faces) yield return sf;
                else if (go is Solid so) foreach (Shell sh in so.Shells) foreach (Face sf in sh.Faces) yield return sf;
                else if (go is Block b) foreach (Face bf in Faces(b.Children)) yield return bf;
            }
        }

        /// <summary>
        /// A spherical octant (the corner of a rounded box, tilted by 35 degrees) bounded by three quarter circles.
        /// The orientation of the sphere, which keeps the poles away from the outline, put the seam exactly through
        /// one corner. The face was then split at the seam, the circles were cut 6.5e-4 away from the corner and a
        /// 0.0013 long seam edge without a neighbour remained, which made CombineConnectedFaces fail for the whole shell.
        /// The seam must be rotated away from the vertices, as it is done for cylinders, cones and tori.
        /// </summary>
        [TestMethod]
        public void a_sphere_seam_through_a_corner_does_not_split_the_face()
        {
            ImportStep import = new ImportStep();
            GeoObjectList imported = import.Read(StepFile("sphere_octant_seam_vertex.stp"));
            Face face = Faces(imported).Single();
            Assert.IsInstanceOfType(face.Surface, typeof(SphericalSurface));
            Assert.AreEqual(3, face.AllEdges.Length, "the three quarter circles, no additional seam edge");
            Assert.IsTrue(face.CheckConsistency(), "consistent face");
            foreach (Edge edge in face.AllEdges)
            {
                Assert.IsNotNull(edge.Curve2D(face), "2d curve of each edge");
                Assert.IsTrue(edge.Curve3D.Length > 1.0, "the circles are not cut at the seam");
            }
        }

        /// <summary>
        /// A sphere made of two hemispheres, which are bounded by a great circle through the poles of the spherical surface
        /// (two half circles between the poles), like a sphere from Rhino. The import rotates the sphere to keep the poles
        /// away from the outline, so each hemisphere has a single open loop which runs once around the period and must be
        /// closed by one of the poles. Two errors made this fail:
        /// the pole was chosen by the area from the pole point, which has a meaningless sign when the loop is not parallel
        /// to the pole line, so one hemisphere was not imported; and the rotated sphere was always oriented inward, because
        /// its axes were left-handed, which is not repaired for faces with an open loop.
        /// </summary>
        [TestMethod]
        public void a_sphere_from_two_hemispheres_is_closed_and_oriented_outward()
        {
            ImportStep import = new ImportStep();
            GeoObjectList imported = import.Read(StepFile("sphere_two_hemispheres.stp"));
            Assert.AreEqual(0, import.Problems.Count, "import problems: " + string.Join("; ", import.Problems.Select(p => $"#{p.Key} {p.Value}")));
            Solid solid = imported.OfType<Solid>().Single();
            Assert.AreEqual(0, solid.Shells[0].OpenEdgesExceptPoles.Length, "closed");
            Assert.AreEqual(4.0 / 3.0 * Math.PI * 1000.0, solid.Volume(1e-3), 4.0, "the volume of the sphere, positive for outward normals");
            foreach (Face face in solid.Shells[0].Faces)
            {
                Assert.IsInstanceOfType(face.Surface, typeof(SphericalSurface));
                GeoPoint2D uv = face.Area.GetExtent().GetCenter();
                GeoPoint p = face.Surface.PointAt(uv);
                Assert.AreEqual(1.0, face.Surface.GetNormal(uv).Normalized * (p - GeoPoint.Origin).Normalized, 1e-6, "the normal points outward");
            }
        }
    }
}
