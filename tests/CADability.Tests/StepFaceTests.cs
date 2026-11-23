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
    }
}
