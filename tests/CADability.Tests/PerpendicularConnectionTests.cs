using System.IO;
using System.Runtime.CompilerServices;
using CADability;
using CADability.GeoObject;

namespace CADability.Tests
{
    [TestClass]
    public class PerpendicularConnectionTests
    {
        static string TestDataDir([CallerFilePath] string thisFile = null)
            => System.IO.Path.GetFullPath(System.IO.Path.Combine(System.IO.Path.GetDirectoryName(thisFile), "..", "..", "CADability.Tests", "TestData"));

        static Face LoadFace(string name)
        {
            string path = System.IO.Path.Combine(TestDataDir(), name);
            using Stream stream = File.Open(path, FileMode.Open);
            JsonSerialize js = new JsonSerialize();
            return js.FromStream(stream) as Face;
        }

        /// <summary>
        /// Face1 is a NURBS surface (domain [0,1]x[0,1]), Face2 a plane (domain spanning tens of units). Starting from
        /// the domain centers, <see cref="Surfaces.PerpendicularConnection"/> must find a point pair whose connection is
        /// perpendicular to both surfaces.
        /// </summary>
        [TestMethod]
        public void PerpendicularConnection_PlaneNurbs_finds_pair()
        {
            Face f1 = LoadFace("Face1.json");
            Face f2 = LoadFace("Face2.json");
            Assert.IsNotNull(f1);
            Assert.IsNotNull(f2);

            ISurface s1 = f1.Surface;
            ISurface s2 = f2.Surface;
            BoundingRect b1 = f1.Domain;
            BoundingRect b2 = f2.Domain;
            GeoPoint2D uv1 = b1.GetCenter();
            GeoPoint2D uv2 = b2.GetCenter();

            bool ok = Surfaces.PerpendicularConnection(s1, b1, s2, b2, ref uv1, ref uv2);
            Assert.IsTrue(ok, "no perpendicular connection found");

            // The result must lie inside both domains.
            Assert.IsTrue(b1.ContainsEps(uv1, b1.Size * 1e-6), "uv1 outside domain");
            Assert.IsTrue(b2.ContainsEps(uv2, b2.Size * 1e-6), "uv2 outside domain");

            // The connection must be parallel to both surface normals (i.e. perpendicular to both surfaces).
            s1.DerivativeAt(uv1, out GeoPoint p1, out GeoVector s1u, out GeoVector s1v);
            s2.DerivativeAt(uv2, out GeoPoint p2, out GeoVector s2u, out GeoVector s2v);
            GeoVector dir = (p1 - p2).Normalized;
            Assert.AreEqual(0.0, (dir ^ (s1u ^ s1v).Normalized).Length, 1e-6, "not perpendicular to surface 1");
            Assert.AreEqual(0.0, (dir ^ (s2u ^ s2v).Normalized).Length, 1e-6, "not perpendicular to surface 2");
        }
    }
}
