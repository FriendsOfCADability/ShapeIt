using CADability;
using CADability.GeoObject;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Linq;
using System.Runtime.CompilerServices;
using Path = System.IO.Path;

namespace CADability.Tests
{
    [TestClass]
    public class NurbsReconstructionTests
    {
        private static string StlFile(string name, [CallerFilePath] string thisFile = "")
            => Path.Combine(Path.GetDirectoryName(thisFile), "Files", "STL", name);

        [TestMethod]
        public void NurbsTest1_UnrecognizedBecomesNurbs()
        {
            ImportSTL import = new ImportSTL();
            import.BuildRawFaces = false;
            import.Read(StlFile("NurbsTest1.stl"));
            StlSurfaceReconstruction rec = import.Reconstruction;
            RecognizedRegion nurbsRegion = rec.Regions.FirstOrDefault(r => r.Kind == RecognizedSurfaceKind.Nurbs);
            Assert.IsNotNull(nurbsRegion, "expected the bumpy region to be recognized as a NURBS surface");
            Assert.IsInstanceOfType(nurbsRegion.Surface, typeof(NurbsSurface));
            Assert.IsTrue(nurbsRegion.MaxError <= rec.NurbsTolerance,
                $"NURBS fit error {nurbsRegion.MaxError} exceeds tolerance {rec.NurbsTolerance}");
            // the other region is the flat plane, so the mesh is fully recognized
            Assert.AreEqual(0, rec.Regions.Count(r => r.Kind == RecognizedSurfaceKind.Unrecognized),
                "no region should remain unrecognized");
        }
    }
}
