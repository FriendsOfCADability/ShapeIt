using CADability.GeoObject;
using System.Collections.Generic;
using System.Linq;

namespace CADability.Tests
{
    /// <summary>
    /// The feature names of faces (<see cref="Shell.AddFeature"/>): they are saved with the solid, and a copy of a face
    /// has a set of names of its own.
    /// </summary>
    [TestClass]
    public class FeatureNamesTests
    {
        private static Solid Box() => Make3D.MakeBox(GeoPoint.Origin, GeoVector.XAxis, GeoVector.YAxis, GeoVector.ZAxis);

        [TestMethod]
        public void FeatureNamesAreSavedWithTheSolid()
        {
            Solid box = Box();
            box.Shells[0].AddFeature("pocket", box.Shells[0].Faces.Take(2));
            Solid read = (Solid)JsonSerialize.FromString(JsonSerialize.ToString(box));
            Dictionary<string, HashSet<Face>> features = read.Shells[0].GetFeatures();
            Assert.IsTrue(features.ContainsKey("pocket"), "the feature name must survive saving and loading");
            Assert.AreEqual(2, features["pocket"].Count);
        }

        [TestMethod]
        public void ACopyHasItsOwnFeatureNames()
        {
            Solid box = Box();
            box.Shells[0].AddFeature("pocket", box.Shells[0].Faces);
            Solid copy = (Solid)box.Clone();
            copy.Shells[0].AddFeature("boss", copy.Shells[0].Faces);
            Assert.IsFalse(box.Shells[0].GetFeatures().ContainsKey("boss"), "naming a feature of the copy must not change the original");
            Assert.IsTrue(copy.Shells[0].GetFeatures().ContainsKey("pocket"), "the copy keeps the names it had");
        }
    }
}
