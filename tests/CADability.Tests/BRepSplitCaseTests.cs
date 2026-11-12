using System.Linq;
using CADability.Attribute;
using CADability.GeoObject;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// The "Split" convention of the BRep regression cases: a solid "Operand1", a shell "Operand2" and a text
    /// "Split" run <see cref="BooleanOperation.SplitSolidByShell"/>. Built in memory, so it does not depend on a
    /// project file.
    /// </summary>
    [TestClass]
    public class BRepSplitCaseTests
    {
        private static Model MakeSplitModel(bool normalUp)
        {
            Model model = new Model();
            Solid box = Make3D.MakeBox(GeoPoint.Origin, 10 * GeoVector.XAxis, 10 * GeoVector.YAxis, 10 * GeoVector.ZAxis);
            box.Style = new Style(BRepCaseReader.Operand1Style);
            GeoVector normal = normalUp ? GeoVector.ZAxis : -GeoVector.ZAxis;
            Plane plane = new Plane(new GeoPoint(0, 0, 5), normal);
            Face face = Face.MakeFace(new PlaneSurface(plane), new BoundingRect(-10, -10, 20, 20));
            Shell cutter = Shell.FromFaces(face);
            cutter.Style = new Style(BRepCaseReader.Operand2Style);
            Text text = Text.Construct();
            text.TextString = "Split";
            model.Add(box);
            model.Add(cutter);
            model.Add(text);
            return model;
        }

        [TestMethod]
        [DataRow(true)]
        [DataRow(false)]
        public void SplitTextSplitsSolidByShell(bool normalUp)
        {
            BRepCase testCase = BRepCaseReader.FromModel(MakeSplitModel(normalUp), "split");
            Assert.IsTrue(testCase.IsRunnable, string.Join("; ", testCase.Problems));
            Assert.AreEqual(BRepOperationKind.SplitByShell, testCase.Operation);
            Assert.AreEqual(2, testCase.Operands.Count);

            Shell[] result = BRepRunner.Execute(testCase);
            Assert.AreEqual(1, result.Length, "onlyInnerParts: one side of the cut");
            Assert.IsTrue(BRepRunner.AllShellsAreValid(result));
            BoundingBox ext = result[0].GetExtent(0.0);
            Assert.AreEqual(5.0, ext.Zmax - ext.Zmin, 1e-6);
            // the part on the side the normals of the shell point to is kept
            Assert.AreEqual(normalUp ? 10.0 : 5.0, ext.Zmax, 1e-6);
        }

        [TestMethod]
        public void SplitNeedsExactlyOneShell()
        {
            Model model = MakeSplitModel(true);
            Shell second = (Shell)model.AllObjects.OfType<Shell>().First().Clone();
            second.Style = new Style(BRepCaseReader.Operand2Style);
            model.Add(second);
            BRepCase testCase = BRepCaseReader.FromModel(model, "split2");
            Assert.IsFalse(testCase.IsRunnable);
        }
    }
}
