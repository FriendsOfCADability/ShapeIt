using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using CADability.GeoObject;
using ShapeIt;
using Path = System.IO.Path;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="ShellExtensions.GetOffset"/> and its parts: the offset of a shell is built from the
    /// offset of every face, a fillet along every convex edge and a spherical patch at the vertices. The interesting
    /// case is an offset which is bigger than the smallest curvature radius of an edge: the pipe which forms the
    /// fillet folds over itself there and the hidden part has to be cut away.
    /// </summary>
    [TestClass]
    public class ShellOffsetTests
    {
        public TestContext TestContext { get; set; }

        /// <summary>
        /// A box with a slanted bore: the edges where the bore meets the two faces are ellipses with the
        /// semi axes 9.725 and 5.875, so their smallest curvature radius is b*b/a == 3.549.
        /// </summary>
        private static Shell BoreShell([CallerFilePath] string thisFile = "")
        {
            string file = Path.Combine(Path.GetDirectoryName(thisFile)!, "Files", "BRep", "OffsetTest1.cdb.json");
            BRepCase testCase = BRepCaseReader.Read(file, BRepOperationKind.OffsetShell, 1.0);
            Assert.AreEqual(0, testCase.Problems.Count, string.Join("; ", testCase.Problems));
            return testCase.Operands[0];
        }

        private static void AssertIsClosedAndConsistent(Shell shell)
        {
            Assert.AreEqual(0, shell.OpenEdgesExceptPoles.Length, "the offset has open edges");
            Assert.IsTrue(shell.CheckConsistency(), "the offset is not a consistent shell");
        }

        [TestMethod]
        public void offset_of_a_box_has_the_exact_volume()
        {
            // the offset of a box by r is the box, grown by r on all sides: its volume is the volume of the box plus
            // the surface times r plus a quarter cylinder along every edge plus one sphere distributed over the corners
            Solid box = Make3D.MakeBox(GeoPoint.Origin, 10 * GeoVector.XAxis, 8 * GeoVector.YAxis, 6 * GeoVector.ZAxis);
            const double r = 1.0;
            Shell[] offset = ShellExtensions.GetOffset(box.Shells[0], r);
            Assert.AreEqual(1, offset.Length);
            AssertIsClosedAndConsistent(offset[0]);
            double expected = 10 * 8 * 6 + 2 * (10 * 8 + 10 * 6 + 8 * 6) * r
                + 4 * (10 + 8 + 6) * Math.PI * r * r / 4.0 + 4.0 / 3.0 * Math.PI * r * r * r;
            Assert.AreEqual(expected, offset[0].Volume(0.01), expected * 1e-4, "wrong volume of the offset");
        }

        [TestMethod]
        public void offset_smaller_than_the_curvature_radius_does_not_fold()
        {
            Shell[] offset = ShellExtensions.GetOffset(BoreShell(), 2.0); // 2 < 3.549, the fillet does not fold
            Assert.AreEqual(1, offset.Length);
            AssertIsClosedAndConsistent(offset[0]);
            AssertNoFoldInsideAFace(offset[0]);
        }

        /// <summary>
        /// With an offset bigger than the smallest curvature radius of the elliptical edge the fillet around it
        /// folds over itself. The two edges of that ellipse must be treated as one chain, otherwise the double curve
        /// of the fold would run from the one fillet into the other and could not be resolved at all.
        /// </summary>
        [TestMethod]
        public void offset_bigger_than_the_curvature_radius_folds_the_fillet()
        {
            Face[] parts = ShellExtensions.GetOffsetParts(BoreShell(), 5.0, out bool allEdgesAreConnected);
            Assert.IsTrue(allEdgesAreConnected, "every edge of this shell should get a fillet");
            int openEdges = parts.Sum(f => f.AllEdges.Count(e => e.SecondaryFace == null));
            Assert.AreEqual(0, openEdges, "the parts of the offset do not fit together");
            // both bores have a fold at the sharp end of their elliptical edge, so both fillets are split there
            Face[] fillets = parts.Where(f => f.Surface is SweptCircle).ToArray();
            Assert.AreEqual(4, fillets.Length, "each of the two fillets should be split into two faces");

            Shell[] offset = ShellExtensions.GetOffset(BoreShell(), 5.0);
            Assert.AreEqual(1, offset.Length);
            AssertIsClosedAndConsistent(offset[0]);
            AssertNoFoldInsideAFace(offset[0]);
        }

        /// <summary>
        /// Inside the fold the normal of the pipe is flipped and the surface lies closer to the edge than the
        /// offset: no face of the result may cover such a point.
        /// </summary>
        private static void AssertNoFoldInsideAFace(Shell shell)
        {
            int tested = 0;
            foreach (Face face in shell.Faces)
            {
                if (!(face.Surface is SweptCircle sweptCircle)) continue;
                BoundingRect ext = face.Area.GetExtent();
                for (int i = 1; i < 20; i++)
                {
                    for (int j = 1; j < 20; j++)
                    {
                        GeoPoint2D uv = new GeoPoint2D(ext.Left + ext.Width * i / 20.0, ext.Bottom + ext.Height * j / 20.0);
                        if (!face.Area.Contains(uv, false)) continue;
                        ++tested;
                        Assert.IsTrue(sweptCircle.UDirection(uv) * sweptCircle.Spine.DirectionAt(uv.x - Math.Floor(uv.x)) > 0.0,
                            $"the fillet is folded over at {uv}, the hidden part has not been cut away");
                    }
                }
            }
            Assert.IsTrue(tested > 100, $"only {tested} points inside the fillets were tested");
        }
    }
}
