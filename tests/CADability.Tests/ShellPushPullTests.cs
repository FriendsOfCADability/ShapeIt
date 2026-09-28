using System;
using System.Collections.Generic;
using System.Linq;
using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="ShellExtensions.PushPull"/> at tangential edges. The body is a prism 40 x 30 x 20 whose vertical
    /// edge at x = 40, y = 30 is rounded with the radius 5: the cylinder is tangential to the side faces x = 40 and y = 30.
    /// </summary>
    [TestClass]
    public class ShellPushPullTests
    {
        private const double r = 5.0;
        private const double d = 3.0;

        private static Shell RoundedPrism()
        {
            Solid box = Make3D.MakeBox(GeoPoint.Origin, 40 * GeoVector.XAxis, 30 * GeoVector.YAxis, 20 * GeoVector.ZAxis);
            Edge vertical = box.Shells[0].Edges.Single(e => e.Curve3D is Line line && Math.Abs(line.StartDirection.z) > 0.5 && (line.StartPoint | new GeoPoint(40, 30, line.StartPoint.z)) < 1e-6);
            Shell shell = box.Shells[0].RoundEdges(new[] { vertical }, r);
            Assert.IsNotNull(shell, "the edge could not be rounded");
            Assert.AreEqual(7, shell.Faces.Length);
            Assert.AreEqual(1200 - r * r + Math.PI * r * r / 4, shell.Volume(0.01) / 20, 1e-3, "wrong volume of the prism");
            return shell;
        }

        private static IEnumerable<Face> FacesWhere(Shell shell, Func<Face, bool> predicate) => shell.Faces.Where(predicate).ToArray();

        private static bool IsTop(Face face) => face.Surface is PlaneSurface && face.GetExtent(0.0).Zmin > 19.0;

        /// <summary>the cylinder and the two side faces tangential to it</summary>
        private static bool IsRoundedSide(Face face) => face.Surface is CylindricalSurface
            || face.Surface is PlaneSurface && (face.GetExtent(0.0).Xmin > 39.0 || face.GetExtent(0.0).Ymin > 29.0);

        private static void AssertValid(Shell shell)
        {
            Assert.AreEqual(0, shell.OpenEdgesExceptPoles.Length, "the shell has open edges");
            Assert.IsTrue(shell.CheckConsistency(), "the shell is not consistent");
            Assert.AreEqual(7, shell.Faces.Length, "the topology has changed");
        }

        /// <summary>
        /// Pulling the top face: at the ends of the arc the moved vertex slides along the tangential edges between the
        /// cylinder and the side faces, which are not moved (the third edge at the vertex is tangential).
        /// </summary>
        [TestMethod]
        public void pulling_a_face_slides_along_an_unmoved_tangential_edge()
        {
            Shell shell = RoundedPrism();
            Assert.IsTrue(shell.PushPull(FacesWhere(shell, IsTop), d));
            AssertValid(shell);
            double expected = (1200 - r * r + Math.PI * r * r / 4) * (20 + d);
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-6);
        }

        /// <summary>
        /// Pulling the cylinder together with both tangential side faces: the tangential edges between them are offset
        /// and the radius grows to r + d around the same axis.
        /// </summary>
        [TestMethod]
        public void pulling_a_tangential_chain_offsets_the_tangential_edges()
        {
            Shell shell = RoundedPrism();
            Assert.IsTrue(shell.PushPull(FacesWhere(shell, IsRoundedSide), d));
            AssertValid(shell);
            double R = r + d;
            double expected = ((40 + d) * (30 + d) - R * R + Math.PI * R * R / 4) * 20;
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-6);
        }

        /// <summary>
        /// Both at once: the offset tangential edges between the moved side faces and the cylinder end in the moved top face.
        /// </summary>
        [TestMethod]
        public void pulling_a_tangential_chain_and_the_top_face()
        {
            Shell shell = RoundedPrism();
            Assert.IsTrue(shell.PushPull(FacesWhere(shell, f => IsTop(f) || IsRoundedSide(f)), d));
            AssertValid(shell);
            double R = r + d;
            double expected = ((40 + d) * (30 + d) - R * R + Math.PI * R * R / 4) * (20 + d);
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-6);
        }

        /// <summary>
        /// Pushing the chain inwards: the radius shrinks to r - d.
        /// </summary>
        [TestMethod]
        public void pushing_a_tangential_chain_shrinks_the_fillet()
        {
            Shell shell = RoundedPrism();
            Assert.IsTrue(shell.PushPull(FacesWhere(shell, IsRoundedSide), -d));
            AssertValid(shell);
            double R = r - d;
            double expected = ((40 - d) * (30 - d) - R * R + Math.PI * R * R / 4) * 20;
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-6);
        }

        private static double V0 => (1200 - r * r + Math.PI * r * r / 4) * 20;

        private static bool IsSide(Face face, bool atX) => face.Surface is PlaneSurface
            && (atX ? face.GetExtent(0.0).Xmin > 39.0 : face.GetExtent(0.0).Ymin > 29.0);

        private static void AssertClosedAndConsistent(Shell shell, int faces)
        {
            Assert.AreEqual(0, shell.OpenEdgesExceptPoles.Length, "the shell has open edges");
            Assert.IsTrue(shell.CheckConsistency(), "the shell is not consistent");
            Assert.AreEqual(faces, shell.Faces.Length);
        }

        /// <summary>
        /// Pulling the side face x = 40 while the fillet stays: the fillet keeps its tangential edge, a planar strip at y = 25
        /// connects it with the moved face.
        /// </summary>
        [TestMethod]
        public void pulling_a_face_next_to_a_fillet_inserts_a_strip()
        {
            Shell shell = RoundedPrism();
            Assert.IsTrue(shell.PushPull(FacesWhere(shell, f => IsSide(f, true)), d));
            AssertClosedAndConsistent(shell, 8);
            double expected = V0 + d * 25 * 20;
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-6);
        }

        [TestMethod]
        public void pushing_a_face_next_to_a_fillet_inserts_a_strip()
        {
            Shell shell = RoundedPrism();
            Assert.IsTrue(shell.PushPull(FacesWhere(shell, f => IsSide(f, true)), -d));
            AssertClosedAndConsistent(shell, 8);
            double expected = V0 - d * 25 * 20;
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-6);
        }

        /// <summary>both faces tangential to the fillet are pulled, the fillet stays: one strip on each side</summary>
        [TestMethod]
        public void pulling_both_faces_next_to_a_fillet_inserts_two_strips()
        {
            Shell shell = RoundedPrism();
            Assert.IsTrue(shell.PushPull(FacesWhere(shell, f => IsSide(f, true) || IsSide(f, false)), d));
            AssertClosedAndConsistent(shell, 9);
            double expected = V0 + d * 25 * 20 + d * 35 * 20;
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-6);
        }

        /// <summary>the lateral face of a cylinder is split into two halves with the same surface: pulling one pulls both</summary>
        [TestMethod]
        public void pulling_half_of_a_cylinder_pulls_the_other_half()
        {
            Solid cylinder = Make3D.MakeCylinder(GeoPoint.Origin, 10 * GeoVector.XAxis, 20 * GeoVector.ZAxis);
            Shell shell = cylinder.Shells[0];
            Face[] lateral = [.. shell.Faces.Where(f => f.Surface is CylindricalSurface)];
            Assert.AreEqual(2, lateral.Length, "the cylinder is expected to be split");
            Assert.IsTrue(shell.PushPull(new[] { lateral[0] }, 2.0));
            AssertClosedAndConsistent(shell, 4);
            double expected = Math.PI * 12 * 12 * 20;
            Assert.AreEqual(expected, shell.Volume(0.01), expected * 1e-5);
        }

        /// <summary>
        /// A box 40 x 30 x 20 whose top edge at y = 30 is rounded with the radius 5, hollowed with the wall thickness 2 and
        /// the top face open. The top face is pulled out by 2 with a strip at y = 25, then the shell is offset inwards. The
        /// cross section of the cavity is [2, 28] x [2, 20] without the corner above the inner fillet (radius 3 around
        /// (25, 15)) and the rounding of the inner edge at the strip (radius 2 around (25, 20)): 453 + 5*pi/4, 36 long.
        /// </summary>
        [TestMethod]
        public void make_hollow_with_a_fillet_at_the_open_face()
        {
            Solid box = Make3D.MakeBox(GeoPoint.Origin, 40 * GeoVector.XAxis, 30 * GeoVector.YAxis, 20 * GeoVector.ZAxis);
            Edge top = box.Shells[0].Edges.Single(e => e.Curve3D is Line line && Math.Abs(line.StartDirection.x) > 0.5
                && Math.Abs(line.StartPoint.y - 30) < 1e-6 && Math.Abs(line.StartPoint.z - 20) < 1e-6);
            Shell shell = box.Shells[0].RoundEdges(new[] { top }, r);
            Assert.IsNotNull(shell);
            Face[] open = [.. shell.Faces.Where(f => f.Surface is PlaneSurface && f.GetExtent(0.0).Zmin > 19.0)];
            Assert.AreEqual(1, open.Length);
            Shell[] hollow = shell.MakeHollow(open, 2.0);
            Assert.AreEqual(1, hollow.Length);
            AssertClosedAndConsistent(hollow[0], hollow[0].Faces.Length);
            double expected = 40 * (575 + Math.PI * 25 / 4) - 36 * (453 + 5 * Math.PI / 4);
            Assert.AreEqual(expected, hollow[0].Volume(0.01), expected * 1e-5);
        }

        /// <summary>
        /// CADability allows neither closed edges (an edge from a vertex back to the same vertex) nor faces with a seam (an edge
        /// with the same face on both sides): periodic faces are split, e.g. into two halves of a cylinder. So the strip in
        /// PushPull never has to deal with a closed tangential edge.
        /// </summary>
        private static void AssertNoClosedEdgesAndNoSeams(Shell shell, string name)
        {
            foreach (Edge edge in shell.Edges)
            {
                Assert.AreNotSame(edge.Vertex1, edge.Vertex2, $"{name}: closed edge");
                Assert.AreNotSame(edge.PrimaryFace, edge.SecondaryFace, $"{name}: face with a seam");
            }
        }

        [TestMethod]
        public void primitives_have_no_closed_edges_and_no_seams()
        {
            AssertNoClosedEdgesAndNoSeams(Make3D.MakeCylinder(GeoPoint.Origin, 10 * GeoVector.XAxis, 20 * GeoVector.ZAxis).Shells[0], "cylinder");
            AssertNoClosedEdgesAndNoSeams(Make3D.MakeCone(GeoPoint.Origin, GeoVector.XAxis, 20 * GeoVector.ZAxis, 10, 5).Shells[0], "cone");
            AssertNoClosedEdgesAndNoSeams(Make3D.MakeSphere(GeoPoint.Origin, 10).Shells[0], "sphere");
            AssertNoClosedEdgesAndNoSeams(Make3D.MakeTorus(GeoPoint.Origin, GeoVector.ZAxis, 20, 5).Shells[0], "torus");
        }

        /// <summary>
        /// A cylinder (radius 10, height 20) whose top rim is rounded with the radius 2: the top face is a disk of radius 8,
        /// tangential to the torus fillet along two arcs. Pulling the top face by 3 inserts two strips, halves of a cylinder of
        /// radius 8, which share their straight edges. The fillet removes the corner area 4 - pi around the axis; by Pappus
        /// this is 2*pi*(100/3 - 8*pi).
        /// </summary>
        [TestMethod]
        public void pulling_the_top_of_a_cylinder_with_a_rounded_rim()
        {
            Shell cylinder = Make3D.MakeCylinder(GeoPoint.Origin, 10 * GeoVector.XAxis, 20 * GeoVector.ZAxis).Shells[0];
            Edge[] rim = [.. cylinder.Edges.Where(e => e.Curve3D is Ellipse && e.Curve3D.StartPoint.z > 19.0)];
            Assert.AreEqual(2, rim.Length, "the top rim is expected to consist of two arcs");
            Shell shell = cylinder.RoundEdges(rim, 2.0);
            Assert.IsNotNull(shell, "the rim could not be rounded");
            AssertNoClosedEdgesAndNoSeams(shell, "rounded cylinder");
            double v0 = 2000 * Math.PI - 2 * Math.PI * (100.0 / 3.0 - 8 * Math.PI);
            Assert.AreEqual(v0, shell.Volume(0.01), v0 * 1e-5, "wrong volume of the rounded cylinder");
            Face[] top = [.. shell.Faces.Where(f => f.Surface is PlaneSurface && f.GetExtent(0.0).Zmin > 19.0)];
            Assert.AreEqual(1, top.Length);
            Assert.IsTrue(top[0].AllEdges.All(e => e.IsTangentialEdge()), "the top face is expected to be tangential to the fillet");
            int faces = shell.Faces.Length;

            Assert.IsTrue(shell.PushPull(top, d));
            AssertClosedAndConsistent(shell, faces + top[0].AllEdges.Length);
            AssertNoClosedEdgesAndNoSeams(shell, "pulled cylinder");
            Assert.AreEqual(2, shell.Faces.Count(f => f.Surface is CylindricalSurface cs && Math.Abs(cs.RadiusX - 8) < 1e-6), "two strips of radius 8");
            double expected = v0 + 64 * Math.PI * d;
            // Volume(0.01) is off by 1.4e-4 relative here, the geometry is exact: Volume(1e-4) agrees to 1e-9
            Assert.AreEqual(expected, shell.Volume(1e-4), expected * 1e-6);
        }
    }
}
