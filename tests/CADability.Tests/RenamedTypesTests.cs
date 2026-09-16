using CADability;
using CADability.GeoObject;
using System;

namespace CADability.Tests
{
    /// <summary>
    /// A project file records the type of every object it holds by name, so a class that is serialized cannot
    /// simply be renamed - every file written before the rename would stop being readable. <see
    /// cref="RenamedTypes"/> is what buys that freedom back, and this is the proof that it does: an object is
    /// written, its type name in the text is put back to what it used to be - which is exactly what an old
    /// file contains - and it has to come back as the class of today.
    /// </summary>
    [TestClass]
    public class RenamedTypesTests
    {
        private const string OldName = "CADability.GeoObject.SweptCircle";
        private const string NewName = "CADability.GeoObject.SweptCircleSurface";

        /// <summary>
        /// A pipe around a helix. The spine has to be neither a line nor a circle, otherwise MakePipeSurface
        /// hands back a CylindricalSurface or a ToroidalSurface and the class under test never appears.
        /// </summary>
        private static SweptCircleSurface PipeAroundAHelix()
        {
            HelicalCurve helix = HelicalCurve.Construct();
            helix.SetHelix(Plane.XYPlane, 10.0, 8.0, 0.0, 4.0 * Math.PI);
            ISurface surface = SweptCircleSurface.MakePipeSurface(helix, 2.0, GeoVector.ZAxis);
            Assert.IsInstanceOfType(surface, typeof(SweptCircleSurface),
                "the test needs a real SweptCircleSurface, not a quadric MakePipeSurface simplified it into");
            return (SweptCircleSurface)surface;
        }

        [TestMethod]
        public void a_class_is_read_back_from_a_file_that_still_has_its_old_name()
        {
            SweptCircleSurface surface = PipeAroundAHelix();
            string written = JsonSerialize.ToString(surface);
            Assert.IsTrue(written.Contains(NewName),
                $"the written text should name the class as '{NewName}', so that putting the old name back "
                + "below really produces what an older file looked like");

            // This is the whole point: the same object as a file written before the rename.
            string asAnOlderFile = written.Replace(NewName, OldName);
            Assert.IsFalse(asAnOlderFile.Contains(NewName), "the substitution has to be complete");

            object read = JsonSerialize.FromString(asAnOlderFile);
            Assert.IsInstanceOfType(read, typeof(SweptCircleSurface),
                $"'{OldName}' has to be resolved to the class it was renamed into. Getting a JsonProxyType "
                + "here means the entry in RenamedTypes is missing, or the name is spelled differently - the "
                + "lookup is case sensitive.");

            // and it is not just the type that survived, the geometry did too
            SweptCircleSurface restored = (SweptCircleSurface)read;
            Assert.AreEqual(surface.Radius, restored.Radius, 1e-12);
            for (int i = 0; i <= 8; i++)
            {
                GeoPoint2D uv = new GeoPoint2D(i / 8.0, i / 8.0 * 2.0 * Math.PI);
                Assert.IsTrue((surface.PointAt(uv) | restored.PointAt(uv)) < 1e-9,
                    $"the surface differs at {uv} after the round trip");
            }
        }

        [TestMethod]
        public void the_current_name_is_of_course_read_as_well()
        {
            // The table must not get in the way of a file written today.
            SweptCircleSurface surface = PipeAroundAHelix();
            object read = JsonSerialize.FromString(JsonSerialize.ToString(surface));
            Assert.IsInstanceOfType(read, typeof(SweptCircleSurface));
        }

        [TestMethod]
        public void resolve_maps_a_renamed_name_and_leaves_everything_else_alone()
        {
            Assert.AreEqual(NewName, RenamedTypes.Resolve(OldName));
            Assert.AreEqual(NewName, RenamedTypes.Resolve(NewName), "resolving twice must not change anything");
            Assert.AreEqual("CADability.GeoObject.Ellipse", RenamedTypes.Resolve("CADability.GeoObject.Ellipse"));
            Assert.AreEqual("", RenamedTypes.Resolve(""));
            Assert.IsNull(RenamedTypes.Resolve(null));
        }

        [TestMethod]
        public void resolve_carries_an_array_suffix_over()
        {
            // One entry has to cover arrays of the class as well, those appear in files as their own $Type.
            Assert.AreEqual(NewName + "[]", RenamedTypes.Resolve(OldName + "[]"));
            Assert.AreEqual(NewName + "[][]", RenamedTypes.Resolve(OldName + "[][]"));
            Assert.AreEqual(NewName + "[,]", RenamedTypes.Resolve(OldName + "[,]"));
        }

        [TestMethod]
        public void a_name_may_not_be_mapped_onto_two_different_classes()
        {
            // A chain of renames has to be entered by CHANGING the existing entry, not by adding a second one,
            // because the lookup is a single step. Adding a contradicting entry is therefore refused.
            Assert.ThrowsException<ArgumentException>(() => RenamedTypes.Add(OldName, "CADability.GeoObject.Ellipse"));
            // ...while repeating what is already there is harmless
            RenamedTypes.Add(OldName, NewName);
            Assert.AreEqual(NewName, RenamedTypes.Resolve(OldName));
        }
    }
}
