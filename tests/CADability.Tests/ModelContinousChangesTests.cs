using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// Guards the octree bookkeeping while an object is being dragged (a "continuous change").
    /// <para>
    /// <see cref="Model"/> takes an object out of the octree on its FIRST change within a continuous
    /// change and collects it; when the change ends, everything collected is put back in at the new
    /// position. Deciding "is this the first change?" used to rely on Wintellect.PowerCollections'
    /// <c>Set&lt;T&gt;.Add</c>, which returned true when the item was ALREADY present - the inverse of
    /// <see cref="HashSet{T}.Add"/>. Swapping the set type without flipping that condition delays the
    /// removal by one step, so during the drag the object stays indexed at the position it already
    /// left and is pickable there.
    /// </para>
    /// <para>
    /// The defect is only observable DURING the drag: once the continuous change ends, both variants
    /// converge, because removal is not extent-based. So these tests probe inside the context frame.
    /// </para>
    /// </summary>
    [TestClass]
    public class ModelContinousChangesTests
    {
        /// <summary>
        /// Builds a model whose octree spans a large cube, so that moving an object around inside it
        /// never extends the tree - extending rebuilds nodes and would mask a stale entry.
        /// </summary>
        private static Model ModelWithWideOctree()
        {
            Model model = Project.CreateSimpleProject().GetActiveModel();
            foreach (GeoPoint corner in new[] { new GeoPoint(-500, -500, -500), new GeoPoint(500, 500, 500) })
            {
                Line anchor = Line.Construct();
                anchor.SetTwoPoints(corner, corner + new GeoVector(1, 1, 1));
                model.Add(anchor);
            }
            // Model.OctTree returns the raw field, it does not build the tree. GetObjectsFromBox does.
            model.GetObjectsFromBox(new BoundingBox(new GeoPoint(0, 0, 0), 1.0));
            return model;
        }

        private static bool IndexedAt(Model model, GeoPoint p, IGeoObject go)
        {
            return System.Array.IndexOf(model.OctTree.GetObjectsFromPoint(p), go) >= 0;
        }

        [TestMethod]
        public void DuringContinousChange_ObjectIsNotLeftIndexedAtThePositionItAlreadyLeft()
        {
            Model model = ModelWithWideOctree();

            Line line = Line.Construct();
            line.SetTwoPoints(new GeoPoint(0, 0, 0), new GeoPoint(10, 0, 0));
            model.Add(line);
            Assert.IsTrue(IndexedAt(model, new GeoPoint(5, 0, 0), line),
                "precondition: the line must be indexed at its original position");

            using (model.Undo.ContextFrame(this))
            {
                line.SetTwoPoints(new GeoPoint(0, 100, 0), new GeoPoint(10, 100, 0));

                // The line has moved away from y = 0, so it must no longer be indexed there.
                Assert.IsFalse(IndexedAt(model, new GeoPoint(5, 0, 0), line),
                    "stale octree entry: the line is still indexed at the position it was dragged away from");
            }
            model.Undo.ClearContext();
        }

        [TestMethod]
        public void AfterContinousChange_ObjectIsIndexedAtItsFinalPositionOnly()
        {
            Model model = ModelWithWideOctree();

            Line line = Line.Construct();
            line.SetTwoPoints(new GeoPoint(0, 0, 0), new GeoPoint(10, 0, 0));
            model.Add(line);
            model.OctTree.GetObjectsFromPoint(new GeoPoint(5, 0, 0)); // make sure the octree exists

            using (model.Undo.ContextFrame(this))
            {
                line.SetTwoPoints(new GeoPoint(0, 100, 0), new GeoPoint(10, 100, 0));
                line.SetTwoPoints(new GeoPoint(0, 200, 0), new GeoPoint(10, 200, 0));
            }
            model.Undo.ClearContext(); // re-inserts everything collected during the change

            Assert.IsTrue(IndexedAt(model, new GeoPoint(5, 200, 0), line),
                "the line must be indexed at its final position");
            Assert.IsFalse(IndexedAt(model, new GeoPoint(5, 0, 0), line),
                "the line must not be left behind at its original position");
        }
    }
}
