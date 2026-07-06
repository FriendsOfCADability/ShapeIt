using System;
using System.Collections.Generic;

namespace CADability
{
    /// <summary>
    /// Split strategy used when building a <see cref="BVHTree"/>.
    /// </summary>
    public enum BVHBuildStrategy
    {
        /// <summary>
        /// Binned surface area heuristic (SAH): places the split where the expected number of box
        /// tests per query is minimal, evaluated at 16 equally spaced candidate positions per axis.
        /// Typically gives noticeably faster queries than <see cref="MedianSplit"/> when the item
        /// boxes vary in size or are unevenly distributed, at a slightly higher build cost. The default.
        /// </summary>
        BinnedSAH,
        /// <summary>
        /// Median split along the longest centroid axis: perfectly balanced tree and the fastest
        /// build, but query performance is usually worse than with <see cref="BinnedSAH"/>.
        /// </summary>
        MedianSplit
    }

    /// <summary>
    /// Immutable bounding volume hierarchy over a list of axis aligned boxes (<see cref="BoundingBox"/>).
    /// The tree is built once from the boxes passed to the constructor and cannot be modified afterwards
    /// (in contrast to <see cref="OctTree{T}"/>, which supports dynamic insertion and removal).
    /// All query results are indices into the original list, so the caller can keep the actual objects
    /// (triangles, faces, uv-patches, ...) in whatever form is convenient.
    /// <para>
    /// All queries are conservative box-level tests: they return the items whose bounding boxes satisfy
    /// the condition. Exact geometric tests on the underlying objects are the caller's responsibility.
    /// </para>
    /// </summary>
    public class BVHTree
    {
        /// <summary>
        /// A node of the flattened tree. Inner nodes reference their two children (which are always
        /// adjacent in the node array), leaf nodes reference a range in <see cref="itemIndices"/>.
        /// </summary>
        private struct Node
        {
            public BoundingBox box; // union of all item boxes below this node
            public int leftChild;   // index of the left child, the right child is leftChild + 1; only valid when count == 0
            public int start;       // start of the range in itemIndices; only valid when count > 0
            public int count;       // number of items in this leaf, 0 for inner nodes
        }

        public const int DefaultLeafSize = 4;
        private const int SAHBinCount = 16; // bins per axis for the binned SAH build

        private readonly Node[] nodes; // flattened tree, nodes[0] is the root (empty when Count == 0)
        private readonly int[] itemIndices; // permutation of 0..Count-1, leaves reference ranges herein
        private readonly BoundingBox[] itemBoxes; // the item boxes in original order
        private int maxDepth; // depth of the deepest node, used to size traversal stacks

        /// <summary>
        /// Number of items in this tree.
        /// </summary>
        public int Count => itemBoxes.Length;

        /// <summary>
        /// The overall extent of all items, i.e. the box of the root node.
        /// </summary>
        public BoundingBox Extent => nodes.Length > 0 ? nodes[0].box : BoundingBox.EmptyBoundingBox;

        /// <summary>
        /// Gets the bounding box of the item with the provided index (as passed to the constructor).
        /// </summary>
        public BoundingBox this[int index] => itemBoxes[index];

        /// <summary>
        /// Builds the hierarchy from the provided boxes. The box at index i corresponds to index i
        /// in all query results. The list is copied, later changes to it do not affect the tree.
        /// </summary>
        /// <param name="boxes">One axis aligned box per item</param>
        /// <param name="leafSize">Maximum number of items per leaf node</param>
        /// <param name="strategy">How to choose the split position, see <see cref="BVHBuildStrategy"/></param>
        public BVHTree(IReadOnlyList<BoundingBox> boxes, int leafSize = DefaultLeafSize, BVHBuildStrategy strategy = BVHBuildStrategy.BinnedSAH)
        {
            if (boxes == null) throw new ArgumentNullException(nameof(boxes));
            if (leafSize < 1) leafSize = 1;
            int n = boxes.Count;
            itemBoxes = new BoundingBox[n];
            for (int i = 0; i < n; i++) itemBoxes[i] = boxes[i];
            itemIndices = new int[n];
            for (int i = 0; i < n; i++) itemIndices[i] = i;
            if (n == 0)
            {
                nodes = new Node[0];
                return;
            }

            // box centroids per axis, used to choose the split axis/position and to partition the items
            double[] cx = new double[n], cy = new double[n], cz = new double[n];
            for (int i = 0; i < n; i++)
            {
                cx[i] = 0.5 * (itemBoxes[i].Xmin + itemBoxes[i].Xmax);
                cy[i] = 0.5 * (itemBoxes[i].Ymin + itemBoxes[i].Ymax);
                cz[i] = 0.5 * (itemBoxes[i].Zmin + itemBoxes[i].Zmax);
            }
            double[][] centroid = { cx, cy, cz };
            double[] sortKeys = new double[n]; // scratch for the median split
            // scratch for the binned SAH build, reused for every node (the build is single threaded)
            BoundingBox[] binBoxes = new BoundingBox[3 * SAHBinCount];
            int[] binCounts = new int[3 * SAHBinCount];
            BoundingBox[] suffixBoxes = new BoundingBox[SAHBinCount];
            List<Node> nodeList = new List<Node>(Math.Max(1, 2 * n / leafSize));

            double SurfaceArea(ref BoundingBox b)
            {
                // half the surface, the constant factor cancels out in the cost comparison
                double dx = b.Xmax - b.Xmin, dy = b.Ymax - b.Ymin, dz = b.Zmax - b.Zmin;
                return dx * dy + dy * dz + dz * dx;
            }

            // Sorts the range by centroid on the provided axis and splits at the median.
            // Always makes progress (both halves non empty), used as strategy and as fallback.
            int PartitionMedian(int start, int count, int axis)
            {
                double[] c = centroid[axis];
                for (int i = 0; i < count; i++) sortKeys[start + i] = c[itemIndices[start + i]];
                Array.Sort(sortKeys, itemIndices, start, count);
                return count / 2;
            }

            // Binned SAH: distributes the items into equally spaced bins along each axis of the
            // centroid bounds, evaluates the SAH cost (child count times child box surface area)
            // at every bin boundary and partitions the range in place at the cheapest one.
            // Returns the size of the left part, or 0 when no usable split was found
            // (the caller then falls back to the median split).
            int PartitionBinnedSAH(int start, int count,
                double cxmin, double cxmax, double cymin, double cymax, double czmin, double czmax)
            {
                for (int i = 0; i < 3 * SAHBinCount; i++)
                {
                    binBoxes[i] = BoundingBox.EmptyBoundingBox;
                    binCounts[i] = 0;
                }
                double kx = cxmax > cxmin ? SAHBinCount / (cxmax - cxmin) : 0.0;
                double ky = cymax > cymin ? SAHBinCount / (cymax - cymin) : 0.0;
                double kz = czmax > czmin ? SAHBinCount / (czmax - czmin) : 0.0;
                for (int i = 0; i < count; i++)
                {
                    int idx = itemIndices[start + i];
                    int bx = Math.Min((int)((cx[idx] - cxmin) * kx), SAHBinCount - 1);
                    int by = Math.Min((int)((cy[idx] - cymin) * ky), SAHBinCount - 1);
                    int bz = Math.Min((int)((cz[idx] - czmin) * kz), SAHBinCount - 1);
                    binBoxes[bx].MinMax(itemBoxes[idx]);
                    binCounts[bx]++;
                    binBoxes[SAHBinCount + by].MinMax(itemBoxes[idx]);
                    binCounts[SAHBinCount + by]++;
                    binBoxes[2 * SAHBinCount + bz].MinMax(itemBoxes[idx]);
                    binCounts[2 * SAHBinCount + bz]++;
                }
                double bestCost = double.MaxValue;
                int bestAxis = -1, bestSplit = -1;
                for (int axis = 0; axis < 3; axis++)
                {
                    int off = axis * SAHBinCount;
                    // suffix pass: suffixBoxes[s] = union of the bins [s..SAHBinCount)
                    BoundingBox acc = BoundingBox.EmptyBoundingBox;
                    for (int s = SAHBinCount - 1; s > 0; s--)
                    {
                        acc.MinMax(binBoxes[off + s]);
                        suffixBoxes[s] = acc;
                    }
                    // prefix pass, evaluating the cost at every bin boundary; on a degenerate
                    // axis all items fall into bin 0 and no boundary yields two non empty parts
                    BoundingBox leftBox = BoundingBox.EmptyBoundingBox;
                    int leftCount = 0;
                    for (int s = 1; s < SAHBinCount; s++)
                    {
                        leftBox.MinMax(binBoxes[off + s - 1]);
                        leftCount += binCounts[off + s - 1];
                        int rightCount = count - leftCount;
                        if (leftCount == 0 || rightCount == 0) continue;
                        double cost = leftCount * SurfaceArea(ref leftBox) + rightCount * SurfaceArea(ref suffixBoxes[s]);
                        if (cost < bestCost)
                        {
                            bestCost = cost;
                            bestAxis = axis;
                            bestSplit = s;
                        }
                    }
                }
                if (bestAxis < 0) return 0;
                // in place partition: items whose bin on the best axis lies left of the split
                // go to the front; uses the same bin formula as above, so the result is exactly
                // the left count that was evaluated and both parts are non empty
                double[] c = centroid[bestAxis];
                double cmin, k;
                if (bestAxis == 0) { cmin = cxmin; k = kx; }
                else if (bestAxis == 1) { cmin = cymin; k = ky; }
                else { cmin = czmin; k = kz; }
                int lo = start, hi = start + count - 1;
                while (lo <= hi)
                {
                    int bin = Math.Min((int)((c[itemIndices[lo]] - cmin) * k), SAHBinCount - 1);
                    if (bin < bestSplit) lo++;
                    else
                    {
                        int tmp = itemIndices[lo];
                        itemIndices[lo] = itemIndices[hi];
                        itemIndices[hi] = tmp;
                        hi--;
                    }
                }
                return lo - start;
            }

            // Recursively fills the node slot nodeIndex with the items itemIndices[start..start+count).
            void BuildInto(int nodeIndex, int start, int count, int depth)
            {
                if (depth > maxDepth) maxDepth = depth;
                Node node = default;
                node.box = BoundingBox.EmptyBoundingBox;
                for (int i = 0; i < count; i++) node.box.MinMax(itemBoxes[itemIndices[start + i]]);
                int mid = 0; // size of the left part, 0 (or count) means: make this node a leaf
                if (count > leafSize)
                {
                    // extent of the centroids, not of the boxes: when all centroids coincide
                    // no split can separate the items and we keep them in one leaf
                    double xmin = double.MaxValue, xmax = double.MinValue;
                    double ymin = double.MaxValue, ymax = double.MinValue;
                    double zmin = double.MaxValue, zmax = double.MinValue;
                    for (int i = 0; i < count; i++)
                    {
                        int idx = itemIndices[start + i];
                        if (cx[idx] < xmin) xmin = cx[idx];
                        if (cx[idx] > xmax) xmax = cx[idx];
                        if (cy[idx] < ymin) ymin = cy[idx];
                        if (cy[idx] > ymax) ymax = cy[idx];
                        if (cz[idx] < zmin) zmin = cz[idx];
                        if (cz[idx] > zmax) zmax = cz[idx];
                    }
                    double dx = xmax - xmin, dy = ymax - ymin, dz = zmax - zmin;
                    if (dx > 0.0 || dy > 0.0 || dz > 0.0)
                    {
                        if (strategy == BVHBuildStrategy.BinnedSAH)
                            mid = PartitionBinnedSAH(start, count, xmin, xmax, ymin, ymax, zmin, zmax);
                        if (mid <= 0 || mid >= count)
                        {
                            int axis;
                            if (dx >= dy && dx >= dz) axis = 0;
                            else if (dy >= dz) axis = 1;
                            else axis = 2;
                            mid = PartitionMedian(start, count, axis);
                        }
                    }
                }
                if (mid <= 0 || mid >= count)
                {
                    node.start = start;
                    node.count = count;
                    nodeList[nodeIndex] = node;
                    return;
                }
                node.leftChild = nodeList.Count;
                node.count = 0;
                nodeList[nodeIndex] = node;
                nodeList.Add(default); // left child slot
                nodeList.Add(default); // right child slot
                BuildInto(node.leftChild, start, mid, depth + 1);
                BuildInto(node.leftChild + 1, start + mid, count - mid, depth + 1);
            }

            nodeList.Add(default); // root slot
            BuildInto(0, 0, n, 1);
            nodes = nodeList.ToArray();
        }

        /// <summary>
        /// Creates a BVH over the triangles of a triangulation as produced by
        /// <see cref="GeoObject.Face.GetTriangulation(double, out GeoPoint[], out GeoPoint2D[], out int[], out BoundingBox)"/>.
        /// Result index k corresponds to the triangle with the vertices
        /// trianglePoint[triangleIndex[3*k]], trianglePoint[triangleIndex[3*k+1]], trianglePoint[triangleIndex[3*k+2]].
        /// </summary>
        public static BVHTree FromTriangles(GeoPoint[] trianglePoint, int[] triangleIndex, int leafSize = DefaultLeafSize, BVHBuildStrategy strategy = BVHBuildStrategy.BinnedSAH)
        {
            if (trianglePoint == null) throw new ArgumentNullException(nameof(trianglePoint));
            if (triangleIndex == null) throw new ArgumentNullException(nameof(triangleIndex));
            int n = triangleIndex.Length / 3;
            BoundingBox[] boxes = new BoundingBox[n];
            for (int i = 0; i < n; i++)
            {
                BoundingBox box = BoundingBox.EmptyBoundingBox;
                box.MinMax(trianglePoint[triangleIndex[3 * i]]);
                box.MinMax(trianglePoint[triangleIndex[3 * i + 1]]);
                box.MinMax(trianglePoint[triangleIndex[3 * i + 2]]);
                boxes[i] = box;
            }
            return new BVHTree(boxes, leafSize, strategy);
        }

        /// <summary>
        /// Generic traversal: yields the indices of all items whose box satisfies <paramref name="nodeTest"/>,
        /// pruning whole subtrees whose node box fails the test. The test must be "monotonic" in the sense
        /// that whenever it fails for a box it also fails for every box contained in it (which is the case
        /// for all natural conditions like "intersects a given box / ray / plane / sphere").
        /// </summary>
        public IEnumerable<int> Traverse(Func<BoundingBox, bool> nodeTest)
        {
            if (nodeTest == null) throw new ArgumentNullException(nameof(nodeTest));
            if (Count == 0) yield break;
            int[] stack = new int[maxDepth + 1];
            int sp = 0;
            stack[sp++] = 0;
            while (sp > 0)
            {
                Node node = nodes[stack[--sp]];
                if (!nodeTest(node.box)) continue;
                if (node.count > 0)
                {
                    for (int i = 0; i < node.count; i++)
                    {
                        int idx = itemIndices[node.start + i];
                        if (nodeTest(itemBoxes[idx])) yield return idx;
                    }
                }
                else
                {
                    stack[sp++] = node.leftChild;
                    stack[sp++] = node.leftChild + 1;
                }
            }
        }

        /// <summary>
        /// Yields the indices of all items whose box interferes with the provided box.
        /// </summary>
        public IEnumerable<int> Overlapping(BoundingBox box)
        {
            return Traverse(b => b.Interferes(box));
        }

        /// <summary>
        /// Dual traversal: yields all pairs (i, j) where the box of item i of this tree interferes with
        /// the box of item j of <paramref name="other"/>. This is the typical candidate search for
        /// surface/surface intersection or collision detection between two triangulated objects.
        /// </summary>
        public IEnumerable<(int, int)> Overlapping(BVHTree other)
        {
            if (other == null) throw new ArgumentNullException(nameof(other));
            if (Count == 0 || other.Count == 0) yield break;
            Stack<(int, int)> stack = new Stack<(int, int)>();
            stack.Push((0, 0));
            while (stack.Count > 0)
            {
                (int ia, int ib) = stack.Pop();
                Node a = nodes[ia];
                Node b = other.nodes[ib];
                if (!a.box.Interferes(b.box)) continue;
                if (a.count > 0 && b.count > 0)
                {
                    for (int i = 0; i < a.count; i++)
                    {
                        int ii = itemIndices[a.start + i];
                        for (int j = 0; j < b.count; j++)
                        {
                            int jj = other.itemIndices[b.start + j];
                            if (itemBoxes[ii].Interferes(other.itemBoxes[jj])) yield return (ii, jj);
                        }
                    }
                }
                else if (b.count > 0 || (a.count == 0 && ExtentSum(ref a.box) >= ExtentSum(ref b.box)))
                {
                    // descend this tree: either the other node is a leaf or this node is the bigger one
                    stack.Push((a.leftChild, ib));
                    stack.Push((a.leftChild + 1, ib));
                }
                else
                {
                    stack.Push((ia, b.leftChild));
                    stack.Push((ia, b.leftChild + 1));
                }
            }
        }

        /// <summary>
        /// Yields all pairs (i, j) with i &lt; j whose item boxes interfere with each other,
        /// e.g. the candidate pairs for a self-intersection test. Each pair is reported exactly once.
        /// </summary>
        public IEnumerable<(int, int)> SelfOverlappingPairs()
        {
            if (Count < 2) yield break;
            Stack<(int, int)> stack = new Stack<(int, int)>();
            stack.Push((0, 0));
            while (stack.Count > 0)
            {
                (int ia, int ib) = stack.Pop();
                Node a = nodes[ia];
                if (ia == ib)
                {
                    if (a.count > 0)
                    {
                        for (int i = 0; i < a.count; i++)
                        {
                            int ii = itemIndices[a.start + i];
                            for (int j = i + 1; j < a.count; j++)
                            {
                                int jj = itemIndices[a.start + j];
                                if (itemBoxes[ii].Interferes(itemBoxes[jj])) yield return Ordered(ii, jj);
                            }
                        }
                    }
                    else
                    {
                        stack.Push((a.leftChild, a.leftChild));
                        stack.Push((a.leftChild + 1, a.leftChild + 1));
                        stack.Push((a.leftChild, a.leftChild + 1));
                    }
                }
                else
                {
                    Node b = nodes[ib];
                    if (!a.box.Interferes(b.box)) continue;
                    if (a.count > 0 && b.count > 0)
                    {
                        for (int i = 0; i < a.count; i++)
                        {
                            int ii = itemIndices[a.start + i];
                            for (int j = 0; j < b.count; j++)
                            {
                                int jj = itemIndices[b.start + j];
                                if (itemBoxes[ii].Interferes(itemBoxes[jj])) yield return Ordered(ii, jj);
                            }
                        }
                    }
                    else if (b.count > 0 || (a.count == 0 && ExtentSum(ref a.box) >= ExtentSum(ref b.box)))
                    {
                        stack.Push((a.leftChild, ib));
                        stack.Push((a.leftChild + 1, ib));
                    }
                    else
                    {
                        stack.Push((ia, b.leftChild));
                        stack.Push((ia, b.leftChild + 1));
                    }
                }
            }
        }

        private static (int, int) Ordered(int i, int j)
        {
            return i < j ? (i, j) : (j, i);
        }

        private static double ExtentSum(ref BoundingBox box)
        {
            // heuristic size measure used to decide which node to descend in the dual traversal
            return (box.Xmax - box.Xmin) + (box.Ymax - box.Ymin) + (box.Zmax - box.Zmin);
        }
    }

    /// <summary>
    /// Thin convenience wrapper around <see cref="BVHTree"/> that keeps the items and extracts their
    /// boxes via a delegate, so query results can be returned as items instead of indices.
    /// There is no interface the items have to implement (in contrast to <see cref="IOctTreeInsertable"/>).
    /// </summary>
    /// <typeparam name="T">Any type; a delegate provides the bounding box per item</typeparam>
    public class BVH<T>
    {
        private readonly IReadOnlyList<T> items;

        /// <summary>
        /// The underlying index based tree, for queries that need indices or custom traversals.
        /// Index i of the tree corresponds to <see cref="this[int]"/>.
        /// </summary>
        public BVHTree Tree { get; }

        public int Count => items.Count;

        public T this[int index] => items[index];

        public BVH(IReadOnlyList<T> items, Func<T, BoundingBox> getBox, int leafSize = BVHTree.DefaultLeafSize, BVHBuildStrategy strategy = BVHBuildStrategy.BinnedSAH)
        {
            if (items == null) throw new ArgumentNullException(nameof(items));
            if (getBox == null) throw new ArgumentNullException(nameof(getBox));
            this.items = items;
            BoundingBox[] boxes = new BoundingBox[items.Count];
            for (int i = 0; i < items.Count; i++) boxes[i] = getBox(items[i]);
            Tree = new BVHTree(boxes, leafSize, strategy);
        }

        /// <summary>
        /// Yields all items whose box satisfies the provided test, see <see cref="BVHTree.Traverse"/>.
        /// </summary>
        public IEnumerable<T> Traverse(Func<BoundingBox, bool> nodeTest)
        {
            foreach (int i in Tree.Traverse(nodeTest)) yield return items[i];
        }

        /// <summary>
        /// Yields all items whose box interferes with the provided box.
        /// </summary>
        public IEnumerable<T> Overlapping(BoundingBox box)
        {
            foreach (int i in Tree.Overlapping(box)) yield return items[i];
        }

        /// <summary>
        /// Yields all pairs of items of this and the other hierarchy whose boxes interfere with each other.
        /// </summary>
        public IEnumerable<(T, TOther)> Overlapping<TOther>(BVH<TOther> other)
        {
            foreach ((int i, int j) in Tree.Overlapping(other.Tree)) yield return (items[i], other.items[j]);
        }
    }
}
