using System;
using System.Collections.Generic;
using System.Linq;
using CADability;
using CADability.GeoObject;

namespace ShapeIt
{
    /// <summary>
    /// The split invariant view of a shell: every set of connected faces that lies on one and the same surface
    /// is a single <b>patch</b>, and the edges between those faces do not exist.
    /// <para>
    /// This exists because where a shell is cut into faces is not part of its meaning. CADability requires a
    /// periodic surface to be split whenever the whole cycle is used - the boundary of a face must always span
    /// less than a full period - which makes many BRep operations simpler at the price of an arbitrary seam.
    /// On top of that a boolean operation splits faces while it works and merges them back afterwards
    /// (<see cref="Shell.CombineConnectedFaces"/>), and how far that merge gets is an implementation detail: it
    /// stops at half a period, and it refuses a few surface types it cannot combine. So the plain face, edge
    /// and vertex counts move when nothing about the result has changed, which is exactly the kind of false
    /// alarm the regression baselines must not contain.
    /// </para>
    /// <para>
    /// The partition is computed by counting only - no geometry is modified, nothing is merged for real. That
    /// matters because these shells are the broken ones: <see cref="Shell.CombineConnectedFaces"/> would be the
    /// obvious way to canonicalize, but it performs real surgery, it can fail on exactly the data this harness
    /// exists for, and it is itself only canonical up to half a period.
    /// </para>
    /// <para>
    /// The Euler characteristic is deliberately <b>not</b> computed here: V - E + F - R is already invariant
    /// under splitting a face (a cut adds one face, one edge and no vertex, or one face, three edges and two
    /// vertices - the sum does not move), so <see cref="ShellMetrics.EulerCharacteristic"/> keeps reading the
    /// shell as it is. It is also the one number a patch complex cannot always express: a closed patch such as
    /// a whole sphere has no boundary at all and is not a valid cell decomposition.
    /// </para>
    /// </summary>
    public sealed class ShellPartition
    {
        /// <summary>Number of patches - the split invariant face count.</summary>
        public int Patches { get; private set; }
        /// <summary>Number of canonical edges: chains of real edges between the same two patches count once.</summary>
        public int Edges { get; private set; }
        /// <summary>Number of canonical vertices: those where more or less than two canonical edges meet.</summary>
        public int Vertices { get; private set; }
        /// <summary>
        /// Inner loops summed over all patches, i.e. every boundary loop of a patch except its first.
        /// <para>
        /// A loop is a connected component of the boundary of a patch, so two loops that TOUCH at a vertex
        /// are counted as one - the union of touching rings in UniteBug4/5/6 is where that happens, and the
        /// value comes out one or two short there. It is deterministic and split invariant all the same (a
        /// pinch belongs to the geometry, not to the splitting), and the raw
        /// <see cref="ShellMetrics.EulerCharacteristic"/> is what carries the exact topology anyway - it has
        /// the same blind spot, since <see cref="Face.HoleCount"/> counts a pinched outline as one loop too.
        /// Counting them apart would mean walking the boundary in order instead of by connectivity.
        /// </para>
        /// </summary>
        public int HoleLoops { get; private set; }
        /// <summary>Vertices carrying at least one pole edge - one per cone apex, two per sphere.</summary>
        public int PoleVertices { get; private set; }
        /// <summary>Total 3d length of the canonical edges; the seams between two faces of a patch do not count.</summary>
        public double EdgeLength { get; private set; }
        /// <summary>Histogram of the surface types over the patches, e.g. "ConicalSurface:2 PlaneSurface:12".</summary>
        public string Surfaces { get; private set; } = "";

        /// <summary>
        /// The precision the surfaces of two neighbouring faces are held against. Same value that
        /// <see cref="Shell.CombineConnectedFaces"/> uses (size * 1e-6), but derived from the exact geometry
        /// rather than from <see cref="Shell.GetExtent"/>, which would drag a triangulation into it and with it
        /// the run to run scatter that <see cref="ShellMetrics.PrecisionFor"/> exists to avoid.
        /// </summary>
        private static double SurfaceIdentityPrecision(Shell shell)
            => ShellMetrics.SizeOf(shell) * 1e-6;

        public static ShellPartition Of(Shell shell)
        {
            ShellPartition result = new ShellPartition();
            Face[] faces = shell.Faces;
            double precision = SurfaceIdentityPrecision(shell);

            Dictionary<Face, int> faceIndex = new Dictionary<Face, int>();
            for (int i = 0; i < faces.Length; i++) faceIndex[faces[i]] = i;

            // 1. sort the edges: an edge with the same surface on both sides is where a face was split, the
            //    rest is the real boundary between two patches. A pole edge is no edge of the solid at all.
            UnionFind patchOf = new UnionFind(faces.Length);
            List<Edge> boundary = new List<Edge>();
            HashSet<Vertex> poleVertices = new HashSet<Vertex>();
            foreach (Edge edge in shell.Edges)
            {
                if (edge.Curve3D == null)
                {
                    if (edge.Vertex1 != null) poleVertices.Add(edge.Vertex1);
                    if (edge.Vertex2 != null) poleVertices.Add(edge.Vertex2);
                    continue;
                }
                // these shells are the broken ones, so an edge without a primary face is not impossible
                if (edge.PrimaryFace == null) continue;
                if (!faceIndex.TryGetValue(edge.PrimaryFace, out int primary)) continue;
                if (edge.SecondaryFace == null) { boundary.Add(edge); continue; } // open: has to show up
                if (!faceIndex.TryGetValue(edge.SecondaryFace, out int secondary)) continue;
                // a face glued to itself along this edge: the seam of a periodic face that was not split.
                // There is no boundary between two patches here either way.
                if (primary == secondary) continue;
                if (IsOneSurface(edge, precision)) patchOf.Union(primary, secondary);
                else boundary.Add(edge);
            }
            result.PoleVertices = poleVertices.Count;

            // 2. the patches themselves
            Dictionary<int, Face> representative = new Dictionary<int, Face>();
            for (int i = 0; i < faces.Length; i++)
            {
                int root = patchOf.Find(i);
                if (!representative.ContainsKey(root)) representative[root] = faces[i];
            }
            result.Patches = representative.Count;
            result.Surfaces = Histogram(representative.Values.Select(
                f => f.Surface == null ? "<null>" : f.Surface.GetType().Name));

            // 3. how many boundary edges meet at each vertex. Splitting a face also cuts the edges of its
            //    neighbours in two, and the new vertex is exactly the one where two boundary edges and nothing
            //    else meet - so a vertex of degree two is an artefact and its two edges are one canonical edge.
            Dictionary<Vertex, List<int>> atVertex = new Dictionary<Vertex, List<int>>();
            for (int i = 0; i < boundary.Count; i++)
            {
                // a closed edge lists its vertex twice, which is what makes its degree come out as two
                AddIncidence(atVertex, boundary[i].Vertex1, i);
                AddIncidence(atVertex, boundary[i].Vertex2, i);
                if (boundary[i].Curve3D != null) result.EdgeLength += boundary[i].Curve3D.Length;
            }

            UnionFind chain = new UnionFind(boundary.Count);
            foreach (KeyValuePair<Vertex, List<int>> at in atVertex)
                if (at.Value.Count == 2) chain.Union(at.Value[0], at.Value[1]);

            HashSet<int> chainsWithAVertex = new HashSet<int>();
            int vertices = 0;
            foreach (KeyValuePair<Vertex, List<int>> at in atVertex)
            {
                if (at.Value.Count == 2) continue;
                ++vertices;
                foreach (int i in at.Value) chainsWithAVertex.Add(chain.Find(i));
            }
            HashSet<int> chains = new HashSet<int>();
            for (int i = 0; i < boundary.Count; i++) chains.Add(chain.Find(i));
            // a chain that closes on itself - the rim of a cylinder, say - has no vertex of a degree other than
            // two, so every one of them was contracted away. Keep one, or the edge would have no end point at all.
            foreach (int root in chains) if (!chainsWithAVertex.Contains(root)) ++vertices;
            result.Edges = chains.Count;
            result.Vertices = vertices;

            // 4. the boundary loops of each patch. Everything except the first loop of a patch is a hole -
            //    that is the same convention Face.HoleCount follows, lifted from the face to the patch.
            Dictionary<int, List<int>> edgesOfPatch = new Dictionary<int, List<int>>();
            for (int i = 0; i < boundary.Count; i++)
            {
                Edge edge = boundary[i];
                int primary = patchOf.Find(faceIndex[edge.PrimaryFace]);
                AddTo(edgesOfPatch, primary, i);
                if (edge.SecondaryFace != null && faceIndex.TryGetValue(edge.SecondaryFace, out int s))
                {
                    int secondary = patchOf.Find(s);
                    if (secondary != primary) AddTo(edgesOfPatch, secondary, i);
                }
            }
            int holeLoops = 0;
            foreach (List<int> ofOnePatch in edgesOfPatch.Values) holeLoops += LoopCount(ofOnePatch, boundary) - 1;
            result.HoleLoops = holeLoops;
            return result;
        }

        /// <summary>The boundary loops of one patch: the connected components of its boundary edges.</summary>
        private static int LoopCount(List<int> edgesOfPatch, List<Edge> boundary)
        {
            Dictionary<int, int> local = new Dictionary<int, int>();
            for (int i = 0; i < edgesOfPatch.Count; i++) local[edgesOfPatch[i]] = i;
            UnionFind loop = new UnionFind(edgesOfPatch.Count);
            Dictionary<Vertex, int> first = new Dictionary<Vertex, int>();
            foreach (int index in edgesOfPatch)
            {
                foreach (Vertex? vertex in new[] { boundary[index].Vertex1, boundary[index].Vertex2 })
                {
                    if (vertex == null) continue;
                    if (first.TryGetValue(vertex, out int other)) loop.Union(local[index], other);
                    else first[vertex] = local[index];
                }
            }
            HashSet<int> roots = new HashSet<int>();
            for (int i = 0; i < edgesOfPatch.Count; i++) roots.Add(loop.Find(i));
            return roots.Count;
        }

        /// <summary>
        /// Whether the two faces of this edge lie on one and the same surface, so that the edge between them is
        /// an artificial split rather than a real edge of the solid.
        /// <para>
        /// This is the test <see cref="Shell.CombineConnectedFaces"/> uses to decide whether two faces may be
        /// merged, minus everything that is about the merge rather than about the surfaces: the half period
        /// limit, and the surface types whose <c>CombineWith</c> is not implemented. Neither says the surfaces
        /// differ, and here nothing is merged, only counted.
        /// </para>
        /// <para>
        /// What remains of the determinant condition is about the geometry: the same surface traversed the
        /// other way round is a different face of the solid, not a split of one. That question is only asked
        /// of the parameter mapping where there is one - see below.
        /// </para>
        /// </summary>
        private static bool IsOneSurface(Edge edge, double precision)
        {
            Face? primaryFace = edge.PrimaryFace, secondaryFace = edge.SecondaryFace;
            ISurface? primary = primaryFace?.Surface;
            ISurface? secondary = secondaryFace?.Surface;
            if (primaryFace == null || secondaryFace == null || primary == null || secondary == null) return false;
            if (ReferenceEquals(primary, secondary)) return true;
            // Two faces are only ever split apart into copies of one surface, so different types cannot be a
            // split - and this keeps SameGeometry, which is expensive for a NURBS surface, off most edges.
            if (primary.GetType() != secondary.GetType()) return false;
            try
            {
                if (!secondary.SameGeometry(secondaryFace.GetUVBounds(), primary,
                    primaryFace.GetUVBounds(), precision, out ModOp2D firstToSecond)) return false;
                if (firstToSecond.Determinant > 0) return true;
                // Several surfaces confirm that it is one and the same surface but cannot say how the two
                // parameter spaces map onto each other, and return ModOp2D.Null - a sphere or a cone whose
                // two faces were built with different axes, for instance. Those are exactly the ones
                // CombineConnectedFaces has to give up on, so they stay split wherever the split happened to
                // fall and it is here that they have to be recognized. The orientation, which the determinant
                // would have answered, is then asked of the geometry directly.
                return firstToSecond.IsNull && NormalsAgree(edge, primaryFace, secondaryFace);
            }
            catch (Exception) { return false; }
        }

        /// <summary>
        /// Whether the two faces lie on their common surface the same way round: their normals at the middle
        /// of the shared edge point to the same side. The middle, because at an end point the normal can be
        /// undefined - that end may be the apex of a cone or the pole of a sphere.
        /// </summary>
        private static bool NormalsAgree(Edge edge, Face primary, Face secondary)
        {
            ICurve? curve = edge.Curve3D;
            if (curve == null) return false;
            GeoPoint point = curve.PointAt(0.5);
            GeoVector first = primary.Surface.GetNormal(primary.Surface.PositionOf(point));
            GeoVector second = secondary.Surface.GetNormal(secondary.Surface.PositionOf(point));
            if (Precision.IsNullVector(first) || Precision.IsNullVector(second)) return false;
            return first.Normalized * second.Normalized > 0.0;
        }

        private static void AddIncidence(Dictionary<Vertex, List<int>> map, Vertex? vertex, int edge)
        {
            if (vertex == null) return;
            AddTo(map, vertex, edge);
        }

        private static void AddTo<TKey>(Dictionary<TKey, List<int>> map, TKey key, int value) where TKey : notnull
        {
            if (!map.TryGetValue(key, out List<int>? list)) map[key] = list = new List<int>();
            list.Add(value);
        }

        private static string Histogram(IEnumerable<string> names)
        {
            Dictionary<string, int> counts = new Dictionary<string, int>(StringComparer.Ordinal);
            foreach (string name in names)
            {
                counts.TryGetValue(name, out int count);
                counts[name] = count + 1;
            }
            return string.Join(" ", counts.OrderBy(kv => kv.Key, StringComparer.Ordinal).Select(kv => kv.Key + ":" + kv.Value));
        }

        /// <summary>Plain union-find over a fixed number of items, with path compression.</summary>
        private sealed class UnionFind
        {
            private readonly int[] parent;
            public UnionFind(int count)
            {
                parent = new int[count];
                for (int i = 0; i < count; i++) parent[i] = i;
            }
            public int Find(int i)
            {
                while (parent[i] != i) i = parent[i] = parent[parent[i]];
                return i;
            }
            public void Union(int a, int b)
            {
                int ra = Find(a), rb = Find(b);
                if (ra != rb) parent[ra] = rb;
            }
        }
    }
}
