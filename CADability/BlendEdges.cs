using CADability;
using CADability.GeoObject;
using CADability.Substitutes;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CADability.GeoObject
{
    public class BlendEdges
    {
        public Shell shell;
        public IEnumerable<Edge> convexEdges;
        public IEnumerable<Edge> concaveEdges;
        public Dictionary<Edge, Shell>? edgeToCutter;

        public BlendEdges(Shell shell, IEnumerable<Edge> edges)
        {
            this.shell = shell;
            // convex and concave: an edge is convex when the angle between the two faces is less than 108°, concave when greater than 180°
            convexEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Convex);
            concaveEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Concave);
        }

        /// <summary>
        /// A vertex where a selected convex and a selected concave edge meet. Exactly three faces and three edges meet
        /// there: two edges of one kind (<see cref="Pair"/>) and one edge of the other kind (<see cref="Odd"/>), which
        /// runs into <see cref="PairFace"/>, the face common to the two pair edges.
        /// </summary>
        protected class MixedVertex
        {
            public Vertex Vertex;
            public Edge Odd;
            public Edge[] Pair;
            public Face PairFace;
            public MixedVertex(Vertex vertex, Edge odd, Edge[] pair, Face pairFace)
            {
                Vertex = vertex;
                Odd = odd;
                Pair = pair;
                PairFace = pairFace;
            }
        }
        /// <summary>
        /// The selected edges, split into stages which are blended one after the other, see <see cref="PlanStages"/>.
        /// </summary>
        private List<List<Edge>>? stages;
        private List<MixedVertex> mixedVertices = [];

        /// <summary>
        /// Checks whether the selection can be blended. Where a convex and a concave edge of the selection meet in a
        /// common vertex, the order of blending matters, see <see cref="PlanStages"/>. Selections, for which there is
        /// no consistent order, are rejected, as well as edges whose convexity changes along their way.
        /// </summary>
        /// <returns>null, if the selection is valid, otherwise a description of the problem</returns>
        public string? CheckSelection()
        {
            int mixedEdges = convexEdges.Concat(concaveEdges).Count(e => HasChangingConvexity(e));
            if (mixedEdges > 0)
                return $"Convex and concave edges cannot be blended together: {mixedEdges} edge(s) change from convex to concave along their way. "
                    + "Blend the convex parts first and then the concave parts of the result, or the other way round.";
            string? problem = PlanStages();
            if (problem == null) return null;
            return "Convex and concave edges cannot be blended together where they meet: " + problem
                + ". Blend them in separate steps: at a vertex where one convex and two concave edges meet (or vice versa), the single edge first.";
        }

        /// <summary>
        /// Splits the selection into stages. At a vertex where a convex and a concave edge of the selection meet (and
        /// three faces), there are two edges of one kind and one "odd" edge of the other kind. The odd edge has to be
        /// blended first: its blend turns the corner of the face common to the other two edges into a tangential
        /// transition, and the two edges together with the new edge between this face and the blend (the "bridge")
        /// form a tangent chain of one kind. In the other order the odd edge would end in the apex of the corner
        /// patch of the two other edges (a horn torus or a cone), which cannot be blended any more.
        /// Selected edges of the same kind, which are connected by a common vertex, stay in the same stage, because
        /// the corners between them are made in one operation. When the order of these groups is contradictory,
        /// or a mixed vertex is not a simple vertex of three faces, the selection is rejected.
        /// </summary>
        /// <returns>null, if there is a valid plan, otherwise a description of the problem</returns>
        protected string? PlanStages()
        {
            stages = null;
            mixedVertices = [];
            HashSet<Edge> convex = new HashSet<Edge>(convexEdges);
            HashSet<Edge> concave = new HashSet<Edge>(concaveEdges);
            List<Edge> selected = convex.Concat(concave).ToList();
            // group the selected edges: edges of the same kind, connected by a common vertex
            Dictionary<Edge, int> group = new Dictionary<Edge, int>();
            for (int i = 0; i < selected.Count; i++) group[selected[i]] = i;
            int find(Edge e)
            {
                int g = group[e];
                while (group[selected[g]] != g) g = group[selected[g]];
                group[e] = g;
                return g;
            }
            HashSet<Vertex> vertices = new HashSet<Vertex>(selected.SelectMany(e => new[] { e.Vertex1, e.Vertex2 }));
            // a vertex may still reference edges of the shells it was made from (e.g. by a boolean operation)
            HashSet<Edge> shellEdges = new HashSet<Edge>(shell.Edges);
            foreach (Vertex vtx in vertices)
            {
                foreach (HashSet<Edge> kind in new[] { convex, concave })
                {
                    List<Edge> sameKind = vtx.Edges.Where(e => kind.Contains(e)).Distinct().ToList();
                    for (int i = 1; i < sameKind.Count; i++) group[selected[find(sameKind[i])]] = find(sameKind[0]);
                }
            }
            // the order between the groups, given by the mixed vertices
            HashSet<(int before, int after)> order = new HashSet<(int, int)>();
            foreach (Vertex vtx in vertices)
            {
                Edge[] vedges = vtx.Edges.Where(e => shellEdges.Contains(e)).Distinct().ToArray();
                if (!vedges.Any(e => convex.Contains(e)) || !vedges.Any(e => concave.Contains(e))) continue;
                GeoPoint p = vtx.Position;
                string where = $"({p.x:G6}, {p.y:G6}, {p.z:G6})";
                if (vedges.Length != 3) return $"{vedges.Length} edges meet at {where}, only vertices with three edges are supported";
                ShellExtensions.AdjacencyType[] kinds = vedges.Select(e => e.Adjacency()).ToArray();
                if (kinds.Any(k => k != ShellExtensions.AdjacencyType.Convex && k != ShellExtensions.AdjacencyType.Concave))
                    return $"a tangential edge meets the selected edges at {where}";
                int oddIndex = Enumerable.Range(0, 3).Single(i => kinds.Count(k => k == kinds[i]) == 1);
                Edge odd = vedges[oddIndex];
                Edge[] pair = vedges.Where((e, i) => i != oddIndex).ToArray();
                Face? pairFace = Edge.CommonFace(pair[0], pair[1]);
                if (pairFace == null) return $"the edges at {where} have no common face";
                mixedVertices.Add(new MixedVertex(vtx, odd, pair, pairFace));
                // the odd edge is selected: otherwise the selected edges at this vertex would be of the same kind
                foreach (Edge e in pair)
                {
                    if (group.ContainsKey(e)) order.Add((find(odd), find(e)));
                }
            }
            // longest path levels of the groups (Kahn's algorithm), a cycle means a contradictory order
            List<int> groups = selected.Select(e => find(e)).Distinct().ToList();
            Dictionary<int, int> level = groups.ToDictionary(g => g, g => 0);
            Dictionary<int, int> incoming = groups.ToDictionary(g => g, g => order.Count(o => o.after == g));
            Queue<int> ready = new Queue<int>(groups.Where(g => incoming[g] == 0));
            int done = 0;
            while (ready.Count > 0)
            {
                int g = ready.Dequeue();
                done++;
                foreach ((int before, int after) in order.Where(o => o.before == g))
                {
                    level[after] = Math.Max(level[after], level[g] + 1);
                    if (--incoming[after] == 0) ready.Enqueue(after);
                }
            }
            if (done < groups.Count)
            {
                GeoPoint p = mixedVertices[0].Vertex.Position;
                return $"the order of blending convex and concave edges is contradictory, e.g. at ({p.x:G6}, {p.y:G6}, {p.z:G6})";
            }
            stages = selected.GroupBy(e => level[find(e)]).OrderBy(g => g.Key).Select(g => g.ToList()).ToList();
            return null;
        }

        /// <summary>
        /// Blends the selection stage by stage, see <see cref="PlanStages"/>. <paramref name="executeStage"/> blends the
        /// provided edges of the provided shell in one operation. For the later stages, the edges are looked up in the
        /// result of the previous stage, and the bridges between pair edges, which are both blended in this stage, are
        /// added.
        /// </summary>
        protected Shell? ExecuteStaged(Func<Shell, List<Edge>, Shell?> executeStage)
        {
            if (CheckSelection() != null || stages == null) return null;
            Dictionary<Edge, int> stageOf = new Dictionary<Edge, int>();
            for (int i = 0; i < stages.Count; i++)
            {
                foreach (Edge e in stages[i]) stageOf[e] = i;
            }
            Shell current = shell;
            for (int i = 0; i < stages.Count; i++)
            {
                List<Edge> edges;
                if (i == 0) edges = stages[0];
                else
                {
                    edges = stages[i].SelectMany(e => FindImages(current, e)).ToList();
                    foreach (MixedVertex mv in mixedVertices)
                    {
                        if (stageOf.TryGetValue(mv.Pair[0], out int s0) && s0 == i && stageOf.TryGetValue(mv.Pair[1], out int s1) && s1 == i
                            && stageOf[mv.Odd] < i) edges.AddRange(FindBridge(current, mv));
                    }
                    edges = edges.Distinct().ToList();
                    if (edges.Count == 0) return null;
                }
                Shell? next = executeStage(current, edges);
                if (next == null) return null;
                current = next;
            }
            return current;
        }

        /// <summary>
        /// The edges of <paramref name="inShell"/>, which are what is left of <paramref name="original"/> after a blending
        /// stage: they lie on the curve of the original edge and between the same surfaces (same normals).
        /// </summary>
        private static List<Edge> FindImages(Shell inShell, Edge original)
        {
            List<Edge> res = [];
            if (original.SecondaryFace == null) return res;
            ICurve curve = original.Curve3D;
            double tolerance = 10 * Precision.eps;
            foreach (Edge edge in inShell.Edges)
            {
                if (edge.SecondaryFace == null || edge.Curve3D == null) continue;
                bool onCurve = true;
                foreach (double t in new[] { 0.0, 0.5, 1.0 })
                {
                    GeoPoint p = edge.Curve3D.PointAt(t);
                    double pos = curve.PositionOf(p);
                    if (pos < -1e-6 || pos > 1 + 1e-6 || (curve.PointAt(pos) | p) > tolerance)
                    {
                        onCurve = false;
                        break;
                    }
                }
                if (!onCurve) continue;
                GeoPoint m = edge.Curve3D.PointAt(0.5);
                GeoVector n1 = NormalAt(edge.PrimaryFace, m), n2 = NormalAt(edge.SecondaryFace, m);
                GeoVector o1 = NormalAt(original.PrimaryFace, m), o2 = NormalAt(original.SecondaryFace, m);
                if ((SameNormal(n1, o1) && SameNormal(n2, o2)) || (SameNormal(n1, o2) && SameNormal(n2, o1))) res.Add(edge);
            }
            return res;
        }

        /// <summary>
        /// The bridge at a mixed vertex after the odd edge has been blended: the edges of the face on the surface of
        /// <see cref="MixedVertex.PairFace"/>, which connect the remainder of the first pair edge with the remainder of
        /// the second pair edge (the edges between this face and the blend of the odd edge).
        /// </summary>
        private static List<Edge> FindBridge(Shell inShell, MixedVertex mv)
        {
            HashSet<Edge> imagesA = new HashSet<Edge>(FindImages(inShell, mv.Pair[0]));
            HashSet<Edge> imagesB = new HashSet<Edge>(FindImages(inShell, mv.Pair[1]));
            List<Edge>? best = null;
            foreach (Edge a in imagesA)
            {
                GeoPoint m = a.Curve3D.PointAt(0.5);
                GeoVector pairNormal = NormalAt(mv.PairFace, m);
                foreach (Face face in new[] { a.PrimaryFace, a.SecondaryFace })
                {
                    if (!SameNormal(NormalAt(face, m), pairNormal)) continue;
                    List<Edge[]> loops = [face.OutlineEdges];
                    for (int h = 0; h < face.HoleCount; h++) loops.Add(face.HoleEdges(h));
                    foreach (Edge[] loop in loops)
                    {
                        int ind = Array.IndexOf(loop, a);
                        if (ind < 0) continue;
                        foreach (int dir in new[] { 1, -1 })
                        {   // walk along the loop until we reach the second pair edge
                            List<Edge> path = [];
                            for (int k = 1; k < loop.Length && path.Count <= 8; k++)
                            {
                                Edge e = loop[((ind + dir * k) % loop.Length + loop.Length) % loop.Length];
                                if (imagesB.Contains(e))
                                {
                                    if (best == null || path.Count < best.Count) best = path;
                                    break;
                                }
                                if (imagesA.Contains(e)) break;
                                path.Add(e);
                            }
                        }
                    }
                }
            }
            return best ?? [];
        }

        private static GeoVector NormalAt(Face face, GeoPoint p)
        {
            return face.Surface.GetNormal(face.Surface.PositionOf(p)).Normalized;
        }

        private static bool SameNormal(GeoVector n1, GeoVector n2)
        {
            return n1 * n2 > 1 - 1e-6;
        }

        /// <summary>
        /// True, if the edge is convex in some parts and concave in others. <see cref="ShellExtensions.Adjacency"/>
        /// only looks at the start of the edge, so it is sampled here along the whole edge. Tangential sample points
        /// (no clear sign) are ignored.
        /// </summary>
        private static bool HasChangingConvexity(Edge edge)
        {
            if (edge.SecondaryFace == null || edge.Curve3D == null) return false;
            bool convex = false, concave = false;
            const int samples = 8;
            for (int i = 0; i <= samples; i++)
            {
                double t = (double)i / samples;
                GeoPoint p = edge.Curve3D.PointAt(t);
                GeoVector dir = edge.Curve3D.DirectionAt(t);
                if (!edge.Forward(edge.PrimaryFace)) dir = -dir;
                GeoVector n1 = edge.PrimaryFace.Surface.GetNormal(edge.PrimaryFace.Surface.PositionOf(p));
                GeoVector n2 = edge.SecondaryFace.Surface.GetNormal(edge.SecondaryFace.Surface.PositionOf(p));
                if (dir.IsNullVector() || n1.IsNullVector() || n2.IsNullVector()) continue;
                double orientation = dir.Normalized * (n1.Normalized ^ n2.Normalized);
                if (orientation > 1e-3) convex = true;
                else if (orientation < -1e-3) concave = true;
            }
            return convex && concave;
        }

        public Dictionary<Vertex, List<Edge>> createVertexToEdges(IEnumerable<Edge> edges)
        {
            Dictionary<Vertex, List<Edge>> vertexToEdges = new Dictionary<Vertex, List<Edge>>();
            foreach (Edge edge in edges)
            {
                foreach (Vertex vertex in new List<Vertex>([edge.Vertex1, edge.Vertex2]))
                {
                    if (!vertexToEdges.TryGetValue(vertex, out List<Edge>? vedges)) vertexToEdges[vertex] = vedges = [];
                    vedges.Add(edge);
                }
            }
            return vertexToEdges;
        }
        private double minBeamDist(GeoPoint from, GeoVector direction)
        {
            GeoPoint[] ints = shell.GetLineIntersection(from, direction);
            double dist = double.MaxValue;
            for (int i = 0; i < ints.Length; i++)
            {
                double pos = Geometry.LinePar(from, direction, ints[i]);
                if (pos > Precision.eps && pos < dist)
                {
                    dist = pos;
                }
            }
            if (dist == double.MaxValue) dist = 0.0; // means no outside intersection
            return dist;
        }
        private Face MakeBigFace(ISurface surface)
        {   // extent the bounds of the surface at least double the area when possible and make a face to split something else with
            BoundingRect br = surface.Domain;
            double left = br.Left;
            double right = br.Right;
            if (surface.IsUPeriodic)
            {
                left = (br.Left + br.Right) / 2 - surface.UPeriod * 0.49;
                right = (br.Left + br.Right) / 2 + surface.UPeriod * 0.49;
            }
            else
            {
                left = br.Left - br.Width;
                right = br.Right + br.Width;
            }
            double bottom = br.Bottom;
            double top = br.Top;
            if (surface.IsVPeriodic)
            {
                bottom = (br.Bottom + br.Top) / 2 - surface.VPeriod * 0.49;
                top = (br.Bottom + br.Top) / 2 + surface.VPeriod * 0.49;
            }
            else
            {
                bottom = br.Bottom - br.Height;
                top = br.Top + br.Height;
            }
            return Face.MakeFace(surface, new BoundingRect(left, bottom, right, top));

        }
        /// <summary>
        /// The blend ends at <paramref name="vtx"/>. The cutter of <paramref name="edge"/> has a planar end face there,
        /// perpendicular to the edge. It is trimmed so that it ends where the faces of the edge end:
        /// <list type="bullet">
        /// <item>when there is a single ending face (the normal dead end), it is trimmed by this face (extended)</item>
        /// <item>when there are several ending faces, it is trimmed by the plane spanned by the two edges which continue the
        /// two faces of the edge at the vertex: the sides of the cutter then end exactly where these faces end.
        /// Extending all the ending faces would give a roof, which leaves parts of the faces of the edge uncovered,
        /// and an ending face may not be extendable at all (a cone beyond its apex)</item>
        /// </list>
        /// Ending faces which touch the vertex only with a singular point (e.g. the apex of a cone) are ignored.
        /// Where the trimming surface leaves a gap between itself and the end face, the cutter has to reach further: a
        /// cutter which was rebuilt longer (see <see cref="ExtendCuttersAtDeadEnds"/>) is only trimmed, otherwise the end
        /// face is extruded and the extrusion is trimmed as well. The extrusion is only correct, when the faces of the edge
        /// are planes: its sides are straight and leave curved faces tangentially.
        /// </summary>
        protected HashSet<Shell>? createDeadEndExtension(Vertex vtx, Edge edge, double length)
        {   // rounding ends here at vertex vtx. vtx and edge is on the shell to be rounded
            if (edgeToCutter == null || !edgeToCutter.TryGetValue(edge, out Shell? cutter)) return null; // no cutter for this edge
            Face? endFace = EndFaceAt(cutter, vtx);
            if (endFace == null) return [cutter]; // should not happen
            Edge freeEdge = endFace.AllEdges
                .Where(e => !Precision.IsEqual(e.Vertex1.Position, vtx.Position) && !Precision.IsEqual(e.Vertex2.Position, vtx.Position))
                .MinBy(e => -new Angle(e.Curve3D.StartPoint-vtx.Position, e.Curve3D.EndPoint-vtx.Position).Radian);
            // the chamfer or rounding edge, the one with the widest opening angle to the vertex (and not coinciding with the vertex)
            if (freeEdge == null) return [cutter]; // should not happen
            List<ISurface>? trimBy = DeadEndTrimSurfaces(vtx, edge, endFace);
            if (trimBy == null || trimBy.Count == 0) return [cutter]; // end straight with the end face of the cutter
            // beamDirection: the direction where the fillet is pointing to
            GeoVector beamDirection = EndFaceNormal(endFace, vtx);
            bool extended = endFace.UserData.Contains("CADability.Cutter.Extended"); // the cutter already reaches beyond the vertex
            Shell? extension = extended ? null : (Make3D.Extrude(endFace.Clone(), 3 * length * beamDirection, null) as Solid)?.Shells[0];
            bool useExtension = false;
            // extension of the cutter, maybe we need part of it
            foreach (ISurface surface in trimBy)
            {   // with this surface we try to trim the cutter or the extension
                GeoPoint2D ip1 = surface.PositionOf(vtx.Position); // vtx is on surface
                if (surface.GetNormal(ip1) * beamDirection < 0) surface.ReverseOrientation(); // below is good, above is bad
                GeoPoint2D ip2 = surface.GetLineIntersection(freeEdge.Curve3D.StartPoint, beamDirection).MinByWithDefault(GeoPoint2D.Invalid, uv => surface.PointAt(uv) | vtx.Position);
                GeoPoint2D ip3 = surface.GetLineIntersection(freeEdge.Curve3D.EndPoint, beamDirection).MinByWithDefault(GeoPoint2D.Invalid, uv => surface.PointAt(uv) | vtx.Position);
                if (ip2.IsValid && ip3.IsValid)
                {
                    surface.SetDomainTo(vtx.Position, surface.PointAt(ip2), surface.PointAt(ip3));
                    Face splitWith = MakeBigFace(surface);
                    (Shell[] upperPart, Shell[] lowerPart) = BooleanOperation.SplitByFace(cutter, splitWith);
                    if (upperPart.Length > 0 && lowerPart.Length > 0)
                    {   // the ending face did split the cutter
                        cutter.UserData.Add("CADability.ReplaceShellBy", lowerPart[0]);
                        cutter = lowerPart[0];
                        edgeToCutter[edge] = cutter; // overwrite existing
                    }
                    if (extension != null)
                    {
                        (upperPart, lowerPart) = BooleanOperation.SplitByFace(extension, splitWith);
                        if (upperPart.Length > 0 && lowerPart.Length > 0)
                        {   // the ending face did split the extension
                            extension = lowerPart[0];
                            useExtension = true;
                        }
                    }
                }
            }
            if (extended) AnnotateCutter(cutter, edge); // the trimmed cutter is a new shell, its edges have no hints yet
            if (useExtension && extension != null) return [cutter, extension];
            else return [cutter];
        }

        /// <summary>
        /// Stores the hints for the <see cref="BooleanOperation"/> in the user data of <paramref name="cutter"/>, which
        /// blends <paramref name="edge"/>: the edges between the blend face and the parts of the cutter on the two faces
        /// of the edge lie in these faces (a tangential intersection, which is hard to calculate), the edges between the
        /// blend face and an end face end in them.
        /// </summary>
        private static void AnnotateCutter(Shell cutter, Edge edge)
        {
            Face? sweptFace = cutter.Faces.FirstOrDefault(f => f.UserData.Contains("CADability.Cutter.SweptFace"));
            if (sweptFace == null || edge.SecondaryFace == null) return;
            Dictionary<Edge, (Face face, bool forward)> edgeLiesInFace = [];
            Dictionary<Edge, HashSet<Face>> edgeEndsInFace = [];
            foreach (Edge e in sweptFace.AllEdges)
            {
                if (e.Curve3D == null) continue;
                Face other = e.OtherFace(sweptFace);
                if (other == null) continue;
                if (other.UserData.Contains("CADability.Cutter.EndFace"))
                {
                    edgeEndsInFace[e] = [edge.PrimaryFace, edge.SecondaryFace];
                    continue;
                }
                GeoPoint m = e.Curve3D.PointAt(0.5);
                foreach (Face face in new[] { edge.PrimaryFace, edge.SecondaryFace })
                {   // the part of the cutter on this face: same surface (the normals may be opposite, depending on convexity)
                    if (face.Surface.GetDistance(m) < 10 * Precision.eps && Math.Abs(NormalAt(face, m) * NormalAt(other, m)) > 1 - 1e-6)
                    {
                        edgeLiesInFace[e] = (face, e.Forward(other));
                        break;
                    }
                }
            }
            cutter.UserData.Add("CADability.Cutter.EdgeLiesInFace", edgeLiesInFace);
            cutter.UserData.Add("CADability.Cutter.EdgeEndsInFace", edgeEndsInFace);
        }

        /// <summary>
        /// The end face of <paramref name="cutter"/> at <paramref name="vtx"/>: the one with a vertex closest to it. (The
        /// distance to the plane of the end face is no good criterion: both end planes of a half circle edge coincide.)
        /// </summary>
        private static Face? EndFaceAt(Shell cutter, Vertex vtx)
        {
            return cutter.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace"))
                .MinBy(f => f.Vertices.Min(v => v.Position | vtx.Position));
        }

        /// <summary>
        /// The normal of the planar end face, pointing away from the cutter, i.e. along the edge beyond the vertex.
        /// </summary>
        private static GeoVector EndFaceNormal(Face endFace, Vertex vtx)
        {
            return endFace.Surface.GetNormal(endFace.Surface.PositionOf(vtx.Position)).Normalized;
        }

        /// <summary>
        /// The surfaces, with which the cutter of <paramref name="edge"/> is trimmed at the dead end <paramref name="vtx"/>
        /// (see <see cref="createDeadEndExtension"/>): the single ending face or the plane of the two continuing edges.
        /// An empty list, if the cutter ends straight with its end face. The surfaces are clones, they may be modified.
        /// </summary>
        private List<ISurface>? DeadEndTrimSurfaces(Vertex vtx, Edge edge, Face endFace)
        {
            // the edges at the vertex: a vertex may still reference edges of the shells it was made from (e.g. by a boolean operation)
            HashSet<Edge> shellEdges = new HashSet<Edge>(shell.Edges);
            List<Edge> vertexEdges = vtx.Edges.Where(e => shellEdges.Contains(e)).Distinct().ToList();
            HashSet<Face> endingFaces = []; // faces on the shell to be rounded where the edge ends
            // this is only one face in most cases.
            foreach (Edge edg in vertexEdges)
            {
                endingFaces.Add(edg.PrimaryFace);
                if (edg.SecondaryFace != null) endingFaces.Add(edg.SecondaryFace);
            }
            endingFaces.Remove(edge.PrimaryFace);
            endingFaces.Remove(edge.SecondaryFace);
            endingFaces.RemoveWhere(f => IsSingularAt(f, vtx));
            List<ISurface> trimBy = [];
            if (endingFaces.Count == 1) trimBy.Add(endingFaces.First().Surface.Clone());
            else if (endingFaces.Count > 1)
            {   // the plane spanned by the edges, which continue the two faces of the edge at the vertex
                Edge? side1 = vertexEdges.Where(e => e != edge && (e.PrimaryFace == edge.PrimaryFace || e.SecondaryFace == edge.PrimaryFace)).TheOnlyOrDefault();
                Edge? side2 = vertexEdges.Where(e => e != edge && (e.PrimaryFace == edge.SecondaryFace || e.SecondaryFace == edge.SecondaryFace)).TheOnlyOrDefault();
                if (side1 == null || side2 == null) return trimBy; // end straight with the end face of the cutter
                GeoVector normal = DirectionFrom(side1, vtx) ^ DirectionFrom(side2, vtx);
                if (normal.Length < Precision.eps) return trimBy; // degenerate, end straight
                if (endFace.Surface is PlaneSurface endPlane && Precision.SameDirection(normal, endPlane.Normal, false)
                    && Math.Abs(endPlane.GetDistance(vtx.Position)) < Precision.eps)
                    return trimBy; // the cutter already ends in this plane
                trimBy.Add(new PlaneSurface(new Plane(vtx.Position, normal)));
            }
            return trimBy;
        }

        /// <summary>
        /// Rebuilds the cutter of <paramref name="edge"/> so that it reaches the provided distance beyond the vertices of
        /// <paramref name="extensions"/> (along the edge). The end faces of the extended ends must be marked with the user
        /// data "CADability.Cutter.Extended". Returns null, if this kind of blend cannot do it: the dead ends are then
        /// filled with an extruded end face.
        /// </summary>
        protected virtual Shell? MakeExtendedCutter(Edge edge, Dictionary<Vertex, double> extensions)
        {
            return null;
        }

        /// <summary>
        /// At the dead ends of <paramref name="vertexToEdges"/> (vertices with only one blended edge) the surface which
        /// trims the cutter (see <see cref="createDeadEndExtension"/>) is usually not the end plane of the cutter, there
        /// is a gap between them on one side. The cutters are rebuilt (<see cref="MakeExtendedCutter"/>) so that they reach
        /// beyond this surface, then trimming them closes the gap with the real faces of the blend.
        /// </summary>
        protected void ExtendCuttersAtDeadEnds(IEnumerable<KeyValuePair<Vertex, List<Edge>>> vertexToEdges)
        {
            if (edgeToCutter == null) return;
            Dictionary<Edge, Dictionary<Vertex, double>> extensions = [];
            foreach (KeyValuePair<Vertex, List<Edge>> ve in vertexToEdges)
            {
                if (ve.Value.Count != 1) continue;
                Edge edge = ve.Value[0];
                Shell? cutter = CutterOf(edge);
                if (cutter == null) continue;
                Face? endFace = EndFaceAt(cutter, ve.Key);
                if (endFace == null) continue;
                List<ISurface>? trimBy = DeadEndTrimSurfaces(ve.Key, edge, endFace);
                if (trimBy == null || trimBy.Count == 0) continue;
                double needed = GapToTrimSurfaces(endFace, ve.Key, trimBy);
                if (needed <= 0.0) continue; // the cutter already reaches the trimming surface everywhere
                if (!extensions.TryGetValue(edge, out Dictionary<Vertex, double>? ext)) extensions[edge] = ext = [];
                ext[ve.Key] = needed;
            }
            foreach (KeyValuePair<Edge, Dictionary<Vertex, double>> kv in extensions)
            {
                Shell? extended = MakeExtendedCutter(kv.Key, kv.Value);
                if (extended != null) edgeToCutter[kv.Key] = extended;
            }
        }

        /// <summary>
        /// How far the cutter must be extended beyond its end face at <paramref name="vtx"/> to reach the trimming surfaces
        /// everywhere: the largest distance along the end face normal from the boundary of the end face to the trimming
        /// surfaces, plus a margin. 0, if the end face is beyond the trimming surfaces everywhere.
        /// </summary>
        private static double GapToTrimSurfaces(Face endFace, Vertex vtx, List<ISurface> trimBy)
        {
            GeoVector beam = EndFaceNormal(endFace, vtx);
            List<GeoPoint> points = endFace.Vertices.Select(v => v.Position).ToList();
            foreach (Edge e in endFace.AllEdges)
            {
                if (e.Curve3D == null) continue;
                for (int i = 1; i < 4; i++) points.Add(e.Curve3D.PointAt(i / 4.0));
            }
            double size = points.Max(p => p | vtx.Position);
            double needed = 0.0;
            foreach (GeoPoint p in points)
            {
                foreach (ISurface surface in trimBy)
                {   // the intersection of the beam through p with the surface, the one closest to p
                    GeoPoint2D[] ips = surface.GetLineIntersection(p, beam);
                    if (ips.Length == 0) continue;
                    double t = ips.Select(uv => (surface.PointAt(uv) - p) * beam).MinBy(d => Math.Abs(d));
                    needed = Math.Max(needed, t);
                }
            }
            if (needed < Precision.eps) return 0.0;
            return needed + 0.25 * size; // a margin, so that the trimming surface cuts the cutter completely
        }

        /// <summary>
        /// True, if <paramref name="vtx"/> is a singular point of the surface of <paramref name="face"/> (e.g. the apex of a cone):
        /// the face touches the vertex in a pole, and its surface cannot be extended beyond it in a meaningful way.
        /// </summary>
        private static bool IsSingularAt(Face face, Vertex vtx)
        {
            if (face.AllEdges.Any(e => e.Curve3D == null && Precision.IsEqual(e.Vertex1.Position, vtx.Position))) return true; // a pole edge
            GeoPoint2D uv = face.Surface.PositionOf(vtx.Position);
            if (face.Surface.GetUSingularities().Any(u => Math.Abs(u - uv.x) < 1e-6)) return true;
            if (face.Surface.GetVSingularities().Any(v => Math.Abs(v - uv.y) < 1e-6)) return true;
            return false;
        }

        /// <summary>
        /// The direction of <paramref name="edge"/> at <paramref name="vtx"/>, pointing away from the vertex.
        /// </summary>
        private static GeoVector DirectionFrom(Edge edge, Vertex vtx)
        {
            if (edge.Vertex1 == vtx) return edge.Curve3D.StartDirection.Normalized;
            else return -edge.Curve3D.EndDirection.Normalized;
        }
        protected HashSet<Shell>? createExtensionTwoEdges(Vertex vtx, Edge edge1, Edge edge2, double length)
        {
            Shell? fillet1 = CutterOf(edge1);
            Shell? fillet2 = CutterOf(edge2);
            if (fillet1 == null || fillet2 == null) return null;
            Face? commonFace = Edge.CommonFace(edge1, edge2);
            Edge? thirdEdge = vtx.AllEdges.Except([edge1, edge2]).TheOnlyOrDefault();
            // in most cases we have a vertex with three faces meeting, so one common face and one third edge
            // if there are more than three faces meeting at the vertex, we cannot handle this currently
            if (commonFace == null)
            {
                GeoVector dir1, dir2;
                if (edge1.Vertex1 == vtx) dir1 = edge1.Curve3D.StartDirection;
                else dir1 = edge1.Curve3D.EndDirection;
                if (edge2.Vertex1 == vtx) dir2 = edge2.Curve3D.StartDirection;
                else dir2 = edge2.Curve3D.EndDirection;
                if (Precision.SameDirection(dir1, dir2, false)) return [fillet1, fillet2]; // tangential connection, we need the two fillets without any connection patch in between
                return null; // there must be a common face other cases are not implemented yet 
            }
            if (commonFace == null) return null; // there must be a common face
            if (edge2.EndVertex(commonFace) == edge1.StartVertex(commonFace)) (edge1, edge2) = (edge2, edge1); // tm make sure, the edges are in the order of the outline
            System.Diagnostics.Debug.Assert(edge1.EndVertex(commonFace) == edge2.StartVertex(commonFace));

            Face? endFace1 = fillet1.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            Face? endFace2 = fillet2.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            if (endFace1 == null || endFace2 == null) return null;
            GeoVector n1 = (endFace1.Surface as PlaneSurface)!.Normal.Normalized; // endfaces are always PlaneSurfaces
            GeoVector n2 = (endFace2.Surface as PlaneSurface)!.Normal.Normalized;

            SweepAngle sw = new SweepAngle(edge1.Curve2D(commonFace).EndDirection, edge2.Curve2D(commonFace).StartDirection);
            if (Math.Abs(sw) < 1e-5)
            {   // tangential connection, we need the two fillets without any connection patch in between
                return [fillet1, fillet2];
            }
            if (sw > 0)
            {
                // convex connection, we have to extent the fillets at this point
                Shell? extension1 = (Make3D.Extrude(endFace1.Clone(), length * n1, null) as Solid)?.Shells[0];
                Shell? extension2 = (Make3D.Extrude(endFace2.Clone(), length * n2, null) as Solid)?.Shells[0];
                if (extension1 != null && extension2 != null)
                {
                    extension1.CopyAttributes(edge1.PrimaryFace);
                    extension2.CopyAttributes(edge2.PrimaryFace);
                    return [fillet1, fillet2, extension1, extension2];
                }
            }
            else
            {
                Shell? extensionPatch = CreateConcavePatch(fillet1, fillet2, commonFace, vtx, edge1, edge2);
                if (extensionPatch != null)
                {
                    extensionPatch.CopyAttributes(edge1.PrimaryFace);
                    return [fillet1, fillet2, extensionPatch];
                }
            }
            return [fillet1, fillet2];

        }
        /// <summary>
        /// The cutter of this edge, null if there is none (creating it may have failed).
        /// </summary>
        protected Shell? CutterOf(Edge edge)
        {
            if (edgeToCutter != null && edgeToCutter.TryGetValue(edge, out Shell? cutter)) return cutter;
            return null;
        }

        protected virtual Shell? CreateConcavePatch(Shell chamfer1, Shell chamfer2, Face commonFace, Vertex vtx, Edge edge1, Edge edge2)
        {
            Face? endFace1 = chamfer1.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            Face? endFace2 = chamfer2.Faces.Where(f => f.UserData.Contains("CADability.Cutter.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            if (endFace1 == null || endFace2 == null) return null;
            List<(Vertex v1, Vertex v2)> cv = ConnectedVertices(endFace1.Vertices, endFace2.Vertices);
            if (cv.Count == 2)
            {
                Vertex? v3Fillet1 = endFace1.Vertices.Except([cv[0].v1, cv[1].v1]).TheOnlyOrDefault();
                Vertex? v3Fillet2 = endFace2.Vertices.Except([cv[0].v2, cv[1].v2]).TheOnlyOrDefault();
                if (v3Fillet1 != null && v3Fillet2 != null)
                {
                    Axis axis = new Axis(cv[0].v1.Position, cv[1].v1.Position);
                    Plane pln = new Plane(axis.Location, axis.Direction);
                    SweepAngle sw = new SweepAngle(pln.Project(v3Fillet1.Position) - GeoPoint2D.Origin, pln.Project(v3Fillet2.Position) - GeoPoint2D.Origin);
                    IGeoObject sld = Make3D.Rotate(endFace1, axis, sw, 0, null);
                    if (sld is Solid s) return s.Shells[0];
                }
            }
            return null;
        }

        protected void Combine(List<HashSet<Shell>> sets)
        {
            // some cutter shells may have been splitted and we have the unsplitted part in the set, so replace it
            for (int i = 0; i < sets.Count; i++)
            {
                foreach (Shell shell in sets[i].Clone())
                {
                    Shell? replaceWith = shell.UserData["CADability.ReplaceShellBy"] as Shell;
                    if (replaceWith != null)
                    {
                        sets[i].Remove(shell);
                        sets[i].Add(replaceWith);
                    }
                }

            }
            bool mergedSomething;
            do
            {
                mergedSomething = false;

                for (int i = 0; i < sets.Count; i++)
                {
                    for (int j = i + 1; j < sets.Count; j++)
                    {
                        if (sets[i].Overlaps(sets[j]))
                        {
                            sets[i].UnionWith(sets[j]);
                            sets.RemoveAt(j);
                            mergedSomething = true;
                            break;
                        }
                    }
                    if (mergedSomething) break;
                }
            }
            while (mergedSomething);
        }

        /// <summary>
        /// Trims <paramref name="curve"/> to the part between <paramref name="startPoint"/> and <paramref name="endPoint"/>
        /// which contains <paramref name="innerPoint"/>. This only makes a difference for closed curves: when the seam of the
        /// closed curve lies inside the wanted part, a plain trim between the two positions yields the complement.
        /// </summary>
        protected void TrimCurve(ICurve curve, GeoPoint startPoint, GeoPoint endPoint, GeoPoint innerPoint)
        {
            if (curve.IsClosed && curve is Ellipse elli)
            {
                double pos1 = curve.PositionOf(startPoint);
                double pos2 = curve.PositionOf(endPoint);
                double posInner = curve.PositionOf(innerPoint);
                if (posInner < Math.Min(pos1, pos2) || posInner > Math.Max(pos1, pos2))
                {   // the wanted part crosses the seam: move the seam into the middle of the unwanted part
                    elli.StartParameter = elli.StartParameter + (pos1 + pos2) / 2.0 * elli.SweepParameter;
                }
            }
            TrimCurve(curve, startPoint, endPoint);
        }

        protected void TrimCurve(ICurve curve, GeoPoint startPoint, GeoPoint endPoint)
        {
            double pos1 = PositionOnCurve(curve, startPoint);
            double pos2 = PositionOnCurve(curve, endPoint);
            if (pos1 > pos2)
            {
                curve.Reverse();
                pos1 = 1 - pos1;
                pos2 = 1 - pos2;
            }
            if (Math.Abs(pos1) > 1e-6 || Math.Abs(1 - pos2) > 1e-6)
            {
                curve.Trim(pos1, pos2);
            }

        }
        /// <summary>
        /// The position of <paramref name="p"/> on <paramref name="curve"/>, exactly 0 or 1 when it is one of its end
        /// points. The end points of the curves which are trimmed here are mostly already the wanted ones, and they are
        /// exact, while <see cref="ICurve.PositionOf(GeoPoint)"/> of an approximated curve may be off by a little,
        /// and trimming there would replace an exact end point by an approximated one. Not for a closed curve, whose
        /// start and end point coincide.
        /// </summary>
        private static double PositionOnCurve(ICurve curve, GeoPoint p)
        {
            if (curve.IsClosed) return curve.PositionOf(p);
            if (Precision.IsEqual(p, curve.StartPoint)) return 0.0;
            if (Precision.IsEqual(p, curve.EndPoint)) return 1.0;
            return curve.PositionOf(p);
        }
        public static List<(Vertex, Vertex)> ConnectedVertices(IEnumerable<Vertex> v1, IEnumerable<Vertex> v2)
        {
            // Result list containing matching vertex pairs
            var result = new List<(Vertex, Vertex)>();

            // Brute-force comparison of all pairs
            foreach (var a in v1)
            {
                foreach (var b in v2)
                {
                    // Check if positions are equal using given precision logic
                    if (Precision.IsEqual(a.Position, b.Position))
                    {
                        result.Add((a, b));
                    }
                }
            }

            return result;
        }

        protected void AppendLookup(Dictionary<Face, Face> start, Dictionary<Face, Face> append)
        {
            foreach (var kv in start.ToList()) // ToList(), weil wir d1 verändern
            {
                if (append.TryGetValue(kv.Value, out var a))
                {
                    start[kv.Key] = a;
                }
                else
                {
                    // leave unchanged
                }
            }
        }
        protected void AppendLookup(Dictionary<Face, Face> start, Dictionary<Face, List<Face>> append)
        {
            foreach (var kv in start.ToList()) // ToList(), weil wir d1 verändern
            {
                if (append.TryGetValue(kv.Value, out var a))
                {   // in most cases, there is only one face in the list: the face, which has been trimmed
                    // sometimes faces have been split into two or more faces. Then we don't have a good solution anyhow
                    start[kv.Key] = a.First();
                }
                else
                {
                    // leave unchanged
                }
            }
        }
        protected void Lookup(Dictionary<Edge, (Face face, bool forward)> edgeToFace, Dictionary<Face, Face> lookup)
        {
            foreach (var kv in edgeToFace.ToList())
            {
                if (lookup.TryGetValue(kv.Value.face, out Face? lookedup)) edgeToFace[kv.Key] = (lookedup, kv.Value.forward);
            }
        }
        protected void Lookup(Dictionary<Edge, HashSet<Face>> edgeToFaces, Dictionary<Face, Face> lookup)
        {
            foreach (var kv in edgeToFaces.ToList())
            {
                HashSet<Face> faces = [];
                foreach (var item in kv.Value)
                {
                    if (lookup.TryGetValue(item, out Face? found)) faces.Add(found);
                }
                edgeToFaces[kv.Key] = faces;
            }
        }
    }
}
