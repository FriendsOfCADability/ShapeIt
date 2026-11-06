using CADability.Attribute;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.Substitutes;
using MathNet.Numerics;
using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using Point = CADability.GeoObject.Point;


namespace CADability
{
    /* This is the attempt to simplify the implementation of BRepIntersection/BRepOperation
     * There were too many exceptions and special cases in BRepOperation, especially concerning the overlapping faces, both
     * same oriantation and opposite orientation. I think, we don't need to consider overlapping faces if we consequently intersect faces
     * which have edges on faces of the other shell. 
     * 
     * - put all faces (of both shells) in a single octtree
     * - use the octtree nodes to find pairs of faces (of different shells) which intersect each other and compute the intersection edges
     *      - check whether edges of one face intersect with the other face
     *      - do not intersect faces with same geometry (eliminates overlapping intersections)
     *      - when there are no egde/face intersections, check whether there is a closed intersection loop of the two faces (with irregular surfaces liek nurbs there might be more than one)
     *      - calculating these intersections generates new vertices on existing edges, which need to be associated to the edges
     *      - all vertices are also kept in an octtree, so when we need a new vertex for intersection, we look in the octtree, whether this vertex already exists
     *        (sometimes endpoints of edges are also intersection points)
     *      - keep the intersection edges in faceToIntersectionEdges
     *      - orient the intersectin edges on both faces
     * - use the new intersection vertices on the edges of the faces to split these edges
     * - use faceToIntersectionEdges to trimm the faces
     *      - the edges of a face to trimm (both outline and holes) plus the intersection edges build a graph in the 2d space of the face
     *      - in this graph identical but opposite oriented edges eliminate each other
     *      - two identical and same oriented edges are reduced to a single edge
     *      - find loops of these edges (identical opposite faces eliminate all edges, these edges are seeds for the connection process)
     *      - organize the loops in outlines (ccw) and holes (cw - clockwise), each hole belongs to a  dedicated outline - there might be outlines in holes of outlines
     *        all intersection edges must be used by the loops. original face edges may be omitted when shortened by intersection edges
     * - collect faces for the resulting shell:
     *      - all faces of the two shells build the pool of faces exept those faces which have been trimmed
     *      - start with trimmed faces (or the edges of empty trimmed faces) and find connecting faces from the pool of faces
     *      - go on connecting until all edges are connected (not all edges of the pool will be used)
     *      - all faces of the connecting process build a shell with no open edges
     *      - the shell may have inner holes, but it is a single shell
    */

    internal class DoubleVertexKey : IComparable<DoubleVertexKey>
    {
        public Vertex vertex1, vertex2;
        public DoubleVertexKey(Vertex f1, Vertex f2)
        {
            if (f1.GetHashCode() < f2.GetHashCode())
            {
                vertex1 = f1;
                vertex2 = f2;
            }
            else
            {
                vertex1 = f2;
                vertex2 = f1;
            }
        }
        public override int GetHashCode()
        {
            return vertex1.GetHashCode() + vertex2.GetHashCode();
        }
        public override bool Equals(object obj)
        {
            DoubleVertexKey other = obj as DoubleVertexKey;
            if (other == null) return false;
            return this.vertex1.GetHashCode() == other.vertex1.GetHashCode() && this.vertex2.GetHashCode() == other.vertex2.GetHashCode();
        }
        #region IComparable<DoubleVertexKey> Members
        int IComparable<DoubleVertexKey>.CompareTo(DoubleVertexKey other)
        {
            int res = vertex1.GetHashCode().CompareTo(other.vertex1.GetHashCode());
            if (res == 0) res = vertex2.GetHashCode().CompareTo(other.vertex2.GetHashCode());
            return res;
        }
        #endregion
    }

    /// <summary>
    /// Boolean operations (union, intersection, difference) of two shells, splitting a shell by a plane or a face,
    /// and connecting a set of faces (see <see cref="SetFaces"/>). See the description at the top of this file.
    /// </summary>
    public class BooleanOperation
    {
        private double precision; // points closer than this are considered identical, set in Execute according to the size of the shells
        private Shell shell1; // the two shells, which are intersected
        private Shell shell2; // either shell1 and shell2 are set or multipleFaces is set
        private double triangulationPrecision = 0.0; // precision for the volume calculations in Result, according to the size of the shells
        private List<Face> multipleFaces; // the faces, which are intersected.
        private Dictionary<Edge, Edge> originalToClonedEdges; // points from the original edges to the clones we are working on
        private Dictionary<Vertex, Vertex> originalToClonedVertices; // points from the original vertices to the clones we are working on
        private Dictionary<Face, Face> originalToClonedFaces; // points from the original faces to the clones we are working on
        private Dictionary<Face, List<Face>> splittedFaces; // Cloned face to splitted face

        private bool allowOpenEdges; // see AllowOpenEdges
        private bool dontReturnShell2 = false; // see DontReturnShell2
        private bool dontCombineConnectedFaces = false; // see DontCombineConnectedFaces
        private bool shellsAreUnchanged; // see Unchanged
        private List<Face> unusedFaces; // faces, which ware not used in the multipleFaces mode

        // spatial search structures, built in Execute: faces to find candidate pairs for intersection, vertices to find or
        // reuse a vertex at a given position (CreateOrFindVertex), edges to find edges passing through a new vertex
        private OctTree<Face> facesOctTree;
        private OctTree<Vertex> verticesOctTree;
        private OctTree<Edge> edgesOctTree;

        Dictionary<Face, Dictionary<Face, ModOp2D>> faceToOverlappingFaces; // all faces which overlap with this face
        Dictionary<Face, Dictionary<Face, ModOp2D>> faceToOppositeFaces; // all faces which are opposite oriented to this face and the ModOp2D to transform 2d curves from one face to the other
        Dictionary<(Edge edge, Face face), (List<Vertex> vertices, bool curveIsInSurface)> FaceEdgeIntersections; // to avoid duplicate calls to GetFaceEdgeIntersection
        // the raw intersections of edges with faces, calculated in advance while CreateFaceIntersections runs, see PrecalculateEdgeIntersections
        Dictionary<(Edge edge, Face face), (bool curveIsOnSurface, GeoPoint[] ips, GeoPoint2D[] uvOnFaces, double[] uOnCurve3Ds)> precalculatedEdgeIntersections;

        HashSet<Face> commonOverlappingFaces; // Faces, which have been newly created as common parts of overlapping faces
        Dictionary<Face, HashSet<Edge>> faceToIntersectionEdges; // faces of both shells with their intersection edges
        HashSet<Face> totalyCoveredByOppositeFace = new HashSet<Face>(); // faces which are totally covered by an opposite oriented face
        Dictionary<Edge, HashSet<Vertex>> edgesToSplit; // original edges and the vertices where they must be split, see AddEdgeSplitVertex and SplitEdges
        Dictionary<Edge, (Face face, bool forward)> edgeLiesInFace; // see EdgeLiesInFace, keys and values are the clones
        Dictionary<Edge, HashSet<Face>> edgeEndsInFace; // see EdgeEndsInFace, keys and values are the clones
        bool shell1IsClosed = true, shell2IsClosed = true; // set to false, if a shell is not required to be closed
        /// <summary>
        /// Create the new intersection edges and collect them in faceToIntersectionEdges
        /// </summary>
        private void CreateFaceIntersections()
        {
            // 1. Collect all face pairs which share a common leaf in the octtree
            var allPairs = facesOctTree.Leaves
                .SelectMany(leaf =>
                {
                    if (multipleFaces != null)
                    {
                        Face[] inLeaf = leaf.list.OrderBy(fc => fc.GetHashCode()).ToArray();
                        return Enumerable.Range(0, inLeaf.Length)
                            .SelectMany(i => Enumerable.Range(i + 1, inLeaf.Length - i - 1)
                                .Select(j => (fc1: inLeaf[i], fc2: inLeaf[j])));
                    }
                    else
                    {
                        var inshell1 = leaf.list.Where(fc => fc.Owner == shell1);
                        var inshell2 = leaf.list.Where(fc => fc.Owner == shell2);
                        return inshell1.SelectMany(fc1 => inshell2.Select(fc2 => (fc1, fc2)));
                    }
                })
                .Distinct()
                .ToList();
            // 2. The intersections of the edges with the faces are the most expensive part of AddFaceFaceIntersection and do not
            // depend on each other: calculate them in advance, in parallel in the release version
            precalculatedEdgeIntersections = PrecalculateEdgeIntersections(allPairs);
            // 3. Iterate over all pairs and calculate the intersection edges. This creates vertices and combines those which are close
            // to each other, so the result depends on the order: it must stay serial
            try
            {
                foreach (var (f1, f2) in allPairs) AddFaceFaceIntersection(f1, f2);
            }
            finally
            {
                precalculatedEdgeIntersections = null;
            }
        }

        /// <summary>
        /// Whether <see cref="PrecalculateEdgeIntersections"/> uses more than one thread. Off in the debug version, so that a debug
        /// session is reproducible. The result must not depend on it: a test switches it off in the release version and compares.
        /// </summary>
#if DEBUG
        internal static bool PrecalculateInParallel = false;
#else
        internal static bool PrecalculateInParallel = true;
#endif
        /// <summary>
        /// Calculates the intersections of edges with faces which <see cref="AddFaceFaceIntersection"/> is going to need, in advance:
        /// in parallel in the release version, one after the other in the debug version, so that a debug session stays reproducible.
        /// <see cref="GetFaceEdgeIntersection"/> takes the results from here and calculates what is not found here itself.
        /// <para>
        /// AddFaceFaceIntersection itself cannot run in parallel. It creates vertices and edges, whose hash codes come from counters,
        /// and it combines vertices which are close to each other, so its result depends on the order of the pairs. And the geometry
        /// is not thread safe: the hulls of surfaces and curves refine themselves while they are used (ParallelepipedHull.SplitCube),
        /// and many caches are filled on demand without a lock. What can run in parallel is the raw intersection of a curve with a
        /// surface (<see cref="Face.RawEdgeIntersection"/>), which depends on nothing but the two. So each pair of a face and an edge
        /// is a task of its own, with a private copy of the surface and a private copy of the curve. The copies are made here,
        /// serially, because cloning takes ids from counters and reads the caches of the originals.
        /// </para>
        /// <para>
        /// The copies of a task are used and refined by this task only, so its result does not depend on the other tasks or on
        /// their order: the release version computes exactly what the debug version computes. It is also the reason, why
        /// <see cref="GetFaceEdgeIntersection"/> takes <see cref="ISurface.IsCurveOnSurface"/> from here instead of asking the
        /// original surface: the answer of the copy is the one the precalculated intersection is based on.
        /// </para>
        /// </summary>
        private Dictionary<(Edge edge, Face face), (bool curveIsOnSurface, GeoPoint[] ips, GeoPoint2D[] uvOnFaces, double[] uOnCurve3Ds)> PrecalculateEdgeIntersections(List<(Face fc1, Face fc2)> pairs)
        {
            // the pairs of a face and an edge AddFaceFaceIntersection is going to need, in a fixed order (the tasks are made in this order)
            List<Face> faces = new List<Face>();
            Dictionary<Face, List<Edge>> edgesOfFace = new Dictionary<Face, List<Edge>>();
            HashSet<(Edge, Face)> collected = new HashSet<(Edge, Face)>();
            foreach (var (fc1, fc2) in pairs)
            {
                foreach (var (fca, fcb) in new[] { (fc1, fc2), (fc2, fc1) })
                {
                    if (!CanBeCopiedForAnotherThread(fca.Surface)) continue;
                    foreach (Edge edge in fcb.Edges)
                    {
                        if (edge.Curve3D == null || edge.PrimaryFace == fca || edge.SecondaryFace == fca) continue;
                        // these are handled differently by AddFaceFaceIntersection
                        if (edgeLiesInFace != null && edgeLiesInFace.TryGetValue(edge, out var liesIn) && liesIn.face == fca) continue;
                        if (edgeEndsInFace != null && edgeEndsInFace.TryGetValue(edge, out var endsIn) && endsIn.Contains(fca)) continue;
                        if (!collected.Add((edge, fca))) continue;
                        if (!edgesOfFace.TryGetValue(fca, out List<Edge> edges))
                        {
                            edgesOfFace[fca] = edges = new List<Edge>();
                            faces.Add(fca);
                        }
                        edges.Add(edge);
                    }
                }
            }
            // the private copies, one task for each pair of a face and an edge
            double prec = precision == 0.0 ? Precision.eps : precision; // as in Face.IntersectAndPosition
            var tasks = new List<(Face face, Edge edge, ISurface surface, BoundingRect uvBounds, BoundingRect domain, ICurve curve)>();
            foreach (Face face in faces)
            {
                // a copy of a NurbsSurface takes over the hull of the original (see NurbsSurface.Clone), build it once here
                // instead of once in every task
                if (face.Surface is NurbsSurface nurbs) _ = nurbs.ParallelepipedHull;
                BoundingRect uvBounds = face.GetUVBounds();
                BoundingRect domain = face.Domain;
                foreach (Edge edge in edgesOfFace[face])
                {
                    ICurve curve = CopyForAnotherThread(edge.Curve3D);
                    if (curve != null) tasks.Add((face, edge, face.Surface.Clone(), uvBounds, domain, curve));
                }
            }
            var results = new (bool curveIsOnSurface, GeoPoint[] ips, GeoPoint2D[] uvOnFaces, double[] uOnCurve3Ds)?[tasks.Count];
            void calculate(int i)
            {
                var task = tasks[i];
                try
                {
                    // GetFaceEdgeIntersection treats a curve in the surface differently
                    if (task.surface.IsCurveOnSurface(task.curve)) results[i] = (true, null, null, null);
                    else
                    {
                        Face.RawEdgeIntersection(task.surface, task.uvBounds, task.domain, task.curve, prec,
                            out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve3Ds);
                        results[i] = (false, ips, uvOnFaces, uOnCurve3Ds);
                    }
                }
                catch (Exception)
                {   // left to GetFaceEdgeIntersection, which calculates it again and throws, if it must, at the right moment
                }
            }
            if (PrecalculateInParallel) System.Threading.Tasks.Parallel.For(0, tasks.Count, calculate);
            else for (int i = 0; i < tasks.Count; i++) calculate(i);
            var res = new Dictionary<(Edge edge, Face face), (bool curveIsOnSurface, GeoPoint[] ips, GeoPoint2D[] uvOnFaces, double[] uOnCurve3Ds)>();
            for (int i = 0; i < tasks.Count; i++)
            {
                if (results[i].HasValue) res[(tasks[i].edge, tasks[i].face)] = results[i].Value;
            }
            return res;
        }
        /// <summary>
        /// True for surfaces whose <see cref="ISurface.Clone"/> shares nothing with the original that changes while it is used.
        /// Exact types: a derived class may keep more.
        /// </summary>
        private static bool CanBeCopiedForAnotherThread(ISurface surface)
        {
            Type type = surface.GetType();
            return type == typeof(PlaneSurface) || type == typeof(CylindricalSurface) || type == typeof(ConicalSurface)
                || type == typeof(SphericalSurface) || type == typeof(ToroidalSurface) || type == typeof(NurbsSurface);
        }
        /// <summary>
        /// A copy of <paramref name="curve"/> which shares nothing with the original that changes while it is used, or null for
        /// the kinds of curves where this is not known to be true (e.g. curves which keep a reference to a surface).
        /// </summary>
        private static ICurve CopyForAnotherThread(ICurve curve)
        {
            Type type = curve.GetType();
            if (type == typeof(Line) || type == typeof(Ellipse) || type == typeof(BSpline)) return curve.Clone();
            if (curve is InterpolatedDualSurfaceCurve idsc && type == typeof(InterpolatedDualSurfaceCurve)
                && CanBeCopiedForAnotherThread(idsc.Surface1) && CanBeCopiedForAnotherThread(idsc.Surface2)) return idsc.CloneUnshared();
            return null;
        }

        /// <summary>
        /// Intersects the two faces (of different shells, or any two faces in the <see cref="SetFaces"/> mode):
        /// <list type="bullet">
        /// <item>Faces with the same surface are not intersected but registered in faceToOverlappingFaces or faceToOppositeFaces,
        /// only their edges are intersected with each other.</item>
        /// <item>Otherwise the intersection vertices are collected: tangential contact points of the surfaces, intersections of the edges
        /// of each face with the other face (<see cref="GetFaceEdgeIntersection"/>) and the vertices provided by <see cref="EdgeLiesInFace"/>
        /// and <see cref="EdgeEndsInFace"/>. Edges which lie in the surface of the other face yield known intersection curves.</item>
        /// <item>Without any intersection vertex, inner closed intersection curves are searched (Surfaces.IntersectInner).</item>
        /// <item>With two or more vertices, <see cref="CreateIntersectionEdges"/> creates the oriented intersection edges, first from the
        /// known curves, then for the remaining segments.</item>
        /// </list>
        /// The original edges which are hit are registered in edgesToSplit, the new edges in faceToIntersectionEdges.
        /// </summary>
        private void AddFaceFaceIntersection(Face fc1, Face fc2)
        {
            // Test for overlapping faces:
            if (fc1.Surface.SameGeometry(fc1.Domain, fc2.Surface, fc2.Domain, this.precision, out ModOp2D firstToSecond))
            {
                // These are overlapping faces, we don't intersect them
                // but we have to remember them in faceToOverlappingFaces and faceToOppositeFaces for later processing
                // depending on their orientation
                bool sameOrientation = true;
                if (firstToSecond.IsNull)
                {   // the u/v systems of the two faces are incompatible
                    // to find out, whether they are same or opposite oriented, we check a common point
                    GeoPoint commonPoint = GeoPoint.Invalid;
                    foreach (Vertex vtx in fc1.Vertices)
                    {
                        if (fc2.Contains(vtx.Position, true))
                        {
                            commonPoint = vtx.Position;
                            break;
                        }
                    }
                    if (!commonPoint.IsValid)
                    {
                        foreach (Vertex vtx in fc2.Vertices)
                        {
                            if (fc1.Contains(vtx.Position, true))
                            {
                                commonPoint = vtx.Position;
                                break;
                            }
                        }
                    }
                    if (!commonPoint.IsValid)
                    {
                        commonPoint = fc1.Surface.PointAt(fc1.Area.GetSomeInnerPoint());
                    }
                    GeoVector n1 = fc1.Surface.GetNormal(fc1.Surface.PositionOf(commonPoint));
                    GeoVector n2 = fc2.Surface.GetNormal(fc2.Surface.PositionOf(commonPoint));
                    sameOrientation = (n1 * n2) > 0;
                }
                else
                {
                    sameOrientation = firstToSecond.Determinant > 0;
                }
                if (sameOrientation)
                {
                    if (multipleFaces == null || !fc1.AllEdgesSet.Intersect(fc2.AllEdgesSet).Any())
                    {   // in case of multipleFaces, this might be splitted cylical faces
                        if (!faceToOverlappingFaces.TryGetValue(fc1, out var overlapping))
                        {
                            overlapping = new Dictionary<Face, ModOp2D>();
                            faceToOverlappingFaces[fc1] = overlapping;
                        }
                        overlapping[fc2] = firstToSecond;
                        if (!faceToOverlappingFaces.TryGetValue(fc2, out overlapping))
                        {
                            overlapping = new Dictionary<Face, ModOp2D>();
                            faceToOverlappingFaces[fc2] = overlapping;
                        }
                        overlapping[fc1] = firstToSecond.GetInverse();
                    }
                }
                else
                {
                    if (!faceToOppositeFaces.TryGetValue(fc1, out var opposite))
                    {
                        opposite = new Dictionary<Face, ModOp2D>();
                        faceToOppositeFaces[fc1] = opposite;
                    }
                    opposite[fc2] = firstToSecond;
                    if (!faceToOppositeFaces.TryGetValue(fc2, out opposite))
                    {
                        opposite = new Dictionary<Face, ModOp2D>();
                        faceToOppositeFaces[fc2] = opposite;
                    }
                    opposite[fc1] = firstToSecond.GetInverse();
                }
                // The edges of the two faces still have to be intersected with each other. Usually the neighbouring
                // faces produce the same intersection vertices anyway, but where two neighbours share the same
                // surface they do not, and the vertices would be missing (HemisphereOnCylinderThreeWays.json).
                foreach (Edge edga in fc1.Edges)
                {
                    if (edga.Curve3D == null) continue;
                    foreach (Edge edgb in fc2.Edges)
                    {
                        if (edgb.Curve3D == null) continue;
                        if (edga == edgb) continue; // same edge, no intersection
                        if (edga.Curve3D.SameGeometry(edgb.Curve3D, precision)) continue; // same curve, no intersection
                        if (edga.Curve3D.DistanceTo(edgb.Curve3D.StartPoint) < precision) continue; // the intersectionpoint already exists:
                        if (edga.Curve3D.DistanceTo(edgb.Curve3D.EndPoint) < precision) continue; // necessary for tangential intersections as in DifferenzBug10
                        if (edgb.Curve3D.DistanceTo(edga.Curve3D.StartPoint) < precision) continue;
                        if (edgb.Curve3D.DistanceTo(edga.Curve3D.EndPoint) < precision) continue;
                        Curves.Intersect(edga.Curve3D, edgb.Curve3D, out double[] par1, out double[] par2, out GeoPoint[] ip);
                        for (int i = 0; i < ip.Length; i++)
                        {
                            Vertex v = CreateOrFindVertex(ip[i]);
                            if (par1[i] > Precision.eps && par1[i] < 1 - Precision.eps && par2[i] > -Precision.eps && par2[i] < 1 + Precision.eps)
                            {
                                AddEdgeSplitVertex(edga, v);
                            }
                            if (par2[i] > Precision.eps && par2[i] < 1 - Precision.eps && par1[i] > -Precision.eps && par1[i] < 1 + Precision.eps)
                            {
                                AddEdgeSplitVertex(edgb, v);
                            }
                        }
                    }
                }
                return; // don't intersect overlapping faces
            }
            HashSet<Edge> sameEdges = FindSameEdges(fc1, fc2);

            bool tangentialIntersectionFound = false;
            HashSet<Vertex> intersectionVertices = new HashSet<Vertex>();
            List<ICurve> knownIntersectionCurves = null;
            List<bool?> knownIntersectionCurveDirections = null;
            List<Vertex> usedVerticedByKnownIntersections = [];
            HashSet<(Vertex, Vertex)> knownCreatedPairs = null;
            List<Vertex> surfaceContactVertices = [];
            if (!sameEdges.Any())
            {   // faces with common edges should not have contacts. Surfaces.TangentialContacts doesn't work with such faces

                // Tangential contacts, e.g. one cylinder inside another one, touching along a single point: the
                // surfaces meet there without crossing, so the ordinary intersection finds nothing.
                SurfaceContact[] surfaceContact = Surfaces.TangentialContacts(fc1.Surface, fc1.Domain, fc2.Surface, fc2.Domain, Precision.eps);
                for (int i = 0; i < surfaceContact.Length; i++)
                {
                    if (surfaceContact[i].Type == ContactType.Crossing)
                    {
                        Vertex v = CreateOrFindVertex(surfaceContact[i].Location);
                        //Vertex v = new Vertex(surfaceContact[i].Location);
                        v.AddPositionOnFace(fc1, surfaceContact[i].uv1);
                        v.AddPositionOnFace(fc2, surfaceContact[i].uv2);
                        intersectionVertices.Add(v);
                        surfaceContactVertices.Add(v);
                        // this touching point may or may not lie on an edge
                        foreach (Edge edge in fc1.Edges.Concat(fc2.Edges))
                        {
                            if (edge.Curve3D != null && edge.Curve3D.DistanceTo(v.Position) < 10 * Precision.eps)
                            {
                                double pos = edge.Curve3D.PositionOf(v.Position);
                                if (pos > Precision.eps && pos < 1 - Precision.eps)
                                {
                                    AddEdgeSplitVertex(edge, v);
                                }
                            }
                        }
                    }
                }
            }
            foreach (var (fca, fcb) in new[] { (fc1, fc2), (fc2, fc1) })
            {
                foreach (Edge edge in fcb.Edges)
                {
                    if (edge.Curve3D != null) // not a pole
                    {
                        bool handled = false;
                        if (edgeLiesInFace != null && edgeLiesInFace.TryGetValue(edge, out (Face fc, bool fw) f) && f.fc == fca)
                        {   // this is a tangential intersection provided by the user of this BoolenOperation
                            // the orientation of the curve is also provided (which is difficult to calculate
                            // because it is tangential)
                            // We don't have to clip the curve of this edge with the face, because the intersection vertices
                            // are calculated by other edge/face intersections
                            int inside = 0;
                            if (fca.Contains(edge.Vertex1.Position, true))
                            {
                                intersectionVertices.Add(edge.Vertex1);
                                usedVerticedByKnownIntersections.Add(edge.Vertex1);
                                ++inside;
                            }
                            if (fca.Contains(edge.Vertex2.Position, true))
                            {
                                intersectionVertices.Add(edge.Vertex2);
                                usedVerticedByKnownIntersections.Add(edge.Vertex2);
                                ++inside;
                            }
                            if (inside == 2)
                            {   // the edge is completely inside the face, if not, we need to calculate it the traditional way to get the intersection with the edge of the other face
                                if (knownIntersectionCurves == null)
                                {
                                    knownIntersectionCurves = [];
                                    knownIntersectionCurveDirections = [];
                                }
                                knownIntersectionCurves.Add(edge.Curve3D.Clone());
                                knownIntersectionCurveDirections.Add((fca == fc1) ? f.fw : !f.fw);
                                handled = true;
                            }
                        }
                        else if (edgeEndsInFace != null && edgeEndsInFace.TryGetValue(edge, out var faces) && faces.Contains(fca))
                        {   // either the startvertex or the endvertex of edge lies on fca
                            // but here we don't know the orientation of the curve
                            if (fca.Surface.GetDistance(edge.Vertex1.Position) < Precision.eps && fca.Contains(edge.Vertex1.Position, true))
                            {
                                intersectionVertices.Add(edge.Vertex1);
                                handled = true;
                            }
                            else if (fca.Surface.GetDistance(edge.Vertex2.Position) < Precision.eps && fca.Contains(edge.Vertex2.Position, true))
                            {
                                intersectionVertices.Add(edge.Vertex2);
                                handled = true;
                            }
                        }
                        if (!handled)
                        {
                            List<Vertex> vtxs = GetFaceEdgeIntersection(fca, edge, out bool curveIsInSurface).ToList();
                            if (curveIsInSurface)
                            {
                                foreach (Vertex vtx in surfaceContactVertices)
                                {
                                    if (!vtxs.Contains(vtx) && edge.Curve3D.DistanceTo(vtx.Position) < 10 * Precision.eps) vtxs.Add(vtx);
                                }
                                // when a curve is in the surface of a face, it might also be clipped by this face. We need those intersections as well.
                                // But when the curve overlaps an edge of the face, clipping in 2d yields arbitrary points along the overlapping part:
                                // then only the vertices of the face on the curve are needed, which are added below
                                bool overlapsOutline = OverlapsOutline(fca, edge);
                                ICurve2D edgeProjected = overlapsOutline ? null : fca.GetProjectedCurve(edge.Curve3D, true);
                                double[] clipped = overlapsOutline ? [] : fca.Area.Clip(edgeProjected, true);
                                for (int i = 0; i < clipped.Length; i++)
                                {
                                    if (clipped[i] > -Precision.eps && clipped[i] < 1 + Precision.eps)
                                    {
                                        GeoPoint p = fca.Surface.PointAt(edgeProjected.PointAt(clipped[i]));
                                        if (clipped[i] < Precision.eps) p = edge.Curve3D.StartPoint;
                                        else if (clipped[i] > 1 - Precision.eps) p = edge.Curve3D.EndPoint;
                                        else if (clipped[i] < 0.001)
                                        {
                                            if (fca.DistanceToEdges(p) >= fca.DistanceToEdges(edge.Curve3D.StartPoint)) p = edge.Curve3D.StartPoint;
                                        }
                                        else if (clipped[i] > 1 - 0.001)
                                        {
                                            if (fca.DistanceToEdges(p) >= fca.DistanceToEdges(edge.Curve3D.EndPoint)) p = edge.Curve3D.EndPoint;
                                        }
                                        if (edge.Curve3D.DistanceTo(p) > 10 * precision) continue;
                                        // an existing vertex close to p is the better point: where the curve touches the outline of the face
                                        // tangentially, the clip position is ill-conditioned and p may be far off the true vertex (e.g. 5e-4
                                        // for an offset of 5e-8). Comparing the distances of both points to the curve doesn't help, they are
                                        // both on the curve up to rounding errors
                                        Vertex? closestVertex = null;
                                        double closestDistance = 10 * precision;
                                        foreach (Vertex va in fca.Vertices)
                                        {
                                            double d = va.Position | p;
                                            if (d < closestDistance && edge.Curve3D.DistanceTo(va.Position) < 10 * precision)
                                            {
                                                closestVertex = va;
                                                closestDistance = d;
                                            }
                                        }
                                        if (closestVertex != null) p = closestVertex.Position;
                                        Vertex vv = CreateOrFindVertex(p);
                                        bool found = false;
                                        foreach (Edge edga in fca.Edges)
                                        {
                                            if (edga.Curve3D != null && edga.Curve3D.DistanceTo(p) < 10 * precision)
                                            {
                                                AddEdgeSplitVertex(edga, vv);
                                                found = true;
                                            }
                                        }
                                        if (found && !vtxs.Contains(vv)) vtxs.Add(vv);
                                    }
                                }
                                foreach (Vertex vtx in fca.Vertices)
                                {
                                    if (edge.Curve3D.DistanceTo(vtx.Position) < precision)
                                    {
                                        if (!vtxs.Contains(vtx)) vtxs.Add(vtx);
                                    }
                                }
                                // 3d intersections of curves is unsafe
                                //foreach (Edge edga in fca.Edges)
                                //{
                                //    if (edga.Curve3D.SameGeometry(edge.Curve3D, precision)) continue;
                                //    Curves.Intersect(edga.Curve3D, edge.Curve3D, out double[] par1, out double[] par2, out GeoPoint[] ips);
                                //    for (int i = 0; i < ips.Length; i++)
                                //    {
                                //        if (Math.Abs(par1[i]) < 0.001 && edge.Curve3D.DistanceTo(edga.Curve3D.StartPoint) < edge.Curve3D.DistanceTo(edga.Curve3D.PointAt(par1[i])))
                                //        {
                                //            par1[i] = 0.0;
                                //            ips[i] = edga.Curve3D.StartPoint;
                                //            par2[i] = edge.Curve3D.PositionOf(ips[i]);
                                //        }
                                //        if (Math.Abs(1.0-par1[i]) < 0.001 && edge.Curve3D.DistanceTo(edga.Curve3D.EndPoint) < edge.Curve3D.DistanceTo(edga.Curve3D.PointAt(par1[i])))
                                //        {
                                //            par1[i] = 1.0;
                                //            ips[i] = edga.Curve3D.EndPoint;
                                //            par2[i] = edge.Curve3D.PositionOf(ips[i]);
                                //        }
                                //        if (Math.Abs(par2[i]) < 0.001 && edga.Curve3D.DistanceTo(edge.Curve3D.StartPoint) < edga.Curve3D.DistanceTo(edge.Curve3D.PointAt(par2[i])))
                                //        {
                                //            par2[i] = 0.0;
                                //            ips[i] = edge.Curve3D.StartPoint;
                                //            par1[i] = edga.Curve3D.PositionOf(ips[i]);
                                //        }
                                //        if (Math.Abs(1.0 - par2[i]) < 0.001 && edga.Curve3D.DistanceTo(edge.Curve3D.EndPoint) < edga.Curve3D.DistanceTo(edge.Curve3D.PointAt(par2[i])))
                                //        {
                                //            par2[i] = 1.0;
                                //            ips[i] = edge.Curve3D.EndPoint;
                                //            par1[i] = edga.Curve3D.PositionOf(ips[i]);
                                //        }
                                //        if (par1[i] > -Precision.eps && par1[i] < 1 + Precision.eps && par2[i] > -Precision.eps && par2[i] < 1 + Precision.eps)
                                //        {   // only add intersection points which are on both curves
                                //            if (edge.Curve3D.DistanceTo(ips[i]) > 10 * precision || edga.Curve3D.DistanceTo(ips[i]) > 10 * precision) continue;
                                //            Vertex vv = CreateOrFindVertex(ips[i]);
                                //            if (!vtxs.Contains(vv)) vtxs.Add(vv);
                                //            if (par1[i] > Precision.eps && par1[i] < 1 - Precision.eps)
                                //            {
                                //                AddEdgeSplitVertex(edga, vv);
                                //            }
                                //        }
                                //    }
                                //}
                            }
                            vtxs.Sort((v1, v2) => edge.Curve3D.PositionOf(v1.Position).CompareTo(edge.Curve3D.PositionOf(v2.Position)));
                            if (curveIsInSurface)
                            {   // why not accumulate the vertices and create the intersection edges later?
                                List<ICurve> curveParts = new List<ICurve>();
                                var list = vtxs as IList<Vertex> ?? vtxs.ToList();
                                for (int i = 0; i < list.Count - 1; i++)
                                {
                                    var a = list[i];
                                    var b = list[i + 1];
                                    ICurve part = edge.Curve3D.Clone();
                                    part.Trim(edge.Curve3D.PositionOf(a.Position), edge.Curve3D.PositionOf(b.Position));
                                    GeoPoint pm = part.PointAt(0.5);
                                    if (fca.Contains(pm, true))
                                    {
                                        // the part lies on the outline of fca: the faces touch along an edge. Common edges are
                                        // handled separately, the generic intersection excludes them too
                                        if (fca.DistanceToEdges(pm) < 10 * precision) continue;
                                        // the surfaces touch tangentially along this curve, it is no intersection
                                        GeoVector na = fca.Surface.GetNormal(fca.Surface.PositionOf(pm)).Normalized;
                                        GeoVector nb = fcb.Surface.GetNormal(fcb.Surface.PositionOf(pm)).Normalized;
                                        if ((na ^ nb).Length < 1e-6) continue;
                                        curveParts.Add(part);
                                    }
                                }
                                if (curveParts.Count > 0 && vtxs.Count() > 1)
                                {
                                    if (knownIntersectionCurves == null)
                                    {
                                        knownIntersectionCurves = [];
                                        knownIntersectionCurveDirections = [];
                                    }
                                    for (int i = 0; i < curveParts.Count; i++)
                                    {
                                        knownIntersectionCurves.Add(curveParts[i]);
                                        knownIntersectionCurveDirections.Add(null); // orientation correct?
                                    }
                                    usedVerticedByKnownIntersections.AddRange(list);
                                }
                                foreach (Vertex vtx in vtxs) foreach (Edge e in edgesOctTree.GetObjectsFromPoint(vtx.Position))
                                {   // this intersection curve lies in the surface of a face. It might also lie on an edge of the face. In this case, we have to split the edge as well
                                    // if (e == edge) continue; why? we need to split this edge as well!
                                    if (e.Vertex1 == vtx || e.Vertex2 == vtx) continue; // the vertex is already on the edge
                                    if (e.Curve3D != null && e.Curve3D.DistanceTo(vtx.Position) < 10 * precision)
                                    {
                                        double pos = e.Curve3D.PositionOf(vtx.Position);
                                        if (pos > 0 && pos < 1) AddEdgeSplitVertex(e, vtx);
                                    }
                                }

                            }
                            intersectionVertices.UnionWith(vtxs);
                        }
                    }
                }
            }
            if (tangentialIntersectionFound) return; // if there is a tangential intersection, we don't need to calculate further intersections
            if (intersectionVertices.Count == 0)
            {
                // test for inner intersections, where no edges are involved
                if (!fc1.GetExtent(0.0).Interferes(fc2.GetExtent(0.0))) return; // bounding boxes don't interfere: no intersection


                IDualSurfaceCurve[] innerCurves = Surfaces.IntersectInner(fc1.Surface, fc1.Domain, fc2.Surface, fc2.Domain);
                if (innerCurves?.Length > 0)
                {
                    if (innerCurves.Length == 1)
                    {   // a single closed curve: we split it, because we don't like closed curves
                        ICurve[] parts = innerCurves[0].Curve3D.Split(0.5);
                        List<Vertex> vertices = new List<Vertex>();
                        for (int i = 0; i < parts.Length; i++)
                        {
                            vertices.Add(new Vertex(parts[i].StartPoint));
                        }
                        CreateIntersectionEdges(fc1, fc2, vertices.ToHashSet(), parts);
                    }
                    else
                    {   // it must be a closed loop, in most cases consisting of two curves
                        // multiple closed loops are not implemented yet
                        // the direction of these curves is arbitrary, so both endpoints have to go into the vertex list
                        HashSet<Vertex> vertices = [];
                        for (int i = 0; i < innerCurves.Length; i++)
                        {
                            vertices.Add(CreateOrFindVertex(innerCurves[i].Curve3D.StartPoint));
                            vertices.Add(CreateOrFindVertex(innerCurves[i].Curve3D.EndPoint));
                        }
                        CreateIntersectionEdges(fc1, fc2, vertices, innerCurves.Select(c => c.Curve3D).ToList());
                    }
                }
            }
            if (intersectionVertices.Count > 1)
            {
                if (knownIntersectionCurves != null)
                {   // exact intersection curves, which are (parts of) edges of one face lying in the surface of the other face
                    foreach (Vertex v in intersectionVertices.Except(usedVerticedByKnownIntersections))
                    {   // additional intersections may lie on the known intersection curves. E.g.when the curve extends beyond the boundary of the face.
                        for (int i = 0; i < knownIntersectionCurves.Count; i++)
                        {
                            double pos = knownIntersectionCurves[i].PositionOf(v.Position);
                            if (pos > Precision.eps && pos < 1 - Precision.eps)
                            {
                                if (knownIntersectionCurves[i].DistanceTo(v.Position) < Precision.eps)
                                {
                                    usedVerticedByKnownIntersections.Add(v);
                                    break;
                                }
                            }
                        }
                    }
                    // the same curve is found from both sides, when an edge of fc1 and an edge of fc2 coincide
                    List<ICurve> distinctCurves = [];
                    List<bool?> distinctDirections = [];
                    for (int i = 0; i < knownIntersectionCurves.Count; i++)
                    {
                        int found = distinctCurves.FindIndex(c => c.SameGeometry(knownIntersectionCurves[i], precision));
                        if (found < 0)
                        {
                            distinctCurves.Add(knownIntersectionCurves[i]);
                            distinctDirections.Add(knownIntersectionCurveDirections[i]);
                        }
                        else if (!distinctDirections[found].HasValue) distinctDirections[found] = knownIntersectionCurveDirections[i];
                    }
                    knownCreatedPairs = [];
                    CreateIntersectionEdges(fc1, fc2, usedVerticedByKnownIntersections.ToHashSet(), distinctCurves, distinctDirections, null, knownCreatedPairs);
                    // The vertices of the known curves are kept: a vertex may also be the end of a further intersection
                    // curve, which is not known. Removing it would lose that curve. The generic calculation below skips
                    // the segments which have already been created from the known curves.
                }
                if (intersectionVertices.Count > 1)
                {
                    if (multipleFaces != null)
                    {   // When there is an common edge of fc1 and fc2 which can be used as the intersection, we don't need to create new intersection edges
                        List<Edge> commonEdges = fc1.AllEdges.Intersect(fc2.AllEdges).ToList();
                        for (int i = 0; i < commonEdges.Count; i++)
                        {
                            if (intersectionVertices.Contains(commonEdges[i].Vertex1) && intersectionVertices.Contains(commonEdges[i].Vertex2))
                            {
                                intersectionVertices.ExceptWith(commonEdges[i].Vertices);
                            }
                        }
                        if (intersectionVertices.Count < 2) return;
                    }
                    // we ant to exclude common edges of fc1 and fc2 from the intersection edges, because they are already present in the faces
                    // a problem is the mixed state: we have both common edges and new intersection edges between the same two vertices. 
                    // so preliminary we exclude only common edges when there is nothing else
                    HashSet<Vertex> usedByCommonEdges = [];
                    foreach ((Vertex v1, Vertex v2) in intersectionVertices.Pairs())
                    {
                        var common = Vertex.ConnectingEdges(v1, v2);
                        if (fc1.Edges.Intersect(common).Any() && fc2.Edges.Intersect(common).Any())
                        {
                            usedByCommonEdges.Add(v1);
                            usedByCommonEdges.Add(v2);
                        }
                    }
                    if (usedByCommonEdges.Count < intersectionVertices.Count)
                        CreateIntersectionEdges(fc1, fc2, intersectionVertices, null, null, knownCreatedPairs);
                }
            }
        }

        /// <summary>
        /// Returns the edges of <paramref name="fc1"/> and <paramref name="fc2"/> which connect the same two vertices and have the same
        /// geometry (both edges of each such pair), i.e. where the two faces touch along a common edge.
        /// </summary>
        private HashSet<Edge> FindSameEdges(Face fc1, Face fc2)
        {
            HashSet<Vertex> commonVertices = fc1.Vertices.Intersect(fc2.Vertices).ToHashSet();
            if (commonVertices.Count < 2) return new HashSet<Edge>();
            HashSet<Edge> sameEdges = new HashSet<Edge>();
            foreach ((Vertex v1, Vertex v2) v in commonVertices.Pairs())
            {
                Edge edgeInfc1 = null, edgeInfc2 = null;
                foreach (Edge e in Vertex.ConnectingEdges(v.v1, v.v2))
                {
                    if (fc1.Edges.Contains(e))
                    {
                        edgeInfc1 = e;
                    }
                    if (fc2.Edges.Contains(e))
                    {
                        edgeInfc2 = e;
                    }
                }
                if (edgeInfc1 != null && edgeInfc2 != null && SameEdge(edgeInfc1, edgeInfc2, precision))
                {
                    sameEdges.Add(edgeInfc1);
                    sameEdges.Add(edgeInfc2);
                }
            }
            return sameEdges;
        }

        /// <summary>
        /// Creates the intersection edges of <paramref name="fc1"/> and <paramref name="fc2"/> which connect the provided intersection vertices.
        /// The intersection curves are calculated through the vertices (Surfaces.Intersect) unless they are provided. Each curve is split
        /// at the vertices on it, segments outside one of the faces are discarded. Each new edge is oriented by <see cref="GetOrientation"/>
        /// (opposite on the two faces) and added to faceToIntersectionEdges for both faces.
        /// </summary>
        /// <param name="fc1">the first face</param>
        /// <param name="fc2">the second face</param>
        /// <param name="intersectionVertices">the vertices where the intersection curves start, end or pass through</param>
        /// <param name="alreadyCalculated">intersection curves which are already known (e.g. edges lying in the other surface), null to calculate them</param>
        /// <param name="orientation">for each curve in <paramref name="alreadyCalculated"/> the known orientation on <paramref name="fc1"/>,
        /// or null, if it must be calculated</param>
        /// <param name="skipPairs">segments between these vertices have already been created and are skipped</param>
        /// <param name="createdPairs">if not null, receives the vertex pairs of the created edges</param>
        private void CreateIntersectionEdges(Face fc1, Face fc2, HashSet<Vertex> intersectionVertices, IList<ICurve> alreadyCalculated = null, IList<bool?> orientation = null,
            HashSet<(Vertex, Vertex)> skipPairs = null, HashSet<(Vertex, Vertex)> createdPairs = null)
        {
#if DEBUG
            // here you can see the two faces and the intersection vertices for this calculation
            DebuggerContainer dc0 = new DebuggerContainer();
            dc0.Add(fc1, fc1.GetHashCode());
            foreach (Edge edg in fc1.AllEdges)
            {
                dc0.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
            }
            dc0.Add(fc2, fc2.GetHashCode());
            foreach (Edge edg in fc2.AllEdges)
            {
                dc0.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
            }
            foreach (Vertex vtx in intersectionVertices)
            {
                Point pnt = Point.Construct();
                pnt.Location = vtx.Position;
                pnt.Symbol = PointSymbol.Cross;
                dc0.Add(pnt, vtx.GetHashCode());
            }
#endif
            var list = intersectionVertices.ToList();

            IEnumerable<(Vertex, Vertex)> pairs =
                from i in Enumerable.Range(0, list.Count)
                from j in Enumerable.Range(i + 1, list.Count - i - 1)
                select (list[i], list[j]);

            if (intersectionVertices.Count < 2) return;

            List<Vertex> usedVertices = new List<Vertex>(intersectionVertices.Count);
            List<GeoPoint> points = new List<GeoPoint>(intersectionVertices.Count); // usedVertices and points are synchronous arrays, so we later can find the vertex from a point (index)
            foreach (Vertex v in intersectionVertices)
            {
                usedVertices.Add(v);
                points.Add(v.Position);
            }

            ICurve[] c3ds = null; // the intersection curves in 3d
            ICurve2D[] crvsOnSurface1 = null; // the intersection curves in the 2d space of face1
            ICurve2D[] crvsOnSurface2 = null; // the intersection curves in the 2d space of face2
            double[,] params3d = null; // the parameters of the intersection points on the 3d curves [curveindex, pointindex]
            double[,] params2dFace1 = null; // the parameters of the intersection points on the 2d curves on face1 [curveindex, pointindex]
            double[,] params2dFace2 = null; // the parameters of the intersection points on the 2d curves on face2 [curveindex, pointindex]
            GeoPoint2D[] paramsuvsurf1 = null; // the uv parameters of the intersection points on surface1 [pointindex]
            GeoPoint2D[] paramsuvsurf2 = null; // the uv parameters of the intersection points on surface2 [pointindex]

            if (alreadyCalculated != null)
            {   // the intersection curves are already calculated. This is the case, when faces intersect with no egdes involved
                c3ds = new ICurve[alreadyCalculated.Count];
                crvsOnSurface1 = new ICurve2D[alreadyCalculated.Count];
                crvsOnSurface2 = new ICurve2D[alreadyCalculated.Count];
                params3d = new double[alreadyCalculated.Count, points.Count]; // the parameters on the tangential edge (often 0 and 1)
                params2dFace1 = new double[alreadyCalculated.Count, points.Count]; // the parameters on the 2d curve on face1
                params2dFace2 = new double[alreadyCalculated.Count, points.Count]; // the parameters on the 2d curve on face2
                paramsuvsurf1 = new GeoPoint2D[points.Count]; // the uv parameters on surface1
                paramsuvsurf2 = new GeoPoint2D[points.Count]; // the uv parameters on surface2
                for (int j = 0; j < points.Count; j++)
                {
                    paramsuvsurf1[j] = fc1.Surface.PositionOf(points[j]);
                    fc1.Surface.AlignIfPole(ref paramsuvsurf1[j], points[points.Count - 1 - j]); // we assume two points here, which should always be the case
                                                                                                 // if there are cases with more than two points, there are probaly several curves and we have to assiciate the correct points to the curves
                    paramsuvsurf2[j] = fc2.Surface.PositionOf(points[j]);
                    fc2.Surface.AlignIfPole(ref paramsuvsurf2[j], points[points.Count - 1 - j]); // we assume two points here, which should always be the case
                }
                for (int i = 0; i < alreadyCalculated.Count; i++)
                {
                    c3ds[i] = alreadyCalculated[i].Clone();
                    crvsOnSurface1[i] = fc1.Surface.GetProjectedCurve(c3ds[i], 0.0);
                    crvsOnSurface2[i] = fc2.Surface.GetProjectedCurve(c3ds[i], 0.0);
                    SurfaceHelper.AdjustPeriodic(fc1.Surface, fc1.Domain, crvsOnSurface1[i]);
                    SurfaceHelper.AdjustPeriodic(fc2.Surface, fc2.Domain, crvsOnSurface2[i]);
                    // TODO: InterpolatedDualSurfaceCurves don't easily adjust to the required domain
                    // we must somehow force them to do so
                    for (int j = 0; j < points.Count; j++) // fill the parameter arrays
                    {
                        double d = c3ds[i].DistanceTo(points[j]);
                        if (d > precision)
                        {   // point is not on the curve
                            params3d[i, j] = double.MinValue;
                            params2dFace1[i, j] = double.MinValue;
                            params2dFace2[i, j] = double.MinValue;
                        }
                        else
                        {
                            params3d[i, j] = c3ds[i].PositionOf(points[j]);
                            SurfaceHelper.AdjustPeriodic(fc1.Surface, fc1.Domain, ref paramsuvsurf1[j]);
                            SurfaceHelper.AdjustPeriodic(fc2.Surface, fc2.Domain, ref paramsuvsurf2[j]);
                            params2dFace1[i, j] = crvsOnSurface1[i].PositionOf(paramsuvsurf1[j]);
                            params2dFace2[i, j] = crvsOnSurface2[i].PositionOf(paramsuvsurf2[j]);
                            if (params2dFace1[i, j] < -Precision.eps || params2dFace1[i, j] > 1 + Precision.eps) params2dFace1[i, j] = double.MinValue; // point is not on the curve
                            if (params2dFace2[i, j] < -Precision.eps || params2dFace2[i, j] > 1 + Precision.eps) params2dFace2[i, j] = double.MinValue; // point is not on the curve
                            if ((crvsOnSurface1[i].PointAt(params2dFace1[i, j]) | paramsuvsurf1[j]) > precision) params2dFace1[i, j] = double.MinValue; // point is not on the curve
                            if ((crvsOnSurface2[i].PointAt(params2dFace2[i, j]) | paramsuvsurf2[j]) > precision) params2dFace2[i, j] = double.MinValue; // point is not on the curve
                        }
                    }
                }
            }
            if (c3ds != null || Surfaces.Intersect(fc1.Surface, fc1.Area.GetExtent(), fc2.Surface, fc2.Area.GetExtent(), points, out c3ds,
                out crvsOnSurface1, out crvsOnSurface2, out params3d, out params2dFace1, out params2dFace2, out paramsuvsurf1, out paramsuvsurf2, precision))
            {
                if (usedVertices.Count < points.Count)
                {
                    // There have been additional vertices created.
                    // This happens e.g. when two cylinders with the same diameter intersect or in general there is a touching point, which was not in points
                    for (int i = usedVertices.Count; i < points.Count; i++)
                    {   // changed to true: accept also on border, which is necessary with UniteBug5
                        if (!fc1.Contains(ref paramsuvsurf1[i], true) || !fc2.Contains(ref paramsuvsurf2[i], true))
                        {
                            for (int j = 0; j < c3ds.Length; j++)
                            {
                                params3d[j, i] = double.MinValue; // this vertex is invalid
                                                                  // but in order to keep points and usedVertices in sync, we still have to add a vertex to usedVertices
                            }
                        }
                        Vertex v = CreateOrFindVertex(points[i]);
                        usedVertices.Add(v);
                    }
                }
                for (int i = 0; i < c3ds.Length; i++) // only a single curve in most situations
                {   // the orientation fof the 3d curve is arbitrary, the two 2d curves have the same orientation as the 3d curve
                    // but in the final result they will have to be opposite oriented
                    SortedDictionary<double, int> sortedIv = new SortedDictionary<double, int>(); // point indices sorted by ascending u, only for curve i (mostly there is only one curve anyway)
                    for (int j = 0; j < points.Count; j++)
                    {
                        if (params3d[i, j] == double.MinValue) continue; // this intersection point lies on a different curve
                        if (params2dFace1[i, j] == double.MinValue) continue; // this intersection point lies on a different curve
                        if (params2dFace2[i, j] == double.MinValue) continue; // this intersection point lies on a different curve
                        sortedIv[params3d[i, j]] = j;
                    }
                    // with closed curves, there will be a problem: which parts to use. This must be considered but is not yet.
                    if (c3ds[i].IsClosed)
                    {
                        if (fc1.Area.Contains(crvsOnSurface1[i].StartPoint, false) && fc2.Area.Contains(crvsOnSurface2[i].StartPoint, false))
                        {
                            // The start point of the closed 2d curves (which corresponds to the start point of the 3d curve) belongs to a valid
                            // part of the curve. So we must not begin with the smallest u, but have to use the part which passes through the start.
                            // Note: the Trim methods (both 2d and 3d) must handle closed curves where the first parameter is greater than the second:
                            // then the part which passes through 0 is meant. Still to verify that all curves implement this.
                            // Here the first entry of sortedIv is removed and appended at the end (with u+1).
                            var fiter = sortedIv.GetEnumerator();
                            fiter.MoveNext();
                            double u0 = fiter.Current.Key;
                            int j0 = fiter.Current.Value;
                            sortedIv.Remove(u0);
                            sortedIv[u0 + 1] = j0;
                        }
                    }
                    List<int> ivIndices = new List<int>(sortedIv.Count);
                    foreach (int ind in sortedIv.Values)
                    {
                        ivIndices.Add(ind);
                    }
                    for (int j = 0; j < ivIndices.Count - 1; j++)
                    {

                        int j1 = ivIndices[j];
                        double u1 = params3d[i, j1];
                        int j2 = ivIndices[j + 1];
                        double u2 = params3d[i, j2];
                        if (skipPairs != null && (skipPairs.Contains((usedVertices[j1], usedVertices[j2])) || skipPairs.Contains((usedVertices[j2], usedVertices[j1]))))
                            continue; // this segment has already been created from a known intersection curve
                        // before trimming we test whether we need this curve at all
                        // if both endpoints of the curve are on the border of the face, the curve may be totally outside
                        bool isOnFace1Border = false, isOnFace2Border = false; // one of the vertices is on the border of this face
                        bool j1IsOnBorder = false, j2IsOnBorder = false;
                        if (fc1.Area.GetPosition(usedVertices[j1].GetPositionOnFace(fc1)) == Border.Position.OnCurve)
                        {
                            j1IsOnBorder = true;
                            isOnFace1Border = true;
                        }
                        if (fc1.Area.GetPosition(usedVertices[j2].GetPositionOnFace(fc1)) == Border.Position.OnCurve)
                        {
                            j2IsOnBorder = true;
                            isOnFace1Border = true;
                        }
                        if (fc2.Area.GetPosition(usedVertices[j1].GetPositionOnFace(fc1)) == Border.Position.OnCurve)
                        {
                            j1IsOnBorder = true;
                            isOnFace2Border = true;
                        }
                        if (fc2.Area.GetPosition(usedVertices[j2].GetPositionOnFace(fc1)) == Border.Position.OnCurve)
                        {
                            j2IsOnBorder = true;
                            isOnFace2Border = true;
                        }
                        if ((j1IsOnBorder && j2IsOnBorder) || sortedIv.Count > 2)
                        {   // both endpoints are on the border: it is not sure that the intersectioncurve is inside the face
                            // more than two intersectionpoints: maybe some segment is outside
                            GeoPoint2D uv = crvsOnSurface1[i].PointAt((params2dFace1[i, j1] + params2dFace1[i, j2]) / 2.0);
                            if (!fc1.Contains(ref uv, true))
                            {   // it still might be a point on the edge,also test it in 3d, because it might be imprecise in 2d
                                uv = fc1.Surface.PositionOf(c3ds[i].PointAt((u1 + u2) / 2.0));
                                if (!fc1.Contains(ref uv, true))
                                {   // still too strong condition, we only need to exclude e.g. wrong halves of a circle
                                    if (fc1.Area.GetPosition(uv, fc1.Domain.Size * 1e-5) == Border.Position.Outside) continue;
                                }
                            }
                            uv = crvsOnSurface2[i].PointAt((params2dFace2[i, j1] + params2dFace2[i, j2]) / 2.0);
                            if (!fc2.Contains(ref uv, true))
                            {
                                uv = fc2.Surface.PositionOf(c3ds[i].PointAt((u1 + u2) / 2.0));
                                if (!fc2.Contains(ref uv, true))
                                {
                                    if (fc2.Area.GetPosition(uv, fc2.Domain.Size * 1e-5) == Border.Position.Outside) continue;
                                }
                            }
                        }

                        ICurve tr = c3ds[i].Clone(); // an InterpolatedDualSurfaceCurve must keep its surfaces
                        tr.Trim(u1, u2);
                        ICurve2D con1 = crvsOnSurface1[i].Trim(params2dFace1[i, j1], params2dFace1[i, j2]);
                        ICurve2D con2 = crvsOnSurface2[i].Trim(params2dFace2[i, j1], params2dFace2[i, j2]);
                        // projected curves are not precise when trimmed. 
                        if (ProjectedCurve.IsPlain(con1)) con1 = fc1.Surface.GetProjectedCurve(tr, Precision.eps);
                        if (ProjectedCurve.IsPlain(con2)) con2 = fc2.Surface.GetProjectedCurve(tr, Precision.eps);
                        {   // we need this test with the trimmed 3d curve in a strange case: a quarter of an ellipse is exactely outside a threequarter cylinder
                            // this part of a 2d curve gets arbitrarily wrong periodic adjusted
                            // PositionOf already returns the value in the surface domain, which equals the
                            // domain of the face (see the remarks on ISurface.Domain)
                            GeoPoint2D uv = fc1.Surface.PositionOf(tr.PointAt(0.5));
                            if (!fc1.Contains(ref uv, true))
                            {   // still too strong condition, we only need to exclude e.g. wrong halves of a circle
                                if (fc1.Area.GetPosition(uv, fc1.Domain.Size * 1e-5) == Border.Position.Outside) continue;
                            }
                            uv = fc2.Surface.PositionOf(tr.PointAt(0.5));
                            if (!fc2.Contains(ref uv, true))
                            {   // still too strong condition, we only need to exclude e.g. wrong halves of a circle
                                if (fc2.Area.GetPosition(uv, fc2.Domain.Size * 1e-5) == Border.Position.Outside) continue;
                            }
                        }
                        // An idea: turn InterpolatedDualSurfaceCurves into BSplines here (2d and 3d) and back into InterpolatedDualSurfaceCurves
                        // at the very end, maybe marked by a flag. The problem would be the precision when the direction is used to find the loops.
                        if (con1 is ProjectedCurve pon1 && pon1.IsCurveOfIntersection && con2 is ProjectedCurve pon2 && pon2.IsCurveOfIntersection &&
                            tr is InterpolatedDualSurfaceCurve idsc)
                        {   // con1 and con2 must refer to tr, otherwise the face cannot be reversed later by ReverseOrientation: that modifies
                            // the surface, and it must be the same surface for all curves
                            pon1.SetCurve3D(idsc);
                            pon2.SetCurve3D(idsc);
                        }
                        // The cross product of the normals specifies the direction of the new edge, no matter where on the curve we compute it.
                        // But if the surfaces are tangential in a point the cross product of the normals will be 0. So we take the better one
                        // if both are bad (e.g. two same diameter cylinders), we take a point in the middle
                        bool dirs1;
                        if (orientation != null && orientation[i].HasValue)
                        {
                            dirs1 = orientation[i] == false;
                        }
                        else
                        {
                            bool? forwardOnFace1 = GetOrientation(fc1, fc2, tr, paramsuvsurf1[j1], paramsuvsurf2[j1], paramsuvsurf1[j2], paramsuvsurf2[j2]);
                            if (forwardOnFace1.HasValue) { dirs1 = forwardOnFace1.Value; }
                            else
                            {
                                continue; // we cannot determine the direction of the edge, so we skip it. 
                            }
                        }
                        // dirs1: the curve is oriented forward on face 1, so it must be reversed on face 2, and vice versa
                        // (GetOrientation compares vectors which are either parallel or antiparallel, so its ">0" test is safe)
                        if (dirs1) con2.Reverse();
                        else con1.Reverse();
                        GeoPoint2D uv1 = con1.PointAt(0.5);
                        GeoPoint2D uv2 = con2.PointAt(0.5);
                        if (!fc1.Contains(ref uv1, true) || !fc2.Contains(ref uv2, true))
                        {   // the condition is too strong. Use the same relaxed condition as above
                            if (fc1.Area.GetPosition(uv1, fc1.Domain.Size * 1e-5) == Border.Position.Outside &&
                                fc2.Area.GetPosition(uv2, fc2.Domain.Size * 1e-5) == Border.Position.Outside) continue;
                        }
                        // we still need this adjustment, because the 2d curves are part of a single long 2d curve and have been trimmed after intersection, trimmed parts may fall into the wrong domain
                        SurfaceHelper.AdjustPeriodic(fc1.Surface, fc1.Domain, con1);
                        SurfaceHelper.AdjustPeriodic(fc2.Surface, fc2.Domain, con2);
                        Edge edge = new Edge(fc1, tr, fc1, con1, dirs1, fc2, con2, !dirs1);
                        //edge.edgeInfo = new EdgeInfo(edge);
                        //edge.edgeInfo.isIntersection = true;
#if DEBUG
                        (tr as IGeoObject).UserData.Add("DebugIntersectionBy1", fc1.GetHashCode());
                        (tr as IGeoObject).UserData.Add("DebugIntersectionBy2", fc2.GetHashCode());
                        if (con2 is ProjectedCurve pc2 && pc2.IsCurveOfIntersection)
                        {
                            // BSpline2D dbgbsp2d = pc2.ToBSpline(0.0);
                        }
#endif
                        if (dirs1) // the trimming of BSplines is sometimes not very exact
                        {
                            if (con1 is BSpline2D)
                            {
                                con1.StartPoint = fc1.PositionOf(tr.StartPoint);
                                con1.EndPoint = fc1.PositionOf(tr.EndPoint);
                            }
                            if (con2 is BSpline2D)
                            {
                                con2.StartPoint = fc2.PositionOf(tr.EndPoint);
                                con2.EndPoint = fc2.PositionOf(tr.StartPoint);
                            }
                        }
                        else
                        {
                            if (con1 is BSpline2D)
                            {
                                con1.StartPoint = fc1.PositionOf(tr.EndPoint);
                                con1.EndPoint = fc1.PositionOf(tr.StartPoint);
                            }
                            if (con2 is BSpline2D)
                            {
                                con2.StartPoint = fc2.PositionOf(tr.StartPoint);
                                con2.EndPoint = fc2.PositionOf(tr.EndPoint);
                            }
                        }
                        edge.Vertex1 = usedVertices[j1];    // this connects the new edge with the two intersection vertices
                        edge.Vertex2 = usedVertices[j2];
                        createdPairs?.Add((edge.Vertex1, edge.Vertex2));
                        {
                            bool addToFace1 = true, addToFace2 = true;
                            Edge[] splitted = null;
                            if (j1IsOnBorder && j2IsOnBorder)
                            {   // a very rare case (like in BRepTest30.cdb.json): the new intersecting edge starts and ends on the border of the face AND contains an already existing vertex of that face.
                                // in this case we have to split the new edge
                                if (isOnFace1Border)
                                {   // first faster check: is the intersection edge tangential to an outline edge
                                    bool tangential = false;
                                    List<Edge> le = edge.Vertex1.EdgesOnFace(fc1);
                                    for (int k = 0; k < le.Count; k++)
                                    {
                                        if (le[k] == edge) continue;
                                        if (le[k].Curve3D == null) continue; // a pole
                                        if (le[k].Vertex1 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.StartDirection, edge.Curve3D.StartDirection, false)) tangential = true;
                                        else if (le[k].Vertex2 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.EndDirection, edge.Curve3D.StartDirection, false)) tangential = true;
                                    }
                                    le = edge.Vertex2.EdgesOnFace(fc1);
                                    for (int k = 0; k < le.Count; k++)
                                    {
                                        if (le[k] == edge) continue;
                                        if (le[k].Curve3D == null) continue; // a pole
                                        if (le[k].Vertex1 == edge.Vertex2 && Precision.SameDirection(le[k].Curve3D.StartDirection, edge.Curve3D.EndDirection, false)) tangential = true;
                                        else if (le[k].Vertex2 == edge.Vertex2 && Precision.SameDirection(le[k].Curve3D.EndDirection, edge.Curve3D.EndDirection, false)) tangential = true;
                                    }
                                    if (tangential)
                                    {
                                        SortedList<double, Vertex> splitPositions = new SortedList<double, Vertex>();
                                        foreach (Vertex vtx in fc1.Vertices)
                                        {
                                            if (vtx != edge.Vertex1 && vtx != edge.Vertex2)
                                            {
                                                if (edge.Curve3D.DistanceTo(vtx.Position) < precision)
                                                {
                                                    double u = edge.Curve3D.PositionOf(vtx.Position);
                                                    if (u > 1e-6 && u < 1 - 1e-6)
                                                    {
                                                        splitPositions.Add(u, vtx);
                                                    }
                                                }
                                            }
                                        }
                                        if (splitPositions.Count > 0) splitted = edge.Split(splitPositions, precision);
                                    }
                                }
                                if (splitted == null && isOnFace2Border)
                                {
                                    bool tangential = false;
                                    List<Edge> le = edge.Vertex1.EdgesOnFace(fc2);
                                    for (int k = 0; k < le.Count; k++)
                                    {
                                        if (le[k].Curve3D != null)
                                        {   // it could be a pole
                                            if (le[k] == edge) continue;
                                            if (le[k].Vertex1 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.StartDirection, edge.Curve3D.StartDirection, false)) tangential = true;
                                            else if (le[k].Vertex2 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.EndDirection, edge.Curve3D.StartDirection, false)) tangential = true;
                                        }
                                    }
                                    le = edge.Vertex2.EdgesOnFace(fc2);
                                    for (int k = 0; k < le.Count; k++)
                                    {
                                        if (le[k] == edge || le[k].Curve3D == null) continue;
                                        if (le[k].Vertex1 == edge.Vertex2 && Precision.SameDirection(le[k].Curve3D.StartDirection, edge.Curve3D.EndDirection, false)) tangential = true;
                                        else if (le[k].Vertex2 == edge.Vertex2 && Precision.SameDirection(le[k].Curve3D.EndDirection, edge.Curve3D.EndDirection, false)) tangential = true;
                                    }
                                    if (tangential)
                                    {
                                        SortedList<double, Vertex> splitPositions = new SortedList<double, Vertex>();
                                        foreach (Vertex vtx in fc2.Vertices)
                                        {
                                            if (vtx != edge.Vertex1 && vtx != edge.Vertex2)
                                            {
                                                if (edge.Curve3D.DistanceTo(vtx.Position) < precision)
                                                {
                                                    double u = edge.Curve3D.PositionOf(vtx.Position);
                                                    if (u > 1e-6 && u < 1 - 1e-6)
                                                    {
                                                        splitPositions.Add(u, vtx);
                                                    }
                                                }
                                            }
                                        }
                                        if (splitPositions.Count > 0) splitted = edge.Split(splitPositions, precision);
                                    }
                                }
                            }
                            HashSet<Edge> addTo;
                            if (addToFace1)
                            {
                                if (!faceToIntersectionEdges.TryGetValue(fc1, out addTo))
                                {
                                    addTo = new HashSet<Edge>(); // a comparer by vertices (to avoid two edges with the same vertices) did not help
                                    faceToIntersectionEdges[fc1] = addTo;
                                }
                                if (splitted != null) addTo.UnionWith(splitted);
                                else addTo.Add(edge);
                            }
                            if (addToFace2)
                            {
                                if (!faceToIntersectionEdges.TryGetValue(fc2, out addTo))
                                {
                                    addTo = new HashSet<Edge>(); //  (new EdgeComparerByVertex());
                                    faceToIntersectionEdges[fc2] = addTo;
                                }
                                if (splitted != null) addTo.UnionWith(splitted);
                                else addTo.Add(edge);
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Calculates the orrientation of the intersection curve. The curve is an intersection between face fc1 and fc2.
        /// uv1sp..uv2ep are uv parameters of the start- and endpoint of the curve on fc1 resp. fc2.
        /// 
        /// </summary>
        /// <param name="fc1"></param>
        /// <param name="fc2"></param>
        /// <param name="intersectionCurve"></param>
        /// <param name="uv1sp"></param>
        /// <param name="uv2sp"></param>
        /// <param name="uv1ep"></param>
        /// <param name="uv2ep"></param>
        /// <returns></returns>
        private bool? GetOrientation(Face fc1, Face fc2, ICurve intersectionCurve, GeoPoint2D uv1sp, GeoPoint2D uv2sp, GeoPoint2D uv1ep, GeoPoint2D uv2ep)
        {
            GeoVector normalsCrossedStart = fc1.Surface.GetNormal(uv1sp).Normalized ^ fc2.Surface.GetNormal(uv2sp).Normalized;
            GeoVector normalsCrossedEnd = fc1.Surface.GetNormal(uv1ep).Normalized ^ fc2.Surface.GetNormal(uv2ep).Normalized;
            if (normalsCrossedStart.Length > 0.01 || normalsCrossedEnd.Length > 0.01)
            {   // simple case: not tangential, the crossproduct of the normals to the face.
                // The result is same direction of cross product and intersection curve
                // maybe one of the uv points is a pole, which has an unstable normal. In this case we use the other point
                bool ipsp = IsPole(fc1.Surface, uv1sp) || IsPole(fc2.Surface, uv2sp); // startpoint is a pole
                bool ipep = IsPole(fc1.Surface, uv1ep) || IsPole(fc2.Surface, uv2ep); // endpoint is a pole
                if (ipsp && ipep)
                {   // bath startpoint and endpoint are poles: check in the middle
                    GeoPoint mp = intersectionCurve.PointAt(0.5);
                    GeoVector ncm = fc1.Surface.GetNormal(fc1.Surface.PositionOf(mp)) ^ fc2.Surface.GetNormal(fc2.Surface.PositionOf(mp));
                    if (ncm.Length > 100 * Precision.eps)
                    {
                        return (ncm * intersectionCurve.DirectionAt(0.5)) > 0;
                    } // else fall through to tangential case
                }
                else if (ipsp)
                {   // startpoint is pole: check at the endpoint
                    return (normalsCrossedEnd * intersectionCurve.EndDirection) > 0;
                }
                else if (IsPole(fc1.Surface, uv1ep) || IsPole(fc2.Surface, uv2ep))
                {   // endpoint is pole: check at the startpoint
                    return (normalsCrossedStart * intersectionCurve.StartDirection) > 0;
                }
                else
                {   // choose the better cross product, but both should be the same result
                    if (normalsCrossedStart.Length > normalsCrossedEnd.Length)
                    {
                        return (normalsCrossedStart * intersectionCurve.StartDirection) > 0;
                    }
                    else
                    {
                        return (normalsCrossedEnd * intersectionCurve.EndDirection) > 0;
                    }
                }
            }
            // it seems to be tangential at the endpoints of the intersection curve: test in the middle of the intersection curve
            // the middle point is never a pole, because a curve never crosses a pole
            GeoPoint m = intersectionCurve.PointAt(0.5);
            GeoVector normalsCrossedMiddle = fc1.Surface.GetNormal(fc1.Surface.PositionOf(m)).Normalized ^ fc2.Surface.GetNormal(fc2.Surface.PositionOf(m)).Normalized;
            if (normalsCrossedMiddle.Length > 0.01) // more than 1.7°
            {
                return (normalsCrossedMiddle * intersectionCurve.DirectionAt(0.5)) > 0;
            }
            else
            {
                // it is also tangential at the midpoint of the intersection curve.
                // We consider the whole intersection curve beeing tangential.
                // It may be an existing edge of fc1 or fc2. Lets try to find them.
                int numberOfEdgesOnFc = 0;
                Face faceWithEdge = null;
                Face otherFace = null;
                foreach (Edge edg in fc1.AllEdges)
                {
                    if (edg.Curve3D != null && Curves.Overlapping(edg.Curve3D, intersectionCurve, Precision.eps, out double u1, out double u2, out double v1, out double v2))
                    {
                        if (edg.Curve3D.DistanceTo(intersectionCurve.PointAt(0.5)) < Precision.eps)
                        {
                            ++numberOfEdgesOnFc;
                            faceWithEdge = fc1;
                            otherFace = fc2;
                            break;
                        }
                    }
                }
                foreach (Edge edg in fc2.AllEdges)
                {
                    if (edg.Curve3D != null && Curves.Overlapping(edg.Curve3D, intersectionCurve, Precision.eps, out double u1, out double u2, out double v1, out double v2))
                    {
                        if (edg.Curve3D.DistanceTo(intersectionCurve.PointAt(0.5)) < Precision.eps)
                        {
                            ++numberOfEdgesOnFc;
                            faceWithEdge = fc2;
                            otherFace = fc1;
                            break;
                        }
                    }
                }
                // three different cases:
                // numberOfEdgesOnFc==0: touching without edges: like a cylinder and a plane can touch
                // numberOfEdgesOnFc==1: one face ends on the intersection, the othe face goes through the intersection curve
                // numberOfEdgesOnFc==2: both faces are connected at the intersection curve
                GeoPoint2D uvf1 = fc1.Surface.PositionOf(m);
                GeoPoint2D uvf2 = fc2.Surface.PositionOf(m);
                GeoVector n1 = fc1.Surface.GetNormal(uvf1).Normalized;
                GeoVector n2 = fc2.Surface.GetNormal(uvf2).Normalized;
                double stepSize = Math.Min(fc1.GetExtent(0.0).Size, fc2.GetExtent(0.0).Size) * 1e-2;
                GeoVector probeDir = n1; // the same or opposite as n2
                GeoVector stepDir = (intersectionCurve.DirectionAt(0.5) ^ probeDir).Normalized;
                Plane plane = new Plane(m, stepDir, probeDir);
                // this plane is perpendicular to the intersection curve and the faces are tangential to the x-axis
                // now we probe the faces with lines left and right of the intersection curve
                for (int t = 0; t < 10; ++t) // try 10 times
                {
                    // make two probes: left and right of the intersection curve
                    GeoPoint fc1right = fc1.GetLineIntersection(m + stepSize * stepDir, probeDir).
                        MinByWithDefault(GeoPoint.Invalid, (p => p | m));
                    GeoPoint fc2right = fc2.GetLineIntersection(m + stepSize * stepDir, probeDir).
                        MinByWithDefault(GeoPoint.Invalid, (p => p | m));
                    GeoPoint fc1left = fc1.GetLineIntersection(m - stepSize * stepDir, probeDir).
                        MinByWithDefault(GeoPoint.Invalid, (p => p | m));
                    GeoPoint fc2left = fc2.GetLineIntersection(m - stepSize * stepDir, probeDir).
                        MinByWithDefault(GeoPoint.Invalid, (p => p | m));
                    if (numberOfEdgesOnFc == 0)
                    {   // the two faces touch but should be defined on both sides of the intersection curve
                        if (fc1right.IsValid && fc2right.IsValid && fc1left.IsValid && fc2left.IsValid)
                        {
                            GeoVector diffRight = fc1right - fc2right;
                            GeoVector diffLeft = fc1left - fc2left;
                            if (diffLeft.Length > 10 * Precision.eps && diffRight.Length > 10 * Precision.eps)
                            {
                                if (diffLeft * diffRight > 0)
                                {   // the two faces are on the same side of the intersection curve, so this is a touching intersection, we don't need it
                                    return null;
                                }
                                else
                                {
                                    // the two faces are on different sides of the intersection curve, so this is a real intersection, we need it
                                    // now it depends on whether the normals have the same or opposite direction
                                    // and which difference goes up or down in the plane. 
                                    // this is not tested yet, it is difficult to find a case with a "S"-surface intersection
                                    // return (n1*n2>0) == plane.ToLocal(diffRight).y > 0;
                                    // so we take the traditional way: look at the normals
                                    GeoPoint2D uv1 = fc1.Surface.PositionOf(fc1right);
                                    GeoPoint2D uv2 = fc2.Surface.PositionOf(fc2right);
                                    GeoVector crossright = fc1.Surface.GetNormal(uv1).Normalized ^ fc2.Surface.GetNormal(uv2).Normalized;
                                    if (crossright.Length > 1e-4)
                                    {
                                        return (crossright * intersectionCurve.DirectionAt(0.5)) > 0;
                                    }
                                    uv1 = fc1.Surface.PositionOf(fc1left);
                                    uv2 = fc2.Surface.PositionOf(fc2left);
                                    GeoVector crossleft = fc1.Surface.GetNormal(uv1).Normalized ^ fc2.Surface.GetNormal(uv2).Normalized;
                                    if (crossleft.Length > 1e-4)
                                    {
                                        return (crossleft * intersectionCurve.DirectionAt(0.5)) > 0;
                                    }
                                }
                            }
                        }
                    }
                    else // if (numberOfEdgesOnFc > 0)
                    {   // one of the faces ends on the intersection curve, we check on the other side
                        // or both faces end on the intersection curve
                        if (fc1right.IsValid && fc2right.IsValid)
                        {
                            GeoPoint2D uv1 = fc1.Surface.PositionOf(fc1right);
                            GeoPoint2D uv2 = fc2.Surface.PositionOf(fc2right);
                            GeoVector crossright = fc1.Surface.GetNormal(uv1).Normalized ^ fc2.Surface.GetNormal(uv2).Normalized;
                            if (crossright.Length > 1e-4)
                            {
                                return (crossright * intersectionCurve.DirectionAt(0.5)) > 0;
                            }
                        }
                        else if (fc1left.IsValid && fc2left.IsValid)
                        {
                            GeoPoint2D uv1 = fc1.Surface.PositionOf(fc1left);
                            GeoPoint2D uv2 = fc2.Surface.PositionOf(fc2left);
                            GeoVector crossleft = fc1.Surface.GetNormal(uv1).Normalized ^ fc2.Surface.GetNormal(uv2).Normalized;
                            if (crossleft.Length > 1e-4)
                            {
                                return (crossleft * intersectionCurve.DirectionAt(0.5)) > 0;
                            }
                        }
                        else if (numberOfEdgesOnFc == 2)
                        {   // both faces end on the intersection curve, but they go on in the same direction
                            // we cannot provide information here. The same intersection curve will be checked
                            // with the other combinations of faces which fold backwards
                            return null;
                        }
                    }
                    stepSize *= 2;
                }
            }
            return null;
        }

        /// <summary>
        /// Whether <paramref name="uv"/> is at a singularity (pole) of the surface, where the normal is not well defined.
        /// </summary>
        private bool IsPole(ISurface surface, GeoPoint2D uv)
        {
            double[] us = surface.GetUSingularities();
            for (int i = 0; i < us.Length; i++) { if (Math.Abs(uv.x - us[i]) < 1e-6) { return true; } }
            double[] vs = surface.GetVSingularities();
            for (int i = 0; i < vs.Length; i++) { if (Math.Abs(uv.y - vs[i]) < 1e-6) { return true; } }
            return false;
        }

        /// <summary>
        /// Whether the curve of <paramref name="edge"/> has a common piece with the curve of an edge of <paramref name="face"/>.
        /// ICurve.SameGeometry only finds identical curves, not overlapping ones. The ends of a common piece are ends of one of
        /// the two curves, so two of the four ends lie on both curves, and so does the point between them.
        /// </summary>
        private bool OverlapsOutline(Face face, Edge edge)
        {
            ICurve curve = edge.Curve3D;
            if (curve == null) return false;
            double tolerance = 10 * precision;
            foreach (Edge faceEdge in face.Edges)
            {
                ICurve other = faceEdge.Curve3D;
                if (other == null) continue;
                List<GeoPoint> common = [];
                foreach (GeoPoint p in new[] { other.StartPoint, other.EndPoint }) if (curve.DistanceTo(p) < tolerance) common.Add(p);
                foreach (GeoPoint p in new[] { curve.StartPoint, curve.EndPoint }) if (other.DistanceTo(p) < tolerance) common.Add(p);
                for (int i = 0; i < common.Count; i++)
                {
                    for (int j = i + 1; j < common.Count; j++)
                    {
                        if ((common[i] | common[j]) < tolerance) continue;
                        GeoPoint between = curve.PointAt((curve.PositionOf(common[i]) + curve.PositionOf(common[j])) / 2.0);
                        if (other.DistanceTo(between) < tolerance) return true;
                    }
                }
            }
            return false;
        }
        /// <summary>
        /// Intersects the curve of <paramref name="edge"/> with <paramref name="face"/> and returns the intersection vertices (existing
        /// vertices are reused, see <see cref="CreateOrFindVertex"/>). Intersections inside the edge are registered in edgesToSplit, as well as
        /// intersections on an edge of the face. The results are cached in FaceEdgeIntersections, and the precalculated raw intersections
        /// (<see cref="PrecalculateEdgeIntersections"/>) are used, if available.
        /// </summary>
        /// <param name="face">the face to intersect with</param>
        /// <param name="edge">the edge, which belongs to a face of the other shell</param>
        /// <param name="curveIsInSurface">true, if the curve of the edge lies in the surface of the face. Then there are no ordinary intersection
        /// points and the caller must clip the curve with the face</param>
        /// <returns>the intersection vertices</returns>
        private IEnumerable<Vertex> GetFaceEdgeIntersection(Face face, Edge edge, out bool curveIsInSurface)
        {
            if (FaceEdgeIntersections.TryGetValue((edge, face), out var found))
            {   // this is often called twice for the same edge/face pair, because the edges are on two faces
                curveIsInSurface = found.curveIsInSurface;
                return found.vertices;
            }
            HashSet<Vertex> res = new HashSet<Vertex>();
            GeoPoint[] ip;
            GeoPoint2D[] uvOnFace;
            double[] uOnCurve3D;
            Border.Position[] position;
            curveIsInSurface = false;
            if (edge.Curve3D == null) return res;
            double prec = precision / edge.Curve3D.Length;
            if (edge.PrimaryFace == face || edge.SecondaryFace == face) return res; // necessary for multipleFaces
            if (edge.PrimaryFace.Surface.SameGeometry(edge.PrimaryFace.Domain, face.Surface, face.Domain, precision, out ModOp2D _)) { curveIsInSurface = true; return res; }
            if (edge.SecondaryFace != null && edge.SecondaryFace.Surface.SameGeometry(edge.SecondaryFace.Domain, face.Surface, face.Domain, precision, out ModOp2D _)) { curveIsInSurface = true; return res; }
            var precalculated = default((bool curveIsOnSurface, GeoPoint[] ips, GeoPoint2D[] uvOnFaces, double[] uOnCurve3Ds));
            bool isPrecalculated = precalculatedEdgeIntersections != null && precalculatedEdgeIntersections.TryGetValue((edge, face), out precalculated);
            if (isPrecalculated ? precalculated.curveIsOnSurface : face.Surface.IsCurveOnSurface(edge.Curve3D))
            {
                if (face.Surface.IsUPeriodic || face.Surface.IsVPeriodic)
                {   // there are cases where the edge.Curve3D spirals around the face. Working with the projected curve would yield the wrong result
                    foreach (Edge faceEdge in face.Edges)
                    {
                        if (faceEdge.Curve3D != null)
                        {
                            if (Curves.Intersect(faceEdge.Curve3D, edge.Curve3D, out double[] u1, out double[] u2, out GeoPoint[] ipoints) > 0)
                            {
                                for (int i = 0; i < ipoints.Length; i++)
                                {
                                    if (u1[i] < -Precision.eps || u1[i] > 1 + Precision.eps || u2[i] < -Precision.eps || u2[i] > 1 + Precision.eps) continue;
                                    Vertex v = CreateOrFindVertex(ipoints[i]);
                                    v.AddPositionOnFace(face, face.PositionOf(ipoints[i]));
                                    AddEdgeSplitVertex(edge, v);
                                    res.Add(v);
                                }
                            }
                        }
                    }
                }
                else if (OverlapsOutline(face, edge))
                {   // the curve overlaps an edge of the face: clipping in 2d yields arbitrary points along the overlapping part.
                    // The curve is clipped by the vertices of the face on it
                    foreach (Vertex v in face.Vertices)
                    {
                        if (edge.Curve3D.DistanceTo(v.Position) < 10 * precision)
                        {
                            AddEdgeSplitVertex(edge, v);
                            res.Add(v);
                        }
                    }
                }
                else
                {
                    ICurve2D c2d = face.Surface.GetProjectedCurve(edge.Curve3D, 0.0);
                    // TODO: there is a problem here: in UniteBug8 there are two differen ways to oproject the ellipse onto the sphere:
                    // u==1,5 and u=4.7 which describe two different part of a projected curve. This cannot be resolved by GetProjectedCurve
                    SurfaceHelper.AdjustPeriodic(face.Surface, face.Domain, c2d);
                    double[] parts = face.Area.Clip(c2d, true); // there could be multiple parts with periodic surfaces, which are not found here
                    for (int i = 0; i < parts.Length; i++)
                    {
                        GeoPoint2D uv = c2d.PointAt(parts[i]);
                        GeoPoint p = face.Surface.PointAt(uv);
                        Vertex v = CreateOrFindVertex(p);
                        v.AddPositionOnFace(face, uv);
                        AddEdgeSplitVertex(edge, v);
                        res.Add(v);
                    }
                }
                foreach (Vertex v in new Vertex[] { edge.Vertex1, edge.Vertex2 })
                {
                    if (!res.Contains(v) && face.Contains(v.Position, true))
                    {
                        AddEdgeSplitVertex(edge, v);
                        res.Add(v);
                    }
                }
                curveIsInSurface = true;
                FaceEdgeIntersections[(edge, face)] = (res.ToList(), curveIsInSurface);
                return res;
            }
            if (isPrecalculated)
            {
#if DEBUG
                face.DebugShowEdgeIntersection(edge); // as Face.IntersectAndPosition does
#endif
                // the uv values may be moved by a period while they are classified, keep the precalculated ones as they are
                face.ClassifyEdgeIntersections(precalculated.ips, (GeoPoint2D[])precalculated.uvOnFaces.Clone(), precalculated.uOnCurve3Ds,
                    precision == 0.0 ? Precision.eps : precision, out ip, out uvOnFace, out uOnCurve3D, out position);
            }
            else face.IntersectAndPosition(edge, out ip, out uvOnFace, out uOnCurve3D, out position, precision);
            for (int i = 0; i < ip.Length; ++i)
            {
                if (uOnCurve3D[i] < -prec || uOnCurve3D[i] > 1.0 + prec || position[i] == Border.Position.Outside)
                {
                    // Intersection points at the ends of the edge are also created, because intersection edges may start or end there.
                    // But they must not be used to split the edge (see SplitEdges).
                    continue;
                }
                // A tangential intersection (e.g. an edge lying in a surface which a fillet touches) is imprecise: the
                // distance grows only with the square of the displacement. When a vertex of the face lies on the edge,
                // this vertex is the exact intersection, use it instead of an almost identical new vertex.
                Vertex? faceVertex = FaceVertexOnEdge(face, edge, ip[i]);
                if (faceVertex != null)
                {   // the position and the uv position of the vertex are exact, the ones of the intersection are not
                    ip[i] = faceVertex.Position;
                    uvOnFace[i] = faceVertex.GetPositionOnFace(face);
                }
                Vertex v = CreateOrFindVertex(ip[i]);
                if (v.Position != ip[i])
                {   // an existing vertex close to the intersection point: its uv position is that of its own (better) position
                    uvOnFace[i] = face.Surface.PositionOf(v.Position);
                    SurfaceHelper.AdjustPeriodic(face.Surface, face.Domain, ref uvOnFace[i]);
                }
                v.AddPositionOnFace(face, uvOnFace[i]);
                AddEdgeSplitVertex(edge, v);
                res.Add(v);
                // not sure what the following is good for:
                if (position[i] == Border.Position.OnCurve)
                {
                    List<Edge> touchedEdges = face.FindEdges(uvOnFace[i]);
                    for (int j = 0; j < touchedEdges.Count; j++)
                    {
                        if (touchedEdges[j].Curve3D == null) continue; // no poles here
                        double cprec = prec / touchedEdges[j].Curve3D.Length; // the length must not be 0
                        double pos = touchedEdges[j].Curve3D.PositionOf(v.Position);
                        if (pos > cprec && pos < 1 - cprec)
                        {
                            AddEdgeSplitVertex(touchedEdges[j], v);
                        }
                    }
                }
                if (operation == Operation.testonly) return res; // a single intersection point is enough for collision detection
            }
            FaceEdgeIntersections[(edge, face)] = (res.ToList(), curveIsInSurface);
            return res;
        }

        /// <summary>
        /// Returns a vertex of <paramref name="face"/>, which lies on the curve of <paramref name="edge"/> (within the
        /// precision) and close to the intersection point <paramref name="ip"/>, or null, if there is no such vertex.
        /// "Close" is much more generous than the precision, because tangential intersections may be that imprecise.
        /// </summary>
        private Vertex? FaceVertexOnEdge(Face face, Edge edge, GeoPoint ip)
        {
            Vertex? res = null;
            double minDist = 100 * precision;
            foreach (Vertex fv in face.Vertices)
            {
                double d = fv.Position | ip;
                if (d < minDist && edge.Curve3D.DistanceTo(fv.Position) < precision)
                {
                    minDist = d;
                    res = fv;
                }
            }
            return res;
        }

        /// <summary>
        /// The kind of operation. All operations are computed as an intersection: for <see cref="Operation.difference"/> the
        /// second shell is reversed first (A - B = A ∩ ¬B), for <see cref="Operation.union"/> both shells are reversed and the
        /// result is reversed back at the end (A ∪ B = ¬(¬A ∩ ¬B)).
        /// <list type="bullet">
        /// <item><see cref="Operation.connectMultiple"/> and <see cref="Operation.connectMultipleReversed"/> are set by <see cref="SetFaces"/>.</item>
        /// <item><see cref="Operation.testonly"/> only checks for a collision, see <see cref="GetCollision"/>.</item>
        /// <item><see cref="Operation.clip"/> is not implemented by BooleanOperation, ShapeIt uses it as a marker for splitting two solids.</item>
        /// </list>
        /// </summary>
        public enum Operation { union, intersection, difference, clip, connectMultiple, connectMultipleReversed, testonly }
        Operation operation;
        /// <summary>
        /// Connect all provided faces, remove the overhangs. Used by Shell.GetOffset. The faces are intersected with each other,
        /// all pairs, not only pairs from different shells. The faces are not cloned, they are modified. Call <see cref="Execute"/> afterwards.
        /// </summary>
        /// <param name="multipleFaces">the faces to connect</param>
        /// <param name="reverseOrientation">reverse the faces before and the result after the operation</param>
        public void SetFaces(IEnumerable<Face> multipleFaces, bool reverseOrientation)
        {
            if (reverseOrientation) operation = Operation.connectMultipleReversed;
            else operation = Operation.connectMultiple;
            this.multipleFaces = new List<Face>(multipleFaces); // we don't clone here, because we dont need it
            if (reverseOrientation) foreach (Face face in multipleFaces) face.ReverseOrientation();

            // fill the OctTree
            BoundingBox ext = BoundingBox.EmptyBoundingBox;
            foreach (Face face in multipleFaces) ext.MinMax(face.GetExtent(0.0));
            // in rare cases the extension isn't a good choice, faces shouldn't exactely reside on the sides of the small cubes of the octtree
            // so we modify the extension a little, to make this case extremely unlikely. The best solution would be to check, whether a vertex
            // falls exactely on the side of a octtree-cube, then throw an exception and try with a different octtree location
            double extsize = ext.Size;
            ext.Expand(extsize * 1e-3);
            ext = ext.Modify(new GeoVector(extsize * 1e-4, extsize * 1e-4, extsize * 1e-4));
            triangulationPrecision = ext.Size * 1e-4;
            foreach (Face face in multipleFaces) face.PreCalcTriangulation(triangulationPrecision);
            OctTree<Vertex> vertexOcttree = new OctTree<Vertex>(ext, extsize * 1e-6);
            foreach (Face face in multipleFaces) vertexOcttree.AddMany(face.Vertices);
            combineVertices(vertexOcttree);
            // everything else should work with execute
        }
        /// <summary>
        /// Creates an BooleanOperation object. Call <see cref="SetShells"/> or <see cref="SetFaces"/>, optionally set some options,
        /// and then <see cref="Execute"/>.
        /// </summary>
        public BooleanOperation()
        {

        }

        /// <summary>
        /// Sets the two shells and the operation. The shells are cloned, the originals remain unchanged. Both shells must be
        /// outward oriented. A shell which is not closed (e.g. a single face to split with) needs <see cref="SetClosedShells"/>.
        /// </summary>
        /// <param name="s1">the first shell, for a difference the shell to subtract from</param>
        /// <param name="s2">the second shell, for a difference the shell to subtract</param>
        /// <param name="operation">the operation to perform</param>
        public void SetShells(Shell s1, Shell s2, Operation operation)
        {
            originalToClonedEdges = new Dictionary<Edge, Edge>();
            originalToClonedVertices = new Dictionary<Vertex, Vertex>();
            originalToClonedFaces = new Dictionary<Face, Face>();
            shell1 = s1.Clone(originalToClonedEdges, originalToClonedVertices, originalToClonedFaces); // we clone the shells to not destroy the originals. We could keep references to the original by UserData
            shell2 = s2.Clone(originalToClonedEdges, originalToClonedVertices, originalToClonedFaces);
            this.operation = operation;
            // the precision of the triangulation for the volume calculations in Result, according to the size of the bounding box.
            // The faces are not triangulated here: Shell.MakeShell discards the triangulation of every face it collects into a
            // result (Face.InvalidateSecondaryData), so a triangulation made here was never used, only computed twice.
            BoundingBox ext = shell1.GetExtent(0.0);
            ext.MinMax(shell2.GetExtent(0.0));
            triangulationPrecision = ext.Size * 1e-4;
#if DEBUG
            // when no color is set, make shell1 red, shell2 green for better visual debugging
            ColorDef red = new ColorDef("red", Color.Red);
            ColorDef green = new ColorDef("green", Color.Green);
            foreach (Face fc in shell1.Faces)
            {
                if (fc.ColorDef == null) fc.ColorDef = red;
            }
            foreach (Face fc in shell2.Faces)
            {
                if (fc.ColorDef == null) fc.ColorDef = green;
            }
#endif
        }

        /// <summary>
        /// Clips <paramref name="toClip"/> by <paramref name="clipBy"/>: the difference of the two faces, each taken as an open shell,
        /// where only the trimmed parts of <paramref name="toClip"/> are returned. Returns null, if nothing remains.
        /// </summary>
        public static Face[] ClipFace(Face toClip, Face clipBy)
        {
            Shell shellToClip = Shell.MakeShell([toClip], false);
            Shell clippingShell = Shell.MakeShell([clipBy], false);
            BooleanOperation bo = new BooleanOperation();
            bo.SetShells(shellToClip, clippingShell, Operation.difference);
            bo.SetClosedShells(false, false);
            bo.DontReturnShell2 = true;
            bo.AllowOpenEdges = true;
            Shell[] resultShells = bo.Execute();
            if (resultShells.Length > 0)
            {
                return resultShells[0].Faces;
            }
            return null;
        }

        /// <summary>
        /// Splits a solid by a plane. The plane is represented by a single face, big enough to exceed the solid, which is subtracted
        /// from the solid. Uses only the first shell of the solid.
        /// </summary>
        /// <param name="solidToSplit">the solid to split</param>
        /// <param name="splitBy">the plane to split by</param>
        /// <param name="onlyInnerParts">if true, only the result of subtracting the plane as it is oriented, otherwise also the result
        /// of subtracting the reversed plane, i.e. the parts on both sides of the plane</param>
        /// <returns>the parts as solids</returns>
        public static Solid[] SplitSolidByPlane(Solid solidToSplit, Plane splitBy, bool onlyInnerParts = false)
        {
            Shell shellToSplit = solidToSplit.Shells[0];
            BoundingBox ext = shellToSplit.GetExtent(0.0);
            GeoPoint2D cnt2d = splitBy.Project(ext.GetCenter());
            BoundingRect br = new BoundingRect(cnt2d, 2 * ext.DiagonalLength, 2 * ext.DiagonalLength);
            Face fcpl = Face.MakeFace(new PlaneSurface(splitBy), br);
            Face fcpl1 = fcpl.Clone() as Face;
            fcpl1.MakeInverseOrientation();
            Shell splittingShell = Shell.MakeShell([fcpl], false); // the plane as a shell with only one face, which exceeds the solid
            BooleanOperation bo = new BooleanOperation();
            bo.SetShells(shellToSplit, splittingShell, Operation.difference);
            Shell[] resultShells = bo.Execute();
            List<Solid> resultSolids = new List<Solid>();
            for (int i = 0; i < resultShells.Length; i++)
            {
                resultSolids.Add(Solid.MakeSolid(resultShells[i]));
            }
            if (!onlyInnerParts)
            {
                bo = new BooleanOperation();
                splittingShell = Shell.MakeShell([fcpl1], false); // the same plane but inversed
                bo.SetShells(shellToSplit, splittingShell, Operation.difference);
                resultShells = bo.Execute();
                for (int i = 0; i < resultShells.Length; i++)
                {
                    resultSolids.Add(Solid.MakeSolid(resultShells[i]));
                }
            }
            return resultSolids.ToArray();
        }

        /// <summary>
        /// Merges vertices of the shells (or faces) which are closer than <see cref="precision"/>, and also the vertices of the intersection
        /// edges, if there are already some.
        /// </summary>
        private void combineVertices(OctTree<Vertex> vo)
        {
            HashSet<Vertex> vertices = new HashSet<Vertex>();
            if (multipleFaces != null)
            {
                foreach (Face fce in multipleFaces) vertices.UnionWith(fce.Vertices);
            }
            else
            {
                vertices.UnionWith(shell1.Vertices);
                vertices.UnionWith(shell2.Vertices);
            }
            foreach (Vertex v in vertices)
            {
                Vertex[] close = vo.GetObjectsCloseTo(v);
                bool found = false;
                for (int j = 0; j < close.Length; j++)
                {
                    if (close[j] == v) found = true;
                    else if ((close[j].Position | v.Position) < precision)
                    {
                        close[j].MergeWith(v);
                        found = true;
                        break;
                    }
                }
                if (!found) vo.AddObject(v);
            }

            if (faceToIntersectionEdges != null)
            {
                var allVerticesOfIntersectionEdges = faceToIntersectionEdges.Values.SelectMany(e => e).ToHashSet().SelectMany(e => new Vertex[] { e.Vertex1, e.Vertex2 }).ToHashSet();
                foreach (Vertex iv in allVerticesOfIntersectionEdges)
                {
                    Vertex[] close = vo.GetObjectsCloseTo(iv);
                    bool found = false;
                    for (int j = 0; j < close.Length; j++)
                    {
                        if ((close[j].Position | iv.Position) < precision)
                        {
                            close[j].MergeWith(iv);
                            found = true;
                            break;
                        }
                    }
                    if (!found) vo.AddObject(iv); // no close vertex found: a vertex of its own
                }
            }
        }

        /// <summary>
        /// Checks whether the surfaces of the two shells touch or intersect, without computing the result. Requires
        /// <see cref="Operation.testonly"/> in <see cref="SetShells"/>. Only edge/face intersections are detected: a shell lying
        /// completely inside the other one is no collision.
        /// </summary>
        /// <param name="commonPoint">some point where an edge of one shell intersects a face of the other shell</param>
        /// <returns>true, if there is an intersection</returns>
        public bool GetCollision(out GeoPoint commonPoint)
        {
            if (operation != Operation.testonly) throw new ApplicationException("Colloision detection only works with operation testonly");
            BoundingBox ext;
            ext = shell1.GetExtent(0.0);
            ext.MinMax(shell2.GetExtent(0.0));
            precision = ext.Size * 1e-6;
            double extsize = ext.Size;
            ext.Expand(extsize * 1e-3);
            ext = ext.Modify(new GeoVector(extsize * 1e-4, extsize * 1e-4, extsize * 1e-4));
            facesOctTree = new OctTree<Face>(ext, extsize * 1e-6);
            verticesOctTree = new OctTree<Vertex>(ext, extsize * 1e-6);
            edgesOctTree = new OctTree<Edge>(ext, extsize * 1e-6);
            facesOctTree.AddMany(shell1.Faces);
            facesOctTree.AddMany(shell2.Faces);
            AddToVertexOctTree(shell1);
            AddToVertexOctTree(shell2);

            edgesToSplit = new Dictionary<Edge, HashSet<Vertex>>();
            faceToIntersectionEdges = new Dictionary<Face, HashSet<Edge>>();
            faceToOverlappingFaces = new Dictionary<Face, Dictionary<Face, ModOp2D>>();
            faceToOppositeFaces = new Dictionary<Face, Dictionary<Face, ModOp2D>>();
            FaceEdgeIntersections = new Dictionary<(Edge, Face), (List<Vertex>, bool)>();

            CreateFaceIntersections();
            if (edgesToSplit.Any())
            {
                Vertex v = edgesToSplit.First().Value.First();
                commonPoint = v.Position;
                return true;
            }
            commonPoint = GeoPoint.Invalid;
            return false;
        }
        /// <summary>
        /// Performs the operation which has been set by <see cref="SetShells"/> or <see cref="SetFaces"/>:
        /// <list type="number">
        /// <item>reverses the shells according to the operation (see <see cref="Operation"/>) and builds the octtrees,</item>
        /// <item>intersects all pairs of faces (<see cref="CreateFaceIntersections"/>), which yields the intersection edges for each face
        /// (faceToIntersectionEdges), the vertices where the original edges must be split (edgesToSplit) and the pairs of overlapping faces,</item>
        /// <item>splits the original edges and handles the overlapping faces (<see cref="ProcessOverlappingFaces"/>),</item>
        /// <item>trims the faces and assembles the resulting shells (<see cref="Result"/>).</item>
        /// </list>
        /// The cloned shells are destroyed in this process, so Execute can only be called once.
        /// </summary>
        /// <returns>the resulting shells, outward oriented, possibly empty</returns>
        public Shell[] Execute()
        {   // we expect the shell sare in a proper state: outward oriented, no full periodic faces
            // don't know, whether we need the followin:
            if (shell1 != null && shell2 != null)
            {
                if (shell1IsClosed && shell1.HasOpenEdgesExceptPoles() && shell1IsClosed) shell1.TryConnectOpenEdges(precision);
                if (shell2IsClosed && shell2.HasOpenEdgesExceptPoles() && shell2IsClosed) shell2.TryConnectOpenEdges(precision);
                //shell1.RecalcVertices();
                //shell2.RecalcVertices();
                if (shell1IsClosed) shell1.CombineConnectedFaces();
                if (shell2IsClosed) shell2.CombineConnectedFaces();
            }

#if DEBUG
            DebuggerContainer dcfcs = new CADability.DebuggerContainer();
            if (shell1 != null && shell2 != null)
            {
                System.Diagnostics.Debug.Assert(shell1.CheckConsistency());
                System.Diagnostics.Debug.Assert(shell2.CheckConsistency());
                foreach (Face fce in shell1.Faces)
                {
                    dcfcs.Add(fce.Clone(), fce.GetHashCode()); // the faces will be destroyed, so keep clones here
                    double ll = fce.GetExtent(0.0).Size * 0.01;
                    ColorDef cd = new ColorDef("debug", Color.FromString("blue"));
                    SimpleShape ss = fce.Area;
                    GeoPoint2D c = ss.GetExtent().GetCenter();
                    GeoPoint pc = fce.Surface.PointAt(c);
                    GeoVector nc = fce.Surface.GetNormal(c);
                    Line l = Line.TwoPoints(pc, pc + ll * nc.Normalized);
                    l.ColorDef = cd;
                    dcfcs.Add(l);
                }
                foreach (Face fce in shell2.Faces)
                {
                    dcfcs.Add(fce.Clone(), fce.GetHashCode()); // the faces will be destroyed, so keep clones here
                    double ll = fce.GetExtent(0.0).Size * 0.01;
                    ColorDef cd = new ColorDef("debug", Color.Brown);
                    SimpleShape ss = fce.Area;
                    GeoPoint2D c = ss.GetExtent().GetCenter();
                    GeoPoint pc = fce.Surface.PointAt(c);
                    GeoVector nc = fce.Surface.GetNormal(c);
                    Line l = Line.TwoPoints(pc, pc + ll * nc.Normalized);
                    l.ColorDef = cd;
                    dcfcs.Add(l);
                }
            }
            else
            {
                foreach (Face fce in multipleFaces)
                {
                    dcfcs.Add(fce.Clone(), fce.GetHashCode()); // the faces will be destroyed, so keep clones here
                    double ll = fce.GetExtent(0.0).Size * 0.01;
                    ColorDef cd = new ColorDef("debug", Color.Brown);
                    SimpleShape ss = fce.Area;
                    GeoPoint2D c = ss.GetExtent().GetCenter();
                    GeoPoint pc = fce.Surface.PointAt(c);
                    GeoVector nc = fce.Surface.GetNormal(c);
                    Line l = Line.TwoPoints(pc, pc + ll * nc.Normalized);
                    l.ColorDef = cd;
                    dcfcs.Add(l);
                }
            }
#endif
            if (multipleFaces != null)
            {

            }
            else
            {
                Vertex[] dumy = shell1.Vertices; // only to make the vertices being calculated (they are cached)
                dumy = shell2.Vertices;
                if (operation == Operation.union)
                {
                    shell1.ReverseOrientation();
                    shell2.ReverseOrientation();
                }
                else if (operation == Operation.difference)
                {   // shell1 - shell2 is the intersection of shell1 with the inverse of shell2
                    shell2.ReverseOrientation();
                }
            }
            BoundingBox ext;
            if (multipleFaces != null)
            {
                ext = BoundingBox.EmptyBoundingBox;
                foreach (Face face in multipleFaces)
                {
                    ext.MinMax(face.GetExtent(0.0));
                }
            }
            else
            {
                ext = shell1.GetExtent(0.0);
                ext.MinMax(shell2.GetExtent(0.0));
            }
            precision = ext.Size * 1e-6;
            // a meassure to decide when points are close egneough to consider them as identical
            // in rare cases the extension isn't a good choice for the octtree, faces shouldn't exactely reside on octtree bounds
            // so we modify the extension a little, to make this case extremely unlikely. The best solution would be to check, whether a vertex
            // falls exactely on the side of a octtree-cube, then throw an exception and try with a different octtree location
            double extsize = ext.Size;
            ext.Expand(extsize * 1e-3);
            ext = ext.Modify(new GeoVector(extsize * 1e-4, extsize * 1e-4, extsize * 1e-4));
            facesOctTree = new OctTree<Face>(ext, extsize * 1e-6);
            verticesOctTree = new OctTree<Vertex>(ext, extsize * 1e-6);
            edgesOctTree = new OctTree<Edge>(ext, extsize * 1e-6);
            if (multipleFaces != null)
            {
                facesOctTree.AddMany(multipleFaces);
                HashSet<Vertex> allVertices = new HashSet<Vertex>();
                foreach (Face face in multipleFaces) allVertices.UnionWith(face.Vertices);
                verticesOctTree.AddMany(allVertices);
                edgesOctTree.AddMany(new HashSet<Edge>(multipleFaces.SelectMany(f => f.Edges)));
            }
            else
            {
                facesOctTree.AddMany(shell1.Faces);
                facesOctTree.AddMany(shell2.Faces);
                AddToVertexOctTree(shell1);
                AddToVertexOctTree(shell2);
                edgesOctTree.AddMany(shell1.Edges);
                edgesOctTree.AddMany(shell2.Edges);
            }
            combineVertices(verticesOctTree);

            edgesToSplit = new Dictionary<Edge, HashSet<Vertex>>();
            faceToIntersectionEdges = new Dictionary<Face, HashSet<Edge>>();
            faceToOverlappingFaces = new Dictionary<Face, Dictionary<Face, ModOp2D>>();
            faceToOppositeFaces = new Dictionary<Face, Dictionary<Face, ModOp2D>>();
            FaceEdgeIntersections = new Dictionary<(Edge, Face), (List<Vertex>, bool)>();

            CreateFaceIntersections();

#if DEBUG
            int dbgNumVert = verticesOctTree.GetAllObjects().Length;
            DebuggerContainer dc3 = new DebuggerContainer();
            foreach (Edge edge in edgesToSplit.Keys)
            {
                dc3.Add(edge.Curve3D as IGeoObject, edge.GetHashCode());
                foreach (Vertex v in edgesToSplit[edge])
                {
                    Point pnt = Point.Construct();
                    pnt.Location = v.Position;
                    pnt.Symbol = PointSymbol.Cross;
                    dc3.Add(pnt, v.GetHashCode());
                }
            }
#endif
            if (operation != Operation.testonly)
            {   // for testonly the edge intersections are sufficient (fast)
                if (multipleFaces != null) SplitMultipleFacesIntersectionEdges();
                SplitEdges(); // split the edges at the intersection vertices found
                ProcessOverlappingFaces();
                combineVertices(verticesOctTree);
            }
#if DEBUG
            int dbgNumVert1 = verticesOctTree.GetAllObjects().Length;
            foreach (KeyValuePair<Face, HashSet<Edge>> item in faceToIntersectionEdges)
            {
                foreach (Edge edg in item.Value)
                {
                    dc3.Add(edg.Curve3D as IGeoObject, item.Key.GetHashCode());
                }
            }
            DebuggerContainer dc4 = new DebuggerContainer();
            HashSet<Vertex> dbgv = new HashSet<Vertex>();
            if (multipleFaces != null)
            {
                foreach (Face fce in multipleFaces) dbgv.UnionWith(fce.Vertices);
            }
            else
            {
                dbgv.UnionWith(shell1.Vertices);
                dbgv.UnionWith(shell2.Vertices); // unfortunately partly from an outdated cache
            }
            foreach (Vertex v in dbgv)
            {
                Point pnt = Point.Construct();
                pnt.Location = v.Position;
                pnt.Symbol = PointSymbol.Cross;
                dc4.Add(pnt, v.GetHashCode());
            }
#endif
            return Result();
        }

        /// <summary>
        /// Only for the <see cref="SetFaces"/> mode: an intersection edge of two faces may cross a third face. Splits the intersection
        /// edges at these points, so that they can be used in the loops of each face.
        /// </summary>
        private void SplitMultipleFacesIntersectionEdges()
        {
            if (multipleFaces == null) return;
            HashSet<Edge> allIntersectionEdges = new HashSet<Edge>();
            foreach (HashSet<Edge> item in faceToIntersectionEdges.Values)
            {
                allIntersectionEdges.UnionWith(item);
            }
            foreach (Edge edge in allIntersectionEdges)
            {
                Face[] otherFaces = facesOctTree.GetObjectsCloseTo(edge.Curve3D as IOctTreeInsertable);
                HashSet<Vertex> vtxs = new HashSet<Vertex>();
                foreach (Face otherFace in otherFaces)
                {
                    if (otherFace == edge.PrimaryFace || otherFace == edge.SecondaryFace) continue;
                    vtxs.UnionWith(GetFaceEdgeIntersection(otherFace, edge, out bool curveIsInSurface));
                }
                vtxs.ExceptWith(edge.Vertices); // do not split an edge at its start or endvertex
                if (!vtxs.Any()) continue;
                SortedList<double, Vertex> sortedVertices = new SortedList<double, Vertex>();
                foreach (Vertex vtx in vtxs)
                {
                    sortedVertices.Add(edge.Curve3D.PositionOf(vtx.Position), vtx);
                }
                Edge[] splitted = edge.Split(sortedVertices, precision);
                // edge doesn't belong to the faces outline, so it is not replaced in primaryFace or secondaryFace
                edge.Vertex1.RemoveEdge(edge); // eliminate the unsplitted edges also from the vertices
                edge.Vertex2.RemoveEdge(edge);
                // the intersectionEdge is beeing replaced by its splitted parts
                if (faceToIntersectionEdges.TryGetValue(edge.PrimaryFace, out HashSet<Edge> primaryEdges))
                {
                    primaryEdges.Remove(edge);
                    primaryEdges.UnionWith(splitted);
                }
                if (faceToIntersectionEdges.TryGetValue(edge.SecondaryFace, out HashSet<Edge> secondaryEdges))
                {
                    secondaryEdges.Remove(edge);
                    secondaryEdges.UnionWith(splitted);
                }

            }
        }

        /// <summary>
        /// Handles the pairs of faces with the same surface, which have not been intersected by <see cref="AddFaceFaceIntersection"/>:
        /// <list type="bullet">
        /// <item>opposite oriented faces: the edges of one face which lie inside the other face become intersection edges of both faces,
        /// reversed, so the common part is removed. A face which is completely covered goes to totalyCoveredByOppositeFace.</item>
        /// <item>same oriented faces: a new common face is created (commonOverlappingFaces), the edges of each face inside the other face
        /// are its intersection edges, they build the outline of the common part in <see cref="Result"/>.</item>
        /// </list>
        /// Must be called after <see cref="SplitEdges"/>, so that each edge is either completely inside or outside the other face.
        /// </summary>
        private void ProcessOverlappingFaces()
        {   // When we have opposite faces which are overlapping, we have to subtract the common part from the result
            // When we have overlapping (same orientation) faces, we have to add the common part to the result

            // since all edges are splitted at the intersection points, each edge is either totally inside or outside the opposite face
            // we don't need to split edges here

            // first deal with opposite faces:
            // subtracting the common part is done by adding the edges inside the opposite face as intersection edges to both faces with the correct orientation
            foreach (var kv in faceToOppositeFaces)
            {
                Face mainFace = kv.Key;
                foreach (Face oppositeFace in kv.Value.Keys)
                {
                    ModOp2D firstToSecond = kv.Value[oppositeFace];
                    bool intersectionFound = false;
                    foreach (Edge edge in oppositeFace.Edges)
                    {
                        if (mainFace.Contains(edge.Curve3D.PointAt(0.5), false)) // the edge lies on the main face
                        {   // this edge, which is totally inside the main face, is dealt as an intersection edge of the opposite face
                            // the intersection edge is reverse to the edge orientation, to eliminate it.
                            bool forward = !edge.Forward(oppositeFace);
                            ICurve2D secondaryCurve2D = edge.Curve2D(oppositeFace).Clone();
                            if (firstToSecond.IsNull)
                            {
                                secondaryCurve2D = mainFace.Surface.GetProjectedCurve(edge.Curve3D, precision);
                                if (!edge.Forward(oppositeFace)) secondaryCurve2D.Reverse(); // still to test!
                            }
                            else secondaryCurve2D = secondaryCurve2D.GetModified(firstToSecond.GetInverse());
                            // the secsecondaryCurve2D is always correct oriented, we don't need to apply "forward" here
                            SurfaceHelper.AdjustPeriodic(mainFace.Surface, mainFace.Domain, secondaryCurve2D);
                            //Edge overlappingEdge = new Edge(oppositeFace, edge.Curve3D.Clone(), oppositeFace, edge.Curve2D(oppositeFace).CloneReverse(true), forward,
                            //    mainFace, secondaryCurve2D, !forward);
                            ICurve c3d = edge.Curve3D.Clone();
                            ICurve2D c2d1 = oppositeFace.Surface.GetProjectedCurve(c3d, 0.0);
                            if (!forward) c2d1.Reverse();
                            SurfaceHelper.AdjustPeriodic(oppositeFace.Surface, oppositeFace.Domain, c2d1);
                            ICurve2D c2d2 = mainFace.Surface.GetProjectedCurve(c3d, 0.0);
                            if (forward) c2d2.Reverse();
                            SurfaceHelper.AdjustPeriodic(mainFace.Surface, mainFace.Domain, c2d2);
                            Edge overlappingEdge = new Edge(oppositeFace, edge.Curve3D.Clone(), oppositeFace, c2d1, forward,
                                mainFace, c2d2, !forward);
                            overlappingEdge.UseVertices(edge.Vertex1, edge.Vertex2);
                            if (!faceToIntersectionEdges.ContainsKey(oppositeFace)) faceToIntersectionEdges[oppositeFace] = new HashSet<Edge>();
                            faceToIntersectionEdges[oppositeFace].Add(overlappingEdge);
                            if (!faceToIntersectionEdges.ContainsKey(mainFace)) faceToIntersectionEdges[mainFace] = new HashSet<Edge>();
                            faceToIntersectionEdges[mainFace].Add(overlappingEdge);
                            intersectionFound = true;
                            // in most cases, the overlapping edge exists already in the mainFace, but duplicates are no problem
                        }
                    }
                    if (!intersectionFound)
                    {
                        if (oppositeFace.Contains(mainFace.Surface.PointAt(mainFace.Area.GetSomeInnerPoint()), true))
                        {
                            totalyCoveredByOppositeFace.Add(mainFace);
                        }
                    }
                }
            }

            // now deal with overlapping faces (same orientation):
            // adding the common part is done by creating a new common face from the intersection edges
            // it will be added to faceToIntersectionEdges and in Result(), the intersection edges will become the outline edges of the common face
            Dictionary<(Face, Face), Face> commonFaces = new Dictionary<(Face, Face), Face>(new UnorderedPairComparer<Face>());
            foreach (var kv in faceToOverlappingFaces)
            {
                Face mainFace = kv.Key;
                foreach (var otherFace in kv.Value.Keys)
                {
                    // create a new common face, which has no outline edges, but only the intersection edges
                    // or maybe it already exists, there is one for each pair of overlapping faces
                    // this will be reached twice for each common face
                    bool commonFaceHasMainSurface;
                    Face commonFace;
                    if (commonFaces.ContainsKey((mainFace, otherFace)))
                    {
                        commonFace = commonFaces[(mainFace, otherFace)];
                        commonFaceHasMainSurface = false;
                    }
                    else
                    {   // create a new face. The outline of the new face is the rectangular domain of the main face. The outline will not be used later, but is is needed for the face
                        // in some situations.
                        BoundingRect ext = mainFace.Domain;
                        ext.InflateRelativeToSize(1.1);
                        commonFace = Face.MakeFace(mainFace.Surface.Clone(), ext);
                        commonFaces[(mainFace, otherFace)] = commonFace;
                        commonFaceHasMainSurface = true;
                    }
                    ModOp2D firstToSecond = kv.Value[otherFace];
                    foreach (Edge edge in otherFace.Edges)
                    {
                        if (mainFace.Contains(edge.Curve3D.PointAt(0.5), true)) // the edge lies on the main face
                        {   // this edge, which is totally inside the main face (because it is already clipped) or identical with an edge of the main face,
                            // is dealt as an intersection edge of the common face. The new edge (commonEdge) will only have a PrimaryFace
                            Edge commonEdge;
                            ICurve2D curveOnPrimaryFace;
                            if (commonFaceHasMainSurface)
                            {// the common face has the surface of the main face
                                if (firstToSecond.IsNull)
                                {
                                    curveOnPrimaryFace = mainFace.Surface.GetProjectedCurve(edge.Curve3D, precision);
                                    if (!edge.Forward(otherFace)) curveOnPrimaryFace.Reverse();
                                }
                                else curveOnPrimaryFace = edge.Curve2D(otherFace).GetModified(firstToSecond.GetInverse());
                                SurfaceHelper.AdjustPeriodic(commonFace.Surface, commonFace.Domain, curveOnPrimaryFace);
                                commonEdge = new Edge(commonFace, edge.Curve3D.Clone(), commonFace, curveOnPrimaryFace, edge.Forward(otherFace));
                            }
                            else
                            { // the common face has the surface of the other face
                                if (firstToSecond.IsNull)
                                {
                                    curveOnPrimaryFace = otherFace.Surface.GetProjectedCurve(edge.Curve3D, precision);
                                    if (!edge.Forward(otherFace)) curveOnPrimaryFace.Reverse();
                                }
                                else curveOnPrimaryFace = edge.Curve2D(otherFace).Clone();
                                SurfaceHelper.AdjustPeriodic(commonFace.Surface, commonFace.Domain, curveOnPrimaryFace);
                                commonEdge = new Edge(commonFace, edge.Curve3D.Clone(), commonFace, curveOnPrimaryFace, edge.Forward(otherFace));
                            }
                            ICurve dbg = commonFace.Surface.Make3dCurve(curveOnPrimaryFace);
                            commonEdge.UseVertices(edge.Vertex1, edge.Vertex2);
                            if (!faceToIntersectionEdges.ContainsKey(commonFace)) faceToIntersectionEdges[commonFace] = new HashSet<Edge>();
                            faceToIntersectionEdges[commonFace].Add(commonEdge);
                        }
                    }
                }
            }
            commonOverlappingFaces = commonFaces.Values.ToHashSet();
        }

        /// <summary>
        /// Adds the vertices of the shell to verticesOctTree, merging them with vertices which are already there at the same position.
        /// </summary>
        private void AddToVertexOctTree(Shell shell)
        {
            Vertex[] vertices = shell.Vertices;
            foreach (var v in vertices)
            {
                var existing = verticesOctTree.GetObjectsCloseTo(v).FirstOrDefault(c => (c.Position | v.Position) < precision);
                if (existing != null) existing.MergeWith(v);
                else verticesOctTree.AddObject(v);
            }
        }
        /// <summary>
        /// Returns the vertex at <paramref name="p"/> (closer than <see cref="precision"/>) from verticesOctTree, or creates a new one there.
        /// This ensures that an intersection point which is found from different face pairs results in a single vertex.
        /// </summary>
        private Vertex CreateOrFindVertex(GeoPoint p)
        {
            Vertex[] close = verticesOctTree.GetObjectsFromPoint(p);
            for (int i = 0; i < close.Length; i++)
            {
                if ((close[i].Position | p) < precision) return close[i];
            }
            Vertex v = new Vertex(p);
            verticesOctTree.AddObject(v);
            return v;
        }

        /// <summary>
        /// Splits a shell by a plane. The plane is represented by a single face, big enough to exceed the shell, and subtracted
        /// from the shell twice: once as it is oriented and once reversed.
        /// </summary>
        /// <param name="shell">the (closed) shell to split, it remains unchanged</param>
        /// <param name="pln">the plane to split by</param>
        /// <returns>lowerPart: the result of subtracting the plane face as oriented by <paramref name="pln"/>,
        /// upperPart: the result of subtracting the reversed plane face.
        /// Note that <see cref="SplitByFace"/> assigns the two results the other way round.</returns>
        public static (Shell[] upperPart, Shell[] lowerPart) SplitByPlane(Shell shell, Plane pln)
        {
            Shell shellToSplit = shell;
            BoundingBox ext = shellToSplit.GetExtent(0.0);
            GeoPoint2D cnt2d = pln.Project(ext.GetCenter());
            BoundingRect br = new BoundingRect(cnt2d, 2 * ext.DiagonalLength, 2 * ext.DiagonalLength);
            Face fcpl = Face.MakeFace(new PlaneSurface(pln), br);
            Face fcpl1 = fcpl.Clone() as Face;
            fcpl1.MakeInverseOrientation();
            Shell splittingShell = Shell.MakeShell([fcpl], false); // the plane as a shell with only one face, which exceeds the solid
            BooleanOperation bo = new BooleanOperation();
            bo.SetShells(shellToSplit, splittingShell, Operation.difference);
            Shell[] lp = bo.Execute();
            bo = new BooleanOperation();
            splittingShell = Shell.MakeShell([fcpl1], false); // the same plane but inversed
            bo.SetShells(shellToSplit, splittingShell, Operation.difference);
            Shell[] up = bo.Execute();
            return (up, lp);
        }
        /// <summary>
        /// Splits a shell by a face, which must cut the shell completely (it must not end inside the shell). The face is subtracted
        /// from the shell twice, once as it is oriented and once reversed, as an open shell.
        /// </summary>
        /// <param name="toSplit">the (closed) shell to split, it remains unchanged</param>
        /// <param name="splitBy">the face to split by</param>
        /// <returns>upperPart: the result of subtracting <paramref name="splitBy"/> as it is oriented,
        /// lowerPart: the result of subtracting the reversed face</returns>
        public static (Shell[] upperPart, Shell[] lowerPart) SplitByFace(Shell toSplit, Face splitBy)
        {
            BooleanOperation bo = new BooleanOperation();
            bo.SetShells(toSplit, Shell.FromFaces(splitBy), Operation.difference);
            bo.SetClosedShells(true, false);
            Shell[] upper = bo.Execute();
            Face reversed = splitBy.Clone() as Face;
            reversed.ReverseOrientation();
            bo = new BooleanOperation();
            bo.SetShells(toSplit, Shell.FromFaces(reversed), Operation.difference);
            bo.SetClosedShells(true, false);
            Shell[] lower = bo.Execute();
            return (upper, lower);

        }
        /// <summary>
        /// Splits a solid by an open shell, which must cut the solid completely. Uses only the first shell of the solid.
        /// </summary>
        /// <param name="solidToSplit">the solid to split</param>
        /// <param name="splitBy">the open shell to split by</param>
        /// <param name="onlyInnerParts">if true, only the result of subtracting <paramref name="splitBy"/> as it is oriented, otherwise
        /// also the result of subtracting the reversed shell, i.e. the parts on both sides</param>
        /// <returns>the parts as solids</returns>
        public static Solid[] SplitSolidByShell(Solid solidToSplit, Shell splitBy, bool onlyInnerParts = true)
        {
            Shell shellToSplit = solidToSplit.Shells[0];
            List<Solid> res = [];
            BooleanOperation bo = new BooleanOperation();
            bo.SetShells(shellToSplit, splitBy, Operation.difference);
            bo.SetClosedShells(true, false);
            Shell[] upper = bo.Execute();
            for (int i = 0; i < upper.Length; i++)
            {
                res.Add(Solid.MakeSolid(upper[i]));
            }
            if (!onlyInnerParts)
            {
                Shell reversed = splitBy.Clone() as Shell;
                reversed.ReverseOrientation();
                bo = new BooleanOperation();
                bo.SetShells(shellToSplit, reversed, Operation.difference);
                bo.SetClosedShells(true, false);
                Shell[] lower = bo.Execute();
                for (int i = 0; i < lower.Length; i++)
                {
                    res.Add(Solid.MakeSolid(lower[i]));
                }
            }
            return res.ToArray();

        }
        /// <summary>
        /// Accept result shells with open edges. Without this, <see cref="Result"/> only returns closed shells. Used for
        /// <see cref="ClipFace(Face, Face)"/> and for connecting faces which don't build a closed shell.
        /// </summary>
        public bool AllowOpenEdges
        {
            set => allowOpenEdges = value;
        }

        /// <summary>
        /// Return only the (trimmed) faces of the first shell, none of the second shell. Used for <see cref="ClipFace(Face, Face)"/>.
        /// </summary>
        public bool DontReturnShell2
        {
            set => dontReturnShell2 = value;
        }
        /// <summary>
        /// Don't merge connected faces with the same surface in the result shells (<see cref="Shell.CombineConnectedFaces"/>).
        /// </summary>
        public bool DontCombineConnectedFaces
        {
            set => dontCombineConnectedFaces = value;
        }
#if DEBUG
        public DebuggerContainer dbgFaceHashCodes
        {
            get
            {
                DebuggerContainer res = new DebuggerContainer();
                foreach (Face face in shell1.Faces)
                {
                    res.Add(face, face.GetHashCode());
                }
                foreach (Face face in shell2.Faces)
                {
                    res.Add(face, face.GetHashCode());
                }
                return res;
            }
        }
#endif

        /// <summary>
        /// The loops found on a face, sorted by their signed area in the 2d space of the face, biggest first: outlines (counterclockwise)
        /// have a positive area, holes (clockwise) a negative one. Loops without area are ignored.
        /// </summary>
        private class LoopCollection : SortedDictionary<double, (List<Edge>, ICurve2D[])>
        {
            private class CompareReverse : IComparer<double>
            {
                int IComparer<double>.Compare(double x, double y)
                {
                    return -x.CompareTo(y); ;
                }
            }
            public LoopCollection() : base(new CompareReverse()) { }
            public void AddUnique(double d, (List<Edge>, ICurve2D[]) val)
            {
                while (this.ContainsKey(d)) d = Geometry.NextDouble(d);
                Add(d, val);
            }
            public void AddUnique(List<Edge> loop, Face onThisFace)
            {
                ICurve2D[] loop2d = onThisFace.Get2DCurves(loop);
                double a = Border.SignedArea(loop2d);
                // ignoring loos with no area. Maybe we ned a better test here
                // there was a case with a line going forward and two lines exactely comming back
                if (Math.Abs(a) > 1e-12) AddUnique(a, (loop, loop2d));
            }
        }

        /// <summary>
        /// A set of edges identified by their two vertices and their geometry (two edges are equal, if they connect the same vertices
        /// and the midpoint of one lies on the other).
        /// </summary>
        private class VertexConnectionSet
        {
            private Dictionary<DoubleVertexKey, ICurve> set = new Dictionary<DoubleVertexKey, ICurve>();
            public void Add(Edge edge)
            {
                set[new DoubleVertexKey(edge.Vertex1, edge.Vertex2)] = edge.Curve3D;
            }
            public bool Contains(Edge edge, double precision)
            {
                if (set.TryGetValue(new DoubleVertexKey(edge.Vertex1, edge.Vertex2), out ICurve crv))
                {
                    if (crv.DistanceTo(edge.Curve3D.PointAt(0.5)) < 10 * precision) return true;
                }
                return false;
            }
            public bool ContainsAll(IEnumerable<Edge> edges, double precision)
            {
                foreach (Edge edg in edges)
                {
                    if (!Contains(edg, precision)) return false;
                }
                return true;
            }
        }
        /// <summary>
        /// After <see cref="Execute"/>: true, if no face of the two shells has been intersected or connected.
        /// </summary>
        public bool Unchanged => shellsAreUnchanged;
        /// <summary>
        /// After <see cref="Execute"/> in the <see cref="SetFaces"/> mode: the faces which have not been discarded.
        /// </summary>
        public List<Face> UnusedFaces => unusedFaces;
        /// <summary>
        /// Maps the faces of the shells passed to <see cref="SetShells"/> to the clones this operation works on.
        /// </summary>
        public Dictionary<Face, Face> OriginalToClonedFaces => originalToClonedFaces;
        /// <summary>
        /// After <see cref="Execute"/>: maps each cloned face which has been split to the new faces made from it.
        /// Together with <see cref="OriginalToClonedFaces"/> this tells which faces of the result come from which original face.
        /// </summary>
        public Dictionary<Face, List<Face>> SplittedFaces => splittedFaces;

        /// <summary>
        /// The second part of <see cref="Execute"/>, which calls this method: it relies on the intersection edges (faceToIntersectionEdges)
        /// and the overlapping faces found before, so calling it without <see cref="Execute"/> makes no sense.
        /// <list type="number">
        /// <item>Each face with intersection edges is trimmed: its outline, its holes and the intersection edges build a graph in the 2d
        /// space of the face, the loops of this graph (<see cref="FindLoops"/>) are the outlines and holes of the new faces.
        /// Identical edges with opposite orientation cancel each other.</item>
        /// <item>Starting with the trimmed faces, all faces which are connected by correctly oriented edges are collected. Faces which
        /// have been trimmed or which are covered by an opposite face are excluded. This decides which faces belong to the result,
        /// only by topology, without any geometric test.</item>
        /// <item>The collected faces are assembled into shells. Untouched parts of the two shells are added, if the material behind them
        /// is inside the other shell (<see cref="IsInsideShell(IEnumerable{Face}, Shell)"/>).</item>
        /// <item>The orientation is restored (union), inward oriented shells become cavities of the shell which contains them, shells
        /// with a negative volume are removed.</item>
        /// </list>
        /// </summary>
        /// <returns>the resulting shells, outward oriented</returns>
        public Shell[] Result()
        {
            // this method relies on
            // - faceToIntersectionEdges: contains all faces, which intersect with other faces as keys and all the intersection edges, 
            //   which are produced by other faces on this face as values.
            // - faceToOverlappingFaces and faceToOppositeFaces: they contain pairs of faces, which overlap, with either same or oppostie orientation.
            // 
            // The main algorithm does the following: split (or trimm or cut) a face according to the intersection edges on this face. 
            // All edges are oriented, so it is possible to find outer edges and holes. A face might produce zero, one or multiple trimmed faces.
            // When all faces are trimmed, connect the trimmed faces with the untouched faces. this is the result.
            //
            // When there are overlapping faces, cut them into non-overlapping parts before the main algorithm.
            //
            // This algorithm does not rely on geometry (coordinates of points and curves) but only on topological information, excetp for
            // - finding, whether two edges connecting the same vertices, are equal (SameEdge) and
            // - checking, whether a 2d loop of edges is inside another 2d loop of edges


#if DEBUG   // show the starting position: dcFaces: all faces with hashCodes of both shells and their normals
            // dcs1e and dcs2e: the 3d edges and their hashCodes
            // dcis: all intersection edges. Here the edges must build one ore more closed curves. there can not be open ends.
            // If there are open ends, some intersection calculation failed!
            DebuggerContainer dcFaces = new CADability.DebuggerContainer();
            if (multipleFaces != null)
            {
                foreach (Face fce in multipleFaces)
                {
                    dcFaces.Add(fce.Clone(), fce.GetHashCode()); // the faces will be destroyed, so keep clones here
                    double ll = fce.GetExtent(0.0).Size * 0.01;
                    ColorDef cd = new ColorDef("debug", Color.Blue);
                    SimpleShape ss = fce.Area;
                    GeoPoint2D c = ss.GetExtent().GetCenter();
                    GeoPoint pc = fce.Surface.PointAt(c);
                    GeoVector nc = fce.Surface.GetNormal(c);
                    Line l = Line.TwoPoints(pc, pc + ll * nc.Normalized);
                    l.ColorDef = cd;
                    dcFaces.Add(l);
                }
            }
            else
            {
                foreach (Face fce in shell1.Faces)
                {
                    dcFaces.Add(fce.Clone(), fce.GetHashCode()); // the faces will be destroyed, so keep clones here
                    double ll = fce.GetExtent(0.0).Size * 0.01;
                    ColorDef cd = new ColorDef("debug", Color.Blue);
                    SimpleShape ss = fce.Area;
                    GeoPoint2D c = ss.GetExtent().GetCenter();
                    GeoPoint pc = fce.Surface.PointAt(c);
                    GeoVector nc = fce.Surface.GetNormal(c);
                    Line l = Line.TwoPoints(pc, pc + ll * nc.Normalized);
                    l.ColorDef = cd;
                    dcFaces.Add(l);
                }
                foreach (Face fce in shell2.Faces)
                {
                    dcFaces.Add(fce.Clone(), fce.GetHashCode()); // use clones, because the faces might be destroyed in course of this routine
                    double ll = fce.GetExtent(0.0).Size * 0.01;
                    ColorDef cd = new ColorDef("debug", Color.Brown);
                    SimpleShape ss = fce.Area;
                    GeoPoint2D c = ss.GetExtent().GetCenter();
                    GeoPoint pc = fce.Surface.PointAt(c);
                    GeoVector nc = fce.Surface.GetNormal(c);
                    Line l = Line.TwoPoints(pc, pc + ll * nc.Normalized);
                    l.ColorDef = cd;
                    dcFaces.Add(l);
                }
            }
            DebuggerContainer dcs1e = new CADability.DebuggerContainer();
            DebuggerContainer dcs2e = new CADability.DebuggerContainer();
            if (multipleFaces != null)
            {
                foreach (Face face in multipleFaces)
                {
                    foreach (Edge edg in face.Edges)
                    {
                        if (edg.Curve3D != null) dcs1e.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
                    }
                }
            }
            else
            {
                foreach (Edge edg in shell1.Edges)
                {
                    if (edg.Curve3D != null) dcs1e.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
                }
                foreach (Edge edg in shell2.Edges)
                {
                    if (edg.Curve3D != null) dcs2e.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
                }
            }
            DebuggerContainer dcis = new CADability.DebuggerContainer(); // <----- dcis shows the intersection curves
            HashSet<Edge> ise = new HashSet<Edge>();
            foreach (KeyValuePair<Face, HashSet<Edge>> item in faceToIntersectionEdges)
            {
                ise.UnionWith(item.Value);
            }
            foreach (Edge edg in ise)
            {
                if (edg.Curve3D != null) dcis.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
            }
#endif
            // we need to retrieve the parts before shell1 and shell2 are beeing destroyed by the intersection process and the edges may be invalid. 
            List<HashSet<Face>> parts1 = null;
            List<HashSet<Face>> parts2 = null;
            HashSet<Edge> edgeOnBothShells = new HashSet<Edge>(); // both shells contain the same edge, this is a non-manifold condition
            Shell shell1Cloned = null; // we need a Clone which keeps the triangulation
            Shell shell2Cloned = null;
            if (shell1 != null && shell2 != null)
            {
                if (shell1 != null) parts1 = shell1.GetConnectedFaceSets();
                if (shell2 != null) parts2 = shell2.GetConnectedFaceSets();
                shell1Cloned = shell1.Clone() as Shell; // we need a Clone which keeps the triangulation
                shell2Cloned = shell2.Clone() as Shell;
                shell1.RecalcEdges();
                foreach (Edge edg in shell1.Edges)
                {
                    IEnumerable<Edge> multipleEdges = Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2);
                    foreach (Edge me in multipleEdges)
                    {
                        if (me == edg) continue;
                        if (shell2.Edges.Contains(me))
                        {
                            if (SameEdge(me, edg, precision))
                            {
                                edgeOnBothShells.Add(me);
                                edgeOnBothShells.Add(edg);
                            }
                        }
                    }
                }
            }

            // if a face is overlapping both in the same orientaiation and the opposite orientation with some other faces
            // there might be intersectionEdges which cross an existing vertex. This is refined here.
            // Maybe this condition also applies to othe faces which are overlapping in the same orientaiation or the opposite orientation, but no case found yet
            Dictionary<Edge, List<Edge>> refinedintersectionEdges = new Dictionary<Edge, List<Edge>>();
            foreach (Face fc in faceToOppositeFaces.Keys.Intersect(faceToOverlappingFaces.Keys))
            {
                if (faceToIntersectionEdges.TryGetValue(fc, out HashSet<Edge> found))
                    splitIntersectionEdges(found, fc, refinedintersectionEdges);
            }
            foreach (Face fc in totalyCoveredByOppositeFace)
            {
                faceToIntersectionEdges.Remove(fc);
            }
            foreach (KeyValuePair<Edge, List<Edge>> kv in refinedintersectionEdges)
            {
                foreach (Face fc in new Face[] { kv.Key.PrimaryFace, kv.Key.SecondaryFace })
                {
                    if (faceToIntersectionEdges.TryGetValue(fc, out HashSet<Edge> edges))
                    {
                        if (edges.Contains(kv.Key))
                        {
                            edges.Remove(kv.Key);
                            edges.UnionWith(kv.Value);
                        }
                    }
                }
            }

            HashSet<Face> discardedFaces = new HashSet<Face>(faceToIntersectionEdges.Keys); // these faces may not appear in the final result, because they will be trimmed
            discardedFaces.UnionWith(totalyCoveredByOppositeFace); // these faces may not appear in the final result, because they are eliminated by their opposite face
            HashSet<Face> trimmedFaces = new HashSet<Face>(); // collection of faces which are trimmed (spitted, cut, edged) during this process
            HashSet<Face> trimmedOverlappingFaces = new HashSet<Face>(); // collection of faces which are the result of overlapping faces
            VertexConnectionSet nonManifoldEdges = new VertexConnectionSet();
            HashSet<Face> nonManifoldCandidates = new HashSet<Face>(); // faces, which are only added because they contain nonManifoldEdges.
            splittedFaces = new Dictionary<Face, List<Face>>(); // maybe the caller wants to know which faces have been splitted into which faces
            foreach (KeyValuePair<Face, HashSet<Edge>> kv in faceToIntersectionEdges)
            {   // faceToIntersectionEdges contains all faces, which are intersected by faces of the relative other shell, as well as those intersection edges
                Face faceToSplit = kv.Key;
                HashSet<Edge> faceEdges = new HashSet<Edge>(faceToSplit.Edges.ToHashSet()); // all outline edges and holes of the face, used edges will be removed
                HashSet<Edge> intersectionEdges = kv.Value.Clone();
                HashSet<Edge> originalEdges = faceToSplit.Edges.ToHashSet();
                HashSet<Vertex> faceVertices = new HashSet<Vertex>(faceToSplit.Vertices);
                if (dontReturnShell2 && faceToSplit.Owner == shell2) continue; // when shell2 is not wanted in the result, skip all its faces

#if DEBUG
                DebuggerContainer dcIntersectingFaces = new DebuggerContainer();
                foreach (Edge edg in intersectionEdges)
                {
                    dcIntersectingFaces.Add(faceToSplit, faceToSplit.GetHashCode());
                }
                DebuggerContainer dcIntEdges2D = new DebuggerContainer();
                foreach (Edge edg in intersectionEdges)
                {
                    dcIntEdges2D.Add(edg.Curve2D(faceToSplit), Color.Blue, edg.GetHashCode());
                    dcIntersectingFaces.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
                }
                foreach (Edge edg in faceToSplit.AllEdges)
                {
                    dcIntEdges2D.Add(edg.Curve2D(faceToSplit), Color.Red, edg.GetHashCode());
                }
#endif
                bool hasNonManifoldEdge = false;
                // some intersection edges are created twice (e.g. when an edge of shell2 is contained in a face of shell1)
                // if the duplicates have the same orientation, discard one of the edges, if they have opposite direction, discard both
                Dictionary<(Vertex, Vertex), Edge> avoidDuplicates = new Dictionary<(Vertex, Vertex), Edge>();
                Dictionary<(Vertex, Vertex), Edge> avoidOriginalEdges = new Dictionary<(Vertex, Vertex), Edge>();
                foreach (Edge edg in faceToSplit.Edges)
                {
                    (Vertex, Vertex) k = (edg.StartVertex(faceToSplit), edg.EndVertex(faceToSplit));
                    avoidOriginalEdges[k] = edg;
                }
                foreach (Edge edg in intersectionEdges.Clone())
                {
                    (Vertex, Vertex) k = (edg.StartVertex(faceToSplit), edg.EndVertex(faceToSplit));
                    (Vertex, Vertex) k1 = (k.Item2, k.Item1);
                    if (avoidDuplicates.ContainsKey(k) && SameEdge(avoidDuplicates[k], edg, precision))
                    {
                        intersectionEdges.Remove(edg); // this is a duplicate edge. It is probably also an original edge
                                                       // it is arbitrary, which intersection edge will be removed.
                        edg.DisconnectFromFace(faceToSplit); // disconnecting is important, because of the vertex->edge->face references
                    }
                    else if (avoidDuplicates.ContainsKey(k1) && SameEdge(avoidDuplicates[k1], edg, precision))
                    {   // Reverse duplicates: remove both intersection edges
                        intersectionEdges.Remove(edg);
                        intersectionEdges.Remove(avoidDuplicates[k1]);
                        edg.DisconnectFromFace(faceToSplit);
                        avoidDuplicates[k1].DisconnectFromFace(faceToSplit);
                        if ((avoidOriginalEdges.ContainsKey(k) && SameEdge(avoidOriginalEdges[k], edg, precision)) ||
                            (avoidOriginalEdges.ContainsKey(k1) && SameEdge(avoidOriginalEdges[k1], edg, precision)))
                        {
                            // two inverse intersection edges are also identical with an original edge:
                            // this will make an ambiguous situation
                            edgeOnBothShells.Add(edg);
                            // hasNonManifoldEdge = true; // excluded for debugging SplitBug1
                        }
                    }
                    else
                    {
                        avoidDuplicates[k] = edg;
                    }
                }
                foreach (Edge edg in intersectionEdges.Clone())
                {
                    (Vertex, Vertex) k = (edg.StartVertex(faceToSplit), edg.EndVertex(faceToSplit));
                    (Vertex, Vertex) k1 = (k.Item2, k.Item1);
                    if (avoidOriginalEdges.ContainsKey(k) && SameEdge(avoidOriginalEdges[k], edg, precision))
                    {
                        intersectionEdges.Remove(edg); // this is an intersection edge identical to an original outline of the face: remove the intersection edge
                        edg.DisconnectFromFace(faceToSplit); // disconnecting is important, because of the vertex->edge->face references
                    }
                    else if (avoidOriginalEdges.ContainsKey(k1) && SameEdge(avoidOriginalEdges[k1], edg, precision))
                    {   // this is an intersection edge invers to an original outline of the face: remove both the intersection edge and the original edge
                        intersectionEdges.Remove(edg);
                        originalEdges.Remove(avoidOriginalEdges[k1]);
                        edg.DisconnectFromFace(faceToSplit);
                    }
                }
                if (commonOverlappingFaces.Contains(faceToSplit)) originalEdges.Clear();
                // when this face is an overlapping face, we do not need the original edges (outline)
                // because they are not part of the result
                // now originalEdges contain all edges of the face, that could be used, intersectionEdges contain all edges that must be used
#if DEBUG       // show the original edges of the faceToSplit (blue) and the intersection edges (red) for this face, where duplicates and reverses are already removed
                // in this 2d display it should be easy to see, which loops should be generated
                double arrowSize = kv.Key.Area.GetExtent().Size * 0.01;
                DebuggerContainer dcedges = new DebuggerContainer();
                dcedges.Add(originalEdges, faceToSplit, arrowSize, Color.Blue, -1);
                dcedges.Add(intersectionEdges, faceToSplit, arrowSize, Color.Red, -1);
                HashSet<Vertex> intVertices = new HashSet<Vertex>();
                foreach (Edge edg in intersectionEdges)
                {
                    dcedges.Add(edg.Vertex1.GetPositionOnFace(faceToSplit), Color.Blue, edg.Vertex1.GetHashCode());
                    dcedges.Add(edg.Vertex2.GetPositionOnFace(faceToSplit), Color.Blue, edg.Vertex2.GetHashCode());
                    intVertices.Add(edg.Vertex1);
                    intVertices.Add(edg.Vertex2);
                }

#endif
                if (intersectionEdges.Count == 0)
                {
                    // all intersection edges are identical or reverse to original edges. If they ware all identical, we can still use this face
                    if (originalEdges.Count == faceToSplit.AllEdgesCount)
                    {
                        if (hasNonManifoldEdge)
                        {
                            trimmedFaces.Add(faceToSplit); // this face must be used, although it might be part of an open shell (hasNonManifoldEdge is very rare)
                            nonManifoldCandidates.Add(faceToSplit); // we dont try to fix open shells with these faces
                        }
                        discardedFaces.Remove(faceToSplit);
                    }
                    continue; // nothing to do here
                }
                LoopCollection loops = new LoopCollection();
                // new simpler method tested in this case. Maybe also works on other operations
                List<List<Edge>> foundLoops = FindLoops(faceToSplit, new HashSet<Edge>(intersectionEdges), new HashSet<Edge>(originalEdges));
                for (int i = 0; i < foundLoops.Count; i++)
                {
                    loops.AddUnique(foundLoops[i], faceToSplit);
                }

#if DEBUG
                DebuggerContainer dcloops = new DebuggerContainer();
                int dbgc = 0;
                foreach ((List<Edge>, ICurve2D[]) item in loops.Values)
                {
                    dcloops.Add(item.Item1, faceToSplit, arrowSize, Color.Blue, ++dbgc);
                }
                if (loops.Count == 0 && !commonOverlappingFaces.Contains(faceToSplit) && !faceToOverlappingFaces.ContainsKey(faceToSplit))
                {
                    System.Diagnostics.Debug.Assert(false, "loops.Count should never be 0");
                }
#endif
                if (loops.Count == 0) continue;
                // when there is a positive loop, which directely contains another positive loop, the outer one has to be removed
                // because otherwise we would create overlapping (and totally covering) trimmed faces.
                // (such loops are created by overlapping faces)
                HashSet<double> keysToRemove = new HashSet<double>();
                foreach (var loop in loops)
                {
                    if (loop.Key < 0) break; // only examine positive loops. Since the loops are ordered, we can stop here
                    KeyValuePair<double, (List<Edge>, ICurve2D[])>? containingLoop = null; // find the smalles containing loop
                    foreach (var item in loops) // look for a loop which contains "loop"
                    {
                        if (Math.Abs(item.Key) <= loop.Key) continue; // too small, cannot be a container, or same loop
                        if (keysToRemove.Contains(item.Key)) continue; // already marked as to remove
                        if (Border.IsInside(item.Value.Item2, loop.Value.Item2[0].PointAt(0.5)) == item.Key > 0) // PointAt(0.5): StartPoint may be coincident with a vertex of the outer loop
                        {   // item contains loop
                            if (containingLoop.HasValue && Math.Abs(containingLoop.Value.Key) < Math.Abs(item.Key)) continue; // we already hav a smaller container, don't want this one
                            containingLoop = item;
                        }
                    }
                    if (containingLoop.HasValue && containingLoop.Value.Key > 0)
                    {   // there is a positive loop directly contained by a positive loop. We need to remove the outer one
                        keysToRemove.Add(containingLoop.Value.Key);
                    }
                }
                // remove these loops (if any, which is rarely the case)
                foreach (double key in keysToRemove)
                {
                    var itemToRemove = loops[key];
                    foreach (Edge edg in itemToRemove.Item1)
                    {   // disconnect also intersection loops in order not to connect this splitted face to the result via these edges
                        if (intersectionEdges.Contains(edg)) edg.RemoveFace(faceToSplit);
                    }
                    loops.Remove(key);
                }
                if (loops.Count == 1 && faceToSplit.AllEdges.ToHashSet().SetEquals(loops.First().Value.Item1))
                {   // this would produce an identical face to faceToSplit. So we don't do anything here but allow the faceToSplit to be used in the result when required
                    discardedFaces.Remove(faceToSplit);
                    continue;
                }

                // Now all necessary loops are created. There is one or more outline (ccw) and zero or more holes
                // If we have more than one outline, we have to match the holes to their enclosing outline
                double[] areas = new double[loops.Count];
                Edge[][] edgeLoop = new Edge[loops.Count][];
                ICurve2D[][] loops2D = new ICurve2D[loops.Count][];
                loops.Keys.CopyTo(areas, 0);
                int ii = 0;
                foreach ((List<Edge>, ICurve2D[]) item in loops.Values)
                {
                    edgeLoop[ii] = item.Item1.ToArray();
                    loops2D[ii] = item.Item2;
                    ++ii;
                }
                // areas is sortet, biggest area first
                int numOutlines = areas.Length;
                for (int i = 0; i < areas.Length; i++)
                {
                    if (areas[i] < 0)
                    {
                        numOutlines = i;
                        break;
                    }
                }
                List<int>[] outlineToHoles = new List<int>[numOutlines]; // for each outline a list (of indices) of the corresponding holes
                for (int i = 0; i < numOutlines; i++)
                {
                    outlineToHoles[i] = new List<int>();
                }
                for (int i = numOutlines; i < areas.Length; i++) // for all holes, begin with the smallest
                {
                    for (int j = numOutlines - 1; j >= 0; --j) // for all outlines, begin with the smallest outline
                    {
                        if (Border.IsInside(loops2D[j], loops2D[i][0].StartPoint))
                        {   // this loop (i) is a potential hole for outline j. But we still need to check for holes covering each other
                            // (holes covering each other only occurs with overlapping faces)
                            bool doAdd = true;
                            for (int k = 0; k < outlineToHoles[j].Count; k++)
                            {   // check the position of hole i to other holes of outline j
                                if (!Border.IsInside(loops2D[outlineToHoles[j][k]], loops2D[i][0].StartPoint)) // since loops2D[..] is a hole, !IsInside means inside the hole
                                {   // the hole we ant to add is contained in another hole of this outline
                                    // actually this never should happen, since we begin with the smalles hole and proceed with the bigger ones
                                    doAdd = false;
                                }
                                else if (!Border.IsInside(loops2D[i], loops2D[outlineToHoles[j][k]][0].StartPoint))
                                {
                                    // this hole i covers another hole of this outline j
                                    outlineToHoles[j][k] = i; // replace the covered hole by this covering hole
                                    doAdd = false;
                                }
                            }
                            if (doAdd) outlineToHoles[j].Add(i); // this hole (i) has found its outline (j)
                            break;
                        }
                    }
                }
#if DEBUG       // show the loops that create the new face(s)
                DebuggerContainer dcLoops = new DebuggerContainer();
                // Show all loops, beginning with biggest loop
                dbgc = 0;
                foreach (var item in loops)
                {
                    foreach (var loop in loops.Values)
                    {
                        dcLoops.Add(loop.Item2, arrowSize, DebuggerContainer.FromInt(dbgc), dbgc);
                    }
                    ++dbgc;
                }
#endif
                // each outline (only one in most cases) creates a new Face. 
                for (int i = 0; i < numOutlines; i++)
                {
                    Face fc = Face.Construct();
                    Edge[][] holes = new Edge[outlineToHoles[i].Count][]; // corresponding list of holes to outline number i
                    for (int j = 0; j < outlineToHoles[i].Count; j++)
                    {
                        holes[j] = edgeLoop[outlineToHoles[i][j]];
                        foreach (Edge edg in holes[j])
                        {
                            edg.ReplaceOrAddFace(faceToSplit, fc);
                        }
                    }
                    foreach (Edge edg in edgeLoop[i])
                    {
                        edg.ReplaceOrAddFace(faceToSplit, fc);
                    }
                    fc.Set(faceToSplit.Surface.Clone(), edgeLoop[i], holes); // we need a clone of the surface because two independent faces shall not have the identical surface
                    fc.CopyAttributes(faceToSplit);
                    if (faceToSplit.Owner != null) fc.UserData["BRepIntersection.IsPartOf"] = faceToSplit.Owner.GetHashCode(); // only hash code here to avoid cloning user data of damaged faces
#if DEBUG
                    fc.AssureTriangles(0.1);
#endif
                    if (commonOverlappingFaces.Contains(faceToSplit)) trimmedOverlappingFaces.Add(fc);
                    else trimmedFaces.Add(fc);
                    if (!splittedFaces.TryGetValue(faceToSplit, out List<Face> newFaces)) splittedFaces[faceToSplit] = newFaces = new List<Face>();
                    newFaces.Add(fc);
                }
            }
            if (dontReturnShell2) discardedFaces.UnionWith(shell2.Faces);

            // Now trimmedFaces contains all faces which are cut by faces of the relative other shell, even those, where the other shell cuts 
            // exactly along existing edges and nothing has been created.
            // The faces, which have been cut, i.e. faceToIntersectionEdges.Keys, are invalid now, we disconnect all edges from these faces
#if DEBUG   // show all trimmed faces
            DebuggerContainer cdTrimmedFaces = new DebuggerContainer();
            foreach (Face fce in trimmedFaces)
            {
                cdTrimmedFaces.Add(fce.Clone(), fce.GetHashCode());
            }
            HashSet<Edge> openTrimmedEdges = new HashSet<Edge>();
            foreach (Face fce in trimmedFaces)
            {
                foreach (Edge edg in fce.AllEdges)
                {
                    if (edg.SecondaryFace == null)
                    {
                        openTrimmedEdges.Add(edg);
                    }
                }
            }
            foreach (Face fce in trimmedFaces)
            {
                Face dbg = fce.Clone() as Face;
                double d = dbg.Area.Area;
            }
#endif
            // find all faces in trimmedFaces, which are identical to trimmedOverlappingFaces
            // these faces are created multiple times and we only need one of them
            HashSet<Face> availableFaces = (multipleFaces != null) ? [.. multipleFaces] : [.. shell1.Faces, .. shell2.Faces];
            availableFaces.ExceptWith(discardedFaces);
            if (trimmedOverlappingFaces.Count > 0) // usually this is empty
            {
                var setComparer = new ImmutableSetComparer<Vertex>(EqualityComparer<Vertex>.Default);
                // remove the identical faces from trimmedFaces, and use the trimmedOverlappingFaces instead
                Dictionary<ImmutableHashSet<Vertex>, List<Face>> trimmedFaceSignatures = new Dictionary<ImmutableHashSet<Vertex>, List<Face>>(setComparer);
                // Create and populate the dictionary: faces with the same vertices are kept in a single list
                foreach (var tfc in trimmedFaces.Union(availableFaces))
                {
                    var key = tfc.Vertices.ToImmutableHashSet(EqualityComparer<Vertex>.Default);
                    if (!trimmedFaceSignatures.TryGetValue(key, out var list)) trimmedFaceSignatures[key] = list = new List<Face>();
                    list.Add(tfc);
                }
                foreach (List<Face> lf in trimmedFaceSignatures.Values)
                {
                    if (lf.Count > 1) // there is more than one: these faces might be identical
                    {
                        for (int i = 1; i < lf.Count; i++) // there are only two, cannot imagine a case with more than two identical faces
                        {
                            // check the case where all vertices of two faces are identical, but the faces are not
                            // eg. two halves of a cylinder
                            if (IdenticalFaces(lf[0], lf[i]))
                            {
                                trimmedFaces.Remove(lf[i]);
                                availableFaces.Remove(lf[i]);
                                discardedFaces.Add(lf[i]);
                            }
                        }
                    }
                }
                // now remove all faces from trimmedFaces, which have the same vertices as a face in trimmedOverlappingFaces
                // and add the faces from trimmedOverlappingFaces instead
                foreach (var ofc in trimmedOverlappingFaces)
                {
                    var key = ofc.Vertices.ToImmutableHashSet(EqualityComparer<Vertex>.Default);
                    if (trimmedFaceSignatures.TryGetValue(key, out var list))
                    {
                        // there is really the case, where a overlapping face has the same vertices as a trimmed face but is still diferent.
                        // So we need to check, whether the overlapping face is really inside the trimmed face
                        // e.g. UniteBug23
                        for (int i = 0; i < list.Count; i++)
                        {
                            GeoPoint somePointOnFace = list[i].Surface.PointAt(list[i].Area.GetSomeInnerPoint());
                            if (ofc.Contains(somePointOnFace, false))
                            {
                                trimmedFaces.Remove(list[i]);
                                discardedFaces.Add(list[i]);
                            }
                        }
                    }
                }
                trimmedFaces.UnionWith(trimmedOverlappingFaces);
            }
            foreach (Face fc in trimmedOverlappingFaces)
            {
                HashSet<Vertex> fcv = new(fc.Vertices);
                foreach (Face fc1 in availableFaces)
                {
                    if (!discardedFaces.Contains(fc1) && fcv.SetEquals(fc1.Vertices))
                    {
                        discardedFaces.Add(fc1);
                        break;
                    }
                }
            }
            // to avoid oppositeCommonFaces to be connected with the trimmedFaces, we destroy these faces

            foreach (Face fce in discardedFaces)
            {   // the 2d outline of a face cannot be computed any more when its edges are disconnected. The geometry of these faces is still
                // needed to decide whether an untouched part of a shell belongs to the result (see "combine shells and holes"), so we force
                // the outline to be cached while the face is still intact
                SimpleShape saveArea = fce.Area;
                foreach (Vertex vtx in fce.Vertices) vtx.RemovePositionOnFace(fce);
                fce.DisconnectAllEdges();
            }

#if DEBUG
            openTrimmedEdges = new HashSet<Edge>();
            foreach (Face fce in trimmedFaces)
            {
                foreach (Edge edg in fce.AllEdges)
                {
                    if (edg.SecondaryFace == null)
                    {
                        openTrimmedEdges.Add(edg);
                    }
                }
            }
#endif
            // All edges of trimmedFaces are connected to either other trimmedfaces or to remaining uncut faces of the two shells.
            // Collect all faces that are reachable from trimmedFaces
            HashSet<Face> allFaces = new HashSet<Face>(trimmedFaces);
            bool added = true;
            bool edgeOnBothShellsAllowed = false;
            HashSet<Edge> alreadyConnectedEdges = new HashSet<Edge>();
            while (added)
            {
                added = false;
                foreach (Face fce in allFaces.Clone()) // use a clone to be able to add faces to allfaces in this foreach loop
                {
                    foreach (Edge edg in fce.Edges)
                    {
                        if (edgeOnBothShells.Contains(edg))
                        {
                            if (!edgeOnBothShellsAllowed) continue;
                            if (alreadyConnectedEdges.Contains(edg)) continue;
                            // this coondition may be too strong,
                            // we need it in TouchingEdgesAndVertices.json, id==30
                            // but it seems somehow inconsistent: we try to find the correct face by another edge
                        }
                        HashSet<Edge> connecting = new HashSet<Edge>(Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2));
                        connecting.Remove(edg);
                        if (!allFaces.Contains(edg.PrimaryFace))
                        {
                            // the following seems to be not necessary, it cannot be used with "UniteBug25.cdb.json"
                            //bool edgeFound = false;
                            //foreach (Edge ce in connecting)
                            //{
                            //    if (allFaces.Contains(ce.PrimaryFace))
                            //    {   // this is probably an overlapping face, which is not connected to the trimmed faces but already belongs to allFaces
                            //        if (SameEdge(ce, edg, precision) && !discardedFaces.Contains(edg.PrimaryFace))
                            //        {
                            //            edgeFound = true;
                            //        }
                            //    }
                            //}
                            if (!discardedFaces.Contains(edg.PrimaryFace) && edg.IsOrientedConnection)
                            {
                                allFaces.Add(edg.PrimaryFace);
                                added = true;
                            }
                            else
                            {
                                discardedFaces.Add(edg.PrimaryFace);
                                edg.DisconnectFromFace(edg.PrimaryFace);
                            }
                        }
                        if (edg.SecondaryFace != null && !allFaces.Contains(edg.SecondaryFace))
                        {
                            //bool edgeFound = false;
                            //foreach (Edge ce in connecting)
                            //{
                            //    if (allFaces.Contains(ce.PrimaryFace))
                            //    {   // this is probably an overlapping face, which is not connected to the trimmed faces but already belongs to allFaces
                            //        if (SameEdge(ce, edg, precision) && !discardedFaces.Contains(edg.SecondaryFace))
                            //        {
                            //            edgeFound = true;
                            //        }
                            //    }
                            //}
                            if (!discardedFaces.Contains(edg.SecondaryFace) && edg.IsOrientedConnection)
                            {
                                allFaces.Add(edg.SecondaryFace);
                                added = true;
                            }
                            else
                            {
                                discardedFaces.Add(edg.SecondaryFace);
                                edg.DisconnectFromFace(edg.SecondaryFace);
                            }
                        }
                        else if (edg.SecondaryFace == null && !nonManifoldEdges.Contains(edg, precision))
                        {
                            if (connecting.Count > 1)
                            {
                                HashSet<Edge> toRemove = new HashSet<Edge>();
                                foreach (Edge ce in connecting)
                                {
                                    if (!SameEdge(ce, edg, precision)) toRemove.Add(ce);
                                    else if (ce.SecondaryFace == null && discardedFaces.Contains(ce.PrimaryFace))
                                    {   // added on 21.05.26: problem with Erdspieß1.cdb.json
                                        toRemove.Add(ce);
                                    }
                                }
                                connecting.ExceptWith(toRemove);
                                foreach (Edge ce in connecting)
                                {
                                    if (ce.SecondaryFace == null && allFaces.Contains(ce.PrimaryFace))
                                    {   // edg is already connected with ce, but two different instances of the edge
                                        edg.MergeWith(ce);
                                        connecting.Clear();
                                        break;
                                    }
                                }
                            }
                            if (connecting.Count == 1) // if we have more than one possibility to connect, there is no criterion to decide which would be thr correct face
                                                       // so we hope to find the correct face with an other path.
                            {
                                Edge con = connecting.First();
                                if (con.SecondaryFace == null && !discardedFaces.Contains(con.PrimaryFace) && SameEdge(con, edg, precision))
                                {   // this is an identical edge, which is not logically connected. This is probably an intersection which coincides with an existing edge.
                                    if (!allFaces.Contains(con.PrimaryFace))
                                    {
                                        allFaces.Add(con.PrimaryFace);
                                        added = true;
                                    }
                                }
                            }
                        }
                    }
                }
                if (!added && !edgeOnBothShellsAllowed && edgeOnBothShells.Any())
                {
                    edgeOnBothShellsAllowed = true;
                    HashSet<Edge> edgesInAllFaces = new HashSet<Edge>();
                    foreach (Face afc in allFaces) edgesInAllFaces.UnionWith(afc.AllEdges);
                    foreach (Edge aedg in edgesInAllFaces)
                    {
                        if (alreadyConnectedEdges.Contains(aedg)) continue;
                        HashSet<Edge> con = Vertex.ConnectingEdges(aedg.Vertex1, aedg.Vertex2).ToHashSet();
                        con.Remove(aedg);
                        con.IntersectWith(edgesInAllFaces);
                        if (con.Count == 1)
                        {
                            if (SameEdge(con.First(), aedg, precision))
                            {
                                alreadyConnectedEdges.Add(aedg);
                                alreadyConnectedEdges.Add(con.First());
                            }
                        }
                    }
                    added = true; // to go more rounds now with those (manifold) edges on both shells
                }
            }

            if (allFaces.Count == 0 && multipleFaces != null)
            {
                // simply connect all faces, there were no intersections
                allFaces.UnionWith(multipleFaces);
            }
            if (allFaces.Count == 0 && allowOpenEdges)
            {
                allFaces.UnionWith(shell1.Faces);
                allFaces.UnionWith(shell2.Faces);
            }
#if DEBUG
            foreach (Face fce in allFaces)
            {
                fce.AssureTriangles(0.1);
                bool ok = fce.CheckConsistency();
            }
#endif

            // the following is probably only necessary when there were overlapping faces:
            // connect open edges in allFaces with each other
            HashSet<Edge> openEdges = new HashSet<Edge>();
            foreach (Face fce in allFaces)
            {
                foreach (Edge edg in fce.Edges)
                {
                    if (edg.SecondaryFace == null && !nonManifoldEdges.Contains(edg, precision)) openEdges.Add(edg);
                }
            }
            ConnectOpenEdges(openEdges);
            // What about "nonManifoldEdges"?
            // 
            List<Shell> res = new List<Shell>(); // the result of this method.
                                                 // allFaces now contains all the trimmed faces plus the faces, which are (directly or indirectly) connected (via edges) to the trimmed faces
            List<Face> nonManifoldParts = new List<Face>();
            shellsAreUnchanged = (allFaces.Count == 0);
            // when allFaces is empty, there were no intersections at all. This case doesn't need a special treatment here: shell1 and shell2
            // are completely untouched and are handled by "combine shells and holes" at the end of this method, which decides for each
            // untouched part of the two shells whether it belongs to the result.
            foreach (Face fc in allFaces)
            {
                string[] allUserDataKeys = fc.UserData.AllItems;
                for (int i = 0; i < allUserDataKeys.Length; i++)
                {
                    if (allUserDataKeys[i].StartsWith("BRep")) fc.UserData.RemoveUserData(allUserDataKeys[i]);
                }
            }
            // vertices in allFaces are still connected to edges of discarded faces. These edges have been disconnected from the discarded faces, but not from the vertices.
            // Remove all edges from the vertices, which are not part of allFaces. This is important, because otherwise we would get a connection to discarded faces via these edges.
            HashSet<Edge> allEdges = new HashSet<Edge>();
            allFaces.SelectMany(fc => fc.Edges).ToList().ForEach(edg => allEdges.Add(edg));
            HashSet<Vertex> allVertices = new HashSet<Vertex>();
            allEdges.SelectMany(edg => new Vertex[] { edg.Vertex1, edg.Vertex2 }).ToList().ForEach(vtx => allVertices.Add(vtx));
            foreach (Vertex vtx in allVertices)
            {
                HashSet<Edge> edgesToRemove = new HashSet<Edge>();
                foreach (Edge edg in vtx.Edges)
                {
                    if (!allEdges.Contains(edg)) edgesToRemove.Add(edg);
                }
                foreach (Edge edg in edgesToRemove) vtx.RemoveEdge(edg);
            }
            while (allFaces.Count > 0)
            {
                HashSet<Face> connected = extractConnectedFaces(allFaces, allFaces.First());
                Shell shell = Shell.MakeShell(connected.ToArray());
#if DEBUG
                // System.Diagnostics.Debug.Assert(shell.CheckConsistency());
#endif
                // res should not have open edges! If so, something went wrong
                if (!allowOpenEdges && shell.HasOpenEdgesExceptPoles())
                {
                    shell.TryConnectOpenEdges(precision);
                }
                if (allowOpenEdges || !shell.HasOpenEdgesExceptPoles())
                {
                    if (!dontCombineConnectedFaces) shell.CombineConnectedFaces(); // two connected faces which have the same surface are merged into one face
                                                                                   // reversing the orientation moved to the end, ater the shells and the holes are combined
#if DEBUG
                    System.Diagnostics.Debug.Assert(shell.CheckConsistency());
                    HashSet<Edge> allEdges1 = shell.Edges.ToHashSet();
                    foreach (Vertex vtx in shell.Vertices)
                    {
                        if (vtx.Edges.Except(allEdges1).Any()) { }
                    }
#endif
                    res.Add(shell);
                }
                else

                {
                    if (nonManifoldEdges.ContainsAll(shell.OpenEdges, precision))
                    {
                        nonManifoldParts.AddRange(shell.Faces);
                        shell = Shell.MakeShell(nonManifoldParts.ToArray());
                        shell.TryConnectOpenEdges(precision);
                        if (!shell.HasOpenEdgesExceptPoles())
                        {
                            if (!dontCombineConnectedFaces) shell.CombineConnectedFaces(); // two connected faces which have the same surface are merged into one face
                                                                                           // this shell is still in the orientation of the intersection, it is reversed at the end together with all other shells.
                                                                                           // For a union the shells have been reversed, so there the enclosed volume has to be negative
                            double volume = shell.Volume(triangulationPrecision);
                            if (operation == Operation.union) volume = -volume;
                            if (volume > Precision.eps * 100) res.Add(shell); // we sometimes get two identical faces, which are inverse oriented
                            nonManifoldParts.Clear();
                        }
                    }
                    else
                    {
                    }
                }
            }
            if (multipleFaces != null)
            {
                unusedFaces = new List<Face>(multipleFaces.Except(discardedFaces));
            }
            // combine shells and holes:
            // The result consists of the shells which have been assembled from the trimmed faces (they are already contained in res) and of
            // those parts of shell1 and shell2 which have not been touched by the intersection at all. A "part" is a connected set of faces
            // (see Shell.GetConnectedFaceSets): a solid with cavities consists of the outer hull and of one part for each cavity, where the
            // normals of a cavity point into the cavity.
            // A part which contains a discarded face or a face which is already used in res has been cut, its relevant faces are already in res.
            // An untouched part belongs to the result, when the material behind it (the side its normals point away from) is inside the other
            // shell. This is the same criterion which is used for the single trimmed faces, only applied to a complete part at once, which is
            // possible because an untouched part cannot cross the other shell.
            // Everything here still happens in the world of the intersection: for a union both shells have been reversed, for a difference only
            // shell2 has been reversed. Since this reversion also applies to the cavities, the rule above is valid for all three operations. The
            // reversion is undone below, when all parts of the result have been collected.
            if (multipleFaces == null && shell1IsClosed && shell2IsClosed && !allowOpenEdges)
            {
                // I think if res is empty, i.e. there were not intersections, we must use both shells in the result. At the ende we have to rejct shells with negative volume
                HashSet<Face> usedFaces = new HashSet<Face>();
                foreach (Shell sh in res) usedFaces.UnionWith(sh.Faces);
                usedFaces.UnionWith(discardedFaces);
                foreach ((Shell shell, List<HashSet<Face>> sparts, Shell other, List<HashSet<Face>> oparts) in new (Shell, List<HashSet<Face>>, Shell, List<HashSet<Face>>)[] { (shell1Cloned, parts1, shell2Cloned, parts2), (shell2Cloned, parts2, shell1Cloned, parts1) })
                {
                    foreach (HashSet<Face> part in sparts)
                    {
                        if (part.Any(fc => usedFaces.Contains(fc))) continue; // this part has been cut, it is already contained in res
                        if (!IsClosedPart(part)) continue; // all parts should be closed
                        if (!IsInsideShell(part, other)) continue; // the material behind this part is not inside the other shell
                        Shell untouchedShell = Shell.MakeShell(part.ToArray());
                        untouchedShell.CombineConnectedFaces();
                        res.Add(untouchedShell);
                    }
                }
            }
            if (res.Count == 0 && discardedFaces.Count > 0)
            {   // there were no intersections, only identical opposite faces, like when glueing two parts together
                // this remains empty in case of intersection and returns the full body in case of union
                if (this.operation == Operation.union || this.operation == Operation.difference)
                {   // for a difference this is the case, when shell2 lies inside shell1 and shares a face with it, like a pocket
                    HashSet<Face> remainingFaces = new HashSet<Face>();
                    remainingFaces.UnionWith(shell1.Faces);
                    remainingFaces.UnionWith(shell2.Faces);
                    remainingFaces.ExceptWith(discardedFaces);
                    Shell connectedShell = Shell.MakeShell(remainingFaces.ToArray(), true);
                    if (!connectedShell.HasOpenEdgesExceptPoles())
                    {
                        connectedShell.CombineConnectedFaces();
                        // two identical shells yield a solid whose cavity is its own outside: it encloses no
                        // volume at all and must not become part of the result
                        if (Math.Abs(connectedShell.Volume(triangulationPrecision)) > triangulationPrecision) res.Add(connectedShell);
                    }
                }
                if (res.Count == 0 && this.operation == Operation.difference)
                {   // this is the case, when shell1 and shell2 are touching with two identical faces. But it is also the case, when shell1
                    // is identical to shell2 (all faces of shell1 are discarded) or lies inside shell2 touching it, then the result is empty.
                    // shell1 remains unchanged only, if its other faces are outside of shell2: shell2 has been reversed, so the material
                    // behind them is inside the reversed shell2. shell2 itself has been destroyed by the intersection, test against its clone.
                    List<Face> remaining = shell1.Faces.Where(fc => !discardedFaces.Contains(fc)).ToList();
                    if (remaining.Count > 0 && shell2Cloned != null && IsInsideShell(remaining, shell2Cloned)) res.Add(shell1Cloned); // shell1 has been destroyed
                }
            }

            for (int i = 0; i < res.Count; i++)
            {
                if (operation == Operation.union || operation == Operation.connectMultipleReversed) res[i].ReverseOrientation(); // both had been reversed and the intersection had been calculated
            }
            if (res.Count > 1)
            {   // an inward oriented shell (it encloses a negative volume) is a cavity and has to be added as a hole to the shell which contains
                // it. An outward oriented shell which resides inside a cavity is not a hole but a separate part of the result.
                List<Shell> combinedShells = new List<Shell>(); // outward oriented shells, enclosing a positive volume
                List<Shell> holes = new List<Shell>(); // inward oriented shells, enclosing a negative volume, i.e. cavities
                foreach (Shell sh in res)
                {   // IsOutwardOriented is the sign of the enclosed volume, but determined by a ray cast. Shell.Volume would be the more direct
                    // criterion, but it triangulates all faces and this cached triangulation would then be used by the consumer of the result
                    if (sh.IsOutwardOriented()) combinedShells.Add(sh); else holes.Add(sh);
                }
                holes.Sort((s1, s2) => s2.GetExtent(0.0).Volume.CompareTo(s1.GetExtent(0.0).Volume)); // sort holes by extent, biggest extent first
                combinedShells.Sort((s1, s2) => s1.Volume(triangulationPrecision).CompareTo(s2.Volume(triangulationPrecision))); // smallest volume first, to find the innermost surrounding shell
                foreach (Shell hole in holes)
                {
                    bool isHole = false;
                    foreach (Shell sh in combinedShells)
                    {
                        if (IsInsideShell(hole.Faces, sh))
                        {
                            sh.AddInnerHole(hole.Faces);
                            isHole = true;
                            break;
                        }
                    }
                    // if a hole is not inside any of the shells, it will be rejected, which is correct
                    //if (!isHole) System.Diagnostics.Debug.Assert(false); // a cavity without a surrounding shell: this should not happen
                }
                combinedShells.Reverse(); // now the biggest is the first one
                for (int i = 0; i < combinedShells.Count - 1; i++)
                {
                    for (int j = i + 1; j < combinedShells.Count; j++)
                    {
                        if (IsInsideShell(combinedShells[j].Faces, combinedShells[i]))
                        {
                            combinedShells.RemoveAt(j);
                            --j; // to also test the following smaller shells
                        }
                    }
                }
                res = combinedShells;
            }
            for (int i = res.Count - 1; i >= 0; --i)
            {
                if (res[i].Volume(triangulationPrecision) < 0) res.RemoveAt(i);
            }
            return res.ToArray();
        }

        /// <summary>
        /// Checks whether the provided faces build a closed part, i.e. each edge is used by two faces of this part. Poles (edges which start and
        /// end at the same vertex) don't need a second face.
        /// </summary>
        /// <param name="part">A connected set of faces</param>
        /// <returns>true, if the part is closed</returns>
        private static bool IsClosedPart(HashSet<Face> part)
        {
            foreach (Face fc in part)
            {
                foreach (Edge edg in fc.AllEdges)
                {
                    if (edg.SecondaryFace == null)
                    {
                        if (edg.Vertex1 == edg.Vertex2) continue; // a pole
                        return false;
                    }
                    if (!part.Contains(edg.PrimaryFace) || !part.Contains(edg.SecondaryFace)) return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Checks whether the provided faces are inside the provided shell, i.e. on the material side of its faces. The faces must not
        /// intersect the shell, then a point on them is unambiguously inside or outside and there is no need to move the test point onto
        /// the material side. The faces are tested one after the other until one of them yields a decision, a tangential ray or a ray
        /// through an edge of the shell yields no decision.
        /// </summary>
        /// <param name="faces">Faces which don't intersect <paramref name="shell"/></param>
        /// <param name="shell">The shell to test against, its orientation defines what "inside" means</param>
        /// <returns>true, if the faces are inside the shell</returns>
        private static bool IsInsideShell(IEnumerable<Face> faces, Shell shell)
        {
            foreach (Face fc in faces)
            {
                GeoPoint2D uv = fc.Area.GetSomeInnerPoint();
                bool inside = IsInsideShell(fc.Surface.PointAt(uv), fc.Surface.GetNormal(uv), shell, out bool decided);
                if (decided) return inside;
            }
            return false;
        }

        /// <summary>
        /// Checks whether the provided point is inside the provided shell, i.e. on the material side of its faces. A ray is cast from the point
        /// and the first intersection ahead decides: when the ray leaves the material there, the point was inside. Unlike
        /// <see cref="Shell.Contains(GeoPoint)"/> this also works for shells with cavities and for inverted shells, where the material extends
        /// to infinity and a ray may leave without any intersection.
        /// </summary>
        /// <param name="point">The point to test, it must not be located on the shell</param>
        /// <param name="direction">The direction of the ray</param>
        /// <param name="shell">The shell to test against, its orientation defines what "inside" means</param>
        /// <param name="decided">false, if the ray is tangential to the shell or hits an edge, then the result is meaningless</param>
        /// <returns>true, if the point is inside the shell</returns>
        private static bool IsInsideShell(GeoPoint point, GeoVector direction, Shell shell, out bool decided)
        {
            List<(double par, bool outward)> ip = shell.GetOrientedLineIntersection(point, direction.Normalized, out bool isBoundaryCase);
            decided = !isBoundaryCase;
            if (isBoundaryCase) return false;
            ip.Sort((a, b) => a.par.CompareTo(b.par));
            for (int i = 0; i < ip.Count; i++)
            {   // at the first intersection ahead of the point the ray either leaves the material (then the point was inside) or enters it
                if (ip[i].par > Precision.eps) return ip[i].outward;
            }
            return !shell.IsOutwardOriented(); // nothing ahead: only an inverted shell has material at infinity
        }

        /// <summary>
        /// Whether the two faces have the same edges with the same orientation: each edge of <paramref name="face2"/> has a counterpart
        /// in <paramref name="face1"/> connecting the same vertices in the same direction.
        /// </summary>
        private bool IdenticalFaces(Face face1, Face face2)
        {
            bool isSameFace = true;
            foreach (Edge edg in face2.AllEdges)
            {
                if (!isSameFace) break;
                bool edgeFound = false;
                foreach (Edge edg1 in Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2))
                {
                    if (edg1.PrimaryFace == face1 || edg1.SecondaryFace == face1)
                    {
                        edgeFound = true;
                        if (edg.StartVertex(face2) != edg1.StartVertex(face1))
                        {// different direction: not the same face
                            isSameFace = false;
                            break;
                        }
                    }
                }
                if (!edgeFound)
                {
                    isSameFace = false;
                    break;
                }
            }
            return isSameFace;
        }

        /// <summary>
        /// Splits intersection edges on <paramref name="faceToSplit"/> which pass through a vertex of another intersection edge, and the
        /// outline edges of the other face at this vertex. Needed for faces which overlap with both same and opposite oriented faces.
        /// </summary>
        /// <param name="intersectionEdges">the intersection edges of the face, the split edges are replaced by their parts</param>
        /// <param name="faceToSplit">the face</param>
        /// <param name="refined">receives for each split edge its parts</param>
        private void splitIntersectionEdges(HashSet<Edge> intersectionEdges, Face faceToSplit, Dictionary<Edge, List<Edge>> refined)
        {   // in some cases there might be intersectionEdges passing through a vertex
            // they have to be splitted at the vertex position
#if DEBUG
            DebuggerContainer dc = new DebuggerContainer();
            foreach (var item in intersectionEdges)
            {
                dc.Add(item.Curve2D(faceToSplit), Color.Red, item.GetHashCode());
                dc.Add(item.Vertex1.GetPositionOnFace(faceToSplit), Color.Blue, item.Vertex1.GetHashCode());
                dc.Add(item.Vertex2.GetPositionOnFace(faceToSplit), Color.Blue, item.Vertex2.GetHashCode());
            }
#endif
            bool found = false;
            do
            {
                found = false;
                foreach (Edge ie1 in intersectionEdges.Clone())
                {
                    foreach (Edge ie2 in intersectionEdges.Clone())
                    {
                        if (ie1 == ie2) continue;
                        foreach (Vertex vtx in new Vertex[] { ie1.Vertex1, ie1.Vertex2 })
                        {
                            if (vtx != ie2.Vertex1 && vtx != ie2.Vertex2)
                            {
                                GeoPoint2D uv = vtx.GetPositionOnFace(faceToSplit);
                                ICurve2D c2d = ie2.Curve2D(faceToSplit);
                                double pos = c2d.PositionOf(uv);
                                if (pos > 1e-6 && pos < 1 - 1e-6 && Precision.IsEqual(uv, c2d.PointAt(pos)))
                                {   // we need to split ie2
                                    Edge[] splitted = ie2.Split(ie2.Curve3D.PositionOf(vtx.Position));
                                    if (splitted != null && splitted.Length > 0)
                                    {
                                        for (int i = 0; i < splitted.Length; i++) splitted[i].UseVertices(vtx);
                                        intersectionEdges.Remove(ie2);
                                        intersectionEdges.UnionWith(splitted);
                                        if (!refined.TryGetValue(ie2, out List<Edge> list)) refined[ie2] = list = new List<Edge>();
                                        list.AddRange(splitted);
                                        found = true;
                                        // now this vertex may also be on an edge of faceToSplit, so we need to split this edge as well
                                        // to get proper loops later on
                                        Face otherFace = ie2.OtherFace(faceToSplit);
                                        uv = vtx.GetPositionOnFace(otherFace);
                                        foreach (Edge fe in otherFace.AllEdges.Clone() as Edge[])
                                        {
                                            ICurve2D fc2d = fe.Curve2D(otherFace);
                                            double fpos = fc2d.PositionOf(uv);
                                            if (fpos > 1e-6 && fpos < 1 - 1e-6 && Precision.IsEqual(uv, fc2d.PointAt(fpos)))
                                            {   // we need to split fe
                                                Edge[] fsplitted = fe.Split(fe.Curve3D.PositionOf(vtx.Position));
                                                if (fsplitted != null && fsplitted.Length > 0)
                                                {
                                                    for (int i = 0; i < fsplitted.Length; i++) fsplitted[i].UseVertices(vtx);
                                                }
                                            }
                                        }
                                        break;

                                    }
                                }
                            }
                        }
                        if (found) break;
                    }
                }
            }
            while (found);
        }

        /// <summary>
        /// Find the loops of the provided edges (<paramref name="intersectionEdges"/> and <paramref name="originalEdges"/>) on the face <paramref name="onThisFace"/>.
        /// the edges are connected by their vertices. <paramref name="onThisFace"/> specifies the 2d surface space on which to find the loops.
        /// If there is a branch, always turn left.
        /// </summary>
        /// <param name="onThisFace"></param>
        /// <param name="intersectionEdges"></param>
        /// <param name="originalEdges"></param>
        /// <returns></returns>
        private List<List<Edge>> FindLoops(Face onThisFace, HashSet<Edge> intersectionEdges, HashSet<Edge> originalEdges)
        {
            const double eps = 1e-2; // for similar angles
            List<List<Edge>> found = new List<List<Edge>>();
            HashSet<Edge> availableEdges = new HashSet<Edge>(intersectionEdges.Union(originalEdges));
            // we need a position in the cyle 0..2pi where we can make a cut while not beeing close to any angles of the egdes
            // We choose availableEdges*2+1 intervals, because we have at most availableEdges*2 angles and each angle can kick out one interval
            // so there will always remain one interval where no angle is close to
            bool[] intervals = new bool[availableEdges.Count * 2 + 1]; // initilized with false
            double angleToIntervalFactor = (availableEdges.Count * 2 + 1) / (Math.PI * 2);
            Dictionary<Vertex, List<(Edge edge, double angle, bool outgoing)>> nodes = new Dictionary<Vertex, List<(Edge, double, bool)>>();
            // for each node (connectionpoint of two or more edges) we keep a list of incomming and outgoing edges to this point
            // the edges are sorted clockwise with respect to the node, so when you enter on an index, the naxt index (modlus) goes to the left.
            // typically there are only two or three edges in a node
            Dictionary<Vertex, Vertex> poleVertices = []; // a pole edge has identical start and endvertex. Bute here we nned different vertices
                                                          // so we invent for each pole a duplicate vertex, which is the endvertex of the pole-edge and the startvertex of the next edge.
                                                          // these vertices will only be used as keys in the nodes dictionary, they are not part of the geometry.
            foreach (Edge edge in originalEdges)
            {
                if (edge.Vertex1 == edge.Vertex2)
                {
                    Vertex v = new Vertex(edge.Vertex1.Position);
                    System.Diagnostics.Debug.Assert(edge.Curve3D == null); // this must be a pole
                    poleVertices[edge.Vertex1] = v;
                }
            }
            foreach (Edge edge in availableEdges)
            {
                ICurve2D c2d = edge.Curve2D(onThisFace);
                Vertex sv = edge.StartVertex(onThisFace);
                Vertex ev = edge.EndVertex(onThisFace);
                if (sv == ev && poleVertices.TryGetValue(sv, out Vertex v)) ev = v; // use the duplicate vertex for the end of a pole edge
                else if (poleVertices.TryGetValue(sv, out Vertex poleEnd)) sv = poleEnd; // use the duplicat vertex for an edge which starts at a pole
                if (!nodes.TryGetValue(sv, out List<(Edge edge, double angle, bool outgoing)> list)) nodes[sv] = list = new List<(Edge edge, double angle, bool outgoing)>();
                double a = c2d.StartDirection.Angle % (Math.PI * 2);
                intervals[(int)(angleToIntervalFactor * a) % intervals.Length] = true; // disable this interval
                list.Add((edge, a, true));
                if (!nodes.TryGetValue(ev, out list)) nodes[ev] = list = new List<(Edge edge, double angle, bool outgoing)>();
                a = (-c2d.EndDirection).Angle % (Math.PI * 2);
                intervals[(int)(angleToIntervalFactor * a) % intervals.Length] = true;
                list.Add((edge, a, false));
            }
            double cutCycle = 0.0; // we use this as an offset to the angles, which maps this interval to the 0,2pi position
            int idx = Array.FindIndex(intervals, b => !b);
            if (idx >= 0) cutCycle = 2 * Math.PI - (idx + 0.5) / angleToIntervalFactor;

            foreach (KeyValuePair<Vertex, List<(Edge edge, double angle, bool outgoing)>> node in nodes)
            {
                node.Value.Sort((a, b) =>
                {
                    double aa = (a.angle + cutCycle) % (Math.PI * 2); // avoids the 0..2pi mess
                    double bb = (b.angle + cutCycle) % (Math.PI * 2);
                    return bb.CompareTo(aa);
                }
                );
                bool needToCheck = false;
                if (node.Value.Count > 1)
                {
                    var lastNode = node.Value.Last();
                    foreach ((Edge edge, double angle, bool outgoing) in node.Value)
                    {
                        double aa = (lastNode.angle + cutCycle) % (Math.PI * 2);
                        double bb = (angle + cutCycle) % (Math.PI * 2);
                        // bb must be greater than aa
                        if (bb - aa < eps)
                        {
                            needToCheck = true;
                            break;
                        }
                        lastNode = (edge, angle, outgoing);
                    }
                }
                if (needToCheck) // there are two or more edges with similar angles, we need to check the order by a small probe circle around the node, because the angle is not sufficient to decide the order
                {
                    double radius = double.MaxValue;
                    GeoPoint2D center = node.Key.GetPositionOnFace(onThisFace);
                    foreach ((Edge edge, double angle, bool outgoing) in node.Value)
                    {
                        GeoPoint2D end = outgoing ? edge.Curve2D(onThisFace).EndPoint : edge.Curve2D(onThisFace).StartPoint;
                        double r = (end | center);
                        if (r < radius) radius = r;
                    }
                    radius /= 5;
                    Circle2D probe = new Circle2D(center, radius);
                    for (int i = 0; i < node.Value.Count; i++)
                    {
                        (Edge edge, double angle, bool outgoing) = node.Value[i];
                        GeoPoint2DWithParameter[] ipa = edge.Curve2D(onThisFace).Intersect(probe);
                        if (ipa != null && ipa.Length > 0)
                        {
                            GeoPoint2DWithParameter pa = outgoing ? ipa.Where(x => x.par1 > 0).OrderBy(x => x.par1).FirstOrDefault() :
                                                                    ipa.Where(x => x.par1 < 1).OrderByDescending(x => x.par1).FirstOrDefault();
                            SurfaceHelper.AdjustPeriodic(onThisFace.Surface, onThisFace.Domain, ref pa.p);
                            double a = (pa.p - center).Angle;
                            node.Value[i] = (edge, a, outgoing);
                        }
                    }
                    node.Value.Sort((a, b) =>
                    {
                        double aa = (a.angle + cutCycle) % (Math.PI * 2); // avoids the 0..2pi mess
                        double bb = (b.angle + cutCycle) % (Math.PI * 2);
                        return bb.CompareTo(aa);
                    });

                }
            }
#if DEBUG
            DebuggerContainer dcv = new DebuggerContainer();
            foreach (Vertex vtx in nodes.Keys)
            {
                dcv.Add(vtx.GetPositionOnFace(onThisFace), Color.Red, vtx.GetHashCode());
            }
#endif
            while (availableEdges.Any())
            {
                Edge current = availableEdges.First();
                List<Edge> collecting = new List<Edge>();
                collecting.Add(current);
                availableEdges.Remove(current);
                while (current != null)
                {
                    List<(Edge edge, double angle, bool outgoing)> node = nodes[current.EndVertex(onThisFace)];
                    // if current is a pole, use the pole-endvertex instead
                    if (current.StartVertex(onThisFace) == current.EndVertex(onThisFace) && poleVertices.TryGetValue(current.StartVertex(onThisFace), out Vertex poleEnd)) node = nodes[poleEnd];
                    int i = node.FindIndex(n => n.edge == current); // current is incoming endge into node i
                    if (i < 0)
                    {   // should never happen
                        current = null;
                        break;
                    }
                    Edge nextEdge = node[(i + 1) % node.Count].edge; // the next edge to the left
                    if (!node[(i + 1) % node.Count].outgoing && Math.Abs(node[(i + 1) % node.Count].angle - node[i].angle) < 1e-10
                        && node.Count > 2)
                    {   // exactely reversing edge, check the next one
                        i += 1;
                        nextEdge = node[(i + 1) % node.Count].edge;
                    }
                    if (node[(i + 1) % node.Count].outgoing)
                    {   // outgoing: this is what we need
                        if (collecting[0] == nextEdge)
                        {   // the loop is closed
                            found.Add(collecting); // add closed loop to the result
                            current = null; // start new loop
                            break;
                        }
                        else if (!availableEdges.Contains(nextEdge))
                        {
                            current = null;
                            break;
                        }
                        else
                        {   // add nextEdge to the loop, make it unavailable and continue with current as nextEdge
                            collecting.Add(nextEdge);
                            availableEdges.Remove(nextEdge);
                            current = nextEdge;
                        }
                    }
                    else
                    {   // wrong direction, stop collecting and reject collected
                        current = null; // stop the loop and dismis
                        break;
                    }
#if DEBUG
                    DebuggerContainer dc = new DebuggerContainer();
                    double arrowSize = onThisFace.Domain.Size * 0.05;
                    dc.Add(collecting, onThisFace, arrowSize, Color.Red, -1);
#endif
                }
            }
            return found;
        }


        /// <summary>
        /// Connects pairs of open edges which connect the same vertices and have the same geometry, with opposite orientation, into a
        /// single edge. Removes pole edges of length 0.
        /// </summary>
        internal void ConnectOpenEdges(HashSet<Edge> openEdges)
        {
            foreach (Edge openEdge in openEdges)
            {
                if (openEdge.PrimaryFace == null) continue; // has already been merged
                if (openEdge.Vertex1 == openEdge.Vertex2)
                {
                    if (openEdge.Curve3D != null && openEdge.Curve3D.Length < Precision.eps)
                    {
                        openEdge.PrimaryFace.RemoveEdge(openEdge);
                    }
                    continue; // don't connect poles
                }
                foreach (Edge edg in Vertex.ConnectingEdges(openEdge.Vertex1, openEdge.Vertex2))
                {
                    if (edg == openEdge) continue;
                    if (!openEdges.Contains(edg)) continue;
                    if ((edg.PrimaryFace == openEdge.PrimaryFace || edg.SecondaryFace == openEdge.PrimaryFace) && BooleanOperation.SameEdge(edg, openEdge, Precision.eps))
                    {
                        edg.RemoveFace(openEdge.PrimaryFace);
                        openEdge.PrimaryFace.ReplaceEdge(openEdge, edg);
                        break;
                    }
                    else if (edg.SecondaryFace == null && openEdge.SecondaryFace == null && BooleanOperation.SameEdge(edg, openEdge, Precision.eps))
                    {
                        if (openEdge.StartVertex(openEdge.PrimaryFace) != edg.StartVertex(edg.PrimaryFace))
                        {   // only correct oriented connections
                            openEdge.MergeWith(edg);
                            edg.DisconnectFromFace(openEdge.SecondaryFace);
                            break;
                        }
                    }
                }
            }
        }


        /// <summary>
        /// Whether two edges, which connect the same vertices, have the same geometry (in either direction). Two half circles of the
        /// same circle connecting the same vertices are not the same edge.
        /// </summary>
        internal static bool SameEdge(Edge e1, Edge e2, double precision)
        {   // it is assumed that the two edges connect the same vertices 
            // it is tested whether they have the same geometry (but maybe different directions) 
            // (two half circles may connect the same vertices but are not geometrically identical when they describe differnt parts of the same circle)
            if (e1.Curve3D != null && e2.Curve3D != null)
            {
                if (e1.Curve3D.SameGeometry(e2.Curve3D, precision) && e1.Curve3D.DistanceTo(e2.Curve3D.PointAt(0.5)) < 100 * precision)
                {
                    double pos = e1.Curve3D.PositionOf(e2.Curve3D.PointAt(0.5));
                    return pos > 0 && pos < 1; // to make sure two half arcs or splines of the same circle are not considered the same edge,
                                               // when they connect the same vertices but describe different parts of the circle 
                }
            }
            return false;
        }

#if DEBUG
        internal static HashSet<Face> collectConnected = new HashSet<Face>();
#endif
        /// <summary>
        /// Return all the faces, which are directely or indirectely connected to "startWith" from the set "allFaces"
        /// Also removes the found faces from "allFaces"
        /// </summary>
        /// <param name="allFaces"></param>
        /// <param name="startWith"></param>
        internal static HashSet<Face> extractConnectedFaces(HashSet<Face> allFaces, Face startWith)
        {
#if DEBUG
            collectConnected.Add(startWith);
#endif
            HashSet<Face> result = new HashSet<Face>();
            result.Add(startWith);
            allFaces.Remove(startWith);
            foreach (Edge edge in startWith.Edges)
            {
                if (allFaces.Contains(edge.SecondaryFace) && edge.IsOrientedConnection)
                {
                    result.UnionWith(extractConnectedFaces(allFaces, edge.SecondaryFace));
                }
                if (allFaces.Contains(edge.PrimaryFace) && edge.IsOrientedConnection)
                {
                    result.UnionWith(extractConnectedFaces(allFaces, edge.PrimaryFace));
                }
            }
            return result;
        }


        /// <summary>
        /// Registers <paramref name="v"/> as a point where <paramref name="edge"/> must be split. All additions to
        /// <see cref="edgesToSplit"/> go through here, so a single conditional breakpoint catches them.
        /// </summary>
        private void AddEdgeSplitVertex(Edge edge, Vertex v)
        {
            if (!edgesToSplit.TryGetValue(edge, out HashSet<Vertex> vertices)) edgesToSplit[edge] = vertices = [];
            vertices.Add(v);
        }
        /// <summary>
        /// Splits the original edges at the vertices collected in edgesToSplit. The parts replace the edge in its faces.
        /// </summary>
        private void SplitEdges()
        {
            // Split all edges at the vertices provided in edgesToSplit
            foreach (KeyValuePair<Edge, HashSet<Vertex>> kv in edgesToSplit)
            {
                Edge edge = kv.Key;
                SortedList<double, Vertex> sortedVertices = new SortedList<double, Vertex>();
                // A tolerance in curve parameters, converted from the 3d precision the vertices were merged
                // with. Dividing by the curve length assumes the curve is parameterized at constant speed.
                // Currently unused: the test below is the strict one - a vertex either splits the edge, or it
                // should never have been created. Kept because that is the tolerance to reach for if the strict
                // test ever turns out to be too strict.
                double prec = precision / edge.Curve3D.Length;
                foreach (Vertex v in kv.Value)
                {   // the edge's own end vertices are no split points, they would only produce pieces of length 0
                    if (v == edge.Vertex1 || v == edge.Vertex2) { continue; }
                    double pos = edge.Curve3D.PositionOf(v.Position);
                    if (pos > 0 && pos < 1 && !sortedVertices.ContainsKey(pos))
                    {
                        sortedVertices.Add(pos, v);
                        v.RemoveEdge(edge);
                    }
                    else
                    {
                        System.Diagnostics.Debug.Assert(false, "an intersection point far enough from both ends to get a vertex of its own "
                            + "must also split the edge, but this one has no parameter strictly inside 0..1: "
                            + "CreateOrFindVertex and SplitEdges disagree about the same point");
                    }
                }
                List<double> toRemove = new List<double>();
                double dlast = -1;
                foreach (double d in sortedVertices.Keys)
                {
                    if (d - dlast < 1e-10) toRemove.Add(d);
                    dlast = d;
                }
                for (int i = 0; i < toRemove.Count; i++)
                {
                    sortedVertices.Remove(toRemove[i]);
                }
                if (sortedVertices.Count > 0)
                {
                    Edge[] splitted = edge.Split(sortedVertices, precision);
                    // edge will be replaced in both faces by the splitted parts of the edge
                    edge.Vertex1.RemoveEdge(edge); // eliminate the unsplitted edges also from the vertices
                    edge.Vertex2.RemoveEdge(edge);
                }
            }
        }
        /// <summary>
        /// Known tangential intersections, provided by the caller (rounding and blending edges): an edge of one shell lies in a face
        /// of the other shell, and forward tells the orientation of the intersection edge on that face. Such an intersection can
        /// hardly be calculated, because the surfaces touch tangentially. Keys and values refer to the original (not cloned) objects,
        /// so this must be set after <see cref="SetShells"/>.
        /// </summary>
        public Dictionary<Edge, (Face face, bool forward)> EdgeLiesInFace
        {
            set
            {
                edgeLiesInFace = [];
                foreach (var item in value)
                {
                    if (originalToClonedEdges.TryGetValue(item.Key, out Edge edg) &&
                        originalToClonedFaces.TryGetValue(item.Value.face, out Face fce))
                    {
                        edgeLiesInFace[edg] = (fce, item.Value.forward);
                    }
                }
            }
        }
        /// <summary>
        /// Edges of one shell which start or end in one of the provided faces of the other shell, provided by the caller (rounding
        /// and blending edges). The vertex on the face is then used as intersection vertex instead of a calculated intersection.
        /// Keys and values refer to the original (not cloned) objects, so this must be set after <see cref="SetShells"/>.
        /// </summary>
        public Dictionary<Edge, HashSet<Face>> EdgeEndsInFace
        {
            set
            {
                edgeEndsInFace = [];
                foreach (var item in value)
                {
                    if (originalToClonedEdges.TryGetValue(item.Key, out Edge edg))
                    {
                        HashSet<Face> fces = new HashSet<Face>();
                        foreach (Face f in item.Value)
                        {
                            if (originalToClonedFaces.TryGetValue(f, out Face fc)) fces.Add(fc);
                        }
                        edgeEndsInFace[edg] = fces;
                    }
                }
            }
        }

#if DEBUG
        public Dictionary<Face, GeoObjectList> dbgNewEdges
        {
            get
            {
                Dictionary<Face, GeoObjectList> res = new Dictionary<Face, GeoObjectList>();
                foreach (Face face in shell1.Faces)
                {
                    GeoObjectList list = new GeoObjectList();
                    list.AddRange(face.Area.DebugList); // the existing outline
                    HashSet<Edge> edges;
                    if (faceToIntersectionEdges.TryGetValue(face, out edges))
                    {   // the new intersection edges
                        foreach (Edge edge in edges)
                        {
                            ICurve2D c2d = edge.Curve2D(face);
                            list.Add(c2d.MakeGeoObject(Plane.XYPlane));
                        }
                    }
                    res[face] = list;
                }
                foreach (Face face in shell2.Faces)
                {
                    GeoObjectList list = new GeoObjectList();
                    list.AddRange(face.Area.DebugList); // the existing outline
                    HashSet<Edge> edges;
                    if (faceToIntersectionEdges.TryGetValue(face, out edges))
                    {   // the new intersection edges
                        foreach (Edge edge in edges)
                        {
                            ICurve2D c2d = edge.Curve2D(face);
                            list.Add(c2d.MakeGeoObject(Plane.XYPlane));
                        }
                    }
                    res[face] = list;
                }
                return res;
            }
        }

#endif
        /// <summary>
        /// Declares whether the shells are closed. Both are assumed to be closed by default. An open shell (e.g. a face to split with)
        /// is not repaired before the operation, and untouched parts of the shells are only added to the result, if both are closed.
        /// </summary>
        public void SetClosedShells(bool shell1IsClosed, bool shell2IsClosed)
        {
            this.shell1IsClosed = shell1IsClosed;
            this.shell2IsClosed = shell2IsClosed;
        }

        /// <summary>
        /// The union of two solids. Uses only the first shell of each solid.
        /// </summary>
        /// <returns>the united solid, or null, if the result is not a single solid (e.g. the solids are disjoint)</returns>
        public static Solid Unite(Solid sld1, Solid sld2)
        {
            BooleanOperation operation = new BooleanOperation();
            operation.SetShells(sld1.Shells[0], sld2.Shells[0], BooleanOperation.Operation.union);
            Shell[] res = operation.Execute();
            if (res != null && res.Length == 1)
            {
                return Solid.MakeSolid(res[0]);
            }
            return null;
        }
        /// <summary>
        /// The difference <paramref name="sld1"/> - <paramref name="sld2"/>. Uses only the first shell of each solid.
        /// </summary>
        /// <returns>the resulting solids, or null, if nothing remains</returns>
        public static Solid[] Subtract(Solid sld1, Solid sld2)
        {
            BooleanOperation operation = new BooleanOperation();
            operation.SetShells(sld1.Shells[0], sld2.Shells[0], BooleanOperation.Operation.difference);
            Shell[] res = operation.Execute();
            if (res != null && res.Length > 0)
            {
                Solid[] sres = new Solid[res.Length];
                for (int i = 0; i < res.Length; i++)
                {
                    sres[i] = Solid.MakeSolid(res[i]);
                }
                return sres;
            }
            return null;
        }
    }
}
