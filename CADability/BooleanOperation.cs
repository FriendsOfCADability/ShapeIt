using CADability.Attribute;
using CADability.Curve2D;
using CADability.DebuggerVisualizers;
using CADability.GeoObject;
using CADability.Shapes;
using Microsoft.VisualStudio.DebuggerVisualizers;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Wintellect.PowerCollections;
#if WEBASSEMBLY
using CADability.WebDrawing;
#else
using Color = System.Drawing.Color;
#endif

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

    /// <summary>
    /// 
    /// </summary>
    public class BooleanOperation : OctTree<BRepItem>
    {
        private Shell shell1; // the two shells, which are intersected
        private Shell shell2; // either shell1 and shell2 are set or multipleFaces is set
        private List<Face> multipleFaces; // the faces, which are intersected. 
        private Dictionary<Edge, Edge> originalToClonedEdges; // points from the original edges to the clones we are working on
        private Dictionary<Vertex, Vertex> originalToClonedVertices; // points from the original vertices to the clones we are working on
        private Dictionary<Face, Face> originalToClonedFaces; // points from the original faces to the clones we are working on

        private PlaneSurface splittingOnplane; // when splitting a Shell with a plane, this is the surface
        private bool allowOpenEdges;
        private bool dontCombineConnectedFaces = false;
        private bool shellsAreUnchanged;
        private List<Face> unusedFaces; // faces, which ware not used in the multipleFaces mode

        private OctTree<Face> facesOctTree;
        private OctTree<Vertex> verticesOctTree;
        private HashSet<(Edge, Face)> dontIntersect = new HashSet<(Edge, Face)>(); // dont intersect these pairs of edges and faces

        Dictionary<DoubleFaceKey, ModOp2D> overlappingFaces; // Faces von verschiedenen Shells, die auf der gleichen Surface beruhen und sich überlappen
        Dictionary<DoubleFaceKey, HashSet<Edge>> overlappingEdges; // relevante Kanten auf den overlappingFaces
        Dictionary<DoubleFaceKey, ModOp2D> oppositeFaces; // Faces von verschiedenen Shells, die auf der gleichen Surface beruhen und sich überlappen aber verschieden orientiert sind
        Dictionary<Face, Dictionary<Face, ModOp2D>> faceToOverlappingFaces; // all faces which overlap with this face
        Dictionary<Face, Dictionary<Face, ModOp2D>> faceToOppositeFaces; // all faces which are opposite oriented to this face and the ModOp2D to transform 2d curves from one face to the other
        HashSet<Face> commonOverlappingFaces; // Faces, which have been newly created as common parts of overlapping faces
        HashSet<Face> cancelledfaces; // Faces, which cancel each other, they have the same area but are opposite oriented 
        Dictionary<Face, HashSet<Edge>> faceToIntersectionEdges; // faces of both shells with their intersection edges
        Dictionary<Face, HashSet<Face>> faceToCommonFaces; // faces which have overlapping common parts on them
        Dictionary<Edge, List<Vertex>> edgesToSplit;
        Dictionary<Face, Edge> tangentialEdges = new Dictionary<Face, Edge>(); // these edges are tangential to a face on the other shell
        HashSet<Edge> edgesNotToUse; // these edges are identical to intersection edges, but are original edges. They must not be used when collecting faces
        HashSet<IntersectionVertex> intersectionVertices; // die Mange aller gefundenen Schnittpunkte (mit Rückverweis zu Kante und face)
        Dictionary<DoubleFaceKey, List<IntersectionVertex>> facesToIntersectionVertices; // Faces mit den zugehörigen Schnittpunkt
        Dictionary<Edge, Tuple<Face, Face>> knownIntersections; // already known intersection edges, some open edges when rounding edges are known before and are tangential
        Dictionary<Edge, Edge> intsEdgeToEdgeShell1; // diese IntersectionEdge ist identisch mit dieser kante auf Shell1
        Dictionary<Edge, Edge> intsEdgeToEdgeShell2;
        Dictionary<Edge, Edge> intsEdgeToIntsEdge; // zwei intersectionEdges sind identisch (beide Zuordnungen finden sich hier)

        /// <summary>
        /// Create the new intersection edges and collect them in faceToIntersectionEdges
        /// </summary>
        private void CreateFaceIntersections()
        {
            // 1. Collect all face pairs which share a common leaf in the octtree
            var allPairs = facesOctTree.Leaves
                .SelectMany(leaf =>
                {
                    var inshell1 = leaf.list.Where(fc => fc.Owner == shell1);
                    var inshell2 = leaf.list.Where(fc => fc.Owner == shell2);
                    return inshell1.SelectMany(fc1 => inshell2.Select(fc2 => (fc1, fc2)));
                })
                .Distinct()
                .ToList();

            bool useParallel = false; // zum Debuggen ausschalten, für Speed anschalten
            // 2. Iterate over all pairs and calculate the intersection edges
            if (useParallel)
            {
                allPairs.AsParallel().ForAll(pair => AddFaceFaceIntersection(pair.fc1, pair.fc2));
            }
            else
            {
                foreach (var (f1, f2) in allPairs) AddFaceFaceIntersection(f1, f2);
            }
        }

        private void AddFaceFaceIntersection(Face fc1, Face fc2)
        {
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
                return; // don't intersect overlapping faces
            }

            HashSet<Vertex> intersectionVertices = new HashSet<Vertex>();
            foreach (Edge edge in fc2.Edges)
            {
                if (edge.Curve3D != null) // not a pole
                {
                    intersectionVertices.UnionWith(AddFaceEdgeIntersection(fc1, edge));
                }
            }
            foreach (Edge edge in fc1.Edges)
            {
                if (edge.Curve3D != null) // not a pole
                {
                    intersectionVertices.UnionWith(AddFaceEdgeIntersection(fc2, edge));
                }
            }
            if (intersectionVertices.Count > 1)
            {
                CreateIntersectionEdges(fc1, fc2, intersectionVertices);
            }
            // TODO: check for inner intersections without any edge involved
        }

        private void CreateIntersectionEdges(Face fc1, Face fc2, HashSet<Vertex> intersectionVertices)
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

            // maybe this intersection is a part of a tangential edge
            if ((tangentialEdges.TryGetValue(fc1, out Edge tangentialEdge) && points.All(p => tangentialEdge.Curve3D.DistanceTo(p) < Precision.eps)) ||
                (tangentialEdges.TryGetValue(fc2, out tangentialEdge) && points.All(p => tangentialEdge.Curve3D.DistanceTo(p) < Precision.eps)))
            {   // there is a tangential edge, provided by the caller, and all intersection points are on this edge. So this is already the intersection curve.
                // We will not need to compute the intersection with Surfaces.Intersect
                // map the points onto the tangential edge
                List<double> positions = points.ConvertAll(p => tangentialEdge.Curve3D.PositionOf(p));
                c3ds = new ICurve[] { tangentialEdge.Curve3D.Clone() }; // a single "intersection" curve
                crvsOnSurface1 = new ICurve2D[] { fc1.Surface.GetProjectedCurve(c3ds[0], 0.0) }; // the 2d curve on face1
                crvsOnSurface2 = new ICurve2D[] { fc2.Surface.GetProjectedCurve(c3ds[0], 0.0) }; // the 2d curve on face2
                params3d = new double[1, points.Count]; // the parameters on the tangential edge (often 0 and 1)
                params2dFace1 = new double[1, points.Count]; // the parameters on the 2d curve on face1
                params2dFace2 = new double[1, points.Count]; // the parameters on the 2d curve on face2
                paramsuvsurf1 = new GeoPoint2D[points.Count]; // the uv parameters on surface1
                paramsuvsurf2 = new GeoPoint2D[points.Count]; // the uv parameters on surface2
                for (int i = 0; i < positions.Count; i++) // fill the parameter arrays
                {
                    params3d[0, i] = positions[i];
                    paramsuvsurf1[i] = fc1.Surface.PositionOf(points[i]);
                    paramsuvsurf2[i] = fc2.Surface.PositionOf(points[i]);
                    double dbgdist = fc1.Surface.PointAt(paramsuvsurf1[i]) | fc2.Surface.PointAt(paramsuvsurf2[i]);
                    double dbgdist1 = fc1.Surface.PointAt(paramsuvsurf1[i]) | points[i];
                    double dbgdist2 = fc2.Surface.PointAt(paramsuvsurf2[i]) | points[i];
                    SurfaceHelper.AdjustPeriodic(fc1.Surface, fc1.Domain, ref paramsuvsurf1[i]);
                    SurfaceHelper.AdjustPeriodic(fc2.Surface, fc2.Domain, ref paramsuvsurf2[i]);
                    params2dFace1[0, i] = crvsOnSurface1[0].PositionOf(paramsuvsurf1[i]);
                    params2dFace2[0, i] = crvsOnSurface2[0].PositionOf(paramsuvsurf2[i]);
                }
            }
            if (c3ds != null || Surfaces.Intersect(fc1.Surface, fc1.Area.GetExtent(), fc2.Surface, fc2.Area.GetExtent(), points, out c3ds, out crvsOnSurface1, out crvsOnSurface2, out params3d, out params2dFace1, out params2dFace2, out paramsuvsurf1, out paramsuvsurf2, precision))
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
                    SortedDictionary<double, int> sortedIv = new SortedDictionary<double, int>(); // Indizes nach u aufsteigend sortiert und nur für Kurve i (meist ohnehin nur eine)
                    for (int j = 0; j < points.Count; j++)
                    {
                        if (params3d[i, j] == double.MinValue) continue; // dieser Schnittpunkt liegt auf einer anderen Kurve
                        sortedIv[params3d[i, j]] = j;
                    }
                    // with closed curves, there will be a problem: which parts to use. This must be considered but is not yet.
                    if (c3ds[i].IsClosed)
                    {
                        if (fc1.Area.Contains(crvsOnSurface1[i].StartPoint, false) && fc2.Area.Contains(crvsOnSurface2[i].StartPoint, false))
                        {
                            // der Startpunkt der geschlossenen 2d-Kurven (die ja dem Startpunkt der 3d Kurve entsprechen) gehört zu den gültigen Kurvenabschnitten
                            // also dürfen wir nicht mit dem kleinsten u beginnen, sondern müssen das Kurvenstück berücksichtigen, welches durch den Anfang  geht.
                            // Beachte: die trimm Funktion (sowohl in 2d als auch in 3d) muss berücksichtigen, dass bei geschlossenen Kurven, der 1. index größer sein kann
                            // als der 2. Index. Das bedeutet dann, das Stück, welches über "0" geht soll geliefert werden. Noch überprüfen, ob das überall implementiert ist
                            // hier wird das erste Element aus dem SortedDictionary entfernt und hinten wieder eingefügt
                            var fiter = sortedIv.GetEnumerator(); // so bekommt man das erste, First() scheint es komischerweise nicht zu geben
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

                        ICurve tr = c3ds[i].Clone(); // InterpolatedDualSurfaceCurve muss die Surfaces erahlten
                        tr.Trim(u1, u2);
                        ICurve2D con1 = crvsOnSurface1[i].Trim(params2dFace1[i, j1], params2dFace1[i, j2]);
                        ICurve2D con2 = crvsOnSurface2[i].Trim(params2dFace2[i, j1], params2dFace2[i, j2]);
                        {   // we need this test with the trimmed 3d curve in a strange case: a quarter of an ellipse is exactely outside a threequarter cylinder
                            // this part of a 2d curve gets arbitrarily wrong periodic adjusted
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
                        // hier am Besten aus InterpolatedDualSurfaceCurve BSplines machen, sowohl in 2d, als auch in 3d und ganz am Ende
                        // wieder zu InterpolatedDualSurfaceCurve machen. GGf mit Flag, damit das klar ist
                        // Problme wäre die Genauigkeit, wenn beim BooleanOperation.generateCycles die Richtung genommen wird...
                        if (con1 is InterpolatedDualSurfaceCurve.ProjectedCurve pon1 && con2 is InterpolatedDualSurfaceCurve.ProjectedCurve pon2 &&
                            tr is InterpolatedDualSurfaceCurve idsc)
                        {   // con1 und con2 müssen auf tr verweisen, sonst kann man das Face später nicht mit "ReverseOrientation" umdrehen. Dort wird nämlich die 
                            // surface verändert, und die muss bei allen Kurven die selbe sein
                            pon1.SetCurve3d(idsc);
                            pon2.SetCurve3d(idsc);
                        }
                        // The cross product of the normals specifies the direction of the new edge, no matter where on the curve we compute it.
                        // But if the surfaces are tangential in a point the cross product of the normals will be 0. So we take the better one
                        // if both are bad (e.g. two same diameter cylinders), we take a point in the middle
                        bool dirs1;
                        GeoVector normalsCrossedStart = fc1.Surface.GetNormal(paramsuvsurf1[j1]).Normalized ^ fc2.Surface.GetNormal(paramsuvsurf2[j1]).Normalized;
                        GeoVector normalsCrossedEnd = fc1.Surface.GetNormal(paramsuvsurf1[j2]).Normalized ^ fc2.Surface.GetNormal(paramsuvsurf2[j2]).Normalized;
#if DEBUG
                        Line l1 = Line.MakeLine(fc1.Surface.PointAt(paramsuvsurf1[j2]), fc1.Surface.PointAt(paramsuvsurf1[j2]) + 10 * fc1.Surface.GetNormal(paramsuvsurf1[j2]).Normalized);
                        Line l2 = Line.MakeLine(fc2.Surface.PointAt(paramsuvsurf2[j2]), fc2.Surface.PointAt(paramsuvsurf2[j2]) + 10 * fc2.Surface.GetNormal(paramsuvsurf2[j2]).Normalized);
#endif
                        if (normalsCrossedStart.Length < 100*Precision.eps && normalsCrossedEnd.Length < 100*Precision.eps)
                        {
                            // it seems to be tangential at the endpoints of the intersection curve: test in the middle of the intersection curve
                            GeoPoint m = tr.PointAt(0.5);
                            GeoVector normalsCrossedMiddle = fc1.Surface.GetNormal(fc1.Surface.PositionOf(m)) ^ fc2.Surface.GetNormal(fc2.Surface.PositionOf(m));
                            if (normalsCrossedMiddle.Length < 10 * Precision.eps)
                            {
                                // it is also tangential at the midpoint of the intersection curve
                                // we now use the uv points in the surfaces and slowly walk from the uv point in the direction of the center of the domain
                                // (2d extent), until we find a point, where the normals are not parallel any more.
                                GeoPoint2D uvf1 = fc1.Surface.PositionOf(m);
                                SurfaceHelper.AdjustPeriodic(fc1.Surface, fc1.Domain, ref uvf1);
                                GeoPoint2D uvf2 = fc2.Surface.PositionOf(m);
                                SurfaceHelper.AdjustPeriodic(fc2.Surface, fc2.Domain, ref uvf2);
                                GeoVector2D toCenter1 = fc1.Domain.GetCenter() - uvf1;
                                GeoVector2D toCenter2 = fc2.Domain.GetCenter() - uvf2;
                                // normalis the step vectors to the size of the extent
                                if (Math.Abs(toCenter1.x) > Math.Abs(toCenter1.y)) toCenter1.Length = fc1.Domain.Width * 1e-3; // 1/1000 of the extent
                                else if (Math.Abs(toCenter1.y) > 0) toCenter1.Length = fc1.Domain.Height * 1e-3; // 1/1000 of the extent
                                else toCenter1 = new GeoVector2D(fc1.Domain.Width * 1e-3, fc1.Domain.Height * 1e-3); // was exactely in the center
                                if (Math.Abs(toCenter2.x) > Math.Abs(toCenter2.y)) toCenter2.Length = fc2.Domain.Width * 1e-3; // 1/1000 of the extent
                                else if (Math.Abs(toCenter2.y) > 0) toCenter2.Length = fc2.Domain.Height * 1e-3; // 1/1000 of the extent
                                else toCenter2 = new GeoVector2D(fc2.Domain.Width * 1e-3, fc2.Domain.Height * 1e-3); // was exactely in the center
                                                                                                                     // walk to the center until the normals are no longer parallel
                                while (fc1.Domain.ContainsEps(uvf1, -0.01) && fc2.Domain.ContainsEps(uvf2, -0.01))
                                {
                                    uvf1 += toCenter1;
                                    uvf2 += toCenter2;
                                    GeoVector nn1 = fc1.Surface.GetNormal(uvf1).Normalized;
                                    GeoVector nn2 = fc2.Surface.GetNormal(uvf2).Normalized;
                                    toCenter1 = 2 * toCenter1;
                                    toCenter2 = 2 * toCenter2;
                                    normalsCrossedMiddle = nn1 ^ nn2;
                                    if (normalsCrossedMiddle.Length > 10*Precision.eps) break;
                                }

                                double tangentialPrecision = (fc1.GetExtent(0.0).Size + fc2.GetExtent(0.0).Size) * Precision.eps;
                                // Still ignoring the case where there could be a real intersection e.g. when a surface crosses a plane like the "S" crosses the tangent at the middle
                                // When this intersection curve coincides with an existing edge on one of the faces, we use the combined normalvector of both involved faces
                                HashSet<Edge> existingEdges = new HashSet<Edge>(Vertex.ConnectingEdges(usedVertices[j1], usedVertices[j2]));
                                GeoVector n1 = fc1.Surface.GetNormal(fc1.Surface.PositionOf(m)).Normalized;
                                GeoVector n2 = fc2.Surface.GetNormal(fc2.Surface.PositionOf(m)).Normalized;
                                HashSet<Edge> onFace1 = existingEdges.Intersect(fc1.Edges).ToHashSet();
                                HashSet<Edge> onFace2 = existingEdges.Intersect(fc2.Edges).ToHashSet();

                                if (normalsCrossedMiddle.Length < Precision.eps) //edgFound)
                                {
                                    normalsCrossedMiddle = n1 ^ n2;
                                    if (normalsCrossedMiddle.Length < Precision.eps)
                                    {   // still not able to decide, the connecting face found is also tangential
                                        // now we go a little bit inside on the face with the edge. This is very seldom the case, so no problem making the same iteration once more
                                        foreach (Edge edg in onFace1)
                                        {
                                            if (edg.Curve3D != null && edg.Curve3D.DistanceTo(m) < tangentialPrecision)
                                            {
                                                ICurve2D c2df1 = edg.Curve2D(fc1);
                                                // from the middle of this edge go a small step into the inside of the face and see what the normal is at that point
                                                GeoPoint2D mp = c2df1.PointAt(0.5);
                                                GeoVector2D mdir = c2df1.DirectionAt(0.5).ToLeft().Normalized;
                                                double len = fc1.Area.GetExtent().Size;
                                                Line2D l2d = new Line2D(mp, mp + len * mdir);
                                                double[] parts = fc1.Area.Clip(l2d, true);
                                                if (parts.Length > 1)
                                                {   // there is a point on face1 close to the intersectioncurve, which we can use for the normal
                                                    n1 += fc1.Surface.GetNormal(l2d.PointAt((parts[0] + parts[1]) / 2.0));
                                                }
                                                break;
                                            }
                                        }
                                        foreach (Edge edg in onFace2)
                                        {
                                            if (edg.Curve3D != null && edg.Curve3D.DistanceTo(m) < tangentialPrecision)
                                            {
                                                ICurve2D c2df2 = edg.Curve2D(fc2);
                                                // from the middle of this edge go a small step into the inside of the face and see what the normal is at that point
                                                GeoPoint2D mp = c2df2.PointAt(0.5);
                                                GeoVector2D mdir = c2df2.DirectionAt(0.5).ToLeft().Normalized;
                                                double len = fc2.Area.GetExtent().Size;
                                                Line2D l2d = new Line2D(mp, mp + len * mdir);
                                                double[] parts = fc2.Area.Clip(l2d, true);
                                                if (parts.Length > 1)
                                                {   // there is a point on face2 close to the intersectioncurve, which we can use for the normal
                                                    n2 += fc2.Surface.GetNormal(l2d.PointAt((parts[0] + parts[1]) / 2.0));
                                                }
                                                break;
                                            }
                                        }
                                        normalsCrossedMiddle = n1 ^ n2;
                                    }
                                }
                                // else: it is a inner intersection. With simple surfaces this cannot be a real intersection
                                // but with nurbs surfaces, this could be the case. This still has to be implemented
                                if (normalsCrossedMiddle.Length < Precision.eps) continue;
                            }
                            dirs1 = (normalsCrossedMiddle * tr.DirectionAt(0.5)) > 0;
                        }
                        else if (normalsCrossedStart.Length > normalsCrossedEnd.Length)
                        {
                            dirs1 = (normalsCrossedStart * tr.StartDirection) > 0;
                        }
                        else
                        {
                            dirs1 = (normalsCrossedEnd * tr.EndDirection) > 0;

                        }
                        // bei diesem Skalarprodukt von 2 Vektoren, die entweder die selbe oder die entgegengesetzte Richtung haben ist ">0" unkritisch
                        if (dirs1) con2.Reverse();
                        else con1.Reverse();
                        GeoPoint2D uv1 = con1.PointAt(0.5);
                        GeoPoint2D uv2 = con2.PointAt(0.5);
                        if (!fc1.Contains(ref uv1, true) || !fc2.Contains(ref uv2, true))
                        {   // the condition is too strong. Use the same relaxed condition as above
                            if (fc1.Area.GetPosition(uv1, fc1.Domain.Size * 1e-5) == Border.Position.Outside &&
                                fc2.Area.GetPosition(uv2, fc2.Domain.Size * 1e-5) == Border.Position.Outside) continue;
                        }
                        Edge edge = new Edge(fc1, tr, fc1, con1, dirs1, fc2, con2, !dirs1);
                        //edge.edgeInfo = new EdgeInfo(edge);
                        //edge.edgeInfo.isIntersection = true;
#if DEBUG
                        (tr as IGeoObject).UserData.Add("DebugIntersectionBy1", fc1.GetHashCode());
                        (tr as IGeoObject).UserData.Add("DebugIntersectionBy2", fc2.GetHashCode());
                        if (con2 is InterpolatedDualSurfaceCurve.ProjectedCurve)
                        {
                            BSpline2D dbgbsp2d = (con2 as InterpolatedDualSurfaceCurve.ProjectedCurve).ToBSpline(0.0);
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
                        edge.Vertex1 = usedVertices[j1];    // damit wird diese Kante mit den beiden Schnittvertices verbunden
                        edge.Vertex2 = usedVertices[j2];
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
                                        if (le[k].Vertex1 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.StartDirection, edge.Curve3D.StartDirection, false)) tangential = true;
                                        else if (le[k].Vertex2 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.EndDirection, edge.Curve3D.StartDirection, false)) tangential = true;
                                    }
                                    le = edge.Vertex2.EdgesOnFace(fc1);
                                    for (int k = 0; k < le.Count; k++)
                                    {
                                        if (le[k] == edge) continue;
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
                                        if (le[k] == edge) continue;
                                        if (le[k].Vertex1 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.StartDirection, edge.Curve3D.StartDirection, false)) tangential = true;
                                        else if (le[k].Vertex2 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.EndDirection, edge.Curve3D.StartDirection, false)) tangential = true;
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
                                    addTo = new HashSet<Edge>(); // (new EdgeComparerByVertex()); // damit werden zwei Kanten mit gleichen Vertices nicht zugefügt, nutzt nichts
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

        private IEnumerable<Vertex> AddFaceEdgeIntersection(Face face, Edge edge)
        {
            List<Vertex> res = new List<Vertex>();
            GeoPoint[] ip;
            GeoPoint2D[] uvOnFace;
            double[] uOnCurve3D;
            Border.Position[] position;
            double prec = precision / edge.Curve3D.Length;
            //if (knownIntersections != null && knownIntersections.TryGetValue(ef.edge, out Tuple<Face, Face> ki))
            //{
            //    if (ki.Item1 == ef.face || ki.Item2 == ef.face) continue;
            //}
            if (edge.PrimaryFace == face || edge.SecondaryFace == face) return res; // necessary for multipleFaces
            if (edge.PrimaryFace.Surface.SameGeometry(edge.PrimaryFace.Domain, face.Surface, face.Domain, precision, out ModOp2D _)) return res;
            if (edge.SecondaryFace != null && edge.SecondaryFace.Surface.SameGeometry(edge.SecondaryFace.Domain, face.Surface, face.Domain, precision, out ModOp2D _)) return res;
            face.IntersectAndPosition(edge, out ip, out uvOnFace, out uOnCurve3D, out position, precision);
            for (int i = 0; i < ip.Length; ++i)
            {
                if (uOnCurve3D[i] < -prec || uOnCurve3D[i] > 1.0 + prec || position[i] == Border.Position.Outside)
                {
                    // Die Endpunkte sollen mit erzeugt werden, damit daraus später Schnittkanten entstehen können
                    // Beim Aufteilen der kanten dürfen die Endpunkte allerdings nicht mit verwendet werden
                    continue;
                }
                //if (knownIntersections != null)
                //{   // knownIntersections are often tangential intersections of round fillets. The precision of tangential intersections is often bad,
                //    // so we try to correct those intersection points with the vertices of known intersection edges, so that the connection of intersection edges is kept.
                //    if (Math.Abs(edge.Curve3D.DirectionAt(uOnCurve3D[i]).Normalized * face.Surface.GetNormal(uvOnFace[i]).Normalized) < 1e-3)
                //    {   // a (rather) tangential intersection, maybe we are close to a known intersection. Tangential intersections have a bad precision.
                //        foreach (KeyValuePair<Edge, Tuple<Face, Face>> item in knownIntersections)
                //        {
                //            if ((face == item.Value.Item1 && (edge.PrimaryFace == item.Value.Item2 || edge.SecondaryFace == item.Value.Item2)) ||
                //                (face == item.Value.Item2 && (edge.PrimaryFace == item.Value.Item1 || edge.SecondaryFace == item.Value.Item1)))
                //            {
                //                if ((item.Key.Vertex1.Position | ip[i]) < (item.Key.Vertex2.Position | ip[i]))
                //                {
                //                    if ((item.Key.Vertex1.Position | ip[i]) < prec * 100)
                //                    {
                //                        ip[i] = item.Key.Vertex1.Position;
                //                        uvOnFace[i] = face.PositionOf(ip[i]);
                //                        uOnCurve3D[i] = edge.Curve3D.PositionOf(ip[i]);
                //                    }
                //                }
                //                else
                //                {
                //                    if ((item.Key.Vertex2.Position | ip[i]) < prec * 100)
                //                    {
                //                        ip[i] = item.Key.Vertex2.Position;
                //                        uvOnFace[i] = face.PositionOf(ip[i]);
                //                        uOnCurve3D[i] = edge.Curve3D.PositionOf(ip[i]);
                //                    }
                //                }
                //            }
                //        }
                //    }
                //}
                Vertex v = CreateOrFindVertex(ip[i]);
                v.AddPositionOnFace(face, uvOnFace[i]);
                // not sure whether we still need IntersectionVertex
                if (!edgesToSplit.ContainsKey(edge)) edgesToSplit[edge] = new List<Vertex>();
                edgesToSplit[edge].Add(v);
                res.Add(v);
                // not sure what the following is good for:
                if (position[i] == Border.Position.OnCurve)
                {
                    List<Edge> touchedEdges = face.FindEdges(uvOnFace[i]);
                    for (int j = 0; j < touchedEdges.Count; j++)
                    {
                        if (touchedEdges[j].Curve3D == null) continue; // no poles here
                        double cprec = prec / touchedEdges[j].Curve3D.Length; // darf natürlich nicht 0 sein!
                        double pos = touchedEdges[j].Curve3D.PositionOf(v.Position);
                        if (pos > cprec && pos < 1 - cprec)
                        {
                            if (!edgesToSplit.ContainsKey(touchedEdges[j])) edgesToSplit[touchedEdges[j]] = new List<Vertex>();
                            edgesToSplit[touchedEdges[j]].Add(v);
                        }
                    }
                }
                if (operation == Operation.testonly) return res; // ein Schnittpunkt reicht hier
            }
            return res;
        }

        /// <summary>
        /// Collect intersections between edges of one shell and faces of the other shell (and vice versa)
        /// setting edgesToSplit, intersectionVertices and facesToIntersectionVertices 
        /// </summary>
        private void createEdgeFaceIntersections()
        {
            // find candidates froom the octtree
            Dictionary<EdgeFaceKey, List<Node<BRepItem>>> edgesToFaces = new Dictionary<EdgeFaceKey, List<OctTree<BRepItem>.Node<BRepItem>>>();
            HashSet<Face> faces = new HashSet<Face>();
            foreach (Node<BRepItem> node in Leaves)
            {
                foreach (BRepItem first in node.list)
                {
                    if (first.Type == BRepItem.ItemType.Edge)
                    {
                        Edge edge = first.edge;
                        IGeoObjectOwner shell = edge.PrimaryFace.Owner;
                        foreach (BRepItem second in node.list)
                        {
                            if (second.Type == BRepItem.ItemType.Face)
                            {
                                if (dontIntersect.Contains((edge, second.face))) continue; // this intersection is not needed, it is probably a direct connection between edge and face
                                if (second.face.Owner == null || second.face.Owner != shell) // owner==null when we don't have two shells but many faces (e.g. offset shell)
                                {   // keine Schnitte von Kanten, die ganz im Face liegen
                                    if (faceToOverlappingFaces.TryGetValue(second.face, out var map) &&
                                        (map.ContainsKey(edge.PrimaryFace) || map.ContainsKey(edge.SecondaryFace)))
                                    {
                                        continue;
                                    }
                                    List<Node<BRepItem>> addInto;
                                    EdgeFaceKey efk = new EdgeFaceKey(edge, second.face);
                                    if (!edgesToFaces.TryGetValue(efk, out addInto))
                                    {
                                        addInto = new List<OctTree<BRepItem>.Node<BRepItem>>();
                                        edgesToFaces[efk] = addInto;
                                    }
                                    addInto.Add(node);
                                    faces.Add(second.face);
                                }
                            }
                        }
                    }
                }
            }
            foreach (Face fce in faces)
            {   // es ist ein Fahler, dass man das hier machen muss, aber ich habe den noch nicht gefunden
                fce.ForceAreaRecalc();
            }
            // edgesToFaces enthält jetzt alle schnittverdächtigen Paare
            // und dazu noch die nodes, wenn man Anfangswerte suchen würde...
#if DEBUG
            DebuggerContainer dcedges = new DebuggerContainer();
            DebuggerContainer dcfaces = new DebuggerContainer();
            foreach (EdgeFaceKey ef in edgesToFaces.Keys)
            {
                dcedges.Add(ef.edge.Curve3D as IGeoObject, ef.edge.GetHashCode());
                dcfaces.Add(ef.face, ef.face.GetHashCode());
            }
#endif
            foreach (EdgeFaceKey ef in edgesToFaces.Keys)
            {
                GeoPoint[] ip;
                GeoPoint2D[] uvOnFace;
                double[] uOnCurve3D;
                Border.Position[] position;
                double prec = precision / ef.edge.Curve3D.Length;
                if (knownIntersections != null && knownIntersections.TryGetValue(ef.edge, out Tuple<Face, Face> ki))
                {
                    if (ki.Item1 == ef.face || ki.Item2 == ef.face) continue;
                }
                if (ef.edge.PrimaryFace == ef.face || ef.edge.SecondaryFace == ef.face) continue; // necessary for multipleFaces
                if (ef.edge.PrimaryFace.Surface.SameGeometry(ef.edge.PrimaryFace.Domain, ef.face.Surface, ef.face.Domain, precision, out ModOp2D _)) continue;
                if (ef.edge.SecondaryFace != null && ef.edge.SecondaryFace.Surface.SameGeometry(ef.edge.SecondaryFace.Domain, ef.face.Surface, ef.face.Domain, precision, out ModOp2D _)) continue;
                ef.face.IntersectAndPosition(ef.edge, out ip, out uvOnFace, out uOnCurve3D, out position, precision);
                for (int i = 0; i < ip.Length; ++i)
                {
                    if (uOnCurve3D[i] < -prec || uOnCurve3D[i] > 1.0 + prec || position[i] == Border.Position.Outside)
                    {
                        // Die Endpunkte sollen mit erzeugt werden, damit daraus später Schnittkanten entstehen können
                        // Beim Aufteilen der kanten dürfen die Endpunkte allerdings nicht mit verwendet werden
                        continue;
                    }
                    if (multipleFaces != null)
                    {   // in multiple face mode we do not want the vertics of the edge to intersect a face, which is connected to this vertex
                        // this would leed to many intersection points which are not required
                        // there ist often a tangential intersection at the start or endvertex. In these cases we get imprecise intersection points
                        // and should handle tangential intersections seperately
                        bool tangential = Math.Abs(ef.edge.Curve3D.DirectionAt(uOnCurve3D[i]) * ef.face.Surface.GetNormal(uvOnFace[i])) < 0.01;
                        double maxDist = tangential ? Precision.eps * 1000 : Precision.eps;
                        //if ((ip[i] | ef.edge.Vertex1.Position) < maxDist && ef.edge.Vertex1.InvolvedFaces.Contains(ef.face)) continue;
                        //if ((ip[i] | ef.edge.Vertex2.Position) < maxDist && ef.edge.Vertex2.InvolvedFaces.Contains(ef.face)) continue;
                    }
                    if (knownIntersections != null)
                    {   // knownIntersections are often tangential intersections of round fillets. The precision of tangential intersections is often bad,
                        // so we try to correct those intersection points with the vertices of known intersection edges, so that the connection of intersection edges is kept.
                        if (Math.Abs(ef.edge.Curve3D.DirectionAt(uOnCurve3D[i]).Normalized * ef.face.Surface.GetNormal(uvOnFace[i]).Normalized) < 1e-3)
                        {   // a (rather) tangential intersection, maybe we are close to a known intersection. Tangential intersections have a bad precision.
                            foreach (KeyValuePair<Edge, Tuple<Face, Face>> item in knownIntersections)
                            {
                                if ((ef.face == item.Value.Item1 && (ef.edge.PrimaryFace == item.Value.Item2 || ef.edge.SecondaryFace == item.Value.Item2)) ||
                                    (ef.face == item.Value.Item2 && (ef.edge.PrimaryFace == item.Value.Item1 || ef.edge.SecondaryFace == item.Value.Item1)))
                                {
                                    if ((item.Key.Vertex1.Position | ip[i]) < (item.Key.Vertex2.Position | ip[i]))
                                    {
                                        if ((item.Key.Vertex1.Position | ip[i]) < prec * 100)
                                        {
                                            ip[i] = item.Key.Vertex1.Position;
                                            uvOnFace[i] = ef.face.PositionOf(ip[i]);
                                            uOnCurve3D[i] = ef.edge.Curve3D.PositionOf(ip[i]);
                                        }
                                    }
                                    else
                                    {
                                        if ((item.Key.Vertex2.Position | ip[i]) < prec * 100)
                                        {
                                            ip[i] = item.Key.Vertex2.Position;
                                            uvOnFace[i] = ef.face.PositionOf(ip[i]);
                                            uOnCurve3D[i] = ef.edge.Curve3D.PositionOf(ip[i]);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    Vertex v = new Vertex(ip[i]);
                    v.AddPositionOnFace(ef.face, uvOnFace[i]);
                    IntersectionVertex iv = new IntersectionVertex();
                    iv.v = v;
                    iv.edge = ef.edge;
                    iv.face = ef.face;
                    iv.uOnEdge = uOnCurve3D[i];
                    iv.isOnFaceBorder = (position[i] == Border.Position.OnCurve); // das ist für später: Verbindung zweier Schnittpunkte, die beide auf dem Rand liegen, ist nicht sicher innerhalb
                    if (ef.edge.PrimaryFace.Owner == shell1)
                    {
                        iv.edgeIsOn1 = true;
                        AddFacesToIntersectionVertices(ef.edge.PrimaryFace, ef.face, iv);
                        AddFacesToIntersectionVertices(ef.edge.SecondaryFace, ef.face, iv);
                    }
                    else
                    {
                        iv.edgeIsOn1 = false;
                        AddFacesToIntersectionVertices(ef.face, ef.edge.PrimaryFace, iv);
                        AddFacesToIntersectionVertices(ef.face, ef.edge.SecondaryFace, iv);
                    }
                    intersectionVertices.Add(iv);
                    if (!edgesToSplit.ContainsKey(ef.edge)) edgesToSplit[ef.edge] = new List<Vertex>();
                    edgesToSplit[ef.edge].Add(v);
                    if (position[i] == Border.Position.OnCurve)
                    {
                        List<Edge> touchedEdges = ef.face.FindEdges(uvOnFace[i]);
                        for (int j = 0; j < touchedEdges.Count; j++)
                        {
                            if (touchedEdges[j].Curve3D == null) continue; // no poles here
                            double cprec = prec / touchedEdges[j].Curve3D.Length; // darf natürlich nicht 0 sein!
                            double pos = touchedEdges[j].Curve3D.PositionOf(v.Position);
                            if (pos > cprec && pos < 1 - cprec)
                            {
                                if (!edgesToSplit.ContainsKey(touchedEdges[j])) edgesToSplit[touchedEdges[j]] = new List<Vertex>();
                                edgesToSplit[touchedEdges[j]].Add(v);
                            }
                        }
                    }
                    if (operation == Operation.testonly) return; // ein Schnittpunkt reicht hier
                }
            }
        }
        /// <summary>
        /// Combine all vertices of both shells and the intersection vertices
        /// </summary>
        private void combineVertices()
        {
            VertexOcttree vo = new VertexOcttree(this.Extend, this.precision);
            Vertex[] vv = shell1.Vertices;
            for (int i = 0; i < vv.Length; i++)
            {
                vo.AddObject(vv[i]);
            }
            vv = shell2.Vertices;
            for (int i = 0; i < vv.Length; i++)
            {
                Vertex[] close = vo.GetObjectsCloseTo(vv[i]);
                bool found = false;
                for (int j = 0; j < close.Length; j++)
                {
                    if ((close[j].Position | vv[i].Position) < precision)
                    {
                        close[j].MergeWith(vv[i]);
                        found = true;
                        break;
                    }
                }
                if (!found) vo.AddObject(vv[i]);
            }
            // eigentlich sind die intersectionVertices -nach dem Splitten der Kanten- shcon alle in den shells vorhanden, so dachte ich zuerst, aber:
            // Schnittpunkte an den Enden der Kanten werden wohl nicht verwendet und kommen hier noch dazu
            foreach (IntersectionVertex iv in intersectionVertices)
            {
                Vertex[] close = vo.GetObjectsCloseTo(iv.v);
                bool found = false;
                for (int j = 0; j < close.Length; j++)
                {
                    if ((close[j].Position | iv.v.Position) < precision)
                    {
                        close[j].MergeWith(iv.v);
                        iv.v = close[j];
                        found = true;
                        break;
                    }
                }
                if (!found) vo.AddObject(iv.v); // die sind alle verschieden
            }
            // vo.combineAll(intersectionVertices); // hier wird zusammengefasst
        }
        private void combineVertices(IEnumerable<Face> faces)
        {
            HashSet<Vertex> vertices = new HashSet<Vertex>();
            foreach (Face fc in faces)
            {
                vertices.UnionWith(fc.Vertices);
            }
            VertexOcttree vo = new VertexOcttree(this.Extend, this.precision);
            foreach (var v in vertices)
            {
                if (vo.IsEmpty) vo.AddObject(v);
                else
                {
                    var existing = vo.GetObjectsCloseTo(v).FirstOrDefault(c => (c.Position | v.Position) < precision);

                    if (existing != null) existing.MergeWith(v);
                    else vo.AddObject(v);
                }
            }
        }
        private void combineVerticesMultipleFaces()
        {
            VertexOcttree vo = new VertexOcttree(this.Extend, this.precision);
            HashSet<Vertex> vertices = new HashSet<Vertex>();
            foreach (Face face in multipleFaces)
            {
                vertices.UnionWith(face.Vertices);
            }
            foreach (Vertex vtx in vertices)
            {
                Vertex[] close;
                if (vo.IsEmpty) close = new Vertex[0];
                else close = vo.GetObjectsCloseTo(vtx);
                bool found = false;
                for (int j = 0; j < close.Length; j++)
                {
                    if ((vtx != close[j]) && (close[j].Position | vtx.Position) < precision)
                    {
                        close[j].MergeWith(vtx);
                        found = true;
                        break;
                    }
                }
                if (!found) vo.AddObject(vtx);
            }
            foreach (IntersectionVertex iv in intersectionVertices)
            {
                Vertex[] close = vo.GetObjectsCloseTo(iv.v);
                bool found = false;
                for (int j = 0; j < close.Length; j++)
                {
                    if ((iv.v != close[j]) && (close[j].Position | iv.v.Position) < precision)
                    {
                        close[j].MergeWith(iv.v);
                        iv.v = close[j];
                        found = true;
                        break;
                    }
                }
                if (!found) vo.AddObject(iv.v); // die sind alle verschieden
            }
            // doesn't help: 
            if (faceToIntersectionEdges != null)
            {
                HashSet<Vertex> vtxs = new HashSet<Vertex>();
                foreach (var item in faceToIntersectionEdges)
                {
                    foreach (Edge edge in item.Value)
                    {
                        vtxs.Add(edge.Vertex1);
                        vtxs.Add(edge.Vertex2);
                    }
                }
                foreach (Vertex vtx in vtxs) vo.AddObject(vtx);
                foreach (Vertex vtx in vtxs)
                {
                    Vertex[] close = vo.GetObjectsCloseTo(vtx);
                    for (int j = 0; j < close.Length; j++)
                    {
                        if ((vtx != close[j]) && (close[j].Position | vtx.Position) < precision)
                        {
                            close[j].MergeWith(vtx);
                        }
                    }
                }
            }
        }
        private void removeIdenticalOppositeFaces()
        {
            cancelledfaces = new HashSet<GeoObject.Face>();
            foreach (DoubleFaceKey dfk in oppositeFaces.Keys)
            {
                HashSet<Vertex> v1 = new HashSet<Vertex>(dfk.face1.Vertices);
                HashSet<Vertex> v2 = new HashSet<Vertex>(dfk.face2.Vertices);
                if (v1.SetEquals(v2))
                {
                    // there could be non identical faces with the same set of vertices. We should test this here!
                    cancelledfaces.Add(dfk.face1);
                    cancelledfaces.Add(dfk.face2);
                }
            }
        }
        private void AddFacesToIntersectionVertices(Face f1, Face f2, IntersectionVertex iv)
        {
            if (f1 == null || f2 == null) return; // possible when a shell is not closed
            if (multipleFaces != null)
            {   // the case with multiple faces and no shells
                IEnumerable<Edge> commonEdges = f1.Edges.Intersect(f2.Edges); // in case of multipleFaces
                foreach (Edge edge in commonEdges)
                {   // dont use the common edges for new intersections
                    if (Precision.IsEqual(edge.Vertex1.Position, iv.v.Position)) return;
                    if (Precision.IsEqual(edge.Vertex2.Position, iv.v.Position)) return;
                }
                DoubleFaceKey dfk;
                if (f2.GetHashCode() < f1.GetHashCode()) dfk = new DoubleFaceKey(f2, f1);
                else dfk = new DoubleFaceKey(f1, f2);
                if (!facesToIntersectionVertices.TryGetValue(dfk, out List<IntersectionVertex> list))
                    facesToIntersectionVertices[dfk] = list = new List<IntersectionVertex>(); ;
                list.Add(iv);
            }
            else
            {   // the case with two shells
                List<IntersectionVertex> list;
                if (!facesToIntersectionVertices.TryGetValue(new DoubleFaceKey(f1, f2), out list))
                {
                    list = new List<IntersectionVertex>();
                    facesToIntersectionVertices[new DoubleFaceKey(f1, f2)] = list;
                }
                list.Add(iv);
            }
        }

        public enum Operation { union, intersection, difference, clip, connectMultiple, testonly }
        Operation operation;
        /// <summary>
        /// Prepare a brep operation for splitting a (closed) shell with a plane. Or for returning the compound shapes on the specified plane.
        /// Here we assume that "toSplit" is properly oriented and han no periodic faces (no seams)
        /// </summary>
        /// <param name="toSplit">the shell to split (must be closed)</param>
        /// <param name="splitBy">the plane to split by</param>
        //public BooleanOperation(Shell toSplit, Plane splitBy)
        //{
        //    shell1 = toSplit.Clone() as Shell;   // clone the shell because its faces will be modified
        //    shell1.AssertOutwardOrientation();
        //    BoundingCube ext = shell1.GetExtent(0.0);
        //    Face fcpl = Face.MakeFace(new PlaneSurface(splitBy), new BoundingRect(GeoPoint2D.Origin, 2 * ext.DiagonalLength, 2 * ext.DiagonalLength));
        //    splittingOnplane = fcpl.Surface as PlaneSurface; // remember the plane by which we split to return a proper SplitResult
        //    Face fcpl1 = fcpl.Clone() as Face;
        //    fcpl1.MakeInverseOrientation();
        //    // shell2 = Shell.MakeShell(new Face[] { fcpl, fcpl1 }, true); // open shell, but the size of the face exceedes the shell to split
        //    shell2 = Shell.MakeShell(new Face[] { fcpl }, false); // open shell, but the size of the face exceedes the shell to split
        //    operation = Operation.difference;
        //    prepare();
        //}
        /// <summary>
        /// Split the shell <paramref name="toSplit"/> by a face. The face <paramref name="splitBy"/> must split the shell into two or more parts
        /// It may not enter the shell without cutting through
        /// </summary>
        /// <param name="toSplit"></param>
        /// <param name="splitBy"></param>
        //public BooleanOperation(Shell toSplit, Face splitBy)
        //{
        //    shell1 = toSplit.Clone() as Shell;   // clone the shell because its faces will be modified
        //    shell1.AssertOutwardOrientation();
        //    BoundingCube ext = shell1.GetExtent(0.0);
        //    shell2 = Shell.MakeShell(new Face[] { splitBy.Clone() as Face }, false); // open shell
        //    operation = Operation.difference;
        //    prepare();
        //}
        //public BooleanOperation(Face toClip, Shell clipBy)
        //{
        //    shell1 = Shell.MakeShell(new Face[] { toClip }, false);
        //    shell2 = clipBy.Clone() as Shell;
        //    operation = Operation.clip;
        //    prepare();
        //}

        //public BooleanOperation(Shell s1, Shell s2, Dictionary<Edge, Tuple<Face, Face>> knownIntersections, Operation operation)
        //{
        //    this.operation = operation;
        //    Dictionary<Edge, Edge> clonedEdges = new Dictionary<Edge, Edge>();
        //    Dictionary<Vertex, Vertex> clonedVertices = new Dictionary<Vertex, Vertex>();
        //    Dictionary<Face, Face> clonedFaces = new Dictionary<Face, Face>();
        //    shell1 = s1.Clone(clonedEdges, clonedVertices, clonedFaces) as Shell; // we need clones, because some Faces are destroyed in the process which would make undo impossible
        //    shell2 = s2.Clone(clonedEdges, clonedVertices, clonedFaces) as Shell;
        //    // set the knownIntersections, respectiong the clone operation
        //    this.knownIntersections = new Dictionary<Edge, Tuple<Face, Face>>();
        //    foreach (KeyValuePair<Edge, Tuple<Face, Face>> item in knownIntersections)
        //    {
        //        this.knownIntersections[clonedEdges[item.Key]] = new Tuple<Face, Face>(clonedFaces[item.Value.Item1], clonedFaces[item.Value.Item2]);
        //    }
        //    if (operation == Operation.union)
        //    {
        //        shell1.ReverseOrientation();
        //        shell2.ReverseOrientation();
        //    }
        //    else if (operation == Operation.difference)
        //    {   // es ist ja shell1 - shell2, also Vereinigung mit dem inversen von shell2
        //        shell2.ReverseOrientation();
        //    }
        //    BoundingCube ext1 = shell1.GetExtent(0.0);
        //    BoundingCube ext2 = shell2.GetExtent(0.0);
        //    BoundingCube ext = ext1;
        //    ext.MinMax(ext2);
        //    // in rare cases the extension isn't a good choice, faces shouldn't exactely reside on the sides of the small cubes of the octtree
        //    // so we modify the extension a little, to make this case extremely unlikely. The best solution would be to check, whether a vertex
        //    // falls exactely on the side of a octtree-cube, then throw an exception and try with a different octtree location
        //    double extsize = ext.Size;
        //    ext.Expand(extsize * 1e-3);
        //    ext = ext.Modify(new GeoVector(extsize * 1e-4, extsize * 1e-4, extsize * 1e-4));
        //    Initialize(ext, extsize * 1e-6); // put all edges and faces into the octtree
        //    foreach (Edge edg in shell1.Edges)
        //    {
        //        base.AddObject(new BRepItem(this, edg));
        //    }
        //    foreach (Edge edg in shell2.Edges)
        //    {
        //        base.AddObject(new BRepItem(this, edg));
        //    }
        //    foreach (Face fc in shell1.Faces)
        //    {
        //        base.AddObject(new BRepItem(this, fc));
        //    }
        //    foreach (Face fc in shell2.Faces)
        //    {
        //        base.AddObject(new BRepItem(this, fc));
        //    }

        //    edgesToSplit = new Dictionary<Edge, List<Vertex>>();
        //    intersectionVertices = new HashSet<IntersectionVertex>();
        //    facesToIntersectionVertices = new Dictionary<DoubleFaceKey, List<IntersectionVertex>>();

        //    findOverlappingFaces(); // setzt overlappingFaces, also Faces von verschiedenen shells, die sich teilweise überlappen oder identisch sind
        //    createEdgeFaceIntersections(); // Schnittpunkte von Edges mit Faces von verschiedenen shells
        //    if (operation != Operation.testonly)
        //    {   // Für testonly genügen die Kantenschnitte (fast)
        //        splitEdges(); // mit den gefundenen Schnittpunkten werden die Edges jetzt gesplittet
        //        combineVertices(); // alle geometrisch identischen Vertices werden zusammengefasst
        //        removeIdenticalOppositeFaces(); // Paare von entgegengesetzt orientierten Faces mit identischer Fläche werden entfernt
        //        createNewEdges(); // faceToIntersectionEdges wird gesetzt, also für jedes Face eine Liste der neuen Schnittkanten
        //        createInnerFaceIntersections(); // Schnittpunkte zweier Faces, deren Kanten sich aber nicht schneiden, finden
        //        combineEdges(); // hier werden intsEdgeToEdgeShell1, intsEdgeToEdgeShell2 und intsEdgeToIntsEdge gesetzt, die aber z.Z. noch nicht verwendet werden

        //    }

        //}
        (Edge, Edge, Edge) EdgeTriple(Edge e1, Edge e2, Edge e3)
        {
            Edge[] sorted = new[] { e1, e2, e3 }.OrderBy(e => e.GetHashCode()).ToArray();
            return (sorted[0], sorted[1], sorted[2]);
        }
        (Face, Face, Face) FaceTriple(Face f1, Face f2, Face f3)
        {
            Face[] sorted = new[] { f1, f2, f3 }.OrderBy(e => e.GetHashCode()).ToArray();
            return (sorted[0], sorted[1], sorted[2]);
        }

        /// <summary>
        /// Connect all provided faces, remove the overhangs. Used by Shell.GetOffset.
        /// </summary>
        /// <param name="shellsToConnect"></param>
        //public BooleanOperation(IEnumerable<Face> facesToConnect)
        //{
        //    operation = Operation.connectMultiple;
        //    multipleFaces = new List<Face>(facesToConnect); // we don't clone here, because we dont need it
        //    foreach (Face face in multipleFaces) face.ReverseOrientation();

        //    // fill the OctTree
        //    BoundingCube ext = BoundingCube.EmptyBoundingCube;
        //    foreach (Face face in multipleFaces) ext.MinMax(face.GetExtent(0.0));
        //    // in rare cases the extension isn't a good choice, faces shouldn't exactely reside on the sides of the small cubes of the octtree
        //    // so we modify the extension a little, to make this case extremely unlikely. The best solution would be to check, whether a vertex
        //    // falls exactely on the side of a octtree-cube, then throw an exception and try with a different octtree location
        //    double extsize = ext.Size;
        //    ext.Expand(extsize * 1e-3);
        //    ext = ext.Modify(new GeoVector(extsize * 1e-4, extsize * 1e-4, extsize * 1e-4));
        //    Initialize(ext, extsize * 1e-6); // initialize the OctTree
        //    // put all edges and faces into the octtree
        //    HashSet<Edge> alreadyAdded = new HashSet<Edge>();
        //    foreach (Face face in multipleFaces)
        //    {
        //        foreach (Edge edg in face.Edges)
        //        {
        //            if (alreadyAdded.Contains(edg)) continue;
        //            AddObject(new BRepItem(this, edg));
        //            alreadyAdded.Add(edg);
        //        }
        //        AddObject(new BRepItem(this, face));
        //    }

        //    edgesToSplit = new Dictionary<Edge, List<Vertex>>();
        //    intersectionVertices = new HashSet<IntersectionVertex>();
        //    facesToIntersectionVertices = new Dictionary<DoubleFaceKey, List<IntersectionVertex>>();

        //    findOverlappingFaces(); // populates overlappingFaces, faces of different shells which overlap or are identical
        //    createEdgeFaceIntersections(); // find intersection of edges with faces from different shells
        //    splitEdges(); // split the edges at the found intersection positions
        //    combineVerticesMultipleFaces(); // combine geometric close vertices
        //    removeIdenticalOppositeFaces(); // 
        //    createNewEdges(); // populate faceToIntersectionEdges : for each face a list of intersection curves
        //    createInnerFaceIntersections(); // find additional intersection curves where faces intersect, but edges don't intersect (rare)
        //    TrimmIntersectionEdges();
        //    combineVerticesMultipleFaces(); // combine geometric close vertices
        //    // combineEdges(); // do we need this?

        //}
        /// <summary>
        /// This is a constructor for connecting and intersecting (open) shells, where we have a set of edge-face pairs, which should not be used for intersecting
        /// </summary>
        /// <param name="s1"></param>
        /// <param name="s2"></param>
        /// <param name="dontIntersect"></param>
        /// <param name="operation"></param>
        //        public BooleanOperation(Shell s1, Shell s2, HashSet<(Edge, Face)> dontIntersect, Operation operation)
        //        {
        //            // we are working on the original data here and assume everything is correctly oriented and connected, no vertex recalculation etc.
        //            // the provided shells are destroyed here
        //            this.operation = operation;
        //            this.dontIntersect = dontIntersect;
        //            this.shell1 = s1;
        //            this.shell2 = s2;
        //            if (operation == Operation.union)
        //            {
        //                shell1.ReverseOrientation();
        //                shell2.ReverseOrientation();
        //            }
        //            BoundingCube ext1 = shell1.GetExtent(0.0);
        //            BoundingCube ext2 = shell2.GetExtent(0.0);
        //            BoundingCube ext = ext1;
        //            ext.MinMax(ext2);
        //            // in rare cases the extension isn't a good choice, faces shouldn't exactely reside on the sides of the small cubes of the octtree
        //            // so we modify the extension a little, to make this case extremely unlikely. The best solution would be to check, whether a vertex
        //            // falls exactely on the side of a octtree-cube, then throw an exception and try with a different octtree location
        //            double extsize = ext.Size;
        //            ext.Expand(extsize * 1e-3);
        //            ext = ext.Modify(new GeoVector(extsize * 1e-4, extsize * 1e-4, extsize * 1e-4));
        //            Initialize(ext, extsize * 1e-6); // der OctTree
        //                                             // put all edges and faces into the octtree
        //            foreach (Edge edg in shell1.Edges)
        //            {
        //                base.AddObject(new BRepItem(this, edg));
        //#if DEBUG
        //                if (edg.Curve3D is InterpolatedDualSurfaceCurve) (edg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
        //#endif
        //            }
        //            foreach (Edge edg in shell2.Edges)
        //            {
        //                base.AddObject(new BRepItem(this, edg));
        //#if DEBUG
        //                if (edg.Curve3D is InterpolatedDualSurfaceCurve) (edg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
        //#endif
        //            }
        //            foreach (Face fc in shell1.Faces)
        //            {
        //                base.AddObject(new BRepItem(this, fc));
        //            }
        //            foreach (Face fc in shell2.Faces)
        //            {
        //                base.AddObject(new BRepItem(this, fc));
        //            }

        //            edgesToSplit = new Dictionary<Edge, List<Vertex>>();
        //            intersectionVertices = new HashSet<IntersectionVertex>();
        //            facesToIntersectionVertices = new Dictionary<DoubleFaceKey, List<IntersectionVertex>>();

        //            findOverlappingFaces(); // setzt overlappingFaces, also Faces von verschiedenen shells, die sich teilweise überlappen oder identisch sind
        //            createEdgeFaceIntersections(); // find intersections between edges and faces of different shells

        //            splitEdges(); // mit den gefundenen Schnittpunkten werden die Edges jetzt gesplittet
        //            combineVertices(); // alle geometrisch identischen Vertices werden zusammengefasst
        //            removeIdenticalOppositeFaces(); // Paare von entgegengesetzt orientierten Faces mit identischer Fläche werden entfernt
        //            createNewEdges(); // faceToIntersectionEdges wird gesetzt, also für jedes Face eine Liste der neuen Schnittkanten
        //            createInnerFaceIntersections(); // Schnittpunkte zweier Faces, deren Kanten sich aber nicht schneiden, finden
        //            combineEdges(); // hier werden intsEdgeToEdgeShell1, intsEdgeToEdgeShell2 und intsEdgeToIntsEdge gesetzt, die aber z.Z. noch nicht verwendet werden
        //        }

        //        public BooleanOperation(Shell s1, Shell s2, Operation operation)
        //        {
        //            // im Konstruktor werden die Schnitte zwischen den shells berechnet. Mit Result, wird dann das Ergebnis geholt.
        //            // Da Result alles aufmischt, kann man es nicht zweimal (z.B. mit verschiedenen Operationen) verwenden. Insofern ist die Trennung
        //            // von Konstruktor und Result willkürlich. Besser sollte man statische Methoden machen Union, Intersection, Difference und alles andere private.
        //#if DEBUG
        //            //foreach (Face fce in s1.Faces)
        //            //{
        //            //    if (fce.Surface is ConicalSurface)
        //            //    {
        //            //        foreach (Edge edg in fce.Edges)
        //            //        {
        //            //            ICurve2D cv = fce.Surface.GetProjectedCurve(edg.Curve3D, 0.0);
        //            //            if (!edg.Forward(fce)) cv.Reverse();
        //            //            if (edg.PrimaryFace==fce)
        //            //            {
        //            //                edg.PrimaryCurve2D = cv;
        //            //            }
        //            //            else
        //            //            {
        //            //                edg.SecondaryCurve2D = cv;
        //            //            }
        //            //            ICurve cv3d = fce.Surface.Make3dCurve(cv);
        //            //        }
        //            //        Face.CheckOutlineDirection(fce, fce.OutlineEdges, Math.PI * 2, 0, null);
        //            //        for (int i = 0; i < fce.HoleCount; i++)
        //            //        {
        //            //            Face.CheckOutlineDirection(fce, fce.HoleEdges(i), Math.PI * 2, 0, null);
        //            //        }
        //            //        fce.ForceAreaRecal();
        //            //    }
        //            //}
        //            foreach (Edge dbgedg in s1.Edges)
        //            {
        //                if (dbgedg.Curve3D is InterpolatedDualSurfaceCurve)
        //                    (dbgedg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
        //            }
        //            foreach (Edge dbgedg in s2.Edges)
        //            {
        //                if (dbgedg.Curve3D is InterpolatedDualSurfaceCurve)
        //                    (dbgedg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
        //            }
        //            System.Diagnostics.Debug.Assert(s1.CheckConsistency());
        //            System.Diagnostics.Debug.Assert(s2.CheckConsistency());
        //#endif
        //            s1.RecalcVertices();
        //            s2.RecalcVertices();
        //#if DEBUG
        //            System.Diagnostics.Debug.Assert(s1.CheckConsistency());
        //            System.Diagnostics.Debug.Assert(s2.CheckConsistency());
        //#endif
        //            s1.SplitPeriodicFaces();
        //            s2.SplitPeriodicFaces();
        //            s1.AssertOutwardOrientation();
        //            s2.AssertOutwardOrientation();
        //            this.operation = operation;
        //#if DEBUG
        //            System.Diagnostics.Debug.Assert(s1.CheckConsistency());
        //            System.Diagnostics.Debug.Assert(s2.CheckConsistency());
        //            foreach (Edge dbgedg in s1.Edges)
        //            {
        //                if (dbgedg.Curve3D is InterpolatedDualSurfaceCurve)
        //                    (dbgedg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
        //            }
        //            foreach (Edge dbgedg in s2.Edges)
        //            {
        //                if (dbgedg.Curve3D is InterpolatedDualSurfaceCurve)
        //                    (dbgedg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
        //            }
        //#endif
        //            shell1 = s1.Clone() as Shell;   // hier wird gekloned, weil die Faces im Verlauf geändert werden und das Original
        //            shell2 = s2.Clone() as Shell;   // unverändert bleiben soll. ZUm Debuggen kann man das Klonen weglassen
        //            if (shell1.HasOpenEdgesExceptPoles()) shell1.TryConnectOpenEdges();
        //            if (shell2.HasOpenEdgesExceptPoles()) shell2.TryConnectOpenEdges();
        //            shell1.RecalcVertices();
        //            shell2.RecalcVertices();
        //            shell1.CombineConnectedFaces();
        //            shell2.CombineConnectedFaces();
        //#if DEBUG
        //            foreach (Edge edg in shell1.Edges)
        //            {
        //                edg.CheckConsistency();
        //            }
        //            foreach (Edge edg in shell2.Edges)
        //            {
        //                edg.CheckConsistency();
        //            }
        //#endif
        //#if DEBUG
        //            System.Diagnostics.Debug.Assert(shell1.CheckConsistency());
        //            System.Diagnostics.Debug.Assert(shell2.CheckConsistency());
        //            foreach (Edge dbgedg in s1.Edges)
        //            {
        //                if (dbgedg.Curve3D is InterpolatedDualSurfaceCurve)
        //                    (dbgedg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
        //            }
        //            foreach (Edge dbgedg in s2.Edges)
        //            {
        //                if (dbgedg.Curve3D is InterpolatedDualSurfaceCurve)
        //                    (dbgedg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
        //            }
        //            DebuggerContainer dcfcs = new CADability.DebuggerContainer();
        //            foreach (Face fce in shell1.Faces)
        //            {
        //                dcfcs.Add(fce.Clone(), fce.GetHashCode()); // die Faces werden kaputt gemacht, deshalb hier clones merken
        //                double ll = fce.GetExtent(0.0).Size * 0.01;
        //                ColorDef cd = new ColorDef("debug", Color.Blue);
        //                SimpleShape ss = fce.Area;
        //                GeoPoint2D c = ss.GetExtent().GetCenter();
        //                GeoPoint pc = fce.Surface.PointAt(c);
        //                GeoVector nc = fce.Surface.GetNormal(c);
        //                Line l = Line.TwoPoints(pc, pc + ll * nc.Normalized);
        //                l.ColorDef = cd;
        //                dcfcs.Add(l);
        //            }
        //            foreach (Face fce in shell2.Faces)
        //            {
        //                dcfcs.Add(fce.Clone(), fce.GetHashCode()); // die Faces werden kaputt gemacht, deshalb hier clones merken
        //                double ll = fce.GetExtent(0.0).Size * 0.01;
        //                ColorDef cd = new ColorDef("debug", Color.Brown);
        //                SimpleShape ss = fce.Area;
        //                GeoPoint2D c = ss.GetExtent().GetCenter();
        //                GeoPoint pc = fce.Surface.PointAt(c);
        //                GeoVector nc = fce.Surface.GetNormal(c);
        //                Line l = Line.TwoPoints(pc, pc + ll * nc.Normalized);
        //                l.ColorDef = cd;
        //                dcfcs.Add(l);
        //            }
        //#endif
        //            Vertex[] dumy = shell1.Vertices; // nur damits berechnet wird
        //            dumy = shell2.Vertices;
        //            if (operation == Operation.union)
        //            {
        //                shell1.ReverseOrientation();
        //                shell2.ReverseOrientation();
        //            }
        //            else if (operation == Operation.difference)
        //            {   // es ist ja shell1 - shell2, also Vereinigung mit dem inversen von shell2
        //                shell2.ReverseOrientation();
        //            }
        //#if DEBUG
        //            DebuggerContainer dc1 = new DebuggerContainer();
        //            foreach (Edge edg in shell1.Edges)
        //            {
        //                if (edg.Curve3D != null) dc1.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
        //                edg.CheckConsistency();
        //            }
        //            DebuggerContainer dc2 = new DebuggerContainer();
        //            foreach (Edge edg in shell2.Edges)
        //            {
        //                if (edg.Curve3D != null) dc2.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
        //                edg.CheckConsistency();
        //            }
        //#endif
        //            BoundingCube ext1 = shell1.GetExtent(0.0);
        //            BoundingCube ext2 = shell2.GetExtent(0.0);
        //            BoundingCube ext = ext1;
        //            ext.MinMax(ext2);
        //            // in rare cases the extension isn't a good choice, faces shouldn't exactely reside on the sides of the small cubes of the octtree
        //            // so we modify the extension a little, to make this case extremely unlikely. The best solution would be to check, whether a vertex
        //            // falls exactely on the side of a octtree-cube, then throw an exception and try with a different octtree location
        //            double extsize = ext.Size;
        //            ext.Expand(extsize * 1e-3);
        //            ext = ext.Modify(new GeoVector(extsize * 1e-4, extsize * 1e-4, extsize * 1e-4));
        //            Initialize(ext, extsize * 1e-6); // der OctTree
        //                                             // put all edges and faces into the octtree
        //            foreach (Edge edg in shell1.Edges)
        //            {
        //                base.AddObject(new BRepItem(this, edg));
        //#if DEBUG
        //                if (edg.Curve3D is InterpolatedDualSurfaceCurve) (edg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
        //#endif
        //            }
        //            foreach (Edge edg in shell2.Edges)
        //            {
        //                base.AddObject(new BRepItem(this, edg));
        //#if DEBUG
        //                if (edg.Curve3D is InterpolatedDualSurfaceCurve) (edg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
        //#endif
        //            }
        //            foreach (Face fc in shell1.Faces)
        //            {
        //                base.AddObject(new BRepItem(this, fc));
        //            }
        //            foreach (Face fc in shell2.Faces)
        //            {
        //                base.AddObject(new BRepItem(this, fc));
        //            }

        //            edgesToSplit = new Dictionary<Edge, List<Vertex>>();
        //            intersectionVertices = new HashSet<IntersectionVertex>();
        //            facesToIntersectionVertices = new Dictionary<DoubleFaceKey, List<IntersectionVertex>>();

        //            findOverlappingFaces(); // setzt overlappingFaces, also Faces von verschiedenen shells, die sich teilweise überlappen oder identisch sind
        //            createEdgeFaceIntersections(); // Schnittpunkte von Edges mit Faces von verschiedenen shells
        //#if DEBUG
        //            DebuggerContainer dc3 = new DebuggerContainer();
        //            foreach (Edge edge in edgesToSplit.Keys)
        //            {
        //                dc3.Add(edge.Curve3D as IGeoObject, edge.GetHashCode());
        //            }
        //            foreach (IntersectionVertex iv in intersectionVertices)
        //            {
        //                Point pnt = Point.Construct();
        //                pnt.Location = iv.v.Position;
        //                pnt.Symbol = PointSymbol.Cross;
        //                dc3.Add(pnt, iv.v.GetHashCode());
        //            }
        //#endif
        //            if (operation != Operation.testonly)
        //            {   // Für testonly genügen die Kantenschnitte (fast)
        //                splitEdges(); // mit den gefundenen Schnittpunkten werden die Edges jetzt gesplittet
        //                combineVertices(); // alle geometrisch identischen Vertices werden zusammengefasst
        //                removeIdenticalOppositeFaces(); // Paare von entgegengesetzt orientierten Faces mit identischer Fläche werden entfernt
        //                createNewEdges(); // faceToIntersectionEdges wird gesetzt, also für jedes Face eine Liste der neuen Schnittkanten
        //                createInnerFaceIntersections(); // Schnittpunkte zweier Faces, deren Kanten sich aber nicht schneiden, finden
        //                combineEdges(); // hier werden intsEdgeToEdgeShell1, intsEdgeToEdgeShell2 und intsEdgeToIntsEdge gesetzt, die aber z.Z. noch nicht verwendet werden
        //            }
        //#if DEBUG
        //            foreach (KeyValuePair<Face, HashSet<Edge>> item in faceToIntersectionEdges)
        //            {
        //                foreach (Edge edg in item.Value)
        //                {
        //                    dc3.Add(edg.Curve3D as IGeoObject, item.Key.GetHashCode());
        //                }
        //            }
        //            DebuggerContainer dc4 = new DebuggerContainer();
        //            HashSet<Vertex> dbgv = new HashSet<Vertex>();
        //            dbgv.UnionWith(shell1.Vertices);
        //            dbgv.UnionWith(shell2.Vertices); // kommt leider teilweise aus dem veralteten cache
        //            foreach (IntersectionVertex iv in intersectionVertices)
        //            {
        //                dbgv.Add(iv.v); // sollte ja schon drin sein!
        //            }
        //            foreach (Vertex v in dbgv)
        //            {
        //                Point pnt = Point.Construct();
        //                pnt.Location = v.Position;
        //                pnt.Symbol = PointSymbol.Cross;
        //                dc4.Add(pnt, v.GetHashCode());
        //            }
        //#endif
        //        }

        /// <summary>
        /// Creates an BooleanOperation object. You must at least set the operation and some shells or faces to work on
        /// </summary>
        public BooleanOperation()
        {

        }

        public void SetShells(Shell s1, Shell s2, Operation operation)
        {
            originalToClonedEdges = new Dictionary<Edge, Edge>();
            originalToClonedVertices = new Dictionary<Vertex, Vertex>();
            originalToClonedFaces = new Dictionary<Face, Face>();
            shell1 = s1.Clone(originalToClonedEdges, originalToClonedVertices, originalToClonedFaces); // we clone the shells to not destroy the originals. We could keep references to the original by UserData
            shell2 = s2.Clone(originalToClonedEdges, originalToClonedVertices, originalToClonedFaces);
            this.operation = operation;
        }

        public Shell[] Execute()
        {   // we expect the shell sare in a proper state: outward oriented, no full periodic faces
            // don't know, whether we need the followin:
            if (shell1.HasOpenEdgesExceptPoles()) shell1.TryConnectOpenEdges();
            if (shell2.HasOpenEdgesExceptPoles()) shell2.TryConnectOpenEdges();
            //shell1.RecalcVertices();
            //shell2.RecalcVertices();
            shell1.CombineConnectedFaces();
            shell2.CombineConnectedFaces();

#if DEBUG
            foreach (Edge edg in shell1.Edges)
            {
                edg.CheckConsistency();
            }
            foreach (Edge edg in shell2.Edges)
            {
                edg.CheckConsistency();
            }
#endif
#if DEBUG
            System.Diagnostics.Debug.Assert(shell1.CheckConsistency());
            System.Diagnostics.Debug.Assert(shell2.CheckConsistency());
            DebuggerContainer dcfcs = new CADability.DebuggerContainer();
            foreach (Face fce in shell1.Faces)
            {
                dcfcs.Add(fce.Clone(), fce.GetHashCode()); // die Faces werden kaputt gemacht, deshalb hier clones merken
                double ll = fce.GetExtent(0.0).Size * 0.01;
                ColorDef cd = new ColorDef("debug", Color.Blue);
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
                dcfcs.Add(fce.Clone(), fce.GetHashCode()); // die Faces werden kaputt gemacht, deshalb hier clones merken
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
#endif
            Vertex[] dumy = shell1.Vertices; // nur damits berechnet wird
            dumy = shell2.Vertices;
            if (operation == Operation.union)
            {
                shell1.ReverseOrientation();
                shell2.ReverseOrientation();
            }
            else if (operation == Operation.difference)
            {   // es ist ja shell1 - shell2, also Vereinigung mit dem inversen von shell2
                shell2.ReverseOrientation();
            }
#if DEBUG
            DebuggerContainer dc1 = new DebuggerContainer();
            foreach (Edge edg in shell1.Edges)
            {
                if (edg.Curve3D != null) dc1.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
                edg.CheckConsistency();
            }
            DebuggerContainer dc2 = new DebuggerContainer();
            foreach (Edge edg in shell2.Edges)
            {
                if (edg.Curve3D != null) dc2.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
                edg.CheckConsistency();
            }
#endif
            BoundingCube ext1 = shell1.GetExtent(0.0);
            BoundingCube ext2 = shell2.GetExtent(0.0);
            BoundingCube ext = ext1;
            ext.MinMax(ext2);
            precision = ext.Size * 1e-6; // a meassure to decide when points are close egneough to consider them as identical
            // in rare cases the extension isn't a good choice, faces shouldn't exactely reside on the sides of the small cubes of the octtree
            // so we modify the extension a little, to make this case extremely unlikely. The best solution would be to check, whether a vertex
            // falls exactely on the side of a octtree-cube, then throw an exception and try with a different octtree location
            double extsize = ext.Size;
            ext.Expand(extsize * 1e-3);
            ext = ext.Modify(new GeoVector(extsize * 1e-4, extsize * 1e-4, extsize * 1e-4));
            facesOctTree = new OctTree<Face>(ext, extsize * 1e-6);
            facesOctTree.AddMany(shell1.Faces);
            facesOctTree.AddMany(shell2.Faces);
            verticesOctTree = new OctTree<Vertex>(ext, extsize * 1e-6);
            AddToVertexOctTree(shell1);
            AddToVertexOctTree(shell2);

            edgesToSplit = new Dictionary<Edge, List<Vertex>>();
            faceToIntersectionEdges = new Dictionary<Face, HashSet<Edge>>();
            faceToOverlappingFaces = new Dictionary<Face, Dictionary<Face, ModOp2D>>();
            faceToOppositeFaces = new Dictionary<Face, Dictionary<Face, ModOp2D>>();

            CreateFaceIntersections();

#if DEBUG
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
            {   // Für testonly genügen die Kantenschnitte (fast)
                SplitEdges(); // mit den gefundenen Schnittpunkten werden die Edges jetzt gesplittet
                ProcessOverlappingFaces();

            }
#if DEBUG
            foreach (KeyValuePair<Face, HashSet<Edge>> item in faceToIntersectionEdges)
            {
                foreach (Edge edg in item.Value)
                {
                    dc3.Add(edg.Curve3D as IGeoObject, item.Key.GetHashCode());
                }
            }
            DebuggerContainer dc4 = new DebuggerContainer();
            HashSet<Vertex> dbgv = new HashSet<Vertex>();
            dbgv.UnionWith(shell1.Vertices);
            dbgv.UnionWith(shell2.Vertices); // kommt leider teilweise aus dem veralteten cache
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
                foreach (var oppositeFace in kv.Value.Keys)
                {
                    ModOp2D firstToSecond = kv.Value[oppositeFace];
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
                            Edge overlappingEdge = new Edge(oppositeFace, edge.Curve3D.Clone(), oppositeFace, edge.Curve2D(oppositeFace).CloneReverse(true), forward,
                                mainFace, secondaryCurve2D, !forward);
                            overlappingEdge.UseVertices(edge.Vertex1, edge.Vertex2);
                            if (!faceToIntersectionEdges.ContainsKey(oppositeFace)) faceToIntersectionEdges[oppositeFace] = new HashSet<Edge>();
                            faceToIntersectionEdges[oppositeFace].Add(overlappingEdge);
                            if (!faceToIntersectionEdges.ContainsKey(mainFace)) faceToIntersectionEdges[mainFace] = new HashSet<Edge>();
                            faceToIntersectionEdges[mainFace].Add(overlappingEdge);
                            // in most cases, the overlapping edge exists already in the mainFace, but duplicates are no problem
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
                    {
                        commonFace = Face.Construct();
                        commonFace.Surface = mainFace.Surface.Clone();
                        commonFaces[(mainFace, otherFace)] = commonFace;
                        commonFaceHasMainSurface = true;
                    }
                    ModOp2D firstToSecond = kv.Value[otherFace];
                    foreach (Edge edge in otherFace.Edges)
                    {
                        if (mainFace.Contains(edge.Curve3D.PointAt(0.5), false)) // the edge lies on the main face
                        {   // this edge, which is totally inside the main face, is dealt as an intersection edge of the common face
                            Edge commonEdge;
                            ICurve2D curveOnPrimaryFace;
                            if (commonFaceHasMainSurface)
                            {// the common face has the surface of the main face
                                if (firstToSecond.IsNull) curveOnPrimaryFace = mainFace.Surface.GetProjectedCurve(edge.Curve3D, precision);
                                else curveOnPrimaryFace = edge.Curve2D(otherFace).GetModified(firstToSecond.GetInverse());
                                if (edge.Forward(otherFace)) curveOnPrimaryFace.Reverse();
                                commonEdge = new Edge(commonFace, edge.Curve3D.Clone(), commonFace, curveOnPrimaryFace, edge.Forward(otherFace));
                            }
                            else
                            { // the common face has the surface of the other face
                                if (firstToSecond.IsNull) curveOnPrimaryFace = otherFace.Surface.GetProjectedCurve(edge.Curve3D, precision);
                                else curveOnPrimaryFace = edge.Curve2D(otherFace).Clone();
                                if (!edge.Forward(otherFace)) curveOnPrimaryFace.Reverse();
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
            foreach (Face fc in commonFaces.Values)
            {   // we have to assign an outline to the common face, otherwise it will generate problems in various situations
                BoundingRect ext = BoundingRect.EmptyBoundingRect;
                foreach (Edge edg in faceToIntersectionEdges[fc])
                {
                    ext.MinMax(edg.Curve2D(fc).GetExtent());
                }
                ext.InflateRelative(1.1); // to create a big egnough outline
                Face dumy = Face.MakeFace(fc.Surface, ext); // to create appropriate edges
                // transfer the edges from dumy to fc (very low level, but efficient)
                foreach (Edge edg in dumy.OutlineEdges)
                {
                    edg.SetPrimary(fc, true);
                    edg.Owner = fc;
                }
                fc.SetOutline(dumy.OutlineEdges);
            }
            commonOverlappingFaces = commonFaces.Values.ToHashSet();
        }

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

        public static (Shell[] upperPart, Shell[] lowerPart) SplitByPlane(Shell shell, Plane pln)
        {
            //BooleanOperation brep = new BooleanOperation(shell, pln);
            //Shell[] upper = brep.Result();
            //pln = new Plane(pln.Location, pln.DirectionY, pln.DirectionX); // the same plane, but reversed
            //brep = new BooleanOperation(shell, pln);
            //Shell[] lower = brep.Result();
            //return (upper, lower);
            throw new NotImplementedException();
        }
        public static (Shell[] upperPart, Shell[] lowerPart) SplitByFace(Shell toSplit, Face splitBy)
        {
            //BooleanOperation brep = new BooleanOperation(toSplit, splitBy);
            //Shell[] upper = brep.Result();
            //Face reversed = splitBy.Clone() as Face;
            //reversed.ReverseOrientation();
            //brep = new BooleanOperation(toSplit, reversed);
            //Shell[] lower = brep.Result();
            //return (upper, lower);
            throw new NotImplementedException();

        }
        /// <summary>
        /// Chamfer or bevel the provided edges. the edges must be connected and only two edges may have a common vertex. The edges must build a path.
        /// We have two distances from the edge to make chamfers with different angles. All edges must belong to the <paramref name="primaryFace"/>. 
        /// The first distance is on the primary face, the second on the other face (in most cases the distances are equal).
        /// </summary>
        /// <param name="primaryFace"></param>
        /// <param name="edges"></param>
        /// <param name="primaryDist"></param>
        /// <param name="secondaryDist"></param>
        /// <returns></returns>
        public static Shell ChamferEdges(Face primaryFace, Edge[] edges, double primaryDist, double secondaryDist)
        {
            Shell toChamfer = primaryFace.Owner as Shell;
            if (toChamfer == null) return null;
            // sort the edges in the connection order
            if (edges.Length > 1)
            {
                HashSet<Edge> toSort = new HashSet<Edge>(edges);
                List<Edge> sorted = new List<Edge>();
                Edge startWith = toSort.First();
                toSort.Remove(startWith);
                sorted.Add(startWith);
                while (toSort.Any())
                {
                    Edge endconnection = new HashSet<Edge>(sorted.Last().EndVertex(primaryFace).AllEdges).Intersect(toSort).FirstOrDefault();
                    if (endconnection != null)
                    {
                        toSort.Remove(endconnection);
                        sorted.Add(endconnection);
                        continue;
                    }
                    Edge startconnection = new HashSet<Edge>(sorted.First().StartVertex(primaryFace).AllEdges).Intersect(toSort).FirstOrDefault();
                    if (startconnection != null)
                    {
                        toSort.Remove(startconnection);
                        sorted.Insert(0, startconnection);
                        continue;
                    }
                    break;
                }
                if (toSort.Count > 0) return null; // the edges are not in a consecutive path
                edges = sorted.ToArray(); // edges are now sorted
            }

            return null;
        }
        //        public static Shell RoundEdges(Shell toRound, Edge[] edges, double radius)
        //        {
        //            // the edges must be connected, and no more than three edges may connect in a vertex
        //            List<Face> fillets = new List<Face>(); // faces, that make the fillet
        //            Dictionary<Edge, Tuple<Face, Face>> tangentialIntersectionEdges = new Dictionary<Edge, Tuple<Face, Face>>();
        //            Dictionary<Edge, Tuple<ICurve, Face>> rawFillets = new Dictionary<Edge, Tuple<ICurve, Face>>(); // the axis curve and simple fillets for each edge without joints
        //            Dictionary<Vertex, List<Edge>> joiningVertices = new Dictionary<Vertex, List<Edge>>(); // for each vertex there may be up to three involved edges 
        //                                                                                                   // 1.: create the simple fillets
        //            foreach (Edge edg in edges)
        //            {
        //                if (edg.SecondaryFace == null) continue;
        //                GeoPoint edgcnt = edg.Curve3D.PointAt(0.5);
        //                GeoVector n1 = edg.PrimaryFace.Surface.GetNormal(edg.PrimaryFace.Surface.PositionOf(edgcnt));
        //                GeoVector n2 = edg.SecondaryFace.Surface.GetNormal(edg.SecondaryFace.Surface.PositionOf(edgcnt));
        //                GeoVector edgdir = edg.Curve3D.DirectionAt(0.5).Normalized;
        //                double orientation = edgdir * (n1.Normalized ^ n2.Normalized);
        //                bool outwardBendedEdge = edg.Forward(edg.PrimaryFace) == (orientation > 0);
        //                if (!outwardBendedEdge) continue; // inward bended edges cannot (yet) be rounded
        //                                                  // the intersection curve of the two faces of the edge, offset by radius, defines the axis of the rounded edge cylinder or extruded circle
        //                ISurface srfc1 = edg.PrimaryFace.Surface.GetOffsetSurface(-radius);
        //                ISurface srfc2 = edg.SecondaryFace.Surface.GetOffsetSurface(-radius);
        //                srfc1.SetBounds(edg.PrimaryFace.GetUVBounds());
        //                srfc2.SetBounds(edg.SecondaryFace.GetUVBounds()); // for BoxedSurfaceEx
        //                ICurve[] cvs = srfc1.Intersect(edg.PrimaryFace.GetUVBounds(), srfc2, edg.SecondaryFace.GetUVBounds());
        //                // there is a problem with the length of the curves: should use "Surfaces.Intersect(srfc1, srfc2);" and fix the length below
        //                if (cvs == null || cvs.Length == 0) continue;
        //                // if there are more than one intersection curves, take the one closest to the edge
        //                ICurve crv = cvs[0];
        //                double mindist = edg.Curve3D.DistanceTo(crv.PointAt(0.5));
        //                for (int i = 1; i < cvs.Length; i++)
        //                {
        //                    double dist = edg.Curve3D.DistanceTo(cvs[i].PointAt(0.5));
        //                    if (dist < mindist)
        //                    {
        //                        mindist = dist;
        //                        crv = cvs[i];
        //                    }
        //                }
        //                // clip the intersection curve to the length of the edge
        //                double pos1 = crv.PositionOf(edg.Vertex1.Position);
        //                double pos2 = crv.PositionOf(edg.Vertex2.Position);
        //                if (pos1 >= -1e-6 && pos1 <= 1.0 + 1e-6 && pos2 >= -1e-6 && pos2 <= 1.0 + 1e-6)
        //                {
        //                    if (pos1 < pos2) crv.Trim(pos1, pos2);
        //                    else
        //                    {
        //                        crv.Trim(pos2, pos1);
        //                        crv.Reverse(); // important, we expect _crv_ going from vertex1 to vertex2
        //                    }
        //                }
        //                else continue; // maybe the offset surfaces of a nurbs surface only intersect with a short intersection curve. What could you do in this case?
        //                crv.Extend(radius, radius); // make it long enough so that two adjacent fillets fully intersect
        //                // update the connection status, the vertex to involved edges list.
        //                if (!joiningVertices.TryGetValue(edg.Vertex1, out List<Edge> joining))
        //                {
        //                    joiningVertices[edg.Vertex1] = joining = new List<Edge>();
        //                }
        //                joining.Add(edg);
        //                if (!joiningVertices.TryGetValue(edg.Vertex2, out joining))
        //                {
        //                    joiningVertices[edg.Vertex2] = joining = new List<Edge>();
        //                }
        //                joining.Add(edg);

        //                // create the "cylindrical" rounding fillet (extruded arc along the curve, not necessary a cylindrical surface, if the axis is not a line)
        //                Ellipse arc = Ellipse.Construct();
        //                GeoVector dirx = edg.Curve3D.PointAt(edg.Curve3D.PositionOf(crv.StartPoint)) - crv.StartPoint;
        //                GeoVector diry = crv.StartDirection ^ dirx;
        //                dirx = crv.StartDirection ^ diry;
        //                Plane pln = new Plane(crv.StartPoint, dirx, diry); // Plane perpendicular to the start-direction of the axis-curve, plane's x-axis pointing away from edge.
        //                arc.SetArcPlaneCenterRadiusAngles(pln, crv.StartPoint, radius, Math.PI / 2.0, Math.PI);
        //                Face fillet = Make3D.ExtrudeCurveToFace(arc, crv);

        //                // create the tangential curves on the two faces
        //                ICurve tan1 = edg.PrimaryFace.Surface.Make3dCurve(edg.PrimaryFace.Surface.GetProjectedCurve(crv, 0.0));
        //                ICurve tan2 = edg.SecondaryFace.Surface.Make3dCurve(edg.SecondaryFace.Surface.GetProjectedCurve(crv, 0.0));
        //                ICurve2D tan12d = fillet.Surface.GetProjectedCurve(tan1, 0.0);
        //                ICurve2D tan22d = fillet.Surface.GetProjectedCurve(tan2, 0.0);
        //                SurfaceHelper.AdjustPeriodic(fillet.Surface, fillet.Domain, tan12d);
        //                SurfaceHelper.AdjustPeriodic(fillet.Surface, fillet.Domain, tan22d);
        //                Line2D l12d, l22d;
        //                if ((tan12d.StartPoint | tan22d.StartPoint) + (tan12d.EndPoint | tan22d.EndPoint) < (tan12d.StartPoint | tan22d.EndPoint) + (tan12d.EndPoint | tan22d.StartPoint))
        //                {
        //                    l12d = new Line2D(tan12d.StartPoint, tan22d.StartPoint);
        //                    l22d = new Line2D(tan12d.EndPoint, tan22d.EndPoint);
        //                }
        //                else
        //                {
        //                    l12d = new Line2D(tan12d.StartPoint, tan22d.EndPoint);
        //                    l22d = new Line2D(tan12d.EndPoint, tan22d.StartPoint);
        //                }
        //                // mark the edges, which are tangential onto the faces of the edge
        //                // these edges remain open and are later needed by the BooleanOperation to get tangential intersection curves
        //                tan12d.UserData.Add("BooleanOperation.OnFace1", true);
        //                tan22d.UserData.Add("BooleanOperation.OnFace2", true);
        //                Border bdr = Border.FromUnorientedList(new ICurve2D[] { tan12d, l12d, tan22d, l22d }, true);
        //                Face part = Face.MakeFace(fillet.Surface.Clone(), new SimpleShape(bdr));
        //                if (part != null)
        //                {
        //                    foreach (Edge fedg in part.Edges)
        //                    {
        //                        if (fedg.PrimaryCurve2D.UserData.Contains("BooleanOperation.OnFace1"))
        //                        {
        //                            tangentialIntersectionEdges[fedg] = new Tuple<Face, Face>(edg.PrimaryFace, part);
        //                            fedg.PrimaryCurve2D.UserData.Remove("BooleanOperation.OnFace1");
        //                        }
        //                        if (fedg.PrimaryCurve2D.UserData.Contains("BooleanOperation.OnFace2"))
        //                        {
        //                            tangentialIntersectionEdges[fedg] = new Tuple<Face, Face>(edg.SecondaryFace, part);
        //                            fedg.PrimaryCurve2D.UserData.Remove("BooleanOperation.OnFace2");
        //                        }
        //                    }
        //                }
        //                rawFillets[edg] = new Tuple<ICurve, Face>(crv, part); // here we keep the original fillets, maybe they will be modified when we make junctions between fillets
        //            }
        //#if DEBUG
        //            DebuggerContainer dcEdges = new DebuggerContainer();
        //            DebuggerContainer dcCurves = new DebuggerContainer();
        //            DebuggerContainer dcFillets = new DebuggerContainer();
        //            foreach (var item in rawFillets)
        //            {
        //                dcEdges.Add(item.Key.Curve3D as IGeoObject, item.Key.GetHashCode());
        //                dcCurves.Add(item.Value.Item1 as IGeoObject);
        //                dcFillets.Add(item.Value.Item2);
        //            }
        //#endif
        //            foreach (KeyValuePair<Vertex, List<Edge>> vertexToEdge in joiningVertices)
        //            {
        //                if (vertexToEdge.Value.Count == 1)
        //                {
        //                    // this is an open end of an edge
        //                    // we can tangentially extend the fillet with a cylindrical face, but we don't know how far,
        //                    // or we could somehow make a brutal end
        //                    Edge edg = vertexToEdge.Value[0];
        //                    ICurve filletAxis = rawFillets[edg].Item1;
        //                    Face filletFace = rawFillets[edg].Item2;
        //                    GeoPoint cnt;
        //                    GeoVector dir;
        //                    Ellipse arc = null;
        //                    if (vertexToEdge.Key == edg.Vertex1) // the axis curve goes from edg.Vertex1 to edg.Vertex2
        //                    {
        //                        cnt = filletAxis.StartPoint;
        //                        foreach (Edge edge in filletFace.Edges)
        //                        {
        //                            if (edge.Curve3D is Ellipse elli)
        //                            {
        //                                if (Precision.IsEqual(elli.Center, cnt))
        //                                {
        //                                    // this is the arc at the vertex we are checking here
        //                                    arc = elli;
        //                                    break;
        //                                }
        //                            }
        //                        }
        //                        dir = -filletAxis.StartDirection.Normalized;
        //                    }
        //                    else // if (vertexToEdge.Key == edg.Vertex2) // which mus be the case
        //                    {
        //                        cnt = filletAxis.EndPoint;
        //                        foreach (Edge edge in filletFace.Edges)
        //                        {
        //                            if (edge.Curve3D is Ellipse elli)
        //                            {
        //                                if (Precision.IsEqual(elli.Center, cnt))
        //                                {
        //                                    // this is the arc at the vertex we are checking here
        //                                    arc = elli;
        //                                    break;
        //                                }
        //                            }
        //                        }
        //                        dir = filletAxis.EndDirection.Normalized;
        //                    }
        //                    if (arc != null)
        //                    {
        //                        // create a cylindrical elongation at the end of the fillet
        //                        Line l1 = Line.TwoPoints(cnt, cnt + 2 * radius * dir); // 2*radius is arbitrary!
        //                        Face filletExtend = Make3D.ExtrudeCurveToFace(arc, l1);
        //                        // this cylindrical face has two line edges, which may or may not be tangential to the primary and secondary face of the rounded edge
        //                        foreach (Edge cedg in filletExtend.Edges)
        //                        {
        //                            if (cedg.Curve3D is Line l)
        //                            {
        //                                if (edg.PrimaryFace.Surface.GetDistance(l.PointAt(0.5)) < Precision.eps)
        //                                {   // the line probably lies in the face, i.e. tangential intersection
        //                                    tangentialIntersectionEdges[cedg] = new Tuple<Face, Face>(edg.PrimaryFace, filletExtend);
        //                                }
        //                                else if (edg.SecondaryFace.Surface.GetDistance(l.PointAt(0.5)) < Precision.eps)
        //                                {   // the line probably lies in the face, i.e. tangential intersection
        //                                    tangentialIntersectionEdges[cedg] = new Tuple<Face, Face>(edg.SecondaryFace, filletExtend);
        //                                }
        //                            }
        //                        }
        //                        fillets.Add(filletExtend);
        //                    }
        //                }
        //                else if (vertexToEdge.Value.Count == 2)
        //                {
        //                    // two edges (to be rounded) are connected at this vertex
        //                    // we try to intersect the two raw fillets. If they do intersect, we cut off the extend
        //                    // if they don't intersect, we add a toroidal fitting part
        //                    // we need to reconstruct tangentialIntersectionEdges when the BRep intersection modifies the faces (and shortens the tangential edges)
        //                    // we use UserData for this purpose.
        //                    GeoVector edge0dir, edg1dir;
        //                    if (vertexToEdge.Value[0].Vertex1 == vertexToEdge.Key) edge0dir = vertexToEdge.Value[0].Curve3D.StartDirection;
        //                    else edge0dir = vertexToEdge.Value[0].Curve3D.EndDirection;
        //                    if (vertexToEdge.Value[1].Vertex1 == vertexToEdge.Key) edg1dir = vertexToEdge.Value[1].Curve3D.StartDirection;
        //                    else edg1dir = vertexToEdge.Value[1].Curve3D.EndDirection;
        //                    if (Precision.SameNotOppositeDirection(edge0dir, edg1dir))
        //                    {
        //                        // nothing to do, the two faces should be connected
        //                    }
        //                    else
        //                    {
        //                        foreach (Edge edg in Extensions.Combine<Edge>(rawFillets[vertexToEdge.Value[0]].Item2.Edges, rawFillets[vertexToEdge.Value[1]].Item2.Edges))
        //                        {
        //                            if (edg.Curve3D is IGeoObject go) go.UserData["BrepFillet.OriginalEdge"] = edg;
        //                        }
        //                        rawFillets[vertexToEdge.Value[0]].Item2.UserData["BrepFillet.OriginalFace"] = rawFillets[vertexToEdge.Value[0]].Item2.GetHashCode();
        //                        rawFillets[vertexToEdge.Value[1]].Item2.UserData["BrepFillet.OriginalFace"] = rawFillets[vertexToEdge.Value[1]].Item2.GetHashCode();
        //                        BooleanOperation bo = new BooleanOperation(Shell.FromFaces(rawFillets[vertexToEdge.Value[0]].Item2), Shell.FromFaces(rawFillets[vertexToEdge.Value[1]].Item2), Operation.intersection);
        //                        bo.AllowOpenEdges = true; // the result should be a shell with two faces, namely the two clipped fillets
        //                        Shell[] bores = bo.Result();
        //                        if (bores != null && bores.Length == 1)
        //                        {
        //                            if (bores[0].Faces.Length == 2) // this should be the case: 
        //                            {
        //                                GeoObjectList fcs = bores[0].Decompose();
        //                                if (fcs.Count == 2)
        //                                {   // this should always be the case when there is an intersection
        //                                    // we have to replace the old faces and edges with the new ones in rawFillets and tangentialIntersectionEdges
        //                                    int hc0 = (int)fcs[0].UserData["BrepFillet.OriginalFace"];
        //                                    int hc1 = (int)fcs[1].UserData["BrepFillet.OriginalFace"];
        //                                    // we did save the HasCodes instead of the objects themselves, because cloning the UserData with a face, which has a UserData with a face... infinite loop
        //                                    Dictionary<Face, Face> oldToNew = new Dictionary<Face, Face>();
        //                                    if (hc0 == rawFillets[vertexToEdge.Value[0]].Item2.GetHashCode())
        //                                    {
        //                                        oldToNew[rawFillets[vertexToEdge.Value[0]].Item2] = fcs[0] as Face;
        //                                        oldToNew[rawFillets[vertexToEdge.Value[1]].Item2] = fcs[1] as Face;
        //                                    }
        //                                    else
        //                                    {
        //                                        oldToNew[rawFillets[vertexToEdge.Value[0]].Item2] = fcs[1] as Face;
        //                                        oldToNew[rawFillets[vertexToEdge.Value[1]].Item2] = fcs[0] as Face;
        //                                    }
        //                                    rawFillets[vertexToEdge.Value[0]] = new Tuple<ICurve, Face>(rawFillets[vertexToEdge.Value[0]].Item1, oldToNew[rawFillets[vertexToEdge.Value[0]].Item2]);
        //                                    rawFillets[vertexToEdge.Value[1]] = new Tuple<ICurve, Face>(rawFillets[vertexToEdge.Value[1]].Item1, oldToNew[rawFillets[vertexToEdge.Value[1]].Item2]);
        //                                    fcs[0].UserData.Remove("BrepFillet.OriginalFace");
        //                                    fcs[1].UserData.Remove("BrepFillet.OriginalFace");
        //                                    foreach (Edge edg in Extensions.Combine<Edge>((fcs[0] as Face).Edges, (fcs[1] as Face).Edges))
        //                                    {
        //                                        if (edg.Curve3D is IGeoObject go)
        //                                        {
        //                                            if (go.UserData.GetData("BrepFillet.OriginalEdge") is Edge edgorg)
        //                                            {
        //                                                if (tangentialIntersectionEdges.TryGetValue(edgorg, out Tuple<Face, Face> faces))
        //                                                {
        //                                                    tangentialIntersectionEdges[edg] = new Tuple<Face, Face>(faces.Item1, oldToNew[faces.Item2]);
        //                                                    tangentialIntersectionEdges.Remove(edgorg);
        //                                                }
        //                                                go.UserData.Remove("BrepFillet.OriginalEdge");
        //                                            }
        //                                        }
        //                                    }
        //                                }
        //                            }
        //                            else { }
        //                        }
        //                        else
        //                        {
        //                            // no proper intersection, try to add a toroidal junction between the two fillets
        //                            Face fc1 = rawFillets[vertexToEdge.Value[0]].Item2;
        //                            Face fc2 = rawFillets[vertexToEdge.Value[1]].Item2;
        //                            GeoPoint cnt1, cnt2;
        //                            GeoVector dir1, dir2;
        //                            if (vertexToEdge.Key == vertexToEdge.Value[0].Vertex1)
        //                            {
        //                                cnt1 = rawFillets[vertexToEdge.Value[0]].Item1.StartPoint;
        //                                dir1 = rawFillets[vertexToEdge.Value[0]].Item1.StartDirection;
        //                            }
        //                            else
        //                            {
        //                                cnt1 = rawFillets[vertexToEdge.Value[0]].Item1.EndPoint;
        //                                dir1 = rawFillets[vertexToEdge.Value[0]].Item1.EndDirection;
        //                            }
        //                            if (vertexToEdge.Key == vertexToEdge.Value[1].Vertex1)
        //                            {
        //                                cnt2 = rawFillets[vertexToEdge.Value[1]].Item1.StartPoint;
        //                                dir2 = rawFillets[vertexToEdge.Value[1]].Item1.StartDirection;
        //                            }
        //                            else
        //                            {
        //                                cnt2 = rawFillets[vertexToEdge.Value[1]].Item1.EndPoint;
        //                                dir2 = rawFillets[vertexToEdge.Value[1]].Item1.EndDirection;
        //                            }
        //                            if (Precision.SameDirection(dir1, dir2, false)) continue; // tangential connection, no need to make a joint
        //                            Ellipse arc1 = null, arc2 = null;
        //                            foreach (Edge edg in fc1.Edges)
        //                            {
        //                                if (edg.Curve3D is Ellipse elli && Precision.IsEqual(cnt1, elli.Center))
        //                                {
        //                                    arc1 = elli;
        //                                    break;
        //                                }
        //                            }
        //                            foreach (Edge edg in fc2.Edges)
        //                            {
        //                                if (edg.Curve3D is Ellipse elli && Precision.IsEqual(cnt2, elli.Center))
        //                                {
        //                                    arc2 = elli;
        //                                    break;
        //                                }
        //                            }
        //                            if (arc1 != null && arc2 != null) // which should always be the case
        //                            {
        //                                if (arc1.Plane.Intersect(arc2.Plane, out GeoPoint loc, out GeoVector tzaxis))
        //                                {
        //                                    GeoPoint tcnt = Geometry.DropPL(cnt1, loc, tzaxis); // center of the torus
        //                                    GeoVector txaxis = (tcnt - cnt1) + (tcnt - cnt2);
        //                                    // toroidal surface with a pole
        //                                    ToroidalSurface ts = new ToroidalSurface(tcnt, txaxis.Normalized, (tzaxis ^ txaxis).Normalized, tzaxis.Normalized, radius, radius);
        //                                    ICurve2D c2d1 = ts.GetProjectedCurve(arc1, 0.0);    // this should be lines with fixed u parameter (vertical 2d lines)
        //                                    ICurve2D c2d2 = ts.GetProjectedCurve(arc2, 0.0);
        //#if DEBUG
        //                                    ICurve dbgc1 = ts.Make3dCurve(c2d1);
        //                                    ICurve dbgc2 = ts.Make3dCurve(c2d2);
        //#endif
        //                                    BoundingRect pext = c2d1.GetExtent();
        //                                    SurfaceHelper.AdjustPeriodic(ts, pext, c2d2);
        //                                    pext.MinMax(c2d2.GetExtent());
        //                                    Face tfillet = Face.MakeFace(ts, pext); // this is a part of the torus, which connects the two rounding fillets
        //                                                                            // one of its edges is a pole
        //                                    fillets.Add(tfillet);
        //                                    // there should be a face which is connected to both involved edges
        //                                    Face commonFace = null;
        //                                    HashSet<Face> commonFaces = vertexToEdge.Value[0].Faces.Intersect(vertexToEdge.Value[1].Faces).ToHashSet();
        //                                    if (commonFaces.Count == 1) commonFace = commonFaces.First();
        //                                    foreach (Edge edg in tfillet.Edges)
        //                                    {
        //                                        if (edg.Curve3D is Ellipse elli)
        //                                        {
        //                                            if (Precision.IsEqual(elli.Center, arc1.Center)) continue;
        //                                            if (Precision.IsEqual(elli.Center, arc2.Center)) continue;
        //                                            if (commonFace != null)
        //                                            {   // this ellipse is the edge of the torus, which is not connected to arc1 or arc2 (not connected to the two rounding fillets)
        //                                                // if the common surface is a plane, this would be tangential
        //                                                if (commonFace.Surface.GetDistance(elli.PointAt(0.5)) < Precision.eps)
        //                                                {
        //                                                    tangentialIntersectionEdges[edg] = new Tuple<Face, Face>(commonFace, tfillet);
        //                                                }
        //                                            }
        //                                        }
        //                                    }
        //                                }
        //                            }
        //                        }
        //                    }
        //                }
        //                else if (vertexToEdge.Value.Count == 3)
        //                {
        //                    // find a sphere defined by the ending arcs of the three fillets, which connect in this vertex
        //                    Face fc1 = rawFillets[vertexToEdge.Value[0]].Item2;
        //                    Face fc2 = rawFillets[vertexToEdge.Value[1]].Item2;
        //                    Face fc3 = rawFillets[vertexToEdge.Value[2]].Item2;
        //                    GeoPoint cnt1, cnt2, cnt3;
        //                    GeoVector dir1, dir2, dir3;
        //                    double u1, u2, u3;
        //                    bool clipAtStart1, clipAtStart2, clipAtStart3;
        //                    if (vertexToEdge.Key == vertexToEdge.Value[0].Vertex1)
        //                    {
        //                        cnt1 = rawFillets[vertexToEdge.Value[0]].Item1.StartPoint;
        //                        dir1 = rawFillets[vertexToEdge.Value[0]].Item1.StartDirection;
        //                        u1 = 0;
        //                        clipAtStart1 = true;
        //                    }
        //                    else
        //                    {
        //                        cnt1 = rawFillets[vertexToEdge.Value[0]].Item1.EndPoint;
        //                        dir1 = rawFillets[vertexToEdge.Value[0]].Item1.EndDirection;
        //                        u1 = 1;
        //                        clipAtStart1 = false;
        //                    }
        //                    if (vertexToEdge.Key == vertexToEdge.Value[1].Vertex1)
        //                    {
        //                        cnt2 = rawFillets[vertexToEdge.Value[1]].Item1.StartPoint;
        //                        dir2 = rawFillets[vertexToEdge.Value[1]].Item1.StartDirection;
        //                        u2 = 0;
        //                        clipAtStart2 = true;
        //                    }
        //                    else
        //                    {
        //                        cnt2 = rawFillets[vertexToEdge.Value[1]].Item1.EndPoint;
        //                        dir2 = rawFillets[vertexToEdge.Value[1]].Item1.EndDirection;
        //                        u2 = 1;
        //                        clipAtStart2 = false;
        //                    }
        //                    if (vertexToEdge.Key == vertexToEdge.Value[2].Vertex1)
        //                    {
        //                        cnt3 = rawFillets[vertexToEdge.Value[2]].Item1.StartPoint;
        //                        dir3 = rawFillets[vertexToEdge.Value[2]].Item1.StartDirection;
        //                        u3 = 0;
        //                        clipAtStart3 = true;
        //                    }
        //                    else
        //                    {
        //                        cnt3 = rawFillets[vertexToEdge.Value[2]].Item1.EndPoint;
        //                        dir3 = rawFillets[vertexToEdge.Value[2]].Item1.EndDirection;
        //                        u3 = 1;
        //                        clipAtStart3 = false;
        //                    }
        //                    if (Precision.SameDirection(dir1, dir2, false)) continue; // tangential connection, cannot join with a sphere
        //                    if (Precision.SameDirection(dir1, dir3, false)) continue; // tangential connection, cannot join with a sphere
        //                    if (Precision.SameDirection(dir2, dir3, false)) continue; // tangential connection, cannot join with a sphere

        //                    double maxerror = GaussNewtonMinimizer.ThreeCurveIntersection(rawFillets[vertexToEdge.Value[0]].Item1, rawFillets[vertexToEdge.Value[1]].Item1, rawFillets[vertexToEdge.Value[2]].Item1, ref u1, ref u2, ref u3);
        //                    if (maxerror < Precision.eps)
        //                    {
        //                        Ellipse arc1 = null, arc2 = null, arc3 = null;
        //                        GeoPoint cnt = new GeoPoint(rawFillets[vertexToEdge.Value[0]].Item1.PointAt(u1), rawFillets[vertexToEdge.Value[1]].Item1.PointAt(u2), rawFillets[vertexToEdge.Value[2]].Item1.PointAt(u3));
        //                        // we want a spherical triangle which does not contain a pole. This is possible, since all arcs span less than 180°
        //                        Plane pln1 = new Plane(rawFillets[vertexToEdge.Value[0]].Item1.PointAt(u1), rawFillets[vertexToEdge.Value[0]].Item1.DirectionAt(u1));
        //                        Plane pln2 = new Plane(rawFillets[vertexToEdge.Value[1]].Item1.PointAt(u2), rawFillets[vertexToEdge.Value[1]].Item1.DirectionAt(u2));
        //                        Plane pln3 = new Plane(rawFillets[vertexToEdge.Value[2]].Item1.PointAt(u3), rawFillets[vertexToEdge.Value[2]].Item1.DirectionAt(u3));
        //                        // planes perpendicular to the extrusion curve in common intersection points of extrusion curves
        //                        // intersected with the appropriate rounding fillets should be the arcs, which also intersect with the sphere
        //                        ICurve[] icvs = rawFillets[vertexToEdge.Value[0]].Item2.GetPlaneIntersection(new PlaneSurface(pln1));
        //                        if (icvs != null && icvs.Length == 1) arc1 = icvs[0] as Ellipse;
        //                        else continue;
        //                        icvs = rawFillets[vertexToEdge.Value[1]].Item2.GetPlaneIntersection(new PlaneSurface(pln2));
        //                        if (icvs != null && icvs.Length == 1) arc2 = icvs[0] as Ellipse;
        //                        else continue;
        //                        icvs = rawFillets[vertexToEdge.Value[2]].Item2.GetPlaneIntersection(new PlaneSurface(pln3));
        //                        if (icvs != null && icvs.Length == 1) arc3 = icvs[0] as Ellipse;
        //                        else continue;
        //                        if (arc1 == null || arc2 == null || arc3 == null) continue; // should not happen
        //                        GeoPoint[] pnts = new GeoPoint[] { arc1.StartPoint, arc1.EndPoint, arc2.StartPoint, arc2.EndPoint, arc3.StartPoint, arc3.EndPoint };
        //                        Plane pln = Plane.FromPoints(pnts, out double maxdist, out bool isLinear);
        //                        if (isLinear || maxdist > Precision.eps) continue; // this should not happen, since there are only 3 points
        //                        GeoVector dirz = arc1.Plane.Normal ^ pln.Normal;
        //                        GeoVector dirx = arc1.Plane.Normal;
        //                        GeoVector diry = dirz ^ dirx;
        //                        SphericalSurface ss = new SphericalSurface(cnt, radius * dirx.Normalized, radius * diry.Normalized, radius * dirz.Normalized);
        //                        GeoPoint2D uv1 = rawFillets[vertexToEdge.Value[0]].Item2.Surface.PositionOf(cnt);

        //                        BoundingRect ext2d = new BoundingRect(ss.PositionOf(arc1.StartPoint));
        //                        GeoPoint2D uv = ss.PositionOf(arc1.EndPoint);
        //                        SurfaceHelper.AdjustPeriodic(ss, ext2d, ref uv);
        //                        ext2d.MinMax(uv);
        //                        uv = ss.PositionOf(arc2.StartPoint);
        //                        SurfaceHelper.AdjustPeriodic(ss, ext2d, ref uv);
        //                        ext2d.MinMax(uv);
        //                        uv = ss.PositionOf(arc2.EndPoint);
        //                        SurfaceHelper.AdjustPeriodic(ss, ext2d, ref uv);
        //                        ext2d.MinMax(uv);
        //                        uv = ss.PositionOf(arc3.StartPoint);
        //                        SurfaceHelper.AdjustPeriodic(ss, ext2d, ref uv);
        //                        ext2d.MinMax(uv);
        //                        uv = ss.PositionOf(arc3.EndPoint);
        //                        SurfaceHelper.AdjustPeriodic(ss, ext2d, ref uv);
        //                        ext2d.MinMax(uv);
        //                        ss.SetBounds(ext2d);
        //                        ICurve2D c2d1 = ss.GetProjectedCurve(arc1, 0.0);
        //                        ICurve2D c2d2 = ss.GetProjectedCurve(arc2, 0.0);
        //                        ICurve2D c2d3 = ss.GetProjectedCurve(arc3, 0.0);
        //                        SimpleShape outline = new SimpleShape(Border.FromUnorientedList(new ICurve2D[] { c2d1, c2d2, c2d3 }, true));
        //                        Face jointFillet = Face.MakeFace(ss, outline);
        //                        if (jointFillet != null) fillets.Add(jointFillet);
        //                        // now we must modify the fillets, i.e. clip them with the sphere. it is easier to clip them with a plane
        //                        // we need to reconstruct tangentialIntersectionEdges when the brep intersection modifies the faces (and shhortens the tangential edges)
        //                        // we use UserData for this purpose.
        //                        foreach (Edge edg in Extensions.Combine<Edge>(rawFillets[vertexToEdge.Value[0]].Item2.Edges, rawFillets[vertexToEdge.Value[1]].Item2.Edges, rawFillets[vertexToEdge.Value[2]].Item2.Edges))
        //                        {
        //                            if (edg.Curve3D is IGeoObject go) go.UserData["BrepFillet.OriginalEdge"] = edg;
        //                        }
        //                        rawFillets[vertexToEdge.Value[0]].Item2.UserData["BrepFillet.OriginalFace"] = rawFillets[vertexToEdge.Value[0]].Item2.GetHashCode();
        //                        rawFillets[vertexToEdge.Value[1]].Item2.UserData["BrepFillet.OriginalFace"] = rawFillets[vertexToEdge.Value[1]].Item2.GetHashCode();
        //                        rawFillets[vertexToEdge.Value[2]].Item2.UserData["BrepFillet.OriginalFace"] = rawFillets[vertexToEdge.Value[1]].Item2.GetHashCode();
        //                        Plane plnarc1 = arc1.Plane;
        //                        Plane plnarc2 = arc2.Plane;
        //                        Plane plnarc3 = arc3.Plane;
        //                        if ((rawFillets[vertexToEdge.Value[0]].Item1.DirectionAt(u1) * plnarc1.Normal < 0) != clipAtStart1) plnarc1.Reverse();
        //                        if ((rawFillets[vertexToEdge.Value[1]].Item1.DirectionAt(u2) * plnarc2.Normal < 0) != clipAtStart2) plnarc2.Reverse();
        //                        if ((rawFillets[vertexToEdge.Value[2]].Item1.DirectionAt(u3) * plnarc3.Normal < 0) != clipAtStart3) plnarc3.Reverse();
        //                        BoundingRect extarc1 = arc1.GetProjectedCurve(plnarc1).GetExtent();
        //                        BoundingRect extarc2 = arc2.GetProjectedCurve(plnarc2).GetExtent();
        //                        BoundingRect extarc3 = arc3.GetProjectedCurve(plnarc3).GetExtent();
        //                        extarc1.InflateRelative(1.1); // to make intersection and only return the clipped fillet
        //                        extarc2.InflateRelative(1.1);
        //                        extarc3.InflateRelative(1.1);
        //                        Face clipFace1 = Face.MakeFace(new PlaneSurface(plnarc1), extarc1);
        //                        Face clipFace2 = Face.MakeFace(new PlaneSurface(plnarc2), extarc2);
        //                        Face clipFace3 = Face.MakeFace(new PlaneSurface(plnarc3), extarc3);
        //                        Dictionary<Face, Face> oldToNew = new Dictionary<Face, Face>();
        //                        BooleanOperation bo = new BooleanOperation(Shell.FromFaces(rawFillets[vertexToEdge.Value[0]].Item2), Shell.FromFaces(clipFace1), Operation.intersection);
        //                        bo.AllowOpenEdges = true; // the result should be a shell with two faces, namely the two clipped fillets
        //                        Shell[] bores = bo.Result();
        //                        if (bores != null && bores.Length == 1 && bores[0].Faces.Length == 1)
        //                        {
        //                            Face clippedFillet = bores[0].Faces[0]; // the only face
        //                            oldToNew[rawFillets[vertexToEdge.Value[0]].Item2] = clippedFillet;
        //                            rawFillets[vertexToEdge.Value[0]] = new Tuple<ICurve, Face>(rawFillets[vertexToEdge.Value[0]].Item1, clippedFillet);
        //                            clippedFillet.UserData.Remove("BrepFillet.OriginalFace");
        //                        }
        //                        bo = new BooleanOperation(Shell.FromFaces(rawFillets[vertexToEdge.Value[1]].Item2), Shell.FromFaces(clipFace2), Operation.intersection);
        //                        bo.AllowOpenEdges = true; // the result should be a shell with two faces, namely the two clipped fillets
        //                        bores = bo.Result();
        //                        if (bores != null && bores.Length == 1 && bores[0].Faces.Length == 1)
        //                        {
        //                            Face clippedFillet = bores[0].Faces[0]; // the only face
        //                            oldToNew[rawFillets[vertexToEdge.Value[1]].Item2] = clippedFillet;
        //                            rawFillets[vertexToEdge.Value[1]] = new Tuple<ICurve, Face>(rawFillets[vertexToEdge.Value[1]].Item1, clippedFillet);
        //                            clippedFillet.UserData.Remove("BrepFillet.OriginalFace");
        //                        }
        //                        bo = new BooleanOperation(Shell.FromFaces(rawFillets[vertexToEdge.Value[2]].Item2), Shell.FromFaces(clipFace3), Operation.intersection);
        //                        bo.AllowOpenEdges = true; // the result should be a shell with two faces, namely the two clipped fillets
        //                        bores = bo.Result();
        //                        if (bores != null && bores.Length == 1 && bores[0].Faces.Length == 1)
        //                        {
        //                            Face clippedFillet = bores[0].Faces[0]; // the only face
        //                            oldToNew[rawFillets[vertexToEdge.Value[2]].Item2] = clippedFillet;
        //                            rawFillets[vertexToEdge.Value[2]] = new Tuple<ICurve, Face>(rawFillets[vertexToEdge.Value[2]].Item1, clippedFillet);
        //                            clippedFillet.UserData.Remove("BrepFillet.OriginalFace");
        //                        }
        //                        foreach (Edge edg in Extensions.Combine<Edge>(rawFillets[vertexToEdge.Value[0]].Item2.Edges, rawFillets[vertexToEdge.Value[1]].Item2.Edges, rawFillets[vertexToEdge.Value[2]].Item2.Edges))
        //                        {
        //                            if (edg.Curve3D is IGeoObject go)
        //                            {
        //                                if (go.UserData.GetData("BrepFillet.OriginalEdge") is Edge edgorg)
        //                                {
        //                                    if (tangentialIntersectionEdges.TryGetValue(edgorg, out Tuple<Face, Face> faces))
        //                                    {
        //                                        tangentialIntersectionEdges[edg] = new Tuple<Face, Face>(faces.Item1, oldToNew[faces.Item2]);
        //                                        tangentialIntersectionEdges.Remove(edgorg);
        //                                    }
        //                                    go.UserData.Remove("BrepFillet.OriginalEdge");
        //                                }
        //                            }
        //                        }

        //                    }
        //                }
        //            }
        //            foreach (KeyValuePair<Edge, Tuple<ICurve, Face>> item in rawFillets)
        //            {   // fillets contains only the junctions up to here
        //                item.Value.Item2.CopyAttributes(item.Key.PrimaryFace); // use the color of the primary face
        //                fillets.Add(item.Value.Item2);
        //            }
        //#if DEBUG
        //            DebuggerContainer dc = new DebuggerContainer();
        //            foreach (KeyValuePair<Edge, Tuple<Face, Face>> item in tangentialIntersectionEdges)
        //            {
        //                dc.Add(item.Key.Curve3D as IGeoObject);
        //            }
        //#endif
        //            Shell[] filletsShell = Make3D.SewFaces(fillets.ToArray()); // the faces are connected at the arcs, the tangential curves should remain unchanged
        //            if (filletsShell.Length == 1)
        //            {
        //                BooleanOperation bo = new BooleanOperation(toRound, filletsShell[0], tangentialIntersectionEdges, Operation.intersection);
        //                Shell[] res = bo.Result();

        //                if (res != null && res.Length == 1) return res[0];
        //            }
        //            return null;
        //        }
        public static Face[] ClipFace(Face toClip, Shell clipBy)
        {
            //BooleanOperation clip = new BooleanOperation(toClip, clipBy);
            //Shell[] parts = clip.Result();
            //List<Face> res = new List<Face>();
            //if (parts != null)
            //{
            //    for (int i = 0; i < parts.Length; i++)
            //    {
            //        GeoObjectList l = parts[i].Decompose();
            //        foreach (IGeoObject geoObject in l)
            //        {
            //            if (geoObject is Face fc) res.Add(fc);
            //        }
            //    }
            //}
            //return res.ToArray();
            throw new NotImplementedException();
        }
        public bool AllowOpenEdges
        {
            set => allowOpenEdges = value;
        }
        public bool DontCombineConnectedFaces
        {
            set => dontCombineConnectedFaces = value;
        }
        private void prepare()
        {
            BoundingCube ext1 = shell1.GetExtent(0.0);
            BoundingCube ext2 = shell2.GetExtent(0.0);
            BoundingCube ext = ext1;
            ext.MinMax(ext2);
            ext.Expand(ext.Size * 1e-6);
            Initialize(ext, ext.Size * 1e-6); // der OctTree
                                              // Alle edges und faces in den OctTree einfügen
            foreach (Edge edg in shell1.Edges)
            {
                base.AddObject(new BRepItem(this, edg));
#if DEBUG
                if (edg.Curve3D is InterpolatedDualSurfaceCurve) (edg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
#endif
            }
            foreach (Edge edg in shell2.Edges)
            {
                base.AddObject(new BRepItem(this, edg));
#if DEBUG
                if (edg.Curve3D is InterpolatedDualSurfaceCurve) (edg.Curve3D as InterpolatedDualSurfaceCurve).CheckSurfaceParameters();
#endif
            }
            foreach (Face fc in shell1.Faces)
            {
                base.AddObject(new BRepItem(this, fc));
            }
            foreach (Face fc in shell2.Faces)
            {
                base.AddObject(new BRepItem(this, fc));
            }

            edgesToSplit = new Dictionary<Edge, List<Vertex>>();
            intersectionVertices = new HashSet<IntersectionVertex>();
            facesToIntersectionVertices = new Dictionary<DoubleFaceKey, List<IntersectionVertex>>();

            findOverlappingFaces(); // setzt overlappingFaces, also Faces von verschiedenen shells, die sich teilweise überlappen oder identisch sind
            createEdgeFaceIntersections(); // Schnittpunkte von Edges mit Faces von verschiedenen shells
#if DEBUG
            DebuggerContainer dc3 = new DebuggerContainer();
            foreach (Edge edge in edgesToSplit.Keys)
            {
                dc3.Add(edge.Curve3D as IGeoObject, edge.GetHashCode());
            }
            foreach (IntersectionVertex iv in intersectionVertices)
            {
                Point pnt = Point.Construct();
                pnt.Location = iv.v.Position;
                pnt.Symbol = PointSymbol.Cross;
                dc3.Add(pnt, iv.v.GetHashCode());
            }
#endif
            if (operation != Operation.testonly)
            {   // Für testonly genügen die Kantenschnitte (fast)
                SplitEdges(); // mit den gefundenen Schnittpunkten werden die Edges jetzt gesplittet
                combineVertices(); // alle geometrisch identischen Vertices werden zusammengefasst
                removeIdenticalOppositeFaces(); // Paare von entgegengesetzt orientierten Faces mit identischer Fläche werden entfernt
                createNewEdges(); // faceToIntersectionEdges wird gesetzt, also für jedes Face eine Liste der neuen Schnittkanten
                                  // createInnerFaceIntersections(); // Schnittpunkte zweier Faces, deren Kanten sich aber nicht schneiden, finden
                                  // createInnerFaceIntersections erstmal weglassen, macht noch zu viele probleme (Futter5.cdb)
                combineEdges(); // hier werden intsEdgeToEdgeShell1, intsEdgeToEdgeShell2 und intsEdgeToIntsEdge gesetzt, die aber z.Z. noch nicht verwendet werden

            }
        }

        private void createInnerFaceIntersections()
        {   // Hier sind Schnittkurven gesucht, die nicht durch kanten gehen. Z.B. zwei Zylinder, die sich nur knapp berühren.
            // Die durch Kantenschnitte ausgelösten Schnittkurven werden ja schon mit "createEdgeFaceIntersections" gefunden
            HashSet<DoubleFaceKey> candidates = new HashSet<DoubleFaceKey>(); // Kandidaten für sich schneidende Faces
            List<Node<BRepItem>> leaves = new List<Node<BRepItem>>(Leaves);
            foreach (Node<BRepItem> node in leaves)
            {
                foreach (BRepItem first in node.list)
                {
                    if (first.Type == BRepItem.ItemType.Face)
                    {
                        Face f1 = first.face;
                        IGeoObjectOwner shell = f1.Owner;
                        foreach (BRepItem second in node.list)
                        {
                            if (second.Type == BRepItem.ItemType.Face)
                            {
                                if (second.face.Owner != shell)
                                {   //wir brauchen die richtige Ordnung in den DoubleFaceKey Objekten:
                                    DoubleFaceKey df;
                                    if (shell == shell1)
                                    {
                                        df = new DoubleFaceKey(f1, second.face);
                                    }
                                    else
                                    {
                                        df = new DoubleFaceKey(second.face, f1);
                                    }
                                    if (df.face1.Surface is PlaneSurface)
                                    {   // Fälle, in denen es keine Schnittkurven gibt, die nicht auch die kanten des einen oder anderen faces schneiden, überspringen
                                        if (df.face2.Surface is PlaneSurface) continue;
                                        if (df.face2.Surface is CylindricalSurface) continue;
                                        if (df.face2.Surface is ConicalSurface) continue;
                                    }
                                    else if (df.face2.Surface is PlaneSurface)
                                    {   // Fälle, in denen es keine Schnittkurven gibt, die nicht auch die kanten des einen oder anderen faces schneiden, überspringen
                                        if (df.face1.Surface is PlaneSurface) continue;
                                        if (df.face1.Surface is CylindricalSurface) continue;
                                        if (df.face1.Surface is ConicalSurface) continue;
                                    }
                                    {
                                        // zwei (Halb-)Zylinder oder (Halb-)Kugeln können nur eine innere Schnittkurve haben. Wenn durch die Kanten schon eine gefunden
                                        // wurde, dann  braucht hier nicht weiter getestet zu werden
                                        if (facesToIntersectionVertices.ContainsKey(df)) continue;
                                        if (facesToIntersectionVertices.ContainsKey(new DoubleFaceKey(df.face2, df.face1))) continue;
                                    }
                                    // es gibt sicherlich noch mehr ausschließbare Fälle
                                    candidates.Add(df);
                                }
                            }
                        }

                    }
                }
            }
            foreach (DoubleFaceKey df in candidates)
            {
                if (!df.face1.GetExtent(0.0).Interferes(df.face2.GetExtent(0.0))) continue;
                BoundingRect ext1, ext2;
                ext1 = df.face1.Area.GetExtent();
                ext2 = df.face2.Area.GetExtent();
                IDualSurfaceCurve[] innerCurves = Surfaces.IntersectInner(df.face1.Surface, ext1, df.face2.Surface, ext2);
                for (int i = 0; i < innerCurves.Length; i++)
                {   // es handelt sich immer um geschlossene Kurven
                    // Da es sich um echte innere Kurven handeln muss, also keine Schnitte mit den Kanten, genügt es, auf einen inneren Punkt zu testen
                    if (!df.face1.Area.Contains(df.face1.Surface.PositionOf(innerCurves[i].Curve3D.StartPoint), false)) continue;
                    if (!df.face2.Area.Contains(df.face2.Surface.PositionOf(innerCurves[i].Curve3D.StartPoint), false)) continue;
                    // es scheint besser zu sein, die Kurve aufzuteilen und zwei Kanten zu erzeugen
                    IDualSurfaceCurve[] parts = innerCurves[i].Split(0.5);
                    if (parts == null || parts.Length != 2) continue; // kommt nicht vor
                    Vertex v1 = null, v2 = null;
                    for (int j = 0; j < 2; j++)
                    {

                        bool dir = ((df.face1.Surface.GetNormal(parts[j].Curve2D1.StartPoint) ^ df.face2.Surface.GetNormal(parts[j].Curve2D2.StartPoint)) * parts[j].Curve3D.StartDirection) > 0;
                        Edge edge = new Edge(df.face1, parts[j].Curve3D, df.face1, parts[j].Curve2D1, dir, df.face2, parts[j].Curve2D1, !dir);
                        if (j == 0)
                        {
                            edge.MakeVertices();
                            v1 = edge.Vertex1;
                            v2 = edge.Vertex2;
                        }
                        else
                        {
                            edge.Vertex1 = v2;
                            edge.Vertex2 = v1;
                        }
                        HashSet<Edge> addTo;
                        if (!faceToIntersectionEdges.TryGetValue(df.face1, out addTo))
                        {
                            addTo = new HashSet<Edge>(); // (new EdgeComparerByVertex()); // damit werden zwei Kanten mit gleichen Vertices nicht zugefügt, nutzt nichts
                            faceToIntersectionEdges[df.face1] = addTo;
                        }
                        addTo.Add(edge);
                        if (!faceToIntersectionEdges.TryGetValue(df.face2, out addTo))
                        {
                            addTo = new HashSet<Edge>(); //  (new EdgeComparerByVertex());
                            faceToIntersectionEdges[df.face2] = addTo;
                        }
                        addTo.Add(edge);
                    }
                }
            }
        }

        private void TrimmIntersectionEdges()
        {
            HashSet<Edge> intersectionEdges = new HashSet<Edge>();
            Dictionary<(Face, Face, Face), Vertex> faceTripleToVertex = new Dictionary<(Face, Face, Face), Vertex>();
            foreach (var item in faceToIntersectionEdges.Values)
            {
                intersectionEdges.UnionWith(item);
            }
            foreach (Edge edge in intersectionEdges)
            {
                List<(double par, bool entering, Vertex v)> edgeIntersection = new List<(double, bool, Vertex)>();
                BRepItem[] closeObjects = GetObjectsCloseTo(edge.Curve3D as IOctTreeInsertable);
                for (int i = 0; i < closeObjects.Length; i++)
                {
                    if (closeObjects[i].face != null && closeObjects[i].face != edge.PrimaryFace && closeObjects[i].face != edge.SecondaryFace)
                    {
                        closeObjects[i].face.Surface.Intersect(edge.Curve3D, closeObjects[i].face.Domain, out GeoPoint[] ips, out GeoPoint2D[] uvs, out double[] us);
                        for (int j = 0; j < ips.Length; j++)
                        {
                            if (closeObjects[i].face.Contains(ips[j], true) && us[j] > Precision.eps && us[j] < 1.0 - Precision.eps)
                            {
                                // this is an intersection with a "third" face 
                                GeoVector curveDir = edge.Curve3D.DirectionAt(us[j]);
                                GeoVector surfaceDir = closeObjects[i].face.Surface.GetNormal(uvs[j]);
                                // if this is a tangential intersection at the beginning or end of the edge, ignore it. A typical scenario for this
                                // intersection is when two faces are tangentially connected and the intersection edge ends at a tangential edge
                                if (Math.Abs(curveDir * surfaceDir) < 1e-3 && (us[j] < Precision.eps * 100 || us[j] > 1.0 - Precision.eps * 100)) continue;
                                bool entering = curveDir * surfaceDir < 0.0;
                                (Face, Face, Face) faceTriple = FaceTriple(edge.PrimaryFace, edge.SecondaryFace, closeObjects[i].face);
                                Vertex vtx;
                                if (faceTripleToVertex.TryGetValue(faceTriple, out Vertex fvtx) && Precision.IsEqual(fvtx.Position, ips[j])) vtx = fvtx;
                                else faceTripleToVertex[faceTriple] = vtx = new Vertex(ips[j]);
                                edgeIntersection.Add((us[j], entering, vtx));
                            }
                        }
                    }
                }
                List<ICurve> parts = new List<ICurve>();
                HashSet<Vertex> useVertices = new HashSet<Vertex>();
                if (edgeIntersection.Count > 0)
                {
                    // this intersection edge crosses a third face. it must be split into several parts
                    edgeIntersection.Sort((a, b) => a.par.CompareTo(b.par));
                    for (int i = edgeIntersection.Count - 1; i > 0; --i)
                    {
                        if (edgeIntersection[i].v == edgeIntersection[i - 1].v) edgeIntersection.RemoveAt(i);
                    }
                    // which parts should we create?
                    // if the edge starts on an open edge, then the startpoint should not be used. It is the offset case where open edges are not connected by fillets
                    // which indicates, they are standing out
                    if (edgeIntersection.First().par > Precision.eps)
                    {
                        bool useVertex = false;
                        for (int i = 0; i < edge.Vertex1.Edges.Length; i++)
                        {
                            if (edge.Vertex1.Edges[i] != edge && edge.Vertex1.Edges[i].SecondaryFace != null)
                            {
                                useVertex = true;
                                break;
                            }
                        }
                        if (useVertex) edgeIntersection.Insert(0, (0.0, !edgeIntersection.First().entering, edge.Vertex1));
                    }
                    if (edgeIntersection.Last().par < 1.0 - Precision.eps)
                    {
                        bool useVertex = false;
                        for (int i = 0; i < edge.Vertex2.Edges.Length; i++)
                        {
                            if (edge.Vertex2.Edges[i] != edge && edge.Vertex2.Edges[i].SecondaryFace != null)
                            {
                                useVertex = true;
                                break;
                            }
                        }
                        if (useVertex) edgeIntersection.Add((1.0, !edgeIntersection.Last().entering, edge.Vertex2));
                    }
                    // now we create all parts. If it is more than one part, I am not sure, wether we need all parts. Maybe the loop algorithm in Result() is not working correct in this caes
                    for (int i = 0; i < edgeIntersection.Count - 1; i++)
                    {
                        ICurve part = edge.Curve3D.Clone();
                        part.Trim(edgeIntersection[i].par, edgeIntersection[i + 1].par);
                        parts.Add(part);
                    }
                    for (int i = 0; i < edgeIntersection.Count; i++) useVertices.Add(edgeIntersection[i].v);
                }
                if (parts.Count > 0)
                {
                    // remove this intersection edge and add the parts instead
                    faceToIntersectionEdges[edge.PrimaryFace].Remove(edge);
                    faceToIntersectionEdges[edge.SecondaryFace].Remove(edge);
                    for (int i = 0; i < parts.Count; i++)
                    {
                        ICurve2D primaryCurve2D = edge.PrimaryFace.Surface.GetProjectedCurve(parts[i], 0.0);
                        if (!edge.Forward(edge.PrimaryFace)) primaryCurve2D.Reverse();
                        SurfaceHelper.AdjustPeriodic(edge.PrimaryFace.Surface, edge.PrimaryFace.Domain, primaryCurve2D);
                        ICurve2D secondaryCurve2D = edge.SecondaryFace.Surface.GetProjectedCurve(parts[i], 0.0);
                        if (!edge.Forward(edge.SecondaryFace)) secondaryCurve2D.Reverse();
                        SurfaceHelper.AdjustPeriodic(edge.SecondaryFace.Surface, edge.SecondaryFace.Domain, secondaryCurve2D);
                        Edge trimmed = new Edge(edge.Owner, parts[i], edge.PrimaryFace, primaryCurve2D, edge.Forward(edge.PrimaryFace), edge.SecondaryFace, secondaryCurve2D, edge.Forward(edge.SecondaryFace));
                        trimmed.UseVertices(new HashSet<Vertex>(useVertices));
                        faceToIntersectionEdges[edge.PrimaryFace].Add(trimmed);
                        faceToIntersectionEdges[edge.SecondaryFace].Add(trimmed);
                    }
                }
            }
        }
        private void combineEdges()
        {
            // manche Schnittkanten sind identisch mit Kanten der Shells, nämlich genau dann, wenn Faces sich überlappen
            // (da die original-Kanten an den Durchstoßstellen aufgetielt sind und die Vertices zusammengefasst wurden,
            // lässt sich das rein combinatorisch bestimmen)
            // Es gibt zwei Fälle: 
            // 1. eine Schnittkante und die Kante einer Shell fallen zusammen
            // 2. eine Schittkante ist doppelt und zu jeder Shell gibt es eine passende Kante
            // (in beiden Fällen ist die Kante im Ergebnis nur einmal vertreten)
            // es sind auch Fälle denkbar, in denen eine Shell schon eine doppelte Kante hat. Die können wir hier nicht gebrauchen

            HashSet<Edge> intersectionEdges = new HashSet<Edge>();
            foreach (HashSet<Edge> se in faceToIntersectionEdges.Values)
            {
                intersectionEdges.UnionWith(se);
            }
            intsEdgeToEdgeShell1 = new Dictionary<Edge, Edge>(); // diese IntersectionEdge ist identisch mit dieser kante auf Shell1
            intsEdgeToEdgeShell2 = new Dictionary<Edge, Edge>();
            intsEdgeToIntsEdge = new Dictionary<Edge, Edge>(); // zwei intersectionEdges sind identisch
            foreach (Edge edg in intersectionEdges)
            {
                foreach (Edge other in edg.Vertex1.AllEdges)
                {
                    if (other != edg && (other.Vertex1 == edg.Vertex2 || other.Vertex2 == edg.Vertex2) && SameEdge(edg, other, this.precision))
                    {
                        if (other.PrimaryFace.Owner == shell1 && (other.SecondaryFace == null || other.SecondaryFace.Owner == shell1)) intsEdgeToEdgeShell1[edg] = other;
                        else if (other.PrimaryFace.Owner == shell2 && (other.SecondaryFace == null || other.SecondaryFace.Owner == shell2)) intsEdgeToEdgeShell2[edg] = other;
                        else intsEdgeToIntsEdge[edg] = other; // wird nochmal mit vertauschten Rollen gefunden
                    }
                }
            }
#if DEBUG
            DebuggerContainer dc1 = new CADability.DebuggerContainer();
            DebuggerContainer dc2 = new CADability.DebuggerContainer();
            DebuggerContainer dc3 = new CADability.DebuggerContainer();
            foreach (Edge edg in intsEdgeToEdgeShell1.Keys)
            {
                dc1.Add(edg.Curve3D as IGeoObject);
            }
            foreach (Edge edg in intsEdgeToEdgeShell2.Keys)
            {
                dc2.Add(edg.Curve3D as IGeoObject);
            }
            foreach (Edge edg in intsEdgeToIntsEdge.Keys)
            {
                dc3.Add(edg.Curve3D as IGeoObject);
            }
#endif
        }

        private void GenerateOverlappingIntersections()
        {
            //throw new NotImplementedException();
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

        private void findOverlappingFaces()
        {
            overlappingFaces = new Dictionary<DoubleFaceKey, ModOp2D>();
            oppositeFaces = new Dictionary<DoubleFaceKey, ModOp2D>();
            faceToOverlappingFaces = new Dictionary<Face, Dictionary<Face, ModOp2D>>();
            // Faces von verschiedenen Shells die identisch sind oder sich überlappen machen Probleme
            // beim Auffinden der Schnitte. Die Kanten und die Flächen berühren sich nur
            HashSet<DoubleFaceKey> candidates = new HashSet<DoubleFaceKey>(); // Kandidaten für parallele faces
            List<Node<BRepItem>> leaves = new List<Node<BRepItem>>(Leaves);
            Dictionary<Face, BRepItem> faceToBrepItem = new Dictionary<Face, BRepItem>();
            foreach (Node<BRepItem> node in leaves)
            {
                foreach (BRepItem first in node.list)
                {
                    if (first.Type == BRepItem.ItemType.Face)
                    {
                        Face f1 = first.face;
                        IGeoObjectOwner shell = f1.Owner;
                        foreach (BRepItem second in node.list)
                        {
                            if (second.Type == BRepItem.ItemType.Face)
                            {
                                if (second.face.Owner != shell)
                                {   //wir brauchen die richtige Ordnung in den DoubleFaceKey Objekten:
                                    if (shell == shell1)
                                    {
                                        candidates.Add(new DoubleFaceKey(f1, second.face));
                                    }
                                    else
                                    {
                                        candidates.Add(new DoubleFaceKey(second.face, f1));
                                    }
                                    faceToBrepItem[f1] = first;
                                    faceToBrepItem[second.face] = second; // to be able to remove from OctTree
                                }
                            }
                        }

                    }
                }
            }
            foreach (DoubleFaceKey df in candidates)
            {
                ModOp2D firstToSecond;
                BoundingRect ext1, ext2;
                df.face1.Surface.GetNaturalBounds(out ext1.Left, out ext1.Right, out ext1.Bottom, out ext1.Top);
                df.face2.Surface.GetNaturalBounds(out ext2.Left, out ext2.Right, out ext2.Bottom, out ext2.Top);
                // Achtung: SameGeometry geht z.Z. nur mit fester ModOp2D. Denkbar sind auch Fälle, bei denen es keine solche Modop gibt
                // aber dennoch die selbe Geometrie vorliegt: z.B. bei nicht-periodischem Zylinder oder verdrehter Kugel. Die brauchen wir aber auch!!!
                if (df.face1.Surface.SameGeometry(ext1, df.face2.Surface, ext2, this.precision, out firstToSecond))
                {   // es gilt nur, wenn die Orientierung die selbe ist
                    GeoVector n1 = df.face1.Surface.GetNormal(ext1.GetCenter());
                    GeoVector n2 = df.face2.Surface.GetNormal(firstToSecond * ext1.GetCenter());
                    if (n1 * n2 > 0)
                    {
                        overlappingFaces.Add(df, firstToSecond);
                        df.face1.UserData.Add("BRepIntersection.OverlapsWith", df.face2);
                        df.face2.UserData.Add("BRepIntersection.OverlapsWith", df.face1);
                    }
                    else
                    {
                        oppositeFaces.Add(df, firstToSecond);
                    }
                    if (!faceToOverlappingFaces.TryGetValue(df.face1, out var setToAddTo))
                    {
                        setToAddTo = new Dictionary<Face, ModOp2D>();
                        faceToOverlappingFaces[df.face1] = setToAddTo;
                    }
                    setToAddTo[df.face2] = firstToSecond;
                    if (!faceToOverlappingFaces.TryGetValue(df.face2, out setToAddTo))
                    {
                        setToAddTo = new Dictionary<Face, ModOp2D>();
                        faceToOverlappingFaces[df.face2] = setToAddTo;
                    }
                    setToAddTo[df.face1] = firstToSecond.GetInverse();
                }
            }

            // tried to split faces at touching points: not necessary
            //foreach (DoubleFaceKey df in candidates)
            //{
            //    GeoPoint[] tp = df.face1.Surface.GetTouchingPoints(df.face1.Area.GetExtent(), df.face2.Surface, df.face2.Area.GetExtent());
            //    if (tp != null && tp.Length > 0)
            //    {
            //        for (int i = 0; i < tp.Length; i++)
            //        {
            //            GeoPoint2D uv1 = df.face1.PositionOf(tp[i]);
            //            GeoPoint2D uv2 = df.face2.PositionOf(tp[i]);
            //            if (df.face1.Contains(ref uv1, true) && df.face2.Contains(ref uv2, true))
            //            {
            //                Face[] replace1 = df.face1.SplitUv(uv1);
            //                Face[] replace2 = df.face2.SplitUv(uv1);
            //                shell1.ReplaceFace(df.face1, replace1, precision);
            //                shell2.ReplaceFace(df.face2, replace2, precision);
            //                base.RemoveObject(faceToBrepItem[df.face1]);
            //                base.RemoveObject(faceToBrepItem[df.face2]);
            //                for (int j = 0; j < replace1.Length; j++)
            //                {
            //                    base.AddObject(new BRepItem(this, replace1[j]));
            //                }
            //                for (int j = 0; j < replace2.Length; j++)
            //                {
            //                    base.AddObject(new BRepItem(this, replace2[j]));
            //                }
            //            }
            //        }
            //    }
            //}

        }
        public CompoundShape SplitResult()
        {
            Shell[] shells = Result();
            CompoundShape res = new CompoundShape();
            for (int i = 0; i < shells.Length; i++)
            {
                foreach (Face fc in shells[i].Faces)
                {
                    ModOp2D fts;
                    if (fc.Surface is PlaneSurface && fc.Surface.SameGeometry(fc.Area.GetExtent(), splittingOnplane, fc.Area.GetExtent(), 0.0, out fts))
                    {
                        if (fts.IsIdentity)
                        { // this is a face created by the splitting plane
                            res.UniteDisjunct(fc.Area);
                        }
                    }
                }
            }
            return res;
        }
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
                AddUnique(Border.SignedArea(loop2d), (loop, loop2d));
            }
        }
        /// <summary>
        /// A dictionary with keys of type double, which manages adding same key entries by incrementing the key until the value can be added
        /// </summary>
        /// <typeparam name="T"></typeparam>
        private class UniqueDoubleReverseDictionary<T> : SortedDictionary<double, T>
        {
            private class CompareReverse : IComparer<double>
            {
                int IComparer<double>.Compare(double x, double y)
                {
                    return -x.CompareTo(y); ;
                }
            }
            public UniqueDoubleReverseDictionary() : base(new CompareReverse()) { }
            public void AddUnique(double d, T val)
            {
                while (this.ContainsKey(d)) d = Geometry.NextDouble(d);
                Add(d, val);
            }
        }
        private class SetEquality<T> : IEqualityComparer<HashSet<T>>
        {
            bool IEqualityComparer<HashSet<T>>.Equals(HashSet<T> x, HashSet<T> y)
            {
                return x.SetEquals(y);
            }

            int IEqualityComparer<HashSet<T>>.GetHashCode(HashSet<T> obj)
            {
                int res = 0;
                foreach (T item in obj)
                {
                    res ^= item.GetHashCode();
                }
                return res;
            }
        }

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
        public bool Unchanged => shellsAreUnchanged;
        public List<Face> UnusedFaces => unusedFaces;
        /// <summary>
        /// Get all face pairs, which intersect each other
        /// </summary>
        public HashSet<(Face, Face)> IntersectingFaces
        {
            get
            {
                HashSet<(Face, Face)> res = new HashSet<(Face, Face)>();
                foreach (KeyValuePair<Face, HashSet<Edge>> kv in faceToIntersectionEdges)
                {
                    Face faceToSplit = kv.Key;
                    HashSet<Edge> intersectionEdges = kv.Value;
                    foreach (Edge edg in intersectionEdges)
                    {
                        Face other = edg.OtherFace(faceToSplit);
                        (Face, Face) item;
                        if (other.GetHashCode() < faceToSplit.GetHashCode()) item = (other, faceToSplit);
                        else item = (faceToSplit, other);
                        res.Add(item);
                    }
                }
                return res;
            }
        }

        /// <summary>
        /// Looks for three edges which share three faces. These edges probably have a common intersection point
        /// </summary>
        public static bool TryFindIntersectingEdges(IEnumerable<Edge> edges, out Edge e1, out Edge e2, out Edge e3)
        {
            e1 = e2 = e3 = null;

            // 1. build adjacency list
            var adj = new Dictionary<Face, Dictionary<Face, List<Edge>>>();
            foreach (var edge in edges)
            {
                var fA = edge.PrimaryFace;
                var fB = edge.SecondaryFace;

                // fA → fB
                if (!adj.TryGetValue(fA, out var nbrA))
                {
                    nbrA = new Dictionary<Face, List<Edge>>();
                    adj[fA] = nbrA;
                }
                if (!nbrA.TryGetValue(fB, out var listAB))
                {
                    listAB = new List<Edge>();
                    nbrA[fB] = listAB;
                }
                listAB.Add(edge);

                // fB → fA 
                if (!adj.TryGetValue(fB, out var nbrB))
                {
                    nbrB = new Dictionary<Face, List<Edge>>();
                    adj[fB] = nbrB;
                }
                if (!nbrB.TryGetValue(fA, out var listBA))
                {
                    listBA = new List<Edge>();
                    nbrB[fA] = listBA;
                }
                listBA.Add(edge);
            }

            // 2. look for: f1 – f2 – f3 – f1
            foreach (var kv1 in adj)
            {
                var f1 = kv1.Key;
                foreach (var f2 in kv1.Value.Keys)
                {
                    if (f2.Equals(f1)) continue;
                    foreach (var f3 in adj[f2].Keys)
                    {
                        if (f3.Equals(f1) || f3.Equals(f2)) continue;
                        if (adj[f3].ContainsKey(f1))
                        {
                            // maybe we have even more edges, not sure how to test
                            e1 = adj[f1][f2].First(); // Edge f1–f2
                            e2 = adj[f2][f3].First(); // Edge f2–f3
                            e3 = adj[f3][f1].First(); // Edge f3–f1
                            return true;
                        }
                    }
                }
            }

            return false;
        }

        public Shell[] Result()
        {
            // this method relies on
            // - faceToIntersectionEdges: contains all faces, which intersect with other faces as keys and all the intersection edges, 
            //   which are produced by other faces on this face as values.
            // - overlappingFaces and oppositeFaces: they contain pairs of faces, which overlap, with either same or oppostie orientation.
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
                    dcFaces.Add(fce.Clone(), fce.GetHashCode()); // die Faces werden kaputt gemacht, deshalb hier clones merken
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
                    dcFaces.Add(fce.Clone(), fce.GetHashCode()); // die Faces werden kaputt gemacht, deshalb hier clones merken
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
            Dictionary<Face, DebuggerContainer> debugTrimmedFaces = new Dictionary<Face, DebuggerContainer>();
            foreach (KeyValuePair<Face, HashSet<Edge>> kv in faceToIntersectionEdges)
            {
                if (kv.Key.OutlineEdges == null || kv.Key.OutlineEdges.Length == 0) continue; // this are faces created from overlapping faces, which do not have outline edges
                debugTrimmedFaces[kv.Key] = new DebuggerContainer();
                debugTrimmedFaces[kv.Key].Add(kv.Key.Clone(), Color.Black, kv.Key.GetHashCode());
                int dbgclr = 1;
                foreach (Edge edg in kv.Value)
                {
                    Face other = edg.OtherFace(kv.Key);
                    if (other != null) debugTrimmedFaces[kv.Key].Add(other.Clone() as Face, DebuggerContainer.FromInt(dbgclr++), other.GetHashCode());
                }
            }
            Dictionary<Face, GeoObjectList> faceToMixedEdgesDebug = new Dictionary<Face, GeoObjectList>();
            foreach (KeyValuePair<Face, HashSet<Edge>> kv in faceToIntersectionEdges)
            {
                GeoObjectList l = new GeoObjectList();
                faceToMixedEdgesDebug[kv.Key] = l;
                l.Add(kv.Key);
                foreach (Edge edg in kv.Value)
                {
                    if (edg.Curve3D != null)
                    {
                        if (edg.Forward(kv.Key)) l.Add(edg.Curve3D as IGeoObject);
                        else
                        {
                            ICurve c3d = edg.Curve3D.Clone();
                            c3d.Reverse();
                            l.Add(c3d as IGeoObject);
                        }
                    }
                }
            }
#endif
#if DEBUG
            Dictionary<Face, DebuggerContainer> dbgFaceTointersectionEdges = new Dictionary<Face, DebuggerContainer>();
            foreach (KeyValuePair<Face, HashSet<Edge>> kv in faceToIntersectionEdges)
            {
                DebuggerContainer dc = new DebuggerContainer();
                dbgFaceTointersectionEdges[kv.Key] = dc;
                double arrowSize = kv.Key.Area.GetExtent().Size * 0.02;
                dc.Add(kv.Value, kv.Key, arrowSize, Color.Red, 0);
                dc.Add(kv.Key.Edges, kv.Key, arrowSize, Color.Blue, 0);
            }
#endif
#if DEBUG
            DebuggerContainer dcif = new DebuggerContainer();
            foreach (KeyValuePair<Face, HashSet<Edge>> kv in faceToIntersectionEdges)
            {
                dcif.Add(kv.Key, kv.Key.GetHashCode());
                foreach (Edge edg in kv.Value)
                {
                    dcif.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
                }
            }
            Dictionary<Face, DebuggerContainer> dbgEdgePositions = new Dictionary<Face, DebuggerContainer>();
            foreach (KeyValuePair<Face, HashSet<Edge>> kv in faceToIntersectionEdges)
            {
                DebuggerContainer dc = new DebuggerContainer();
                dbgEdgePositions[kv.Key] = dc;
                double arrowSize = kv.Key.Area.GetExtent().Size * 0.02;
                dc.Add(kv.Value, kv.Key, arrowSize, Color.Red, 0);
                dc.Add(kv.Key.Edges, kv.Key, arrowSize, Color.Blue, 0);
            }
#endif
            HashSet<Face> discardedFaces = new HashSet<Face>(faceToIntersectionEdges.Keys); // these faces may not appear in the final result, because they will be trimmed
            HashSet<Face> trimmedFaces = new HashSet<Face>(); // collection of faces which are trimmed (spitted, cut, edged) during this process
            VertexConnectionSet nonManifoldEdges = new VertexConnectionSet();
            HashSet<Face> nonManifoldCandidates = new HashSet<Face>(); // faces, which are only added because they contain nonManifoldEdges.
            foreach (KeyValuePair<Face, HashSet<Edge>> kv in faceToIntersectionEdges)
            {   // faceToIntersectionEdges contains all faces, which are intersected by faces of the relative other shell, as well as those intersection edges
                Face faceToSplit = kv.Key;
#if DEBUG       // show the faceToSplit and all other faces, which caused the intersectionEdges
                // does not work for overlapping faces
                debugTrimmedFaces.TryGetValue(kv.Key, out DebuggerContainer dcInvolvedFaces);
#endif
                HashSet<Edge> faceEdges = new HashSet<Edge>(faceToSplit.Edges.ToHashSet()); // all outline edges and holes of the face, used edges will be removed
                HashSet<Edge> intersectionEdges = kv.Value.Clone();
                HashSet<Edge> originalEdges = faceToSplit.Edges.ToHashSet();
                HashSet<Vertex> faceVertices = new HashSet<Vertex>(faceToSplit.Vertices);
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
                            nonManifoldEdges.Add(edg);
                            hasNonManifoldEdge = true;
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
                        // in Difference2.cdb.json we need to keep this edge
                        edg.DisconnectFromFace(faceToSplit);
                    }
                }
                if (commonOverlappingFaces.Contains(faceToSplit)) originalEdges.Clear(); // when this face is part of overlapping faces, we do not need the original edges (outline)
                                                                                         // because they are not part of the result
                                                                                         // now originalEdges contain all edges of the face, that could be used, intersectionEdges contain all edges that must be used
#if DEBUG       // show the original edges of the faceToSplit (blue) and the intersection edges (red) for this face, where duplicates and reverses are already removed
                // in this 2d display it should be easy to see, which loops should be generated
                double arrowSize = kv.Key.Area.GetExtent().Size * 0.01;
                DebuggerContainer dcedges = new DebuggerContainer();
                dcedges.Add(originalEdges, faceToSplit, arrowSize, Color.Blue, -1);
                dcedges.Add(intersectionEdges, faceToSplit, arrowSize, Color.Red, -1);
                foreach (Edge edg in intersectionEdges)
                {
                    dcedges.Add(edg.Vertex1.Position, Color.Blue, edg.Vertex1.GetHashCode());
                    dcedges.Add(edg.Vertex2.Position, Color.Blue, edg.Vertex2.GetHashCode());
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
#endif

                // If there is no positive loop, the outline loop of the face must be available and must be used.
                // actually we would have to topologically sort the loops in order to decide whether the faces outline must be used or not.
                // and what about the untouched holes of the face?
                double biggestArea = double.MinValue;
                foreach (double a in loops.Keys)
                {
                    if (Math.Abs(a) > Math.Abs(biggestArea)) biggestArea = a;
                }
                if (biggestArea < 0 && !commonOverlappingFaces.Contains(faceToSplit)) // when no loop, we don't need the outline
                {
                    foreach ((List<Edge>, ICurve2D[]) item in loops.Values) faceEdges.ExceptWith(item.Item1);
                    if (faceEdges.IsSupersetOf(faceToSplit.OutlineEdges))
                    {
                        // there is no outline loop, only holes (or nothing). We have to use the outline loop of the face, which is not touched
                        List<Edge> outline = new List<Edge>(faceToSplit.OutlineEdges);
                        loops.AddUnique(outline, faceToSplit);
                        faceEdges.ExceptWith(outline); // we would not need that
                    }
                }
                // we also add the holes of the faceToSplit, as long as it was not used by intersections and is not enclosed by a bigger hole
                // created from intersection edges
                for (int i = 0; i < faceToSplit.HoleCount; i++)
                {
                    if (faceEdges.IsSupersetOf(faceToSplit.HoleEdges(i))) // this hole is untouched by the loops
                    {
                        List<Edge> hole = new List<Edge>(faceToSplit.HoleEdges(i));
                        ICurve2D[] c2ds = faceToSplit.Get2DCurves(hole);
                        // find the closest loop, which encloses this faceToSplit's hole. 
                        // if this loop is positiv oriented (outline), then we need this hole
                        double area = Border.SignedArea(c2ds); // the area of this hole
                        double closest = double.MaxValue;
                        GeoPoint2D testPoint = c2ds[0].StartPoint; // some point on this loop to test, whether this loop is enclosed by another loop
                        double enclosedBy = 0.0;
                        foreach (var loop in loops)
                        {
                            if ((Math.Abs(loop.Key) > Math.Abs(area)) && (Math.Abs(loop.Key) < closest)) // an enclosing hole must be bigger.
                            {
                                if (Border.IsInside(loop.Value.Item2, testPoint) == (loop.Key > 0)) // IsInside respects orientation, that is why "== (loop.Key > 0)" is needed
                                {
                                    closest = Math.Abs(loop.Key);
                                    enclosedBy = loop.Key;
                                }
                            }
                        }
                        // in order to use a hole, it must be contained in a outer, positive loop
                        if (enclosedBy > 0.0) loops.AddUnique(area, (hole, c2ds));
                        faceEdges.ExceptWith(hole); // we would not need that
                    }
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
                        {
                            outlineToHoles[j].Add(i); // this hole (i) has found its outline (j)
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
                    System.Diagnostics.Debug.Assert(fc.CheckConsistency());
                    if (fc.GetHashCode() == 115)
                    {
                        SimpleShape ss = fc.Area;
                    }
#endif

                    trimmedFaces.Add(fc);
                }
            }
            if (operation == Operation.clip) return ClippedParts(trimmedFaces);
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
#endif
            // to avoid oppositeCommonFaces to be connected with the trimmedFaces, we destroy these faces
            foreach (Face fce in discardedFaces) fce.DisconnectAllEdges(); // to avoid connecting with discardedFaces
                                                                           // if we have two open edges in the trimmed faces which are identical, connect them
            Dictionary<DoubleVertexKey, Edge> trimmedEdges = new Dictionary<DoubleVertexKey, Edge>();
            foreach (Face fce in trimmedFaces)
            {
                foreach (Edge edg in fce.AllEdges)
                {
                    DoubleVertexKey dvk = new DoubleVertexKey(edg.Vertex1, edg.Vertex2);
                    if (nonManifoldEdges.Contains(edg, precision)) // is empty in most cases
                    {
                        if (edg.SecondaryFace != null)
                        {   // seperate nonManifold edges, they should not be used for collecting faces for the shell
                            edg.SecondaryFace.SeperateEdge(edg);
                        }
                    }
                    else if (edg.SecondaryFace == null || !trimmedFaces.Contains(edg.SecondaryFace) || !trimmedFaces.Contains(edg.PrimaryFace))
                    {   // only those edges, which 
                        if (trimmedEdges.TryGetValue(dvk, out Edge other))
                        {
                            if (other == edg) continue;
                            if (SameEdge(edg, other, precision))
                            {
                                if (edg.SecondaryFace != null)
                                {
                                    if (!trimmedFaces.Contains(edg.SecondaryFace)) edg.RemoveFace(edg.SecondaryFace);
                                    else if (!trimmedFaces.Contains(edg.PrimaryFace)) edg.RemoveFace(edg.PrimaryFace);
                                }
                                if (other.SecondaryFace != null)
                                {
                                    if (!trimmedFaces.Contains(other.SecondaryFace)) other.RemoveFace(other.SecondaryFace);
                                    else if (!trimmedFaces.Contains(other.PrimaryFace)) other.RemoveFace(other.PrimaryFace);
                                }
                                other.PrimaryFace.ReplaceEdge(other, edg);
                                trimmedEdges.Remove(dvk);
                            }
                        }
                        else
                        {
                            trimmedEdges[dvk] = edg;
                        }
                    }
                }
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
            while (added)
            {
                added = false;
                foreach (Face fce in allFaces.Clone()) // use a clone to be able to add faces to allfaces in this foreach loop
                {
                    foreach (Edge edg in fce.Edges)
                    {
                        if (!allFaces.Contains(edg.PrimaryFace))
                        {
                            if (!discardedFaces.Contains(edg.PrimaryFace) && edg.IsOrientedConnection)
                            {
                                allFaces.Add(edg.PrimaryFace);
                                added = true;
                            }
                            else
                            {
                                edg.DisconnectFromFace(edg.PrimaryFace);
                            }
                        }
                        if (edg.SecondaryFace != null && !allFaces.Contains(edg.SecondaryFace))
                        {
                            if (!discardedFaces.Contains(edg.SecondaryFace) && edg.IsOrientedConnection)
                            {
                                allFaces.Add(edg.SecondaryFace);
                                added = true;
                            }
                            else
                            {
                                edg.DisconnectFromFace(edg.SecondaryFace);
                            }
                        }
                        else if (edg.SecondaryFace == null && !nonManifoldEdges.Contains(edg, precision))
                        {
                            HashSet<Edge> connecting = new HashSet<Edge>(Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2));
                            connecting.Remove(edg);
                            if (connecting.Count > 1)
                            {
                                HashSet<Edge> toRemove = new HashSet<Edge>();
                                foreach (Edge ce in connecting)
                                {
                                    if (!SameEdge(ce, edg, precision)) toRemove.Add(ce);
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
            }
            if (allFaces.Count == 0 && discardedFaces.Count > 0)
            {   // there were no intersections, only identical opposite faces, like when glueing two parts together
                // this remains empty in case of intersection and returns the full body in case of union
                if (this.operation == Operation.union)
                {
                    allFaces.UnionWith(shell1.Faces);
                    allFaces.UnionWith(shell2.Faces);
                    allFaces.ExceptWith(discardedFaces);
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
            if (allFaces.Count == 0)
            {
                if (operation == Operation.union)
                {
                    shell1.ReverseOrientation(); // both shells have been reversed, undo this reversion
                    shell2.ReverseOrientation();
                    if (shell2.Contains(shell1.Vertices[0].Position)) res.Add(shell2); // shell2 contains shell1, the result ist shell2
                    else if (shell1.Contains(shell2.Vertices[0].Position)) res.Add(shell1); // shell1 contains shell2, the result ist shell1
                    else
                    {   // shells are disjunct, as union we return both
                        res.Add(shell1);
                        res.Add(shell2);
                    }
                }
                else if (operation == Operation.difference)
                {   // no intersection: shell1 - shell2, shell2 is reversed, i.e. IsInside means outside of the original
                    if (shell2.Contains(shell1.Vertices[0].Position)) res.Add(shell1); // shell1 remains unchanged, because shell2 is outside
                    else if (shell1.Contains(shell2.Vertices[0].Position))
                    {   // this is a solid with an inner hole. This is currently not implemented by Solid as it is very rarely used
                        // the correct result would be shell1 and shell2 in its reversed form
                        res.Add(shell1);
                        res.Add(shell2);
                    }
                    // else: the result is empty, because shell2 contains shell1 completely (nothing to do here, return empty result)
                }
                else if (operation == Operation.intersection)
                {
                    if (shell2.Contains(shell1.Vertices[0].Position)) res.Add(shell1); // shell2 contains shell1, the result ist shell1
                    else if (shell1.Contains(shell2.Vertices[0].Position)) res.Add(shell2); // shell1 contains shell2, the result ist shell2
                                                                                            // else: shells are disjunct, the result is empty

                }
            }
            foreach (Face fc in allFaces)
            {
                string[] allUserDataKeys = fc.UserData.AllItems;
                for (int i = 0; i < allUserDataKeys.Length; i++)
                {
                    if (allUserDataKeys[i].StartsWith("BRep")) fc.UserData.RemoveUserData(allUserDataKeys[i]);
                }
            }
            while (allFaces.Count > 0)
            {
                HashSet<Face> connected = extractConnectedFaces(allFaces, allFaces.First());
                Shell shell = Shell.MakeShell(connected.ToArray());
#if DEBUG
                bool ok = shell.CheckConsistency();
#endif
                // res should not have open edges! If so, something went wrong
                if (shell.HasOpenEdgesExceptPoles())
                {
                    shell.TryConnectOpenEdges();
                }
                if (allowOpenEdges || !shell.HasOpenEdgesExceptPoles())
                {
                    if (!dontCombineConnectedFaces) shell.CombineConnectedFaces(); // two connected faces which have the same surface are merged into one face
                    if (operation == Operation.union || operation == Operation.connectMultiple) shell.ReverseOrientation(); // both had been reversed and the intersection had been calculated
                    res.Add(shell);
                }
                else
                {
                    if (nonManifoldEdges.ContainsAll(shell.OpenEdges, precision))
                    {
                        nonManifoldParts.AddRange(shell.Faces);
                        shell = Shell.MakeShell(nonManifoldParts.ToArray());
                        shell.TryConnectOpenEdges();
                        if (!shell.HasOpenEdgesExceptPoles())
                        {
                            if (!dontCombineConnectedFaces) shell.CombineConnectedFaces(); // two connected faces which have the same surface are merged into one face
                            if (operation == Operation.union) shell.ReverseOrientation(); // both had been reversed and the intersection had been calculated
                            if (shell.Volume(Precision.eps) > Precision.eps * 100) res.Add(shell); // we sometimes get two identical faces, which are inverse oriented
                            nonManifoldParts.Clear();
                        }
                    }
                    else
                    {
                        // Try to fix missing faces. This helps to close shells, but it is actually a bug in the above code.
                        // when some of the faces are nonmanifold faces, we don't try to fix, because we know, these faces are uncertain candidates
                        try
                        {
                            if (operation != Operation.connectMultiple && !nonManifoldCandidates.Intersect(shell.Faces).Any() && TryFixMissingFaces(shell))
                            {
                                if (!dontCombineConnectedFaces)
                                    shell.CombineConnectedFaces(); // two connected faces which have the same surface are merged into one face
                                if (operation == Operation.union) shell.ReverseOrientation(); // both had been reversed and the intersection had been calculated
                                if (shell.Volume(Precision.eps) > Precision.eps * 100) res.Add(shell); // we sometimes get two identical faces, which are inverse oriented
                            }
                            shell.RecalcVertices();
                            shell.TryConnectOpenEdges();
                        }
                        catch (Exception) { }
                    }
                }
            }
            if (multipleFaces != null)
            {
                unusedFaces = new List<Face>(multipleFaces.Except(discardedFaces));
            }
            return res.ToArray();
        }

        private List<List<Edge>> FindLoops(Face onThisFace, HashSet<Edge> intersectionEdges, HashSet<Edge> originalEdges)
        {
            List<List<Edge>> found = new List<List<Edge>>();
            HashSet<Edge> availableEdges = new HashSet<Edge>(intersectionEdges.Union(originalEdges));
            // 
            Dictionary<Vertex, List<(Edge edge, double angle, bool outgoing)>> nodes = new Dictionary<Vertex, List<(Edge, double, bool)>>();
            foreach (Edge edge in availableEdges)
            {
                ICurve2D c2d = edge.Curve2D(onThisFace);
                Vertex sv = edge.StartVertex(onThisFace);
                Vertex ev = edge.EndVertex(onThisFace);
                if (!nodes.TryGetValue(sv, out List<(Edge edge, double angle, bool outgoing)> list)) nodes[sv] = list = new List<(Edge edge, double angle, bool outgoing)>();
                list.Add((edge, c2d.StartDirection.Angle, true));
                if (!nodes.TryGetValue(ev, out list)) nodes[ev] = list = new List<(Edge edge, double angle, bool outgoing)>();
                list.Add((edge, (-c2d.EndDirection).Angle, false));
            }
            foreach (KeyValuePair<Vertex, List<(Edge edge, double angle, bool outgoing)>> node in nodes)
            {
                node.Value.Sort((a, b) =>
                {
                    if (a.angle < b.angle) return 1;
                    if (a.angle > b.angle) return -1;
                    // TODO: same angle do some work
                    return 0;
                }
                );
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
                    int i = node.FindIndex(n => n.edge == current); // current is incoming endge into node i
                    if (i < 0)
                    {   // should never happen
                        current = null;
                        break;
                    }
                    Edge nextEdge = node[(i + 1) % node.Count].edge; // the next edge to the left
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
        /// After recombining the clipped faces and the original unmodified faces, there may be some faces missing. This is actually a bug, but with tangential faces
        /// it is almost impossible to avoid this. So here we try to find the faces, which could fill loops of open edges. We use the faces of the
        /// underlying shells of the operation as surfaces on which we create new faces, which fill the gaps.
        /// </summary>
        /// <param name="shell"></param>
        /// <returns></returns>
        private bool TryFixMissingFaces(Shell shell)
        {
            BoundingCube ext = shell1.GetExtent(0.0);
            ext.MinMax(shell2.GetExtent(0.0));
            // add all faces from the original shells to an octtree
            OctTree<Face> originalFaces = new OctTree<Face>(ext, ext.Size * 1e-6);
            foreach (Face face in shell1.Faces)
            {
                originalFaces.AddObject(face);
            }
            foreach (Face face in shell2.Faces)
            {
                originalFaces.AddObject(face);
            }
            // find original faces, which have the openedges of the shell to fix on them
            Dictionary<Face, List<Edge>> potentialFaces = new Dictionary<Face, List<Edge>>();
            foreach (Edge edge in shell.OpenEdges)
            {
                IEnumerable<Face> facesWithBothVertices = originalFaces.GetObjectsFromPoint(edge.Vertex1.Position).Intersect(originalFaces.GetObjectsFromPoint(edge.Vertex2.Position));
                foreach (Face face in facesWithBothVertices)
                {
                    double dist = Math.Abs(face.Surface.GetDistance(edge.Vertex1.Position)) + Math.Abs(face.Surface.GetDistance(edge.Vertex2.Position));
                    if (dist < Precision.eps * 10)
                    {
                        if (edge.Curve3D is IDualSurfaceCurve dsc) // which it often is
                        {
                            if (dsc.Surface1.SameGeometry(dsc.GetCurveOnSurface(dsc.Surface1).GetExtent(), face.Surface, face.Domain, Precision.eps, out _) ||
                                dsc.Surface2.SameGeometry(dsc.GetCurveOnSurface(dsc.Surface2).GetExtent(), face.Surface, face.Domain, Precision.eps, out _))
                            {
                                if (!potentialFaces.TryGetValue(face, out List<Edge> edges)) potentialFaces[face] = edges = new List<Edge>();
                                edges.Add(edge); // this edge fits to this original face
                            }
                        }
                        else
                        {
                            if (edge.Curve3D != null && Math.Abs(face.Surface.GetDistance(edge.Curve3D.PointAt(0.5))) < 10 * Precision.eps)
                            {   // the middle point of the edge is also close to this face
                                GeoPoint2D uv1 = face.Surface.PositionOf(edge.Vertex1.Position);
                                GeoPoint2D uv2 = face.Surface.PositionOf(edge.Vertex2.Position);
                                GeoPoint2D uv3 = face.Surface.PositionOf(edge.Curve3D.PointAt(0.5));
                                SurfaceHelper.AdjustPeriodic(face.Surface, face.Domain, ref uv1);
                                SurfaceHelper.AdjustPeriodic(face.Surface, face.Domain, ref uv2);
                                SurfaceHelper.AdjustPeriodic(face.Surface, face.Domain, ref uv3);
                                if (face.Area.Contains(uv1, true) && face.Area.Contains(uv2, true) && face.Area.Contains(uv3, true))
                                //if (face.Contains(edge.Vertex1.Position, true) && face.Contains(edge.Vertex2.Position, true) && face.Contains(edge.Curve3D.PointAt(0.5), true))
                                {
                                    if (!potentialFaces.TryGetValue(face, out List<Edge> edges)) potentialFaces[face] = edges = new List<Edge>();
                                    edges.Add(edge); // this edge fits to this original face
                                }
                            }
                        }
                    }
                }
            }
            // check for the faces with the most open edges on them
            List<Face> facesToAdd = new List<Face>();
            HashSet<Edge> stillOpenEdges = new HashSet<Edge>(shell.OpenEdges);
            // orient the surfaces according to the operation
            foreach (KeyValuePair<Face, List<Edge>> item in potentialFaces.OrderByDescending(kv => kv.Value.Count))
            {
                // the orientation of the shells are correct. In case of union they are reversed later
                List<ICurve2D> uvCurves = new List<ICurve2D>();
                Dictionary<ICurve2D, Edge> usedEdge = new Dictionary<ICurve2D, Edge>();
                foreach (Edge edge in item.Value)
                {
                    if (!stillOpenEdges.Contains(edge)) continue;
                    ICurve2D crv = item.Key.Surface.GetProjectedCurve(edge.Curve3D, 0.0);
                    SurfaceHelper.AdjustPeriodic(item.Key.Surface, item.Key.Domain, crv);
                    uvCurves.Add(crv);
                    usedEdge[crv] = edge;
                    crv.UserData.Add("usedEdge", edge);
                }
                CompoundShape cs = CompoundShape.CreateFromList(uvCurves.ToArray(), Precision.eps);
                if (cs != null && cs.SimpleShapes.Length == 1)
                {
                    List<Edge[]> faceBounds = new List<Edge[]>();
                    List<Edge> outline = new List<Edge>();
                    foreach (ICurve2D segment in cs.SimpleShapes[0].Segments)
                    {
                        if (segment.UserData.Contains("usedEdge"))
                        {
                            Edge found = segment.UserData.GetData("usedEdge") as Edge;
                            outline.Add(found);
                            stillOpenEdges.Remove(found);
                            segment.UserData.Remove("usedEdge");
                        }
                        else
                        {
                            // create a new edge, which doesn't correspond to an open existing edge. This should not happen
                            return false;
                        }
                    }
                    faceBounds.Add(outline.ToArray());
                    for (int i = 0; i < cs.SimpleShapes[0].NumHoles; i++)
                    {
                        List<Edge> hole = new List<Edge>();
                        foreach (ICurve2D segment in cs.SimpleShapes[0].Hole(i).Segments)
                        {
                            if (segment.UserData.Contains("usedEdge"))
                            {
                                Edge found = segment.UserData.GetData("usedEdge") as Edge;
                                hole.Add(found);
                                stillOpenEdges.Remove(found);
                                segment.UserData.Remove("usedEdge");
                            }
                            else
                            {
                                // create a new edge, which doesn't correspond to an open existing edge. This should not happen
                                return false;
                            }
                        }
                        faceBounds.Add(hole.ToArray());
                    }
                    // facesToAdd.AddIfNotNull();
                    Face newFace = Face.MakeFace(item.Key.Surface, faceBounds.ToArray());
                    if (newFace != null) shell.AddIntegratedFace(newFace, true);
                }
                if (!stillOpenEdges.Any()) break; // all open edges have been used
            }

            return shell.OpenEdgesExceptPoles.Length == 0;
        }

        private Shell[] ClippedParts(HashSet<Face> trimmedFaces)
        {
            List<Face> clipped = new List<Face>();
            foreach (Face face in trimmedFaces)
            {
                if (face.UserData.Contains("BRepIntersection.IsPartOf"))
                {
                    int original = (int)face.UserData.GetData("BRepIntersection.IsPartOf");
                    if (original == shell1.GetHashCode())
                    {
                        face.UserData.Remove("BRepIntersection.IsPartOf");
                        clipped.Add(face);
                    }
                }
            }
            List<Shell> res = new List<Shell>();
            for (int i = 0; i < clipped.Count; i++)
            {
                Shell part = Shell.MakeShell(new Face[] { clipped[i] });
                res.Add(part);
            }
            return res.ToArray();
        }

        private List<Edge> FindLoop(Edge edg, Vertex startVertex, Face onThisFace, HashSet<Edge> intersectionEdges, HashSet<Edge> originalEdges)
        {
            List<Edge> res = new List<Edge>();
            res.Add(edg);
            if (startVertex == null) startVertex = edg.StartVertex(onThisFace);
            Vertex endVertex = edg.EndVertex(onThisFace);
            HashSet<Vertex> usedVertices = new HashSet<Vertex>(); // to encounter inner loops
            usedVertices.Add(startVertex);
            while (!usedVertices.Contains(endVertex))
            {
                List<Edge> connected = endVertex.ConditionalEdges(delegate (Edge e)
                {
                    if (!intersectionEdges.Contains(e)) return false;
                    return e.StartVertex(onThisFace) == endVertex;
                });

                if (connected.Count > 1) // can intersection edges contain poles?
                {
                    // filter a pole:
                    for (int i = 0; i < connected.Count; i++)
                    {
                        if (connected[i].Curve3D == null)
                        {
                            res.Add(connected[i]); // insert a pole, usedVertices and endVertex stay the same
                            intersectionEdges.Remove(connected[i]); // so we don't find it again
                            connected.RemoveAt(i);
                            break;
                        }
                    }
                }
                if (connected.Count > 1)
                {   // very rare case:
                    // multiple intersection edges start from the current edge
                    // there should be only one valid path to the startVertex
                    List<Edge> toAdd = null;
                    int besti = -1;
                    double maxangle = double.MinValue;
                    ICurve2D lastCurve = res[res.Count - 1].Curve2D(onThisFace);
                    // there are more than two intersection edges on this face connected in a single point. this could be two cylinders with a touching point
                    // here we have to use the curve which turns most to the left, otherwise we will get a chain of (intersection-) edgeswhich are self crossing.
                    // in theory the "most to the left"-decision could be difficult, when two connected curves have the same starting direction, but one turns more to the left
                    // no such case fund until now
                    for (int i = 0; i < connected.Count; i++)
                    {
                        SweepAngle swa = new SweepAngle(lastCurve.EndDirection, connected[i].Curve2D(onThisFace).StartDirection);
                        if (swa.Radian > maxangle)
                        {
                            maxangle = swa.Radian;
                            besti = i;
                        }
                    }
                    if (besti >= 0)
                    {
                        List<Edge> sub = FindLoop(connected[besti], startVertex, onThisFace, intersectionEdges, originalEdges);
                        if (sub != null)
                        {
                            if (toAdd != null) throw new ApplicationException("BRepOpration: cannot find loop"); // should never happen
                            toAdd = sub;
                        }
                    }
                    if (toAdd != null)
                    {
                        res.AddRange(toAdd);
                        endVertex = startVertex;
                        break; // we are done, the subLoop ends at startVertex
                    }
                    else
                    {
                        intersectionEdges.ExceptWith(res);
                        originalEdges.ExceptWith(res);
                        return null; // no path leads to the startVertex
                    }
                }
                else if (connected.Count == 1)
                {   // there is exactely one intersection edge starting at endVertex: use this edges
                    res.Add(connected[0]);
                    usedVertices.Add(endVertex);
                    endVertex = connected[0].EndVertex(onThisFace);
                    continue;
                }
                bool intersectionEdgeEndHere = false;
                bool lastEdgeIsOutline = originalEdges.Contains(res[res.Count - 1]);
                connected = endVertex.ConditionalEdges(delegate (Edge e)
                {
                    if (lastEdgeIsOutline && intersectionEdges.Contains(e) && e.EndVertex(onThisFace) == endVertex) intersectionEdgeEndHere = true; // there is an intersection edge ending at this current endvertex
                    if (!originalEdges.Contains(e)) return false;
                    return e.StartVertex(onThisFace) == endVertex;
                });
                if (connected.Count > 1)
                {
                    // filter a pole:
                    for (int i = 0; i < connected.Count; i++)
                    {
                        if (connected[i].Curve3D == null)
                        {
                            res.Add(connected[i]); // insert a pole, usedVertices and endVertex stay the same
                            intersectionEdges.Remove(connected[i]); // so we don't find it again
                            connected.RemoveAt(i);
                            break;
                        }
                    }
                }
                if (connected.Count == 0 || intersectionEdgeEndHere)
                {
                    // (connected.Count == 0): dead end, no connection at endVertex
                    // intersectionEdgeEndHere: cannot go on, because we are following original edges and crossing at a vertex, 
                    // where an intersection edge ends. This is not allowed (e.g.: breps4)
                    intersectionEdges.ExceptWith(res);
                    originalEdges.ExceptWith(res);
                    return null;
                }
                else if (connected.Count == 1)
                {
                    res.Add(connected[0]);
                    usedVertices.Add(endVertex);
                    endVertex = connected[0].EndVertex(onThisFace);
                }
                else
                {
                    throw new ApplicationException("BRepOpration: cannot find loop, too many connections"); // should never happen
                }
            }
            if (startVertex != endVertex)
            {
                // if we have encountered an inner loop: remove all edges at the beginning until we reach an edge starting at endVertex
                while (res.Count > 0 && res[0].StartVertex(onThisFace) != endVertex) res.RemoveAt(0);
            }
            intersectionEdges.ExceptWith(res);
            originalEdges.ExceptWith(res);
            return res;
        }


        private void SubtractCommonFaces(HashSet<Face> oppositeCommonFaces)
        {
            foreach (KeyValuePair<Face, HashSet<Face>> kv in faceToCommonFaces)
            {
                if (faceToIntersectionEdges.TryGetValue(kv.Key, out HashSet<Edge> intersectionEdges))
                {
                    // remove this face from faceToIntersectionEdges and add new faces
                    List<Face> result = new List<Face>();
                    result.Add(kv.Key);
                    foreach (Face fc in kv.Value) // subtract all these faces
                    {
                        List<Face> remaining = new List<Face>();
                        for (int i = 0; i < result.Count; i++)
                        {
                            bool secondIsOpposite = fc.UserData.ContainsData("BRepIntersection.IsOpposite");
                            List<Face> diff = Difference(result[i], fc, ModOp2D.Identity, secondIsOpposite);
                            if (diff.Count == 0)
                            {   // all or nothing
                                Dictionary<Face, HashSet<Edge>> common = Common(result[i], fc, ModOp2D.Identity);
                                if (common.Count == 0) remaining.Add(result[i]);
                            }
                            else
                            {
                                remaining.AddRange(diff);
                            }
                        }
                        result = remaining;
                    }
                    for (int i = 0; i < result.Count; i++)
                    {
                        HashSet<Edge> isedgs = new HashSet<Edge>();
                        foreach (Edge edg in intersectionEdges)
                        {
                            if (edg.PrimaryFace == kv.Key || edg.SecondaryFace == kv.Key)
                            {   // otherwise edg has already been related to another result[i]
                                GeoPoint2D mp = edg.Curve2D(kv.Key).PointAt(0.5);
                                if (result[i].Contains(ref mp, true))
                                {
                                    edg.ReplaceFace(kv.Key, result[i]);
                                    isedgs.Add(edg);
                                }
                            }
                        }
                        faceToIntersectionEdges.Add(result[i], isedgs);
                    }
                    faceToIntersectionEdges.Remove(kv.Key); // this one has been replaced by result[i], which is a part of it
                }
            }

        }

        public static bool IsSameFace(HashSet<Edge> edges, HashSet<Vertex> vertices, Face fce, double precision)
        {
            if (vertices != null)
            {
                HashSet<Vertex> fcev = new HashSet<Vertex>(fce.Vertices);
                if (!vertices.SetEquals(fcev))
                {
                    return false; // must have exactely the same vertices to be equal
                }
            }
            foreach (Edge edg in edges)
            {
                bool edgeFound = false;
                foreach (Edge edg1 in Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2))
                {
                    if (edg1.PrimaryFace == fce || edg1.SecondaryFace == fce)
                    {
                        if (SameEdge(edg, edg1, precision)) edgeFound = true;
                    }
                }
                if (!edgeFound) return false;
            }
            return true;
        }

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

        private HashSet<Face> CollectOverlappingCommonFaces(HashSet<Face> discardedFaces)
        {
            HashSet<Face> commonFaces = new HashSet<Face>();
            foreach (KeyValuePair<DoubleFaceKey, ModOp2D> ov in overlappingFaces)
            {
#if DEBUG
                DebuggerContainer dc = new DebuggerContainer();
                dc.Add(ov.Key.face1);
                dc.Add(ov.Key.face2);
#endif
                Dictionary<Face, HashSet<Edge>> common = Common(ov.Key.face1, ov.Key.face2, ov.Value.GetInverse());
                if (common.Count > 0)
                {
                    discardedFaces.Add(ov.Key.face1);
                    discardedFaces.Add(ov.Key.face2);
                    HashSet<Face> ftc;
                    if (!faceToCommonFaces.TryGetValue(ov.Key.face1, out ftc)) faceToCommonFaces[ov.Key.face1] = ftc = new HashSet<Face>();
                    ftc.UnionWith(common.Keys);
                    if (!faceToCommonFaces.TryGetValue(ov.Key.face2, out ftc)) faceToCommonFaces[ov.Key.face2] = ftc = new HashSet<Face>();
                    ftc.UnionWith(common.Keys); // use the same faces, if we make clones, these clones will not be used in the result, but still exist when collecting faces
                                                //foreach (Face fce in common.Keys)
                                                //{
                                                //    Face clone = fce.CloneWithVertices();
                                                //    clone.ReplaceSurface(ov.Key.face2.Surface, ov.Value);
                                                //    ftc.Add(clone);
                                                //}
                }
                foreach (KeyValuePair<Face, HashSet<Edge>> item in common)
                {
                    commonFaces.Add(item.Key);
#if DEBUG
                    Face dbf = item.Key.Clone() as Face;
                    dbf.ColorDef = new ColorDef("violet", Color.Violet);
                    dc.Add(dbf);
#endif
                }
            }
            return commonFaces;
        }
        private HashSet<Face> CollectTotallyCoveredFaces()
        {
            HashSet<Face> res = new HashSet<Face>();
            foreach (KeyValuePair<Face, HashSet<Face>> item in faceToCommonFaces)
            {
                CompoundShape cs = new CompoundShape(item.Key.Area);
                foreach (Face fc in item.Value)
                {
                    bool dbg = fc.UserData.Contains("BRepIntersection.IsOpposite");
                    cs.Subtract(fc.Area);
                }
                if (cs.Empty) res.Add(item.Key);
            }
            return res;
        }
        private HashSet<Face> CollectOppositeCommonFaces(HashSet<Face> discardedFaces)
        {
            HashSet<Face> commonFaces = new HashSet<Face>();
            foreach (KeyValuePair<DoubleFaceKey, ModOp2D> op in oppositeFaces)
            {
                Dictionary<Face, HashSet<Edge>> common = Common(op.Key.face1, op.Key.face2, op.Value.GetInverse());
                if (common.Count > 0)
                {
                    discardedFaces.Add(op.Key.face1);
                    discardedFaces.Add(op.Key.face2);
                    op.Key.face1.UserData["BRepIntersection.OppositeKey"] = true;
                    op.Key.face2.UserData["BRepIntersection.OppositeKey"] = true;
                    HashSet<Face> ftc;
                    if (!faceToCommonFaces.TryGetValue(op.Key.face1, out ftc)) faceToCommonFaces[op.Key.face1] = ftc = new HashSet<Face>();
                    ftc.UnionWith(common.Keys);
                    if (!faceToCommonFaces.TryGetValue(op.Key.face2, out ftc)) faceToCommonFaces[op.Key.face2] = ftc = new HashSet<Face>();
                    foreach (Face fce in common.Keys)
                    {
                        Face clone = fce.CloneWithVertices();
                        clone.ModifySurface(op.Key.face2.Surface, op.Value); // if op.Value.Determinant<0 then the 2d edges are reversed
                        ftc.Add(clone);
                        clone.UserData["BRepIntersection.IsOpposite"] = true; // clone is only used to subtract from other faces, here we need to mark that it is in the opposite faces
                        commonFaces.Add(clone); // 
                    }
                }
                commonFaces.UnionWith(common.Keys);
            }
            return commonFaces;
        }

        private void ReplaceFaceToMixedEdges(Face toReplace, IEnumerable<Face> faces)
        {
            throw new NotImplementedException();
        }

        [Obsolete]
        private void ReduceOverlappingFaces(HashSet<Face> generatedFaces)
        {
            // overlappingFaces and oppositeFaces contain pairs of faces, that share the same surface and have the same or opposite orientation
            // Here we reduce these faces (split them into parts) so that the remaining parts don't overlap.
            // With same oriented overlapping, we make 3 parts: the symmetric difference and the common part. With opposite oriented overlapping,
            // we only make the symmetric difference parts. The intersection edges must be distributed onto the splitted parts
            // all new created faces are collected in generatedFaces
            Dictionary<Face, HashSet<Face>> replacedBy = new Dictionary<Face, HashSet<Face>>(); // this face from faceToIntersectionEdges has been replaced by these Faces
            while (overlappingFaces.Count > 0)
            {
                KeyValuePair<DoubleFaceKey, ModOp2D> kv = overlappingFaces.FirstOrDefault();
                // Split the two faces into 3 categories, each may have multiple faces or can be empty:
                // face1 minus face2, face2 minus face1 and common. And distribute the intersection edges of the original faces to the splitted faces
                Dictionary<Face, HashSet<Edge>> f1MinusF2 = DifferenceDeprecated(kv.Key.face1, kv.Key.face2, kv.Value.GetInverse(), false);
                Dictionary<Face, HashSet<Edge>> f2MinusF1 = DifferenceDeprecated(kv.Key.face2, kv.Key.face1, kv.Value, false);
                Dictionary<Face, HashSet<Edge>> common = Common(kv.Key.face1, kv.Key.face2, kv.Value.GetInverse());
                overlappingFaces.Remove(kv.Key);
                List<DoubleFaceKey> toRemove = new List<DoubleFaceKey>();
                List<KeyValuePair<DoubleFaceKey, ModOp2D>> toAdd = new List<KeyValuePair<DoubleFaceKey, ModOp2D>>();
                // now we have three sets of new faces. If face1 or face2 are also involved in other overlappings, we have to replace face1 by f1MinusF2 and common and
                // face2 by f2MinusF1 and common in these entries, i.e. remove the entries containing face1 or face2 and add new entries containing the splitted faces instead.
                foreach (KeyValuePair<DoubleFaceKey, ModOp2D> ov in overlappingFaces)
                {
                    if (ov.Key.face1 == kv.Key.face1)
                    {
                        toRemove.Add(ov.Key);
                        foreach (KeyValuePair<Face, HashSet<Edge>> kv1 in Enumerable.Concat(f1MinusF2, common))
                        {
                            DoubleFaceKey dfk = new DoubleFaceKey(kv1.Key, ov.Key.face2);
                            toAdd.Add(new KeyValuePair<DoubleFaceKey, ModOp2D>(dfk, ov.Value));
                        }
                    }
                    if (ov.Key.face2 == kv.Key.face2)
                    {
                        toRemove.Add(ov.Key);
                        foreach (KeyValuePair<Face, HashSet<Edge>> kv1 in Enumerable.Concat(f2MinusF1, common))
                        {
                            DoubleFaceKey dfk = new DoubleFaceKey(ov.Key.face1, kv1.Key);
                            toAdd.Add(new KeyValuePair<DoubleFaceKey, ModOp2D>(dfk, ov.Value));
                        }
                    }
                }
                foreach (DoubleFaceKey tr in toRemove)
                {
                    overlappingFaces.Remove(tr);
                }
                foreach (KeyValuePair<DoubleFaceKey, ModOp2D> ta in toAdd)
                {
                    overlappingFaces.Add(ta.Key, ta.Value);
                }
                faceToIntersectionEdges.Remove(kv.Key.face1);
                faceToIntersectionEdges.Remove(kv.Key.face2);
                foreach (KeyValuePair<Face, HashSet<Edge>> kv1 in Enumerable.Concat(Enumerable.Concat(f1MinusF2, common), f2MinusF1))
                {
                    faceToIntersectionEdges.Add(kv1.Key, kv1.Value);
                    generatedFaces.Add(kv1.Key);
                }
            }
            while (oppositeFaces.Count > 0)
            {
                KeyValuePair<DoubleFaceKey, ModOp2D> kv = oppositeFaces.FirstOrDefault();
                // Split the two faces into 3 categories, each may have multiple faces or can be empty:
                // face1 minus face2, face2 minus face1 and common. And distribute the intersection edges of the original faces to the splitted faces
                Dictionary<Face, HashSet<Edge>> f1MinusF2 = DifferenceDeprecated(kv.Key.face1, kv.Key.face2, kv.Value.GetInverse(), true);
                Dictionary<Face, HashSet<Edge>> f2MinusF1 = DifferenceDeprecated(kv.Key.face2, kv.Key.face1, kv.Value, true);
                oppositeFaces.Remove(kv.Key);
                List<DoubleFaceKey> toRemove = new List<DoubleFaceKey>();
                List<KeyValuePair<DoubleFaceKey, ModOp2D>> toAdd = new List<KeyValuePair<DoubleFaceKey, ModOp2D>>();
                foreach (KeyValuePair<DoubleFaceKey, ModOp2D> ov in oppositeFaces)
                {
                    if (ov.Key.face1 == kv.Key.face1)
                    {
                        toRemove.Add(ov.Key);
                        foreach (KeyValuePair<Face, HashSet<Edge>> kv1 in f1MinusF2)
                        {
                            DoubleFaceKey dfk = new DoubleFaceKey(kv1.Key, ov.Key.face2);
                            toAdd.Add(new KeyValuePair<DoubleFaceKey, ModOp2D>(dfk, ov.Value));
                        }
                    }
                    if (ov.Key.face2 == kv.Key.face2)
                    {
                        toRemove.Add(ov.Key);
                        foreach (KeyValuePair<Face, HashSet<Edge>> kv1 in f2MinusF1)
                        {
                            DoubleFaceKey dfk = new DoubleFaceKey(ov.Key.face1, kv1.Key);
                            toAdd.Add(new KeyValuePair<DoubleFaceKey, ModOp2D>(dfk, ov.Value));
                        }
                    }
                }
                foreach (DoubleFaceKey tr in toRemove)
                {
                    oppositeFaces.Remove(tr);
                }
                foreach (KeyValuePair<DoubleFaceKey, ModOp2D> ta in toAdd)
                {
                    oppositeFaces.Add(ta.Key, ta.Value);
                }
                faceToIntersectionEdges.Remove(kv.Key.face1);
                faceToIntersectionEdges.Remove(kv.Key.face2);
                foreach (KeyValuePair<Face, HashSet<Edge>> kv1 in Enumerable.Concat(f1MinusF2, f2MinusF1))
                {
                    faceToIntersectionEdges.Add(kv1.Key, kv1.Value);
                    generatedFaces.Add(kv1.Key);
                }
            }
        }

        private Dictionary<Face, HashSet<Edge>> Common(Face face1, Face face2, ModOp2D secondToFirst)
        {
            bool reverseSecond = secondToFirst.Determinant < 0;
            //if (reverseSecond)
            //{   // a very common case: the faces are identical but reversed
            //    SimpleShape s1 = face1.Area;
            //    SimpleShape s2 = face2.Area;
            //    bool identical = true;
            //    s2 = s2.GetModified(secondToFirst);
            //    foreach (ICurve2D segment in s1.Segments)
            //    {
            //        if (!s2.IsPointOnBorder(segment.EndPoint,Precision.eps))
            //        {
            //            identical = false;
            //            break;
            //        }
            //    }
            //    foreach (ICurve2D segment in s2.Segments)
            //    {
            //        if (!s1.IsPointOnBorder(segment.EndPoint, Precision.eps))
            //        {
            //            identical = false;
            //            break;
            //        }
            //    }
            //    CompoundShape cs = SimpleShape.Subtract(s1, s2);
            //    if (cs.Empty)
            //    {

            //    }
            //}
            Dictionary<Face, HashSet<Edge>> res = new Dictionary<Face, HashSet<Edge>>();
            HashSet<Edge> toUse = new HashSet<Edge>();
            HashSet<Edge> ie1 = new HashSet<Edge>(); // empty set
            HashSet<Edge> ie2 = new HashSet<Edge>(); // empty set
            if (faceToIntersectionEdges.TryGetValue(face1, out HashSet<Edge> ie11)) ie1.UnionWith(ie11);
            if (faceToIntersectionEdges.TryGetValue(face2, out HashSet<Edge> ie22)) ie2.UnionWith(ie22);
            ie1.UnionWith(face1.Edges.ToHashSet());
            ie2.UnionWith(face2.Edges.ToHashSet());
            Face fc = Face.Construct(); // a placeholder for orientation only, it will not be fully constructed
            BoundingRect domain = face1.Area.GetExtent();
            foreach (Edge edg in face1.Edges)
            {
                // Add all edges of face1, which are inside face2
                // if face2 has an intersection edge identical to this edge, then it is inside face2
                HashSet<Edge> insideFace2 = new HashSet<Edge>(Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2).Intersect(ie2));
                bool isInside = false, isOpposite = false;
                foreach (Edge edgi in insideFace2)
                {
                    if (SameEdge(edgi, edg, precision))
                    {
                        if (reverseSecond != (edgi.StartVertex(face2) == edg.StartVertex(face1))) isInside = true; // same direction
                                                                                                                   // else isOpposite = true;
                                                                                                                   //break;
                    }
                }
                if (!isInside && isOpposite) continue;
                if (!isInside)
                {   // not all edges are intersection edges: e.g. two concentric cylinders with the same radius, but different seams
                    if (face2.Contains(edg.Curve3D.PointAt(0.5), false)) isInside = true; // better in 2d with firstToSecond changed to false (don't accept on curve) because of RohrHalter1.cdb.json
                }
                if (isInside)
                {
                    Edge clone = edg.CloneWithVertices();
                    clone.SetPrimary(fc, edg.Curve2D(face1).Clone(), edg.Forward(face1));
                    toUse.Add(clone);
                }
            }
            foreach (Edge edg in face2.Edges)
            {
                // Add all edges of face2, which are inside face1
                HashSet<Edge> connectingEdges = new HashSet<Edge>(Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2));
                HashSet<Edge> cmn = connectingEdges.Intersect(toUse).ToHashSet();
                if (cmn.Count > 0 && cmn.Any(e => SameEdge(e, edg, precision)))
                {
                    continue; // this edge is common to face1 and face2, we already have it in toUse
                }
                HashSet<Edge> insideFace1 = connectingEdges.Intersect(ie1).ToHashSet();
                bool isInside = false, isOpposite = false;
                foreach (Edge edgi in insideFace1)
                {
                    if (SameEdge(edgi, edg, precision))
                    {
                        if (reverseSecond != (edgi.StartVertex(face1) == edg.StartVertex(face2))) isInside = true; // same direction
                                                                                                                   // else isOpposite = true; // commented out because of breps2b
                                                                                                                   // break;
                    }
                }
                if (!isInside && isOpposite) continue;
                if (!isInside)
                {   // not all edges are intersection edges: e.g. two concentric cylinders with the same radius, but different seams
                    if (face1.Contains(edg.Curve3D.PointAt(0.5), false)) isInside = true; // changed to false, see above
                }
                if (isInside)
                {
                    Edge clone = edg.CloneWithVertices();
                    ICurve2D c2d;
                    if ((clone.Curve3D is InterpolatedDualSurfaceCurve))
                    {
                        bool onSurface1 = false;
                        if ((clone.Curve3D as InterpolatedDualSurfaceCurve).Surface1 == face2.Surface) onSurface1 = true;
                        else if ((clone.Curve3D as InterpolatedDualSurfaceCurve).Surface2 == face2.Surface) onSurface1 = false;
                        else if ((clone.Curve3D as InterpolatedDualSurfaceCurve).Surface1.SameGeometry(((clone.Curve3D as InterpolatedDualSurfaceCurve).Surface1 as ISurfaceImpl).usedArea, face2.Surface, (face2.Surface as ISurfaceImpl).usedArea, precision, out ModOp2D dumy)) onSurface1 = true;
                        else if ((clone.Curve3D as InterpolatedDualSurfaceCurve).Surface2.SameGeometry(((clone.Curve3D as InterpolatedDualSurfaceCurve).Surface2 as ISurfaceImpl).usedArea, face2.Surface, (face2.Surface as ISurfaceImpl).usedArea, precision, out dumy)) onSurface1 = false;
                        (clone.Curve3D as InterpolatedDualSurfaceCurve).ReplaceSurface(face2.Surface, face1.Surface, secondToFirst);
                        if (onSurface1) c2d = (clone.Curve3D as InterpolatedDualSurfaceCurve).CurveOnSurface1;
                        else c2d = (clone.Curve3D as InterpolatedDualSurfaceCurve).CurveOnSurface2;
                        if (reverseSecond) c2d.Reverse();
                    }
                    else
                    {
                        c2d = edg.Curve2D(face2).GetModified(secondToFirst);
                    }
                    SurfaceHelper.AdjustPeriodic(face1.Surface, domain, c2d);
                    clone.SetPrimary(fc, c2d, edg.Forward(face2));
                    if (reverseSecond) clone.Reverse(fc);
                    toUse.Add(clone);
                }
            }
            HashSet<Edge> toDisconnect = toUse.Clone(); // toUse will be empty after GetLoops. We need to disconnect the edges from fc at the end
            List<List<Edge>> loops = GetLoops(toUse, fc);
            Dictionary<List<Edge>, List<List<Edge>>> loopsToHoles = SortLoopsTopologically(loops, fc);
            foreach (KeyValuePair<List<Edge>, List<List<Edge>>> loopToHoles in loopsToHoles)
            {
                Face face = Face.Construct();
                foreach (Edge edg in loopToHoles.Key) edg.ReplaceFace(fc, face);
                for (int i = 0; i < loopToHoles.Value.Count; i++)
                {
                    for (int j = 0; j < loopToHoles.Value[i].Count; j++)
                    {
                        loopToHoles.Value[i][j].ReplaceFace(fc, face);
                    }
                }
                face.Set(face1.Surface.Clone(), loopToHoles.Key, loopToHoles.Value);
                // the common part cannot contain intersection edges, or can it??? If so, see below (Difference)
                face.CopyAttributes(face1);
#if DEBUG
                face.UserData.Clear();
                face.UserData.Add("PartOf", face1.GetHashCode() + 100000 * face2.GetHashCode());
#endif
                res[face] = new HashSet<Edge>(); // empty set, the common part cannot contain intersection edges, 
                                                 // because they would have to intersect both faces, which would mean a self intersection on one shell
            }
            foreach (Edge edg in toDisconnect)
            {
                edg.DisconnectFromFace(fc);
            }
            return res;

        }

        /// <summary>
        /// Make the difference face1 - face2 (which can be any number of faces, including none) and distribute the intersection edges of face1
        /// to the resulting faces
        /// </summary>
        /// <param name="face1"></param>
        /// <param name="face2"></param>
        /// <param name="secondToFirst"></param>
        /// <returns></returns>
        private Dictionary<Face, HashSet<Edge>> DifferenceDeprecated(Face face1, Face face2, ModOp2D secondToFirst, bool secondIsOpposite)
        {
            Dictionary<Face, HashSet<Edge>> res = new Dictionary<Face, HashSet<Edge>>();
            HashSet<Edge> toUse = new HashSet<Edge>();
            if (!faceToIntersectionEdges.TryGetValue(face1, out HashSet<Edge> ie1)) ie1 = new HashSet<Edge>(); // empty set
            if (!faceToIntersectionEdges.TryGetValue(face2, out HashSet<Edge> ie2)) ie2 = new HashSet<Edge>(); // empty set
            ie1.UnionWith(face1.Edges.ToHashSet());
            ie2.UnionWith(face2.Edges.ToHashSet());
            Face fc = Face.Construct(); // a placeholder for orientation only, it will not be fully constructed
            BoundingRect domain = face1.Area.GetExtent();
            foreach (Edge edg in face1.Edges)
            {
                // Add all edges of face1, which are not inside face2
                // if face2 has an intersection edge identical to this edge, then it is inside face2
                HashSet<Edge> insideFace2 = (new HashSet<Edge>(Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2))).Intersect(ie2).ToHashSet();
                if (insideFace2.Count == 0)
                {
                    Edge clone = edg.CloneWithVertices();
                    clone.SetPrimary(fc, edg.Curve2D(face1), edg.Forward(face1));
                    toUse.Add(clone);
                }
            }
            foreach (Edge edg in face2.Edges)
            {
                // Add all edges of face2, which are inside face1
                HashSet<Edge> connecting = new HashSet<Edge>(Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2));
                HashSet<Edge> insideFace1 = connecting.Intersect(ie1).ToHashSet(); // can be more than one
                bool isInside = false;
                foreach (Edge edgi in insideFace1)
                {
                    if (SameEdge(edgi, edg, precision))
                    {
                        isInside = true;
                        break;
                    }
                }
                if (isInside)
                {
                    HashSet<Edge> onFace1 = connecting.Intersect(face1.Edges.ToHashSet()).ToHashSet();
                    bool notOnFace1 = true;
                    foreach (Edge edg1 in onFace1)
                    {
                        if (SameEdge(edg1, edg, precision) && (edg1.StartVertex(face1) == edg.StartVertex(face2) != secondIsOpposite)) notOnFace1 = false;
                    }
                    if (notOnFace1)
                    {   // edg is not an edge on face1
                        Edge clone = edg.CloneWithVertices();
                        ICurve2D c2d = edg.Curve2D(face2).GetModified(secondToFirst);
                        //c2d = face1.Surface.GetProjectedCurve(edg.Curve3D, 0.0);
                        //if (!edg.Forward(face2)) c2d.Reverse();
                        SurfaceHelper.AdjustPeriodic(face1.Surface, domain, c2d);
                        clone.SetPrimary(fc, c2d, edg.Forward(face2));
                        if (!secondIsOpposite) clone.Reverse(fc);
                        toUse.Add(clone);
                    }
                }
            }
            if (toUse.Count > 0)
            {
                List<List<Edge>> loops = GetLoops(toUse, fc);
                Dictionary<List<Edge>, List<List<Edge>>> loopsToHoles = SortLoopsTopologically(loops, fc);
                foreach (KeyValuePair<List<Edge>, List<List<Edge>>> loopToHoles in loopsToHoles)
                {
                    Face face = Face.Construct();
                    foreach (Edge edg in loopToHoles.Key) edg.ReplaceFace(fc, face);
                    for (int i = 0; i < loopToHoles.Value.Count; i++)
                    {
                        for (int j = 0; j < loopToHoles.Value[i].Count; j++)
                        {
                            loopToHoles.Value[i][j].ReplaceFace(fc, face);
                        }
                    }
                    face.Set(face1.Surface.Clone(), loopToHoles.Key, loopToHoles.Value);
                    HashSet<Edge> onNewFace = face.Edges.ToHashSet();
                    HashSet<Edge> intersectionEdges = new HashSet<Edge>();
                    foreach (Edge ie in ie1)
                    {
                        HashSet<Edge> onOutline = (new HashSet<Edge>(Vertex.ConnectingEdges(ie.Vertex1, ie.Vertex2))).Intersect(onNewFace).ToHashSet();
                        bool isInside = false;
                        foreach (Edge edg in onOutline)
                        {
                            if (SameEdge(edg, ie, precision))
                            {
                                isInside = true;
                                break;
                            }
                        }
                        if (isInside)
                        {   // this intersection edge is on the outline of the new face
                            intersectionEdges.Add(ie.CloneReplaceFace(face1, face, true));
                        }
                        else
                        {
                            GeoPoint2D testPoint = ie.Curve2D(face1).PointAt(0.5);
                            if (face.Contains(ref testPoint, false))
                            {   // this intersection edge is inside the new face
                                intersectionEdges.Add(ie.CloneReplaceFace(face1, face, true));
                            }
                        }
                    }
                    face.CopyAttributes(face1);
                    res[face] = intersectionEdges;
                }
            }
            return res;
        }
        /// <summary>
        /// Make the difference face1 - face2 (which can be any number of faces, including none) and distribute the intersection edges of face1
        /// to the resulting faces
        /// </summary>
        /// <param name="face1"></param>
        /// <param name="face2"></param>
        /// <param name="secondToFirst"></param>
        /// <returns></returns>
        private List<Face> Difference(Face face1, Face face2, ModOp2D secondToFirst, bool secondIsOpposite)
        {
            List<Face> res = new List<Face>();
            HashSet<Edge> toUse = new HashSet<Edge>();
            if (!faceToIntersectionEdges.TryGetValue(face1, out HashSet<Edge> ie1)) ie1 = new HashSet<Edge>(); // empty set
            if (!faceToIntersectionEdges.TryGetValue(face2, out HashSet<Edge> ie2)) ie2 = new HashSet<Edge>(); // empty set
            ie1.UnionWith(face1.Edges.ToHashSet());
            ie2.UnionWith(face2.Edges.ToHashSet());
            Face fc = Face.Construct(); // a placeholder for orientation only, it will not be fully constructed
            BoundingRect domain = face1.Area.GetExtent();
            foreach (Edge edg in face1.Edges)
            {
                // Add all edges of face1, which are not inside face2
                // if face2 has an intersection edge identical to this edge, then it is inside face2
                HashSet<Edge> insideFace2 = (new HashSet<Edge>(Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2))).Intersect(ie2).ToHashSet();
                foreach (Edge if2 in insideFace2.Clone())
                {
                    if (!SameEdge(if2, edg, precision)) insideFace2.Remove(if2);
                }
                if (insideFace2.Count == 0)
                {
                    Edge clone = edg.CloneWithVertices();
                    clone.SetPrimary(fc, edg.Curve2D(face1), edg.Forward(face1));
                    toUse.Add(clone);
                }
            }
            foreach (Edge edg in face2.Edges)
            {
                // Add all edges of face2, which are inside face1
                HashSet<Edge> connecting = new HashSet<Edge>(Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2));
                HashSet<Edge> insideFace1 = connecting.Intersect(ie1).ToHashSet(); // can be more than one
                bool isInside = false;
                foreach (Edge edgi in insideFace1)
                {
                    if (SameEdge(edgi, edg, precision))
                    {
                        isInside = true;
                        break;
                    }
                }
                if (isInside)
                {
                    HashSet<Edge> onFace1 = connecting.Intersect(face1.Edges.ToHashSet()).ToHashSet();
                    bool notOnFace1 = true;
                    foreach (Edge edg1 in onFace1)
                    {
                        if (SameEdge(edg1, edg, precision) && (edg1.StartVertex(face1) == edg.StartVertex(face2) != secondIsOpposite)) notOnFace1 = false;
                    }
                    if (notOnFace1)
                    {   // edg is not an edge on face1
                        Edge clone = edg.CloneWithVertices();
                        ICurve2D c2d = edg.Curve2D(face2).GetModified(secondToFirst);
                        //c2d = face1.Surface.GetProjectedCurve(edg.Curve3D, 0.0);
                        //if (!edg.Forward(face2)) c2d.Reverse();
                        SurfaceHelper.AdjustPeriodic(face1.Surface, domain, c2d);
                        clone.SetPrimary(fc, c2d, edg.Forward(face2));
                        if (!secondIsOpposite) clone.Reverse(fc);
                        toUse.Add(clone);
                    }
                }
            }
            if (toUse.Count > 0)
            {
                List<List<Edge>> loops = GetLoops(toUse, fc);
                Dictionary<List<Edge>, List<List<Edge>>> loopsToHoles = SortLoopsTopologically(loops, fc);
                foreach (KeyValuePair<List<Edge>, List<List<Edge>>> loopToHoles in loopsToHoles)
                {
                    Face face = Face.Construct();
                    foreach (Edge edg in loopToHoles.Key) edg.ReplaceFace(fc, face);
                    for (int i = 0; i < loopToHoles.Value.Count; i++)
                    {
                        for (int j = 0; j < loopToHoles.Value[i].Count; j++)
                        {
                            loopToHoles.Value[i][j].ReplaceFace(fc, face);
                        }
                    }
                    face.Set(face1.Surface.Clone(), loopToHoles.Key, loopToHoles.Value);
                    HashSet<Edge> onNewFace = face.Edges.ToHashSet();
                    face.CopyAttributes(face1);
                    res.Add(face);
                }
            }
            return res;
        }

        /// <summary>
        /// Sort the set of loops into a dictionary, so that all dictionary entries have a positive loop (ccw, outline) as a key and all negative loops
        /// (cw, hole), which are located inside this outline, as value. All in respect to the provided face.
        /// </summary>
        /// <param name="loops"></param>
        /// <param name="face"></param>
        /// <returns></returns>
        private Dictionary<List<Edge>, List<List<Edge>>> SortLoopsTopologically(List<List<Edge>> loops, Face face)
        {
            Dictionary<int, List<int>> resIndices = new Dictionary<int, List<int>>();
            ICurve2D[][] loops2d = new ICurve2D[loops.Count][];
            for (int i = 0; i < loops.Count; i++)
            {
                loops2d[i] = new ICurve2D[loops[i].Count];
                for (int j = 0; j < loops[i].Count; j++)
                {
                    loops2d[i][j] = loops[i][j].Curve2D(face);
                }
            }
            UniqueDoubleReverseDictionary<int> sortedLoops = new UniqueDoubleReverseDictionary<int>();
            for (int i = 0; i < loops2d.Length; i++)
            {
                sortedLoops.AddUnique(Border.SignedArea(loops2d[i]), i);
            }
            // sortedLoops now contains the indices of all loop, beginning with the bigges outline, ending with the holes
            foreach (KeyValuePair<double, int> kv in sortedLoops)
            {
                if (kv.Key > 0) resIndices[kv.Value] = new List<int>(); // an outline with no holes yet
                else
                {   // a hole, here all outlines are already in the resulting dictionary
                    foreach (int outline in resIndices.Keys)
                    {
                        if (Border.IsInside(loops2d[outline], loops2d[kv.Value][0].StartPoint))
                        {
                            resIndices[outline].Add(kv.Value); // this loop, which is a hole, belongs to that outline
                            break;
                        }
                    }
                }
            }
            Dictionary<List<Edge>, List<List<Edge>>> res = new Dictionary<List<Edge>, List<List<Edge>>>();
            foreach (KeyValuePair<int, List<int>> item in resIndices)
            {
                res[loops[item.Key]] = new List<List<Edge>>();
                for (int i = 0; i < item.Value.Count; i++)
                {
                    res[loops[item.Key]].Add(loops[item.Value[i]]);
                }
            }
            return res;
        }

        /// <summary>
        /// Takes a set of edges and tries to connect them to closed loops. 
        /// </summary>
        /// <param name="workingSet">work on this set, which will be emptied</param>
        /// <param name="face">orientation in respect to this face</param>
        /// <returns></returns>
        private List<List<Edge>> GetLoops(HashSet<Edge> workingSet, Face face)
        {
#if DEBUG
            DebuggerContainer dc = new DebuggerContainer();
            double arrowSize = 1;
            dc.Add(workingSet, face, arrowSize, Color.Red, -1);
#endif
            Dictionary<Edge, List<Edge>> adjacencyList = new Dictionary<Edge, List<Edge>>();
            // adjacencyList assigns all egdes, which start at the endvertex of edg to edg
            foreach (Edge edg in workingSet)
            {
                List<Edge> outgoing = edg.EndVertex(face).EdgesOnFace(face);
                if (!adjacencyList.TryGetValue(edg, out List<Edge> le)) adjacencyList[edg] = le = new List<Edge>();
                for (int i = 0; i < outgoing.Count; i++)
                {
                    if (outgoing[i] != edg && outgoing[i].StartVertex(face) == edg.EndVertex(face)) le.Add(outgoing[i]);
                }
            }
            List<List<Edge>> ecs = Graph.GetAllLoops(adjacencyList);
            for (int i = ecs.Count - 1; i >= 0; --i)
            {
                List<ICurve2D> crvs2d = new List<ICurve2D>();
                for (int j = 0; j < ecs[i].Count; j++)
                {
                    crvs2d.Add(ecs[i][j].Curve2D(face));
                }
                // only use correct oriented borders. We have at least one case, where there is a wrong oriented cycle: "Overlapping2.cdb.json"
                // no, we also need the holes!
                // if (Border.SignedArea(crvs2d.ToArray()) < 0) ecs.RemoveAt(i);
            }
#if DEBUG
            DebuggerContainer dc1 = new DebuggerContainer();
            for (int i = 0; i < ecs.Count; i++)
            {
                for (int j = 0; j < ecs[i].Count; j++)
                {
                    dc1.Add(ecs[i][j].Curve3D as IGeoObject);
                }
            }
#endif
            return ecs;

        }

        private List<List<Edge>> GetCommon(Face face1, Face face2, ModOp2D face2To1)
        {
            ModOp2D face1To2 = face2To1.GetInverse();
            HashSet<Vertex> commonVertices = (face1.Vertices).Intersect(face2.Vertices).ToHashSet();
            Dictionary<Vertex, List<Edge>> connections = new Dictionary<Vertex, List<Edge>>();
            foreach (Vertex vtx in commonVertices)
            {

                List<Edge> con1 = face1.FindConnection(vtx, commonVertices);
                List<Edge> con2 = face2.FindConnection(vtx, commonVertices);
                if (con1.Count > 0 && con2.Count > 0 && SameConnection(con1, con2, face1, face2, precision))
                {
                    connections[vtx] = con1;
                }
                else
                {
                    if (con1.Count > 0)
                    {
                        GeoPoint2D testPoint = face1To2 * con1[0].Curve2D(face1).PointAt(0.5);
                        if (face2.Contains(ref testPoint, false)) connections[vtx] = con1;
                    }
                    if (con2.Count > 0)
                    {
                        GeoPoint2D testPoint = face2To1 * con2[0].Curve2D(face2).PointAt(0.5);
                        if (face1.Contains(ref testPoint, false)) connections[vtx] = con2;
                    }
                }
            }
            // connections is all connections but no duplicates
            KeyValuePair<Vertex, List<Edge>> kv = Enumerable.FirstOrDefault(connections);
            Vertex startVertex = kv.Key;
            List<Edge> startSegment = kv.Value;
            List<List<Edge>> loops = new List<List<Edge>>();
            List<Edge> loop = new List<Edge>();
            while (kv.Key != null)
            {
                loop.AddRange(startSegment);
                connections.Remove(kv.Key);

            }
            return loops;
        }

        //private int CompareReverse(double x, double y) { return -x.CompareTo(y); }
        private int ComparePair((List<Edge>, ICurve2D[]) x, (List<Edge>, ICurve2D[]) y)
        {
            if (x.Item1.Count == y.Item1.Count)
            {
                for (int i = 0; i < x.Item1.Count; i++)
                {
                    if (x.Item1[i].GetHashCode() != y.Item1[i].GetHashCode())
                    {
                        return (x.Item1[i].GetHashCode().CompareTo(y.Item1[i].GetHashCode()));
                    }
                }
                return 0;
            }
            else return x.Item1.Count.CompareTo(y.Item1.Count);
        }

        /// <summary>
        /// Check whether the two lists of (connected) edges describe the same (geometric) path.
        /// </summary>
        /// <param name="con1"></param>
        /// <param name="con2"></param>
        /// <param name="onThisFace"></param>
        /// <param name="precision"></param>
        /// <returns></returns>
        internal static bool SameConnection(List<Edge> con1, List<Edge> con2, Face onThisFace, double precision)
        {
            if (con1.Count == 1 && con2.Count == 1) return SameEdge(con1[0], con2[0], precision);
            else if (con1.Count >= con2.Count)
            {   // it is egnough to test the inner vertices of the connection with more inner vertices against the 3d curves of the other connection
                for (int i = 0; i < con1.Count - 1; i++)
                {
                    GeoPoint p = con1[i].EndVertex(onThisFace).Position;
                    bool hit = false;
                    for (int j = 0; j < con2.Count; j++)
                    {
                        if (con2[j].Curve3D != null) hit = con2[j].Curve3D.DistanceTo(p) < 10 * precision; // using precision might fail
                        if (hit) break;
                    }
                    if (!hit) return false;
                }
                return true;
            }
            else
            {
                return SameConnection(con2, con1, onThisFace, precision);
            }
        }
        internal static bool SameConnection(List<Edge> con1, List<Edge> con2, Face face1, Face face2, double precision)
        {
            if (con1.Count == 1 && con2.Count == 1) return SameEdge(con1[0], con2[0], precision);
            else if (con1.Count >= con2.Count) return SameConnection(con1, con2, face1, precision);
            else return SameConnection(con2, con1, face2, precision);

        }

        private List<List<Edge>> FindPath(HashSet<Edge> set, Vertex vertex1, Vertex vertex2)
        {   // find one or more paths (or none of course) of connected edges from the provided set, which goes from vertex1 to vertex2
            List<List<Edge>> res = new List<List<Edge>>();
            HashSet<Edge> startWith = vertex1.AllEdges.Intersect(set).ToHashSet();
            foreach (Edge edg in startWith)
            {
                Vertex endVertex = edg.OtherVertex(vertex1);
                if (endVertex == vertex2)
                {
                    List<Edge> singleEdge = new List<Edge>();
                    singleEdge.Add(edg);
                    res.Add(singleEdge);
                }
                else
                {
                    HashSet<Edge> usable = set.Clone();
                    usable.Remove(edg);
                    List<List<Edge>> secondPart = FindPath(usable, endVertex, vertex2);
                    foreach (List<Edge> le in secondPart)
                    {
                        le.Insert(0, edg);
                    }
                    res.AddRange(secondPart);
                }
            }
            return res;
        }

        internal static bool SameEdge(Edge e1, Edge e2, double precision)
        {   // it is assumed that the two edges connect the same vertices 
            // it is tested whether they have the same geometry (but maybe different directions) 
            // (two half circles may connect the same vertices but are not geometrically identical when they describe differnt parts of the same circle)
            if (e1.Curve3D != null && e2.Curve3D != null) return e1.Curve3D.DistanceTo(e2.Curve3D.PointAt(0.5)) < 10 * precision; // nur precision war zu knapp
            return false;
        }

        internal static bool SameOrPartialEdge(Edge part, Edge full, double precision)
        {   // tests whether the two edges connect the same vertices and 
            // have the same geometry (but maybe different directions) (two half circles may connect the same vertices but are not geometrically identical
            // when they describe differnt parts of the same circle)
            if (part.Curve3D != null && full.Curve3D != null)
            {
                if (full.Curve3D.DistanceTo(part.Vertex1.Position) > 10 * precision) return false;
                if (full.Curve3D.DistanceTo(part.Vertex2.Position) > 10 * precision) return false;
                if (full.Curve3D.DistanceTo(part.Curve3D.PointAt(0.5)) > 10 * precision) return false;
                return true;
            }
            return false;
        }

        internal static IEnumerable<(Edge, Edge)> EdgePairs(IList<Edge> edges)
        {
            for (int i = 0; i < edges.Count; i++)
            {
                if (i == 0)
                    yield return (edges[edges.Count - 1], edges[0]);
                else
                    yield return (edges[i - 1], edges[i]);
            }
        }

#if DEBUG
        internal static HashSet<Face> collectConnected = new HashSet<Face>();
#endif
        /// <summary>
        /// Return all the faces, which are directely or indirectely connected to "startWith" from the set "allFaces"
        /// Also remove thos found faces from "allfaces"
        /// </summary>
        /// <param name="allFaces"></param>
        /// <param name="startWith"></param>
        /// <param name="result"></param>
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
        /// Liefert die Vereinigung der beiden Shells. Das können mehrere Shells sein, denn es kann eine innere Höhlung entstehen.
        /// </summary>
        /// <returns></returns>
        public int GetOverlappingFaces(out Face[] onShell1, out Face[] onShell2, out ModOp2D[] firstToSecond)
        {
            onShell1 = new Face[overlappingFaces.Count];
            onShell2 = new Face[overlappingFaces.Count];
            firstToSecond = new ModOp2D[overlappingFaces.Count];
            int ind = 0;
            foreach (KeyValuePair<DoubleFaceKey, ModOp2D> kv in overlappingFaces)
            {
                onShell1[ind] = kv.Key.face1;
                onShell2[ind] = kv.Key.face2;
                firstToSecond[ind] = kv.Value;
                ++ind;
            }
            return overlappingFaces.Count;
        }
        internal void ConnectOpenEdges(Edge[] openEdges)
        {
            OrderedMultiDictionary<DoubleVertexKey, Edge> dict = new OrderedMultiDictionary<DoubleVertexKey, Edge>(true);
            for (int i = 0; i < openEdges.Length; ++i)
            {
                dict.Add(new DoubleVertexKey(openEdges[i].Vertex1, openEdges[i].Vertex2), openEdges[i]);
            }
            foreach (KeyValuePair<DoubleVertexKey, ICollection<Edge>> kv in dict)
            {
                if (kv.Value.Count == 2)
                {
                    Edge e1 = null;
                    Edge e2 = null;
                    foreach (Edge e in kv.Value)
                    {
                        if (e1 == null) e1 = e;
                        else e2 = e;
                    }
                    if (e1.Curve3D.SameGeometry(e2.Curve3D, precision))
                    {
                        e1.SetSecondary(e2.PrimaryFace, e2.Curve2D(e2.PrimaryFace), e2.Forward(e2.PrimaryFace));
                        e2.PrimaryFace.ReplaceEdge(e2, new Edge[] { e1 });
                    }
                }
            }
        }
        private void SplitEdges()
        {
            // Split all edges at the vertices provided in edgesToSplit
            foreach (KeyValuePair<Edge, List<Vertex>> kv in edgesToSplit)
            {
                Edge edge = kv.Key;
                HashSet<Vertex> vertexSet = new HashSet<Vertex>(kv.Value); // einzelne vertices können doppelt vorkommen
                SortedList<double, Vertex> sortedVertices = new SortedList<double, Vertex>();
                double prec = precision / edge.Curve3D.Length * 2.0; // darf natürlich nicht 0 sein!
                foreach (Vertex v in vertexSet)
                {
                    double pos = edge.Curve3D.PositionOf(v.Position);
                    if (pos > prec && pos < 1 - prec && !sortedVertices.ContainsKey(pos)) sortedVertices.Add(pos, v); // keine Endpunkte, sonst entstehen beim Aufteilen Stücke der Länge 0
                    if (v != edge.Vertex1 && v != edge.Vertex2) v.RemoveEdge(edge);
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
        private void createNewEdges()
        {
            overlappingEdges = new Dictionary<DoubleFaceKey, HashSet<Edge>>();
            faceToIntersectionEdges = new Dictionary<Face, HashSet<Edge>>();
            edgesNotToUse = new HashSet<Edge>();
            // wir haben eine Menge Schnittpunkte, die Face-Paaren zugeordnet sind. Für jedes Face-Paar, welches Schnittpunkte enthält sollen hier die neuen Kanten bestimmt werden
            // Probleme dabei sind: 
            // - es ist bei mehr als 2 Schnittpunkten nicht klar, welche Abschnitte dazugehören
            // - zwei Surfaces können mehr als eine Schnittkurve haben
            HashSet<Edge> created = new HashSet<CADability.Edge>(new EdgeComparerByVertexAndFace());
            foreach (KeyValuePair<DoubleFaceKey, List<IntersectionVertex>> item in facesToIntersectionVertices)
            {
                // cancelledfaces was used not to create an intersection edge which is identical to an edge on two opposing faces
                // but at least with "RohrHalter5.cdb.json" we do need this intersection edge
                // if (cancelledfaces.Contains(item.Key.face1) || cancelledfaces.Contains(item.Key.face2)) continue;
                // also edgesOnOverlappingFaces is no longer used, so we don't need the following:
                //HashSet<Edge> edgesOnOverlappingFaces = new HashSet<Edge>();
                //HashSet<Face> overlapping1 = findOverlappingPartner(item.Key.face1);
                //if (overlapping1.Count > 0)
                //{
                //    foreach (Edge edg in item.Key.face2.Edges)
                //    {
                //        if (overlapping1.Contains(edg.OtherFace(item.Key.face2))) edgesOnOverlappingFaces.Add(edg);
                //    }
                //}
                //HashSet<Face> overlapping2 = findOverlappingPartner(item.Key.face2);
                //if (overlapping2.Count > 0)
                //{
                //    foreach (Edge edg in item.Key.face1.Edges)
                //    {
                //        if (overlapping2.Contains(edg.OtherFace(item.Key.face1))) edgesOnOverlappingFaces.Add(edg);
                //    }
                //}
                //HashSet<Edge> existsOnFace1 = edgesOnOverlappingFaces.Intersect(new HashSet<Edge>(item.Key.face1.AllEdges).ToHashSet());
                //HashSet<Edge> existsOnFace2 = edgesOnOverlappingFaces.Intersect(new HashSet<Edge>(item.Key.face2.AllEdges).ToHashSet());
                //HashSet<Edge> existsOnBothFaces = new HashSet<Edge>();

                HashSet<Vertex> involvedVertices = new HashSet<Vertex>();
                for (int i = 0; i < item.Value.Count; i++)
                {
                    involvedVertices.Add(item.Value[i].v);
                }
#if DEBUG
                DebuggerContainer dc0 = new DebuggerContainer();
                dc0.Add(item.Key.face1, item.Key.face1.GetHashCode());
                foreach (Edge edg in item.Key.face1.AllEdges)
                {
                    dc0.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
                }
                dc0.Add(item.Key.face2, item.Key.face2.GetHashCode());
                foreach (Edge edg in item.Key.face2.AllEdges)
                {
                    dc0.Add(edg.Curve3D as IGeoObject, edg.GetHashCode());
                }
                foreach (Vertex vtx in involvedVertices)
                {
                    Point pnt = Point.Construct();
                    pnt.Location = vtx.Position;
                    pnt.Symbol = PointSymbol.Cross;
                    dc0.Add(pnt, vtx.GetHashCode());
                }
#endif
                List<Vertex> possibleVerticesOnCommonEdge = new List<Vertex>(item.Key.face1.Vertices.Intersect(item.Key.face2.Vertices).Intersect(involvedVertices));
                if (possibleVerticesOnCommonEdge.Count == 2)
                {   // maybe the intersection curve is an existing edge on both faces: we don't need it
                    // but this could happen multiple times and we only check for a single edge!
                    List<Edge> commonEdges = new List<Edge>(possibleVerticesOnCommonEdge[0].AllEdges.Intersect(possibleVerticesOnCommonEdge[1].AllEdges));
                    foreach (Edge edg in commonEdges)
                    {
                        bool found1 = false, found2 = false;
                        if (edg.PrimaryFace == item.Key.face1) found1 = true;
                        if (edg.SecondaryFace == item.Key.face1) found1 = true;
                        if (edg.PrimaryFace == item.Key.face2) found2 = true;
                        if (edg.SecondaryFace == item.Key.face2) found2 = true;
                        if (found1 && found2)
                        {   // this edge is an already existing edge on both faces (face1 and face2) and connects the two vertices in possibleVerticesOnCommonEdge
                            involvedVertices.ExceptWith(possibleVerticesOnCommonEdge);
                            break;
                        }
                    }
                }

                if (involvedVertices.Count < 2) continue;

                List<Vertex> usedVertices = new List<Vertex>(involvedVertices.Count);
                List<GeoPoint> points = new List<GeoPoint>(involvedVertices.Count); // usedVertices and points are synchronous arrays, so we later can find the vertex from a point (index)
                foreach (Vertex v in involvedVertices)
                {
                    usedVertices.Add(v);
                    points.Add(v.Position);
                }

                if (involvedVertices.Count == 2 && knownIntersections != null)
                {
                    bool wasKnownintersection = false;
                    foreach (KeyValuePair<Edge, Tuple<Face, Face>> ki in knownIntersections)
                    {
                        // check, whether the two vertices, which define the intersection curve, are located on this tangential (knownIntersection) edge.
                        if (ki.Value.Item1 == item.Key.face1 && ki.Value.Item2 == item.Key.face2)
                        {
                            // if so, clip the 3d curve and use it as an new intersection edge
                            double pos1 = ki.Key.Curve3D.PositionOf(usedVertices[0].Position);
                            double pos2 = ki.Key.Curve3D.PositionOf(usedVertices[1].Position);
                            // since this are tangential intersections, we must allow greater roundoff errors
                            if (pos1 > -1e-6 && pos1 < 1 + 1e-6 && pos2 > -1e-6 && pos2 < 1 + 1e-6 && (ki.Key.Curve3D.PointAt(pos1) | usedVertices[0].Position) < 100 * Precision.eps && (ki.Key.Curve3D.PointAt(pos2) | usedVertices[1].Position) < 100 * Precision.eps)
                            {   // this is a known (and probably tangential) intersection edge, where the edge already exists (e.g. when rounding edges of a shell)
                                Edge edge;
                                if (ki.Key.PrimaryFace == item.Key.face2) // this is a rounding fillet
                                {
                                    ICurve crv = ki.Key.Curve3D.Clone();
                                    if (pos1 < pos2) crv.Trim(pos1, pos2);
                                    else crv.Trim(pos2, pos1);
                                    ICurve2D curve2D = item.Key.face1.Surface.GetProjectedCurve(crv, 0.0);
                                    if (ki.Key.Forward(ki.Key.PrimaryFace)) curve2D.Reverse();
                                    SurfaceHelper.AdjustPeriodic(item.Key.face1.Surface, item.Key.face1.Domain, curve2D);
                                    edge = new Edge(item.Key.face1, crv, item.Key.face1, curve2D, !ki.Key.Forward(ki.Key.PrimaryFace));

                                    //edge.edgeInfo = new EdgeInfo(edge);
                                    //edge.edgeInfo.isIntersection = true;
                                    edge.UseVerticesForce(usedVertices.ToArray()); // use the already existing vertices

                                    HashSet<Edge> addTo;
                                    if (!faceToIntersectionEdges.TryGetValue(item.Key.face1, out addTo))
                                    {
                                        addTo = new HashSet<Edge>(); // (new EdgeComparerByVertex()); // damit werden zwei Kanten mit gleichen Vertices nicht zugefügt, nutzt nichts
                                        faceToIntersectionEdges[item.Key.face1] = addTo;
                                    }
                                    addTo.Add(edge);
                                    wasKnownintersection = true;
                                    break; // the loop over knownIntersections
                                }
                            }
                        }
                    }
                    if (wasKnownintersection) continue; // with the loop over facesToIntersectionVertices, no intersection calculation needed
                }

                ICurve[] c3ds;
                ICurve2D[] crvsOnSurface1;
                ICurve2D[] crvsOnSurface2;
                double[,] params3d;
                double[,] params2dFace1;
                double[,] params2dFace2;
                GeoPoint2D[] paramsuvsurf1;
                GeoPoint2D[] paramsuvsurf2;
                if (Surfaces.Intersect(item.Key.face1.Surface, item.Key.face1.Area.GetExtent(), item.Key.face2.Surface, item.Key.face2.Area.GetExtent(), points, out c3ds, out crvsOnSurface1, out crvsOnSurface2, out params3d, out params2dFace1, out params2dFace2, out paramsuvsurf1, out paramsuvsurf2, precision))
                {
                    if (usedVertices.Count < points.Count)
                    {
                        // There have been additional vertices created.
                        // This happens e.g. when two cylinders with the same diameter intersect or in general there is a touching point, which was not in points
                        for (int i = usedVertices.Count; i < points.Count; i++)
                        {   // changed to true: accept also on border, which is necessary with UniteBug5
                            if (!item.Key.face1.Contains(ref paramsuvsurf1[i], true) || !item.Key.face2.Contains(ref paramsuvsurf2[i], true))
                            {
                                for (int j = 0; j < c3ds.Length; j++)
                                {
                                    params3d[j, i] = double.MinValue; // this vertex is invalid
                                                                      // but in order to keep points and usedVertices in sync, we still have to add a vertex to usedVertices
                                }
                            }
                            Vertex v = new Vertex(points[i]);
                            usedVertices.Add(v);
                        }
                    }
                    for (int i = 0; i < c3ds.Length; i++) // meist nur eine Kurve
                    {   // die Orientierung der 3d Kurve ist zufällig, hat nichts mit der Topologie der Shell zu tun.
                        // Die beiden 2d Kurven haben die selbe Orientierung wie die 3d Kurve, aber die beiden 2d Kurven müssen letztlich
                        // gegenläufig orientiert sein, so dass hier entschiedn werden muss, welche umgedreht wird
                        SortedDictionary<double, int> sortedIv = new SortedDictionary<double, int>(); // Indizes nach u aufsteigend sortiert und nur für Kurve i (meist ohnehin nur eine)
                        for (int j = 0; j < points.Count; j++)
                        {
                            if (params3d[i, j] == double.MinValue) continue; // dieser Schnittpunkt liegt auf einer anderen Kurve
                            sortedIv[params3d[i, j]] = j;
                        }
                        // aus den mehreren Punkten (meist 2) unter Berücksichtigung der Richtungen Kanten erzeugen
                        // bei nicht geschlossenen Kurven sollten immer zwei aufeinanderfolgende Punkte einen gültigen Kurvenabschnitt bilden.
                        // Obwohl auch hierbei ein doppelt auftretenden Punkt ein Problem machen könnte (Fälle sind schwer vorstellbar).
                        // Im schlimmsten Fall müssten man für alle Abschnitte (nicht nur die geraden) 
                        // Bei geschlossenen Kurven ist nicht klar, welche Punktepaare gültige Kurvenabschnitte erzeugen. 
                        if (c3ds[i].IsClosed)
                        {
                            if (item.Key.face1.Area.Contains(crvsOnSurface1[i].StartPoint, false) && item.Key.face2.Area.Contains(crvsOnSurface2[i].StartPoint, false))
                            {
                                // der Startpunkt der geschlossenen 2d-Kurven (die ja dem Startpunkt der 3d Kurve entsprechen) gehört zu den gültigen Kurvenabschnitten
                                // also dürfen wir nicht mit dem kleinsten u beginnen, sondern müssen das Kurvenstück berücksichtigen, welches durch den Anfang  geht.
                                // Beachte: die trimm Funktion (sowohl in 2d als auch in 3d) muss berücksichtigen, dass bei geschlossenen Kurven, der 1. index größer sein kann
                                // als der 2. Index. Das bedeutet dann, das Stück, welches über "0" geht soll geliefert werden. Noch überprüfen, ob das überall implementiert ist
                                // hier wird das erste Element aus dem SortedDictionary entfernt und hinten wieder eingefügt
                                var fiter = sortedIv.GetEnumerator(); // so bekommt man das erste, First() scheint es komischerweise nicht zu geben
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
                            // before trimming we test whether we need this curve at all
                            // if both endpoints of the curve are on the border of the face, the curve may be totally outside
                            bool isOnFace1Border = false, isOnFace2Border = false;
                            bool j1IsOnBorder = false, j2IsOnBorder = false;
                            foreach (IntersectionVertex iv in item.Value)
                            {
                                if (iv.v == usedVertices[j1] && iv.isOnFaceBorder)
                                {
                                    if (iv.edgeIsOn1) isOnFace1Border = true;
                                    else isOnFace2Border = true;
                                    j1IsOnBorder = true;
                                }
                                if (iv.v == usedVertices[j2] && iv.isOnFaceBorder)
                                {
                                    if (iv.edgeIsOn1) isOnFace1Border = true;
                                    else isOnFace2Border = true;
                                    j2IsOnBorder = true;
                                }
                            }

                            if ((j1IsOnBorder && j2IsOnBorder) || sortedIv.Count > 2)
                            {   // both endpoints are on the border: it is not sure that the intersectioncurve is inside the face
                                // more than two intersectionpoints: maybe some segment is outside
                                GeoPoint2D uv = crvsOnSurface1[i].PointAt((params2dFace1[i, j1] + params2dFace1[i, j2]) / 2.0);
                                if (!item.Key.face1.Contains(ref uv, true))
                                {   // it still might be a point on the edge,also test it in 3d, because it might be imprecise in 2d
                                    uv = item.Key.face1.Surface.PositionOf(c3ds[i].PointAt((u1 + u2) / 2.0));
                                    if (!item.Key.face1.Contains(ref uv, true))
                                    {   // still too strong condition, we only need to exclude e.g. wrong halves of a circle
                                        if (item.Key.face1.Area.GetPosition(uv, item.Key.face1.Domain.Size * 1e-5) == Border.Position.Outside) continue;
                                    }
                                }
                                uv = crvsOnSurface2[i].PointAt((params2dFace2[i, j1] + params2dFace2[i, j2]) / 2.0);
                                if (!item.Key.face2.Contains(ref uv, true))
                                {
                                    uv = item.Key.face2.Surface.PositionOf(c3ds[i].PointAt((u1 + u2) / 2.0));
                                    if (!item.Key.face2.Contains(ref uv, true))
                                    {
                                        if (item.Key.face2.Area.GetPosition(uv, item.Key.face2.Domain.Size * 1e-5) == Border.Position.Outside) continue;
                                    }
                                }
                            }

                            ICurve tr = c3ds[i].Clone(); // InterpolatedDualSurfaceCurve muss die Surfaces erahlten
                            tr.Trim(u1, u2);
                            ICurve2D con1 = crvsOnSurface1[i].Trim(params2dFace1[i, j1], params2dFace1[i, j2]);
                            ICurve2D con2 = crvsOnSurface2[i].Trim(params2dFace2[i, j1], params2dFace2[i, j2]);
                            // hier am Besten aus InterpolatedDualSurfaceCurve BSplines machen, sowohl in 2d, als auch in 3d und ganz am Ende
                            // wieder zu InterpolatedDualSurfaceCurve machen. GGf mit Flag, damit das klar ist
                            // Problme wäre die Genauigkeit, wenn beim BooleanOperation.generateCycles die Richtung genommen wird...
                            if (con1 is InterpolatedDualSurfaceCurve.ProjectedCurve && con2 is InterpolatedDualSurfaceCurve.ProjectedCurve &&
                                tr is InterpolatedDualSurfaceCurve)
                            {   // con1 und con2 müssen auf tr verweisen, sonst kann man das Face später nicht mit "ReverseOrientation" umdrehen. Dort wird nämlich die 
                                // surface verändert, und die muss bei allen Kurven die selbe sein
                                (con1 as InterpolatedDualSurfaceCurve.ProjectedCurve).SetCurve3d(tr as InterpolatedDualSurfaceCurve);
                                (con2 as InterpolatedDualSurfaceCurve.ProjectedCurve).SetCurve3d(tr as InterpolatedDualSurfaceCurve);
                            }
                            // das Kreuzprodukt im Start (oder End oder Mittel) -Punkt hat die selbe Reichung wie die 3d Kurve: con1 umdrehen
                            // andere Richtung: con2 umdrehen
                            // The cross product of the normals specifies the direction of the new edge, no matter where on the curve we compute it.
                            // But if the surfaces are tangential in a point the cross product of the normals will be 0. So we take the better one
                            // if both are bad (e.g. two same diameter cylinders), we take a point in the middle
                            bool dirs1;
                            GeoVector normalsCrossedStart = item.Key.face1.Surface.GetNormal(paramsuvsurf1[j1]) ^ item.Key.face2.Surface.GetNormal(paramsuvsurf2[j1]);
                            GeoVector normalsCrossedEnd = item.Key.face1.Surface.GetNormal(paramsuvsurf1[j2]) ^ item.Key.face2.Surface.GetNormal(paramsuvsurf2[j2]);
                            if (normalsCrossedStart.Length < Precision.eps && normalsCrossedEnd.Length < Precision.eps)
                            {
                                // it seems to be tangential at the endpoints of the intersection curve: test in the middle of the intersection curve
                                GeoPoint m = tr.PointAt(0.5);
                                GeoVector normalsCrossedMiddle = item.Key.face1.Surface.GetNormal(item.Key.face1.Surface.PositionOf(m)) ^ item.Key.face2.Surface.GetNormal(item.Key.face2.Surface.PositionOf(m));
                                if (normalsCrossedMiddle.Length < Precision.eps)
                                {
                                    // it is also tangential at the midpoint of the intersection curve
                                    // we now use the uv points in the surfaces and slowly walk from the uv point in the direction of the center of the domain
                                    // (2d extent), until we find a point, where the normals are not parallel any more.
                                    GeoPoint2D uvf1 = item.Key.face1.Surface.PositionOf(m);
                                    SurfaceHelper.AdjustPeriodic(item.Key.face1.Surface, item.Key.face1.Domain, ref uvf1);
                                    GeoPoint2D uvf2 = item.Key.face2.Surface.PositionOf(m);
                                    SurfaceHelper.AdjustPeriodic(item.Key.face2.Surface, item.Key.face2.Domain, ref uvf2);
                                    GeoVector2D toCenter1 = item.Key.face1.Domain.GetCenter() - uvf1;
                                    GeoVector2D toCenter2 = item.Key.face2.Domain.GetCenter() - uvf2;
                                    // normalis the step vectors to the size of the extent
                                    if (Math.Abs(toCenter1.x) > Math.Abs(toCenter1.y)) toCenter1.Length = item.Key.face1.Domain.Width * 1e-3; // 1/1000 of the extent
                                    else if (Math.Abs(toCenter1.y) > 0) toCenter1.Length = item.Key.face1.Domain.Height * 1e-3; // 1/1000 of the extent
                                    else toCenter1 = new GeoVector2D(item.Key.face1.Domain.Width * 1e-3, item.Key.face1.Domain.Height * 1e-3); // was exactely in the center
                                    if (Math.Abs(toCenter2.x) > Math.Abs(toCenter2.y)) toCenter2.Length = item.Key.face2.Domain.Width * 1e-3; // 1/1000 of the extent
                                    else if (Math.Abs(toCenter2.y) > 0) toCenter2.Length = item.Key.face2.Domain.Height * 1e-3; // 1/1000 of the extent
                                    else toCenter2 = new GeoVector2D(item.Key.face2.Domain.Width * 1e-3, item.Key.face2.Domain.Height * 1e-3); // was exactely in the center
                                                                                                                                               // walk to the center until the normals are no longer parallel
                                    while (item.Key.face1.Domain.ContainsEps(uvf1, -0.01) && item.Key.face2.Domain.ContainsEps(uvf2, -0.01))
                                    {
                                        uvf1 += toCenter1;
                                        uvf2 += toCenter2;
                                        GeoVector nn1 = item.Key.face1.Surface.GetNormal(uvf1).Normalized;
                                        GeoVector nn2 = item.Key.face2.Surface.GetNormal(uvf2).Normalized;
                                        toCenter1 = 2 * toCenter1;
                                        toCenter2 = 2 * toCenter2;
                                        normalsCrossedMiddle = nn1 ^ nn2;
                                        if (normalsCrossedMiddle.Length > Precision.eps) break;
                                    }

                                    double tangentialPrecision = (item.Key.face1.GetExtent(0.0).Size + item.Key.face2.GetExtent(0.0).Size) * Precision.eps;
                                    // Still ignoring the case where there could be a real intersection e.g. when a surface crosses a plane like the "S" crosses the tangent at the middle
                                    // When this intersection curve coincides with an existing edge on one of the faces, we use the combined normalvector of both involved faces
                                    HashSet<Edge> existingEdges = new HashSet<Edge>(Vertex.ConnectingEdges(usedVertices[j1], usedVertices[j2]));
                                    GeoVector n1 = item.Key.face1.Surface.GetNormal(item.Key.face1.Surface.PositionOf(m)).Normalized;
                                    GeoVector n2 = item.Key.face2.Surface.GetNormal(item.Key.face2.Surface.PositionOf(m)).Normalized;
                                    HashSet<Edge> onFace1 = existingEdges.Intersect(item.Key.face1.Edges).ToHashSet();
                                    HashSet<Edge> onFace2 = existingEdges.Intersect(item.Key.face2.Edges).ToHashSet();
                                    //bool edgFound = false;
                                    //// it was Precision.eps before, but a tangential intersection at "Difference2.cdb.json" failed, which should have been there 
                                    //// in many cases we are close to an edge on one of the faces or both.
                                    //// find this edge and step a little inside into this face
                                    //foreach (Edge edg in onFace1)
                                    //{
                                    //    if (edg.Curve3D != null && edg.Curve3D.DistanceTo(m) < tangentialPrecision)
                                    //    {
                                    //        Face otherFace = edg.OtherFace(item.Key.face1);
                                    //        n1 += otherFace.Surface.GetNormal(otherFace.Surface.PositionOf(m)).Normalized;
                                    //        edgFound = true;
                                    //        break;
                                    //    }
                                    //}
                                    //foreach (Edge edg in onFace2)
                                    //{
                                    //    if (edg.Curve3D != null && edg.Curve3D.DistanceTo(m) < tangentialPrecision)
                                    //    {
                                    //        Face otherFace = edg.OtherFace(item.Key.face2);
                                    //        n2 += otherFace.Surface.GetNormal(otherFace.Surface.PositionOf(m)).Normalized;
                                    //        edgFound = true;
                                    //        break;
                                    //    }
                                    //}
                                    if (normalsCrossedMiddle.Length < Precision.eps) //edgFound)
                                    {
                                        normalsCrossedMiddle = n1 ^ n2;
                                        if (normalsCrossedMiddle.Length < Precision.eps)
                                        {   // still not able to decide, the connecting face found is also tangential
                                            // now we go a little bit inside on the face with the edge. This is very seldom the case, so no problem making the same iteration once more
                                            foreach (Edge edg in onFace1)
                                            {
                                                if (edg.Curve3D != null && edg.Curve3D.DistanceTo(m) < tangentialPrecision)
                                                {
                                                    ICurve2D c2df1 = edg.Curve2D(item.Key.face1);
                                                    // from the middle of this edge go a small step into the inside of the face and see what the normal is at that point
                                                    GeoPoint2D mp = c2df1.PointAt(0.5);
                                                    GeoVector2D mdir = c2df1.DirectionAt(0.5).ToLeft().Normalized;
                                                    double len = item.Key.face1.Area.GetExtent().Size;
                                                    Line2D l2d = new Line2D(mp, mp + len * mdir);
                                                    double[] parts = item.Key.face1.Area.Clip(l2d, true);
                                                    if (parts.Length > 1)
                                                    {   // there is a point on face1 close to the intersectioncurve, which we can use for the normal
                                                        n1 += item.Key.face1.Surface.GetNormal(l2d.PointAt((parts[0] + parts[1]) / 2.0));
                                                    }
                                                    break;
                                                }
                                            }
                                            foreach (Edge edg in onFace2)
                                            {
                                                if (edg.Curve3D != null && edg.Curve3D.DistanceTo(m) < tangentialPrecision)
                                                {
                                                    ICurve2D c2df2 = edg.Curve2D(item.Key.face2);
                                                    // from the middle of this edge go a small step into the inside of the face and see what the normal is at that point
                                                    GeoPoint2D mp = c2df2.PointAt(0.5);
                                                    GeoVector2D mdir = c2df2.DirectionAt(0.5).ToLeft().Normalized;
                                                    double len = item.Key.face2.Area.GetExtent().Size;
                                                    Line2D l2d = new Line2D(mp, mp + len * mdir);
                                                    double[] parts = item.Key.face2.Area.Clip(l2d, true);
                                                    if (parts.Length > 1)
                                                    {   // there is a point on face2 close to the intersectioncurve, which we can use for the normal
                                                        n2 += item.Key.face2.Surface.GetNormal(l2d.PointAt((parts[0] + parts[1]) / 2.0));
                                                    }
                                                    break;
                                                }
                                            }
                                            normalsCrossedMiddle = n1 ^ n2;
                                        }
                                    }
                                    // else: it is a inner intersection. With simple surfaces this cannot be a real intersection
                                    // but with nurbs surfaces, this could be the case. This still has to be implemented
                                    if (normalsCrossedMiddle.Length < Precision.eps) continue;
                                }
                                dirs1 = (normalsCrossedMiddle * tr.DirectionAt(0.5)) > 0;
                            }
                            else if (normalsCrossedStart.Length > normalsCrossedEnd.Length)
                            {
                                dirs1 = (normalsCrossedStart * tr.StartDirection) > 0;
                            }
                            else
                            {
                                dirs1 = (normalsCrossedEnd * tr.EndDirection) > 0;

                            }
                            // bei diesem Skalarprodukt von 2 Vektoren, die entweder die selbe oder die entgegengesetzte Richtung haben ist ">0" unkritisch
                            if (dirs1) con2.Reverse();
                            else con1.Reverse();
                            GeoPoint2D uv1 = con1.PointAt(0.5);
                            GeoPoint2D uv2 = con2.PointAt(0.5);
                            if (!item.Key.face1.Contains(ref uv1, true) || !item.Key.face2.Contains(ref uv2, true))
                            {   // the condition is too strong. Use the same relaxed condition as above
                                if (item.Key.face1.Area.GetPosition(uv1, item.Key.face1.Domain.Size * 1e-5) == Border.Position.Outside &&
                                    item.Key.face2.Area.GetPosition(uv2, item.Key.face2.Domain.Size * 1e-5) == Border.Position.Outside) continue;
                            }
                            Edge edge = new Edge(item.Key.face1, tr, item.Key.face1, con1, dirs1, item.Key.face2, con2, !dirs1);
                            //edge.edgeInfo = new EdgeInfo(edge);
                            //edge.edgeInfo.isIntersection = true;
#if DEBUG
                            (tr as IGeoObject).UserData.Add("DebugIntersectionBy1", item.Key.face1.GetHashCode());
                            (tr as IGeoObject).UserData.Add("DebugIntersectionBy2", item.Key.face2.GetHashCode());
                            if (con2 is InterpolatedDualSurfaceCurve.ProjectedCurve)
                            {
                                BSpline2D dbgbsp2d = (con2 as InterpolatedDualSurfaceCurve.ProjectedCurve).ToBSpline(0.0);
                            }
#endif
                            if (dirs1) // the trimming of BSplines is sometimes not very exact
                            {
                                if (con1 is BSpline2D)
                                {
                                    con1.StartPoint = item.Key.face1.PositionOf(tr.StartPoint);
                                    con1.EndPoint = item.Key.face1.PositionOf(tr.EndPoint);
                                }
                                if (con2 is BSpline2D)
                                {
                                    con2.StartPoint = item.Key.face2.PositionOf(tr.EndPoint);
                                    con2.EndPoint = item.Key.face2.PositionOf(tr.StartPoint);
                                }
                            }
                            else
                            {
                                if (con1 is BSpline2D)
                                {
                                    con1.StartPoint = item.Key.face1.PositionOf(tr.EndPoint);
                                    con1.EndPoint = item.Key.face1.PositionOf(tr.StartPoint);
                                }
                                if (con2 is BSpline2D)
                                {
                                    con2.StartPoint = item.Key.face2.PositionOf(tr.StartPoint);
                                    con2.EndPoint = item.Key.face2.PositionOf(tr.EndPoint);
                                }
                            }
                            edge.Vertex1 = usedVertices[j1];    // damit wird diese Kante mit den beiden Schnittvertices verbunden
                            edge.Vertex2 = usedVertices[j2];
                            {
                                created.Add(edge);
                                // diese neue Kante in das Dictionary einfügen
                                bool addToFace1 = true, addToFace2 = true;
                                Edge[] splitted = null;
                                if (j1IsOnBorder && j2IsOnBorder)
                                {   // a very rare case (like in BRepTest30.cdb.json): the new intersecting edge starts and ends on the border of the face AND contains an already existing vertex of that face.
                                    // in this case we have to split the new edge
                                    if (isOnFace1Border)
                                    {   // first faster check: is the intersection edge tangential to an outline edge
                                        bool tangential = false;
                                        List<Edge> le = edge.Vertex1.EdgesOnFace(item.Key.face1);
                                        for (int k = 0; k < le.Count; k++)
                                        {
                                            if (le[k] == edge) continue;
                                            if (le[k].Vertex1 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.StartDirection, edge.Curve3D.StartDirection, false)) tangential = true;
                                            else if (le[k].Vertex2 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.EndDirection, edge.Curve3D.StartDirection, false)) tangential = true;
                                        }
                                        le = edge.Vertex2.EdgesOnFace(item.Key.face1);
                                        for (int k = 0; k < le.Count; k++)
                                        {
                                            if (le[k] == edge) continue;
                                            if (le[k].Vertex1 == edge.Vertex2 && Precision.SameDirection(le[k].Curve3D.StartDirection, edge.Curve3D.EndDirection, false)) tangential = true;
                                            else if (le[k].Vertex2 == edge.Vertex2 && Precision.SameDirection(le[k].Curve3D.EndDirection, edge.Curve3D.EndDirection, false)) tangential = true;
                                        }
                                        if (tangential)
                                        {
                                            SortedList<double, Vertex> splitPositions = new SortedList<double, Vertex>();
                                            foreach (Vertex vtx in item.Key.face1.Vertices)
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
                                        List<Edge> le = edge.Vertex1.EdgesOnFace(item.Key.face2);
                                        for (int k = 0; k < le.Count; k++)
                                        {
                                            if (le[k] == edge) continue;
                                            if (le[k].Vertex1 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.StartDirection, edge.Curve3D.StartDirection, false)) tangential = true;
                                            else if (le[k].Vertex2 == edge.Vertex1 && Precision.SameDirection(le[k].Curve3D.EndDirection, edge.Curve3D.StartDirection, false)) tangential = true;
                                        }
                                        le = edge.Vertex2.EdgesOnFace(item.Key.face2);
                                        for (int k = 0; k < le.Count; k++)
                                        {
                                            if (le[k] == edge || le[k].Curve3D == null) continue;
                                            if (le[k].Vertex1 == edge.Vertex2 && Precision.SameDirection(le[k].Curve3D.StartDirection, edge.Curve3D.EndDirection, false)) tangential = true;
                                            else if (le[k].Vertex2 == edge.Vertex2 && Precision.SameDirection(le[k].Curve3D.EndDirection, edge.Curve3D.EndDirection, false)) tangential = true;
                                        }
                                        if (tangential)
                                        {
                                            SortedList<double, Vertex> splitPositions = new SortedList<double, Vertex>();
                                            foreach (Vertex vtx in item.Key.face2.Vertices)
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
                                    if (!faceToIntersectionEdges.TryGetValue(item.Key.face1, out addTo))
                                    {
                                        addTo = new HashSet<Edge>(); // (new EdgeComparerByVertex()); // damit werden zwei Kanten mit gleichen Vertices nicht zugefügt, nutzt nichts
                                        faceToIntersectionEdges[item.Key.face1] = addTo;
                                    }
                                    if (splitted != null) addTo.UnionWith(splitted);
                                    else addTo.Add(edge);
                                }
                                if (addToFace2)
                                {
                                    if (!faceToIntersectionEdges.TryGetValue(item.Key.face2, out addTo))
                                    {
                                        addTo = new HashSet<Edge>(); //  (new EdgeComparerByVertex());
                                        faceToIntersectionEdges[item.Key.face2] = addTo;
                                    }
                                    if (splitted != null) addTo.UnionWith(splitted);
                                    else addTo.Add(edge);
                                }
                            }
                        }
                    }
                }
#if DEBUG
                else
                {

                }
#endif
            }
        }

        private static bool EdgesContainConnection(IEnumerable<Edge> edges, Vertex v1, Vertex v2)
        {
            foreach (Edge edg in edges)
            {
                if (edg.Vertex1 == v1 && edg.Vertex2 == v2) return true;
                if (edg.Vertex1 == v2 && edg.Vertex2 == v1) return true;
            }
            return false;
        }

        private HashSet<Face> findOverlappingPartner(Face face1)
        {
            HashSet<Face> res = new HashSet<Face>();
            foreach (DoubleFaceKey dfk in overlappingFaces.Keys)
            {
                if (dfk.face1 == face1) res.Add(dfk.face2);
                if (dfk.face2 == face1) res.Add(dfk.face1);
            }
            foreach (DoubleFaceKey dfk in oppositeFaces.Keys)
            {
                if (dfk.face1 == face1) res.Add(dfk.face2);
                if (dfk.face2 == face1) res.Add(dfk.face1);
            }
            return res;
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
                    list.AddRange(face.Area.DebugList); // das ist der bestehende Rand
                    HashSet<Edge> edges;
                    if (faceToIntersectionEdges.TryGetValue(face, out edges))
                    {   // das sind die neuen Kanten
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
                    list.AddRange(face.Area.DebugList); // das ist der bestehende Rand
                    HashSet<Edge> edges;
                    if (faceToIntersectionEdges.TryGetValue(face, out edges))
                    {   // das sind die neuen Kanten
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
        private bool ContainesHole(Face face, List<Edge> outline, List<Edge> hole)
        {
            ICurve2D[] bdroutline = new ICurve2D[outline.Count];
            for (int i = 0; i < bdroutline.Length; ++i)
            {
                bdroutline[i] = outline[i].Curve2D(face);
            }
            Border bdr = new Border(bdroutline);
            return bdr.GetPosition(hole[0].Curve2D(face).StartPoint) == Border.Position.Inside;
            // return Border.OutlineContainsPoint(outline, hole[0].Curve2D(face).StartPoint);
        }
        private bool IsHole(Face face, List<Edge> outline)
        {   // feststellen, ob die orientierte Liste von Edges rechtsrum (hole) oder linksrum (outline) geht
            GeoPoint sp = outline[0].StartVertex(face).Position;
            GeoPoint ep = outline[outline.Count - 1].EndVertex(face).Position;
            if ((sp | ep) > precision) return false;
            ICurve2D[] curves = new ICurve2D[outline.Count];
            for (int i = 0; i < curves.Length; ++i)
            {
                curves[i] = outline[i].Curve2D(face);

            }
            return !Border.CounterClockwise(curves);
        }
        protected override bool SplitNode(Node<BRepItem> node, BRepItem objectToAdd)
        {
#if DEBUG
            DebuggerContainer dc = new DebuggerContainer();
            foreach (BRepItem brep in node.list)
            {
                if (brep.edge != null)
                {
                    if (brep.edge.Curve3D != null) dc.Add(brep.edge.Curve3D as IGeoObject);
                }
                if (brep.face != null) dc.Add(brep.face);
                if (brep.vertex != null) dc.Add(brep.vertex.Position, Color.Red, 0);
            }
#endif
            if (node.deepth < 3 && node.list.Count > 3) return true; // noch einjustieren
            if (node.deepth > 8) return false; // Notbremse
                                               // Notbremse kann auftreten wenn mehrere Vertices einer Shell identisch sind oder Kanten
                                               // sich schneiden (dann sind 4 faces in einem Punkt, jeweils 2 von jeder Shell
                                               // solche Fälle müssten ggf vorab gechecked werden
            if (objectToAdd.Type == BRepItem.ItemType.Vertex)
            {   // keine zwei Vertices aus der selben Shell und auch Schnittvertices getrennt
                // von allen anderen
                // Warum keine zwei vertices aus der selben Shell? Das teilt den Octtree unnötig auf
                // wo gerkeine verschiedenen Shells beteiligt sind
                foreach (BRepItem bi in node.list)
                {
                    //if (bi.Type == BRepItem.ItemType.Vertex)
                    //{
                    //    //if (bi.vertex.Edges[0].PrimaryFace.Owner == objectToAdd.vertex.Edges[0].PrimaryFace.Owner ||
                    //    if (bi.isIntersection || objectToAdd.isIntersection)
                    //        return true;
                    //}
                }
            }
            else if (objectToAdd.Type == BRepItem.ItemType.Face)
            {   // eine der beiden Shells darf nur einfach vertreten sein, warum?
                int nums1 = 0;
                int nums2 = 0;
                if (objectToAdd.face.Owner == shell1) ++nums1;
                else ++nums2;
                foreach (BRepItem bi in node.list)
                {
                    if (bi.Type == BRepItem.ItemType.Face)
                    {
                        if (bi.face.Owner == shell1) ++nums1;
                        else ++nums2;
                    }
                }
                // return (nums1 > 1 && nums2 > 1); warum?
            }
            return false;
        }
        // erstmal weglassen, nicht klar ob das was bringt. In OctTree auch auskommentiert
        //protected override bool FilterHitTest(object objectToAdd, OctTree<BRepItem>.Node<BRepItem> node)
        //{
        //    if (node.list == null) return false; // Abkürzung nur wenn es eine Liste hat
        //    BRepItem bri = objectToAdd as BRepItem;
        //    if (bri.Type==BRepItem.ItemType.Edge)
        //    {
        //        foreach (BRepItem  bi in node.list)
        //        {
        //            if (bi.Type is Vertex)
        //            {
        //                foreach (Edge e in bi.vertex.Edges)
        //                {
        //                    if (e == bri.edge) return true;
        //                }
        //            }
        //        }
        //    }
        //    return false;
        //}

        /// <summary>
        /// Checks whether the two shells intersect each other
        /// </summary>
        /// <returns></returns>
        public bool Intersect(out GeoPoint anyIntersectionPoint)
        {
            if (edgesToSplit.Count > 0)
            {
                foreach (List<Vertex> list in edgesToSplit.Values)
                {
                    if (list.Count > 0)
                    {
                        anyIntersectionPoint = list[0].Position;
                        return true;
                    }
                }
            }
            anyIntersectionPoint = GeoPoint.Origin;
            return false;
        }

        /// <summary>
        /// When we know tangential intersections, we can provide them here. They will be taken into account when splitting the faces.
        /// </summary>
        /// <param name="tangentialEdges">Face of one Shell contains this edge of the other shell</param>
        public void AddTangentialEdges(Dictionary<Face, Edge> tangentialEdges)
        {
            foreach (KeyValuePair<Face, Edge> kv in tangentialEdges)
            {
                if (originalToClonedFaces.TryGetValue(kv.Key, out Face fc) && originalToClonedEdges.TryGetValue(kv.Value, out Edge edg))
                    this.tangentialEdges[fc] = edg;
            }
        }

        public GeoObjectList DebugEdgesToSplit
        {
            get
            {
                GeoObjectList res = new GeoObjectList();
                //Dictionary<Face, HashSet<Edge>> faceToIntersectionEdges;
                //Dictionary<Edge, List<Vertex>> edgesToSplit;
                //Dictionary<Face, HashSet<Edge>> facesToSplit; // Faces, dies gesplitted werden sollen und deren originale oder gesplittete
                ColorDef cdp = new ColorDef("point", Color.Red);
                ColorDef cde = new ColorDef("edge", Color.Blue);
                foreach (KeyValuePair<Edge, List<Vertex>> item in edgesToSplit)
                {
                    if (item.Key.Curve3D != null)
                    {
                        ((item.Key.Curve3D as IGeoObject) as IColorDef).ColorDef = cde;
                        res.Add(item.Key.Curve3D as IGeoObject);
                    }
                    foreach (Vertex v in item.Value)
                    {
                        CADability.GeoObject.Point p = CADability.GeoObject.Point.Construct();
                        p.Location = v.Position;
                        p.Symbol = PointSymbol.Cross;
                        p.ColorDef = cdp;
                        res.Add(p);
                    }
                }
                return res;
            }
        }
    }

    public static class BOExtension
    {
        /// <summary>
        /// Check whether the Face face is totaly covered by the corresponding face(s) in the provided dict.
        /// </summary>
        /// <param name="dict"></param>
        /// <param name="face"></param>
        /// <param name="precision"></param>
        /// <returns></returns>
        public static bool ContainsSameFace(this Dictionary<Face, HashSet<Face>> dict, Face face, double precision)
        {
            if (dict.TryGetValue(face, out HashSet<Face> commonWith))
            {
                Set<Edge> edges = face.AllEdgesSet;
                Set<Vertex> vertices = new Set<Vertex>();
                foreach (Edge edg in edges)
                {
                    vertices.Add(edg.Vertex1);
                    vertices.Add(edg.Vertex2);
                }
                Set<Vertex> otherVertices = new Set<Vertex>();
                Set<Edge> otherEdges = new Set<Edge>();
                foreach (Face fce in commonWith)
                {
                    otherVertices.AddMany(fce.Vertices);
                    otherEdges.AddMany(fce.Edges);
                }
                foreach (Edge edg in edges)
                {
                    bool edgeFound = false;
                    foreach (Edge edg1 in Vertex.ConnectingEdges(edg.Vertex1, edg.Vertex2))
                    {
                        if (BRepOperation.SameEdge(edg, edg1, precision))
                        {
                            edgeFound = true;
                            break;
                        }
                    }
                    if (!edgeFound) return false;
                }
                return true;
            }
            return false;
        }

    }
}
