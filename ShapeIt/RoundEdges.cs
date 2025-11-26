using CADability;
using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using ExCSS;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms.VisualStyles;
using static ShapeIt.ShellExtensions;

namespace ShapeIt
{
    internal class RoundEdges
    {
        Shell shell;
        IEnumerable<Edge> convexEdges;
        IEnumerable<Edge> concaveEdges;
        double radius;
        /// <summary>
        /// - fillet: the face which rounds the edge
        /// - frontEnd: the two arcs at the end of the fillet face
        /// - tangent: the edges tangential to the primary face [0] and secondary face [1]
        /// </summary>
        Dictionary<Edge, Shell>? edgeToFillet;
        /// <summary>
        /// Tool to round the given edges of a shell with the given radius.
        /// </summary>
        /// <param name="shell"></param>
        /// <param name="edges"></param>
        /// <param name="radius"></param>
        public RoundEdges(Shell shell, IEnumerable<Edge> edges, double radius)
        {
            this.shell = shell;
            this.radius = Math.Abs(radius); // radius may not be negative
            // convex and concave: an edge is convex when the angle between the two faces is less than 108°, concave when greater than 180°
            convexEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Convex);
            concaveEdges = edges.Where(e => e.Adjacency() == ShellExtensions.AdjacencyType.Concave);
        }

        public Shell? Execute()
        {
            // 1. make a raw fillet for each edge. The fillet is a swept circle around a spine curve. The spine curve is the intersection
            // of two offset surfaces of the adjacent faces. The fillet starts and ends with a circular arc and has two tangential edges to the adjacent faces.
            // Later we have to trim or extent the fillet faces to get a proper result. The end vertices of the edge lie in the planes of the front arcs.
            edgeToFillet = createFillets(convexEdges);
            // there are one or more edges meeting at a vertex.
            Dictionary<Vertex, List<Edge>> vertexToEdges = createVertexToEdges(convexEdges);

            List<HashSet<Shell>> roundingShells = []; // each hashset contains the faces of one rounding shell: the fillet and maybe some patches
            foreach (var ve in vertexToEdges)
            {
                if (ve.Value.Count == 1)
                { // the fillet ends here, there are different cases:
                    // There is one or more "impact" faces
                    HashSet<Shell>? filletAndExtension = createDeadEndExtension(ve.Key, ve.Value[0]);
                    if (filletAndExtension != null) roundingShells.Add(filletAndExtension);
                }
                else if (ve.Value.Count == 2)
                {
                    HashSet<Shell>? filletAndExtension = createExtensionTwoEdges(ve.Key, ve.Value[0], ve.Value[1]);
                    if (filletAndExtension != null) roundingShells.Add(filletAndExtension);
                }
            }
            Combine(roundingShells);

            // here we have the convex rounded edges as shells, which have to be subtracted from the shell on which the edges are to be rounded
            Shell? toOperateOn = shell.Clone() as Shell;
            for (int i = 0; i < roundingShells.Count; i++)
            {
                foreach (var item in roundingShells[i])
                {
                    BooleanOperation bo = new BooleanOperation();
                    bo.SetShells(toOperateOn, item, BooleanOperation.Operation.difference);
                    Shell[] roundedShells = bo.Execute();
                    if (roundedShells != null && roundedShells.Length == 1) toOperateOn = roundedShells[0];
                }
            }

            return toOperateOn;
        }

        private HashSet<Shell>? createExtensionTwoEdges(Vertex vtx, Edge edge1, Edge edge2)
        {
            Face? commonFace = Edge.CommonFace(edge1, edge2);
            Edge? thirdEdge = vtx.AllEdges.Except([edge1, edge2]).TheOnlyOrDefault();
            // in most cases we have a vertex with three faces meeting, so one common face and one third edge
            // if there are more than three faces meeting at the vertex, we cannot handle this currently
            if (commonFace == null) return null; // there must be a common face
            GeoVector normalCommon = commonFace.Surface.GetNormal(vtx.GetPositionOnFace(commonFace)).Normalized;
            Shell? fillet1 = edgeToFillet?[edge1];
            Shell? fillet2 = edgeToFillet?[edge2];
            if (fillet1 == null || fillet2 == null) return null;
            Face? endFace1 = fillet1.Faces.Where(f => f.UserData.Contains("CADability.Fillet.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            Face? endFace2 = fillet2.Faces.Where(f => f.UserData.Contains("CADability.Fillet.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            if (endFace1 == null && endFace2 == null) return null;
            GeoVector n1 = (endFace1.Surface as PlaneSurface).Normal.Normalized; // endfaces are always PlaneSurfaces
            GeoVector n2 = (endFace2.Surface as PlaneSurface).Normal.Normalized;
            double orientation = (n1 ^ n2) * normalCommon;
            if (Math.Abs(orientation) < 1e-6)
            {   // tangential connection, we need the two fillets without any connection patch in between
                return [fillet1, fillet2];
            }
            if (orientation < 0)
            {
                // convex connection, we have to extent the fillets at this point
                Shell? extension1 = (Make3D.Extrude(endFace1.Clone(), 2 * radius * n1, null) as Solid)?.Shells[0];
                Shell? extension2 = (Make3D.Extrude(endFace2.Clone(), 2 * radius * n2, null) as Solid)?.Shells[0];
                if (extension1 != null && extension2 != null) return [fillet1, fillet2, extension1, extension2];
            }
            return [fillet1, fillet2];

        }

        /// <summary>
        /// Create a fillet as a single face, consisting of a ISurfaceOfExtrusion surface (cylinder, torus, sweptCircle) and four edges:
        /// two arcs at the ends and two tangential curves to the adjacent faces
        /// </summary>
        /// <param name="edges"></param>
        /// <returns> a dictionary with
        /// - fillet: the face the bounds of the edge
        /// - frontEnd: the two arcs at the end of the fillet face
        /// - tangent: the edges tangential to the primary face [0] and secondary face [1]
        /// for each edge
        /// </returns>
        private Dictionary<Edge, Shell> createFillets(IEnumerable<Edge> edges)
        {
            Dictionary<Edge, Shell> edgeToFillet = new();
            foreach (Edge edgeToRound in edges)
            {
                Shell? filletShell = MakeConvexFilletShell(edgeToRound, radius);
                if (filletShell != null)
                {
                    edgeToFillet[edgeToRound] = filletShell;
                }
            }
            return edgeToFillet;
        }

        private Dictionary<Vertex, List<Edge>> createVertexToEdges(IEnumerable<Edge> edges)
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
        private HashSet<Shell>? createDeadEndExtension(Vertex vtx, Edge edge)
        {
            if (edgeToFillet == null) return null;
            Shell fillet = edgeToFillet[edge];
            Face? endFace = fillet.Faces.Where(f => f.UserData.Contains("CADability.Fillet.EndFace")).MinBy(f => f.Surface.GetDistance(vtx.Position));
            if (endFace == null) return [fillet]; // should not happen
            // the end face of the simple fillet has 3 edges and 3 vertices: the two edges, which are connected to vtx and the third edge, which is a circular arc
            // and convex from the vtx position. We test the beam from the vertex vtx to the outside of the shell,  and the two beams from the arc to the outside.
            // if the vertex beam is totally outside the shell and the two other beams leave the shell in a short distance (<2*radius), then we extend the fillet
            GeoPoint2D uv = vtx.GetPositionOnFace(endFace);
            GeoVector beamDirection = endFace.Surface.GetNormal(uv).Normalized;
            double vtxbeam = minBeamDist(vtx.Position, beamDirection);
            Edge endArc = endFace.AllEdges.First(e => e.Vertex1 != vtx && e.Vertex2 != vtx);
            if (endArc == null) return [fillet]; // should not happen
            double startbeam = minBeamDist(endArc.Curve3D.PointAt(0.01), beamDirection); // not exactely endpoint, because it is tangential to a face
            double endbeam = minBeamDist(endArc.Curve3D.PointAt(0.99), beamDirection);
            double middlebeam = minBeamDist(endArc.Curve3D.PointAt(0.5), beamDirection);
#if DEBUG
            // to watch the position:
            DebuggerContainer dcPos = new DebuggerContainer();
            dcPos.Add(shell);
            dcPos.Add(fillet);
            dcPos.Add(Line.MakeLine(vtx.Position, vtx.Position + 3 * radius * beamDirection), System.Drawing.Color.Red);
            dcPos.Add(Line.MakeLine(endArc.Curve3D.PointAt(0.01), endArc.Curve3D.PointAt(0.01) + 3 * radius * beamDirection), System.Drawing.Color.Green);
            dcPos.Add(Line.MakeLine(endArc.Curve3D.PointAt(0.99), endArc.Curve3D.PointAt(0.99) + 3 * radius * beamDirection), System.Drawing.Color.Green);
            dcPos.Add(Line.MakeLine(endArc.Curve3D.PointAt(0.5), endArc.Curve3D.PointAt(0.5) + 3 * radius * beamDirection), System.Drawing.Color.Green);
#endif
            if (vtxbeam == 0.0 && startbeam < 3 * radius && endbeam < 3 * radius && middlebeam < 3 * radius)
            {   // the beamm from the vertex is outside the shell, the other two beams leave the shell in a short distance or ar outside: make a short extension
                Shell? extension = (Make3D.Extrude(endFace.Clone(), 3 * radius * beamDirection, null) as Solid)?.Shells[0];
                if (extension != null) return [fillet, extension];
            }
            return [fillet];

            /*
            ISurfaceOfExtrusion? filletSurface = fillet.Surface as ISurfaceOfExtrusion;
            if (filletSurface == null) return null;
            Edge? openEdge = edgeToFillet[edge].frontEnd?.MinBy(edg => edg.Curve3D.DistanceTo(vtx.Position));
            if (openEdge == null) return null;
            Ellipse? ellipse = openEdge?.Curve3D as Ellipse;
            bool ellipseIsForwardOnFillet = openEdge.Forward(fillet);
            // we need to check whether we need a face extension on the third (impact) face here
            if (ellipse == null) return null; // should not happen
            GeoPoint cnt = ellipse.Center;
            GeoVector edgeDirAtTheEnd; // the direction at the end of the fillet, at the position of this vertex, poiting outward in the extension of the fillet
            if (vtx == edge.Vertex1) edgeDirAtTheEnd = -edge.Curve3D.StartDirection.Normalized;
            else if (vtx == edge.Vertex2) edgeDirAtTheEnd = edge.Curve3D.EndDirection.Normalized;
            else return null; // this should not happen
            IEnumerable<Face> thirdFaces = vtx.InvolvedFaces.Except([edge.PrimaryFace, edge.SecondaryFace]);
            GeoVector dirOfImpactFaces = GeoVector.NullVector;
            foreach (Face fc in thirdFaces)
            {
                dirOfImpactFaces += fc.Surface.GetNormal(vtx.GetPositionOnFace(fc)).Normalized;
            }
            if (dirOfImpactFaces.IsNullVector()) return null; // no impact faces?
                                                              // we have to decide how to extent the fillet in this direction, there is no simple, definite and obvious solution
                                                              // first check, whether the two tangents of the fillet intersect the shell
            if (edgeToFillet[edge].tangent == null || edgeToFillet[edge].tangent?.Length != 2) return null; // there must be two tangents where the fillet touches the shell
            bool shortExtensionIsOk = true;
            bool allOutside = true;
            foreach (GeoPoint startPoint in new List<GeoPoint>([ellipse.PointAt(0.05), ellipse.PointAt(0.5), ellipse.PointAt(0.95)]))
            {
                // lets see, whether the extension intersects the shell close to the point
                if (shell.IsInside(startPoint)) // this point on the fillet ellipse is inside, check whether we can savely expand it
                {
                    allOutside = false;
                    GeoPoint[] extIntsect = shell.GetLineIntersection(startPoint, edgeDirAtTheEnd);
                    bool ok = false;
                    for (int i = 0; i < extIntsect.Length; i++)
                    {
                        double par = Geometry.LinePar(startPoint, edgeDirAtTheEnd, extIntsect[i]);
                        if (par > 0 && par < 2 * radius)
                        {
                            ok = true;
                            break;
                        }
                    }
                    if (!ok)
                    {
                        shortExtensionIsOk = false;
                        break;
                    }
                }
            }
            if (allOutside)
            {   // the whole ending arc seems to be outside the shell, no extension needed (impact face is bended towards the edge
                if (!curveIntersectsShell(ellipse)) return [fillet];
            }
            double impact = dirOfImpactFaces.Normalized * edgeDirAtTheEnd.Normalized;
            // if the impact faces point in the same direction as the fillet, they are probably trimmed by the fillet
            // if it points the other way, it may have to be extended
            Face? extruded = Make3D.Extrude(ellipse, 2 * radius * edgeDirAtTheEnd, null) as Face;
            if (shortExtensionIsOk && impact > 0 && extruded != null)
            {   // the linear extension of the fillet intersects the shell in less than 2*radius so lets try a simple cylindrical extension fo 2*radius
                return [extruded, fillet];
            }
            Face impactFace = vtx.Faces.Except([edge.PrimaryFace, edge.SecondaryFace]).First(); // how to deal with multiple?
            if (impact < 0 && impactFace != null)
            {   // we brutally end the fillet with a perpendicular plane surface
                // the vertex should lie in the ellipse plane
                if (Math.Abs(ellipse.Plane.Distance(vtx.Position)) < Precision.eps)
                {   // one edge of the plane is the ellipse, the other edges is a circular arc through the vertex and the two ellipse end points
                    Plane plane = new Plane(vtx.Position, -edgeDirAtTheEnd); // origin is vtx.Position, pointing towards the fillet
                    PlaneSurface endCapPlane = new PlaneSurface(plane);
                    Arc2D? filletEndArc = ellipse.GetProjectedCurve(plane) as Arc2D;
                    // which endpoint of the arc is on which face?
                    GeoPoint pointOnPrimaryFace, pointOnSecondaryFace;
                    bool orientation;
                    if (Math.Abs(edge.PrimaryFace.Surface.GetDistance(ellipse.StartPoint)) + Math.Abs(edge.SecondaryFace.Surface.GetDistance(ellipse.EndPoint)) < Math.Abs(edge.PrimaryFace.Surface.GetDistance(ellipse.EndPoint)) + Math.Abs(edge.SecondaryFace.Surface.GetDistance(ellipse.StartPoint)))
                    {
                        orientation = ellipseIsForwardOnFillet;
                        pointOnPrimaryFace = ellipse.StartPoint;
                        pointOnSecondaryFace = ellipse.EndPoint;
                    }
                    else
                    {
                        orientation = !ellipseIsForwardOnFillet;
                        pointOnPrimaryFace = ellipse.EndPoint;
                        pointOnSecondaryFace = ellipse.StartPoint;
                    }
                    double d = pointOnPrimaryFace | vtx.Position;
                    ICurve? crvPrimFace = null, crvSecFace = null;
                    ICurve2D? crv2dPrimFace = null, crv2dSecFace = null;
                    IDualSurfaceCurve[] dscOnPrimFace = edge.PrimaryFace.Surface.GetDualSurfaceCurves(edge.PrimaryFace.Domain, endCapPlane, new BoundingRect(GeoPoint2D.Origin, 2 * d, 2 * d), [vtx.Position, pointOnPrimaryFace]);
                    IDualSurfaceCurve[] dscOnSecFace = edge.SecondaryFace.Surface.GetDualSurfaceCurves(edge.SecondaryFace.Domain, endCapPlane, new BoundingRect(GeoPoint2D.Origin, 2 * d, 2 * d), [vtx.Position, pointOnPrimaryFace]);
                    if (dscOnPrimFace != null && dscOnSecFace != null && dscOnPrimFace.Length > 0 && dscOnSecFace.Length > 0)
                    {
                        dscOnPrimFace[0].Trim(pointOnPrimaryFace, vtx.Position);
                        crvPrimFace = dscOnPrimFace[0].Curve3D;
                        crv2dPrimFace = dscOnPrimFace[0].Curve2D2;
                        if (!orientation)
                        {
                            crv2dPrimFace.Reverse();
                            crvPrimFace.Reverse();
                        }
                        dscOnSecFace[0].Trim(pointOnSecondaryFace, vtx.Position);
                        crvSecFace = dscOnSecFace[0].Curve3D;
                        crv2dSecFace = dscOnSecFace[0].Curve2D2;
                        if (orientation)
                        {
                            crv2dSecFace.Reverse();
                            crvSecFace.Reverse();
                        }
                        Face planarEnd = Face.Construct(); // this will be the end cap, outlined by the arc and the two intersections with the adjacent faces
                        ICurve2D eArc2d = ellipse.GetProjectedCurve(plane);
                        if (ellipseIsForwardOnFillet) eArc2d.Reverse();
                        Edge eArc = new Edge(planarEnd, ellipse.Clone() as ICurve, planarEnd, eArc2d, !ellipseIsForwardOnFillet);
                        Edge ePrimFace = new Edge(planarEnd, crvPrimFace, planarEnd, crv2dPrimFace, true);
                        Edge eSecFace = new Edge(planarEnd, crvSecFace, planarEnd, crv2dSecFace, true);
                        if (orientation) planarEnd.Set(endCapPlane, [[eArc, ePrimFace, eSecFace]]);
                        else planarEnd.Set(endCapPlane, [[eArc, eSecFace, ePrimFace]]);
                        // now we go half way back to the center of the fillet and make two faces: the parts of the adjacent faces that are to be cut off.
                        // This helps to get out of the solid, if we went into it with the fillet
                        ICurve2D? tangentOnPrim = null, tangentOnSec = null;
                        ISurface primSurface = edge.PrimaryFace.Surface.Clone();
                        ISurface secSurface = edge.SecondaryFace.Surface.Clone();
                        primSurface.ReverseOrientation();
                        secSurface.ReverseOrientation();
                        ICurve2D capOnPrim = primSurface.GetProjectedCurve(crvPrimFace, 0.0);
                        ICurve2D capOnSec = secSurface.GetProjectedCurve(crvSecFace, 0.0);
                        GeoPoint2D tmp = GeoPoint2D.Invalid, tms = GeoPoint2D.Invalid, emp, ems; // middle points on primary, secondary surface
                        foreach (Edge tedg in edgeToFillet[edge].tangent)
                        {
                            if (Math.Abs(edge.PrimaryFace.Surface.GetDistance(tedg.Curve3D.PointAt(0.5))) < Math.Abs(edge.SecondaryFace.Surface.GetDistance(tedg.Curve3D.PointAt(0.5))))
                            {
                                // tedg is on primary face
                                tangentOnPrim = primSurface.GetProjectedCurve(tedg.Curve3D, 0.0);
                                if ((tedg.Curve3D.EndPoint | vtx.Position) < (tedg.Curve3D.StartPoint | vtx.Position))
                                {
                                    tangentOnPrim = tangentOnPrim.Trim(0.5, 1.0);
                                    tmp = tangentOnPrim.StartPoint;
                                }
                                else
                                {
                                    tangentOnPrim = tangentOnPrim.Trim(0.0, 0.5);
                                    tmp = tangentOnPrim.EndPoint;
                                }
                            }
                            else
                            {
                                // tedg is on primary face
                                tangentOnSec = secSurface.GetProjectedCurve(tedg.Curve3D, 0.0);
                                if ((tedg.Curve3D.EndPoint | vtx.Position) < (tedg.Curve3D.StartPoint | vtx.Position))
                                {
                                    tangentOnSec = tangentOnSec.Trim(0.5, 1.0);
                                    tms = tangentOnSec.StartPoint;
                                }
                                else
                                {
                                    tangentOnSec = tangentOnSec.Trim(0.0, 0.5);
                                    tms = tangentOnSec.EndPoint;
                                }
                            }
                        }
                        ICurve2D edgOnPrim = primSurface.GetProjectedCurve(edge.Curve3D, 0.0);
                        ICurve2D edgOnSec = secSurface.GetProjectedCurve(edge.Curve3D, 0.0);
                        if (tangentOnPrim != null && tangentOnSec != null && edgOnPrim != null && edgOnSec != null)
                        {
                            if ((edgOnPrim.StartPoint | capOnPrim.PointAt(0.5)) < (edgOnPrim.EndPoint | capOnPrim.PointAt(0.5)))
                            {
                                edgOnPrim = edgOnPrim.Trim(0.0, 0.5);
                                emp = edgOnPrim.EndPoint;
                            }
                            else
                            {
                                edgOnPrim = edgOnPrim.Trim(0.5, 1.0);
                                emp = edgOnPrim.StartPoint;
                            }
                            if ((edgOnSec.StartPoint | capOnSec.PointAt(0.5)) < (edgOnSec.EndPoint | capOnSec.PointAt(0.5)))
                            {
                                edgOnSec = edgOnSec.Trim(0.0, 0.5);
                                ems = edgOnSec.EndPoint;
                            }
                            else
                            {
                                edgOnSec = edgOnSec.Trim(0.5, 1.0);
                                ems = edgOnSec.StartPoint;
                            }
                            Border bPrim = Border.FromUnorientedList([edgOnPrim, capOnPrim, tangentOnPrim, new Line2D(emp, tmp)], true);
                            Border bSec = Border.FromUnorientedList([edgOnSec, capOnSec, tangentOnSec, new Line2D(ems, tms)], true);
                            Face primStrip = Face.MakeFace(primSurface, new SimpleShape(bPrim));
                            Face secStrip = Face.MakeFace(secSurface, new SimpleShape(bSec));
                            dontUseForOverlapping.AddRange([primStrip, secStrip]);
                            return [fillet, planarEnd, primStrip, secStrip]; // don't use primStrip and secStrip as overlapping in BooleanOperation!!!
                        }
                    }
                }
            }
            #region tried_impact_face
            //if (impact < 0 && impactFace != null && extruded != null)
            //{   // maybe we can extend the impact face to the rounding extension
            //    GeoPoint2D[] spInts = impactFace.Surface.GetLineIntersection(ellipse.StartPoint, edgeDirAtTheEnd);
            //    GeoPoint2D[] epInts = impactFace.Surface.GetLineIntersection(ellipse.EndPoint, edgeDirAtTheEnd);
            //    if (spInts != null && spInts.Length > 0 && epInts != null && epInts.Length > 0)
            //    {
            //        GeoPoint sp = GeoPoint.Invalid;
            //        GeoPoint ep = GeoPoint.Invalid;
            //        double mindist = double.MaxValue;
            //        for (int i = 0; i < spInts.Length; i++)
            //        {
            //            GeoPoint p = impactFace.Surface.PointAt(spInts[i]);
            //            double par = Geometry.LinePar(ellipse.StartPoint, edgeDirAtTheEnd, p);
            //            if (par > 0 && par < mindist)
            //            {
            //                mindist = par;
            //                sp = p;
            //            }
            //        }
            //        mindist = double.MaxValue;
            //        for (int i = 0; i < spInts.Length; i++)
            //        {
            //            GeoPoint p = impactFace.Surface.PointAt(epInts[i]);
            //            double par = Geometry.LinePar(ellipse.EndPoint, edgeDirAtTheEnd, p);
            //            if (par >= 0 && par < mindist)
            //            {
            //                mindist = par;
            //                ep = p;
            //            }
            //        }
            //        if (sp.IsValid && ep.IsValid)
            //        {
            //            // both edges of the extruded face lie on the impact face.
            //            // now there must be two edges of the impact face at this vertex, which contain the points sp and ep
            //            List<Edge> edgesOnImpactFace = vtx.EdgesOnFace(impactFace);
            //            List<ICurve2D> curvesForImpactFaceExtension = [];
            //            for (int i = 0; i < edgesOnImpactFace.Count; i++)
            //            {
            //                if (edgesOnImpactFace[i].Curve3D.DistanceTo(sp) < Precision.eps)
            //                {
            //                    ICurve curve = edgesOnImpactFace[i].Curve3D.Clone();
            //                    double pos = curve.PositionOf(sp);
            //                    if (edgesOnImpactFace[i].Vertex1 == vtx) curve.Trim(0, pos);
            //                    else curve.Trim(pos, 1.0);
            //                    curvesForImpactFaceExtension.Add(impactFace.Surface.GetProjectedCurve(curve, 0.0));
            //                }
            //            }
            //            if (curvesForImpactFaceExtension.Count == 2)
            //            {
            //                // now create a face from the two curves and the intersection of the extruded face with the impactFace
            //                BoundingRect ext = impactFace.Domain;
            //                GeoPoint2D uv = impactFace.Surface.PositionOf(sp);
            //                SurfaceHelper.AdjustPeriodic(impactFace.Surface, ext, ref uv);
            //                ext.MinMax(uv);
            //                uv = impactFace.Surface.PositionOf(ep);
            //                SurfaceHelper.AdjustPeriodic(impactFace.Surface, ext, ref uv);
            //                ext.MinMax(uv);
            //                IDualSurfaceCurve[] iscurve = impactFace.Surface.GetDualSurfaceCurves(ext, extruded.Surface, extruded.Domain, [sp, ep]);
            //                if (iscurve != null && iscurve.Length == 1)
            //                {
            //                    curvesForImpactFaceExtension.Add(iscurve[0].Curve2D1);
            //                    Face? extendedImpactFace = Face.MakeFace(impactFace.Surface.Clone(), [curvesForImpactFaceExtension]);
            //                    if (extendedImpactFace != null)
            //                    {
            //                        return [extendedImpactFace, fillet];
            //                    }
            //                }
            //            }
            //        }
            //    }
            //}
            #endregion
            // if nothing worked out, 
            GeoVector torusXAxis = (vtx.Position - cnt).Normalized;
            ToroidalSurface ts = new ToroidalSurface(vtx.Position, torusXAxis, edgeDirAtTheEnd.Normalized, -(torusXAxis ^ edgeDirAtTheEnd).Normalized, vtx.Position | cnt, radius);
            // the ellipse lies on the torus surface, but we need to define a domain that covers the ellipse
            GeoPoint2D uvsp = ts.PositionOf(ellipse.StartPoint);
            GeoPoint2D uvep = ts.PositionOf(ellipse.EndPoint);
            BoundingRect torusDomain = new BoundingRect(uvep, uvsp); // this should work without periodic adjustments, because we are at the inside of the torus surface
            torusDomain.Left = torusDomain.Right - Math.PI; // a 180° segment is sufficient
            Face torusFace = Face.MakeFace(ts, torusDomain);
            torusFace.ReverseOrientation();
            return [torusFace, fillet];
            */
        }
        private bool curveIntersectsShell(ICurve curve)
        {
            for (int i = 0; i < shell.Faces.Length; i++)
            {
                shell.Faces[i].Intersect(curve, out GeoPoint[] ip, out GeoPoint2D[] uvOnFace, out double[] uOnCurve);
                for (int k = 0; k < uOnCurve.Length; k++)
                {
                    if (-Precision.eps <= uOnCurve[k] && uOnCurve[k] <= 1.0 + Precision.eps && shell.Faces[i].Contains(ref uvOnFace[k], true)) return true;
                }
            }
            return false;
        }
        /// <summary>
        /// Combine overlapping sets into single sets
        /// </summary>
        /// <param name="sets"></param>
        private void Combine(List<HashSet<Shell>> sets)
        {
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
    }
}
