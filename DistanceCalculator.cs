using CADability.GeoObject;
using CADability;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeIt
{
    internal class DistanceCalculator
    {
        /// <summary>
        /// Calculate the distance between the two BRep objects (edge, face or vertex) <paramref name="fromHere"/> and <paramref name="toHere"/>. When both objects are vertices,
        /// the result will be a connection between these two vertices if <paramref name="preferredDirection"/> is the null vector. If there is a preferred direction, the result
        /// will be a line parallel to preferred direction. And if <paramref name="preferredPoint"/> is valid, it will go through it.
        /// If the two objects are faces, the result will be a perpendicular connection between these faces or if preferred direction is valid, it will be a connection in this direction
        /// Also the preferred point will be taken into account, if valid. The same is true for two edges.
        /// The <paramref name="degreeOfFreedom"/> vector will be invalid, if it is two vertices directely, if two vertices with preferred direction or two lines, the <paramref name="degreeOfFreedom"/>
        /// will be in the direction of the lines. Whith two planar faces, the <paramref name="degreeOfFreedom"/> will be a null vector, which means that the plane perpendicular to the 
        /// direction of the result is a degree of freedom. <paramref name="dimStartPoint"/> and <paramref name="dimEndPoint"/> may be used for the display of a dimension line. If
        /// provided by this method. E.g. the closest position of an arc
        /// </summary>
        /// <param name="fromHere"></param>
        /// <param name="toHere"></param>
        /// <param name="preferredDirection"></param>
        /// <param name="preferredPoint"></param>
        /// <param name="degreeOfFreedom"></param>
        /// <param name="dimStartPoint">actual point, which was used for the calculation</param>
        /// <param name="dimEndPoint">actual point, which was used for the calculation</param>
        /// <returns></returns>
        public static (GeoPoint startPoint, GeoPoint endPoint) DistanceBetweenObjects(object fromHere, object toHere, GeoVector preferredDirection, GeoPoint preferredPoint,
            out GeoVector degreeOfFreedom, out GeoPoint dimStartPoint, out GeoPoint dimEndPoint)
        {
            degreeOfFreedom = GeoVector.Invalid;
            dimStartPoint = dimEndPoint = GeoPoint.Invalid;

            bool swapped = false;
            GeoPoint startPoint = GeoPoint.Invalid;
            GeoPoint endPoint = GeoPoint.Invalid;

            // order according to Face->GeoPoint->ICurve to simplify the following cases (C# 8 or greater not available)
            string fname = fromHere.GetType().Name;
            if (fromHere is Edge e3)
            {
                fromHere = e3.Curve3D;
                fname = "ICurve";
            }
            else if (fromHere is ICurve) fname = "ICurve";
            else if (fromHere is Vertex vtx1)
            {
                fromHere = vtx1.Position;
                fname = "GeoPoint";
            }
            string tname = toHere.GetType().Name;
            if (toHere is Edge e4)
            {
                toHere = e4.Curve3D;
                tname = "ICurve";
            }
            else if (toHere is ICurve) tname = "ICurve";
            else if (toHere is Vertex vtx2)
            {
                toHere = vtx2.Position;
                tname = "GeoPoint";
            }
            if (string.Compare(fname, tname) > 0)
            {
                var temp = fromHere;
                fromHere = toHere;
                toHere = temp;
                swapped = true;
            }

            if (fromHere is GeoPoint p1 && toHere is GeoPoint p2)
            {
                if (preferredDirection.IsValid() && !preferredDirection.IsNullVector())
                {   // we need to use this direction, but I don't think we should consider preferredPoint
                    Plane pln = new Plane(p1, preferredDirection); // the plane, perpendicular to the preferred direction
                    GeoPoint v2p = pln.ToLocal(p2);
                    double dist = pln.Distance(p2);
                    startPoint = pln.ToGlobal(new GeoPoint(v2p.x / 2.0, v2p.y / 2.0, 0.0));
                    endPoint = startPoint + dist * preferredDirection.Normalized;
                    degreeOfFreedom = pln.ToGlobal(new GeoVector(v2p.x, v2p.y, 0.0)); // this is the connection of the two points in the plane
                    if (degreeOfFreedom.IsNullVector()) degreeOfFreedom = GeoVector.Invalid;
                }
                else
                {   // simply the connection of both vertices
                    startPoint = p1;
                    endPoint = p2;
                }
            }
            else if (fromHere is GeoPoint p3 && toHere is ICurve c3)
            {
                if (c3.GetPlanarState() == PlanarState.Planar)
                {
                    Plane pln = c3.GetPlane();
                    endPoint = pln.FootPoint(p3);
                    startPoint = p3;
                }
                else if (c3 is Line line)
                {
                    endPoint = Geometry.DropPL(p3, line.StartPoint, line.EndPoint);
                    startPoint = p3;
                }
            }
            else if (fromHere is ICurve c1 && toHere is ICurve c2)
            {
                if (preferredDirection.IsValid() && !preferredDirection.IsNullVector())
                {   // we need to use this direction
                    // check for extreme positions of the curve in this direction. E.g. an arc typically has a maximum or minimum, which would be useful here
                    // if multiple points ar available, no criterium is know to prefer one of them
                    double[] ep1 = c1.GetExtrema(preferredDirection);
                    double[] ep2 = c2.GetExtrema(preferredDirection);
                    if (ep1 == null || ep1.Length == 0) dimStartPoint = c1.StartPoint; // maybe there is a better solution
                    else dimStartPoint = c1.PointAt(ep1[0]);
                    if (ep2 == null || ep2.Length == 0) dimEndPoint = c2.EndPoint; // maybe there is a better solution
                    else dimStartPoint = c2.PointAt(ep2[0]);
                    GeoPoint m = new GeoPoint(dimStartPoint, dimEndPoint);
                    if (preferredPoint.IsValid) m = preferredPoint;
                    startPoint = Geometry.DropPL(dimStartPoint, m, preferredDirection);
                    endPoint = Geometry.DropPL(dimEndPoint, m, preferredDirection);
                    if (Curves.GetCommonPlane(c1, c2, out Plane commonPlane))
                    {
                        commonPlane.Intersect(new Plane(m, preferredDirection), out GeoPoint _, out degreeOfFreedom);
                    }
                    else degreeOfFreedom = GeoVector.NullVector;
                }
                else
                {
                    if (c1 is Line l1 && c2 is Line l2)
                    {   // two lines
                        if (Precision.SameDirection(l1.StartDirection, l2.StartDirection, false))
                        {   // two parallel lines define a plane
                            GeoVector dir = Geometry.DropPL(l2.StartPoint, l1.StartPoint, l1.EndPoint) - l2.StartPoint;
                            if (!dir.IsNullVector())
                            {   // otherwise the lines are identical, there is no solution
                                GeoPoint m = new GeoPoint(l1.PointAt(0.5), l2.PointAt(0.5));
                                if (preferredPoint.IsValid) m = preferredPoint;
                                startPoint = Geometry.DropPL(l1.StartPoint, m, dir);
                                endPoint = Geometry.DropPL(l2.StartPoint, m, dir);
                                degreeOfFreedom = l1.StartDirection;
                            }
                        }
                        else
                        {   // two general lines: find the closts connection
                            double dist = Geometry.DistLL(l1.StartPoint, l1.StartDirection, l2.StartPoint, l2.StartDirection, out double par1, out double par2);
                            if (dist > 0)
                            {
                                startPoint = l1.PointAt(par1);
                                endPoint = l2.PointAt(par2);
                                // no degree of freedom
                            }
                        }
                    }
                    else if (Curves.GetCommonPlane(c1, c2, out Plane commonPlane))
                    {   // the edges are in a common plane
                        double pos1 = 0.5, pos2 = 0.5;
                        if (Curves.NewtonMinDist(c1, ref pos1, c2, ref pos2))
                        {
                            startPoint = c1.PointAt(pos1);
                            endPoint = c2.PointAt(pos2);
                        }
                    }
                    else
                    {
                        Line line = c1 as Line;
                        ICurve curve = null;
                        if (line == null)
                        {
                            line = c2 as Line;
                            curve = c1;
                        }
                        else
                        {
                            curve = c2;
                        }
                        if (line != null)
                        {   // one of the curves is a line, the other doesnt share a plane with the line

                        }

                    }
                }
            }
            else if (fromHere is Face f1 && toHere is Face f2)
            {   // use Surfaces.ParallelDistance for most cases!!!
                if (!(f1.Surface is PlaneSurface) && f2.Surface is PlaneSurface)
                {   // first should be plane
                    Face tmp = f1;
                    f1 = f2;
                    f2 = tmp;
                    swapped = true;
                }
                if (f1.Surface is PlaneSurface ps1 && f2.Surface is PlaneSurface ps2)
                {
                    if (Precision.SameDirection(ps1.Normal, ps2.Normal, false))
                    {   // two parallel planes
                        GeoPoint m;
                        if (preferredPoint.IsValid) m = preferredPoint;
                        else m = new GeoPoint(ps1.Location, ps2.Location);
                        startPoint = ps1.PointAt(ps1.GetLineIntersection(m, ps1.Normal)[0]); // there must be a intersection
                        endPoint = ps2.PointAt(ps2.GetLineIntersection(m, ps2.Normal)[0]); // there must be a intersection
                    }
                    // non parallel surfaces don't work. How would the distance be defined?
                }
                else if (f1.Surface is PlaneSurface ps && f2.Surface is CylindricalSurface cy)
                {
                    if (Surfaces.ParallelDistance(f1.Surface, f1.Domain, f2.Surface, f2.Domain, preferredPoint, out GeoPoint2D uv1, out GeoPoint2D uv2))
                    {
                        startPoint = f1.Surface.PointAt(uv1);
                        endPoint = f2.Surface.PointAt(uv2);
                    }
                }
                else if (f1.Surface is CylindricalSurface cyl1 && f2.Surface is CylindricalSurface cyl2)
                {
                    if (cyl1.SameGeometry(f1.Domain, cyl2, f2.Domain, Precision.eps, out ModOp2D _))
                    {
                        // a diameter of the cylinder is expected
                        if (!preferredDirection.IsNullVector())
                        {

                        }
                        if (preferredPoint.IsValid)
                        {
                            GeoPoint pax = Geometry.DropPL(preferredPoint, cyl1.Location, cyl1.Axis);
                            GeoPoint2D[] pp = cyl1.GetLineIntersection(pax, preferredPoint - pax);
                            if (pp.Length == 2)
                            {
                                startPoint = cyl1.PointAt(pp[0]);
                                endPoint = cyl1.PointAt(pp[1]);
                            }
                        }
                    }
                }
                // other combinations with ISurface.GetExtremePositions, but there is some work to do....
            }
            else if (toHere is ICurve c && fromHere is GeoPoint p)
            {
                if (preferredDirection.IsValid() && !preferredDirection.IsNullVector())
                {
                    if (c is Line line)
                    {
                        if (Precision.IsPerpendicular(line.StartDirection, preferredDirection, false))
                        {
                            if (preferredPoint.IsValid) startPoint = Geometry.DropPL(preferredPoint, line.StartPoint, line.EndPoint);
                            else startPoint = Geometry.DropPL(p, line.StartPoint, line.EndPoint);
                            Plane pln = new Plane(p, preferredDirection);
                            endPoint = pln.FootPoint(startPoint);
                            degreeOfFreedom = line.StartDirection;
                        }
                    }
                    // other cases to implement
                }
            }
            else if (fromHere is Face f && toHere is GeoPoint pp)
            {
                if (preferredDirection.IsValid() && !preferredDirection.IsNullVector())
                {
                    if (f.Surface is PlaneSurface ps && Precision.SameDirection(ps.Normal, preferredDirection, false))
                    {
                        if (preferredPoint.IsValid)
                        {
                            startPoint = ps.Plane.FootPoint(preferredPoint);
                            Plane pln = new Plane(pp, preferredDirection);
                            endPoint = pln.FootPoint(startPoint);
                        }
                        else
                        {
                            startPoint = ps.Plane.FootPoint(pp);
                            endPoint = pp;
                        }
                    }
                }
            }
            else if (fromHere is Face ff && toHere is ICurve cc)
            {

            }
            if (swapped) return (endPoint, startPoint);
            else return (startPoint, endPoint);
        }

        internal static bool DistanceBetweenFaceAndEdge(Face face, GeoPoint faceTouchingPoint, Edge edge, GeoPoint edgeTouchingPoint, out GeoPoint2D onFace, out double onEdge)
        {
            onFace = GeoPoint2D.Invalid;
            onEdge = double.NaN;

            // TODO: many cases need to be implemented
            if (face.Surface is PlaneSurface ps)
            {   // distance to line or arc

            }
            else if (face.Surface is CylindricalSurface cyl)
            {
                if (edge.Curve3D is Line line)
                {
                    double dist = Geometry.DistLL(cyl.Location, cyl.Axis, line.StartPoint, line.StartDirection, out double par1, out double par2);
                    if (dist > cyl.RadiusX)
                    {   // we must be outside the cylinder, or are there valid other cases?
                        GeoPoint sp = cyl.Location + par1 * cyl.Axis;
                        GeoPoint ep = line.StartPoint + par2 * line.StartDirection;
                        GeoPoint2D[] ip2d = cyl.GetLineIntersection(ep, sp - ep);
                        onFace = ip2d.MinBy(p2d => cyl.PointAt(p2d) | faceTouchingPoint);
                        onEdge = par2;
                        return true;
                    }
                }
            }
            return false;
        }
    }
}
