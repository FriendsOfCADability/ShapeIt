using CADability.Attribute;
using CADability.Curve2D;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using CADability.Shapes;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using Wintellect.PowerCollections;
using MathNet.Numerics.LinearAlgebra.Factorization;
using System.Linq;
using MathNet.Numerics;
using CADability.Substitutes;

namespace CADability.GeoObject
{
    public class Surfaces
    {
        internal static ICurve Intersect(PlaneSurface surface1, BoundingRect bounds1, CylindricalSurface surface2, BoundingRect bounds2, List<GeoPoint> points)
        {
            IDualSurfaceCurve[] cvs = surface2.GetPlaneIntersection(surface1, bounds2.Left, bounds2.Right, bounds2.Bottom, bounds2.Top, 0.0);
            if (cvs.Length == 2 && cvs[0].Curve3D is Line l0 && cvs[1].Curve3D is Line l1)
            {   // two lines, find the closer one
                if ((l0 as ICurve).DistanceTo(points[0]) < (l1 as ICurve).DistanceTo(points[0])) return l0;
                else return l1;
            }
            for (int i = 0; i < cvs.Length; ++i)
            {
                ICurve c3d = cvs[i].Curve3D;
                double spar = c3d.PositionOf(points[0]);
                double epar = c3d.PositionOf(points[points.Count - 1]);
                if (c3d is Line)
                {   // eine von zwei Linien ist die falsche, ist aber egal hier
                    Line line = Line.Construct();
                    line.SetTwoPoints(points[0], points[points.Count - 1]);
                    return line;
                }
                else if (c3d is Ellipse)
                {   // richtiger Abschnitt der Ellipse finden, kann über 0 gehen
                    // es ist immer eine volle Ellipse, unabhängig von umin...vmax
                    // Startparameter ist immer 0.0
                    Ellipse elli = c3d as Ellipse;
                    elli.StartParameter = 0.0;
                    elli.SweepParameter = Math.PI * 2.0; // damit klare Verhältnisse vorliegen
                    elli.StartParameter = elli.ParameterOf(points[0]);
                    epar = elli.PositionOf(points[points.Count - 1]); // zwischen 0 und 1
                    elli.SweepParameter = epar * Math.PI * 2.0;
                    if (points.Count > 2)
                    {   // einfacher Fall, Zwischenpunkte prüfen
                        int j = points.Count / 2; // mittlerer Punkt
                        double pos = elli.PositionOf(points[j]);
                        if (pos < 0.0 || pos > 1.0)
                        {
                            elli.SweepParameter = elli.SweepParameter - 2.0 * Math.PI;
                        }
                        double dbg = elli.PositionOf(points[points.Count - 1]);
                    }
                    else
                    {   // kein Zwischenpunkt, eigentlich Lage in bounds2 überprüfen
                        GeoPoint2D tp = surface2.PositionOf(elli.PointAt(0.5));
                        SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref tp);
                        if (tp.x < bounds2.Left || tp.x > bounds2.Right) elli.SweepParameter = elli.SweepParameter - 2.0 * Math.PI;
                        //if (elli.SweepParameter > Math.PI)
                        //{
                        //    elli.SweepParameter = elli.SweepParameter - 2.0 * Math.PI;
                        //}
                    }
                    return elli;
                }
            }
            return null;
        }
        internal static ICurve Intersect(PlaneSurface surface1, BoundingRect bounds1, ToroidalSurface surface2, BoundingRect bounds2, List<GeoPoint> points)
        {
            IDualSurfaceCurve[] cvs = surface2.GetPlaneIntersection(surface1, bounds2.Left, bounds2.Right, bounds2.Bottom, bounds2.Top, 0.0);
            if (cvs.Length == 1) return cvs[0].Curve3D;
            if (cvs.Length > 1)
            {
                // the toroidal bounds are typically only a quarter of the full 0..2*PI square, so there should only be one valid solution inside
                for (int i = 0; i < cvs.Length; i++)
                {
                    if (bounds2.Contains(cvs[i].Curve2D1.PointAt(0.5))) return cvs[i].Curve3D;
                }
                return cvs[0].Curve3D; // no good constraint
            }
            return null;
        }
        /// <summary>
        /// Intersect two surfaces where we can calculate exact intersection curves without approximation. this should be implemented for plane, cylinder, cone, sphere and torus.
        /// The resulting intersection curves are either infinite or closed.
        /// </summary>
        /// <param name="surface1"></param>
        /// <param name="surface2"></param>
        /// <returns>null, if not implemented for the provided surfaces, otherwise the intersection curves, which also may be none (empty array)</returns>
        internal static ICurve[] Intersect(ISurface surface1, ISurface surface2)
        {   // this should be used for intersections, where no seed points can be calculated. It must be implemented for all combinations of canonical surfaces (plane, cylinder, cone, sphere, torus), 
            if (surface1 is PlaneSurface ps1 && surface2 is PlaneSurface ps2)
            {   // the simplest case
                if (ps1.Plane.Intersect(ps2.Plane, out GeoPoint loc, out GeoVector dir))
                {
                    return new ICurve[] { Line.TwoPoints(loc, loc + dir) };
                }
                else return new ICurve[0];
            }
            if (surface2 is PlaneSurface)
            {   // swap surfaces to need less checks
                ISurface tmp = surface1;
                surface1 = surface2;
                surface2 = tmp;
            }
            if (surface1 is PlaneSurface ps)
            {
                if (surface2 is ICylinder cy)
                {   // a plane and a cylinder
                    if (Precision.IsPerpendicular(cy.Axis.Direction, ps.Normal, false))
                    {   // two lines, tangential or no result
                        Plane lower = new Plane(cy.Axis.Location, cy.Axis.Direction);
                        GeoPoint2D sp2d = lower.Project(ps.Location);
                        GeoVector2D dir2d = lower.Project(ps.Normal).ToLeft();
                        GeoPoint2D[] ips = Geometry.IntersectLC(sp2d, dir2d, GeoPoint2D.Origin, cy.Radius);
                        ICurve[] res = new ICurve[ips.Length];
                        for (int i = 0; i < ips.Length; i++)
                        {
                            GeoPoint p = lower.ToGlobal(ips[i]);
                            res[i] = Line.TwoPoints(p, p + cy.Axis.Direction);
                        }
                        return res;
                    }
                    // otherwise we have a circle or an ellipse
                    GeoPoint cnt = ps.Plane.Intersect(cy.Axis.Location, cy.Axis.Direction);
                    Ellipse elli = Ellipse.Construct();
                    if (Precision.SameDirection(ps.Plane.Normal, cy.Axis.Direction, false))
                    {   // this is a perpendicular intersection, the result will be a circle
                        cy.Axis.Direction.ArbitraryNormals(out GeoVector dirx, out GeoVector diry);
                        dirx.Length = cy.Radius;
                        diry.Length = cy.Radius;
                        elli.SetEllipseArcCenterAxis(cnt, dirx, diry, 0, Math.PI * 2.0);
                        return new ICurve[] { elli };
                    }
                    else if (!Precision.IsPerpendicular(cy.Axis.Direction, ps.Plane.Normal, false))
                    {
                        GeoVector minAx = cy.Axis.Direction ^ ps.Plane.Normal;
                        minAx.Length = cy.Radius;
                        GeoPoint2D[] ips = surface2.GetLineIntersection(cnt, minAx ^ ps.Plane.Normal);
                        if (ips != null && ips.Length > 0)
                        {
                            GeoVector majAx = surface2.PointAt(ips[0]) - cnt;
                            elli.SetEllipseCenterAxis(cnt, majAx, minAx);
                            return new ICurve[] { elli };
                        }
                        else return null;
                    }
                }
                if (surface2 is ISphere sph)
                {
                    double d = ps.Plane.Distance(sph.Center);
                    if (d < sph.Radius)
                    {
                        double r = Math.Sqrt(sph.Radius * sph.Radius - d * d);
                        Ellipse elli = Ellipse.Construct();
                        elli.SetCirclePlaneCenterRadius(ps.Plane, ps.Plane.ToGlobal(ps.Plane.Project(sph.Center)), r);
                        return new ICurve[] { elli };
                    }
                    return null;
                }
                if (surface2 is ICone icn)
                {

                }
            }
            return null;
            // still to implement intersections of standard surfaces:
            // //plane - plane
            // //plane - cylinder
            // //plane - sphere
            // plane - cone
            // plane - torus
            // cylinder - cylinder
            // cylinder - sphere
            // cylinder - cone
            // cylinder - torus
            // sphere - sphere
            // sphere - cone
            // sphere - torus
            // cone - cone
            // cone - torus
            // torus - torus
            // the results may be lines, ellipses or InterpolatedDualSurfaceCurves (8 points should be enough)
        }
        internal static ICurve Intersect(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, List<GeoPoint> points)
        {   // es muss einen Schnitt geben, sonst wird das hier garnicht aufgerufen, schließlich gibt es ja auch schon Punkte
            if (surface1 is PlaneSurface && surface2 is PlaneSurface && points.Count == 2)
            {   // hier sind die beiden Punkte schon bekannt und richtig
                Line line = Line.Construct();
                line.SetTwoPoints(points[0], points[points.Count - 1]);
                return line;
            }
            if (surface1 is PlaneSurface && surface2 is CylindricalSurface)
            {
                ICurve res = Intersect(surface1 as PlaneSurface, bounds1, surface2 as CylindricalSurface, bounds2, points);
                if (res != null) return res;
            }
            if (surface2 is PlaneSurface && surface1 is CylindricalSurface)
            {
                ICurve res = Intersect(surface2 as PlaneSurface, bounds2, surface1 as CylindricalSurface, bounds1, points);
                if (res != null) return res;
            }
            if (surface1 is PlaneSurface && surface2 is ToroidalSurface)
            {
                ICurve res = Intersect(surface1 as PlaneSurface, bounds1, surface2 as ToroidalSurface, bounds2, points);
                if (res != null) return res;
            }
            if (surface2 is PlaneSurface && surface1 is ToroidalSurface)
            {
                ICurve res = Intersect(surface2 as PlaneSurface, bounds2, surface1 as ToroidalSurface, bounds1, points);
                if (res != null) return res;
            }
            if (surface1 is ISurfaceOfRevolution sr1)
            {
                if ((surface2 is ISurfaceOfRevolution sr2 && Precision.SameAxis(sr1.Axis, sr2.Axis)) || (surface2 is ISphere sph && Precision.IsPointOnAxis(sph.Center, sr1.Axis)))
                {
                    {   // two surfaces of revolution with the same axis: the result is a circle
                        Plane axisPlane = Plane.XYPlane; // to avoid unassigned
                        surface2.Intersect(sr1.Curve, bounds2, out GeoPoint[] ips, out GeoPoint2D[] uv, out double[] u);
                        List<IDualSurfaceCurve> ldsc = new List<IDualSurfaceCurve>();
                        for (int i = 0; i < ips.Length; i++)
                        {
                            if (!Precision.IsPointOnAxis(ips[i], sr1.Axis))
                            {
                                Plane perpToAxis = new Plane(ips[i], sr1.Axis.Direction);
                                ldsc.AddRange(surface1.GetPlaneIntersection(new PlaneSurface(perpToAxis), bounds1.Left, bounds1.Right, bounds1.Bottom, bounds1.Top, Precision.eps));
                                // these are intersections with surface2 perpendicular to the axis where the curve of surface1 intersects
                                // i.e. circles of intersection
                            }
                        }
                        IDualSurfaceCurve c = ldsc.MinByWithDefault(null, c => points.Sum(p => c.Curve3D.DistanceTo(p))); // closest curve to the provided points
                        if (c != null) return c.Curve3D;
                    }
                }
            }
            if (surface1 is ISphere && surface2 is ISurfaceOfRevolution)
            {
                return Intersect(surface2, bounds2, surface1, bounds1, points);
            }
            {
                ICurve[] crvs = surface1.Intersect(bounds1, surface2, bounds2);
                for (int i = 0; i < crvs.Length; i++)
                {
                    double dsum = 0.0;
                    for (int j = 0; j < points.Count; j++)
                    {
                        dsum += crvs[i].DistanceTo(points[j]);
                    }
                    if (dsum < points.Count * Precision.eps) return crvs[i];
                }
            }
            // kein bekannter Fall. Hier ist es aber gefährlich wenn wir tangential sind,
            // denn dann funktioniert InterpolatedDualSurfaceCurve sehr schlecht, vor allem die Richtung ger Kurve
            // wird oft falsch
            bool tangential = false;
            for (int i = 0; i < points.Count; i++)
            {
                GeoPoint2D uv1 = surface1.PositionOf(points[i]);
                GeoPoint2D uv2 = surface2.PositionOf(points[i]);
                GeoVector n1 = surface1.GetNormal(uv1);
                GeoVector n2 = surface2.GetNormal(uv2);
                GeoVector z = n1.Normalized ^ n2.Normalized;
                if (z.Length < 0.1) // auf 0.1 reduziert, da immer noch Konvergenzprobleme
                {
                    tangential = true; // wenn nur an einer Stelle tangential, dann tangential. Die Kurven werden sonst zu schlecht!
                    break;
                }
            }
            if (tangential)
            {
                ICurve res = surface1.Intersect(bounds1, surface2, bounds2, points[0]);
                if (res != null)
                {
                    if (points.Count == 2)
                    {   // these are supposed to be start- and endpoint of the curve
                        // better return all of the curve, used in this way by Parametrics
                        //if ((res.StartPoint | points[0]) + (res.EndPoint | points[1]) > (res.StartPoint | points[1]) + (res.EndPoint | points[0])) res.Reverse();
                        //// maybe we have to trimm here
                        //res.StartPoint = points[0];
                        //res.EndPoint = points[1];
                    }
                    return res;
                }
                return null; // tangentiale Flächen können keine dualsurfacecurve haben, das konvergiert nicht
            }
            {
                InterpolatedDualSurfaceCurve idsc = new InterpolatedDualSurfaceCurve(surface1, bounds1, surface2, bounds2, points);
                return idsc;
            }
            //return null;
        }
        internal static IDualSurfaceCurve[] TestTangentialIntersections(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, IList<GeoPoint> points, IList<GeoPoint2D> uvon1, IList<GeoPoint2D> uvon2)
        {
            bool isTangential = true;
            for (int i = 0; i < points.Count; i++)
            {
                GeoVector n1 = surface1.GetNormal(uvon1[i]);
                GeoVector n2 = surface2.GetNormal(uvon2[i]);
                if (!Precision.SameDirection(n1, n2, false))
                {
                    isTangential = false;
                    break;
                }
            }
            if (isTangential)
            {
                if (surface1 is SweptCircle sc1 && (Math.Abs(Math.Abs(surface2.GetDistance(sc1.Spine.StartPoint)) - sc1.Radius) < Precision.eps)
                    && (Math.Abs(Math.Abs(surface2.GetDistance(sc1.Spine.EndPoint)) - sc1.Radius) < Precision.eps)
                    && (Math.Abs(Math.Abs(surface2.GetDistance(sc1.Spine.PointAt(0.5))) - sc1.Radius) < Precision.eps))
                {   // the spine seems to be parallel to surface2, so we have a tangential intersection along the spine
                    ICurve2D crv2d1 = surface1.GetProjectedCurve(sc1.Spine, Precision.eps);
                    ICurve2D crv2d2 = surface2.GetProjectedCurve(sc1.Spine, Precision.eps);
                    ICurve crv = surface2.Make3dCurve(surface2.GetProjectedCurve(sc1.Spine, Precision.eps));
                    return new IDualSurfaceCurve[] { new DualSurfaceCurve(crv, surface1, crv2d1, surface2, crv2d2) };
                }
                if (surface2 is SweptCircle sc2 && (Math.Abs(Math.Abs(surface1.GetDistance(sc2.Spine.StartPoint)) - sc2.Radius) < Precision.eps)
                    && (Math.Abs(Math.Abs(surface1.GetDistance(sc2.Spine.EndPoint)) - sc2.Radius) < Precision.eps)
                    && (Math.Abs(Math.Abs(surface1.GetDistance(sc2.Spine.PointAt(0.5))) - sc2.Radius) < Precision.eps))
                {   // the spine seems to be parallel to surface2, so we have a tangential intersection along the spine
                    ICurve2D crv2d1 = surface2.GetProjectedCurve(sc2.Spine, Precision.eps);
                    ICurve2D crv2d2 = surface1.GetProjectedCurve(sc2.Spine, Precision.eps);
                    ICurve crv = surface1.Make3dCurve(surface1.GetProjectedCurve(sc2.Spine, Precision.eps));
                    return new IDualSurfaceCurve[] { new DualSurfaceCurve(crv, surface1, crv2d1, surface2, crv2d2) };
                }
            }
            return null;
        }
        internal static bool Intersect(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, List<GeoPoint> points,
            out ICurve[] crvs3d, out ICurve2D[] crvsOnSurface1, out ICurve2D[] crvsOnSurface2, out double[,] params3d, out double[,] params2dsurf1, out double[,] params2dsurf2,
            out GeoPoint2D[] paramsuvsurf1, out GeoPoint2D[] paramsuvsurf2, double precision)
        {   // vor allem für BRepOperation.
            // es dient z.Z. vor allem dafür, einfache Schnitte schnell zu berechnen und das Ergebnis in 3d und 2d mit allen u und uv Werten für die gegebenen Punkte zu liefern,
            // damit diese nicht hinterher mehrfach neu berechnet werden müssen.
            // ein Schnitt zweier surfaces kann auch mehrere Schnittlinien liefern (vor allem bei NURBS, aber auch Zylinder/Ebene, Zylinder Kugel u.s.w.)
            // Die Punkte kommen meist von mehreren Edge/Face Schnitten

            // zuordnen der Punkte auf die Kurve(n)
            List<ICurve> lcrvs3d = new List<ICurve>(); // Ergebnisse als Listen
            List<ICurve2D> lcrvsOnSurface1 = new List<ICurve2D>();
            List<ICurve2D> lcrvsOnSurface2 = new List<ICurve2D>();
            paramsuvsurf1 = new GeoPoint2D[points.Count];
            paramsuvsurf2 = new GeoPoint2D[points.Count];
            for (int j = 0; j < points.Count; j++)
            {
                paramsuvsurf1[j] = surface1.PositionOf(points[j]);
                paramsuvsurf2[j] = surface2.PositionOf(points[j]);
                SurfaceHelper.AdjustPeriodic(surface1, bounds1, ref paramsuvsurf1[j]);
                SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref paramsuvsurf2[j]);
                bounds1.MinMax(paramsuvsurf1[j]);
                bounds2.MinMax(paramsuvsurf2[j]);
            }
            // maybe we are tangential here, special case: a swept circle where the spine is on an offset surface
            //IDualSurfaceCurve[] dbg = TestTangentialIntersections(surface1, bounds1, surface2, bounds2, points, paramsuvsurf1, paramsuvsurf2);
            IDualSurfaceCurve[] dscs = surface1.GetDualSurfaceCurves(bounds1, surface2, bounds2, points, null);

            if (points.Count > paramsuvsurf1.Length)
            {   // there were points added by GetDualSurfaceCurves
                paramsuvsurf1 = new GeoPoint2D[points.Count];
                paramsuvsurf2 = new GeoPoint2D[points.Count];
                for (int j = 0; j < points.Count; j++)
                {
                    paramsuvsurf1[j] = surface1.PositionOf(points[j]);
                    paramsuvsurf2[j] = surface2.PositionOf(points[j]);
                    SurfaceHelper.AdjustPeriodic(surface1, bounds1, ref paramsuvsurf1[j]);
                    SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref paramsuvsurf2[j]);
                }
            }
            // if there are closed curves in the result we try to split them
            List<IDualSurfaceCurve> brokenClosedCurves = new List<IDualSurfaceCurve>();
            List<int> toRemove = new List<int>();
            for (int i = 0; i < dscs.Length; i++)
            {

                if (dscs[i].Curve3D.IsClosed)
                {
                    List<GeoPoint> pointsOnCurve = new List<GeoPoint>();
                    for (int j = 0; j < points.Count; j++)
                    {
                        if (dscs[i].Curve3D.DistanceTo(points[j]) < precision)
                        {
                            pointsOnCurve.Add(points[j]);
                        }
                    }
                    if (pointsOnCurve.Count > 1)
                    {
                        ICurve[] parts3D = dscs[i].Curve3D.Split(dscs[i].Curve3D.PositionOf(pointsOnCurve[0]), dscs[i].Curve3D.PositionOf(pointsOnCurve[1]));
                        if (parts3D.Length == 2)
                        {
                            brokenClosedCurves.Add(new DualSurfaceCurve(parts3D[0], surface1, surface1.GetProjectedCurve(parts3D[0], precision), surface2, surface2.GetProjectedCurve(parts3D[0], precision)));
                            brokenClosedCurves.Add(new DualSurfaceCurve(parts3D[1], surface1, surface1.GetProjectedCurve(parts3D[1], precision), surface2, surface2.GetProjectedCurve(parts3D[1], precision)));
                            toRemove.Add(i);
                        }
                    }
                }
            }
            if (brokenClosedCurves.Count > 0)
            {
                List<IDualSurfaceCurve> res = new List<IDualSurfaceCurve>(dscs);
                for (int i = toRemove.Count - 1; i >= 0; --i)
                {
                    res.RemoveAt(toRemove[i]);
                }
                res.AddRange(brokenClosedCurves);
                dscs = res.ToArray();
            }
            params3d = new double[dscs.Length, points.Count];
            params2dsurf1 = new double[dscs.Length, points.Count];
            params2dsurf2 = new double[dscs.Length, points.Count];
            for (int i = 0; i < dscs.Length; i++)
            {
                ICurve cv = dscs[i].Curve3D;
                lcrvs3d.Add(cv);
                ICurve2D cvons1 = dscs[i].Curve2D1;
                ICurve2D cvons2 = dscs[i].Curve2D2;
                SurfaceHelper.AdjustPeriodic(surface1, bounds1, cvons1);
                SurfaceHelper.AdjustPeriodic(surface2, bounds2, cvons2);
                lcrvsOnSurface1.Add(cvons1);
                lcrvsOnSurface2.Add(cvons2);
                for (int j = 0; j < points.Count; j++)
                {
                    double d = cv.DistanceTo(points[j]);
                    if (d < 10 * precision) // auf die nächstgelegene Kurve mappen
                    {
                        params3d[i, j] = cv.PositionOf(points[j]);
                        params2dsurf1[i, j] = cvons1.PositionOf(paramsuvsurf1[j]);
                        params2dsurf2[i, j] = cvons2.PositionOf(paramsuvsurf2[j]);
                        if (params2dsurf1[i, j] == double.MinValue || params2dsurf2[i, j] == double.MinValue)
                        {
                            params3d[i, j] = double.MinValue; // indication: the point "j" doesn't belong to curve i
                            params2dsurf1[i, j] = double.MinValue;
                            params2dsurf2[i, j] = double.MinValue;
                        }
                    }
                    else
                    {
                        params3d[i, j] = double.MinValue; // indication: the point "j" doesn't belong to curve i
                        params2dsurf1[i, j] = double.MinValue;
                        params2dsurf2[i, j] = double.MinValue;
                    }
                }
            }
            crvs3d = lcrvs3d.ToArray();
            crvsOnSurface1 = lcrvsOnSurface1.ToArray();
            crvsOnSurface2 = lcrvsOnSurface2.ToArray();
            return crvs3d.Length > 0;
        }
        internal static ICurve Intersect(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, GeoPoint seed)
        {
            // zwei beliebige Flächen sollen geschnitten werden, beide mit endlichen Intervall. Bei periodischen ist dieses intervall jeweils kleiner als die Periode.
            // Es gibt ausgehend von "seed" nur maximal eine Kurve als Ergebnis.
            // Dieses Ergebnis wird von Intersect verwendet, um einen teil der gegebenen punkte abzuarbeiten und wenn nicht alle verbraucht, nochmal mit einem anderen punkt aufzurufen
            if (surface1 is PlaneSurface)
            {
                if (surface2 is PlaneSurface)
                {   // Ebene/Ebene, noch implementieren

                }
                if (surface2 is CylindricalSurface)
                {
                    CylindricalSurface cyl = (surface2 as CylindricalSurface);
                    ICurve[] icvs = cyl.Intersect(bounds2, surface1, bounds1);
                    for (int i = 0; i < icvs.Length; i++)
                    {
                        if (icvs[i].DistanceTo(seed) < Precision.eps) return icvs[i];
                    }
                    IDualSurfaceCurve[] cvs = cyl.GetPlaneIntersection(surface1 as PlaneSurface, bounds2.Left, bounds2.Right, bounds2.Bottom, bounds2.Top, Precision.eps);
                    if (cvs != null)
                    {
                        if (cvs.Length == 2)
                        {   // zwei Linien, hier mit bounds1 trimmen
                            int ind = 1;    // welche der beiden Linien ist gemeint?
                            if (cvs[0].Curve3D.DistanceTo(seed) < cvs[1].Curve3D.DistanceTo(seed)) ind = 0;
                            GeoPoint2D sp = surface1.PositionOf(cvs[ind].Curve3D.StartPoint);
                            GeoPoint2D ep = surface1.PositionOf(cvs[ind].Curve3D.EndPoint);
                            GeoVector2D dir = ep - sp;
                            double pmin = double.MinValue, pmax = double.MaxValue;
                            if (Math.Abs(dir.x) > 1e-10)
                            {
                                double p = (bounds1.Left - sp.x) / dir.x;
                                if (p > 0 && p < pmax) pmax = p;
                                if (p <= 0 && p > pmin) pmin = p;
                                p = (bounds1.Right - sp.x) / dir.x;
                                if (p > 0 && p < pmax) pmax = p;
                                if (p <= 0 && p > pmin) pmin = p;
                            }
                            if (Math.Abs(dir.y) > 1e-10)
                            {
                                double p = (bounds1.Bottom - sp.y) / dir.y;
                                if (p > 0 && p < pmax) pmax = p;
                                if (p <= 0 && p > pmin) pmin = p;
                                p = (bounds1.Top - sp.y) / dir.y;
                                if (p > 0 && p < pmax) pmax = p;
                                if (p <= 0 && p > pmin) pmin = p;
                            }

                            if (pmin != double.MinValue && pmax != double.MaxValue)
                            {
                                Line res = Line.Construct();
                                res.SetTwoPoints(surface1.PointAt(sp + pmin * dir), surface1.PointAt(sp + pmax * dir));
                                if (res.Length > 0) return res; // könnte man auch noch mit dem Zylinder trimmen, ist aber denke ich nicht nötig
                            }
                            return null;
                        }
                        else
                        {
                            if (cvs[0].Curve3D is Ellipse)
                            {
                                // genau eine Ellipse
                                Ellipse elli = cvs[0].Curve3D as Ellipse;
                                GeoPoint p1 = elli.Plane.Intersect(surface2.PointAt(bounds2.GetLowerLeft()), cyl.Axis);
                                GeoPoint p2 = elli.Plane.Intersect(surface2.PointAt(bounds2.GetLowerRight()), cyl.Axis);
                                GeoPoint p3 = elli.Plane.Intersect(surface2.PointAt(bounds2.GetLowerMiddle()), cyl.Axis); // als Testpunkt, welcher Teil geliefert werden soll
                                double par1 = elli.PositionOf(p1);
                                double par2 = elli.PositionOf(p2);
                                ICurve[] parts = elli.Split(par1, par2);
                                if (parts.Length == 2)
                                {   // trimmen an der Ebene wohl nicht nötig
                                    if (parts[0].DistanceTo(p3) < parts[1].DistanceTo(p3)) return parts[0];
                                    else return parts[1];
                                }
                            }
                            else if (cvs[0].Curve3D is Line)
                            {
                                GeoPoint2D sp = surface1.PositionOf(cvs[0].Curve3D.StartPoint);
                                GeoPoint2D ep = surface1.PositionOf(cvs[0].Curve3D.EndPoint);
                                GeoVector2D dir = ep - sp;
                                double pmin = double.MinValue, pmax = double.MaxValue;
                                if (Math.Abs(dir.x) > 1e-10)
                                {
                                    double p = (bounds1.Left - sp.x) / dir.x;
                                    if (p > 0 && p < pmax) pmax = p;
                                    if (p <= 0 && p > pmin) pmin = p;
                                    p = (bounds1.Right - sp.x) / dir.x;
                                    if (p > 0 && p < pmax) pmax = p;
                                    if (p <= 0 && p > pmin) pmin = p;
                                }
                                if (Math.Abs(dir.y) > 1e-10)
                                {
                                    double p = (bounds1.Bottom - sp.y) / dir.y;
                                    if (p > 0 && p < pmax) pmax = p;
                                    if (p <= 0 && p > pmin) pmin = p;
                                    p = (bounds1.Top - sp.y) / dir.y;
                                    if (p > 0 && p < pmax) pmax = p;
                                    if (p <= 0 && p > pmin) pmin = p;
                                }
                                if (pmin != double.MinValue && pmax != double.MaxValue)
                                {
                                    Line res = Line.Construct();
                                    res.SetTwoPoints(surface1.PointAt(sp + pmin * dir), surface1.PointAt(sp + pmax * dir));
                                    if (res.Length > 0) return res; // könnte man auch noch mit dem Zylinder trimmen, ist aber denke ich nicht nötig
                                }
                                return null;
                            }

                        }
                    }

                }
                else if (surface2 is ConicalSurface)
                {
                    ConicalSurface cnl = (surface2 as ConicalSurface);
                    IDualSurfaceCurve[] cvs = cnl.GetPlaneIntersection(surface1 as PlaneSurface, bounds1.Left, bounds1.Right, bounds1.Bottom, bounds1.Top, Precision.eps);
                    if (cvs != null)
                    {
                        if (cvs.Length == 2 && cvs[0].Curve3D is Line && cvs[1].Curve3D is Line)
                        {   // zwei Linien, hier mit bounds1 trimmen
                            int ind = 1;    // welche der beiden Linien ist gemeint?
                            if (cvs[0].Curve3D.DistanceTo(seed) < cvs[1].Curve3D.DistanceTo(seed)) ind = 0;
                            GeoPoint2D sp = surface1.PositionOf(cvs[ind].Curve3D.StartPoint);
                            GeoPoint2D ep = surface1.PositionOf(cvs[ind].Curve3D.EndPoint);
                            GeoVector2D dir = ep - sp;
                            double pmin = double.MinValue, pmax = double.MaxValue;
                            if (Math.Abs(dir.x) > 1e-10)
                            {
                                double p = (bounds1.Left - sp.x) / dir.x;
                                if (p > 0 && p < pmax) pmax = p;
                                if (p < 0 && p > pmin) pmin = p;
                                p = (bounds1.Right - sp.x) / dir.x;
                                if (p > 0 && p < pmax) pmax = p;
                                if (p < 0 && p > pmin) pmin = p;
                            }
                            if (Math.Abs(dir.y) > 1e-10)
                            {
                                double p = (bounds1.Bottom - sp.y) / dir.y;
                                if (p > 0 && p < pmax) pmax = p;
                                if (p < 0 && p > pmin) pmin = p;
                                p = (bounds1.Top - sp.y) / dir.y;
                                if (p > 0 && p < pmax) pmax = p;
                                if (p < 0 && p > pmin) pmin = p;
                            }
                            Line res = Line.Construct();
                            res.SetTwoPoints(surface1.PointAt(sp + pmin * dir), surface1.PointAt(sp + pmax * dir));
                            return res; // könnte man auch noch mit dem Zylinder trimmen, ist aber denke ich nicht nötig
                        }
                        else if (cvs.Length == 1 && cvs[0].Curve3D is Ellipse)
                        {
                            // genau eine Ellipse
                            Ellipse elli = cvs[0].Curve3D as Ellipse;
                            Line ln1 = cnl.FixedU(bounds2.Left, bounds2.Bottom, bounds2.Top) as Line;
                            Line ln2 = cnl.FixedU(bounds2.Right, bounds2.Bottom, bounds2.Top) as Line;
                            Line ln3 = cnl.FixedU((bounds2.Left + bounds2.Right) / 2.0, bounds2.Bottom, bounds2.Top) as Line;
                            GeoPoint p1 = elli.Plane.Intersect(ln1.StartPoint, ln1.StartDirection);
                            GeoPoint p2 = elli.Plane.Intersect(ln2.StartPoint, ln2.StartDirection);
                            GeoPoint p3 = elli.Plane.Intersect(ln3.StartPoint, ln3.StartDirection); // als Testpunkt, welcher Teil geliefert werden soll
                            double par1 = elli.PositionOf(p1);
                            double par2 = elli.PositionOf(p2);
                            ICurve[] parts = elli.Split(par1, par2);
                            if (parts.Length == 2)
                            {   // trimmen an der Ebene wohl nicht nötig
                                if (parts[0].DistanceTo(p3) < parts[1].DistanceTo(p3)) return parts[0];
                                else return parts[1];
                            }
                        }
                        else
                        {
                            // Hyperbeln, Parabel auf jeder Hälfte des Doppelkegels eine
                            // der Kegel ist nie ein Doppelkegel, deshalb muss immer die Kurve geliefert werden, deren Koordinaten in der Einheitsform positiv sind
                            // bounds ist immer in y nur positiv oder nur negativ
                            for (int i = 0; i < cvs.Length; i++)
                            {
                                if (Math.Sign(cnl.PositionOf(cvs[i].Curve3D.PointAt(0.5)).y) == Math.Sign(bounds2.Bottom + bounds2.Top)) return cvs[i].Curve3D;
                            }
                        }
                    }

                }
            }
            else if (surface2 is PlaneSurface)
            {
                return Intersect(surface2, bounds2, surface1, bounds1, seed);
            }
            // allgemeine Lösung (hier noch die Methode mit seed in ISurface einführen!)
            ICurve[] ic = surface1.Intersect(bounds1, surface2, bounds2);
            for (int i = 0; i < ic.Length; i++)
            {
                if (ic[i].DistanceTo(seed) < Precision.eps) return ic[i];
            }
            return null;
        }
        private static ICurve BestTangentialCurve(ISurface surface1, ISurface surface2, List<GeoPoint> points)
        {
            return null;
        }
        internal static bool PlaneIntersection(Plane pln, ISurface surface1, ISurface surface2, out GeoPoint[] ip, out GeoPoint2D[] uv1, out GeoPoint2D[] uv2)
        {
            List<GeoPoint> lip = new List<CADability.GeoPoint>();
            List<GeoPoint2D> luv1 = new List<CADability.GeoPoint2D>();
            List<GeoPoint2D> luv2 = new List<CADability.GeoPoint2D>();

            // wird oft aufgerufen mit 2 mal Zylinder/Kegel/Kugel/Ebene 
            ICurve2D[] c2d1 = CurvesOnPlane(pln, surface1);
            ICurve2D[] c2d2 = CurvesOnPlane(pln, surface2);
            for (int i = 0; i < c2d1.Length; i++)
            {
                for (int j = 0; j < c2d2.Length; j++)
                {
                    GeoPoint2DWithParameter[] ip2d = c2d1[i].Intersect(c2d2[j]);
                    for (int k = 0; k < ip2d.Length; k++)
                    {
                        GeoPoint ipk = pln.ToGlobal(ip2d[k].p);
                        lip.Add(ipk);
                        luv1.Add(surface1.PositionOf(ipk));
                        luv2.Add(surface2.PositionOf(ipk));
                    }
                }
            }

            ip = lip.ToArray();
            uv1 = luv1.ToArray();
            uv2 = luv2.ToArray();
            return ip.Length > 0;
        }
        private static ICurve2D[] CurvesOnPlane(Plane pln, ISurface surface)
        {
            double umin, umax, vmin, vmax;
            surface.GetNaturalBounds(out umin, out umax, out vmin, out vmax);
            if (surface is ISurfacePlaneIntersection) // noch für Ebene, Kugel, Kegel implementieren
            {
                return (surface as ISurfacePlaneIntersection).GetPlaneIntersection(pln, umin, umax, vmin, vmax);
            }
            // bei folgendem wird überflüssigerweise auch die Kurve auf surface berechnet, die oft ein BSpline ist
            IDualSurfaceCurve[] dscs1 = surface.GetPlaneIntersection(new GeoObject.PlaneSurface(pln), umin, umax, vmin, vmax, Precision.eps);
            ICurve2D[] res = new ICurve2D[dscs1.Length];
            for (int i = 0; i < res.Length; i++)
            {
                res[i] = dscs1[i].Curve2D2;
            }
            return res;
        }
        /// <summary>
        /// Try to find a single intersection point of three surfaces close to a provided seed point <paramref name="ip"/>.
        /// </summary>
        /// <param name="surface1"></param>
        /// <param name="bounds1"></param>
        /// <param name="surface2"></param>
        /// <param name="bounds2"></param>
        /// <param name="surface3"></param>
        /// <param name="bounds3"></param>
        /// <param name="ip"></param>
        /// <param name="uv1"></param>
        /// <param name="uv2"></param>
        /// <param name="uv3"></param>
        /// <returns></returns>
        public static bool IntersectThreeSurfaces(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, ISurface surface3, BoundingRect bounds3, ref GeoPoint ip,
            out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint2D uv3)
        {
#if DEBUG
            //DebuggerContainer dc = new DebuggerContainer();
            //dc.Add(Face.MakeFace(surface1, bounds1), 1);
            //dc.Add(Face.MakeFace(surface2, bounds2), 2);
            //dc.Add(Face.MakeFace(surface3, bounds3), 3);
#endif
            ISurface[] surfaces = new ISurface[] { surface1, surface2, surface3 };
            BoundingRect[] bounds = new BoundingRect[] { bounds1, bounds2, bounds3 };
            // if two of the surfaces are planes, use the surface/line intersection
            bool simplePlaneIntersection = true;
            int pln1 = -1, pln2 = -1, other = -1;
            for (int i = 0; i < 3; i++)
            {
                if (surfaces[i] is PlaneSurface)
                {
                    if (pln1 < 0) pln1 = i;
                    else if (pln2 < 0) pln2 = i;
                    else other = i;
                }
                else
                {
                    other = i;
                }
                if (!(surfaces[i] is PlaneSurface || surfaces[i] is CylindricalSurface || surfaces[i] is ConicalSurface || surfaces[i] is SphericalSurface)) simplePlaneIntersection = false;
            }
            if (pln2 >= 0)
            {
                // two planar surfaces
                IDualSurfaceCurve[] isl = surfaces[pln1].GetPlaneIntersection(surfaces[pln2] as PlaneSurface, bounds[pln1].Left, bounds[pln1].Right, bounds[pln1].Bottom, bounds[pln1].Top, 0.0);
                if (isl.Length == 1 && isl[0].Curve3D is Line line)
                {
                    GeoPoint2D[] ips = surfaces[other].GetLineIntersection(line.StartPoint, line.EndPoint - line.StartPoint);
                    if (ips.Length > 0)
                    {
                        GeoPoint tp = ip;
                        GeoPoint2D uvo = ips.MinBy(p => surfaces[other].PointAt(p) | tp);
                        ip = surfaces[other].PointAt(uvo);
                        uv1 = surface1.PositionOf(ip);
                        uv2 = surface2.PositionOf(ip);
                        uv3 = surface3.PositionOf(ip);
                        return true;
                    }
                }
            }
            // if one of the surfaces is a planar surface handle it on the plans 2d system
            List<GeoPoint> candidates = new List<GeoPoint>();
            if (simplePlaneIntersection)
            {   // for surfaces, which provide a simple plane intersection, the following is good. If not, better use NewtonIntersect
                for (int i = 0; i < 3; i++)
                {
                    if (surfaces[i] is PlaneSurface pls)
                    {
                        int o1 = (i + 1) % 3;
                        int o2 = (i + 2) % 3;
                        IDualSurfaceCurve[] dsc1 = surfaces[o1].GetPlaneIntersection(pls, bounds[o1].Left, bounds[o1].Right, bounds[o1].Bottom, bounds[o1].Top, 0.0);
                        IDualSurfaceCurve[] dsc2 = surfaces[o2].GetPlaneIntersection(pls, bounds[o2].Left, bounds[o2].Right, bounds[o2].Bottom, bounds[o2].Top, 0.0);
                        if (dsc1 != null && dsc2 != null)
                        {
                            for (int j = 0; j < dsc1.Length; ++j)
                            {
                                for (int k = 0; k < dsc2.Length; k++)
                                {
                                    ICurve2D c1, c2;
                                    if (dsc1[j].Surface1 == pls) c1 = dsc1[j].Curve2D1;
                                    else c1 = dsc1[j].Curve2D2;
                                    if (dsc2[k].Surface1 == pls) c2 = dsc2[k].Curve2D1;
                                    else c2 = dsc2[k].Curve2D2;
                                    GeoPoint2DWithParameter[] ips2d = c1.Intersect(c2);
                                    for (int l = 0; l < ips2d.Length; l++)
                                    {
                                        candidates.Add(pls.PointAt(ips2d[l].p));
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (candidates.Count > 0)
            {   // here we could check, which of the candidates is more precise
                GeoPoint lip = ip;
                ip = Hlp.GetClosest(candidates, p => p | lip);
                uv1 = surface1.PositionOf(ip);
                uv2 = surface2.PositionOf(ip);
                uv3 = surface3.PositionOf(ip);
                return true;
            }
            uv1 = surface1.PositionOf(ip);
            uv2 = surface2.PositionOf(ip);
            uv3 = surface3.PositionOf(ip);
            if (BoxedSurfaceExtension.SurfacesIntersectionLM(surface1, surface2, surface3, ref uv1, ref uv2, ref uv3, ref ip)) return true;
            return NewtonIntersect(surface1, bounds1, surface2, bounds2, surface3, bounds3, ref ip, out uv1, out uv2, out uv3);

        }
        internal static bool NewtonIntersect(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, ISurface surface3, BoundingRect bounds3, ref GeoPoint ip,
            out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint2D uv3)
        {
            // ausgehend vom Anfangspunkt wird versucht ein Schnittpunkt mit Newton zu finden. Konvergiert Newton nicht
            // innerhalb der bounds, wird false geliefert. Der Aufrufer muss dann ggf. unterteilen und weiter suchen
            // Verwendet wird das z.Z. von BRepOperation
            // zuerst noch die einfachen Fälle aussortieren
            uv1 = surface1.PositionOf(ip);
            uv2 = surface2.PositionOf(ip);
            uv3 = surface3.PositionOf(ip);
            // double serr = NewtonLMIntersection(surface1, surface2, surface3, ref uv1, ref uv2, ref uv3);
            GeoPoint p1 = surface1.PointAt(uv1);
            GeoPoint p2 = surface2.PointAt(uv2);
            GeoPoint p3 = surface3.PointAt(uv3);
            GeoVector u1 = surface1.UDirection(uv1);
            GeoVector v1 = surface1.VDirection(uv1);
            GeoVector u2 = surface2.UDirection(uv2);
            GeoVector v2 = surface2.VDirection(uv2);
            GeoVector u3 = surface3.UDirection(uv3);
            GeoVector v3 = surface3.VDirection(uv3);
            double error = (p1 | p2) + (p2 | p3) + (p3 | p1);
            while (error > 0)
            {   // Newton, allgeimer Schnitt von 3 Ebenen mit u/v Lösungen
                // Mit Normalengleichungen der Ebene braucht man nur 3 Gleichungen, muss aber dann noch die 3 u/v Punkte bestimmen
                // Wenn die Flächen recht tangential sind, ist die Fehlerabfrage schlecht, da man in uv noch sehr daneben liegen kann.
                // BRep-Operationen brauchen aber genaue Werte. Deshalb wird hier bis zu "Rauschen" konvergiert, durch die error/2 Bedingung
                // aber maximal 48 mal
                Matrix m = DenseMatrix.OfArray(new double[6, 6] { { u1.x, v1.x, -u2.x, -v2.x, 0, 0 }, { u1.y, v1.y, -u2.y, -v2.y, 0, 0 }, { u1.z, v1.z, -u2.z, -v2.z, 0, 0 }, { 0, 0, -u2.x, -v2.x, u3.x, v3.x }, { 0, 0, -u2.y, -v2.y, u3.y, v3.y }, { 0, 0, -u2.z, -v2.z, u3.z, v3.z } });
                try
                {
                    Vector x = (Vector)m.Solve(new DenseVector(new double[] { p2.x - p1.x, p2.y - p1.y, p2.z - p1.z, p2.x - p3.x, p2.y - p3.y, p2.z - p3.z }));
                    if (!x.IsValid())
                    {
                        if (error < Precision.eps) break; // geht wohl nicht besser
                        else return false;
                    }
                    uv1.x += x[0];
                    uv1.y += x[1];
                    uv2.x += x[2];
                    uv2.y += x[3];
                    uv3.x += x[4];
                    uv3.y += x[5];
                    p1 = surface1.PointAt(uv1);
                    p2 = surface2.PointAt(uv2);
                    p3 = surface3.PointAt(uv3);
                    double e = (p1 | p2) + (p2 | p3) + (p3 | p1);
                    if (e > error / 2.0)
                    {   // konvergiert nicht gut
                        if (e < Precision.eps) break; // wir sind ja schon gut
                        else return false;
                    }
                    error = e;
                    u1 = surface1.UDirection(uv1);
                    v1 = surface1.VDirection(uv1);
                    u2 = surface2.UDirection(uv2);
                    v2 = surface2.VDirection(uv2);
                    u3 = surface3.UDirection(uv3);
                    v3 = surface3.VDirection(uv3);
                }
                catch (ApplicationException)
                {
                    return false;
                }
            }
            ip = new GeoPoint(p1, p2, p3);
            return true;
        }

        /// <summary>
        /// Newton Levenberg-Marquardt intersection of three surfaces with provided start positions. The result is in <paramref name="uv1"/>, 
        /// <paramref name="uv1"/> and <paramref name="uv3"/>.
        /// </summary>
        /// <param name="surface1">surface 1</param>
        /// <param name="surface2">surface 2</param>
        /// <param name="surface3">surface 3</param>
        /// <param name="uv1">startpoint on surface 1</param>
        /// <param name="uv2">startpoint on surface 2</param>
        /// <param name="uv3">startpoint on surface 3</param>
        /// <returns>double.MaxValue if not converged, square of error otherwise</returns>
        public static double NewtonLMIntersection(ISurface surface1, ISurface surface2, ISurface surface3, ref GeoPoint2D uv1, ref GeoPoint2D uv2, ref GeoPoint2D uv3)
        {
            const int dim = 6;

            // Dummy-Vektoren für observedX, observedY und Gewichte
            var dummyX = Vector<double>.Build.Dense(dim, 0.0);
            var observedY = Vector<double>.Build.Dense(dim, 0.0);
            var weights = Vector<double>.Build.Dense(dim, 1.0);

            // Modell: liefert den Residualvektor r(p) mit p = [u1,v1,u2,v2,u3,v3]
            Func<Vector<double>, Vector<double>, Vector<double>> model = (p, x) =>
            {
                // Parameter entpacken
                double u1 = p[0], v1 = p[1],
                       u2 = p[2], v2 = p[3],
                       u3 = p[4], v3 = p[5];

                // Punkte auf den Flächen auswerten
                GeoPoint P1 = surface1.PointAt(new GeoPoint2D(u1, v1));
                GeoPoint P2 = surface2.PointAt(new GeoPoint2D(u2, v2));
                GeoPoint P3 = surface3.PointAt(new GeoPoint2D(u3, v3));

                var r = Vector<double>.Build.Dense(dim);
                // S1 - S2
                r[0] = P1.x - P2.x;
                r[1] = P1.y - P2.y;
                r[2] = P1.z - P2.z;
                // S1 - S3
                r[3] = P1.x - P3.x;
                r[4] = P1.y - P3.y;
                r[5] = P1.z - P3.z;
                return r;
            };

            // Analytische Jacobimatrix J(p)
            Func<Vector<double>, Vector<double>, Matrix<double>> jacobian = (p, x) =>
            {
                double u1 = p[0], v1 = p[1],
                       u2 = p[2], v2 = p[3],
                       u3 = p[4], v3 = p[5];

                // Erste Ableitungen ermitteln
                GeoVector du1 = surface1.UDirection(new GeoPoint2D(u1, v1));
                GeoVector dv1 = surface1.VDirection(new GeoPoint2D(u1, v1));
                GeoVector du2 = surface2.UDirection(new GeoPoint2D(u2, v2));
                GeoVector dv2 = surface2.VDirection(new GeoPoint2D(u2, v2));
                GeoVector du3 = surface3.UDirection(new GeoPoint2D(u3, v3));
                GeoVector dv3 = surface3.VDirection(new GeoPoint2D(u3, v3));

                var J = Matrix<double>.Build.Dense(dim, dim, 0.0);

                // Zeilen 0–2: P1 - P2
                for (int i = 0; i < 3; i++)
                {
                    // P1-Komponente
                    J[i, 0] = du1[i];
                    J[i, 1] = dv1[i];
                    // P2-Komponente (negativ)
                    J[i, 2] = -du2[i];
                    J[i, 3] = -dv2[i];
                }
                // Zeilen 3–5: P1 - P3
                for (int i = 0; i < 3; i++)
                {
                    J[3 + i, 0] = du1[i];
                    J[3 + i, 1] = dv1[i];
                    J[3 + i, 4] = -du3[i];
                    J[3 + i, 5] = -dv3[i];
                }

                return J;
            };

            // ObjectiveModel mit analytischer Jacobi
            var objective = ObjectiveFunction.NonlinearModel(model, jacobian, dummyX, observedY, weights);

            // Startwerte
            double[] initial = { uv1.x, uv1.y, uv2.x, uv2.y, uv3.x, uv3.y };
            double[] lower = Enumerable.Repeat(double.NegativeInfinity, dim).ToArray();
            double[] upper = Enumerable.Repeat(double.PositiveInfinity, dim).ToArray();
            double[] scales = Enumerable.Repeat(1.0, dim).ToArray();
            bool[] fixedParams = new bool[dim]; // alle false

            // Solver konfigurieren und ausführen
            var solver = new LevenbergMarquardtMinimizer(
                initialMu: 1e-3,
                gradientTolerance: 1e-6,
                stepTolerance: 1e-6,
                functionTolerance: 1e-6,
                maximumIterations: 100);

            var result = solver.FindMinimum(objective, initial, lower, upper, scales, fixedParams);

            // Prüfen, ob konvergiert
            if (result.ReasonForExit == ExitCondition.Converged)
            {
                var sol = result.MinimizingPoint;
                uv1 = new GeoPoint2D(sol[0], sol[1]);
                uv2 = new GeoPoint2D(sol[2], sol[3]);
                uv3 = new GeoPoint2D(sol[4], sol[5]);
                return result.ModelInfoAtMinimum.Value;
            }
            else
            {
                // Kein konvergentes Ergebnis
                return double.MaxValue;
            }
        }
        internal static bool Overlapping(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, double precision, out ModOp2D From1To2)
        {   // zwei Oberflächen überlappen sich wenn sie eine gemeinsame Fläche haben
            // hier erstmal Abfragen nach gleichem Typ dort ist es besser zu lösen

            if ((surface1.GetType() == surface2.GetType()) && (!(surface1 is NurbsSurface)) && (!(surface1 is RuledSurface)))
            {
                return surface1.SameGeometry(bounds1, surface2, bounds2, Precision.eps, out From1To2);
            }
            // erste Bedingung ist, dass es mindestens zwei Eckunkte gibt, die in der jeweils anderen Fläche liegen
            // dann gibt es mehrere Fälle:
            // Alle Eckpunkte der einen Fläche liegen in der anderen: dann braucht man nur die eine zu berücksichtigen
            // gemischt: also es gibt einen von 1, der in 2 liegt und einen von 2, der in 1 liegt (oder jeweils mehrere)
            // Ein solches gemischte nicht identisches Paar wird gebraucht. Es halt also uv Werte in beiden Flächen
            // der uv Zwischenpunkt in beiden Flächen muss also auch in 3d identisch sein. Damit haben wir 
            // zwei uv-Tripel, die aufeinander abgebildet eine ModOp2D geben
            // Wir brinden das 2. Rechteck in das uv System des ersten und betrachten die Schnittfläche.
            // Auf dieser Fläche muss alles identisch sein. Es genügt die Basispunkte und die Normalen dort
            // zu betrachten
            // leider können sich die beiden Flächen auch so überdecken, dass keine Eckpunkte der einen in der jeweils anderen liegen
            // dann müssen aber die Kanten sich schneiden, jeweils 2 von der einen mit 2 von der anderen Fläche
            // hier wird nicht berücksichtigt, dass die Flächen völlig zueinander verzerrt sind, da gäbe es dann auch keine ModOp2D
            BoxedSurfaceEx bs1 = (surface1 as ISurfaceImpl).BoxedSurfaceEx;
            BoxedSurfaceEx bs2 = (surface2 as ISurfaceImpl).BoxedSurfaceEx;
            From1To2 = ModOp2D.Identity;
            if (!bs1.IsCloseTo(bs2)) return false;
            // die Eckpunkte bestimmen
            GeoPoint ll2 = surface2.PointAt(bounds2.GetLowerLeft());
            GeoPoint lr2 = surface2.PointAt(bounds2.GetLowerRight());
            GeoPoint ul2 = surface2.PointAt(bounds2.GetUpperLeft());
            GeoPoint ur2 = surface2.PointAt(bounds2.GetUpperRight());

            GeoPoint ll1 = surface1.PointAt(bounds2.GetLowerLeft());
            GeoPoint lr1 = surface1.PointAt(bounds2.GetLowerRight());
            GeoPoint ul1 = surface1.PointAt(bounds2.GetUpperLeft());
            GeoPoint ur1 = surface1.PointAt(bounds2.GetUpperRight());

            GeoPoint2D ll2on1, lr2on1, ul2on1, ur2on1, ll1on2, lr1on2, ul1on2, ur1on2;
            bool valid1 = false, valid2 = false;
            if (bs1.IsCloseTo(ll2))
            {
                // valid1 = true;
                ll2on1 = surface1.PositionOf(ll2);
                valid1 |= bounds1.Contains(ll2on1);
            }
            else ll2on1 = GeoPoint2D.Invalid;
            if (bs1.IsCloseTo(lr2))
            {
                // valid1 = true;
                lr2on1 = surface1.PositionOf(lr2);
                valid1 |= bounds1.Contains(lr2on1);
            }
            else lr2on1 = GeoPoint2D.Invalid;
            if (bs1.IsCloseTo(ul2))
            {
                // valid1 = true;
                ul2on1 = surface1.PositionOf(ul2);
                valid1 |= bounds1.Contains(ul2on1);
            }
            else ul2on1 = GeoPoint2D.Invalid;
            if (bs1.IsCloseTo(ur2))
            {
                // valid1 = true;
                ur2on1 = surface1.PositionOf(ur2);
                valid1 |= bounds1.Contains(ur2on1);
            }
            else ur2on1 = GeoPoint2D.Invalid;

            if (bs2.IsCloseTo(ll1))
            {
                // valid2 = true;
                ll1on2 = surface2.PositionOf(ll1);
                valid2 |= bounds1.Contains(ll1on2);
            }
            else ll1on2 = GeoPoint2D.Invalid;
            if (bs2.IsCloseTo(lr1))
            {
                // valid2 = true;
                lr1on2 = surface2.PositionOf(lr1);
                valid2 |= bounds1.Contains(lr1on2);
            }
            else lr1on2 = GeoPoint2D.Invalid;
            if (bs2.IsCloseTo(ul1))
            {
                // valid2 = true;
                ul1on2 = surface2.PositionOf(ul1);
                valid2 |= bounds1.Contains(ul1on2);
            }
            else ul1on2 = GeoPoint2D.Invalid;
            if (bs2.IsCloseTo(ur1))
            {
                // valid2 = true;
                ur1on2 = surface2.PositionOf(ur1);
                valid2 |= bounds1.Contains(ur1on2);
            }
            else ur1on2 = GeoPoint2D.Invalid;

            if (!valid1 && !valid2)
            {   // jetzt immer noch die Möglichkeit, dass sie sich überschneiden ohne gemeinsamen Eckpunkt
                ICurve bottom1 = surface1.FixedV(bounds1.Bottom, bounds1.Left, bounds1.Right);
                ICurve top1 = surface1.FixedV(bounds1.Top, bounds1.Left, bounds1.Right);
                ICurve left1 = surface1.FixedU(bounds1.Left, bounds1.Bottom, bounds1.Top);
                ICurve right1 = surface1.FixedU(bounds1.Right, bounds1.Bottom, bounds1.Top);

                ICurve bottom2 = surface2.FixedV(bounds2.Bottom, bounds2.Left, bounds2.Right);
                ICurve top2 = surface2.FixedV(bounds2.Top, bounds2.Left, bounds2.Right);
                ICurve left2 = surface2.FixedU(bounds2.Left, bounds2.Bottom, bounds2.Top);
                ICurve right2 = surface2.FixedU(bounds2.Right, bounds2.Bottom, bounds2.Top);

                // jede Kurve der 1. Fläche mit jeder der 2. Fläche schneiden
                double[] ipars = Curves.Intersect(bottom1, bottom2, true);

                // funktioniert noch nicht, nur für ebene Kurven
                From1To2 = ModOp2D.Identity;

                return false;
            }
            else
            {   // und hier kommt die schwierige Aufgabe, die gemeinsame Überschneidungsfläche zu finden und zu sehen, ob wir innerhalb dieser 
                // auch identisch sind
                // eigentlich müssten wir die Schnitte der 4 Kanten miteinander testen
                // um den jeweiligen uv Bereich zu bestimmen, in dem die Punkte getestet werden müssen
                double[] intu;
                double[] intv;
                bool pointsChecked = false; ;
                surface2.GetSafeParameterSteps(bounds2.Left, bounds2.Right, bounds2.Bottom, bounds2.Top, out intu, out intv);
                for (int i = 0; i < intu.Length; ++i)
                {
                    for (int j = 0; j < intv.Length; ++j)
                    {
                        GeoPoint2D uv2 = new GeoPoint2D(intu[i], intv[j]);
                        GeoPoint testPoint = surface2.PointAt(uv2);
                        GeoVector normal = surface2.GetNormal(uv2);
                        if (bs1.IsClose(testPoint))
                        {
                            GeoPoint2D uv1;
                            if (bs1.PositionOf(testPoint, out uv1))
                            {
                                // hier noch überprüfen, ob testPoint senkrecht über uv1 liegt, sonst 
                                // gibts Probleme am Rand
                                pointsChecked = true;
                                if ((surface1.PointAt(uv1) | testPoint) > precision) return false;
                                if (!Precision.SameDirection(surface1.GetNormal(uv1), normal, false)) return false;
                            }
                        }
                    }
                }
                // und auch noch umgekehrt testen:
                surface1.GetSafeParameterSteps(bounds1.Left, bounds1.Right, bounds1.Bottom, bounds1.Top, out intu, out intv);
                for (int i = 0; i < intu.Length; ++i)
                {
                    for (int j = 0; j < intv.Length; ++j)
                    {
                        GeoPoint2D uv1 = new GeoPoint2D(intu[i], intv[j]);
                        GeoPoint testPoint = surface1.PointAt(uv1);
                        GeoVector normal = surface1.GetNormal(uv1);
                        if (bs2.IsClose(testPoint))
                        {
                            GeoPoint2D uv2;
                            if (bs2.PositionOf(testPoint, out uv2))
                            {
                                // hier noch überprüfen, ob testPoint senkrecht über uv1 liegt, sonst 
                                // gibts Probleme am Rand
                                pointsChecked = true;
                                if ((surface2.PointAt(uv2) | testPoint) > precision) return false;
                                if (!Precision.SameDirection(surface2.GetNormal(uv2), normal, false)) return false;
                            }
                        }
                    }
                }
                // besser so: eine Liste von uv-Paaren erstellen, die sicher in beiden Flächen vorkommen:
                // also die Eckpunkte der einen in der anderen und umgekehrt
                // und die Schnittpunkte der Randkurven (nicht Berührpunkte z.B. bei überlappenden Kurven)
                // ein konvexes Polygon daraus bilden. Die Eckpunkte (sind es nicht immer 4?) auf Gleichheit überprüfen
                // und ggf. noch Innenpunkte bzw. Normalenvektoren (bei diesen ist die Genauigkeit ein Problem)
                return pointsChecked;
            }

            // From1To2 = ModOp2D.Identity;

            // return false;
        }

        internal static IDualSurfaceCurve[] IntersectInner(ISurface surface1, BoundingRect ext1, ISurface surface2, BoundingRect ext2)
        {
            int ep = surface1.GetExtremePositions(ext1, surface2, ext2, out List<Tuple<double, double, double, double>> extremePositions);
            List<IDualSurfaceCurve> res = new List<IDualSurfaceCurve>();
            if (extremePositions != null && extremePositions.Count > 0)
            {   // lets calculate some seeds from the fixed curves on one of the surfaces
                List<GeoPoint> seeds = new List<GeoPoint>();
                for (int i = 0; i < extremePositions.Count; i++)
                {
                    ICurve crv = null;
                    if (!double.IsNaN(extremePositions[i].Item1) && ext1.Left <= extremePositions[i].Item1 && ext1.Right >= extremePositions[i].Item1) crv = surface1.FixedU(extremePositions[i].Item1, ext1.Bottom, ext1.Top);
                    else if (!double.IsNaN(extremePositions[i].Item2) && ext1.Bottom <= extremePositions[i].Item2 && ext1.Top >= extremePositions[i].Item2) crv = surface1.FixedV(extremePositions[i].Item2, ext1.Left, ext1.Right);
                    if (!double.IsNaN(extremePositions[i].Item3) && ext2.Left <= extremePositions[i].Item3 && ext2.Right >= extremePositions[i].Item3) crv = surface2.FixedU(extremePositions[i].Item3, ext2.Bottom, ext2.Top);
                    else if (!double.IsNaN(extremePositions[i].Item4) && ext2.Bottom <= extremePositions[i].Item4 && ext2.Top >= extremePositions[i].Item4) crv = surface2.FixedV(extremePositions[i].Item4, ext2.Left, ext2.Right);
                    if (crv != null)
                    {
                        if (!double.IsNaN(extremePositions[i].Item1) && !double.IsNaN(extremePositions[i].Item2))
                        {
                            // crv is on surface1
                            surface2.Intersect(crv, ext2, out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve3Ds);
                            for (int j = 0; j < ips.Length; j++)
                            {
                                if (ext2.Contains(uvOnFaces[j]) && uOnCurve3Ds[j] >= 0.0 && uOnCurve3Ds[j] <= 1.0) seeds.Add(ips[j]);
                            }

                        }
                        else
                        {
                            // crv is on surface2
                            surface1.Intersect(crv, ext1, out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve3Ds);
                            for (int j = 0; j < ips.Length; j++)
                            {
                                if (ext1.Contains(uvOnFaces[j]) && uOnCurve3Ds[j] >= 0.0 && uOnCurve3Ds[j] <= 1.0) seeds.Add(ips[j]);
                            }
                        }
                    }
                }
                // seeds are not sorted. The curve must be closed. We simply repeat the first seed as the last seed
                if (seeds.Count > 1)
                {   // seeds are not sorted. But if we have 4 seeds, two curves have been used , the first curve created seed 0 and 1, the second 2 and 3
                    // so we better exchange 1 and 2
                    if (seeds.Count == 4)
                    {
                        GeoPoint tmp = seeds[1];
                        seeds[1] = seeds[2];
                        seeds[2] = tmp;
                    }
                    if (!Precision.IsEqual(seeds.First(), seeds.Last())) seeds.Add(seeds.First());
                    for (int i = seeds.Count - 1; i > 0; --i)
                    {
                        if (Precision.IsEqual(seeds[i], seeds[i - 1])) seeds.RemoveAt(i);
                    }
                    // if (Precision.IsEqual(seeds.First(), seeds.Last())) seeds.RemoveAt(seeds.Count - 1);
                    // there might be a problem with only two seed points: we need two different curves as a result, which go one way or the other from seed0 to seed1. GetDualSurfaceCurves maybe returns only one.
                    IDualSurfaceCurve[] candidates = surface1.GetDualSurfaceCurves(ext1, surface2, ext2, seeds, null);
                    if (candidates.Length>1)
                    {
                        List<IDualSurfaceCurve> tmp = new List<IDualSurfaceCurve>(candidates);
                        for (int i = tmp.Count - 1; i > 0; --i)
                        {
                            for (int j = 0; j < i; j++)
                            {
                                if (tmp[j].Curve3D.SameGeometry(tmp[i].Curve3D,Precision.eps))
                                {
                                    tmp.RemoveAt(i);
                                    break;
                                }
                            }
                        }
                        candidates = tmp.ToArray();
                    }
                    for (int i = 0; i < candidates.Length; i++)
                    {
                        // here we have different behaviour: some surfaces return closed curves, but sometimes the same curve twice,
                        // other surfaces return splitted curves which are always open. BooleanOperation can handle both cases
                        bool isClosed = candidates[i].Curve3D.IsClosed || Precision.Equals(candidates[i].Curve3D.StartPoint, candidates[i].Curve3D.EndPoint);
                        if (isClosed)
                        {
                            if (res.Count > 0)
                            {
                                if (res.Last().Curve3D.DistanceTo(candidates[i].Curve3D.StartPoint) < Precision.eps &&
                                    res.Last().Curve3D.DistanceTo(candidates[i].Curve3D.EndPoint) < Precision.eps) continue; // this is the same closed curve
                            }
                            res.Add(candidates[i]);
                        }
                        else
                        {
                            res.Add(candidates[i]);
                        }
                    }
                }
            }
            if (res.Count == 0 && ep == -1)
            {
                // GetExtremePositions is not implemented for all surface combinations. If it fails (returns -1) and GetDualSurfaceCurves didnt return anything
                // then we have to use burte force to get a seed point
                // problem with helicalsurface in "Stativgewinde1.cdb.json"
                //GeoPoint seed = IntersectionSeedPoint(surface1, ext1, surface2, ext2);
                //if (seed.IsValid)
                //{
                //    GeoPoint2D uv1 = surface1.PositionOf(seed);
                //    SurfaceHelper.AdjustPeriodic(surface1, ext1, ref uv1);
                //    GeoPoint2D uv2 = surface1.PositionOf(seed);
                //    SurfaceHelper.AdjustPeriodic(surface2, ext2, ref uv2);
                //    if (!ext1.Contains(uv1)) uv1 = GeoPoint2D.Invalid;
                //    if (!ext2.Contains(uv2)) uv2 = GeoPoint2D.Invalid;
                //    if (uv1.IsValid || uv2.IsValid)
                //    {
                //        extremePositions = new List<Tuple<double, double, double, double>>();
                //        extremePositions.Add(new Tuple<double, double, double, double>(uv1.x, uv1.y, uv2.x, uv2.y));
                //        candidates = surface1.GetDualSurfaceCurves(ext1, surface2, ext2, null, extremePositions);
                //        res = new List<IDualSurfaceCurve>();
                //        for (int i = 0; i < candidates.Length; i++)
                //        {
                //            if (candidates[i].Curve3D.IsClosed) res.Add(candidates[i]);
                //            else if (Precision.Equals(candidates[i].Curve3D.StartPoint, candidates[i].Curve3D.EndPoint)) res.Add(candidates[i]);
                //        }
                //    }
                //}
            }
            return res.ToArray();
        }
        private static GeoPoint IntersectionSeedPoint(ISurface surface1, BoundingRect ext1, ISurface surface2, BoundingRect ext2)
        {
            Face fc1 = Face.MakeFace(surface1, new SimpleShape(ext1.ToBorder()));
            Face fc2 = Face.MakeFace(surface2, new SimpleShape(ext2.ToBorder()));
            BoundingBox ext = fc1.GetExtent(0.0) + fc1.GetExtent(0.0);
            bool found = false;
            GeoPoint seed = GeoPoint.Invalid;
            bool SplitTestFunction(OctTree<Face>.Node<Face> node, Face objectToAdd)
            {
                if (found) return false;
                if (node.list != null && node.list.Count > 0 && !node.list.Contains(objectToAdd))
                {
                    if (node.size > ext.Size * 1e-3) return true;
                    GeoPoint testPoint = node.center;
                    if (NewtonIntersect(surface1, ext1, surface2, ext2, ref testPoint))
                    {
                        found = true;
                        seed = testPoint;
                        return false;
                    }
                }
                return false;
            }
            OctTree<Face> ot = new OctTree<Face>(ext, Precision.eps, SplitTestFunction);
            ot.AddObject(fc1);
            ot.AddObject(fc2);
            return seed;
        }

        private static bool NewtonIntersect(ISurface surface1, BoundingRect ext1, ISurface surface2, BoundingRect ext2, ref GeoPoint testPoint)
        {
            GeoPoint2D uv1 = surface1.PositionOf(testPoint);
            SurfaceHelper.AdjustPeriodic(surface1, ext1, ref uv1);
            GeoPoint2D uv2 = surface2.PositionOf(testPoint);
            SurfaceHelper.AdjustPeriodic(surface2, ext2, ref uv2);
            double dist = surface1.PointAt(uv1) | surface2.PointAt(uv2);
            while (dist > Precision.eps)
            {
                Plane pln1 = new Plane(surface1.PointAt(uv1), surface1.GetNormal(uv1));
                Plane pln2 = new Plane(surface2.PointAt(uv2), surface2.GetNormal(uv2));
                if (pln1.Intersect(pln2, out GeoPoint loc, out GeoVector dir))
                {
                    testPoint = Geometry.DropPL(testPoint, loc, dir);
                    uv1 = surface1.PositionOf(testPoint);
                    uv2 = surface2.PositionOf(testPoint);
                    double newdist = surface1.PointAt(uv1) | surface2.PointAt(uv2);
                    if (newdist > dist) return false;
                    else dist = newdist;
                }
                else break;
            }
            return (dist <= Precision.eps);
        }
        internal static bool NewtonIntersect(ISurface surface, BoundingRect ext, ICurve crv, ref GeoPoint testPoint)
        {
            GeoPoint2D uv = surface.PositionOf(testPoint);
            SurfaceHelper.AdjustPeriodic(surface, ext, ref uv);
            double u = crv.PositionOf(testPoint);
            double dist = surface.PointAt(uv) | crv.PointAt(u);
            try
            {
                int maxIteration = 10;
                while (maxIteration > 0) // Precision.eps)
                {
                    --maxIteration;
                    Plane pln = new Plane(surface.PointAt(uv), surface.GetNormal(uv));
                    GeoPoint loc = testPoint;
                    GeoVector dir = crv.DirectionAt(u);
                    testPoint = pln.Intersect(testPoint, dir);
                    uv = surface.PositionOf(testPoint);
                    u = crv.PositionOf(testPoint);
                    double newdist = surface.PointAt(uv) | testPoint;
                    if (newdist >= dist) break;
                    else dist = newdist;
                }
            }
            catch (PlaneException) { } // we are tangential!
            return (dist <= Precision.eps);
        }

        internal static bool NewtonPerpendicular(ISurface surface, BoundingRect ext, ICurve crv, ref GeoPoint testPoint)
        {

            //GeoVector du, dv, duu, dvv, duv;
            //GeoPoint loc;
            //surface.Derivation2At(uv, out loc, out du, out dv, out duu, out dvv, out duv);
            //GeoVector n = du ^ dv;
            //GeoVector diruv = n ^ dir;
            //GeoVector2D duvs = Geometry.Dir2D(du, dv, diruv);
            //surface.DerivationAt(uv + duvs, out loc, out du, out dv);
            //n = du ^ dv;
            //surface.DerivationAt(uv - duvs, out loc, out du, out dv);
            //n = du ^ dv;
            //surface.DerivationAt(uv + duvs.ToLeft(), out loc, out du, out dv);
            //n = du ^ dv;
            //surface.DerivationAt(uv + duvs.ToRight(), out loc, out du, out dv);
            //n = du ^ dv;

            return false;
        }
        /// <summary>
        /// A <paramref name="surface"/> should be tangential to other <paramref name="tangentialSurfaces"/>. This is used in the parametrics
        /// and only implemented for a few cases here.
        /// 
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="tangentialSurfaces"></param>
        /// <returns></returns>
        internal static ISurface ModifyTangential(ISurface surface, List<ISurface> tangentialSurfaces)
        {
            if (surface is PlaneSurface pls)
            {
                if (tangentialSurfaces.Count == 2 && tangentialSurfaces[0] is CylindricalSurface cyl0 && tangentialSurfaces[1] is CylindricalSurface cyl1)
                {
                    // a plane tangential to two cylinders
                    if (Precision.SameDirection(cyl0.Axis, cyl1.Axis, false))
                    {
                        PlaneSurface res = null;
                        Plane pln = new Plane(cyl0.Location, cyl0.Axis);
                        Circle2D c0 = new Circle2D(GeoPoint2D.Origin, cyl0.RadiusX);
                        Circle2D c1 = new Circle2D(pln.Project(cyl1.Location), cyl1.RadiusX);
                        GeoPoint2D[] tp = Curves2D.TangentLines(c0, c1);
                        double mindist = double.MaxValue;
                        for (int i = 0; i < tp.Length; i += 2)
                        {
                            GeoPoint tp0 = pln.ToGlobal(tp[i]);
                            GeoPoint tp1 = pln.ToGlobal(tp[i + 1]);
                            double d = pls.GetDistance(tp0) + pls.GetDistance(tp1);
                            if (d < mindist)
                            {
                                mindist = d;
                                res = new PlaneSurface(tp0, tp1 - tp0, pln.Normal);
                            }
                        }
                        return res;
                    }

                }
            }
            else if (surface is CylindricalSurface cyl)
            {
                if (tangentialSurfaces.Count == 2 && tangentialSurfaces[0] is ToroidalSurface tor0 && tangentialSurfaces[1] is ToroidalSurface tor1)
                {
                    if (Precision.SameDirection(tor0.ZAxis, tor1.ZAxis, false))
                    {
                        Plane pln = new Plane(tor0.Location, tor0.XAxis, tor0.YAxis);
                        Circle2D c0 = new Circle2D(GeoPoint2D.Origin, tor0.XAxis.Length);
                        Circle2D c1 = new Circle2D(pln.Project(tor1.Location), tor1.XAxis.Length);
                        GeoPoint2D[] tp = Curves2D.TangentLines(c0, c1);
                        double mindist = double.MaxValue;
                        CylindricalSurface res = null;
                        for (int i = 0; i < tp.Length; i += 2)
                        {
                            GeoPoint tp0 = pln.ToGlobal(tp[i]);
                            GeoPoint tp1 = pln.ToGlobal(tp[i + 1]);
                            double d = Geometry.DistPL(tp0, cyl.Location, cyl.Axis) + Geometry.DistPL(tp1, cyl.Location, cyl.Axis);
                            if (d < mindist)
                            {
                                mindist = d;
                                GeoVector dirz = tp1 - tp0;
                                GeoVector dirx = dirz ^ pln.Normal;
                                GeoVector diry = pln.Normal;
                                dirx.Length = tor0.MinorRadius;
                                diry.Length = tor0.MinorRadius;
                                res = new CylindricalSurface(tp0, dirx, diry, dirz);
                            }
                        }
                        return res;
                    }
                }
            }

            return null;
        }
        /// <summary>
        /// Find intersection curves of two cylinders. Result may be 0, 1 or 2 curves. Infinite lines (parallel or tangential cylinders) are returned as finite Lines.
        /// The other cases result in closed curves (ellipses or BSplines) with arbitrary start/end points
        /// </summary>
        /// <param name="cylinder1"></param>
        /// <param name="cylinder2"></param>
        /// <returns></returns>
        internal static ICurve[] Intersect(ICylinder cylinder1, ICylinder cylinder2)
        {
            if (Precision.SameDirection(cylinder1.Axis.Direction, cylinder2.Axis.Direction, false))
            {

            }
            throw new NotImplementedException();
        }

        /// <summary>
        /// Finds a pair of points, one on each surface, so that the straight connection between the two points is
        /// perpendicular to both surfaces at these points (i.e. the connection is parallel to the surface normal on
        /// both surfaces). Such a pair is a critical point of the distance between the two surfaces (local minimum,
        /// maximum or saddle of the distance). There may be more than one such pair; this method returns the first one
        /// found starting from the provided <paramref name="uv1"/> and <paramref name="uv2"/>.
        /// The search is restricted to the parameter rectangles <paramref name="bounds1"/> and <paramref name="bounds2"/>.
        /// A pair where the two points coincide (an intersection point of the surfaces) is not considered valid, because
        /// the perpendicularity is undefined there.
        /// </summary>
        /// <param name="surface1">The first surface.</param>
        /// <param name="bounds1">The valid u/v domain on the first surface.</param>
        /// <param name="surface2">The second surface.</param>
        /// <param name="bounds2">The valid u/v domain on the second surface.</param>
        /// <param name="uv1">On input the starting position on the first surface, on output the result (if found).</param>
        /// <param name="uv2">On input the starting position on the second surface, on output the result (if found).</param>
        /// <returns>true if a valid perpendicular connection has been found, false otherwise.</returns>
        public static bool PerpendicularConnection(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, ref GeoPoint2D uv1, ref GeoPoint2D uv2)
        {
            // The connection d = P1 - P2 is perpendicular to both surfaces when it is orthogonal to both tangents of
            // each surface. This yields four residuals (one per tangent) in four unknowns (u1, v1, u2, v2):
            //   r0 = d * S1u, r1 = d * S1v, r2 = d * S2u, r3 = d * S2v
            // This residual vector is exactly the gradient of 1/2 * |P1 - P2|^2, so its roots are the critical points
            // of the distance between the surfaces. We solve the (square) system with the Levenberg-Marquardt
            // minimizer, which minimizes ||r||^2 and is far more robust than an undamped Gauss-Newton step.

            // Residual r(p), p = [u1, v1, u2, v2]; the observed vector is ignored (target is the zero vector).
            Vector<double> Residual(Vector<double> p, Vector<double> observed)
            {
                surface1.DerivativeAt(new GeoPoint2D(p[0], p[1]), out GeoPoint P1, out GeoVector S1u, out GeoVector S1v);
                surface2.DerivativeAt(new GeoPoint2D(p[2], p[3]), out GeoPoint P2, out GeoVector S2u, out GeoVector S2v);
                GeoVector d = P1 - P2;
                return new DenseVector(new double[] { d * S1u, d * S1v, d * S2u, d * S2v });
            }

            // Analytic Jacobian J = d r / d p (4 x 4); this is the Hessian of 1/2 * |P1 - P2|^2.
            // It needs the second derivatives, so both surfaces must implement Derivative2At.
            Matrix<double> Jacobian(Vector<double> p, Vector<double> observed)
            {
                surface1.Derivative2At(new GeoPoint2D(p[0], p[1]), out GeoPoint P1, out GeoVector S1u, out GeoVector S1v, out GeoVector S1uu, out GeoVector S1vv, out GeoVector S1uv);
                surface2.Derivative2At(new GeoPoint2D(p[2], p[3]), out GeoPoint P2, out GeoVector S2u, out GeoVector S2v, out GeoVector S2uu, out GeoVector S2vv, out GeoVector S2uv);
                GeoVector d = P1 - P2;
                Matrix<double> J = new DenseMatrix(4, 4);
                // d (d/du1, d/dv1) = (S1u, S1v); d (d/du2, d/dv2) = (-S2u, -S2v)
                J[0, 0] = S1u * S1u + d * S1uu; J[0, 1] = S1u * S1v + d * S1uv; J[0, 2] = -(S2u * S1u); J[0, 3] = -(S2v * S1u);
                J[1, 0] = S1u * S1v + d * S1uv; J[1, 1] = S1v * S1v + d * S1vv; J[1, 2] = -(S2u * S1v); J[1, 3] = -(S2v * S1v);
                J[2, 0] = S1u * S2u; J[2, 1] = S1v * S2u; J[2, 2] = -(S2u * S2u) + d * S2uu; J[2, 3] = -(S2v * S2u) + d * S2uv;
                J[3, 0] = S1u * S2v; J[3, 1] = S1v * S2v; J[3, 2] = -(S2u * S2v) + d * S2uv; J[3, 3] = -(S2v * S2v) + d * S2vv;
                return J;
            }

            Vector<double> observedX = new DenseVector(4); // ignored, just provides the right length
            Vector<double> observedY = new DenseVector(4); // target is the zero vector: minimize ||r||^2

            // Note: we deliberately do NOT pass MathNet's box constraints (lowerBound/upperBound) here. When the two
            // parameter domains differ strongly in scale (e.g. a NURBS surface on [0,1]x[0,1] against a plane on a
            // domain spanning tens of units), the bounded variant of the Levenberg-Marquardt minimizer stalls far away
            // from the actual solution. The unconstrained solve converges reliably; we enforce the domains afterwards
            // by rejecting a result that lies outside bounds1/bounds2.
            NonlinearMinimizationResult result;
            try
            {
                IObjectiveModel model = ObjectiveFunction.NonlinearModel(Residual, Jacobian, observedX, observedY);
                LevenbergMarquardtMinimizer lm = new LevenbergMarquardtMinimizer(maximumIterations: 100);
                result = lm.FindMinimum(model, new DenseVector(new double[] { uv1.x, uv1.y, uv2.x, uv2.y }));
            }
            catch
            {
                return false;
            }

            if (result.ReasonForExit != ExitCondition.Converged && result.ReasonForExit != ExitCondition.RelativePoints && result.ReasonForExit != ExitCondition.RelativeGradient)
            {
                return false;
            }

            GeoPoint2D ruv1 = new GeoPoint2D(result.MinimizingPoint[0], result.MinimizingPoint[1]);
            GeoPoint2D ruv2 = new GeoPoint2D(result.MinimizingPoint[2], result.MinimizingPoint[3]);

            // The result must lie inside the provided domains.
            if (!bounds1.ContainsEps(ruv1, bounds1.Size * 1e-6) || !bounds2.ContainsEps(ruv2, bounds2.Size * 1e-6)) return false;

            // Verify the geometric condition independently of the optimizer: the connection must be parallel to both
            // surface normals, and the two points must not coincide (an intersection point is not a valid solution).
            surface1.DerivativeAt(ruv1, out GeoPoint p1, out GeoVector s1u, out GeoVector s1v);
            surface2.DerivativeAt(ruv2, out GeoPoint p2, out GeoVector s2u, out GeoVector s2v);
            GeoVector dir = p1 - p2;
            double dist = dir.Length;
            if (dist < Precision.eps) return false; // surfaces touch / intersect here, perpendicularity is undefined
            dir = (1.0 / dist) * dir;
            GeoVector n1 = (s1u ^ s1v).Normalized;
            GeoVector n2 = (s2u ^ s2v).Normalized;
            // |dir ^ n| is the sine of the angle between the connection and the surface normal; it must be ~0.
            if ((dir ^ n1).Length > 1e-6 || (dir ^ n2).Length > 1e-6) return false;

            uv1 = ruv1;
            uv2 = ruv2;
            return true;
        }
    }
}
