using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.UserInterface;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using Wintellect.PowerCollections;
using static CADability.InterpolatedDualSurfaceCurve.SurfacePoint;
using static CADability.ProjectedEdge;

namespace CADability
{
    /* Projekt: InterpolatedDualSurfaceCurve für Schnitte zwischen Ebene, Zylinder, Kugel, Kegel, Torus, SurfaceOfRevolution (einfach Form), SurfaceOfLinearExtrusion (einfache Form).
     * Die Idee: Nehmen wir Torus/Zylinder: die FixedU und FixedV Kurven diese Flächen sind Linien oder Kreise.
     * Wenn man jetzt die Kurvenschaar zu FixedU bzw. FixedV betrachtet, so sind die jeweiligen Schnitte mit der anderen Fläche einfach zu berechnen (Torus: naja...)
     * Aber leider gibt es die Fälle, wo diese Kurven tangential zur anderen Fläche liegen. An diesen Stellen wäre die jeweils andere FixedUV Kurve natürlich besser.
     * Es gilt also diese Tangentialpunkte zu bestimmen und damit die Bereiche festzulegen, in denen mit FixedU bzw. mit FixedV gearbeitet werden muss.
     * Die idealen Intervallgrenzen wären die, wo FixedU und FixedV mit dem gleichen Winkel zur Ebene im Schnittpunkt stehen. Die sind nicht unbedingt leicht zu finden.
     * Vielleicht genügt es ja, Zwischenpunkte zwischen den Tangentialpunkten zu nehmen. Das sollte recht unkritisch sein.
     * Man müsste von InterpolatedDualSurfaceCurve ableiten. Die Punktbestimmung läuft eigentlich ziemlich genau wie in ApproximatePosition.
     * Zusätzlich zu den BasePoints gibt es für jeden Abschnitt (es sind immer geschlossene Kurven) noch die Information, ob mit FixedU oder mit FixedV gearbeitet wird
     * und das Intervall des nicht festen Parameters für diesen Abschnitt. ApproximatePosition würde dann nicht mehr iterativ arbeiten, sondern direkt den Punkt finden.
     * PositionOf
     */

    /// <summary>
    /// Internal: ein Kante, gegeben durch zwei Oberflächen und ein Array von 3d/2d/2d Punkten
    /// </summary>
    [Serializable()]
    [JsonVersion(1)]
    public class InterpolatedDualSurfaceCurve : GeneralCurve, IDualSurfaceCurve, IJsonSerialize, IExportStep, IJsonSerializeDone, IDeserializationCallback, IOrientation
    {
        ISurface surface1; // the two surfaces
        ISurface surface2;
        BoundingRect bounds1 = BoundingRect.EmptyBoundingRect, bounds2 = BoundingRect.EmptyBoundingRect; // the uv region, where these surfaces are beeing used
        SurfacePoint[] basePoints; // some points, especially start and endpoint, of the curve, that have been calculated
        bool forwardOriented; // the crossproduct surface1.Normal^surface2.Normal is the direction of the curve if true
        bool isTangential = false; // we need a different point approximation for curves which describe the tangential intersection of two surfaces
        BSpline approxBSpline; // BSpline for approximation
        SortedList<double, SurfacePoint> hashedPositions; // already calculated points on the curve

        [Serializable()]
        [JsonVersion(serializeAsStruct = true, version = 1)]
        internal struct SurfacePoint : ISerializable, IJsonSerialize
        {
            public SurfacePoint(GeoPoint p3d, GeoPoint2D psurface1, GeoPoint2D psurface2)
            {
                this.p3d = p3d;
                this.psurface1 = psurface1;
                this.psurface2 = psurface2;
            }
            public GeoPoint p3d;
            public GeoPoint2D psurface1;
            public GeoPoint2D psurface2;

            public GeoPoint2D PointOnSurface(ISurface surface, BoundingRect bounds)
            {
                GeoPoint2D ps = surface.PositionOf(p3d);
                if (surface.IsUPeriodic)
                {
                    double um = (bounds.Left + bounds.Right) / 2;
                    while (Math.Abs(ps.x - um) > Math.Abs(ps.x - surface.UPeriod - um)) ps.x -= surface.UPeriod;
                    while (Math.Abs(ps.x - um) > Math.Abs(ps.x + surface.UPeriod - um)) ps.x += surface.UPeriod;
                }
                if (surface.IsVPeriodic)
                {
                    double vm = (bounds.Bottom + bounds.Top) / 2;
                    while (Math.Abs(ps.y - vm) > Math.Abs(ps.y - surface.VPeriod - vm)) ps.y -= surface.VPeriod;
                    while (Math.Abs(ps.y - vm) > Math.Abs(ps.y + surface.VPeriod - vm)) ps.y += surface.VPeriod;
                }
                return ps;
            }
            static bool SnapToNearestPeriod(ref double curr, double prev, bool isPeriodic, double period)
            {
                if (!isPeriodic || period <= 0) return false;

                double delta = curr - prev;
                double snappedDelta = period * Math.Round(delta / period);

                if (snappedDelta != 0.0)
                {
                    curr -= snappedDelta;
                    return true;
                }
                return false;
            }
            internal static bool FixSurfacePoint2D(ref GeoPoint2D curr, in GeoPoint2D prev, bool isUPeriodic, double uPeriod, bool isVPeriodic, double vPeriod)
            {
                bool changed = false;
                changed |= SnapToNearestPeriod(ref curr.x, prev.x, isUPeriodic, uPeriod);
                changed |= SnapToNearestPeriod(ref curr.y, prev.y, isVPeriodic, vPeriod);
                return changed;
            }

            [Flags]
            internal enum SurfaceFixFlags
            {
                None = 0,
                Surface1 = 1,
                Surface2 = 2
            }
            internal SurfaceFixFlags FixAgainstNeighbour(in SurfacePoint prev, ISurface s1, ISurface s2)
            {
                SurfaceFixFlags changed = SurfaceFixFlags.None;
                if (FixSurfacePoint2D(ref psurface1, prev.psurface1, s1.IsUPeriodic, s1.UPeriod, s1.IsVPeriodic, s1.VPeriod)) changed |= SurfaceFixFlags.Surface1;
                if (FixSurfacePoint2D(ref psurface2, prev.psurface2, s2.IsUPeriodic, s2.UPeriod, s2.IsVPeriodic, s2.VPeriod)) changed |= SurfaceFixFlags.Surface2;
                return changed;
            }

            #region ISerializable Members
            public SurfacePoint(SerializationInfo info, StreamingContext context)
            {
                p3d = (GeoPoint)info.GetValue("P3d", typeof(GeoPoint));
                psurface1 = (GeoPoint2D)info.GetValue("Surface1", typeof(GeoPoint2D));
                psurface2 = (GeoPoint2D)info.GetValue("Surface2", typeof(GeoPoint2D));
            }
            void ISerializable.GetObjectData(SerializationInfo info, StreamingContext context)
            {
                info.AddValue("P3d", p3d);
                info.AddValue("Surface1", psurface1);
                info.AddValue("Surface2", psurface2);
            }
            public SurfacePoint(IJsonReadStruct data)
            {
                p3d = data.GetValue<GeoPoint>();
                psurface1 = data.GetValue<GeoPoint2D>();
                psurface2 = data.GetValue<GeoPoint2D>();
            }

            public void GetObjectData(IJsonWriteData data)
            {
                data.AddValues(p3d, psurface1, psurface2);
            }

            public void SetObjectData(IJsonReadData data)
            {
            }

            #endregion
        }
        ExplicitPCurve3D approxPolynom; // polynom of degree 3 approximating segments between basePoints
#if DEBUG
        static int idcnt = 0;
        int id;
#endif
        [Serializable()]
        public class ProjectedCurve : GeneralCurve2D, ISerializable, IJsonSerialize
        {
            InterpolatedDualSurfaceCurve curve3d;
            bool onSurface1;
            bool reversed;
            BSpline2D approxBSpline = null;
            public ProjectedCurve(InterpolatedDualSurfaceCurve curve3d, bool onSurface1)
            {
                this.curve3d = curve3d;
                this.onSurface1 = onSurface1;
                reversed = false;
                // base.MakeTringulation(); // zuerst mal zum Debuggen, später dynamisch
                // nicht mehr, da ggf. noch periodisch verschoben wird
            }
            public ProjectedCurve(InterpolatedDualSurfaceCurve curve3d, ProjectedCurve toCloneFrom)
            {
                this.curve3d = curve3d;
                this.onSurface1 = toCloneFrom.onSurface1;
                reversed = toCloneFrom.reversed;
            }
            public ProjectedCurve(InterpolatedDualSurfaceCurve curve3d, bool onSurface1, bool reversed)
            {
                this.curve3d = curve3d;
                this.onSurface1 = onSurface1;
                this.reversed = reversed;
            }
            protected override void GetTriangulationBasis(out GeoPoint2D[] points, out GeoVector2D[] directions, out double[] parameters)
            {
#if DEBUG
                curve3d.CheckSurfaceParameters();
#endif
                // it is difficult to find a good solution here: 
                // so we use a couple of points, but could miss some infplection points this way

                int n = 12;
                parameters = new double[n + 1];
                points = new GeoPoint2D[n + 1];
                directions = new GeoVector2D[n + 1];
                for (int i = 0; i < n + 1; i++)
                {
                    parameters[i] = i / (double)n;
                    points[i] = PointAt(parameters[i]);
                    directions[i] = DirectionAt(parameters[i]);
                }
                return;

                // this was the old code

                //List<GeoPoint2D> lpoint = new List<GeoPoint2D>();
                //List<GeoVector2D> ldirections = new List<GeoVector2D>();
                //List<double> lparameters = new List<double>();
                //if (onSurface1)
                //{
                //    double par;
                //    for (int i = 0; i < curve3d.basePoints.Length; ++i)
                //    {
                //        int ii;
                //        if (reversed) ii = curve3d.basePoints.Length - i - 1;
                //        else ii = i;
                //        GeoPoint2D p = curve3d.basePoints[ii].psurface1;
                //        double pm = (double)i / (double)(curve3d.basePoints.Length - 1);
                //        if (reversed) par = 1.0 - pm;
                //        else par = pm;
                //        GeoVector dir = curve3d.DirectionAt(par);
                //        if (reversed) dir.Reverse();
                //        GeoVector u = curve3d.surface1.UDirection(p);
                //        GeoVector v = curve3d.surface1.VDirection(p);
                //        GeoVector n = u ^ v;
                //        Matrix m = DenseMatrix.OfColumnArrays(u, v, n);
                //        Vector s = (Vector)m.Solve(new DenseVector(dir));
                //        if (s.IsValid())
                //        {
                //            if (lpoint.Count > 0)
                //            {
                //                // bei periodischen darf der Abstand nicht zu groß werden
                //                if (curve3d.surface1.IsUPeriodic)
                //                {
                //                    while (p.x - lpoint[lpoint.Count - 1].x > curve3d.surface1.UPeriod / 2) p.x -= curve3d.surface1.UPeriod;
                //                    while (lpoint[lpoint.Count - 1].x - p.x > curve3d.surface1.UPeriod / 2) p.x += curve3d.surface1.UPeriod;
                //                }
                //                if (curve3d.surface1.IsVPeriodic)
                //                {
                //                    while (p.y - lpoint[lpoint.Count - 1].y > curve3d.surface1.VPeriod / 2) p.y -= curve3d.surface1.VPeriod;
                //                    while (lpoint[lpoint.Count - 1].y - p.y > curve3d.surface1.VPeriod / 2) p.y += curve3d.surface1.VPeriod;
                //                }
                //            }
                //            lpoint.Add(p);
                //            if (reversed) lparameters.Add(1.0 - par);
                //            else lparameters.Add(par);
                //            ldirections.Add(new GeoVector2D(s[0], s[1]));
                //        }
                //    }
                //}
                //else
                //{
                //    double par;
                //    for (int i = 0; i < curve3d.basePoints.Length; ++i)
                //    {
                //        int ii;
                //        if (reversed) ii = curve3d.basePoints.Length - i - 1;
                //        else ii = i;
                //        GeoPoint2D p = curve3d.basePoints[ii].psurface2;
                //        double pm = (double)i / (double)(curve3d.basePoints.Length - 1);
                //        if (reversed) par = 1.0 - pm;
                //        else par = pm;
                //        GeoVector dir = curve3d.DirectionAt(par);
                //        if (reversed) dir.Reverse();
                //        GeoVector u = curve3d.surface2.UDirection(p);
                //        GeoVector v = curve3d.surface2.VDirection(p);
                //        GeoVector n = u ^ v;
                //        Matrix m = DenseMatrix.OfColumnArrays(u, v, n);
                //        Vector s = (Vector)m.Solve(new DenseVector(dir));
                //        if (s.IsValid())
                //        {
                //            lpoint.Add(p);
                //            if (reversed) lparameters.Add(1.0 - par);
                //            else lparameters.Add(par);
                //            ldirections.Add(new GeoVector2D(s[0], s[1]));
                //        }
                //    }
                //}
                //points = lpoint.ToArray();
                //directions = ldirections.ToArray();
                //parameters = lparameters.ToArray();
            }
            protected BSpline2D ApproxBSpline
            {
                get
                {
                    if (approxBSpline != null) return approxBSpline;
                    // do not use base.ToBSpline, it will throw a stack overflow
                    GeoPoint2D[] bp;
                    if (onSurface1) bp = curve3d.basePoints.Select(bp => bp.psurface1).ToArray();
                    else bp = curve3d.basePoints.Select(bp => bp.psurface2).ToArray();
                    approxBSpline = new BSpline2D(bp, 3, false);
                    SortedList<double, GeoPoint2D> throughPoints = new SortedList<double, GeoPoint2D>();
                    for (int i = 0; i < bp.Length; i++) throughPoints[curve3d.PositionOf(curve3d.basePoints[i].p3d)] = bp[i];
                    double prec = new BoundingRect(bp).Size * 1e-3;
                    do
                    {
                        SortedList<double, GeoPoint2D> refinedPoints = new SortedList<double, GeoPoint2D>();

                        for (int i = 0; i < throughPoints.Count - 1; i++)
                        {
                            double par = throughPoints.Keys[i] + 0.5 * (throughPoints.Keys[i + 1] - throughPoints.Keys[i]);
                            curve3d.ApproximatePosition(par, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p3d);
                            GeoPoint2D uv = onSurface1 ? uv1 : uv2;
                            double dist = approxBSpline.Distance(uv);
                            // we cannot use the distance of the BSpline to this real curve, since calculating this distance would need the BSpline again
                            if (dist > prec)
                            {
                                refinedPoints[par] = uv;
                            }
                        }
                        if (refinedPoints.Count == 0) break; // all ok
                        foreach (var kvp in refinedPoints) throughPoints[kvp.Key] = kvp.Value;
                        bp = throughPoints.Values.ToArray();
                        approxBSpline = new BSpline2D(bp, 3, false);
                        if (bp.Length > 100) break; // something is wrong
                    } while (true);

                    return approxBSpline;
                }
            }
            public override double GetArea()
            {
                double a = ApproxBSpline.GetArea();
                if (Precision.IsEqual(ApproxBSpline.StartPoint,PointAt(0))==reversed) { }
                if (reversed) return -a;
                else return a;
            }
            public override double GetAreaFromPoint(GeoPoint2D p)
            {
                double a = ApproxBSpline.GetAreaFromPoint(p);
                if (reversed) return -a;
                else return a;
            }
            public override BoundingRect GetExtent()
            {
                return ApproxBSpline.GetExtent();
            }
            public override double Length => ApproxBSpline.Length;
            public override double Sweep => reversed ? -ApproxBSpline.Sweep : ApproxBSpline.Sweep;
            public override GeoVector2D DirectionAt(double par)
            {
                GeoPoint2D uv1, uv2;
                GeoPoint p;
                if (reversed) par = 1.0 - par;
                GeoVector dir;
                curve3d.ApproximatePosition(par, out uv1, out uv2, out p);
                if (curve3d.isTangential)
                {
                    GeoPoint2D pp = onSurface1 ? uv1 : uv2;
                    double pos = ApproxBSpline.PositionOf(pp); // the parameters of the BSpline are not the same as those of this curve
                    return ApproxBSpline.DirectionAt(pos); // no good results with the following in case of tangential
                }
                else
                {
                    dir = curve3d.surface1.GetNormal(uv1).Normalized ^ curve3d.surface2.GetNormal(uv2).Normalized;
                    if (dir.Length < 0.001)
                    {   // no good value for dir, use the BSpline approximation
                        GeoPoint2D pp = onSurface1 ? uv1 : uv2;
                        double pos = ApproxBSpline.PositionOf(pp); // the parameters of the BSpline are not the same as those of this curve
                        return ApproxBSpline.DirectionAt(pos); // no good results with the following in case of tangential
                    }
                }
                if (!curve3d.forwardOriented) dir.Reverse();
                GeoVector2D adir;
                {
                    GeoPoint2D pp = onSurface1 ? uv1 : uv2;
                    double pos = ApproxBSpline.PositionOf(pp); // the parameters of the BSpline are not the same as those of this curve
                    adir = ApproxBSpline.DirectionAt(pos); // no good results with the following in case of tangential
                }
                if (onSurface1)
                {
                    if (reversed) dir.Reverse();
                    GeoVector u = curve3d.surface1.UDirection(uv1);
                    GeoVector v = curve3d.surface1.VDirection(uv1);
                    if (Geometry.DecomposeInPlane(u, v, dir, out double s, out double t))
                    {
                        GeoVector2D res = new GeoVector2D(s, t);
                        res.Length = adir.Length;
                        return res;
                    }
                    else
                    {
                        return adir;
                    }
                }
                else
                {
                    if (reversed) dir.Reverse();
                    GeoVector u = curve3d.surface2.UDirection(uv2);
                    GeoVector v = curve3d.surface2.VDirection(uv2);
                    if (Geometry.DecomposeInPlane(u, v, dir, out double s, out double t))
                    {
                        GeoVector2D res = new GeoVector2D(s, t);
                        res.Length = adir.Length;
                        return res;
                    }
                    else
                    {
                        return adir;
                    }
                }
            }
            public override GeoPoint2D PointAt(double par)
            {
                GeoPoint2D uv1, uv2;
                GeoPoint p;
                if (reversed) par = 1.0 - par;
                curve3d.ApproximatePosition(par, out uv1, out uv2, out p);
                if (onSurface1) return uv1;
                else return uv2;
            }
            public override double PositionOf(GeoPoint2D p)
            {   // in die 3d Situation übersetzen, demit die periodischen Flächen keine Probleme machen
                GeoPoint p3d;
                if (onSurface1) p3d = curve3d.surface1.PointAt(p);
                else p3d = curve3d.surface2.PointAt(p);
                double res = curve3d.PositionOf(p3d);
                if (reversed) return 1 - res;
                else return res;
            }
            public override GeoPoint2D StartPoint
            {
                get
                {
                    if (reversed)
                    {
                        if (onSurface1) return curve3d.basePoints[curve3d.basePoints.Length - 1].psurface1;
                        else return curve3d.basePoints[curve3d.basePoints.Length - 1].psurface2;
                    }
                    else
                    {
                        if (onSurface1) return curve3d.basePoints[0].psurface1;
                        else return curve3d.basePoints[0].psurface2;
                    }
                }
                set
                {   // das wird gebraucht, um kleine Lücken in einem Border zu schließen
                    if (reversed)
                    {
                        if (onSurface1) curve3d.basePoints[curve3d.basePoints.Length - 1].psurface1 = value;
                        else curve3d.basePoints[curve3d.basePoints.Length - 1].psurface2 = value;
                    }
                    else
                    {
                        if (onSurface1) curve3d.basePoints[0].psurface1 = value;
                        else curve3d.basePoints[0].psurface2 = value;
                    }
                    base.StartPoint = value;
                }
            }
            public override GeoPoint2D EndPoint
            {
                get
                {
                    if (reversed)
                    {
                        if (onSurface1) return curve3d.basePoints[0].psurface1;
                        else return curve3d.basePoints[0].psurface2;
                    }
                    else
                    {
                        if (onSurface1) return curve3d.basePoints[curve3d.basePoints.Length - 1].psurface1;
                        else return curve3d.basePoints[curve3d.basePoints.Length - 1].psurface2;
                    }
                }
                set
                {
                    if (reversed)
                    {
                        if (onSurface1) curve3d.basePoints[0].psurface1 = value;
                        else curve3d.basePoints[0].psurface2 = value;
                    }
                    else
                    {
                        if (onSurface1) curve3d.basePoints[curve3d.basePoints.Length - 1].psurface1 = value;
                        else curve3d.basePoints[curve3d.basePoints.Length - 1].psurface2 = value;
                    }
                    base.EndPoint = value;
                }
            }
            public override GeoVector2D StartDirection
            {
                get
                {
                    return DirectionAt(0.0);
                }
            }
            public override GeoVector2D EndDirection
            {
                get
                {
                    return DirectionAt(1.0);
                }
            }
            public override ICurve2D Trim(double StartPos, double EndPos)
            {
                double sp = StartPos;
                double ep = EndPos;
                InterpolatedDualSurfaceCurve clone = curve3d.Clone() as InterpolatedDualSurfaceCurve;
                clone.Trim(sp, ep);
                ProjectedCurve res = new ProjectedCurve(clone, onSurface1);
                res.reversed = reversed;
                res.ClearTriangulation();
                return res;
            }
            public override void Reverse()
            {
                reversed = !reversed;
                base.ClearTriangulation();
            }
            public override ICurve2D Clone()
            {
                ProjectedCurve res = new ProjectedCurve(curve3d.Clone() as InterpolatedDualSurfaceCurve, onSurface1);
                res.reversed = reversed;
                res.ClearTriangulation();
                res.UserData.CloneFrom(UserData);
                return res;
            }
            public override ICurve2D CloneReverse(bool reverse)
            {
                ProjectedCurve res = new ProjectedCurve(curve3d.Clone() as InterpolatedDualSurfaceCurve, onSurface1);
                if (reverse) res.reversed = !reversed;
                else res.reversed = reversed;
                res.ClearTriangulation();
                res.UserData.CloneFrom(UserData);
                return res;
            }
            public override ICurve2D GetModified(ModOp2D m)
            {
                // das geht ja eigentlich nicht, denn diese Kurve ist ja gegeben durch die 3d Kurve, und kann nicht einfach woandershin verschoben werden
                // ABER: nach einer Modifikation der Surface stimmen die basePoints der curve3d nicht mehr. Eigentlich müsste die curve3d das mitbekommen.
                // Die Methode ISurface.ReverseOrientation() verändert nämlich die surface. Die curve3d hier upzudaten ist ein Trick, der zwar nicht schadet, es ist aber nicht
                // die richtige Stelle es zu tun. Wir z.Z. nur bei Face.ReverseOrientation verwendet.
                curve3d.ModifySurfacePoints(onSurface1, m);
                ProjectedCurve res = new ProjectedCurve(curve3d, onSurface1);
                // curve3d.PointAt(0.1111111);
                // curve3d nicht clonen!!
                // if (m.Determinant < 0) res.reversed = !reversed;
                // else 
                res.reversed = reversed;
                res.ClearTriangulation();
                res.UserData.CloneFrom(UserData);
                return res;
            }
            public override bool IsClosed
            {
                get
                {
                    return false; // sollte nie geschlossen sein, oder?
                }
            }
            public override void Move(double x, double y)
            {
                ISurface surface;
                if (onSurface1) surface = curve3d.surface1;
                else surface = curve3d.surface2;
                if (x != 0 && surface.UPeriod != 0.0)
                {
                    if (Math.IEEERemainder(Math.Abs(x), surface.UPeriod) != 0.0) throw new ApplicationException("cannot move ProjectedCurve");
                }
                if (y != 0 && surface.VPeriod != 0.0)
                {
                    if (Math.IEEERemainder(Math.Abs(y), surface.VPeriod) != 0.0) throw new ApplicationException("cannot move ProjectedCurve");
                }
                GeoVector2D diff = new GeoVector2D(x, y);
                if (onSurface1)
                {
                    for (int i = 0; i < curve3d.basePoints.Length; i++)
                    {
                        curve3d.basePoints[i].psurface1 += diff;
                    }
                }
                else
                {
                    for (int i = 0; i < curve3d.basePoints.Length; i++)
                    {
                        curve3d.basePoints[i].psurface2 += diff;
                    }
                }
                curve3d.InvalidateSecondaryData();
                base.ClearTriangulation();
            }
            #region ISerializable Members
            protected ProjectedCurve(SerializationInfo info, StreamingContext context)
                : base(info, context)
            {
                curve3d = info.GetValue("Curve3d", typeof(InterpolatedDualSurfaceCurve)) as InterpolatedDualSurfaceCurve;
                onSurface1 = info.GetBoolean("OnSurface1");
                reversed = info.GetBoolean("Reversed");
            }
            void ISerializable.GetObjectData(SerializationInfo info, StreamingContext context)
            {
                base.GetObjectData(info, context);
                info.AddValue("Curve3d", curve3d);
                info.AddValue("OnSurface1", onSurface1);
                info.AddValue("Reversed", reversed);
            }
            protected ProjectedCurve() { } // needed for IJsonSerialize
            public void GetObjectData(IJsonWriteData data)
            {
                base.JSonGetObjectData(data);
                data.AddProperty("Curve3d", curve3d);
                data.AddProperty("OnSurface1", onSurface1);
                data.AddProperty("Reversed", reversed);
            }

            public void SetObjectData(IJsonReadData data)
            {
                base.JSonSetObjectData(data);
                curve3d = data.GetProperty<InterpolatedDualSurfaceCurve>("Curve3d");
                onSurface1 = data.GetProperty<bool>("OnSurface1");
                reversed = data.GetProperty<bool>("Reversed");
            }

            #endregion
#if DEBUG
            GeoObjectList Debug
            {
                get
                {
                    GeoPoint2D[] pnts = new GeoPoint2D[101];
                    for (int i = 0; i < 101; ++i)
                    {
                        pnts[i] = PointAt(i / 100.0);
                    }
                    Polyline2D pl2d = new Polyline2D(pnts);
                    return new GeoObjectList(pl2d.MakeGeoObject(Plane.XYPlane));
                }
            }
#endif

            public override void Copy(ICurve2D toCopyFrom)
            {
                ProjectedCurve pc = toCopyFrom as ProjectedCurve;
                if (pc != null)
                {
                    curve3d = pc.curve3d;
                    onSurface1 = pc.onSurface1;
                    reversed = pc.reversed;
                }
            }

            internal void ReplaceSurface(ISurface oldSurface, ISurface newSurface)
            {
                curve3d.ReplaceSurface(oldSurface, newSurface);
            }
            internal bool IsOnSurface1
            {
                get
                {
                    return onSurface1;
                }
                set
                {
                    onSurface1 = value;
                }
            }
            internal bool IsReversed
            {
                get
                {
                    return reversed;
                }
            }
            internal void SetCurve3d(InterpolatedDualSurfaceCurve c3d)
            {
                if ((c3d.StartPoint | curve3d.StartPoint) + (c3d.EndPoint | curve3d.EndPoint) > (c3d.StartPoint | curve3d.EndPoint) + (c3d.EndPoint | curve3d.StartPoint)) reversed = !reversed;
                curve3d = c3d;
                // es muss sich hier um eine geometrisch identische Kurve handeln (Richtung?)
                ClearTriangulation();
            }

            public override bool TryPointDeriv2At(double position, out GeoPoint2D point, out GeoVector2D deriv, out GeoVector2D deriv2)
            {
                curve3d.ApproximatePosition(position, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p);
                GeoPoint2D pp = onSurface1 ? uv1 : uv2;
                double pos = ApproxBSpline.PositionOf(pp); // the parameters of the BSpline are not the same as those of this curve
                return ApproxBSpline.TryPointDeriv2At(pos, out point, out deriv, out deriv2);
            }

            internal InterpolatedDualSurfaceCurve Curve3D
            {
                get
                {
                    return curve3d;
                }
            }
        }

        private void ModifySurfacePoints(bool onSurface1, ModOp2D m)
        {
            for (int i = 0; i < basePoints.Length; i++)
            {
                if (onSurface1) basePoints[i].psurface1 = m * basePoints[i].psurface1;
                else basePoints[i].psurface2 = m * basePoints[i].psurface2;
            }
            if (m.Determinant < 0) forwardOriented = !forwardOriented; // die Orientierung der Fläche hat sich umgedreht, damit ist auch das Kreuzprodukt andersrum
            InvalidateSecondaryData();
#if DEBUG
            CheckSurfaceParameters();
#endif
        }

        protected InterpolatedDualSurfaceCurve()
        {
            hashedPositions = new SortedList<double, SurfacePoint>();
#if DEBUG
            id = idcnt++;
#endif
        }
        internal InterpolatedDualSurfaceCurve(ISurface surface1, ISurface surface2, SurfacePoint[] basePoints, bool forwardOriented, ExplicitPCurve3D approxPolynom = null)
            : this()
        {
            // der 1. und der letzte Punkt müssen exakt sein, die anderen nur Näherungswerte, die aber eindeutig zur Fläche führen
            this.surface1 = surface1;
            this.surface2 = surface2;
            this.basePoints = basePoints;
            this.forwardOriented = forwardOriented;
            this.approxPolynom = approxPolynom;
            CheckSurfaceExtents();
            AdjustBasePointsPeriodic();
#if DEBUG
            CheckSurfaceParameters();
#endif
        }
        internal InterpolatedDualSurfaceCurve(ISurface surface1, ISurface surface2, SurfacePoint[] basePoints, bool isTangential = false)
            : this(surface1, BoundingRect.EmptyBoundingRect, surface2, BoundingRect.EmptyBoundingRect, basePoints, isTangential)
        {   // we should always have bounds
        }
        internal InterpolatedDualSurfaceCurve(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, SurfacePoint[] basePoints, bool isTangential = false)
            : this()
        {
            // der 1. und der letzte Punkt müssen exakt sein, die anderen nur Näherungswerte, die aber eindeutig zur Fläche führen
            double dbg = basePoints[0].p3d | basePoints[basePoints.Length - 1].p3d;
            this.surface1 = surface1;
            this.surface2 = surface2;
            this.basePoints = basePoints;
            this.isTangential = isTangential;
            this.bounds1 = bounds1;
            this.bounds2 = bounds2;
            // wierum orientiert?
            // manchmal am Anfang oder Ende tangetial, deshalb besser in der mitte testen
            int n = basePoints.Length / 2; // es müssen mindesten 3 sein
            GeoVector v = surface1.GetNormal(basePoints[n].psurface1) ^ surface2.GetNormal(basePoints[n].psurface2);
            GeoVector v0;
            if (basePoints.Length == 2)
                v0 = basePoints[1].p3d - basePoints[0].p3d;
            else
                v0 = basePoints[n + 1].p3d - basePoints[n - 1].p3d;
            Angle a = new Angle(v, v0);
            forwardOriented = (a.Radian < Math.PI / 2.0);
            CheckSurfaceExtents();
            AdjustBasePointsPeriodic();
            BSpline toUpdateBasepoints = ApproxBSpline;
#if DEBUG
            CheckSurfaceParameters();
#endif
        }
        public InterpolatedDualSurfaceCurve(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, GeoPoint startPoint, GeoPoint endPoint, bool isTangential = false)
            : this()
        {
            // die Bounds dienen dazu bei periodischen Flächen die richtigen Parameterwerte zu finden
            // diese Parameterbereiche sind wichtig, es darf also niemal PositionOf verwendet werden, sonst müssen wir
            // bounds1 und bounds2 speichern, um in den richtigen Bereich zu kommen
            this.surface1 = surface1;
            this.surface2 = surface2;
            this.bounds1 = bounds1;
            this.bounds2 = bounds2;
            this.isTangential = isTangential;
            List<SurfacePoint> points = new List<SurfacePoint>();
            SurfacePoint sp = new SurfacePoint();
            sp.p3d = startPoint;
            sp.psurface1 = sp.PointOnSurface(surface1, bounds1);
            sp.psurface2 = sp.PointOnSurface(surface2, bounds2);
            points.Add(sp);
            SurfacePoint ep = new SurfacePoint();
            ep.p3d = endPoint;
            ep.psurface1 = ep.PointOnSurface(surface1, bounds1);
            ep.psurface2 = ep.PointOnSurface(surface2, bounds2);
            points.Add(ep);
            basePoints = points.ToArray();
            CheckPeriodic();
            points.Clear();
            points.AddRange(basePoints); // damit die periodic Änderungen auch dort wirksam sind
            while (points.Count < 9)
            {
                double maxdist = double.MinValue;
                int ind = -1;
                for (int i = 0; i < points.Count - 1; ++i)
                {
                    double d = points[i].p3d | points[i + 1].p3d;
                    if (d > maxdist)
                    {
                        maxdist = d;
                        ind = i;
                    }
                }
                GeoPoint2D uv1, uv2;
                GeoPoint p;
                ApproximatePosition((ind + 0.5) / (basePoints.Length - 1), out uv1, out uv2, out p);
                //GeoPoint p = new GeoPoint(points[ind].p3d, points[ind + 1].p3d);
                points.Insert(ind + 1, new SurfacePoint(p, uv1, uv2));
                basePoints = points.ToArray(); // damit basePoints für die nächste Runde zu Verfügung steht
            }
            // wierum orientiert?
            // manchmal am Anfang oder Ende tangetial, deshalb besser in der mitte testen
            int n = basePoints.Length / 2; // es müssen mindesten 3 sein
            GeoVector v = surface1.GetNormal(basePoints[n].psurface1) ^ surface2.GetNormal(basePoints[n].psurface2);
            GeoVector v0 = basePoints[n + 1].p3d - basePoints[n - 1].p3d;
            Angle a = new Angle(v, v0);
            forwardOriented = (a.Radian < Math.PI / 2.0);
            CheckSurfaceExtents();
            AdjustBasePointsPeriodic();
            BSpline bsp = ApproxBSpline; // make sure it is created and the basepoints are refined
            hashedPositions.Clear();
            CheckPeriodic();
#if DEBUG
            CheckSurfaceParameters();
#endif
        }
        public InterpolatedDualSurfaceCurve(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, GeoPoint[] pts, List<GeoPoint2D> uvpts1 = null, List<GeoPoint2D> uvpts2 = null, bool isTangential = false)
        : this(surface1, bounds1, surface2, bounds2, new List<GeoPoint>(pts), uvpts1, uvpts2, isTangential)
        {
        }
        public InterpolatedDualSurfaceCurve(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, List<GeoPoint> pts, List<GeoPoint2D> uvpts1 = null, List<GeoPoint2D> uvpts2 = null, bool isTangential = false)
            : this()
        {
            // die Bounds dienen dazu bei periodischen Flächen die richtigen Parameterwerte zu finden
            // diese Parameterbereiche sind wichtig, es darf also niemal PositionOf verwendet werden, sonst müssen wir
            // bounds1 und bounds2 speichern, um in den richtigen Bereich zu kommen
            this.surface1 = surface1;
            this.surface2 = surface2;
            this.bounds1 = bounds1;
            this.bounds2 = bounds2;
            this.isTangential = isTangential;
            List<SurfacePoint> points = new List<SurfacePoint>();
            for (int i = 0; i < pts.Count; ++i)
            {
                SurfacePoint sp = new SurfacePoint();
                sp.p3d = pts[i];
                if (uvpts1 != null)
                    sp.psurface1 = uvpts1[i];
                else
                    sp.psurface1 = sp.PointOnSurface(surface1, bounds1);
                if (uvpts2 != null)
                    sp.psurface2 = uvpts2[i];
                else
                    sp.psurface2 = sp.PointOnSurface(surface2, bounds2);
                points.Add(sp);
            }
            if (points.Count == 2)
            {   // sometimes we have an ambiguous curve here: a half circle on a rotational surface, which could be either way around.
                // since "ApproximatePosition" doesn't care about the u/v bounds, we try a different approach here: choose a fixed u or v curve
                // in the bounds of such a surface and intersect with the other surface. The old approach was bad, more use of bounds1 and bounds2
                // could help further
                List<GeoPoint> intermediatePoints = new List<GeoPoint>();
                if (surface1.IsUPeriodic && Math.Abs(points[0].psurface1.x - points[1].psurface1.x) > surface1.UPeriod / 3.0)
                {   // the curve spans more than 1/3 of a total period
                    ICurve fixedCurve = surface1.FixedU((points[0].psurface1.x + points[1].psurface1.x) / 2.0, bounds1.Bottom, bounds1.Top);
                    surface2.Intersect(fixedCurve, bounds2, out GeoPoint[] ips, out GeoPoint2D[] uvOn2, out double[] uOnCurve3Ds);
                    intermediatePoints.AddRange(ips);
                }
                if (surface1.IsVPeriodic && Math.Abs(points[0].psurface1.y + points[1].psurface1.y) > surface1.VPeriod / 3.0)
                {   // the curve spans more than 1/3 of a total period
                    ICurve fixedCurve = surface1.FixedV((points[0].psurface1.y + points[1].psurface1.y) / 2.0, bounds1.Left, bounds1.Right);
                    surface2.Intersect(fixedCurve, bounds2, out GeoPoint[] ips, out GeoPoint2D[] uvOn2, out double[] uOnCurve3Ds);
                    intermediatePoints.AddRange(ips);
                }
                if (surface2.IsUPeriodic && Math.Abs(points[0].psurface2.x - points[1].psurface2.x) > surface2.UPeriod / 3.0)
                {   // the curve spans more than 1/3 of a total period
                    ICurve fixedCurve = surface2.FixedU((points[0].psurface2.x + points[1].psurface2.x) / 2.0, bounds2.Bottom, bounds2.Top);
                    surface1.Intersect(fixedCurve, bounds1, out GeoPoint[] ips, out GeoPoint2D[] uvOn2, out double[] uOnCurve3Ds);
                    intermediatePoints.AddRange(ips);
                }
                if (surface2.IsVPeriodic && Math.Abs(points[0].psurface2.y + points[1].psurface2.y) > surface2.VPeriod / 3.0)
                {   // the curve spans more than 1/3 of a total period
                    ICurve fixedCurve = surface2.FixedV((points[0].psurface2.y + points[1].psurface2.y) / 2.0, bounds2.Left, bounds2.Right);
                    surface1.Intersect(fixedCurve, bounds1, out GeoPoint[] ips, out GeoPoint2D[] uvOn2, out double[] uOnCurve3Ds);
                    intermediatePoints.AddRange(ips);
                }
                if (intermediatePoints.Count > 0)
                {
                    double mindist = double.MaxValue;
                    int ind = -1;
                    for (int i = 0; i < intermediatePoints.Count; i++)
                    {
                        double d = (intermediatePoints[i] | points[0].p3d) + (intermediatePoints[i] | points[1].p3d);
                        if (d < mindist)
                        {
                            mindist = d;
                            ind = i;
                        }
                    }
                    if (ind >= 0)
                    {
                        SurfacePoint sp = new SurfacePoint();
                        sp.p3d = intermediatePoints[ind];
                        sp.psurface1 = surface1.PositionOf(sp.p3d);
                        sp.psurface2 = surface1.PositionOf(sp.p3d);
                        SurfaceHelper.AdjustPeriodic(surface1, bounds1, ref sp.psurface1);
                        SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref sp.psurface2);
                        points.Insert(1, sp);
                    }
                }
                /* old approach:
                if (!(surface1 is PlaneSurface))
                {
                    ICurve fixedCurve;
                    GeoPoint2D uv0 = points[0].psurface1;
                    GeoPoint2D uv1 = points[1].psurface1;
                    double du = Math.Abs(uv0.x - uv1.x);
                    double dv = Math.Abs(uv0.y - uv1.y);
                    if (du > dv) fixedCurve = surface1.FixedU((uv0.x + uv1.x) / 2.0, bounds1.Bottom, bounds1.Top);
                    else fixedCurve = surface1.FixedV((uv0.y + uv1.y) / 2.0, bounds1.Left, bounds1.Right);
                    surface2.Intersect(fixedCurve, bounds2, out GeoPoint[] ips, out GeoPoint2D[] uvOn2, out double[] uOnCurve3Ds);
                    double mind = double.MaxValue;
                    GeoPoint2D cnt = bounds1.GetCenter();
                    int indFound = -1;
                    for (int i = 0; i < ips.Length; i++)
                    {   // find the best intersection point
                        GeoPoint2D uv = surface1.PositionOf(ips[i]);
                        double d = Math.Abs(uv.x - cnt.x) / bounds1.Width + Math.Abs(uv.y - cnt.y) / bounds1.Height;
                        if (d < mind)
                        {
                            indFound = i;
                            mind = d;
                        }
                    }
                    if (indFound >= 0)
                    {
                        SurfacePoint sp = new SurfacePoint();
                        sp.p3d = ips[indFound];
                        sp.psurface1 = surface1.PositionOf(sp.p3d);
                        sp.psurface2 = uvOn2[indFound];
                        points.Insert(1, sp);
                    }
                }
                else if (!(surface2 is PlaneSurface))
                {
                    ICurve fixedCurve;
                    GeoPoint2D uv0 = points[0].psurface2;
                    GeoPoint2D uv1 = points[1].psurface2;
                    double du = Math.Abs(uv0.x - uv1.x);
                    double dv = Math.Abs(uv0.y - uv1.y);
                    if (du > dv) fixedCurve = surface2.FixedU((uv0.x + uv1.x) / 2.0, bounds2.Bottom, bounds2.Top);
                    else fixedCurve = surface2.FixedV((uv0.y + uv1.y) / 2.0, bounds2.Left, bounds2.Right);
                    surface1.Intersect(fixedCurve, bounds1, out GeoPoint[] ips, out GeoPoint2D[] uvOn1, out double[] uOnCurve3Ds);
                    double mind = double.MaxValue;
                    GeoPoint2D cnt = bounds2.GetCenter();
                    int indFound = -1;
                    for (int i = 0; i < ips.Length; i++)
                    {   // find the best intersection point
                        GeoPoint2D uv = surface2.PositionOf(ips[i]);
                        double d = Math.Abs(uv.x - cnt.x) / bounds2.Width + Math.Abs(uv.y - cnt.y) / bounds2.Height;
                        if (d < mind)
                        {
                            indFound = i;
                            mind = d;
                        }
                    }
                    if (indFound >= 0)
                    {
                        SurfacePoint sp = new SurfacePoint();
                        sp.p3d = ips[indFound];
                        sp.psurface2 = surface2.PositionOf(sp.p3d);
                        sp.psurface1 = uvOn1[indFound];
                        points.Insert(1, sp);
                    }
                } */
            }
            basePoints = points.ToArray();
            // determin the orientation: sometimes both surfaces are tangential at the start or endpoint, so we use an intermedite point when available
            int n = Math.Min(basePoints.Length / 2, basePoints.Length - 2);
            GeoVector v = surface1.GetNormal(basePoints[n].psurface1) ^ surface2.GetNormal(basePoints[n].psurface2);
            GeoVector v0;
            if (n == 0) v0 = basePoints[n + 1].p3d - basePoints[n].p3d; // only two points
            else v0 = basePoints[n + 1].p3d - basePoints[n - 1].p3d;
            Angle a = new Angle(v, v0);
            forwardOriented = (a.Radian < Math.PI / 2.0); // we need this value for ApproximatePosition

            // Recalculate the positions of the inner points, which are sometimes not precise
            for (int i = 1; i < basePoints.Length - 1; ++i)
            {
                GeoPoint2D uv1, uv2; // do not pass out basePoints[i].psurface1 as parameter, since uv1 is manipulated several times inside ApproximatePosition
                ApproximatePosition((double)i / (double)(basePoints.Length - 1), out uv1, out uv2, out basePoints[i].p3d, true);
                basePoints[i].psurface1 = uv1;
                basePoints[i].psurface2 = uv2;
            }
            AdjustPeriodic(bounds1, bounds2);

            points.Clear();
            points.AddRange(basePoints); // damit die periodic Änderungen auch dort wirksam sind
            while (points.Count < 9)
            {
                double maxdist = double.MinValue;
                int ind = -1;
                for (int i = 0; i < points.Count - 1; ++i)
                {
                    double d = points[i].p3d | points[i + 1].p3d;
                    if (d > maxdist)
                    {
                        maxdist = d;
                        ind = i;
                    }
                }
                GeoPoint2D uv1, uv2;
                GeoPoint p;
                ApproximatePosition((ind + 0.5) / (basePoints.Length - 1), out uv1, out uv2, out p);
                hashedPositions.Clear(); // die Werte hier sind unnütz, da die basePoints sich ja immer noch ändern
                approxPolynom = null;
                //GeoPoint p = new GeoPoint(points[ind].p3d, points[ind + 1].p3d);
                points.Insert(ind + 1, new SurfacePoint(p, uv1, uv2));
                basePoints = points.ToArray(); // damit basePoints für die nächste Runde zu Verfügung steht
                v = surface1.GetNormal(basePoints[ind + 1].psurface1) ^ surface2.GetNormal(basePoints[ind + 1].psurface2);
                v0 = basePoints[ind + 2].p3d - basePoints[ind].p3d;
                a = new Angle(v, v0);
                forwardOriented = (a.Radian < Math.PI / 2.0); // recalculate, because for exactly half circles the first result is ambiguous
            }
            double baseLength = 0.0;
            double minLength = double.MaxValue;
            int mlInd = -1;
            for (int i = 1; i < points.Count; i++)
            {
                double d = points[i].p3d | points[i - 1].p3d;
                if (d < minLength)
                {
                    minLength = d;
                    mlInd = i;
                }
                baseLength += d;
            }
            // remove basepoints, which are too close
            double avgLength = baseLength / (points.Count - 1);
            while (minLength < avgLength * 0.1 && (points.Count > 9 || minLength < Precision.eps))
            {
                int toRemove = mlInd;
                if (toRemove == points.Count - 1) --toRemove;
                else if (toRemove == 0) toRemove = 1;
                else if (toRemove > 1 && toRemove < points.Count - 2)
                {
                    double d1 = points[mlInd].p3d | points[mlInd + 1].p3d;
                    double d2 = points[mlInd - 1].p3d | points[mlInd - 2].p3d;
                    if (d1 > d2) --toRemove;
                }
                points.RemoveAt(toRemove);
                minLength = double.MaxValue;
                mlInd = -1;
                for (int i = 1; i < points.Count; i++)
                {
                    double d = points[i].p3d | points[i - 1].p3d;
                    if (d < minLength)
                    {
                        minLength = d;
                        mlInd = i;
                    }
                }
            }
            basePoints = points.ToArray();

#if DEBUG
            //CheckSurfaceParameters();
#endif
            AdjustBasePointsPeriodic();
            BSpline bsp = ApproxBSpline; // make sure it is created and the basepoints are refined
            CheckPeriodic(); // erst nach dieser Schleife, denn ApproximatePosition mach die uv-position evtl. falsch
            CheckSurfaceExtents();
            AdjustBasePointsPeriodic();

#if DEBUG
            CheckSurfaceParameters();
#endif
        }
        [Obsolete("approximated polynom has been replaced by approxBSpline")]
        private void InitApproxPolynom()
        {   // The ExplicitPCurve3D (a polynom) approximates the intersection curve, passing throught the basepoints with the correct direction in the basepoints
            GeoPoint[] epnts = new GeoPoint[basePoints.Length];
            GeoVector[] edirs = new GeoVector[basePoints.Length];
            BoundingRect ext1 = BoundingRect.EmptyBoundingRect;
            BoundingRect ext2 = BoundingRect.EmptyBoundingRect;
            for (int i = 0; i < basePoints.Length; i++)
            {
                epnts[i] = basePoints[i].p3d;
                ext1.MinMax(basePoints[i].psurface1);
                ext2.MinMax(basePoints[i].psurface2);
            }
            ext1.Inflate(ext1.Size * 0.1);
            ext2.Inflate(ext2.Size * 0.1);
            for (int i = 0; i < basePoints.Length; i++)
            {
                edirs[i] = surface1.GetNormal(basePoints[i].psurface1) ^ surface2.GetNormal(basePoints[i].psurface2);
                if (forwardOriented)
                    edirs[i] = surface1.GetNormal(basePoints[i].psurface1) ^ surface2.GetNormal(basePoints[i].psurface2);
                else
                    edirs[i] = -surface1.GetNormal(basePoints[i].psurface1) ^ surface2.GetNormal(basePoints[i].psurface2);
                double l;
                if (i == 0) l = epnts[1] | epnts[0];
                else if (i == basePoints.Length - 1) l = epnts[basePoints.Length - 1] | epnts[basePoints.Length - 2];
                else l = ((epnts[i] | epnts[i - 1]) + (epnts[i] | epnts[i + 1])) / 2.0;
                if (Precision.IsNullVector(edirs[i]))
                {   // this is a touching point. We assume, that touching points (if there are any) are always basepoints.
                    // To get a good tangential direction of the curve in that point we construct a plane based by the normal vector and the connection to the next basepoint
                    // this plane intersects the surfaces in a curve. We use the tangent verctor of this curve as the tangent vector 
                    GeoVector dir, dir1 = GeoVector.NullVector, dir2 = GeoVector.NullVector;
                    if (i == 0) dir = epnts[i + 1] - epnts[i];
                    else if (i == basePoints.Length - 1) dir = epnts[i] - epnts[i - 1];
                    else dir = (epnts[i + 1] - epnts[i - 1]);
                    Plane pln = new Plane(epnts[i], surface1.GetNormal(basePoints[i].psurface1), dir);
                    IDualSurfaceCurve[] dsc1 = surface1.GetPlaneIntersection(new PlaneSurface(pln), ext1.Left, ext1.Right, ext1.Bottom, ext1.Top, 0.0);
                    pln = new Plane(epnts[i], surface2.GetNormal(basePoints[i].psurface2), dir); // this is almost the same plane as above
                    IDualSurfaceCurve[] dsc2 = surface2.GetPlaneIntersection(new PlaneSurface(pln), ext2.Left, ext2.Right, ext2.Bottom, ext2.Top, 0.0);
                    for (int j = 0; j < dsc1.Length; j++)
                    {
                        double pos = dsc1[j].Curve3D.PositionOf(epnts[i]);
                        if (pos >= -1e-6 && pos <= 1 + 1e-6)
                        {
                            dir1 = dsc1[j].Curve3D.DirectionAt(pos);
                            if (dir1 * dir < 0) dir1 = -dir1;
                        }
                    }
                    for (int j = 0; j < dsc2.Length; j++)
                    {
                        double pos = dsc2[j].Curve3D.PositionOf(epnts[i]);
                        if (pos >= -1e-6 && pos <= 1 + 1e-6)
                        {
                            dir2 = dsc2[j].Curve3D.DirectionAt(pos);
                            if (dir2 * dir < 0) dir2 = -dir2;
                        }
                    }
                    GeoVector dir12 = dir1 + dir2; // dir1 and dir2 should be very similar, if one of them is the nullvector, there is no problem
                    if (!dir12.IsNullVector()) edirs[i] = dir12;
                    else edirs[i] = dir;
                }
                edirs[i].Length = l * basePoints.Length;
            }
            approxPolynom = ExplicitPCurve3D.FromPointsDirections(epnts, edirs);
        }
        internal void CheckSurfaceExtents()
        {
            if (bounds1.IsEmpty() && surface1 is ISurfaceImpl simpl1)
            {
                if (simpl1.usedArea.IsEmpty() || simpl1.usedArea.IsInfinite)
                {
                    BoundingRect ext = BoundingRect.EmptyBoundingRect;
                    for (int i = 0; i < basePoints.Length; i++)
                    {
                        ext.MinMax(basePoints[i].psurface1);
                    }
                    simpl1.usedArea = ext;
                }
                bounds1 = simpl1.usedArea;
            }
            if (bounds2.IsEmpty() && surface2 is ISurfaceImpl simpl2)
            {
                if (simpl2.usedArea.IsEmpty() || simpl2.usedArea.IsInfinite)
                {
                    BoundingRect ext = BoundingRect.EmptyBoundingRect;
                    for (int i = 0; i < basePoints.Length; i++)
                    {
                        ext.MinMax(basePoints[i].psurface2);
                    }
                    simpl2.usedArea = ext;
                }
                bounds2 = simpl2.usedArea;
            }

        }
#if DEBUG
        internal void CheckSurfaceParameters()
        {   // check auf parameterfehler im 2d
            for (int i = 0; i < basePoints.Length - 1; i++)
            {
                if ((surface1.IsUPeriodic && Math.Abs(basePoints[i + 1].psurface1.x - basePoints[i].psurface1.x) > 1) ||
                    (surface1.IsVPeriodic && Math.Abs(basePoints[i + 1].psurface1.y - basePoints[i].psurface1.y) > 1) ||
                    (surface2.IsUPeriodic && Math.Abs(basePoints[i + 1].psurface2.x - basePoints[i].psurface2.x) > 1) ||
                    (surface2.IsVPeriodic && Math.Abs(basePoints[i + 1].psurface2.y - basePoints[i].psurface2.y) > 1))
                { }
            }
            if (hashedPositions.Any())
            {
                SurfacePoint sfp = hashedPositions.First().Value;
                foreach (var item in hashedPositions)
                {
                    if ((surface1.IsUPeriodic && Math.Abs(sfp.psurface1.x - item.Value.psurface1.x) > 1) ||
                        (surface1.IsVPeriodic && Math.Abs(sfp.psurface1.y - item.Value.psurface1.y) > 1) ||
                        (surface2.IsUPeriodic && Math.Abs(sfp.psurface2.x - item.Value.psurface2.x) > 1) ||
                        (surface2.IsVPeriodic && Math.Abs(sfp.psurface2.y - item.Value.psurface2.y) > 1))
                    { }
                    sfp = item.Value;
                }
            }
            if ((basePoints[basePoints.Length - 1].p3d | basePoints[basePoints.Length - 2].p3d) == 0.0 || (basePoints[0].p3d | basePoints[1].p3d) == 0.0)
            {

            }
            double d = 0.0;
            for (int i = 1; i < basePoints.Length; i++)
            {
                d += basePoints[i].psurface1 | basePoints[i - 1].psurface1;
            }
            d /= basePoints.Length - 1;
            for (int i = 1; i < basePoints.Length; i++)
            {
                if ((basePoints[i].psurface1 | basePoints[i - 1].psurface1) > 5 * d)
                {
                }
            }
            for (int i = 1; i < basePoints.Length; i++)
            {
                d += basePoints[i].psurface2 | basePoints[i - 1].psurface2;
            }
            d /= basePoints.Length - 1;
            for (int i = 1; i < basePoints.Length; i++)
            {
                if ((basePoints[i].psurface2 | basePoints[i - 1].psurface2) > 5 * d)
                {
                }
            }
            Surface2.GetNaturalBounds(out double umin, out double umax, out double vmin, out double vmax);
            for (int i = 0; i < basePoints.Length; i++)
            {
                if ((surface1.PointAt(basePoints[i].psurface1) | basePoints[i].p3d) > 0.1)
                {
                    return;
                }
                if ((surface2.PointAt(basePoints[i].psurface2) | basePoints[i].p3d) > 0.1)
                {
                    return;
                }
                if (basePoints[i].psurface2.x < umin || basePoints[i].psurface2.x > umax)
                {

                }
            }
        }
#endif
        internal void Repair(BoundingRect bounds1, BoundingRect bounds2)
        {
            BoundingCube ext = BoundingCube.EmptyBoundingCube;
            for (int i = 0; i < basePoints.Length; i++) ext.MinMax(basePoints[i].p3d);
            double eps = ext.Size * 1e-5;
            bool needsRepair = false;
            for (int i = 0; i < basePoints.Length; i++)
            {
                if ((surface1.PointAt(basePoints[i].psurface1) | basePoints[i].p3d) > eps)
                {
                    needsRepair = true;
                    break;
                }
                if ((surface2.PointAt(basePoints[i].psurface2) | basePoints[i].p3d) > eps)
                {
                    needsRepair = true;
                    break;
                }
            }
            if (needsRepair)
            {
                RecalcSurfacePoints(bounds1, bounds2);
            }
        }
        private void CheckPeriodic()
        {
            for (int i = 1; i < basePoints.Length; i++)
            {
                AdjustPeriodic(ref basePoints[i].psurface1, true, i - 1);
                AdjustPeriodic(ref basePoints[i].psurface2, false, i - 1);
            }
            //for (int i = 1; i < basePoints.Length; i++)
            //{
            //    double d = basePoints[i].psurface1 | basePoints[i - 1].psurface1;
            //    if (surface1.IsUPeriodic)
            //    {
            //        GeoPoint2D uv1 = basePoints[i].psurface1;
            //        uv1.x = uv1.x + surface1.UPeriod;
            //        double d1 = uv1 | basePoints[i - 1].psurface1;
            //        if (d1 < d)
            //        {
            //            d = d1;
            //            basePoints[i].psurface1 = uv1;
            //        }
            //        uv1.x = basePoints[i].psurface1.x - surface1.UPeriod;
            //        d1 = uv1 | basePoints[i - 1].psurface1;
            //        if (d1 < d)
            //        {
            //            d = d1;
            //            basePoints[i].psurface1 = uv1;
            //        }
            //    }
            //    if (surface1.IsVPeriodic)
            //    {
            //        GeoPoint2D uv1 = basePoints[i].psurface1;
            //        uv1.y = uv1.y + surface1.VPeriod;
            //        double d1 = uv1 | basePoints[i - 1].psurface1;
            //        if (d1 < d)
            //        {
            //            d = d1;
            //            basePoints[i].psurface1 = uv1;
            //        }
            //        uv1.y = basePoints[i].psurface1.y - surface1.VPeriod;
            //        d1 = uv1 | basePoints[i - 1].psurface1;
            //        if (d1 < d)
            //        {
            //            d = d1;
            //            basePoints[i].psurface1 = uv1;
            //        }
            //    }
            //    d = basePoints[i].psurface2 | basePoints[i - 1].psurface2;
            //    if (surface2.IsUPeriodic)
            //    {
            //        GeoPoint2D uv1 = basePoints[i].psurface2;
            //        uv1.x = uv1.x + surface2.UPeriod;
            //        double d1 = uv1 | basePoints[i - 1].psurface2;
            //        if (d1 < d)
            //        {
            //            d = d1;
            //            basePoints[i].psurface2 = uv1;
            //        }
            //        uv1.x = basePoints[i].psurface2.x - surface2.UPeriod;
            //        d1 = uv1 | basePoints[i - 1].psurface2;
            //        if (d1 < d)
            //        {
            //            d = d1;
            //            basePoints[i].psurface2 = uv1;
            //        }
            //    }
            //    if (surface2.IsVPeriodic)
            //    {
            //        GeoPoint2D uv1 = basePoints[i].psurface2;
            //        uv1.y = uv1.y + surface2.VPeriod;
            //        double d1 = uv1 | basePoints[i - 1].psurface2;
            //        if (d1 < d)
            //        {
            //            d = d1;
            //            basePoints[i].psurface2 = uv1;
            //        }
            //        uv1.y = basePoints[i].psurface2.y - surface2.VPeriod;
            //        d1 = uv1 | basePoints[i - 1].psurface2;
            //        if (d1 < d)
            //        {
            //            d = d1;
            //            basePoints[i].psurface2 = uv1;
            //        }
            //    }
            //}
        }
        internal void AdjustPeriodic(ref GeoPoint2D uv1, ref GeoPoint2D uv2)
        {
            if (bounds1.IsEmpty() || bounds1.IsInfinite || bounds1.IsInvalid()) bounds1 = (surface1 as ISurfaceImpl).usedArea;
            if (bounds2.IsEmpty() || bounds2.IsInfinite || bounds2.IsInvalid()) bounds2 = (surface2 as ISurfaceImpl).usedArea;
            SurfaceHelper.AdjustPeriodic(surface1, bounds1, ref uv1);
            SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref uv2);
        }
        internal void AdjustBasePointsPeriodic()
        {
            if (!surface1.IsUPeriodic && !surface1.IsVPeriodic && !surface2.IsUPeriodic && !surface2.IsVPeriodic) return;
            SurfaceHelper.AdjustPeriodic(surface1, bounds1, ref basePoints[0].psurface1); // make sure the first point is in the correct periodic range
            SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref basePoints[0].psurface2);
            SurfaceFixFlags uvfixed = SurfaceFixFlags.None;
            for (int i = 1; i < basePoints.Length; i++)
            {   // set all points periodicity relative to the previous one
                uvfixed |= basePoints[i].FixAgainstNeighbour(basePoints[i - 1], surface1, surface2);
            }
            // if we had to change points on a surface, we shift all points on that surface so that the average position is in the center of the bounds
            if (uvfixed == SurfaceFixFlags.Surface1)
            {
                double u = basePoints.Sum(b => b.psurface1.x) / basePoints.Length;
                double v = basePoints.Sum(b => b.psurface1.y) / basePoints.Length;
                double du = u - (bounds1.Left + bounds1.Right) / 2.0;
                double dv = v - (bounds1.Bottom + bounds1.Top) / 2.0;
                du = surface1.IsUPeriodic ? surface1.UPeriod * Math.Round(du / surface1.UPeriod) : 0.0;
                dv = surface1.IsVPeriodic ? surface1.VPeriod * Math.Round(du / surface1.VPeriod) : 0.0;
                if (du != 0.0 || dv != 0.0)
                {
                    for (int i = 0; i < basePoints.Length; i++)
                    {
                        basePoints[i].psurface1.x -= du;
                        basePoints[i].psurface1.y -= dv;
                    }
                }
            }
            if (uvfixed == SurfaceFixFlags.Surface2)
            {
                double u = basePoints.Sum(b => b.psurface2.x) / basePoints.Length;
                double v = basePoints.Sum(b => b.psurface2.y) / basePoints.Length;
                double du = u - (bounds2.Left + bounds2.Right) / 2.0;
                double dv = v - (bounds2.Bottom + bounds2.Top) / 2.0;
                du = surface2.IsUPeriodic ? surface2.UPeriod * Math.Round(du / surface2.UPeriod) : 0.0;
                dv = surface2.IsVPeriodic ? surface2.VPeriod * Math.Round(du / surface2.VPeriod) : 0.0;
                if (du != 0.0 || dv != 0.0)
                {
                    for (int i = 0; i < basePoints.Length; i++)
                    {
                        basePoints[i].psurface1.x -= du;
                        basePoints[i].psurface1.y -= dv;
                    }
                }
            }
        }
        internal void AdjustPeriodic(BoundingRect b1, BoundingRect b2)
        {   // we need to consider the whole curve, not just individual points, because the bounds may be too narrow and some points fall outside
            // we expect that the 2d points are in a row and have no periodic jumps
            GeoPoint2D[] p2d = basePoints.Select(b => b.psurface1).ToArray();
            SurfaceHelper.AdjustPeriodic(surface1, b1, p2d);
            for (int i = 0; i < basePoints.Length; i++) basePoints[i].psurface1 = p2d[i];
            p2d = basePoints.Select(b => b.psurface2).ToArray();
            SurfaceHelper.AdjustPeriodic(surface2, b2, p2d);
            for (int i = 0; i < basePoints.Length; i++) basePoints[i].psurface2 = p2d[i];
        }
        private void AdjustPeriodic(ref SurfacePoint toAdjust, int ind)
        {
            AdjustPeriodic(ref toAdjust.psurface1, true, ind);
            AdjustPeriodic(ref toAdjust.psurface2, false, ind);
        }
        private void AdjustPeriodic(ref GeoPoint2D toAdjust, bool onSurface1, int ind)
        {
            if (onSurface1)
            {
                if (surface1.IsUPeriodic)
                {
                    while (toAdjust.x - basePoints[ind].psurface1.x > surface1.UPeriod / 2) toAdjust.x -= surface1.UPeriod;
                    while (basePoints[ind].psurface1.x - toAdjust.x > surface1.UPeriod / 2) toAdjust.x += surface1.UPeriod;
                }
                if (surface1.IsVPeriodic)
                {
                    while (toAdjust.y - basePoints[ind].psurface1.y > surface1.VPeriod / 2) toAdjust.y -= surface1.VPeriod;
                    while (basePoints[ind].psurface1.y - toAdjust.y > surface1.VPeriod / 2) toAdjust.y += surface1.VPeriod;
                }
            }
            else
            {
                if (surface2.IsUPeriodic)
                {
                    while (toAdjust.x - basePoints[ind].psurface2.x > surface2.UPeriod / 2) toAdjust.x -= surface2.UPeriod;
                    while (basePoints[ind].psurface2.x - toAdjust.x > surface2.UPeriod / 2) toAdjust.x += surface2.UPeriod;
                }
                if (surface2.IsVPeriodic)
                {
                    while (toAdjust.y - basePoints[ind].psurface2.y > surface2.VPeriod / 2) toAdjust.y -= surface2.VPeriod;
                    while (basePoints[ind].psurface2.y - toAdjust.y > surface2.VPeriod / 2) toAdjust.y += surface2.VPeriod;
                }
            }
        }
        internal DualSurfaceCurve ToDualSurfaceCurve()
        {
            return new DualSurfaceCurve(this, surface1, new ProjectedCurve(this, true), surface2, new ProjectedCurve(this, false));
        }
        private SurfacePoint GetPoint(GeoPoint fromHere)
        {   // liefert einen Punkt möglichst nahe bei fromHere
            GeoPoint2D uv1 = surface1.PositionOf(fromHere);
            GeoPoint2D uv2 = surface2.PositionOf(fromHere);
            GeoPoint p1 = surface1.PointAt(uv1);
            GeoPoint p2 = surface2.PointAt(uv2);
            while ((p1 | p2) > Precision.eps) // zu unsichere Bedingung, vor allem wenns tangential wird...
            {
                GeoVector normal = surface1.GetNormal(uv1) ^ surface2.GetNormal(uv2);
                GeoPoint location = new GeoPoint(surface1.PointAt(uv1), surface1.PointAt(uv2));
                Plane pln = new Plane(location, normal); // in dieser Ebene suchen wir jetzt den Schnittpunkt
                // nach dem Tangentenverfahren, da wir ja schon ganz nah sind, oder?
                // p1,du1,dv1 und p2,du2,dv2 sind die beiden Tangentialebenen, p3,du3,dv3 die senkerechte Ebene
                // daraus ergeben sich 6 Gleichungen mit 6 unbekannten
                // p1+u1*du1+v1*dv1 = p3+u3*du3+v3*dv3
                // p2+u2*du2+v2*dv2 = p3+u3*du3+v3*dv3
                // umgeformt: (u3 und v3 interessieren nicht, deshalb Vorzeichen egal
                // u1*du1 + v1*dv1 + 0      + 0      - u3*du3 - v3*dv3 = p3-p1
                // 0      + 0      + u2*du2 + v2*dv2 - u3*du3 - v3*dv3 = p3-p1
                GeoVector du1 = surface1.UDirection(uv1);
                GeoVector dv1 = surface1.VDirection(uv1);
                GeoVector du2 = surface2.UDirection(uv2);
                GeoVector dv2 = surface2.VDirection(uv2);
                GeoVector du3 = pln.DirectionX;
                GeoVector dv3 = pln.DirectionY;
#if DEBUG
                PlaneSurface pls1 = new PlaneSurface(new Plane(p1, du1, dv1));
                SimpleShape ss1 = new SimpleShape(new BoundingRect(-1, -1, 1, 1));
                Face fc1 = Face.MakeFace(pls1, ss1);
                PlaneSurface pls2 = new PlaneSurface(new Plane(p2, du2, dv2));
                SimpleShape ss2 = new SimpleShape(new BoundingRect(-1, -1, 1, 1));
                Face fc2 = Face.MakeFace(pls2, ss2);
                PlaneSurface pls3 = new PlaneSurface(new Plane(location, du3, dv3));
                SimpleShape ss3 = new SimpleShape(new BoundingRect(-1, -1, 1, 1));
                Face fc3 = Face.MakeFace(pls3, ss3);
                DebuggerContainer dc = new DebuggerContainer();
                dc.Add(fc1);
                dc.Add(fc2);
                dc.Add(fc3);
#endif
                Matrix m = DenseMatrix.OfArray(new double[,]
                {
                    {du1.x,dv1.x,0,0,du3.x,dv3.x},
                    {du1.y,dv1.y,0,0,du3.y,dv3.y},
                    {du1.z,dv1.z,0,0,du3.z,dv3.z},
                    {0,0,du2.x,dv2.x,du3.x,dv3.x},
                    {0,0,du2.y,dv2.y,du3.y,dv3.y},
                    {0,0,du2.z,dv2.z,du3.z,dv3.z},
                });
                Matrix s = (Matrix)m.Solve(DenseMatrix.OfArray(new double[,] { { location.x - p1.x }, { location.y - p1.y }, { location.z - p1.z } ,
                { location.x - p2.x }, { location.y - p2.y }, { location.z - p2.z } }));
                uv1.x += s[0, 0];
                uv1.y += s[1, 0];
                uv2.x += s[2, 0];
                uv2.y += s[3, 0];
                p1 = surface1.PointAt(uv1);
                p2 = surface2.PointAt(uv2);
            }
            return new SurfacePoint(new GeoPoint(p1, p2), uv1, uv2);
        }
        private SurfacePoint[] Interpolate(SurfacePoint sp1, SurfacePoint sp2)
        {
            double du1 = sp1.psurface1.x - sp2.psurface1.x;
            double dv1 = sp1.psurface1.y - sp2.psurface1.y;
            double du2 = sp1.psurface2.x - sp2.psurface2.x;
            double dv2 = sp1.psurface2.y - sp2.psurface2.y;
            // wir suchen die Fläche, in dessen u/v system die Linie am nächsten zu einer Achse ist
            double f1, f2;
            if (Math.Abs(du1) > Math.Abs(dv1)) f1 = Math.Abs(dv1) / Math.Abs(du1);
            else f1 = Math.Abs(du1) / Math.Abs(dv1);
            if (Math.Abs(du2) > Math.Abs(dv2)) f2 = Math.Abs(dv2) / Math.Abs(du2);
            else f2 = Math.Abs(du2) / Math.Abs(dv2);
            ISurface withCurve, opposite;
            if (f1 < f2)
            {   // erste Fläche ist besser
                withCurve = surface1;
                opposite = surface2;
            }
            else
            {
                withCurve = surface2;
                opposite = surface1;
            }
            return null;
        }
        public ISurface Surface1
        {
            get
            {
                return surface1;
            }
            internal set
            {   // es muss sich um einen Clone handeln
                surface1 = value;
            }
        }
        public ISurface Surface2
        {
            get
            {
                return surface2;
            }
            internal set
            {   // es muss sich um einen Clone handeln
                surface2 = value;
            }
        }
        public ICurve2D CurveOnSurface1
        {
            get
            {   // evtl Cache?
                return new ProjectedCurve(this, true);
            }
        }
        public ICurve2D CurveOnSurface2
        {
            get
            {
                return new ProjectedCurve(this, false);
            }
        }
        protected override void InvalidateSecondaryData()
        {
            base.InvalidateSecondaryData();
            hashedPositions.Clear();
            approxPolynom = null;
            approxBSpline = null;
#if DEBUG
            CheckSurfaceParameters();
#endif
        }
        internal void ReplaceSurface(ISurface oldSurface, ISurface newSurface)
        {   // die beiden surfaces müssen geometrisch identisch sein
            if (surface1 == oldSurface) surface1 = newSurface;
            if (surface2 == oldSurface) surface2 = newSurface;
        }
        internal void ReplaceSurface(ISurface oldSurface, ISurface newSurface, ModOp2D oldToNew)
        {   // die beiden surfaces müssen geometrisch identisch sein
            if (surface1 == oldSurface)
            {
                surface1 = newSurface;
                ModifySurfacePoints(true, oldToNew);
            }
            else if (surface2 == oldSurface)
            {
                surface2 = newSurface;
                ModifySurfacePoints(false, oldToNew);
            }
            else if (surface1.SameGeometry((surface1 as ISurfaceImpl).usedArea, oldSurface, (oldSurface as ISurfaceImpl).usedArea, Precision.eps, out ModOp2D dumy))
            {
                surface1 = newSurface;
                ModifySurfacePoints(true, oldToNew);
            }
            else if (surface2.SameGeometry((surface2 as ISurfaceImpl).usedArea, oldSurface, (oldSurface as ISurfaceImpl).usedArea, Precision.eps, out dumy))
            {
                surface2 = newSurface;
                ModifySurfacePoints(false, oldToNew);
            }
            else
            {

            }
        }
        internal InterpolatedDualSurfaceCurve CloneTrimmed(double startPos, double endPos, ProjectedCurve c1, ProjectedCurve c2, out ICurve2D c1trimmed, out ICurve2D c2trimmed)
        {
            InterpolatedDualSurfaceCurve res = new InterpolatedDualSurfaceCurve(surface1, surface2, basePoints.Clone() as SurfacePoint[], forwardOriented);
            res.Trim(startPos, endPos);
            c1trimmed = new ProjectedCurve(res, c1);
            c2trimmed = new ProjectedCurve(res, c2);
            return res;
        }
        public BSpline ToBSpline(double precision)
        {
            List<GeoPoint> throughPoints = new List<GeoPoint>();
            for (int i = 0; i < basePoints.Length; i++)
            {
                throughPoints.Add(basePoints[i].p3d);
            }
            BSpline res = BSpline.Construct();
            res.ThroughPoints(throughPoints.ToArray(), 3, false);
            // hier noch Genauikeit überprüfen und throughpoints vermehren
            return res;
        }
        internal GeoPoint[] BasePoints
        {
            get
            {
                GeoPoint[] res = new GeoPoint[basePoints.Length];
                for (int i = 0; i < basePoints.Length; i++)
                {
                    res[i] = basePoints[i].p3d;
                }
                return res;
            }
        }

#if DEBUG
        new DebuggerContainer Debug
        {
            get
            {
                DebuggerContainer res = new DebuggerContainer();
                Polyline pl = Polyline.Construct();
                GeoPoint[] pnts = new GeoPoint[basePoints.Length];
                for (int i = 0; i < basePoints.Length; ++i)
                {
                    pnts[i] = basePoints[i].p3d;
                    GeoVector u = surface1.UDirection(basePoints[i].psurface1);
                    GeoVector v = surface1.VDirection(basePoints[i].psurface1);
                    PlaneSurface pls = new PlaneSurface(pnts[i], u, v, u ^ v);
                    Face fc = Face.MakeFace(pls, new SimpleShape(new BoundingRect(-1, -1, 1, 1)));
                    fc.ColorDef = new CADability.Attribute.ColorDef("Surface1", System.Drawing.Color.Red);
                    res.Add(fc);
                    u = surface2.UDirection(basePoints[i].psurface2);
                    v = surface2.VDirection(basePoints[i].psurface2);
                    pls = new PlaneSurface(pnts[i], u, v, u ^ v);
                    fc = Face.MakeFace(pls, new SimpleShape(new BoundingRect(-1, -1, 1, 1)));
                    fc.ColorDef = new CADability.Attribute.ColorDef("Surface1", System.Drawing.Color.Green);
                    res.Add(fc);
                }
                pl.SetPoints(pnts, false);
                double size = pl.GetBoundingCube().Size / 10;
                res.Add(pl);
                return res;
            }
        }
        internal IGeoObject Debug100Points
        {
            get
            {
                GeoPoint[] dbgpnts = new CADability.GeoPoint[100];
                for (int i = 0; i < dbgpnts.Length; i++)
                {
                    dbgpnts[i] = PointAt(i / (double)(dbgpnts.Length - 1));
                }
                Polyline dbgpl = Polyline.Construct();
                dbgpl.SetPoints(dbgpnts, false);
                return dbgpl;
            }
        }
        internal GeoObjectList DebugOrientation
        {
            get
            {
                GeoObjectList res = new GeoObjectList();
                Polyline pl = Debug100Points as Polyline;
                res.Add(pl);
                double l = pl.Length / 50;
                for (int i = 0; i < 100; i++)
                {
                    GeoVector dir = (this as IOrientation).OrientationAt(i / 99.0);
                    Line line = Line.TwoPoints(pl.GetPoint(i), pl.GetPoint(i) + l * dir.Normalized);
                    res.Add(line);
                }
                return res;
            }
        }
        GeoObjectList DebugBasePoints
        {
            get
            {
                GeoObjectList res = new GeoObjectList();
                Polyline pl = Polyline.Construct();
                GeoPoint[] pnts = new GeoPoint[basePoints.Length];
                for (int i = 0; i < basePoints.Length; ++i)
                {
                    pnts[i] = basePoints[i].p3d;
                }
                pl.SetPoints(pnts, false);
                pl.ColorDef = new Attribute.ColorDef("org", System.Drawing.Color.Red);
                res.Add(pl);
                pl = Polyline.Construct();
                pnts = new GeoPoint[basePoints.Length];
                for (int i = 0; i < basePoints.Length; ++i)
                {
                    pnts[i] = surface1.PointAt(basePoints[i].psurface1);
                }
                pl.SetPoints(pnts, false);
                pl.ColorDef = new Attribute.ColorDef("surf1", System.Drawing.Color.Green);
                res.Add(pl);
                pl = Polyline.Construct();
                pnts = new GeoPoint[basePoints.Length];
                for (int i = 0; i < basePoints.Length; ++i)
                {
                    pnts[i] = surface2.PointAt(basePoints[i].psurface2);
                }
                pl.SetPoints(pnts, false);
                pl.ColorDef = new Attribute.ColorDef("surf1", System.Drawing.Color.Blue);
                res.Add(pl);
                return res;
            }
        }
        IGeoObject DebugHashedCurve1
        {
            get
            {
                SortedList<double, SurfacePoint> sl = new SortedList<double, SurfacePoint>(hashedPositions);
                List<GeoPoint2D> pnts = new List<GeoPoint2D>();
                foreach (SurfacePoint sp in sl.Values)
                {
                    pnts.Add(sp.psurface1);
                }
                Polyline2D pl2d = new Polyline2D(pnts.ToArray());
                return pl2d.MakeGeoObject(Plane.XYPlane);
            }
        }
        IGeoObject DebugHashedCurve2
        {
            get
            {
                SortedList<double, SurfacePoint> sl = new SortedList<double, SurfacePoint>(hashedPositions);
                List<GeoPoint2D> pnts = new List<GeoPoint2D>();
                foreach (SurfacePoint sp in sl.Values)
                {
                    pnts.Add(sp.psurface2);
                }
                Polyline2D pl2d = new Polyline2D(pnts.ToArray());
                return pl2d.MakeGeoObject(Plane.XYPlane);
            }
        }
        GeoObjectList DebugSurface
        {
            get
            {
                GeoObjectList res = new GeoObjectList();
                BoundingRect bnd = BoundingRect.EmptyBoundingRect;
                for (int i = 0; i < basePoints.Length; i++)
                {
                    bnd.MinMax(basePoints[i].psurface1);
                }
                bnd.Inflate(1.0);
                res.Add(Face.MakeFace(surface1, new SimpleShape(Border.MakeRectangle(bnd))));
                bnd = BoundingRect.EmptyBoundingRect;
                for (int i = 0; i < basePoints.Length; i++)
                {
                    bnd.MinMax(basePoints[i].psurface2);
                }
                bnd.Inflate(1.0);
                res.Add(Face.MakeFace(surface2, new SimpleShape(Border.MakeRectangle(bnd))));
                return res;
            }
        }
#endif
        #region IGeoObject override
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.IGeoObjectImpl.GetBoundingCube ()"/>
        /// </summary>
        /// <returns></returns>
        public override BoundingCube GetBoundingCube()
        {
            BoundingCube res = new BoundingCube();
            for (int i = 0; i < basePoints.Length; ++i)
            {
                res.MinMax(basePoints[i].p3d);
            }
            return res;
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.IGeoObjectImpl.Modify (ModOp)"/>
        /// </summary>
        /// <param name="m"></param>
        public override void Modify(ModOp m)
        {
            surface1 = surface1.GetModified(m);
            surface2 = surface2.GetModified(m);
            // nicht:
            // surface2.Modify(m);
            // denn man weiß nicht von wem die surface noch verwendet wird
            for (int i = 0; i < basePoints.Length; ++i)
            {
                basePoints[i].p3d = m * basePoints[i].p3d;
            }
            InvalidateSecondaryData();
            if (approxPolynom != null) approxPolynom = approxPolynom.GetModified(m);
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.IGeoObjectImpl.GetExtent (double)"/>
        /// </summary>
        /// <param name="precision"></param>
        /// <returns></returns>
        public override BoundingCube GetExtent(double precision)
        {
            BoundingCube res = BoundingCube.EmptyBoundingCube;
            for (int i = 0; i < basePoints.Length; ++i)
            {
                res.MinMax(basePoints[i].p3d);
            }
            return res;
        }
        public BoundingRect GetBoundingRect(bool onSurface1)
        {
            BoundingRect res = BoundingRect.EmptyBoundingRect;
            if (onSurface1)
            {
                for (int i = 0; i < basePoints.Length; i++)
                {
                    res.MinMax(basePoints[i].psurface1);
                }
            }
            else
            {
                for (int i = 0; i < basePoints.Length; i++)
                {
                    res.MinMax(basePoints[i].psurface2);
                }
            }
            return res;
        }
        //public override bool HitTest(ref BoundingCube cube, double precision)
        //{   // soll in GeneralCurve mit TetraederHülle gemacht werden, vorläufig:
        //    for (int i = 0; i < basePoints.Length - 1; ++i)
        //    {
        //        GeoPoint sp = basePoints[i].p3d;
        //        GeoPoint ep = basePoints[i + 1].p3d;
        //        if (cube.Interferes(ref sp, ref ep)) return true;
        //    }
        //    return false;
        //}
        //public override bool HitTest(Projection projection, BoundingRect rect, bool onlyInside)
        //{   // sollte in GeneralCurve gelöst werden, hier erstmal:
        //    if (onlyInside)
        //    {
        //        BoundingRect ext = BoundingRect.EmptyBoundingRect;
        //        for (int i = 0; i < basePoints.Length; ++i)
        //        {
        //            ext.MinMax(projection.ProjectUnscaled(basePoints[i].p3d));
        //        }
        //        return ext <= rect;
        //    }
        //    else
        //    {
        //        ClipRect clr = new ClipRect(ref rect);
        //        for (int i = 0; i < basePoints.Length - 1; ++i)
        //        {
        //            if (clr.LineHitTest(projection.ProjectUnscaled(basePoints[i].p3d), projection.ProjectUnscaled(basePoints[i + 1].p3d)))
        //                return true;
        //        }
        //        return false;
        //    }
        //}
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.IGeoObjectImpl.Position (GeoPoint, GeoVector, double)"/>
        /// </summary>
        /// <param name="fromHere"></param>
        /// <param name="direction"></param>
        /// <param name="precision"></param>
        /// <returns></returns>
        public override double Position(GeoPoint fromHere, GeoVector direction, double precision)
        {   // vorläufig mal auf die Polylinien beziehen
            double res = double.MaxValue;
            for (int i = 0; i < basePoints.Length - 1; ++i)
            {
                double pos1, pos2;
                double d = Geometry.DistLL(basePoints[i].p3d, basePoints[i + 1].p3d - basePoints[i].p3d, fromHere, direction, out pos1, out pos2);
                if (pos1 >= 0.0 && pos1 <= 1.0 && pos2 < res) res = pos2;
            }
            return res;
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.IGeoObjectImpl.PaintTo3D (IPaintTo3D)"/>
        /// </summary>
        /// <param name="paintTo3D"></param>
        public override void PaintTo3D(IPaintTo3D paintTo3D)
        {
            base.PaintTo3D(paintTo3D);
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.IGeoObjectImpl.CopyGeometry (IGeoObject)"/>
        /// </summary>
        /// <param name="ToCopyFrom"></param>
        public override void CopyGeometry(IGeoObject ToCopyFrom)
        {
            InterpolatedDualSurfaceCurve other = ToCopyFrom as InterpolatedDualSurfaceCurve;
            basePoints = other.basePoints.Clone() as SurfacePoint[];
            forwardOriented = other.forwardOriented;
            surface1 = other.surface1;
            surface2 = other.surface2;
            InvalidateSecondaryData();
#if DEBUG
            CheckSurfaceParameters();
#endif
        }
        public override void FindSnapPoint(SnapPointFinder spf)
        {
            if (!spf.Accept(this)) return;
            if (spf.SnapToObjectCenter)
            {
                GeoPoint Center = (this as ICurve).PointAt(0.5);
                spf.Check(Center, this, SnapPointFinder.DidSnapModes.DidSnapToObjectCenter);
            }
            if (spf.SnapToObjectSnapPoint)
            {
                spf.Check(StartPoint, this, SnapPointFinder.DidSnapModes.DidSnapToObjectSnapPoint);
                spf.Check(EndPoint, this, SnapPointFinder.DidSnapModes.DidSnapToObjectSnapPoint);
            }
            if (spf.SnapToDropPoint && spf.BasePointValid)
            {
                //GeoPoint toTest = Geometry.DropPL(spf.BasePoint, startPoint, endPoint);
                //spf.Check(toTest, this, SnapPointFinder.DidSnapModes.DidSnapToDropPoint);
            }
            if (spf.SnapToObjectPoint)
            {
                double par = PositionOf(spf.SourcePoint3D, spf.Projection.ProjectionPlane);
                // TODO: hier ist eigentlich gefragt der nächste punkt auf der Linie im Sinne des Projektionsstrahls
                if (par >= 0.0 && par <= 1.0)
                {
                    spf.Check(PointAt(par), this, SnapPointFinder.DidSnapModes.DidSnapToObjectPoint);
                }
            }
        }
        #endregion
        #region ICurve Members
        public override GeoPoint StartPoint
        {
            get
            {
                return basePoints[0].p3d;
            }
            set
            {   // es darf hier nur um minimale Änderungen gehen, nicht um trimmen
                // wird nur von BRepOperation verwendet
                SurfacePoint sp = new SurfacePoint(value, surface1.PositionOf(value), surface2.PositionOf(value));
                AdjustPeriodic(ref sp, 0);
                basePoints[0] = sp;
                InvalidateSecondaryData();
            }
        }
        public override GeoPoint EndPoint
        {
            get
            {
                return basePoints[basePoints.Length - 1].p3d;
            }
            set
            {
                SurfacePoint sp = new SurfacePoint(value, surface1.PositionOf(value), surface2.PositionOf(value));
                AdjustPeriodic(ref sp, basePoints.Length - 1);
                basePoints[basePoints.Length - 1] = sp;
                InvalidateSecondaryData();
            }
        }
        private int SegmentOfParameter(double par)
        {
            int ind = (int)Math.Floor(par * (basePoints.Length - 1));
            if (ind >= basePoints.Length - 1) ind = basePoints.Length - 2; // es muss immer ind und ind+1 gültig sein
            if (ind < 0) ind = 0;
            return ind;
        }
        private void ApproximatePosition(double position, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p)
        {
            ApproximatePosition(position, out uv1, out uv2, out p, false);
        }
        /// <summary>
        /// Refines the collection of base points by adjusting their distribution to ensure a more uniform spacing.
        /// </summary>
        /// <remarks>This method iteratively evaluates the distances between consecutive base points and
        /// adjusts the collection  by either adding or removing points based on their relative spacing. Points that are
        /// too far apart will  have new points inserted between them, while points that are too close together will
        /// have one of them removed.  The process continues until no further adjustments are needed.  This operation
        /// invalidates any cached approximations or secondary data that depend on the base points.</remarks>
        private void RefineBasePoints()
        {
            bool changed = true;
            while (changed)
            {
                changed = false;
                double length = 0.0;
                for (int i = 0; i < basePoints.Length - 1; i++)
                {
                    length += basePoints[i + 1].p3d | basePoints[i].p3d;
                }
                length /= (basePoints.Length - 1);
                for (int i = 0; i < basePoints.Length - 1; i++)
                {
                    double d = basePoints[i + 1].p3d | basePoints[i].p3d;
                    if (d > 1.5 * length)
                    {
                        changed = true;
                        approxBSpline = null;
                        hashedPositions.Clear();
                        ApproximatePosition((i + 0.5) / (basePoints.Length - 1), out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p, false);
                        List<SurfacePoint> bpl = new List<SurfacePoint>(basePoints);
                        bpl.Insert(i + 1, new SurfacePoint(p, uv1, uv2));
                        basePoints = bpl.ToArray();
                        break; // nur einen Punkt pro Schleifendurchlauf einfügen
                    }
                    if (d < 0.5 * length && basePoints.Length > 2)
                    {
                        changed = true;
                        approxBSpline = null;
                        hashedPositions.Clear();
                        List<SurfacePoint> bpl = new List<SurfacePoint>(basePoints);
                        bpl.RemoveAt(i + 1);
                        basePoints = bpl.ToArray();
                        break; // nur einen Punkt pro Schleifendurchlauf entfernen
                    }
                }
            }
            InvalidateSecondaryData();
        }

        public SortedDictionary<double, GeoPoint> RefineByAngle(SortedDictionary<double, GeoPoint> src, double maxAngle)
        {
            // Arbeitskopie, die wir mutieren dürfen:
            var dict = new SortedDictionary<double, GeoPoint>(src);


            // Iterativ über Segmente: wenn gesplittet wurde, bleibt i stehen und prüft die neuen Teilsegmente.
            var keys = dict.Keys.ToList();
            int i = 0;

            while (i < keys.Count - 1)
            {
                double t0 = keys[i];
                double t1 = keys[i + 1];

                // zu kurze Segmente nicht mehr teilen
                if (t1 - t0 <= Precision.eps) { i++; continue; }

                double angle = new Angle((approxBSpline as ICurve).DirectionAt(t0), (approxBSpline as ICurve).DirectionAt(t1));
                if (angle <= maxAngle)
                {
                    // Segment ok -> nächstes
                    i++;
                    continue;
                }

                // Split nötig: Mid-Parameter
                double tm = 0.5 * (t0 + t1);

                ApproximatePosition(tm, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint pm);
                dict[tm] = pm;

                // tm in Keys-Liste an der richtigen Stelle einfügen
                keys.Insert(i + 1, tm);

                // optional: Abbruch, falls zu viele Splits auf diesem Intervall passiert sind
                // einfache Heuristik: limitieren über Länge des Intervalls ggü. Ursprung
                // oder zähle Splits pro Basisintervall (hier kurz & pragmatisch über Count)
                if (keys.Count > src.Count + (10 * (src.Count - 1)))
                    break;

                // nicht i++: wir prüfen zuerst (t0, tm), dann erneut (tm, t1) in der nächsten Schleife
            }

            return dict;
        }

        private BSpline ApproxBSpline
        {
            get
            {
#if DEBUG
                CheckSurfaceParameters();
#endif
                if (approxBSpline != null) return approxBSpline;
                approxBSpline = BSpline.Construct();
                approxBSpline.ThroughPoints(basePoints.Select(bp => bp.p3d).ToArray(), 3, false);
                // this BSpline has non uniform knot values. They are calculated by the distance of the base points
                SortedDictionary<double, GeoPoint> bpl = new SortedDictionary<double, GeoPoint>();
                for (int i = 0; i < 10; i++)
                {
                    double pos = i / (double)9;
                    ApproximatePosition(pos, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p);
                    bpl[pos] = p; // p is calculated with the above approxBSpline as a start value
                }
                bpl = RefineByAngle(bpl, Math.PI / 2.0);
                approxBSpline.ThroughPoints(bpl.Values.ToArray(), 3, false); // new BSpline with refined points
                GeoPoint[] knpnts = approxBSpline.KnotPoints; // this are the points on the BSpline at the knot values, we want to use them as uniformly distributed base points
                basePoints = new SurfacePoint[knpnts.Length];
                for (int i = 0; i < knpnts.Length; i++)
                {
                    GeoPoint2D uv1 = surface1.PositionOf(knpnts[i]);
                    GeoPoint2D uv2 = surface2.PositionOf(knpnts[i]);
                    basePoints[i] = new SurfacePoint(knpnts[i], uv1, uv2);
                }
                AdjustBasePointsPeriodic();
                double[] knots = approxBSpline.Knots;
                hashedPositions.Clear();
                for (int i = 0; i < knots.Length; i++)
                {
                    hashedPositions[knots[i]] = basePoints[i];
                }
#if DEBUG
                CheckSurfaceParameters();
#endif
                return approxBSpline;
            }
        }
        private void ApproximatePosition(double position, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p, bool refineBasePoints)
        {
            lock (hashedPositions)
            {
                // Zuerst nachsehen, ob der Punkt schon bekannt ist
                // IGeoObject dbg = this.DebugBasePoints;
                (bool hasLower, double lowerKey, SurfacePoint lowerValue, bool hasUpper, double upperKey, SurfacePoint upperValue, bool exact, int exactIndex) = hashedPositions.Neighbors(position);
                if (exact)
                {
                    SurfacePoint found = hashedPositions.Values[exactIndex];
                    p = found.p3d;
                    uv1 = found.psurface1;
                    uv2 = found.psurface2;
                    return;
                }
                Plane normalPlane; // Plane normal tu the BSpline or segment-polyline at position
                if (approxBSpline == null)
                {   // in the constructor we need to calculate a few basepoints before we can build the BSpline

                    int ind = (int)Math.Floor(position * (basePoints.Length - 1));
                    if (ind < 0) ind = 0;
                    if (ind > basePoints.Length - 1) ind = basePoints.Length - 1;
                    double d = position * (basePoints.Length - 1) - ind;
                    GeoPoint location;
                    if (d > 0.0 && ind < basePoints.Length - 1) location = basePoints[ind].p3d + d * (basePoints[ind + 1].p3d - basePoints[ind].p3d);
                    else location = basePoints[ind].p3d;
                    if (ind == basePoints.Length - 1) --ind;
                    GeoVector normal = basePoints[ind + 1].p3d - basePoints[ind].p3d;
                    normalPlane = new Plane(location, normal);
                }
                else
                {
                    normalPlane = new Plane((approxBSpline as ICurve).PointAt(position), (approxBSpline as ICurve).DirectionAt(position));
                }
                if (isTangential)
                {
                    GeoPoint2D uv1s, uv2s;
                    if (hasLower && hasUpper)
                    {
                        double d1 = (position - lowerKey) / (upperKey - lowerKey);
                        double d2 = (upperKey - position) / (upperKey - lowerKey);
                        uv1s = new GeoPoint2D((1.0 - d1) * lowerValue.psurface1.x + d1 * upperValue.psurface1.x, (1.0 - d1) * lowerValue.psurface1.y + d1 * upperValue.psurface1.y);
                        uv2s = new GeoPoint2D((1.0 - d1) * lowerValue.psurface2.x + d1 * upperValue.psurface2.x, (1.0 - d1) * lowerValue.psurface2.y + d1 * upperValue.psurface2.y);
                    }
                    else if (hasLower)
                    {
                        uv1s = lowerValue.psurface1;
                        uv2s = lowerValue.psurface2;
                    }
                    else if (hasUpper)
                    {
                        uv1s = upperValue.psurface1;
                        uv2s = upperValue.psurface2;
                    }
                    else
                    {
                        uv1s = surface1.PositionOf(normalPlane.Location);
                        uv2s = surface2.PositionOf(normalPlane.Location);
                    }

                    if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(normalPlane.Location, normalPlane.Normal, surface1, surface2, out uv1, out uv2, uv1s, uv2s))
                    {
                        // if (BoxedSurfaceExtension.FindTangentialIntersectionPointJ(normalPlane.Location, normalPlane.Normal, surface1, surface2, out uv1, out uv2))
                        // FindTangentialIntersectionPointJ is maybe faster, but we will have to check its reliability
                        p = new GeoPoint(surface1.PointAt(uv1), surface2.PointAt(uv2));
                        SurfacePoint spt = new SurfacePoint(p, uv1, uv2);
                        if (hasLower && hasUpper)
                        {
                            if (position - lowerKey < upperKey - position) spt.FixAgainstNeighbour(lowerValue, surface1, surface2);
                            else spt.FixAgainstNeighbour(upperValue, surface1, surface2);
                        }
                        else if (hasLower) spt.FixAgainstNeighbour(lowerValue, surface1, surface2);
                        else if (hasUpper) spt.FixAgainstNeighbour(upperValue, surface1, surface2);
                        else AdjustPeriodic(ref spt.psurface1, ref spt.psurface2);
                        hashedPositions[position] = spt;
                        uv1 = spt.psurface1;
                        uv2 = spt.psurface2;
                        return;
                    }
                }
                else
                {
                    PlaneSurface ps = new PlaneSurface(normalPlane);
                    p = normalPlane.Location;
                    GeoPoint2D uvplane = GeoPoint2D.Origin;
                    uv1 = surface1.PositionOf(normalPlane.Location);
                    uv2 = surface2.PositionOf(normalPlane.Location);
                    AdjustPeriodic(ref uv1, ref uv2);
                    if (BoxedSurfaceExtension.SurfacesIntersectionLM(ps, surface1, surface2, ref uvplane, ref uv1, ref uv2, ref p))
                    {
                        SurfacePoint spt = new SurfacePoint(p, uv1, uv2);
                        if (hasLower && hasUpper)
                        {
                            if (position - lowerKey < upperKey - position) spt.FixAgainstNeighbour(lowerValue, surface1, surface2);
                            else spt.FixAgainstNeighbour(upperValue, surface1, surface2);
                        }
                        else if (hasLower) spt.FixAgainstNeighbour(lowerValue, surface1, surface2);
                        else if (hasUpper) spt.FixAgainstNeighbour(upperValue, surface1, surface2);
                        else AdjustPeriodic(ref spt.psurface1, ref spt.psurface2);
                        hashedPositions[position] = spt;
                        uv1 = spt.psurface1;
                        uv2 = spt.psurface2;
                        return;
                    }
                }
                // we should not reach this point. It could be there is an inner point, which is tangential or the curve is tangential but isTangential is false
                {
                    p = normalPlane.Location;
                    uv1 = surface1.PositionOf(normalPlane.Location);
                    uv2 = surface2.PositionOf(normalPlane.Location);
                    SurfacePoint spt = new SurfacePoint(p, uv1, uv2);
                    if (hasLower && hasUpper)
                    {
                        if (position - lowerKey < upperKey - position) spt.FixAgainstNeighbour(lowerValue, surface1, surface2);
                        else spt.FixAgainstNeighbour(upperValue, surface1, surface2);
                    }
                    else if (hasLower) spt.FixAgainstNeighbour(lowerValue, surface1, surface2);
                    else if (hasUpper) spt.FixAgainstNeighbour(upperValue, surface1, surface2);
                    else AdjustPeriodic(ref spt.psurface1, ref spt.psurface2);
                    uv1 = spt.psurface1;
                    uv2 = spt.psurface2;
                    hashedPositions[position] = spt;
                }
#if DEBUG
                CheckSurfaceParameters();
#endif
                // throw new ApplicationException("InterpolatedDualSurfaceCurve: intermediate point could not be calculated");
            }
        }
        private void ApproximatePositionOld(double position, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p, bool refineBasePoints)
        {
            lock (hashedPositions)
            {
                // Zuerst nachsehen, ob der Punkt schon bekannt ist
                SurfacePoint found;
                // IGeoObject dbg = this.DebugBasePoints;
                if (hashedPositions.TryGetValue(position, out found))
                {
                    p = found.p3d;
                    uv1 = found.psurface1;
                    uv2 = found.psurface2;
                    return;
                }
                int ind = (int)Math.Floor(position * (basePoints.Length - 1));
                // die mit basepoint übereinstimmenden Punkte nicht ins dictionary nehmen
                if (ind > basePoints.Length - 1)
                {
                    uv1 = basePoints[basePoints.Length - 1].psurface1;
                    uv2 = basePoints[basePoints.Length - 1].psurface2;
                    p = basePoints[basePoints.Length - 1].p3d;
                    return;
                }
                if (ind < 0)
                {
                    uv1 = basePoints[0].psurface1;
                    uv2 = basePoints[0].psurface2;
                    p = basePoints[0].p3d;
                    return;
                }
                if (position * (basePoints.Length - 1) - ind == 0.0 && !refineBasePoints)
                {   // Index genau getroffen
                    uv1 = basePoints[ind].psurface1;
                    uv2 = basePoints[ind].psurface2;
                    p = basePoints[ind].p3d;
                    return;
                }
                // Eine Ebene senkrecht zur Verbindung der beiden Basispunkte. Auf dieser und den beiden Flächen
                // muss der Schnittpunkt liegen.
                double d = position * (basePoints.Length - 1) - ind;
                GeoPoint location;
                if (d > 0.0 && ind < basePoints.Length - 1) location = basePoints[ind].p3d + d * (basePoints[ind + 1].p3d - basePoints[ind].p3d);
                else location = basePoints[ind].p3d;
                if (ind == basePoints.Length - 1)
                {
                    --ind; // wenns genau um den letzten Punkt geht, dann das vorletzte Intervall nehmen
                    d = 1.0;
                }

                GeoVector normal = basePoints[ind + 1].p3d - basePoints[ind].p3d;
                Plane pln = new Plane(location, normal);
                // diese Ebene bleibt fix, es wird jetzt auf den beiden surfaces tangential fortgeschritten, bis
                // ein Schnittpunkt gefunden ist
                // Hier Sonderfälle abhandeln, nämlich eine der beiden Flächen ist eine Ebene, dann nur Schnitte mit Linie berechnen
                // oder vielleicht auch, wenn es eine einfache Schnittkurve gibt
                PlaneSurface pls = null;
                ISurface other = null;
                if (surface1 is PlaneSurface)
                {
                    pls = surface1 as PlaneSurface;
                    other = surface2;
                }
                else if (surface2 is PlaneSurface)
                {
                    pls = surface2 as PlaneSurface;
                    other = surface1;
                }

                if (surface1 is ISurfacePlaneIntersection && surface2 is ISurfacePlaneIntersection)
                {   // Schnittpunkt der beiden einfachen Kurven, die die Ebene mit den surfaces schneidet
                    // geht schneller als die Iteration
                    if (approxPolynom == null) InitApproxPolynom();
                    GeoPoint ppPolynom = GeoPoint.Invalid;
                    if (approxPolynom != null)
                    {
                        GeoPoint pp = ppPolynom = approxPolynom.PointAt(position);
                        pln = new Plane(pp, approxPolynom.DirectionAt(position));
                    }

                    // die folgende extentbestimmung könnte man rausnehmen
                    BoundingRect ext1 = BoundingRect.EmptyBoundingRect;
                    BoundingRect ext2 = BoundingRect.EmptyBoundingRect;
                    for (int i = 0; i < basePoints.Length; i++)
                    {
                        ext1.MinMax(basePoints[i].psurface1);
                        ext2.MinMax(basePoints[i].psurface2);
                    }
                    ICurve2D[] c2d1 = (surface1 as ISurfacePlaneIntersection).GetPlaneIntersection(pln, ext1.Left, ext1.Right, ext1.Bottom, ext1.Top);
                    ICurve2D[] c2d2 = (surface2 as ISurfacePlaneIntersection).GetPlaneIntersection(pln, ext2.Left, ext2.Right, ext2.Bottom, ext2.Top);
                    double md = double.MaxValue;
                    GeoPoint pfound = GeoPoint.Origin;
                    for (int i = 0; i < c2d1.Length; i++)
                    {
                        for (int j = 0; j < c2d2.Length; j++)
                        {
                            //TempTriangulatedCurve2D tt1 = new TempTriangulatedCurve2D(c2d1[i]);
                            //TempTriangulatedCurve2D tt2 = new TempTriangulatedCurve2D(c2d2[j]);
                            //GeoPoint2DWithParameter[] ips = tt1.Intersect(tt2);
                            GeoPoint2DWithParameter[] ips = c2d1[i].Intersect(c2d2[j]);
#if DEBUG
                            DebuggerContainer dc = new DebuggerContainer();
                            dc.Add(c2d1[i], System.Drawing.Color.Red, i);
                            dc.Add(c2d2[j], System.Drawing.Color.Blue, j);
#endif
                            for (int k = 0; k < ips.Length; k++)
                            {
#if DEBUG
                                dc.Add(ips[k].p, System.Drawing.Color.Black, k);
#endif
                                GeoPoint pp = pln.ToGlobal(ips[k].p);
                                double dd = Geometry.DistPL(pp, basePoints[ind].p3d, basePoints[ind + 1].p3d);
                                if (dd < md)
                                {
                                    md = dd;
                                    pfound = pp;
                                }
                            }
                        }
                    }
                    if (md < double.MaxValue && !ppPolynom.IsValid || (ppPolynom | pfound) < (basePoints[ind].p3d | basePoints[ind + 1].p3d))
                    {
                        uv1 = surface1.PositionOf(pfound);
                        uv2 = surface2.PositionOf(pfound);
                        SurfacePoint sp0 = new SurfacePoint(pfound, uv1, uv2);
                        AdjustPeriodic(ref sp0, ind);
                        uv1 = sp0.psurface1;
                        uv2 = sp0.psurface2;
                        hashedPositions[position] = sp0;
                        p = pfound;
                        return;

                    }
                }
                uv1 = basePoints[ind].psurface1 + d * (basePoints[ind + 1].psurface1 - basePoints[ind].psurface1);
                uv2 = basePoints[ind].psurface2 + d * (basePoints[ind + 1].psurface2 - basePoints[ind].psurface2);
                if (isTangential)
                {
                    GeoVector nn = basePoints[ind + 1].p3d - basePoints[ind].p3d;
                    GeoPoint pp = basePoints[ind].p3d + d * nn;
                    if (BoxedSurfaceExtension.FindTangentialIntersectionPoint(pp, nn, surface1, surface2, out uv1, out uv2))
                    {
                        CheckPeriodic(ref uv1, true, ind);
                        CheckPeriodic(ref uv2, false, ind);
                        p = new GeoPoint(surface1.PointAt(uv1), surface2.PointAt(uv2));
                        SurfacePoint spt = new SurfacePoint(p, uv1, uv2);
                        AdjustPeriodic(ref spt, ind);
                        uv1 = spt.psurface1;
                        uv2 = spt.psurface2;
                        hashedPositions[position] = spt;
                        return;
                    }
                }
                if (approxPolynom == null) InitApproxPolynom();
                if (approxPolynom != null && basePoints.Length > 2)
                {
                    GeoPoint pp = approxPolynom.PointAt(position);
                    uv1 = surface1.PositionOf(pp);
                    AdjustPeriodic(ref uv1, true, ind);
                    uv2 = surface2.PositionOf(pp);
                    AdjustPeriodic(ref uv2, false, ind);
                    pln = new Plane(pp, approxPolynom.DirectionAt(position));
                }
                // pln was calculated as a plane perpendicular to the chord between basePoints[ind] and basePoints[ind + 1] at the offset provided by the parameter
                // there was an attempt to use a better plane by using the approximation polygon defined by the two basepoints and the directions at these points,
                // there were some problems with this approach, now it seems to work
                GeoPoint p1, p2;
                GeoVector du1, dv1, du2, dv2, n1, n2;
                //surface1.DerivationAt(uv1, out p1, out du1, out dv1);
                //surface2.DerivationAt(uv2, out p2, out du2, out dv2);
                n1 = surface1.GetNormal(uv1); // there is a problem with singularities (cone, sphere), but it seems GetNormal is working
                n2 = surface2.GetNormal(uv2);
                n1 = n1.Normalized;
                n2 = n2.Normalized;
                double sn1n2 = Math.Abs(Math.Sin((new SweepAngle(n1, n2)).Radian)); // bei gleicher Richtung, also tangential kleiner wert, bei senkrecht 1
                p1 = surface1.PointAt(uv1);
                p2 = surface2.PointAt(uv2);
                double mindist = double.MaxValue;
                d = (p1 | p2);
                bool didntConvert = false;
                int conversionCounter = 0;
                // Ein großes Problem machen die tangentialen Flächen: dort werden nach dem folgenden Verfahren
                // keine guten Schnittpunkte gefunden. Im tangentialen Fall müsste man mit PositionOf und PointAt
                // und den jeweiligen Mittelpunkten arbeiten. Das konvergiert zwar langsamer, aber es sollte wenigsten konvergieren...
                double prec = sn1n2 * Precision.eps;
                while (d > prec && d < mindist) // zu unsichere Bedingung, vor allem wenns tangential wird...
                {   // bricht auch ab, wenns nicht mehr konvergiert
                    // dann könnte man anderes Verfahren nehmen: die Kurven auf der Ebene pln bstimmen und
                    // die beiden Kurven schneiden
                    mindist = d;
                    // nach dem Tangentenverfahren, da wir ja schon ganz nah sind, oder?
                    // p1,du1,dv1 und p2,du2,dv2 sind die beiden Tangentialebenen, p3,du3,dv3 die senkerechte Ebene
                    // daraus ergeben sich 6 Gleichungen mit 6 unbekannten
                    // p1+u1*du1+v1*dv1 = p3+u3*du3+v3*dv3
                    // p2+u2*du2+v2*dv2 = p3+u3*du3+v3*dv3
                    // umgeformt: (u3 und v3 interessieren nicht, deshalb Vorzeichen egal
                    // u1*du1 + v1*dv1 + 0      + 0      - u3*du3 - v3*dv3 = p3-p1
                    // 0      + 0      + u2*du2 + v2*dv2 - u3*du3 - v3*dv3 = p3-p1
                    du1 = surface1.UDirection(uv1);
                    dv1 = surface1.VDirection(uv1);
                    du2 = surface2.UDirection(uv2);
                    dv2 = surface2.VDirection(uv2);
                    GeoVector du3 = pln.DirectionX;
                    GeoVector dv3 = pln.DirectionY;
#if DEBUG
                    bool doit = false;
                    if (doit)
                    {
                        PlaneSurface pls1 = new PlaneSurface(new Plane(p1, du1, dv1));
                        SimpleShape ss1 = new SimpleShape(new BoundingRect(-1, -1, 1, 1));
                        Face fc1 = Face.MakeFace(pls1, ss1);
                        PlaneSurface pls2 = new PlaneSurface(new Plane(p2, du2, dv2));
                        SimpleShape ss2 = new SimpleShape(new BoundingRect(-1, -1, 1, 1));
                        Face fc2 = Face.MakeFace(pls2, ss2);
                        PlaneSurface pls3 = new PlaneSurface(new Plane(location, du3, dv3));
                        SimpleShape ss3 = new SimpleShape(new BoundingRect(-1, -1, 1, 1));
                        Face fc3 = Face.MakeFace(pls3, ss3);
                        DebuggerContainer dc = new DebuggerContainer();
                        fc1.ColorDef = new CADability.Attribute.ColorDef("clr1", System.Drawing.Color.Red);
                        fc2.ColorDef = new CADability.Attribute.ColorDef("clr2", System.Drawing.Color.Green);
                        fc3.ColorDef = new CADability.Attribute.ColorDef("clr3", System.Drawing.Color.Violet);
                        dc.Add(fc1);
                        dc.Add(fc2);
                        dc.Add(fc3);
                        Line dbgl = Line.Construct();
                        dbgl.SetTwoPoints(p1, p2);
                        dc.Add(dbgl);
                        dbgl = Line.Construct();
                        dbgl.SetTwoPoints(basePoints[ind].p3d, basePoints[ind + 1].p3d);
                        dc.Add(dbgl);
                    }
#endif
                    Matrix m = DenseMatrix.OfArray(new double[,]
                    {
                        {du1.x,dv1.x,0,0,du3.x,dv3.x},
                        {du1.y,dv1.y,0,0,du3.y,dv3.y},
                        {du1.z,dv1.z,0,0,du3.z,dv3.z},
                        {0,0,du2.x,dv2.x,du3.x,dv3.x},
                        {0,0,du2.y,dv2.y,du3.y,dv3.y},
                        {0,0,du2.z,dv2.z,du3.z,dv3.z},
                    });
                    Matrix s = (Matrix)m.Solve(DenseMatrix.OfArray(new double[,] { { location.x - p1.x }, { location.y - p1.y }, { location.z - p1.z } ,
                        { location.x - p2.x }, { location.y - p2.y }, { location.z - p2.z } }));
                    if (s.IsValid())
                    {
                        GeoPoint2D uv1alt = uv1;
                        GeoPoint2D uv2alt = uv2;
                        GeoPoint p1alt = p1;
                        GeoPoint p2alt = p2;
                        uv1.x += s[0, 0];
                        uv1.y += s[1, 0];
                        uv2.x += s[2, 0];
                        uv2.y += s[3, 0];
                        p1 = surface1.PointAt(uv1);
                        p2 = surface2.PointAt(uv2);
                        double d1 = uv1 | uv1alt;
                        double d2 = uv2 | uv2alt;
                        double d3 = p1 | p1alt;
                        double d4 = p2 | p2alt;
                        d = p1 | p2;

                        if (d > mindist || double.IsNaN(d))
                        {   // es ist schlechter geworden
                            conversionCounter += 1;
                            if (conversionCounter < 5 && !double.IsNaN(d))
                            {
                                mindist = Geometry.NextDouble(d);
                            }
                            else
                            {
                                uv1 = uv1alt;
                                uv2 = uv2alt;
                                p1 = p1alt;
                                p2 = p2alt;
                                didntConvert = true;
                            }
                        }
                    }
                    else didntConvert = true;
                }
                p = new GeoPoint(p1, p2);
                if (didntConvert)
                {   // möglicherweise tangentiale Berührung der beiden Flächen, versuchen mit pointAt, position of zu konvergieren
                    if (approxPolynom != null && basePoints.Length > 2)
                    {
                        GeoPoint pp = approxPolynom.PointAt(position);
                        uv1 = surface1.PositionOf(pp);
                        AdjustPeriodic(ref uv1, true, ind);
                        uv2 = surface2.PositionOf(pp);
                        AdjustPeriodic(ref uv2, false, ind);
                        p1 = surface1.PointAt(uv1);
                        p2 = surface2.PointAt(uv2);
                        p = new GeoPoint(p1, p2);
                    }

                    d = p1 | p2;
                    double basedist = basePoints[ind + 1].p3d | basePoints[ind].p3d;
                    while (d > Precision.eps)
                    {
                        GeoPoint cnt = new GeoPoint(p1, p2);
                        GeoPoint2D uv1t = surface1.PositionOf(cnt);
                        GeoPoint2D uv2t = surface2.PositionOf(cnt);
                        GeoPoint p1t = surface1.PointAt(uv1t);
                        GeoPoint p2t = surface2.PointAt(uv2t);
                        double dt = p1t | p2t;
                        if (dt < d)
                        {   // ist allemal besser
                            p1 = p1t;
                            p2 = p2t;
                            uv1 = uv1t;
                            uv2 = uv2t;
                        }
                        if (dt > d * 0.75)
                        {   // konvergiert nicht gescheit
                            break;
                        }
                        d = dt;
                        p = new GeoPoint(p1, p2);
                        if ((p | basePoints[ind + 1].p3d) > basedist || (p | basePoints[ind].p3d) > basedist)
                        {   // ein Punkte ist aus dem Ruder gelaufen: der Abstand des neuen Punktes zu einem seiner beiden 
                            // basePoint Nachbarn sollte nie größer sein als der Abstand der beiden basePoints
                            // hier einfach lineare Interpolation
                            d = position * (basePoints.Length - 1) - ind;
                            uv1 = basePoints[ind].psurface1 + d * (basePoints[ind + 1].psurface1 - basePoints[ind].psurface1);
                            uv2 = basePoints[ind].psurface2 + d * (basePoints[ind + 1].psurface2 - basePoints[ind].psurface2);
                            break;
                        }
                    }
                }
                CheckPeriodic(ref uv1, true, ind);
                CheckPeriodic(ref uv2, false, ind);
                // System.Diagnostics.Trace.WriteLine("Parameter, Punkt: " + position.ToString() + ", " + p.ToString());
                SurfacePoint sp = new SurfacePoint(p, uv1, uv2);
                AdjustPeriodic(ref sp, ind);
                uv1 = sp.psurface1;
                uv2 = sp.psurface2;
                hashedPositions[position] = sp;
            }
        }
        internal BoundingRect Domain1
        {
            get
            {
                BoundingRect res = BoundingRect.EmptyBoundingRect;
                for (int i = 0; i < basePoints.Length; i++)
                {
                    res.MinMax(basePoints[i].psurface1);
                }
                return res;
            }
        }
        internal BoundingRect Domain2
        {
            get
            {
                BoundingRect res = BoundingRect.EmptyBoundingRect;
                for (int i = 0; i < basePoints.Length; i++)
                {
                    res.MinMax(basePoints[i].psurface2);
                }
                return res;
            }
        }
        private void CheckPeriodic(ref GeoPoint2D uv, bool onSurface1, int index)
        {
            // es wurde ein uv Wert auf einer Oberfläche gefunden
            // wenn diese aber periodisch ist, dann sollte er in der Nähe
            // der basepoint[ind] bzw. basepoint[ind+1] liegen
            GeoPoint2D uv1, uv2;
            double uperiod = 0.0;
            double vperiod = 0.0;
            ISurface surface;
            if (onSurface1)
            {
                surface = surface1;
                uv1 = basePoints[index].psurface1;
                uv2 = basePoints[index + 1].psurface1;
            }
            else
            {
                surface = surface2;
                uv1 = basePoints[index].psurface2;
                uv2 = basePoints[index + 1].psurface2;
            }
            if (surface.IsUPeriodic) uperiod = surface.UPeriod;
            if (surface.IsVPeriodic) vperiod = surface.UPeriod;
            if (uperiod > 0.0)
            {
                double d0 = Math.Abs(uv.x - (uv1.x + uv2.x) / 2);
                double d1 = Math.Abs(uv.x + uperiod - (uv1.x + uv2.x) / 2);
                double d2 = Math.Abs(uv.x - uperiod - (uv1.x + uv2.x) / 2);
                if (d1 < d0) uv.x += uperiod;
                if (d2 < d0) uv.x -= uperiod;
                // ansonsten bleibt er ja unverändert
            }
            if (vperiod > 0.0)
            {
                double d0 = Math.Abs(uv.y - (uv1.y + uv2.y) / 2);
                double d1 = Math.Abs(uv.y + vperiod - (uv1.y + uv2.y) / 2);
                double d2 = Math.Abs(uv.y - vperiod - (uv1.y + uv2.y) / 2);
                if (d1 < d0) uv.y += vperiod;
                if (d2 < d0) uv.y -= vperiod;
            }
        }
        public override GeoVector StartDirection
        {
            get
            {
                GeoVector adir = (ApproxBSpline as ICurve).StartDirection;
                if (isTangential)
                {
                    return adir;
                }
                else
                {
                    GeoVector v = surface1.GetNormal(basePoints[0].psurface1) ^ surface2.GetNormal(basePoints[0].psurface2);
                    if (!forwardOriented) v.Reverse();
                    v.Length = adir.Length; // make the same lengt as the approximating BSpline would have. This is very close
                    return v;
                }
            }
        }
        public override GeoVector EndDirection
        {
            get
            {
                GeoVector adir = (ApproxBSpline as ICurve).EndDirection;
                if (isTangential)
                {
                    return adir;
                }
                else
                {
                    GeoVector v = surface1.GetNormal(basePoints[basePoints.Length - 1].psurface1) ^ surface2.GetNormal(basePoints[basePoints.Length - 1].psurface2);
                    if (!forwardOriented) v.Reverse();
                    v.Length = adir.Length; // make the same lengt as the approximating BSpline would have. This is very close to the factual length
                    return v;
                }
            }
        }
        // in/out: Su, Sv, Suu, Suv, Svv (beide Flächen), nHat (gemeinsame Normale), p (Ebennormal)
        static GeoVector TangentDirectionAtContact(
            GeoVector Su1, GeoVector Sv1, GeoVector Suu1, GeoVector Suv1, GeoVector Svv1,
            GeoVector Su2, GeoVector Sv2, GeoVector Suu2, GeoVector Suv2, GeoVector Svv2,
            GeoVector nHat, GeoVector p, GeoVector tangentHint /* z.B. Polyline-Richtung */)
        {
            // 1) Schnellrichtung
            var t0 = (p ^ nHat).Normalized; // ^ = Kreuzprodukt

            // 2) Shape-Operatoren
            Matrix<double> I(GeoVector Su, GeoVector Sv)
            {
                return DenseMatrix.OfArray(new double[,] {
            { Su*Su, Su*Sv },      // * = Skalarprodukt
            { Su*Sv, Sv*Sv }
        });
            }
            Matrix<double> II(GeoVector Suu, GeoVector Suv, GeoVector Svv, GeoVector n)
            {
                return DenseMatrix.OfArray(new double[,] {
            { n*Suu, n*Suv },
            { n*Suv, n*Svv }
        });
            }

            var I1 = I(Su1, Sv1); var II1 = II(Suu1, Suv1, Svv1, nHat);
            var I2 = I(Su2, Sv2); var II2 = II(Suu2, Suv2, Svv2, nHat);

            // stabile Inversion via SVD/Pinv (hier einfach .Inverse() angenommen)
            var S1 = I1.Inverse() * II1;
            var S2 = I2.Inverse() * II2;
            var A = S1 - S2;                 // 2x2

            // Nullraumrichtung von A (kleinster Singulärvektor von A)
            var ATA = A.Transpose() * A;
            var evd = ATA.Evd();
            int idxMin = evd.EigenValues.Real().MinimumIndex();
            var a = evd.EigenVectors.Column(idxMin); // 2D

            // schlecht konditioniert? -> Fallback
            if (a.Norm(2) < 1e-12) return t0;

            // 3D-Tangente aus Fläche 1
            var t = (a[0] * Su1 + a[1] * Sv1);
            // auf Tangentialebene projizieren
            t -= (t * nHat) * nHat;
            if (t.Length < 1e-16) return t0;

            t = t.Normalized;
            // mit Schnitt-Ebene konsistent (optional)
            t -= (t * p) * p; t = t.Normalized;

            // Orientierung konsistent zur Polylinie
            if (t * tangentHint < 0) t = -t;
            return t;
        }


        public override GeoVector DirectionAt(double Position)
        {
            GeoPoint2D uv1, uv2;
            GeoPoint p;
            ApproximatePosition(Position, out uv1, out uv2, out p);
            GeoVector dir;
            if (isTangential)
            {
                dir = (ApproxBSpline as ICurve).DirectionAt(Position); // here we cannnot use the normals of the surfaces, they are parallel
                surface1.Derivation2At(uv1, out _, out GeoVector su1, out GeoVector sv1, out GeoVector suu1, out GeoVector suv1, out GeoVector svv1);
                surface2.Derivation2At(uv2, out _, out GeoVector su2, out GeoVector sv2, out GeoVector suu2, out GeoVector suv2, out GeoVector svv2);
                GeoVector dirt = TangentDirectionAtContact(su1, sv1, suu1, suv1, svv1, su2, sv2, suu2, suv2, svv2, (su1 ^ sv1 + su2 ^ sv2).Normalized, dir, dir);
                dirt.Length = dir.Length;
                dir = dirt;
            }
            else
            {
                dir = surface1.GetNormal(uv1) ^ surface2.GetNormal(uv2);
                dir.Length = (ApproxBSpline as ICurve).DirectionAt(Position).Length; // make the same lengt as the approximating BSpline would have. This is very close
            }
            if (!forwardOriented) dir.Reverse();
            return dir;
        }
        public override GeoPoint PointAt(double Position)
        {
            GeoPoint2D uv1, uv2;
            GeoPoint p;
#if DEBUG
            // double oldLength = hashedPositionsLength();
#endif
            ApproximatePosition(Position, out uv1, out uv2, out p);
#if DEBUG
            //if (oldLength > 0.0 && hashedPositionsLength() / oldLength > 2)
            //{   // something invalid happened

            //}
#endif
            return p;
        }
#if DEBUG
        double hashedPositionsLength()
        {   // the length of the curve according to the util now calculated points
            SortedDictionary<double, GeoPoint> sortedPoints = new SortedDictionary<double, GeoPoint>();
            for (int i = 0; i < basePoints.Length; i++)
            {
                sortedPoints[(double)(i) / (double)(basePoints.Length - 1)] = basePoints[i].p3d;
                lock (hashedPositions)
                {
                    foreach (KeyValuePair<double, SurfacePoint> item in hashedPositions)
                    {
                        sortedPoints[item.Key] = item.Value.p3d;
                    }
                }
            }
            GeoPoint lastPoint = GeoPoint.Invalid;
            double res = 0.0;
            foreach (GeoPoint item in sortedPoints.Values)
            {
                if (lastPoint.IsValid) res += item | lastPoint;
                lastPoint = item;
            }
            return res;
        }
#endif
        public override double PositionAtLength(double position)
        {
            throw new Exception("The method or operation is not implemented.");
        }
        public override double PositionOf(GeoPoint p)
        {
            double ppos = TetraederHull.PositionOf(p);
            //if (approxPolynom == null) InitApproxPolynom();
            //double pos1 = approxPolynom.PositionOf(p, out double md);
            double pos1 = (ApproxBSpline as ICurve).PositionOf(p);

            if ((pos1 != double.MaxValue) && (PointAt(pos1) | p) < (PointAt(ppos) | p))
                return pos1;

            return ppos;

            //Unreachable code
            /* if (Math.Abs(pos1 - ppos) > 0.1 && md < Precision.eps)
            { }
            return ppos;
            double res = -1.0;
            double maxdist = double.MaxValue;
            if (approxPolynom != null)
            {
                double pos = approxPolynom.PositionOf(p, out maxdist);
                return pos;
            }
            for (int i = 0; i < basePoints.Length - 1; ++i)
            {
                GeoPoint dp = Geometry.DropPL(p, basePoints[i].p3d, basePoints[i + 1].p3d);
                double pos = Geometry.LinePar(basePoints[i].p3d, basePoints[i + 1].p3d - basePoints[i].p3d, dp);
                bool valid = (pos >= 0.0 && pos <= 1.0);
                if (valid)
                {
                    double d = Geometry.DistPL(p, basePoints[i].p3d, basePoints[i + 1].p3d);
                    if (d <= maxdist)
                    {
                        maxdist = d;
                        res = i + pos;
                    }
                }
            }
            // if (res < 0.0) Wenn der punkt fast genau ein basepoint ist
            // kann er trotzdem oben verworfen werden, deshalb hier noch der test auf alle Basepoints
            // und Punkte, die nahe einem Basepoint liegen, aber im ungültigen Winkelbereich
            // werden mit womöglich ganz anderen Linien in Verbindung gebracht
            {   // keine passende Strecke gefunden
                for (int i = 0; i < basePoints.Length; i++)
                {
                    double d = p | basePoints[i].p3d;
                    if (d < maxdist)
                    {
                        maxdist = d;
                        res = i;
                    }
                }
            }
            return res / (basePoints.Length - 1);*/
        }
        public override double PositionOf(GeoPoint p, double prefer)
        {
            throw new Exception("The method or operation is not implemented.");
        }
        public override double PositionOf(GeoPoint p, Plane pl)
        {
            throw new Exception("The method or operation is not implemented.");
        }
        public override double Length
        {
            get
            {   // nur eine grobe Annäherung hier, die nie 0 sein sollte.
                // man müsste irgendwie extrapolieren
                double d = 0.0;
                for (int i = 0; i < basePoints.Length - 1; ++i)
                {
                    d += basePoints[i].p3d | basePoints[i + 1].p3d;
                }
                return d;
            }
        }
        public override ICurve[] Split(double Position)
        {
            List<SurfacePoint> l1 = new List<SurfacePoint>();
            List<SurfacePoint> l2 = new List<SurfacePoint>();
            int ind = (int)Math.Floor(Position * (basePoints.Length - 1));
            if (Position * (basePoints.Length - 1) - ind == 0.0)
            {   // die Split-Position liegt genau auf einem basepoint
                for (int i = 0; i < basePoints.Length; i++)
                {
                    if (i <= ind) l1.Add(basePoints[i]);
                    if (i >= ind) l2.Add(basePoints[i]);
                }
            }
            else
            {   // es wird ein Zwischenpunkt eingefügt
                GeoPoint2D uv1, uv2;
                GeoPoint p;
                ApproximatePosition(Position, out uv1, out uv2, out p);
                SurfacePoint p1 = new SurfacePoint(p, uv1, uv2);
                l2.Add(p1); // die 2. Liste fängt mit dem neuen Punkt an
                for (int i = 0; i < basePoints.Length; i++)
                {
                    if (i <= ind) l1.Add(basePoints[i]);
                    else l2.Add(basePoints[i]);
                }
                l1.Add(p1); // die 1. Liste hört mit dem neuen Punkt auf
            }
            for (int i = l1.Count - 1; i > 0; --i)
            {
                if ((l1[i].p3d | l1[i - 1].p3d) == 0.0) l1.RemoveAt(i);
            }
            for (int i = l2.Count - 1; i > 0; --i)
            {
                if ((l2[i].p3d | l2[i - 1].p3d) == 0.0) l2.RemoveAt(i);
            }
            InterpolatedDualSurfaceCurve dsc1 = new InterpolatedDualSurfaceCurve(surface1.Clone(), surface2.Clone(), l1.ToArray());
            InterpolatedDualSurfaceCurve dsc2 = new InterpolatedDualSurfaceCurve(surface1.Clone(), surface2.Clone(), l2.ToArray());
            return new ICurve[] { dsc1, dsc2 };
        }

        internal void RecalcSurfacePoints(BoundingRect bounds1, BoundingRect bounds2)
        {
            for (int i = 0; i < basePoints.Length; i++)
            {
                basePoints[i].psurface1 = surface1.PositionOf(basePoints[i].p3d);
                SurfaceHelper.AdjustPeriodic(surface1, bounds1, ref basePoints[i].psurface1);
                basePoints[i].psurface2 = surface2.PositionOf(basePoints[i].p3d);
                SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref basePoints[i].psurface2);
            }
            InvalidateSecondaryData();
            int n = basePoints.Length / 2; // es müssen mindesten 3 sein
            GeoVector v = surface1.GetNormal(basePoints[n].psurface1) ^ surface2.GetNormal(basePoints[n].psurface2);
            GeoVector v0 = basePoints[n + 1].p3d - basePoints[n - 1].p3d;
            Angle a = new Angle(v, v0);
            forwardOriented = (a.Radian < Math.PI / 2.0);
            CheckPeriodic();
#if DEBUG
            CheckSurfaceParameters();
#endif
        }

        public override ICurve[] Split(double Position1, double Position2)
        {
            GeoPoint2D uv1, uv2;
            GeoPoint p;
            ApproximatePosition(Position1, out uv1, out uv2, out p);
            SurfacePoint p1 = new SurfacePoint(p, uv1, uv2);
            ApproximatePosition(Position2, out uv1, out uv2, out p);
            SurfacePoint p2 = new SurfacePoint(p, uv1, uv2);
            List<SurfacePoint> l1 = new List<SurfacePoint>();
            List<SurfacePoint> l2 = new List<SurfacePoint>();
            int i1, i2;
            if (Position1 < Position2)
            {
                i1 = (int)Math.Ceiling(Position1 * (basePoints.Length - 1));
                i2 = (int)Math.Ceiling(Position2 * (basePoints.Length - 1));
            }
            else
            {
                i2 = (int)Math.Ceiling(Position1 * (basePoints.Length - 1));
                i1 = (int)Math.Ceiling(Position2 * (basePoints.Length - 1));
            }
            for (int i = i1; i < i2; ++i)
            {
                if (!Precision.IsEqual(basePoints[i].p3d, p1.p3d) && !Precision.IsEqual(basePoints[i].p3d, p2.p3d)) l1.Add(basePoints[i]);
            }
            for (int i = i2; i < basePoints.Length; ++i)
            {
                if (!Precision.IsEqual(basePoints[i].p3d, p1.p3d) && !Precision.IsEqual(basePoints[i].p3d, p2.p3d)) l2.Add(basePoints[i]);
            }
            for (int i = 1; i < i1; ++i) // hier ist ja geschlossen, also ist der erste und der letzte Basepoint identisch
            {
                if (!Precision.IsEqual(basePoints[i].p3d, p1.p3d) && !Precision.IsEqual(basePoints[i].p3d, p2.p3d)) l2.Add(basePoints[i]);
            }
            if (Position1 < Position2)
            {
                l1.Insert(0, p1);
                l1.Add(p2);
                l2.Insert(0, p2);
                l2.Add(p1);
            }
            else
            {
                l1.Insert(0, p2);
                l1.Add(p1);
                l2.Insert(0, p1);
                l2.Add(p2);
            }
            InterpolatedDualSurfaceCurve dsc1 = new InterpolatedDualSurfaceCurve(surface1.Clone(), surface2.Clone(), l1.ToArray());
            InterpolatedDualSurfaceCurve dsc2 = new InterpolatedDualSurfaceCurve(surface1.Clone(), surface2.Clone(), l2.ToArray());
            return new ICurve[] { dsc1, dsc2 };
        }
        public override bool IsClosed
        {
            get
            {
                return Precision.IsEqual(StartPoint, EndPoint);
            }
        }
        public override void Reverse()
        {
            Array.Reverse(basePoints);
            forwardOriented = !forwardOriented;
            InvalidateSecondaryData();
        }
        public override void Trim(double StartPos, double EndPos)
        {
            // if (StartPos <= 0 && EndPos >= 1) return; ; // nichts zu tun
            List<SurfacePoint> spl = new List<SurfacePoint>();
            GeoPoint2D uv1, uv2;
            GeoPoint p;
            ApproximatePosition(StartPos, out uv1, out uv2, out p);
            spl.Add(new SurfacePoint(p, uv1, uv2));
            if (StartPos > EndPos && IsClosed)
            {
                for (int i = 0; i < basePoints.Length; ++i)
                {
                    double pos = (double)i / (double)(basePoints.Length - 1);
                    // keine fast identischen Punkte zufügen, die führen zu Nullvektoren in der Differenz
                    if (pos > StartPos + 1e-3) spl.Add(basePoints[i]);
                }
                for (int i = 1; i < basePoints.Length; ++i)
                {
                    double pos = (double)i / (double)(basePoints.Length - 1);
                    // keine fast identischen Punkte zufügen, die führen zu Nullvektoren in der Differenz
                    if (pos < EndPos - 1e-3) spl.Add(basePoints[i]);
                    else break;
                }
            }
            else
            {
                for (int i = 0; i < basePoints.Length; ++i)
                {
                    double pos = (double)i / (double)(basePoints.Length - 1);
                    // keine fast identischen Punkte zufügen, die führen zu Nullvektoren in der Differenz
                    if (pos > StartPos + 1e-3 && pos < EndPos - 1e-3) spl.Add(basePoints[i]);
                }
                if (spl.Count == 1)
                {   // es müssen mindesten 3 basepoints vorhanden sein
                    double pos = (EndPos + StartPos) / 2.0;
                    ApproximatePosition(pos, out uv1, out uv2, out p);
                    spl.Add(new SurfacePoint(p, uv1, uv2));
                }
            }
            ApproximatePosition(EndPos, out uv1, out uv2, out p);
            spl.Add(new SurfacePoint(p, uv1, uv2));
            basePoints = spl.ToArray();
#if DEBUG
            CheckSurfaceParameters();
#endif
            InvalidateSecondaryData();
        }
        public override IGeoObject Clone()
        {
            // BRepIntersection createNewEdges erwartet, dass die surfaces erhalten bleiben, also nicht gecloned werden
            // wenn das von anderer Seite anders erwartet wird, dann muss eine methode zum Austausch der Surfaces gemacht werden
#if DEBUG
            CheckSurfaceParameters();
#endif
            SurfacePoint[] spnts = new SurfacePoint[basePoints.Length];
            for (int i = 0; i < basePoints.Length; i++)
            {   // we need a deep copy, independant surface points
                spnts[i] = new SurfacePoint(basePoints[i].p3d, basePoints[i].psurface1, basePoints[i].psurface2);
            }
            return new InterpolatedDualSurfaceCurve(surface1.Clone(), bounds1, surface2.Clone(), bounds2, basePoints.Select(sp => sp.p3d).ToArray(), null, null, isTangential);
            // Clone introduced because of independant surfaces for BRep operations
            // forwardOriented is calculated by the order of the base points
        }
        internal void SetSurfaces(ISurface surface1, ISurface surface2, bool swapped)
        {
#if DEBUG
            bool ok = swapped == surface1.SameGeometry(BoundingRect.UnitBoundingRect, this.surface2, BoundingRect.UnitBoundingRect, 1e-6, out ModOp2D dumy);
#endif
            bool ok1 = surface1.SameGeometry(BoundingRect.UnitBoundingRect, this.surface1, BoundingRect.UnitBoundingRect, Precision.eps, out ModOp2D dumy1);
            bool ok2 = surface2.SameGeometry(BoundingRect.UnitBoundingRect, this.surface2, BoundingRect.UnitBoundingRect, Precision.eps, out ModOp2D dumy2);
            if (ok1 && ok2)
            {
                this.surface1 = surface1;
                this.surface2 = surface2;
                return;
            }
            else
            {
                ok1 = surface1.SameGeometry(BoundingRect.UnitBoundingRect, this.surface2, BoundingRect.UnitBoundingRect, Precision.eps, out dumy1);
                ok2 = surface2.SameGeometry(BoundingRect.UnitBoundingRect, this.surface1, BoundingRect.UnitBoundingRect, Precision.eps, out dumy2);
                if (ok1 && ok2)
                {
                    this.surface1 = surface1;
                    this.surface2 = surface2;
                    // if (swapped)
                    {
                        forwardOriented = !forwardOriented; // falls die surfaces getauscht wurden
                        if (basePoints != null)
                        {
                            for (int i = 0; i < basePoints.Length; i++)
                            {
                                GeoPoint2D tmp = basePoints[i].psurface1;
                                basePoints[i].psurface1 = basePoints[i].psurface2;
                                basePoints[i].psurface2 = tmp;
                            }
                        }
                        hashedPositions.Clear();
                    }
                    return;
                }
                else throw new ApplicationException("Wrong surfaces in InterpolatedDualSurfaceCurve.SetSurfaces");
            }
        }
        public override ICurve CloneModified(ModOp m)
        {
            SurfacePoint[] sp = basePoints.Clone() as SurfacePoint[];
            for (int i = 0; i < sp.Length; ++i)
            {
                sp[i].p3d = m * sp[i].p3d;
            }
            InterpolatedDualSurfaceCurve ipdsc = new InterpolatedDualSurfaceCurve(surface1.GetModified(m), surface2.GetModified(m), sp, forwardOriented);
            return ipdsc;
        }
        public override PlanarState GetPlanarState()
        {
            GeoPoint[] bp = new GeoPoint[basePoints.Length];
            for (int i = 0; i < bp.Length; i++)
            {
                bp[i] = basePoints[i].p3d;
            }
            double maxDist;
            bool isLinear;
            Plane.FromPoints(bp, out maxDist, out isLinear);
            if (isLinear) return PlanarState.UnderDetermined;
            if (maxDist < Precision.eps) return PlanarState.Planar;
            return PlanarState.NonPlanar;
            throw new Exception("The method or operation is not implemented.");
        }
        public override Plane GetPlane()
        {
            if (surface1 is PlaneSurface) return (surface1 as PlaneSurface).Plane;
            if (surface2 is PlaneSurface) return (surface2 as PlaneSurface).Plane;
            GeoPoint[] bp = new GeoPoint[basePoints.Length];
            for (int i = 0; i < bp.Length; i++)
            {
                bp[i] = basePoints[i].p3d;
            }
            double maxDist;
            bool isLinear;
            return Plane.FromPoints(bp, out maxDist, out isLinear);
        }
        public override bool IsInPlane(Plane p)
        {
            if (surface1 is PlaneSurface) return (surface1 as PlaneSurface).Plane.SamePlane(p);
            if (surface2 is PlaneSurface) return (surface2 as PlaneSurface).Plane.SamePlane(p);
            for (int i = 0; i < basePoints.Length; i++)
            {
                if (Math.Abs(p.Distance(basePoints[i].p3d)) > Precision.eps) return false;
            }
            return true;
        }
        public override CADability.Curve2D.ICurve2D GetProjectedCurve(Plane p)
        {
            return base.GetProjectedCurve(p);
        }
        public override string Description
        {
            get
            {
                return StringTable.GetString("GeneralCurve.Description");
            }
        }
        public override bool IsComposed
        {
            get
            {
                return false;
            }
        }
        public override ICurve[] SubCurves
        {
            get { throw new Exception("The method or operation is not implemented."); }
        }

        ICurve IDualSurfaceCurve.Curve3D
        {
            get
            {
                return this;
            }
        }

        ISurface IDualSurfaceCurve.Surface1
        {
            get
            {
                return surface1;
            }
        }

        ICurve2D IDualSurfaceCurve.Curve2D1
        {
            get
            {
                return CurveOnSurface1;
            }
        }

        ISurface IDualSurfaceCurve.Surface2
        {
            get
            {
                return surface2;
            }
        }

        ICurve2D IDualSurfaceCurve.Curve2D2
        {
            get
            {
                return CurveOnSurface2;
            }
        }
        ICurve2D IDualSurfaceCurve.GetCurveOnSurface(ISurface onThisSurface)
        {
            if (onThisSurface == surface1) return CurveOnSurface1;
            else if (onThisSurface == surface2) return CurveOnSurface2;
            else return null;
        }

        void IDualSurfaceCurve.SwapSurfaces()
        {
            (surface1, surface2) = (surface2, surface1);
            (bounds1, bounds2) = (bounds2, bounds1);
            for (int i = 0; i < basePoints.Length; i++)
            {
                GeoPoint2D t = basePoints[i].psurface1;
                basePoints[i].psurface1 = basePoints[i].psurface2;
                basePoints[i].psurface2 = t;
            }
            forwardOriented = !forwardOriented;
            InvalidateSecondaryData();
            //ICurve2D tc = CurveOnSurface1;
            //CurveOnSurface1 ... werden ja jedesmal neu berechnet. oder
        }

        public override ICurve Approximate(bool linesOnly, double maxError)
        {
#if DEBUG
            if (maxError < 0)
            {
                // nur zum Polynomfunktionen Debuggen!
                double[] knots = new double[basePoints.Length];
                for (int i = 0; i < basePoints.Length; i++)
                {
                    knots[i] = (double)i / (basePoints.Length - 1);
                }
                ExplicitPCurve3D dbg = ExplicitPCurve3D.FromCurve(this, knots, 3, true);
                return null;
            }
#endif
            if (linesOnly)
            {
                return Curves.ApproximateLinear(this, maxError);
            }
            else
            {
                ArcLineFitting3D alf = new ArcLineFitting3D(this, maxError, true, Math.Max(GetBasePoints().Length, 5));
                return alf.Approx;
            }
        }
        public override double[] TangentPosition(GeoVector direction)
        {
            throw new Exception("The method or operation is not implemented.");
        }
        public override double[] GetSelfIntersections()
        {
            throw new Exception("The method or operation is not implemented.");
        }
        public override bool SameGeometry(ICurve other, double precision)
        {
            if ((StartPoint | other.StartPoint) < precision && (EndPoint | other.EndPoint) < precision)
            {
                // gleiche Richtung
                for (double par = 0.25; par < 1.0; par += 0.25)
                {
                    if ((PointAt(par) | other.PointAt(par)) > precision) return false;
                }
                return true;
            }
            if ((EndPoint | other.StartPoint) < precision && (StartPoint | other.EndPoint) < precision)
            {
                for (double par = 0.25; par < 1.0; par += 0.25)
                {
                    if ((PointAt(par) | other.PointAt(1 - par)) > precision) return false;
                }
                return true;
            }
            return false;
        }
        protected override double[] GetBasePoints()
        {
            double[] res = new double[basePoints.Length];
            for (int i = 0; i < res.Length; ++i)
            {
                res[i] = (double)i / (double)(basePoints.Length - 1);
            }
            return res;
        }
        #endregion
        #region ISerializable members
        protected InterpolatedDualSurfaceCurve(SerializationInfo info, StreamingContext context)
            : base(info, context)
        {
            surface1 = info.GetValue("Surface1", typeof(ISurface)) as ISurface;
            surface2 = info.GetValue("Surface2", typeof(ISurface)) as ISurface;
            basePoints = info.GetValue("BasePoints", typeof(SurfacePoint[])) as SurfacePoint[];
            hashedPositions = new SortedList<double, SurfacePoint>();
            try
            {
                forwardOriented = (bool)info.GetValue("ForwardOriented", typeof(bool));
            }
            catch (SerializationException)
            {
                forwardOriented = true; // fehlte früher mit subtilen Folgen
            }
        }

        internal void SurfacesSwapped()
        {
            forwardOriented = !forwardOriented;
        }

        /// <summary>
        /// Implements <see cref="ISerializable.GetObjectData"/>
        /// </summary>
        /// <param name="info">The <see cref="System.Runtime.Serialization.SerializationInfo"/> to populate with data.</param>
        /// <param name="context">The destination (<see cref="System.Runtime.Serialization.StreamingContext"/>) for this serialization.</param>
        public override void GetObjectData(SerializationInfo info, StreamingContext context)
        {
            base.GetObjectData(info, context);
            info.AddValue("Surface1", surface1);
            info.AddValue("Surface2", surface2);
            info.AddValue("BasePoints", basePoints);
            info.AddValue("ForwardOriented", forwardOriented);
#if DEBUG
            CheckSurfaceParameters();
#endif
        }
        public override void GetObjectData(IJsonWriteData data)
        {
            base.GetObjectData(data);
            data.AddProperty("Surface1", surface1);
            data.AddProperty("Surface2", surface2);
            data.AddProperty("BasePoints", basePoints);
            data.AddProperty("ForwardOriented", forwardOriented);
            data.AddProperty("IsTangential", isTangential);
            data.AddProperty("Bounds1", bounds1);
            data.AddProperty("Bounds2", bounds2);
        }

        public override void SetObjectData(IJsonReadData data)
        {
            base.SetObjectData(data);
            surface1 = data.GetPropertyOrDefault<ISurface>("Surface1");
            surface2 = data.GetPropertyOrDefault<ISurface>("Surface2");
            basePoints = data.GetPropertyOrDefault<SurfacePoint[]>("BasePoints");
            forwardOriented = (bool)data.GetProperty("ForwardOriented");
            if (data.Version >= 1)
            {
                isTangential = data.GetPropertyOrDefault<bool>("IsTangential");
                bounds1 = data.GetPropertyOrDefault<BoundingRect>("Bounds1");
                bounds2 = data.GetPropertyOrDefault<BoundingRect>("Bounds2");
            }
            data.RegisterForSerializationDoneCallback(this);
        }
        void IJsonSerializeDone.SerializationDone(JsonSerialize jsonSerialize)
        {
            if (jsonSerialize.GetTypeVersion(this.GetType()) < 1)
            {
                // Parameter isTangential was introduced in version 1
                // we have to determine it
                isTangential = true;
                for (int i = 1; i < basePoints.Length - 1; i++)
                {
                    GeoVector n1 = surface1.GetNormal(basePoints[i].psurface1).Normalized;
                    GeoVector n2 = surface2.GetNormal(basePoints[i].psurface2).Normalized;
                    if ((n1 ^ n2).Length > 1e-4)
                    {
                        isTangential = false;
                        break;

                    }
                }
                if (surface1 is ISurfaceImpl simpl1)
                {
                    if (simpl1.usedArea.IsEmpty() || simpl1.usedArea.IsInfinite)
                    {
                        BoundingRect ext = BoundingRect.EmptyBoundingRect;
                        for (int i = 0; i < basePoints.Length; i++)
                        {
                            ext.MinMax(basePoints[i].psurface1);
                        }
                        simpl1.usedArea = ext;
                    }
                    bounds1 = simpl1.usedArea;
                }
                if (surface2 is ISurfaceImpl simpl2)
                {
                    if (simpl2.usedArea.IsEmpty() || simpl2.usedArea.IsInfinite)
                    {
                        BoundingRect ext = BoundingRect.EmptyBoundingRect;
                        for (int i = 0; i < basePoints.Length; i++)
                        {
                            ext.MinMax(basePoints[i].psurface2);
                        }
                        simpl2.usedArea = ext;
                    }
                    bounds2 = simpl2.usedArea;
                }
            }
#if DEBUG
            CheckSurfaceParameters();
#endif
        }
        void IDeserializationCallback.OnDeserialization(object sender)
        {
            if (surface1 is ISurfaceImpl simpl1)
            {
                if (simpl1.usedArea.IsEmpty() || simpl1.usedArea.IsInfinite)
                {
                    BoundingRect ext = BoundingRect.EmptyBoundingRect;
                    for (int i = 0; i < basePoints.Length; i++)
                    {
                        ext.MinMax(basePoints[i].psurface1);
                    }
                    simpl1.usedArea = ext;
                }
            }
            if (surface2 is ISurfaceImpl simpl2)
            {
                if (simpl2.usedArea.IsEmpty() || simpl2.usedArea.IsInfinite)
                {
                    BoundingRect ext = BoundingRect.EmptyBoundingRect;
                    for (int i = 0; i < basePoints.Length; i++)
                    {
                        ext.MinMax(basePoints[i].psurface2);
                    }
                    simpl2.usedArea = ext;
                }
            }
        }

        int IExportStep.Export(ExportStep export, bool topLevel)
        {
            return (ToBSpline(export.Precision) as IExportStep).Export(export, topLevel);
        }

        IDualSurfaceCurve[] IDualSurfaceCurve.Split(double v)
        {
            ICurve[] crvs = (this as ICurve).Split(v);
            IDualSurfaceCurve[] res = new IDualSurfaceCurve[crvs.Length];
            for (int i = 0; i < crvs.Length; i++)
            {
                res[i] = crvs[i] as IDualSurfaceCurve;
            }
            return res;
        }

        GeoVector IOrientation.OrientationAt(double u)
        {
            GeoPoint2D uv1, uv2;
            GeoPoint p;
            ApproximatePosition(u, out uv1, out uv2, out p);
            GeoVector v = surface1.GetNormal(uv1).Normalized + surface2.GetNormal(uv2).Normalized;
            return v;
        }

        void IDualSurfaceCurve.Trim(GeoPoint startPoint, GeoPoint endPoint)
        {
            if (Precision.IsEqual(startPoint, StartPoint) && Precision.IsEqual(endPoint, EndPoint)) return;
            double posSp = PositionOf(startPoint);
            double posEp = PositionOf(endPoint);
            Trim(posSp, posEp);
        }

        #endregion
    }
}
