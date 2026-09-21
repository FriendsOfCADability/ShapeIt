using CADability.Curve2D;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.Substitutes;
using CADability.UserInterface;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using static CADability.InterpolatedDualSurfaceCurve.SurfacePoint;

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
    public class InterpolatedDualSurfaceCurve : GeneralCurve, IDualSurfaceCurve, IJsonSerialize, IExportStep, IJsonSerializeDone, IOrientation
    {
        ISurface surface1; // the two surfaces
        ISurface surface2;
#if DEBUG
        internal
#endif
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
            /// <summary>
            /// A shift by whole periods, made by <see cref="Move"/>. It is added to the uv values stored in the 3d curve
            /// and to the approximation computed from the 3d curve. So moving this 2d curve moves all of it and leaves
            /// the 3d curve unchanged, which is shared with the edge and with the 2d curve on the other surface.
            /// </summary>
            GeoVector2D offset = GeoVector2D.NullVector;
            public ProjectedCurve(InterpolatedDualSurfaceCurve curve3d, bool onSurface1)
            {
                this.curve3d = curve3d;
                this.onSurface1 = onSurface1;
                reversed = false;
            }
            public ProjectedCurve(InterpolatedDualSurfaceCurve curve3d, ProjectedCurve toCloneFrom)
            {
                this.curve3d = curve3d;
                this.onSurface1 = toCloneFrom.onSurface1;
                reversed = toCloneFrom.reversed;
                offset = toCloneFrom.offset;
                BSpline2D init = ApproxBSpline;
            }
            public ProjectedCurve(InterpolatedDualSurfaceCurve curve3d, bool onSurface1, bool reversed)
            {
                this.curve3d = curve3d;
                this.onSurface1 = onSurface1;
                this.reversed = reversed;
            }
            protected override void GetTriangulationBasis(out GeoPoint2D[] points, out GeoVector2D[] directions, out double[] parameters)
            {
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
            }
            protected BSpline2D ApproxBSpline
            {
                get
                {
                    // The BSpline always runs in the direction of the 3d curve. A reversed projected curve is
                    // marked by the "reversed" flag instead, which PointAt and DirectionAt take into account.
                    if (approxBSpline != null) return approxBSpline;
                    // we need a BSpline here, which is precise and has the same parametrisation as the curve3d
                    Func<double, GeoPoint2D> curve = (pos =>
                    {
                        GeoPoint p = curve3d.PointAt(pos);
                        if (onSurface1)
                        {
                            GeoPoint2D uv = curve3d.surface1.PositionOf(p);
                            SurfaceHelper.AdjustPeriodic(curve3d.surface1, curve3d.bounds1, ref uv);
                            return uv + offset;
                        }
                        else
                        {
                            GeoPoint2D uv = curve3d.surface2.PositionOf(p);
                            SurfaceHelper.AdjustPeriodic(curve3d.surface2, curve3d.bounds2, ref uv);
                            return uv + offset;
                        }
                    });
                    approxBSpline = BSpline2D.Approximate(curve, Precision.eps, 0, 1);
                    if (DualSurfaceCurveDiagnostics.Enabled) DualSurfaceCurveDiagnostics.ObserveProjectedCurve(onSurface1 ? curve3d.surface1 : curve3d.surface2,
                        onSurface1 ? curve3d.bounds1 : curve3d.bounds2, curve3d.basePoints, onSurface1, offset, approxBSpline);
                    return approxBSpline;
                }
            }
            public override double GetArea()
            {
                double a = ApproxBSpline.GetArea();
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
                // since the ApproxBSpline  has the same parametrisation as the curve3d, we can use it directly
                if (reversed) return -ApproxBSpline.DirectionAt(1.0 - par);
                else return ApproxBSpline.DirectionAt(par);
            }
            public override GeoPoint2D PointAt(double par)
            {
                if (reversed) par = 1.0 - par;
                return ApproxBSpline.PointAt(par);
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
                        if (onSurface1) return curve3d.basePoints[curve3d.basePoints.Length - 1].psurface1 + offset;
                        else return curve3d.basePoints[curve3d.basePoints.Length - 1].psurface2 + offset;
                    }
                    else
                    {
                        if (onSurface1) return curve3d.basePoints[0].psurface1 + offset;
                        else return curve3d.basePoints[0].psurface2 + offset;
                    }
                }
                set
                {   // das wird gebraucht, um kleine Lücken in einem Border zu schließen
                    DualSurfaceCurveDiagnostics.Count("ProjectedCurve.StartPoint setter: writes uv into the shared 3d curve");
                    if (reversed)
                    {
                        if (onSurface1) curve3d.basePoints[curve3d.basePoints.Length - 1].psurface1 = value - offset;
                        else curve3d.basePoints[curve3d.basePoints.Length - 1].psurface2 = value - offset;
                    }
                    else
                    {
                        if (onSurface1) curve3d.basePoints[0].psurface1 = value - offset;
                        else curve3d.basePoints[0].psurface2 = value - offset;
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
                        if (onSurface1) return curve3d.basePoints[0].psurface1 + offset;
                        else return curve3d.basePoints[0].psurface2 + offset;
                    }
                    else
                    {
                        if (onSurface1) return curve3d.basePoints[curve3d.basePoints.Length - 1].psurface1 + offset;
                        else return curve3d.basePoints[curve3d.basePoints.Length - 1].psurface2 + offset;
                    }
                }
                set
                {
                    DualSurfaceCurveDiagnostics.Count("ProjectedCurve.EndPoint setter: writes uv into the shared 3d curve");
                    if (reversed)
                    {
                        if (onSurface1) curve3d.basePoints[0].psurface1 = value - offset;
                        else curve3d.basePoints[0].psurface2 = value - offset;
                    }
                    else
                    {
                        if (onSurface1) curve3d.basePoints[curve3d.basePoints.Length - 1].psurface1 = value - offset;
                        else curve3d.basePoints[curve3d.basePoints.Length - 1].psurface2 = value - offset;
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
                DualSurfaceCurveDiagnostics.Count("ProjectedCurve.Trim: clones and trims the 3d curve");
                double sp = StartPos;
                double ep = EndPos;
                InterpolatedDualSurfaceCurve clone = curve3d.Clone() as InterpolatedDualSurfaceCurve;
                clone.Trim(sp, ep);
                ProjectedCurve res = new ProjectedCurve(clone, onSurface1);
                res.reversed = reversed;
                res.offset = offset;
                res.ClearTriangulation();
                return res;
            }
            public override void Reverse()
            {
                reversed = !reversed;
                base.ClearTriangulation();
                approxBSpline = null;
            }
            public override ICurve2D Clone()
            {
                DualSurfaceCurveDiagnostics.Count("ProjectedCurve.Clone: clones the 3d curve");
                ProjectedCurve res = new ProjectedCurve(curve3d.Clone() as InterpolatedDualSurfaceCurve, onSurface1);
                res.reversed = reversed;
                res.offset = offset;
                res.ClearTriangulation();
                res.UserData.CloneFrom(UserData);
                return res;
            }
            public override ICurve2D CloneReverse(bool reverse)
            {
                DualSurfaceCurveDiagnostics.Count("ProjectedCurve.CloneReverse: clones the 3d curve");
                ProjectedCurve res = new ProjectedCurve(curve3d.Clone() as InterpolatedDualSurfaceCurve, onSurface1);
                if (reverse) res.reversed = !reversed;
                else res.reversed = reversed;
                res.offset = offset;
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
                DualSurfaceCurveDiagnostics.Count(m.IsIdentity ? "ProjectedCurve.GetModified, identity" : "ProjectedCurve.GetModified: modifies the base points of the shared 3d curve");
                curve3d.ModifySurfacePoints(onSurface1, m);
                ProjectedCurve res = new ProjectedCurve(curve3d, onSurface1); // do not clone curve3d!
                res.reversed = reversed;
                res.offset = m * offset; // the stored uv values have been modified, the offset follows the linear part
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
                DualSurfaceCurveDiagnostics.Count(x == 0.0 && y == 0.0 ? "ProjectedCurve.Move(0, 0)" : "ProjectedCurve.Move by periods");
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
                // This used to shift the uv values stored in the shared 3d curve, but not the approximation, which is
                // computed from the 3d points: afterwards StartPoint and EndPoint were a period away from PointAt.
                offset = offset + new GeoVector2D(x, y);
                base.ClearTriangulation();
                approxBSpline = null;
            }
            #region ISerializable Members
            protected ProjectedCurve(SerializationInfo info, StreamingContext context)
                : base(info, context)
            {
                curve3d = info.GetValue("Curve3d", typeof(InterpolatedDualSurfaceCurve)) as InterpolatedDualSurfaceCurve;
                onSurface1 = info.GetBoolean("OnSurface1");
                reversed = info.GetBoolean("Reversed");
                try
                {
                    offset = (GeoVector2D)info.GetValue("Offset", typeof(GeoVector2D));
                }
                catch (SerializationException)
                {   // written before the offset existed
                    offset = GeoVector2D.NullVector;
                }
            }
            void ISerializable.GetObjectData(SerializationInfo info, StreamingContext context)
            {
                base.GetObjectData(info, context);
                info.AddValue("Curve3d", curve3d);
                info.AddValue("OnSurface1", onSurface1);
                info.AddValue("Reversed", reversed);
                info.AddValue("Offset", offset);
            }
            protected ProjectedCurve() { } // needed for IJsonSerialize
            public void GetObjectData(IJsonWriteData data)
            {
                base.JSonGetObjectData(data);
                data.AddProperty("Curve3d", curve3d);
                data.AddProperty("OnSurface1", onSurface1);
                data.AddProperty("Reversed", reversed);
                if (offset.x != 0.0 || offset.y != 0.0) data.AddProperty("Offset", offset);
            }

            public void SetObjectData(IJsonReadData data)
            {
                base.JSonSetObjectData(data);
                curve3d = data.GetProperty<InterpolatedDualSurfaceCurve>("Curve3d");
                onSurface1 = data.GetProperty<bool>("OnSurface1");
                reversed = data.GetProperty<bool>("Reversed");
                offset = data.GetPropertyOrDefault<GeoVector2D>("Offset");
            }

            #endregion
#if DEBUG
            public GeoObjectList Debug
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
                    offset = pc.offset;
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
                DualSurfaceCurveDiagnostics.Count("ProjectedCurve.SetCurve3d: re-links a 2d curve to another 3d curve");
                if ((c3d.StartPoint | curve3d.StartPoint) + (c3d.EndPoint | curve3d.EndPoint) > (c3d.StartPoint | curve3d.EndPoint) + (c3d.EndPoint | curve3d.StartPoint)) reversed = !reversed;
                curve3d = c3d;
                // es muss sich hier um eine geometrisch identische Kurve handeln (Richtung?)
                ClearTriangulation();
            }

            public override bool TryPointDeriv2At(double position, out GeoPoint2D point, out GeoVector2D deriv, out GeoVector2D deriv2)
            {   // the approximation has the parametrization of the 3d curve, a reversed curve runs through it backwards
                bool ok = ApproxBSpline.TryPointDeriv2At(reversed ? 1.0 - position : position, out point, out deriv, out deriv2);
                if (reversed) deriv = -deriv; // the second derivative keeps its sign
                return ok;
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
            DualSurfaceCurveDiagnostics.Count("IDSC.ModifySurfacePoints" + (m.Determinant < 0 ? ", orientation reversed" : ""));
            for (int i = 0; i < basePoints.Length; i++)
            {
                if (onSurface1) basePoints[i].psurface1 = m * basePoints[i].psurface1;
                else basePoints[i].psurface2 = m * basePoints[i].psurface2;
            }
            if (m.Determinant < 0) forwardOriented = !forwardOriented; // die Orientierung der Fläche hat sich umgedreht, damit ist auch das Kreuzprodukt andersrum
            InvalidateSecondaryData();
        }

        protected InterpolatedDualSurfaceCurve()
        {
            hashedPositions = new SortedList<double, SurfacePoint>();
#if DEBUG
            id = idcnt++;
#endif
        }
        internal InterpolatedDualSurfaceCurve(ISurface surface1, ISurface surface2, SurfacePoint[] basePoints, bool isTangential = false)
            : this(surface1, BoundingRect.EmptyBoundingRect, surface2, BoundingRect.EmptyBoundingRect, basePoints, isTangential)
        {   // we should always have bounds
        }
        internal InterpolatedDualSurfaceCurve(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, SurfacePoint[] basePoints, bool isTangential = false)
            : this()
        {
            DualSurfaceCurveDiagnostics.ConstructionProbe probe = DualSurfaceCurveDiagnostics.BeginConstruction(surface1, bounds1, surface2, bounds2);
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
            if (basePoints.Length == 2) RefineBasePoints();
            CheckSurfaceExtents();
            AdjustBasePointsPeriodic();
            BSpline toUpdateBasepoints = ApproxBSpline;
            DualSurfaceCurveDiagnostics.EndConstruction(probe, this.surface1, this.surface2, this.basePoints, this.isTangential);
        }
        private void Init()
        {
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
            if (basePoints.Length == 2) RefineBasePoints();
            CheckSurfaceExtents();
            AdjustBasePointsPeriodic();
            BSpline toUpdateBasepoints = ApproxBSpline;
        }
        public InterpolatedDualSurfaceCurve(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, GeoPoint startPoint, GeoPoint endPoint, bool isTangential = false)
            : this()
        {
            DualSurfaceCurveDiagnostics.ConstructionProbe probe = DualSurfaceCurveDiagnostics.BeginConstruction(surface1, bounds1, surface2, bounds2);
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
            DualSurfaceCurveDiagnostics.EndConstruction(probe, this.surface1, this.surface2, this.basePoints, this.isTangential);
        }
        public InterpolatedDualSurfaceCurve(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, GeoPoint[] pts, List<GeoPoint2D> uvpts1 = null, List<GeoPoint2D> uvpts2 = null, bool isTangential = false, BSpline approxBSpline = null)
        : this(surface1, bounds1, surface2, bounds2, pts.ToList(), uvpts1, uvpts2, isTangential, approxBSpline)
        {
        }
        public InterpolatedDualSurfaceCurve(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2, List<GeoPoint> pts, List<GeoPoint2D> uvpts1 = null, List<GeoPoint2D> uvpts2 = null, bool isTangential = false, BSpline approxBSpline = null)
            : this()
        {
            DualSurfaceCurveDiagnostics.ConstructionProbe probe = DualSurfaceCurveDiagnostics.BeginConstruction(surface1, bounds1, surface2, bounds2);
            // die Bounds dienen dazu bei periodischen Flächen die richtigen Parameterwerte zu finden
            // diese Parameterbereiche sind wichtig, es darf also niemal PositionOf verwendet werden, sonst müssen wir
            // bounds1 und bounds2 speichern, um in den richtigen Bereich zu kommen
            // bounds may change in future:
            // each surface must provide its domain (at least in the periodic parameters) and must return the correct value upon PositionOf or GetProjectedCurve.
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
                if (surface1.IsVPeriodic && Math.Abs(points[0].psurface1.y - points[1].psurface1.y) > surface1.VPeriod / 3.0)
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
                if (surface2.IsVPeriodic && Math.Abs(points[0].psurface2.y - points[1].psurface2.y) > surface2.VPeriod / 3.0)
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
                        sp.psurface2 = surface2.PositionOf(sp.p3d);
                        SurfaceHelper.AdjustPeriodic(surface1, bounds1, ref sp.psurface1);
                        SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref sp.psurface2);
                        points.Insert(1, sp);
                    }
                }
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
                ApproximatePosition((double)i / (double)(basePoints.Length - 1), out uv1, out uv2, out basePoints[i].p3d);
                basePoints[i].psurface1 = uv1;
                basePoints[i].psurface2 = uv2;
                hashedPositions.Clear(); // die Werte hier sind unnütz, da die basePoints sich ja immer noch ändern
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

            AdjustBasePointsPeriodic();
            this.approxBSpline = approxBSpline; // may be null, the it will be calculated in the next line
            BSpline bsp = ApproxBSpline; // make sure it is created and the basepoints are refined
            CheckPeriodic(); // erst nach dieser Schleife, denn ApproximatePosition mach die uv-position evtl. falsch
            CheckSurfaceExtents();
            AdjustBasePointsPeriodic();

            DualSurfaceCurveDiagnostics.EndConstruction(probe, this.surface1, this.surface2, this.basePoints, this.isTangential);
        }
        internal void CheckSurfaceExtents()
        {   // without bounds the domain of the surface is used, and where there is none the extent of the base points.
            // The curve never writes a domain onto a surface: the surface belongs to a face and is shared with other edges.
            if (bounds1.IsEmpty() && surface1 is ISurfaceImpl simpl1) bounds1 = simpl1.HasDomain ? simpl1.Domain : GetBoundingRect(true);
            if (bounds2.IsEmpty() && surface2 is ISurfaceImpl simpl2) bounds2 = simpl2.HasDomain ? simpl2.Domain : GetBoundingRect(false);
        }
        internal void Repair(BoundingRect bounds1, BoundingRect bounds2)
        {
            DualSurfaceCurveDiagnostics.Count("IDSC.Repair");
            BoundingBox ext = BoundingBox.EmptyBoundingBox;
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
                DualSurfaceCurveDiagnostics.Count("IDSC.Repair: needed a repair");
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
        }
        internal void AdjustPeriodic(ref GeoPoint2D uv1, ref GeoPoint2D uv2)
        {
            if (bounds1.IsEmpty() || bounds1.IsInfinite) bounds1 = (surface1 as ISurfaceImpl).Domain;
            if (bounds2.IsEmpty() || bounds2.IsInfinite) bounds2 = (surface2 as ISurfaceImpl).Domain;
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
            if ((uvfixed & SurfaceFixFlags.Surface1) != 0)
            {
                double u = basePoints.Sum(b => b.psurface1.x) / basePoints.Length;
                double v = basePoints.Sum(b => b.psurface1.y) / basePoints.Length;
                double du = u - (bounds1.Left + bounds1.Right) / 2.0;
                double dv = v - (bounds1.Bottom + bounds1.Top) / 2.0;
                du = surface1.IsUPeriodic ? surface1.UPeriod * Math.Round(du / surface1.UPeriod) : 0.0;
                dv = surface1.IsVPeriodic ? surface1.VPeriod * Math.Round(dv / surface1.VPeriod) : 0.0;
                if (du != 0.0 || dv != 0.0)
                {
                    for (int i = 0; i < basePoints.Length; i++)
                    {
                        basePoints[i].psurface1.x -= du;
                        basePoints[i].psurface1.y -= dv;
                    }
                }
            }
            if ((uvfixed & SurfaceFixFlags.Surface2) != 0)
            {
                double u = basePoints.Sum(b => b.psurface2.x) / basePoints.Length;
                double v = basePoints.Sum(b => b.psurface2.y) / basePoints.Length;
                double du = u - (bounds2.Left + bounds2.Right) / 2.0;
                double dv = v - (bounds2.Bottom + bounds2.Top) / 2.0;
                du = surface2.IsUPeriodic ? surface2.UPeriod * Math.Round(du / surface2.UPeriod) : 0.0;
                dv = surface2.IsVPeriodic ? surface2.VPeriod * Math.Round(dv / surface2.VPeriod) : 0.0;
                if (du != 0.0 || dv != 0.0)
                {
                    for (int i = 0; i < basePoints.Length; i++)
                    {
                        basePoints[i].psurface2.x -= du;
                        basePoints[i].psurface2.y -= dv;
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
            approxBSpline = null;
        }
        internal void ReplaceSurface(ISurface oldSurface, ISurface newSurface)
        {   // die beiden surfaces müssen geometrisch identisch sein
            if (surface1 == oldSurface) surface1 = newSurface;
            if (surface2 == oldSurface) surface2 = newSurface;
        }
        internal void ReplaceSurface(ISurface oldSurface, ISurface newSurface, ModOp2D oldToNew)
        {   // die beiden surfaces müssen geometrisch identisch sein
            DualSurfaceCurveDiagnostics.Count("IDSC.ReplaceSurface with ModOp2D");
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
            else if (surface1.SameGeometry((surface1 as ISurfaceImpl).Domain, oldSurface, (oldSurface as ISurfaceImpl).Domain, Precision.eps, out ModOp2D dumy))
            {
                surface1 = newSurface;
                ModifySurfacePoints(true, oldToNew);
            }
            else if (surface2.SameGeometry((surface2 as ISurfaceImpl).Domain, oldSurface, (oldSurface as ISurfaceImpl).Domain, Precision.eps, out dumy))
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
            // this used to pass forwardOriented, which bound to the parameter isTangential
            InterpolatedDualSurfaceCurve res = new InterpolatedDualSurfaceCurve(surface1, surface2, basePoints.Clone() as SurfacePoint[], isTangential);
            res.Trim(startPos, endPos);
            c1trimmed = new ProjectedCurve(res, c1);
            c2trimmed = new ProjectedCurve(res, c2);
            return res;
        }
        public BSpline ToBSpline(double precision)
        {
            return ApproxBSpline; // better than through basepoints
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
                    fc.ColorDef = new CADability.Attribute.ColorDef("Surface1", Color.Red);
                    res.Add(fc);
                    u = surface2.UDirection(basePoints[i].psurface2);
                    v = surface2.VDirection(basePoints[i].psurface2);
                    pls = new PlaneSurface(pnts[i], u, v, u ^ v);
                    fc = Face.MakeFace(pls, new SimpleShape(new BoundingRect(-1, -1, 1, 1)));
                    fc.ColorDef = new CADability.Attribute.ColorDef("Surface1", Color.Green);
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
                pl.ColorDef = new Attribute.ColorDef("org", Color.Red);
                res.Add(pl);
                pl = Polyline.Construct();
                pnts = new GeoPoint[basePoints.Length];
                for (int i = 0; i < basePoints.Length; ++i)
                {
                    pnts[i] = surface1.PointAt(basePoints[i].psurface1);
                }
                pl.SetPoints(pnts, false);
                pl.ColorDef = new Attribute.ColorDef("surf1", Color.Green);
                res.Add(pl);
                pl = Polyline.Construct();
                pnts = new GeoPoint[basePoints.Length];
                for (int i = 0; i < basePoints.Length; ++i)
                {
                    pnts[i] = surface2.PointAt(basePoints[i].psurface2);
                }
                pl.SetPoints(pnts, false);
                pl.ColorDef = new Attribute.ColorDef("surf1", Color.Blue);
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
        public DebuggerContainer DebugFaces
        {
            get
            {
                DebuggerContainer res = new DebuggerContainer();
                res.Add(Face.MakeFace(surface1, bounds1), Color.MediumVioletRed);
                res.Add(Face.MakeFace(surface2, bounds2), Color.SeaShell);
                return res;
            }
        }
#endif
        #region IGeoObject override
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.IGeoObjectImpl.GetBoundingCube ()"/>
        /// </summary>
        /// <returns></returns>
        public override BoundingBox GetBoundingCube()
        {   // not the base points: the curve bulges out between them. PointAt is the approximating spline, so its extent is exact
            return ApproxBSpline.GetBoundingCube();
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
        }
        /// <summary>
        /// Overrides <see cref="CADability.GeoObject.IGeoObjectImpl.GetExtent (double)"/>
        /// </summary>
        /// <param name="precision"></param>
        /// <returns></returns>
        public override BoundingBox GetExtent(double precision)
        {
            return ApproxBSpline.GetBoundingCube();
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
                    if (d > 1.5 * length || basePoints.Length == 2)
                    {
                        changed = true;
                        approxBSpline = null;
                        hashedPositions.Clear();
                        ApproximatePosition((i + 0.5) / (basePoints.Length - 1), out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p);
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

        private BSpline ApproxBSpline
        {
            get
            {
                if (approxBSpline != null) return approxBSpline;
                BSpline bsp = BSpline.Construct();
                bsp.ThroughPoints(BasePoints, 3, false);
                // BasePoints are unevenly distributed. The parameter of bsp is running much more evenly
                // since the knots are calculated according to the chord length
                // setting approxBSpline changes the "speed" of the parameter, makes it more even
                approxBSpline = bsp;
                hashedPositions.Clear(); // don't use hased positions, they are no more correct
                Func<double, GeoPoint> curve = (pos) => // input parameter for BSpline.Approximate
                {
                    ApproximatePosition(pos, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p);
                    return p;
                };
                approxBSpline = BSpline.Approximate(curve);
                hashedPositions.Clear(); // don't use hashed positions, they are no more correct
                return approxBSpline;
            }
        }

        /// <summary>
        /// The minimizer also reports success when it stalls somewhere without having found a common point of the two
        /// surfaces and the plane. And near a node of the intersection it may converge to the other branch, whose tangent
        /// crosses the plane at a large angle. <paramref name="planeFromSpline"/>: the plane is perpendicular to the
        /// approximating spline, so its normal is a good estimate of the tangent. A plane perpendicular to a chord
        /// between two base points is not, so the tangent is not checked then.
        /// </summary>
        private bool IsPlausibleIntersection(GeoPoint2D uv1, GeoPoint2D uv2, Plane plane, bool planeFromSpline)
        {
            GeoPoint p1 = surface1.PointAt(uv1), p2 = surface2.PointAt(uv2);
            double tolerance = 100 * Precision.eps;
            if ((p1 | p2) > tolerance) return false;
            if (Math.Abs(plane.Distance(p1)) > tolerance) return false;
            if (!planeFromSpline) return true;
            GeoVector n1 = surface1.GetNormal(uv1), n2 = surface2.GetNormal(uv2);
            if (n1.IsNullVector() || n2.IsNullVector()) return true;
            GeoVector tangent = n1.Normalized ^ n2.Normalized;
            if (tangent.Length < 1e-2) return true; // the surfaces (almost) touch here, n1 x n2 is no usable tangent
            return Math.Abs(tangent.Normalized * plane.Normal.Normalized) >= Math.Cos(30.0 / 180.0 * Math.PI);
        }
        private void ApproximatePosition(double position, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p)
        {
            lock (hashedPositions)
            {
                // Zuerst nachsehen, ob der Punkt schon bekannt ist
                (bool hasLower, double lowerKey, SurfacePoint lowerValue, bool hasUpper, double upperKey, SurfacePoint upperValue, bool exact, int exactIndex) = hashedPositions.Neighbors(position);
                if (exact)
                {
                    SurfacePoint found = hashedPositions.Values[exactIndex];
                    p = found.p3d;
                    uv1 = found.psurface1;
                    uv2 = found.psurface2;
                    DualSurfaceCurveDiagnostics.RecordCacheHit();
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
                    GeoVector normal = (approxBSpline as ICurve).DirectionAt(position);
                    normalPlane = new Plane((approxBSpline as ICurve).PointAt(position), normal);
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
                        if (DualSurfaceCurveDiagnostics.Enabled) DualSurfaceCurveDiagnostics.RecordRefinement("tangential solver", false,
                            approxBSpline != null, isTangential, surface1, uv1, surface2, uv2, normalPlane, p);
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
                    if (BoxedSurfaceExtension.SurfacesIntersectionLM(ps, surface1, surface2, ref uvplane, ref uv1, ref uv2, ref p)
                        && IsPlausibleIntersection(uv1, uv2, normalPlane, approxBSpline != null))
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
                        if (DualSurfaceCurveDiagnostics.Enabled) DualSurfaceCurveDiagnostics.RecordRefinement("transversal solver (LM)", false,
                            approxBSpline != null, isTangential, surface1, uv1, surface2, uv2, normalPlane, p);
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
                    if (DualSurfaceCurveDiagnostics.Enabled) DualSurfaceCurveDiagnostics.RecordRefinement(
                        isTangential ? "fallback after the tangential solver failed" : "fallback after the transversal solver failed", true,
                        approxBSpline != null, isTangential, surface1, uv1, surface2, uv2, normalPlane, p);
                }
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
                    if (v.Length < 1e-5) return adir; // if the two surfaces are tangential at the start point, then the cross product is zero and we take the direction of the approximating BSpline)
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
                    if (v.Length < 1e-5) return adir; // if the two surfaces are tangential at the end point, then the cross product is zero and we take the direction of the approximating BSpline
                    if (!forwardOriented) v.Reverse();
                    v.Length = adir.Length; // make the same lengt as the approximating BSpline would have. This is very close to the factual length
                    return v;
                }
            }
        }
        public override GeoVector DirectionAt(double Position)
        {
            return (ApproxBSpline as ICurve).DirectionAt(Position);
        }
        public override GeoPoint PointAt(double Position)
        {
            return (ApproxBSpline as ICurve).PointAt(Position);
        }
        public override double PositionAtLength(double position)
        {
            return (ApproxBSpline as ICurve).PositionAtLength(position);
        }
        public override double PositionOf(GeoPoint p)
        {
            double ppos = TetraederHull.PositionOf(p);
            double pos1 = (ApproxBSpline as ICurve).PositionOf(p);

            if ((pos1 != double.MaxValue) && (PointAt(pos1) | p) < (PointAt(ppos) | p))
                return pos1;

            return ppos;
        }
        public override double PositionOf(GeoPoint p, double prefer)
        {
            return (ApproxBSpline as ICurve).PositionOf(p, prefer);
        }
        public override double PositionOf(GeoPoint p, Plane pl)
        {   // FindSnapPoint uses this
            return (ApproxBSpline as ICurve).PositionOf(p, pl);
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
        /// <summary>
        /// The positions of the base points on this curve. They come from the approximating spline, which is
        /// parametrized by the chord length of the base points: base point i is in general not at i/(n-1).
        /// </summary>
        private double[] BasePointPositions()
        {
            double[] res = new double[basePoints.Length];
            for (int i = 1; i < basePoints.Length - 1; i++) res[i] = PositionOf(basePoints[i].p3d);
            res[basePoints.Length - 1] = 1.0;
            return res;
        }
        public override ICurve[] Split(double Position)
        {
            double[] positions = BasePointPositions();
            SurfacePoint splitPoint;
            int atBasePoint = Array.FindIndex(positions, pos => Math.Abs(pos - Position) < 1e-9);
            if (atBasePoint >= 0) splitPoint = basePoints[atBasePoint]; // keep the exact point
            else
            {
                ApproximatePosition(Position, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p);
                splitPoint = new SurfacePoint(p, uv1, uv2);
            }
            List<SurfacePoint> l1 = new List<SurfacePoint> { basePoints[0] };
            List<SurfacePoint> l2 = new List<SurfacePoint>();
            for (int i = 1; i < basePoints.Length - 1; i++)
            {   // base points very close to the split position would make a tiny segment, they are left out (as in Trim)
                if (positions[i] < Position - 1e-3) l1.Add(basePoints[i]);
                else if (positions[i] > Position + 1e-3) l2.Add(basePoints[i]);
            }
            l2.Add(basePoints[basePoints.Length - 1]);
            l1.Add(splitPoint);
            l2.Insert(0, splitPoint);
            for (int i = l1.Count - 1; i > 0; --i)
            {
                if ((l1[i].p3d | l1[i - 1].p3d) == 0.0) l1.RemoveAt(i);
            }
            for (int i = l2.Count - 1; i > 0; --i)
            {
                if ((l2[i].p3d | l2[i - 1].p3d) == 0.0) l2.RemoveAt(i);
            }
            InterpolatedDualSurfaceCurve dsc1 = new InterpolatedDualSurfaceCurve(surface1.Clone(), surface2.Clone(), l1.ToArray(), isTangential);
            InterpolatedDualSurfaceCurve dsc2 = new InterpolatedDualSurfaceCurve(surface1.Clone(), surface2.Clone(), l2.ToArray(), isTangential);
            return new ICurve[] { dsc1, dsc2 };
        }

        internal void RecalcSurfacePoints(BoundingRect bounds1, BoundingRect bounds2)
        {
            DualSurfaceCurveDiagnostics.Count("IDSC.RecalcSurfacePoints");
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
        }

        public override ICurve[] Split(double Position1, double Position2)
        {   // only for closed curves: one part from the smaller to the bigger position, the other one across the start point
            if (Position2 < Position1) (Position1, Position2) = (Position2, Position1);
            double[] positions = BasePointPositions();
            ApproximatePosition(Position1, out GeoPoint2D uv1, out GeoPoint2D uv2, out GeoPoint p);
            SurfacePoint p1 = new SurfacePoint(p, uv1, uv2);
            ApproximatePosition(Position2, out uv1, out uv2, out p);
            SurfacePoint p2 = new SurfacePoint(p, uv1, uv2);
            List<SurfacePoint> l1 = new List<SurfacePoint> { p1 };
            List<SurfacePoint> l2 = new List<SurfacePoint> { p2 };
            // the last base point of a closed curve is the first one again, so it is not used here
            for (int i = 0; i < basePoints.Length - 1; i++)
            {
                if (positions[i] > Position1 + 1e-3 && positions[i] < Position2 - 1e-3) l1.Add(basePoints[i]);
            }
            for (int i = 0; i < basePoints.Length - 1; i++)
            {
                if (positions[i] > Position2 + 1e-3) l2.Add(basePoints[i]);
            }
            for (int i = 0; i < basePoints.Length - 1; i++)
            {
                if (positions[i] < Position1 - 1e-3) l2.Add(basePoints[i]);
            }
            l1.Add(p2);
            l2.Add(p1);
            InterpolatedDualSurfaceCurve dsc1 = new InterpolatedDualSurfaceCurve(surface1.Clone(), surface2.Clone(), l1.ToArray(), isTangential);
            InterpolatedDualSurfaceCurve dsc2 = new InterpolatedDualSurfaceCurve(surface1.Clone(), surface2.Clone(), l2.ToArray(), isTangential);
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
            DualSurfaceCurveDiagnostics.Count("IDSC.Reverse");
            Array.Reverse(basePoints);
            forwardOriented = !forwardOriented;
            InvalidateSecondaryData();
        }
        public override void Trim(double StartPos, double EndPos)
        {
            if (StartPos <= Precision.eps && EndPos >= 1 - Precision.eps) return; // trim from start to end, nothing to do
            DualSurfaceCurveDiagnostics.Count("IDSC.Trim");
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
                    pos = PositionOf(basePoints[i].p3d);
                    // keine fast identischen Punkte zufügen, die führen zu Nullvektoren in der Differenz
                    if (pos > StartPos + 1e-3) spl.Add(basePoints[i]);
                }
                for (int i = 1; i < basePoints.Length; ++i)
                {
                    double pos = (double)i / (double)(basePoints.Length - 1);
                    pos = PositionOf(basePoints[i].p3d);
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
                    pos = PositionOf(basePoints[i].p3d);
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
            InvalidateSecondaryData();
            BSpline init = ApproxBSpline;
        }
        public override IGeoObject Clone()
        {
            return new InterpolatedDualSurfaceCurve(surface1.Clone(), bounds1, surface2.Clone(), bounds2, basePoints.Select(sp => sp.p3d).ToArray(), basePoints.Select(sp => sp.psurface1).ToList(), basePoints.Select(sp => sp.psurface2).ToList(), isTangential, approxBSpline);
            // Clone introduced because of independant surfaces for BRep operations
            // forwardOriented is calculated by the order of the base points
        }
        internal void SetSurfaces(ISurface surface1, ISurface surface2, bool swapped)
        {
            DualSurfaceCurveDiagnostics.Count("IDSC.SetSurfaces");
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
                        (bounds1, bounds2) = (bounds2, bounds1);
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
            InterpolatedDualSurfaceCurve ipdsc = new InterpolatedDualSurfaceCurve(surface1.GetModified(m), surface2.GetModified(m), sp, IsTangential);
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

        public bool IsTangential => isTangential;

        /// <summary>
        /// True when the two surfaces clearly intersect at every inner base point, i.e. the curve is certainly no
        /// tangential intersection. The end points are not considered, a curve may end where the surfaces touch.
        /// </summary>
        private bool IsTransversalAtAllInnerPoints()
        {
            if (basePoints.Length < 3) return false;
            for (int i = 1; i < basePoints.Length - 1; i++)
            {
                GeoVector n1 = surface1.GetNormal(basePoints[i].psurface1);
                GeoVector n2 = surface2.GetNormal(basePoints[i].psurface2);
                if (n1.IsNullVector() || n2.IsNullVector()) return false;
                if ((n1.Normalized ^ n2.Normalized).Length < 1e-2) return false;
            }
            return true;
        }

        ICurve2D IDualSurfaceCurve.GetCurveOnSurface(ISurface onThisSurface)
        {
            if (onThisSurface == surface1) return CurveOnSurface1;
            else if (onThisSurface == surface2) return CurveOnSurface2;
            else return null;
        }

        void IDualSurfaceCurve.SwapSurfaces()
        {
            DualSurfaceCurveDiagnostics.Count("IDSC.SwapSurfaces");
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
        }

        public override ICurve Approximate(bool linesOnly, double maxError)
        {
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
                // same direction
                for (double par = 0.25; par < 1.0; par += 0.25)
                {
                    if ((PointAt(par) | other.PointAt(par)) > precision)
                    {   // the curves may run with different "speed", so we need to test the distance (which is more expensive)
                        if (other.DistanceTo(PointAt(par))>precision) return false;
                    }
                }
                return true;
            }
            if ((EndPoint | other.StartPoint) < precision && (StartPoint | other.EndPoint) < precision)
            {
                for (double par = 0.25; par < 1.0; par += 0.25)
                {
                    if ((PointAt(par) | other.PointAt(1 - par)) > precision) 
                    {   // the curves may run with different "speed", so we need to test the distance (which is more expensive)
                        if (other.DistanceTo(PointAt(par)) > precision) return false;
                    }
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
        }
        public void GetObjectData(IJsonWriteData data)
        {
            data.AddProperty("Surface1", surface1);
            data.AddProperty("Surface2", surface2);
            data.AddProperty("BasePoints", basePoints);
            data.AddProperty("ForwardOriented", forwardOriented);
            data.AddProperty("IsTangential", isTangential);
            data.AddProperty("Bounds1", bounds1);
            data.AddProperty("Bounds2", bounds2);
        }

        public void SetObjectData(IJsonReadData data)
        {
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
                if (surface1 is ISurfaceImpl simpl1) bounds1 = simpl1.HasDomain ? simpl1.Domain : GetBoundingRect(true);
                if (surface2 is ISurfaceImpl simpl2) bounds2 = simpl2.HasDomain ? simpl2.Domain : GetBoundingRect(false);
            }
            jsonSerialize.InvokeSerializationDoneCallback(surface1);
            jsonSerialize.InvokeSerializationDoneCallback(surface2);
            // Files written while CloneTrimmed handed forwardOriented to the parameter isTangential may contain transversal
            // curves marked as tangential. Their points would be computed with the solver for touching surfaces, which
            // fails for them, so every point would silently remain unrefined.
            if (isTangential && IsTransversalAtAllInnerPoints()) isTangential = false;
            DualSurfaceCurveDiagnostics.ConstructionProbe probe = DualSurfaceCurveDiagnostics.BeginConstruction(surface1, bounds1, surface2, bounds2);
            Init();
            DualSurfaceCurveDiagnostics.EndConstruction(probe, surface1, surface2, basePoints, isTangential,
                "JSON deserialization, type version " + jsonSerialize.GetTypeVersion(this.GetType()).ToString());
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

        internal void SetBounds(BoundingRect bounds1, BoundingRect bounds2)
        {
            DualSurfaceCurveDiagnostics.Count("IDSC.SetBounds");
            if (!bounds1.IsEmpty()) this.bounds1 = bounds1;
            if (!bounds2.IsEmpty()) this.bounds2 = bounds2;
            hashedPositions.Clear();
            RecalcSurfacePoints(bounds1, bounds2);
        }

        #endregion
    }
}
