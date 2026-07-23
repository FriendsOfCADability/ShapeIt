using CADability.GeoObject;
using CADability.UserInterface;
using System;
using System.Collections.Generic;

namespace CADability.Actions
{
    /// <summary>
    /// Rounds the corner between two curves that meet at a common endpoint: the two curves are shortened to the tangent
    /// points and a tangential fillet arc of the given radius is inserted. The user approaches a corner; the neighbouring
    /// curve is found automatically via the pick ray (<see cref="ConstructAction.CurrentMouseBeam"/>). If a curve is a
    /// path/polyline, only its end segment takes part in the computation and the result is recombined into a new path.
    /// The geometric core is <see cref="RoundOffGeometry"/>.
    /// </summary>
    internal class RoundObjectsAction : ConstructAction
    {
        private LengthInput radiusInput; // the input field for the fillet radius
        private double radius; // the current fillet radius

        // one curve of a corner, reduced to the simple segment that touches the corner
        private class CornerCurve
        {
            public ICurve original;      // the model object to remove on apply
            public ICurve segment;       // the simple segment at the corner used for the geometry
            public List<ICurve> remnant; // the other segments of a composed curve, recombined afterwards (original order)
        }

        // one computed fillet at a corner
        private class RoundInfo
        {
            public List<IGeoObject> originals; // the objects to remove on apply
            public ICurve segA;                // the two simple segments meeting at the corner
            public ICurve segB;
            public List<ICurve> remnant;       // the other segments (of a composed curve), recombined afterwards
            public Ellipse arc;
            public GeoPoint corner;
        }

        public override void OnSetAction()
        {
            base.ActiveObject = null;
            base.TitleId = "ToolsRoundOff";
            radius = ConstrDefaults.DefaultRoundRadius;

            CurveInput curveInput = new CurveInput("ToolsRound.Object"); // the corner to be rounded
            curveInput.ModifiableOnly = true;
            curveInput.HitCursor = CursorTable.GetCursor("RoundOff.cur");
            curveInput.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(MouseOverCornersToRound);

            radiusInput = new LengthInput("ToolsRound.Radius");
            radiusInput.DefaultLength = ConstrDefaults.DefaultRoundRadius;
            radiusInput.ForwardMouseInputTo = curveInput; // keep processing mouse input for the corner
            radiusInput.SetLengthEvent += new LengthInput.SetLengthDelegate(SetRadius);

            base.SetInput(curveInput, radiusInput);
            base.ShowActiveObject = false;
            base.OnSetAction();
        }

        private bool SetRadius(double length)
        {
            if (length >= 0.0)
            {
                radius = length;
                return true;
            }
            return false;
        }

        private bool MouseOverCornersToRound(CurveInput sender, ICurve[] curves, bool up)
        {
            FeedBack.ClearSelected();
            if (radius <= 0.0) return false; // no radius, nothing to round
            RoundInfo roundInfo = ComputeBestCorner(curves, base.CurrentMouseBeam);
            if (roundInfo == null) return false;
            if (up)
            {   // perform the rounding
                using (base.Frame.Project.Undo.UndoFrame)
                {
                    ApplyRound(roundInfo);
                }
            }
            else
            {   // preview the fillet arc
                FeedBack.AddSelected(roundInfo.arc as IGeoObject);
            }
            return true;
            // when true is returned and up == true, this action terminates, because there are no more open (unfixed) inputs.
        }

        /// <summary>
        /// Finds the pair of connected curves whose shared corner the pick ray passes closest to, and returns the fillet,
        /// or null. The mouse position itself is useless here because it lies in the active drawing plane, which is
        /// unrelated to the curves; the decisive measure is the distance to the pick ray. Each (possibly composed) curve
        /// is reduced to the simple segment at the corner so that a common plane always exists.
        /// </summary>
        private RoundInfo ComputeBestCorner(ICurve[] curves, Axis beam)
        {
            List<ICurve> candidates = new List<ICurve>();
            foreach (ICurve c in curves)
                if (c != null && !candidates.Contains(c)) candidates.Add(c);

            // add nearby model curves so the neighbouring curve of the corner is found even if only one is under the cursor
            Model model = CurrentMouseView.Model;
            if (model != null && curves.Length > 0)
            {
                GeoObjectList closeObjects = model.GetObjectsCloseTo(curves[0] as IOctTreeInsertable);
                foreach (IGeoObject go in closeObjects)
                {
                    if (go.Layer != null && !CurrentMouseView.ProjectedModel.IsLayerVisible(go.Layer)) continue; // hidden layer
                    if (go.Owner is Edge) continue; // no edges of solids
                    if (go is ICurve c && !candidates.Contains(c)) candidates.Add(c);
                }
            }

            RoundInfo best = null;
            double bestDist = double.MaxValue;

            // corners between two distinct curves that share an endpoint
            for (int i = 0; i < candidates.Count; i++)
            {
                for (int j = i + 1; j < candidates.Count; j++)
                {
                    if (!ShareEndpoint(candidates[i], candidates[j], beam, out GeoPoint corner)) continue;
                    CornerCurve cc1 = MakeCornerCurve(candidates[i], corner);
                    CornerCurve cc2 = MakeCornerCurve(candidates[j], corner);
                    if (cc1 == null || cc2 == null) continue;
                    List<ICurve> remnant = new List<ICurve>();
                    remnant.AddRange(cc1.remnant);
                    remnant.AddRange(cc2.remnant);
                    EvaluateCorner(cc1.segment, cc2.segment, remnant,
                        new List<IGeoObject> { candidates[i] as IGeoObject, candidates[j] as IGeoObject },
                        corner, beam, ref best, ref bestDist);
                }
            }

            // corners inside a single path/polyline (and its seam if it is closed): a single path has no partner to be
            // joined with, so it is rounded with its own adjacent segments
            foreach (ICurve candidate in candidates)
            {
                if (!candidate.IsComposed) continue;
                ICurve[] subs = candidate.SubCurves;
                if (subs.Length < 2) continue;
                for (int k = 0; k < subs.Length - 1; k++) // inner vertices
                    EvaluateSelfCorner(candidate, subs, k, k + 1, subs[k].EndPoint, candidates, beam, ref best, ref bestDist);
                if (candidate.IsClosed) // seam of a closed path
                    EvaluateSelfCorner(candidate, subs, subs.Length - 1, 0, candidate.StartPoint, candidates, beam, ref best, ref bestDist);
            }
            return best;
        }

        // evaluate a corner inside a single composed curve, between segment indices a and b, at the vertex 'corner'
        private void EvaluateSelfCorner(ICurve composed, ICurve[] subs, int a, int b, GeoPoint corner,
            List<ICurve> candidates, Axis beam, ref RoundInfo best, ref double bestDist)
        {
            // if a further curve attaches at this vertex it would be three curves and ambiguous -> not roundable
            if (ExternalAttachment(candidates, composed, corner)) return;
            List<ICurve> remnant = new List<ICurve>();
            for (int k = 0; k < subs.Length; k++)
                if (k != a && k != b) remnant.Add(subs[k].Clone());
            EvaluateCorner(subs[a].Clone(), subs[b].Clone(), remnant,
                new List<IGeoObject> { composed as IGeoObject }, corner, beam, ref best, ref bestDist);
        }

        // core evaluation: gate by the pick-ray distance, then compute the fillet of the two simple segments
        private void EvaluateCorner(ICurve segA, ICurve segB, List<ICurve> remnant, List<IGeoObject> originals,
            GeoPoint corner, Axis beam, ref RoundInfo best, ref double bestDist)
        {
            // the ray must pass closer to the corner than to the far ends of both segments, otherwise the user is
            // hovering elsewhere (along a segment or near a neighbouring corner)
            double dist = DistancePointBeam(corner, beam);
            if (dist >= bestDist) return;
            if (dist > DistancePointBeam(FarEnd(segA, corner), beam)) return;
            if (dist > DistancePointBeam(FarEnd(segB, corner), beam)) return;

            // the pick point in the segments' plane is where the ray crosses that plane (not the mouse position)
            if (!Curves.GetCommonPlane(segA, segB, out Plane pl)) return;
            GeoPoint pick;
            if (Math.Abs(pl.Normal.Normalized * beam.Direction.Normalized) < 1e-8)
                pick = corner; // ray parallel to the plane: fall back to the corner
            else
                pick = pl.Intersect(beam.Location, beam.Direction);

            // curves and corner are known, so skip the pick-based quadrant filter (always the inner-angle fillet)
            if (RoundOffGeometry.TryComputeRoundOff(segA, segB, pick, radius, true,
                    base.ActiveDrawingPlane, out Ellipse arc, out GeoPoint cornerOut))
            {
                bestDist = dist;
                best = new RoundInfo { originals = originals, segA = segA, segB = segB, remnant = remnant, arc = arc, corner = cornerOut };
            }
        }

        // true if a candidate curve other than 'self' has an endpoint at the given vertex
        private static bool ExternalAttachment(List<ICurve> candidates, ICurve self, GeoPoint vertex)
        {
            foreach (ICurve c in candidates)
            {
                if (c == self) continue;
                if (Precision.IsEqual(c.StartPoint, vertex) || Precision.IsEqual(c.EndPoint, vertex)) return true;
            }
            return false;
        }

        /// <summary>
        /// Reduces a curve to the simple segment at the corner; for a composed curve (path/polyline) the remaining
        /// segments become the remnant. Returns null if the corner is not at an end of the curve.
        /// </summary>
        private static CornerCurve MakeCornerCurve(ICurve curve, GeoPoint corner)
        {
            if (curve.IsComposed)
            {
                ICurve[] subs = curve.SubCurves;
                if (subs.Length == 0) return null;
                List<ICurve> remnant = new List<ICurve>();
                ICurve segment;
                if (Precision.IsEqual(curve.StartPoint, corner))
                {   // corner at the path start: the first segment is the working segment
                    segment = subs[0].Clone();
                    for (int k = 1; k < subs.Length; k++) remnant.Add(subs[k].Clone());
                }
                else if (Precision.IsEqual(curve.EndPoint, corner))
                {   // corner at the path end: the last segment is the working segment
                    segment = subs[subs.Length - 1].Clone();
                    for (int k = 0; k < subs.Length - 1; k++) remnant.Add(subs[k].Clone());
                }
                else return null; // corner not at a path end, cannot round here
                return new CornerCurve { original = curve, segment = segment, remnant = remnant };
            }
            // simple curve
            if (!Precision.IsEqual(curve.StartPoint, corner) && !Precision.IsEqual(curve.EndPoint, corner)) return null;
            return new CornerCurve { original = curve, segment = curve, remnant = new List<ICurve>() };
        }

        private void ApplyRound(RoundInfo roundInfo)
        {
            IGeoObject attrSource = roundInfo.originals[0];
            IGeoObjectOwner owner = attrSource.Owner;
            if (owner == null) return; // should never happen

            // the two arc endpoints are the tangent points; assign each to the segment it lies on and shorten it
            GeoPoint tpA, tpB;
            if (DistanceToCurve(roundInfo.arc.StartPoint, roundInfo.segA) <= DistanceToCurve(roundInfo.arc.EndPoint, roundInfo.segA))
            {
                tpA = roundInfo.arc.StartPoint;
                tpB = roundInfo.arc.EndPoint;
            }
            else
            {
                tpA = roundInfo.arc.EndPoint;
                tpB = roundInfo.arc.StartPoint;
            }
            ICurve trimmedA = TrimToTangent(roundInfo.segA, tpA, roundInfo.corner);
            ICurve trimmedB = TrimToTangent(roundInfo.segB, tpB, roundInfo.corner);

            // collect all resulting segments: the remnant of the composed curve plus the two shortened segments and the arc
            List<ICurve> parts = new List<ICurve>();
            parts.AddRange(roundInfo.remnant);
            parts.Add(trimmedA);
            parts.Add(roundInfo.arc);
            parts.Add(trimmedB);
            foreach (ICurve part in parts) (part as IGeoObject).CopyAttributes(attrSource);

            // remove the originals; they are replaced by the recombined result
            foreach (IGeoObject original in roundInfo.originals) original.Owner?.Remove(original);

            bool composed = roundInfo.remnant.Count > 0;
            foreach (IGeoObject original in roundInfo.originals)
                if ((original as ICurve)?.IsComposed == true) composed = true;
            if (composed || Frame.GetBooleanSetting("Construct.MakePath", true))
            {   // combine the whole result into a single (new) path
                Path path = Path.FromSegments(parts, false);
                if (path != null)
                {
                    (path as IGeoObject).CopyAttributes(attrSource);
                    owner.Add(path as IGeoObject);
                    return;
                }
            }
            foreach (ICurve part in parts) owner.Add(part as IGeoObject);
        }

        /// <summary>
        /// Returns a clone of <paramref name="curve"/> shortened to the tangent point, keeping the part away from the corner.
        /// </summary>
        private static ICurve TrimToTangent(ICurve curve, GeoPoint tangentPoint, GeoPoint corner)
        {
            ICurve clone = curve.Clone();
            double t = clone.PositionOf(tangentPoint);
            double c = clone.PositionOf(corner);
            if (Math.Abs(c - 1.0) < Math.Abs(c - 0.0)) clone.Trim(0.0, t); // corner is at the curve's end
            else clone.Trim(t, 1.0); // corner is at the curve's start
            return clone;
        }

        private static double DistanceToCurve(GeoPoint p, ICurve curve)
        {
            double pos = curve.PositionOf(p);
            if (pos < 0.0) pos = 0.0;
            if (pos > 1.0) pos = 1.0;
            return p | curve.PointAt(pos);
        }

        /// <summary>
        /// Returns true if the two curves share an endpoint (within precision); <paramref name="corner"/> is set to the
        /// shared endpoint the pick ray passes closest to.
        /// </summary>
        private static bool ShareEndpoint(ICurve a, ICurve b, Axis beam, out GeoPoint corner)
        {
            GeoPoint[] pa = { a.StartPoint, a.EndPoint };
            GeoPoint[] pb = { b.StartPoint, b.EndPoint };
            corner = GeoPoint.Origin;
            double best = double.MaxValue;
            bool found = false;
            foreach (GeoPoint x in pa)
                foreach (GeoPoint y in pb)
                    if (Precision.IsEqual(x, y))
                    {
                        double d = DistancePointBeam(x, beam);
                        if (d < best) { best = d; corner = x; found = true; }
                    }
            return found;
        }

        // perpendicular distance from a point to the (infinite) pick ray
        private static double DistancePointBeam(GeoPoint p, Axis beam)
        {
            double dirLength = beam.Direction.Length;
            if (dirLength < 1e-12) return p | beam.Location;
            return ((p - beam.Location) ^ beam.Direction).Length / dirLength;
        }

        // the endpoint of the curve that is not the corner
        private static GeoPoint FarEnd(ICurve curve, GeoPoint corner)
        {
            return Precision.IsEqual(curve.StartPoint, corner) ? curve.EndPoint : curve.StartPoint;
        }

        public override string GetID()
        {
            return "ToolsRoundOff";
        }
    }
}
