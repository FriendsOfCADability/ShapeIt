using CADability.GeoObject;
using CADability.UserInterface;
using System;
using System.Collections.Generic;

namespace CADability.Actions
{
    /// <summary>
    /// Rounds (fillet) or chamfers corners of curves. The operation (<see cref="CornerGeometry.Operation"/>) is fixed per
    /// invocation (from the menu); the size input is the fillet radius or the chamfer edge length accordingly. In
    /// <see cref="Mode.SingleCorner"/> the user approaches one corner (the two curves are found via the pick ray
    /// <see cref="ConstructAction.CurrentMouseBeam"/>; a picked path/polyline is reduced to its end segment); in
    /// <see cref="Mode.AllCorners"/> every corner of the picked path, polyline or connected chain is processed at once.
    /// The geometric core is <see cref="CornerGeometry"/>.
    /// </summary>
    internal class CornerCurvesAction : ConstructAction
    {
        /// <summary>What is processed: one corner or every corner of the picked object.</summary>
        public enum Mode
        {
            SingleCorner, // the one corner the user points at
            AllCorners    // every corner of the picked path / polyline / connected chain
        }
        private readonly CornerGeometry.Operation operation; // fillet or chamfer, fixed per invocation
        private Mode mode; // the current mode, preset by the constructor and adjustable via modeInput
        private MultipleChoiceInput modeInput; // the input field to choose the mode
        private LengthInput sizeInput; // the input field for the fillet radius / chamfer edge length
        private double size; // the current fillet radius or chamfer edge length

        // The result is computed from the current pick, size and mode whenever any of them changes, shown as feedback and
        // remembered here; it is applied to the model only in OnDone, when all inputs are fixed.
        private CornerInfo pendingCorner;     // the single-corner result to apply
        private MultiCornerInfo pendingMulti;  // the all-corners result to apply
        private ICurve[] singleCurves;         // curves under the cursor at the last pick (single-corner mode)
        private Axis singleBeam;               // the pick ray at the last pick (single-corner mode)
        private ICurve allPicked;              // the object picked (all-corners mode)

        public CornerCurvesAction(CornerGeometry.Operation operation = CornerGeometry.Operation.Fillet, Mode mode = Mode.SingleCorner)
        {
            this.operation = operation;
            this.mode = mode;
        }

        // one curve of a corner, reduced to the simple segment that touches the corner
        private class CornerCurve
        {
            public ICurve original;      // the model object to remove on apply
            public ICurve segment;       // the simple segment at the corner used for the geometry
            public List<ICurve> remnant; // the other segments of a composed curve, recombined afterwards (original order)
        }

        // one computed corner result: the two shortened segments joined by the corner curve (arc or chamfer line)
        private class CornerInfo
        {
            public List<IGeoObject> originals; // the objects to remove on apply
            public ICurve segA;                // the two simple segments meeting at the corner
            public ICurve segB;
            public List<ICurve> remnant;       // the other segments (of a composed curve), recombined afterwards
            public ICurve cornerCurve;         // the fillet arc or the chamfer line
            public GeoPoint corner;
        }

        // one fully-processed object: the objects to remove and the parts of the result (shortened segments + corner curves)
        private class MultiCornerInfo
        {
            public List<IGeoObject> originals;
            public List<ICurve> resultParts;
        }

        public override void OnSetAction()
        {
            base.ActiveObject = null;
            UpdateTitle();
            bool fillet = operation == CornerGeometry.Operation.Fillet;
            size = fillet ? ConstrDefaults.DefaultRoundRadius : ConstrDefaults.DefaultCutOffLength;

            CurveInput curveInput = new CurveInput("ToolsRound.Object"); // the corner to be processed
            curveInput.ModifiableOnly = true;
            curveInput.HitCursor = CursorTable.GetCursor(fillet ? "RoundOff.cur" : "CutOff.cur");
            curveInput.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(MouseOverCorners);

            // the label differs by operation: fillet radius vs. chamfer edge length
            sizeInput = new LengthInput(fillet ? "ToolsRound.Radius" : "ToolsCutOff.Length");
            sizeInput.DefaultLength = fillet ? ConstrDefaults.DefaultRoundRadius : ConstrDefaults.DefaultCutOffLength;
            sizeInput.ForwardMouseInputTo = curveInput; // keep processing mouse input for the corner
            sizeInput.SetLengthEvent += new LengthInput.SetLengthDelegate(SetSize);

            modeInput = new MultipleChoiceInput("ToolsRound.Mode", "ToolsRound.Mode.Values", (int)mode);
            modeInput.Optional = true;
            modeInput.ForwardMouseInputTo = curveInput;
            modeInput.SetChoiceEvent += new MultipleChoiceInput.SetChoiceDelegate(SetMode);

            base.SetInput(curveInput, sizeInput, modeInput);
            base.ShowActiveObject = false;
            base.OnSetAction();
        }

        private bool SetSize(double length)
        {
            if (length >= 0.0)
            {
                size = length;
                Recompute(); // the size changed: rebuild the result and the preview
                return true;
            }
            return false;
        }

        private void SetMode(int val)
        {   // the user changed the mode in the property grid
            mode = (Mode)val;
            UpdateTitle();
            Recompute();
        }

        private void UpdateTitle()
        {   // the title reflects the operation and the mode
            base.TitleId = operation == CornerGeometry.Operation.Fillet
                ? (mode == Mode.AllCorners ? "ToolsRoundMultiple" : "ToolsRoundOff")
                : (mode == Mode.AllCorners ? "ToolsCutOffMultiple" : "ToolsCutOff");
        }

        private bool MouseOverCorners(CurveInput sender, ICurve[] curves, bool up)
        {
            // Only remember what the user points at and show the preview; the result is applied in OnDone once the curve
            // input and the size input are both fixed (the user may pick the corner and the size in any order).
            if (mode == Mode.AllCorners)
                allPicked = curves.Length > 0 ? curves[0] : null;
            else
            {
                singleCurves = (ICurve[])curves.Clone();
                singleBeam = base.CurrentMouseBeam;
            }
            Recompute();
            return mode == Mode.AllCorners ? pendingMulti != null : pendingCorner != null;
            // returning true on up fixes the curve input; the action ends when the size input is fixed as well
        }

        /// <summary>
        /// Recomputes the result for the current pick, size and mode, updates the feedback and stores the result
        /// (<see cref="pendingCorner"/> / <see cref="pendingMulti"/>) for OnDone. Called whenever the pick, the size or
        /// the mode changes.
        /// </summary>
        private void Recompute()
        {
            FeedBack.ClearSelected();
            pendingCorner = null;
            pendingMulti = null;
            if (size <= 0.0) return; // no size, nothing to do
            if (mode == Mode.AllCorners)
            {
                if (allPicked == null) return;
                pendingMulti = ComputeAllCorners(allPicked);
                if (pendingMulti != null)
                    foreach (ICurve part in pendingMulti.resultParts) FeedBack.AddSelected(part as IGeoObject);
            }
            else
            {
                if (singleCurves == null) return;
                pendingCorner = ComputeBestCorner(singleCurves, singleBeam);
                if (pendingCorner != null) FeedBack.AddSelected(pendingCorner.cornerCurve as IGeoObject);
            }
        }

        /// <summary>
        /// Finds the pair of connected curves whose shared corner the pick ray passes closest to, and returns the result,
        /// or null. The mouse position itself is useless here because it lies in the active drawing plane, which is
        /// unrelated to the curves; the decisive measure is the distance to the pick ray. Each (possibly composed) curve
        /// is reduced to the simple segment at the corner so that a common plane always exists.
        /// </summary>
        private CornerInfo ComputeBestCorner(ICurve[] curves, Axis beam)
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

            CornerInfo best = null;
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
            // joined with, so it is processed with its own adjacent segments
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
            List<ICurve> candidates, Axis beam, ref CornerInfo best, ref double bestDist)
        {
            // if a further curve attaches at this vertex it would be three curves and ambiguous -> not processable
            if (ExternalAttachment(candidates, composed, corner)) return;
            List<ICurve> remnant = new List<ICurve>();
            for (int k = 0; k < subs.Length; k++)
                if (k != a && k != b) remnant.Add(subs[k].Clone());
            EvaluateCorner(subs[a].Clone(), subs[b].Clone(), remnant,
                new List<IGeoObject> { composed as IGeoObject }, corner, beam, ref best, ref bestDist);
        }

        // core evaluation: gate by the pick-ray distance, then compute the corner curve of the two simple segments
        private void EvaluateCorner(ICurve segA, ICurve segB, List<ICurve> remnant, List<IGeoObject> originals,
            GeoPoint corner, Axis beam, ref CornerInfo best, ref double bestDist)
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

            if (CornerGeometry.TryComputeCornerCurve(segA, segB, pick, size, operation, base.ActiveDrawingPlane,
                    out ICurve cornerCurve, out GeoPoint cornerOut))
            {
                bestDist = dist;
                best = new CornerInfo { originals = originals, segA = segA, segB = segB, remnant = remnant, cornerCurve = cornerCurve, corner = cornerOut };
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
                else return null; // corner not at a path end, cannot process here
                return new CornerCurve { original = curve, segment = segment, remnant = remnant };
            }
            // simple curve
            if (!Precision.IsEqual(curve.StartPoint, corner) && !Precision.IsEqual(curve.EndPoint, corner)) return null;
            return new CornerCurve { original = curve, segment = curve, remnant = new List<ICurve>() };
        }

        private void ApplyCorner(CornerInfo cornerInfo)
        {
            IGeoObject attrSource = cornerInfo.originals[0];
            IGeoObjectOwner owner = attrSource.Owner;
            if (owner == null) return; // should never happen

            // the two corner-curve endpoints are the cut points; assign each to the segment it lies on and shorten it
            GeoPoint tpA, tpB;
            if (DistanceToCurve(cornerInfo.cornerCurve.StartPoint, cornerInfo.segA) <= DistanceToCurve(cornerInfo.cornerCurve.EndPoint, cornerInfo.segA))
            {
                tpA = cornerInfo.cornerCurve.StartPoint;
                tpB = cornerInfo.cornerCurve.EndPoint;
            }
            else
            {
                tpA = cornerInfo.cornerCurve.EndPoint;
                tpB = cornerInfo.cornerCurve.StartPoint;
            }
            ICurve trimmedA = TrimToTangent(cornerInfo.segA, tpA, cornerInfo.corner);
            ICurve trimmedB = TrimToTangent(cornerInfo.segB, tpB, cornerInfo.corner);

            // collect all resulting segments: the remnant of the composed curve plus the two shortened segments and the corner curve
            List<ICurve> parts = new List<ICurve>();
            parts.AddRange(cornerInfo.remnant);
            parts.Add(trimmedA);
            parts.Add(cornerInfo.cornerCurve);
            parts.Add(trimmedB);
            foreach (ICurve part in parts) (part as IGeoObject).CopyAttributes(attrSource);

            // remove the originals; they are replaced by the recombined result
            foreach (IGeoObject original in cornerInfo.originals) original.Owner?.Remove(original);

            bool composed = cornerInfo.remnant.Count > 0;
            foreach (IGeoObject original in cornerInfo.originals)
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
        /// Returns a clone of <paramref name="curve"/> shortened to the cut point, keeping the part away from the corner.
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

        // ---- "all corners" mode ----------------------------------------------------------------------------------

        /// <summary>
        /// Processes every corner of <paramref name="picked"/> at once. The ordered segments come from a path/polyline or,
        /// for a single curve, from a connected chain of model curves. Each inner corner (and the seam of a closed
        /// outline) gets a corner curve, and each segment is shortened by the corner curves of its two neighbours.
        /// Returns null if nothing can be processed.
        /// </summary>
        private MultiCornerInfo ComputeAllCorners(ICurve picked)
        {
            if (picked == null) return null;
            if (!GetChain(picked, out List<ICurve> segments, out bool closed, out List<IGeoObject> originals)) return null;
            List<ICurve> resultParts = CornerGeometry.AllCorners(segments, closed, size, operation, base.ActiveDrawingPlane);
            if (resultParts == null) return null;
            foreach (ICurve part in resultParts) (part as IGeoObject).CopyAttributes(picked as IGeoObject);
            return new MultiCornerInfo { originals = originals, resultParts = resultParts };
        }

        /// <summary>
        /// Provides the ordered segments of the picked object plus the objects to remove: a path/polyline directly, or,
        /// for a single curve, a connected chain of model curves (CreateFromModel). Returns false for fewer than two
        /// segments.
        /// </summary>
        private bool GetChain(ICurve picked, out List<ICurve> segments, out bool closed, out List<IGeoObject> originals)
        {
            segments = new List<ICurve>();
            originals = new List<IGeoObject>();
            closed = false;
            if (picked.IsComposed) // path or polyline
            {
                foreach (ICurve c in picked.SubCurves) segments.Add(c.Clone());
                closed = picked.IsClosed;
                originals.Add(picked as IGeoObject);
            }
            else
            {   // a single curve: build a chain of connected model curves
                Path chain = Path.CreateFromModel(picked, Frame.ActiveView.Model, Frame.ActiveView.Projection, true);
                if (chain == null) return false;
                for (int i = 0; i < chain.Count; i++)
                {
                    segments.Add(chain.Curve(i).Clone());
                    IGeoObject original = null;
                    if ((chain.Curve(i) as IGeoObject).UserData.ContainsData("CADability.Path.Original"))
                        original = (chain.Curve(i) as IGeoObject).UserData.GetData("CADability.Path.Original") as IGeoObject;
                    if (original != null && !originals.Contains(original)) originals.Add(original);
                }
                closed = chain.IsClosed;
            }
            return segments.Count >= 2 && originals.Count > 0;
        }

        private void ApplyMultiCorner(MultiCornerInfo multi)
        {
            IGeoObject attrSource = multi.originals[0];
            IGeoObjectOwner owner = attrSource.Owner;
            if (owner == null) return; // should never happen
            foreach (IGeoObject original in multi.originals) original.Owner?.Remove(original);

            if (Frame.GetBooleanSetting("Construct.MakePath", true))
            {   // combine the whole result into a single path
                Path path = Path.FromSegments(multi.resultParts, false);
                if (path != null)
                {
                    (path as IGeoObject).CopyAttributes(attrSource);
                    owner.Add(path as IGeoObject);
                    return;
                }
            }
            foreach (ICurve part in multi.resultParts) owner.Add(part as IGeoObject);
        }

        public override void OnDone()
        {   // all inputs are fixed: now apply the result that was computed and previewed during the interaction
            if (mode == Mode.AllCorners)
            {
                if (pendingMulti != null)
                    using (base.Frame.Project.Undo.UndoFrame) ApplyMultiCorner(pendingMulti);
            }
            else
            {
                if (pendingCorner != null)
                    using (base.Frame.Project.Undo.UndoFrame) ApplyCorner(pendingCorner);
            }
            base.OnDone();
        }

        public override string GetID()
        {
            return operation == CornerGeometry.Operation.Fillet ? "ToolsRoundOff" : "ToolsCutOff";
        }
    }
}
