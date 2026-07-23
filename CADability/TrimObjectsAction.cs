using CADability.Actions;
using CADability.GeoObject;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Linq;

namespace CADability.Actions

{
    internal class TrimObjectsAction : ConstructAction
    {
        // Specifies what happens to the picked curves.
        public enum Mode
        {
            Trim,                 // remove the picked part, keep the rest
            SplitToPath,          // split the curve and join the parts into a single path
            SplitToSingleObjects  // split the curve and add the parts as individual objects
        }
        private Mode mode; // the current mode, preset by the constructor and adjustable via modeInput
        private MultipleChoiceInput modeInput; // the input field to choose the mode

        /// <summary>
        /// Creates the action. <paramref name="mode"/> presets the mode input field.
        /// </summary>
        public TrimObjectsAction(Mode mode = Mode.Trim)
        {
            this.mode = mode;
        }

        // The optional stop curve: if set, the picked curves are trimmed at their intersections with this curve
        // (whether the intersection is inside the stop curve or not). If null, each curve is trimmed at its next inner
        // intersection(s) with any nearby curve in the model.
        private ICurve stopCurve;
        private CurveInput stopObject; // the input field for stopCurve

        // Describes which part of a single curve has to be removed.
        private class TrimInfo
        {
            public ICurve curve;       // the curve to be trimmed
            public double startParam;  // lower boundary parameter of the part to remove
            public double endParam;    // upper boundary parameter of the part to remove
            public bool closed;        // true if the curve is closed
            public bool keepSeamPiece; // for a closed curve: keep the arc crossing the seam (true) or the inner arc (false)
        }

        public override void OnSetAction()
        {
            base.ActiveObject = null;
            UpdateTitle();
            stopObject = new CurveInput("ToolsTrim.SourceObject");
            stopObject.Optional = true;
            stopObject.ModifiableOnly = true;
            stopObject.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(MouseOverStopObject);
            stopObject.CurveSelectionChangedEvent += new CurveInput.CurveSelectionChangedDelegate(StopObjectChanged);
            CurveInput curveInput = new CurveInput("ToolsTrim.Object"); // the input field for the curves to be trimmed
            curveInput.ModifiableOnly = true;
            curveInput.HitCursor = CursorTable.GetCursor("Trim.cur");
            curveInput.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(MouseOverCurvesToTrim);
            curveInput.PreferPath = true; // prefer paths, i.e. do not return sub curves of a path
            modeInput = new MultipleChoiceInput("ToolsTrim.Mode", "ToolsTrim.Mode.Values", (int)mode);
            modeInput.Optional = true;
            modeInput.ForwardMouseInputTo = curveInput; // keep processing mouse input for the curve while the mode is shown
            modeInput.SetChoiceEvent += new MultipleChoiceInput.SetChoiceDelegate(SetMode);
            base.SetInput(curveInput, stopObject, modeInput);
            base.ShowActiveObject = false;
            base.OnSetAction();
            if (stopCurve != null)
            {
                stopObject.SetCurves([stopCurve], stopCurve); // show in the input field
            }
        }

        private bool MouseOverStopObject(CurveInput sender, ICurve[] curves, bool up)
        {
            if (up)
            {   // update the display in the stopObject input field; show all curves so the user can pick one
                if (curves.Length == 0) sender.SetCurves(curves, null);
                else sender.SetCurves(curves, curves[0]);
            }
            if (curves.Length > 0)
            {
                stopCurve = curves[0]; // simply use the first one
                return true;
            }
            else stopCurve = null;
            return false;
        }

        private void StopObjectChanged(CurveInput sender, ICurve selectedCurve)
        {   // the user chose a different stop curve
            stopCurve = selectedCurve;
        }

        private void SetMode(int val)
        {   // the user changed the mode in the property grid
            mode = (Mode)val;
            UpdateTitle();
        }

        private void UpdateTitle()
        {   // the title reflects the current mode: "Trim object" or "Split object"
            base.TitleId = mode == Mode.Trim ? "ToolsTrim" : "ToolsTrimSplit";
        }

        private bool MouseOverCurvesToTrim(CurveInput sender, ICurve[] curves, bool up)
        {
            // Handle mouse over events for the curves to be trimmed. When several curves are under the cursor, all of
            // them are trimmed at once: the user has no influence on their order anyway, and if he wants to trim a
            // single object he can zoom in until only that object lies under the cursor.
            FeedBack.ClearSelected();
            List<TrimInfo> trimInfos = new List<TrimInfo>();
            foreach (ICurve curve in curves)
            {
                if (curve == stopCurve) continue; // never trim the stop curve itself
                TrimInfo trimInfo = ComputeTrimInfo(curve);
                if (trimInfo != null) trimInfos.Add(trimInfo);
            }
            if (trimInfos.Count == 0) return false;
            if (up)
            {
                using (base.Frame.Project.Undo.UndoFrame)
                {
                    foreach (TrimInfo trimInfo in trimInfos)
                    {
                        // insert the resulting parts into the same owner as the original (relevant for blocks etc.)
                        IGeoObjectOwner owner = (trimInfo.curve as IGeoObject).Owner;
                        if (owner == null) continue; // should never happen
                        owner.Remove(trimInfo.curve as IGeoObject);
                        if (mode == Mode.Trim)
                        {   // keep all parts except the one under the cursor
                            foreach (ICurve remaining in GetRemainingParts(trimInfo))
                            {
                                owner.Add(remaining as IGeoObject);
                            }
                        }
                        else
                        {   // split: keep the whole curve, but cut into pieces at the boundaries
                            List<ICurve> parts = GetAllParts(trimInfo);
                            if (mode == Mode.SplitToPath)
                            {
                                Path path = Path.FromSegments(parts, true);
                                if (path != null)
                                {
                                    (path as IGeoObject).CopyAttributes(trimInfo.curve as IGeoObject);
                                    owner.Add(path as IGeoObject);
                                }
                            }
                            else // Mode.SplitToSingleObjects
                            {
                                foreach (ICurve part in parts)
                                {
                                    owner.Add(part as IGeoObject);
                                }
                            }
                        }
                    }
                }
            }
            else
            {   // highlight the part under the cursor: it is removed when trimming, or becomes a separate piece when splitting
                foreach (TrimInfo trimInfo in trimInfos)
                {
                    ICurve pickedPart = GetRemovedPart(trimInfo);
                    if (pickedPart != null) FeedBack.AddSelected(pickedPart as IGeoObject);
                }
            }
            return true;
            // when true is returned and up == true, this action terminates, because there are no more open (unfixed) inputs.
        }

        /// <summary>
        /// Determines which part of <paramref name="toTrim"/> has to be removed, based on the current cursor position
        /// and the intersections with the stop curve or with the nearby model curves. Returns null if the curve cannot
        /// be trimmed.
        /// </summary>
        private TrimInfo ComputeTrimInfo(ICurve toTrim)
        {
            if (toTrim == null) return null;
            if (!Curves.MinDist(base.CurrentMouseBeam, toTrim, out double parCurve, out double parBeam, out double distance))
            {   // should not happen
                return null;
            }
            if (parCurve < 0) parCurve = 0;
            if (parCurve > 1) parCurve = 1;
            if (stopCurve != null)
            {
                // The user explicitly chose a stop curve, so trim at its intersections regardless of whether they lie
                // inside or outside the stop curve.
                int num = Curves.Intersect(toTrim, stopCurve, out double[] trimPos, out double[] onStopCurve, out GeoPoint[] intersectionPoints);
                if (num > 0)
                {
                    return BuildTrimInfo(toTrim, new List<double>(trimPos), parCurve);
                }
                return null;
            }
            else
            {
                // No stop curve: use the inner intersections with all curves in the model close to toTrim.
                GeoObjectList closeObjects = CurrentMouseView.Model.GetObjectsCloseTo(toTrim as IOctTreeInsertable);
                HashSet<double> intersectionParameters = new HashSet<double>();
                foreach (IGeoObject go in closeObjects)
                {
                    // layer visibility is controlled by the view, so a curve on a layer hidden in the current view must
                    // not take part in the trimming
                    if (go.Layer != null && !CurrentMouseView.ProjectedModel.IsLayerVisible(go.Layer)) continue;
                    if (go is ICurve curve)
                    {
                        int num = Curves.Intersect(toTrim, curve, out double[] trimPos, out double[] curvePos, out GeoPoint[] intersectionPoints);
                        for (int i = 0; i < num; i++)
                        {   // only consider inner intersections: inside the other curve (including its ends) and inside toTrim
                            if (curvePos[i] > -1e-6 && curvePos[i] < 1 + 1e-6 && trimPos[i] > 0 && trimPos[i] < 1)
                            {
                                intersectionParameters.Add(trimPos[i]);
                            }
                        }
                    }
                }
                if (intersectionParameters.Any())
                {
                    return BuildTrimInfo(toTrim, intersectionParameters.ToList(), parCurve);
                }
            }
            return null;
        }

        /// <summary>
        /// Builds the <see cref="TrimInfo"/> for <paramref name="toTrim"/>. <paramref name="trimPos"/> holds the
        /// (unsorted) parameters on toTrim where it is cut, <paramref name="parCurve"/> is the parameter under the
        /// cursor, i.e. inside the part the user wants to remove. Returns null if the curve cannot be trimmed.
        /// </summary>
        private TrimInfo BuildTrimInfo(ICurve toTrim, List<double> trimPos, double parCurve)
        {
            trimPos.Sort();
            TrimInfo trimInfo = new TrimInfo() { curve = toTrim, closed = toTrim.IsClosed };
            if (toTrim.IsClosed)
            {
                // A closed curve needs two boundaries to cut a single piece out of it.
                if (trimPos.Count < 2) return null;
                double? lower = null, upper = null;
                for (int i = 0; i < trimPos.Count; i++)
                {
                    if (trimPos[i] < parCurve) lower = trimPos[i]; // largest intersection below the pick
                    else if (trimPos[i] > parCurve) { upper = trimPos[i]; break; } // smallest intersection above the pick
                }
                if (lower.HasValue && upper.HasValue)
                {   // the pick lies between two intersections: remove the inner arc, keep the arc crossing the seam
                    trimInfo.startParam = lower.Value;
                    trimInfo.endParam = upper.Value;
                    trimInfo.keepSeamPiece = true;
                }
                else
                {   // the pick lies on the arc crossing the seam: remove that arc, keep the inner arc between the first
                    // and the last intersection
                    trimInfo.startParam = trimPos[0];
                    trimInfo.endParam = trimPos[trimPos.Count - 1];
                    trimInfo.keepSeamPiece = false;
                }
            }
            else
            {
                // For an open curve the ends (0.0 / 1.0) act as implicit boundaries.
                trimInfo.startParam = 0.0;
                trimInfo.endParam = 1.0;
                for (int i = 0; i < trimPos.Count; i++)
                {
                    if (trimPos[i] < parCurve) trimInfo.startParam = trimPos[i];
                    else if (trimPos[i] > parCurve)
                    {
                        trimInfo.endParam = trimPos[i];
                        break;
                    }
                }
            }
            return trimInfo;
        }

        /// <summary>
        /// Returns the part of the curve that will be removed (used for the feedback preview).
        /// </summary>
        private ICurve GetRemovedPart(TrimInfo trimInfo)
        {
            if (trimInfo.closed)
            {
                // parts[0] is the inner arc between the two parameters, parts[1] is the complement crossing the seam
                ICurve[] parts = trimInfo.curve.Split(trimInfo.startParam, trimInfo.endParam);
                if (parts.Length >= 2) return trimInfo.keepSeamPiece ? parts[0] : parts[1];
                return null;
            }
            else
            {
                ICurve removed = trimInfo.curve.Clone();
                removed.Trim(trimInfo.startParam, trimInfo.endParam);
                return removed;
            }
        }

        /// <summary>
        /// Returns the parts of the curve that remain after trimming; they replace the original curve. The original
        /// attributes are copied onto the remaining parts.
        /// </summary>
        private List<ICurve> GetRemainingParts(TrimInfo trimInfo)
        {
            List<ICurve> remaining = new List<ICurve>();
            if (trimInfo.closed)
            {
                // parts[0] is the inner arc between the two parameters, parts[1] is the complement crossing the seam
                ICurve[] parts = trimInfo.curve.Split(trimInfo.startParam, trimInfo.endParam);
                if (parts.Length >= 2) remaining.Add(trimInfo.keepSeamPiece ? parts[1] : parts[0]);
            }
            else
            {
                ICurve clone = trimInfo.curve.Clone();
                if (trimInfo.startParam == 0.0)
                {
                    clone.Trim(trimInfo.endParam, 1.0);
                    remaining.Add(clone);
                }
                else if (trimInfo.endParam == 1.0)
                {
                    clone.Trim(0.0, trimInfo.startParam);
                    remaining.Add(clone);
                }
                else
                {   // the removed part lies in the middle, so two parts remain
                    clone.Trim(0.0, trimInfo.startParam);
                    remaining.Add(clone);
                    clone = trimInfo.curve.Clone();
                    clone.Trim(trimInfo.endParam, 1.0);
                    remaining.Add(clone);
                }
            }
            foreach (ICurve curve in remaining)
            {
                (curve as IGeoObject).CopyAttributes(trimInfo.curve as IGeoObject);
            }
            return remaining;
        }

        /// <summary>
        /// Returns all parts of the curve after splitting it at the boundaries (used for the split modes). The parts are
        /// ordered so that they can be joined into a path. The original attributes are copied onto every part.
        /// </summary>
        private List<ICurve> GetAllParts(TrimInfo trimInfo)
        {
            List<ICurve> parts = new List<ICurve>();
            if (trimInfo.closed)
            {   // the two arcs together form the whole closed curve
                parts.AddRange(trimInfo.curve.Split(trimInfo.startParam, trimInfo.endParam));
            }
            else
            {
                ICurve clone;
                if (trimInfo.startParam == 0.0)
                {   // a single boundary near the start: two parts
                    clone = trimInfo.curve.Clone(); clone.Trim(0.0, trimInfo.endParam); parts.Add(clone);
                    clone = trimInfo.curve.Clone(); clone.Trim(trimInfo.endParam, 1.0); parts.Add(clone);
                }
                else if (trimInfo.endParam == 1.0)
                {   // a single boundary near the end: two parts
                    clone = trimInfo.curve.Clone(); clone.Trim(0.0, trimInfo.startParam); parts.Add(clone);
                    clone = trimInfo.curve.Clone(); clone.Trim(trimInfo.startParam, 1.0); parts.Add(clone);
                }
                else
                {   // two boundaries around the pick: three parts
                    clone = trimInfo.curve.Clone(); clone.Trim(0.0, trimInfo.startParam); parts.Add(clone);
                    clone = trimInfo.curve.Clone(); clone.Trim(trimInfo.startParam, trimInfo.endParam); parts.Add(clone);
                    clone = trimInfo.curve.Clone(); clone.Trim(trimInfo.endParam, 1.0); parts.Add(clone);
                }
            }
            foreach (ICurve curve in parts)
            {
                (curve as IGeoObject).CopyAttributes(trimInfo.curve as IGeoObject);
            }
            return parts;
        }

        public override string GetID()
        {
            return "ToolsTrim";
        }
    }
}
