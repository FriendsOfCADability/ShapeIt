using CADability.Curve2D;
using CADability.GeoObject;
using CADability.UserInterface;
using System.Collections.Generic;

namespace CADability.Actions
{
    internal class ExpandObjectsAction : ConstructAction
    {
        // The optional stop curve: if set, the picked curves are extended up to their intersection with this curve.
        // If null, each curve is extended up to the next curve in the model that lies in its extension.
        private ICurve stopCurve;
        private CurveInput stopObject; // the input field for stopCurve

        // Describes how a single curve has to be extended.
        private class ExpandInfo
        {
            public ICurve curve;      // the curve to extend
            public bool lowerEnd;     // true: move the start point; false: move the end point
            public GeoPoint newPoint; // the new position of that end point
        }

        public override void OnSetAction()
        {
            base.ActiveObject = null;
            base.TitleId = "ToolsExpand";
            stopObject = new CurveInput("ToolsExpand.SourceObject");
            stopObject.Optional = true;
            stopObject.Decomposed = true;
            stopObject.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(MouseOverStopObject);
            stopObject.CurveSelectionChangedEvent += new CurveInput.CurveSelectionChangedDelegate(StopObjectChanged);
            CurveInput curveInput = new CurveInput("ToolsExpand.Object"); // the input field for the curves to be extended
            curveInput.ModifiableOnly = true;
            curveInput.HitCursor = CursorTable.GetCursor("Expand.cur");
            curveInput.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(MouseOverCurvesToExpand);
            curveInput.PreferPath = true; // prefer paths, i.e. extend a whole path instead of one of its sub curves
            base.SetInput(curveInput, stopObject);
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

        private bool MouseOverCurvesToExpand(CurveInput sender, ICurve[] curves, bool up)
        {
            // Handle mouse over events for the curves to be extended. When several curves are under the cursor, all of
            // them are extended at once: the user has no influence on their order anyway, and if he wants to extend a
            // single object he can zoom in until only that object lies under the cursor.
            FeedBack.ClearSelected();
            List<ExpandInfo> expandInfos = new List<ExpandInfo>();
            foreach (ICurve curve in curves)
            {
                if (curve == stopCurve) continue; // never extend the stop curve itself
                ExpandInfo expandInfo = ComputeExpandInfo(curve);
                if (expandInfo != null) expandInfos.Add(expandInfo);
            }
            if (expandInfos.Count == 0) return false;
            if (up)
            {   // extend the curves in place
                using (base.Frame.Project.Undo.UndoFrame)
                {
                    foreach (ExpandInfo expandInfo in expandInfos)
                    {
                        if (expandInfo.lowerEnd) expandInfo.curve.StartPoint = expandInfo.newPoint;
                        else expandInfo.curve.EndPoint = expandInfo.newPoint;
                    }
                }
            }
            else
            {   // show the extended curves as a preview
                foreach (ExpandInfo expandInfo in expandInfos)
                {
                    FeedBack.AddSelected(GetPreview(expandInfo) as IGeoObject);
                }
            }
            return true;
            // when true is returned and up == true, this action terminates, because there are no more open (unfixed) inputs.
        }

        /// <summary>
        /// Returns a clone of the curve extended to its new end point (used for the feedback preview).
        /// </summary>
        private ICurve GetPreview(ExpandInfo expandInfo)
        {
            ICurve preview = expandInfo.curve.Clone();
            if (expandInfo.lowerEnd) preview.StartPoint = expandInfo.newPoint;
            else preview.EndPoint = expandInfo.newPoint;
            (preview as IGeoObject).CopyAttributes(expandInfo.curve as IGeoObject);
            return preview;
        }

        /// <summary>
        /// Determines how <paramref name="toExtend"/> has to be extended, based on the current cursor position and the
        /// intersections with the stop curve or with the nearby model curves. Returns null if the curve cannot be
        /// extended (closed curves and NURBS/BSplines are not extendable, or there is no intersection to extend to).
        /// </summary>
        private ExpandInfo ComputeExpandInfo(ICurve toExtend)
        {
            if (toExtend == null) return null;
            if (toExtend.IsClosed) return null;    // a closed curve cannot be extended
            if (toExtend is BSpline) return null;  // a NURBS curve cannot be sensibly extended

            // The pick position decides which end is extended: the start ("lower") end when the first half is picked,
            // otherwise the end. It is evaluated in 2D, because in 3D the pick point may be far from the curve.
            ICurve2D curve2D = toExtend.GetProjectedCurve(CurrentMouseView.Projection.ProjectionPlane);
            if (curve2D == null) return null;
            GeoPoint2D pick2D = CurrentMouseView.Projection.ProjectUnscaled(base.CurrentMousePosition);
            bool lowerEnd = curve2D.PositionOf(pick2D) < 0.5;

            double[] cutPlace;
            if (stopCurve != null)
            {   // intersections with the explicitly chosen stop curve, including those on the extension of toExtend
                cutPlace = Curves.Intersect(toExtend, stopCurve, false);
            }
            else
            {   // intersections with all model curves that lie in the extension of the relevant end
                ProjectedModel.IntersectionMode mode = lowerEnd
                    ? ProjectedModel.IntersectionMode.StartExtension
                    : ProjectedModel.IntersectionMode.EndExtension;
                cutPlace = base.Frame.ActiveView.ProjectedModel.GetIntersectionParameters(toExtend, mode);
            }

            // find the closest intersection beyond the relevant end: for the start end the largest parameter below 0,
            // for the end the smallest parameter above 1 (the curve's own end points are excluded)
            const double eps = 1e-8;
            double param = lowerEnd ? double.MinValue : double.MaxValue;
            bool found = false;
            for (int i = 0; i < cutPlace.Length; i++)
            {
                if (lowerEnd)
                {
                    if (cutPlace[i] > param && cutPlace[i] < -eps) { param = cutPlace[i]; found = true; }
                }
                else
                {
                    if (cutPlace[i] < param && cutPlace[i] > 1.0 + eps) { param = cutPlace[i]; found = true; }
                }
            }
            if (!found) return null;
            return new ExpandInfo() { curve = toExtend, lowerEnd = lowerEnd, newPoint = toExtend.PointAt(param) };
        }

        public override string GetID()
        {
            return "ToolsExpand";
        }
    }
}
