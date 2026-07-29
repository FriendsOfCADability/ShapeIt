using CADability;
using CADability.Actions;
using CADability.GeoObject;
using CADability.Substitutes;
using System;
using System.Collections.Generic;
using static CADability.Actions.ConstructAction;

namespace ShapeIt
{
    /// <summary>
    /// Rotates one or more objects about an axis by a given angle. This is the modern replacement for
    /// CADability's <see cref="CADability.Actions.RotateObjects"/>, implemented like <see cref="ReflectObjectsAction"/>:
    /// the preview is shown through a <see cref="Feedback"/> object (in the objects' own colors) instead of a
    /// working <see cref="Block"/>, and the copy mechanism (<c>Modify.CopyObjects</c> + Shift key) is kept.
    ///
    /// An axis is always offered (pick a line/arc, or enter point and direction). A rotation <em>center point</em>
    /// is offered only when all objects are curves that lie in a common plane (or a single/collinear line): the
    /// rotation axis is then perpendicular to that plane through the picked point -- the typical "sketch" case.
    /// </summary>
    internal class RotateObjectsAction : ConstructAction
    {
        private readonly GeoObjectList originals; // the objects to be rotated (references, modified in place in OnDone)
        // The curves to be rotated (only filled when all objects are curves). Used to derive the sketch plane
        // for the center-point case (the axis is perpendicular to it).
        private readonly List<ICurve> reflectableCurves = new List<ICurve>();
        private readonly bool offerCenter;      // whether a rotation center point may be offered (see constructor)
        private readonly bool curvesHaveFixedPlane; // true when the object curves alone determine a plane
        private Plane curvesPlane;              // that plane (valid only when curvesHaveFixedPlane)

        private GeoPoint axisPoint;             // a point on the rotation axis
        private GeoVector axisVector;           // the rotation axis direction
        private Angle rotationAngle;            // the rotation angle
        private bool copyObject;
        // Start and end point define the angle as an alternative to typing it: projected into the plane
        // perpendicular to the axis (through axisPoint) they span the angle start->axisPoint->end.
        private GeoPoint startPoint, endPoint;
        private bool startPointValid, endPointValid;

        private AngleInput rotationAngleInput;
        private CurveInput axisLineInput;
        private GeoPointInput axisPointInput;
        private GeoPointInput centerPointInput; // only offered for the planar-curves case
        private GeoPointInput startPointInput, endPointInput;
        private GeoVectorInput axisVectorInput;
        private Feedback feedback;

        public RotateObjectsAction(GeoObjectList list)
        {
            originals = new GeoObjectList(list); // keep the originals; they are modified/cloned in OnDone

            // A center point is offered only when there is no BRep object (Face/Shell/Solid) and all objects are
            // curves that either lie in a common plane, or are under-determined (a single line / collinear lines,
            // which do not span a plane on their own -- GetPlanarState catches that case, GetCommonPlane cannot).
            bool hasBRep = false;
            bool allCurves = originals.Count > 0;
            bool allUnderDetermined = originals.Count > 0;
            foreach (IGeoObject go in originals)
            {
                if (go is Face || go is Shell || go is Solid) hasBRep = true;
                if (go is ICurve curve)
                {
                    reflectableCurves.Add(curve);
                    if (curve.GetPlanarState() != PlanarState.UnderDetermined) allUnderDetermined = false;
                }
                else
                {
                    allCurves = false;
                    allUnderDetermined = false;
                }
            }
            curvesHaveFixedPlane = allCurves && Curves.GetCommonPlane(reflectableCurves, out curvesPlane);
            offerCenter = !hasBRep && allCurves && (curvesHaveFixedPlane || allUnderDetermined);
        }

        public override void OnSetAction()
        {
            base.TitleId = "RotateObjects";
            copyObject = ConstrDefaults.DefaultCopyObjects;

            // Seed a sensible default axis: through the center of the objects, perpendicular to their common
            // plane if there is one, otherwise along the drawing plane normal.
            BoundingBox result = BoundingBox.EmptyBoundingBox;
            foreach (IGeoObject go in originals) result.MinMax(go.GetBoundingCube());
            axisPoint = result.GetCenter();
            base.BasePoint = axisPoint;
            axisVector = OrientToView(curvesHaveFixedPlane ? curvesPlane.Normal : base.ActiveDrawingPlane.Normal);

            rotationAngleInput = new AngleInput("RotateObjects.Angle", rotationAngle);
            rotationAngleInput.SetAngleEvent += new AngleInput.SetAngleDelegate(SetRotationAngle);
            rotationAngleInput.GetAngleEvent += new AngleInput.GetAngleDelegate(() => rotationAngle);
            rotationAngleInput.CalculateAngleEvent += new AngleInput.CalculateAngleDelegate(CalculateRotationAngle);

            // pick a line (axis directly) or an arc/circle (axis is its normal) to define the rotation axis
            axisLineInput = new CurveInput("Constr.Rotate.AxisLine");
            axisLineInput.Decomposed = true; // single elements only, even from a polyline or path
            axisLineInput.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(AxisLineCurves);
            axisLineInput.CurveSelectionChangedEvent += new CurveInput.CurveSelectionChangedDelegate(AxisLineChanged);

            axisPointInput = new GeoPointInput("Constr.Face.PathRotate.AxisPoint", axisPoint);
            axisPointInput.SetGeoPointEvent += new GeoPointInput.SetGeoPointDelegate(SetAxisPoint);
            axisPointInput.GetGeoPointEvent += new GeoPointInput.GetGeoPointDelegate(() => axisPoint);
            axisPointInput.DefinesBasePoint = true;
            axisPointInput.Optional = true;

            axisVectorInput = new GeoVectorInput("RotateObjects.Vector", axisVector);
            axisVectorInput.SetGeoVectorEvent += new GeoVectorInput.SetGeoVectorDelegate(SetAxisVector);
            axisVectorInput.GetGeoVectorEvent += new GeoVectorInput.GetGeoVectorDelegate(() => axisVector);
            axisVectorInput.Optional = true;

            // Start and end point are an alternative to the angle input: they define the rotation angle
            // together with the axis point. Both start out optional; the mutual toggle with the angle input
            // is handled in SetStartPoint/SetEndPoint (angle -> optional) and SetRotationAngle (points -> optional).
            startPointInput = new GeoPointInput("Objects.StartPoint");
            startPointInput.SetGeoPointEvent += new GeoPointInput.SetGeoPointDelegate(SetStartPoint);
            startPointInput.GetGeoPointEvent += new GeoPointInput.GetGeoPointDelegate(() => startPoint);
            startPointInput.Optional = true;

            endPointInput = new GeoPointInput("Objects.EndPoint");
            endPointInput.SetGeoPointEvent += new GeoPointInput.SetGeoPointDelegate(SetEndPoint);
            endPointInput.GetGeoPointEvent += new GeoPointInput.GetGeoPointDelegate(() => endPoint);
            endPointInput.Optional = true;

            BooleanInput copy = new BooleanInput("Modify.CopyObjects", "YesNo.Values");
            copy.DefaultBoolean = ConstrDefaults.DefaultCopyObjects;
            copy.SetBooleanEvent += new BooleanInput.SetBooleanDelegate(SetCopy);

            if (offerCenter)
            {
                // For a planar sketch the rotation center point is the primary (active) input; the axis inputs
                // stay optional. Picking a center defines the axis (perpendicular to the sketch plane).
                centerPointInput = new GeoPointInput("Objects.RefPoint", axisPoint);
                centerPointInput.SetGeoPointEvent += new GeoPointInput.SetGeoPointDelegate(SetCenterPoint);
                centerPointInput.GetGeoPointEvent += new GeoPointInput.GetGeoPointDelegate(() => axisPoint);
                centerPointInput.DefinesBasePoint = true;
                axisLineInput.Optional = true;
                base.SetInput(centerPointInput, rotationAngleInput, startPointInput, endPointInput, axisLineInput, axisPointInput, axisVectorInput, copy);
            }
            else
            {
                // No center for BRep / non-planar objects: the axis line is the primary input.
                base.SetInput(axisLineInput, rotationAngleInput, startPointInput, endPointInput, axisPointInput, axisVectorInput, copy);
            }

            base.ShowActiveObject = false;

            feedback = new Feedback();
            // the rotated preview clones should keep the colors of their originals, not a single action color
            feedback.CreatedObjectsOwnColor = true;
            feedback.Attach(CurrentMouseView);

            base.OnSetAction();

            Recalc();
        }

        private bool SetRotationAngle(Angle angle)
        {
            rotationAngle = angle;
            UseAngleInput(); // typing an angle switches back: angle required, the start/end points optional
            Recalc();
            return true;
        }

        // Drive the angle from the mouse while the angle input is active: project the pivot (axis point) and the
        // mouse onto the drawing plane and take the direction angle to the plane's x-axis. This is the natural
        // rotate-by-mouse behavior for the 2D/sketch case and is harmless otherwise, since the drawing plane
        // never degenerates to a line in the view.
        private double CalculateRotationAngle(GeoPoint mousePosition)
        {
            Plane pln = base.ActiveDrawingPlane;
            GeoVector2D dir = pln.Project(mousePosition) - pln.Project(axisPoint);
            if (dir.Length < Precision.eps) return rotationAngle; // no meaningful direction, keep the current angle
            return dir.Angle;
        }

        // Orient an axis derived from a plane normal so it points to the same side as the drawing plane normal
        // (which faces the viewer). A typed or mouse-driven angle is interpreted counterclockwise about the
        // drawing plane normal, so the applied rotation axis must point the same way for the direction to match
        // what the user sees. Start/end-point angles are self-consistent and are unaffected by this choice.
        private GeoVector OrientToView(GeoVector normal)
        {
            if (normal * base.ActiveDrawingPlane.Normal < 0) return -normal;
            return normal;
        }

        private void SetStartPoint(GeoPoint p)
        {
            startPoint = p;
            startPointValid = true;
            UseStartEndForAngle();
            UpdateAngleFromPoints();
        }

        private void SetEndPoint(GeoPoint p)
        {
            endPoint = p;
            endPointValid = true;
            UseStartEndForAngle();
            UpdateAngleFromPoints();
        }

        // Once both points are known, derive the rotation angle: project them into the plane perpendicular to
        // the axis through the axis point, then measure the swept angle start -> axisPoint -> end there.
        private bool UpdateAngleFromPoints()
        {
            if (!startPointValid || !endPointValid || Precision.IsNullVector(axisVector)) return false;
            Plane anglePlane = new Plane(axisPoint, axisVector);
            GeoPoint2D center2D = anglePlane.Project(axisPoint);
            GeoVector2D from = anglePlane.Project(startPoint) - center2D;
            GeoVector2D to = anglePlane.Project(endPoint) - center2D;
            if (from.Length < Precision.eps || to.Length < Precision.eps) return false;
            rotationAngle = new SweepAngle(from, to);
            return Recalc();
        }

        // Toggle the input optionality: the two points define the angle (angle input no longer required).
        private void UseStartEndForAngle()
        {
            rotationAngleInput.Optional = true;
            startPointInput.Optional = false;
            endPointInput.Optional = false;
        }

        // Toggle back: the angle input is used directly (the two points are no longer required).
        private void UseAngleInput()
        {
            rotationAngleInput.Optional = false;
            startPointInput.Optional = true;
            endPointInput.Optional = true;
        }

        private bool AxisLineCurves(CurveInput sender, ICurve[] Curves, bool up)
        {   // only curves that can define an axis: a straight line, or a circle/arc (axis is its normal)
            List<ICurve> usable = new List<ICurve>();
            foreach (ICurve c in Curves)
            {
                if (c is Line || c is Ellipse) usable.Add(c);
            }
            ICurve[] usableCurves = usable.ToArray();
            if (up)
            {
                if (usableCurves.Length == 0) sender.SetCurves(usableCurves, null);
                else sender.SetCurves(usableCurves, usableCurves[0]);
            }
            if (usableCurves.Length > 0)
            {
                UseAxisInput();
                SetAxisFromCurve(usableCurves[0]);
                return Recalc();
            }
            return false;
        }

        private void AxisLineChanged(CurveInput sender, ICurve SelectedCurve)
        {
            UseAxisInput();
            SetAxisFromCurve(SelectedCurve);
            Recalc();
        }

        private void SetAxisFromCurve(ICurve curve)
        {
            if (curve is Line line)
            {
                axisPoint = line.StartPoint;
                axisVector = line.StartDirection;
            }
            else if (curve is Ellipse elli)
            {
                axisPoint = elli.Center;
                axisVector = elli.Plane.Normal;
            }
        }

        private void SetAxisPoint(GeoPoint p)
        {
            axisPoint = p;
            UseAxisInput();
            axisVectorInput.Optional = false; // a point alone is not enough; the direction is required too
            Recalc();
        }

        private bool SetAxisVector(GeoVector vec)
        {
            if (Precision.IsNullVector(vec)) return false;
            axisVector = vec;
            UseAxisInput();
            axisPointInput.Optional = false;
            return Recalc();
        }

        // The center point defines the axis for a planar sketch: perpendicular to the sketch plane through p.
        private void SetCenterPoint(GeoPoint p)
        {
            GeoVector normal;
            if (curvesHaveFixedPlane)
            {
                normal = curvesPlane.Normal;
            }
            else
            {
                // under-determined object curves (single line / collinear): the plane is spanned by the object
                // curve together with the chosen center point. Reject a point that lies on the line.
                if (!Curves.GetCommonPlane(p, reflectableCurves[0], out Plane pl)) return;
                normal = pl.Normal;
            }
            axisPoint = p;
            axisVector = OrientToView(normal);
            // the center now fully defines the axis; keep it as the defining input, the axis inputs stay optional
            if (centerPointInput != null) centerPointInput.Optional = false;
            axisLineInput.Optional = true;
            axisPointInput.Optional = true;
            axisVectorInput.Optional = true;
            Recalc();
        }

        // Switch the input optionality so the axis is defined by line/point+vector rather than the center point.
        private void UseAxisInput()
        {
            if (centerPointInput != null) centerPointInput.Optional = true;
            axisLineInput.Optional = true;
            axisPointInput.Optional = true;
            axisVectorInput.Optional = true;
        }

        private void SetCopy(bool val)
        {
            copyObject = val;
        }

        // Rebuilds the preview: clones of the originals, rotated with the current axis/angle, shown via Feedback.
        private bool Recalc()
        {
            feedback.Clear();
            // mark the rotation center / axis point with a screen-sized crosshair lying in the rotation plane
            if (!Precision.IsNullVector(axisVector)) feedback.SetCrosshair(axisPoint, axisVector);
            else feedback.ClearCrosshair();
            if (!Precision.IsNullVector(axisVector) && (double)rotationAngle != 0.0)
            {
                ModOp m = ModOp.Rotate(axisPoint, axisVector, new SweepAngle(rotationAngle));
                foreach (IGeoObject go in originals)
                {
                    IGeoObject cl = go.Clone();
                    cl.Modify(m);
                    feedback.CreatedObjects.Add(cl);
                }
            }
            feedback.Refresh();
            return feedback.CreatedObjects.Count > 0;
        }

        public override string GetID()
        {
            return "RotateObjects";
        }

        public override void OnDone()
        {
            if (!Precision.IsNullVector(axisVector))
            {
                ModOp m = ModOp.Rotate(axisPoint, axisVector, new SweepAngle(rotationAngle));
                using (Frame.Project.Undo.UndoFrame)
                {
                    if (((Frame.UIService.ModifierKeys & Keys.Shift) != 0) || copyObject)
                    {
                        // keep the originals and add rotated copies
                        GeoObjectList cloned = new GeoObjectList();
                        foreach (IGeoObject go in originals)
                        {
                            IGeoObject cl = go.Clone();
                            cl.Modify(m);
                            cloned.Add(cl);
                        }
                        base.Frame.Project.GetActiveModel().Add(cloned);
                    }
                    else
                    {
                        originals.Modify(m);
                    }
                }
            }

            base.OnDone();
        }

        public override void OnRemoveAction()
        {
            feedback.Detach();
            base.OnRemoveAction();
        }
    }
}
