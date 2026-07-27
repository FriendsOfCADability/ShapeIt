using CADability;
using CADability.Actions;
using CADability.Attribute;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Linq;
using static CADability.Actions.ConstructAction;

namespace ShapeIt
{
    /// <summary>
    /// Creates one or more solids by rotating the given face(s) about an axis. This is the modern
    /// replacement for CADability's <see cref="Constr3DFaceRotate"/>, implemented analogously to
    /// <see cref="ExtrudeFacesAction"/>: the faces are fixed (passed in the constructor), the preview is
    /// shown through a <see cref="Feedback"/> object instead of an ActiveObject, and the resulting solids'
    /// attributes are edited via an <see cref="AttributeSet"/> hosted in an InputContainer.
    /// </summary>
    internal class RotateFacesAction : ConstructAction
    {
        private List<Face> faces;
        // The rotation axis. axisPoint is a point on the axis, axisVector its direction.
        // static so the last entered axis and angles are retained for the next instantiation of the action
        // (same mechanism as height/heightOffset in ExtrudeFacesAction).
        private static GeoPoint axisPoint;
        private static GeoVector axisVector;
        private static Angle rotationAngle, offsetAngle;
        private static bool axisInitialized; // false until the axis has been seeded from the first face
        private AngleInput rotationAngleInput, offsetAngleInput;
        private CurveInput axisLineInput;
        private GeoPointInput axisPointInput;
        private GeoPointInput axisSecondPointInput; // alternative to the vector: the direction is (secondPoint - axisPoint)
        private GeoVectorInput axisVectorInput;
        private AttributeSet attributes; // color, layer, style, ... for the created solids, shown in the property panel
        private InputContainer attributeInput; // hosts the attribute editors; opened on activation
        private Feedback feedback;

        public RotateFacesAction(IEnumerable<Face> faces)
        {
            this.faces = faces.Select(face => (face.Clone() as Face)!).ToList();
            // Seed a sensible default axis from the first face on the very first use. Afterwards the last
            // used axis is retained (static fields), just like the last entered angles.
            if (!axisInitialized)
            {
                GeoPoint2D sip = this.faces[0].Area.GetSomeInnerPoint();
                axisPoint = this.faces[0].Surface.PointAt(sip);
                if (this.faces[0].Surface is PlaneSurface planeSurface)
                {
                    // an axis lying in the face plane produces a proper revolution
                    axisVector = planeSurface.DirectionX.Normalized;
                }
                else
                {
                    GeoVector normal = this.faces[0].Surface.GetNormal(sip);
                    normal.ArbitraryNormals(out GeoVector dirx, out GeoVector diry);
                    axisVector = dirx.Normalized;
                }
                rotationAngle = Math.PI;
                axisInitialized = true;
            }
        }

        public override void OnSetAction()
        {
            base.TitleId = "Constr.Solid.FaceRotate";

            SeparatorInput separatorAxis = new SeparatorInput("Constr.Face.Rotate.SeparatorAxis");
            // pick a line or (arc/circle) that defines the axis of rotation, as an alternative to typing point and direction
            axisLineInput = new CurveInput("Constr.Face.PathRotate.AxisLine");
            axisLineInput.Decomposed = true; // single elements only, even from a polyline or path
            axisLineInput.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(AxisLineCurves);
            axisLineInput.CurveSelectionChangedEvent += new CurveInput.CurveSelectionChangedDelegate(AxisLineChanged);
            // axisLineInput.Optional = true;

            SeparatorInput separatorAngle = new SeparatorInput("Constr.Face.Rotate.SeparatorAngle");
            rotationAngleInput = new AngleInput("Constr.Face.PathRotate.Angle", rotationAngle);
            rotationAngleInput.SetAngleEvent += new AngleInput.SetAngleDelegate(SetRotationAngle);
            rotationAngleInput.GetAngleEvent += new AngleInput.GetAngleDelegate(() => rotationAngle);

            offsetAngleInput = new AngleInput("Constr.Face.PathRotate.AngleOffset", offsetAngle);
            offsetAngleInput.SetAngleEvent += new AngleInput.SetAngleDelegate(SetOffsetAngle);
            offsetAngleInput.GetAngleEvent += new AngleInput.GetAngleDelegate(() => offsetAngle);
            offsetAngleInput.Optional = true;

            axisPointInput = new GeoPointInput("Constr.Face.PathRotate.AxisPoint", axisPoint);
            axisPointInput.SetGeoPointEvent += new GeoPointInput.SetGeoPointDelegate(SetAxisPoint);
            axisPointInput.GetGeoPointEvent += new GeoPointInput.GetGeoPointDelegate(() => axisPoint);
            axisPointInput.DefinesBasePoint = true;
            axisPointInput.Optional = true;

            // Second point defining the axis direction (direction = secondPoint - axisPoint). It is derived
            // from the current axisPoint/axisVector for display, so it stays consistent with the vector input.
            axisSecondPointInput = new GeoPointInput("Constr.Face.Rotate.AxisSecondPoint");
            axisSecondPointInput.SetGeoPointEvent += new GeoPointInput.SetGeoPointDelegate(SetAxisSecondPoint);
            axisSecondPointInput.GetGeoPointEvent += new GeoPointInput.GetGeoPointDelegate(() => axisPoint + axisVector);
            axisSecondPointInput.Optional = true;

            axisVectorInput = new GeoVectorInput("Constr.Face.PathRotate.AxisVector", axisVector);
            axisVectorInput.SetGeoVectorEvent += new GeoVectorInput.SetGeoVectorDelegate(SetAxisVector);
            axisVectorInput.GetGeoVectorEvent += new GeoVectorInput.GetGeoVectorDelegate(() => axisVector);
            axisVectorInput.Optional = true;

            // The attribute editors (color, layer, style, ...) for the resulting solids. Instead of abusing an
            // ActiveObject as an attribute carrier, we host our own AttributeSet in an InputContainer. It starts
            // from the previously used attributes (LastUsed), falling back to the project defaults for solids.
            attributes = AttributeSet.LastUsed;
            attributes.SetDefaults(Frame, Style.EDefaultFor.Solids);
            attributeInput = new InputContainer("GeoObject.Attributes");
            attributeInput.SetShowProperties(attributes.GetPropertyEntries(Frame));

            base.SetInput(separatorAxis, axisLineInput, separatorAngle, rotationAngleInput, offsetAngleInput, axisPointInput, axisSecondPointInput, axisVectorInput, attributeInput);

            base.ShowActiveObject = false;

            feedback = new Feedback();
            feedback.Attach(CurrentMouseView);
            // subscribe only after feedback exists, so a change event can safely update the preview
            attributes.Changed += OnAttributesChanged; // update the preview color live while the user edits it

            base.OnSetAction();

            Recalc();
        }

        public override void OnActivate(CADability.Actions.Action OldActiveAction, bool SettingAction)
        {
            base.OnActivate(OldActiveAction, SettingAction);
            // Now the action is added to the property page (propertyTreeView is set), so the attribute
            // container can be expanded right away instead of showing up collapsed.
            attributeInput.Open(true);
        }

        private void OnAttributesChanged()
        {   // the user changed color/layer/style in the panel: show the preview in the new color immediately
            if (attributes.ColorDef != null) feedback.SetCreatedObjectsColor(attributes.ColorDef.Color);
        }

        bool Recalc()
        {
            feedback.Clear();
            // show the preview in the color the user picked for the result (see attribute panel)
            if (attributes.ColorDef != null) feedback.CreatedObjectsColor = attributes.ColorDef.Color;
            if (!axisVector.IsNullVector())
            {
                double sw = rotationAngle;
                if (sw == 0.0) sw = Math.PI * 2.0; // an angle of 0 means a full revolution
                for (int i = 0; i < faces.Count; i++)
                {
                    IGeoObject shape = Make3D.Rotate((faces[i].Clone() as Face)!, new Axis(axisPoint, axisVector), sw, offsetAngle, null);
                    if (shape != null) feedback.CreatedObjects.Add(shape);
                }
            }
            feedback.Refresh();
            return feedback.CreatedObjects.Count > 0;
        }

        private bool AxisLineCurves(CurveInput sender, ICurve[] Curves, bool up)
        {   // only use curves that can define an axis: a straight line or a circle/arc (axis is its normal)
            Curves = Curves.Where(c => c is Line || c is Ellipse).ToArray();
            if (up)
                if (Curves.Length == 0) sender.SetCurves(Curves, null);
                else sender.SetCurves(Curves, Curves[0]);
            if (Curves.Length > 0)
            {
                axisLineInput.Optional = false;
                axisPointInput.Optional = true;
                axisSecondPointInput.Optional = true;
                axisVectorInput.Optional = true;
                SetAxisFromCurve(Curves[0]);
                return Recalc();
            }
            return false;
        }

        private void AxisLineChanged(CurveInput sender, ICurve SelectedCurve)
        {
            axisLineInput.Optional = false;
            axisPointInput.Optional = true;
            axisSecondPointInput.Optional = true;
            axisVectorInput.Optional = true;
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

        private bool SetRotationAngle(Angle angle)
        {   // do not reject 0: a 0 angle is interpreted as a full 360 degree revolution
            rotationAngle = angle;
            Recalc();
            return true;
        }

        private bool SetOffsetAngle(Angle angle)
        {
            offsetAngle = angle;
            Recalc();
            return true;
        }

        private void SetAxisPoint(GeoPoint p)
        {
            axisPoint = p;
            axisLineInput.Optional = true;
            axisPointInput.Optional = false;
            axisSecondPointInput.Optional = false;
            axisVectorInput.Optional = true;
            Recalc();
        }

        // The second point defines the axis direction as (secondPoint - axisPoint). Feeding it back into
        // axisVector keeps the vector input and this point input in sync (both describe the same axis).
        private void SetAxisSecondPoint(GeoPoint p)
        {
            GeoVector vec = p - axisPoint;
            if (!Precision.IsNullVector(vec)) axisVector = vec;
            axisLineInput.Optional = true;
            axisPointInput.Optional = false;
            axisSecondPointInput.Optional = false;
            axisVectorInput.Optional = true;
            Recalc();
        }

        private bool SetAxisVector(GeoVector vec)
        {
            if (Precision.IsNullVector(vec)) return false;
            axisVector = vec;
            axisLineInput.Optional = true;
            axisPointInput.Optional = false;
            axisSecondPointInput.Optional = true;
            axisVectorInput.Optional = false;
            return Recalc();
        }

        public override string GetID()
        { return "Constr.Solid.FaceRotate"; }

        public override void OnDone()
        {
            Model model = Frame.ActiveView.Model;

            if (!axisVector.IsNullVector())
            {
                double sw = rotationAngle;
                if (sw == 0.0) sw = Math.PI * 2.0; // an angle of 0 means a full revolution
                for (int i = 0; i < faces.Count; i++)
                {
                    IGeoObject shape = Make3D.Rotate((faces[i].Clone() as Face)!, new Axis(axisPoint, axisVector), sw, offsetAngle, Frame.Project);
                    if (shape != null)
                    {
                        if (shape is Solid solid)
                        {
                            // apply the attributes (color, layer, style, ...) the user chose in the attribute panel
                            attributes.ApplyTo(solid);
                            model.Add(solid);
                        }
                        else if (shape is Shell shell && !shell.HasOpenEdgesExceptPoles())
                        {
                            solid = Solid.MakeSolid(shell);
                            attributes.ApplyTo(solid);
                            model.Add(solid);
                        }
                    }
                }
            }

            base.OnDone();
        }

        public override void OnRemoveAction()
        {
            // AttributeSet.LastUsed is a shared, persistent instance, so we must unsubscribe to avoid a leak
            attributes.Changed -= OnAttributesChanged;
            feedback.Detach();
            base.OnRemoveAction();
        }
    }
}
