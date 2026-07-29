using CADability;
using CADability.Actions;
using CADability.Attribute;
using CADability.GeoObject;
using MathNet.Numerics.LinearAlgebra.Factorization;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static CADability.Actions.ConstructAction;

namespace ShapeIt
{
    internal class ExtrudeFacesAction : ConstructAction
    {
        private List<Face> faces;
        private GeoVector extrusionDirection;
        private ICurve extrusionCurve; // either extrusionCurve or extrusionDirection is valid
        GeoPoint extrusionOrigin; // for calculating extrusionLength from the mouse movement
        // static so the last entered length and offset are retained for the next instantiation of the action
        // (same mechanism as height/heightOffset in CADability's Constr3DFaceExtrude)
        private static double extrusionLength, extrusionOffset;
        private LengthInput extrusionLengthInput, extrusionOffsetInput;
        private CurveInput pipeInput;
        // start and end point of a user defined extrusion direction; extrusionDirection is computed as (end - start)
        private GeoPoint directionStartPoint, directionEndPoint;
        private bool directionStartValid, directionEndValid;
        private AttributeSet attributes; // color, layer, style, ... for the created solids, shown in the property panel
        private InputContainer attributeInput; // hosts the attribute editors; opened on activation
        private Feedback feedback;



        public ExtrudeFacesAction(IEnumerable<Face> faces)
        {
            this.faces = faces.Select(face => (face.Clone() as Face)!).ToList();
            if (this.faces[0].Surface is PlaneSurface planeSurface)
            {
                extrusionDirection = planeSurface.Normal;
                extrusionOrigin = planeSurface.Location;
            }
            else
            {
                GeoPoint2D sip = this.faces[0].Area.GetSomeInnerPoint();
                extrusionDirection = this.faces[0].Surface.GetNormal(sip); // arbitrary normal direction
                extrusionOrigin = this.faces[0].Surface.PointAt(sip);
            }
        }

        public override void OnSetAction()
        {
            base.TitleId = "Constr.Solid.FaceExtrude";

            SeparatorInput separatorHeight = new SeparatorInput("Constr.Face.Extrude.SeparatorHeight");
            // input for extrusion with optional offset. The face(s) must be planar, the extrusion and offset is along the normal of the plane(s)
            extrusionLengthInput = new LengthInput("Constr.Face.Extrude.Height");
            extrusionLengthInput.SetLengthEvent += new LengthInput.SetLengthDelegate(SetExtrusionLength);
            extrusionLengthInput.GetLengthEvent += new LengthInput.GetLengthDelegate(() => { return extrusionLength; });
            extrusionLengthInput.CalculateLengthEvent += new LengthInput.CalculateLengthDelegate(CalculateExtrusionLength);

            extrusionOffsetInput = new LengthInput("Constr.Face.Extrude.HeightOffset");
            extrusionOffsetInput.SetLengthEvent += new LengthInput.SetLengthDelegate(SetExtrusionOffset);
            extrusionOffsetInput.GetLengthEvent += new LengthInput.GetLengthDelegate(() => { return extrusionOffset; });
            extrusionOffsetInput.CalculateLengthEvent += new LengthInput.CalculateLengthDelegate(CalculateExtrusionOffset);
            extrusionOffsetInput.Optional = true;

            SeparatorInput separatorVector = new SeparatorInput("Constr.Face.Extrude.SeparatorVector");
            // inputs defining an extrusion vector (when normal to faces plane isn't appropriate)
            GeoPointInput extrusionDirectionStart = new GeoPointInput("Constr.Face.ExtrusionDirection.Start");
            GeoPointInput extrusionDirectionEnd = new GeoPointInput("Constr.Face.ExtrusionDirection.End");
            extrusionDirectionStart.Optional = true;
            extrusionDirectionEnd.Optional = true;
            extrusionDirectionStart.SetGeoPointEvent += new GeoPointInput.SetGeoPointDelegate(SetExtrusionDirectionStart);
            extrusionDirectionStart.GetGeoPointEvent += new GeoPointInput.GetGeoPointDelegate(() => directionStartPoint);
            extrusionDirectionEnd.SetGeoPointEvent += new GeoPointInput.SetGeoPointDelegate(SetExtrusionDirectionEnd);
            extrusionDirectionEnd.GetGeoPointEvent += new GeoPointInput.GetGeoPointDelegate(() => directionEndPoint);

            SeparatorInput separatorPipe = new SeparatorInput("Constr.Face.Extrude.SeparatorPipe");
            // inputs defining a curve for the extrusion
            pipeInput = new CurveInput("Constr.Face.Extrude.Pipe");
            pipeInput.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(pipeInputCurves);
            pipeInput.CurveSelectionChangedEvent += new CurveInput.CurveSelectionChangedDelegate(pipeInputCurveChanged);
            pipeInput.Optional = true;

            // The attribute editors (color, layer, style, ...) for the resulting solids. Instead of abusing an
            // ActiveObject as an attribute carrier, we host our own AttributeSet in an InputContainer. It starts
            // from the previously used attributes (LastUsed), falling back to the project defaults for solids.
            attributes = AttributeSet.LastUsed;
            attributes.SetDefaults(Frame, Style.EDefaultFor.Solids);
            attributeInput = new InputContainer("GeoObject.Attributes");
            attributeInput.SetShowProperties(attributes.GetPropertyEntries(Frame));

            base.SetInput(separatorHeight, extrusionLengthInput, extrusionOffsetInput, separatorVector, extrusionDirectionStart, extrusionDirectionEnd, separatorPipe, pipeInput, attributeInput);

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
            for (int i = 0; i < faces.Count; i++)
            {
                if (extrusionCurve != null)
                {
                    Path? path = null;
                    if (extrusionCurve is Path p) path = p;
                    else path = Path.FromSegments([extrusionCurve], true);
                    IGeoObject shape = Make3D.MakePipe(faces[i], path, null);
                    if (shape != null) feedback.CreatedObjects.Add(shape);
                }
                else
                {
                    Face faceWithOffset = (faces[i].Clone() as Face)!;
                    if (extrusionOffset != 0) faceWithOffset.Modify(ModOp.Translate(extrusionOffset * extrusionDirection));
                    if (!Precision.IsNullVector(extrusionLength * extrusionDirection))
                    {
                        IGeoObject shape = Make3D.MakePrism(faceWithOffset, extrusionLength * extrusionDirection, null, false);
                        if (shape != null) feedback.CreatedObjects.Add(shape);
                    }
                }
            }
            feedback.Refresh();
            return feedback.CreatedObjects.Count > 0;
        }
        private void pipeInputCurveChanged(CurveInput sender, ICurve SelectedCurve)
        {
            extrusionCurve = SelectedCurve;
            Recalc();
        }

        private bool pipeInputCurves(CurveInput sender, ICurve[] TheCurves, bool up)
        {
            if (TheCurves == null || TheCurves.Length == 0) return false;
            extrusionCurve = TheCurves[0];
            return Recalc();
        }

        private double CalculateExtrusionOffset(GeoPoint mousePosition)
        {
            Plane pln = new Plane(extrusionOrigin, extrusionDirection);
            return pln.Distance(mousePosition);
        }

        private bool SetExtrusionOffset(double l)
        {
            extrusionOffset = l;
            return Recalc();
        }

        private double CalculateExtrusionLength(GeoPoint mousePosition)
        {   // calculate the extrusionLength as the distance of the current mouse position from the plane of the first face
            // SetExtrusionLength will be called with the result 
            Plane pln = new Plane(extrusionOrigin, extrusionDirection);
            return pln.Distance(mousePosition) - extrusionOffset;
        }

        private bool SetExtrusionLength(double l)
        {
            if (l == 0) return false;
            extrusionLength = l;
            return Recalc();
        }

        private void SetExtrusionDirectionStart(GeoPoint p)
        {
            directionStartPoint = p;
            directionStartValid = true;
            UpdateExtrusionDirection();
        }

        private void SetExtrusionDirectionEnd(GeoPoint p)
        {
            directionEndPoint = p;
            directionEndValid = true;
            UpdateExtrusionDirection();
        }

        // Once both the start and end point of the direction have been entered, derive the
        // extrusion direction from their difference (end - start). The direction is normalized
        // so that extrusionLength keeps controlling the magnitude of the extrusion.
        private void UpdateExtrusionDirection()
        {
            if (!directionStartValid || !directionEndValid) return;
            GeoVector dir = new GeoVector(directionStartPoint, directionEndPoint);
            if (dir.IsNullVector()) return;
            extrusionDirection = dir.Normalized;
            Recalc();
        }

        public override string GetID()
        { return "Constr.Solid.FaceExtrude"; }

        public override void OnDone()
        {
            Model model = Frame.ActiveView.Model;

            for (int i = 0; i < faces.Count; i++)
            {
                IGeoObject shape = null;
                if (extrusionCurve != null)
                {
                    Path path = null;
                    if (extrusionCurve is Path p) path = p;
                    else path = Path.FromSegments([extrusionCurve], true);
                    shape = Make3D.MakePipe(faces[i], path, Frame.Project);
                }
                else
                {
                    Face faceWithOffset = (faces[i].Clone() as Face)!;
                    if (extrusionOffset != 0) faceWithOffset.Modify(ModOp.Translate(extrusionOffset * extrusionDirection));
                    if (!Precision.IsNullVector(extrusionLength * extrusionDirection))
                        shape = Make3D.MakePrism(faceWithOffset, extrusionLength * extrusionDirection, Frame.Project, false);
                }
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
