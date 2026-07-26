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
        private double extrusionLength, extrusionOffset;
        private LengthInput extrusionLengthInput, extrusionOffsetInput;
        private CurveInput pipeInput;
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

            SeparatorInput separatorHeight = new SeparatorInput("Constr.Face.PathExtrude.SeparatorHeight");
            // input for extrusion with optional offset. The face(s) must be planar, the extrusion and offset is along the normal of the plane(s)
            extrusionLengthInput = new LengthInput("Constr.Face.PathExtrude.Height");
            extrusionLengthInput.SetLengthEvent += new LengthInput.SetLengthDelegate(SetExtrusionLength);
            extrusionLengthInput.GetLengthEvent += new LengthInput.GetLengthDelegate(() => { return extrusionLength; });
            extrusionLengthInput.CalculateLengthEvent += new LengthInput.CalculateLengthDelegate(CalculateExtrusionLength);

            extrusionOffsetInput = new LengthInput("Constr.Face.PathExtrude.HeightOffset");
            extrusionOffsetInput.SetLengthEvent += new LengthInput.SetLengthDelegate(SetExtrusionOffset);
            extrusionOffsetInput.GetLengthEvent += new LengthInput.GetLengthDelegate(() => { return extrusionOffset; });
            extrusionOffsetInput.CalculateLengthEvent += new LengthInput.CalculateLengthDelegate(CalculateExtrusionOffset);
            extrusionOffsetInput.Optional = true;

            SeparatorInput separatorVector = new SeparatorInput("Constr.Face.PathExtrude.SeparatorVector");
            // inputs defining an extrusion vector (when normal to faces plane isn't appropriate)
            GeoPointInput extrusionDirectionStart = new GeoPointInput("Constr.Face.ExtrusionDirection.Start");
            GeoPointInput extrusionDirectionEnd = new GeoPointInput("Constr.Face.ExtrusionDirection.End");
            extrusionDirectionStart.Optional = true;
            extrusionDirectionEnd.Optional = true;

            SeparatorInput separatorPipe = new SeparatorInput("Constr.Face.PathExtrude.SeparatorPipe");
            // inputs defining a curve for the extrusion
            pipeInput = new CurveInput("Constr.Face.PathExtrude.Pipe");
            pipeInput.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(pipeInputCurves);
            pipeInput.CurveSelectionChangedEvent += new CurveInput.CurveSelectionChangedDelegate(pipeInputCurveChanged);
            pipeInput.Optional = true;

            base.SetInput(separatorHeight, extrusionLengthInput, extrusionOffsetInput, separatorVector, extrusionDirectionStart, extrusionDirectionEnd, separatorPipe, pipeInput);

            base.ShowAttributes = true;
            base.ShowActiveObject = false;

            feedback = new Feedback();
            feedback.Attach(CurrentMouseView);

            base.OnSetAction();

            Recalc();
        }

        bool Recalc()
        {
            feedback.Clear();
            for (int i = 0; i < faces.Count; i++)
            {
                if (extrusionCurve != null)
                {
                    Path? path = null;
                    if (extrusionCurve is Path p) path = p;
                    else path = Path.FromSegments([extrusionCurve], true);
                    IGeoObject shape = Make3D.MakePipe(faces[i], path, null);
                    if (shape != null) feedback.FrontFaces.Add(shape);
                }
                else
                {
                    Face faceWithOffset = (faces[i].Clone() as Face)!;
                    if (extrusionOffset != 0) faceWithOffset.Modify(ModOp.Translate(extrusionOffset * extrusionDirection));
                    if (!Precision.IsNullVector(extrusionLength * extrusionDirection))
                    {
                        IGeoObject shape = Make3D.MakePrism(faceWithOffset, extrusionLength * extrusionDirection, null, false);
                        if (shape != null) feedback.FrontFaces.Add(shape);
                    }
                }
            }
            feedback.Refresh();
            return feedback.FrontFaces.Count > 0;
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
            return true;
        }

        private double CalculateExtrusionLength(GeoPoint mousePosition)
        {   // calculate the extrusionLength as the distance of the current mouse position from the plane of the first face
            // SetExtrusionLength will be called with the result 
            Plane pln = new Plane(extrusionOrigin,extrusionDirection);
            return pln.Distance(mousePosition)-extrusionOffset;
        }

        private bool SetExtrusionLength(double l)
        {
            if (l == 0) return false;
            extrusionLength = l;
            return Recalc();
        }

        public override string GetID()
        { return "Constr.Solid.FaceExtrude"; }

        public override void OnDone()
        {
            Model model = Frame.ActiveView.Model;
            Style style = Frame.Project.StyleList.GetDefault(Style.EDefaultFor.Solids);

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
                        solid.Style = style;
                        model.Add(solid);
                    }
                    else if (shape is Shell shell && !shell.HasOpenEdgesExceptPoles())
                    {
                        solid = Solid.MakeSolid(shell);
                        solid.Style = style;
                        model.Add(solid);
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
