using CADability;
using CADability.Actions;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeIt
{
    internal class HelicalExtrudeAction : ConstructAction
    {
        private Face faceToExtrude;
        private Axis axis;
        private double pitch;
        private double extrusionHeight;
        private double extrusionOffset;

        private static Axis defaultAxis = Axis.InvalidAxis;
        private static double defaultPitch = 10.0;
        private static double defaultExtrusionHeight = 50.0;
        private static double defaultExtrusionOffset = 0.0;

        CurveInput? axisLineInput;
        GeoPointInput? axisPointInput;
        GeoVectorInput? axisDirectionInput;
        LengthInput? pitchInput;
        LengthInput? extrusionHeightInput;
        LengthInput? extrusionOffsetInput;

        Feedback? feedback;

        public override string GetID()
        {
            return "Construct.HelicalExtrude";
        }

        public HelicalExtrudeAction(Face faceToExtrude)
        {
            this.faceToExtrude = faceToExtrude;
        }
        public override void OnSetAction()
        {
            TitleId = "Construct.HelicalExtrude";
            if (defaultAxis.IsValid)
            {
                axis = defaultAxis;
            }
            else
            {
                axis = new Axis(faceToExtrude.GetExtent(0.0).GetCenter(), faceToExtrude.Surface.GetNormal(faceToExtrude.Area.GetSomeInnerPoint()));
            }
            pitch = defaultPitch;
            extrusionHeight = defaultExtrusionHeight;
            extrusionOffset = defaultExtrusionOffset;

            axisLineInput = new CurveInput("Helical.AxisLine");
            axisLineInput.Decomposed = true; // nur Einzelelemente, auch bei Polyline und Pfad
            axisLineInput.MouseOverCurvesEvent += AxisLine;
            axisLineInput.CurveSelectionChangedEvent += AxisLineChanged;
            axisLineInput.Optional = true;

            axisPointInput = new GeoPointInput("Helical.AxisPoint");
            axisPointInput.Optional = true;
            axisPointInput.SetGeoPointEvent += (p) => { axis.Location = p; Recalc(); };
            axisPointInput.GetGeoPointEvent += () => axis.Location;

            axisDirectionInput = new GeoVectorInput("Helical.AxisDirection");
            axisDirectionInput.Optional = true;
            axisDirectionInput.SetGeoVectorEvent += (v) => { axis.Direction = v; Recalc(); return true; };
            axisDirectionInput.GetGeoVectorEvent += () => axis.Direction;

            pitchInput = new LengthInput("Helical.Pitch");
            pitchInput.SetLengthEvent += (l) => { if (l > Precision.eps) { pitch = l; Recalc(); return true; } return false; };
            pitchInput.GetLengthEvent += () => pitch;

            extrusionHeightInput = new LengthInput("Helical.ExtrusionHeight");
            extrusionHeightInput.SetLengthEvent += (l) => { extrusionHeight = l; Recalc(); return true; };
            extrusionHeightInput.GetLengthEvent += () => extrusionHeight;

            extrusionOffsetInput = new LengthInput("Helical.ExtrusionOffset");
            extrusionOffsetInput.SetLengthEvent += (l) => { extrusionOffset = l; Recalc(); return true; };
            extrusionOffsetInput.GetLengthEvent += () => extrusionOffset;

            SetInput(axisLineInput, axisPointInput, axisDirectionInput, pitchInput, extrusionHeightInput, extrusionOffsetInput);

            feedback = new Feedback();
            feedback.Attach(CurrentMouseView);
            base.OnSetAction();
        }

        private void AxisLineChanged(CurveInput sender, ICurve SelectedCurve)
        {
            if (SelectedCurve is Line l)
            {
                axis = new Axis(l.StartPoint, l.StartDirection.Normalized);
                Recalc();
            }
        }

        private bool AxisLine(CurveInput sender, ICurve[] curves, bool up)
        {
            for (int i = 0; i < curves.Length; i++)
            {
                if (curves[i] is Line l)
                {
                    if (up)
                    {
                        axis = new Axis(l.StartPoint, l.StartDirection.Normalized);
                        Recalc();
                    }
                    return true;
                }
            }
            return false;
        }

        private void Recalc()
        {
            feedback?.Clear();

            Shell shell = Make3D.MakeHelicalSolid(faceToExtrude, axis, pitch, extrusionHeight, extrusionOffset, true);
            feedback?.FrontFaces.AddRange(shell.Faces);
            feedback?.Refresh();
        }

        public override void OnDone()
        {
            Shell shell = Make3D.MakeHelicalSolid(faceToExtrude, axis, pitch, extrusionHeight, extrusionOffset, true);
            if (shell!=null)
            {
                Solid sld = Solid.MakeSolid(shell);
                Frame.Project.GetActiveModel()?.Add(sld);
                defaultAxis = axis;
                defaultPitch = pitch;
                defaultExtrusionHeight = extrusionHeight;
                defaultExtrusionOffset = extrusionOffset;
            }
            base.OnDone();
        }
        public override void OnRemoveAction()
        {
            feedback?.Detach();
            base.OnRemoveAction();
        }

    }
}
