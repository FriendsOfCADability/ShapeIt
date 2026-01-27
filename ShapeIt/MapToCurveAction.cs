using CADability;
using CADability.Actions;
using CADability.GeoObject;
using MathNet.Numerics.LinearAlgebra.Factorization;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeIt
{
    internal class MapToCurveAction : ConstructAction
    {
        List<Solid> solidsToMap;

        CurveInput? sourceCurve;
        CurveInput? targetCurve;
        BooleanInput? reverse;
        MultipleChoiceInput? mode;
        LengthInput? offset;
        Feedback? feedback;

        ICurve? mouseOverTargetCurve;
        List<Solid>? modifiedSolids;

        public MapToCurveAction(IEnumerable<Solid> solidsToMap)
        {
            this.solidsToMap = new List<Solid>(solidsToMap);

        }
        public override string GetID()
        {
            return "Construct.MapToCurve";
        }
        public override void OnSetAction()
        {
            TitleId = "Construct.MapToCurve";

            sourceCurve = new CurveInput("MapToCurve.SourceCurve");
            sourceCurve.MouseOverCurvesEvent += SourceCurve_MouseOverCurvesEvent;

            targetCurve = new CurveInput("MapToCurve.TargetCurve");
            targetCurve.MouseOverCurvesEvent += TargetCurve_MouseOverCurvesEvent;

            reverse = new BooleanInput("MapToCurve.Reverse", "MapToCurve.Reverse.Values");
            reverse.Optional = true;
            reverse.SetBooleanEvent += Reverse_SetBooleanEvent;

            mode = new MultipleChoiceInput("MapToCurve.Mode", "MapToCurve.Mode.Values", 1);
            mode.SetChoiceEvent += Mode_SetChoiceEvent;
            mode.Optional = true;

            offset = new LengthInput("MapToCurve.Offset");
            offset.SetLengthEvent += Offset_SetLengthEvent;
            offset.Optional = true;

            SetInput(sourceCurve, targetCurve, reverse, mode, offset);

            feedback = new Feedback();
            feedback.Attach(CurrentMouseView);
            base.OnSetAction();
        }

        private bool Offset_SetLengthEvent(double Length)
        {
            Recalc();
            return true;
        }

        private void Mode_SetChoiceEvent(int val)
        {
            Recalc();
        }

        private void Reverse_SetBooleanEvent(bool val)
        {
            Recalc();
        }

        private bool SourceCurve_MouseOverCurvesEvent(CurveInput sender, ICurve[] Curves, bool up)
        {
            List<ICurve> usableCurves = new List<ICurve>();
            // Curves = Curves.Where(c => c is Line || c is Ellipse).ToArray();
            if (up)
            {
                if (Curves.Length == 0) sender.SetCurves(Curves, null); // will be displayed in property grid
                else sender.SetCurves(Curves, Curves[0]);
            }
            if (Curves.Length > 0)
            {
            }
            Recalc();
            return Curves.Length > 0;
        }
        private bool Recalc()
        {
            feedback!.Clear();
            modifiedSolids = [];

            ICurve? sc = sourceCurve!.GetSelectedCurve();
            ICurve? tc = targetCurve!.GetSelectedCurve();
            if (tc == null) tc = mouseOverTargetCurve; // not fixed yet
            if (sc != null && tc != null)
            {
                for (int i = 0; i < solidsToMap.Count; i++)
                {
                    GeoPoint cnt = solidsToMap[i].GetExtent(0.0).GetCenter();
                    double pos = sc.PositionOf(cnt);
                    if (pos >= -Precision.eps && pos <= 1 + Precision.eps)
                    {
                        GeoPoint fromLocation = sc.PointAt(pos);
                        ICurve toCalculateLength = sc.Clone();
                        toCalculateLength.Trim(0.0, pos);
                        double length = toCalculateLength.Length;
                        double tcpos = tc.PositionAtLength(length);
                        GeoPoint toLocation = tc.PointAt(tcpos);
                        GeoVector fromDirection = sc.DirectionAt(pos);
                        GeoVector toDirection = tc.DirectionAt(tcpos);
                        ModOp translate = ModOp.Translate(toLocation - fromLocation);
                        ModOp rotate = ModOp.Rotate(toLocation, fromDirection, toDirection);
                        Solid? clone = solidsToMap[i].Clone() as Solid;
                        if (clone != null)
                        {
                            clone.Modify(rotate * translate);
                            modifiedSolids.Add(clone);
                        }
                    }
                    feedback.FrontFaces.AddRange(modifiedSolids);
                }
            }
            feedback.Refresh();
            return modifiedSolids.Count > 0;
        }
        private bool TargetCurve_MouseOverCurvesEvent(CurveInput sender, ICurve[] Curves, bool up)
        {
            List<ICurve> usableCurves = [];
            // Curves = Curves.Where(c => c is Line || c is Ellipse).ToArray();
            if (up)
            {
                if (Curves.Length == 0) sender.SetCurves(Curves, null); // will be displayed in property grid
                else sender.SetCurves(Curves, Curves[0]);
            }
            if (Curves.Length > 0)
            {
                mouseOverTargetCurve = Curves[0];
            }
            else
            {
                mouseOverTargetCurve = null;
            }
            if (up)
            {
                return Recalc();
            }
            else
            {
                Recalc();
                return Curves.Length > 0;
            }
        }
        public override void OnDone()
        {
            Recalc();
            if (modifiedSolids != null && modifiedSolids.Count > 0)
            {
                IGeoObjectOwner owner = solidsToMap[0].Owner; // Model or Block
                using (Frame.Project.Undo.UndoFrame)
                {
                    for (int i = 0; i < solidsToMap.Count; ++i) owner.Remove(solidsToMap[i]);
                    for (int i = 0; i < modifiedSolids.Count; ++i) owner.Add(modifiedSolids[i]);
                }
            }
            feedback?.Clear();
            base.OnDone();
        }
        public override void OnRemoveAction()
        {
            feedback?.Detach();
            base.OnRemoveAction();
        }

    }
}
