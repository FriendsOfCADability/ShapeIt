using CADability.Actions;
using CADability.GeoObject;
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

        CurveInput fromCurve;
        CurveInput toCurve;
        Feedback feedback;

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

            fromCurve = new CurveInput("MapToCurve.FromCurve");
            fromCurve.MouseOverCurvesEvent += FromCurve_MouseOverCurvesEvent;

            toCurve = new CurveInput("MapToCurve.ToCurve");
            toCurve.MouseOverCurvesEvent += ToCurve_MouseOverCurvesEvent;

            SetInput(fromCurve, toCurve);

            base.OnSetAction();
            feedback = new Feedback();
            feedback.Attach(CurrentMouseView);
            base.OnSetAction();
        }

        private bool FromCurve_MouseOverCurvesEvent(CurveInput sender, ICurve[] Curves, bool up)
        {
            List<ICurve> usableCurves = new List<ICurve>();
            Curves = Curves.Where(c => c is Line || c is Ellipse).ToArray();
            if (up)
                if (Curves.Length == 0) sender.SetCurves(Curves, null); // will be displayed in property grid
                else sender.SetCurves(Curves, Curves[0]);
            if (Curves.Length > 0)
            {   // einfach die erste Kurve nehmen
                //if (Curves[0] is Line line)
                //{
                //    axisPoint = line.StartPoint;
                //    axisVector = line.StartDirection;
                //    axisOrLine = false;
                //}
                //else if (Curves[0] is Ellipse elli)
                //{
                //    axisPoint = elli.Center;
                //    axisVector = elli.Plane.Normal;
                //    axisOrLine = false;
                //}
                //updateOptional();
                //return rotateOrg(true);
            }
            base.FeedBack.ClearSelected();
            base.ShowActiveObject = false;
            return false;
        }
        private bool ToCurve_MouseOverCurvesEvent(CurveInput sender, ICurve[] TheCurves, bool up)
        {
            throw new NotImplementedException();
        }
    }
}
