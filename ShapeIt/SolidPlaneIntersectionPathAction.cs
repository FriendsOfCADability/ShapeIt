using CADability.Actions;
using CADability.GeoObject;
using CADability;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static CADability.Actions.ConstructAction;
using CADability.Curve2D;
using System.Drawing.Drawing2D;

namespace ShapeIt
{
    internal class SolidPlaneIntersectionPathAction : ConstructAction
    {
        private Plane plane; // die zentrale Ebene
        private Polyline feedBackPolyLine; // Darstellungsrechteck für die Ebene
        private FeedBackPlane feedBackplane;
        private double width, height; // Breite und Höhe der Ebene für Feedback
        private Solid solidToIntersect;
        public SolidPlaneIntersectionPathAction(Solid sld)
        {
            solidToIntersect = sld;
        }
        private void RecalcFeedbackPolyLine()
        {
            GeoPoint2D center = plane.Project(base.CurrentMousePosition); // aktueller Mauspunkt in der Ebene
            center = GeoPoint2D.Origin;
            GeoPoint[] p = new GeoPoint[4];
            p[0] = plane.ToGlobal(new GeoPoint2D(center.x + width / 2.0, center.y + height / 2.0));
            p[1] = plane.ToGlobal(new GeoPoint2D(center.x + width / 2.0, center.y - height / 2.0));
            p[2] = plane.ToGlobal(new GeoPoint2D(center.x - width / 2.0, center.y - height / 2.0));
            p[3] = plane.ToGlobal(new GeoPoint2D(center.x - width / 2.0, center.y + height / 2.0));
            feedBackPolyLine.SetPoints(p, true);// Ebene als Rechteck (4 Punkte) darstellen
            Plane fpln = new Plane(plane.ToGlobal(center), plane.DirectionX, plane.DirectionY);
            feedBackplane.Set(fpln, width, height);
        }
        bool SetPlaneInput(Plane val)
        {
            plane = val;
            RecalcFeedbackPolyLine();
            return true;
        }
        public override void OnSetAction()
        {
            //            base.ActiveObject = Polyline.Construct();
            base.TitleId = "Constr.PlaneIntersection";
            plane = base.ActiveDrawingPlane;
            feedBackPolyLine = Polyline.Construct();
            Rectangle rect = Frame.ActiveView.DisplayRectangle; // sinnvolles Standardrechteck
            width = Frame.ActiveView.Projection.DeviceToWorldFactor * rect.Width / 2.0;
            height = Frame.ActiveView.Projection.DeviceToWorldFactor * rect.Height / 2.0;
            base.ActiveObject = feedBackPolyLine;

            PlaneInput planeInput = new PlaneInput("Constr.Plane");
            planeInput.SetPlaneEvent += new PlaneInput.SetPlaneDelegate(SetPlaneInput);

            base.SetInput(planeInput);
            base.ShowAttributes = true;

            feedBackplane = new FeedBackPlane(plane, width, height);
            base.ShowActiveObject = false;
            base.OnSetAction();
            RecalcFeedbackPolyLine();
            base.Frame.SnapMode |= SnapPointFinder.SnapModes.SnapToFaceSurface;
        }
        public override void OnActivate(CADability.Actions.Action OldActiveAction, bool SettingAction)
        {
            base.FeedBack.Add(feedBackplane);
            base.OnActivate(OldActiveAction, SettingAction);
        }
        public override void OnInactivate(CADability.Actions.Action NewActiveAction, bool RemovingAction)
        {
            base.FeedBack.Remove(feedBackplane);
            base.OnInactivate(NewActiveAction, RemovingAction);
        }
        public override void OnRemoveAction()
        {
            base.OnRemoveAction();
            base.Frame.SnapMode &= ~SnapPointFinder.SnapModes.SnapToFaceSurface;
        }
        public override string GetID()
        { return "Constr.PlaneIntersection"; }
        private ICurve[] Intersect(PlaneSurface pls)
        {
            Plane plane = pls.Plane;
            BoundingCube bc = solidToIntersect.GetBoundingCube();
            if (bc.Interferes(plane))
            {
                return solidToIntersect.GetPlaneIntersection(pls);
            }
            return new ICurve[0];
        }
        public override void OnDone()
        {
            if (base.ActiveObject != null)
            {
                using (Frame.Project.Undo.UndoFrame)
                {
                    Frame.ActiveView.Canvas.Cursor = "WaitCursor";
                    GeoObjectList ToAdd = new GeoObjectList();
                    Model m = Frame.ActiveView.Model;
                    PlaneSurface pls = new PlaneSurface(plane);
                    ICurve[] crvs = Intersect(pls);
                    if (crvs != null)
                    {
                        for (int k = 0; k < crvs.Length; k++)
                        {
                            IGeoObject go = crvs[k] as IGeoObject;
                            go.CopyAttributes(base.ActiveObject);
                            ToAdd.Add(go);
                        }
                    }
                    List<Path> paths = Path.FromSegments(new List<ICurve>(crvs));
                    foreach (Path path in paths)
                    {
                        path.CopyAttributes(base.ActiveObject);
                        m.Add(path);
                    }
                    base.KeepAsLastStyle(ActiveObject);
                    base.ActiveObject = null;
                    base.OnDone();
                }
            }
        }
    }
}
