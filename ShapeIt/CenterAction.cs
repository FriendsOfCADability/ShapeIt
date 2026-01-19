using CADability;
using CADability.Actions;
using CADability.Curve2D;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeIt
{
    public class CenterAction : ConstructAction
    {
        public Shell shell; // the shell beeing manipulated by this action
        public Shell originalShell;
        public HashSet<object> centerOntoObjects; // the one or two objects (faces, edges or vertices) onto which it should be ceneterd
        public object centerPosition; // if null, the extent of facesToBeCentered is centered, if bRep object this object defines the center of facesToBeCentered
        private double ratio = 0.5;

        private BRepObjectInput centerOnObjects1; // one of the objects to center on. If this is something with an axis, it may be the only object reqired, otherwise we need
        private BRepObjectInput centerOnObjects2; // the second object to center on. Now we have to center between the first and second
        private DoubleInput ratioInput; // normally 0.5, bot may be between 0 and 1
        private StringInput nameInput; // Name for the Parametrics when added to the shell
        private BooleanInput preserveInput; // preserve this value, when other parametrics are applied
        private string name = "";
        private Feedback feedback; // to display the feedback

        private Parametric parametric;
        private bool validResult = false;

        public override string GetID()
        {
            return "Constr.Parametrics.Center";
        }

        public CenterAction(Shell shellToCenter)
        {
            originalShell = shellToCenter;
            centerOntoObjects = new HashSet<object>();
            feedback = new Feedback();
        }

        public override void OnSetAction()
        {
            base.TitleId = "Constr.Parametrics.Center";

            List<InputObject> actionInputs = new List<InputObject>();

            centerOnObjects1 = new BRepObjectInput("Center.CenterOnToFirst");
            centerOnObjects1.MultipleInput = false;
            centerOnObjects1.MouseOverBRepObjectsEvent += MouseOverObject;
            actionInputs.Add(centerOnObjects1);

            centerOnObjects2 = new BRepObjectInput("Center.CenterOnToSecond");
            centerOnObjects2.MultipleInput = false;
            centerOnObjects2.MouseOverBRepObjectsEvent += MouseOverObject;
            actionInputs.Add(centerOnObjects2);

            ratioInput = new DoubleInput("Center.Ratio");
            ratioInput.GetDoubleEvent += RatioInput_GetDoubleEvent;
            ratioInput.SetDoubleEvent += RatioInput_SetDoubleEvent;
            ratioInput.CalculateDoubleEvent += MouseOnRatio;
            actionInputs.Add(ratioInput);

            SeparatorInput separator = new SeparatorInput("Parametrics.AssociateParametric");
            actionInputs.Add(separator);
            nameInput = new StringInput("Parametrics.ParametricsName");
            nameInput.SetStringEvent += NameInput_SetStringEvent;
            nameInput.GetStringEvent += NameInput_GetStringEvent;
            nameInput.Optional = true;
            actionInputs.Add(nameInput);

            preserveInput = new BooleanInput("Parametrics.PreserveValue", "YesNo.Values", false);
            actionInputs.Add(preserveInput);

            SetInput(actionInputs.ToArray());
            base.OnSetAction();

            feedback.Attach(CurrentMouseView);
            ratioInput.ForceValue(0.5); // already makes the first Recalc(9 and so the first feedback update

        }

        private double MouseOnRatio(GeoPoint MousePosition)
        {
            return ratio; // don't do anything, mousinput on ratio not required
        }

        public override void OnViewsChanged()
        {
            feedback.Detach();
            feedback.Attach(CurrentMouseView);
            base.OnViewsChanged();
        }
        public override void OnDone()
        {
            if (validResult)
            {
                string parametricsName = nameInput.Content;
                Solid sld = originalShell.Owner as Solid;
                if (sld != null)
                {   // the shell was part of a Solid
                    IGeoObjectOwner owner = sld.Owner; // Model or Block
                    using (Frame.Project.Undo.UndoFrame)
                    {
                        owner.Remove(sld);
                        Solid replacement = Solid.MakeSolid(shell);
                        replacement.CopyAttributes(sld);
                        owner.Add(replacement);
                    }
                }
                else
                {
                    IGeoObjectOwner owner = shell.Owner;
                    using (Frame.Project.Undo.UndoFrame)
                    {
                        owner.Remove(originalShell);
                        owner.Add(shell);
                    }
                }
            }
            base.OnDone();
        }
        public static GeoVector GetTranslationForCentering(IEnumerable<Face> facesToBeCentered, object centerPosition, IEnumerable<object> centerOntoObjects, double ratio)
        {   // currently only extent centering implemented
            GeoPoint pointToCenter; // from where to center the faces
            GeoVector axisToCenter = GeoVector.NullVector; // this vector defines a plane for the movement, typically the axis of facesToBeCentered
            GeoPoint startPoint = GeoPoint.Invalid, endPoint = GeoPoint.Invalid; // two points for centering

            if (centerPosition is Vertex vtx) pointToCenter = vtx.Position;
            else if (centerPosition is Line ln)
            {   // an axis
                pointToCenter = ln.StartPoint;
                axisToCenter = ln.StartDirection;
            }
            // and more cases
            else
            {   // default: the extent of facesToBeCentered
                BoundingBox ext = new BoundingBox(facesToBeCentered);
                pointToCenter = ext.GetCenter();
            }
            if (centerOntoObjects.Count() == 1)
            {   // if we have a single axis, this is the center, regardless of ratio
                object f = centerOntoObjects.First();
                if (f is Edge edg && edg.Curve3D is Ellipse ellipse)
                {
                    axisToCenter = ellipse.Plane.Normal;
                    Plane plane = new Plane(pointToCenter, axisToCenter);
                    startPoint = endPoint = plane.ToGlobal(plane.Project(ellipse.Center)); // the center of the ellipse projected into the movement-plane
                }
                if (f is Face face && face.Surface is ISurfaceOfRevolution sr)
                {
                    axisToCenter = sr.Axis.Direction;
                    Plane plane = new Plane(pointToCenter, axisToCenter);
                    startPoint = endPoint = plane.ToGlobal(plane.Project(sr.Axis.Location)); // the axis intersection with the movement-plane
                }
                if (startPoint.IsValid && endPoint.IsValid)
                {
                    GeoPoint toMoveTo = startPoint + ratio * (endPoint - startPoint);
                    return toMoveTo - pointToCenter;
                }
            }
            else if (centerOntoObjects.Count() > 1)
            {   // if we have multiple objects there are many cases (which have to be implemented)
                (startPoint, endPoint) = DistanceCalculator.DistanceBetweenObjects(centerOntoObjects.First(), centerOntoObjects.ElementAt(1), GeoVector.Invalid, pointToCenter, out GeoVector degreeOfFreedom, out GeoPoint dStartPoint, out GeoPoint dEndPoint);
                if (startPoint.IsValid && endPoint.IsValid)
                {
                    GeoPoint toMoveTo = startPoint + ratio * (endPoint - startPoint);
                    // we have three possible situation concerning the degreeOfFreedom: if the degreeOfFreedom is invalid, then we must place the facesToBeCentered
                    // on this line (either the center of extent or the axis)
                    // if the degreeOfFreedom is the nullvector (e.g. between two planes) we need to move the line (startPoint-endPoint) to pass through the center
                    // if degreeOfFreedom is a real vector(e.g. between two lines), we can move the line (startPoint-endPoint) along this degreeOfFreedom to pass through the center
                    if (!degreeOfFreedom.IsValid())
                    {   // strictly position the faces between startPoint and endPoint
                        if (axisToCenter.IsNullVector())
                        {   // no freedom here
                            return toMoveTo - pointToCenter;
                        }
                        else
                        {
                            Plane movingPlane = new Plane(pointToCenter, axisToCenter);
                            return movingPlane.ToGlobal(movingPlane.Project(toMoveTo)) - pointToCenter;
                        }
                    }
                    else if (degreeOfFreedom.IsNullVector())
                    {   // the line (startPoint-endPoint) defines the direction, we can freely move this line
                        GeoVector moveLine = pointToCenter - Geometry.DropPL(pointToCenter, startPoint, endPoint);
                        toMoveTo = startPoint + moveLine + ratio * (endPoint - startPoint);
                        return toMoveTo - pointToCenter;
                    }
                    else
                    {   // eg. between two parallel lines, we can move along the center
                        Plane movingPlane = new Plane(pointToCenter, degreeOfFreedom, endPoint - startPoint);
                        return movingPlane.ToGlobal(movingPlane.Project(toMoveTo)) - pointToCenter;
                    }
                }
                // the following should be deleted
                IEnumerable<Edge> edges = centerOntoObjects.OfType<Edge>();
                IEnumerable<Vertex> vertices = centerOntoObjects.OfType<Vertex>();
                IEnumerable<Face> faces = centerOntoObjects.OfType<Face>();
                if (edges.Count() > 1)
                {
                    Edge edge = edges.First();
                    Plane commonPlane = Plane.XYPlane;
                    Edge secondEdge = centerOntoObjects.Skip(1).FirstOrDefault(o =>
                    {
                        if (o is Edge edge1 && Curves.GetCommonPlane(edge.Curve3D, edge1.Curve3D, out commonPlane)) return true;
                        return false;
                    }) as Edge;
                    if (secondEdge != null)
                    {
                        ICurve2D c1 = edge.Curve3D.GetProjectedCurve(commonPlane);
                        ICurve2D c2 = secondEdge.Curve3D.GetProjectedCurve(commonPlane);
                        GeoPoint2D p1 = GeoPoint2D.Invalid, p2 = GeoPoint2D.Invalid;
                        if (c1 is Line2D && c2 is Line2D && Precision.SameDirection(c1.StartDirection, c2.StartDirection, false))
                        {
                            GeoPoint2D c = commonPlane.Project(pointToCenter);
                            p1 = c1.PerpendicularFoot(c)[0];
                            p2 = c2.PerpendicularFoot(c)[0];
                        }
                        else
                        {
                            double dist = Curves2D.SimpleMinimumDistance(c1, c2, out p1, out p2);
                        }
                        Plane movingPlane = new Plane(pointToCenter, commonPlane.Normal);
                        if (p1.IsValid && p2.IsValid)
                        {
                            startPoint = movingPlane.ToGlobal(movingPlane.Project(commonPlane.ToGlobal(p1)));
                            endPoint = movingPlane.ToGlobal(movingPlane.Project(commonPlane.ToGlobal(p2)));
                        }
                    }
                }
                else if (faces.Count() > 1)
                {
                    Face face = faces.First();
                    Face otherFace = faces.ElementAt(1);
                    // DistanceCalculater does something similar
                    if (Surfaces.ParallelDistance(face.Surface, face.Domain, otherFace.Surface, otherFace.Domain, GeoPoint.Invalid, out GeoPoint2D uv1, out GeoPoint2D uv2))
                    {
                        GeoPoint p1 = face.Surface.PointAt(uv1);
                        GeoPoint p2 = face.Surface.PointAt(uv2);
                        if (face.Surface is PlaneSurface && otherFace.Surface is PlaneSurface)
                        {
                            // we are free to move the two points so that the line goes through pointToCenter
                            GeoPoint closest = Geometry.DropPL(pointToCenter, p1, p2);
                            GeoVector translate = pointToCenter - closest;
                            startPoint = p1 + translate;
                            endPoint = p2 + translate;
                        }
                        else
                        {   // with two parallel cylinders we still have a degree of freedom, tilted cylinder define p1, p2 exactely

                        }
                    }
                }
                else if (vertices.Count() > 1)
                {
                    Vertex v1 = vertices.ElementAt(0);
                    Vertex v2 = vertices.ElementAt(1);
                    startPoint = v1.Position;
                    endPoint = v2.Position;
                }
                // else more cases to implement. The result should be startPoint and endPoint
            }
            return GeoVector.NullVector;
        }
        private bool Recalc()
        {
            validResult = false;
            GeoVector translation = GetTranslationForCentering(originalShell.Faces, centerPosition, centerOntoObjects, ratio);
            if (translation.IsNullVector())
            {   // show feedback even if not changed
                feedback.Clear();
                feedback.ShadowFaces.Add(originalShell);
                foreach (var item in centerOntoObjects)
                {
                    if (item is IGeoObject go) feedback.FrontFaces.Add(go);
                    else if (item is Edge edg) feedback.FrontFaces.Add(edg.Curve3D as IGeoObject);
                    //else if (item is Vertex vtx) feedback.FrontFaces.Add();
                }
                feedback.Refresh();
                return false;
            }
            feedback.Clear();
            shell = originalShell.Clone() as Shell;
            shell.Modify(ModOp.Translate(translation));
            validResult = true;
            feedback.ShadowFaces.Add(shell);
            foreach (var item in centerOntoObjects)
            {
                if (item is IGeoObject go) feedback.FrontFaces.Add(go);
                else if (item is Edge edg) feedback.FrontFaces.Add(edg.Curve3D as IGeoObject);
                //else if (item is Vertex vtx) feedback.FrontFaces.Add();
            }
            feedback.Refresh();
            return true;
        }

        private bool RatioInput_SetDoubleEvent(double val)
        {
            if (val > 0 && val < 1)
            {
                ratio = val;
                Recalc();
                return true;
            }
            return false;
        }

        private double RatioInput_GetDoubleEvent()
        {
            return ratio;
        }

        private bool MouseOverObject(BRepObjectInput sender, object[] bRepObjects, bool up)
        {
            bool onFirst = sender.ResourceId == "Center.CenterOnToFirst"; // don't know, whether we need this
            if (up)
            {
                bool found = false;
                IEnumerable<Vertex> vertices = bRepObjects.OfType<Vertex>();
                if (vertices.Any())
                {   // prefer vertices, because they always come together with edges and faces
                    foreach (Vertex vtx in vertices)
                    {
                        if (centerOntoObjects.Contains(vtx)) centerOntoObjects.Remove(vtx);
                        else centerOntoObjects.Add(vtx);
                        sender.SetBRepObject(centerOntoObjects.ToArray(), null);
                        found = true;
                    }
                }
                else
                {
                    IEnumerable<Edge> edges = bRepObjects.OfType<Edge>();
                    if (edges.Any())
                    {
                        foreach (Edge edge in edges)
                        {
                            if (centerOntoObjects.Contains(edge)) centerOntoObjects.Remove(edge);
                            else centerOntoObjects.Add(edge);
                            sender.SetBRepObject(centerOntoObjects.ToArray(), null);
                            found = true;
                        }
                    }
                    else // only faces
                    {
                        for (int i = 0; i < bRepObjects.Length; i++)
                        {
                            if (bRepObjects[i] is Vertex || bRepObjects[i] is Face || bRepObjects[i] is Edge)
                            {
                                if (centerOntoObjects.Contains(bRepObjects[i])) centerOntoObjects.Remove(bRepObjects[i]);
                                else centerOntoObjects.Add(bRepObjects[i]);
                                sender.SetBRepObject(centerOntoObjects.ToArray(), null);
                                found = true;
                            }
                        }
                    }
                }
                if (found && Recalc()) sender.Fixed = true;
                return found;
            }
            else
            {
                for (int i = 0; i < bRepObjects.Length; i++)
                {
                    if (bRepObjects[i] is Vertex) return true;
                    if (bRepObjects[i] is Face) return true;
                    if (bRepObjects[i] is Edge) return true;
                }
            }
            return false;
        }

        private string NameInput_GetStringEvent()
        {
            return name;
        }

        private void NameInput_SetStringEvent(string val)
        {
            name = val;
        }
        public override void OnRemoveAction()
        {
            feedback.Detach();
            base.OnRemoveAction();
        }
        public override void OnDisplayChanged(DisplayChangeArg d)
        {
            Recalc(); // the position and size of the arrows should change
            base.OnDisplayChanged(d);
        }
    }
}
