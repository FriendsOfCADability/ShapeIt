using CADability;
using CADability.Actions;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;

namespace ShapeIt
{
    public class ParametricsDistanceAction : ConstructAction
    {
        private GeoPoint point1; // the first point of the line defining the distance
        private GeoPoint point2; // the second point of the line defining the distance. these points are not changed but together with distance they define how facesToMove and facesToKeep have to be moved
        private double distance; // the actual distance as provided by distanceInput
        private Shell shell; // the shell containing the edge
        private HashSet<Face> forwardFaces; // list of the faces in the forward direction
        private HashSet<Face> backwardFaces; // list of the faces in the backward direction
        private GeoPoint touchingPoint; // here we should display the feedback arrow
        private Line axis = null; // if an axis is provided, we need another object for the distance
        private bool validResult;
        private LengthInput distanceInput; // input filed of the distance
        private MultipleChoiceInput modeInput; // input field for the mode "forward", backward", "symmetric"
        private GeoObjectInput forwardFacesInput; // input field for more faces to modify in forward direction
        private GeoObjectInput backwardFacesInput; // input field for more faces to modify in bacward direction
        private enum Mode { forward, symmetric, backward };
        private Mode mode;
        private GeoObjectList feedbackDimension; // feedback arrows
        private Feedback feedback; // to display the feedback
        private Shell feedbackResult; // the shell, which will be the result

        private StringInput nameInput;
        private string parametricsName;
        private BooleanInput preserveInput;
        private ParametricDistanceProperty parametricProperty;

        //public ParametricsDistanceAction(object distanceFromHere, IFrame frame)
        //{
        //    this.distanceFromHere = distanceFromHere;
        //    feedbackPlane = frame.ActiveView.Projection.ProjectionPlane; // TODO: find a better plane!
        //    IGeoObject owner;
        //    if (distanceFromHere is Vertex vtx)
        //    {
        //        owner = vtx.Edges[0].Curve3D as IGeoObject;
        //        offsetStartPoint = vtx.Position;
        //    }
        //    else if (distanceFromHere is Edge edge)
        //    {
        //        owner = edge.PrimaryFace;
        //        offsetStartPoint = edge.Curve3D.PointAt(0.5);
        //    }
        //    else if (distanceFromHere is Face fc)
        //    {
        //        owner = fc;
        //        GeoPoint2D inner = fc.Area.GetSomeInnerPoint();
        //        offsetStartPoint = fc.Surface.PointAt(inner);
        //    }
        //    else throw new ApplicationException("ParametricsDistance must be called with a vertex, edge or face");
        //    offsetFeedBack = frame.ActiveView.Projection.MakeArrow(offsetStartPoint, offsetStartPoint, feedbackPlane, Projection.ArrowMode.circleArrow);
        //    while (owner != null && !(owner is Shell)) owner = owner.Owner as IGeoObject;
        //    shell = owner as Shell; // this is the shell to be modified
        //    this.frame = frame;
        //    faces2 = new List<Face>();
        //    faces1 = new List<Face>();
        //    facesToMoveIsFixed = false;
        //}

        //public ParametricsDistanceAction(IEnumerable<Face> facesToMove, IFrame frame)
        //{
        //    this.frame = frame;
        //    this.faces2 = new List<Face>(facesToMove);
        //    this.faces1 = new List<Face>();
        //    facesToMoveIsFixed = true;
        //    shell = this.faces2[0].Owner as Shell;
        //    distanceFromHere = null;
        //}

        public ParametricsDistanceAction(IEnumerable<Face> facesToMove, Line axis, IFrame frame) : base()
        {
            forwardFaces = new HashSet<Face>(facesToMove);
            backwardFaces = new HashSet<Face>();
            shell = forwardFaces.First().Owner as Shell;
            this.axis = axis;
            feedbackDimension = new GeoObjectList(axis);
            feedback = new Feedback();

        }
        //public ParametricsDistanceAction(Edge fromHere, Edge toHere, Line feedback, Plane plane, IFrame frame)
        //{
        //    distanceFromHere = fromHere;
        //    distanceToHere = toHere;
        //    offsetStartPoint = feedback.StartPoint;
        //    offsetFeedBack = frame.ActiveView.Projection.MakeArrow(feedback.StartPoint, feedback.EndPoint, plane, Projection.ArrowMode.circleArrow);
        //    feedbackPlane = plane;
        //    originalOffset = feedback.EndPoint - feedback.StartPoint;
        //    shell = fromHere.PrimaryFace.Owner as Shell;
        //    faces2 = new List<Face>();
        //    faces1 = new List<Face>();
        //    if (toHere != null)
        //    {
        //        if (!toHere.PrimaryFace.Surface.IsExtruded(originalOffset)) faces2.Add(toHere.PrimaryFace);
        //        if (!toHere.SecondaryFace.Surface.IsExtruded(originalOffset)) faces2.Add(toHere.SecondaryFace);
        //        facesToMoveIsFixed = true;
        //    }
        //    if (fromHere != null)
        //    {
        //        if (!fromHere.PrimaryFace.Surface.IsExtruded(originalOffset)) faces1.Add(fromHere.PrimaryFace);
        //        if (!fromHere.SecondaryFace.Surface.IsExtruded(originalOffset)) faces1.Add(fromHere.SecondaryFace);
        //    }
        //}
        public ParametricsDistanceAction(IEnumerable<Face> part1, IEnumerable<Face> part2, GeoPoint point1, GeoPoint point2, GeoPoint touchingPoint, IFrame frame) : base()
        {
            //distanceFromHere = fromHere;
            //distanceToHere = toHere;
            forwardFaces = new HashSet<Face>(part2);
            backwardFaces = new HashSet<Face>(part1);
            shell = forwardFaces.First().Owner as Shell;
            this.point2 = point2;
            this.point1 = point1;
            this.touchingPoint = touchingPoint;
            feedbackDimension = FeedbackArrow.MakeLengthArrow(shell, forwardFaces.First(), backwardFaces.First(), null, point2 - point1, touchingPoint, frame.ActiveView, FeedbackArrow.ArrowFlags.secondRed);
            feedback = new Feedback();
        }

        public ParametricsDistanceAction(IEnumerable<Face> part1, GeoPoint point1, GeoPoint touchingPoint, IFrame frame) : base()
        {
            //distanceFromHere = fromHere;
            //distanceToHere = toHere;
            forwardFaces = new HashSet<Face>();
            backwardFaces = new HashSet<Face>(part1);
            shell = backwardFaces.First().Owner as Shell;
            this.point1 = point1;
            this.touchingPoint = touchingPoint;
            // feedbackDimension = FeedbackArrow.MakeLengthArrow(shell, forwardFaces.First(), backwardFaces.First(), null, point2 - point1, touchingPoint, frame.ActiveView, FeedbackArrow.ArrowFlags.secondRed);
            feedback = new Feedback();
        }

        public override string GetID()
        {
            return "Constr.Parametrics.DistanceTo";
        }
        public override bool OnCommand(string MenuId)
        {
            switch (MenuId)
            {
                case "MenuId.Parametrics.DistanceTo":
                    Frame.SetAction(this); // this is the way this action comes to life
                    return true;
            }
            return false;
        }
        public override void OnSetAction()
        {
            base.TitleId = "Constr.Parametrics.DistanceTo";
            if (feedbackDimension != null) feedback.Arrows.AddRange(feedbackDimension);
            if (forwardFaces != null) feedback.FrontFaces.AddRange(forwardFaces);
            if (backwardFaces != null) feedback.BackFaces.AddRange(backwardFaces);
            feedbackResult = shell.Clone() as Shell;
            feedback.ShadowFaces.Add(feedbackResult);
            List<InputObject> actionInputs = new List<InputObject>();

            if (forwardFaces.Count == 0) // we have no other object for distance meassuring yet
            {
                GeoObjectInput toObjectInput = new GeoObjectInput("DistanceTo.ToObject"); // to which object do we calculate the distance of the axis
                toObjectInput.FacesOnly = true;
                toObjectInput.EdgesOnly = true;
                toObjectInput.Points = true;
                toObjectInput.MouseOverGeoObjectsEvent += ToObject_MouseOverGeoObjectsEvent;
                //toObjectInput.GeoObjectSelectionChangedEvent += ToObject_GeoObjectSelectionChangedEvent;
                actionInputs.Add(toObjectInput);
            }

            distance = point1 | point2;
            distanceInput = new LengthInput("DistanceTo.Distance");
            distanceInput.GetLengthEvent += DistanceInput_GetLengthEvent;
            distanceInput.SetLengthEvent += DistanceInput_SetLengthEvent;
            actionInputs.Add(distanceInput);

            forwardFacesInput = new GeoObjectInput("DistanceTo.MoreForwardObjects");
            forwardFacesInput.MultipleInput = true;
            forwardFacesInput.FacesOnly = true;
            forwardFacesInput.Optional = true;
            forwardFacesInput.MouseOverGeoObjectsEvent += new GeoObjectInput.MouseOverGeoObjectsDelegate(OnMouseOverForwardFaces);
            actionInputs.Add(forwardFacesInput);

            backwardFacesInput = new GeoObjectInput("DistanceTo.MoreBackwardObjects");
            backwardFacesInput.MultipleInput = true;
            backwardFacesInput.FacesOnly = true;
            backwardFacesInput.Optional = true;
            backwardFacesInput.MouseOverGeoObjectsEvent += new GeoObjectInput.MouseOverGeoObjectsDelegate(OnMouseOverBackwardFaces);
            actionInputs.Add(backwardFacesInput);

            modeInput = new MultipleChoiceInput("DistanceTo.Mode", "DistanceTo.Mode.Values", 0);
            modeInput.SetChoiceEvent += ModeInput_SetChoiceEvent;
            actionInputs.Add(modeInput);
            //modeInput.GetChoiceEvent += ModeInput_GetChoiceEvent;

            SeparatorInput separator = new SeparatorInput("Parametrics.AssociateParametric");
            actionInputs.Add(separator);
            nameInput = new StringInput("Parametrics.ParametricsName");
            nameInput.SetStringEvent += (string val) => parametricsName = val;
            nameInput.GetStringEvent += () => parametricsName ?? string.Empty;
            nameInput.Optional = true;
            actionInputs.Add(nameInput);

            preserveInput = new BooleanInput("Parametrics.PreserveValue", "YesNo.Values", false);
            actionInputs.Add(preserveInput);

            SetInput(actionInputs.ToArray());
            base.OnSetAction();

            feedback.Attach(CurrentMouseView);
            validResult = false;
        }
        public override void OnViewsChanged()
        {
            feedback.Detach();
            feedback.Attach(CurrentMouseView);
            base.OnViewsChanged();
        }

        private bool OnMouseOverMoreFaces(GeoObjectInput sender, IGeoObject[] theGeoObjects, bool up, HashSet<Face> moreFaces)
        {
            List<Face> faces = new List<Face>();
            foreach (IGeoObject geoObject in theGeoObjects)
            {
                if (geoObject is Face face && !face.Surface.IsExtruded(point2 - point1)) faces.Add(face);
            }
            if (faces.Count > 0)
            {
                if (up)
                {
                    foreach (Face face in faces)
                    {
                        if (moreFaces.Contains(face)) moreFaces.Remove(face); // not sure whethwer to remove
                        else moreFaces.Add(face);
                    }
                    sender.SetGeoObject(moreFaces.ToArray(), null);
                    Refresh();
                }
                return faces.Count > 0;
            }
            return false;

        }
        private bool OnMouseOverBackwardFaces(GeoObjectInput sender, IGeoObject[] theGeoObjects, bool up)
        {
            return OnMouseOverMoreFaces(sender, theGeoObjects, up, backwardFaces);
        }

        private bool OnMouseOverForwardFaces(GeoObjectInput sender, IGeoObject[] theGeoObjects, bool up)
        {
            return OnMouseOverMoreFaces(sender, theGeoObjects, up, forwardFaces);
        }
        private bool DistanceFromAxis(IGeoObject[] geoObjects)
        {
            double distance = 0.0;
            for (int i = 0; i < geoObjects.Length; i++)
            {
                object candidate = geoObjects[i];
                if (geoObjects[i].Owner is Edge edg) candidate = edg;
                if (candidate is Face fc)
                {   // is it possible to specify a distance between this face and the axis?
                    if (fc.Surface is PlaneSurface ps && Precision.IsPerpendicular(ps.Normal, axis.StartDirection, false))
                    {   // we can use the distance to the plane 
                        point1 = axis.PointAt(0.5);
                        point2 = ps.Plane.ToGlobal(ps.Plane.Project(point1));
                        distance = point1 | point2;
                    }
                }
                Line lineToCheck = null;
                if (candidate is Edge edge && edge.Curve3D is Line line)
                {
                    lineToCheck = line;
                }
                if (candidate is Line axl && axl.UserData.Contains("CADability.AxisOf"))
                {
                    lineToCheck = axl;
                }
                if (lineToCheck != null)
                {
                    double dist = Geometry.DistLL(axis.StartPoint, axis.StartDirection, lineToCheck.StartPoint, lineToCheck.StartDirection, out double par1, out double par2);
                    if (par1 != double.MaxValue && dist > 0.0)
                    {
                        point1 = axis.StartPoint + par1 * axis.StartDirection;
                        point2 = lineToCheck.StartPoint + par2 * lineToCheck.StartDirection;
                        distance = point1 | point2;
                    }
                }
                if (distance > 0.0) return true; // we have found a suitable object
            }
            return false;
        }
        private bool ToObject_MouseOverGeoObjectsEvent(GeoObjectInput sender, IGeoObject[] geoObjects, bool up)
        {   // we need to implement more cases here, resulting in faceToMove, faceToKeep (maybe null) and a reference point from where to calculate foot-points for the offset vector

            //Projection.PickArea pa = CurrentMouseView.Projection.GetPickSpace(new Rectangle(sender.currentMousePoint.X - 5, sender.currentMousePoint.Y - 5, 10, 10));
            //if (axis != null && DistanceFromAxis(geoObjects))
            //{
            //    return true;
            //}
            //return false;

            Face found = null;
            GeoPoint sp = GeoPoint.Invalid, ep = GeoPoint.Invalid;
            Face other = backwardFaces.First();
            for (int i = 0; i < geoObjects.Length; i++)
            {
                if (geoObjects[i] is Face fc)
                {
                    if (Surfaces.ParallelDistance(other.Surface, BoundingRect.HalfInfinitBoundingRect, fc.Surface, BoundingRect.HalfInfinitBoundingRect, 
                        touchingPoint, out GeoPoint2D uv1, out GeoPoint2D uv2))
                    {
                        found = fc;
                        sp = other.Surface.PointAt(uv1);
                        ep = fc.Surface.PointAt(uv2);
                        break;
                    }
                }
            }
            if (found != null && up) 
            { 
            }
            return found != null;
        }

        private int ModeInput_GetChoiceEvent()
        {
            return (int)mode;
        }

        private void ModeInput_SetChoiceEvent(int val)
        {
            mode = (Mode)val;
            Refresh();
        }

        public override void OnDone()
        {
            if (validResult && feedbackResult != null)
            {
                Solid sld = shell.Owner as Solid;
                if (sld != null)
                {   // the shell was part of a Solid
                    IGeoObjectOwner owner = sld.Owner; // Model or Block
                    using (Frame.Project.Undo.UndoFrame)
                    {
                        owner.Remove(sld);
                        Solid replacement = Solid.MakeSolid(feedbackResult as Shell);
                        replacement.CopyAttributes(sld);
                        owner.Add(replacement);
                        if (!string.IsNullOrEmpty(parametricsName) && parametricProperty != null)
                        {
                            parametricProperty.Name = parametricsName;
                            parametricProperty.Preserve = preserveInput.Value;
                            replacement.Shells[0].AddParametricProperty(parametricProperty);
                        }
                    }
                }
                else
                {
                    IGeoObjectOwner owner = shell.Owner;
                    using (Frame.Project.Undo.UndoFrame)
                    {
                        owner.Remove(shell);
                        owner.Add(feedbackResult);
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
        public override void OnDisplayChanged(DisplayChangeArg d)
        {
            Refresh(); // the position and size of the arrows should change
            base.OnDisplayChanged(d);
        }
        private bool Refresh()
        {
            validResult = false;
            if (forwardFaces.Count > 0 && backwardFaces.Count > 0)
            {
                FeedBack.ClearSelected();
                GeoPoint startPoint = point1;
                GeoPoint endPoint = point2;
                GeoVector dir = (point2 - point1).Normalized;
                double originalDistance = point2 | point1;
                Shell sh = null;
                Face feedbackFrom = forwardFaces.First(), feedbackTo = backwardFaces.First();
                Dictionary<Face, Face> faceDict = null;

                for (int m = 0; m <= 1; m++)
                {   // first try without moving connected faces, if this yields no result, try with moving connected faced
                    Parametric parametric = new Parametric(shell);
                    Dictionary<Face, GeoVector> allFacesToMove = new Dictionary<Face, GeoVector>();
                    GeoVector offset = (distance - originalDistance) * dir;
                    switch (mode)
                    {
                        case Mode.backward:
                            foreach (Face face in forwardFaces)
                            {
                                allFacesToMove[face] = offset;
                            }
                            foreach (Face face in backwardFaces)
                            {
                                allFacesToMove[face] = GeoVector.NullVector;
                            }
                            break;
                        case Mode.symmetric:
                            foreach (Face face in forwardFaces)
                            {
                                allFacesToMove[face] = 0.5 * offset;
                            }
                            foreach (Face face in backwardFaces)
                            {
                                allFacesToMove[face] = -0.5 * offset;
                            }
                            break;
                        case Mode.forward:
                            foreach (Face face in backwardFaces)
                            {
                                allFacesToMove[face] = -offset;
                            }
                            foreach (Face face in forwardFaces)
                            {
                                allFacesToMove[face] = GeoVector.NullVector;
                            }
                            break;
                    }
                    parametric.MoveFaces(allFacesToMove, offset, m == 1);
                    if (parametric.Apply())
                    {
                        sh = parametric.Result();
                        if (sh != null)
                        {
                            ParametricDistanceProperty.Mode pmode = 0;
                            if (m == 1) pmode |= ParametricDistanceProperty.Mode.connected;
                            if (mode == Mode.symmetric) pmode |= ParametricDistanceProperty.Mode.symmetric;
                            // create the ParametricDistanceProperty here, because here we have all the information
                            parametric.GetDictionaries(out faceDict, out Dictionary<Edge, Edge> edgeDict, out Dictionary<Vertex, Vertex> vertexDict);
                            // facesToKeep etc. contains original objects of the shell, affectedObjects contains objects of the sh = pm.Result()
                            // the parametricProperty will be applied to sh, so we need the objects from sh
                            if (mode == Mode.backward)
                            {
                                parametricProperty = new ParametricDistanceProperty("", Extensions.LookUp(forwardFaces, faceDict),
                                    Extensions.LookUp(backwardFaces, faceDict),
                                    parametric.GetAffectedObjects(), pmode, point2, point1);
                            }
                            else
                            {
                                parametricProperty = new ParametricDistanceProperty("", Extensions.LookUp(backwardFaces, faceDict),
                                    Extensions.LookUp(forwardFaces, faceDict),
                                    parametric.GetAffectedObjects(), pmode, point1, point2);
                            }
                            faceDict.TryGetValue(feedbackFrom, out feedbackFrom);
                            faceDict.TryGetValue(feedbackTo, out feedbackTo);
                            break;
                        }
                    }
                }
                feedback.Clear();
                switch (mode)
                {
                    case Mode.forward:
                        endPoint = point2 + (distance - originalDistance) * dir;
                        feedbackDimension = FeedbackArrow.MakeLengthArrow(shell, feedbackFrom, feedbackTo, null, endPoint - startPoint, touchingPoint, CurrentMouseView, FeedbackArrow.ArrowFlags.secondRed);
                        break;
                    case Mode.backward:
                        startPoint = point1 - (distance - originalDistance) * dir;
                        feedbackDimension = FeedbackArrow.MakeLengthArrow(shell, feedbackFrom, feedbackTo, null, endPoint - startPoint, touchingPoint, CurrentMouseView, FeedbackArrow.ArrowFlags.firstRed );
                        break;
                    case Mode.symmetric:
                        startPoint = point1 - 0.5 * (distance - originalDistance) * dir;
                        endPoint = point2 + 0.5 * (distance - originalDistance) * dir;
                        feedbackDimension = FeedbackArrow.MakeLengthArrow(shell, feedbackFrom, feedbackTo, null, endPoint - startPoint, touchingPoint, CurrentMouseView, FeedbackArrow.ArrowFlags.firstRed | FeedbackArrow.ArrowFlags.secondRed);
                        break;
                }
                if (sh != null)
                {
                    feedbackResult = sh;
                    validResult = true;
                    if (feedbackDimension != null) feedback.Arrows.AddRange(feedbackDimension);
                    if (mode == Mode.backward)
                    {
                        if (forwardFaces != null) feedback.BackFaces.AddRange(Extensions.LookUp(forwardFaces, faceDict));
                        if (backwardFaces != null) feedback.FrontFaces.AddRange(Extensions.LookUp(backwardFaces, faceDict));
                    }
                    else
                    {
                        if (forwardFaces != null) feedback.FrontFaces.AddRange(Extensions.LookUp(forwardFaces, faceDict));
                        if (backwardFaces != null) feedback.BackFaces.AddRange(Extensions.LookUp(backwardFaces, faceDict));
                    }
                    feedback.ShadowFaces.Add(sh);
                    feedback.Refresh();
                    return true;
                }
                else
                {
                    feedbackResult = null;
                    validResult = false;
                    if (feedbackDimension != null) feedback.Arrows.AddRange(feedbackDimension);
                    if (forwardFaces != null) feedback.FrontFaces.AddRange(forwardFaces);
                    if (backwardFaces != null) feedback.BackFaces.AddRange(backwardFaces);
                    feedback.ShadowFaces.Add(shell.Clone());
                    feedback.Refresh();
                    return false;
                }

            }
            return false;

        }
        private bool DistanceInput_SetLengthEvent(double length)
        {
            validResult = false;
            distance = length;
            return Refresh();
            return false;
        }

        private double DistanceInput_GetLengthEvent()
        {
            return distance;
        }

    }
}

