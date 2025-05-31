using CADability;
using CADability.Actions;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ShapeIt
{
    /// <summary>
    /// This action starts with a object to position. This might be one or more faces (like a feature) and a touching point or axis where to meassure.
    /// 
    /// </summary>
    internal class ParametricPositionAction : ConstructAction
    {
        private HashSet<Face> facesToMove; // all the faces to move, currently fixed at construction time
        private object toMeassureTo; // the face or axis to meassure to, may be part of facesToMove
        private GeoPoint touchingPoint; // here toMeassureFrom was touched, e.g. preferred side of cylinder
        private object toMeassureFrom; // an edge vertex or face to meassure to. This stays fixed
        private Feedback feedback; // to display the feedback
        private Shell feedbackResult; // the shell, which will be the result
        private Shell shell; // this is the shell we are manipulating here
        private bool validResult; // is the feedbackResult valid as the result of this action
        double distance; // the current value of the distance
        GeoPoint startPoint, endPoint; // the two points for the dimension arrow

        private BRepObjectInput fromObjectInput; // select the object from which to meassure. This might be a face or and edge (or even a vertex?)
        private LengthInput distanceInput; // the distance value to change
        private StringInput nameInput;
        private string parametricsName;
        private BooleanInput preserveInput;
        private ParametricDistanceProperty parametricProperty;
        public ParametricPositionAction(HashSet<Face> facesToMove, object toMeassureTo, GeoPoint touchingPoint)
        {
            this.facesToMove = facesToMove;
            this.toMeassureTo = toMeassureTo;
            this.touchingPoint = touchingPoint;
            feedback = new Feedback();
            shell = facesToMove.First().Owner as Shell; // this mus exist
        }
        public override string GetID()
        {
            return "Constr.Parametrics.BrepMove";
        }

        public override void OnSetAction()
        {
            base.TitleId = "Constr.Parametrics.BrepMove";
            feedbackResult = shell.Clone() as Shell;
            // feedback.ShadowFaces.Add(feedbackResult); Don't show the whole solid, it is distracting
            feedback.FrontFaces.AddRange(facesToMove);
            List<InputObject> actionInputs = new List<InputObject>();

            fromObjectInput = new BRepObjectInput("DistanceTo.ToObject"); // to which object do we calculate the distance of the facesToMove
            fromObjectInput.MultipleInput = false;
            fromObjectInput.MouseOverBRepObjectsEvent += MouseOverObject;
            actionInputs.Add(fromObjectInput);

            distanceInput = new LengthInput("DistanceTo.Distance");
            distanceInput.GetLengthEvent += () => distance;
            distanceInput.SetLengthEvent += (double val) => { distance = val; return Refresh(); };
            actionInputs.Add(distanceInput);

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
            feedback.Refresh();
            validResult = false;
        }

        private bool Refresh()
        {
            validResult = false;
            if (toMeassureFrom != null && toMeassureTo != null)
            {   // startPoint and endPoint are the unmodified points where the original objects have been meassured
                double startDistance = startPoint | endPoint;
                GeoVector offset = (distance - startDistance) / startDistance * (endPoint - startPoint);
                Dictionary<Face, Face> faceDict = null;
                feedbackResult = null;
                IGeoObject toMeassureToModified = null; ;
                for (int m = 0; m <= 1; m++)
                {   // first try without moving connected faces, if this yields no result, try with moving connected faced
                    Parametric parametric = new Parametric(shell);
                    Dictionary<Face, GeoVector> allFacesToMove = new Dictionary<Face, GeoVector>();
                    foreach (Face face in facesToMove)
                    {
                        allFacesToMove[face] = offset;
                    }
                    parametric.MoveFaces(allFacesToMove, offset, m == 1);
                    if (parametric.Apply())
                    {
                        Shell sh = parametric.Result();
                        if (sh != null)
                        {
                            ParametricDistanceProperty.Mode pmode = 0;
                            if (m == 1) pmode |= ParametricDistanceProperty.Mode.connected;
                            // create the ParametricDistanceProperty here, because here we have all the information
                            parametric.GetDictionaries(out faceDict, out Dictionary<Edge, Edge> edgeDict, out Dictionary<Vertex, Vertex> vertexDict);
                            parametricProperty = new ParametricDistanceProperty("", new Face[] { },
                                    Extensions.LookUp(facesToMove, faceDict),
                                    parametric.GetAffectedObjects(), pmode, startPoint, endPoint);
                            feedbackResult = sh;
                            if (toMeassureTo is Face fc)
                            {
                                if (faceDict.TryGetValue(fc, out Face mfc)) toMeassureToModified = mfc;
                            }
                            else if (toMeassureTo is Edge edg)
                            {
                                if (edgeDict.TryGetValue(edg, out Edge medg)) toMeassureToModified = Helper.ThickLines(medg.Curve3D as IGeoObject);
                            }
                            validResult = true;
                            break;
                        }
                    }
                }
                if (feedbackResult != null)
                {
                    feedback.Clear();
                    feedback.FrontFaces.AddRange(facesToMove);
                    if (toMeassureToModified != null)
                    {
                        feedback.BackFaces.Add(toMeassureToModified);
                        feedback.Arrows.AddRange(FeedbackArrow.MakeLengthArrow(shell, startPoint, endPoint + offset, null, endPoint - startPoint, touchingPoint, CurrentMouseView, FeedbackArrow.ArrowFlags.secondRed));
                    }
                    feedback.Refresh(); // show face or edge
                }
                else
                {
                    feedback.Clear();
                    feedback.FrontFaces.AddRange(facesToMove);
                    IGeoObject go = Helper.ToGeoObject(toMeassureTo);
                    if (go != null)
                    {
                        feedback.BackFaces.Add(Helper.ThickLines(go));
                        feedback.Arrows.AddRange(FeedbackArrow.MakeLengthArrow(shell, startPoint, endPoint, null, endPoint - startPoint, touchingPoint, CurrentMouseView, FeedbackArrow.ArrowFlags.secondRed));
                    }
                    feedback.Refresh(); // show face or edge
                }
            }
            return validResult;
        }

        private bool MouseOverObject(BRepObjectInput sender, object[] bRepObjects, bool up)
        {
            // we need an object with which we can calculate a distance to toMeassureTo
            object found = null;
            int pickRadius = Frame.GetIntSetting("Select.PickRadius", 5); // pickArea to calculate other touchingPoint
            Projection.PickArea pa = CurrentMouseView.Projection.GetPickSpace(new Rectangle(sender.currentMousePoint.X - pickRadius, sender.currentMousePoint.Y - pickRadius, pickRadius * 2, pickRadius * 2));
            GeoPoint sp = GeoPoint.Invalid, ep = GeoPoint.Invalid;
            if (toMeassureTo is Face fc)
            {
                for (int i = 0; i < bRepObjects.Length; i++)
                {
                    if (bRepObjects[i] is Face otherFace)
                    {
                        if (Surfaces.ParallelDistance(fc.Surface, BoundingRect.HalfInfinitBoundingRect, otherFace.Surface, BoundingRect.HalfInfinitBoundingRect,
                            touchingPoint, out GeoPoint2D uv1, out GeoPoint2D uv2))
                        {
                            found = otherFace;
                            ep = fc.Surface.PointAt(uv1);
                            sp = otherFace.Surface.PointAt(uv2);
                            break;
                        }
                    }
                    else if (bRepObjects[i] is Edge otherEdge)
                    {
                        double z = (otherEdge.Curve3D as IGeoObject).Position(pa.FrontCenter, pa.Direction, 0.0);
                        GeoPoint otherTouchingPoint = pa.FrontCenter + z * pa.Direction;
                        if (DistanceCalculator.DistanceBetweenFaceAndEdge(fc, touchingPoint, otherEdge, otherTouchingPoint, out GeoPoint2D onFace, out double onEdge))
                        {
                            found = otherEdge;
                            ep = fc.Surface.PointAt(onFace);
                            sp = otherEdge.Curve3D.PointAt(onEdge);
                            break;
                        }
                    }
                }
            }
            if (found != null && up)
            {
                startPoint = sp;
                endPoint = ep;
                distance = sp | ep;
                distanceInput.ForceValue(distance);
                toMeassureFrom = found;
                Refresh();
            }
            feedback.Clear(); // we do not have a modified shell yet, since we don't have an object to meassure from
            feedback.FrontFaces.AddRange(facesToMove);
            IGeoObject go = Helper.ToGeoObject(found);
            if (go != null)
            {
                feedback.BackFaces.Add(Helper.ThickLines(go));
                feedback.Arrows.AddRange(FeedbackArrow.MakeLengthArrow(shell, sp, ep, null, ep - sp, touchingPoint, CurrentMouseView, FeedbackArrow.ArrowFlags.secondRed));
            }
            feedback.Refresh(); // show face or edge
            return found != null;
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
    }
}
