using CADability;
using CADability.Actions;
using CADability.GeoObject;
using System.Collections.Generic;
using System.Linq;

namespace ShapeIt

{
    /// <summary>
    /// This action modifies two parts of the solid: frontSide and backSide, which are parallel. Typically used to modify the thickness or gauge of a part.
    /// Parametric.OffsetFaces method according to the provided input
    /// </summary>
    public class ParametricsOffsetAction : ConstructAction
    {
        private HashSet<Face> forwardFaces; // the faces, which are beeing moved forward
        private HashSet<Face> backwardFaces; // the faces, which are beeing moved backward
        private IFrame frame;
        private Shell shell;
        private Face touchedFace; // the face which was selected (do we need it?)
        private GeoPoint touchingPoint; // the point where the face was touched

        private LengthInput distanceInput; // input filed of the distance
        private MultipleChoiceInput modeInput; // forward, symmetric, backward
        private Feedback feedback; // to display the feedback
        private enum Mode { forward, symmetric, backward };
        private Mode mode;
        private bool validResult;
        private double gauge; // the original distance of the unmodified shell at the beginning
        private double distance; // the value in the distancInput edit field
        private GeoVector direction; // the direction of the movement
        private Shell feedbackResult; // the shell, which will be the result

        public ParametricsOffsetAction(HashSet<Face> frontSide, HashSet<Face> backSide, IFrame frame, Face touchedFace, GeoPoint touchingPoint, double thickness)
        {
            this.forwardFaces = frontSide;
            this.backwardFaces = backSide;
            this.frame = frame;
            this.touchedFace = touchedFace;
            this.touchingPoint = touchingPoint;
            shell = frontSide.First().Owner as Shell;
            distance = gauge = thickness;
            direction = touchedFace.Surface.GetNormal(touchedFace.Surface.PositionOf(touchingPoint)).Normalized;
            feedback = new Feedback();
        }

        public override string GetID()
        {
            return "Constr.Parametrics.Offset";
        }
        public override void OnSetAction()
        {

            base.TitleId = "Constr.Parametrics.DistanceTo";


            distanceInput = new LengthInput("DistanceTo.Distance");
            distanceInput.SetLengthEvent += (double length) =>
            {
                distance= length;
                return Refresh();
            };
            distanceInput.GetLengthEvent += () => distance;

            modeInput = new MultipleChoiceInput("DistanceTo.Mode", "DistanceTo.Mode.Values", 0);
            modeInput.SetChoiceEvent += (int val) =>
            {
                mode = (Mode)val;
                Refresh();
            };
            modeInput.GetChoiceEvent += () => (int)mode;

            SetInput(distanceInput, modeInput);
            base.OnSetAction();

            feedback.Attach(CurrentMouseView);
            GeoObjectList feedbackDimension = FeedbackArrow.MakeLengthArrow(shell, touchedFace, backwardFaces.First(), touchedFace, direction, touchingPoint, CurrentMouseView, FeedbackArrow.ArrowFlags.secondRed);
            if (feedbackDimension != null) feedback.Arrows.AddRange(feedbackDimension);
            if (forwardFaces != null) feedback.FrontFaces.AddRange(forwardFaces);
            if (backwardFaces != null) feedback.BackFaces.AddRange(backwardFaces);
            //Shell feedbackResult = shell.Clone() as Shell; // don't show the unmodified shell at the beginning
            //feedback.ShadowFaces.Add(feedbackResult);
            feedback.Refresh();

            validResult = false;
        }
        public override void OnViewsChanged()
        {
            feedback.Detach();
            feedback.Attach(CurrentMouseView);
            base.OnViewsChanged();
        }

        private bool Refresh()
        {
            validResult = false;
            if (forwardFaces.Count > 0 && backwardFaces.Count > 0)
            {
                FeedBack.ClearSelected();
                GeoVector dir = direction;
                double originalDistance = gauge;
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
                            //if (mode == Mode.backward)
                            //{
                            //    parametricProperty = new ParametricDistanceProperty("", Extensions.LookUp(forwardFaces, faceDict),
                            //        Extensions.LookUp(backwardFaces, faceDict),
                            //        parametric.GetAffectedObjects(), pmode, point2, point1);
                            //}
                            //else
                            //{
                            //    parametricProperty = new ParametricDistanceProperty("", Extensions.LookUp(backwardFaces, faceDict),
                            //        Extensions.LookUp(forwardFaces, faceDict),
                            //        parametric.GetAffectedObjects(), pmode, point1, point2);
                            //}
                            faceDict.TryGetValue(feedbackFrom, out feedbackFrom);
                            faceDict.TryGetValue(feedbackTo, out feedbackTo);
                            break;
                        }
                    }
                }
                feedback.Clear();
                GeoObjectList feedbackDimension = null;
                switch (mode)
                {
                    case Mode.forward:
                        feedbackDimension = FeedbackArrow.MakeLengthArrow(shell, feedbackFrom, feedbackTo, null, distance*direction, touchingPoint, CurrentMouseView, FeedbackArrow.ArrowFlags.secondRed);
                        break;
                    case Mode.backward:
                        feedbackDimension = FeedbackArrow.MakeLengthArrow(shell, feedbackFrom, feedbackTo, null, distance * direction, touchingPoint, CurrentMouseView, FeedbackArrow.ArrowFlags.firstRed);
                        break;
                    case Mode.symmetric:
                        feedbackDimension = FeedbackArrow.MakeLengthArrow(shell, feedbackFrom, feedbackTo, null, distance * direction, touchingPoint, CurrentMouseView, FeedbackArrow.ArrowFlags.firstRed | FeedbackArrow.ArrowFlags.secondRed);
                        break;
                }
                if (sh != null)
                {
                    validResult = true;
                    feedbackResult = sh;

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

        public override void OnDone()
        {
            if (validResult && feedbackResult!=null)
            {
                Solid sld = shell.Owner as Solid;
                if (sld != null)
                {   // the shell was part of a Solid
                    IGeoObjectOwner owner = sld.Owner; // Model or Block
                    using (Frame.Project.Undo.UndoFrame)
                    {
                        owner.Remove(sld);
                        Solid replacement = Solid.MakeSolid(feedbackResult);
                        replacement.CopyAttributes(sld);
                        owner.Add(replacement);
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
    }
}