using CADability;
using CADability.Actions;
using CADability.GeoObject;
using CADability.UserInterface;
using MathNet.Numerics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ShapeIt
{
    /// <summary>
    /// Action to modify the angle between faces.
    /// The angle between two faces is determined depending on the surfaces:
    /// For two planes, the axis is the intersection line, the angle is the angle between the normals.
    /// For objects with an axis (cylinder cone, torus?) the axis is the cross product of the two directions, or the shortest line between these axis.
    /// When parallel, it is the normal of the common plane somewhere at the extent of the faces.
    /// For axis object and plane this is ambiguous, when the axis is perpendicular to the plane
    /// </summary>
    public class ParametricsAngleAction : ConstructAction
    {
        Shell shell;
        List<Face> forwardFaces; // all faces in the forward direction of the rotation
        List<Face> backwardFaces; // all faces in the backward direction of the rotation
        Face primaryRotationFace; // the face which is beeing rotated and which specifies the angle
        Face referenceFace; // the face to which the angle is meassured
        Axis rotationAxis; // the axis of rotation. The location is close to the mouse position of the click point
        GeoVector fromHere; // Reference vector for the start of the rotation, perpendicular to the rotationAxis
        GeoVector toHereStartValue; // start value of the above
        GeoVector toHere; // the vector to be modified, perpendicular to the rotationAxis
        Edge axisEdge = null; // the edge, which acts as the axis
        Feedback feedback;
        GeoObjectList feedbackDimension;
        private Shell feedbackResult; // the shell, which will be the result
        private bool resultIsValid = false;

        string parametricsName; // the name for the parametrics in the shall, if provided
        Parametric parametric;

        GeoObjectInput facetoRotate;
        GeoObjectInput moreFacestoRotate;
        GeoObjectInput reference;
        AngleInput rotationAngle;
        StringInput nameInput;
        BooleanInput preserveInput;

        public ParametricsAngleAction(Face faceToRotate, Face referenceFace, Axis axis, GeoVector fromHere, GeoVector toHere, Edge axisEdge, IFrame frame)
        {   // there will be other constructors, e.g. for a cylinder to plane angle etc.
            shell = faceToRotate.Owner as Shell;
            if (shell == null) throw new ApplicationException("faceToRotate must be part of a shell");
            forwardFaces = new List<Face> { faceToRotate };
            backwardFaces = new List<Face> { referenceFace };
            rotationAxis = axis;
            this.fromHere = fromHere;
            this.axisEdge = axisEdge;
            toHereStartValue = this.toHere = toHere;
            feedbackDimension = FeedbackArrow.MakeArcArrow(shell, rotationAxis, fromHere, toHere, frame.ActiveView, FeedbackArrow.ArrowFlags.secondRed);
            feedback = new Feedback();
        }
        /// <summary>
        /// Calculates a rotation axis between the face <paramref name="toRotate"/> and the <paramref name="referenceFace"/>. The returned  axis will be 
        /// close to the <paramref name="pickBeam"/>. The returned vectors fromHere and toHere are the current position of the enclosing angle.
        /// If <paramref name="axisEdge"/> is provided, this edge is the axis and the location is at the center of this edge
        /// </summary>
        /// <param name="toRotate"></param>
        /// <param name="referenceFace"></param>
        /// <param name="pickBeam"></param>
        /// <returns></returns>
        static public (Axis axis, GeoVector fromHere, GeoVector toHere) GetRotationAxis(Face toRotate, Face referenceFace, Axis pickBeam, Edge axisEdge = null)
        {
            Line rotationLine = null;
            GeoPoint rotationPoint = GeoPoint.Origin;
            Edge edge = null;
            if (axisEdge != null && axisEdge.Curve3D is Line axline)
            {
                rotationLine = axline;
                rotationPoint = axline.PointAt(0.5);
                edge = axisEdge;
            }
            else
            {
                List<Edge> commonEdges = new List<Edge>(toRotate.AllEdges.Intersect(referenceFace.AllEdges));
                // Implementation for lines:
                double minDist = double.MaxValue;
                for (int i = 0; i < commonEdges.Count; ++i)
                {
                    if (commonEdges[i].Curve3D is Line line && !commonEdges[i].IsTangentialEdge())
                    {
                        Geometry.ConnectLines(line.StartPoint, line.StartDirection, pickBeam.Location, pickBeam.Direction, out double pos1, out double pos2);
                        GeoPoint p1 = line.StartPoint + pos1 * line.StartDirection;
                        GeoPoint p2 = pickBeam.Location + pos2 * pickBeam.Direction;
                        double d = p1 | p2;
                        if (d < minDist)
                        {
                            minDist = d;
                            rotationLine = line;
                            pos1 = Math.Max(Math.Min(pos1, 1.0), 0.0);
                            rotationPoint = line.StartPoint + pos1 * line.StartDirection; // on the line, which is the edge
                            edge = commonEdges[i];
                        }
                    }
                }
            }
            if (rotationLine != null)
            {
                // find the direction from where to go and the direction to where to go
                GeoVector fromHere = referenceFace.Surface.GetNormal(referenceFace.Surface.PositionOf(rotationPoint)) ^ rotationLine.StartDirection;
                if (!edge.Forward(referenceFace)) fromHere = -fromHere;
                GeoVector toHere = toRotate.Surface.GetNormal(toRotate.Surface.PositionOf(rotationPoint)) ^ rotationLine.StartDirection;
                if (!edge.Forward(toRotate)) toHere = -toHere;
                return (new Axis(rotationPoint, rotationLine.StartDirection), fromHere, toHere);
            }
            return (Axis.InvalidAxis, GeoVector.NullVector, GeoVector.NullVector);
        }
        public override string GetID()
        {
            return "MenuId.Parametrics.Angle";
        }
        public override void OnSetAction()
        {
            base.TitleId = "Constr.Parametrics.Angle";
            if (feedbackDimension != null) feedback.Arrows.AddRange(feedbackDimension);
            if (forwardFaces != null) feedback.FrontFaces.AddRange(forwardFaces);
            if (backwardFaces != null) feedback.BackFaces.AddRange(backwardFaces);
            feedbackResult = shell.Clone() as Shell;
            feedback.ShadowFaces.Add(feedbackResult);

            List<InputObject> actionInputs = new List<InputObject>();

            facetoRotate = new GeoObjectInput("Parametrics.Angle.FaceToRotate");
            facetoRotate.MultipleInput = false;
            facetoRotate.FacesOnly = true;
            facetoRotate.Optional = false;
            facetoRotate.MouseOverGeoObjectsEvent += new GeoObjectInput.MouseOverGeoObjectsDelegate(OnMouseOverFaceToRotate);
            actionInputs.Add(facetoRotate);

            moreFacestoRotate = new GeoObjectInput("Parametrics.Angle.MoreFacesToRotate");
            moreFacestoRotate.MultipleInput = true;
            moreFacestoRotate.FacesOnly = true;
            moreFacestoRotate.Optional = true;
            moreFacestoRotate.MouseOverGeoObjectsEvent += new GeoObjectInput.MouseOverGeoObjectsDelegate(OnMouseOverMoreFacesToRotate);
            actionInputs.Add(moreFacestoRotate);

            reference = new GeoObjectInput("Parametrics.Angle.Reference");
            reference.MultipleInput = false;
            reference.FacesOnly = true;
            reference.Optional = true;
            reference.MouseOverGeoObjectsEvent += new GeoObjectInput.MouseOverGeoObjectsDelegate(OnMouseOverReferenceFace);
            actionInputs.Add(reference);

            rotationAngle = new AngleInput("Parametrics.Angle.Angle");
            rotationAngle.GetAngleEvent += GetRotationAngle;
            rotationAngle.SetAngleEvent += SetRotationAngle;
            actionInputs.Add(rotationAngle);

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


            facetoRotate.SetGeoObject(forwardFaces.ToArray(), forwardFaces[0]);
            facetoRotate.Fixed = forwardFaces.Count > 0;
            reference.SetGeoObject(backwardFaces.ToArray(), backwardFaces[0]);
            reference.Fixed = backwardFaces.Count > 0;
        }
        public override void OnViewsChanged()
        {
            feedback.Detach();
            feedback.Attach(CurrentMouseView);
            base.OnViewsChanged();
        }
        public override void OnRemoveAction()
        {
            feedback.Detach();
            base.OnRemoveAction();
        }
        private bool SetRotationAngle(Angle angle)
        {
            SweepAngle startingAngle = new SweepAngle(fromHere, toHereStartValue);
            double da = angle - startingAngle;
            // this is the delta angle, test in which direction
            ModOp rot1 = ModOp.Rotate(rotationAxis.Location, rotationAxis.Direction, da);
            double a1 = new SweepAngle(fromHere, rot1 * toHereStartValue);
            ModOp rot2 = ModOp.Rotate(rotationAxis.Location, rotationAxis.Direction, -da);
            double a2 = new SweepAngle(fromHere, rot2 * toHereStartValue);
            if (Math.Abs(a2-angle)<Math.Abs(a1-angle))
            {
                rot1 = rot2;
            }
            toHere = rot1 * toHereStartValue;
            Refresh();
            return true;
        }
        private Angle GetRotationAngle()
        {
            return new Angle(fromHere, toHere); // should be between -180° and +180°
        }
        private bool Refresh()
        {
            feedback.Clear();
            ModOp rotation = ModOp.Rotate(rotationAxis.Location, rotationAxis.Direction, new SweepAngle(toHereStartValue, toHere)); // the delta rotation
            if (!Precision.SameDirection(rotation*toHereStartValue,toHere, false)) rotation = ModOp.Rotate(rotationAxis.Location, -rotationAxis.Direction, new SweepAngle(toHereStartValue, toHere)); // the delta rotation
            GeoVector debug = rotation * toHereStartValue;
            feedbackDimension = FeedbackArrow.MakeArcArrow(shell, rotationAxis, fromHere, toHere, CurrentMouseView, FeedbackArrow.ArrowFlags.secondRed);
            if (feedbackDimension != null) feedback.Arrows.AddRange(feedbackDimension);
            Dictionary<Face, ModOp> rotatingFaces = new Dictionary<Face, ModOp>();
            foreach (Face face in forwardFaces) rotatingFaces[face] = rotation;
            parametric = new Parametric(shell);
            parametric.RotateFaces(rotatingFaces, rotationAxis, false);
            if (parametric.Apply())
            {
                Shell sh = parametric.Result();
                if (sh != null)
                {
                    feedbackResult = sh;
                    parametric.GetDictionaries(out Dictionary<Face, Face> faceDict, out Dictionary<Edge, Edge> edgeDict, out Dictionary<Vertex, Vertex> vertexDict);
                    if (forwardFaces != null) feedback.FrontFaces.AddRange(Extensions.LookUp(forwardFaces,faceDict));
                    if (backwardFaces != null) feedback.BackFaces.AddRange(Extensions.LookUp(backwardFaces, faceDict));
                    feedback.ShadowFaces.Add(feedbackResult);
                    feedback.Refresh();
                    resultIsValid = true;
                    return true;
                }
            }
            feedbackResult = shell.Clone() as Shell;
            if (forwardFaces != null) feedback.FrontFaces.AddRange(forwardFaces);
            if (backwardFaces != null) feedback.BackFaces.AddRange(backwardFaces);
            feedback.ShadowFaces.Add(feedbackResult);
            resultIsValid = false;
            return false;
        }
        private bool OnMouseOverFaceToRotate(GeoObjectInput sender, IGeoObject[] geoObjects, bool up)
        {
            List<Face> faces = geoObjects.OfType<Face>().ToList();
            if (faces.Count > 0)
            {
                if (up)
                {
                    primaryRotationFace = faces[0];
                    sender.SetGeoObject(new IGeoObject[] { primaryRotationFace }, primaryRotationFace);
                    Refresh();
                }
            }
            return faces.Count > 0;
        }
        private bool OnMouseOverMoreFacesToRotate(GeoObjectInput sender, IGeoObject[] geoObjects, bool up)
        {
            List<Face> faces = geoObjects.OfType<Face>().ToList();
            if (faces.Count > 0)
            {
                if (up)
                {
                    HashSet<Face> current = new HashSet<Face>(sender.GetGeoObjects().OfType<Face>());
                    foreach (Face face in faces)
                    {
                        if (current.Contains(face)) current.Remove(face);
                        else current.Add(face);
                        sender.SetGeoObject(current.ToArray(), null);
                        forwardFaces = new List<Face>(current);
                    }
                    Refresh();
                }
            }
            return faces.Count > 0;
        }
        private bool OnMouseOverReferenceFace(GeoObjectInput sender, IGeoObject[] geoObjects, bool up)
        {
            List<Face> faces = geoObjects.OfType<Face>().ToList();
            if (faces.Count > 0)
            {
                if (up)
                {
                    referenceFace = faces[0];
                    sender.SetGeoObject(new IGeoObject[] { referenceFace }, referenceFace);
                    Refresh();
                }
            }
            return faces.Count > 0;
        }
        private string NameInput_GetStringEvent()
        {
            if (parametricsName == null) return string.Empty;
            else return parametricsName;
        }
        private void NameInput_SetStringEvent(string val)
        {
            parametricsName = val;
        }
        public override void OnDone()
        {
            if (Refresh() && feedbackResult != null)
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
                        if (!string.IsNullOrEmpty(parametricsName))
                        {
                            ParametricRotationProperty parametricProperty = new ParametricRotationProperty(parametricsName, backwardFaces, forwardFaces, parametric.GetAffectedObjects(), 0, axisEdge);
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
