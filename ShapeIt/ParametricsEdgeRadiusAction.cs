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
    internal class ParametricsEdgeRadiusAction : ConstructAction
    {
        private IEnumerable<Edge> edges; // the edges, which must be circles and have a cylindrical or conical surfaces at least at one face
        private GeoPoint touchingPoint; // the point, where the edge was touched
        private bool useDiameter; // use diameter instead of radius
        private Shell shell; // the shell beeing manipulated by this action
        private double radius;
        private HashSet<Face> affectedFaces;
        private Feedback feedback;
        private Shell feedbackResult;
        private bool validResult;
        private string name;

        private LengthInput radiusInput;
        private StringInput nameInput;
        private BooleanInput preserveInput;

        public override string GetID()
        {
            return "Constr.Parametrics.EdgeRadius";
        }
        public ParametricsEdgeRadiusAction(IEnumerable<Edge> edges, GeoPoint touchingPoint, bool useDiameter)
        {
            this.edges = edges;
            this.touchingPoint = touchingPoint;
            this.useDiameter = useDiameter;
            shell = edges.First().Owner.Owner as Shell;
            radius = (edges.First().Curve3D as Ellipse).Radius;
            affectedFaces = new HashSet<Face>();
            foreach (Edge edge in edges)
            {
                affectedFaces.Add(edge.PrimaryFace);
                affectedFaces.Add(edge.SecondaryFace);
            }
            feedback = new Feedback();
        }
        public override void OnSetAction()
        {
            base.TitleId = "Constr.Parametrics.Extrude";
            feedbackResult = shell.Clone() as Shell;

            List<InputObject> actionInputs = new List<InputObject>();

            string resourceId = useDiameter ? "Edge.Diameter" : "Edge.Radius";
            radiusInput = new LengthInput(resourceId);
            radiusInput.GetLengthEvent += () => useDiameter ? radius * 2 : radius;
            radiusInput.SetLengthEvent += (double val) => { if (useDiameter) radius = val / 2.0; else radius = val; return Refresh(); };
            actionInputs.Add(radiusInput);

            SeparatorInput separator = new SeparatorInput("Parametrics.AssociateParametric");
            actionInputs.Add(separator);
            nameInput = new StringInput("Parametrics.ParametricsName");
            nameInput.SetStringEvent += (string val) => { name = val; };
            nameInput.GetStringEvent += () => name;
            nameInput.Optional = true;
            actionInputs.Add(nameInput);

            preserveInput = new BooleanInput("Parametrics.PreserveValue", "YesNo.Values", false);
            actionInputs.Add(preserveInput);

            SetInput(actionInputs.ToArray());
            base.OnSetAction();

            validResult = false;

            feedback.Attach(CurrentMouseView);
            Refresh();
        }

        bool Refresh()
        {
            feedback.Clear();
            validResult = false;
            Parametric pm = new Parametric(shell);
            if (pm.ModifyEdgeRadius(edges, radius))
            {
                if (pm.Apply())
                {
                    feedbackResult = pm.Result();
                    if (feedbackResult != null)
                    {
                        validResult = true;
                        pm.GetDictionaries(out Dictionary<Face, Face> faceDict, out Dictionary<Edge, Edge> edgeDict, out Dictionary<Vertex, Vertex> vertexDict);
                        HashSet<Edge> modifiedEdges = new HashSet<Edge>(edges.Select((e) => edgeDict[e]));
                        GeoObjectList thickEdges = Helper.ThickCurvesFromEdges(edges);
                        feedback.ShadowFaces.AddRange(thickEdges);
                        HashSet<Face> modifiedFaces = new HashSet<Face>(affectedFaces.Select((f) => faceDict[f]));
                        feedback.FrontFaces.AddRange(modifiedFaces);
                    }
                }
            }
            feedback.Refresh();
            return false;
        }
        public override void OnDone()
        {
            if (validResult && feedbackResult != null)
            {
                string parametricsName = nameInput.Content;
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
                        // here parametricProperty is already consistant with the BRep objects of feedbackResult
                        //if (!string.IsNullOrEmpty(parametricsName) && parametricProperty != null)
                        //{
                        //    parametricProperty.Name = parametricsName;
                        //    parametricProperty.Preserve = preserveInput.Value;
                        //    replacement.Shells[0].AddParametricProperty(parametricProperty);
                        //}
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
            feedbackResult = null;
            base.OnDone();
        }
        public override void OnRemoveAction()
        {
            feedback.Detach();
            base.OnRemoveAction();
        }

    }
}
