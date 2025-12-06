
using CADability;
using CADability.Actions;
using CADability.GeoObject;
using ExCSS;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using static CADability.Actions.ConstructAction;

namespace ShapeIt
{
    internal class ChamferEdgesAction : ConstructAction
    {
        List<Edge> edges;
        Shell shell;
        LengthInput length1Input;
        LengthInput length2Input;
        double length1, length2;
        private Feedback feedback;
        Shell? result;

        public override string GetID()
        {
            return "Construct.RoundEdges";
        }
        public ChamferEdgesAction(IEnumerable<Edge> edges)
        {
            this.edges = edges.ToList();
            shell = edges.First().Owner.Owner as Shell; // owner of the edge is a face, owner of the face is a shell
            this.edges = new List<Edge>();
            foreach (Edge edge in edges)
            {
                if (edge.Owner.Owner == shell) this.edges.Add(edge); // only edges of a songle shell should be used, all other edges are ignored
            }
        }
        public override void OnSetAction()
        {
            TitleId = "Construct.CamferEdges";

            length1Input = new LengthInput("CamferEdges.Distance1");
            length1Input.SetLengthEvent += Length1Changed;

            length2Input = new LengthInput("CamferEdges.Distance1");
            length2Input.SetLengthEvent += Length2Changed;
            length1 = length2 = 2.0;
            SetInput(length1Input, length2Input);

            base.OnSetAction();
            feedback = new Feedback();
            feedback.Attach(CurrentMouseView);
            base.OnSetAction();
        }
        private bool Length1Changed(double length)
        {
            length1 = length;
            return Recalc();
        }
        private bool Length2Changed(double length)
        {
            length2 = length;
            return Recalc();
        }

        private bool Recalc()
        {
            feedback.Clear();
            
            Dictionary<Edge, Edge> clonedEdges = new Dictionary<Edge, Edge>();
            Shell shellToRound = shell.Clone(clonedEdges);
            IEnumerable<Edge> cledges = clonedEdges.Where(kv => edges.Contains(kv.Key)).Select(kv => kv.Value);
            result = shellToRound.ChamferEdges(cledges, Math.Abs(length1), Math.Abs(length2));
            if (result != null)
            {
                feedback.FrontFaces.Add(shellToRound);
                feedback.Refresh();
                return true;
            }
            else
            {
                feedback.Refresh();
                return false;
            }
        }

        public override void OnDone()
        {
            if (result != null)
            {
                Solid? sld = shell.Owner as Solid;

                using (Frame.Project.Undo.UndoFrame)
                {
                    if (sld != null)
                    {
                        sld.SetShell(result);
                    }
                    else
                    {
                        var owner = shell.Owner;
                        if (owner != null)
                        {
                            owner.Remove(shell);
                            owner.Add(result);
                        }
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
