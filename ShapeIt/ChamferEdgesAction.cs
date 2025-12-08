
using CADability;
using CADability.Actions;
using CADability.Attribute;
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
        BooleanInput sameLengthInput;
        double length1, length2;
        private Feedback feedback;
        Shell? result;
        Layer transparent;
        Layer original;

        public override string GetID()
        {
            return "Construct.RoundEdges";
        }
        public ChamferEdgesAction(IEnumerable<Edge> edges)
        {
            this.edges = edges.ToList();
            if (!(edges.First().Owner.Owner is Shell s)) throw new ApplicationException("ChamferEdgesAction: edges are not part of a shell");
            shell = s; // owner of the edge is a face, owner of the face is a shell
            this.edges = new List<Edge>();
            foreach (Edge edge in edges)
            {
                if (edge.Owner.Owner == shell) this.edges.Add(edge); // only edges of a songle shell should be used, all other edges are ignored
            }
            feedback = new Feedback();
            transparent = new Layer("TransparentForAction");
            transparent.Transparency = 128;
            original = shell.Layer;
        }
        public override void OnSetAction()
        {
            TitleId = "Construct.ChamferEdges";

            sameLengthInput = new BooleanInput("ChamferEdges.Symmetric", "ChamferEdges.Symmetric.Values");
            sameLengthInput.SetBooleanEvent += SameLengthChanged;
            sameLengthInput.Optional = true;

            length1Input = new LengthInput("ChamferEdges.Distance1");
            length1Input.SetLengthEvent += Length1Changed;
            length1Input.GetLengthEvent += GetLength1;

            length2Input = new LengthInput("ChamferEdges.Distance2");
            length2Input.SetLengthEvent += Length2Changed;
            length2Input.Optional = true;
            length2Input.GetLengthEvent += GetLength2;

            length1 = length2 = 2.0;
            SetInput(sameLengthInput, length1Input, length2Input);

            base.OnSetAction();
            feedback.Attach(CurrentMouseView);
            shell.Layer = transparent;
            sameLengthInput.ForceValue(true);
            length2Input.Optional = true;
            length2Input.ReadOnly = true;
        }

        private double GetLength1()
        {
            return length1;
        }
        private double GetLength2()
        {
            return length2;
        }

        private void SameLengthChanged(bool val)
        {
            length2Input.Optional = val;
            length2Input.ReadOnly = val;
        }

        private bool Length1Changed(double length)
        {
            length1 = length;
            if (sameLengthInput.Value) length2 = length1;
            return Recalc();
        }
        private bool Length2Changed(double length)
        {
            length2 = length;
            return Recalc();
        }

        private bool Recalc()
        {
            shell.Layer = transparent;
            CurrentMouseView.InvalidateAll();
            feedback.Clear();

            Dictionary<Edge, Edge> clonedEdges = new Dictionary<Edge, Edge>();
            Shell shellToRound = shell.Clone(clonedEdges);
            shellToRound.Layer = original;
            IEnumerable<Edge> cledges = clonedEdges.Where(kv => edges.Contains(kv.Key)).Select(kv => kv.Value);
            result = shellToRound.ChamferEdges(cledges, Math.Abs(length1), Math.Abs(length2));
            if (result != null)
            {
                feedback.FrontFaces.Add(result);
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
            shell.Layer = original;
            feedback.Detach();
            base.OnRemoveAction();
            CurrentMouseView.InvalidateAll();
        }
    }
}
