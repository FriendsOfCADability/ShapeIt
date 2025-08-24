﻿using CADability;
using CADability.Actions;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace ShapeIt
{
    internal class RoundEdgesAction : ConstructAction
    {
        List<Edge> edges;
        Shell shell;
        LengthInput radiusInput;
        private Feedback feedback;
        Shell result;

        public override string GetID()
        {
            return "Construct.RoundEdges";
        }
        public RoundEdgesAction(IEnumerable<Edge> edges)
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
            TitleId = "Construct.OffsetSolid";

            radiusInput = new LengthInput("OffsetSolid.Distance");
            radiusInput.SetLengthEvent += RadiusChanged;

            SetInput(radiusInput);

            base.OnSetAction();
            feedback = new Feedback();
            feedback.Attach(CurrentMouseView);
            base.OnSetAction();
        }
        private bool RadiusChanged(double length)
        {
            //_cts?.Cancel();
            //_cts?.Dispose();

            return Recalc(length);
        }

        private bool Recalc(double length)
        {
            feedback.Clear();
            Dictionary<Edge, Edge> clonedEdges = new Dictionary<Edge, Edge>();
            Shell shellToRound = shell.Clone(clonedEdges);
            IEnumerable<Edge> cledges = clonedEdges.Where(kv => edges.Contains(kv.Key)).Select(kv => kv.Value);
            result = shellToRound.RoundEdges(cledges, Math.Abs(length));
            if (result!=null)
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
                Solid sld = shell.Owner as Solid;

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
