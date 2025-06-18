using CADability;
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
    internal class OffsetSolidAction : ConstructAction
    {
        CancellationTokenSource _cts = null; // not yet implemented
        Solid solid;
        LengthInput distanceInput;
        private Feedback feedback;

        Shell[] result = null;

        public override string GetID()
        {
            return "Construct.OffsetSolid";
        }

        public OffsetSolidAction(Solid solid)
        {
            this.solid = solid;
        }
        public override void OnSetAction()
        {
            TitleId = "Construct.OffsetSolid";

            distanceInput = new LengthInput("OffsetSolid.Distance");
            distanceInput.SetLengthEvent += DistanceChanged;

            SetInput(distanceInput);

            base.OnSetAction();
            feedback = new Feedback();
            feedback.Attach(CurrentMouseView);

        }

        private bool DistanceChanged(double length)
        {
            _cts?.Cancel();
            _cts?.Dispose();

            return Recalc(length);
        }

        private bool Recalc(double length)
        {
            feedback.Clear();
            Shell shell = solid.Shells[0].Clone() as Shell;
            result = ShellExtensions.GetOffsetNew(shell, length);
            if (result.Length > 0)
            {
                for (int i = 0; i < result.Length; i++)
                {
                    feedback.FrontFaces.Add(result[i]);
                }
                feedback.Refresh();
                return true;
            }
            feedback.Refresh();
            return false;
        }

        public override void OnDone()
        {
            if (result?.Length > 0)
            {
                IGeoObjectOwner owner = solid.Owner; // usually the model
                using (Frame.Project.Undo.UndoFrame)
                {
                    owner.Remove(solid);
                    for (int i = 0; i < result.Length; i++)
                    {
                        Solid sld = Solid.Construct();
                        sld.SetShell(result[i]);
                        sld.CopyAttributes(solid);
                        owner.Add(sld);
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
