using CADability;
using CADability.Actions;
using CADability.GeoObject;
using System.Collections.Generic;
using System.Linq;

namespace ShapeIt
{
    /// <summary>
    /// Interactive shelling of a solid: the user provides the wall thickness and optionally the faces
    /// which are to be removed (the openings). The original faces stay the outer skin.
    /// </summary>
    internal class MakeHollowAction : ConstructAction
    {
        private Shell shell; // the shell to be hollowed out
        private HashSet<Face> openFaces; // the faces of the shell which will be removed
        private double thickness; // the wall thickness
        private List<Shell> result = new List<Shell>(); // the closed hollow shells for the current input
        private bool showResult; // true: show the hollow result, false: show the original with its openings
        private LengthInput thicknessInput;
        private BRepObjectInput openFacesInput;
        private Feedback feedback;

        public override string GetID()
        {
            return "Construct.MakeHollow";
        }
        public MakeHollowAction(Face face)
        {
            shell = face.Owner as Shell;
            openFaces = new HashSet<Face> { face }; // the face the user clicked on is the first opening
        }
        public override void OnSetAction()
        {
            TitleId = "Construct.MakeHollow";

            thicknessInput = new LengthInput("MakeHollow.Thickness");
            thicknessInput.SetLengthEvent += (double length) =>
            {
                thickness = length;
                return Recalc();
            };
            thicknessInput.GetLengthEvent += () => thickness;

            openFacesInput = new BRepObjectInput("MakeHollow.OpenFaces");
            openFacesInput.MultipleInput = true;
            openFacesInput.Optional = true; // without openings the result is a closed body with an enclosed cavity
            openFacesInput.MouseOverBRepObjectsEvent += OnMouseOverFaces;
            openFacesInput.SetBRepObject(openFaces.ToArray(), null);

            SetInput(thicknessInput, openFacesInput);

            // the feedback must exist before base.OnSetAction, which activates the first input and calls InputChanged
            feedback = new Feedback();
            feedback.Attach(CurrentMouseView);
            base.OnSetAction();
        }
        protected override void InputChanged(object activeInput)
        {
            // The hollow body is covered by the original, so it is only visible when the original is hidden.
            // But the openings can only be picked on the visible original.
            showResult = activeInput == thicknessInput;
            RefreshFeedback();
        }
        private bool OnMouseOverFaces(BRepObjectInput sender, object[] bRepObjects, bool up)
        {
            // only faces of the shell to be hollowed out are accepted
            Face face = bRepObjects.OfType<Face>().FirstOrDefault(f => f.Owner == shell);
            if (face == null) return false;
            if (up)
            {   // clicking a face toggles its membership in the openings
                if (!openFaces.Remove(face)) openFaces.Add(face);
                sender.SetBRepObject(openFaces.ToArray(), null);
                Recalc();
            }
            return true;
        }
        private bool Recalc()
        {
            result.Clear();
            if (shell != null && thickness > Precision.eps)
            {
                // The clone has new Face objects, so the open faces have to be mapped onto the clone.
                Dictionary<Face, Face> clonedFaces = new Dictionary<Face, Face>();
                Shell toHollow = shell.Clone(null, null, clonedFaces);
                Shell[] hollowShells = toHollow.MakeHollow(openFaces.Select(f => clonedFaces[f]), thickness);
                result.AddRange(hollowShells.Where(s => s.OpenEdgesExceptPoles.Length == 0));
            }
            RefreshFeedback();
            return result.Count > 0;
        }
        private void RefreshFeedback()
        {
            if (feedback == null) return;
            feedback.Clear();
            if (showResult && result.Count > 0)
            {
                feedback.CreatedObjectsOwnColor = true;
                feedback.CreatedObjects.AddRange(result.ToArray());
                feedback.Hide(shell);
            }
            else
            {
                feedback.Show(shell);
                if (!showResult) feedback.FrontFaces.AddRange(openFaces.ToArray());
            }
            feedback.Refresh();
        }
        public override void OnDone()
        {
            if (result.Count > 0)
            {
                using (Frame.Project.Undo.UndoFrame)
                {
                    if (shell.Owner is Solid sld)
                    {
                        if (result.Count == 1) sld.SetShell(result[0]); // keeps the attributes of the solid
                        else
                        {
                            IGeoObjectOwner owner = sld.Owner;
                            owner.Remove(sld);
                            foreach (Shell sh in result)
                            {
                                Solid hollowSolid = Solid.MakeSolid(sh);
                                hollowSolid.CopyAttributes(sld);
                                owner.Add(hollowSolid);
                            }
                        }
                    }
                    else if (shell.Owner != null)
                    {
                        IGeoObjectOwner owner = shell.Owner;
                        owner.Remove(shell);
                        foreach (Shell sh in result)
                        {
                            sh.CopyAttributes(shell);
                            owner.Add(sh);
                        }
                    }
                }
            }
            base.OnDone();
        }
        public override void OnRemoveAction()
        {
            feedback.Clear();
            feedback.Detach();
            base.OnRemoveAction();
        }
    }
}
