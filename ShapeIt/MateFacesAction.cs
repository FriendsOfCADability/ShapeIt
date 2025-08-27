using CADability;
using CADability.Actions;
using CADability.GeoObject;
using System;
using System.CodeDom;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ShapeIt
{
    internal class MateFacesAction : ConstructAction
    {
        private Face toMove; // the Face to be modified
        private Face targetFace; // the face to be matched
        private bool matchFromOutside = true;
        private Shell toModify; // for feedback (clone it!)
        private bool copyObject; // modify the shell or solid or make a copy

        private BRepObjectInput targetFaceinput; // define the face where to move
        private MultipleChoiceInput modeInput; // from inside or outside
        private Feedback feedback;

        public override string GetID()
        {
            return "Construct.MateFaces";
        }

        public MateFacesAction(Face toMove)
        {
            this.toMove = toMove;
            toModify = toMove.Owner as Shell;
        }

        public override void OnSetAction()
        {
            TitleId = "Construct.MateFaces";

            copyObject = ConstrDefaults.DefaultCopyObjects;

            targetFaceinput = new BRepObjectInput("Mate.Target");
            targetFaceinput.MultipleInput = false;
            targetFaceinput.MouseOverBRepObjectsEvent += (BRepObjectInput sender, object[] bRepObjects, bool up) =>
            {
                Face foundTarget = null;
                for (int i = 0; i < bRepObjects.Length; i++)
                {
                    if (bRepObjects[i] is Face face)
                    {
                        if (face.Owner != toMove.Owner) // must be of different shells
                        {
                            foundTarget = face;
                        }
                    }
                }
                if (foundTarget != null)
                {
                    RefreshFeedback(Recalc(foundTarget), foundTarget);
                }
                if (up && foundTarget != null)
                {
                    targetFace = foundTarget;
                    sender.SetBRepObject(new object[] { targetFace }, targetFace);
                }
                return foundTarget != null;
            };

            modeInput = new MultipleChoiceInput("Mate.Mode", "Mate.Mode.Values");
            modeInput.SetChoiceEvent += (int val) =>
            {
                matchFromOutside = val == 1;
                if (targetFace != null)
                {
                    RefreshFeedback(Recalc(targetFace), targetFace);
                }
            };
            modeInput.GetChoiceEvent += () =>
            {
                return matchFromOutside ? 1 : 0;
            };

            BooleanInput copy = new BooleanInput("Modify.CopyObjects", "YesNo.Values");
            copy.DefaultBoolean = ConstrDefaults.DefaultCopyObjects;
            copy.SetBooleanEvent += (val) => { copyObject = val; };
            copy.GetBooleanEvent += () => { return copyObject; };

            // maybe some matching point input

            SetInput(targetFaceinput, modeInput, copy);

            base.OnSetAction();
            feedback = new Feedback();
            feedback.Attach(CurrentMouseView);
        }

        public override void OnRemoveAction()
        {
            feedback.Detach();
            base.OnRemoveAction();
        }
        private ModOp Recalc(Face targetFace)
        {
            if (targetFace != null)
            {
                GeoPoint2D tcnt2d = targetFace.Area.GetExtent().GetCenter();
                GeoPoint tcnt = targetFace.Surface.PointAt(tcnt2d);
                GeoPoint2D scnt2d = toMove.Area.GetExtent().GetCenter();
                GeoPoint scnt = toMove.Surface.PointAt(scnt2d);
                GeoVector tnormal = targetFace.Surface.GetNormal(tcnt2d);
                GeoVector snormal = toMove.Surface.GetNormal(scnt2d);
                // we have the normals and point of the source and target faces
                // when the normals are parallel and same direction, we only have to move
                // when they are parallel and opposite direction, it is ambiguous how to rotat the sours object. We use the z axis if possible else the x-axis
                // when they are not parallel, the rotation axis is the cross product
                ModOp rotate;
                if ((tnormal ^ snormal).Length < Precision.eps) // Nullvector check is too strong
                {   // same or opposite direction
                    if ((tnormal * snormal < 0) == !matchFromOutside)
                    { // opposite direction
                        GeoVector rotationAxis;
                        if (Precision.SameDirection(tnormal, GeoVector.ZAxis, false)) rotationAxis = GeoVector.XAxis;
                        else rotationAxis = GeoVector.ZAxis;
                        rotate = ModOp.Rotate(scnt, rotationAxis, SweepAngle.Opposite);
                    }
                    else
                    { // same direction
                        rotate = ModOp.Identity; // no rotation in this case
                    }
                }
                else
                {
                    if (matchFromOutside) rotate = ModOp.Rotate(scnt, snormal, -tnormal);
                    else rotate = ModOp.Rotate(scnt, snormal, tnormal);
                }
                ModOp move = ModOp.Translate(tcnt - scnt);
                ModOp modify = move * rotate;
                return modify;
            }
            return ModOp.Null;
        }

        private void RefreshFeedback(ModOp modify, Face targetFace)
        {
            Shell fb = toModify.Clone() as Shell;
            fb.Modify(modify);
            feedback.Clear();
            feedback.FrontFaces.Add(toMove);
            if (targetFace != null) feedback.BackFaces.Add(targetFace);
            feedback.ShadowFaces.Add(fb);
            feedback.Refresh();

        }
        public override void OnDone()
        {
            if (targetFace != null)
            {
                ModOp modify = Recalc(targetFace);
                if (!modify.IsNull)
                {
                    Solid sld = toModify.Owner as Solid;
                    if (sld != null)
                    {   // the shell was part of a Solid
                        IGeoObjectOwner owner = sld.Owner; // Model or Block
                        using (Frame.Project.Undo.UndoFrame)
                        {
                            if (copyObject)
                            {
                                Solid cln = sld.Clone() as Solid;
                                cln.Modify(modify);
                                owner.Add(cln);
                            }
                            else
                            {
                                sld.Modify(modify);
                            }
                        }
                    }
                    else
                    {   // a shell not part of a solid
                        IGeoObjectOwner owner = (toMove.Owner as Shell).Owner;
                        using (Frame.Project.Undo.UndoFrame)
                        {
                            if (copyObject)
                            {
                                Shell cln = toModify.Clone() as Shell;
                                cln.Modify(modify);
                                owner.Add(cln);
                            }
                            else
                            {
                                toModify.Modify(modify);
                            }
                        }
                    }
                }
            }
            base.OnDone();
        }
    }
}
