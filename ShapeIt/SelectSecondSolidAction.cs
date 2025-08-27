using CADability;
using CADability.GeoObject;
using CADability.Actions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static CADability.BRepOperation;

namespace ShapeIt
{
    internal class SelectSecondSolidAction : ConstructAction
    {
        private Solid firstSolid;
        private Solid secondSolid;
        private BRepOperation.Operation operation;
        private GeoObjectInput selectSolid;
        public override string GetID()
        {
            return "Parametrics.SecondSolid";
        }

        public SelectSecondSolidAction(Solid firstSolid, BRepOperation.Operation operation)
        {
            this.firstSolid = firstSolid;
            this.operation = operation;
            secondSolid = null;
        }

        public override void OnSetAction()
        {
            switch (operation)
            {
                case Operation.union:
                    TitleId = "SelectSolid.Union";
                    break;
                case Operation.intersection:
                    TitleId = "SelectSolid.Intersection";
                    break;
                case Operation.difference:
                    TitleId = "SelectSolid.Difference";
                    break;
                case Operation.clip: // misused for split here
                    TitleId = "SelectSolid.Split";
                    break;
            }

            List<InputObject> actionInputs = new List<InputObject>();

            selectSolid = new GeoObjectInput("SelectSolid.Solid");
            selectSolid.MultipleInput = false;
            selectSolid.FacesOnly = false;
            selectSolid.Optional = false;
            selectSolid.MouseOverGeoObjectsEvent += new GeoObjectInput.MouseOverGeoObjectsDelegate(OnMouseOverSolid);
            actionInputs.Add(selectSolid);

            SetInput(actionInputs.ToArray());

            base.OnSetAction();
        }

        private bool OnMouseOverSolid(GeoObjectInput sender, IGeoObject[] TheGeoObjects, bool up)
        {
            Solid found = null;
            for (int i = 0; i < TheGeoObjects.Length; i++)
            {
                if (TheGeoObjects[i] is Solid sld && sld != firstSolid)
                {
                    found = sld;
                    break;
                }
            }
            if (up && found != null)
            {
                sender.SetGeoObject(new IGeoObject[] { found }, found);
            }
            return found != null;
        }
        public override void OnDone()
        {
            if (selectSolid.GetGeoObjects()!=null && selectSolid.GetGeoObjects()[0] is Solid sld) { secondSolid = sld; }
            if (secondSolid != null)
            {
                switch (operation)
                {
                    case Operation.union:
                        Solid res = Solid.Unite(firstSolid, secondSolid);
                        if (res != null)
                        {
                            using (Frame.Project.Undo.UndoFrame)
                            {
                                IGeoObjectOwner owner = firstSolid.Owner;
                                firstSolid.Owner.Remove(firstSolid);
                                secondSolid.Owner.Remove(secondSolid);
                                owner.Add(res);
                            }
                        }
                        break;
                    case Operation.intersection:
                        Solid[] intersections = Solid.Intersect(firstSolid, secondSolid);
                        if (intersections != null && intersections.Length > 0)
                        {
                            using (Frame.Project.Undo.UndoFrame)
                            {
                                IGeoObjectOwner owner = firstSolid.Owner;
                                firstSolid.Owner.Remove(firstSolid);
                                secondSolid.Owner.Remove(secondSolid);
                                for (int i = 0; i < intersections.Length; i++)
                                    owner.Add(intersections[i]);
                            }
                        }
                        break;
                    case Operation.difference:
                        Solid[] difference = Solid.Subtract(firstSolid, secondSolid);
                        if (difference != null && difference.Length > 0)
                        {
                            using (Frame.Project.Undo.UndoFrame)
                            {
                                IGeoObjectOwner owner = firstSolid.Owner;
                                firstSolid.Owner.Remove(firstSolid);
                                secondSolid.Owner.Remove(secondSolid);
                                for (int i = 0; i < difference.Length; i++)
                                    owner.Add(difference[i]);
                            }
                        }
                        break;
                    case Operation.clip: // misused for splitting here
                        List<Solid> fragments = new List<Solid>();
                        Solid[] res1 = Solid.Subtract(firstSolid, secondSolid);
                        Solid[] res2 = Solid.Subtract(secondSolid, firstSolid);
                        Solid[] res3 = Solid.Intersect(firstSolid, secondSolid);
                        if (res1 != null && res1.Length > 0 && res2 != null && res2.Length > 0)
                        {
                            if (res1 != null) fragments.AddRange(res1);
                            if (res2 != null) fragments.AddRange(res2);
                            if (res3 != null) fragments.AddRange(res3);
                        }
                        else
                        {
                            fragments.Add(firstSolid);
                        }
                        if (fragments != null && fragments.Count > 0)
                        {
                            using (Frame.Project.Undo.UndoFrame)
                            {
                                IGeoObjectOwner owner = firstSolid.Owner;
                                firstSolid.Owner.Remove(firstSolid);
                                secondSolid.Owner.Remove(secondSolid);
                                for (int i = 0; i < fragments.Count; i++)
                                    owner.Add(fragments[i]);
                            }
                        }
                        break;
                }
            }
            base.OnDone();
        }
    }
}
