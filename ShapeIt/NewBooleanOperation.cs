using CADability;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeIt
{
    /// <summary>
    /// Will be moved to CADability to replace the Solid static methods
    /// </summary>
    internal class NewBooleanOperation
    {
        static public Solid Unite(Solid solid1, Solid solid2)
        {
            BooleanOperation bo = new BooleanOperation();
            bo.SetShells(solid1.Shells[0], solid2.Shells[0], BooleanOperation.Operation.union);
            Shell[] res = bo.Execute();
            if (res.Length == 1)
            {
                return Solid.MakeSolid(res[0]);
            }
            else
            {
                return null;
            }
        }
        static public Solid[] Subtract(Solid first, Solid second)
        {
            BooleanOperation bo = new BooleanOperation();
            // bo.DontCombineConnectedFaces = true; // performance test!
            bo.SetShells(first.Shells[0], second.Shells[0], BooleanOperation.Operation.difference);
            Shell[] res = bo.Execute();
            Solid[] sres = new Solid[res.Length];
            for (int i = 0; i < res.Length; i++)
            {
                sres[i] = Solid.MakeSolid(res[i]);
            }
            return sres;
        }
        static public Solid[] Intersect(Solid solid1, Solid solid2)
        {
            BooleanOperation bo = new BooleanOperation();
            bo.SetShells(solid1.Shells[0], solid2.Shells[0], BooleanOperation.Operation.intersection);
            Shell[] res = bo.Execute();
            Solid[] sres = new Solid[res.Length];
            for (int i = 0; i < res.Length; i++)
            {
                sres[i] = Solid.MakeSolid(res[i]);
            }
            return sres;
        }
        static public void OperateWithMany(Solid solid, IEnumerable<Solid> other, IFrame frame, string operation)
        {
            switch (operation)
            {
                case "MenuId.Solid.RemoveFromAll":
                    using (frame.Project.Undo.UndoFrame)
                    {
                        foreach (Solid sld in other)
                        {
                            Solid[] res = Subtract(sld, solid);
                            if (res != null)
                            {
                                IGeoObjectOwner owner = sld.Owner;
                                owner.Remove(sld);
                                for (int i = 0; i < res.Length; i++)
                                {
                                    owner.Add(res[i]);
                                }
                            }
                        }
                        solid.Owner.Remove(solid);
                    }
                    break;
                case "MenuId.Solid.UniteWithAll":
                    using (frame.Project.Undo.UndoFrame)
                    {
                        Solid accumulate = solid;
                        int count = 0;
                        foreach (Solid sld in other)
                        {
                            Solid tmp = Unite(sld, accumulate);
                            if (tmp != null)
                            {
                                accumulate = tmp;
                                count++;
                                sld.Owner.Remove(sld);
                            }
                        }
                        IGeoObjectOwner owner = solid.Owner;
                        if (owner != null)
                        {
                            owner.Remove(solid);
                            owner.Add(accumulate);
                        }
                    }
                    break;
                case "MenuId.Solid.RemoveAll":
                    using (frame.Project.Undo.UndoFrame)
                    {
                        List<Solid> fragments = new List<Solid>();
                        fragments.Add(solid);
                        foreach (Solid sld in other)
                        {
                            List<Solid> newfragments = new List<Solid>();
                            for (int i = 0; i < fragments.Count; i++)
                            {
                                Solid[] res = Subtract(fragments[i], sld);
                                if (res != null && res.Length > 0)
                                {
                                    newfragments.AddRange(res);
                                }
                                else
                                {
                                    newfragments.Add(fragments[i]);
                                }
                            }
                            fragments = newfragments;
                        }
                        IGeoObjectOwner owner = solid.Owner;
                        owner.Remove(solid);
                        foreach (Solid sld in other)
                        {
                            owner.Remove(sld);
                        }
                        for (int i = 0; i < fragments.Count; i++)
                        {
                            fragments[i].CopyAttributes(solid);
                            owner.Add(fragments[i]);
                        }
                    }
                    break;
                case "MenuId.Solid.SplitWithAll":
                    using (frame.Project.Undo.UndoFrame)
                    {
                        List<Solid> fragments = new List<Solid>();
                        fragments.Add(solid);
                        foreach (Solid sld in other)
                        {
                            List<Solid> newfragments = new List<Solid>();
                            for (int i = 0; i < fragments.Count; i++)
                            {
                                Solid[] res1 = Subtract(fragments[i], sld);
                                Solid[] res2 = Subtract(sld, fragments[i]);
                                Solid[] res3 = Intersect(fragments[i], sld);
                                if (res1 != null && res1.Length > 0)
                                {
                                    if (res1 != null) newfragments.AddRange(res1);
                                    if (res2 != null) newfragments.AddRange(res2);
                                    if (res3 != null) newfragments.AddRange(res3);
                                }
                                else
                                {
                                    newfragments.Add(fragments[i]);
                                }
                            }
                            fragments = newfragments;
                        }
                        IGeoObjectOwner owner = solid.Owner;
                        owner.Remove(solid);
                        foreach (Solid sld in other)
                        {
                            owner.Remove(sld);
                        }
                        for (int i = 0; i < fragments.Count; i++)
                        {
                            owner.Add(fragments[i]);
                        }
                    }
                    break;
                case "MenuId.Solid.IntersectWithAll":
                    using (frame.Project.Undo.UndoFrame)
                    {
                        List<Solid> fragments = new List<Solid>();
                        fragments.Add(solid);
                        foreach (Solid sld in other)
                        {
                            List<Solid> newfragments = new List<Solid>();
                            for (int i = 0; i < fragments.Count; i++)
                            {
                                Solid[] res = Intersect(fragments[i], sld);
                                if (res != null && res.Length > 0)
                                {
                                    newfragments.AddRange(res);
                                }
                                else
                                {
                                    newfragments.Add(fragments[i]);
                                }
                            }
                            fragments = newfragments;
                        }
                        IGeoObjectOwner owner = solid.Owner;
                        owner.Remove(solid);
                        foreach (Solid sld in other)
                        {
                            owner.Remove(sld);
                        }
                        for (int i = 0; i < fragments.Count; i++)
                        {
                            owner.Add(fragments[i]);
                        }
                    }
                    break;
            }
        }
    }
}