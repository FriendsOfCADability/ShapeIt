using CADability.GeoObject;
using CADability.Substitutes;
using System;
using System.Collections;


namespace CADability.Actions
{
    /// <summary>
    /// 
    /// </summary>
    public class ReflectObjects : ConstructAction
    {
        private Block block; // der Arbeitsblock zum Darstellen
        private GeoObjectList originals; // zur Sicherung
        private bool copyObject;
        private ModOp reflectModOp;
        private ModOp reflectModOpStart;
        private CurveInput reflectLine;

        public ReflectObjects(GeoObjectList list)
        {
            block = Block.Construct();
            foreach (IGeoObject go in list)
            {
                block.Add(go.Clone());
            }
            originals = new GeoObjectList(list); // originale merken
        }


        void SetReflectPoint(GeoPoint p)
        { // spiegeln am Punkt
            for (int i = 0; i < originals.Count; ++i)
            { // Urzustand
                block.Item(i).CopyGeometry(originals[i]);
            }
            reflectModOp = ModOp.ReflectPoint(p);
            block.Modify(reflectModOp);
            reflectLine.Optional = true; // damit er raus kann zu onDone
        }

        private bool ReflectLine(CurveInput sender, ICurve[] Curves, bool up)
        {   // ... nur die sinnvolen Kurven verwenden
            ArrayList usableCurves = new ArrayList();
            for (int i = 0; i < Curves.Length; ++i)
            {
                Line l = Curves[i] as Line;
                if (l != null)
                {
                    usableCurves.Add(Curves[i]);
                }
            }
            // ...hier wird der ursprüngliche Parameter überschrieben. Hat ja keine Auswirkung nach außen.
            Curves = (ICurve[])usableCurves.ToArray(typeof(ICurve));
            if (up)
                if (Curves.Length == 0)
                {
                    sender.SetCurves(Curves, null); // ...die werden jetzt im ControlCenter dargestellt (nur bei up)
                    OnDone();
                    return true;
                }
                else sender.SetCurves(Curves, Curves[0]);
            for (int i = 0; i < originals.Count; ++i)
            { // Urzustand
                block.Item(i).CopyGeometry(originals[i]);
            }
            if (Curves.Length > 0)
            {	// einfach die erste Kurve nehmen
                ICurve iCurve = Curves[0]; // nun drehen um 180 Grad
                reflectModOp = ModOp.Rotate(iCurve.StartPoint, iCurve.StartDirection, new SweepAngle(Math.PI));
                block.Modify(reflectModOp);
                return true;
            }
            reflectModOp = reflectModOpStart; // zurücksetzen
            block.Modify(reflectModOp);
            return false;
        }

        private void ReflectLineChanged(CurveInput sender, ICurve SelectedCurve)
        {
            for (int i = 0; i < originals.Count; ++i)
            { // Urzustand
                block.Item(i).CopyGeometry(originals[i]);
            }  // nun drehen um 180 Grad
            reflectModOp = ModOp.Rotate(SelectedCurve.StartPoint, SelectedCurve.StartDirection, new SweepAngle(Math.PI));
            block.Modify(reflectModOp);
        }

        bool SetReflectPlane(Plane val)
        {  // erstmal den Urprungszustand herstellen, "block" ist ja schon gespiegelt 
            for (int i = 0; i < originals.Count; ++i)
            {
                block.Item(i).CopyGeometry(originals[i]);
            }
            // jetzt berechnen
            reflectModOp = ModOp.ReflectPlane(val);
            block.Modify(reflectModOp);
            reflectLine.Optional = true;
            // er liefert immer true zurück, da es ja immer eine Lösung gibt
            return true;
        }

        private void SetCopy(bool val)
        {
            copyObject = val;
        }


        public override void OnSetAction()
        {
            base.ActiveObject = block;
            base.TitleId = "ReflectObjects";
            copyObject = ConstrDefaults.DefaultCopyObjects;

            GeoPointInput reflectPoint = new GeoPointInput("ReflectObjects.Point");
            //           reflectPoint.GetGeoPointEvent += new GeoPointInput.GetGeoPointDelegate(GetReflectPoint);
            reflectPoint.SetGeoPointEvent += new GeoPointInput.SetGeoPointDelegate(SetReflectPoint);
            reflectPoint.Optional = true;

            reflectLine = new CurveInput("ReflectObjects.Line");
            reflectLine.Decomposed = true; // nur Einzelelemente, auch bei Polyline und Pfad
            reflectLine.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(ReflectLine);
            reflectLine.CurveSelectionChangedEvent += new CurveInput.CurveSelectionChangedDelegate(ReflectLineChanged);

            PlaneInput reflectPlane = new PlaneInput("ReflectObjects.Plane");
            reflectPlane.SetPlaneEvent += new PlaneInput.SetPlaneDelegate(SetReflectPlane);
            //            reflectPlane.GetPlaneEvent += new PlaneInput.GetPlaneDelegate(GetReflectPlane);
            reflectPlane.Optional = true;

            BooleanInput copy = new BooleanInput("Modify.CopyObjects", "YesNo.Values");
            copy.DefaultBoolean = ConstrDefaults.DefaultCopyObjects;
            copy.SetBooleanEvent += new CADability.Actions.ConstructAction.BooleanInput.SetBooleanDelegate(SetCopy);


            base.SetInput(reflectPoint, reflectLine, reflectPlane, copy);
            BoundingBox result = BoundingBox.EmptyBoundingCube;
            foreach (IGeoObject go in originals)
            {
                result.MinMax(go.GetBoundingCube());
            }
            GeoPoint blockCenter = result.GetCenter();
            block.RefPoint = blockCenter;
            base.BasePoint = blockCenter;
            reflectModOp = ModOp.ReflectPlane(new Plane(base.BasePoint, base.ActiveDrawingPlane.Normal, base.ActiveDrawingPlane.DirectionY));
            reflectModOpStart = reflectModOp;
            block.Modify(reflectModOp);
            base.OnSetAction();
        }



        public override string GetID()
        {
            return "ReflectObjects";
        }

        public override void OnDone()
        {
            // reflection inverses the orientation of the objects. For Shells and Solids this is probably not intended. So we reverse the orientation after modification
            using (Frame.Project.Undo.UndoFrame)
            {
                if (((Frame.UIService.ModifierKeys & Keys.Shift) != 0) || copyObject)
                {
                    GeoObjectList cloned = new GeoObjectList();
                    foreach (IGeoObject go in originals)
                    {
                        IGeoObject cl = go.Clone();
                        cl.Modify(reflectModOp);
                        if (reflectModOp.Determinant < 0)
                        {
                            if (cl is Solid sld)
                            {
                                for (int i = 0; i < sld.Shells.Length; i++)
                                {
                                    sld.Shells[i].ReverseOrientation();
                                }
                            }
                            else if (cl is Shell sh)
                            {
                                sh.ReverseOrientation();
                            }
                        }
                        cloned.Add(cl);
                    }
                    base.Frame.Project.GetActiveModel().Add(cloned);
                }
                else
                {
                    originals.Modify(reflectModOp);
                    if (reflectModOp.Determinant < 0)
                    {
                        foreach (IGeoObject go in originals)
                        {
                            if (go is Solid sld)
                            {
                                for (int i = 0; i < sld.Shells.Length; i++)
                                {
                                    sld.Shells[i].ReverseOrientation();
                                }
                            }
                            else if (go is Shell sh)
                            {
                                sh.ReverseOrientation();
                            }
                        }
                    }
                }
            }
            base.ActiveObject = null; // damit es nicht gleich eingefügt wird
            base.OnDone();
        }

    }
}

