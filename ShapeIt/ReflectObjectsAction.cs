using CADability;
using CADability.Actions;
using CADability.GeoObject;
using CADability.Substitutes;
using System;
using System.Collections.Generic;
using static CADability.Actions.ConstructAction;

namespace ShapeIt
{
    /// <summary>
    /// Reflects (mirrors) one or more objects. This is the modern replacement for CADability's
    /// <see cref="CADability.Actions.ReflectObjects"/>, implemented analogously to
    /// <see cref="RotateFacesAction"/> and <see cref="ExtrudeFacesAction"/>: the preview is shown through a
    /// <see cref="Feedback"/> object instead of a working <see cref="Block"/>.
    ///
    /// Unlike the CADability action it does not offer a reflection point. It always offers a mirror
    /// <em>plane</em>. Only when <em>all</em> objects are curves (<see cref="ICurve"/>) lying in a common
    /// plane it additionally offers a mirror <em>line</em> (the typical "sketch" case, where symmetric shapes
    /// are built from planar curves). A picked line is only accepted when it lies in that common plane.
    /// </summary>
    internal class ReflectObjectsAction : ConstructAction
    {
        private readonly GeoObjectList originals; // the objects to be reflected (references, modified in place in OnDone)
        private ModOp reflectModOp; // the currently active reflection
        private bool copyObject;
        private PlaneInput reflectPlaneInput;
        private CurveInput reflectLineInput; // only created/offered for the planar-curves case
        private Feedback feedback;
        // The curves to be reflected (only filled when all objects are curves). A picked mirror axis is
        // validated dynamically against these: axis line and object curves must share a common plane.
        private readonly List<ICurve> reflectableCurves = new List<ICurve>();
        // true when a mirror line may be offered in addition to the mirror plane (see constructor)
        private readonly bool offerLine;

        public ReflectObjectsAction(GeoObjectList list)
        {
            originals = new GeoObjectList(list); // keep the originals; they are modified/cloned in OnDone

            // Decide which mirror objects to offer: a plane is always offered. A line is offered only when
            // there is no BRep object (Face/Shell/Solid) and all objects are curves that either lie in a
            // common plane, or are under-determined (a single line / collinear lines, which do not span a
            // plane on their own -- GetPlanarState catches that case, GetCommonPlane cannot).
            bool hasBRep = false;
            bool allCurves = originals.Count > 0;
            bool allUnderDetermined = originals.Count > 0;
            foreach (IGeoObject go in originals)
            {
                if (go is Face || go is Shell || go is Solid) hasBRep = true;
                if (go is ICurve curve)
                {
                    reflectableCurves.Add(curve);
                    if (curve.GetPlanarState() != PlanarState.UnderDetermined) allUnderDetermined = false;
                }
                else
                {
                    allCurves = false;
                    allUnderDetermined = false;
                }
            }
            bool curvesArePlanarSet = allCurves && Curves.GetCommonPlane(reflectableCurves, out _);
            offerLine = !hasBRep && allCurves && (curvesArePlanarSet || allUnderDetermined);
        }

        public override void OnSetAction()
        {
            base.TitleId = "ReflectObjects";
            copyObject = ConstrDefaults.DefaultCopyObjects;

            // Seed a sensible default mirror: a plane perpendicular to the drawing plane through the center of
            // the objects (same default as CADability's ReflectObjects).
            BoundingBox result = BoundingBox.EmptyBoundingBox;
            foreach (IGeoObject go in originals) result.MinMax(go.GetBoundingCube());
            GeoPoint center = result.GetCenter();
            base.BasePoint = center;
            reflectModOp = ModOp.ReflectPlane(new Plane(center, base.ActiveDrawingPlane.Normal, base.ActiveDrawingPlane.DirectionY));

            reflectPlaneInput = new PlaneInput("ReflectObjects.Plane");
            reflectPlaneInput.SetPlaneEvent += new PlaneInput.SetPlaneDelegate(SetReflectPlane);

            BooleanInput copy = new BooleanInput("Modify.CopyObjects", "YesNo.Values");
            copy.DefaultBoolean = ConstrDefaults.DefaultCopyObjects;
            copy.SetBooleanEvent += new BooleanInput.SetBooleanDelegate(SetCopy);

            if (offerLine)
            {
                // The line is the primary (active) input; picking one fixes the input and finishes the action.
                // The plane stays optional; setting it makes the line optional as well, so it too can finish.
                reflectLineInput = new CurveInput("ReflectObjects.Line");
                reflectLineInput.Decomposed = true; // single elements only, even from a polyline or path
                reflectLineInput.MouseOverCurvesEvent += new CurveInput.MouseOverCurvesDelegate(ReflectLineCurves);
                reflectLineInput.CurveSelectionChangedEvent += new CurveInput.CurveSelectionChangedDelegate(ReflectLineChanged);
                reflectPlaneInput.Optional = true;
                base.SetInput(reflectLineInput, reflectPlaneInput, copy);
            }
            else
            {
                // No line for BRep objects: the plane is the required input.
                base.SetInput(reflectPlaneInput, copy);
            }

            base.ShowActiveObject = false;

            feedback = new Feedback();
            // the mirrored preview clones should keep the colors of their originals, not a single action color
            feedback.CreatedObjectsOwnColor = true;
            feedback.Attach(CurrentMouseView);

            base.OnSetAction();

            Recalc();
        }

        private bool SetReflectPlane(Plane val)
        {
            reflectModOp = ModOp.ReflectPlane(val);
            if (reflectLineInput != null) reflectLineInput.Optional = true; // the plane defines the mirror; the line is no longer required
            Recalc();
            return true; // there is always a solution
        }

        private bool ReflectLineCurves(CurveInput sender, ICurve[] Curves, bool up)
        {
            // Only straight lines that share a common plane with the objects can serve as a mirror axis.
            List<ICurve> usable = new List<ICurve>();
            foreach (ICurve c in Curves)
            {
                if (c is Line line && TryGetMirrorFromAxis(line, out _)) usable.Add(c);
            }
            ICurve[] usableCurves = usable.ToArray();
            if (up)
            {
                if (usableCurves.Length == 0) sender.SetCurves(usableCurves, null);
                else sender.SetCurves(usableCurves, usableCurves[0]);
            }
            if (usableCurves.Length > 0 && TryGetMirrorFromAxis((Line)usableCurves[0], out ModOp mirror))
            {
                reflectModOp = mirror;
                return Recalc();
            }
            return false;
        }

        private void ReflectLineChanged(CurveInput sender, ICurve SelectedCurve)
        {
            if (SelectedCurve is Line line && TryGetMirrorFromAxis(line, out ModOp mirror))
            {
                reflectModOp = mirror;
                Recalc();
            }
        }

        // A picked axis line is valid only if it shares a common plane with the object curves (the sketch
        // plane). The mirror plane then contains the axis line and stands perpendicular to that sketch plane,
        // so reflecting maps the planar objects to their mirror image within the plane. This also covers the
        // single-line case, where the object line alone does not span a plane: the plane is determined by the
        // object line together with the axis line.
        private bool TryGetMirrorFromAxis(Line axisLine, out ModOp mirror)
        {
            mirror = ModOp.Identity;
            List<ICurve> all = new List<ICurve>(reflectableCurves) { axisLine };
            if (!Curves.GetCommonPlane(all, out Plane sketchPlane)) return false;
            mirror = ModOp.ReflectPlane(new Plane(axisLine.StartPoint, axisLine.StartDirection, sketchPlane.Normal));
            return true;
        }

        private void SetCopy(bool val)
        {
            copyObject = val;
        }

        // Rebuilds the preview: clones of the originals, reflected with the current ModOp, shown via Feedback.
        private bool Recalc()
        {
            feedback.Clear();
            bool reverse = reflectModOp.Determinant < 0; // reflection inverts orientation; fix normals for the preview
            foreach (IGeoObject go in originals)
            {
                IGeoObject cl = go.Clone();
                cl.Modify(reflectModOp);
                if (reverse) ReverseIfBRep(cl);
                feedback.CreatedObjects.Add(cl);
            }
            feedback.Refresh();
            return feedback.CreatedObjects.Count > 0;
        }

        private static void ReverseIfBRep(IGeoObject go)
        {
            if (go is Solid solid)
            {
                for (int i = 0; i < solid.Shells.Length; i++) solid.Shells[i].ReverseOrientation();
            }
            else if (go is Shell shell)
            {
                shell.ReverseOrientation();
            }
        }

        public override string GetID()
        {
            return "ReflectObjects";
        }

        public override void OnDone()
        {
            using (Frame.Project.Undo.UndoFrame)
            {
                if (((Frame.UIService.ModifierKeys & Keys.Shift) != 0) || copyObject)
                {
                    // keep the originals and add reflected copies
                    GeoObjectList cloned = new GeoObjectList();
                    foreach (IGeoObject go in originals)
                    {
                        IGeoObject cl = go.Clone();
                        cl.Modify(reflectModOp);
                        cloned.Add(cl);
                    }
                    base.Frame.Project.GetActiveModel().Add(cloned);
                }
                else
                {
                    // reflect the originals in place. Reflection inverts the orientation; for Shells and Solids
                    // this is not intended, so we reverse the orientation again.
                    originals.Modify(reflectModOp);
                    if (reflectModOp.Determinant < 0)
                    {
                        foreach (IGeoObject go in originals) ReverseIfBRep(go);
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
