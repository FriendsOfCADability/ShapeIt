using CADability;
using CADability.Actions;
using CADability.Attribute;
using CADability.Curve2D;
using CADability.Forms;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.Substitutes;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms.Design;
using System.Windows.Forms.VisualStyles;
using System.Xml.Linq;
using Wintellect.PowerCollections;
using static CADability.Projection;

namespace ShapeIt
{
    /// <summary>
    /// Class to enable Edges to reside in an <see cref="OctTree{T}"/>
    /// </summary>
    internal class ModellingPropertyEntries : PropertyEntryImpl, ICommandHandler
    {
        private List<IPropertyEntry> subEntries = new List<IPropertyEntry>(); // the list of property entries in this property page
        private IFrame cadFrame; // the frame (root object of cadability)
        private SelectObjectsAction selectAction; // the select action, which runs parallel to the modelling property page
        private bool modelligIsActive; // if true, there is no normal or "old" CADability selection, but every mouseclick of the SelectObjectsAction is handled here
        private bool selectionTabForced;
        private Feedback feedback;
        private PickArea lastPickArea; // for ComposeModellingEntries to avoid passing it to all methods
        private System.Drawing.Point mouseDownPosition; // to check whether the user is dragging a selection rectangle
        private int dragWidth = 5; // Delta to to check for start dragging the selection rectangle
        private System.Drawing.Point mouseCurrentPosition; // when dragging a selection rectangle, the second point
        private bool isDragging = false; // the user is dragging a selection rectangle
        private bool isMoving = false; // the user is moving the selected objects
        private bool isHotspotMoving = false; // the user is movind a hotspot
        private HashSet<IGeoObject> selectedObjects = new HashSet<IGeoObject>(); // the currently selected root objects 
        private GeoObjectList movingObjects = new GeoObjectList(); // a copy of the selected objects, used for moving
        private GeoPoint moveObjectsDownPoint;
        private bool downOnSelectedObjects;
        private ModOp accumulatedMovement; // when dragging objects this is the accumulation movement in respect to the original objects
        private HashSet<IGeoObject> currentlySelected = new HashSet<IGeoObject>(); // all curves, edges, faces which are currently shown in the subentries
        private Dictionary<IGeoObject, IGeoObject> selectedSameSurfaceBudies = new Dictionary<IGeoObject, IGeoObject>();

        private GeoObjectList selectedChildObjects = new GeoObjectList(); // when a face or feature is selected, it is listed here to maybe move it later
        private bool movingChildObject; // the current move operation is on a feature or a face, which is not in the model
        private HashSet<IHotSpot> activeHotspots = new HashSet<IHotSpot>();
        private IHotSpot selectedHotspot;
        private IHotSpot hotspotUnderCursor;
        private IGeoObject selectedObjectUnderCursor;

        public ModellingPropertyEntries(IFrame cadFrame) : base("Modelling.Properties")
        {
            this.cadFrame = cadFrame;
            selectAction = cadFrame.ActiveAction as SelectObjectsAction;
            if (selectAction != null)
            {
                selectAction.SelectedObjectListChangedEvent += SelectedObjectListChanged;
                selectAction.ShowSelectedObjects = false; // we do the selection display with this class not with the SelectObjectsAction
            }
            cadFrame.ActionTerminatedEvent += ActionTerminated;
            cadFrame.ActionStartedEvent += ActionStarted;

            cadFrame.ViewsChangedEvent += ViewsChanged;
            modelligIsActive = true;
            selectionTabForced = false;

            cadFrame.ProcessContextMenuEvent += ProcessContextMenu;

            feedback = new Feedback();
            feedback.Attach(cadFrame.ActiveView);
            FeedbackArrow.SetNumberFormat(cadFrame);
            ViewsChanged(cadFrame); // first initialisation
        }

        private void ProcessContextMenu(ICommandHandler target, string MenuId, ref bool Processed)
        {
            if (MenuId == "MenuId.Object.Delete")
            {
                if (propertyPage.GetCurrentSelection() is IHandleKey handleKey)
                {    // so we don't have to implement it twice
                    if (handleKey.HandleKeyCommand(new KeyEventArgs(Keys.Delete))) Processed = true;
                }
            }
        }

        private void SelectedObjectListChanged(SelectObjectsAction sender, GeoObjectList selectedObjects)
        {
            // the user is in "old" selection mode and has changed the selction. Clear all selections in modelling mode
            if (!cadFrame.ControlCenter.GetPropertyPage("Modelling").IsOnTop()) Clear();
            return; // do we need this at all? selection here happens with filter mouse messages, this makes things complicated

        }
        #region CADability events
        private void ViewsChanged(IFrame theFrame)
        {
            cadFrame = theFrame;
            selectAction = cadFrame.ActiveAction as SelectObjectsAction; // which should always be the case, and it is maybe new one
            if (selectAction != null)
            {
                selectAction.FilterMouseMessagesEvent -= FilterSelectMouseMessages; // no problem, if it wasn't set
                selectAction.FilterMouseMessagesEvent += FilterSelectMouseMessages;
                selectAction.SelectedObjectListChangedEvent += SelectedObjectListChanged;
            }
            feedback.Attach(theFrame.ActiveView);
            if (modelligIsActive)
            {
                propertyPage?.BringToFront();
                cadFrame.SetControlCenterFocus("Modelling", "Modelling.Properties", true, false);
            }
        }
        private void ActionStarted(CADability.Actions.Action action)
        {
            if (modelligIsActive && !isHotspotMoving)
            {
                Select(); // selects this entry, which is the top entry for the page. By this, the current selected entry is beeing unselected and the display
                          // of feedback objects is removed
                feedback.Clear(); // additionally, but probably not necessary, because unselect of the current entry did the same
                cadFrame.ActiveView.Invalidate(PaintBuffer.DrawingAspect.Select, cadFrame.ActiveView.DisplayRectangle);
                subEntries.Clear();
                IPropertyPage pp = propertyPage;
                if (pp != null)
                {
                    pp.Remove(this); // to reflect this newly composed entry
                    pp.Add(this, true);
                }
            }
        }
        private void ActionTerminated(CADability.Actions.Action action)
        {
            if (modelligIsActive)
            {
                cadFrame.ControlCenter.ShowPropertyPage("Modelling");
            }
        }
        /// <summary>
        /// Returns the point on the drawing plane of the current mouse position
        /// </summary>
        /// <param name="e"></param>
        /// <param name="vw"></param>
        /// <returns></returns>
        private GeoPoint PointOnDrawingPlane(MouseEventArgs e, IView vw)
        {
            Axis mouseBeam = vw.Projection.PointBeam(e.Location);
            return vw.Projection.DrawingPlanePoint(vw.Projection.ProjectionPlane.Intersect(mouseBeam));
        }
        private enum CursorPosition { EmptySpace, OverObject, OverSelectedObject, OverHotSpot }
        private CursorPosition GetCursorPosition(MouseEventArgs e, IView vw)
        {
            int pickRadius = selectAction.Frame.GetIntSetting("Select.PickRadius", 5);
            PickArea pickArea = vw.Projection.GetPickSpace(new Rectangle(e.Location.X - pickRadius, e.Location.Y - pickRadius, pickRadius * 2, pickRadius * 2));
            foreach (IHotSpot hotspot in activeHotspots)
            {
                GeoPoint hp = hotspot.GetHotspotPosition();
                if (BoundingCube.UnitBoundingCube.Contains(pickArea.ToUnitBox * hp))
                {
                    hotspotUnderCursor = hotspot;
                    return CursorPosition.OverHotSpot;
                }
            }
            foreach (IGeoObject go in selectedObjects)
            {
                if (go.HitTest(pickArea, false))
                {
                    selectedObjectUnderCursor = go;
                    return CursorPosition.OverSelectedObject;
                }
            }
            foreach (IGeoObject go in selectedChildObjects)
            {
                if (go.HitTest(pickArea, false))
                {
                    selectedObjectUnderCursor = go;
                    return CursorPosition.OverSelectedObject;
                }
            }
            IEnumerable<Layer> visiblaLayers = new List<Layer>();
            if (vw is ModelView mv) visiblaLayers = mv.GetVisibleLayers();
            if (vw.Model.GetObjectsFromRect(pickArea, new Set<Layer>(visiblaLayers), PickMode.singleFaceAndCurve, null).Count > 0) return CursorPosition.OverObject;
            else return CursorPosition.EmptySpace;
        }

        private void FilterSelectMouseMessages(SelectObjectsAction.MouseAction mouseAction, CADability.Substitutes.MouseEventArgs e, IView vw, ref bool handled)
        {
            if (modelligIsActive && cadFrame.ControlCenter.GetPropertyPage("Modelling").IsOnTop())
            {
                CursorPosition cp = GetCursorPosition(e, vw);
                if (mouseAction == SelectObjectsAction.MouseAction.MouseDown && e.Button == CADability.Substitutes.MouseButtons.Left)
                {
                    downOnSelectedObjects = false;
                    isHotspotMoving = false;
                    if (cp == CursorPosition.OverSelectedObject)
                    {
                        downOnSelectedObjects = true;
                        accumulatedMovement = ModOp.Identity;
                        moveObjectsDownPoint = PointOnDrawingPlane(e, vw);
                    }
                    if (cp == CursorPosition.OverHotSpot)
                    {
                        isHotspotMoving = true;
                        hotspotUnderCursor.StartDrag(cadFrame);
                    }
                    mouseDownPosition = e.Location;
                }
                else if (mouseAction == SelectObjectsAction.MouseAction.MouseMove && e.Button == CADability.Substitutes.MouseButtons.Left)
                {
                    if (isDragging)
                    {
                        vw.SetCursor("SmallMove");
                        mouseCurrentPosition = e.Location;
                        feedback.selectionRectangle = Rectangle.FromLTRB(Math.Min(mouseCurrentPosition.X, mouseDownPosition.X),
                            Math.Min(mouseCurrentPosition.Y, mouseDownPosition.Y),
                            Math.Max(mouseCurrentPosition.X, mouseDownPosition.X),
                            Math.Max(mouseCurrentPosition.Y, mouseDownPosition.Y));
                        feedback.Refresh();
                    }
                    else if (isMoving)
                    {
                        if (downOnSelectedObjects)
                        {
                            if ((cadFrame.UIService.ModifierKeys & Keys.Control) == Keys.Control || movingChildObject) vw.SetCursor("MoveAndCopy");
                            else vw.SetCursor("Move");
                            MoveSelectedObjects(e, vw, false);
                        }
                    }
                    else
                    {
                        // get the cursor from the current mouse position
                        vw.SetCursor("Arrow"); // TODO: change to CursorFromMousePosition (depending on edge, face)
                    }
                    // left button is pressed and the mouse is moving. Check the distance to the last mouseDownPosition to switch on
                    // dragging mode
                    if (Math.Abs(e.X - mouseDownPosition.X) > dragWidth || Math.Abs(e.Y - mouseDownPosition.Y) > dragWidth)
                    {
                        if (downOnSelectedObjects)
                        {
                            if (!isMoving)
                            {
                                isMoving = true;
                                vw.SetCursor("Move");
                                movingObjects.Clear();
                                // prefer the selectedChildObject, which is maybe a feature or a face
                                foreach (IGeoObject go in selectedChildObjects)
                                {
                                    movingObjects.Add(go.Clone());
                                    movingChildObject = true;
                                }
                                if (movingObjects.Count == 0)
                                {
                                    movingChildObject = false;
                                    foreach (IGeoObject go in selectedObjects)
                                    {
                                        movingObjects.Add(go.Clone());
                                    }
                                }
                            }
                        }
                        else
                        {
                            isDragging = true;
                            vw.SetCursor("SmallMove");
                        }
                    }
                }
                else if (mouseAction == SelectObjectsAction.MouseAction.MouseMove && e.Button == CADability.Substitutes.MouseButtons.None)
                {
                    vw.SetCursor(GetCursor(e.Location, vw));
                }
                else if (mouseAction == SelectObjectsAction.MouseAction.MouseUp && e.Button == CADability.Substitutes.MouseButtons.Left)
                {
                    if (isMoving)
                    {
                        MoveSelectedObjects(e, vw, true);
                        handled = true;
                        return;
                    }
                    PickArea pickArea;
                    bool multiple;
                    bool onlyInside;
                    if (isDragging)
                    {
                        feedback.selectionRectangle = Rectangle.Empty;
                        feedback.Refresh();
                        isDragging = false;
                        Rectangle winrect = new Rectangle(Math.Min(mouseDownPosition.X, e.Location.X), Math.Min(mouseDownPosition.Y, e.Location.Y),
                            Math.Abs(mouseDownPosition.X - e.Location.X), Math.Abs(mouseDownPosition.Y - e.Location.Y));
                        if (winrect.Width == 0) winrect.Inflate(1, 0);
                        if (winrect.Height == 0) winrect.Inflate(0, 1);
                        pickArea = vw.Projection.GetPickSpace(winrect);
                        multiple = true;
                        onlyInside = mouseDownPosition.X < e.Location.X;
                    }
                    else
                    {
                        int pickRadius = selectAction.Frame.GetIntSetting("Select.PickRadius", 5);
                        pickArea = vw.Projection.GetPickSpace(new Rectangle(e.Location.X - pickRadius, e.Location.Y - pickRadius, pickRadius * 2, pickRadius * 2));
                        multiple = false;
                        onlyInside = false;
                    }
                    GeoObjectList objectsUnderCursor = GetObjectsUnderCursor(e.Location, vw, pickArea, multiple, onlyInside);
                    // when only a single click (not dragging a selections rectangle) then we also want to show the parent objects (i.e. the solid)
                    // when the control key is pressed, we want to add or remove from the current selection
                    bool addOrRemove = (cadFrame.UIService.ModifierKeys & Keys.Control) == Keys.Control;
                    ComposeModellingEntries(objectsUnderCursor, vw, pickArea, true, addOrRemove); // when should we use alsoParent==false?
                    IsOpen = true;
                    Refresh();
                }
                handled = true;
            }
        }

        private void MoveSelectedObjects(MouseEventArgs e, IView vw, bool mouseUp)
        {
            if (movingObjects.Count > 0)
            {
                feedback.Clear();
                GeoPoint currentPoint = PointOnDrawingPlane(e, vw);
                ModOp translate = ModOp.Translate(currentPoint - moveObjectsDownPoint);
                accumulatedMovement = translate * accumulatedMovement;
                moveObjectsDownPoint = currentPoint;
                foreach (IGeoObject go in movingObjects)
                {
                    go.Modify(translate);
                }
                feedback.ShadowFaces.AddRange(movingObjects);
                feedback.Refresh();
                if (mouseUp)
                {
                    if ((cadFrame.UIService.ModifierKeys & Keys.Control) == Keys.Control || movingChildObject)
                    {
                        cadFrame.Project.GetActiveModel().Add(movingObjects);
                        Clear();
                        ComposeModellingEntries(movingObjects, vw, null);
                    }
                    else
                    {
                        foreach (IGeoObject go in selectedObjects)
                        {
                            go.Modify(accumulatedMovement);
                        }
                        GeoObjectList l = new GeoObjectList(selectedObjects);
                        Clear();
                        ComposeModellingEntries(l, vw, null);
                    }
                    movingObjects.Clear();
                    downOnSelectedObjects = false;
                    isMoving = false;
                }
            }
        }

        internal bool OnEscape()
        {
            // the following was commented out, because the shortcuts on the direct menu do the same jab
            //if (accumulatedObjects.Count > 1 && isAccumulating)
            //{
            //    StopAccumulating();
            //    IPropertyPage pp = propertyPage;
            //    if (pp != null)
            //    {
            //        pp.SelectEntry(this); // to unselect the currently selected entry and clear feedback objects
            //        subEntries.Clear();
            //        pp.Remove(this); // to reflect this newly composed entry
            //        pp.Add(this, true);
            //    }
            //    return true;
            //}
            return false;
        }

        #endregion
        #region PropertyEntry implementation
        public override PropertyEntryType Flags => PropertyEntryType.Selectable | PropertyEntryType.GroupTitle | PropertyEntryType.Checkable | PropertyEntryType.HasSubEntries; // PropertyEntryType.ContextMenu | 
        public override IPropertyEntry[] SubItems => subEntries.ToArray();
        public override string Value
        {
            get
            {
                // Value == "0"(not checked), "1"(checked), "2" (indetermined or disabled)
                return modelligIsActive ? "1" : "0";
            }
        }
        public override void ButtonClicked(PropertyEntryButton button)
        {
            if (button == PropertyEntryButton.check)
            {
                modelligIsActive = !modelligIsActive;
                propertyPage?.Refresh(this);
                cadFrame.ActionStack.FindAction(typeof(SelectObjectsAction));
                if (cadFrame.ActiveAction is SelectObjectsAction selectAction) selectAction.ShowSelectedObjects = !modelligIsActive;
            }
        }
        public override void Selected(IPropertyEntry previousSelected)
        {
            feedback.Clear();
            cadFrame.ActiveView.Invalidate(PaintBuffer.DrawingAspect.Select, cadFrame.ActiveView.DisplayRectangle);
            base.Selected(previousSelected);
        }
        public override void Added(IPropertyPage pp)
        {
            pp.OnPreProcessKeyDown += OnPreProcessKeyDown;
            pp.OnSelectionChanged += OnSelectionChanged;
            base.Added(pp);
        }
        /// <summary>
        /// We need this, because IPropertyEntry.Parent is not correct implemented
        /// </summary>
        /// <param name="fromHere"></param>
        /// <param name="findThis"></param>
        /// <returns></returns>
        private IPropertyEntry FindParent(IPropertyEntry fromHere, IPropertyEntry findThis)
        {
            if (fromHere.Flags.HasFlag(PropertyEntryType.HasSubEntries))
            {
                for (int i = 0; i < fromHere.SubItems.Length; i++)
                {
                    if (fromHere.SubItems[i] == findThis) return fromHere;
                    IPropertyEntry found = FindParent(fromHere.SubItems[i], findThis);
                    if (found != null) return found;
                }
            }
            return null;
        }

        private void OnSelectionChanged(IPropertyEntry unselected, IPropertyEntry selected)
        {
            // the selection of property entries has changed. Is there a single IGeoObject selected, which could show hotspots?
            bool canShowHotspots = false;
            if (selected != null)
            {
                IPropertyEntry current = selected;
                while (current != null)
                {
                    if (current is IDisplayHotSpots)
                    {
                        canShowHotspots = true;
                        break;
                    }
                    current = FindParent(this, current);
                }
            }
            // IGeoObjectShowProperty is the parent class of all geo objects property entries
            if (!canShowHotspots && feedback.hotSpots.Count > 0)
            {
                activeHotspots.Clear();
                feedback.hotSpots.Clear();
                feedback.Refresh();
            }
        }

        public override void Removed(IPropertyPage pp)
        {
            pp.OnPreProcessKeyDown -= OnPreProcessKeyDown;
            pp.OnSelectionChanged -= OnSelectionChanged;
            base.Removed(pp);
        }
        #endregion
        private GeoObjectList GetObjectsUnderCursor(System.Drawing.Point mousePoint, IView vw, PickArea pickArea, bool multiple, bool onlyInside)
        {
            GeoObjectList objectsUnderCursor = new GeoObjectList();
            IEnumerable<Layer> visiblaLayers = new List<Layer>();
            if (vw is ModelView mv) visiblaLayers = mv.GetVisibleLayers();
            HashSet<IGeoObject> objects = new HashSet<IGeoObject>();
            PickMode pm = multiple ? PickMode.onlyEdges : PickMode.singleEdge; // first edges, they should have a higher priority in resourceIdOfEntryToSelect (see below)
            objects.UnionWith(vw.Model.GetObjectsFromRect(pickArea, new Set<Layer>(visiblaLayers), pm, null)); // returns all edges under the cursor
            pm = multiple ? PickMode.children : PickMode.singleChild;
            objects.UnionWith(vw.Model.GetObjectsFromRect(pickArea, new Set<Layer>(visiblaLayers), pm, null)); // returns all the faces curves or text objects under the cursor
            if (onlyInside)
            {
                objects = new HashSet<IGeoObject>(objects.Where(go => go.HitTest(pickArea, true)));
            }
            return new GeoObjectList(objects);
        }
        private string GetCursor(System.Drawing.Point location, IView vw)
        {
            int pickRadius = selectAction.Frame.GetIntSetting("Select.PickRadius", 5);
            PickArea pickArea = vw.Projection.GetPickSpace(new Rectangle(location.X - pickRadius, location.Y - pickRadius, pickRadius * 2, pickRadius * 2));
            IEnumerable<Layer> visiblaLayers = new List<Layer>();
            if (vw is ModelView mv) visiblaLayers = mv.GetVisibleLayers();
            if (vw.Model.GetObjectsFromRect(pickArea, new Set<Layer>(visiblaLayers), PickMode.singleFaceAndCurve, null).Count > 0) return "Hand";
            else return "Arrow";
        }

        private void ComposeModellingEntries(GeoObjectList objectsUnderCursor, IView vw, PickArea pickArea, bool alsoParent = true, bool addRemove = false)
        {   // a mouse left button up took place. Compose all the entries for objects, which can be handled by 
            // the object(s) under the mouse cursor
            Axis clickBeam = Axis.InvalidAxis;
            if (pickArea != null)
            {
                lastPickArea = pickArea;
                clickBeam = new Axis(pickArea.FrontCenter, pickArea.Direction);
            }
            subEntries.Clear(); // build a new list of modelling properties
            activeHotspots.Clear(); // something in the selection of objects has changed. Remove all hotspots
            feedback.hotSpots.Clear();
            feedback.Refresh();

            // complete the same surface faces and edges in objectsUnderCursor
            Dictionary<IGeoObject, IGeoObject> sameSurfaceBudies = new Dictionary<IGeoObject, IGeoObject>();
            for (int i = objectsUnderCursor.Count - 1; i >= 0; --i)
            {
                if (objectsUnderCursor[i] is ICurve crv)
                {
                    if ((crv as IGeoObject).Owner is Edge edg)
                    {
                        HashSet<Edge> connectedSameGeometryEdges = Shell.ConnectedSameGeometryEdges(new Edge[] { edg });
                        connectedSameGeometryEdges.ExceptWith(new Edge[] { edg });
                        foreach (Edge ec in connectedSameGeometryEdges)
                        {
                            sameSurfaceBudies[objectsUnderCursor[i]] = ec.Curve3D as IGeoObject; // there could be multiple, this is not implemented yet
                        }
                    }
                }
                if (objectsUnderCursor[i] is Face fc)
                {
                    foreach (Face fs in fc.GetSameSurfaceConnected())
                    {
                        sameSurfaceBudies[objectsUnderCursor[i]] = fc; // there could be multiple, this is not implemented yet
                    }
                }
            }
            if (addRemove)
            {   // remove those objects, which are already selected and add the objectsUnderCursor
                HashSet<IGeoObject> common = new HashSet<IGeoObject>(currentlySelected.Intersect(objectsUnderCursor));
                currentlySelected.UnionWith(objectsUnderCursor);
                currentlySelected.ExceptWith(common);
                foreach (var kvp in sameSurfaceBudies)
                {
                    selectedSameSurfaceBudies[kvp.Key] = kvp.Value;
                }
            }
            else
            {   // the old selection is replaced by the new selection
                currentlySelected.Clear();
                currentlySelected.UnionWith(objectsUnderCursor);
                selectedSameSurfaceBudies = sameSurfaceBudies;
            }
            // what to focus after the selection changed?
            string resourceIdOfEntryToSelect = String.Empty; // may contain several ids
            IPropertyEntry pe = propertyPage.GetCurrentSelection();
            if (pe != null) resourceIdOfEntryToSelect = pe.ResourceId; // resource id of what is currently selected, if objectsUnderCursor was removed

            {   // test, whethere there are hidden objects and if so, make a menu to show them
                bool thereAreHiddenObjects = false;
                Model m = Frame.Project.GetActiveModel();
                foreach (IGeoObject go in m.AllObjects)
                {   // hidden objects are in a layer named "CADability.Hidden" and contain a reference to their original layer
                    if (go.Layer != null && go.Layer.Name == "CADability.Hidden" && go.UserData.ContainsData("CADability.OriginalLayer"))
                    {
                        thereAreHiddenObjects = true;
                        break;
                    }
                }
                if (thereAreHiddenObjects)
                {
                    DirectMenuEntry showHiddenObjects = new DirectMenuEntry("MenuId.ShowHidden");
                    showHiddenObjects.ExecuteMenu = (frame) =>
                    {
                        foreach (IGeoObject go in m.AllObjects)
                        {   // restore the original layer and remove the reference to it in the user data
                            if (go.Layer != null && go.Layer.Name == "CADability.Hidden" && go.UserData.ContainsData("CADability.OriginalLayer"))
                            {
                                Layer layer = go.UserData.GetData("CADability.OriginalLayer") as Layer;
                                go.Layer = layer;
                                go.UserData.Remove("CADability.OriginalLayer");
                            }
                        }
                        Clear();
                        return true;
                    };
                    showHiddenObjects.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        if (selected)
                        {
                            foreach (IGeoObject go in m.AllObjects)
                            {   // restore the original layer and remove the reference to it in the user data
                                if (go.Layer != null && go.Layer.Name == "CADability.Hidden" && go.UserData.ContainsData("CADability.OriginalLayer"))
                                {
                                    feedback.ShadowFaces.Add(go);
                                }
                            }
                        }
                        feedback.Refresh();
                        return true;
                    };
                    subEntries.Add(showHiddenObjects);
                }
            }

            // show actions for all vertices, edges, faces and curves in 
            // distibute objects into their categories
            List<Face> faces = new List<Face>();
            List<Shell> shells = new List<Shell>(); // only Shells, which are not part of a Solid
            List<Solid> solids = new List<Solid>();
            List<Face> axis = new List<Face>(); // the axis of these faces, which may be cylindrical, conical or toroidal
            List<Edge> edges = new List<Edge>();
            List<ICurve> curves = new List<ICurve>(); // curves, but not axis
            List<Text> texts = new List<Text>(); // Text objects, to extrude
            Dictionary<Solid, List<Face>> solidToFaces = new Dictionary<Solid, List<Face>>();

            selectedObjects.Clear();
            foreach (IGeoObject go in currentlySelected)
            {
                {   // add top level objects to selectedObjects to be able to copy to clipboard.
                    // it would be better to use the object of the current selection, but it is not easily available
                    IGeoObjectOwner owner = go.Owner;
                    IGeoObject current = go;
                    while (owner != null && !(owner is Model))
                    {
                        if (owner is Edge edge) owner = edge.Owner.Owner;
                        else if (owner is IGeoObject goo) owner = goo.Owner;
                        else owner = null;
                        if (owner is IGeoObject g) current = g;
                    }
                    if (owner is Model) selectedObjects.Add(current);
                }
                if (go is Solid sld)
                {
                    solids.Add(sld);
                }
                else if (go is Face fc)
                {
                    faces.Add(fc);
                    if (alsoParent && fc.Owner is Shell sh)
                    {
                        if (sh.Owner is Solid solid)
                        {
                            solids.Add(solid);
                            if (!solidToFaces.TryGetValue(solid, out List<Face> fcs)) solidToFaces[solid] = fcs = new List<Face>();
                            fcs.Add(fc);
                        }
                        else shells.Add(sh);
                    }
                }
                else if (go is ICurve crv)
                {
                    if (go.Owner is Edge edge)
                    {
                        if (!edge.ConnectsSameSurfaces()) edges.Add(edge);
                    }
                    else if (go.UserData.Contains("CADability.AxisOf"))
                    {
                        Face faceWithAxis = (go).UserData.GetData("CADability.AxisOf") as Face;
                        if (faceWithAxis != null) axis.Add(faceWithAxis);
                    }
                    else
                    {
                        curves.Add(crv);
                    }
                }
                else if (go is Text text)
                {
                    texts.Add(text);
                }
            }

            // What would we select
            {
                if (edges.Count > 0) resourceIdOfEntryToSelect = "MultipleEdges.Properties" + "|" + "MenuId.Edge";
                else if (curves.Count > 0) resourceIdOfEntryToSelect = "MultipleCurves.Properties" + "|" + "MenuId.CurveMenus";
                else if (faces.Count > 0) resourceIdOfEntryToSelect = "MultipleFaces.Properties" + "|" + "MenuId.Face";
                else if (solids.Count > 0) resourceIdOfEntryToSelect = "MultipleSolids.Properties" + "|" + "MenuId.Solid";
                if (texts.Count > 0) resourceIdOfEntryToSelect = "MenuId.TextMenus";
            }


            // find features from either edges or faces
            if (edges.Count > 0) // are there features defined by the selected edges?
            {
                Shell edgesShell = (edges.First().Owner as Face).Owner as Shell;
                HashSet<Edge> connectedEdges = Shell.ConnectedSameGeometryEdges(edges); // add all faces which have the same surface and are connected
                if (edgesShell != null)
                {
                    try
                    {
                        if (edgesShell.FeatureFromLoops(connectedEdges, out IEnumerable<Face> featureFaces, out List<Face> connection, out bool isGap))
                        {
                            // there is a feature
                            IPropertyEntry fp = GetFeatureProperties(vw, featureFaces, connection, isGap);
                            if (fp != null && fp.SubItems.Length > 0) subEntries.Add(fp);
                        }
                    }
                    catch (Exception ex) { };
                }
            }
            if (faces.Count > 0) // are there features defined by the selected faces?
            {
                Shell facesShell = faces.First().Owner as Shell;
                if (facesShell != null)
                {
                    HashSet<Face> connectedFaces = Shell.ConnectedSameGeometryFaces(faces); // add all faces which have the same surface and are connected
                    try
                    {
                        if (facesShell.FeatureFromFaces(connectedFaces, out IEnumerable<Face> featureFaces, out List<Face> connection, out bool isGap))
                        {
                            // there is a feature. Multiple different features are not considered, we would need FeaturesFromFaces
                            // which would return more than one feature
                            IPropertyEntry fp = GetFeatureProperties(vw, featureFaces, connection, isGap);
                            if (fp != null && fp.SubItems.Length > 0) subEntries.Add(fp);
                        }
                    }
                    catch (Exception ex) { };
                }
            }
            // Add the menus for Faces
            if (faces.Count > 1)
            {
                SelectEntry multipleFaces = new SelectEntry("MultipleFaces.Properties", true);
                multipleFaces.IsSelected = (selected, frame) =>
                {
                    feedback.Clear();
                    if (selected)
                    {
                        feedback.FrontFaces.AddRange(faces);
                    }
                    feedback.Refresh();
                    return true;
                };
                multipleFaces.Add(GetFacesProperties(faces, vw));
                foreach (Face face in faces)
                {
                    multipleFaces.Add(GetFaceProperties(vw, face, clickBeam));
                }
                subEntries.Add(multipleFaces);
            }
            else if (faces.Any()) subEntries.Add(GetFaceProperties(vw, faces.First(), clickBeam));
            // add the menus for Edges
            if (edges.Count > 1)
            {
                SelectEntry multipleEdges = new SelectEntry("MultipleEdges.Properties", true);
                GeoObjectList select = Helper.ThickCurvesFromEdges(edges);
                multipleEdges.IsSelected = (selected, frame) =>
                {
                    feedback.Clear();
                    if (selected)
                    {
                        feedback.ShadowFaces.AddRange(select);
                    }
                    feedback.Refresh();
                    return true;
                };
                multipleEdges.Add(GetEdgesProperties(edges.Select(e => e.Curve3D).ToList()));
                foreach (Edge edg in edges)
                {
                    multipleEdges.Add(GetEdgeProperties(vw, edg, clickBeam));
                }
                subEntries.Add(multipleEdges);
            }
            else if (edges.Any()) subEntries.Add(GetEdgeProperties(vw, edges.First(), clickBeam));

            // add the menus for Solids 
            if (solids.Count > 1)
            {
                SelectEntry multipleSolids = new SelectEntry("MultipleSolids.Properties", true);
                multipleSolids.IsSelected = (selected, frame) =>
                {
                    feedback.Clear();
                    if (selected)
                    {
                        feedback.FrontFaces.AddRange(solids);
                    }
                    feedback.Refresh();
                    return true;
                };
                multipleSolids.Add(GetSolidsProperties(solids, vw));
                foreach (Solid sld in solids)
                {
                    multipleSolids.Add(GetSolidProperties(vw, sld, solidToFaces.TryGetValue(sld, out List<Face> found) ? found : null));
                }
                subEntries.Add(multipleSolids);
            }
            else if (solids.Any()) subEntries.Add(GetSolidProperties(vw, solids.First(), solidToFaces.TryGetValue(solids.First(), out List<Face> found) ? found : null));

            bool suppresRuledSolid = false;
            switch (curves.Count)
            {
                case 0: break;
                case 1:
                    subEntries.Add(GetCurveProperties(vw, curves[0], suppresRuledSolid));
                    break;
                default: // which is >1, but only in C# Version 9
                    SelectEntry multipleCurves = new SelectEntry("MultipleCurves.Properties", true);
                    GeoObjectList select = Helper.ThickCurvesFromCurves(curves);
                    multipleCurves.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        if (selected)
                        {
                            feedback.ShadowFaces.AddRange(select);
                        }
                        feedback.Refresh();
                        return true;
                    };
                    multipleCurves.Add(GetCurvesProperties(curves.ToList(), vw));
                    foreach (ICurve crv in curves)
                    {
                        multipleCurves.Add(GetCurveProperties(vw, crv, true));
                    }
                    subEntries.Add(multipleCurves);
                    break;
            }
            for (int i = 0; i < texts.Count; i++)
            {
                AddTextProperties(vw, texts[i]);
            }



            IPropertyPage pp = propertyPage;
            if (pp != null)
            {
                // pp.Refresh(this); // doesn't do the job, so we must remove and add
                pp.Remove(this); // to reflect this newly composed entry
                pp.Add(this, true);
                for (int i = 0; i < subEntries.Count; ++i)
                {
                    pp.OpenSubEntries(subEntries[i], false);
                }
                for (int i = 0; i < subEntries.Count; i++)
                {
                    if (resourceIdOfEntryToSelect.Contains(subEntries[i].ResourceId))
                    {
                        pp.OpenSubEntries(subEntries[i], true);
                        if (subEntries[i].ResourceId == "MenuId.CurveMenus")
                        {   // there is only a single curve selected: also open the curve properties to show the hotspots
                            for (int j = 0; j < subEntries[i].SubItems.Length; j++)
                            {
                                if (subEntries[i].SubItems[j] is IDisplayHotSpots)
                                {
                                    pp.OpenSubEntries(subEntries[i].SubItems[j], true);
                                }
                            }
                        }
                        else
                        {
                            pp.SelectEntry(subEntries[i]);
                        }
                        break;
                    }
                }
            }
        }

        private IPropertyEntry[] GetFacesProperties(List<Face> faces, IView vw)
        {
            List<IPropertyEntry> res = new List<IPropertyEntry>();
            Shell facesShell = faces.First().Owner as Shell;
            if (facesShell != null)
            {
                HashSet<Face> connectedFaces = Shell.ConnectedSameGeometryFaces(faces); // add all faces which have the same surface and are connected
                if (facesShell.FeatureFromFaces(connectedFaces, out IEnumerable<Face> featureFaces, out List<Face> connection, out bool isGap))
                {
                    // there is a feature. Multiple different features are not considered, we would need FeaturesFromFaces
                    // which would return more than one feature
                    try
                    {
                        IPropertyEntry fp = GetFeatureProperties(vw, featureFaces, connection, isGap);
                        if (fp != null && fp.SubItems.Length > 0) res.Add(fp);
                    }
                    catch (Exception ex) { } // TODO: there should be no exceptions, but sometimes are: check!
                }
            }
            return res.ToArray();
        }
        private IPropertyEntry[] GetSolidsProperties(List<Solid> solids, IView vw)
        {
            List<IPropertyEntry> res = new List<IPropertyEntry>();

            res.Add(ModifyMenu(solids));

            DirectMenuEntry exportSTL = new DirectMenuEntry("MenuId.Export.Solid"); // export this solids to a stl or step file
            exportSTL.ExecuteMenu = (frame) =>
            {   // show open file dialog
                string filter = StringTable.GetString("File.STEP.Filter") + "|" + StringTable.GetString("File.STL.Filter");

                int filterIndex = 1;
                string filename = null;
                if (cadFrame.UIService.ShowSaveFileDlg("MenuId.Export", StringTable.GetString("MenuId.Export"), filter, ref filterIndex, ref filename) == DialogResult.OK
                    && filename != null)
                {
                    switch (filterIndex)
                    {
                        case 1:
                            {
                                Project stepProject = Project.CreateSimpleProject();
                                Model model = stepProject.GetActiveModel();
                                foreach (Solid sld in solids)
                                {
                                    model.Add(sld.Clone()); // add a clone, otherwise sld will be removed from the current model
                                }
                                ExportStep exportStep = new ExportStep();
                                exportStep.WriteToFile(filename, stepProject);
                            }
                            break;
                        case 2:
                            using (PaintToSTL pstl = new PaintToSTL(filename, Settings.GlobalSettings.GetDoubleValue("Export.STL.Precision", 0.005)))
                            {
                                pstl.Init();
                                foreach (Solid sld in solids)
                                {
                                    sld.PaintTo3D(pstl);
                                }
                            }
                            break;
                    }
                }
                return true;
            };
            exportSTL.IsSelected = (selected, frame) =>
            {   // this is the standard selection behaviour for BRep operations
                feedback.Clear();
                if (selected)
                {
                    feedback.ShadowFaces.AddRange(solids);
                }
                feedback.Refresh();
                return true;
            };
            res.Add(exportSTL);

            DirectMenuEntry mhhide = new DirectMenuEntry("MenuId.Solid.Hide"); // hide this solid
            mhhide.ExecuteMenu = (frame) =>
            {
                foreach (Solid sld in solids)
                {
                    if (!sld.UserData.ContainsData("CADability.OriginalLayer"))
                    {
                        if (sld.Layer == null)
                        {
                            Style sldstl = frame.Project.StyleList.GetDefault(Style.EDefaultFor.Solids);
                            if (sldstl != null && sldstl.Layer != null) sld.Layer = sldstl.Layer;
                        }
                        sld.UserData.Add("CADability.OriginalLayer", sld.Layer);
                    }
                    Layer layer = frame.Project.LayerList.CreateOrFind("CADability.Hidden");
                    sld.Layer = layer;
                    if (frame.ActiveView is ModelView mv) mv.SetLayerVisibility(layer, false);
                }
                ComposeModellingEntries(new GeoObjectList(), vw, null);
                return true;
            };
            res.Add(mhhide);

            return res.ToArray();
        }

        private IPropertyEntry[] GetEdgesProperties(List<ICurve> curves)
        {
            List<IPropertyEntry> res = new List<IPropertyEntry>();
            List<Edge> edges = new List<Edge>(curves.Select(c => (c as IGeoObject).Owner as Edge));
            DirectMenuEntry makeCurves = new DirectMenuEntry("MenuId.EdgesToCurves"); // too bad, no icon yet, 
            makeCurves.ExecuteMenu = (frame) =>
            {
                GeoObjectList allEdgesAsCurves = new GeoObjectList(curves.Select(c => (c as IGeoObject).Clone())); // need to clone, since the curves ARE the actual edges!
                Style stl = frame.Project.StyleList.GetDefault(Style.EDefaultFor.Curves);
                if (stl != null)
                {
                    foreach (IGeoObject go in allEdgesAsCurves) go.SetNamedAttribute("Style", stl);
                }
                frame.Project.GetActiveModel().Add(allEdgesAsCurves);
                ComposeModellingEntries(allEdgesAsCurves, frame.ActiveView, null, false, false); // no parents, replace selection
                return true;
            };
            makeCurves.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                if (selected) feedback.FrontFaces.AddRange(curves.OfType<IGeoObject>());
                feedback.Refresh();
                return true;
            };
            res.Add(makeCurves);
            DirectMenuEntry makeFillet = new DirectMenuEntry("MenuId.Fillet"); // too bad, no icon yet, 
            makeFillet.ExecuteMenu = (frame) =>
            {
                frame.SetAction(new Constr3DFillet(edges));
                return true;
            };
            makeFillet.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                if (selected) feedback.FrontFaces.AddRange(curves.OfType<IGeoObject>());
                feedback.Refresh();
                return true;
            };
            res.Add(makeFillet);
            return res.ToArray();
        }

        private IPropertyEntry[] GetCurvesProperties(List<ICurve> curves, IView vw)
        {
            List<IPropertyEntry> res = new List<IPropertyEntry>();
            if (curves.Count > 1)
            {   // here we work with the original curves. MakePath in GetMakepath needs them to remove them properly from the model, when a path is created
                if (CanMakePath(curves)) res.Add(GetMakePath(vw, curves));
            }
            
            res.Add(ModifyMenu(curves.OfType<IGeoObject>())); 

            MultiObjectsProperties mop = new MultiObjectsProperties(cadFrame, new GeoObjectList(curves.OfType<IGeoObject>()));
            res.Add(mop.attributeProperties);
            // check the condition to make a ruled solid:
            // two closed curves not in the same plane
            // we may not use the curves direct, because adding them to a path would remove them from the model
            for (int i = 0; i < curves.Count; i++) curves[i] = curves[i].Clone();
            List<Path> paths = Path.FromSegments(curves);
            // if we have two paths which are flat but not in the same plane, we could make a ruled solid directly
            // if we need more user control, e.g. specifying synchronous points on each path, we woould need a more
            // sophisticated action
            if (paths.Count == 2 && paths[0].GetPlanarState() == PlanarState.Planar && paths[1].GetPlanarState() == PlanarState.Planar)
            {
                if (!Precision.IsEqual(paths[0].GetPlane(), paths[1].GetPlane()) && paths[0].IsClosed && paths[1].IsClosed)
                {
                    try
                    {
                        Solid sld = Make3D.MakeRuledSolid(paths[0], paths[1], cadFrame.Project);
                        if (sld != null)
                        {
                            DirectMenuEntry makeRuledSolid = new DirectMenuEntry("MenuId.Constr.Solid.RuledSolid");
                            makeRuledSolid.IsSelected = (selected, frame) =>
                            {
                                feedback.Clear();
                                if (selected) feedback.FrontFaces.Add(sld);
                                feedback.Refresh();
                                return true;
                            };
                            makeRuledSolid.ExecuteMenu = (frame) =>
                            {
                                frame.Project.GetActiveModel().Add(sld);
                                return true;
                            };
                            res.Add(makeRuledSolid);
                        }
                    }
                    catch (NotImplementedException) { }
                }
            }
            if (paths.Count == 1 && paths[0].IsClosed && paths[0].GetPlanarState() == PlanarState.Planar)
            {
                Plane pln = paths[0].GetPlane();
                Face fc = Face.MakeFace(new GeoObjectList(paths[0]));
                if (fc != null)
                {
                    DirectMenuEntry extrude = new DirectMenuEntry("MenuId.Constr.Solid.FaceExtrude"); // too bad, no icon yet, would be 159
                    extrude.ExecuteMenu = (frame) =>
                    {
                        frame.SetAction(new Constr3DFaceExtrude(fc));
                        return true;
                    };
                    res.Add(extrude);
                    DirectMenuEntry rotate = new DirectMenuEntry("MenuId.Constr.Solid.FaceRotate"); // too bad, no icon yet, would be 160
                    rotate.ExecuteMenu = (frame) =>
                    {
                        frame.SetAction(new Constr3DFaceRotate(new GeoObjectList(fc)));
                        return true;
                    };
                    res.Add(rotate);
                }
            }
            return res.ToArray();
        }
        /// <summary>
        /// Empty the modelling tab page, leave only the title
        /// </summary>
        private void Clear()
        {
            IPropertyPage pp = propertyPage;
            if (pp != null)
            {
                IPropertyEntry selection = pp.GetCurrentSelection();
                if (selection != null)
                {
                    selection.UnSelected(selection); // to regenerate the feedback display
                                                     // and by passing selected as "previousSelected" parameter, they can only regenerate projection dependant feedback 
                }
                subEntries.Clear();
                pp.Remove(this); // to reflect this newly composed entry
                pp.Add(this, true);
            }
            activeHotspots.Clear();
            feedback.hotSpots.Clear();
            feedback.Clear();
            feedback.Refresh();
        }


        private IPropertyEntry GetCurveProperties(IView vw, ICurve curve, bool suppresRuledSolid)
        {
            SelectEntry curveMenus = new SelectEntry("MenuId.CurveMenus", true);
            curveMenus.LabelText = (curve as IGeoObject).Description;
            curveMenus.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                if (selected)
                {
                    feedback.ShadowFaces.Add(curve as IGeoObject);
                }
                feedback.Refresh();

                return true;
            };
            if ((curve as IGeoObject).Owner is Model)
            {
                SelectEntry modifyMenu = ModifyMenu(new IGeoObject[] { curve as IGeoObject });
                curveMenus.TestShortcut = (key) =>
                {
                    if (key == Keys.Delete)
                    {
                        cadFrame.ActiveView.Model.Remove(curve as IGeoObject);
                        Clear();
                        return true;
                    }
                    return false;
                };
                curveMenus.Add(modifyMenu);
                IPropertyEntry curveProperties = (curve as IGeoObject).GetShowProperties(cadFrame);
                if (curveProperties is IDisplayHotSpots dh) dh.HotspotChangedEvent += HotspotChanged;
                curveMenus.Add(curveProperties);
            }
            if (curve.IsClosed && curve.GetPlanarState() == PlanarState.Planar)
            {
                Plane plane = curve.GetPlane();
                Face fc = Face.MakeFace(new GeoObjectList(curve as IGeoObject));
                if (fc != null)
                {
                    DirectMenuEntry extrude = new DirectMenuEntry("MenuId.Constr.Solid.FaceExtrude"); // too bad, no icon yet, would be 159
                    extrude.ExecuteMenu = (frame) =>
                    {
                        Constr3DFaceExtrude action = new Constr3DFaceExtrude(fc);
                        action.ActionDoneEvent += (ConstructAction ca, bool success) =>
                        {   // remove the curve from which the extrusion was made
                            if (success)
                            {
                                cadFrame.ActiveView.Model.Remove(curve as IGeoObject);
                            }
                        };
                        frame.SetAction(action);
                        return true;
                    };
                    extrude.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        if (selected) feedback.ShadowFaces.Add(fc);
                        feedback.Refresh();
                        return true;
                    };
                    curveMenus.Add(extrude);
                    DirectMenuEntry rotate = new DirectMenuEntry("MenuId.Constr.Solid.FaceRotate"); // too bad, no icon yet, would be 160
                    rotate.ExecuteMenu = (frame) =>
                    {
                        Constr3DFaceRotate action = new Constr3DFaceRotate(new GeoObjectList(fc));
                        action.ActionDoneEvent += (ConstructAction ca, bool success) =>
                        {   // remove the curve from which the extrusion was made
                            if (success)
                            {
                                cadFrame.ActiveView.Model.Remove(curve as IGeoObject);
                            }
                        };
                        frame.SetAction(action);
                        return true;
                    };
                    rotate.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        if (selected) feedback.ShadowFaces.Add(fc);
                        feedback.Refresh();
                        return true;
                    };
                    curveMenus.Add(rotate);
                }
                if (!suppresRuledSolid)
                {
                    DirectMenuEntry ruled = new DirectMenuEntry("MenuId.Constr.Solid.RuledSolid"); // too bad, no icon yet, would be 161
                    ruled.ExecuteMenu = (frame) =>
                    {
                        Constr3DRuledSolid action = new Constr3DRuledSolid(new GeoObjectList(curve as IGeoObject), frame);
                        action.ActionDoneEvent += (ConstructAction ca, bool success) =>
                        {   // remove the curve from which the extrusion was made
                            if (success)
                            {
                                cadFrame.ActiveView.Model.Remove(curve as IGeoObject);
                            }
                        };
                        frame.SetAction(action);
                        return true;
                    };
                    curveMenus.Add(ruled);
                }
            }
            return curveMenus;
        }

        private void HotspotChanged(IHotSpot sender, HotspotChangeMode mode)
        {
            switch (mode)
            {
                case HotspotChangeMode.Visible:
                    activeHotspots.Add(sender);
                    break;
                case HotspotChangeMode.Invisible:
                    activeHotspots.Remove(sender);
                    break;
                case HotspotChangeMode.Selected:
                    selectedHotspot = sender;
                    break;
                case HotspotChangeMode.Deselected:
                    if (selectedHotspot == sender) selectedHotspot = null;
                    break;
                case HotspotChangeMode.Moved:
                    break;
            }
            feedback.hotSpots = activeHotspots.ToList();
            feedback.Refresh();
        }

        private void AddTextProperties(IView vw, Text text)
        {
            SelectEntry textMenus = new SelectEntry("MenuId.TextMenus", true);
            textMenus.LabelText = (text as IGeoObject).Description;
            textMenus.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                if (selected)
                {
                    feedback.ShadowFaces.Add(text as IGeoObject);
                }
                feedback.Refresh();

                return true;
            };
            if ((text as IGeoObject).Owner is Model)
            {
                SelectEntry modifyMenu = ModifyMenu(new IGeoObject[]{ text});
                textMenus.TestShortcut = (key) =>
                {
                    if (key == Keys.Delete)
                    {
                        cadFrame.ActiveView.Model.Remove(text as IGeoObject);
                        Clear();
                        return true;
                    }
                    return false;
                };
                textMenus.Add(modifyMenu);
                textMenus.Add((text as IGeoObject).GetShowProperties(cadFrame));
            }
            DirectMenuEntry extrude = new DirectMenuEntry("MenuId.Constr.Solid.FaceExtrude"); // too bad, no icon yet, would be 159
            extrude.ExecuteMenu = (frame) =>
            {
                Constr3DFaceExtrude action = new Constr3DFaceExtrude(new GeoObjectList(text));
                action.ActionDoneEvent += (ConstructAction ca, bool success) =>
                {   // remove the curve from which the extrusion was made
                    if (success)
                    {
                        cadFrame.ActiveView.Model.Remove(text);
                    }
                };
                frame.SetAction(action);
                return true;
            };
            extrude.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                if (selected) feedback.ShadowFaces.Add(text);
                feedback.Refresh();
                return true;
            };
            textMenus.Add(extrude);
            DirectMenuEntry rotate = new DirectMenuEntry("MenuId.Constr.Solid.FaceRotate"); // too bad, no icon yet, would be 160
            rotate.ExecuteMenu = (frame) =>
            {
                Constr3DFaceRotate action = new Constr3DFaceRotate(new GeoObjectList(new GeoObjectList(text)));
                action.ActionDoneEvent += (ConstructAction ca, bool success) =>
                {   // remove the curve from which the extrusion was made
                    if (success)
                    {
                        cadFrame.ActiveView.Model.Remove(text);
                    }
                };
                frame.SetAction(action);
                return true;
            };
            rotate.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                if (selected) feedback.ShadowFaces.Add(text);
                feedback.Refresh();
                return true;
            };
            textMenus.Add(rotate);
            subEntries.Add(textMenus);
        }
        /// <summary>
        /// Creates an entry for general modification like move, rotat, reflect for all provided objects
        /// </summary>
        /// <param name="gos"></param>
        /// <returns></returns>
        private SelectEntry ModifyMenu(IEnumerable<IGeoObject> gos)
        {
            SelectEntry modifyMenu = new SelectEntry("MenuId.GeneralModifications");
            GeoObjectList capturedList = new GeoObjectList(gos);
            modifyMenu.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                if (selected)
                {
                    feedback.ShadowFaces.AddRange(capturedList);
                }
                feedback.Refresh();
                return true;
            };
            modifyMenu.TestShortcut = (key) =>
            {   // directly delete the selected object
                if (key == Keys.Delete)
                {
                    foreach (IGeoObject go in capturedList)
                    {
                        cadFrame.ActiveView.Model.Remove(go);
                    }
                    Clear();
                    return true;
                }
                return false;
            };

            MenuWithHandler move = new MenuWithHandler("MenuId.Object.Move");
            move.OnCommand = (e) =>
            {
                Clear();
                cadFrame.SetAction(new MoveObjects(capturedList));
                return true;
            };
            MenuWithHandler rotate = new MenuWithHandler("MenuId.Object.Rotate");
            rotate.OnCommand = (e) =>
            {
                Clear();
                cadFrame.SetAction(new RotateObjects(capturedList));
                return true;
            };
            MenuWithHandler scale = new MenuWithHandler("MenuId.Object.Scale");
            scale.OnCommand = (e) =>
            {
                Clear();
                cadFrame.SetAction(new ScaleObjects(capturedList));
                return true;
            };
            MenuWithHandler reflect = new MenuWithHandler("MenuId.Object.Reflect");
            reflect.OnCommand = (e) =>
            {
                Clear();
                cadFrame.SetAction(new ReflectObjects(capturedList));
                return true;
            };
            MenuWithHandler copy = new MenuWithHandler("MenuId.Edit.Copy");
            copy.OnCommand = (e) =>
            {
                cadFrame.UIService.SetClipboardData(capturedList, true);
                return true;
            };
            MenuWithHandler delete = new MenuWithHandler("MenuId.Object.Delete");
            delete.OnCommand = (e) =>
            {
                cadFrame.ActiveView.Model.Remove(capturedList);
                Clear();
                return true;
            };
            modifyMenu.Menu = new MenuWithHandler[] { move, rotate, scale, reflect, MenuWithHandler.Separator, copy, delete };
            return modifyMenu;
        }

        private IPropertyEntry GetMakePath(IView vw, List<ICurve> curves)
        {
            List<ICurve> cc = new List<ICurve>(curves); // the captured list of curves for the following lambdas
            DirectMenuEntry mp = new DirectMenuEntry("MenuId.Object.MakePath");
            mp.IsSelected = (selected, frame) =>
            {   // show the selected curves as feedback
                feedback.Clear();
                if (selected)
                {
                    for (int i = 0; i < cc.Count; i++)
                    {
                        feedback.FrontFaces.Add(cc[i] as IGeoObject);
                    }
                }
                vw.Invalidate(PaintBuffer.DrawingAspect.Select, vw.DisplayRectangle);
                return true;
            };
            mp.ExecuteMenu = (frame) =>
            {
                using (frame.Project.Undo.UndoFrame)
                {
                    List<ICurve> useForPathCreation = new List<ICurve>(cc);
                    List<Path> created = Path.FromSegments(useForPathCreation); // all curves used for a path are removed from useForPathCreation
                    for (int i = 0; i < cc.Count; i++)
                    {   // rmeove all those curves, which have been used in the created paths
                        if (!useForPathCreation.Contains(cc[i])) vw.Model.Remove(cc[i] as IGeoObject);
                    }
                    vw.Model.Add(created.ToArray());
                    ComposeModellingEntries(new GeoObjectList(created as IEnumerable<IGeoObject>), vw, null);
                }
                return true;
            };
            return mp;
        }

        private IPropertyEntry GetSolidProperties(IView vw, Solid sld, List<Face> fromFaces)
        {
            SelectEntry solidMenus = new SelectEntry("MenuId.Solid", true); // the container for all menus or properties of the solid
            solidMenus.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                if (selected) feedback.ShadowFaces.Add(sld);
                vw.Invalidate(PaintBuffer.DrawingAspect.Select, vw.DisplayRectangle);
                return true;
            };
            if (sld.Owner is Model)
            {   // switch to selection tab and use "old" modification functions like move and delete
                solidMenus.Add(ModifyMenu(new IGeoObject[] { sld }));
                // here we process the delete key!
                solidMenus.TestShortcut = (key) =>
                {
                    if (key == Keys.Delete)
                    {
                        cadFrame.ActiveView.Model.Remove(sld);
                        Clear();
                        return true;
                    }
                    return false;
                };
                IPropertyEntry solidProperties = sld.GetShowProperties(cadFrame);
                // wish I could rename the Label here to something like "Properties"
                solidMenus.Add(solidProperties);
            }

            if (fromFaces != null)
            {
                foreach (Face fc in fromFaces)
                {   // build the menu with extrusions and rotations
                    solidMenus.Add(GetExtensionProperties(fc, lastPickArea, vw).ToArray());
                }
            }
            // Split by plane is always possible
            DirectMenuEntry splitByPlane = new DirectMenuEntry("MenuId.Solid.SplitByPlane");
            splitByPlane.ExecuteMenu = (frame) =>
            {
                cadFrame.SetAction(new SplitSolidByPlane(sld));
                this.Clear();
                return true;
            };
            splitByPlane.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                if (selected) feedback.ShadowFaces.Add(sld);
                feedback.Refresh();
                return true;
            };
            solidMenus.Add(splitByPlane);
            // Make intersection lines or path
            DirectMenuEntry planeIntersectionPath = new DirectMenuEntry("MenuId.Solid.PlaneIntersetionPath");
            planeIntersectionPath.ExecuteMenu = (frame) =>
            {
                cadFrame.SetAction(new SolidPlaneIntersectionPathAction(sld));
                this.Clear();
                return true;
            };
            planeIntersectionPath.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                if (selected) feedback.ShadowFaces.Add(sld);
                feedback.Refresh();
                return true;
            };
            solidMenus.Add(planeIntersectionPath);


            Model owner = sld.Owner as Model;
            if (owner != null)
            {
                GeoObjectList fromBox = owner.GetObjectsFromBox(sld.GetExtent(0.0));
                List<Solid> otherSolids = new List<Solid>();
                for (int i = 0; i < fromBox.Count; i++)
                {
                    if (fromBox[i] is Solid solid && sld != solid) otherSolids.Add(solid);
                }
                if (otherSolids.Count > 0)
                {   // there are other solids close to this solid, it is not guaranteed that these other solids interfere with this solid 
                    Func<bool, IFrame, bool> ShowThisAndAll = (selected, frame) =>
                    {   // this is the standard selection behaviour for BRep operations
                        feedback.Clear();
                        if (selected)
                        {
                            feedback.ShadowFaces.Add(sld);
                            feedback.BackFaces.AddRange(otherSolids);
                        }
                        feedback.Refresh();
                        return true;
                    };
                    Func<bool, IFrame, bool> ShowThis = (selected, frame) =>
                    {   // this is the standard selection behaviour for BRep operations
                        feedback.Clear();
                        if (selected)
                        {
                            feedback.ShadowFaces.Add(sld);
                        }
                        feedback.Refresh();
                        return true;
                    };
                    BRepOpWith bRepOp = new BRepOpWith(sld, otherSolids, cadFrame); // it is all implemented in BRepOpWith, so we use the ICommandHandler to forward it
                    DirectMenuEntry mhSubtractFrom = new DirectMenuEntry("MenuId.Solid.RemoveFrom");
                    mhSubtractFrom.ExecuteMenu = (frame) =>
                    {
                        this.Clear();
                        frame.SetAction(new SelectSecondSolidAction(sld, BRepOperation.Operation.difference));
                        return true;
                    };
                    mhSubtractFrom.IsSelected = ShowThis;
                    solidMenus.Add(mhSubtractFrom);
                    DirectMenuEntry mhSubtractFromAll = new DirectMenuEntry("MenuId.Solid.RemoveFromAll");
                    mhSubtractFromAll.ExecuteMenu = (frame) =>
                    {
                        this.Clear();
                        (bRepOp as ICommandHandler).OnCommand("MenuId.Solid.RemoveFromAll");
                        return true;
                    };
                    mhSubtractFromAll.IsSelected = ShowThisAndAll;
                    solidMenus.Add(mhSubtractFromAll);
                    DirectMenuEntry mhSubtractAllFromThis = new DirectMenuEntry("MenuId.Solid.RemoveAll");
                    mhSubtractAllFromThis.ExecuteMenu = (frame) =>
                    {
                        this.Clear();
                        (bRepOp as ICommandHandler).OnCommand("MenuId.Solid.RemoveAll");
                        return true;
                    };
                    mhSubtractAllFromThis.IsSelected = ShowThisAndAll;
                    solidMenus.Add(mhSubtractAllFromThis);
                    DirectMenuEntry mhUniteWith = new DirectMenuEntry("MenuId.Solid.UniteWith");
                    mhUniteWith.ExecuteMenu = (frame) =>
                    {
                        this.Clear();
                        frame.SetAction(new SelectSecondSolidAction(sld, BRepOperation.Operation.union));
                        return true;
                    };
                    mhUniteWith.IsSelected = ShowThis;
                    solidMenus.Add(mhUniteWith);
                    DirectMenuEntry mhUniteWithAll = new DirectMenuEntry("MenuId.Solid.UniteWithAll");
                    mhUniteWithAll.ExecuteMenu = (frame) =>
                    {
                        this.Clear();
                        (bRepOp as ICommandHandler).OnCommand("MenuId.Solid.UniteWithAll");
                        return true;
                    };
                    mhUniteWithAll.IsSelected = ShowThisAndAll;
                    solidMenus.Add(mhUniteWithAll);
                    DirectMenuEntry mhIntersectWith = new DirectMenuEntry("MenuId.Solid.IntersectWith");
                    mhIntersectWith.ExecuteMenu = (frame) =>
                    {
                        this.Clear();
                        frame.SetAction(new SelectSecondSolidAction(sld, BRepOperation.Operation.intersection));
                        return true;
                    };
                    mhIntersectWith.IsSelected = ShowThis;
                    solidMenus.Add(mhIntersectWith);
                    DirectMenuEntry mhIntersectWithAll = new DirectMenuEntry("MenuId.Solid.IntersectWithAll");
                    mhIntersectWithAll.ExecuteMenu = (frame) =>
                    {
                        this.Clear();
                        (bRepOp as ICommandHandler).OnCommand("MenuId.Solid.IntersectWithAll");
                        return true;
                    };
                    mhIntersectWithAll.IsSelected = ShowThisAndAll;
                    solidMenus.Add(mhIntersectWithAll);
                    DirectMenuEntry mhSplitWith = new DirectMenuEntry("MenuId.Solid.SplitWith");
                    mhSplitWith.ExecuteMenu = (frame) =>
                    {
                        this.Clear();
                        frame.SetAction(new SelectSecondSolidAction(sld, BRepOperation.Operation.clip));
                        return true;
                    };
                    mhSplitWith.IsSelected = ShowThis;
                    solidMenus.Add(mhSplitWith);
                    DirectMenuEntry mhSplitWithAll = new DirectMenuEntry("MenuId.Solid.SplitWithAll");
                    mhSplitWithAll.ExecuteMenu = (frame) =>
                    {
                        this.Clear();
                        (bRepOp as ICommandHandler).OnCommand("MenuId.Solid.SplitWithAll");
                        return true;
                    };
                    mhSplitWithAll.IsSelected = ShowThisAndAll;
                    solidMenus.Add(mhSplitWithAll);
                }
            }
            DirectMenuEntry exportSTL = new DirectMenuEntry("MenuId.Export.Solid"); // export this solid to a stl file
            exportSTL.ExecuteMenu = (frame) =>
            {   // show open file dialog
                string filter = StringTable.GetString("File.STEP.Filter") + "|" + StringTable.GetString("File.STL.Filter");

                int filterIndex = 1;
                string filename = null;
                if (cadFrame.UIService.ShowSaveFileDlg("MenuId.Export", StringTable.GetString("MenuId.Export"), filter, ref filterIndex, ref filename) == DialogResult.OK
                    && filename != null)
                {
                    switch (filterIndex)
                    {
                        case 1:
                            {
                                Project stepProject = Project.CreateSimpleProject();
                                Model model = stepProject.GetActiveModel();
                                model.Add(sld.Clone()); // add a clone, otherwise sld will be removed from the current model
                                ExportStep exportStep = new ExportStep();
                                exportStep.WriteToFile(filename, stepProject);
                            }
                            break;
                        case 2:
                            using (PaintToSTL pstl = new PaintToSTL(filename, Settings.GlobalSettings.GetDoubleValue("Export.STL.Precision", 0.005)))
                            {
                                pstl.Init();
                                sld.PaintTo3D(pstl);
                            }
                            break;
                    }
                }
                return true;
            };
            exportSTL.IsSelected = (selected, frame) =>
            {   // this is the standard selection behaviour for BRep operations
                feedback.Clear();
                if (selected)
                {
                    feedback.ShadowFaces.Add(sld);
                }
                feedback.Refresh();
                return true;
            };
            solidMenus.Add(exportSTL);
            if (sld.Layer != null && sld.Layer.Name == "CADability.Transparent")
            {
                DirectMenuEntry mhtr = new DirectMenuEntry("MenuId.MakeOpaque");
                mhtr.ExecuteMenu = (frame) =>
                {   // reset the layer of this solid
                    Layer layer = sld.UserData.GetData("CADability.OriginalLayer") as Layer;
                    if (layer != null)
                    {
                        sld.Layer = layer;
                        sld.UserData.RemoveUserData("CADability.OriginalLayer");
                    }
                    else
                    {
                        Style sldstl = frame.Project.StyleList.GetDefault(Style.EDefaultFor.Solids);
                        if (sldstl != null && sldstl.Layer != null) sld.Layer = sldstl.Layer;
                    }
                    return true;
                };
                solidMenus.Add(mhtr);
            }
            else
            {
                DirectMenuEntry mhtr = new DirectMenuEntry("MenuId.MakeTransparent");
                mhtr.ExecuteMenu = (frame) =>
                {   // reset the layer of this solid
                    sld.UserData.Add("CADability.OriginalLayer", sld.Layer);
                    Layer layer = frame.Project.LayerList.CreateOrFind("CADability.Transparent");
                    layer.Transparency = 128; // should be configurable
                    sld.Layer = layer;
                    return true;
                };
                solidMenus.Add(mhtr);
            }
            DirectMenuEntry mhhide = new DirectMenuEntry("MenuId.Solid.Hide"); // hide this solid
            mhhide.ExecuteMenu = (frame) =>
            {
                if (!sld.UserData.ContainsData("CADability.OriginalLayer"))
                {
                    if (sld.Layer == null)
                    {
                        Style sldstl = frame.Project.StyleList.GetDefault(Style.EDefaultFor.Solids);
                        if (sldstl != null && sldstl.Layer != null) sld.Layer = sldstl.Layer;
                    }
                    sld.UserData.Add("CADability.OriginalLayer", sld.Layer);
                }
                Layer layer = frame.Project.LayerList.CreateOrFind("CADability.Hidden");
                sld.Layer = layer;
                if (frame.ActiveView is ModelView mv) mv.SetLayerVisibility(layer, false);
                ComposeModellingEntries(new GeoObjectList(), vw, null);
                return true;
            };
            solidMenus.Add(mhhide);

            //MenuWithHandler mhsel = new MenuWithHandler();
            //mhsel.ID = "MenuId.Selection.Set";
            //mhsel.Text = StringTable.GetString("MenuId.Selection.Set", StringTable.Category.label);
            //mhsel.Target = new SetSelection(this, frame.ActiveAction as SelectObjectsAction);
            //solidMenus.Add(mhsel);
            //MenuWithHandler mhadd = new MenuWithHandler();
            //mhadd.ID = "MenuId.Selection.Add";
            //mhadd.Text = StringTable.GetString("MenuId.Selection.Add", StringTable.Category.label);
            //mhadd.Target = new SetSelection(this, frame.ActiveAction as SelectObjectsAction);
            //solidMenus.Add(mhadd);
            //MenuWithHandler mhremove = new MenuWithHandler();
            //mhremove.ID = "MenuId.Remove";
            //mhremove.Text = StringTable.GetString("MenuId.Remove", StringTable.Category.label);
            //mhremove.Target = SimpleMenuCommand.HandleCommand((menuId) =>
            //{
            //    Model model = Owner as Model;
            //    if (model != null) model.Remove(this);
            //    return true;
            //});
            //solidMenus.Add(mhremove);
            return solidMenus;
        }
        private IPropertyEntry GetEdgeProperties(IView vw, Edge edg, Axis clickBeam)
        {
            SelectEntry edgeMenus = new SelectEntry("MenuId.Edge", true); // the container for all edge related menus or properties
            HashSet<Edge> edges = Shell.ConnectedSameGeometryEdges(new Edge[] { edg });
            GeoObjectList selection = Helper.ThickCurvesFromEdges(edges);
            edgeMenus.IsSelected = (selected, frame) =>
            {   // show the provided edge and the "same geometry connected" edges as feedback
                feedback.Clear();
                if (selected) feedback.ShadowFaces.AddRange(selection);
                feedback.Refresh();
                return true;
            };

            Shell owningShell = edg.PrimaryFace.Owner as Shell;

            if (edg.Curve3D is Ellipse && (edg.PrimaryFace.Surface is CylindricalSurface || edg.SecondaryFace.Surface is CylindricalSurface
                || edg.PrimaryFace.Surface is ConicalSurface || edg.SecondaryFace.Surface is ConicalSurface))
            {   // lets try to change the radius of this edge
                HashSet<Face> affectedFaces = new HashSet<Face>();
                foreach (Edge edge in edges)
                {
                    affectedFaces.Add(edge.PrimaryFace);
                    affectedFaces.Add(edge.SecondaryFace);
                }
                for (int i = 0; i < 2; ++i)
                {
                    string resourceId = string.Empty;
                    switch (i)
                    {
                        case 0: resourceId = "MenuId.Edge.Radius"; break;
                        case 1: resourceId = "MenuId.Edge.Diameter"; break;
                    }
                    DirectMenuEntry edgeRadiusOrDiameter = new DirectMenuEntry(resourceId);
                    edgeRadiusOrDiameter.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        if (selected)
                        {
                            feedback.ShadowFaces.AddRange(selection);
                            feedback.FrontFaces.AddRange(affectedFaces);
                        }
                        feedback.Refresh();
                        return true;
                    };
                    bool useDiameter = i == 1;
                    edgeRadiusOrDiameter.ExecuteMenu = (frame) =>
                    {
                        cadFrame.SetAction(new ParametricsEdgeRadiusAction(edges, GeoPoint.Invalid, useDiameter));
                        return true;
                    };
                    edgeMenus.Add(edgeRadiusOrDiameter);
                }
            }
            DirectMenuEntry mhdist = new DirectMenuEntry("MenuId.Parametrics.DistanceTo");
            // mhdist.Target = new ParametricsDistanceActionOld(edg, selectAction.Frame); 
            // TODO!
            edgeMenus.Add(mhdist);

            if (!edg.IsTangentialEdge() && edg.SecondaryFace != null)
            {
                DirectMenuEntry makeFillet = new DirectMenuEntry("MenuId.Fillet");
                makeFillet.ExecuteMenu = (frame) =>
                {
                    cadFrame.SetAction(new Constr3DFillet(edges));
                    return true;
                };
                makeFillet.IsSelected = (selected, frame) =>
                {
                    feedback.Clear();
                    if (selected) feedback.ShadowFaces.AddRange(selection);
                    feedback.Refresh();
                    return true;
                };
                edgeMenus.Add(makeFillet);

                (Axis rotationAxis1, GeoVector fromHere1, GeoVector toHere1) = ParametricsAngleAction.GetRotationAxis(edg.PrimaryFace, edg.SecondaryFace, clickBeam);
                if (rotationAxis1.IsValid)
                {
                    DirectMenuEntry rotateMenu = new DirectMenuEntry("MenuId.FaceAngle");
                    rotateMenu.ExecuteMenu = (frame) =>
                    {
                        ParametricsAngleAction pa = new ParametricsAngleAction(edg.PrimaryFace, edg.SecondaryFace, rotationAxis1, fromHere1, toHere1, edg, selectAction.Frame);
                        selectAction.Frame.SetAction(pa);
                        return true;
                    };
                    rotateMenu.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        GeoObjectList feedbackArrow = FeedbackArrow.MakeArcArrow(owningShell, rotationAxis1, fromHere1, toHere1, vw, FeedbackArrow.ArrowFlags.secondRed);
                        if (selected) feedback.Arrows.AddRange(feedbackArrow);
                        feedback.Refresh();
                        return true;
                    };
                    edgeMenus.Add(rotateMenu);
                }
                (Axis rotationAxis2, GeoVector fromHere2, GeoVector toHere2) = ParametricsAngleAction.GetRotationAxis(edg.SecondaryFace, edg.PrimaryFace, clickBeam);
                if (rotationAxis1.IsValid)
                {
                    DirectMenuEntry rotateMenu = new DirectMenuEntry("MenuId.FaceAngle");
                    rotateMenu.ExecuteMenu = (frame) =>
                    {
                        ParametricsAngleAction pa = new ParametricsAngleAction(edg.SecondaryFace, edg.PrimaryFace, rotationAxis2, fromHere2, toHere2, edg, selectAction.Frame);
                        selectAction.Frame.SetAction(pa);
                        return true;
                    };
                    rotateMenu.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        GeoObjectList feedbackArrow = FeedbackArrow.MakeArcArrow(owningShell, rotationAxis2, fromHere2, toHere2, vw, FeedbackArrow.ArrowFlags.secondRed);
                        if (selected) feedback.Arrows.AddRange(feedbackArrow);
                        feedback.Refresh();
                        return true;
                    };
                    edgeMenus.Add(rotateMenu);
                }
            }
            return edgeMenus;
        }

        private IPropertyEntry GetFaceProperties(IView vw, Face fc, Axis clickBeam)
        {
            if (!(fc.Owner is Shell)) return null;
            HashSet<Face> faces = Shell.ConnectedSameGeometryFaces(new Face[] { fc }); // in case of half cylinders etc. use the whole cylinder

            // where did the user touch the face? We need this point for the display of the dimensioning arrow
            GeoPoint2D[] ips2d = fc.Surface.GetLineIntersection(clickBeam.Location, clickBeam.Direction);
            if (ips2d.Length > 1)
            {
                double minPos = double.MaxValue;
                for (int i = 0; i < ips2d.Length; i++)
                {
                    bool isInside = false;
                    foreach (Face face in faces)
                    {
                        if (face.Contains(ref ips2d[i], true)) isInside = true;
                    }
                    if (isInside)
                    {
                        GeoPoint p = fc.Surface.PointAt(ips2d[i]);
                        double pos = Geometry.LinePar(clickBeam.Location, clickBeam.Direction, p);
                        if (pos < minPos)
                        {
                            minPos = pos;
                            ips2d[0] = ips2d[i];
                        }
                    }
                }
            }
            GeoPoint touchingPoint; // the point where to attach the dimension feedback
            if (ips2d.Length == 0) touchingPoint = fc.Surface.PointAt(fc.Area.GetSomeInnerPoint());
            else touchingPoint = fc.Surface.PointAt(ips2d[0]);

            // faceEntry: a simple group entry, which contains all face modelling menus
            SelectEntry faceEntries = new SelectEntry("MenuId.Face", true); // only handles selection
            faceEntries.IsSelected = (selected, frame) =>
            {   // show the provided face and the "same geometry connected" faces as feedback
                feedback.Clear();
                if (selected)
                {
                    feedback.ShadowFaces.AddRange(faces.ToArray());
                    selectedChildObjects.Clear();
                    selectedChildObjects.AddRange(faces);
                }
                else selectedChildObjects.Clear();
                feedback.Refresh();
                return true;
            };

            // add more menus for faces with specific surfaces
            faceEntries.Add(GetSurfaceSpecificSubmenus(fc, vw, touchingPoint).ToArray());

            if (fc.Owner is Shell owningShell)
            {
                // check whether we can mate
                Model model = owningShell.Owner as Model;
                if (model == null) model = (owningShell.Owner as Solid).Owner as Model;
                bool canMate = false;
                foreach (IGeoObject geoObject in model)
                {
                    if (geoObject is Solid && geoObject != owningShell.Owner) canMate = true;
                    if (geoObject is Shell && geoObject != owningShell) canMate = true;
                    if (canMate) break;
                }
                if (canMate)
                {
                    DirectMenuEntry mate = new DirectMenuEntry("MenuId.Mate");
                    mate.ExecuteMenu = (frame) =>
                    {
                        MateFacesAction po = new MateFacesAction(fc);
                        cadFrame.SetAction(po);
                        return true;
                    };
                    mate.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        if (selected)
                        {
                            feedback.FrontFaces.Add(fc);
                        }
                        feedback.Refresh();
                        return true;
                    };
                    faceEntries.Add(mate);
                }

                // can we find a thickness or gauge in the shell?
                double thickness = owningShell.GetGauge(fc, out HashSet<Face> frontSide, out HashSet<Face> backSide);
                if (thickness != double.MaxValue && thickness > 0.0 && frontSide.Count > 0)
                {
                    DirectMenuEntry gauge = new DirectMenuEntry("MenuId.Gauge");
                    gauge.ExecuteMenu = (frame) =>
                    {
                        ParametricsOffsetAction po = new ParametricsOffsetAction(frontSide, backSide, cadFrame, fc, touchingPoint, thickness);
                        selectAction.Frame.SetAction(po);
                        return true;
                    };
                    gauge.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        if (selected)
                        {
                            feedback.FrontFaces.AddRange(frontSide.ToArray());
                            feedback.FrontFaces.AddRange(backSide.ToArray());
                            feedback.Arrows.AddRange(FeedbackArrow.MakeLengthArrow(owningShell, fc, backSide.First(), fc, fc.Surface.GetNormal(fc.Surface.PositionOf(touchingPoint)), touchingPoint, vw, FeedbackArrow.ArrowFlags.secondRed));
                        }
                        feedback.Refresh();
                        return true;
                    };
                    faceEntries.Add(gauge);
                }
                // is there a way to center these faces in the shell?
                // betterprovide a test, whether this is possible at all
                {
                    DirectMenuEntry center = new DirectMenuEntry("MenuId.Center");
                    center.ExecuteMenu = (frame) =>
                    {
                        ParametricsCenterAction pca = new ParametricsCenterAction(faces);
                        selectAction.Frame.SetAction(pca);
                        return true;
                    };
                    center.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        if (selected)
                        {
                            feedback.ShadowFaces.AddRange(faces.ToArray());
                        }
                        feedback.Refresh();
                        return true;
                    };
                    faceEntries.Add(center);
                }
                // a menu to allow the distance to another face or edge be selected in the action
                DirectMenuEntry faceDistTo = new DirectMenuEntry("MenuId.FaceDistanceTo");
                faceDistTo.ExecuteMenu = (frame) =>
                {
                    ParametricPositionAction pp = new ParametricPositionAction(faces, fc, touchingPoint);
                    selectAction.Frame.SetAction(pp);
                    return true;
                };
                faceDistTo.IsSelected = (selected, frame) =>
                {
                    feedback.Clear();
                    if (selected)
                    {
                        feedback.FrontFaces.Add(fc);
                    }
                    feedback.Refresh();
                    return true;
                };
                faceEntries.Add(faceDistTo);

                // what distances on the face can we find
                int n = owningShell.GetFaceDistances(fc, touchingPoint, out List<Face> distanceTo, out List<double> distance, out List<GeoPoint> pointsFrom, out List<GeoPoint> pointsTo);
                for (int j = 0; j < n; j++)
                {
                    if (backSide == null || !backSide.Contains(distanceTo[j])) // this is not already used as gauge
                    {
                        HashSet<Face> capturedFaceI = new HashSet<Face>(new Face[] { fc });
                        HashSet<Face> capturedDistTo = new HashSet<Face>(new Face[] { distanceTo[j] });
                        double capturedDistance = distance[j];
                        GeoPoint capturedPoint1 = pointsFrom[j];
                        GeoPoint capturedPoint2 = pointsTo[j];
                        DirectMenuEntry faceDist = new DirectMenuEntry("MenuId.FaceDistance");
                        faceDist.ExecuteMenu = (frame) =>
                        {
                            ParametricsDistanceAction pd = new ParametricsDistanceAction(capturedDistTo, capturedFaceI, capturedPoint2, capturedPoint1, touchingPoint, selectAction.Frame);
                            selectAction.Frame.SetAction(pd);
                            return true;
                        };
                        faceDist.IsSelected = (selected, frame) =>
                        {
                            feedback.Clear();
                            if (selected)
                            {
                                feedback.FrontFaces.AddRange(capturedFaceI.ToArray());
                                feedback.BackFaces.AddRange(capturedDistTo.ToArray());
                                feedback.Arrows.AddRange(FeedbackArrow.MakeLengthArrow(owningShell, capturedFaceI.First(), capturedDistTo.First(), fc,
                                    capturedPoint2 - capturedPoint1, touchingPoint, vw, FeedbackArrow.ArrowFlags.secondRed));
                            }
                            feedback.Refresh();
                            return true;
                        };
                        faceEntries.Add(faceDist);
                    }
                }
                // Check for possibilities to rotate this face.
                List<DirectMenuEntry> rotateMenus = new List<DirectMenuEntry>();
                if (fc.Surface is PlaneSurface ps)
                {
                    foreach (Edge edge in fc.OutlineEdges)
                    {
                        if (!edge.IsTangentialEdge())
                        {
                            Face capturedFace = fc;
                            Face otherFace = edge.OtherFace(fc);
                            if (otherFace != null)
                            {
                                (Axis rotationAxis, GeoVector fromHere, GeoVector toHere) = ParametricsAngleAction.GetRotationAxis(fc, otherFace, clickBeam);
                                if (rotationAxis.IsValid)
                                {
                                    DirectMenuEntry rotateMenu = new DirectMenuEntry("MenuId.FaceAngle");
                                    rotateMenu.ExecuteMenu = (frame) =>
                                    {
                                        ParametricsAngleAction pa = new ParametricsAngleAction(capturedFace, otherFace, rotationAxis, fromHere, toHere, edge, selectAction.Frame);
                                        selectAction.Frame.SetAction(pa);
                                        return true;
                                    };
                                    rotateMenu.IsSelected = (selected, frame) =>
                                    {
                                        feedback.Clear();
                                        GeoObjectList feedbackArrow = FeedbackArrow.MakeArcArrow(owningShell, rotationAxis, fromHere, toHere, vw, FeedbackArrow.ArrowFlags.secondRed);
                                        if (selected)
                                        {
                                            feedback.Arrows.AddRange(feedbackArrow);
                                        }
                                        feedback.Refresh();
                                        return true;
                                    };
                                    rotateMenus.Add(rotateMenu);
                                }
                            }
                        }
                    }
                }
                if (rotateMenus.Count > 0)
                {
                    if (rotateMenus.Count == 1) faceEntries.Add(rotateMenus[0]);
                    else
                    {
                        SimplePropertyGroup rm = new SimplePropertyGroup("MenuId.FacesAngle");
                        rm.Add(rotateMenus.ToArray());
                        faceEntries.Add(rm);
                    }
                }
                // else: more to come!
            }
            return faceEntries;
        }
        private List<IPropertyEntry> GetExtensionProperties(Face fc, Projection.PickArea pa, IView vw)
        {
            List<IPropertyEntry> res = new List<IPropertyEntry>();
            if (!(fc.Owner is Shell)) return res;
            FindExtrusionLoops(fc, pa, out List<ICurve[]> loopCurves, out List<Face[]> loopFaces, out List<Edge[]> loopEdges, out List<Plane> loopPlanes, out List<SimpleShape> loopShapes, out GeoPoint pointOnFace);
            for (int i = 0; i < loopCurves.Count; i++)
            {
                Face extrFace = Face.MakeFace(new PlaneSurface(loopPlanes[i]), loopShapes[i]);
                // in order to show an arrow on menu selection, we use the maximum extent of the selected face in direction of the plane
                Face toGetExtent = fc.Clone() as Face;
                toGetExtent.Modify(loopPlanes[i].CoordSys.GlobalToLocal);
                BoundingCube ext = toGetExtent.GetBoundingCube();
                GeoPoint zmax = loopPlanes[i].CoordSys.LocalToGlobal * new GeoPoint(0, 0, ext.Zmax);
                GeoPoint zmin = loopPlanes[i].CoordSys.LocalToGlobal * new GeoPoint(0, 0, ext.Zmin);
                Plane arrowPlane = new Plane(loopPlanes[i].Location, fc.Surface.GetNormal(fc.PositionOf(loopPlanes[i].Location)));
                // GeoObjectList feedbackArrow = vw.Projection.MakeArrow(zmin, zmax, arrowPlane, Projection.ArrowMode.circleArrow);
                // which part of the face is used for meassurement? Try to fingd an edge or a vertex with the appropriate distance
                object maxObject = null, minObject = null;
                foreach (Edge edg in fc.Edges)
                {
                    if (Math.Abs(loopPlanes[i].Distance(edg.Vertex1.Position) - ext.Zmax) < Precision.eps && maxObject == null) maxObject = edg.Vertex1;
                    if (Math.Abs(loopPlanes[i].Distance(edg.Vertex1.Position) - ext.Zmin) < Precision.eps && minObject == null) minObject = edg.Vertex1;
                    if (Math.Abs(loopPlanes[i].Distance(edg.Vertex2.Position) - ext.Zmax) < Precision.eps && maxObject == null) maxObject = edg.Vertex2;
                    if (Math.Abs(loopPlanes[i].Distance(edg.Vertex2.Position) - ext.Zmin) < Precision.eps && minObject == null) minObject = edg.Vertex2;
                }
                if (minObject == null)
                {   // nothing found, maybe it is an edge like an arc, where we use the maximum or minimum distance
                    minObject = fc.Edges.MinBy(e => loopPlanes[i].Distance(e.Vertex1.Position)).Vertex1;
                }
                if (maxObject == null)
                {   // nothing found, maybe it is an edge like an arc, where we use the maximum or minimum distance
                    maxObject = fc.Edges.MinBy(e => -loopPlanes[i].Distance(e.Vertex1.Position)).Vertex1;
                }
                DirectMenuEntry extrudeMenu = new DirectMenuEntry("MenuId.ExtrusionLength");
                Face[] lfaces = loopFaces[i]; // captured values
                Edge[] ledges = loopEdges[i];
                Plane lplane = loopPlanes[i];
                extrudeMenu.IsSelected = (selected, frame) =>
                {
                    if (selected)
                    {
                        GeoObjectList feedbackArrow = FeedbackArrow.MakeLengthArrow(fc.Owner as Shell, minObject, maxObject, fc, lplane.Normal, pointOnFace, vw);
                        feedback.Clear();
                        feedback.SelectedObjects.Add(extrFace);
                        feedback.Arrows.AddRange(feedbackArrow);
                        feedback.Refresh();
                    }
                    return true;
                };
                extrudeMenu.ExecuteMenu = (menuId) =>
                {
                    GeoVector crossDir = lplane.Normal ^ fc.Surface.GetNormal(fc.PositionOf(pointOnFace));
                    Face arrow1 = FeedbackArrow.MakeSimpleTriangle(pointOnFace, lplane.Normal, crossDir, vw.Projection);
                    Face arrow2 = FeedbackArrow.MakeSimpleTriangle(pointOnFace, -lplane.Normal, -crossDir, vw.Projection);
                    ParametricsExtrudeAction pea = new ParametricsExtrudeAction(minObject, maxObject, lfaces, ledges, lplane, extrFace,
                        pointOnFace, new Face[] { arrow1, arrow2 }, cadFrame);
                    selectAction.Frame.SetAction(pea);
                    return true;
                };
                res.Add(extrudeMenu);
            }
            return res;
        }

        /// <summary>
        /// Add the properties of a feature or detail of a solid to the list of modelling properties
        /// </summary>
        /// <param name="vw"></param>
        /// <param name="featureFaces">Faces of the shell, which belong to the feature</param>
        /// <param name="connection">Additional faces, which seperate the feature from the shell</param>
        /// <param name="isGap">Is a hole in the solid or part standing out</param>
        private IPropertyEntry GetFeatureProperties(IView vw, IEnumerable<Face> featureFaces, List<Face> connection, bool isGap)
        {
            if (featureFaces.Count() == 1 && connection.Count == 1)
            {
                if (featureFaces.First().Surface.SameGeometry(featureFaces.First().Domain, connection[0].Surface, connection[0].Domain, Precision.eps, out ModOp2D _))
                    return null;
            }

            Dictionary<Edge, Edge> clonedEdges = new Dictionary<Edge, Edge>();
            Dictionary<Vertex, Vertex> clonedVertices = new Dictionary<Vertex, Vertex>();
            List<Face> ff = new List<Face>(featureFaces.Select(fc => fc.Clone(clonedEdges, clonedVertices) as Face));
            GeoPoint2D uv = ff[0].Area.GetSomeInnerPoint(); // testpoint for orientation
            GeoPoint xyz = ff[0].Surface.PointAt(uv);
            GeoVector normal = ff[0].Surface.GetNormal(uv);
            foreach (Face fc in connection)
            {
                fc.ReverseOrientation();
            }
            ff.AddRange(connection.Select(fc => fc.Clone() as Face));
            BoundingCube ext = BoundingCube.EmptyBoundingCube;
            ext.MinMax(ff);
            Shell.ConnectFaces(ff.ToArray(), Math.Max(Precision.eps, ext.Size * 1e-6));
            Shell feature = Shell.FromFaces(ff.ToArray());
            feature.AssertOutwardOrientation();
            uv = ff[0].Surface.PositionOf(xyz);
            bool wasInverse = normal * ff[0].Surface.GetNormal(uv) < 0;
#if DEBUG
            bool ok = feature.CheckConsistency();
#endif
            //double v = feature.Volume(ext.Size / 1000); // maybe it is totally flat
            //if (v < Precision.eps) return null; // Volume calculation is too slow
            Solid featureSolid = Solid.MakeSolid(feature.Clone() as Shell);
            Shell shell = featureFaces.First().Owner as Shell; // this is the original shell of the solid
            SelectEntry featureMenu = new SelectEntry("MenuId.Feature", true);
            featureMenu.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                if (selected)
                {
                    feedback.FrontFaces.Add(feature);
                    selectedChildObjects.Clear();
                    selectedChildObjects.Add(Solid.MakeSolid(feature)); // we want a solid to be moved around, when the user starts moving this feature
                }
                else selectedChildObjects.Clear();
                feedback.Refresh();
                return true;
            };

            DirectMenuEntry copyFeature = new DirectMenuEntry("MenuId.Feature.CopyToClipboard");
            DirectMenuEntry positionFeature = new DirectMenuEntry("MenuId.Feature.Position");
            DirectMenuEntry nameFeature = new DirectMenuEntry("MenuId.Feature.Name");
            DirectMenuEntry removeFeature = new DirectMenuEntry("MenuId.Feature.Remove");
            DirectMenuEntry splitFeature = new DirectMenuEntry("MenuId.Feature.Split");
            featureMenu.Add(new IPropertyEntry[] { copyFeature, positionFeature, nameFeature, removeFeature, splitFeature });
            // this is very rudimentary. We have to provide a version of ParametricsDistanceAction, where you can select from and to object. Only axis is implemented
            GeoObjectList fa = feature.FeatureAxis;
            Line axis = null;
            //foreach (IGeoObject geoObject in fa)
            //{
            //    Face axisOf = geoObject.UserData.GetData("CADability.AxisOf") as Face;
            //    if (axisOf != null)
            //    {
            //        if (featureI.Contains(axisOf))
            //        {
            //            axis = geoObject as Line;
            //        }
            //    }
            //    if (axis != null) break;
            //}
            copyFeature.ExecuteMenu = (menuId) =>
            {
                Solid sldToCopy = Solid.MakeSolid(feature.Clone() as Shell);
                selectAction.Frame.UIService.SetClipboardData(new GeoObjectList(sldToCopy), true);
                Clear();
                return true;
            };
            copyFeature.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                feedback.FrontFaces.Add(feature);
                feedback.Refresh();
                return true;
            };
            positionFeature.ExecuteMenu = (frame) =>
            {
                ParametricsDistanceActionOld pd = new ParametricsDistanceActionOld(feature, selectAction.Frame);
                selectAction.Frame.SetAction(pd);
                Clear();
                return true;
            };
            positionFeature.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                feedback.FrontFaces.Add(feature);
                feedback.Refresh();
                return true;
            };
            nameFeature.ExecuteMenu = (frame) =>
            {
                string name = shell.GetNewFeatureName();
                shell.AddFeature(name, featureFaces);
                if (shell.Owner is Solid sld)
                {
                    selectAction.SetSelectedObject(sld);
                    IPropertyEntry toEdit = selectAction.Frame.ControlCenter.FindItem(name);
                    if (toEdit != null)
                    {

                        List<IPropertyEntry> parents = new List<IPropertyEntry>();
                        if (toEdit != null)
                        {
                            IPropertyEntry p = toEdit;
                            while ((p = p.Parent as IPropertyEntry) != null)
                            {
                                parents.Add(p);
                            }
                            IPropertyPage propertyPage = parents[parents.Count - 1].Parent as IPropertyPage;
                            if (propertyPage != null)
                            {
                                for (int k = parents.Count - 1; k >= 0; --k)
                                {
                                    propertyPage.OpenSubEntries(parents[k], true);
                                }
                                toEdit.StartEdit(false);
                            }
                        }
                    }
                }
                return true;
            };
            nameFeature.IsSelected = (selected, frame) =>
            {
                feedback.Clear();
                feedback.FrontFaces.Add(feature);
                feedback.Refresh();
                return true;
            };
            removeFeature.ExecuteMenu = (frame) =>
            {
                bool addRemoveOk = shell.AddAndRemoveFaces(connection, featureFaces);
                feedback.Clear();
                feedback.Refresh();
                Clear();
                return true;
            };
            splitFeature.ExecuteMenu = (frame) =>
            {
                feedback.Clear();
                feedback.Refresh();
                Solid solid = shell.Owner as Solid;
                Solid[] remaining = Solid.Subtract(solid, featureSolid);
                if (remaining != null && remaining.Length > 0)
                {
                    IGeoObjectOwner owner = solid.Owner;
                    owner.Remove(solid);
                    for (int i = 0; i < remaining.Length; ++i) owner.Add(remaining[i]);
                    owner.Add(featureSolid);
                    ComposeModellingEntries(new GeoObjectList(featureSolid), vw, null);
                }
                else
                {
                    Clear();
                }
                return true;
            };
            return featureMenu;
        }
        /// <summary>
        /// There is a face <paramref name="fc"/> pointed at by <paramref name="pa"/>. We try to find a loop of curves on faces
        /// passing through pa. This loop msut be perpendicular to all edges it crosses, so that it could have been used 
        /// in an extrusion operation to build all faces, which are touched by the loop curves. To each loop curve there is a face,
        /// on which this curve resides. There may be several (or none) loops and sets of faces as a result. For each result there 
        /// is also a plane, in which all loop curves reside and which is the plane at which the shell can be split to extent or shrink 
        /// the shape along the edges.
        /// </summary>
        /// <param name="fc"></param>
        /// <param name="pa"></param>
        /// <param name="loopCurves"></param>
        /// <param name="loopFaces"></param>
        /// <param name="loopEdges"></param>
        /// <param name="loopPlanes"></param>
        /// <returns></returns>
        private void FindExtrusionLoops(Face fc, Projection.PickArea pa, out List<ICurve[]> loopCurves, out List<Face[]> loopFaces,
            out List<Edge[]> loopEdges, out List<Plane> loopPlanes, out List<SimpleShape> loopShapes, out GeoPoint pointOnFace)
        {
            loopCurves = new List<ICurve[]>();
            loopFaces = new List<Face[]>();
            loopEdges = new List<Edge[]>();
            loopPlanes = new List<Plane>();
            loopShapes = new List<SimpleShape>();
            pointOnFace = GeoPoint.Invalid;
            List<Plane> planeCandidates = new List<Plane>();
            List<Edge> edgeCandidates = new List<Edge>();
            // find the point, which was hit by the mouse
            GeoPoint[] ips = fc.GetLineIntersection(pa.FrontCenter, pa.Direction);
            if (ips.Length > 0)
            {
                pointOnFace = ips.MinBy(p => (p | pa.FrontCenter)); // this is the point on the face, defined by the mouse position (and closest to the viewer)
                Shell shell = fc.Owner as Shell;
                // now find edges on the outline of the face, which are candidates for extrusion, i.e. are lines and the direction is a extrusion direction of the face
                List<GeoVector> directions = new List<GeoVector>();
                foreach (Edge edge in fc.OutlineEdges)
                {
                    if (edge.Curve3D is Line line && fc.Surface.IsExtruded(line.StartDirection))
                    {
                        if (!directions.Exists(d => Precision.SameDirection(d, line.StartDirection, false)))
                        {
                            directions.Add(line.StartDirection);
                        }
                    }
                }
                // we create an octtree of the edges of the shell to have fast acces to the edges from a point
                BoundingCube ext = shell.GetExtent(0.0); // for symmetric objects this sometimes leads to problems, because we later try to find edges from a point
                OctTree<EdgeInOctTree> edgeOctTree = new OctTree<EdgeInOctTree>(ext, Precision.eps);
                edgeOctTree.AddMany(shell.Edges.Select(e => new EdgeInOctTree(e)));

                for (int i = 0; i < directions.Count; i++) // all possible extrusion directions found for the face fc
                {
                    Plane plane = new Plane(pointOnFace, directions[i]); // the plane perpendicular to the possible extrusion direction
                    PlaneSurface planeSurface = new PlaneSurface(plane); // the same plane as a surface
                    ICurve[] intCrvs = shell.GetPlaneIntersection(planeSurface); // it would be better, if GetPlaneIntersection could return the faces and edges, which are intersected
                    ICurve2D[] intCrvs2D = new ICurve2D[intCrvs.Length];
                    for (int j = 0; j < intCrvs.Length; j++)
                    {
                        intCrvs2D[j] = planeSurface.GetProjectedCurve(intCrvs[j], 0.0);
                    }
                    CompoundShape cs = CompoundShape.CreateFromList(intCrvs2D, Precision.eps);
                    if (cs == null) continue;
                    for (int j = 0; j < cs.SimpleShapes.Length; ++j)
                    {
                        Border.Position pos = cs.SimpleShapes[j].GetPosition(GeoPoint2D.Origin, Precision.eps);
                        bool validShape = true;
                        if (pos == Border.Position.OnCurve)
                        {   // There must be a SimpleShape which passes through (0,0), since the plane's location is on the face
                            // The point (0,0) might also reside on a hole, which is no problem.
                            List<ICurve> crvs = new List<ICurve>();
                            List<Edge> edges = new List<Edge>();
                            foreach (ICurve2D curve2D in cs.SimpleShapes[j].Segments)
                            {
                                crvs.Add(planeSurface.Make3dCurve(curve2D));
                                GeoPoint pointOnEdge = planeSurface.PointAt(curve2D.EndPoint);
                                EdgeInOctTree[] eo = edgeOctTree.GetObjectsFromBox(new BoundingCube(pointOnEdge, ext.Size / 1000)); // search from point failed sometimes
                                Edge edgeFound = null;
                                foreach (Edge edg in eo.Select(e => e.Edge))
                                {
                                    if (edg.Curve3D is Line line)
                                    {
                                        if ((line as ICurve).DistanceTo(planeSurface.PointAt(curve2D.EndPoint)) < Precision.eps &&
                                            Precision.SameDirection(line.StartDirection, directions[i], false))
                                        {
                                            edgeFound = edg;
                                            break;
                                        }
                                    }
                                }
                                if (edgeFound != null) { edges.Add(edgeFound); }
                                else
                                {
                                    validShape = false;
                                    break;
                                }
                            }
                            if (validShape)
                            {   // all vertices of the simpleshape have a corresponding edge
                                HashSet<Face> hFaces = new HashSet<Face>();
                                foreach (Edge edg in edges)
                                {
                                    hFaces.Add(edg.PrimaryFace);
                                    hFaces.Add(edg.SecondaryFace);
                                }
                                loopCurves.Add(crvs.ToArray());
                                loopFaces.Add(hFaces.ToArray());
                                loopEdges.Add(edges.ToArray());
                                loopPlanes.Add(plane);
                                loopShapes.Add(cs.SimpleShapes[j]);
                            }
                        }
                        else
                        {
                            validShape = false;
                        }
                    }
                }
            }
        }
        private bool CanMakePath(List<ICurve> curves)
        {
            List<GeoPoint> points = new List<GeoPoint>();
            for (int i = 0; i < curves.Count; i++)
            {
                if (!curves[i].IsClosed)
                {
                    for (int j = 0; j < points.Count; j++)
                    {
                        if (Precision.IsEqual(curves[i].StartPoint, points[j])) return true;
                        if (Precision.IsEqual(curves[i].EndPoint, points[j])) return true;
                    }
                    points.Add(curves[i].StartPoint);
                    points.Add(curves[i].EndPoint);
                }
            }
            return false;
        }
        internal void OnProjectionChanged()
        {
            IPropertyEntry selection = propertyPage?.GetCurrentSelection();
            if (selection != null)
            {
                selection.Selected(selection); // to regenerate the feedback display
                // and by passing selected as "previousSelected" parameter, they can only regenerate projection dependant feedback 
            }
        }

        private List<IPropertyEntry> GetSurfaceSpecificSubmenus(Face face, IView vw, GeoPoint touchingPoint)
        {
            Shell shell = face.Owner as Shell;
            List<IPropertyEntry> res = new List<IPropertyEntry>();
            if (shell == null) return res;
            if (face.IsFillet())
            {
                // handling of a fillet: change radius or remove fillet, composed to a group
                HashSet<Face> connectedFillets = new HashSet<Face>();
                CollectConnectedFillets(face, connectedFillets);
                SelectEntry fch = new SelectEntry("MenuId.Fillet", true);
                fch.IsSelected = (selected, frame) =>
                {
                    feedback.Clear();
                    if (selected) feedback.FrontFaces.AddRange(connectedFillets.ToArray());
                    vw.Invalidate(PaintBuffer.DrawingAspect.Select, vw.DisplayRectangle);
                    return true;
                };
                DirectMenuEntry fr = new DirectMenuEntry("MenuId.Fillet.ChangeRadius");
                fr.ExecuteMenu = (frame) =>
                {
                    ParametricsRadiusAction pr = new ParametricsRadiusAction(connectedFillets.ToArray(), selectAction.Frame, true, touchingPoint);
                    selectAction.Frame.SetAction(pr);
                    return true;
                };
                fr.IsSelected = (selected, frame) =>
                {
                    feedback.Clear();
                    if (selected) feedback.FrontFaces.AddRange(connectedFillets.ToArray());
                    vw.Invalidate(PaintBuffer.DrawingAspect.Select, vw.DisplayRectangle);
                    return true;
                };
                DirectMenuEntry fd = new DirectMenuEntry("MenuId.Fillet.Remove");
                fd.ExecuteMenu = (frame) =>
                {
                    Face[] involvedFaces = connectedFillets.ToArray();
                    Shell orgShell = involvedFaces[0].Owner as Shell;
                    if (orgShell != null)
                    {
                        RemoveFillet rf = new RemoveFillet(involvedFaces[0].Owner as Shell, new HashSet<Face>(involvedFaces));
                        Shell sh = rf.Result();
                        if (sh != null)
                        {
                            using (selectAction.Frame.Project.Undo.UndoFrame)
                            {
                                sh.CopyAttributes(orgShell);
                                IGeoObjectOwner owner = orgShell.Owner;
                                owner.Remove(orgShell);
                                owner.Add(sh);
                            }
                        }
                    }
                    return true;
                };
                fd.IsSelected = (selected, frame) =>
                {
                    feedback.Clear();
                    if (selected) feedback.FrontFaces.AddRange(connectedFillets.ToArray());
                    vw.Invalidate(PaintBuffer.DrawingAspect.Select, vw.DisplayRectangle);
                    return true;
                };
                fch.Add(new IPropertyEntry[] { fr, fd });
                res.Add(fch);
            }
            IEnumerable<Face> connected = face.GetSameSurfaceConnected();
            // if (connected.Any())
            {
                List<Face> lconnected = new List<Face>(connected);
                lconnected.Add(face);
                BoundingCube ext = BoundingCube.EmptyBoundingCube;
                foreach (Face fc in connected)
                {
                    ext.MinMax(fc.GetExtent(0.0));
                }
                // maybe a full sphere, cone, cylinder or torus:
                // except for the sphere: position axis
                // except for the cone: change radius or diameter
                // for the cone: smaller and larger diameter
                // for cone and cylinder: total length
                if (face.Surface is CylindricalSurface || face.Surface is CylindricalSurfaceNP || face.Surface is ToroidalSurface)
                {
                    DirectMenuEntry mh = new DirectMenuEntry("MenuId.FeatureDiameter");
                    mh.ExecuteMenu = (frame) =>
                    {
                        ParametricsRadiusAction pr = new ParametricsRadiusAction(lconnected.ToArray(), selectAction.Frame, false, touchingPoint);
                        selectAction.Frame.SetAction(pr);
                        return true;
                    };
                    mh.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        if (selected)
                        {
                            feedback.ShadowFaces.AddRange(lconnected.ToArray());
                            GeoObjectList arrow = FeedbackArrow.MakeLengthArrow(shell, lconnected.First(), lconnected.Last(), null, GeoVector.NullVector, touchingPoint, vw, FeedbackArrow.ArrowFlags.firstRed | FeedbackArrow.ArrowFlags.secondRed);
                            feedback.Arrows.AddRange(arrow);
                        }
                        feedback.Refresh();
                        return true;
                    };
                    res.Add(mh);
                }
                if (face.Surface is CylindricalSurface || face.Surface is CylindricalSurfaceNP || face.Surface is ConicalSurface)
                {
                    Line axis = null;

                    if (face.Surface is ICylinder cyl) axis = cyl.Axis.Clip(ext);
                    if (face.Surface is ConicalSurface cone) axis = cone.AxisLine(face.Domain.Bottom, face.Domain.Top);
                    DirectMenuEntry mh = new DirectMenuEntry("MenuId.AxisPosition");
                    mh.ExecuteMenu = (frame) =>
                    {
                        ParametricsDistanceAction pd = new ParametricsDistanceAction(lconnected, axis, selectAction.Frame);
                        selectAction.Frame.SetAction(pd);
                        return true;
                    };
                    mh.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        if (selected) feedback.FrontFaces.AddRange(lconnected.ToArray());
                        vw.Invalidate(PaintBuffer.DrawingAspect.Select, vw.DisplayRectangle);
                        return true;
                    };
                    res.Add(mh);
                }
            }
            if (face.Surface is PlaneSurface pls)
            {
                // set this planar face as drawing plane
                DirectMenuEntry dp = new DirectMenuEntry("MenuId.SetDrawingPlane");
                dp.ExecuteMenu = (frame) =>
                {
                    Plane drawingPlane = pls.Plane;
                    GeoPoint2D loc = pls.PositionOf(pls.Plane.Location);
                    if (!face.Contains(ref loc, true))
                    {
                        drawingPlane.Location = pls.PointAt(pls.PositionOf(touchingPoint));
                    }
                    vw.Projection.DrawingPlane = drawingPlane;
                    vw.Invalidate(PaintBuffer.DrawingAspect.Background, vw.DisplayRectangle);
                    return true;
                };
                dp.IsSelected = (selected, frame) =>
                    {
                        feedback.Clear();
                        if (selected) feedback.FrontFaces.Add(face);
                        feedback.Refresh();
                        return true;
                    };
                res.Add(dp);
                // try to find parallel outline edges to modify the distance
                Edge[] outline = face.OutlineEdges;
                for (int j = 0; j < outline.Length - 1; j++)
                {
                    for (int k = j + 1; k < outline.Length; k++)
                    {
                        if (outline[j].Curve3D is Line l1 && outline[k].Curve3D is Line l2)
                        {
                            if (Precision.SameDirection(l1.StartDirection, l2.StartDirection, false))
                            {
                                // two parallel outline lines, we could parametrize the distance
                                Edge o1 = outline[j]; // capture the two edges
                                Edge o2 = outline[k];
                                (GeoPoint p1, GeoPoint p2) = DistanceCalculator.DistanceBetweenObjects(l1, l2, pls.Normal ^ l1.StartDirection, touchingPoint, out GeoVector _, out GeoPoint dp1, out GeoPoint dp2);
                                if ((p1 | p2) > Precision.eps)
                                {
                                    DirectMenuEntry mh = new DirectMenuEntry("MenuId.EdgeDistance");
                                    mh.ExecuteMenu = (frame) =>
                                    {
                                        ParametricsDistanceAction pd = new ParametricsDistanceAction(new Face[] { o2.OtherFace(face) }, new Face[] { o1.OtherFace(face) }, p2, p1, touchingPoint, cadFrame);
                                        cadFrame.SetAction(pd);
                                        return true;
                                    };
                                    mh.IsSelected = (selected, frame) =>
                                    {
                                        feedback.Clear();
                                        if (selected)
                                        {
                                            GeoObjectList fb = FeedbackArrow.MakeLengthArrow(shell, o1.Curve3D, o2.Curve3D, face, pls.Normal ^ l1.StartDirection, touchingPoint, vw, FeedbackArrow.ArrowFlags.secondRed);
                                            feedback.Arrows.AddRange(fb);
                                            feedback.FrontFaces.Add(o1.OtherFace(face));
                                            feedback.BackFaces.Add(o2.OtherFace(face));
                                        }
                                        feedback.Refresh();
                                        return true;
                                    };
                                    res.Add(mh);
                                }
                            }
                        }
                    }
                }
                // still plane surface: Extrude this face
                // we also could extrude non planar faces, but this is more complex
                DirectMenuEntry extrudeFace = new DirectMenuEntry("MenuId.ExtrudeFace");
                extrudeFace.ExecuteMenu = (frame) =>
                {
                    cadFrame.SetAction(new Constr3DFaceExtrude(face));
                    return true;
                };
                extrudeFace.IsSelected = (selected, frame) =>
                {
                    feedback.Clear();
                    if (selected)
                    {
                        feedback.ShadowFaces.Add(face);
                    }
                    feedback.Refresh();
                    return true;
                };
                res.Add(extrudeFace);

            }
            if (res.Count > 6)
            {
                List<IPropertyEntry> lm = new List<IPropertyEntry>();
                for (int i = 4; i < res.Count; i++)
                {
                    lm.Add(res[i]);
                }
                res.RemoveRange(4, lm.Count);
                SimplePropertyGroup subMenu = new SimplePropertyGroup("MenuId.More");
                subMenu.Add(lm.ToArray());
                res.Add(subMenu);
            }
            return res;
        }
        private void CollectConnectedFillets(Face face, HashSet<Face> connectedFillets)
        {
            if (!connectedFillets.Contains(face))
            {
                connectedFillets.Add(face);
                if (face.Surface is ISurfaceOfArcExtrusion extrusion)
                {
                    foreach (Edge edge in face.OutlineEdges)
                    {
                        Face otherFace = edge.OtherFace(face);
                        if (otherFace.IsFillet() && otherFace.Surface is ISurfaceOfArcExtrusion otherExtrusion && Precision.IsEqual(extrusion.Radius, otherExtrusion.Radius))
                        {
                            // if (edge.Curve2D(face).DirectionAt(0.5).IsMoreHorizontal == extrusion.ExtrusionDirectionIsV && otherFace.Surface is ISurfaceOfArcExtrusion arcExtrusion)
                            {
                                CollectConnectedFillets(otherFace, connectedFillets);
                            }
                        }
                        else if (otherFace.Surface is SphericalSurface ss && Precision.IsEqual(ss.RadiusX, extrusion.Radius))
                        {
                            CollectConnectedFillets(otherFace, connectedFillets);
                        }
                    }
                }
                else if (face.Surface is SphericalSurface ss)
                {   // at a sphere a fillet might branch out
                    foreach (Edge edge in face.OutlineEdges)
                    {
                        Face otherFace = edge.OtherFace(face);
                        if (edge.IsTangentialEdge() && otherFace.IsFillet() && otherFace.Surface is ISurfaceOfArcExtrusion otherExtrusion && Precision.IsEqual(ss.RadiusX, otherExtrusion.Radius))
                        {
                            CollectConnectedFillets(otherFace, connectedFillets);
                        }

                    }
                }
            }
        }
        /// <summary>
        /// Handle keystrokes forwarded from propertyPage
        /// </summary>
        /// <param name="keyEventArgs"></param>
        private void OnPreProcessKeyDown(KeyEventArgs keyEventArgs)
        {
            if (propertyPage.GetCurrentSelection() is IHandleKey handleKey)
            {   // the selected item has priority to handle the keystroke
                if (handleKey.HandleKeyCommand(keyEventArgs)) keyEventArgs.SuppressKeyPress = true;
            }
            if (!keyEventArgs.SuppressKeyPress)
            {
                ProcessShortcut(this, keyEventArgs);
            }
        }

        private bool ProcessShortcut(IPropertyEntry propertyEntry, KeyEventArgs keyEventArgs)
        {
            if (propertyEntry is IHandleKey handleKey)
            {
                if (handleKey.HandleKeyCommand(keyEventArgs))
                {
                    keyEventArgs.Handled = true; // what about SuppressKeyPress?
                    return true;
                }
            }
            if (propertyEntry.Flags.HasFlag(PropertyEntryType.HasSubEntries) && propertyEntry.IsOpen)
            {
                for (int i = 0; i < propertyEntry.SubItems.Length; i++)
                {
                    if (ProcessShortcut(propertyEntry.SubItems[i], keyEventArgs)) return true;
                }
            }
            return false;
        }

        public bool OnCommand(string MenuId)
        {
            switch (MenuId)
            {
                case "MenuId.Edit.Copy":
                    {
                        GeoObjectList toCopy;
                        toCopy = new GeoObjectList(selectedObjects);
                        cadFrame.UIService.SetClipboardData(toCopy, true);
                    }
                    return true;
                case "MenuId.Edit.Paste":
                    Clear();
                    object data = Frame.UIService.GetClipboardData(typeof(GeoObjectList));
                    if (data is GeoObjectList l)
                    {
                        if (l != null && l.Count > 0)
                            using (Frame.Project.Undo.UndoFrame)
                            {
                                foreach (IGeoObject go in l)
                                {
                                    if (go.Style != null && go.Style.Name == "CADability.EdgeStyle")
                                    {
                                        go.Style = Frame.Project.StyleList.Current;
                                    }
                                    AttributeListContainer.UpdateObjectAttrinutes(Frame.Project, go);
                                    go.UpdateAttributes(Frame.Project);
                                    Frame.ActiveView.Model.Add(go);
                                }
                            }
                        if (l != null) ComposeModellingEntries(l, cadFrame.ActiveView, null);
                    }
                    return true;
                case "MenuId.ShowHidden":
                    {
                        Model m = Frame.Project.GetActiveModel();
                        foreach (IGeoObject go in m.AllObjects)
                        {
                            if (go.UserData.ContainsData("CADability.OriginalLayer"))
                            {
                                Layer layer = go.UserData.GetData("CADability.OriginalLayer") as Layer;
                                if (layer != null) go.Layer = layer;
                            }
                        }
                        // if (cadFrame.ActiveView is ModelView mv) mv.SetLayerVisibility(layer, true);
                    }
                    return true;
                default: return false;
            }
        }

        public bool OnUpdateCommand(string MenuId, CommandState CommandState)
        {
            switch (MenuId)
            {
                case "MenuId.Edit.Copy":
                    {
                        CommandState.Enabled = selectedObjects.Any();
                    }
                    return true;
                case "MenuId.Edit.Paste":
                    CommandState.Enabled = Frame.UIService.HasClipboardData(typeof(GeoObjectList));
                    return true;
                default: return false;
            }
        }

        public void OnSelected(MenuWithHandler selectedMenu, bool selected)
        {
            throw new NotImplementedException();
        }

    }
}
