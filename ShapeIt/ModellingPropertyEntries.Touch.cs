using CADability;
using CADability.Attribute;
using CADability.GeoObject;
using CADability.Substitutes;
using System.Collections.Generic;
using Wintellect.PowerCollections;
using static CADability.Projection;
using Point = CADability.Substitutes.Point;

namespace ShapeIt
{
    /// <summary>
    /// Touch/tablet support for the modelling page — kept as a separate partial-class file so
    /// the touch-specific picking can be developed independently of the core modelling logic
    /// (ModellingPropertyEntries.cs). Called from the browser head (Gl3DViewport.ModellingPick);
    /// the desktop heads compile this file but do not call it.
    /// </summary>
    internal partial class ModellingPropertyEntries
    {
        /// <summary>
        /// Touch tap pick (browser/tablet head): like the mouse-up pick in FilterSelectMouseMessages,
        /// but with a finger-sized aperture and EDGE PRIORITY. The standard pick's depth filter in
        /// GetObjectsUnderCursor drops an edge whenever any point of a face inside the aperture is
        /// closer to the viewer — with a finger-sized radius that is almost always the case, so edges
        /// were practically unselectable by touch. Here an edge found in the aperture is accepted when
        /// it bounds one of the front faces under the finger (or nothing is in front of it at all);
        /// ComposeModellingEntries' category priority (edges before curves/faces/solids) then
        /// pre-selects the edge while face and solid stay available in the entry list. Returns false
        /// when the tap should instead be delivered as a normal click: modelling page not active, or a
        /// hotspot is under the finger (a hotspot tap must start the hotspot interaction).
        /// addRemove: toggle the tapped objects in/out of the current selection (touch counterpart of
        /// Ctrl+click). wholeSolid: resolve every hit to its top-level object (the Solid/Shell, or the
        /// curve itself) so a tap selects the whole body instead of a face or edge.
        /// </summary>
        public bool TouchPick(Point location, IView vw, int pickRadius, bool addRemove = false, bool wholeSolid = false)
        {
            if (!modelligIsActive || !cadFrame.ControlCenter.GetPropertyPage("Modelling").IsOnTop()) return false;
            MouseEventArgs e = new MouseEventArgs { X = location.X, Y = location.Y, Location = location };
            if (GetCursorPosition(e, vw) == CursorPosition.OverHotSpot) return false;
            PickArea pickArea = vw.Projection.GetPickSpace(new Rectangle(location.X - pickRadius, location.Y - pickRadius, pickRadius * 2, pickRadius * 2));
            IEnumerable<Layer> visibleLayers = new List<Layer>();
            if (vw is ModelView mv) visibleLayers = mv.GetVisibleLayers();
            GeoObjectList edges = vw.Model.GetObjectsFromRect(pickArea, new Set<Layer>(visibleLayers), PickMode.singleEdge, null);
            GeoObjectList facesAndCurves = vw.Model.GetObjectsFromRect(pickArea, new Set<Layer>(visibleLayers), PickMode.singleChild, null);
            GeoObjectList objects = new GeoObjectList();
            foreach (IGeoObject go in edges)
            {
                bool visible = facesAndCurves.Count == 0; // nothing in front that could hide the edge
                if (go.Owner is Edge edg)
                {
                    foreach (IGeoObject fgo in facesAndCurves)
                    {
                        if (fgo is Face fc && (edg.PrimaryFace == fc || edg.SecondaryFace == fc)) { visible = true; break; }
                    }
                }
                if (visible) objects.Add(go);
            }
            foreach (IGeoObject go in facesAndCurves) objects.Add(go);
            if (wholeSolid)
            {   // resolve each hit to its top-level object (same owner walk as in ComposeModellingEntries)
                GeoObjectList roots = new GeoObjectList();
                HashSet<IGeoObject> seen = new HashSet<IGeoObject>();
                foreach (IGeoObject go in objects)
                {
                    IGeoObject current = go;
                    IGeoObjectOwner owner = go.Owner;
                    try
                    {
                        while (owner != null && !(owner is Model))
                        {
                            if (owner is Edge edge) owner = edge.Owner.Owner;
                            else if (owner is IGeoObject goo) owner = goo.Owner;
                            else owner = null;
                            if (owner is IGeoObject g) current = g;
                        }
                    }
                    catch { /* incomplete owner chain: keep what we have */ }
                    if (seen.Add(current)) roots.Add(current);
                }
                objects = roots;
            }
            ComposeModellingEntries(objects, vw, pickArea, true, addRemove, preferMostSpecific: true);
            IsOpen = true;
            Refresh();
            return true;
        }
    }
}
