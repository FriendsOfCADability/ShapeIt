using CADability.GeoObject;
using CADability;
using CADability.Curve2D;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CADability.Substitutes;

namespace ShapeIt
{
    internal class Feedback
    {
        private IView view;
        public GeoObjectList FrontFaces = new GeoObjectList(); // List of front faces for distance for feedback
        public GeoObjectList BackFaces = new GeoObjectList(); // List of back faces for distance for feedback
        public GeoObjectList ShadowFaces = new GeoObjectList(); // List of faces, usually the result of an operation, displayed as a transparent overlay
        public GeoObjectList CreatedObjects = new GeoObjectList(); // List of objects created by the action, displayed in the action's color (see CreatedObjectsColor), slightly transparent
        public GeoObjectList SelectedObjects = new GeoObjectList(); // List of selected objects to be displayed, when a entry is selected, displayed as brim
        public GeoObjectList Arrows = new GeoObjectList(); // List of highlighted objects to be displayed, when a entry is selected
        public Rectangle selectionRectangle = Rectangle.Empty;
        public List<IHotSpot> hotSpots = new List<IHotSpot>();
        public IHotSpot selectedHotSpot;
        private IPaintTo3DList frontFacesDisplayList = null;
        private IPaintTo3DList backFacesDisplayList = null;
        private IPaintTo3DList shadowFacesDisplayList = null;
        private IPaintTo3DList createdObjectsDisplayList = null;
        private IPaintTo3DList selectedObjectsDisplayList = null;
        private IPaintTo3DList arrowsDisplayList = null;
        private int handleSize;
        private Color handleColor;

        Color frontColor, backColor, selectColor, shadowColor;
        // Color and transparency for the CreatedObjects list. The action sets CreatedObjectsColor to the
        // color of its chosen attribute; the alpha (0..255) makes the not-yet-final shape a bit transparent.
        public Color CreatedObjectsColor = Color.LightBlue;
        public int CreatedObjectsAlpha = 210;
        // When true, the CreatedObjects are painted in their own colors (the attributes carried by the objects)
        // instead of the single CreatedObjectsColor override. Used e.g. by ReflectObjectsAction, whose preview
        // are mirrored clones that should keep the colors of their originals.
        public bool CreatedObjectsOwnColor = false;
        // Optional 2D icon drawn at a point, lying in a given plane (so it is drawn with the normal projection),
        // but at a fixed on-screen size regardless of the zoom. The curves are defined in pixel units; in
        // OnRepaint they are turned into GeoObjects in the plane and scaled so that one 2D unit == one screen
        // pixel. This can render any symbol (crosshair, arrows, ...), see SetIcon / SetCrosshair.
        private List<ICurve2D> iconCurves;
        private GeoPoint iconLocation;
        private GeoVector iconDirX, iconDirY; // two orthonormal directions spanning the icon plane (unless iconFacesViewer)
        private bool iconFacesViewer; // when true the plane directions are taken from the current view, so the icon always faces the viewer
        public double CrosshairDiameterPixels = 8.0;
        public Feedback()
        {
            frontColor = Color.LightGreen;
            backColor = Color.PaleVioletRed;
            selectColor = Color.LightPink;
            shadowColor = Color.Yellow;
        }
        public void Attach(IView vw)
        {
            if (view != null) Detach();
            view = vw;
            vw.SetPaintHandler(PaintBuffer.DrawingAspect.Select, OnRepaint);
            handleSize = view.Canvas.Frame.GetIntSetting("Select.HandleSize", 3); // Größe der Handles
            handleColor = view.Canvas.Frame.GetColorSetting("Select.HandleColor", Color.DarkBlue); // die Farbe für die Handles
        }
        public void Detach()
        {
            view.RemovePaintHandler(PaintBuffer.DrawingAspect.Select, OnRepaint);
        }

        public void Clear()
        {
            FrontFaces.Clear();
            BackFaces.Clear();
            ShadowFaces.Clear();
            CreatedObjects.Clear();
            SelectedObjects.Clear();
            Arrows.Clear();
            hotSpots.Clear(); // was commented out, why?
            // selectedHotSpot = null;
            // reset the display lists, so they will be recreated when the next OnRepaintSelect is called
            frontFacesDisplayList = null;
            backFacesDisplayList = null;
            shadowFacesDisplayList = null;
            createdObjectsDisplayList = null;
            selectedObjectsDisplayList = null;
            arrowsDisplayList = null;
        }

        /// <summary>
        /// Updates the color of the CreatedObjects list and repaints. Only the color override is rebuilt,
        /// the geometry is reused, so this is cheap enough to call live while the user picks a color.
        /// </summary>
        public void SetCreatedObjectsColor(Color color)
        {
            CreatedObjectsColor = color;
            createdObjectsDisplayList = null; // force the display list to be rebuilt with the new color
            if (view != null) Refresh();
        }

        /// <summary>
        /// Shows an icon at <paramref name="location"/>, lying in the plane perpendicular to
        /// <paramref name="normal"/>. The 2D curves are given in pixel units; the icon keeps a fixed screen
        /// size regardless of the zoom and can represent any symbol. If <paramref name="normal"/> is the null
        /// vector the icon instead faces the viewer (billboard): its plane is taken from the current view
        /// projection, so it is always seen from the front, from any perspective.
        /// </summary>
        public void SetIcon(IEnumerable<ICurve2D> curves2D, GeoPoint location, GeoVector normal)
        {
            if (curves2D == null) { ClearIcon(); return; }
            iconCurves = new List<ICurve2D>(curves2D);
            iconLocation = location;
            iconFacesViewer = Precision.IsNullVector(normal);
            if (!iconFacesViewer)
            {
                normal.Normalized.ArbitraryNormals(out GeoVector dirX, out GeoVector dirY);
                iconDirX = dirX.Normalized;
                iconDirY = dirY.Normalized;
            }
        }

        public void ClearIcon()
        {
            iconCurves = null;
        }

        /// <summary>
        /// Convenience over <see cref="SetIcon"/>: a crosshair (a circle plus two lines reaching slightly
        /// beyond it, <see cref="CrosshairDiameterPixels"/> across) at <paramref name="center"/>, lying in the
        /// plane perpendicular to <paramref name="normal"/> (e.g. the rotation plane).
        /// </summary>
        public void SetCrosshair(GeoPoint center, GeoVector normal)
        {
            double r = 0.5 * CrosshairDiameterPixels;
            double ext = 1.4 * r; // the two lines extend a bit beyond the circle
            List<ICurve2D> curves = new List<ICurve2D>
            {
                new Circle2D(GeoPoint2D.Origin, r),
                new Line2D(new GeoPoint2D(-ext, 0.0), new GeoPoint2D(ext, 0.0)),
                new Line2D(new GeoPoint2D(0.0, -ext), new GeoPoint2D(0.0, ext)),
            };
            SetIcon(curves, center, normal);
        }

        public void ClearCrosshair() => ClearIcon();

        // Draws the current icon in its plane, scaled so that its pixel-unit 2D curves span the intended number
        // of screen pixels. We always use parallel projection, so WorldToDeviceFactor is uniform across the view
        // and a single scale factor (world units per pixel) yields the wanted size everywhere. The 2D curves are
        // turned into GeoObjects via ICurve2D.MakeGeoObject and drawn through their own PaintTo3D (no direct
        // primitive calls); a fine paint precision keeps pixel-sized arcs/circles smooth.
        private void PaintIcon(IPaintTo3D paintTo3D, IView vw)
        {
            if (iconCurves == null || iconCurves.Count == 0) return;
            double pixelToWorld = 1.0 / vw.Projection.WorldToDeviceFactor; // world units per screen pixel
            GeoVector dirX, dirY;
            if (iconFacesViewer)
            {
                // billboard: use the screen-aligned world directions of the current view so the icon always
                // faces the viewer (upright and unforeshortened), independent of how the model is rotated
                Plane projPlane = vw.Projection.ProjectionPlane;
                dirX = projPlane.DirectionX;
                dirY = projPlane.DirectionY;
            }
            else
            {
                dirX = iconDirX;
                dirY = iconDirY;
            }
            Plane pl = new Plane(iconLocation, dirX, dirY);
            ModOp scale = ModOp.Scale(iconLocation, pixelToWorld);
            double oldPrecision = paintTo3D.Precision;
            paintTo3D.Precision = 0.02 * pixelToWorld; // ~0.02 px, fine enough for smooth arcs at this size
            paintTo3D.SetColor(handleColor);
            paintTo3D.SetLineWidth(null);
            paintTo3D.SetLinePattern(null);
            foreach (ICurve2D c2d in iconCurves)
            {
                IGeoObject go = c2d.MakeGeoObject(pl);
                if (go == null) continue;
                go.Modify(scale); // the curves are in pixel units; scale so one 2D unit == one screen pixel
                go.PaintTo3D(paintTo3D);
            }
            paintTo3D.Precision = oldPrecision;
        }

        public void Refresh()
        {
            // in den Frame ein CancellationTokenSource einbauen, mit EnableCancellation() starten
            // mit ThrowIfCancellationRequested abbrechen
            // hier Face für Face/Shell/Solid mit PreCalcTriangulation abbrechbar berechnen 
            // Invalidate muss als this.BeginInvoke(() => control.Invalidate()); implementiert werden 
            //double precision = view.Projection.WorldToDeviceFactor;
            //view.Canvas.Frame
            view.Invalidate(PaintBuffer.DrawingAspect.Select, view.DisplayRectangle);
        }

        private void OnRepaint(Rectangle IsInvalid, IView view, IPaintTo3D PaintToSelect)
        {
            // save the state of PaintToSelect
            PaintToSelect.PushState();
            bool oldSelect = PaintToSelect.SelectMode;
            bool pse = PaintToSelect.PaintSurfaceEdges;

            PaintToSelect.UseZBuffer(true);
            PaintToSelect.Blending(true);
            // if the display lists are null, regenerate them
            // PaintToSelect.PaintSurfaceEdges = false;

            if (selectedObjectsDisplayList == null)
            {
                PaintToSelect.OpenList("selected-objects");
                PaintToSelect.SetColor(selectColor, 1); // switch on color override with this color
                foreach (IGeoObject go in SelectedObjects)
                {
                    go.PaintTo3D(PaintToSelect);
                }
                selectedObjectsDisplayList = PaintToSelect.CloseList();
                PaintToSelect.SetColor(selectColor, -1);
            }
            if (shadowFacesDisplayList == null)
            {
                PaintToSelect.OpenList("shadow-faces");
                PaintToSelect.SetColor(Color.FromArgb(128, shadowColor), 1);
                foreach (IGeoObject go in ShadowFaces)
                {
                    go.PaintTo3D(PaintToSelect);
                }
                shadowFacesDisplayList = PaintToSelect.CloseList();
                PaintToSelect.SetColor(shadowColor, -1);
            }
            if (frontFacesDisplayList == null)
            {
                PaintToSelect.OpenList("front-faces");
                PaintToSelect.SetColor(frontColor, 1);
                foreach (IGeoObject go in FrontFaces)
                {
                    go.PaintTo3D(PaintToSelect);
                }
                frontFacesDisplayList = PaintToSelect.CloseList();
                PaintToSelect.SetColor(frontColor, -1);
            }

            if (backFacesDisplayList == null)
            {
                PaintToSelect.OpenList("back-faces");
                PaintToSelect.SetColor(backColor, 1);
                foreach (IGeoObject go in BackFaces)
                {
                    go.PaintTo3D(PaintToSelect);
                }
                backFacesDisplayList = PaintToSelect.CloseList();
                PaintToSelect.SetColor(backColor, -1);
            }
            if (createdObjectsDisplayList == null)
            {   // objects being created by the action, shown in the action's chosen color, a bit transparent
                PaintToSelect.OpenList("created-objects");
                if (CreatedObjectsOwnColor)
                {   // paint each object in its own color (from its attributes), no override
                    PaintToSelect.SetColor(Color.Black); // fallback for objects that carry no own color (ColorDef == null)
                    foreach (IGeoObject go in CreatedObjects)
                    {
                        go.PaintTo3D(PaintToSelect);
                    }
                    createdObjectsDisplayList = PaintToSelect.CloseList();
                }
                else
                {
                    PaintToSelect.SetColor(Color.FromArgb(CreatedObjectsAlpha, CreatedObjectsColor), 1);
                    foreach (IGeoObject go in CreatedObjects)
                    {
                        go.PaintTo3D(PaintToSelect);
                    }
                    createdObjectsDisplayList = PaintToSelect.CloseList();
                    PaintToSelect.SetColor(CreatedObjectsColor, -1);
                }
            }
            PaintToSelect.SetColor(Color.Black); // color to display the arrows an text. objects should have ColorDef==null, so they don't set the color
            bool oldTriangulateText = PaintToSelect.TriangulateText;
            PaintToSelect.TriangulateText = false;
            if (arrowsDisplayList == null)
            {   // no color override, we use actual colors of the objects
                PaintToSelect.OpenList("arrow-objects");
                PaintToSelect.SetColor(Color.Black); // in case of no color specified
                foreach (IGeoObject go in Arrows)
                {
                    go.PaintTo3D(PaintToSelect);
                }
                arrowsDisplayList = PaintToSelect.CloseList();
            }

            // show the display lists            
            // a small amount to the front
            ModOp toViewer = ModOp.Translate(-2 * PaintToSelect.Precision * view.Projection.Direction);
            PaintToSelect.PushMultModOp(toViewer);
            if (shadowFacesDisplayList != null) PaintToSelect.List(shadowFacesDisplayList);
            toViewer = ModOp.Translate(-4 * PaintToSelect.Precision * view.Projection.Direction);
            PaintToSelect.PopModOp();
            PaintToSelect.PushMultModOp(toViewer);
            if (frontFacesDisplayList != null) PaintToSelect.List(frontFacesDisplayList);
            if (backFacesDisplayList != null) PaintToSelect.List(backFacesDisplayList);
            if (createdObjectsDisplayList != null) PaintToSelect.List(createdObjectsDisplayList);
            PaintToSelect.SelectMode = true;
            if (selectedObjectsDisplayList != null) PaintToSelect.SelectedList(selectedObjectsDisplayList, 6);// width of the brim
            PaintToSelect.SelectMode = false;
            PaintToSelect.PopModOp();
            // even more to the front to clearly show the domension lines
            toViewer = ModOp.Translate(-6 * PaintToSelect.Precision * view.Projection.Direction);
            PaintToSelect.PushMultModOp(toViewer);
            if (arrowsDisplayList != null) PaintToSelect.List(arrowsDisplayList);
            PaintIcon(PaintToSelect, view);

            // restore the state of PaintToSelect
            PaintToSelect.PopModOp();
            PaintToSelect.TriangulateText = oldTriangulateText;
            PaintToSelect.SelectMode = oldSelect;
            PaintToSelect.PaintSurfaceEdges = pse;
            PaintToSelect.PopState();

            if (!selectionRectangle.IsEmpty)
            {
                Color bckgnd = view.Canvas.Frame.GetColorSetting("Colors.Background", Color.AliceBlue);
                Color infocolor;
                if (bckgnd.GetBrightness() > 0.5) infocolor = Color.Black;
                else infocolor = Color.White;

                PaintToSelect.SetColor(infocolor);
                PaintToSelect.Line2D(selectionRectangle.Left, selectionRectangle.Bottom, selectionRectangle.Right, selectionRectangle.Bottom);
                PaintToSelect.Line2D(selectionRectangle.Right, selectionRectangle.Bottom, selectionRectangle.Right, selectionRectangle.Top);
                PaintToSelect.Line2D(selectionRectangle.Right, selectionRectangle.Top, selectionRectangle.Left, selectionRectangle.Top);
                PaintToSelect.Line2D(selectionRectangle.Left, selectionRectangle.Top, selectionRectangle.Left, selectionRectangle.Bottom);

            }
            foreach (IHotSpot hsp in hotSpots)
            {
                GeoPoint p = hsp.GetHotspotPosition();
                PointF pf = view.Projection.ProjectF(p);
                PaintTo3D.PaintHandle(PaintToSelect, pf, handleSize, handleColor);
            }
            if (selectedHotSpot != null)
            {
                GeoPoint p = selectedHotSpot.GetHotspotPosition();
                PointF pf = view.Projection.ProjectF(p);
                if (handleSize > 1) PaintTo3D.PaintHandle(PaintToSelect, pf, handleSize - 1, handleColor);
                if (handleSize > 2) PaintTo3D.PaintHandle(PaintToSelect, pf, handleSize - 2, handleColor);
            }

        }
    }
}
