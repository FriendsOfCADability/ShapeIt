using CADability.GeoObject;
using CADability;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using CADability.Attribute;

namespace ShapeIt
{
    internal class Feedback
    {
        private IView view;
        public GeoObjectList FrontFaces = new GeoObjectList(); // List of front faces for distance for feedback
        public GeoObjectList BackFaces = new GeoObjectList(); // List of back faces for distance for feedback
        public GeoObjectList ShadowFaces = new GeoObjectList(); // List of faces, usually the result of an operation, displayed as a transparent overlay
        public GeoObjectList SelectedObjects = new GeoObjectList(); // List of selected objects to be displayed, when a entry is selected, displayed as brim
        public GeoObjectList Arrows = new GeoObjectList(); // List of highlighted objects to be displayed, when a entry is selected
        public Rectangle selectionRectangle = Rectangle.Empty;
        public List<IHotSpot> hotSpots = new List<IHotSpot>();
        public IHotSpot selectedHotSpot;
        private IPaintTo3DList frontFacesDisplayList = null;
        private IPaintTo3DList backFacesDisplayList = null;
        private IPaintTo3DList shadowFacesDisplayList = null;
        private IPaintTo3DList selectedObjectsDisplayList = null;
        private IPaintTo3DList arrowsDisplayList = null;
        private int handleSize;
        private Color handleColor;

        Color frontColor, backColor, selectColor, shadowColor;
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
            SelectedObjects.Clear();
            Arrows.Clear();
            // hotSpots.Clear(); // hotspots are removed regularly
            // selectedHotSpot = null;
            // reset the display lists, so they will be recreated when the next OnRepaintSelect is called
            frontFacesDisplayList = null;
            backFacesDisplayList = null;
            shadowFacesDisplayList = null;
            selectedObjectsDisplayList = null;
            arrowsDisplayList = null;
        }

        public void Refresh()
        {
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
            PaintToSelect.SelectMode = true;
            if (selectedObjectsDisplayList != null) PaintToSelect.SelectedList(selectedObjectsDisplayList, 6);// width of the brim
            PaintToSelect.SelectMode = false;
            PaintToSelect.PopModOp();
            // even more to the front to clearly show the domension lines
            toViewer = ModOp.Translate(-6 * PaintToSelect.Precision * view.Projection.Direction);
            PaintToSelect.PushMultModOp(toViewer);
            if (arrowsDisplayList != null) PaintToSelect.List(arrowsDisplayList);

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
