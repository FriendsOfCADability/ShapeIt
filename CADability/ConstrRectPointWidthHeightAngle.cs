using CADability.GeoObject;
using System;
using MouseEventArgs = CADability.Substitutes.MouseEventArgs;


namespace CADability.Actions
{
    /// <summary>
    /// 
    /// </summary>
    internal class ConstrRectPointWidthHeightAngle : ConstructAction
    {
        public ConstrRectPointWidthHeightAngle()
        { }

        private Polyline polyLine;
        private GeoVectorInput ang;
        private LengthInput height;
        private LengthInput width;
        private GeoPointInput startPointInput;
        private GeoPointInput centerPointInput;
        private static bool isCentered = false;

        private void StartPoint(GeoPoint p)
        {
            polyLine.SetRectangle(p, polyLine.RectangleWidth * base.ActiveDrawingPlane.DirectionX, polyLine.RectangleHeight * base.ActiveDrawingPlane.DirectionY);
            SetCentered(false);
        }

        private void CenterPoint(GeoPoint p)
        {
            polyLine.SetRectangle(p - polyLine.RectangleWidth / 2 * base.ActiveDrawingPlane.DirectionX - polyLine.RectangleHeight / 2 * base.ActiveDrawingPlane.DirectionY, polyLine.RectangleWidth * base.ActiveDrawingPlane.DirectionX, polyLine.RectangleHeight * base.ActiveDrawingPlane.DirectionY);
            SetCentered(true);
        }

        private double WidthCalculate(GeoPoint MousePosition)
        {  // falls die Breite über einen Punkt im Raum über dem jetzigen Rechteck bestimmt wird:
           // der Lotfußpunkt von MousePosition auf die NebenAchse (y-Direction)
            if (isCentered)
            {
                double dist = Math.Abs(Geometry.DistPL(MousePosition, polyLine.RectangleLocation + 0.5 * polyLine.ParallelogramMainDirection, polyLine.ParallelogramSecondaryDirection)); // center in xdirection
                if (dist > Precision.eps)
                {   // Neues Rechteck mit neuer Orientierung im Raum, x-Vektor: Lotfußpunkt, Mausposition
                    polyLine.CenteredRectangleWidth = 2 * dist;
                    // nun die Breite zurückliefern
                    return polyLine.RectangleWidth;
                }
            }
            else
            {
                GeoPoint p = Geometry.DropPL(MousePosition, polyLine.RectangleLocation, polyLine.ParallelogramSecondaryDirection);
                if (!Precision.IsEqual(MousePosition, p))
                {   // Neues Rechteck mit neuer Orientierung im Raum, x-Vektor: Lotfußpunkt, Mausposition
                    polyLine.SetRectangle(polyLine.RectangleLocation, new GeoVector(p, MousePosition), polyLine.ParallelogramSecondaryDirection);
                    // nun die Breite zurückliefern
                    return polyLine.RectangleWidth;
                }
            }
            return 0;
        }

        private bool Width(double length)
        {
            if (length > Precision.eps)
            {
                if (isCentered) polyLine.CenteredRectangleWidth = length;
                else polyLine.RectangleWidth = length;
                return true;
            }
            return false;
        }

        private double HeightCalculate(GeoPoint MousePosition)
        {   // falls die Höhe über einen Punkt im Raum über dem jetzigen Rechteck bestimmt wird:
            // der Lotfußpunkt von MousePosition auf die HauptAchse (x-Direction)
            if (isCentered)
            {
                double dist = Math.Abs(Geometry.DistPL(MousePosition, polyLine.RectangleLocation + 0.5 * polyLine.ParallelogramSecondaryDirection, polyLine.ParallelogramMainDirection)); // center in xdirection
                if (dist > Precision.eps)
                {   // Neues Rechteck mit neuer Orientierung im Raum, x-Vektor: Lotfußpunkt, Mausposition
                    polyLine.CenteredRectangleHeight = 2 * dist;
                    // nun die Breite zurückliefern
                    return polyLine.RectangleHeight;
                }
            }
            else
            {
                GeoPoint p = Geometry.DropPL(MousePosition, polyLine.RectangleLocation, polyLine.ParallelogramMainDirection);
                if (!Precision.IsEqual(MousePosition, p))
                {   // Neues Rechteck mit neuer Orientierung im Raum, y-Vektor: Lotfußpunkt, Mausposition
                    polyLine.SetRectangle(polyLine.RectangleLocation, polyLine.ParallelogramMainDirection, new GeoVector(p, MousePosition));
                    // nun die Höhe zurückliefern
                    return polyLine.RectangleHeight;
                }
            }
            return 0;
        }

        private bool Height(double length)
        {
            if (length > Precision.eps)
            {
                if (isCentered) polyLine.CenteredRectangleHeight = length;
                else polyLine.RectangleHeight = length;
                return true;
            }
            return false;
        }
        private bool RectAngle(GeoVector vector)
        {   // derWinkel als x-Achsen Vektor
            if (!vector.IsNullVector())
            {
                vector.Norm();
                //if (ActiveDrawingPlane.Normal != vector) // Spezialfall, ausschliessen, sonst krachts
                //    line.SetRectangle(line.RectangleLocation,line.RectangleWidth*vector,line.RectangleHeight*(ActiveDrawingPlane.Normal^vector));
                GeoVector2D v1 = base.ActiveDrawingPlane.Project(vector);
                GeoVector v2 = base.ActiveDrawingPlane.ToGlobal(v1);
                if (!v2.IsNullVector())
                {
                    v2.Norm();
                    if (ActiveDrawingPlane.Normal != v2) // Spezialfall, ausschliessen, sonst krachts
                        polyLine.SetRectangle(polyLine.RectangleLocation, polyLine.RectangleWidth * v2, polyLine.RectangleHeight * (ActiveDrawingPlane.Normal ^ v2));
                }
                return true;
            }
            return false;
        }

        protected override bool FindTangentialPoint(MouseEventArgs e, IView vw, out GeoPoint found)
        {
            double mindist = double.MaxValue;
            found = GeoPoint.Origin;
            if (CurrentInput == width && startPointInput.Fixed)
            {
                GeoObjectList l = base.GetObjectsUnderCursor(e.Location);
                l.DecomposeAll();
                for (int i = 0; i < l.Count; i++)
                {
                    if (l[i] is ICurve)
                    {
                        double[] tanpos = (l[i] as ICurve).TangentPosition(polyLine.ParallelogramSecondaryDirection);
                        if (tanpos != null)
                        {
                            for (int j = 0; j < tanpos.Length; j++)
                            {
                                GeoPoint p = (l[i] as ICurve).PointAt(tanpos[j]);
                                double d = base.WorldPoint(e.Location) | p;
                                if (d < mindist)
                                {
                                    mindist = d;
                                    found = p;
                                }
                            }
                        }
                    }
                }
            }
            if (CurrentInput == height && startPointInput.Fixed)
            {
                GeoObjectList l = base.GetObjectsUnderCursor(e.Location);
                l.DecomposeBlocks(true);
                l.DecomposeBlockRefs();
                for (int i = 0; i < l.Count; i++)
                {
                    if (l[i] is ICurve)
                    {
                        double[] tanpos = (l[i] as ICurve).TangentPosition(polyLine.StartDirection);
                        if (tanpos != null)
                        {
                            for (int j = 0; j < tanpos.Length; j++)
                            {
                                GeoPoint p = (l[i] as ICurve).PointAt(tanpos[j]);
                                double d = base.WorldPoint(e.Location) | p;
                                if (d < mindist)
                                {
                                    mindist = d;
                                    found = p;
                                }
                            }
                        }
                    }
                }
            }
            return mindist != double.MaxValue;
        }


        public override void OnSetAction()
        {
            polyLine = Polyline.Construct();
            polyLine.SetRectangle(ConstrDefaults.DefaultStartPoint, new GeoVector(ConstrDefaults.DefaultRectWidth, 0.0, 0.0), new GeoVector(0.0, ConstrDefaults.DefaultRectHeight, 0.0));
            base.BasePoint = ConstrDefaults.DefaultStartPoint;
            base.ActiveObject = polyLine;
            base.TitleId = "Constr.Rect.PointWidthHeightAngle";

            startPointInput = new GeoPointInput("Rect.StartPoint");
            startPointInput.Optional = isCentered;
            if (!isCentered)
            {
                startPointInput.DefinesBasePoint = true;
                startPointInput.DefaultGeoPoint = ConstrDefaults.DefaultStartPoint;
            }
            startPointInput.SetGeoPointEvent += new ConstructAction.GeoPointInput.SetGeoPointDelegate(StartPoint);
            startPointInput.GetGeoPointEvent += () => polyLine.RectangleLocation;
            startPointInput.canOverrideDrawingPlane = true;

            centerPointInput = new GeoPointInput("Rect.CenterPoint");

            if (isCentered)
            {
                centerPointInput.DefaultGeoPoint = ConstrDefaults.DefaultStartPoint;
                centerPointInput.DefinesBasePoint = true;
            }
            centerPointInput.Optional = !isCentered;
            centerPointInput.SetGeoPointEvent += new ConstructAction.GeoPointInput.SetGeoPointDelegate(CenterPoint);
            centerPointInput.GetGeoPointEvent += () => polyLine.Center;
            centerPointInput.canOverrideDrawingPlane = true;

            width = new LengthInput("Rect.Width");
            width.DefaultLength = ConstrDefaults.DefaultRectWidth;
            width.SetLengthEvent += new ConstructAction.LengthInput.SetLengthDelegate(Width);
            width.CalculateLengthEvent += new CADability.Actions.ConstructAction.LengthInput.CalculateLengthDelegate(WidthCalculate);
            // width.ForwardMouseInputTo = startPointInput;

            height = new LengthInput("Rect.Height");
            height.DefaultLength = ConstrDefaults.DefaultRectHeight;
            height.SetLengthEvent += new ConstructAction.LengthInput.SetLengthDelegate(Height);
            height.CalculateLengthEvent += new CADability.Actions.ConstructAction.LengthInput.CalculateLengthDelegate(HeightCalculate);
            // height.ForwardMouseInputTo = startPointInput;

            ang = new GeoVectorInput("Rect.Angle");
            ang.IsAngle = true;
            ang.DefaultGeoVector = ConstrDefaults.DefaultRectAngle;
            ang.SetGeoVectorEvent += new CADability.Actions.ConstructAction.GeoVectorInput.SetGeoVectorDelegate(RectAngle);
            ang.ForwardMouseInputTo = startPointInput;
            base.SetInput(startPointInput, centerPointInput, width, height, ang);
            base.ShowAttributes = true;
            base.OnSetAction();
        }

        private void SetCentered(bool centered)
        {
            if (centered)
            {
                centerPointInput.Optional = false;
                startPointInput.Optional = true;
            }
            else
            {
                centerPointInput.Optional = true;
                startPointInput.Optional = false;
            }
            isCentered = centered;
        }

        public override void OnRemoveAction()
        {
            base.OnRemoveAction();
        }

        public override string GetID()
        {
            return "Constr.Rect.PointWidthHeightAngle";
        }

        public override void OnDone()
        {
            ConstrDefaults.DefaultStartPoint.Point = polyLine.RectangleLocation + polyLine.ParallelogramMainDirection + polyLine.ParallelogramSecondaryDirection;
            // wird auf den Diagonalpunkt gesetzt
            base.OnDone();
        }
    }
}

