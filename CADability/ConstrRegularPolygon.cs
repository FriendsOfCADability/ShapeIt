using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Text;

namespace CADability.Actions
{
    internal class ConstrRegularPolygon : ConstructAction
    {
        private Polyline polyLine;

        GeoPointInput center;
        GeoPointInput diag1, diag2;
        IntInput numPoints;
        AngleInput angle;
        LengthInput radius, sideLength;
        MultipleChoiceInput radiusMode;

        public override string GetID()
        {
            return "Constr.Polygon";
        }

        public override void OnSetAction()
        {
            polyLine = Polyline.Construct();
            Plane pln = new Plane(ConstrDefaults.DefaultArcCenter, base.ActiveDrawingPlane.DirectionX, base.ActiveDrawingPlane.DirectionY);
            polyLine.SetRegularPolygon(pln, ConstrDefaults.DefaultArcRadius, 0.0, 6);

            base.ActiveObject = polyLine;
            base.TitleId = "Constr.Polygon";

            center = new GeoPointInput("Rect.Polygon.Center");
            center.DefinesBasePoint = true;
            center.canOverrideDrawingPlane = true;
            center.DefaultGeoPoint = ConstrDefaults.DefaultArcCenter;
            center.SetGeoPointEvent += (p) => Recalc(center);

            diag1 = new GeoPointInput("Rect.Polygon.DiagonalPoint1");
            diag1.SetGeoPointEvent += (p) => Recalc(diag1);
            diag1.Optional = true; // typically center and diag2 define the polygon
            diag2 = new GeoPointInput("Rect.Polygon.DiagonalPoint1");
            diag2.SetGeoPointEvent += (p) => Recalc(diag2);
            numPoints = new IntInput("Rect.Polygon.NumberOfSides", 6);
            numPoints.SetIntEvent += (i) => Recalc(numPoints);
            numPoints.SetMinMax(3, 1000, true);

            angle = new AngleInput("Rect.Polygon.Angle");
            angle.SetAngleEvent += (a) => Recalc(angle);
            angle.Optional = true;
            radius = new LengthInput("Rect.Polygon.Radius");
            radius.SetLengthEvent += (l) => Recalc(radius);
            radius.Optional = true;
            sideLength = new LengthInput("Rect.Polygon.SideLength");
            sideLength.SetLengthEvent += (l) => Recalc(sideLength);
            sideLength.Optional = true;
            radiusMode = new MultipleChoiceInput("Rect.Polygon.RadiusMode", "Rect.Polygon.RadiusMode.Values");
            radiusMode.Optional = true;

            base.SetInput(radiusMode, center, diag1, diag2, numPoints, angle, radius, sideLength);
            base.ShowAttributes = true;

            base.OnSetAction();
        }

        private bool Recalc(InputObject inp)
        {
            // there are 3 szenarios: 1: center and diagonal points, 2: two diagonal points or 3:center, radius and angle
            // 4: center and side length
            int mode = 0;
            Plane pln = ActiveDrawingPlane;
            if (center.Fixed || inp == center)
            {
                pln.Location = center.Point;
                if (diag2.Fixed || inp == diag2) { mode = 1; }
                else if (angle.Fixed || inp == angle || radius.Fixed || inp == radius) { mode = 3; }
                else if (sideLength.Fixed || inp == sideLength) { mode = 4; }
            }
            else if (diag1.Fixed || inp == diag1)
            {
                if (diag2.Fixed || inp == diag2)
                {
                    pln.Location = new GeoPoint(diag1.Point, diag2.Point);
                    mode = 2;
                }
                else
                {
                    pln.Location = center.Point;
                }
            }
            else if (diag2.Fixed || inp == diag2)
            {
                if (diag1.Fixed || inp == diag1)
                {
                    pln.Location = new GeoPoint(diag1.Point, diag2.Point);
                    mode = 2;
                }
                else
                {
                    pln.Location = center.Point;
                }
            }
            double oradius = 0.0;
            GeoPoint cnt = GeoPoint.Origin;
            double a = 0.0;
            int n = numPoints.IntValue;
            switch (mode)
            {
                case 0: // not yet known
                    // nothing but center specified
                    oradius = ConstrDefaults.DefaultArcRadius;
                    cnt = center.Point;
                    a = 0.0;
                    break;
                case 1: // center and diagonal point
                    center.Optional = false;
                    diag2.Optional = false;
                    diag1.Optional = true;
                    radius.Optional = true;
                    sideLength.Optional = true;
                    angle.Optional = true;

                    oradius = diag2.Point | center.Point;
                    cnt = center.Point;
                    a = (pln.Project(diag2.Point) - pln.Project(center.Point)).Angle;
                    break;
                case 2: // two diagonal points
                    center.Optional = true;
                    diag2.Optional = false;
                    diag1.Optional = false;
                    radius.Optional = true;
                    sideLength.Optional = true;
                    angle.Optional = true;

                    oradius = (diag2.Point | diag1.Point) / 2.0;
                    cnt = new GeoPoint(diag2.Point, diag1.Point);
                    a = (pln.Project(diag2.Point) - pln.Project(diag1.Point)).Angle;
                    break;
                case 3: // center, radius, angle
                    center.Optional = false;
                    diag2.Optional = true;
                    diag1.Optional = true;
                    radius.Optional = false;
                    sideLength.Optional = true;
                    angle.Optional = false;

                    oradius = radius.Length;
                    cnt = center.Point;
                    a = angle.Angle;
                    break;
                case 4: // center, side length, angle
                    center.Optional = false;
                    diag2.Optional = true;
                    diag1.Optional = true;
                    radius.Optional = true;
                    sideLength.Optional = false;
                    angle.Optional = false;

                    oradius = sideLength.Length / 2.0 / Math.Sin(Math.PI / n);
                    cnt = center.Point;
                    a = angle.Angle;
                    break;
            }
            if (radiusMode.Choice == 1) oradius = oradius / Math.Cos(Math.PI / n); // from inner radius to outer radius

            polyLine.SetRegularPolygon(pln, oradius, a, n);
            return true;
        }
    }
}
