using CADability.GeoObject;
using CADability;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static CADability.Projection;
using System.Globalization;
using CADability.Forms;
using CADability.Attribute;
using System.Drawing;
using CADability.Shapes;

namespace ShapeIt
{
    internal class FeedbackArrow
    {
        public enum ArrowFlags
        {
            None = 0,
            firstRed = 1,
            secondRed = 2,
        }
        private static int arrowSize = 12; // should be a píxel on screen value
        private static NumberFormatInfo numberFormatInfo;
        public static ColorDef blackParts = new ColorDef("blackArrows", Color.Black);
        public static ColorDef redParts = new ColorDef("blackArrows", Color.Red);
        private static LineWidth arrowLineWidth = new LineWidth("arrow", 0.0);

        public static void SetNumberFormat(IFrame frame)
        {
            numberFormatInfo = (NumberFormatInfo)CultureInfo.CurrentCulture.NumberFormat.Clone();
            numberFormatInfo.NumberDecimalDigits = frame.GetIntSetting("Formatting.Coordinate.Digits", 3);
        }

        /// <summary>
        /// Create an arrow, which visualizes the change of a distance on a shell. The arrow should be visible, i.e. not covered by other parts of this shell.
        /// It must connect the two objects <paramref name="fromHere"/> and <paramref name="toHere"/> and should be parallel to the face <paramref name="onThisFace"/>.
        /// Maybe in future it should also contain the length as a number as a string, which shows the distance.
        /// </summary>
        /// <param name="onThisShell"></param>
        /// <param name="fromHere"></param>
        /// <param name="toHere"></param>
        /// <param name="onThisFace"></param>
        /// <param name="pickPoint"></param>
        /// <param name="projection"></param>
        /// <returns></returns>
        public static GeoObjectList MakeLengthArrow(Shell onThisShell, object fromHere, object toHere, Face onThisFace, GeoVector forceDirection, GeoPoint pickPoint, IView vw, ArrowFlags flags = ArrowFlags.None)
        {
            Projection projection = vw.Projection;
            GeoObjectList res = new GeoObjectList();
            (GeoPoint startPoint, GeoPoint endPoint) d = DistanceCalculator.DistanceBetweenObjects(fromHere, toHere, forceDirection, pickPoint, out GeoVector freedom, out GeoPoint dimStartPoint, out GeoPoint dimEndPoint);
            if (d.startPoint.IsValid)
            {   // find a position, where the arrow would be visible
                GeoVector dir = d.endPoint - d.startPoint;
                if (dir.IsNullVector()) dir = GeoVector.XAxis; // if "0" is entered for the distance
                Plane pln;
                GeoVector xdir = projection.Direction ^ (d.endPoint - d.startPoint);
                if (xdir.IsNullVector()) pln = new Plane(d.startPoint, projection.Direction);
                else pln = new Plane(d.startPoint, xdir, (d.endPoint - d.startPoint));
                // now we have two points, which tell the points, where the distance is meassured, at the point, where the user clicked.
                // if fromHere and toHere are faces, we would like to put arrows from the outside onto the face. startPoint to endPoint specify the direction.
                // The arrows are simple triangles rotated to show maximum widt in the direction of the viewer
                if (fromHere is Face ff)
                {
                    GeoVector n = ff.Surface.GetNormal(ff.Surface.PositionOf(d.startPoint));
                    Face triangle;
                    if (n * dir < 0) triangle = FeedbackArrow.MakeSimpleTriangle(d.startPoint, -dir, projection);
                    else triangle = FeedbackArrow.MakeSimpleTriangle(d.startPoint, dir, projection);
                    triangle.ColorDef = blackParts;
                    res.Add(triangle);
                }
                if (toHere is Face ft)
                {
                    GeoVector n = ft.Surface.GetNormal(ft.Surface.PositionOf(d.startPoint));
                    Face triangle;
                    if (n * dir < 0) triangle = FeedbackArrow.MakeSimpleTriangle(d.endPoint, -dir, projection);
                    else triangle = FeedbackArrow.MakeSimpleTriangle(d.endPoint, dir, projection);
                    triangle.ColorDef = blackParts;
                    res.Add(triangle);
                }
                // now find a place where to display the dimension line
                Plane testplane = new Plane(new GeoPoint(d.startPoint, d.endPoint), dir); // a plane perpendicular to the connection of the two points, which represent the distance
                if (onThisShell != null)
                {
                    double arrowLength = projection.DeviceToWorldFactor * arrowSize;
                    int[] evaluation = new int[8]; // how good is the dimension in this direction visible
                    GeoPoint[] dimStartPoints = new GeoPoint[8];
                    GeoPoint[] dimEndPoints = new GeoPoint[8];
                    double minLength = double.MaxValue;
                    int closest = -1;
                    for (int i = 0; i < 8; i++)
                    {
                        GeoVector lineDir = testplane.ToGlobal(Angle.Deg(i * 45).Direction); // the beam from the center to the outside, to find a point for the dimension line
                        GeoPoint poutside = onThisShell.GetLineIntersection(testplane.Location, lineDir).MinByWithDefault(testplane.Location, p => -Geometry.LinePar(testplane.Location, lineDir, p));
                        // if no intersection, we get testPlane.Location
                        double l = poutside | testplane.Location;
                        if (l < minLength)
                        {
                            closest = i;
                            minLength = l;
                        }
                        dimStartPoints[i] = poutside - 0.5 * dir + arrowLength * 5 * lineDir; // the endpoint of the dimension help line
                        dimEndPoints[i] = poutside + 0.5 * dir + arrowLength * 5 * lineDir; // the endpoint of the other dimension help line
                        GeoPoint2D tstpnt1 = projection.ProjectionPlane.Project(dimStartPoints[i]);
                        PickArea pa = projection.GetPickSpace(new BoundingRect(tstpnt1, 1, 1));
                        double z0 = Geometry.LinePar(pa.FrontCenter, pa.Direction, dimStartPoints[i]);
                        double pd1 = vw.Model.GetPickDistance(projection.GetPickSpace(new BoundingRect(tstpnt1, projection.DeviceToWorldFactor * 1, projection.DeviceToWorldFactor * 1)), null);
                        if (pd1 == double.MaxValue) evaluation[i] += 2; // is on free area
                        else if (pd1 > z0) evaluation[i] += 1; // is in front of something
                        tstpnt1 = projection.ProjectionPlane.Project(dimEndPoints[i]);
                        pa = projection.GetPickSpace(new BoundingRect(tstpnt1, 1, 1));
                        z0 = Geometry.LinePar(pa.FrontCenter, pa.Direction, dimEndPoints[i]);
                        pd1 = vw.Model.GetPickDistance(projection.GetPickSpace(new BoundingRect(tstpnt1, projection.DeviceToWorldFactor * 1, projection.DeviceToWorldFactor * 1)), null);
                        if (pd1 == double.MaxValue) evaluation[i] += 2; // is on free area
                        else if (pd1 > z0) evaluation[i] += 1; // is in front of something
                        GeoPoint2D onScreen = projection.Project(dimStartPoints[i]);
                        if (onScreen.x < 0 || onScreen.x > projection.Width || onScreen.y < 0 || onScreen.y > projection.Height) evaluation[i] -= 2; // not visible on screen
                        onScreen = projection.Project(dimEndPoints[i]);
                        if (onScreen.x < 0 || onScreen.x > projection.Width || onScreen.y < 0 || onScreen.y > projection.Height) evaluation[i] -= 2; // not visible on screen
                        double perp = projection.Direction.Normalized * lineDir;
                        if (Math.Abs(perp) < 0.3) evaluation[i] += 2; // the view direction should be close to perpendicular to the dimension
                        else if (Math.Abs(perp) < 0.6) evaluation[i] += 1;
                    }
                    if (closest>=0) evaluation[closest] += 1; // very close to clickpoint
                    closest = 0;
                    for (int i = 1; i < 8; i++) if (evaluation[i] > evaluation[closest]) closest = i;
                    //for (int i = 0; i < 8; i++)
                    {
                        GeoVector lineDir = testplane.ToGlobal(Angle.Deg(closest * 45).Direction); // the beam from the center to the outside, to find a point for the dimension line
                        GeoPoint poutside = onThisShell.GetLineIntersection(testplane.Location, lineDir).MinByWithDefault(testplane.Location, p => -Geometry.LinePar(testplane.Location, lineDir, p));
                        // with an empty list of intersection we will get testplane.Location
                        Line l1 = Line.MakeLine(d.startPoint, poutside - 0.5 * dir + arrowLength * 5 * lineDir);
                        Line l2 = Line.MakeLine(d.endPoint, poutside + 0.5 * dir + arrowLength * 5 * lineDir);
                        res.Add(l1);
                        res.Add(l2);
                        Line l3 = Line.MakeLine(l1.EndPoint - arrowLength * lineDir, l2.EndPoint - arrowLength * lineDir);
                        res.Add(l3);
                        GeoPoint2D tstpnt1 = projection.ProjectionPlane.Project(l3.StartPoint);
                        PickArea pa = projection.GetPickSpace(new BoundingRect(tstpnt1, 1, 1));
                        double z0 = Geometry.LinePar(pa.FrontCenter, pa.Direction, l3.StartPoint);
                        double pd1 = vw.Model.GetPickDistance(projection.GetPickSpace(new BoundingRect(tstpnt1, 1, 1)), null);
                        Face fromArrow = MakeSimpleTriangle(l3.StartPoint, l3.StartDirection, l1.StartDirection, projection);
                        Face toArrow = MakeSimpleTriangle(l3.EndPoint, -l3.EndDirection, l1.StartDirection, projection);
                        if (flags.HasFlag(ArrowFlags.firstRed)) fromArrow.ColorDef = redParts;
                        else fromArrow.ColorDef = blackParts;
                        if (flags.HasFlag(ArrowFlags.secondRed)) toArrow.ColorDef = redParts;
                        else toArrow.ColorDef = blackParts;
                        res.Add(fromArrow);
                        res.Add(toArrow);
                        GeoPoint2D txtloc = projection.WorldToWindow(l3.PointAt(0.5));
                        GeoPoint2D txtp1 = projection.WorldToWindow(l3.PointAt(0.5) + arrowLength * l3.StartDirection.Normalized);
                        GeoPoint2D txtp2 = projection.WorldToWindow(l3.PointAt(0.5) + arrowLength * lineDir.Normalized);
                        GeoVector2D txtd1 = txtp1 - txtloc;
                        GeoVector2D txtd2 = txtp2 - txtloc;
                        // 4 modes to position the text, which is in the plane defined by beam and the dimension line
                        GeoVector txtDir;
                        GeoVector glyphDir;
                        Text.AlignMode verAlignMode = Text.AlignMode.Center;
                        Text.LineAlignMode lineAlignMode = Text.LineAlignMode.Center;
                        bool perpToDimLine;
                        if (Math.Abs(txtd1.y) < Math.Abs(txtd2.y))
                        {
                            txtDir = arrowLength * l3.StartDirection.Normalized;
                            glyphDir = arrowLength * lineDir.Normalized;
                            perpToDimLine = false;
                        }
                        else
                        {
                            glyphDir = arrowLength * l3.StartDirection.Normalized;
                            txtDir = arrowLength * lineDir.Normalized;
                            perpToDimLine = true;
                        }
                        if ((glyphDir ^ txtDir) * projection.Direction < 0) glyphDir = -glyphDir; // needs to point to the viewer, otherwise it is mirror writing
                        if (projection.ProjectionPlane.ToLocal(glyphDir).y < 0)
                        {   // text would be upside down otherwise
                            txtDir = -txtDir;
                            glyphDir = -glyphDir;
                        }
                        if (projection.ProjectionPlane.ToLocal(lineDir).x < 0)
                        {   // text on the left side of dimenasion line
                            if (perpToDimLine)
                            {
                                verAlignMode = Text.AlignMode.Center;
                                lineAlignMode = Text.LineAlignMode.Right;
                            }
                            else
                            {
                                verAlignMode = Text.AlignMode.Bottom;
                                lineAlignMode = Text.LineAlignMode.Center;
                            }
                        }
                        else
                        {
                            if (perpToDimLine)
                            {
                                verAlignMode = Text.AlignMode.Center;
                                lineAlignMode = Text.LineAlignMode.Left;
                            }
                            else
                            {
                                verAlignMode = Text.AlignMode.Top;
                                lineAlignMode = Text.LineAlignMode.Center;
                            }
                        }

                        Text txt = Text.Construct();
                        txt.SetDirections(txtDir, glyphDir);
                        txt.Location = l3.PointAt(0.5) + arrowLength * lineDir;
                        txt.Font = "Arial";
                        txt.Alignment = verAlignMode;
                        txt.LineAlignment = lineAlignMode;
                        txt.TextSize = 2 * arrowLength;

                        txt.TextString = (d.startPoint | d.endPoint).ToString("f", numberFormatInfo);
                        res.Add(txt);
                    }
                }
                for (int i = 0; i < res.Count; i++)
                {
                    if (res[i] is IColorDef cd && cd.ColorDef == null) cd.ColorDef = blackParts;
                    if (res[i] is ILineWidth lw) lw.LineWidth = arrowLineWidth;
                }
            }
            return res;
        }
        public static GeoObjectList MakeArcArrow(Shell onThisShell, Axis axis, GeoVector fromHere, GeoVector toHere, IView vw, ArrowFlags flags = ArrowFlags.None)
        {
            GeoObjectList res = new GeoObjectList();
            GeoVector axisDirection = axis.Direction.Normalized; // to make further calculation easier
            fromHere.Norm();
            toHere.Norm();
            double arrowWorldSize = vw.Projection.DeviceToWorldFactor * arrowSize;
            double radius = arrowWorldSize * 10; // radius of the arc
            Plane arcPlane = new Plane(axis.Location, axisDirection); // normal to the rotation axis
            Plane fromPlane = new Plane(axis.Location, fromHere, axisDirection);
            Plane toPlane = new Plane(axis.Location, toHere, axisDirection);
            // axis.Location is the center for the arc dimension
            Ellipse e = Ellipse.Construct(); // the arc of the arrow
            e.SetArcPlaneCenterStartEndPoint(arcPlane, arcPlane.Project(axis.Location), arcPlane.Project(axis.Location + radius * fromHere), arcPlane.Project(axis.Location + radius * toHere), arcPlane, true);
            if (Math.Abs(e.SweepParameter) > Math.PI)
                e.SetArcPlaneCenterStartEndPoint(arcPlane, arcPlane.Project(axis.Location), arcPlane.Project(axis.Location + radius * fromHere), arcPlane.Project(axis.Location + radius * toHere), arcPlane, false);
            // e is now the arc with less than 180° sweepparameter. Maybe this arc is insid the shell, so you cannot see the dimension
            if (onThisShell.Contains(e.PointAt(0.5)))
            {   // we are inside the shell, so we check with the arc that is rotated 180°
                e.StartParameter += Math.PI;
                if (onThisShell.Contains(e.PointAt(0.5)))
                {   // still inside, so use the first one
                    e.StartParameter -= Math.PI;
                }
            }

            // the first line of dimension
            GeoPoint p1 = axis.Location;
            GeoPoint p2 = e.StartPoint;
            GeoVector dirl = p2 - p1;
            Line firstLeg = Line.TwoPoints(p1-0.1*dirl, p2+0.1*dirl);
            p2 = e.EndPoint;
            dirl = p2 - p1;
            Line secondLeg = Line.TwoPoints(p1 - 0.1 * dirl, p2 + 0.1 * dirl);
            res.Add(firstLeg);
            res.Add(secondLeg);
            res.Add(e); // we dont check visibility, because there are no easy alternatives

            Face startArrow = MakeSimpleTriangle(e.StartPoint, e.StartDirection, vw.Projection);
            Face endArrow = MakeSimpleTriangle(e.EndPoint, -e.EndDirection, vw.Projection);
            if (flags.HasFlag(ArrowFlags.firstRed)) startArrow.ColorDef = redParts;
            else startArrow.ColorDef = blackParts;
            if (flags.HasFlag(ArrowFlags.secondRed)) endArrow.ColorDef = redParts;
            else endArrow.ColorDef = blackParts;
            res.Add(startArrow); 
            res.Add(endArrow);

            Text txt = Text.Construct();
            GeoVector dir = 1.1*(e.PointAt(0.5) - p1); // points to the text location
            GeoVector glyphDir = dir;
            GeoVector txtDir = arcPlane.ToGlobal(arcPlane.Project(dir).ToLeft()); // direction of the text, maybe we have to tourn it, because it would show mirror text
            if (vw.Projection.Direction * (txtDir ^ glyphDir) > 0) txtDir = -txtDir;
            GeoVector2D txtDir2D = vw.Projection.ProjectionPlane.Project(txtDir);
            if (txtDir2D.x<0)
            {   // text goes from right to left, i.e. it is upside down.
                txtDir = -txtDir; // rotate by 180°
                glyphDir = -glyphDir;
            }
            txt.SetDirections(txtDir, glyphDir);
            txt.Location = p1 + dir; ;
            txt.Font = "Arial";
            txt.Alignment = Text.AlignMode.Center;
            txt.LineAlignment = Text.LineAlignMode.Center;
            txt.TextSize = 2 * arrowWorldSize;

            txt.TextString = Math.Abs(e.SweepParameter/Math.PI*180).ToString("f", numberFormatInfo) + "°";
            res.Add(txt);

            return res;

        }
        public static Face MakeSimpleTriangle(GeoPoint point, GeoVector dir, Projection projection)
        {
            GeoVector hor = projection.Direction ^ dir;
            if (hor.IsNullVector()) hor = GeoVector.XAxis ^ dir;
            if (hor.IsNullVector()) hor = GeoVector.YAxis ^ dir;
            hor.Norm();
            double arrowLength = projection.DeviceToWorldFactor * arrowSize;
            GeoPoint p1 = point + arrowLength * dir.Normalized + (arrowLength / 2) * hor;
            GeoPoint p2 = point + arrowLength * dir.Normalized - (arrowLength / 2) * hor;
            return Face.MakeFace(p1, p2, point);
        }
        public static Face MakeSimpleTriangle(GeoPoint point, GeoVector dir, GeoVector perpDir, Projection projection)
        {
            GeoVector hor = perpDir;
            if (hor.IsNullVector()) hor = GeoVector.XAxis ^ dir;
            if (hor.IsNullVector()) hor = GeoVector.YAxis ^ dir;
            hor.Norm();
            double arrowLength = projection.DeviceToWorldFactor * arrowSize;
            GeoPoint p1 = point + arrowLength * dir.Normalized + (arrowLength / 2) * hor;
            GeoPoint p2 = point + arrowLength * dir.Normalized - (arrowLength / 2) * hor;
            return Face.MakeFace(p1, p2, point);
        }
    }
}
