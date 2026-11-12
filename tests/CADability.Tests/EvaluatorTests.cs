using System.Collections.Generic;
using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// The expression evaluator used by the MCP server (workspace.set, filters, templates).
    /// </summary>
    [TestClass]
    public class EvaluatorTests
    {
        private static object Eval(string expr, Dictionary<string, object>? values = null)
            => Evaluator.Evaluate(expr, values ?? new Dictionary<string, object>());

        [TestMethod]
        public void Arithmetic()
        {
            Assert.AreEqual(7.0, Convert.ToDouble(Eval("1 + 2 * 3")), 1e-12);
            Assert.AreEqual(30.0, Convert.ToDouble(Eval("asin(0.5)*180/pi")), 1e-9);
        }

        /// <summary>atan2(y, x) is the angle of the point (x, y), as in Math.Atan2 and as its message says.</summary>
        [TestMethod]
        public void Atan2TakesYFirst()
        {
            Assert.AreEqual(Math.Atan2(1, 2), Convert.ToDouble(Eval("atan2(1, 2)")), 1e-12);
            Assert.AreEqual(Math.PI / 2, Convert.ToDouble(Eval("atan2(1, 0)")), 1e-12);
        }

        /// <summary>random with a seed is a pure function of the seed; without a seed it only has to stay in range.</summary>
        [TestMethod]
        public void RandomWithAndWithoutSeed()
        {
            double a = Convert.ToDouble(Eval("random(2, 5, 7)"));
            Assert.AreEqual(a, Convert.ToDouble(Eval("random(2, 5, 7)")), 0.0);
            Assert.IsTrue(a >= 2 && a < 5);
            Assert.AreNotEqual(a, Convert.ToDouble(Eval("random(2, 5, 8)")));
            Assert.AreEqual(Convert.ToDouble(Eval("random(7)")), (a - 2) / 3, 1e-12); // same seed, only scaled

            var values = new Dictionary<string, object> { ["i"] = 3.0 };
            Assert.AreEqual(Convert.ToDouble(Eval("random(10 + i)", values)), Convert.ToDouble(Eval("random(13)")), 0.0);

            var seen = new HashSet<double>();
            for (int i = 0; i < 100; i++)
            {
                double u = Convert.ToDouble(Eval("random()"));
                Assert.IsTrue(u >= 0 && u < 1);
                double r = Convert.ToDouble(Eval("random(-1, 1)"));
                Assert.IsTrue(r >= -1 && r < 1);
                double s = Convert.ToDouble(Eval($"random({i})"));
                Assert.IsTrue(s >= 0 && s < 1);
                seen.Add(s);
            }
            Assert.IsTrue(seen.Count > 95, "different seeds should give different values");
        }

        [TestMethod]
        public void PropertyOfAVariable()
        {
            var values = new Dictionary<string, object> { ["pt"] = new GeoPoint(1, 2, 3) };
            Assert.AreEqual(2.0, Convert.ToDouble(Eval("pt.y", values)), 1e-12);
        }

        [TestMethod]
        public void MethodCallWithArgument()
        {
            var values = new Dictionary<string, object> { ["box"] = new BoundingBox(GeoPoint.Origin, new GeoPoint(2, 4, 6)) };
            Assert.AreEqual(true, Eval("box.Contains(p(1,1,1))", values));
        }

        [TestMethod]
        public void MethodCallWithoutArgument()
        {
            var values = new Dictionary<string, object> { ["box"] = new BoundingBox(GeoPoint.Origin, new GeoPoint(2, 4, 6)) };
            object center = Eval("box.GetCenter()", values);
            Assert.IsInstanceOfType(center, typeof(GeoPoint));
            Assert.AreEqual(new GeoPoint(1, 2, 3), (GeoPoint)center);
        }

        [TestMethod]
        public void MethodCallFollowedByProperty()
        {
            var values = new Dictionary<string, object> { ["box"] = new BoundingBox(GeoPoint.Origin, new GeoPoint(2, 4, 6)) };
            Assert.AreEqual(3.0, Convert.ToDouble(Eval("box.GetCenter().z", values)), 1e-12);
        }

        [TestMethod]
        public void DefinedAndNot()
        {
            var values = new Dictionary<string, object> { ["hull"] = 1.0 };
            Assert.AreEqual(true, Eval("defined(\"hull\")", values));
            Assert.AreEqual(false, Eval("defined(\"shade\")", values));
            Assert.AreEqual(true, Eval("!defined(\"shade\")", values));
        }

        [TestMethod]
        public void FarthestAndClosestPoint()
        {
            var values = new Dictionary<string, object>
            {
                ["pts"] = new List<GeoPoint> { new GeoPoint(1, 0, 0), new GeoPoint(5, 0, 0), new GeoPoint(0, -3, 0) },
                ["box"] = Make3D.MakeBox(GeoPoint.Origin, 10 * GeoVector.XAxis, 20 * GeoVector.YAxis, 30 * GeoVector.ZAxis)
            };
            Assert.AreEqual(new GeoPoint(5, 0, 0), (GeoPoint)Eval("farthest(pts, p(0,0,0))", values));
            Assert.AreEqual(new GeoPoint(1, 0, 0), (GeoPoint)Eval("closest(pts, p(0,0,0))", values));
            // a solid contributes its vertices
            Assert.AreEqual(new GeoPoint(10, 20, 30), (GeoPoint)Eval("farthest(box, p(-1,-1,-1))", values));
            Assert.AreEqual(new GeoPoint(0, 0, 0), (GeoPoint)Eval("closest(box, p(-1,-1,-1))", values));
        }

        [TestMethod]
        public void IntegerTimesVector()
        {
            // sign() yields an int, which must scale a vector like a double
            Assert.AreEqual(new GeoVector(-1, -2, -3), (GeoVector)Eval("sign(0-4)*v(1,2,3)"));
            Assert.AreEqual(new GeoVector(2, 4, 6), (GeoVector)Eval("v(1,2,3)*2"));
        }

        [TestMethod]
        public void IndexIntoList()
        {
            var values = new Dictionary<string, object>
            {
                ["pts"] = new List<GeoPoint> { new GeoPoint(1, 0, 0), new GeoPoint(5, 0, 0), new GeoPoint(0, -3, 0) },
                ["i"] = 1.0,
                ["single"] = new GeoPoint(7, 8, 9)
            };
            Assert.AreEqual(new GeoPoint(1, 0, 0), (GeoPoint)Eval("pts[0]", values));
            Assert.AreEqual(new GeoPoint(0, -3, 0), (GeoPoint)Eval("pts[i+1]", values));
            Assert.AreEqual(new GeoPoint(0, -3, 0), (GeoPoint)Eval("pts[-1]", values));
            Assert.AreEqual(new GeoPoint(0, -3, 0), (GeoPoint)Eval("pts[pts.count-1]", values));
            Assert.AreEqual(5.0, Convert.ToDouble(Eval("pts[1].x", values)), 1e-12);
            Assert.AreEqual(-5.0, Convert.ToDouble(Eval("-pts[i].x", values)), 1e-12);
            Assert.AreEqual(4.0, Convert.ToDouble(Eval("pts[1] | pts[0]", values)), 1e-12);
            Assert.AreEqual(5.0, Convert.ToDouble(Eval("max(pts[0].x, pts[1].x)", values)), 1e-12);
            // a single object is a list of one
            Assert.AreEqual(new GeoPoint(7, 8, 9), (GeoPoint)Eval("single[0]", values));
        }

        [TestMethod]
        public void InvalidIndex()
        {
            var values = new Dictionary<string, object> { ["pts"] = new List<GeoPoint> { new GeoPoint(1, 0, 0) } };
            StringAssert.Contains(Assert.ThrowsException<Exception>(() => Eval("pts[1]", values)).Message, "out of range");
            StringAssert.Contains(Assert.ThrowsException<Exception>(() => Eval("pts[0.5]", values)).Message, "whole number");
            StringAssert.Contains(Assert.ThrowsException<Exception>(() => Eval("pts[0", values)).Message, "]");
            Assert.ThrowsException<Exception>(() => Eval("pts]0[", values));
            Assert.ThrowsException<Exception>(() => Eval("pts[0,1]", values));
            Assert.ThrowsException<Exception>(() => Eval("pts[]", values));
        }

        /// <summary>count(list) and arclength(points[, i]): what a template needs to work along a polyline.</summary>
        [TestMethod]
        public void CountAndArcLengthOfAPointList()
        {
            var values = new Dictionary<string, object>
            {
                ["path"] = new List<GeoPoint> { new GeoPoint(0, 0, 0), new GeoPoint(3, 4, 0), new GeoPoint(3, 4, 10) }
            };
            Assert.AreEqual(3, Convert.ToInt32(Eval("count(path)", values)));
            Assert.AreEqual(15.0, Convert.ToDouble(Eval("arclength(path)", values)), 1e-12);
            Assert.AreEqual(0.0, Convert.ToDouble(Eval("arclength(path, 0)", values)), 1e-12);
            Assert.AreEqual(5.0, Convert.ToDouble(Eval("arclength(path, 1)", values)), 1e-12);
            Assert.AreEqual(5.0, Convert.ToDouble(Eval("arclength(path, -2)", values)), 1e-12);
            Assert.AreEqual(4.0, Convert.ToDouble(Eval("path[count(path) - 2].y", values)), 1e-12);
            Assert.ThrowsException<Exception>(() => Eval("arclength(path, 3)", values));
            Assert.ThrowsException<Exception>(() => Eval("count(1)", values));
        }

        /// <summary>min and max take the int results of round, floor and ceil like doubles.</summary>
        [TestMethod]
        public void MinMaxWithIntegerArguments()
        {
            Assert.AreEqual(2.0, Convert.ToDouble(Eval("min(round(2.4), 3.5)")), 1e-12);
            Assert.AreEqual(3, Eval("max(round(2.6), floor(1.5))"));
        }

        /// <summary>
        /// ICurve members are reachable although most curves implement them explicitly (Line.Length, BSpline.StartPoint),
        /// and the ICurve meaning wins over a public member of the same name (BSpline.PointAt takes a knot parameter).
        /// </summary>
        [TestMethod]
        public void CurveMembersThroughTheInterface()
        {
            Line line = Line.TwoPoints(new GeoPoint(0, 0, 0), new GeoPoint(10, 0, 0));
            BSpline spline = BSpline.Construct();
            spline.ThroughPoints(new[] { new GeoPoint(0, 0, 0), new GeoPoint(5, 5, 0), new GeoPoint(10, 0, 0) }, 3, false);
            var values = new Dictionary<string, object> { ["l"] = line, ["s"] = spline };
            Assert.AreEqual(10.0, Convert.ToDouble(Eval("l.Length", values)), 1e-12);
            Assert.AreEqual(5.0, Convert.ToDouble(Eval("l.PointAt(0.5).x", values)), 1e-12);
            Assert.AreEqual(1.0, Convert.ToDouble(Eval("normalize(l.DirectionAt(0.3)).x", values)), 1e-12);
            Assert.AreEqual(0.0, Convert.ToDouble(Eval("s.StartPoint.x", values)), 1e-9);
            Assert.AreEqual(10.0, Convert.ToDouble(Eval("s.PointAt(1).x", values)), 1e-9); // ICurve: position 1 is the end
            Assert.AreEqual(false, Eval("l.IsComposed", values));
            Assert.AreEqual(false, Eval("s.IsClosed", values));
        }

        /// <summary>type() tells the curve kinds apart; after the test the specific members are used.</summary>
        [TestMethod]
        public void TypeTestAndSpecificMembers()
        {
            Ellipse arc = Ellipse.Construct();
            arc.SetArcPlaneCenterRadiusAngles(Plane.XYPlane, new GeoPoint(1, 2, 0), 5, 0, Math.PI / 2);
            Polyline pl = Polyline.Construct();
            pl.SetPoints(new[] { new GeoPoint(0, 0, 0), new GeoPoint(10, 0, 0), new GeoPoint(10, 10, 0) }, false);
            var values = new Dictionary<string, object> { ["a"] = arc, ["p"] = pl, ["both"] = new List<ICurve> { arc, pl } };
            Assert.AreEqual("Ellipse", Eval("type(a)", values));
            Assert.AreEqual(true, Eval("type(a) == \"Ellipse\"", values));
            Assert.AreEqual(5.0, Convert.ToDouble(Eval("a.Radius", values)), 1e-12);
            Assert.AreEqual(2.0, Convert.ToDouble(Eval("a.Center.y", values)), 1e-12);
            Assert.AreEqual(true, Eval("a.IsArc", values));
            Assert.AreEqual("Polyline", Eval("type(p)", values));
            Assert.AreEqual(3, Convert.ToInt32(Eval("p.PointCount", values)));
            Assert.AreEqual(10.0, Convert.ToDouble(Eval("p.GetPoint(2).y", values)), 1e-12);
            Assert.AreEqual("List", Eval("type(both)", values));
            Assert.AreEqual("Polyline", Eval("type(both[1])", values));
            Assert.AreEqual(2, Convert.ToInt32(Eval("count(both)", values)));
            Assert.AreEqual(1, Convert.ToInt32(Eval("count(a)", values)));
            Assert.AreEqual(5.0, Convert.ToDouble(Eval("both[0].Radius", values)), 1e-12);
        }

        /// <summary>vertices, makepath, getcommonplane and getextent of single curves and lists of curves.</summary>
        [TestMethod]
        public void CurveListFunctions()
        {
            Line l1 = Line.TwoPoints(new GeoPoint(10, 0, 0), new GeoPoint(10, 10, 0));
            Line l2 = Line.TwoPoints(new GeoPoint(0, 0, 0), new GeoPoint(10, 0, 0)); // not in path order
            Ellipse arc = Ellipse.Construct();
            arc.SetArcPlaneCenterRadiusAngles(Plane.XYPlane, GeoPoint.Origin, 10, 0, Math.PI / 2);
            var values = new Dictionary<string, object> { ["lines"] = new List<ICurve> { l1, l2 }, ["a"] = arc };
            Assert.AreEqual("Path", Eval("type(makepath(lines))", values));
            Assert.AreEqual(20.0, Convert.ToDouble(Eval("makepath(lines).Length", values)), 1e-12);
            Assert.AreEqual(3, Convert.ToInt32(Eval("count(vertices(makepath(lines)))", values)));
            Assert.AreEqual(2, Convert.ToInt32(Eval("count(makepath(lines).SubCurves)", values)));
            Assert.AreEqual(10.0, Convert.ToDouble(Eval("vertices(makepath(lines))[-1].y", values)) + Convert.ToDouble(Eval("vertices(makepath(lines))[0].y", values)), 1e-12);
            Assert.AreEqual(1.0, Math.Abs(Convert.ToDouble(Eval("getcommonplane(lines).Normal.z", values))), 1e-12);
            Assert.AreEqual(10.0, Convert.ToDouble(Eval("getextent(lines).Xmax", values)), 1e-12);
            Assert.AreEqual(0.0, Convert.ToDouble(Eval("getextent(lines).Zmax", values)), 1e-12);
            Assert.AreEqual("Ellipse", Eval("type(makepath(a))", values), "a single curve stays what it is");
            Assert.ThrowsException<Exception>(() => Eval("vertices(a)", values), "an arc has no vertices without a maximal error");
            int n = Convert.ToInt32(Eval("count(vertices(a, 0.01))", values));
            Assert.IsTrue(n > 5, $"the arc is approximated by {n} points");
            Assert.AreEqual(10.0, Convert.ToDouble(Eval("distance(vertices(a, 0.01)[2], p(0,0,0))", values)), 1e-9);
            Assert.AreEqual(Convert.ToDouble(Eval("getextent(a).Xmax", values)), Convert.ToDouble(Eval("GetExtent(a).Xmax", values)), 0.0, "function names are case insensitive");
        }

        /// <summary>The curves of a profile (2D): a polyline keeps its vertices and gives them as 2D points.</summary>
        [TestMethod]
        public void VerticesOf2DCurves()
        {
            var values = new Dictionary<string, object>
            {
                ["c"] = new List<CADability.Curve2D.ICurve2D> { new CADability.Curve2D.Polyline2D(new[] { new GeoPoint2D(0, 0), new GeoPoint2D(4, 0), new GeoPoint2D(4, 3) }) }
            };
            Assert.AreEqual("Polyline2D", Eval("type(c)", values));
            Assert.AreEqual(3, Convert.ToInt32(Eval("count(vertices(c))", values)));
            Assert.AreEqual(3.0, Convert.ToDouble(Eval("c[0].GetVertex(2).y", values)), 1e-12);
            Assert.AreEqual(7.0, Convert.ToDouble(Eval("arclength(vertices(c))", values)), 1e-12);
            Assert.AreEqual(4.0, Convert.ToDouble(Eval("getextent(c).Right", values)), 1e-12);
        }
    }
}
