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
    }
}
