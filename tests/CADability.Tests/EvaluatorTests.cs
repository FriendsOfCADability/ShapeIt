using System.Collections.Generic;

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
    }
}
