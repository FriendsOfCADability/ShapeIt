using System.Reflection;
using CADability.GeoObject;
using CADability.Curve2D;

namespace CADability.Tests
{
    // temporary probe, not to be committed
    [TestClass]
    public class ScratchRoundEdges5
    {
        public TestContext TestContext { get; set; } = null!;

        private static double CylGap(CylindricalSurface cyl, GeoPoint p)
        {   // signed distance to a (circular) cylinder: distance to the axis minus the radius
            return Geometry.DistPL(p, cyl.Location, cyl.Axis) - cyl.RadiusX;
        }

        private static double MinGap(SweptCircleSurface sw, CylindricalSurface cyl, double u)
        {   // smallest distance of the circle at u from the cylinder: coarse sampling, then golden section
            int n = 720; int best = 0; double bestD = double.MaxValue;
            for (int k = 0; k < n; k++)
            {
                double d = Math.Abs(CylGap(cyl, sw.PointAt(new GeoPoint2D(u, k * 2 * Math.PI / n))));
                if (d < bestD) { bestD = d; best = k; }
            }
            double a = (best - 1) * 2 * Math.PI / n, b = (best + 1) * 2 * Math.PI / n;
            Func<double, double> f = v => Math.Abs(CylGap(cyl, sw.PointAt(new GeoPoint2D(u, v))));
            for (int i = 0; i < 100; i++)
            {
                double c = b - 0.618 * (b - a), d = a + 0.618 * (b - a);
                if (f(c) < f(d)) b = d; else a = c;
            }
            return f((a + b) / 2);
        }

        [TestMethod]
        public void Probe()
        {
            string file = System.IO.Path.Combine(System.IO.Path.GetDirectoryName(typeof(ScratchRoundEdges5).Assembly.Location)!, "Files", "CDB", "RoundEdges5.cdb.json");
            Project project = Project.ReadFromFile(file, "cdb");
            Model model = project.GetActiveModel();
            Solid solid = model.AllObjects.OfType<Solid>().First();
            Shell shell = solid.Shells[0];
            List<Edge> edges = shell.Edges.Where(e => e.PrimaryFace.Surface is CylindricalSurface && e.SecondaryFace?.Surface is CylindricalSurface
                && !e.PrimaryFace.Surface.SameGeometry(e.PrimaryFace.Domain, e.SecondaryFace.Surface, e.SecondaryFace.Domain, 1e-6, out _)).ToList();
            TestContext.WriteLine($"faces={shell.Faces.Length} edges={shell.Edges.Length} candidate edges={edges.Count}");
            foreach (Edge e in edges) TestContext.WriteLine($"  edge {e.GetHashCode()} {e.Curve3D.GetType().Name} len={e.Curve3D.Length} adjacency={e.Adjacency()} size={shell.GetExtent(0.0).Size}");

            RoundEdges re = new RoundEdges(shell, edges, 5.0);
            MethodInfo mfs = typeof(RoundEdges).GetMethod("MakeFilletShell", BindingFlags.Instance | BindingFlags.NonPublic)!;
            foreach (Edge e in edges)
            {
                bool convex = e.Adjacency() == ShellExtensions.AdjacencyType.Convex;
                var sw0 = System.Diagnostics.Stopwatch.StartNew();
                Shell? fillet = mfs.Invoke(re, new object[] { e, 5.0, convex }) as Shell;
                TestContext.WriteLine($"MakeFilletShell {sw0.ElapsedMilliseconds} ms");
                if (fillet == null) { TestContext.WriteLine("no fillet"); continue; }
                Face sweptFace = fillet.Faces.First(f => f.Surface is SweptCircleSurface);
                foreach (Edge fe in sweptFace.AllEdges) if (fe.Curve3D is InterpolatedDualSurfaceCurve ic)
                {
                    double maxPosErr = 0, maxPosErrAt = 0;
                    for (int k = 0; k <= 50; k++) { double u = k / 50.0; double pe = Math.Abs(ic.PositionOf(ic.PointAt(u)) - u) * ic.Length; if (pe > maxPosErr) { maxPosErr = pe; maxPosErrAt = u; } }
                    {
                        MethodInfo ap = typeof(InterpolatedDualSurfaceCurve).GetMethod("ApproximatePosition", BindingFlags.Instance | BindingFlags.NonPublic)!;
                        FieldInfo hp = typeof(InterpolatedDualSurfaceCurve).GetField("hashedPositions", BindingFlags.Instance | BindingFlags.NonPublic)!;
                        object hashed = hp.GetValue(ic)!;
                        bool has0 = (bool)hashed.GetType().GetMethod("ContainsKey")!.Invoke(hashed, new object[] { 0.0 })!;
                        object?[] a = new object?[] { 0.0, null, null, null };
                        ap.Invoke(ic, a);
                        GeoPoint p0 = (GeoPoint)a[3]!;
                        hashed.GetType().GetMethod("Clear")!.Invoke(hashed, null);
                        object?[] a2 = new object?[] { 0.0, null, null, null };
                        ap.Invoke(ic, a2);
                        GeoPoint p0fresh = (GeoPoint)a2[3]!;
                        TestContext.WriteLine($"  hashed0={has0} ApproximatePosition(0)-start={p0 | ic.StartPoint:E2} fresh={p0fresh | ic.StartPoint:E2} fresh-PointAt(0)={p0fresh | ic.PointAt(0):E2}");
                    }
                    TestContext.WriteLine($"  IDSC tangential={ic.IsTangential} start-PointAt(0)={ic.StartPoint | ic.PointAt(0):E2} end-PointAt(1)={ic.EndPoint | ic.PointAt(1):E2} PositionOf(start)={ic.PositionOf(ic.StartPoint):E2} PositionOf(end)-1={ic.PositionOf(ic.EndPoint) - 1:E2} max |PositionOf(PointAt(u))-u|*len={maxPosErr:E2} at {maxPosErrAt}");
                }
                SweptCircleSurface sw = (SweptCircleSurface)sweptFace.Surface;
                InterpolatedDualSurfaceCurve spine = (InterpolatedDualSurfaceCurve)sw.Spine;
                ICurve evalSpine = (ICurve)typeof(SweptCircleSurface).GetProperty("PreciseSpine", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(sw)!;
                TestContext.WriteLine($"  evaluation spine: {evalSpine.GetType().Name}");
                CylindricalSurface off1 = (CylindricalSurface)spine.Surface1, off2 = (CylindricalSurface)spine.Surface2;
                CylindricalSurface top = (CylindricalSurface)e.PrimaryFace.Surface, bottom = (CylindricalSurface)e.SecondaryFace.Surface;
                TestContext.WriteLine($"edge {e.GetHashCode()} convex={convex} spine len={spine.Length} radius={sw.Radius} offRadii={off1.RadiusX},{off2.RadiusX} baseRadii={top.RadiusX},{bottom.RadiusX}");
                double maxSpine = 0, maxTop = 0, maxBot = 0, maxCircle = 0;
                int n = 400;
                for (int i = 0; i <= n; i++)
                {
                    double u = i / (double)n;
                    GeoPoint sp = evalSpine.PointAt(u);
                    double e1 = CylGap(off1, sp), e2 = CylGap(off2, sp);
                    maxSpine = Math.Max(maxSpine, Math.Max(Math.Abs(e1), Math.Abs(e2)));
                    // contact gap: smallest distance of the swept circle at u to the base cylinders (signed)
                    double gTop = MinGap(sw, top, u), gBot = MinGap(sw, bottom, u);
                    maxTop = Math.Max(maxTop, gTop); maxBot = Math.Max(maxBot, gBot);
                    if (false) TestContext.WriteLine($"   u={u:F3} spineErr={e1:E2},{e2:E2} gapTop={gTop:E2} gapBottom={gBot:E2}");
                }
                TestContext.WriteLine($"  max spineErr={maxSpine:E2} gapTop={maxTop:E2} gapBottom={maxBot:E2} circleRadiusErr={maxCircle:E2}");
                // the exact intersection points of the spine
                double maxBase = 0;
                FieldInfo bpf = typeof(InterpolatedDualSurfaceCurve).GetField("basePoints", BindingFlags.Instance | BindingFlags.NonPublic)!;
                Array bps = (Array)bpf.GetValue(spine)!;
                foreach (object bp in bps)
                {
                    GeoPoint p3d = (GeoPoint)bp.GetType().GetField("p3d")!.GetValue(bp)!;
                    maxBase = Math.Max(maxBase, Math.Max(Math.Abs(CylGap(off1, p3d)), Math.Abs(CylGap(off2, p3d))));
                }
                TestContext.WriteLine($"  basePoints={bps.Length} max basepoint error={maxBase:E2}");
                // the tangential edges of the swept face
                foreach (Edge fe in sweptFace.AllEdges)
                {
                    ICurve c = fe.Curve3D;
                    Face other = fe.OtherFace(sweptFace);
                    if (other == null) { TestContext.WriteLine($"  OPEN swept edge {c.GetType().Name} len={c.Length} start={c.StartPoint} end={c.EndPoint} fillet open edges={fillet.OpenEdgesExceptPoles.Length} faces={fillet.Faces.Length}"); continue; }
                    double dOther = 0, dSwept = 0;
                    for (int i = 0; i <= 100; i++)
                    {
                        GeoPoint p = c.PointAt(i / 100.0);
                        dOther = Math.Max(dOther, other.Surface.GetDistance(p));
                        dSwept = Math.Max(dSwept, sw.GetDistance(p));
                    }
                    TestContext.WriteLine($"  swept edge {c.GetType().Name} other={other.Surface.GetType().Name} maxDistOther={dOther:E2} maxDistSwept={dSwept:E2} startDistOther={other.Surface.GetDistance(c.StartPoint):E2}");
                }
            }
        }

        [TestMethod]
        public void RoundConcave()
        {
            string file = System.IO.Path.Combine(System.IO.Path.GetDirectoryName(typeof(ScratchRoundEdges5).Assembly.Location)!, "Files", "CDB", "RoundEdges5.cdb.json");
            Project project = Project.ReadFromFile(file, "cdb");
            Shell shell = project.GetActiveModel().AllObjects.OfType<Solid>().First().Shells[0];
            double before = shell.Volume(0.01);
            List<Edge> edges = shell.Edges.Where(e => e.PrimaryFace.Surface is CylindricalSurface && e.SecondaryFace?.Surface is CylindricalSurface
                && e.Adjacency() == ShellExtensions.AdjacencyType.Concave).ToList();
            TestContext.WriteLine($"concave edges: {edges.Count}");
            var sw0 = System.Diagnostics.Stopwatch.StartNew();
            Shell? res = null;
            try { res = new RoundEdges(shell, edges, 5.0).Execute(); }
            catch (Exception ex) { TestContext.WriteLine("EXC " + ex.Message.Replace((char)10, (char)32).Substring(0, Math.Min(300, ex.Message.Length)) + " @ " + string.Join(" | ", (ex.StackTrace ?? "").Split((char)10).Take(6))); }
            TestContext.WriteLine($"RoundEdges {sw0.ElapsedMilliseconds} ms");
            if (res == null) { TestContext.WriteLine("RESULT null"); return; }
            TestContext.WriteLine($"RESULT faces={res.Faces.Length} (before {shell.Faces.Length}) consistent={res.CheckConsistency()} open={res.OpenEdgesExceptPoles.Length} vol={res.Volume(0.01)} (before {before}) swept={res.Faces.Count(f => f.Surface is SweptCircleSurface)}");
        }

        [TestMethod]
        public void DifferenceBug13()
        {
            string file = System.IO.Path.Combine(System.IO.Path.GetDirectoryName(typeof(ScratchRoundEdges5).Assembly.Location)!, "Files", "BRep", "DifferenceBug13.cdb.json");
            var c = ShapeIt.BRepCaseReader.Read(file);
            Shell shell = c.Operands[0];
            foreach (Face f in shell.Faces)
            {
                if (f.Surface is not SweptCircleSurface sw) continue;
                ICurve eval = (ICurve)typeof(SweptCircleSurface).GetProperty("PreciseSpine", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(sw)!;
                TestContext.WriteLine($"face {f.GetHashCode()} spine={sw.Spine.GetType().Name} tangential={(sw.Spine as InterpolatedDualSurfaceCurve)?.IsTangential} eval={eval.GetType().Name} consistent={f.CheckConsistency()} radius={sw.Radius}");
                if (sw.Spine is InterpolatedDualSurfaceCurve idsc) TestContext.WriteLine($"   spine surfaces {idsc.Surface1.GetType().Name}/{idsc.Surface2.GetType().Name} len={idsc.Length}");
                double maxd = 0, maxAt = 0;
                for (int i = 0; i <= 200; i++) { double u = i / 200.0; double d = eval.PointAt(u) | sw.Spine.PointAt(u); if (d > maxd) { maxd = d; maxAt = u; } }
                TestContext.WriteLine($"   max |eval - spine| = {maxd:E2} at {maxAt}");
                foreach (double u in new double[] { 0.0, 5.66429135845003E-06, 1e-4, 1e-3, 0.999, 0.9999, 0.999990747297435, 1.0 })
                    TestContext.WriteLine($"   u={u:R} |eval-spine|={eval.PointAt(u) | sw.Spine.PointAt(u):E2} angle(dir)={new SweepAngle(eval.DirectionAt(u), sw.Spine.DirectionAt(u)).Radian:E2}");
                foreach (Edge e in f.AllEdges)
                {
                    ICurve2D c2d = e.Curve2D(f);
                    double ds = f.Surface.PointAt(c2d.StartPoint) | e.StartVertex(f).Position, de = f.Surface.PointAt(c2d.EndPoint) | e.EndVertex(f).Position;
                    TestContext.WriteLine($"   edge {e.Curve3D?.GetType().Name} startDev={ds:E2} endDev={de:E2} uvStart={c2d.StartPoint} uvEnd={c2d.EndPoint}");
                }
            }
        }
    }
}
