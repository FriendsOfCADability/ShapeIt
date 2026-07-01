using CADability.Attribute;
using CADability.Curve2D;
using CADability.Shapes;
using CADability.Substitutes;
using CADability.UserInterface;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.Optimization;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.ComTypes;
using Wintellect.PowerCollections;

namespace CADability.GeoObject
{
    /// <summary>
    /// Encloses a surface (<see cref="ISurface"/>) within a rectangular u/v patch by a set of (possibly
    /// overlapping) parallelepipeds ("Spat", see the nested <see cref="ParEpi"/> class), organized in an
    /// OctTree for fast spatial queries. Used to quickly decide whether two surfaces intersect or a surface
    /// intersects a curve, and to provide good starting values for the subsequent Newton iterations.
    /// (Formerly named BoxedSurfaceEx; replaced the older AABB based BoxedSurface.)
    /// </summary>
    internal class ParallelepipedHull
    {
        public class DidntConverge : ApplicationException
        {
            public DidntConverge() { }
        }
        public class ParEpi : IOctTreeInsertable, IQuadTreeInsertable, IDebuggerVisualizer
        {   // ParallelEpiped, in German also called "Spat" or parallelotope
            // Defined by a point and three vectors that should form a right-handed system; for flat patches normal may be 0
            // diru is the averaged u direction, dirv the averaged v direction
            public GeoPoint loc;
            public GeoVector diru, dirv, normal;
            public ModOp toUnit; // mapping into the coordinate space of the parallelepiped, so that it describes the unit cube [0,1]
            public BoundingRect uvPatch; // encloses this patch
            public GeoPoint pll, plr, pul, pur; // the 4 corner points and the 4 directions
            public GeoVector nll, nlr, nul, nur; // the normals at the corners
            public bool isFlat;
            internal bool isFolded; // here we cannot compute with the derivatives, because the normals point every which way
                                    // WeakReference<Matrix> quad3dTo2d; // a matrix, which converts 3d points to 2d uv positions with a quadratic approximation
            WeakReference quad3dTo2d; // a matrix, which converts 3d points to 2d uv positions with a quadratic approximation
#if DEBUG
            public static int idCounter = 0;
            public int id;
#endif
            public ParEpi()
            {
#if DEBUG
                id = idCounter++;
#endif
            }
            #region IOctTreeInsertable Members
            BoundingBox IOctTreeInsertable.GetExtent(double precision)
            {
                GeoPoint locz = loc + normal;
                return new BoundingBox(loc, loc + diru, loc + dirv, loc + diru + dirv, locz, locz + diru, locz + dirv, locz + diru + dirv);
            }
            bool IOctTreeInsertable.HitTest(ref BoundingBox cube, double precision)
            {
                return cube.Interferes(loc, diru, dirv, normal);
            }
            bool IOctTreeInsertable.HitTest(Projection projection, BoundingRect rect, bool onlyInside)
            {   // hopefully not needed
                throw new Exception("The method or operation is not implemented.");
            }
            double IOctTreeInsertable.Position(GeoPoint fromHere, GeoVector direction, double precision)
            {   // hopefully not needed
                throw new Exception("The method or operation is not implemented.");
            }
            bool IOctTreeInsertable.HitTest(Projection.PickArea area, bool onlyInside)
            {
                throw new Exception("The method or operation is not implemented.");
            }
            #endregion
            public double Size
            {
                get
                {
                    return diru.Length + dirv.Length + normal.Length;
                }
            }
            public GeoPoint GetCenter()
            {
                return loc + 0.5 * diru + 0.5 * dirv + 0.5 * normal;
            }
            public bool Interferes(GeoPoint startPoint, GeoVector direction, double maxdist, bool onlyForward)
            {
                if (isFlat)
                {
                    // loc + a*diru + b*dirv == sp + c*direction
                    // a*diru + b*dirv -c*direction = sp - loc
                    Matrix m = DenseMatrix.OfRowArrays(diru, dirv, direction);
                    Vector s = (Vector)m.Transpose().Solve(new DenseVector(startPoint - loc));
                    if (s.IsValid())
                    {
                        return (s[0] >= 0.0) && (s[0] <= 1.0) && (s[1] >= 0.0) && (s[1] <= 1.0);
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    return BoundingBox.UnitBoundingCube.Interferes(toUnit * startPoint, toUnit * direction, maxdist, onlyForward);
                }
            }
            public bool Interferes(GeoPoint startPoint, GeoPoint endPoint)
            {
                if (isFlat)
                {
                    // loc + a*diru + b*dirv == sp + c*direction
                    // a*diru + b*dirv -c*direction = sp - loc
                    Matrix m = DenseMatrix.OfRowArrays(diru, dirv, endPoint - startPoint);
                    Vector s = (Vector)m.Transpose().Solve(new DenseVector(startPoint - loc));
                    if (s.IsValid())
                    {
                        return (s[0] >= 0.0) && (s[0] <= 1.0) && (s[1] >= 0.0) && (s[1] <= 1.0);
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    startPoint = toUnit * startPoint;
                    endPoint = toUnit * endPoint;
                    return BoundingBox.UnitBoundingCube.Interferes(ref startPoint, ref endPoint);
                }
            }
            public bool ClipLine(ref GeoPoint sp, ref GeoPoint ep)
            {
                GeoPoint usp = toUnit * sp;
                GeoPoint uep = toUnit * ep;
                if (BoundingBox.UnitBoundingCube.ClipLine(ref usp, ref uep))
                {
                    ModOp inv = toUnit.GetInverse();
                    sp = inv * usp;
                    ep = inv * uep;
                    return true;
                }
                return false;
            }
            public Face[] GetSides()
            {
                GeoPoint locz = loc + normal;
                GeoPoint[] points = new GeoPoint[] { loc, loc + diru, loc + dirv, loc + diru + dirv, locz, locz + diru, locz + dirv, locz + diru + dirv };
                Face[] faces = new Face[6];
                faces[0] = Face.MakeFace(points[0], points[1], points[3], points[2]);
                faces[1] = Face.MakeFace(points[0], points[4], points[5], points[1]);
                faces[2] = Face.MakeFace(points[0], points[2], points[6], points[4]);
                faces[3] = Face.MakeFace(points[1], points[5], points[7], points[3]);
                faces[4] = Face.MakeFace(points[2], points[3], points[7], points[6]);
                faces[5] = Face.MakeFace(points[4], points[6], points[7], points[5]);
                return faces;
            }
            public void GetPlanes(out GeoPoint[] pos, out GeoVector[] dirx, out GeoVector[] diry)
            {
                pos = new GeoPoint[6];
                dirx = new GeoVector[6];
                diry = new GeoVector[6];
                GeoPoint locz = loc + normal;
                GeoPoint[] points = new GeoPoint[] { loc, loc + diru, loc + dirv, loc + diru + dirv, locz, locz + diru, locz + dirv, locz + diru + dirv };
                pos[0] = points[1];
                pos[1] = points[4];
                pos[2] = points[2];
                pos[3] = points[5];
                pos[4] = points[3];
                pos[5] = points[6];
                dirx[0] = points[0] - points[1];
                dirx[1] = points[0] - points[4];
                dirx[2] = points[0] - points[2];
                dirx[3] = points[1] - points[5];
                dirx[4] = points[2] - points[3];
                dirx[5] = points[4] - points[6];
                diry[0] = points[3] - points[1];
                diry[1] = points[5] - points[4];
                diry[2] = points[6] - points[2];
                diry[3] = points[7] - points[5];
                diry[4] = points[7] - points[3];
                diry[5] = points[7] - points[6];
            }
            public Solid GetSolid()
            {
                Shell sh = Shell.Construct();
                sh.SetFaces(GetSides());
                Solid so = Solid.Construct();
                so.SetShell(sh);
                return so;
            }
            internal Solid AsBox
            {
                get
                {
                    return GetSolid();
                }
            }
            internal BoundingBox BoundingBox
            {
                get
                {
                    GeoPoint locz = loc + normal;
                    return new BoundingBox(loc, loc + diru, loc + dirv, loc + diru + dirv, locz, locz + diru, locz + dirv, locz + diru + dirv);
                }
            }
            internal bool Interferes(GeoPoint tb1, GeoPoint tb2, GeoPoint t3, GeoPoint t4)
            {   // test against tetraeder
                return BoundingBox.UnitBoundingCube.Interferes(toUnit * tb1, toUnit * tb2, toUnit * t3, toUnit * t4);
            }
            internal bool Interferes(ParEpi other)
            {
                if (isFlat)
                {
                    if (other.isFlat)
                    {   // this is not correct!!!
                        GeoPoint p1 = other.pll;
                        GeoPoint p2 = other.plr;
                        GeoPoint p3 = other.pul;
                        GeoPoint p4 = other.pur;
                        if (this.ClipLine(ref p1, ref p2)) return true;
                        if (this.ClipLine(ref p2, ref p3)) return true;
                        if (this.ClipLine(ref p3, ref p4)) return true;
                        if (this.ClipLine(ref p4, ref p1)) return true;
                        return false;
                    }
                    else
                    {
                        return other.Interferes(this);
                    }
                }
                return BoundingBox.UnitBoundingCube.Interferes(toUnit * other.loc, toUnit * other.diru, toUnit * other.dirv, toUnit * other.normal);
            }
            internal bool Interferes(ICurve curve, double u1, double u2, GeoPoint tb1, GeoPoint tb2, GeoPoint t3, GeoPoint t4)
            {   // does the curve segment pass through this ParEpi?
                // end points are tested too often, should make a private method without end point test!
                if (!Interferes(tb1, tb2, t3, t4)) return false;
                if (BoundingBox.UnitBoundingCube.Contains(toUnit * tb1)) return true;
                if (BoundingBox.UnitBoundingCube.Contains(toUnit * tb2)) return true;
                // start and end point not inside, but the tetraeder is, so it must be split
                if (u2 - u1 > 1e-3)
                {
                    GeoPoint tv1, tv2, tv3, tv4, pm;
                    double parm;
                    TetraederHull.SplitTetraeder(curve, tb1, tb2, u1, u2, out pm, out parm, out tv1, out tv2, out tv3, out tv4);
                    return Interferes(curve, u1, parm, tb1, pm, tv1, tv2) | Interferes(curve, parm, u2, pm, tb2, tv3, tv4);
                }
                return true; // subdivision too small and nothing decided yet, return true to be safe
            }
            internal bool Contains(GeoPoint p)
            {
                return BoundingBox.UnitBoundingCube.Contains(toUnit * p);
            }

            /// <summary>
            /// Fast calculation for a starting position for a PositionOf for the provided surface and a 3d point p3d, of which the position is queried.
            /// Uses a weak reference to a matrix, which is calculated on the first call or when it has been claimed by GC
            /// </summary>
            /// <param name="p3d"></param>
            /// <param name="surface"></param>
            /// <returns></returns>
            internal GeoPoint2D PositionOf(GeoPoint p3d, ISurface surface)
            {
                // calculate quadric for u anf v parameter
                // ax² + by² + cz² + dxy + eyz + fxz + gx + hy + iz + j == u (same for v)
                // we need 10 samples for a linear system, or we could make a larger system and solve it with QR
                Matrix x = null;
                if (quad3dTo2d != null && quad3dTo2d.IsAlive)
                {
                    x = quad3dTo2d.Target as Matrix;
                }
                if (x == null)
                {
                    Matrix m = new DenseMatrix(10, 10);
                    Matrix b = new DenseMatrix(10, 2);
                    for (int i = 0; i < 10; i++)
                    {
                        GeoPoint2D uv;
                        if (i < 4)
                        {
                            uv.x = uvPatch.Left + (i % 3) * uvPatch.Width / 2.0;
                            uv.y = uvPatch.Bottom + (i / 3) * uvPatch.Height / 2.0;
                        }
                        else if (i < 8)
                        {
                            uv.x = uvPatch.Left + ((i + 1) % 3) * uvPatch.Width / 2.0;
                            uv.y = uvPatch.Bottom + ((i + 1) / 3) * uvPatch.Height / 2.0;
                        }
                        else
                        {
                            if (uvPatch.Width > uvPatch.Height)
                            {
                                uv.x = uvPatch.Left + (i % 4 + 1) * uvPatch.Width / 3.0;
                                uv.y = uvPatch.Bottom + uvPatch.Height / 2.0;
                            }
                            else
                            {
                                uv.x = uvPatch.Left + uvPatch.Width / 2.0;
                                uv.y = uvPatch.Bottom + (i % 4 + 1) * uvPatch.Height / 3.0;
                            }
                        }
                        b[i, 0] = uv.x;
                        b[i, 1] = uv.y;
                        GeoPoint p = surface.PointAt(uv);
                        m[i, 0] = p.x * p.x;
                        m[i, 1] = p.y * p.y;
                        m[i, 2] = p.z * p.z;
                        m[i, 3] = p.x * p.y;
                        m[i, 4] = p.y * p.z;
                        m[i, 5] = p.x * p.z;
                        m[i, 6] = p.x;
                        m[i, 7] = p.y;
                        m[i, 8] = p.z;
                        m[i, 9] = 1.0;
                    }
                    x = (Matrix)m.Solve(b);
                    if (!x.IsValid()) x = new DenseMatrix(0, 0); // we need this to state, that there is no quadratic form (maybe linear in one direction)
                    quad3dTo2d = new WeakReference(x);
                }
                if (x.IsValid() && x.RowCount > 0)
                {
                    GeoPoint2D res = new GeoPoint2D(
                    x[0, 0] * p3d.x * p3d.x +
                    x[1, 0] * p3d.y * p3d.y +
                    x[2, 0] * p3d.z * p3d.z +
                    x[3, 0] * p3d.x * p3d.y +
                    x[4, 0] * p3d.y * p3d.z +
                    x[5, 0] * p3d.x * p3d.z +
                    x[6, 0] * p3d.x +
                    x[7, 0] * p3d.y +
                    x[8, 0] * p3d.z +
                    x[9, 0] * 1.0,
                    x[0, 1] * p3d.x * p3d.x +
                    x[1, 1] * p3d.y * p3d.y +
                    x[2, 1] * p3d.z * p3d.z +
                    x[3, 1] * p3d.x * p3d.y +
                    x[4, 1] * p3d.y * p3d.z +
                    x[5, 1] * p3d.x * p3d.z +
                    x[6, 1] * p3d.x +
                    x[7, 1] * p3d.y +
                    x[8, 1] * p3d.z +
                    x[9, 1] * 1.0);
                    if (uvPatch.ContainsEps(res, -0.1)) return res; // allow 10% outside (maybe this is too much)
                }
                return GeoPoint2D.Invalid;
            }

            public BoundingRect GetExtent()
            {
                return uvPatch;
            }

            public bool HitTest(ref BoundingRect rect, bool includeControlPoints)
            {
                return uvPatch.Interferes(ref rect);
            }

            GeoObjectList IDebuggerVisualizer.GetList()
            {
#if DEBUG
                Solid sld = GetSolid();
                IntegerProperty ip = new IntegerProperty(id, "Debug.Hint");
                sld.UserData.Add("Debug", ip);
                Layer l = new Attribute.Layer("ParEpi");
                l.Transparency = 180;
                sld.Layer = l;
                GeoObjectList res = new GeoObjectList(sld);
                res.Add(Line.TwoPoints(pll, plr));
                res.Add(Line.TwoPoints(plr, pur));
                res.Add(Line.TwoPoints(pur, pul));
                res.Add(Line.TwoPoints(pul, pll));
                res.Add(Line.TwoPoints(pll, pll + normal.Length * nll.Normalized));
                res.Add(Line.TwoPoints(plr, plr + normal.Length * nlr.Normalized));
                res.Add(Line.TwoPoints(pul, pul + normal.Length * nul.Normalized));
                res.Add(Line.TwoPoints(pur, pur + normal.Length * nur.Normalized));
                return new GeoObjectList(sld);
#else
                return null;
#endif
            }

            /// <summary>
            /// Tests whether the tetraeder spanned by the four points <paramref name="t1"/>, <paramref name="t2"/>,
            /// <paramref name="t3"/> and <paramref name="t4"/> (as provided by a <see cref="TetraederHull"/> segment)
            /// penetrates this parallelepiped. <paramref name="t1"/> is the curve point with parameter
            /// <paramref name="tstart"/>, <paramref name="t2"/> the curve point with parameter <paramref name="tend"/>;
            /// <paramref name="t3"/> and <paramref name="t4"/> are the apex points of the tetraeder. On success
            /// <paramref name="t"/> returns a good starting value for the curve parameter to start a Newton iteration
            /// for the actual curve/surface intersection: if the secant t1->t2 runs through this parallelepiped it is
            /// the middle of the part that lies inside, otherwise it is the foot point on the secant of the
            /// parallelepiped corner that comes closest to it.
            /// </summary>
            internal bool InterferesWithTetraeder(GeoPoint t1, GeoPoint t2, GeoPoint t3, GeoPoint t4, double tstart, double tend, out double t)
            {
                // Liang-Barsky clipping of a single half space "p*f <= q" for the segment parameter f in [lo,hi].
                bool ClipAxis(double p, double q, ref double lo, ref double hi)
                {
                    if (Math.Abs(p) < 1e-13) return q >= 0.0; // segment parallel to this plane: inside iff q>=0
                    double r = q / p;
                    if (p < 0.0) { if (r > hi) return false; if (r > lo) lo = r; }
                    else { if (r < lo) return false; if (r < hi) hi = r; }
                    return true;
                }

                // Default starting parameter: the middle of the curve segment. Used when the tetraeder does not
                // interfere or no better estimate can be found.
                t = (tstart + tend) / 2.0;

                // Does the tetraeder really penetrate this parallelepiped?
                if (!Interferes(t1, t2, t3, t4)) return false;

                // Clip the secant t1->t2 against this parallelepiped (which is the unit cube in toUnit space).
                GeoPoint p0 = toUnit * t1;
                GeoPoint p1 = toUnit * t2;
                GeoVector dir = p1 - p0;
                double fmin = 0.0, fmax = 1.0;
                bool through = ClipAxis(-dir.x, p0.x, ref fmin, ref fmax)        // x >= 0
                            && ClipAxis(dir.x, 1.0 - p0.x, ref fmin, ref fmax)   // x <= 1
                            && ClipAxis(-dir.y, p0.y, ref fmin, ref fmax)        // y >= 0
                            && ClipAxis(dir.y, 1.0 - p0.y, ref fmin, ref fmax)   // y <= 1
                            && ClipAxis(-dir.z, p0.z, ref fmin, ref fmax)        // z >= 0
                            && ClipAxis(dir.z, 1.0 - p0.z, ref fmin, ref fmax)   // z <= 1
                            && fmin <= fmax;

                double frac;
                if (through)
                {
                    // The secant passes through the parallelepiped: take the middle of the part that lies inside.
                    frac = (fmin + fmax) / 2.0;
                }
                else
                {
                    // The secant misses the parallelepiped although the (curved, "fat") tetraeder hits it. Use the
                    // foot point on the secant of the parallelepiped corner that comes closest to the secant.
                    GeoPoint locz = loc + normal;
                    GeoPoint[] corners = new GeoPoint[] { loc, loc + diru, loc + dirv, loc + diru + dirv,
                                                          locz, locz + diru, locz + dirv, locz + diru + dirv };
                    GeoVector secant = t2 - t1;
                    double len2 = secant * secant;
                    double bestDist2 = double.MaxValue;
                    frac = 0.5;
                    for (int i = 0; i < corners.Length; i++)
                    {
                        double f = len2 > 1e-26 ? ((corners[i] - t1) * secant) / len2 : 0.5;
                        double fc = Math.Max(0.0, Math.Min(1.0, f));
                        GeoVector offset = corners[i] - (t1 + fc * secant);
                        double dist2 = offset * offset;
                        if (dist2 < bestDist2)
                        {
                            bestDist2 = dist2;
                            frac = fc;
                        }
                    }
                }
                t = tstart + frac * (tend - tstart);
                return true;
            }

            public object ReferencedObject
            {
                get { return null; }
            }

            public double Volume
            {
                get
                {
                    return normal * (diru ^ dirv);
                }
            }
        }
        QuadTree<ParEpi> quadtree;
        OctTree<ParEpi> octtree;
        BoundingRect uvbounds;
        ISurface surface;
        double[] uSingularities;
        double[] vSingularities;
        GeoPoint2D[] extrema;
        public ParallelepipedHull(ISurface surface, BoundingRect extent)
        {
            if (extent.IsEmpty() || extent.IsInfinite) throw new ApplicationException("ParallelepipedHull with undefined extent");
            this.surface = surface;
            uSingularities = surface.GetUSingularities();
            vSingularities = surface.GetVSingularities();
            double[] usteps, vsteps;
            surface.GetSafeParameterSteps(extent.Left, extent.Right, extent.Bottom, extent.Top, out usteps, out vsteps);
            // make at least 4 ParEpis
            if (usteps.Length == 2) usteps = new double[] { usteps[0], (usteps[0] + usteps[1]) / 2.0, usteps[1] };
            if (vsteps.Length == 2) vsteps = new double[] { vsteps[0], (vsteps[0] + vsteps[1]) / 2.0, vsteps[1] };
            // experimental:
#if DEBUGx
            if (surface is NurbsSurface)
            {
                GeoPoint2D cnt = extent.GetCenter();
                GeoPoint ll = surface.PointAt(extent.GetLowerLeft());
                GeoPoint lr = surface.PointAt(extent.GetLowerRight());
                GeoPoint ul = surface.PointAt(extent.GetUpperLeft());
                GeoPoint ur = surface.PointAt(extent.GetUpperRight());
                GeoVector normal = (ur - ll) ^ (lr - ul);
                DebuggerContainer dc = new DebuggerContainer();
                Line l = Line.Construct();
                l.SetTwoPoints(ur, ll);
                // dc.Add(l);
                l = Line.Construct();
                l.SetTwoPoints(lr, ul);
                // dc.Add(l);
                l = Line.Construct();
                GeoPoint mp = new GeoPoint(ll, lr, ul, ur);
                l.SetTwoPoints(mp, mp + 10 * normal);
                // dc.Add(l);

                GeoPoint pc = surface.PointAt(cnt);
                GeoVector diru, dirv;
                surface.DerivationAt(cnt, out pc, out diru, out dirv);
                l = Line.Construct();
                l.SetTwoPoints(pc, pc + 10 * diru);
                dc.Add(l);
                l = Line.Construct();
                l.SetTwoPoints(pc, pc + 10 * dirv);
                dc.Add(l);

                GeoVector du;
                GeoVector dv;
                GeoVector duu;
                GeoVector dvv;
                GeoVector duv;
                surface.Derivation2At(cnt, out pc, out du, out dv, out duu, out dvv, out duv);
                l = Line.Construct();
                l.SetTwoPoints(pc, pc + 10 * du);
                dc.Add(l);
                l = Line.Construct();
                l.SetTwoPoints(pc, pc + 10 * dv);
                dc.Add(l);


                GeoPoint2D better;
                double umin, umax, vmin, vmax;
                (surface as NurbsSurface).GetNaturalBounds(out umin, out umax, out vmin, out vmax);
                findMaxPlaneDist(cnt, (umax - umin) + (vmax - vmin), normal, out better);
                DirMinimum dm = new DirMinimum(surface);
                GeoPoint2D uv;
                GeoPoint val;
                bool ok = dm.GetSR1Rect(normal, extent, out uv, out val);

                findMaxPlaneDist(extent.GetLowerLeft(), extent.GetLowerRight(), normal, out uv);
                GeoPoint p1 = surface.PointAt(uv);
                findMaxPlaneDist(extent.GetLowerRight(), extent.GetUpperRight(), normal, out uv);
                GeoPoint p2 = surface.PointAt(uv);
                findMaxPlaneDist(extent.GetUpperRight(), extent.GetUpperLeft(), normal, out uv);
                GeoPoint p3 = surface.PointAt(uv);
                findMaxPlaneDist(extent.GetUpperLeft(), extent.GetLowerLeft(), normal, out uv);
                GeoPoint p4 = surface.PointAt(uv);
                l = Line.Construct();
                l.SetTwoPoints(p1, p2);
                dc.Add(l);
                l = Line.Construct();
                l.SetTwoPoints(p2, p3);
                dc.Add(l);
                l = Line.Construct();
                l.SetTwoPoints(p3, p4);
                dc.Add(l);
                l = Line.Construct();
                l.SetTwoPoints(p4, p1);
                dc.Add(l);

                DebuggerContainer dc1 = new DebuggerContainer();
                GeoObjectList dbggr = (surface as NurbsSurface).DebugGrid;
            }
#endif
            // don't uncomment the following, unless there is a better solution for C.17.2.063.0000.STEP
            //if (surface is NurbsSurface)
            //{   // this still needs to be clarified in NurbsSurface. But AddCube also takes care, and for not too wild NurbsSurfaces this should work well
            //    usteps = new double[] { extent.Left, extent.Left + extent.Width * 0.333, extent.Left + extent.Width * 0.666, extent.Right };
            //    vsteps = new double[] { extent.Bottom, extent.Bottom + extent.Height * 0.333, extent.Bottom + extent.Height * 0.666, extent.Top };
            //}
            uvbounds = new BoundingRect(usteps[0], vsteps[0], usteps[usteps.Length - 1], vsteps[vsteps.Length - 1]);
            octtree = new OctTree<ParEpi>(surface.GetPatchExtent(new BoundingRect(usteps[0], vsteps[0], usteps[usteps.Length - 1], vsteps[vsteps.Length - 1]), true), 0.0); // better true
            quadtree = new QuadTree<ParEpi>(extent);
            quadtree.MaxDeepth = -1; // dynamic
            for (int i = 0; i < usteps.Length - 1; ++i)
            {
                for (int j = 0; j < vsteps.Length - 1; ++j)
                {
                    AddCube(new BoundingRect(usteps[i], vsteps[j], usteps[i + 1], vsteps[j + 1]));
                }
            }
            // extrema = surface.GetExtrema(); computed only on demand
#if DEBUG
            int num = octtree.GetAllObjects().Length;
            if (num > maxCnt)
            {
                maxCnt = num;
                // System.Diagnostics.Trace.WriteLine("ParallelepipedHull, count: " + maxCnt.ToString());
            }
            //ParEpi[] allParEpis = octtree.GetAllObjects();
            //double maxVloume = 0;
            //double avgVolume = 0;
            //DebuggerContainer[] dcs = new DebuggerContainer[allParEpis.Length];
            //ColorDef cdGreen = new ColorDef("green", Color.Green);
            //ColorDef cdRed = new ColorDef("red", Color.Red);
            //Layer trnsp = new Layer("transparent");
            //trnsp.Transparency = 128;
            //Layer opq = new Layer("opaque");
            //for (int i = 0; i < allParEpis.Length; i++)
            //{
            //    double v = allParEpis[i].Volume;
            //    if (v > maxVloume) maxVloume = v;
            //    avgVolume += v;
            //    dcs[i] = new DebuggerContainer();
            //    Solid sld = allParEpis[i].AsBox;
            //    sld.ColorDef = cdGreen;
            //    sld.Layer = trnsp;
            //    dcs[i].Add(sld);
            //    Face fc = Face.MakeFace(surface, allParEpis[i].uvPatch);
            //    fc.ColorDef = cdRed;
            //    fc.Layer = opq;
            //    dcs[i].Add(fc);
            //    Line l = Line.TwoPoints(allParEpis[i].pll, allParEpis[i].pll + allParEpis[i].nll);
            //    l.ColorDef = cdRed;
            //    l.Layer = opq;
            //    dcs[i].Add(l);
            //    l = Line.TwoPoints(allParEpis[i].plr, allParEpis[i].plr + allParEpis[i].nlr);
            //    l.ColorDef = cdRed;
            //    l.Layer = opq;
            //    dcs[i].Add(l);
            //    l = Line.TwoPoints(allParEpis[i].pul, allParEpis[i].pul + allParEpis[i].nul);
            //    l.ColorDef = cdRed;
            //    l.Layer = opq;
            //    dcs[i].Add(l);
            //    l = Line.TwoPoints(allParEpis[i].pur, allParEpis[i].pur + allParEpis[i].nur);
            //    l.ColorDef = cdRed;
            //    l.Layer = opq;
            //    dcs[i].Add(l);
            //}
            //avgVolume /= allParEpis.Length;
            //if (maxVloume > avgVolume * 10)
            //{

            //}
#endif
        }
#if DEBUG
        static int maxCnt = 0;
        static int dbgcnt = 0;
#endif
        private void AddCube(BoundingRect uvPatch)
        {
            if (surface is IRestrictedDomain rd)
            {
                if (!(rd.IsInside(uvPatch.GetLowerLeft()) || rd.IsInside(uvPatch.GetLowerRight()) || rd.IsInside(uvPatch.GetUpperLeft()) || rd.IsInside(uvPatch.GetUpperRight()))) return;
            }
            bool singu = IsUSingularity(uvPatch.Left) || IsUSingularity(uvPatch.Right);
            bool singv = IsVSingularity(uvPatch.Bottom) || IsVSingularity(uvPatch.Top);
            ParEpi cube = new ParEpi();
            cube.uvPatch = uvPatch;
            cube.pll = surface.PointAt(uvPatch.GetLowerLeft());
            cube.plr = surface.PointAt(uvPatch.GetLowerRight());
            cube.pul = surface.PointAt(uvPatch.GetUpperLeft());
            cube.pur = surface.PointAt(uvPatch.GetUpperRight());
            cube.nll = surface.GetNormal(uvPatch.GetLowerLeft());
            cube.nlr = surface.GetNormal(uvPatch.GetLowerRight());
            cube.nul = surface.GetNormal(uvPatch.GetUpperLeft());
            cube.nur = surface.GetNormal(uvPatch.GetUpperRight());
            cube.nll.NormIfNotNull();
            cube.nlr.NormIfNotNull();
            cube.nul.NormIfNotNull();
            cube.nur.NormIfNotNull();
            bool toosmall = uvPatch.Size < (uvbounds.Size * 1e-5) || uvPatch.Width < (uvbounds.Width * 1e-3) || uvPatch.Height < (uvbounds.Height * 1e-3);
            // emergency brake: at singular locations the patch becomes too small
            GeoVector udirl = surface.UDirection(uvPatch.GetMiddleLeft());
            GeoVector udirr = surface.UDirection(uvPatch.GetMiddleRight());
            GeoVector vdirb = surface.VDirection(uvPatch.GetLowerMiddle());
            GeoVector vdirt = surface.VDirection(uvPatch.GetUpperMiddle());
            udirl.NormIfNotNull();
            udirr.NormIfNotNull();
            vdirb.NormIfNotNull();
            vdirt.NormIfNotNull();
            // the normals are normalized, right???
            // the normals should not span angles larger than 45° (the value can still be tuned better
            double lim = Math.Sqrt(2.0) / 2.0; // 45°, but there is no logical reason for it
            lim = 0.5; // changed to 60°
                       //if ((Math.Abs(cube.nll * cube.nlr) < lim || Math.Abs(cube.nll * cube.nul) < lim || Math.Abs(cube.nll * cube.nur) < lim ||
                       //    Math.Abs(cube.nlr * cube.nul) < lim || Math.Abs(cube.nlr * cube.nur) < lim || Math.Abs(cube.nul * cube.nur) < lim) && !toosmall)
                       // Math.Abs removed, because otherwise very large angles (>135°) would be dropped...
            GeoVector ncnt = surface.GetNormal(uvPatch.GetCenter());
            ncnt.NormIfNotNull();
            // normals at singular points are invalid. Replace them with normals at the center
            if (IsUSingularity(uvPatch.Left))
            {
                cube.nll = cube.nul = ncnt;
            }
            if (IsUSingularity(uvPatch.Right))
            {
                cube.nlr = cube.nur = ncnt;
            }
            if (IsVSingularity(uvPatch.Bottom))
            {
                cube.nll = cube.nlr = ncnt;
            }
            if (IsVSingularity(uvPatch.Top))
            {
                cube.nul = cube.nur = ncnt;
            }
            bool isFolded = false; // the patch contains a fold; the corner normals point in completely different directions and we have reached a twentieth of the surface
            if ((cube.nll * ncnt) < 0 || (cube.nlr * ncnt) < 0 || (cube.nul * ncnt) < 0 || (cube.nur * ncnt) < 0)
            {
#if DEBUG
                GeoObjectList dbgl = new GeoObjectList();
                for (int i = 0; i < 10; i++)
                {
                    GeoPoint[] pnts = new GeoPoint[10];
                    for (int j = 0; j < 10; j++)
                    {
                        GeoPoint2D uv = new GeoPoint2D(cube.uvPatch.Left + i * cube.uvPatch.Width / 9, cube.uvPatch.Bottom + j * cube.uvPatch.Height / 9);
                        pnts[j] = surface.PointAt(uv);
                    }
                    try
                    {
                        if (!Precision.IsEqual(pnts))
                        {
                            Polyline pl = Polyline.FromPoints(pnts);
                            dbgl.Add(pl);
                        }
                    }
                    catch (PolylineException)
                    { }
                }
                dbgl.Add(Line.TwoPoints(cube.pll, cube.pll + 10 * cube.nll));
                dbgl.Add(Line.TwoPoints(cube.plr, cube.plr + 10 * cube.nlr));
                dbgl.Add(Line.TwoPoints(cube.pul, cube.pul + 10 * cube.nul));
                dbgl.Add(Line.TwoPoints(cube.pur, cube.pur + 10 * cube.nur));
                dbgl.Add(Line.TwoPoints(surface.PointAt(uvPatch.GetCenter()), surface.PointAt(uvPatch.GetCenter()) + 20 * ncnt));
                DebuggerContainer dcextra = new DebuggerContainer();
                for (int i = 0; i < 20; i++)
                {
                    GeoPoint2D uv = new GeoPoint2D((cube.uvPatch.Left + cube.uvPatch.Right) / 2, cube.uvPatch.Bottom + i * 1.0 / 19);
                    GeoPoint p0 = surface.PointAt(uv);
                    try
                    {
                        GeoPoint p1 = p0 + 10 * surface.UDirection(uv).Normalized;
                        dcextra.Add(Line.TwoPoints(p0, p1), i);
                    }
                    catch { }
                }
#endif
                isFolded = uvPatch.Size < (uvbounds.Size * 0.05) || uvPatch.Width < (uvbounds.Width * 0.05) || uvPatch.Height < (uvbounds.Height * 0.05);
            }

            if (((cube.nll * cube.nlr) < lim || (cube.nll * cube.nul) < lim || (cube.nll * cube.nur) < lim ||
                (cube.nlr * cube.nul) < lim || (cube.nlr * cube.nur) < lim || (cube.nul * cube.nur) < lim ||
                (udirl * udirr) < 0.1 || (vdirb * vdirt) < 0.1) && !toosmall && !isFolded)
            // condition (udirl * udirr) < 0.1 || (vdirb * vdirt) < 0.1 ) is for flat rotated curves, which are rotated more than about 65°
            {   // condition (udirl * udirr) < lim || (vdirb * vdirt) < lim introduced, because an almost flat torus patch that spans 180° in u but only little in v
                // would otherwise pass the test, which is not good!
                // simply splitting into four here sometimes yields bad results
                // therefore better check here and split only into two
#if DEBUG
                DebuggerContainer dc = new DebuggerContainer();
                Face fc = Face.MakeFace(this.surface.Clone(), uvPatch); // added clone to not change the usedArea of the surface, no two faces with the same surface
                dc.Add(fc, 0);
                double len = ((cube.plr | cube.pul) + (cube.pll | cube.pur)) / 4.0;
                dc.Add(Line.TwoPoints(cube.pll, cube.pll + len * cube.nll));
                dc.Add(Line.TwoPoints(cube.plr, cube.plr + len * cube.nlr));
                dc.Add(Line.TwoPoints(cube.pul, cube.pul + len * cube.nul));
                dc.Add(Line.TwoPoints(cube.pur, cube.pur + len * cube.nur));
                dc.Add(Line.TwoPoints(surface.PointAt(uvPatch.GetCenter()), surface.PointAt(uvPatch.GetCenter()) + len * ncnt));
                dc.Add(Line.TwoPoints(cube.pll, cube.pll + len * surface.UDirection(uvPatch.GetLowerLeft())), 0);
                dc.Add(Line.TwoPoints(cube.pll, cube.pll + len * surface.VDirection(uvPatch.GetLowerLeft())), 1);
                dc.Add(Line.TwoPoints(surface.PointAt(uvPatch.GetMiddleLeft()), surface.PointAt(uvPatch.GetMiddleLeft()) + len * udirl));
                dc.Add(Line.TwoPoints(surface.PointAt(uvPatch.GetMiddleRight()), surface.PointAt(uvPatch.GetMiddleRight()) + len * udirr));
                dc.Add(Line.TwoPoints(surface.PointAt(uvPatch.GetLowerMiddle()), surface.PointAt(uvPatch.GetLowerMiddle()) + len * vdirb));
                dc.Add(Line.TwoPoints(surface.PointAt(uvPatch.GetUpperMiddle()), surface.PointAt(uvPatch.GetUpperMiddle()) + len * vdirt));
#endif
                int split = 0; // 1: split in u, 2: split in v, 3: split both
                if (((cube.nll * cube.nlr) < lim || (cube.nul * cube.nur) < lim) || (udirl * udirr) < lim) split = 1;
                else if ((cube.nll * cube.nul) < lim || (cube.nlr * cube.nur) < lim || (vdirb * vdirt) < lim) split = 2;
                else
                {   // only diagonal corners are divergent, no good criterion which parameter to split. So we split both
                    // splitu = ((cube.pll | cube.plr) + (cube.pul | cube.pur) < (cube.pll | cube.pul) + (cube.plr | cube.pur)); // changed > to < (12.3.20)
                    split = 3;
                }
                if (split == 1)
                {
                    double um = (uvPatch.Left + uvPatch.Right) / 2.0;
                    BoundingRect left, right;
                    left = uvPatch;
                    left.Right = um;
                    right = uvPatch;
                    right.Left = um;
                    AddCube(left);
                    AddCube(right);
                }
                else if (split == 2)
                {
                    double vm = (uvPatch.Bottom + uvPatch.Top) / 2.0;
                    BoundingRect bottom, top;
                    bottom = uvPatch;
                    bottom.Top = vm;
                    top = uvPatch;
                    top.Bottom = vm;
                    AddCube(bottom);
                    AddCube(top);
                }
                else if (split == 3)
                {
                    GeoPoint2D cnt = uvPatch.GetCenter();
                    BoundingRect ll, lr, ul, ur;
                    ll = uvPatch;
                    ll.Right = cnt.x;
                    ll.Top = cnt.y;
                    lr = uvPatch;
                    lr.Left = cnt.x;
                    lr.Top = cnt.y;
                    ul = uvPatch;
                    ul.Right = cnt.x;
                    ul.Bottom = cnt.y;
                    ur = uvPatch;
                    ur.Left = cnt.x;
                    ur.Bottom = cnt.y;
                    AddCube(ll);
                    AddCube(lr);
                    AddCube(ul);
                    AddCube(ur);
                }
            }
            else // if (!toosmall) why, we need these
            {
                cube.isFolded = isFolded;
                calcParallelEpiped(cube);
                // cube.boundingCube = surface.GetPatchExtent(uvPatch);
                try
                {
                    octtree.AddObject(cube);
                }
                catch { }
#if DEBUG
                ++dbgcnt;
                //Face dbgfc = Face.MakeFace(surface, cube.uvPatch);
                //GeoObjectList dbglist = (cube as IDebuggerVisualizer).GetList();
                //dbglist.Add(dbgfc);
                //ISurface dbgsrf = (surface as NurbsSurface).GetCanonicalForm(Precision.eps, null);
                //Face dbgfc = Face.MakeFace(dbgsrf, new BoundingRect(0, 0, Math.PI, Math.PI));
#endif
                quadtree.AddObject(cube);
            }
        }

        public void GetPatchHull(BoundingRect uvpatch, out GeoPoint loc, out GeoVector dir1, out GeoVector dir2, out GeoVector dir3)
        {
            // is not working
            // determine the 4 corner points
            GeoPoint pll = surface.PointAt(uvbounds.GetLowerLeft());
            GeoPoint plr = surface.PointAt(uvbounds.GetLowerLeft());
            GeoPoint pul = surface.PointAt(uvbounds.GetUpperLeft());
            GeoPoint pur = surface.PointAt(uvbounds.GetUpperRight());
            // determine the three directions
            GeoVector normal = (pur - pll) ^ (pul - plr); // perpendicular to the two diagonals, length arbitrary for now
            GeoVector diru = (plr - pll) + (pur - pul); // averaged u direction, length arbitrary for now
            GeoVector dirv = (pul - pll) + (pur - plr); // likewise in v
            try
            {
                Matrix m = (Matrix)DenseMatrix.OfColumnArrays(diru, dirv, normal).Inverse();
                if (m.IsValid())
                {
                    // first we use a cube, since the minima/maxima are easier to determine here
                    BoundingBox bc = new BoundingBox(m * pll, m * plr, m * pul, m * pur);
                    // determine minima and maxima in all 6 directions
                    GeoVector[] dirs = new GeoVector[6];
                    dirs[0] = normal;
                    dirs[1] = -normal;
                    dirs[2] = diru;
                    dirs[3] = -diru;
                    dirs[4] = dirv;
                    dirs[5] = -dirv;
                    DirMinimum dm = new DirMinimum(surface);
                    GeoPoint2D uv;
                    GeoPoint mx;
                    for (int i = 0; i < dirs.Length; i++)
                    {
                        // maximum on the surface
                        if (dm.GetNewtonRect(dirs[i], uvpatch, out uv, out mx))
                        {
                            bc.MinMax(m * mx);
                        }
                        // maximum on the boundary curve
                        if (dm.GetNewtonLine(dirs[i], uvpatch.GetLowerLeft(), uvpatch.GetLowerRight(), out uv, out mx))
                        {
                            bc.MinMax(m * mx);
                        }
                        if (dm.GetNewtonLine(dirs[i], uvpatch.GetLowerRight(), uvpatch.GetUpperRight(), out uv, out mx))
                        {
                            bc.MinMax(m * mx);
                        }
                        if (dm.GetNewtonLine(dirs[i], uvpatch.GetUpperLeft(), uvpatch.GetUpperRight(), out uv, out mx))
                        {
                            bc.MinMax(m * mx);
                        }
                        if (dm.GetNewtonLine(dirs[i], uvpatch.GetLowerLeft(), uvpatch.GetUpperLeft(), out uv, out mx))
                        {
                            bc.MinMax(m * mx);
                        }
                    }
                    loc = GeoPoint.Origin + bc.Xmin * diru + bc.Ymin * dirv + bc.Zmin * normal;
                    dir1 = bc.XDiff * diru;
                    dir2 = bc.YDiff * dirv;
                    if (bc.ZDiff == 0.0) bc.Zmax = bc.Zmin + Precision.eps;
                    dir3 = bc.ZDiff * normal; // ZDiff==0.0 is a real problem, i.e. a completely flat piece
                                              // this should be noted separately and respected in various situations

                    //m = Matrix.RowVector(dir1, dir2, dir3).Inverse();
                    //cube.toUnit.SetData(m, m * (-cube.loc));

                    // TEST:
                    //m = Matrix.RowVector(cube.diru, cube.dirv, cube.normal).Inverse();
                    //bc = new BoundingBox(cube.toUnit * cube.pll, cube.toUnit * cube.plr, cube.toUnit * cube.pul, cube.toUnit * cube.pur);
                }
                else
                {   // at least two vectors are linearly dependent, this should not happen
                    System.Diagnostics.Debug.Assert(false, "inavlid Patch in ParallelEpiped");
                }
            }
            catch (System.ApplicationException)
            {   // the matrix is singular, meaning the u and v directions are parallel
                // and that should not happen for any surface. Even if singular, then surely not
                // singular over an entire patch
                System.Diagnostics.Debug.Assert(false, "inavlid Patch in ParallelEpiped");
            }
            // so that the result is assigned:
            loc = GeoPoint.Origin;
            dir1 = GeoVector.NullVector;
            dir2 = GeoVector.NullVector;
            dir3 = GeoVector.NullVector;
        }
        private void calcParallelEpiped(ParEpi cube)
        {
            // the parallelepiped is computed here as follows:
            // determine the 3 directions from the 4 3d corner points of the u/v patch
            // (they are not necessarily optimal, but easy to determine and generally quite good)
            // take the 4 3d corner points into the parallelepiped and then also the extrema
            // of the 4 boundary curves in the respective directions. However, not the real
            // extrema are taken, but the tetraeder points spanned by the directions.
            // finally the intersection points of 3 planes each, which pass tangentially
            // through the corner points, are added.
            // IMPROVEMENT:
            // 1. Give the surface itself a chance to determine a good parallelepiped. The standard surfaces
            // can surely do this very well and quickly.
            // 2. Use DirMinimum to determine the minima in the 3 directions, both of the boundary curves and of the
            // surface as a whole. That is surely much smaller than our roughly estimated parallelepiped in this code here.
            // "GetPatchHull" is meant to take over this task.

            // 7.7.2016: IMPROVEMENT: (not yet implemented)
            // Use findMaxPlaneDist to determine the maximum of the surface in normal direction (cube.normal)
            // if no maximum, then determine the maxima of the 4 boundary curves in normal direction.
            // Determine the maxima of the curves towards the side faces, i.e. UDirection towards cube.udir^cube.normal, v analogously
            // Use these found maxima for bc.MinMax. This makes the cube much smaller!
            cube.normal = (cube.pur - cube.pll) ^ (cube.pul - cube.plr); // perpendicular to the two diagonals, length arbitrary for now
            if (IsVSingularity(cube.uvPatch.Bottom) || IsVSingularity(cube.uvPatch.Top) || IsUSingularity(cube.uvPatch.Left) || IsUSingularity(cube.uvPatch.Right))
            {
                cube.normal = surface.GetNormal(cube.uvPatch.GetCenter());
            }
            cube.diru = (cube.plr - cube.pll) + (cube.pur - cube.pul); // averaged u direction, length arbitrary for now
            cube.dirv = (cube.pul - cube.pll) + (cube.pur - cube.plr); // likewise in v
                                                                       // the 3 vectors are now the correct ones, the lengths are not yet right and the location is also missing
                                                                       // bring various points into this system in order to then normalize it
            if (cube.isFolded)
            {   // here only roughly
                cube.loc = new GeoPoint(cube.pll, cube.plr, cube.pul, cube.pur);
                cube.normal = new GeoVector(cube.nll, cube.nlr, cube.nul, cube.nur);
                cube.diru = (cube.plr - cube.pll) + (cube.pur - cube.pul);
                cube.dirv = (cube.pul - cube.pll) + (cube.pur - cube.plr);
                Matrix m = DenseMatrix.OfColumnArrays(cube.diru, cube.dirv, cube.normal);

                if (m.Rank() < 3)
                {
                }
                else
                {
                    m = (Matrix)m.Inverse();
                    GeoPoint mpll = m * cube.pll;
                    GeoPoint mplr = m * cube.plr;
                    GeoPoint mpul = m * cube.pul;
                    GeoPoint mpur = m * cube.pur;
                    GeoPoint loc = new GeoPoint(Math.Min(Math.Min(mpll.x, mplr.x), Math.Min(mpul.x, mpur.x)), Math.Min(Math.Min(mpll.y, mplr.y), Math.Min(mpul.y, mpur.y)), Math.Min(Math.Min(mpll.z, mplr.z), Math.Min(mpul.z, mpur.z)));

                    cube.toUnit.SetData(m, -loc);
                    mpll = cube.toUnit * cube.pll;
                    mplr = cube.toUnit * cube.plr;
                    mpul = cube.toUnit * cube.pul;
                    mpur = cube.toUnit * cube.pur;
                    cube.loc = cube.toUnit.GetInverse() * GeoPoint.Origin;
                }

            }
            else
            {
                try
                {
                    Matrix m = (Matrix)DenseMatrix.OfColumnArrays(cube.diru, cube.dirv, cube.normal).Inverse();
                    BoundingBox bc = new BoundingBox(m * cube.pll, m * cube.plr, m * cube.pul, m * cube.pur);
                    // for the new method (7.7.2016)
                    //                GeoPoint2D found;
                    //                bool innerMaximum = false;
                    //#if DEBUG
                    //                GeoPoint dbg1 = cube.pll;
                    //                GeoPoint dbg2 = cube.pur;
                    //#endif
                    //                if (findMaxPlaneDist(cube.uvPatch.GetCenter(), cube.diru ^ cube.dirv, out found))
                    //                {
                    //                    if (cube.uvPatch.Contains(found))
                    //                    {
                    //                        GeoPoint pp = surface.PointAt(found);
                    //                        bc.MinMax(m * pp);
                    //                        innerMaximum = true;
                    //#if DEBUG
                    //                        dbg1 = pp;
                    //#endif
                    //                    }
                    //                }
                    //                if (findMaxPlaneDist(cube.uvPatch.GetCenter(), cube.normal, out found))
                    //                {
                    //                    if (cube.uvPatch.Contains(found))
                    //                    {
                    //                        GeoPoint pp = surface.PointAt(found);
                    //                        bc.MinMax(m * pp);
                    //                        innerMaximum = true;
                    //#if DEBUG
                    //                        dbg2 = pp;
                    //#endif
                    //                    }
                    //                }
                    // include the tetraeder points of the curve segments. Here it is not guaranteed that they are
                    // well-behaved, i.e. have no extreme inflection points. Possibly make a method that returns a list
                    // of such points, then also only one in the flat case
                    double dbgsz = bc.Size;
                    GeoPoint tv1, tv2;
                    if (!IsVSingularity(cube.uvPatch.Bottom))
                    {
                        TetraederHull.GetTetraederPoints(cube.pll, cube.plr, surface.UDirection(cube.uvPatch.GetLowerLeft()), surface.UDirection(cube.uvPatch.GetLowerRight()), out tv1, out tv2);
                        bc.MinMax(m * tv1);
                        bc.MinMax(m * tv2);
                    }
                    if (!IsVSingularity(cube.uvPatch.Top))
                    {
                        TetraederHull.GetTetraederPoints(cube.pul, cube.pur, surface.UDirection(cube.uvPatch.GetUpperLeft()), surface.UDirection(cube.uvPatch.GetUpperRight()), out tv1, out tv2);
                        bc.MinMax(m * tv1);
                        bc.MinMax(m * tv2);
                    }
                    if (!IsUSingularity(cube.uvPatch.Left))
                    {
                        TetraederHull.GetTetraederPoints(cube.pll, cube.pul, surface.VDirection(cube.uvPatch.GetLowerLeft()), surface.VDirection(cube.uvPatch.GetUpperLeft()), out tv1, out tv2);
                        bc.MinMax(m * tv1);
                        bc.MinMax(m * tv2);
                    }
                    if (!IsUSingularity(cube.uvPatch.Right))
                    {
                        TetraederHull.GetTetraederPoints(cube.plr, cube.pur, surface.VDirection(cube.uvPatch.GetLowerRight()), surface.VDirection(cube.uvPatch.GetUpperRight()), out tv1, out tv2);
                        bc.MinMax(m * tv1);
                        bc.MinMax(m * tv2);
                    }
                    // intersect the 4 tangent planes at the corner points and add these points
                    GeoPoint ip;
                    if (Plane.Intersect3Planes(cube.pll, cube.nll, cube.plr, cube.nlr, cube.pul, cube.nul, out ip))
                    {
                        GeoPoint mip = m * ip;
                        if (mip.x >= bc.Xmin && mip.x <= bc.Xmax && mip.y >= bc.Ymin && mip.y <= bc.Ymax)
                        {   // may only affect z; intersection points outside do not count (saddle surfaces or such)
                            bc.Zmin = Math.Min(bc.Zmin, mip.z);
                            bc.Zmax = Math.Max(bc.Zmax, mip.z);
                        }
                    }
                    if (Plane.Intersect3Planes(cube.pll, cube.nll, cube.plr, cube.nlr, cube.pur, cube.nur, out ip))
                    {
                        GeoPoint mip = m * ip;
                        if (mip.x >= bc.Xmin && mip.x <= bc.Xmax && mip.y >= bc.Ymin && mip.y <= bc.Ymax)
                        {   // may only affect z; intersection points outside do not count (saddle surfaces or such)
                            bc.Zmin = Math.Min(bc.Zmin, mip.z);
                            bc.Zmax = Math.Max(bc.Zmax, mip.z);
                        }
                    }
                    if (Plane.Intersect3Planes(cube.pll, cube.nll, cube.pul, cube.nul, cube.pur, cube.nur, out ip))
                    {
                        GeoPoint mip = m * ip;
                        if (mip.x >= bc.Xmin && mip.x <= bc.Xmax && mip.y >= bc.Ymin && mip.y <= bc.Ymax)
                        {   // may only affect z; intersection points outside do not count (saddle surfaces or such)
                            bc.Zmin = Math.Min(bc.Zmin, mip.z);
                            bc.Zmax = Math.Max(bc.Zmax, mip.z);
                        }
                    }
                    if (Plane.Intersect3Planes(cube.plr, cube.nlr, cube.pul, cube.nul, cube.pur, cube.nur, out ip))
                    {
                        GeoPoint mip = m * ip;
                        if (mip.x >= bc.Xmin && mip.x <= bc.Xmax && mip.y >= bc.Ymin && mip.y <= bc.Ymax)
                        {   // may only affect z; intersection points outside do not count (saddle surfaces or such)
                            bc.Zmin = Math.Min(bc.Zmin, mip.z);
                            bc.Zmax = Math.Max(bc.Zmax, mip.z);
                        }
                    }
                    // set the location and lengths so that a unit cube results
                    cube.loc = GeoPoint.Origin + bc.Xmin * cube.diru + bc.Ymin * cube.dirv + bc.Zmin * cube.normal;
                    cube.diru = bc.XDiff * cube.diru;
                    cube.dirv = bc.YDiff * cube.dirv;
                    if (bc.ZDiff == 0.0) bc.Zmax = bc.Zmin + Precision.eps;
                    double zdiff = bc.ZDiff;
                    if (zdiff == 0.0) zdiff = Precision.eps;
                    GeoVector normal = zdiff * cube.normal; // ZDiff==0.0 is a real problem, i.e. a completely flat piece
                    if (normal.IsNullVector()) normal = cube.normal;
                    cube.normal = normal; // ZDiff==0.0 is a real problem, i.e. a completely flat piece
                                          // this should be noted separately and respected in various situations
                    cube.isFlat = bc.ZDiff < Precision.eps;
                    if (Math.Abs(cube.Volume) < Precision.eps)
                    {
                    }

                    m = (Matrix)DenseMatrix.OfColumnArrays(cube.diru, cube.dirv, cube.normal).Inverse();
                    cube.toUnit.SetData(m, m * (-cube.loc));

#if DEBUGx
                    BoundingBox dbgext = new BoundingBox(cube.pll, cube.plr, cube.pul, cube.pur);
                    // System.Diagnostics.Trace.WriteLine("ParallelEpiped " + cube.id.ToString() + ": " + cube.Volume.ToString());
                    //if (dbgext.Size < cube.BoundingBox.Size * 0.01)
                    if (cube.Volume > 1000)
                    {
                        GeoObjectList dbgl = new GeoObjectList();
                        dbgl.Add(cube.AsBox);
                        for (int i = 0; i < 10; i++)
                        {
                            GeoPoint[] pnts = new GeoPoint[10];
                            for (int j = 0; j < 10; j++)
                            {
                                GeoPoint2D uv = new GeoPoint2D(cube.uvPatch.Left + i * cube.uvPatch.Width / 9, cube.uvPatch.Bottom + j * cube.uvPatch.Height / 9);
                                pnts[j] = surface.PointAt(uv);
                            }
                            Polyline pl = Polyline.FromPoints(pnts);
                            dbgl.Add(pl);
                        }
                    }
#endif
                    //#if DEBUG
                    //                DebuggerContainer dc = new DebuggerContainer();
                    //                Face fc = Face.MakeFace(surface, new SimpleShape(cube.uvPatch.ToBorder()));
                    //                dc.Add(fc);
                    //                dc.Add(cube.AsBox);
                    //                Line l = Line.Construct();
                    //                l.SetTwoPoints(dbg1, dbg2);
                    //                dc.Add(l);
                    //#endif

                    // TEST:
                    //m = Matrix.RowVector(cube.diru, cube.dirv, cube.normal).Inverse();
                    bc = new BoundingBox(cube.toUnit * cube.pll, cube.toUnit * cube.plr, cube.toUnit * cube.pul, cube.toUnit * cube.pur);
                }
                catch (System.ApplicationException)
                {   // the matrix is singular, meaning the u and v directions are parallel
                    // and that should not happen for any surface. Even if singular, then surely not
                    // singular over an entire patch
                    cube.loc = new GeoPoint(cube.pll, cube.plr, cube.pul, cube.pur);
                    cube.normal = new GeoVector(cube.nll, cube.nlr, cube.nul, cube.nur);
                    cube.diru = (cube.plr - cube.pll) + (cube.pur - cube.pul);
                    cube.dirv = (cube.pul - cube.pll) + (cube.pur - cube.plr);
                    Matrix m = DenseMatrix.OfColumnArrays(cube.diru, cube.dirv, cube.normal);
                    if (m.Rank() < 3)
                    {
                    }
                    else
                    {
                        m = (Matrix)m.Inverse();
                        cube.toUnit.SetData(m, m * (-cube.loc));
                    }
                    // System.Diagnostics.Debug.Assert(false, "inavlid Patch in ParallelEpiped");
                }
            }
        }
        public BoundingBox GetRawExtent()
        {
            return octtree.Extend;
        }
        private bool IsVSingularity(double v)
        {
            for (int i = 0; i < vSingularities.Length; i++)
            {
                if (Math.Abs(v - vSingularities[i]) < 1e-8) return true;
            }
            return false;
        }
        private bool IsUSingularity(double u)
        {
            for (int i = 0; i < uSingularities.Length; i++)
            {
                if (Math.Abs(u - uSingularities[i]) < 1e-8) return true;
            }
            return false;
        }
        /// <summary>
        /// Determins, whether the surface is hit by the provided bounding cube, and if so returns an arbitrary point of the surface.
        /// Problem: we would need several points, if there are more disjunct surface segments which interfere with the cube
        /// </summary>
        /// <param name="test"></param>
        /// <param name="uv"></param>
        /// <returns></returns>
        public bool HitTest(BoundingBox test, out GeoPoint2D uv)
        {
            ParEpi[] hits = octtree.GetObjectsFromBox(test);
            List<ParEpi> totest = new List<ParEpi>();
            List<ParEpi> untested = new List<ParEpi>();
            for (int i = 0; i < hits.Length; ++i)
            {
                if (test.Interferes(hits[i].loc, hits[i].diru, hits[i].dirv, hits[i].normal))
                {
                    // test the the four known points of the patch
                    if (test.Contains(hits[i].pll))
                    {
                        uv = hits[i].uvPatch.GetLowerLeft();
                        return true;
                    }
                    if (test.Contains(hits[i].pul))
                    {
                        uv = hits[i].uvPatch.GetUpperLeft();
                        return true;
                    }
                    if (test.Contains(hits[i].plr))
                    {
                        uv = hits[i].uvPatch.GetLowerRight();
                        return true;
                    }
                    if (test.Contains(hits[i].pur))
                    {
                        uv = hits[i].uvPatch.GetUpperRight();
                        return true;
                    }
                    uv = hits[i].uvPatch.GetCenter();
                    if (test.Contains(surface.PointAt(uv))) return true; // test the center of the patch
                    totest.Add(hits[i]);
                }
            }
            if (totest.Count == 0)
            {
                uv = GeoPoint2D.Invalid;
                return false;
            }

            for (int i = 0; i < totest.Count; ++i)
            {
                // tests, whether an edge of the testcube intersects with the patch
                GeoPoint[,] lines = test.Lines;
                Line line = Line.Construct();
                for (int k = 0; k < 12; k++)
                {
                    if (totest[i].Interferes(lines[k, 0], lines[k, 1]))
                    {
                        line.SetTwoPoints(lines[k, 0], lines[k, 1]);
                        GeoPoint[] ips;
                        GeoPoint2D[] uvOnFaces;
                        double[] uOnCurve3Ds;
                        surface.Intersect(line, totest[i].uvPatch, out ips, out uvOnFaces, out uOnCurve3Ds);
                        for (int j = 0; j < ips.Length; j++)
                        {
                            if (totest[i].uvPatch.Contains(uvOnFaces[j]) && uOnCurve3Ds[j] >= 0.0 && uOnCurve3Ds[j] <= 1.0)
                            {
                                uv = uvOnFaces[j];
                                return true;
                            }
                        }
                    }
                }
                if (extrema == null) calcExtrema();
                // does the surface have local extrema? i.e. bulges, so that the surface can intersect a plane
                // without the boundary curves of a uv patch intersecting the plane?
                // if there are no such extrema, then it suffices to intersect the boundary curves with the cube
                // and the cube edges with the surface. If neither yields intersections, then
                // there is no hit
                if (extrema.Length > 0)
                {   // there are extrema (in axis direction). Is this patch affected by them?
                    // only then do we need to split
                    bool buldgeInPatch = false;
                    for (int j = 0; j < extrema.Length; j++)
                    {
                        if (totest[i].uvPatch.Contains(extrema[j]))
                        {
                            buldgeInPatch = true;
                            break;
                        }
                    }
                    if (buldgeInPatch)
                    {
                        if (SplitHit(totest[i], test, out uv, untested)) return true;
                    }
                }

            }
            // untested contains a list of not yet tested parallelepipeds, which are reduced here now
            while (untested.Count > 0)
            {
                List<ParEpi> stillToTest = new List<ParEpi>();
                foreach (ParEpi cb in untested)
                {
                    if (SplitHit(cb, test, out uv, stillToTest)) return true;
                }
                untested = stillToTest;
            }
#if DEBUG
            if (extrema != null)
            {
                GeoObjectList dbgex = new GeoObject.GeoObjectList();
                for (int i = 0; i < extrema.Length; i++)
                {
                    Point pp = Point.Construct();
                    pp.Location = surface.PointAt(extrema[i]);
                    pp.Symbol = PointSymbol.Cross;
                    dbgex.Add(pp);
                }
            }
#endif
            uv = GeoPoint2D.Origin;
            return false;
        }

        private void calcExtrema()
        {
            if (extrema != null) return;
            ParEpi[] allcubes = octtree.GetAllObjects();
            List<GeoPoint2D> extr = new List<GeoPoint2D>();
            foreach (GeoVector dir in GeoVector.MainAxis)
            {
                foreach (ParEpi cube in allcubes)
                {
                    // does dir even lie in the space "spanned" by the patch?
                    // 3 normal vectors give the condition a*n1+b*n2+c*n3==dir, if a, b and c have the same sign
                    bool dotest = false;
                    Matrix m = DenseMatrix.OfColumnArrays(cube.nll, cube.nlr, cube.nul);
                    Vector x = (Vector)m.Solve(new DenseVector(dir));
                    if (x.IsValid() && Math.Sign(x[0]) == Math.Sign(x[1]) && Math.Sign(x[0]) == Math.Sign(x[2]))
                    {
#if DEBUG
                        //DebuggerContainer dc = new CADability.DebuggerContainer();
                        //Face fce = Face.MakeFace(surface, new SimpleShape(Border.MakeRectangle(cube.uvPatch)));
                        //dc.Add(fce);
                        //dc.Add(cube.AsBox);
                        //dc.Add(Line.MakeLine(GeoPoint.Origin, GeoPoint.Origin + cube.nll));
                        //dc.Add(Line.MakeLine(GeoPoint.Origin, GeoPoint.Origin + cube.nlr));
                        //dc.Add(Line.MakeLine(GeoPoint.Origin, GeoPoint.Origin + cube.nul));
#endif
                        dotest = true;
                    }
                    if (!dotest)
                    {
                        m = DenseMatrix.OfColumnArrays(cube.nur, cube.nul, cube.nlr);
                        x = (Vector)m.Solve(new DenseVector(dir));
                        if (x.IsValid() && Math.Sign(x[0]) == Math.Sign(x[1]) && Math.Sign(x[0]) == Math.Sign(x[2]))
                        {
                            dotest = true;
                        }
                    }
                    if (dotest)
                    {
                        GeoPoint2D mp;
                        if (findMaxPlaneDist(cube.uvPatch.GetCenter(), cube.uvPatch.Size, dir, out mp))
                        {
                            if (cube.uvPatch.Contains(mp)) extr.Add(mp);
                        }
                    }
                }
            }
            extrema = extr.ToArray();
        }

        private bool SplitHit(ParEpi toSplit, BoundingBox test, out GeoPoint2D uv, List<ParEpi> unknown)
        {
            // split toSplit until either a hit with test is found
            // or there is no overlap anymore
            // the split parts are only examined at their corner points, and whether the parallelepiped intersects the cube.
            // No intersection: return false, a point inside: return true
            // otherwise it is added to the list of unknowns
            octtree.RemoveObject(toSplit);
            quadtree.RemoveObject(toSplit);
            ParEpi[] subCubes = new ParEpi[4];
            subCubes[0] = new ParEpi();
            subCubes[1] = new ParEpi();
            subCubes[2] = new ParEpi();
            subCubes[3] = new ParEpi();
            double hcenter = (toSplit.uvPatch.Left + toSplit.uvPatch.Right) / 2.0;
            double vcenter = (toSplit.uvPatch.Bottom + toSplit.uvPatch.Top) / 2.0;
            subCubes[0].uvPatch = new BoundingRect(toSplit.uvPatch.Left, toSplit.uvPatch.Bottom, hcenter, vcenter);
            subCubes[1].uvPatch = new BoundingRect(toSplit.uvPatch.Left, vcenter, hcenter, toSplit.uvPatch.Top);
            subCubes[2].uvPatch = new BoundingRect(hcenter, toSplit.uvPatch.Bottom, toSplit.uvPatch.Right, vcenter);
            subCubes[3].uvPatch = new BoundingRect(hcenter, vcenter, toSplit.uvPatch.Right, toSplit.uvPatch.Top);
            for (int i = 0; i < 4; ++i)
            {
                subCubes[i].pll = surface.PointAt(subCubes[i].uvPatch.GetLowerLeft());
                subCubes[i].plr = surface.PointAt(subCubes[i].uvPatch.GetLowerRight());
                subCubes[i].pul = surface.PointAt(subCubes[i].uvPatch.GetUpperLeft());
                subCubes[i].pur = surface.PointAt(subCubes[i].uvPatch.GetUpperRight());
                subCubes[i].nll = surface.GetNormal(subCubes[i].uvPatch.GetLowerLeft());
                subCubes[i].nlr = surface.GetNormal(subCubes[i].uvPatch.GetLowerRight());
                subCubes[i].nul = surface.GetNormal(subCubes[i].uvPatch.GetUpperLeft());
                subCubes[i].nur = surface.GetNormal(subCubes[i].uvPatch.GetUpperRight());
                subCubes[i].nll.NormIfNotNull();
                subCubes[i].nlr.NormIfNotNull();
                subCubes[i].nul.NormIfNotNull();
                subCubes[i].nur.NormIfNotNull();
                calcParallelEpiped(subCubes[i]);
                octtree.AddObject(subCubes[i]);
                quadtree.AddObject(subCubes[i]);
            }
            for (int i = 0; i < 4; ++i)
            {
                if (test.Interferes(subCubes[i].loc, subCubes[i].diru, subCubes[i].dirv, subCubes[i].normal) && subCubes[i].Size > Precision.eps * 100)
                {
                    //uv = subCubes[i].uvPatch.GetCenter();
                    //if (test.Contains(surface.PointAt(uv))) return true;
                    // the center is only recomputed when splitting, here only the already known points
                    if (test.Contains(subCubes[i].pll))
                    {
                        uv = subCubes[i].uvPatch.GetLowerLeft();
                        return true;
                    }
                    if (test.Contains(subCubes[i].pul))
                    {
                        uv = subCubes[i].uvPatch.GetUpperLeft();
                        return true;
                    }
                    if (test.Contains(subCubes[i].plr))
                    {
                        uv = subCubes[i].uvPatch.GetLowerRight();
                        return true;
                    }
                    if (test.Contains(subCubes[i].pur))
                    {
                        uv = subCubes[i].uvPatch.GetUpperRight();
                        return true;
                    }
                    unknown.Add(subCubes[i]);
                }
            }
            uv = GeoPoint2D.Origin;
            return false;
        }
        internal bool IsClose(GeoPoint p3d)
        {
            ParEpi[] cubes = octtree.GetObjectsFromPoint(p3d);
            for (int i = 0; i < cubes.Length; ++i)
            {
                if (cubes[i].Contains(p3d)) return true;
            }
            return false;
        }
        internal void RawPointNormalAt(GeoPoint2D uv, out GeoPoint location, out GeoVector normal)
        {
            foreach (ParEpi pe in quadtree.ObjectsFromRect(new BoundingRect(uv)))
            {
                if (pe.uvPatch.Contains(uv))
                {
                    double u = (uv.x - pe.uvPatch.Left) / (pe.uvPatch.Right - pe.uvPatch.Left);
                    double v = (uv.y - pe.uvPatch.Bottom) / (pe.uvPatch.Top - pe.uvPatch.Bottom);
                    // u and v are between 0 and 1
                    location = new GeoPoint(new GeoPoint(pe.pll, pe.plr, u), new GeoPoint(pe.pul, pe.pur, u), v);
                    normal = new GeoVector(new GeoVector(pe.nll, pe.nlr, u), new GeoVector(pe.nul, pe.nur, u), v);
#if DEBUG
                    //GeoPoint ll = surface.PointAt(uv);
                    //GeoVector nn = surface.GetNormal(uv);
#endif
                    return;
                }
            }
            location = surface.PointAt(uv);
            normal = surface.GetNormal(uv);
        }
        internal GeoPoint RawPointAt(GeoPoint2D uv)
        {
            foreach (ParEpi pe in quadtree.ObjectsFromRect(new BoundingRect(uv)))
            {
                if (pe.uvPatch.Contains(uv))
                {
                    double u = (uv.x - pe.uvPatch.Left) / (pe.uvPatch.Right - pe.uvPatch.Left);
                    double v = (uv.y - pe.uvPatch.Bottom) / (pe.uvPatch.Top - pe.uvPatch.Bottom);
                    // u and v are between 0 and 1
                    return new GeoPoint(new GeoPoint(pe.pll, pe.plr, u), new GeoPoint(pe.pul, pe.pur, u), v);
                }
            }
            return surface.PointAt(uv);
        }
        internal HashSet<ParEpi> GetCommonParEpis(ParallelepipedHull other)
        {
            HashSet<ParEpi> res = new HashSet<ParEpi>();
            foreach (ParEpi pe in octtree.GetAllObjects())
            {
                ParEpi[] others = other.octtree.GetObjectsCloseTo(pe);
                bool found = false;
                foreach (ParEpi op in others)
                {
                    if (pe.Interferes(op))
                    {
                        res.Add(op);
                        found = true;
                    }
                }
                if (found) res.Add(pe);
            }
            return res;
        }
        public bool PositionOf(GeoPoint p3d, out GeoPoint2D res)
        {
            ParEpi[] cubes = octtree.GetObjectsFromPoint(p3d);
            List<ParEpi> containingPoint = new List<ParEpi>();
            for (int i = 0; i < cubes.Length; i++)
            {
                if (cubes[i].Contains(p3d)) containingPoint.Add(cubes[i]);
            }
            if (containingPoint.Count > 0) cubes = containingPoint.ToArray();
            if (cubes.Length == 0)
            {
                double size = octtree.Extend.Size / 10000;
                while (cubes.Length == 0)
                {
                    cubes = octtree.GetObjectsFromBox(new BoundingBox(p3d, size));
                    size *= 2.0;
                }
            }
            // so here we know some cubes that were found by the octtree. We look for the best one
            res = GeoPoint2D.Origin;
            double minDist = double.MaxValue;
            for (int i = 0; i < cubes.Length; i++)
            {
                GeoPoint2D tmp;
                if (PositionOf(p3d, cubes[i], out tmp, out double d))
                {
                    if (d < minDist)
                    {
                        minDist = d;
                        res = tmp;
                    }
                }
            }
            if (minDist < double.MaxValue) return true;

            if (uSingularities.Length > 0)
            {
                double v = (uvbounds.Bottom + uvbounds.Top) / 2.0;
                for (int i = 0; i < uSingularities.Length; i++)
                {
                    GeoPoint pole = surface.PointAt(new GeoPoint2D(uSingularities[i], v));
                    double d = pole | p3d;
                    if (d < minDist)
                    {
                        res = new GeoPoint2D(uSingularities[i], v);
                        minDist = d;
                    }
                }
            }
            if (vSingularities.Length > 0)
            {
                double u = (uvbounds.Left + uvbounds.Right) / 2.0;
                for (int i = 0; i < vSingularities.Length; i++)
                {
                    GeoPoint pole = surface.PointAt(new GeoPoint2D(u, vSingularities[i]));
                    double d = pole | p3d;
                    if (d < minDist)
                    {
                        res = new GeoPoint2D(u, vSingularities[i]);
                        minDist = d;
                    }
                }
            }
            if (minDist < Precision.eps) return true;
            //BoundingRect ext = BoundingRect.EmptyBoundingRect;
            //for (int i = 0; i < cubes.Length; i++)
            //{
            //    ext.MinMax(cubes[i].uvPatch);
            //}

            //ICurve crv = surface.FixedU((ext.Left + ext.Right) / 2, ext.Bottom, ext.Top);
            //double u = crv.PositionOf(p3d);
            //GeoPoint pu = crv.PointAt(u);
            //crv = surface.FixedV((ext.Bottom + ext.Top) / 2, ext.Left, ext.Right);
            //double v = crv.PositionOf(p3d);
            //GeoPoint pv = crv.PointAt(v);

            return false; // the point is outside the surface
        }
        public GeoPoint2D[] GetLineIntersection(GeoPoint startPoint, GeoVector direction)
        {
#if DEBUG
            //DebuggerContainer dc = Debug;
#endif
            List<GeoPoint2D> res = new List<GeoPoint2D>();
            ParEpi[] cubes = octtree.GetObjectsFromLine(startPoint, direction, 0.0);
#if DEBUG
            DebuggerContainer dc = new DebuggerContainer();
            for (int i = 0; i < cubes.Length; i++)
            {
                dc.Add(cubes[i].AsBox);
            }
#endif
            for (int i = 0; i < cubes.Length; i++)
            {
                if (cubes[i].Interferes(startPoint, direction, 0.0, false))
                {   // the octtree also returns cubes that do not touch the line
                    List<GeoPoint2D> ips = GetLineIntersection(startPoint, direction, cubes[i]);
                    bool add = true;
                    for (int j = 0; j < ips.Count; j++)
                    {   // we want the points only once. usually there is just one
                        for (int k = 0; k < res.Count; k++)
                        {
                            double d = ips[j] | res[k];
                            if (d < cubes[i].uvPatch.Size * 1e-6) add = false;
                        }
                        if (add) res.Add(ips[j]);
                    }
                }
            }
            return res.ToArray();
        }
        private List<GeoPoint2D> GetLineIntersection(GeoPoint startPoint, GeoVector direction, ParEpi cube)
        {
            // to improve performance NewtonLineIntersection could also return several intersection points that lie outside
            // the patch, and the corresponding patches would then no longer be used. But that is difficult.
            List<GeoPoint2D> res = new List<GeoPoint2D>();
            GeoPoint2D uvSurface;
            if (NewtonLineIntersection(cube.uvPatch, startPoint, direction, out uvSurface))
            {
                res.Add(uvSurface);
                // it may be that there is a second intersection point in this patch (more than two should
                // not occur, otherwise the subdivision of the ParallelepipedHull is bad.
                // the criterion whether there could be another intersection point is difficult. Here we check whether
                // the normal vectors at the corners and at the intersection point have the same direction
                // as the line. If not, there could still be an intersection. But this is not necessary.
                // probably better but more expensive would be to check whether there is a maximum or minimum in the
                // direction of the line.
                int sgn = Math.Sign(surface.GetNormal(cube.uvPatch.GetLowerLeft()) * direction);
                bool split = false;
                split = split || Math.Sign(surface.GetNormal(cube.uvPatch.GetLowerRight()) * direction) != sgn;
                split = split || Math.Sign(surface.GetNormal(cube.uvPatch.GetUpperLeft()) * direction) != sgn;
                split = split || Math.Sign(surface.GetNormal(cube.uvPatch.GetUpperRight()) * direction) != sgn;
                split = split || Math.Sign(surface.GetNormal(uvSurface) * direction) != sgn;
                //double dbg1 = surface.GetNormal(cube.uvPatch.GetLowerLeft()) * direction;
                //double dbg2 = surface.GetNormal(cube.uvPatch.GetLowerRight()) * direction;
                //double dbg3 = surface.GetNormal(cube.uvPatch.GetUpperLeft()) * direction;
                //double dbg4 = surface.GetNormal(cube.uvPatch.GetUpperRight()) * direction;
                //double dbg5 = surface.GetNormal(uvSurface) * direction;
                if (split)
                {
#if DEBUG
                    //{
                    //    DebuggerContainer dc = new DebuggerContainer();
                    //    Face fc = Face.MakeFace(surface, new CADability.Shapes.SimpleShape(cube.uvPatch.ToBorder()));
                    //    dc.Add(fc);
                    //    Line l3d = Line.MakeLine(startPoint, startPoint + direction);
                    //    dc.Add(l3d);
                    //    dc.Add(cube.GetSolid());
                    //}
#endif
                    // here we consider the uvPatch and split it into 4 pieces such that the found intersection point
                    // is not part of them. But we do not recurse, we only do this once
                    double eps = cube.uvPatch.Size / 1000; // size of the square hole in the patch
                    BoundingRect[] subpatches = new BoundingRect[]{
                        new BoundingRect(uvSurface.x + eps, uvSurface.y - eps, cube.uvPatch.Right, cube.uvPatch.Top),
                        new BoundingRect(cube.uvPatch.Left, uvSurface.y + eps, uvSurface.x + eps, cube.uvPatch.Top),
                        new BoundingRect(cube.uvPatch.Left, cube.uvPatch.Bottom, uvSurface.x - eps, uvSurface.y + eps),
                        new BoundingRect(uvSurface.x - eps, cube.uvPatch.Bottom, cube.uvPatch.Right, uvSurface.y - eps)};
                    for (int i = 0; i < subpatches.Length; ++i)
                    {
                        if (subpatches[i].Left < subpatches[i].Right && subpatches[i].Bottom < subpatches[i].Top)
                        {   // can occur for intersections at the border
                            if (NewtonLineIntersection(subpatches[i], startPoint, direction, out uvSurface))
                            {
                                res.Add(uvSurface);
                            }
                        }
                    }
                }
            }
            return res;
        }
        /// <summary>
        /// Finds the location of maximum or minimum distance of the curve (given by the segment [sp, ep] in uv) to the given plane (norm)
        /// The plane usually passes through the two points at sp and ep, then there is always a maximum in between.
        /// </summary>
        /// <param name="sp"></param>
        /// <param name="ep"></param>
        /// <param name="result"></param>
        /// <returns></returns>
        internal bool findMaxPlaneDist(GeoPoint2D sp, GeoPoint2D ep, GeoVector norm, out GeoPoint2D result)
        {
            // find the u/v value between sp and ep where the tangent plane is parallel to the secant
            // The function f(u, v) = (u-uc)²*d2u + (v-vc)²*d2v + (u-uc) * (v-vc) * duv + (u-uc) * du + (v-vc) * dv + p0 approximates the surface at the point (uc,vc)
            // with u = u0 + a*t, v = v0 + b*t
            // g(t) = (u0 + a*t -uc)²*d2u + (v0 + b*t-vc)²*d2v + (u0 + a*t-uc)*(v0 + b*t-vc)*duv + (u0 + a*t-uc)*du + (v0 + b*t-vc)*dv + p0 is the approximated curve from sp to ep
            // g'(t) = a*duv*(b*t+v0-vc)+2*b*d2v*(b*t+v0-vc)+b*duv*(a*t+u0-uc)+2*a*d2u*(a*t+u0-uc)+b*dv+a*du
            // g'(t)*n = 
            // (a*duvx*(b*t+v0-vc)+2*b*d2vx*(b*t+v0-vc)+b*duvx*(a*t+u0-uc)+2*a*d2ux*(a*t+u0-uc)+b*dvx+a*dux)*nx + 
            // (a*duvy*(b*t+v0-vc)+2*b*d2vy*(b*t+v0-vc)+b*duvy*(a*t+u0-uc)+2*a*d2uy*(a*t+u0-uc)+b*dvy+a*duy)*ny + 
            // (a*duvz*(b*t+v0-vc)+2*b*d2vz*(b*t+v0-vc)+b*duvz*(a*t+u0-uc)+2*a*d2uz*(a*t+u0-uc)+b*dvz+a*duz)*nz
            // must become zero, i.e. the direction of the curve must be perpendicular to the normal

            // usually converges very quickly (in 2 steps, unless pathological)
            GeoPoint location;
            GeoVector du;
            GeoVector dv;
            GeoVector duu;
            GeoVector dvv;
            GeoVector duv;
            double a = ep.x - sp.x;
            double b = ep.y - sp.y;
            GeoVector2D ab = new GeoVector2D(a, b);
            double t = 0.5;
            result = sp + t * ab;
            try // e.g. when the derivatives become 0
            {
                norm.Norm();

                surface.Derivative2At(result, out location, out du, out dv, out duu, out dvv, out duv);
                GeoVector d2u = 0.5 * duu;
                GeoVector d2v = 0.5 * dvv;
#if DEBUG
                DebuggerContainer dc = new DebuggerContainer();
                GeoPoint lastPoint = location;
#endif
                double u0 = sp.x;
                double v0 = sp.y;
                double uc = result.x;
                double vc = result.y;
                double a2 = (2.0 * a);
                double b2 = (2.0 * b);
                double aa = a * a;
                double aa2 = (2.0 * aa);
                double bb = b * b;
                double bb2 = (2.0 * bb);
                double ab2 = (a2 * b);
                double bduvz = (b * duv.z);
                double bduvy = (b * duv.y);
                double bduvx = (b * duv.x);
                double aduvz = (a * duv.z);
                double aduvy = (a * duv.y);
                double aduvx = (a * duv.x);

                GeoVector curvedir = a * du + b * dv;

                double err = Math.Abs(curvedir.Normalized * norm); // this value should become 0
                bool atend = false; // if t runs out of range, atend is set. If it then runs out again, the end is the maximum
                for (int k = 0; k < 10; k++)
                {
                    t = (((-b) * dv.x - a * du.x) * norm.x + ((-b) * dv.y - a * du.y) * norm.y + ((-b) * dv.z - a * du.z) * norm.z + ((((-2.0) * a) * d2u.z - bduvz) * norm.z + (((-2.0) * a) * d2u.y - bduvy) * norm.y + (((-2.0) * a) * d2u.x - bduvx) * norm.x) * u0 + ((a2 * d2u.z + bduvz) * norm.z + (a2 * d2u.y + bduvy) * norm.y + (a2 * d2u.x + bduvx) * norm.x) * uc + ((((-2.0) * b) * d2v.z - aduvz) * norm.z + (((-2.0) * b) * d2v.y - aduvy) * norm.y + (((-2.0) * b) * d2v.x - aduvx) * norm.x) * v0 + ((b2 * d2v.z + aduvz) * norm.z + (b2 * d2v.y + aduvy) * norm.y + (b2 * d2v.x + aduvx) * norm.x) * vc) / ((aa2 * d2u.z + bb2 * d2v.z + ab2 * duv.z) * norm.z + (aa2 * d2u.y + bb2 * d2v.y + ab2 * duv.y) * norm.y + (aa2 * d2u.x + bb2 * d2v.x + ab2 * duv.x) * norm.x);

                    if (t < 0.0)
                    {
                        t = 0.0;
                        if (atend)
                        {
                            result = sp;
                            return true;
                        }
                        atend = true;
                    }
                    else if (t > 1.0)
                    {
                        t = 1.0;
                        if (atend)
                        {
                            result = ep;
                            return true;
                        }
                        atend = true;
                    }
                    else atend = false;
                    result = sp + t * ab;
                    uc = result.x; // for the next t
                    vc = result.y;
                    surface.Derivative2At(result, out location, out du, out dv, out duu, out dvv, out duv);
                    d2u = 0.5 * duu;
                    d2v = 0.5 * dvv;
                    bduvz = (b * duv.z); // for the next t
                    bduvy = (b * duv.y);
                    bduvx = (b * duv.x);
                    aduvz = (a * duv.z);
                    aduvy = (a * duv.y);
                    aduvx = (a * duv.x);

                    curvedir = a * du + b * dv;
                    double te = Math.Abs(curvedir.Normalized * norm); // the new error
                    if (te > err) return false; // does not converge
                    err = te;
                    if (err < 1e-10) return true;
#if DEBUG
                    Line l = Line.Construct();
                    l.SetTwoPoints(lastPoint, location);
                    dc.Add(l);
                    lastPoint = location;
#endif
                }

                return true;
            }
            catch (ApplicationException)
            {
                return false;
            }
        }
        /// <summary>
        /// Newton method for finding a maximum for a given direction, starting from a (u,v) point
        /// A 2nd degree polynomial in u and v is created at this point whose 1st and 2nd derivatives match the surface, so it approximates the surface at this point.
        /// For this polynomial the maximum in this direction can be found easily (2 linear equations such that diru and dirv are perpendicular to the given direction)
        /// This (u,v) point is transferred onto the surface for the next iteration step.
        /// </summary>
        /// <param name="startpos"></param>
        /// <param name="normal"></param>
        /// <param name="result"></param>
        /// <returns></returns>
        internal bool findMaxPlaneDist(GeoPoint2D startpos, double maxStepSize, GeoVector normal, out GeoPoint2D result)
        {
            // duu, dv etc.: the 1st and 2nd derivatives with respect to u and v, d2u = 1/2 * duu
            // The function f(u,v) = u²*d2u+v²*d2v+u*v*duv+u*du+v*dv+p0 approximates the surface at a point (d2u etc. are 3d vectors/points)
            // df/du(u,v) = 2*u*d2u + du + v*duv, df/dv(u,v) = 2*v*d2v + dv + u*duv are the two directions of the approximated surface
            //  (2*u*d2u + du + v*duv)*n = 0 and (2*v*d2v + dv + u*duv)*n = 0 (dot products, directions perpendicular to the given normal)
            // 2*d2u is the 2nd derivative of the nurbs, duu = 2*d2u, from which the two equations result:
            // u * (duu * n) + v * (duv * n) = -du*n and
            // u * (duv * n) + v * (dvv * n) = -dv*n
            // converges well when near the solution, otherwise it can jump around wildly. Then the step is reduced until it runs stably again
            GeoPoint location;
            GeoVector du;
            GeoVector dv;
            GeoVector duu;
            GeoVector dvv;
            GeoVector duv;
            result = startpos;
            try // e.g. when the derivatives become 0
            {
                normal.NormIfNotNull();
                surface.Derivative2At(result, out location, out du, out dv, out duu, out dvv, out duv);
#if DEBUG
                DebuggerContainer dc = new DebuggerContainer();
                GeoPoint lastPoint = location;
#endif
                double err = Math.Abs(du.Normalized * normal) + Math.Abs(dv.Normalized * normal); // this value should become 0
                for (int k = 0; k < 10; k++)
                {
                    Matrix m = new DenseMatrix(2, 2);
                    m[0, 0] = duu * normal;
                    m[1, 0] = m[0, 1] = duv * normal;
                    m[1, 1] = dvv * normal;
                    Vector b = new DenseVector(2);
                    b[0] = -du * normal;
                    b[1] = -dv * normal;
                    Vector x;

                    if (m.QR().IsFullRank) x = (Vector)m.QR().Solve(b); // qr is more stable when dealing with very small values, right?
                    else x = (Vector)m.Solve(b);
                    if (x.IsValid())
                    {
                        GeoVector2D step = new GeoVector2D(x[0], x[1]);
                        if (step.Length > maxStepSize * 2) return false;
                        double te;
                        do
                        {
                            surface.Derivative2At(result + step, out location, out du, out dv, out duu, out dvv, out duv);
                            te = Math.Abs(du.Normalized * normal) + Math.Abs(dv.Normalized * normal);
                            if (te < err) break;
                            step = 0.3 * step; // if the error does not get smaller, take a third of the step
                        } while (step.Length > 1e-10);
                        err = te;
                        result = result + step;
                        if (err < 1e-10) break;
#if DEBUG
                        Line l = Line.Construct();
                        l.SetTwoPoints(lastPoint, location);
                        dc.Add(l);
                        lastPoint = location;
#endif
                    }
                    else return false; // linear (at least in one direction)
                }

                return true;
            }
            catch (ApplicationException)
            {
                return false;
            }
        }

        private bool NewtonLineIntersection(BoundingRect boundingRect, GeoPoint startPoint, GeoVector direction, out GeoPoint2D uvSurface)
        {   // if there is an intersection point in this patch, it must also be found.
            // It is not certain whether, starting from the 4 corner points and the center, an intersection point will be found
            // reliably with the Newton method. It may be that there is an intersection point, but all initial conditions converge
            // to a different intersection point. Such an example would still have to be found and used as a criterion for the
            // subdivision into cubes.
            double umin, umax, vmin, vmax;
            surface.GetNaturalBounds(out umin, out umax, out vmin, out vmax);
            if (umin == double.MinValue || umax == double.MaxValue)
            {
                umin = uvbounds.Left;
                umax = uvbounds.Right;
            }
            if (vmin == double.MinValue || vmax == double.MaxValue)
            {
                vmin = uvbounds.Bottom;
                vmax = uvbounds.Top;
            }
            BoundingRect maximumuvRect = new BoundingRect(umin, vmin, umax, vmax); // so that it does not run out of range, clipping to the boundary would be better, right?
            maximumuvRect.Inflate(Precision.eps);
            boundingRect.Inflate(Precision.eps); // ot find intrsections exactely on the border of the surface
            for (int c = 0; c < 5; ++c)
            {
                // try with different start points. If no solution can be found from any start point, then
                // give up
                switch (c)
                {
                    default:
                    case 0:
                        uvSurface = boundingRect.GetCenter();
                        break;
                    case 1:
                        uvSurface = boundingRect.GetLowerLeft();
                        break;
                    case 2:
                        uvSurface = boundingRect.GetUpperRight();
                        break;
                    case 3:
                        uvSurface = boundingRect.GetUpperLeft();
                        break;
                    case 4:
                        uvSurface = boundingRect.GetLowerRight();
                        break;
                }
                GeoVector udir = surface.UDirection(uvSurface);
                GeoVector vdir = surface.VDirection(uvSurface); // their length must be correct too!
                GeoPoint loc = surface.PointAt(uvSurface);
                double error = Geometry.DistPL(loc, startPoint, direction);
                int errorcount = 0;
                int outside = 0; // if an intersection is outside, do not give up immediately, only after twice in a row outside
                                 // so that points oscillating at the border can be found
#if DEBUG
                {
                    //DebuggerContainer dc = new DebuggerContainer();
                    //try
                    //{
                    //    Face fc = Face.MakeFace(surface, new CADability.Shapes.SimpleShape(boundingRect));
                    //    dc.Add(fc);
                    //}
                    //catch (Polyline2DException) { }
                    //try
                    //{
                    //    Plane pln = new Plane(loc, udir, vdir);
                    //    Face plnfc = Face.MakeFace(new PlaneSurface(loc, udir, vdir, udir ^ vdir), new CADability.Shapes.SimpleShape(CADability.Shapes.Border.MakeRectangle(0, 1, 0, 1)));
                    //    dc.Add(plnfc);
                    //    Line ln = Line.Construct();
                    //    ln.SetTwoPoints(startPoint - direction, startPoint + direction);
                    //    dc.Add(ln);
                    //}
                    //catch (PlaneException) { }
                }
#endif

                while (true) // either a break or return will come
                {
                    Matrix m = DenseMatrix.OfColumnArrays(udir, vdir, direction);
                    Vector s = (Vector)m.Solve(new DenseVector(startPoint - loc));
                    if (s.IsValid())
                    {
                        double du = s[0];
                        double dv = s[1];
                        if (du > umax - umin) du = umax - umin;
                        if (du < umin - umax) du = umin - umax;
                        if (dv > vmax - vmin) dv = vmax - vmin;
                        if (dv < vmin - vmax) dv = vmin - vmax;
                        double l = s[2];
                        uvSurface.x += du; // or -=
                        uvSurface.y += dv; // or -=
                        loc = surface.PointAt(uvSurface);
                        udir = surface.UDirection(uvSurface);
                        vdir = surface.VDirection(uvSurface); // their length must be correct too!
                        double e = Geometry.DistPL(loc, startPoint, direction);
#if DEBUG
                        //DebuggerContainer dc = new DebuggerContainer();
                        //try
                        //{
                        //    Face fc = Face.MakeFace(surface, new CADability.Shapes.SimpleShape(boundingRect));
                        //    dc.Add(fc);
                        //}
                        //catch (Polyline2DException) { }
                        //try
                        //{
                        //    Plane pln = new Plane(loc, udir, vdir);
                        //    Face plnfc = Face.MakeFace(new PlaneSurface(pln), new CADability.Shapes.SimpleShape(CADability.Shapes.Border.MakeRectangle(-1, 1, -1, 1)));
                        //    dc.Add(plnfc);
                        //}
                        //catch (PlaneException) { }
                        //Line ln = Line.Construct();
                        //ln.SetTwoPoints(startPoint - direction, startPoint + direction);
                        //dc.Add(ln);
                        //ln = Line.Construct();
                        //ln.SetTwoPoints(loc, loc + udir);
                        //dc.Add(ln, 1);
                        //ln = Line.Construct();
                        //ln.SetTwoPoints(loc, loc + vdir);
                        //dc.Add(ln, 2);
#endif
                        if (!boundingRect.Contains(uvSurface))
                        {
                            if (!maximumuvRect.Contains(uvSurface)) break; // because we cannot continue computing with it
                                                                           // at most one could clip to the maximumuvRect range
                            ++outside;
                            if (outside > 2) break; // runs out of the patch. Try with a different start value
                        }
                        else
                        {
                            outside = 0; // inside, reset
                        }
                        if (e >= error)
                        {
                            ++errorcount;
                            // sometimes it makes a small detour before it converges, so do not abort immediately
                            if (errorcount > 5) break; // does not converge, abort with this start value
                        }
                        else if (e < Math.Max(Precision.eps, loc.Size * 1e-6))
                        {
                            return true; // found!
                        }
                        error = e;
                    }
                    else
                    {   // singular matrix
                        break;
                    }
                }
            }
            uvSurface = boundingRect.GetCenter(); // just needs to have a value
            return false; // no solution found
        }
        public ICurve[] GetSelfIntersection()
        {
            // The idea: two cubes overlap. Consider the u and v directions at the corners.
            // If two of them point in opposite directions (>90°), then there is the possibility of
            // self-intersection. Then split further. Ultimately intersect fixed u or v curve segments
            // with the surface.
            return null;
        }
        class ComputeIntersectionCurve
        {
            class UVPatch : IQuadTreeInsertable
            {   // each patch holds at most two IntersectionPoints.
                // in doubt at the corner points, patches that have only one point are ignored
                public BoundingRect extent;
                public IntersectionPoint point1;
                public IntersectionPoint point2;
                public void Add(IntersectionPoint intersectionPoint)
                {
                    if (point1 == null) point1 = intersectionPoint;
                    else point2 = intersectionPoint;
                }
                internal void Remove(IntersectionPoint toRemove)
                {
                    if (point1 == toRemove)
                    {
                        point1 = point2;
                        point2 = null;
                    }
                    else if (point2 == toRemove)
                    {
                        point2 = null;
                    }
                }

                #region IQuadTreeInsertable Members

                BoundingRect IQuadTreeInsertable.GetExtent()
                {
                    return extent;
                }

                bool IQuadTreeInsertable.HitTest(ref BoundingRect rect, bool includeControlPoints)
                {
                    return extent.Interferes(ref rect);
                }

                object IQuadTreeInsertable.ReferencedObject
                {
                    get { throw new Exception("The method or operation is not implemented."); }
                }

                #endregion
            }
            class IntersectionPoint
            {   // A point is always the intersection of a mesh edge, i.e. a boundary of a uvPatch
                // If points lie exactly on corners, then onPatch3 and onPatch4 are also set
                public GeoPoint p;
                public GeoPoint2D pSurface1; // this is always the ParallelepipedHull
                public GeoPoint2D pSurface2; // this is the other surface
                public bool fixedu; // intersection of an edge with fixed u, if false with fixed v
                public bool isOnPatchVertex; // lies exactly on a corner of the patch
                                             // the point lies on edges of these two patches (can also be just one, border, what about corners?)
                public UVPatch onPatch1;
                public UVPatch onPatch2;
                int hashCode;
                static int hashCodeCounter = 0;
                public IntersectionPoint()
                {
                    hashCode = ++hashCodeCounter;
                }

                public void AddPatch(UVPatch uvPatch)
                {
                    if (isOnPatchVertex)
                    {   // this is a corner point of a patch, then only the upper or right ones ever count
                        if (fixedu)
                        {   // the point must be the lower edge of the patch, because here only the upper ones should count
                            if (uvPatch.extent.Top == pSurface1.y) return;
                        }
                        else
                        {
                            if (uvPatch.extent.Right == pSurface1.x) return;
                        }
                    }
                    if (onPatch1 == null) onPatch1 = uvPatch;
                    else if (onPatch2 == null) onPatch2 = uvPatch;
                    else
                    {   // must not occur, for breakpoint
                    }
                    uvPatch.Add(this);
                }
                public override int GetHashCode()
                {
                    return hashCode;
                }
                public override bool Equals(object obj)
                {
                    return (obj as IntersectionPoint).hashCode == hashCode;
                }
                internal void RemovePatch(UVPatch uVPatch)
                {
                    if (onPatch1 == uVPatch)
                    {
                        onPatch1 = onPatch2;
                        onPatch2 = null;
                    }
                    else if (onPatch2 == uVPatch)
                    {
                        onPatch2 = null;
                    }
                }
            }
            ParallelepipedHull ParallelepipedHull;
            ISurfaceImpl toIntersectWith;
            BoundingRect uvSize;
            Dictionary<double, List<IntersectionPoint>> uIntersections; // intersection points for fixed u already determined
            Dictionary<double, List<IntersectionPoint>> vIntersections;
            QuadTree<UVPatch> patches;
            Set<IntersectionPoint> intersectionPoints; // intersection points found so far
            List<IntersectionPoint> onPatchVertex; // discarded intersection points, since they occur twice and lie exactly
                                                   // on the corner of a patch. Possibly curves have to be rejoined at them
            public ComputeIntersectionCurve(ParallelepipedHull ParallelepipedHull, ISurfaceImpl toIntersectWith, double umin, double umax, double vmin, double vmax)
            {
                this.ParallelepipedHull = ParallelepipedHull;
                this.toIntersectWith = toIntersectWith;
                uIntersections = new Dictionary<double, List<IntersectionPoint>>();
                vIntersections = new Dictionary<double, List<IntersectionPoint>>();
                uvSize = new BoundingRect(umin, vmin, umax, vmax);
                patches = new QuadTree<UVPatch>(uvSize);
                intersectionPoints = new Set<IntersectionPoint>();
                onPatchVertex = new List<IntersectionPoint>();
            }
            List<IntersectionPoint> FixedParameterIntersections(double uv, bool uParameter)
            {   // this should return ALL intersection points for a fixed u or v value.
                List<IntersectionPoint> res = new List<IntersectionPoint>();
                if (uParameter)
                {
                    if (!uIntersections.TryGetValue(uv, out res))
                    {
                        ICurve fu = ParallelepipedHull.surface.FixedU(uv, ParallelepipedHull.uvbounds.Bottom, ParallelepipedHull.uvbounds.Top);

                        GeoPoint[] ips;
                        GeoPoint2D[] uvOnSurface;
                        double[] uOnCurve;
                        toIntersectWith.Intersect(fu, uvSize, out ips, out uvOnSurface, out uOnCurve);
                        NewtonMend(toIntersectWith, fu, ref ips, ref uvOnSurface, ref uOnCurve);
                        res = new List<IntersectionPoint>();
                        for (int i = 0; i < ips.Length; ++i)
                        {
                            IntersectionPoint ip = new IntersectionPoint();
                            ip.p = ips[i];
                            double vOnSurface1 = ParallelepipedHull.uvbounds.Bottom + uOnCurve[i] * ParallelepipedHull.uvbounds.Height; // see below
                            ip.pSurface1 = new GeoPoint2D(uv, vOnSurface1);
                            // ip.pSurface1 = ParallelepipedHull.surface.PositionOf(ips[i]);
                            ip.pSurface2 = uvOnSurface[i];
                            ip.fixedu = true;
                            res.Add(ip);
                            // the patches are still missing
                        }
                        uIntersections[uv] = res;
                    }
                }
                else
                {
                    if (!vIntersections.TryGetValue(uv, out res))
                    {
                        ICurve fv = ParallelepipedHull.surface.FixedV(uv, ParallelepipedHull.uvbounds.Left, ParallelepipedHull.uvbounds.Right);
                        GeoPoint[] ips;
                        GeoPoint2D[] uvOnSurface;
                        double[] uOnCurve;
                        toIntersectWith.Intersect(fv, uvSize, out ips, out uvOnSurface, out uOnCurve);
                        NewtonMend(toIntersectWith, fv, ref ips, ref uvOnSurface, ref uOnCurve);
                        res = new List<IntersectionPoint>();
                        for (int i = 0; i < ips.Length; ++i)
                        {
                            IntersectionPoint ip = new IntersectionPoint();
                            ip.p = ips[i];
                            // we must get to the "natural" parameter of u
                            // uOnCurve is in the 0..1 system of the curve. hopefully this is linear
                            double uOnSurface1 = ParallelepipedHull.uvbounds.Left + uOnCurve[i] * ParallelepipedHull.uvbounds.Width;
                            ip.pSurface1 = new GeoPoint2D(uOnSurface1, uv);
                            // ip.pSurface1 = ParallelepipedHull.surface.PositionOf(ips[i]);
                            ip.pSurface2 = uvOnSurface[i];
                            ip.fixedu = false;
                            res.Add(ip);
                            // the patches are still missing
                        }
                        vIntersections[uv] = res;
                    }
                }
                return res;
            }

            private void NewtonMend(ISurface surface, ICurve curve, ref GeoPoint[] ips, ref GeoPoint2D[] uvOnSurface, ref double[] uOnCurve)
            {   // Mend means repair, improve. The termination criterion when finding intersection points is when the point on the curve
                // and the point on the surface are less than eps apart. But that is not good, because for flat intersections
                // it can still be too far from the real intersection point. Therefore improve here with Newton and
                // terminate taking the angle into account.
                for (int i = 0; i < ips.Length; ++i)
                {
                    GeoVector udir = surface.UDirection(uvOnSurface[i]);
                    GeoVector vdir = surface.VDirection(uvOnSurface[i]);
                    GeoVector normal = udir ^ vdir;
                    GeoVector direction = curve.DirectionAt(uOnCurve[i]);
                    double eps = Precision.eps * Math.Abs(GeoVector.Cos(normal, direction));
                    GeoPoint pOnSurface = surface.PointAt(uvOnSurface[i]);
                    GeoPoint pOnCurve = curve.PointAt(uOnCurve[i]);
                    double error = pOnCurve | pOnSurface;
                    while (error > eps)
                    {
                        Matrix m = DenseMatrix.OfColumnArrays(udir, vdir, direction);
                        Vector s = (Vector)m.Solve(new DenseVector(pOnCurve - pOnSurface));
                        if (s.IsValid())
                        {
                            double du = s[0];
                            double dv = s[1];
                            double l = s[2];
                            uvOnSurface[i].x += du;
                            uvOnSurface[i].y += dv;
                            uOnCurve[i] -= l;
                            pOnSurface = surface.PointAt(uvOnSurface[i]);
                            pOnCurve = curve.PointAt(uOnCurve[i]);
                            udir = surface.UDirection(uvOnSurface[i]);
                            vdir = surface.VDirection(uvOnSurface[i]);
                            normal = udir ^ vdir;
                            direction = curve.DirectionAt(uOnCurve[i]);
                            eps = Precision.eps * Math.Abs(GeoVector.Cos(normal, direction));
                            double e = pOnCurve | pOnSurface;
                            if (e >= error) return; // does not converge, should not happen during improvement
                            error = e;
                        }
                        else // singular matrix
                        {
                            return;
                        }
                    }
                    ips[i] = new GeoPoint(pOnSurface, pOnCurve);
                }
            }
            void CheckIntersectinPoints(ParEpi cube)
            {
                List<IntersectionPoint> left = FixedParameterIntersections(cube.uvPatch.Left, true);
                List<IntersectionPoint> right = FixedParameterIntersections(cube.uvPatch.Right, true);
                List<IntersectionPoint> bottom = FixedParameterIntersections(cube.uvPatch.Bottom, false);
                List<IntersectionPoint> top = FixedParameterIntersections(cube.uvPatch.Top, false);
                List<IntersectionPoint> inPatch = new List<IntersectionPoint>();
                UVPatch uvPatch = new UVPatch();
                uvPatch.extent = cube.uvPatch;
                // first find all that also lie in the patch, since FixedParameterIntersections returns all of them
                for (int i = 0; i < left.Count; ++i)
                {   // first snap to corner points, if it is a corner point
                    if (Math.Abs(left[i].pSurface1.y - cube.uvPatch.Bottom) < ParallelepipedHull.uvbounds.Height * 1e-6)
                    {
                        left[i].pSurface1.y = cube.uvPatch.Bottom;
                        left[i].isOnPatchVertex = true;
                    }
                    if (Math.Abs(left[i].pSurface1.y - cube.uvPatch.Top) < ParallelepipedHull.uvbounds.Height * 1e-6)
                    {
                        left[i].pSurface1.y = cube.uvPatch.Top;
                        left[i].isOnPatchVertex = true;
                    }
                    if (left[i].pSurface1.y >= cube.uvPatch.Bottom && left[i].pSurface1.y <= cube.uvPatch.Top)
                    {
                        inPatch.Add(left[i]);
                    }
                }
                for (int i = 0; i < right.Count; ++i)
                {
                    if (Math.Abs(right[i].pSurface1.y - cube.uvPatch.Bottom) < ParallelepipedHull.uvbounds.Height * 1e-6)
                    {
                        right[i].pSurface1.y = cube.uvPatch.Bottom;
                        right[i].isOnPatchVertex = true;
                    }
                    if (Math.Abs(right[i].pSurface1.y - cube.uvPatch.Top) < ParallelepipedHull.uvbounds.Height * 1e-6)
                    {
                        right[i].pSurface1.y = cube.uvPatch.Top;
                        right[i].isOnPatchVertex = true;
                    }
                    if (right[i].pSurface1.y >= cube.uvPatch.Bottom && right[i].pSurface1.y <= cube.uvPatch.Top)
                    {
                        inPatch.Add(right[i]);
                    }
                }
                for (int i = 0; i < bottom.Count; ++i)
                {
                    if (Math.Abs(bottom[i].pSurface1.x - cube.uvPatch.Left) < ParallelepipedHull.uvbounds.Width * 1e-6)
                    {
                        bottom[i].pSurface1.x = cube.uvPatch.Left;
                        bottom[i].isOnPatchVertex = true;
                    }
                    if (Math.Abs(bottom[i].pSurface1.x - cube.uvPatch.Right) < ParallelepipedHull.uvbounds.Width * 1e-6)
                    {
                        bottom[i].pSurface1.x = cube.uvPatch.Right;
                        bottom[i].isOnPatchVertex = true;
                    }
                    if (bottom[i].pSurface1.x >= cube.uvPatch.Left && bottom[i].pSurface1.x <= cube.uvPatch.Right)
                    {
                        inPatch.Add(bottom[i]);
                    }
                }
                for (int i = 0; i < top.Count; ++i)
                {
                    if (Math.Abs(top[i].pSurface1.x - cube.uvPatch.Left) < ParallelepipedHull.uvbounds.Width * 1e-6)
                    {
                        top[i].pSurface1.x = cube.uvPatch.Left;
                        top[i].isOnPatchVertex = true;
                    }
                    if (Math.Abs(top[i].pSurface1.x - cube.uvPatch.Right) < ParallelepipedHull.uvbounds.Width * 1e-6)
                    {
                        top[i].pSurface1.x = cube.uvPatch.Right;
                        top[i].isOnPatchVertex = true;
                    }
                    if (top[i].pSurface1.x >= cube.uvPatch.Left && top[i].pSurface1.x <= cube.uvPatch.Right)
                    {
                        inPatch.Add(top[i]);
                    }
                }
                // if an intersection point lies exactly on the corner of a uvPatch, it often occurs twice
                // once for each side. One is discarded here. This can cause curves to be
                // interrupted. With the help of onPatchVertex they could be reassembled
                bool removed;
                do
                {
                    removed = false;
                    for (int i = 0; i < inPatch.Count - 1; ++i)
                    {
                        for (int j = i + 1; j < inPatch.Count; ++j)
                        {
                            if ((inPatch[i].p | inPatch[j].p) < 2 * Precision.eps ||
                                (Math.Abs(inPatch[i].pSurface1.x - inPatch[j].pSurface1.x) < cube.uvPatch.Width * 1e-6 &&
                                Math.Abs(inPatch[i].pSurface1.y - inPatch[j].pSurface1.y) < cube.uvPatch.Height * 1e-6))
                            {   // each point has the error Precision.eps, therefore 2*
                                onPatchVertex.Add(inPatch[j]);
                                inPatch.RemoveAt(j);
                                removed = true;
                                break; // there are always only pairs and j>i, so we can break without problem
                                       // unfortunately for tangents there are often multiple almost identical points
                                       // that should better be solved there, then this loop would simply suffice
                            }
                        }
                    }
                } while (removed);
                if (inPatch.Count < 2)
                {
                    return;
                }
                else if (inPatch.Count == 2)
                {
                    for (int i = 0; i < inPatch.Count; ++i)
                    {
                        inPatch[i].AddPatch(uvPatch);
                        intersectionPoints.Add(inPatch[i]);
                    }
                }
                else
                {
                    // here some could sit on the corners, that must be checked first
                    // in general: multiple intersections, so split
#if DEBUG
                    DebuggerContainer dc = new DebuggerContainer();
                    try
                    {
                        Face fcdbg = Face.MakeFace(ParallelepipedHull.surface, new CADability.Shapes.SimpleShape(uvPatch.extent));
                        dc.Add(fcdbg);
                    }
                    catch (Polyline2DException) { }
                    BoundingRect uvplane = BoundingRect.EmptyBoundingRect;
                    for (int i = 0; i < inPatch.Count; ++i)
                    {
                        uvplane.MinMax(inPatch[i].pSurface2);
                        Point pnt = Point.Construct();
                        pnt.Location = inPatch[i].p;
                        pnt.Symbol = PointSymbol.Cross;
                        pnt.ColorDef = new CADability.Attribute.ColorDef("xxx", Color.Black);
                        dc.Add(pnt);
                    }
                    uvplane.Inflate(1.0, 1.0);
                    Face fcpln = Face.MakeFace(this.toIntersectWith, new CADability.Shapes.SimpleShape(uvplane));
                    dc.Add(fcpln);
#endif
                    ParEpi[] cubes = ParallelepipedHull.SplitCube(cube); // is done for real, i.e. also split in the octtree
                    for (int i = 0; i < cubes.Length; ++i)
                    {
                        CheckIntersectinPoints(cubes[i]);
                    }
                }
            }
            public IDualSurfaceCurve[] GetIntersectionCurves(BoundingRect br)
            {
                // Hangs for parallel surfaces, is split completely if the surfaces are identical.
                ParEpi[] cubes = ParallelepipedHull.octtree.GetObjectsCloseTo(toIntersectWith);
                for (int i = 0; i < cubes.Length; ++i)
                {
                    if (cubes[i].uvPatch.Interferes(ref br)) CheckIntersectinPoints(cubes[i]);
                }
                IntersectionPoint startwith = FindSinglePatchIntersectionPoint();
                List<List<IntersectionPoint>> curves = new List<List<IntersectionPoint>>();
                while (startwith != null)
                {   // open curves, corner points in the uvPatch are not yet considered...
                    List<IntersectionPoint> curve = new List<IntersectionPoint>();
                    IntersectionPoint goOnWith = startwith;
                    while (goOnWith != null)
                    {   // always continue with onPatch1, since the point loses its origin patch when used
                        if (intersectionPoints.Contains(goOnWith))
                        {
                            curve.Add(goOnWith);
                        }
                        else break;
                        IntersectionPoint toRemove = goOnWith;
                        if (goOnWith.onPatch1 == null)
                        {
                            goOnWith = null;
                        }
                        else
                        {
                            if (goOnWith.onPatch1.point1 == goOnWith)
                            {
                                goOnWith = goOnWith.onPatch1.point2;
                            }
                            else
                            {
                                goOnWith = goOnWith.onPatch1.point1;
                            }
                        }
                        if (goOnWith != null) goOnWith.RemovePatch(toRemove.onPatch1);
                        RemoveIntersectionPoint(toRemove); // only remove here, since it is also removed from the patches
                    }
                    if (curve.Count > 1) curves.Add(curve);
                    startwith = FindSinglePatchIntersectionPoint();
                }
                if (intersectionPoints.Count > 0)
                {
                    startwith = intersectionPoints.GetAny(); // some start, now there are closed ones
                }
                while (startwith != null)
                {   // closed curves
                    List<IntersectionPoint> curve = new List<IntersectionPoint>();
                    IntersectionPoint goOnWith = startwith;
                    while (goOnWith != null)
                    {
                        if (curve.Count == 0 || ((curve[curve.Count - 1].p | goOnWith.p) > Precision.eps))
                        {   // not the same point twice
                            curve.Add(goOnWith);
                        }
                        intersectionPoints.Remove(goOnWith);
                        if (goOnWith.onPatch1.point1 == goOnWith)
                        {
                            goOnWith = goOnWith.onPatch1.point2;
                        }
                        else
                        {
                            goOnWith = goOnWith.onPatch1.point1;
                        }
                        if (!intersectionPoints.Contains(goOnWith)) break;
                        if (goOnWith == startwith)
                        {
                            if (curve.Count == 0 || ((curve[curve.Count - 1].p | goOnWith.p) > Precision.eps))
                            {
                                curve.Add(goOnWith); // so that it is closed
                            }
                            break;
                        }
                    }
                    if (curve.Count > 1) curves.Add(curve);
                    if (intersectionPoints.Count > 0)
                    {
                        startwith = intersectionPoints.GetAny(); // some start, now there are closed ones
                    }
                    else
                    {
                        startwith = null;
                    }
                }
                IDualSurfaceCurve[] res = new IDualSurfaceCurve[curves.Count];
                for (int i = 0; i < curves.Count; ++i)
                {
                    res[i] = MakeDualSurfaceCurve(curves[i]);
                }
                //res[1].Curve2D1.PointAt(0.55555);
                //GeoPoint2D[] p2d = new GeoPoint2D[100];
                //for (int i = 0; i < 100; i++)
                //{
                //    p2d[i] = res[1].Curve2D1.PointAt(i/100.0);
                //}
                //Polyline2D pl2d = new Polyline2D(p2d);
                return res;
            }

            private void RemoveIntersectionPoint(IntersectionPoint toRemove)
            {
                if (toRemove.onPatch1 != null) toRemove.onPatch1.Remove(toRemove);
                if (toRemove.onPatch2 != null) toRemove.onPatch2.Remove(toRemove);
                intersectionPoints.Remove(toRemove);
            }

            private IDualSurfaceCurve MakeDualSurfaceCurve(List<IntersectionPoint> list)
            {   // very many points may have been created on the curve if the surface had to be subdivided into many
                // pieces. too many points are a nuisance for the further use of the curve
                // e.g. high triangle counts etc., therefore we reduce here, at least 5 remain, right?
#if DEBUG
                GeoPoint[] pl = new GeoPoint[list.Count];
                for (int i = 0; i < pl.Length; ++i)
                {
                    pl[i] = list[i].p;
                }
                Polyline pol = Polyline.Construct();
                pol.SetPoints(pl, false);
#endif
                while (list.Count > 10)
                {
                    bool removed = false;
                    for (int i = list.Count - 2; i > 0; --i)
                    {
                        if (Geometry.DistPL(list[i].p, list[i + 1].p, list[i - 1].p) < (list[i + 1].p | list[i - 1].p) * 1e-3)
                        {
                            list.RemoveAt(i);
                            --i;
                            removed = true;
                        }
                        if (list.Count < 3) break;
                    }
                    if (!removed) break;
                }
                InterpolatedDualSurfaceCurve.SurfacePoint[] basePoints = new InterpolatedDualSurfaceCurve.SurfacePoint[list.Count];
                for (int i = 0; i < list.Count; ++i)
                {
                    basePoints[i] = new InterpolatedDualSurfaceCurve.SurfacePoint(list[i].p, list[i].pSurface1, list[i].pSurface2);
                }
                InterpolatedDualSurfaceCurve isc = new InterpolatedDualSurfaceCurve(ParallelepipedHull.surface, toIntersectWith, basePoints);
                return isc.ToDualSurfaceCurve();
            }

            private IntersectionPoint FindSinglePatchIntersectionPoint()
            {
                IntersectionPoint res = null;
                foreach (IntersectionPoint ip in intersectionPoints)
                {
                    if (ip.onPatch2 == null)
                    {
                        res = ip;
                        break;
                    }
                }
                if (res != null) intersectionPoints.Remove(res);
                return res;
            }
#if DEBUG
            DebuggerContainer Debug
            {
                get
                {
                    Set<UVPatch> allPatches = new Set<UVPatch>();
                    foreach (IntersectionPoint ip in intersectionPoints)
                    {
                        allPatches.Add(ip.onPatch1);
                        if (ip.onPatch2 != null)
                        {
                            allPatches.Add(ip.onPatch2);
                        }
                    }
                    DebuggerContainer dc = new DebuggerContainer();
                    foreach (UVPatch uvp in allPatches)
                    {
                        dc.Add(new Line2D(new GeoPoint2D(uvp.extent.Left, uvp.extent.Bottom), new GeoPoint2D(uvp.extent.Right, uvp.extent.Bottom)), Color.Blue, 0);
                        dc.Add(new Line2D(new GeoPoint2D(uvp.extent.Right, uvp.extent.Bottom), new GeoPoint2D(uvp.extent.Right, uvp.extent.Top)), Color.Blue, 0);
                        dc.Add(new Line2D(new GeoPoint2D(uvp.extent.Right, uvp.extent.Top), new GeoPoint2D(uvp.extent.Left, uvp.extent.Top)), Color.Blue, 0);
                        dc.Add(new Line2D(new GeoPoint2D(uvp.extent.Left, uvp.extent.Top), new GeoPoint2D(uvp.extent.Left, uvp.extent.Bottom)), Color.Blue, 0);
                        if (uvp.point1 != null && uvp.point2 != null)
                        {
                            dc.Add(new Line2D(uvp.point1.pSurface1, uvp.point2.pSurface1), Color.Red, 0);
                        }
                    }
                    return dc;
                }
            }
#endif

        }

        public virtual IDualSurfaceCurve[] GetPlaneIntersection(PlaneSurface pl, double umin, double umax, double vmin, double vmax, double precision)
        {
            Plane pln = pl.Plane;
            ParEpi[] cubesOnPlane = octtree.GetObjectsFromPlane(pln);
            if (cubesOnPlane.Length == 0) return new IDualSurfaceCurve[0];
            BoundingRect plbounds = BoundingRect.EmptyBoundingRect;
            for (int i = 0; i < cubesOnPlane.Length; i++)
            {
                plbounds.MinMax(pln.Project(cubesOnPlane[i].pll));
                plbounds.MinMax(pln.Project(cubesOnPlane[i].plr));
                plbounds.MinMax(pln.Project(cubesOnPlane[i].pul));
                plbounds.MinMax(pln.Project(cubesOnPlane[i].pur)); // is it enough to take the 4 points of the other surface?
                                                                   // it should be fast, the extent itself is not so important
            }
            PlaneSurface other = pl.Clone() as PlaneSurface;
            plbounds.Inflate(plbounds.Size); // this should make it large enough in any case
            other.usedArea = plbounds; // without usedArea one cannot build a ParallelepipedHull from it, and Intersect needs that
            ICurve[] cvs = Intersect(new BoundingRect(umin, vmin, umax, vmax), other, plbounds, new List<GeoPoint>());
#if DEBUG
            Face dbgfc = Face.MakeFace(other, new SimpleShape(Border.MakeRectangle(plbounds)));
#endif
            IDualSurfaceCurve[] res = new IDualSurfaceCurve[cvs.Length];
            for (int i = 0; i < res.Length; i++)
            {
                res[i] = cvs[i] as IDualSurfaceCurve;
            }
            return res;

            //Unreachable code
            /*
            // old text:
            ComputeIntersectionCurve cic = new ComputeIntersectionCurve(this, pl, umin, umax, vmin, vmax);
#if DEBUG
            ParEpi[] allCubes = this.octtree.GetAllObjects();
            // System.Diagnostics.Trace.WriteLine("Number of boxes: " + allCubes.Length.ToString());
#endif
            return cic.GetIntersectionCurves(new BoundingRect(umin, vmin, umax, vmax));
            */
        }
        public virtual IDualSurfaceCurve[] GetSurfaceIntersection(ISurface surface, double umin, double umax, double vmin, double vmax, double precision)
        {
            // to use the better method "Intersect" (see GetPlaneIntersection), one would need to know a region on "surface"
            // check this for the concrete call case.
            ComputeIntersectionCurve cic = new ComputeIntersectionCurve(this, surface as ISurfaceImpl, umin, umax, vmin, vmax);
            return cic.GetIntersectionCurves(new BoundingRect(umin, vmin, umax, vmax));
        }
        struct Position
        {
            public double u;   // on the curve
            public GeoPoint2D uv; // on the surface
            public BoundingRect patch; // surface patch
            public GeoPoint pcurve;
            public GeoPoint psurface;
            public double distance; // signed distance
            public GeoVector dir; // direction of the curve
            public GeoVector normal; // normal of the surface
        }
        public void IntersectEx(ICurve curve, out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve3Ds)
        {
            List<GeoPoint> lips = new List<GeoPoint>();
            List<GeoPoint2D> luvOnFace = new List<GeoPoint2D>();
            List<double> luOnCurve = new List<double>();
            Dictionary<double, Position> relevantPositions = new Dictionary<double, Position>();
            TetraederHull th = new TetraederHull(curve); // this must be stored in the curve!!!
                                                         // collect all relevant ParEpis
            Set<ParEpi> relevantCubes = new Set<ParEpi>();
            for (int i = 0; i < th.TetraederBase.Length - 1; ++i)
            {
                ParEpi[] cubes;
                if (th.IsLine(i))
                {
                    cubes = octtree.GetObjectsCloseTo(new OctTreeLine(th.TetraederBase[i], th.TetraederBase[i + 1]));
                }
                else if (th.IsTriangle(i))
                {
                    cubes = octtree.GetObjectsCloseTo(new OctTreeTriangle(th.TetraederBase[i], th.TetraederVertex[2 * i], th.TetraederBase[i + 1]));
                }
                else
                {
                    cubes = octtree.GetObjectsCloseTo(new OctTreeTetraeder(th.TetraederBase[i], th.TetraederVertex[2 * i], th.TetraederVertex[2 * i + 1], th.TetraederBase[i + 1]));
                }
                relevantCubes.AddMany(cubes);
            }
            // add all base points to the relevant points, if they are near the surface
            for (int i = 0; i < th.TetraederBase.Length; ++i)
            {
                GeoPoint2D uv;
                if (PositionOf(th.TetraederBase[i], out uv))
                {   // unfortunately the cube is unknown here, so it still has to be searched for
                    GeoPoint surfp = surface.PointAt(uv);
                    ParEpi[] cubes = octtree.GetObjectsFromPoint(surfp);
                    BoundingRect patch = BoundingRect.EmptyBoundingRect;
                    for (int j = 0; j < cubes.Length; j++)
                    {
                        if (cubes[j].uvPatch.Contains(uv))
                        {
                            patch = cubes[j].uvPatch;
                            relevantCubes.Remove(cubes[j]);
                            break;
                        }
                    }
                    if (!patch.IsEmpty())
                    {
                        Position pos = new Position();
                        pos.u = th.TetraederParams[i];
                        pos.uv = uv;
                        pos.patch = patch;
                        pos.pcurve = curve.PointAt(pos.u);
                        pos.psurface = surfp;
                        pos.normal = surface.GetNormal(uv).Normalized;
                        pos.distance = Geometry.LinePar(surfp, pos.normal, pos.pcurve);
                        pos.dir = curve.DirectionAt(pos.u);
                        relevantPositions[pos.u] = pos;
                    }
                }
            }
            // if the curve is large compared to the surface, the list of relevant points can also be empty
            // now all ParEpis not yet considered still have to generate points
            foreach (ParEpi cube in relevantCubes)
            {
                double u = curve.PositionOf(cube.GetCenter());
                GeoPoint pcurve = curve.PointAt(u);
                GeoPoint2D uv;
                if (PositionOf(pcurve, out uv))
                {
                    if (cube.uvPatch.Contains(uv))
                    {   // add as above
                    }
                }
            }
            // all patches along the relevant points must be connected. If there are gaps, then take intermediate
            // points and determine the patch...
            uvOnFaces = luvOnFace.ToArray();
            ips = lips.ToArray();
            uOnCurve3Ds = luOnCurve.ToArray();
        }

        public void Intersect(ICurve curve, BoundingRect uvExtent, out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve3Ds)
        {
            // Considerations 19.11.09:
            // 1.: The hull should consist of ParEpis that are not too distorted: if the u or v direction
            // is more than 8 times the respective other direction, then split that direction.
            // 2.: determine points on the curve: all corner points of the tetraeder curve plus additional points. For each
            // point we need the corresponding uv position on the surface - if there is one. For each candidate ParEpi
            // we need a point. Additionally entry and exit points, i.e. from points that lie outside the
            // hull to points that lie inside. The points as a dictionary u->(uv, distance, n*dir)
            // 3.: With these points as a basis we can proceed: if two points have different distance (sign)
            // then with Newton or bisection (depending on how steep and whether Newton converges). For points with the same
            // distance (sign) and different sign of n*dir (which indicates approaching or moving away) determine intermediate
            // points. Either one finds a different distance, then as above (Newton or bisection) or one finds
            // n*dir==0.0, then a minimum without intersection point is found.

            List<GeoPoint> lips = new List<GeoPoint>();
            List<GeoPoint2D> luvOnFace = new List<GeoPoint2D>();
            List<double> luOnCurve = new List<double>();
            if (curve is ISimpleCurve)
            {
                ParEpi[] cubes = octtree.GetObjectsCloseTo(curve as IOctTreeInsertable);
                for (int j = 0; j < cubes.Length; ++j)
                {
                    BoundingBox bc = cubes[j].BoundingBox;
                    if (cubes[j].uvPatch.Interferes(ref uvExtent) && (curve as IOctTreeInsertable).HitTest(ref bc, 0.0))
                    {   // only check the relevant cubes
                        // there is a bug: GetCurveIntersection only finds single intersection points where there might be multiple intersections
                        GetCurveIntersection(curve as ISimpleCurve, cubes[j], lips, luvOnFace, luOnCurve);
                    }
                }
            }
            else
            {
                TetraederHull th = new TetraederHull(curve); // this must be stored in the curve!!!
                /* not sufficiently safeguarded test for no intersection
                // first exclude the case that there are no intersection points
                // determine all tetraeder points to their uv position with respect to the surface
                // if all lie outside or all on the same side of the surface, then there is no intersection
                List<GeoPoint2D> uvPoints = new List<GeoPoint2D>();
                List<double> orientation = new List<double>();
                for (int i = 0; i < th.TetraederBase.Length; ++i)
                {
                    GeoPoint2D uv;
                    if (PositionOf(th.TetraederBase[i], out uv))
                    {
                        GeoPoint loc;
                        GeoVector du, dv;
                        surface.DerivationAt(uv, out loc, out du, out dv);
                        Matrix m = Matrix.RowVector(udir, vdir, udir ^ vdir);
                        Matrix s = m.SaveSolve(Matrix.RowVector(th.TetraederBase[i] - loc));
                        if (s != null)
                        {
                            uvPoints.Add(uv);
                            orientation.Add(s[2, 0]); // above or below the tangent plane
                        }
                    }
                }
                bool noIntersection = uvPoints.Count == 0;
                bool left = true;
                bool right = true;
                bool bottom = true;
                bool top = true;
                double umin,umx,vmin,vmax;
                surface.GetNaturalBounds(out umin, out umx, out vmin, out vmax);
                for (int i = 0; i < uvPoints.Count; i++)
                {
                    if (uvPoints[i].x > umin) left = false;
                    if (uvPoints[i].x < umax) right = false;
                    if (uvPoints[i].y > vmin) bottom = false;
                    if (uvPoints[i].y < vmax) top = false;
                }
                noIntersection |= left | right | bottom | top; // count==0: it is and stays true
                if (!noIntersection)
                {
                    // all points on the same side?
                    bool neg = true, pos = true;
                    for (int i = 0; i <orientation.Count; i++)
                    {
                        if (orientation[i] >= 0) neg = false;
                        if (orientation[i] <= 0) pos = false;
                    }
                    noIntersection |= neg | pos;
                }
                */
                for (int i = 0; i < th.TetraederBase.Length - 1; ++i)
                {
                    ParEpi[] cubes;

                    if (th.IsLine(i))
                    {
                        cubes = octtree.GetObjectsCloseTo(new OctTreeLine(th.TetraederBase[i], th.TetraederBase[i + 1]));
                    }
                    else if (th.IsTriangle(i))
                    {
                        cubes = octtree.GetObjectsCloseTo(new OctTreeTriangle(th.TetraederBase[i], th.TetraederVertex[2 * i], th.TetraederBase[i + 1]));
#if DEBUG
                        DebuggerContainer dccubes = new DebuggerContainer();
                        for (int j = 0; j < cubes.Length; j++)
                        {
                            dccubes.Add(cubes[j].AsBox, j);
                        }
#endif
                    }
                    else
                    {
                        cubes = octtree.GetObjectsCloseTo(new OctTreeTetraeder(th.TetraederBase[i], th.TetraederVertex[2 * i], th.TetraederVertex[2 * i + 1], th.TetraederBase[i + 1]));
                    }
                    for (int j = 0; j < cubes.Length; ++j)
                    {
                        if (cubes[j].uvPatch.Interferes(ref uvExtent) && cubes[j].Interferes(th.TetraederBase[i], th.TetraederBase[i + 1], th.TetraederVertex[2 * i], th.TetraederVertex[2 * i + 1]))
                        {
                            if (cubes[j].Interferes(curve, th.TetraederParams[i], th.TetraederParams[i + 1], th.TetraederBase[i], th.TetraederBase[i + 1], th.TetraederVertex[2 * i], th.TetraederVertex[2 * i + 1]))
                            {
                                GeoPoint2D uvStart = cubes[j].uvPatch.GetCenter();
                                double tStart = (th.TetraederParams[i] + th.TetraederParams[i + 1]) / 2;
                                bool found = false;
                                GeoPoint ip;
                                if (curve is InterpolatedDualSurfaceCurve dsc)
                                {
                                    GeoPoint closePoint;
                                    // choose a starting point in the middle of either the tetraeder or the uv patch, which ever is smaller
                                    double thDist = th.TetraederVertex[i] | th.TetraederVertex[i + 1];
                                    double cubeSize = cubes[j].Size;
                                    if (thDist < cubeSize) closePoint = curve.PointAt(tStart);
                                    else closePoint = surface.PointAt(uvStart);

                                    GeoPoint2D uv1 = dsc.Surface1.PositionOf(curve.PointAt(tStart));
                                    GeoPoint2D uv2 = dsc.Surface2.PositionOf(curve.PointAt(tStart));
                                    uvStart = surface.PositionOf(closePoint);
                                    ip = closePoint;
                                    if (BoxedSurfaceExtension.SurfacesIntersectionLM(dsc.Surface1, dsc.Surface2, surface, ref uv1, ref uv2, ref uvStart, ref ip))
                                    {
                                        luOnCurve.Add(curve.PositionOf(ip));
                                        luvOnFace.Add(uvStart);
                                        lips.Add(ip);
                                        found = true;
                                    }
                                }
                                // TODO: introduce flags to indicate whether surface or curve provide 2nd derivative
                                if (!found && BoxedSurfaceExtension.CurveSurfaceIntersection(surface, curve, ref uvStart, ref tStart, out ip))
                                {
                                    if (cubes[j].uvPatch.Contains(uvStart) && th.TetraederParams[i] <= tStart && tStart <= th.TetraederParams[i + 1])
                                    {
                                        luOnCurve.Add(tStart);
                                        luvOnFace.Add(uvStart);
                                        lips.Add(ip);
                                        found = true;
                                    }
                                }
                                //if (BoxedSurfaceExtension.CurveSurfaceIntersection(surface, curve, cubes[j].uvPatch, th.TetraederParams[i], th.TetraederParams[i + 1], ref uvStart, ref tStart, out ip))
                                //{
                                //    // Performance test: almost the same, CurveSurfaceIntersectionwith the TrustRegionNewtonCGMinimizer is a little slower than LevenbergMarquardtMinimizer
                                //    // TrustRegionDogLegMinimizer and TrustRegionNewtonCGMinimizer are about the same
                                //    for (int ii = 0; ii < 1000; ii++)
                                //    {
                                //        uvStart = cubes[j].uvPatch.GetCenter();
                                //        tStart = (th.TetraederParams[i] + th.TetraederParams[i + 1]) / 2;
                                //        BoxedSurfaceExtension.CurveSurfaceIntersection(surface, curve, cubes[j].uvPatch, th.TetraederParams[i], th.TetraederParams[i + 1], ref uvStart, ref tStart, out ip);
                                //    }
                                //    for (int ii = 0; ii < 1000; ii++)
                                //    {
                                //        uvStart = cubes[j].uvPatch.GetCenter();
                                //        tStart = (th.TetraederParams[i] + th.TetraederParams[i + 1]) / 2;
                                //        BoxedSurfaceExtension.CurveSurfaceIntersectionDL(surface, curve, cubes[j].uvPatch, th.TetraederParams[i], th.TetraederParams[i + 1], ref uvStart, ref tStart, out ip);
                                //    }
                                //}
                                if (!found && BoxedSurfaceExtension.CurveSurfaceIntersectionLM(surface, curve, ref uvStart, ref tStart, out ip))
                                {
                                    if (cubes[j].uvPatch.Contains(uvStart) && th.TetraederParams[i] <= tStart && tStart <= th.TetraederParams[i + 1])
                                    {
                                        luOnCurve.Add(tStart);
                                        luvOnFace.Add(uvStart);
                                        lips.Add(ip);
                                        found = true;
                                    }
                                }
                                if (!found) GetCurveIntersection(curve, th.TetraederParams[i], th.TetraederParams[i + 1], cubes[j], lips, luvOnFace, luOnCurve);
                            }
                        }
                    }
                }
            }
            bool startPointIncluded = false;
            bool endPointIncluded = false;
            for (int i = lips.Count - 1; i > 0; --i)
            {   // eliminate duplicate points, epsilon is a problem!
                if (Math.Abs(luOnCurve[i] - luOnCurve[i - 1]) < 1e-8)
                {
                    lips.RemoveAt(i);
                    luOnCurve.RemoveAt(i);
                    luvOnFace.RemoveAt(i);
                }
            }
            for (int i = 0; i < lips.Count; ++i)
            {   // eliminate duplicate points, epsilon is a problem!
                if (Math.Abs(luOnCurve[i]) < 1e-5) startPointIncluded = true;
                if (Math.Abs(1.0 - luOnCurve[i]) < 1e-5) endPointIncluded = true;
            }
            if (!startPointIncluded)
            {
                GeoPoint2D[] ft = surface.PerpendicularFoot(curve.StartPoint);
                for (int i = 0; i < ft.Length; i++)
                {
                    if ((surface.PointAt(ft[i]) | curve.StartPoint) < Precision.eps)
                    {
                        lips.Add(curve.StartPoint);
                        luOnCurve.Add(0.0);
                        luvOnFace.Add(ft[i]);
                        break;
                    }
                }
            }
            if (!endPointIncluded)
            {
                GeoPoint2D[] ft = surface.PerpendicularFoot(curve.EndPoint);
                for (int i = 0; i < ft.Length; i++)
                {
                    if ((surface.PointAt(ft[i]) | curve.EndPoint) < Precision.eps)
                    {
                        lips.Add(curve.EndPoint);
                        luOnCurve.Add(1.0);
                        luvOnFace.Add(ft[i]);
                        break;
                    }
                }
            }
            uvOnFaces = luvOnFace.ToArray();
            ips = lips.ToArray();
            uOnCurve3Ds = luOnCurve.ToArray();
            for (int i = 0; i < uvOnFaces.Length; i++)
            {
                SurfaceHelper.AdjustPeriodic(surface, uvbounds, ref uvOnFaces[i]);
            }
        }

        private void GetCurveIntersection(ISimpleCurve curve, ParEpi cube, List<GeoPoint> lips, List<GeoPoint2D> luvOnFace, List<double> luOnCurve)
        {
            GeoPoint ip;
            GeoPoint2D uv;
            double u;
            switch (NewtonCurveIntersection(curve, cube, out ip, out uv, out u))
            {
                case CurveIntersectionMode.simpleIntersection:
                    {
                        if (Math.Abs(surface.GetNormal(uv).Normalized * (curve as ICurve).DirectionAt(u).Normalized)<0.01)
                        {
                            BoxedSurfaceExtension.CurveSurfaceIntersectionLM_Tangential(surface, curve as ICurve, ref uv, ref u, out ip);
                        }
                        lips.Add(ip);
                        luvOnFace.Add(uv);
                        luOnCurve.Add(u);
                    }
                    break;
                case CurveIntersectionMode.noIntersection:
                    {   // if Newton does not find an intersection point, there can be several reasons (which one should perhaps distinguish)
                        // 1. Newton runs out of the patch. Newton would e.g. oscillate back and forth if the intersection point lies at an
                        // inflection point. Perhaps the patch should be made a bit larger so that such cases run smoothly
                        // 2. Newton does not converge. This only happens when one is too far from the intersection point. So subdivide then.
                        // 3. There is no intersection point. Then there is no need to split.
                        // The cases are however not easy to distinguish, therefore here we simply check against 1/100 of the maximum size
                        // and then abort. But this is not very effective
                        uv = cube.uvPatch.GetCenter();
                        u = 0.5; // in the middle, unimportant
                        if (BoxedSurfaceExtension.CurveSurfaceIntersection(surface, curve as ICurve, ref uv, ref u, out GeoPoint tmpip))
                        {
                            lips.Add(tmpip);
                            luvOnFace.Add(uv);
                            luOnCurve.Add(u);
                        }
                        else if (cube.Size > octtree.Extend.Size / 100)
                        {
                            ParEpi[] splitted = SplitCube(cube);
                            for (int i = 0; i < splitted.Length; ++i)
                            {
                                BoundingBox bc = splitted[i].BoundingBox;
                                // the test is unfortunately rather weak, but modifying the curve with ToUnit and then testing is
                                // too expensive. Perhaps make an extra interface that allows the test with a parallelepiped
                                if ((curve as IOctTreeInsertable).HitTest(ref bc, 0.0))
                                {
                                    GetCurveIntersection(curve, splitted[i], lips, luvOnFace, luOnCurve);
                                }
                            }
                        }
                    }
                    break;
                case CurveIntersectionMode.curveInSurface:
                    // do not add any points here
                    break;
            }

        }
        private enum CurveIntersectionMode { noIntersection, simpleIntersection, curveInSurface, tangential }
        private CurveIntersectionMode NewtonCurveIntersection(ISimpleCurve curve, ParEpi cube, out GeoPoint ip, out GeoPoint2D uv, out double u)
        {
            ICurve icurve = curve as ICurve;
            uv = cube.uvPatch.GetCenter();
            u = 0.5; // in the middle, unimportant
                     // BoxedSurfaceExtension.CurveSurfaceIntersection(surface, curve as ICurve, ref uv, ref u);
                     // BoxedSurfaceExtension.CurveSurfaceIntersection(surface, curve as ICurve, ref uv, ref u, out GeoPoint dbgip);
            GeoVector udir = surface.UDirection(uv);
            GeoVector vdir = surface.VDirection(uv); // their length must be correct too!
            GeoPoint loc = surface.PointAt(uv);
            GeoPoint curvepoint = icurve.PointAt(u);
            double error = curvepoint | loc;
            int errorcount = 0;
#if DEBUG
            //DebuggerContainer dc = new DebuggerContainer();
            //try
            //{
            //    Face fc = Face.MakeFace(surface, new CADability.Shapes.SimpleShape(cube.uvPatch));
            //    dc.Add(fc);
            //}
            //catch (Polyline2DException) { }
            //dc.Add(curve as IGeoObject);
#endif
            ip = new GeoPoint(loc, curvepoint);
            try
            {
                while (error > 0) //  Math.Max(Precision.eps, loc.Size * 1e-6))
                {
                    double[] pars = curve.GetPlaneIntersection(new Plane(loc, udir, vdir));
                    if (pars.Length == 0)
                    {
                        if (CheckCurveInSurface(icurve, 0.0, 1.0, cube, u, uv))
                        {
                            return CurveIntersectionMode.curveInSurface;
                        }
                        else
                        {
                            return CurveIntersectionMode.noIntersection;
                        }
                    }
                    double bestDistance = double.MaxValue;
                    for (int i = 0; i < pars.Length; ++i)
                    {
                        GeoPoint cp = icurve.PointAt(pars[i]);
                        double d = loc | cp;
                        if (d < bestDistance)
                        {
                            bestDistance = d;
                            curvepoint = cp;
                            u = pars[i];
                        }
                    }

                    //Matrix m = Matrix.RowVector(udir, vdir, udir ^ vdir);
                    Matrix m = DenseMatrix.OfColumnArrays(udir, vdir, icurve.DirectionAt(u));
                    Vector s = (Vector)m.Solve(new DenseVector(curvepoint - loc));
                    if (s.IsValid())
                    {
                        double du = s[0];
                        double dv = s[1];
                        uv.x += du;
                        uv.y += dv;
                        loc = surface.PointAt(uv);
                        double e = loc | curvepoint;
                        if (e >= error / 2.0)
                        {
                            if (e < Math.Max(Precision.eps, loc.Size * 1e-6))
                            {   // goal reached, cannot get more precise
                                break;
                            }
                            ++errorcount;
                            // sometimes it makes a small detour before it converges, so do not abort immediately
                            if (errorcount > 5) // does not converge, split patch
                            {
                                if (CheckCurveInSurface(icurve, 0.0, 1.0, cube, u, uv))
                                {
                                    return CurveIntersectionMode.curveInSurface;
                                }
                                else
                                {
                                    return CurveIntersectionMode.noIntersection;
                                }
                            }
                        }
                        else
                        {
                            error = e;
                        }
                        if (!cube.uvPatch.ContainsEps(uv, -0.1))
                        {
                            ++errorcount;
                            // sometimes it makes a small detour before it converges, so do not abort immediately
                            if (errorcount > 5) // does not converge, split patch
                            {
                                if (CheckCurveInSurface(icurve, 0.0, 1.0, cube, u, uv))
                                {
                                    return CurveIntersectionMode.curveInSurface;
                                }
                                else
                                {
                                    return CurveIntersectionMode.noIntersection;
                                }
                            }
                        }
                    }
                    else
                    {   // singular matrix
                        //if (CheckCurveInSurface(icurve, 0.0, 1.0, cube, u, uv))
                        //{
                        //    return CurveIntersectionMode.curveInSurface;
                        //}
                        //else
                        //{
                        //    return CurveIntersectionMode.noIntersection;
                        //}
                        return CurveIntersectionMode.noIntersection;
                    }
                    udir = surface.UDirection(uv);
                    vdir = surface.VDirection(uv);
                }
            }
            catch
            {
                return CurveIntersectionMode.noIntersection;
            }
            if (errorcount > 0 && !cube.uvPatch.ContainsEps(uv, -0.1)) return CurveIntersectionMode.noIntersection;
            ip = new GeoPoint(loc, curvepoint);
            return CurveIntersectionMode.simpleIntersection;
        }
        private CurveIntersectionMode NewtonCurveIntersection(ICurve curve, double spar, double epar, ParEpi cube, out GeoPoint ip, out GeoPoint2D uv, out double u)
        {
            if (epar <= spar)
            {
                ip = GeoPoint.Origin;
                uv = GeoPoint2D.Origin;
                u = 0.0;
                return CurveIntersectionMode.noIntersection;
            }
            // for curves that can easily determine an exact intersection with a plane, i.e. line, circle, ellipse
            // one should not work with the tangent here but with the real curve (for a line it does not matter)
            uv = cube.uvPatch.GetCenter();
            u = (spar + epar) / 2.0;
            if (!cube.Contains(curve.PointAt(u)))
            {   // it happens that the start point of the curve (curve very long compared to the cube) is too bad
                u = curve.PositionOf(cube.GetCenter());
            }
            GeoVector udir = surface.UDirection(uv);
            GeoVector vdir = surface.VDirection(uv); // their length must be correct too!
            GeoPoint loc = surface.PointAt(uv);
            GeoVector curvedir = curve.DirectionAt(u);
            GeoPoint curvepoint = curve.PointAt(u);
            double error = curvepoint | loc;
            int errorcount = 0;
            bool firstpass = true;
#if DEBUG
            DebuggerContainer dc = new DebuggerContainer();
            try
            {
                Face fc = Face.MakeFace(surface, new CADability.Shapes.SimpleShape(cube.uvPatch));
                dc.Add(fc);
            }
            catch (Polyline2DException) { }
            ICurve trcurve = curve.Clone();
            trcurve.Trim(spar, epar);
            dc.Add(trcurve as IGeoObject);
            dc.Add(curvepoint, Color.Red, 0);
            dc.Add(loc, Color.Green, 1);
#endif
            ip = new GeoPoint(loc, curvepoint);
            while (error > 0) // Math.Max(Precision.eps, loc.Size * 1e-6))
            {
                double tan = (udir ^ vdir).Normalized * curvedir.Normalized;
                if (Math.Abs(tan) < 0.01) return CurveIntersectionMode.tangential;
                Matrix m = DenseMatrix.OfColumnArrays(udir, vdir, curvedir);
                Vector s = (Vector)m.Solve(new DenseVector(curvepoint - loc));
                if (s.IsValid())
                {
                    double du = s[0];
                    double dv = s[1];
                    double dcurve = s[2];
                    uv.x += du; // or -=
                    uv.y += dv; // or -=
                    loc = surface.PointAt(uv);
                    u -= dcurve; // or -=?
                    curvepoint = curve.PointAt(u);
                    double e = loc | curvepoint;
                    if (e >= error / 2.0 && !firstpass) // does not converge well, but this is rare
                    {
                        if (e < Math.Max(Precision.eps, loc.Size * 1e-10))
                        {   // goal reached, cannot get more precise
                            error = e;
                            break;
                        }
                        ++errorcount;
                        // sometimes it makes a small detour before it converges, so do not abort immediately
                        if (errorcount > 5) // does not converge, split patch
                            if (CheckCurveInSurface(curve, spar, epar, cube, u, uv))
                            {
                                return CurveIntersectionMode.curveInSurface;
                            }
                            else
                            {
                                return CurveIntersectionMode.noIntersection;
                            }
                    }
                    else
                    {
                        error = e;
                        firstpass = false;
                    }
                    if (!cube.uvPatch.ContainsEps(uv, -0.1) || u < spar || u > epar)
                    {
                        ++errorcount;
                        // sometimes it makes a small detour before it converges, so do not abort immediately
                        if (errorcount > 5) // does not converge, split patch
                        {
                            if (CheckCurveInSurface(curve, spar, epar, cube, u, uv))
                            {
                                return CurveIntersectionMode.curveInSurface;
                            }
                            else
                            {
                                return CurveIntersectionMode.noIntersection;
                            }
                        }
                    }
                }
                else
                {   // singular matrix
                    return CurveIntersectionMode.noIntersection;
                }
                udir = surface.UDirection(uv);
                vdir = surface.VDirection(uv);
                curvedir = curve.DirectionAt(u);
            }
            if (errorcount > 0 && (!cube.uvPatch.ContainsEps(uv, -0.1) || u < spar || u > epar)) return CurveIntersectionMode.noIntersection;
            ip = new GeoPoint(loc, curvepoint);
            return CurveIntersectionMode.simpleIntersection;
        }

        internal static bool NewtonCurveIntersection(ICurve curve, ISurface surface, BoundingRect bounds, ref GeoPoint ip, out GeoPoint2D uv, out double u)
        {
            // for curves that can easily determine an exact intersection with a plane, i.e. line, circle, ellipse
            // one should not work with the tangent here but with the real curve (for a line it does not matter)
            uv = surface.PositionOf(ip);
            u = curve.PositionOf(ip);
            GeoVector udir = surface.UDirection(uv);
            GeoVector vdir = surface.VDirection(uv); // their length must be correct too!
            GeoPoint loc = surface.PointAt(uv);
            GeoVector curvedir = curve.DirectionAt(u);
            GeoPoint curvepoint = curve.PointAt(u);
            double error = curvepoint | loc;
#if DEBUG
            {
                DebuggerContainer dc = new DebuggerContainer();
                try
                {
                    Face fc = Face.MakeFace(surface, new CADability.Shapes.SimpleShape(bounds));
                    dc.Add(fc);
                }
                catch (Polyline2DException) { }
                dc.Add(curve as IGeoObject);
            }
#endif
            ip = new GeoPoint(loc, curvepoint);
            while (error > 0)
            {
                Matrix m = DenseMatrix.OfColumnArrays(udir, vdir, curvedir);
                Vector s = (Vector)m.Solve(new DenseVector(curvepoint - loc));
                if (s.IsValid())
                {
                    double du = s[0];
                    double dv = s[1];
                    double dcurve = s[2];
                    uv.x += du; // or -=
                    uv.y += dv; // or -=
                    loc = surface.PointAt(uv);
                    u -= dcurve; // or -=?
                    curvepoint = curve.PointAt(u);
                    double e = loc | curvepoint;
                    if (e >= error / 2.0) // so that it comes to an end
                    {
                        break;
                    }
                    error = e;
                }
                else
                {   // singular matrix
                    //if (CheckCurveInSurface(curve, spar, epar, cube, u, uv))
                    //{
                    //    return CurveIntersectionMode.curveInSurface;
                    //}
                    //else
                    //{
                    //    return CurveIntersectionMode.noIntersection;
                    //}
                    break;
                }
                udir = surface.UDirection(uv);
                vdir = surface.VDirection(uv);
                curvedir = curve.DirectionAt(u);
            }
            if (error < Precision.eps)
            {
                ip = new GeoPoint(loc, curvepoint);
                return true;
            }
            return false;
        }

        private bool CheckCurveInSurface(ICurve curve, double spar, double epar, ParEpi cube, double u, GeoPoint2D uv)
        {   // the problem: determine whether the curve lies in the surface. Of course the curve can also protrude from the surface
            // and that causes a problem. Maybe work with GetNaturalBounds? How is PositionOf defined when the point
            // lies next to the surface?
            double umin, umax, vmin, vmax;
            surface.GetNaturalBounds(out umin, out umax, out vmin, out vmax);
            if (umin == double.MinValue || umax == double.MaxValue)
            {
                umin = uvbounds.Left;
                umax = uvbounds.Right;
            }
            if (vmin == double.MinValue || vmax == double.MaxValue)
            {
                vmin = uvbounds.Bottom;
                vmax = uvbounds.Top;
            }

            List<double> checkBetween = new List<double>(); // two points on the curve with which we want to check, i.e.
                                                            // that potentially lie on the surface
            GeoPoint sp = curve.PointAt(spar);
            if (IsClose(sp))
            {
                GeoPoint2D uv1 = surface.PositionOf(sp);
                if (Precision.IsEqual(surface.PointAt(uv1), sp))
                    checkBetween.Add(spar);
            }
            GeoPoint ep = curve.PointAt(epar);
            if (IsClose(ep))
            {
                GeoPoint2D uv1 = surface.PositionOf(ep);
                if (Precision.IsEqual(surface.PointAt(uv1), ep))
                    checkBetween.Add(epar);
            }
            if (checkBetween.Count < 2)
            {
                // so not the whole curve lies inside, but at most one point
                double[] par1;
                double[] par2;
                GeoPoint[] ip;
                ICurve frame = surface.FixedU(umin, vmin, vmax);
                if (!IsUSingularity(umin))
                {
                    Curves.Intersect(frame, curve, out par1, out par2, out ip);
                    if (par2.Length > 0)
                    {
                        checkBetween.AddRange(par2);
                    }
                }
                if (!IsUSingularity(umax))
                {
                    frame = surface.FixedU(umax, vmin, vmax);
                    Curves.Intersect(frame, curve, out par1, out par2, out ip);
                    if (par2.Length > 0)
                    {
                        checkBetween.AddRange(par2);
                    }
                }
                if (!IsVSingularity(vmin))
                {
                    frame = surface.FixedV(vmin, umin, umax);
                    Curves.Intersect(frame, curve, out par1, out par2, out ip);
                    if (par2.Length > 0)
                    {
                        checkBetween.AddRange(par2);
                    }
                }
                if (!IsVSingularity(umax))
                {
                    frame = surface.FixedV(vmax, umin, umax);
                    Curves.Intersect(frame, curve, out par1, out par2, out ip);
                    if (par2.Length > 0)
                    {
                        checkBetween.AddRange(par2);
                    }
                }
            }
            if (checkBetween.Count < 2) return false; // the curve does not lie inside
            checkBetween.Sort();
            for (int i = checkBetween.Count - 1; i > 0; --i)
            {
                if (checkBetween[i] - checkBetween[i - 1] < 1e-6) checkBetween.RemoveAt(i);
            }
            if (checkBetween.Count < 2) return false; // the curve does not lie inside

            double tu = checkBetween[0];
            GeoPoint p = curve.PointAt(tu);
            uv = surface.PositionOf(p);
            if (!Precision.IsPerpendicular(curve.DirectionAt(tu), surface.GetNormal(uv), false)) return false;
            tu = checkBetween[checkBetween.Count - 1];
            p = curve.PointAt(tu);
            uv = surface.PositionOf(p);
            if (!Precision.IsPerpendicular(curve.DirectionAt(tu), surface.GetNormal(uv), false)) return false;
            return true; // two points and their tangent tested
        }

        private void GetCurveIntersection(ICurve curve, double spar, double epar, ParEpi cube, List<GeoPoint> lips, List<GeoPoint2D> luvOnFace, List<double> luOnCurve)
        {   // tries to find an intersection point with the Newton method, if not, then it is split (both curve and surface)
            // and tried again on interference of tetraeder with cube
            GeoPoint ip;
            GeoPoint2D uv;
            double u;
            if (curve is IDualSurfaceCurve)
            {
                IDualSurfaceCurve dsc = (curve as IDualSurfaceCurve);
                BoundingRect ext1 = BoundingRect.InfinitBoundingRect;
                dsc.Surface1.GetNaturalBounds(out ext1.Left, out ext1.Right, out ext1.Bottom, out ext1.Top);
                BoundingRect ext2 = BoundingRect.InfinitBoundingRect;
                dsc.Surface2.GetNaturalBounds(out ext2.Left, out ext2.Right, out ext2.Bottom, out ext2.Top);
                // the ext are not used at all
                // if there are several intersection points of the surface between spar and epar, then we have a problem here:
                // the start point must be correct
                GeoPoint sp = curve.PointAt(spar);
                GeoPoint ep = curve.PointAt(epar);
                if (cube.ClipLine(ref sp, ref ep))
                {
                    ip = new GeoPoint(sp, ep); // midpoint on the clipped piece
                }
                else
                {
                    GeoPoint mp = Geometry.DropPL(cube.GetCenter(), sp, ep);
                    double lpos = Geometry.LinePar(sp, ep, mp);
                    if (lpos >= 0 && lpos <= 1)
                    {
                        double mpar = spar + lpos * (epar - spar);
                        ip = curve.PointAt(mpar);
                    }
                    else
                    {
                        ip = new GeoPoint(surface.PointAt(cube.uvPatch.GetCenter()), curve.PointAt((spar + epar) / 2.0)); // midpoint of the curve and midpoint of the uvPatch
                    }
                }
                GeoPoint2D uv1, uv2, uv3;
                if (Surfaces.NewtonIntersect(this.surface, cube.uvPatch, dsc.Surface1, ext1, dsc.Surface2, ext2, ref ip, out uv1, out uv2, out uv3))
                {
                    if (cube.Contains(ip))
                    {
                        lips.Add(ip);
                        luvOnFace.Add(uv1);
                        luOnCurve.Add(curve.PositionOf(ip));
                        return; // is there a problem if there are 2 intersection points?
                                // well: in a sense yes: a ParEpi and a tetraeder should always contain only one intersection point
                                // that is why the test was introduced here whether the cube also contains the intersection point, because if not, we may have slipped out of the cube
                                // although there was still an intersection point in it. But through the splitting that now follows we then find the correct one
                    }
                }
            }
            if ((!cube.Contains(curve.PointAt(spar)) || !cube.Contains(curve.PointAt(epar))) && !(curve is Line) && curve is IExplicitPCurve3D)
            {
                // first shorten the curve so that it lies completely inside the cube.
                GeoPoint[] loc;
                GeoVector[] dirx, diry;
                cube.GetPlanes(out loc, out dirx, out diry); // these are the 6 sides as faces with PlaneSurface and SimpleShape
                ExplicitPCurve3D excrv = (curve as IExplicitPCurve3D).GetExplicitPCurve3D();
                List<double> planeIntersectionParams = new List<double>();
                for (int i = 0; i < loc.Length; i++)
                {
                    double[] ips = excrv.GetPlaneIntersection(loc[i], dirx[i], diry[i]);
                    for (int j = 0; j < ips.Length; j++)
                    {
                        GeoPoint p = excrv.PointAt(ips[j]);
                        Matrix m = DenseMatrix.OfRowArrays(dirx[i], diry[i], dirx[i] ^ diry[i]);
                        Vector mres = (Vector)m.Transpose().Solve(new DenseVector(p - loc[i]));
                        if (mres.IsValid())
                        {
                            double x = mres[0];
                            double y = mres[1]; // mres[2, 0] must be 0
                            if (x >= 0 && x <= 1 && y >= 0 && y <= 1)
                            {
                                // entering or exiting intersection? If the sides were correctly oriented, this would be easy
                                planeIntersectionParams.Add(curve.PositionOf(p)); // curve and excrv do not have the same parameters
                            }
                        }
                    }
                }
                // which segment applies now
                if (planeIntersectionParams.Count > 0)
                {
                    planeIntersectionParams.Sort();
                    if (spar < planeIntersectionParams[0]) planeIntersectionParams.Insert(0, spar);
                    if (epar > planeIntersectionParams[planeIntersectionParams.Count - 1]) planeIntersectionParams.Add(epar);
                    for (int i = 0; i < planeIntersectionParams.Count - 1; i++)
                    {
                        GeoPoint p = curve.PointAt((planeIntersectionParams[i] + planeIntersectionParams[i + 1]) / 2.0);
                        if (cube.Contains(p))
                        {
                            if (planeIntersectionParams[i] == spar)
                            {
                                epar = planeIntersectionParams[i + 1];
                                break;
                            }
                            else if (planeIntersectionParams[i + 1] == epar)
                            {
                                spar = planeIntersectionParams[i];
                                break;
                            }
                            else
                            {
                                spar = planeIntersectionParams[i];
                                epar = planeIntersectionParams[i + 1];
                                break;
                            }
                        }
                    }
                }
            }
            switch (NewtonCurveIntersection(curve, spar, epar, cube, out ip, out uv, out u))
            {
                case CurveIntersectionMode.simpleIntersection:
                    {
                        if (cube.Contains(ip))
                        {
                            lips.Add(ip);
                            luvOnFace.Add(uv);
                            luOnCurve.Add(u);
                            return;
                        }
                    }
                    break;
                case CurveIntersectionMode.noIntersection:
                    {
                    }
                    break;
                case CurveIntersectionMode.curveInSurface:
                    // do not add any points here
                    return;
                case CurveIntersectionMode.tangential:
                    {   // bisection with spar, epar
                        GeoPoint sp = curve.PointAt(spar);
                        GeoPoint ep = curve.PointAt(epar);
                        double sd = MinDist(sp);
                        double ed = MinDist(ep);
                        if (sd == double.MaxValue || ed == double.MaxValue) break;
                        GeoPoint mp;
                        while (Math.Sign(sd) != Math.Sign(ed))
                        {
                            if (sd == 0.0)
                            {
                                lips.Add(sp);
                                luvOnFace.Add(surface.PositionOf(sp));
                                luOnCurve.Add(spar);
                                break;
                            }
                            else if (ed == 0.0)
                            {
                                lips.Add(sp);
                                luvOnFace.Add(surface.PositionOf(sp));
                                luOnCurve.Add(spar);
                                break;
                            }
                            else if (epar - spar < 1e-6)
                            {
                                mp = new GeoPoint(sp, ep);
                                GeoPoint mps = surface.PointAt(surface.PositionOf(mp));
                                if ((mp | mps) < Precision.eps)
                                {
                                    lips.Add(mp);
                                    luvOnFace.Add(surface.PositionOf(mp));
                                    luOnCurve.Add((spar + epar) / 2.0);
                                }
                                break;
                            }
                            double mpar = (spar + epar) / 2.0;
                            mp = curve.PointAt(mpar);
                            double ms = MinDist(mp);
                            if (Math.Sign(ms) != Math.Sign(sd))
                            {
                                ed = ms;
                                epar = mpar;
                                ep = mp;
                            }
                            else
                            {
                                sd = ms;
                                spar = mpar;
                                sp = mp;
                            }
                        }
                    }
                    return;
            }
            // nothing found, so split
            if (cube.Size > octtree.Extend.Size / 100)
            {   // see case CurveIntersectionMode.noIntersection of GetCurveIntersection(ISimpleCurve curve
                ParEpi[] splitted = SplitCube(cube);
                GeoPoint p1 = curve.PointAt(spar);
                GeoPoint p2 = curve.PointAt(epar);
                GeoPoint tv1, tv2, tv3, tv4, pm;
                double parm;
                TetraederHull.SplitTetraeder(curve, p1, p2, spar, epar, out pm, out parm, out tv1, out tv2, out tv3, out tv4);
                for (int i = 0; i < splitted.Length; ++i)
                {
                    if (splitted[i].Interferes(p1, pm, tv1, tv2))
                    {
                        if (splitted[i].Interferes(curve, spar, parm, p1, pm, tv1, tv2))
                        {
                            GetCurveIntersection(curve, spar, parm, splitted[i], lips, luvOnFace, luOnCurve);
                        }
                    }
                    if (splitted[i].Interferes(pm, p2, tv3, tv4))
                    {
                        if (splitted[i].Interferes(curve, parm, epar, pm, p2, tv3, tv4))
                        {
                            GetCurveIntersection(curve, parm, epar, splitted[i], lips, luvOnFace, luOnCurve);
                        }
                    }
                }
            }

        }

        private double MinDist(GeoPoint p)
        {
            GeoPoint2D[] fps = surface.PerpendicularFoot(p); // implement PerpendicularFoot here and take the cube into account
            double mindist = double.MaxValue;
            GeoPoint2D bestUv = GeoPoint2D.Origin;
            GeoPoint bestPoint = GeoPoint.Origin;
            for (int i = 0; i < fps.Length; i++)
            {
                GeoPoint p0 = surface.PointAt(fps[i]);
                double d = p0 | p;
                if (d < mindist)
                {
                    mindist = d;
                    bestUv = fps[i];
                    bestPoint = p0;
                }
            }
            if (mindist < double.MaxValue)
            {
                GeoVector nor = surface.GetNormal(bestUv);
                if (((p - bestPoint) * nor) > 0)
                {
                    return mindist;
                }
                else
                {
                    return -mindist;
                }
            }
            return double.MaxValue; // should not happen
        }

        private bool PositionOfWithFixedCurves(GeoPoint p3d, ParEpi found, out GeoPoint2D res)
        {
            res = found.uvPatch.GetCenter();
            GeoVector dirx = surface.UDirection(res);
            GeoVector diry = surface.VDirection(res);
            GeoVector dirz = dirx ^ diry; // the normal
            GeoPoint loc = surface.PointAt(res);
            BoundingRect natbound = new BoundingRect();
            surface.GetNaturalBounds(out natbound.Left, out natbound.Right, out natbound.Bottom, out natbound.Top);
            BoundingRect natbound2 = natbound;
            natbound2.Inflate(natbound.Width, natbound.Height);
            // can also become infinite (double.MinValue/MaxValue), but that does not matter...
            double mindist = Geometry.DistPL(p3d, loc, dirz);
            int missed = 0;
            bool acceptDiverge = true;
            bool fixedu = true;
            while (mindist > Precision.eps) // was *100 (18.7.14)
            {
                if (fixedu)
                {
                    double u = (res.x - natbound.Left) / natbound.Width;
                    ICurve crv = surface.FixedU(u, natbound.Bottom, natbound.Top);
                    double v = crv.PositionOf(p3d);
                    res.y = crv.PositionToParameter(v);
                    fixedu = false;
                }
                else
                {
                    double v = (res.y - natbound.Bottom) / natbound.Height;
                    ICurve crv = surface.FixedV(v, natbound.Left, natbound.Right);
                    double u = crv.PositionOf(p3d);
                    res.x = crv.PositionToParameter(u);
                    fixedu = true;
                }
                if (!found.uvPatch.ContainsEps(res, -0.001))
                {
                    ++missed;
                    // experimentally let it run outwards as well
                    //if (missed > 2) return false;
                    if (!natbound2.Contains(res)) return false;
                    if (!natbound.Contains(res)) // reintroduced, because for some NURBS endless loop
                    {   // if completely outside, then set to the boundary
                        // fine, as long as it converges
                        SurfaceHelper.AdjustPeriodic(surface, natbound, ref res);
                        if (res.x < natbound.Left) res.x = natbound.Left;
                        if (res.x > natbound.Right) res.x = natbound.Right;
                        if (res.y < natbound.Bottom) res.y = natbound.Bottom;
                        if (res.y > natbound.Top) res.y = natbound.Top;
                    }
                }
                else
                {
                    missed = 0; // this allows it to jump back and forth
                }
                dirx = surface.UDirection(res);
                diry = surface.VDirection(res);
                loc = surface.PointAt(res);
                dirz = dirx ^ diry; // the normal
                double d = Geometry.DistPL(p3d, loc, dirz);
                if (d > mindist * 0.9)
                {
                    if (!acceptDiverge) return false; // does not converge or poorly, "*0.9" added (18.7.14)
                    else
                    {
                        acceptDiverge = false; // diverging once is ok
                    }
                }
                mindist = d;
            }
            if (double.IsNaN(mindist)) return false;
            if (missed > 2)
            {
                BoundingRect brcopy = found.uvPatch;
                brcopy.Inflate(brcopy.Width / 100, brcopy.Height / 100);
                bool ok = brcopy.Contains(res);
                return ok;
            }
            return true;

        }

        /// <summary>
        /// Newton approximation of footpoint of p3d on this path of the surface.
        /// </summary>
        /// <param name="p3d"></param>
        /// <param name="found"></param>
        /// <param name="res"></param>
        /// <returns></returns>
        private bool PositionOf(GeoPoint p3d, ParEpi found, out GeoPoint2D res, out double mindist)
        {
#if DEBUGx
            // bool dbg = PositionOfWithFixedCurves(p3d, found, out res); 
            res = found.PositionOf(p3d, surface); // this is usually a good guess by the ParEpi
            if (!res.IsValid) res = found.uvPatch.GetCenter();
            GeoPoint2D start = res;
            var at = PerformanceTimer.AllTimers;
            using (new PerformanceTick("LM"))
            {
                for (int i = 0; i < 100; i++)
                {
                    res = start;
                    BoxedSurfaceExtension.PositionOfLM(surface, p3d, ref res, out double mdmn);
                }
            }
            using (new PerformanceTick("MN"))
            {
                for (int i = 0; i < 100; i++)
                {
                    res = start;
                    BoxedSurfaceExtension.PositionOfMN(surface, p3d, ref res, out double mdmn);
                }
            }
#endif
            // starting from the u/v center of found, search the foot point with the tangent method
            // if the patch is left, then exception

            res = found.PositionOf(p3d, surface); // this is usually a good guess by the ParEpi
            if (!res.IsValid) res = found.uvPatch.GetCenter();
            GeoVector dirx = surface.UDirection(res);
            GeoVector diry = surface.VDirection(res);
            GeoVector dirz = dirx ^ diry; // the normal
            GeoPoint loc = surface.PointAt(res);
            BoundingRect natbound = new BoundingRect();
            surface.GetNaturalBounds(out natbound.Left, out natbound.Right, out natbound.Bottom, out natbound.Top);
            BoundingRect natbound2 = natbound;
            natbound2.Inflate(natbound.Width, natbound.Height); // may also be infinite
            mindist = Geometry.DistPL(p3d, loc, dirz);
            int missed = 0;
            bool acceptDiverge = true;
            while (mindist > Math.Min(Precision.eps, found.Size * 1e-4)) // was *100 (18.7.14)
            {
                if (dirx.IsNullVector() || diry.IsNullVector() || dirz.IsNullVector()) return false;
                Matrix m = DenseMatrix.OfRowArrays(dirx, diry, dirz);
                Vector mres = (Vector)m.Transpose().Solve(new DenseVector(p3d - loc));
                if (!mres.IsValid()) return false;
                res.x += mres[0];
                res.y += mres[1];
                if (!found.uvPatch.ContainsEps(res, -0.05))
                {
                    ++missed;
                    // experimentally let it run outwards as well
                    //if (missed > 2) return false;
                    if (!natbound2.Contains(res) && missed > 1) return false; // too far outside
                                                                              // if the 2d point is outside the natural bounds we stopped the whole process, but I don't see a reason why, as long as it converges
                                                                              //if (!natbound.Contains(res)) // reintroduced, because for some NURBS endless loop
                                                                              //{   // if completely outside, then set to the boundary
                                                                              //    // fine, as long as it converges
                                                                              //    SurfaceHelper.AdjustPeriodic(surface, natbound, ref res);
                                                                              //    if (res.x < natbound.Left) res.x = natbound.Left;
                                                                              //    if (res.x > natbound.Right) res.x = natbound.Right;
                                                                              //    if (res.y < natbound.Bottom) res.y = natbound.Bottom;
                                                                              //    if (res.y > natbound.Top) res.y = natbound.Top;
                                                                              //}
                }
                else
                {
                    missed = 0; // this allows it to jump back and forth
                }
                dirx = surface.UDirection(res);
                diry = surface.VDirection(res);
                loc = surface.PointAt(res);
                dirz = dirx ^ diry; // the normal
                double d = Geometry.DistPL(p3d, loc, dirz);
                if (d > mindist * 0.9)
                {
                    if (!acceptDiverge)
                    {
                        if (mindist < Precision.eps) break;
                        if (Math.Abs(mindist - d) < Precision.eps * 0.1) break; // doesn't change any more
                        return false; // does not converge or poorly, "*0.9" added (18.7.14)
                    }
                    else
                    {
                        acceptDiverge = false; // diverging once is ok
                    }
                }
                mindist = d;
            }
            if (double.IsNaN(mindist)) return false;

            if (missed > 2)
            {   // need this case when an almost closed nurbs surface finds a wrong point on the other side of the domain (like in MO21775-001-00.stp)
                BoundingRect brcopy = found.uvPatch;
                brcopy.Inflate(brcopy.Width / 100, brcopy.Height / 100);
                bool ok = brcopy.Contains(res);
                return ok;
            }
            return true;
        }
        private void SplitCubes(ParEpi[] cubes)
        {
            for (int i = 0; i < cubes.Length; i++)
            {
                SplitCube(cubes[i]);
            }
        }
        private ParEpi[] SplitCube(ParEpi toSplit)
        {   // here the object itself is refined
            lock (this)
            {
                octtree.RemoveObject(toSplit);
                quadtree.RemoveObject(toSplit);
                ParEpi[] subCubes = new ParEpi[4];
                subCubes[0] = new ParEpi();
                subCubes[1] = new ParEpi();
                subCubes[2] = new ParEpi();
                subCubes[3] = new ParEpi();
                // when splitting, strict care must be taken that the boundaries of adjacent patches
                // are identical, i.e. do not differ due to rounding errors
                double hcenter = (toSplit.uvPatch.Left + toSplit.uvPatch.Right) / 2.0;
                double vcenter = (toSplit.uvPatch.Bottom + toSplit.uvPatch.Top) / 2.0;
                subCubes[0].uvPatch = new BoundingRect(toSplit.uvPatch.Left, toSplit.uvPatch.Bottom, hcenter, vcenter);
                subCubes[1].uvPatch = new BoundingRect(toSplit.uvPatch.Left, vcenter, hcenter, toSplit.uvPatch.Top);
                subCubes[2].uvPatch = new BoundingRect(hcenter, toSplit.uvPatch.Bottom, toSplit.uvPatch.Right, vcenter);
                subCubes[3].uvPatch = new BoundingRect(hcenter, vcenter, toSplit.uvPatch.Right, toSplit.uvPatch.Top);
                for (int i = 0; i < 4; ++i)
                {
                    subCubes[i].pll = surface.PointAt(subCubes[i].uvPatch.GetLowerLeft());
                    subCubes[i].plr = surface.PointAt(subCubes[i].uvPatch.GetLowerRight());
                    subCubes[i].pul = surface.PointAt(subCubes[i].uvPatch.GetUpperLeft());
                    subCubes[i].pur = surface.PointAt(subCubes[i].uvPatch.GetUpperRight());
                    subCubes[i].nll = surface.GetNormal(subCubes[i].uvPatch.GetLowerLeft());
                    subCubes[i].nlr = surface.GetNormal(subCubes[i].uvPatch.GetLowerRight());
                    subCubes[i].nul = surface.GetNormal(subCubes[i].uvPatch.GetUpperLeft());
                    subCubes[i].nur = surface.GetNormal(subCubes[i].uvPatch.GetUpperRight());
                    subCubes[i].nll.NormIfNotNull();
                    subCubes[i].nlr.NormIfNotNull();
                    subCubes[i].nul.NormIfNotNull();
                    subCubes[i].nur.NormIfNotNull();
                    calcParallelEpiped(subCubes[i]);
                    //subCubes[i].boundingCube = surface.GetPatchExtent(new BoundingRect(subCubes[i].uvPatch.Left, subCubes[i].uvPatch.Bottom, subCubes[i].uvPatch.Right, subCubes[i].uvPatch.Top));
                    octtree.AddObject(subCubes[i]);
                    quadtree.AddObject(subCubes[i]);
                }
                return subCubes;
            }
        }
        private ParEpi[] SubCubes(ParEpi toSplit)
        {   // here only sub-cubes are returned, but no refinement is done
            ParEpi[] subCubes = new ParEpi[4];
            subCubes[0] = new ParEpi();
            subCubes[1] = new ParEpi();
            subCubes[2] = new ParEpi();
            subCubes[3] = new ParEpi();
            double hcenter = (toSplit.uvPatch.Left + toSplit.uvPatch.Right) / 2.0;
            double vcenter = (toSplit.uvPatch.Bottom + toSplit.uvPatch.Top) / 2.0;
            subCubes[0].uvPatch = new BoundingRect(toSplit.uvPatch.Left, toSplit.uvPatch.Bottom, hcenter, vcenter);
            subCubes[1].uvPatch = new BoundingRect(toSplit.uvPatch.Left, vcenter, hcenter, toSplit.uvPatch.Top);
            subCubes[2].uvPatch = new BoundingRect(hcenter, toSplit.uvPatch.Bottom, toSplit.uvPatch.Right, vcenter);
            subCubes[3].uvPatch = new BoundingRect(hcenter, vcenter, toSplit.uvPatch.Right, toSplit.uvPatch.Top);
            for (int i = 0; i < 4; ++i)
            {
                subCubes[i].pll = surface.PointAt(subCubes[i].uvPatch.GetLowerLeft());
                subCubes[i].plr = surface.PointAt(subCubes[i].uvPatch.GetLowerRight());
                subCubes[i].pul = surface.PointAt(subCubes[i].uvPatch.GetUpperLeft());
                subCubes[i].pur = surface.PointAt(subCubes[i].uvPatch.GetUpperRight());
                subCubes[i].nll = surface.GetNormal(subCubes[i].uvPatch.GetLowerLeft());
                subCubes[i].nlr = surface.GetNormal(subCubes[i].uvPatch.GetLowerRight());
                subCubes[i].nul = surface.GetNormal(subCubes[i].uvPatch.GetUpperLeft());
                subCubes[i].nur = surface.GetNormal(subCubes[i].uvPatch.GetUpperRight());
                subCubes[i].nll.NormIfNotNull();
                subCubes[i].nlr.NormIfNotNull();
                subCubes[i].nul.NormIfNotNull();
                subCubes[i].nur.NormIfNotNull();
                calcParallelEpiped(subCubes[i]);
                //subCubes[i].boundingCube = surface.GetPatchExtent(new BoundingRect(subCubes[i].uvPatch.Left, subCubes[i].uvPatch.Bottom, subCubes[i].uvPatch.Right, subCubes[i].uvPatch.Top));
            }
            return subCubes;
        }
#if DEBUG
        internal DebuggerContainer Debug
        {
            get
            {
                DebuggerContainer dc = new DebuggerContainer();
                //double[] usteps, vsteps;
                //double umin, umax, vmin, vmax;
                //surface.GetNaturalBounds(out umin, out umax, out vmin, out vmax);
                //surface.GetSafeParameterSteps(umin, umax, vmin, vmax, out usteps, out vsteps);
                //ParEpi[] all = octtree.GetObjectsFromBox(surface.GetPatchExtent(new BoundingRect(usteps[0], vsteps[0], usteps[usteps.Length - 1], vsteps[vsteps.Length - 1])));
                ParEpi[] all = octtree.GetAllObjects();
                Layer transparent = new Layer("Transparent");
                transparent.Transparency = 100;
                Layer solid = new Layer("Solid");
                ColorDef cd = new ColorDef("Green", Color.Green);
                ColorDef cdr = new ColorDef("Red", Color.Red);
                for (int i = 0; i < all.Length; ++i)
                {
                    try
                    {
                        Solid sld = all[i].GetSolid();
                        sld.ColorDef = cd;
                        sld.Layer = transparent;
                        dc.Add(sld, all[i].id);
                        //dc.Add((all[i] as IDebuggerVisualizer).GetList()); // with quadrilateral and normals
                    }
                    catch (ApplicationException) { }
                }
                double umin = uvbounds.Left;
                double umax = uvbounds.Right;
                double vmin = uvbounds.Bottom;
                double vmax = uvbounds.Top;
                if (surface is IRestrictedDomain restrictedDomain)
                {
                    int n = 10;
                    for (int i = 1; i < n; i++)
                    {
                        Line2D l2d = new Line2D(new GeoPoint2D(umin + i * (umax - umin) / n, vmin), new GeoPoint2D(umin + i * (umax - umin) / n, vmax));
                        double[] ips = restrictedDomain.Clip(l2d);
                        for (int j = 0; j < ips.Length; j += 2)
                        {
                            dc.Add(surface.Make3dCurve(l2d.Trim(ips[j], ips[j + 1])) as IGeoObject);
                        }
                        l2d = new Line2D(new GeoPoint2D(umin, vmin + i * (vmax - vmin) / n), new GeoPoint2D(umax, vmin + i * (vmax - vmin) / n));
                        ips = restrictedDomain.Clip(l2d);
                        for (int j = 0; j < ips.Length; j += 2)
                        {
                            dc.Add(surface.Make3dCurve(l2d.Trim(ips[j], ips[j + 1])) as IGeoObject);
                        }
                    }
                }
                else
                {
                    int n = 50;
                    for (int i = 0; i <= n; i++)
                    {   // along the diagonal
                        GeoPoint[] pu = new GeoPoint[n + 1];
                        GeoPoint[] pv = new GeoPoint[n + 1];
                        for (int j = 0; j <= n; j++)
                        {
                            pu[j] = surface.PointAt(new GeoPoint2D(umin + j * (umax - umin) / n, vmin + i * (vmax - vmin) / n));
                            pv[j] = surface.PointAt(new GeoPoint2D(umin + i * (umax - umin) / n, vmin + j * (vmax - vmin) / n));
                        }
                        try
                        {
                            Polyline plu = Polyline.Construct();
                            plu.SetPoints(pu, false);
                            plu.ColorDef = cdr;
                            plu.Layer = solid;
                            dc.Add(plu);
                        }
                        catch (PolylineException)
                        {
                            Point pntu = Point.Construct();
                            pntu.Location = pu[0];
                            pntu.Symbol = PointSymbol.Cross;
                            pntu.ColorDef = cdr;
                            pntu.Layer = solid;
                            dc.Add(pntu);
                        }
                        try
                        {
                            Polyline plv = Polyline.Construct();
                            plv.SetPoints(pv, false);
                            plv.ColorDef = cdr;
                            plv.Layer = solid;
                            dc.Add(plv);
                        }
                        catch (PolylineException)
                        {
                            Point pntv = Point.Construct();
                            pntv.Location = pv[0];
                            pntv.Symbol = PointSymbol.Cross;
                            pntv.ColorDef = cdr;
                            pntv.Layer = solid;
                            dc.Add(pntv);
                        }
                    }
                }
                return dc;
            }
        }
        internal int DebugCount
        {
            get
            {
                ParEpi[] all = octtree.GetAllObjects();
                return all.Length;
            }
        }
#endif

        internal bool IsCloseTo(ParallelepipedHull bs2)
        {   // determines whether there are overlapping parallelepipeds
            ParEpi[] all2 = bs2.octtree.GetAllObjects();
            for (int i = 0; i < all2.Length; ++i)
            {
                ParEpi[] all1 = octtree.GetObjectsCloseTo(all2[i]);
                for (int j = 0; j < all1.Length; ++j)
                {
                    if (all1[j].Interferes(all2[i])) return true;
                }
            }
            return false;
        }
        internal bool IsCloseTo(GeoPoint p)
        {   // determines whether there are overlapping parallelepipeds
            ParEpi[] all1 = octtree.GetObjectsFromPoint(p);
            for (int j = 0; j < all1.Length; ++j)
            {
                if (all1[j].Contains(p))
                {
                    return true;
                }
            }
            return false;
        }
        private struct GeoPoint3d2d2d
        {
            public GeoPoint p3d;
            public GeoPoint2D uv1, uv2;
            public GeoPoint3d2d2d(GeoPoint p3d, GeoPoint2D uv1, GeoPoint2D uv2)
            {
                this.p3d = p3d;
                this.uv1 = uv1;
                this.uv2 = uv2;
            }

            internal void AdjustPeriodic(ParallelepipedHull bs1, ParallelepipedHull bs2, BoundingRect bounds1, BoundingRect bounds2)
            {
                if (bs1.surface.IsUPeriodic)
                {
                    while (bounds1.Right - uv1.x > bs1.surface.UPeriod) uv1.x += bs1.surface.UPeriod;
                    while (uv1.x - bounds1.Left > bs1.surface.UPeriod) uv1.x -= bs1.surface.UPeriod;
                }
                if (bs1.surface.IsVPeriodic)
                {
                    while (bounds1.Top - uv1.y > bs1.surface.VPeriod) uv1.y += bs1.surface.VPeriod;
                    while (uv1.y - bounds1.Bottom > bs1.surface.VPeriod) uv1.y -= bs1.surface.VPeriod;
                }
                if (bs2.surface.IsUPeriodic)
                {
                    while (bounds2.Right - uv2.x > bs2.surface.UPeriod) uv2.x += bs2.surface.UPeriod;
                    while (uv2.x - bounds2.Left > bs2.surface.UPeriod) uv2.x -= bs2.surface.UPeriod;
                }
                if (bs2.surface.IsVPeriodic)
                {
                    while (bounds2.Top - uv2.y > bs2.surface.VPeriod) uv2.y += bs2.surface.VPeriod;
                    while (uv2.y - bounds2.Bottom > bs2.surface.VPeriod) uv2.y -= bs2.surface.VPeriod;
                }
            }
        }
        private class LinkedIntersectionPoint : IOctTreeInsertable
        {
            public GeoPoint ip;
            public GeoPoint2D uv1, uv2;
            public GeoVector cross; // cross product of the normal vectors at this point: direction of the curve, NullVector if tangential
            public GeoVector2D dir1, dir2; // directions in the respective uv system
            public LinkedIntersectionPoint next1, next2, prev1, prev2, next, prev; // linked lists in uv1, uv2, mixed together
            public int ui1, vi1, ui2, vi2; // index of the fixedu/v line, for linking
            public Tuple<int, int> enterOnSurf1, enterOnSurf2; // remember the field number where this point is an entry point
            [Flags]
            public enum emode { in1 = 1, in2 = 2, onVertex1 = 4, onVertex2 = 8, seed = 16, bottom = 32, top = 64, left = 128, right = 256, isCyclicalStart = 512 }
            public emode mode;
#if DEBUG
            public static int idCounter = 0;
            public int id;
            public LinkedIntersectionPoint()
            {
                id = idCounter++;
            }
            public override int GetHashCode()
            {
                return id;
            }
#endif

            internal static List<LinkedIntersectionPoint> CreateIntersections(ISurface surface1, ISurface surface2, BoundingRect bounds1, BoundingRect bounds2, bool on1, double par, double pmin, double pmax, bool fixedu)
            {
                List<LinkedIntersectionPoint> res = new List<LinkedIntersectionPoint>();
                ICurve crv;
                if (on1)
                {
                    if (fixedu) crv = surface1.FixedU(par, pmin, pmax);
                    else crv = surface1.FixedV(par, pmin, pmax);
                }
                else
                {
                    if (fixedu) crv = surface2.FixedU(par, pmin, pmax);
                    else crv = surface2.FixedV(par, pmin, pmax);
                }
                if (crv == null)
                {
                    return res;
                }
                GeoPoint[] ips;
                GeoPoint2D[] uvOnFaces;
                double[] uOnCurve3Ds;
                if (on1) surface2.Intersect(crv, bounds2, out ips, out uvOnFaces, out uOnCurve3Ds);
                else surface1.Intersect(crv, bounds1, out ips, out uvOnFaces, out uOnCurve3Ds);
                for (int i = 0; i < ips.Length; i++)
                {
                    LinkedIntersectionPoint lip = new LinkedIntersectionPoint();
                    lip.ip = ips[i];
                    if (on1)
                    {
                        lip.uv2 = uvOnFaces[i];
                        lip.mode = emode.in1;
                        lip.uv1 = surface1.PositionOf(ips[i]);
                        if (fixedu) lip.uv1.x = par;
                        else lip.uv1.y = par;
                    }
                    else
                    {
                        lip.uv1 = uvOnFaces[i];
                        lip.mode = emode.in2;
                        lip.uv2 = surface2.PositionOf(ips[i]);
                        if (fixedu) lip.uv2.x = par; // snap
                        else lip.uv2.y = par;
                    }
                    lip.AdjustPeriodic(surface1, surface2, bounds1, bounds2);
                    if (!bounds1.ContainsEps(lip.uv1, -1e-6) || !bounds2.ContainsEps(lip.uv2, -1e-6)) continue;
                    lip.ui1 = lip.vi1 = lip.ui2 = lip.vi2 = -1;
                    GeoVector diru1, dirv1, diru2, dirv2;
                    GeoPoint ip;
                    surface1.DerivativeAt(lip.uv1, out ip, out diru1, out dirv1);
                    surface2.DerivativeAt(lip.uv2, out ip, out diru2, out dirv2);
                    GeoVector n1 = (diru1 ^ dirv1).Normalized;
                    GeoVector n2 = (diru2 ^ dirv2).Normalized;
                    lip.cross = n1 ^ n2;
                    if (lip.cross.Length < 1e-4)
                    {   // approximately tangential
                        lip.dir1 = GeoVector2D.NullVector;
                        lip.dir2 = GeoVector2D.NullVector;
                    }
                    else
                    {
                        Matrix m = DenseMatrix.OfColumnArrays(diru1, dirv1, n1);
                        Vector s = (Vector)m.Solve(new DenseVector(lip.cross));
                        if (s.IsValid())
                        {
                            lip.dir1 = new GeoVector2D(s[0], s[1]).Normalized;
                        }
                        m = DenseMatrix.OfColumnArrays(diru2, dirv2, n2);
                        s = (Vector)m.Solve(new DenseVector(lip.cross));
                        if (s.IsValid())
                        {
                            lip.dir2 = new GeoVector2D(s[0], s[1]).Normalized;
                        }
                    }
                    res.Add(lip);
                }
                return res;
            }

            private void AdjustPeriodic(ISurface surface1, ISurface surface2, BoundingRect bounds1, BoundingRect bounds2)
            {
                SurfaceHelper.AdjustPeriodic(surface1, bounds1, ref uv1);
                SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref uv2);
            }

            internal static LinkedIntersectionPoint CreateFromSeed(ISurface surface1, ISurface surface2, BoundingRect bounds1, BoundingRect bounds2, GeoPoint ip)
            {
                LinkedIntersectionPoint lip = new LinkedIntersectionPoint();
                lip.ip = ip;

                lip.uv2 = surface2.PositionOf(ip);
                lip.mode = emode.seed;
                lip.uv1 = surface1.PositionOf(ip);
                lip.AdjustPeriodic(surface1, surface2, bounds1, bounds2);
                lip.ui1 = lip.vi1 = lip.ui2 = lip.vi2 = -1;
                GeoVector diru1, dirv1, diru2, dirv2;
                surface1.DerivativeAt(lip.uv1, out ip, out diru1, out dirv1);
                surface2.DerivativeAt(lip.uv2, out ip, out diru2, out dirv2);
                GeoVector n1 = (diru1 ^ dirv1).Normalized;
                GeoVector n2 = (diru2 ^ dirv2).Normalized;
                lip.cross = n1 ^ n2;
                if (lip.cross.Length < 1e-4)
                {   // approximately tangential
                    lip.dir1 = GeoVector2D.NullVector;
                    lip.dir2 = GeoVector2D.NullVector;
                }
                else
                {
                    Matrix m = DenseMatrix.OfColumnArrays(diru1, dirv1, n1);
                    Vector s = (Vector)m.Solve(new DenseVector(lip.cross));
                    if (s.IsValid())
                    {
                        lip.dir1 = new GeoVector2D(s[0], s[1]);
                    }
                    m = DenseMatrix.OfColumnArrays(diru2, dirv2, n2);
                    s = (Vector)m.Solve(new DenseVector(lip.cross));
                    if (s.IsValid())
                    {
                        lip.dir2 = new GeoVector2D(s[0], s[1]);
                    }
                }
                return lip;
            }

            BoundingBox IOctTreeInsertable.GetExtent(double precision)
            {
                return new CADability.BoundingBox(ip);
            }

            bool IOctTreeInsertable.HitTest(ref BoundingBox cube, double precision)
            {
                return cube.Contains(ip);
            }

            bool IOctTreeInsertable.HitTest(Projection projection, BoundingRect rect, bool onlyInside)
            {
                throw new NotImplementedException();
            }

            bool IOctTreeInsertable.HitTest(Projection.PickArea area, bool onlyInside)
            {
                throw new NotImplementedException();
            }

            double IOctTreeInsertable.Position(GeoPoint fromHere, GeoVector direction, double precision)
            {
                throw new NotImplementedException();
            }

            internal void MergeWith(LinkedIntersectionPoint other)
            {
                if (other.mode.HasFlag(emode.in1))
                {
                    if (other.ui1 != -1) ui1 = other.ui1;
                    if (other.vi1 != -1) vi1 = other.vi1;
                    uv1 = other.uv1;
                    mode |= emode.in1;
                }
                if (other.mode.HasFlag(emode.in2))
                {
                    if (other.ui2 != -1) ui2 = other.ui2;
                    if (other.vi2 != -1) vi2 = other.vi2;
                    uv2 = other.uv2;
                    mode |= emode.in2;
                }
                if (other.mode.HasFlag(emode.seed))
                {   // seed overwrites the data, should be more precise!
                    uv1 = other.uv1;
                    uv2 = other.uv2;
                    ip = other.ip;
                    mode |= emode.seed;
                }

            }

            internal void AddToGrid(Dictionary<Tuple<int, int>, List<LinkedIntersectionPoint>> surf1ips, Dictionary<Tuple<int, int>, List<LinkedIntersectionPoint>> surf2ips, List<double> uknots1, List<double> vknots1, List<double> uknots2, List<double> vknots2)
            {
                int uind = ui1;
                int vind = vi1;
                if (uind < 0) uind = findInd(uknots1, uv1.x);
                if (vind < 0) vind = findInd(vknots1, uv1.y);
                List<LinkedIntersectionPoint> llip;
                if (!surf1ips.TryGetValue(new Tuple<int, int>(uind, vind), out llip))
                {
                    llip = new List<LinkedIntersectionPoint>();
                    surf1ips[new Tuple<int, int>(uind, vind)] = llip;
                }
                llip.Add(this);
                if (ui1 >= 0 && vi1 >= 0) // on crossing
                {
                    if (!surf1ips.TryGetValue(new Tuple<int, int>(uind + 1, vind + 1), out llip))
                    {
                        llip = new List<LinkedIntersectionPoint>();
                        surf1ips[new Tuple<int, int>(uind + 1, vind + 1)] = llip;
                    }
                    llip.Add(this);
                }
                if (ui1 >= 0) // on edge
                {
                    if (!surf1ips.TryGetValue(new Tuple<int, int>(uind + 1, vind), out llip))
                    {
                        llip = new List<LinkedIntersectionPoint>();
                        surf1ips[new Tuple<int, int>(uind + 1, vind)] = llip;
                    }
                    llip.Add(this);
                }
                if (vi1 >= 0) // on edge
                {
                    if (!surf1ips.TryGetValue(new Tuple<int, int>(uind, vind + 1), out llip))
                    {
                        llip = new List<LinkedIntersectionPoint>();
                        surf1ips[new Tuple<int, int>(uind, vind + 1)] = llip;
                    }
                    llip.Add(this);
                }
                // likewise with 2
                uind = ui2;
                vind = vi2;
                if (uind < 0) uind = findInd(uknots2, uv2.x);
                if (vind < 0) vind = findInd(vknots2, uv2.y);
                if (!surf2ips.TryGetValue(new Tuple<int, int>(uind, vind), out llip))
                {
                    llip = new List<LinkedIntersectionPoint>();
                    surf2ips[new Tuple<int, int>(uind, vind)] = llip;
                }
                llip.Add(this);
                if (ui2 >= 0 && vi2 >= 0) // on crossing
                {
                    if (!surf2ips.TryGetValue(new Tuple<int, int>(uind + 1, vind + 1), out llip))
                    {
                        llip = new List<LinkedIntersectionPoint>();
                        surf2ips[new Tuple<int, int>(uind + 1, vind + 1)] = llip;
                    }
                    llip.Add(this);
                }
                if (ui2 >= 0) // on edge
                {
                    if (!surf2ips.TryGetValue(new Tuple<int, int>(uind + 1, vind), out llip))
                    {
                        llip = new List<LinkedIntersectionPoint>();
                        surf2ips[new Tuple<int, int>(uind + 1, vind)] = llip;
                    }
                    llip.Add(this);
                }
                if (vi2 >= 0) // on edge
                {
                    if (!surf2ips.TryGetValue(new Tuple<int, int>(uind, vind + 1), out llip))
                    {
                        llip = new List<LinkedIntersectionPoint>();
                        surf2ips[new Tuple<int, int>(uind, vind + 1)] = llip;
                    }
                    llip.Add(this);
                }
            }

            private int findInd(List<double> knots, double val)
            {   // linear search, not efficient
                for (int i = 0; i < knots.Count - 1; i++)
                {
                    if (val >= knots[i] && val <= knots[i + 1]) return i + 1;
                }
                if (val < knots[0]) return 0;
                if (val > knots[knots.Count - 1]) return knots.Count;
                return -1; // does not occur
            }
        }
        internal ICurve[] Intersect(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, List<GeoPoint> seeds, List<Tuple<double, double, double, double>> additionalSearchPositions = null)
        {
            List<ICurve> res = new List<GeoObject.ICurve>();
            // the u and v values used by the possibly intersecting ParEpi
            // this results in a grid for each surface
            SortedSet<double> uVal1 = new SortedSet<double>();
            SortedSet<double> vVal1 = new SortedSet<double>();
            SortedSet<double> uVal2 = new SortedSet<double>();
            SortedSet<double> vVal2 = new SortedSet<double>();
            if ((other as ISurfaceImpl).usedArea.IsEmpty()) (other as ISurfaceImpl).usedArea = otherBounds;
            ParallelepipedHull otherBS = (other as ISurfaceImpl).ParallelepipedHull;
            foreach (ParEpi pe in octtree.GetAllObjects())
            {
                if (pe.uvPatch.Interferes(ref thisBounds))
                {
                    foreach (ParEpi otherPe in otherBS.octtree.GetObjectsCloseTo(pe))
                    {
                        if (otherPe.uvPatch.Interferes(ref otherBounds))
                        {
                            if (pe.Interferes(otherPe))
                            {
                                uVal1.Add(pe.uvPatch.Left);
                                uVal1.Add(pe.uvPatch.Right);
                                vVal1.Add(pe.uvPatch.Bottom);
                                vVal1.Add(pe.uvPatch.Top);
                                uVal2.Add(otherPe.uvPatch.Left);
                                uVal2.Add(otherPe.uvPatch.Right);
                                vVal2.Add(otherPe.uvPatch.Bottom);
                                vVal2.Add(otherPe.uvPatch.Top);
                            }
                        }
                    }
                }
            }
            if (seeds == null && additionalSearchPositions != null)
            {   // this is a special case: typically this is called to find intersections of two faces, where the edge/face intersections already have been calculated and are provided in the seeds.
                // But it is possible that two faces have closed loops of intersection curves, which do not cross their bounds (edges). (E.g. two spheres intersect without edge intersection)
                // If so, there must be some u or v parameters for additional checks provided. These "additionalSearchPositions" may have u or v values on this or the other surface.
                for (int i = 0; i < additionalSearchPositions.Count; i++)
                {
                    if (!double.IsNaN(additionalSearchPositions[i].Item1)) addOrAdjust(uVal1, additionalSearchPositions[i].Item1);
                    if (!double.IsNaN(additionalSearchPositions[i].Item2)) addOrAdjust(vVal1, additionalSearchPositions[i].Item2);
                    if (!double.IsNaN(additionalSearchPositions[i].Item3)) addOrAdjust(uVal2, additionalSearchPositions[i].Item3);
                    if (!double.IsNaN(additionalSearchPositions[i].Item4)) addOrAdjust(vVal2, additionalSearchPositions[i].Item4);
                }
            }
            if (uVal1.Count == 0) return res.ToArray(); // nothing, no overlap
            bool splitted = true; // the uVal1 etc. lists can be extended if too many points lie in one field. Then we have to start over from the beginning
            int splitcount = 0;
            while (splitted)
            {
                splitted = false;
                if (++splitcount > 10) return res.ToArray(); // something went wrong with splitting, no result
#if DEBUG
                LinkedIntersectionPoint.idCounter = 0;
#endif
                double u1prec = smallesDiff(uVal1) * 1e-3 * (splitcount + 1); // precision for individual parameters, there should be no intersection point here
                double v1prec = smallesDiff(vVal1) * 1e-3 * (splitcount + 1); // *(splitcount+1): to avoid endless ping pong
                double u2prec = smallesDiff(uVal2) * 1e-3 * (splitcount + 1);
                double v2prec = smallesDiff(vVal2) * 1e-3 * (splitcount + 1);
#if DEBUG
                DebuggerContainer dcuv1 = new DebuggerContainer();
                DebuggerContainer dcuv2 = new DebuggerContainer();
                DebuggerContainer dc3d1 = new DebuggerContainer();
                DebuggerContainer dc3d2 = new DebuggerContainer();
                foreach (double u in uVal1)
                {
                    dcuv1.Add(new Line2D(new GeoPoint2D(u, vVal1.Min), new GeoPoint2D(u, vVal1.Max)), Color.Black, 1);
                    dc3d1.Add(surface.FixedU(u, vVal1.Min, vVal1.Max) as IGeoObject, Color.Black);
                }
                foreach (double v in vVal1)
                {
                    dcuv1.Add(new Line2D(new GeoPoint2D(uVal1.Min, v), new GeoPoint2D(uVal1.Max, v)), Color.Black, 1);
                    dc3d1.Add(surface.FixedV(v, uVal1.Min, uVal1.Max) as IGeoObject, Color.Black);
                }
                foreach (double u in uVal2)
                {
                    dcuv2.Add(new Line2D(new GeoPoint2D(u, vVal2.Min), new GeoPoint2D(u, vVal2.Max)), Color.Black, 1);
                    dc3d2.Add(other.FixedU(u, vVal2.Min, vVal2.Max) as IGeoObject, Color.Black);
                }
                foreach (double v in vVal2)
                {
                    dcuv2.Add(new Line2D(new GeoPoint2D(uVal2.Min, v), new GeoPoint2D(uVal2.Max, v)), Color.Black, 1);
                    dc3d2.Add(other.FixedV(v, uVal2.Min, uVal2.Max) as IGeoObject, Color.Black);
                }
#endif
                Set<LinkedIntersectionPoint> allIps = new Set<LinkedIntersectionPoint>();
                List<double> uknots1 = new List<double>(uVal1); // these will hopefully come sorted
                List<double> vknots1 = new List<double>(vVal1);
                List<double> uknots2 = new List<double>(uVal2);
                List<double> vknots2 = new List<double>(vVal2);
                BoundingRect bounds1 = new BoundingRect(uVal1.Min, vVal1.Min, uVal1.Max, vVal1.Max);
                BoundingRect bounds2 = new BoundingRect(uVal2.Min, vVal2.Min, uVal2.Max, vVal2.Max);
                List<LinkedIntersectionPoint> l;
                for (int i = 0; i < uknots1.Count; i++)
                {
                    l = LinkedIntersectionPoint.CreateIntersections(surface, other, bounds1, bounds2, true, uknots1[i], vVal1.Min, vVal1.Max, true);
                    foreach (LinkedIntersectionPoint lip in l)
                    {
                        lip.ui1 = i;
                        if (testAndChangeParameter(lip.uv1.y, vVal1, v1prec))
                        {
                            splitted = true; // this restarts from the beginning
                            break;
                        }
                    }
                    if (splitted) break;
                    allIps.AddMany(l);
                }
                if (splitted) continue;
                for (int i = 0; i < vknots1.Count; i++)
                {
                    l = LinkedIntersectionPoint.CreateIntersections(surface, other, bounds1, bounds2, true, vknots1[i], uVal1.Min, uVal1.Max, false);
                    foreach (LinkedIntersectionPoint lip in l)
                    {
                        lip.vi1 = i;
                        if (testAndChangeParameter(lip.uv1.x, uVal1, u1prec))
                        {
                            splitted = true; // this restarts from the beginning
                            break;
                        }
                    }
                    if (splitted) break;
                    allIps.AddMany(l);
                }
                if (splitted) continue;
                for (int i = 0; i < uknots2.Count; i++)
                {
                    l = LinkedIntersectionPoint.CreateIntersections(surface, other, bounds1, bounds2, false, uknots2[i], vVal2.Min, vVal2.Max, true);
                    foreach (LinkedIntersectionPoint lip in l)
                    {
                        lip.ui2 = i;
                        if (testAndChangeParameter(lip.uv2.y, vVal2, v2prec))
                        {
                            splitted = true; // this restarts from the beginning
                            break;
                        }
                    }
                    if (splitted) break;
                    allIps.AddMany(l);
                }
                if (splitted) continue;
                for (int i = 0; i < vknots2.Count; i++)
                {
                    l = LinkedIntersectionPoint.CreateIntersections(surface, other, bounds1, bounds2, false, vknots2[i], uVal2.Min, uVal2.Max, false);
                    foreach (LinkedIntersectionPoint lip in l)
                    {
                        lip.vi2 = i;
                        if (testAndChangeParameter(lip.uv2.x, uVal2, u2prec))
                        {
                            splitted = true; // this restarts from the beginning
                            break;
                        }
                    }
                    if (splitted) break;
                    allIps.AddMany(l);
                }
                if (splitted) continue;
                //if (seeds != null)
                //{
                //    for (int i = 0; i < seeds.Length; i++)
                //    {
                //        allIps.Add(LinkedIntersectionPoint.CreateFromSeed(surface, other, bounds1, bounds2, seeds[i]));
                //    }
                //}
#if DEBUG
                double d1 = ((uVal1.Max - uVal1.Min) + (vVal1.Max - vVal1.Min)) / 100.0;
                double d2 = ((uVal2.Max - uVal2.Min) + (vVal2.Max - vVal2.Min)) / 100.0;
                int ipcont = -1;
                foreach (LinkedIntersectionPoint lip in allIps)
                {
                    ++ipcont;
                    if ((lip.mode & LinkedIntersectionPoint.emode.in1) != 0)
                    {
                        if (lip.dir1.IsNullVector())
                        {
                            Circle2D c2d = new Circle2D(lip.uv1, d1);
                            dcuv1.Add(c2d, Color.Red, lip.id);
                            c2d = new Circle2D(lip.uv2, d2);
                            dcuv2.Add(c2d, Color.Blue, lip.id);
                        }
                        else
                        {
                            Line2D l2d = new Line2D(lip.uv1, lip.uv1 + d1 * lip.dir1.Normalized);
                            dcuv1.Add(l2d, Color.Red, lip.id);
                            l2d = new Line2D(lip.uv2, lip.uv2 + d2 * lip.dir2.Normalized);
                            dcuv2.Add(l2d, Color.Blue, lip.id);
                        }
                    }
                    if ((lip.mode & LinkedIntersectionPoint.emode.in2) != 0)
                    {
                        if (lip.dir2.IsNullVector())
                        {
                            Circle2D c2d = new Circle2D(lip.uv2, d2);
                            dcuv2.Add(c2d, Color.Red, lip.id);
                            c2d = new Circle2D(lip.uv1, d1);
                            dcuv1.Add(c2d, Color.Blue, lip.id);
                        }
                        else
                        {
                            Line2D l2d = new Line2D(lip.uv2, lip.uv2 + d2 * lip.dir2.Normalized);
                            dcuv2.Add(l2d, Color.Red, lip.id);
                            l2d = new Line2D(lip.uv1, lip.uv1 + d1 * lip.dir1.Normalized);
                            dcuv1.Add(l2d, Color.Blue, lip.id);
                        }
                    }
                    if ((lip.mode & LinkedIntersectionPoint.emode.seed) != 0)
                    {
                        if (lip.dir2.IsNullVector())
                        {
                            Circle2D c2d = new Circle2D(lip.uv2, d2);
                            dcuv2.Add(c2d, Color.Green, lip.id);
                            c2d = new Circle2D(lip.uv1, d1);
                            dcuv1.Add(c2d, Color.Green, lip.id);
                        }
                        else
                        {
                            Line2D l2d = new Line2D(lip.uv2, lip.uv2 + d2 * lip.dir2.Normalized);
                            dcuv2.Add(l2d, Color.Green, lip.id);
                            l2d = new Line2D(lip.uv1, lip.uv1 + d1 * lip.dir1.Normalized);
                            dcuv1.Add(l2d, Color.Green, lip.id);
                        }

                    }
                }
#endif
                // remove duplicates
                double prec = (octtree.Extend.Size + otherBS.octtree.Extend.Size) * 1e-6;
                OctTree<LinkedIntersectionPoint> ipocttree = new OctTree<LinkedIntersectionPoint>(octtree.Extend + otherBS.octtree.Extend, octtree.precision);
                Set<LinkedIntersectionPoint> toRemove = new Set<LinkedIntersectionPoint>();
                foreach (LinkedIntersectionPoint lip in allIps)
                {
                    if (ipocttree.IsEmpty) ipocttree.AddObject(lip);
                    else
                    {
                        LinkedIntersectionPoint[] close = ipocttree.GetObjectsCloseTo(lip);
                        bool merged = false;
                        for (int i = 0; i < close.Length; i++)
                        {
                            if ((close[i].ip | lip.ip) < prec)
                            {
                                close[i].MergeWith(lip);
                                merged = true;
                                toRemove.Add(lip);
                                break;
                            }
                        }
                        if (!merged) ipocttree.AddObject(lip);
                    }
                }
                allIps.RemoveMany(toRemove);
                // sort into the chessboard given by the u and v values
                // index 0 is left resp. below the grid, index knots.Count is right resp. above the grid. This way points outside can also be sorted in sensibly
                Dictionary<Tuple<int, int>, List<LinkedIntersectionPoint>> surf1ips = new Dictionary<Tuple<int, int>, List<LinkedIntersectionPoint>>();
                Dictionary<Tuple<int, int>, List<LinkedIntersectionPoint>> surf2ips = new Dictionary<Tuple<int, int>, List<LinkedIntersectionPoint>>();
                foreach (LinkedIntersectionPoint lip in allIps)
                {
                    lip.AddToGrid(surf1ips, surf2ips, uknots1, vknots1, uknots2, vknots2);
                }
#if DEBUG
                for (int i = 0; i < allIps.Debug.Length; i++)
                {
                    if (allIps.Debug[i].mode == LinkedIntersectionPoint.emode.seed)
                    {

                    }
                }
#endif
                // establish the connections within a chessboard field in the respective surface
                for (int s = 1; s <= 2; ++s) // both surfaces
                {
                    if (splitted) break;
                    Dictionary<Tuple<int, int>, List<LinkedIntersectionPoint>> surfips;
                    if (s == 1) surfips = surf1ips;
                    else surfips = surf2ips;
                    foreach (KeyValuePair<Tuple<int, int>, List<LinkedIntersectionPoint>> kv in surfips)
                    {
                        if (splitted) break;
                        // the list contains all intersection points in this chessboard field
                        // we now only look for those that belong to surface1
                        List<LinkedIntersectionPoint> entering = new List<LinkedIntersectionPoint>();
                        List<LinkedIntersectionPoint> leaving = new List<LinkedIntersectionPoint>();
                        List<LinkedIntersectionPoint> tangential = new List<LinkedIntersectionPoint>();

                        for (int i = 0; i < kv.Value.Count; i++)
                        {
                            LinkedIntersectionPoint lip = kv.Value[i];
                            GeoVector2D dir;
                            int ui, vi;
                            LinkedIntersectionPoint.emode testMode;
                            if (s == 1)
                            {
                                dir = lip.dir1;
                                ui = lip.ui1;
                                vi = lip.vi1;
                                testMode = LinkedIntersectionPoint.emode.in1;
                            }
                            else
                            {
                                dir = lip.dir2;
                                ui = lip.ui2;
                                vi = lip.vi2;
                                testMode = LinkedIntersectionPoint.emode.in2;
                            }
                            if (lip.mode.HasFlag(testMode))
                            {
                                if (dir.IsNullVector()) tangential.Add(lip);
                                else if (ui >= 0 && vi >= 0)
                                {   // lies on a corner
                                    if (ui == kv.Key.Item1 && vi == kv.Key.Item2)
                                    {   // is upper right corner point
                                        if (dir.x <= 0 && dir.y <= 0) entering.Add(lip);
                                        if (dir.x >= 0 && dir.y >= 0) leaving.Add(lip);
                                    }
                                    else if (ui + 1 == kv.Key.Item1 && vi == kv.Key.Item2)
                                    {   // is upper left corner point
                                        if (dir.x >= 0 && dir.y <= 0) entering.Add(lip);
                                        if (dir.x <= 0 && dir.y >= 0) leaving.Add(lip);
                                    }
                                    else if (ui == kv.Key.Item1 && vi + 1 == kv.Key.Item2)
                                    {   // is lower right corner point
                                        if (dir.x <= 0 && dir.y >= 0) entering.Add(lip);
                                        if (dir.x >= 0 && dir.y <= 0) leaving.Add(lip);
                                    }
                                    else if (ui + 1 == kv.Key.Item1 && vi + 1 == kv.Key.Item2)
                                    {   // is lower left corner point
                                        if (dir.x >= 0 && dir.y >= 0) entering.Add(lip);
                                        if (dir.x <= 0 && dir.y <= 0) leaving.Add(lip);
                                    }
                                    else
                                    {

                                    }
                                }
                                else if (ui >= 0)
                                {   // left or right edge
                                    if (ui == kv.Key.Item1)
                                    {   // is right edge
                                        if (dir.x <= 0) entering.Add(lip);
                                        if (dir.x >= 0) leaving.Add(lip);
                                    }
                                    else if (ui + 1 == kv.Key.Item1)
                                    {   // is left edge
                                        if (dir.x >= 0) entering.Add(lip);
                                        if (dir.x <= 0) leaving.Add(lip);
                                    }
                                    else
                                    {

                                    }
                                }
                                else if (vi >= 0)
                                {   // lower or upper edge
                                    if (vi == kv.Key.Item2)
                                    {   // is upper edge
                                        if (dir.y <= 0) entering.Add(lip);
                                        if (dir.y >= 0) leaving.Add(lip);
                                    }
                                    else if (vi + 1 == kv.Key.Item2)
                                    {   // is lower edge
                                        if (dir.y >= 0) entering.Add(lip);
                                        if (dir.y <= 0) leaving.Add(lip);
                                    }
                                    else
                                    {

                                    }
                                }
                            }
                        }
                        if (tangential.Count == 1 && (entering.Count + leaving.Count) == 1)
                        {
                            if (entering.Count == 0) entering.Add(tangential[0]);
                            else leaving.Add(tangential[0]);
                            tangential.Clear();
                        }
                        if (entering.Count > 1 || leaving.Count > 1)
                        {
                            // in this patch one cannot connect unambiguously. We must introduce an additional separating line and start over from the beginning
                            for (int ii = 0; ii < entering.Count; ii++)
                            {
                                for (int jj = 0; jj < leaving.Count; jj++)
                                {
                                    if (s == 1)
                                    {
                                        if (entering[ii].ui1 == leaving[jj].ui1 && entering[ii].ui1 >= 0)
                                        {
                                            vVal1.Add((entering[ii].uv1.y + leaving[jj].uv1.y) / 2.0);
                                            splitted = true;
                                            break;
                                        }
                                        else if (entering[ii].vi1 == leaving[jj].vi1 && entering[ii].vi1 >= 0)
                                        {
                                            uVal1.Add((entering[ii].uv1.x + leaving[jj].uv1.x) / 2.0);
                                            splitted = true;
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        if (entering[ii].ui2 == leaving[jj].ui2 && entering[ii].ui2 >= 0)
                                        {
                                            vVal2.Add((entering[ii].uv2.y + leaving[jj].uv2.y) / 2.0);
                                            splitted = true;
                                            break;
                                        }
                                        else if (entering[ii].vi2 == leaving[jj].vi2 && entering[ii].vi2 >= 0)
                                        {
                                            uVal2.Add((entering[ii].uv2.x + leaving[jj].uv2.x) / 2.0);
                                            splitted = true;
                                            break;
                                        }
                                    }
                                    if (splitted) break;
                                }
                                if (splitted) break;
                            }
                            if (!splitted)
                            {   // maybe there is a intersection tangential to a fixed-u or fixed-v line. This can be removed by varying the uvals or vvals by a small amount
                                if (entering.Count > 1)
                                {
                                    if (s == 1)
                                    {
                                        double badval = double.MaxValue;
                                        bool badu = true;
                                        bool up = true;

                                        double umin = double.MaxValue;
                                        double vmin = double.MaxValue;
                                        double umax = double.MinValue;
                                        double vmax = double.MinValue;
                                        for (int ii = 0; ii < entering.Count; ii++)
                                        {
                                            if (entering[ii].ui1 >= 0 && Math.Abs(entering[ii].dir1.x) < 1e-3)
                                            {
                                                badval = entering[ii].uv1.x;
                                                badu = true;
                                                up = entering[ii].dir1.x < 0;
                                            }
                                            if (entering[ii].vi1 >= 0 && Math.Abs(entering[ii].dir1.y) < 1e-3)
                                            {
                                                badval = entering[ii].uv1.y;
                                                badu = false;
                                                up = entering[ii].dir1.y < 0;
                                            }
                                            umin = Math.Min(umin, entering[ii].uv1.x);
                                            umax = Math.Max(umax, entering[ii].uv1.x);
                                            vmin = Math.Min(vmin, entering[ii].uv1.y);
                                            vmax = Math.Max(vmax, entering[ii].uv1.y);
                                        }
                                        if (badval < double.MaxValue)
                                        {
                                            if (badu)
                                            {
                                                uVal1.Remove(badval);
                                                if (up) uVal1.Add(badval + u1prec);
                                                else uVal1.Add(badval - u1prec);
                                            }
                                            else
                                            {
                                                vVal1.Remove(badval);
                                                if (up) vVal1.Add(badval + v1prec);
                                                else vVal1.Add(badval - v1prec);
                                            }
                                        }
                                        else
                                        {
                                            uVal1.Add((umax + umin) / 2.0);
                                            vVal1.Add((vmax + vmin) / 2.0);
                                        }

                                        splitted = true;
                                        break;
                                    }
                                    else
                                    {
                                        double badval = double.MaxValue;
                                        bool badu = true;
                                        bool up = true;

                                        double umin = double.MaxValue;
                                        double vmin = double.MaxValue;
                                        double umax = double.MinValue;
                                        double vmax = double.MinValue;
                                        for (int ii = 0; ii < entering.Count; ii++)
                                        {
                                            if (entering[ii].ui2 >= 0 && Math.Abs(entering[ii].dir2.x) < 1e-3)
                                            {
                                                badval = entering[ii].uv2.x;
                                                badu = true;
                                                up = entering[ii].dir2.x < 0;
                                            }
                                            if (entering[ii].vi2 >= 0 && Math.Abs(entering[ii].dir2.y) < 1e-3)
                                            {
                                                badval = entering[ii].uv2.y;
                                                badu = false;
                                                up = entering[ii].dir2.y < 0;
                                            }
                                            umin = Math.Min(umin, entering[ii].uv2.x);
                                            umax = Math.Max(umax, entering[ii].uv2.x);
                                            vmin = Math.Min(vmin, entering[ii].uv2.y);
                                            vmax = Math.Max(vmax, entering[ii].uv2.y);
                                        }
                                        if (badval < double.MaxValue)
                                        {
                                            if (badu)
                                            {
                                                uVal2.Remove(badval);
                                                if (up) uVal2.Add(badval + u2prec);
                                                else uVal2.Add(badval - u2prec);
                                            }
                                            else
                                            {
                                                vVal2.Remove(badval);
                                                if (up) vVal2.Add(badval + v2prec);
                                                else vVal2.Add(badval - v2prec);
                                            }
                                        }
                                        else
                                        {
                                            uVal2.Add((umax + umin) / 2.0);
                                            vVal2.Add((vmax + vmin) / 2.0);
                                        }

                                        splitted = true;
                                        break;
                                    }
                                }
                                else // leaving.Count>1
                                {
                                    if (s == 1)
                                    {
                                        double badval = double.MaxValue;
                                        bool badu = true;
                                        bool up = true;

                                        double umin = double.MaxValue;
                                        double vmin = double.MaxValue;
                                        double umax = double.MinValue;
                                        double vmax = double.MinValue;
                                        for (int ii = 0; ii < leaving.Count; ii++)
                                        {
                                            if (leaving[ii].ui1 >= 0 && Math.Abs(leaving[ii].dir1.x) < 1e-3)
                                            {
                                                badval = leaving[ii].uv1.x;
                                                badu = true;
                                                up = leaving[ii].dir1.x < 0;
                                            }
                                            if (leaving[ii].vi1 >= 0 && Math.Abs(leaving[ii].dir1.y) < 1e-3)
                                            {
                                                badval = leaving[ii].uv1.y;
                                                badu = false;
                                                up = leaving[ii].dir1.y < 0;
                                            }
                                            umin = Math.Min(umin, leaving[ii].uv1.x);
                                            umax = Math.Max(umax, leaving[ii].uv1.x);
                                            vmin = Math.Min(vmin, leaving[ii].uv1.y);
                                            vmax = Math.Max(vmax, leaving[ii].uv1.y);
                                        }
                                        if (badval < double.MaxValue)
                                        {
                                            if (badu)
                                            {
                                                uVal1.Remove(badval);
                                                if (up) uVal1.Add(badval + u1prec);
                                                else uVal1.Add(badval - u1prec);
                                            }
                                            else
                                            {
                                                vVal1.Remove(badval);
                                                if (up) vVal1.Add(badval + v1prec);
                                                else vVal1.Add(badval - v1prec);
                                            }
                                        }
                                        else
                                        {
                                            uVal1.Add((umax + umin) / 2.0);
                                            vVal1.Add((vmax + vmin) / 2.0);
                                        }

                                        splitted = true;
                                        break;
                                    }
                                    else
                                    {
                                        double badval = double.MaxValue;
                                        bool badu = true;
                                        bool up = true;

                                        double umin = double.MaxValue;
                                        double vmin = double.MaxValue;
                                        double umax = double.MinValue;
                                        double vmax = double.MinValue;
                                        for (int ii = 0; ii < leaving.Count; ii++)
                                        {
                                            if (leaving[ii].ui2 >= 0 && Math.Abs(leaving[ii].dir2.x) < 1e-3)
                                            {
                                                badval = leaving[ii].uv2.x;
                                                badu = true;
                                                up = leaving[ii].dir2.x < 0;
                                            }
                                            if (leaving[ii].vi2 >= 0 && Math.Abs(leaving[ii].dir2.y) < 1e-3)
                                            {
                                                badval = leaving[ii].uv2.y;
                                                badu = false;
                                                up = leaving[ii].dir2.y < 0;
                                            }
                                            umin = Math.Min(umin, leaving[ii].uv2.x);
                                            umax = Math.Max(umax, leaving[ii].uv2.x);
                                            vmin = Math.Min(vmin, leaving[ii].uv2.y);
                                            vmax = Math.Max(vmax, leaving[ii].uv2.y);
                                        }
                                        if (badval < double.MaxValue)
                                        {
                                            if (badu)
                                            {
                                                uVal2.Remove(badval);
                                                if (up) uVal2.Add(badval + u2prec);
                                                else uVal2.Add(badval - u2prec);
                                            }
                                            else
                                            {
                                                vVal2.Remove(badval);
                                                if (up) vVal2.Add(badval + v2prec);
                                                else vVal2.Add(badval - v2prec);
                                            }
                                        }
                                        else
                                        {
                                            uVal2.Add((umax + umin) / 2.0);
                                            vVal2.Add((vmax + vmin) / 2.0);
                                        }

                                        splitted = true;
                                        break;
                                    }
                                }
                            }
                        }
                        if (entering.Count == 1 && leaving.Count == 1)
                        {
                            if (s == 1)
                            {
#if DEBUG
                                if (entering[0].next1 != null || leaving[0].prev1 != null || entering[0].enterOnSurf1 != null)
                                {

                                }
#endif
                                entering[0].next1 = leaving[0];
                                leaving[0].prev1 = entering[0];
                            }
                            else
                            {
#if DEBUG
                                if (entering[0].next2 != null || leaving[0].prev2 != null || entering[0].enterOnSurf2 != null)
                                {

                                }
#endif
                                entering[0].next2 = leaving[0];
                                leaving[0].prev2 = entering[0];
                            }
                        }
                        if (entering.Count == 1)
                        {
                            if (s == 1)
                            {
                                entering[0].enterOnSurf1 = kv.Key;
                            }
                            else
                            {
                                entering[0].enterOnSurf2 = kv.Key;
                            }
                        }
                    }
                }
                if (splitted) continue;
                // in each field of the chessboard, insert the respective points of the other chessboard into the chain
                // each point knows in which field it is an entry point
#if DEBUG
                foreach (LinkedIntersectionPoint lip in allIps)
                {
                    lip.next = null;
                    lip.prev = null;
                }
#endif
                List<LinkedIntersectionPoint> startPointOn1 = new List<LinkedIntersectionPoint>(); // not used any further, right?
                List<LinkedIntersectionPoint> startPointOn2 = new List<LinkedIntersectionPoint>();
                List<LinkedIntersectionPoint> enterPoints = new List<LinkedIntersectionPoint>();
                foreach (LinkedIntersectionPoint lip in allIps)
                {
                    if (lip.prev1 == null) // lip.next1 != null is not necessary, can also be the only point
                    {
                        if ((lip.enterOnSurf1 != null && ((lip.ui1 == 0) || lip.ui1 == uknots1.Count - 1)) ||
                        (lip.enterOnSurf1 != null && ((lip.vi1 == 0) || lip.vi1 == vknots1.Count - 1)))
                            enterPoints.Add(lip);
                        else
                            startPointOn1.Add(lip);
                    }
                    if (lip.prev2 == null) // lip.next2 != null is not necessary, can also be the only point
                    {
                        if ((lip.enterOnSurf2 != null && ((lip.ui2 == 0) || lip.ui2 == uknots2.Count - 1)) ||
                        (lip.enterOnSurf2 != null && ((lip.vi2 == 0) || lip.vi2 == vknots2.Count - 1)))
                        {
                            enterPoints.Add(lip);
                        }
                        else
                            startPointOn2.Add(lip);
                    }
                }
                LinkedIntersectionPoint cyclicalStartPoint = null;
                if (enterPoints.Count == 0 && allIps.Count > 0)
                {   // probably a closed curve, not crossing the bounds of either thisBound nor otherBounds
                    cyclicalStartPoint = allIps.GetAny();
                    cyclicalStartPoint.mode |= LinkedIntersectionPoint.emode.isCyclicalStart;
                    enterPoints.Add(cyclicalStartPoint); // for the following loop 
                }
                List<LinkedIntersectionPoint> startPoints = new List<LinkedIntersectionPoint>(enterPoints); // in case of cyclical curves we cann add more to startPoints
                                                                                                            // all intersection points are now connected via next1/prev1 or next2/prev2. these two chains are now combined so that next/prev are valid
                foreach (LinkedIntersectionPoint enterPoint in enterPoints)
                {
                    LinkedIntersectionPoint currentOn1 = null, currentOn2 = null, current = null;
                    if (enterPoint.enterOnSurf1 != null)
                    {
                        current = currentOn1 = enterPoint;
                    }
                    else if (enterPoint.enterOnSurf2 != null)
                    {
                        current = currentOn2 = enterPoint;
                    }
                    while (current != null)
                    {
                        if (current.mode.HasFlag(LinkedIntersectionPoint.emode.in1) && current.mode.HasFlag(LinkedIntersectionPoint.emode.in2))
                        {
                            if (current.next2 != null && surf1ips[current.enterOnSurf1].Contains(current.next2))
                            {
                                current.next = current.next2;
                                currentOn2 = current.next2;
                            }
                            else if (current.next1 != null && surf2ips[current.enterOnSurf2].Contains(current.next1))
                            {
                                current.next = current.next1;
                                currentOn1 = current.next1;
                            }
                            else
                            {   // should not happen, only when next on both grids is again on both grids
                                if (current.next2 != null)
                                {
                                    current.next = current.next2;
                                    currentOn2 = current.next2;
                                }
                                else if (current.next1 != null)
                                {
                                    current.next = current.next1;
                                    currentOn1 = current.next1;
                                }
                            }

                        }
                        else if (current.mode.HasFlag(LinkedIntersectionPoint.emode.in1))
                        {
                            if (currentOn2 == null && current.enterOnSurf1 != null)
                            {
                                foreach (LinkedIntersectionPoint s2 in surf1ips[current.enterOnSurf1])
                                {
                                    if (s2.mode.HasFlag(LinkedIntersectionPoint.emode.in2))
                                    {
                                        if (s2.prev2 == null)
                                        {
                                            currentOn2 = s2;
                                            break;
                                        }
                                        else if (current == cyclicalStartPoint)
                                        {
                                            LinkedIntersectionPoint s22 = s2;
                                            while (s22.prev2 != null && surf1ips[current.enterOnSurf1].Contains(s22.prev2)) s22 = s22.prev2;
                                            currentOn2 = s22;
                                            break;
                                        }
                                    }
                                }
                            }
                            if (currentOn2 != null && current.enterOnSurf1 != null && surf1ips[current.enterOnSurf1].Contains(currentOn2))
                            {
                                current.next = currentOn2;
                                currentOn2 = currentOn2.next2;
                            }
                            else
                            {
                                current.next = current.next1;
                            }
                            currentOn1 = current.next1;
                        }
                        else if (current.mode.HasFlag(LinkedIntersectionPoint.emode.in2))
                        {
                            if (currentOn1 == null && current.enterOnSurf2 != null)
                            {
                                foreach (LinkedIntersectionPoint s1 in surf2ips[current.enterOnSurf2])
                                {
                                    if (s1.mode.HasFlag(LinkedIntersectionPoint.emode.in1))
                                    {
                                        if (s1.prev1 == null)
                                        {
                                            currentOn1 = s1;
                                            break;
                                        }
                                        else if (current == cyclicalStartPoint)
                                        {   // find first intersectionpoint on surface1 which is in the mesh of current
                                            LinkedIntersectionPoint s11 = s1;
                                            while (s11.prev1 != null && surf2ips[current.enterOnSurf2].Contains(s11.prev1)) s11 = s11.prev1;
                                            currentOn1 = s11;
                                            break;
                                        }
                                    }
                                }
                            }
                            if (currentOn1 != null && current.enterOnSurf2 != null && surf2ips[current.enterOnSurf2].Contains(currentOn1))
                            {
                                current.next = currentOn1;
                                currentOn1 = currentOn1.next1;
                            }
                            else
                            {
                                current.next = current.next2;
                            }
                            currentOn2 = current.next2;
                        }
                        current = current.next;
                        if (current != null && current == cyclicalStartPoint)
                        {
                            current = null;
                            // we have closed a loop, maybe there are more loops
                            foreach (LinkedIntersectionPoint lip in allIps)
                            {
                                if (lip.next == null)
                                {
                                    current = cyclicalStartPoint = lip;
                                    startPoints.Add(current);
                                    current.mode |= LinkedIntersectionPoint.emode.isCyclicalStart;
                                }
                            }
                        }
                    }
                }
                //                foreach (LinkedIntersectionPoint lip in allIps)
                //                {   // do the back-linking
                //                    if (lip.next != null)
                //                    {
                //                        if (lip.enterOnSurf1 != null && ((lip.enterOnSurf1.Item1 == 0) || lip.enterOnSurf1.Item1 == uknots1.Count)) continue;
                //                        if (lip.enterOnSurf1 != null && ((lip.enterOnSurf1.Item2 == 0) || lip.enterOnSurf1.Item2 == vknots1.Count)) continue;
                //                        if (lip.enterOnSurf2 != null && ((lip.enterOnSurf2.Item1 == 0) || lip.enterOnSurf2.Item1 == uknots2.Count)) continue;
                //                        if (lip.enterOnSurf2 != null && ((lip.enterOnSurf2.Item2 == 0) || lip.enterOnSurf2.Item2 == vknots2.Count)) continue;
                //#if DEBUG
                //                        System.Diagnostics.Trace.Assert(lip.next.prev == lip);
                //#endif
                //                    }
                //                }
                // the seeds are not yet sorted in (unless they are identical to an intersection point)
                // closed curves are not yet considered!
                // List<LinkedIntersectionPoint> endPoints = new List<LinkedIntersectionPoint>();
                //foreach (LinkedIntersectionPoint lip in allIps)
                //{
                //    if (lip.prev == null && lip.mode != LinkedIntersectionPoint.emode.seed) startPoints.Add(lip);
                //    // if (lip.next == null) endPoints.Add(lip);
                //}
                //List<LinkedIntersectionPoint> singularSeeds = new List<LinkedIntersectionPoint>();
                //for (int i = 0; i < startPoints.Count; i++)
                //{
                //    if (startPoints[i].next == null && startPoints[i].mode == LinkedIntersectionPoint.emode.seed)
                //    {
                //        // a single seed that does not lie on an edge and is not linked in
                //        singularSeeds.Add(startPoints[i]);
                //    }
                //}
#if DEBUG
                foreach (LinkedIntersectionPoint lip in allIps)
                {
                    //if ((lip.mode & LinkedIntersectionPoint.emode.in1) != 0)
                    //{
                    //    if (lip.next1 != null)
                    //    {
                    //        Line2D l2d = new Line2D(lip.uv1, lip.next1.uv1);
                    //        dcuv1.Add(l2d, Color.Black, lip.id);
                    //    }
                    //}
                    //if ((lip.mode & LinkedIntersectionPoint.emode.in2) != 0)
                    //{
                    //    if (lip.next2 != null)
                    //    {
                    //        Line2D l2d = new Line2D(lip.uv2, lip.next2.uv2);
                    //        dcuv2.Add(l2d, Color.Black, lip.id);
                    //    }
                    //}
                }
                for (int i = 0; i < startPoints.Count; i++)
                {
                    List<GeoPoint> points = new List<GeoPoint>();
                    LinkedIntersectionPoint st = startPoints[i];
                    bool isClosed = st.mode.HasFlag(LinkedIntersectionPoint.emode.isCyclicalStart);
                    while (st != null)
                    {
                        points.Add(st.ip);
                        st = st.next;
                        if (st == startPoints[i]) break; // loop, closed
                        if (points.Count > allIps.Count)
                        {
                            // this must not happen, a breakpoint here
                            break;
                        }
                    }
                    if (points.Count > 1)
                    {
                        Polyline pl = Polyline.Construct();
                        pl.SetPoints(points.ToArray(), isClosed);
                        dc3d1.Add(pl, Color.Red);
                        dc3d2.Add(pl, Color.Red);
                    }
                }
                //List<LinkedIntersectionPoint> spon1 = new List<GeoObject.ParallelepipedHull.LinkedIntersectionPoint>();
                //foreach (LinkedIntersectionPoint lip in allIps)
                //{
                //    if (lip.mode.HasFlag(LinkedIntersectionPoint.emode.in1) && lip.prev1 == null) spon1.Add(lip);
                //    if (lip.next1 != null)
                //    {
                //        Line2D l2d = new Line2D(lip.uv1, lip.next1.uv1);
                //        dcuv1.Add(l2d.Trim(0.0, 0.9), Color.Orange, 1);
                //    }
                //    if (lip.next2 != null)
                //    {
                //        Line2D l2d = new Line2D(lip.uv2, lip.next2.uv2);
                //        dcuv2.Add(l2d.Trim(0.0, 0.9), Color.Orange, 1);
                //    }
                //}
#endif
                // the following test determines whether two points immediately form a cycle in one of the two uv nets.
                // This could be a wrong connection of two independent intersection curves
                foreach (LinkedIntersectionPoint lip in allIps)
                {
                    if (lip.next1 != null && lip.next1.next1 == lip)
                    {
                        // two consecutive ones are cyclic: insert an intermediate stage
                        if (Math.Abs(lip.uv1.x - lip.next1.uv1.x) > Math.Abs(lip.uv1.y - lip.next1.uv1.y))
                        {
                            if (uVal1.Add((lip.uv1.x + lip.next1.uv1.x) / 2.0))
                            {
                                splitted = true;
                                break;
                            }
                        }
                        else
                        {
                            if (vVal1.Add((lip.uv1.y + lip.next1.uv1.y) / 2.0))
                            {
                                splitted = true;
                                break;
                            }
                        }
                    }
                    if (lip.next2 != null && lip.next2.next2 == lip)
                    {
                        // two consecutive ones are cyclic: insert an intermediate stage
                        if (Math.Abs(lip.uv2.x - lip.next2.uv2.x) > Math.Abs(lip.uv2.y - lip.next2.uv2.y))
                        {
                            if (uVal2.Add((lip.uv2.x + lip.next2.uv2.x) / 2.0))
                            {
                                splitted = true;
                                break;
                            }
                        }
                        else
                        {
                            if (vVal2.Add((lip.uv2.y + lip.next2.uv2.y) / 2.0))
                            {
                                splitted = true;
                                break;
                            }
                        }
                    }
                }
                if (splitted) continue;

                for (int i = 0; i < startPoints.Count; i++)
                {
                    List<GeoPoint> points = new List<GeoPoint>();
                    List<GeoPoint2D> uvpoints1 = new List<GeoPoint2D>();
                    List<GeoPoint2D> uvpoints2 = new List<GeoPoint2D>();
                    LinkedIntersectionPoint st = startPoints[i];
                    double totlen = 0.0;
                    while (st != null)
                    {
                        if (points.Count > 0)
                        {
                            totlen += points[points.Count - 1] | st.ip;
                        }
                        points.Add(st.ip);
                        uvpoints1.Add(st.uv1);
                        uvpoints2.Add(st.uv2);
                        st = st.next;
                        if (st == startPoints[i]) break; // loop, closed
                        if (points.Count > allIps.Count) break; // an inner loop has formed. this must not happen
                    }
                    if (points.Count > allIps.Count)
                    {   // an inner loop, this is not allowed. Split where the segment meets inward
                        Set<LinkedIntersectionPoint> connected = new Set<LinkedIntersectionPoint>();
                        st = startPoints[i];
                        LinkedIntersectionPoint last = null;
                        while (st != null)
                        {
                            if (connected.Contains(st))
                            {
                                if (last.mode.HasFlag(LinkedIntersectionPoint.emode.in1) && st.mode.HasFlag(LinkedIntersectionPoint.emode.in1))
                                {
                                    if (Math.Abs(last.uv1.x - st.uv1.x) > Math.Abs(last.uv1.y - st.uv1.y))
                                    {
                                        uVal1.Add((last.uv1.x + st.uv1.x) / 2.0);
                                        splitted = true;
                                        break;
                                    }
                                    else
                                    {
                                        vVal1.Add((last.uv1.y + st.uv1.y) / 2.0);
                                        splitted = true;
                                        break;
                                    }
                                }
                                else
                                {
                                    if (Math.Abs(last.uv2.x - st.uv2.x) > Math.Abs(last.uv2.y - st.uv2.y))
                                    {
                                        uVal2.Add((last.uv2.x + st.uv2.x) / 2.0);
                                        splitted = true;
                                        break;
                                    }
                                    else
                                    {
                                        vVal2.Add((last.uv2.y + st.uv2.y) / 2.0);
                                        splitted = true;
                                        break;
                                    }
                                }
                            }
                            else
                            {
                                connected.Add(st);
                            }
                            last = st;
                            st = st.next;
                            if (st == startPoints[i]) break; // loop, closed
                        }
                        if (splitted) break;
                    }
                    if (splitted) break;
                    if (points.Count > 1)
                    {
                        if (seeds != null && seeds.Count > 0 && !startPoints[i].mode.HasFlag(LinkedIntersectionPoint.emode.isCyclicalStart))
                        {   // append single seeds shortly before or after the curve. Inside it is not necessary, they are found later with PositionOf
                            Polyline2D p2d1 = new Polyline2D(uvpoints1.ToArray());
                            Polyline2D p2d2 = new Polyline2D(uvpoints2.ToArray());
                            for (int j = 0; j < seeds.Count; j++)
                            // for (int j = 0; j < singularSeeds.Count; j++)
                            {
                                GeoPoint2D uv1 = this.surface.PositionOf(seeds[j]);
                                SurfaceHelper.AdjustPeriodic(this.surface, thisBounds, ref uv1);
                                GeoPoint2D uv2 = other.PositionOf(seeds[j]);
                                SurfaceHelper.AdjustPeriodic(other, otherBounds, ref uv2);
                                double pos1 = p2d1.PositionOf(uv1);
                                double pos2 = p2d2.PositionOf(uv2);
                                double dp1 = uv1 | p2d1.PointAt(pos1);
                                double dp2 = uv2 | p2d2.PointAt(pos2);
                                if (dp1 < p2d1.Length * 1e-2 && dp2 < p2d2.Length * 1e-2)
                                {
                                    if (pos1 < 1e-10 && pos2 < 1e-10 && pos1 > -0.5 && pos2 > -0.5)
                                    {
                                        uvpoints1.Insert(0, uv1);
                                        uvpoints2.Insert(0, uv2);
                                        points.Insert(0, seeds[j]);
                                        if (j < seeds.Count - 1)
                                        {   // of course no longer necessary on the last time
                                            p2d1 = new Polyline2D(uvpoints1.ToArray()); // the polylines must be remade, because two almost identical seeds behind the end would otherwise be inserted in random order
                                            p2d2 = new Polyline2D(uvpoints2.ToArray());
                                        }
                                    }
                                    else if (pos1 > 1 - 1e-10 && pos2 > 1 - 1e-10 && pos1 < 1.5 && pos2 < 1.5)
                                    {
                                        uvpoints1.Add(uv1);
                                        uvpoints2.Add(uv2);
                                        points.Add(seeds[j]);
                                        if (j < seeds.Count - 1)
                                        {
                                            p2d1 = new Polyline2D(uvpoints1.ToArray()); // the polylines must be remade, because two almost identical seeds behind the end would otherwise be inserted in random order
                                            p2d2 = new Polyline2D(uvpoints2.ToArray());
                                        }
                                    }
                                }
                            }
                        }
                        double mindist = totlen * 0.001; // remove points that lie too close together
                        for (int j = 0; j < points.Count - 1; j++)
                        {
                            double d = points[j] | points[j + 1];
                            if (d < mindist)
                            {
                                if (j == 0)
                                {
                                    points.RemoveAt(1);
                                    uvpoints1.RemoveAt(1);
                                    uvpoints2.RemoveAt(1);
                                }
                                else if (j == points.Count - 2)
                                {
                                    uvpoints1.RemoveAt(points.Count - 2);
                                    uvpoints2.RemoveAt(points.Count - 2);
                                    points.RemoveAt(points.Count - 2);
                                }
                                else
                                {
                                    if ((points[j] | points[j - 1]) < (points[j + 1] | points[j + 2]))
                                    {
                                        points.RemoveAt(j);
                                        uvpoints1.RemoveAt(j);
                                        uvpoints2.RemoveAt(j);
                                    }
                                    else
                                    {
                                        points.RemoveAt(j + 1);
                                        uvpoints1.RemoveAt(j + 1);
                                        uvpoints2.RemoveAt(j + 1);
                                    }
                                }
                                --j;
                            }
                        }
                        if (startPoints[i].mode.HasFlag(LinkedIntersectionPoint.emode.isCyclicalStart))
                        {   // a closed curve, repeat the first point
                            points.Add(points[0]);
                            uvpoints1.Add(uvpoints1[0]);
                            uvpoints2.Add(uvpoints2[0]);
                        }
                        res.Add(new InterpolatedDualSurfaceCurve(surface, thisBounds, other, otherBounds, points, uvpoints1, uvpoints2));
#if DEBUG
                        ICurve2D c2d = (res[0] as InterpolatedDualSurfaceCurve).CurveOnSurface1;
                        //c2d.PointAt(0.001);
                        //c2d.PointAt(0.999);
                        GeoPoint dbgp = res[0].PointAt(0.48);
#endif
                    }
                    if (splitted)
                    {
                        res.Clear();
                        continue;
                    }
                }
            }
            return res.ToArray();
        }

        private bool testAndChangeParameter(double val, SortedSet<double> ssd, double prec)
        {   // removes the value val from the set (if it is inside up to prec) and replaces it by one changed by prec
            SortedSet<double> found = ssd.GetViewBetween(val - prec, val + prec);
            if (found.Count == 0) return false;
            double foundval = found.Min; // only has one anyway
            double toAdd;
            if (foundval == ssd.Min) toAdd = foundval - 4 * prec;
            else if (foundval == ssd.Max) toAdd = foundval + 4 * prec;
            else if (val < foundval) toAdd = foundval + 4 * prec;
            else toAdd = foundval - 4 * prec;
            ssd.Remove(foundval); // correct this value now. Prec is 1/1000 of the interval, so it can be added/subtracted without problems
            ssd.Add(toAdd);
            return true;
        }

        private double smallesDiff(SortedSet<double> ssd)
        {
            SortedSet<double>.Enumerator ssde = ssd.GetEnumerator();
            if (!ssde.MoveNext()) return 0.0;
            double last = ssde.Current;
            double res = double.MaxValue;
            while (ssde.MoveNext())
            {
                double d = ssde.Current;
                res = Math.Min(res, d - last);
                last = d;
            }
            return res;
        }

        private void addOrAdjust(SortedSet<double> ssd, double val)
        {   // add this new value to the set. If there is a similar value, replace it, except if it is a minimum or maximum, then don't add it
            if (val < ssd.Min || val > ssd.Max) return; // don't add a value outside the bounds
            double prec = (ssd.Max - ssd.Min) * 1e-4;
            SortedSet<double> found = ssd.GetViewBetween(val - prec, val + prec); // there should be only one at maximum
            if (found.Count == 0)
            {
                ssd.Add(val);
            }
            else
            {
                double foundval = found.Min; // there is probably only one
                if (foundval == ssd.Min) return; // don't modify the minimum
                else if (foundval == ssd.Max) return; // don't modify the maximum
                else
                {
                    ssd.Remove(foundval); // replace this one by the new value
                    ssd.Add(val);
                }
            }
        }

        internal ICurve[] IntersectOld(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, GeoPoint[] seeds)
        {
            // IntersectNew(thisBounds, other, otherBounds, seeds);
            // Idea: without seed, may return several curves
            // intersect all boxes against each other: intersect the edges of the patches with the other patch and vice versa:
            // no intersection point: there could be an inner closed curve (ignore for now)
            // two intersection points: a curve segment for DualSurfaceCurve found
            // several intersection points (must be an even number): subdivide patches until only two points each. Causes problems on tangency
            // combine all snippets
            ParallelepipedHull otherBS = (other as ISurfaceImpl).ParallelepipedHull;
            // cached intersection points, so that for each u and v the intersections with the other surface only have to be computed once
            Dictionary<double, List<GeoPoint3d2d2d>> uInts = new Dictionary<double, List<GeoPoint3d2d2d>>();
            Dictionary<double, List<GeoPoint3d2d2d>> vInts = new Dictionary<double, List<GeoPoint3d2d2d>>();
            Dictionary<double, List<GeoPoint3d2d2d>> ouInts = new Dictionary<double, List<GeoPoint3d2d2d>>();
            Dictionary<double, List<GeoPoint3d2d2d>> ovInts = new Dictionary<double, List<GeoPoint3d2d2d>>();
#if DEBUG
            Set<ParEpi> usedForIntersection = new Set<ParEpi>();
            SortedSet<double> uVal = new SortedSet<double>();
            SortedSet<double> vVal = new SortedSet<double>();
            SortedSet<double> ouVal = new SortedSet<double>();
            SortedSet<double> ovVal = new SortedSet<double>();
#endif
            List<GeoPoint3d2d2d> ips = new List<GeoPoint3d2d2d>();
            foreach (ParEpi pe in octtree.GetAllObjects())
            {
                if (pe.uvPatch.Interferes(ref thisBounds))
                {
                    foreach (ParEpi otherPe in otherBS.octtree.GetObjectsCloseTo(pe))
                    {
                        if (otherPe.uvPatch.Interferes(ref otherBounds))
                        {
                            if (pe.Interferes(otherPe))
                            {
#if DEBUG
                                usedForIntersection.Add(pe);
                                usedForIntersection.Add(otherPe);
                                uVal.Add(pe.uvPatch.Left);
                                uVal.Add(pe.uvPatch.Right);
                                vVal.Add(pe.uvPatch.Bottom);
                                vVal.Add(pe.uvPatch.Top);
                                ouVal.Add(otherPe.uvPatch.Left);
                                ouVal.Add(otherPe.uvPatch.Right);
                                ovVal.Add(otherPe.uvPatch.Bottom);
                                ovVal.Add(otherPe.uvPatch.Top);
#endif

                                ips.AddRange(FindPatchIntersection(this, otherBS, pe, otherPe, uInts, vInts, ouInts, ovInts, thisBounds, otherBounds));
                            }
                        }
                    }
                }
            }
#if DEBUG
            DebuggerContainer dccrv = new DebuggerContainer();
            ColorDef cdt = new ColorDef("this", Color.Red);
            ColorDef cdo = new ColorDef("other", Color.Blue);
            foreach (double u in uVal)
            {
                ICurve crv = this.surface.FixedU(u, thisBounds.Bottom, thisBounds.Top);
                (crv as IColorDef).ColorDef = cdt;
                dccrv.Add(crv as IGeoObject, 0);
            }
            foreach (double v in vVal)
            {
                ICurve crv = this.surface.FixedV(v, thisBounds.Left, thisBounds.Right);
                (crv as IColorDef).ColorDef = cdt;
                dccrv.Add(crv as IGeoObject, 0);
            }
            foreach (double u in ouVal)
            {
                ICurve crv = other.FixedU(u, otherBounds.Bottom, otherBounds.Top);
                (crv as IColorDef).ColorDef = cdo;
                dccrv.Add(crv as IGeoObject, 1);
            }
            foreach (double v in ovVal)
            {
                ICurve crv = other.FixedV(v, otherBounds.Left, otherBounds.Right);
                (crv as IColorDef).ColorDef = cdo;
                dccrv.Add(crv as IGeoObject, 1);
            }
#endif
            // ips now contains curve snippets connected in groups of 2, which now still have to be sorted
            // how are the snippets clipped with thisBounds/otherBounds?
            // one can sort in both uv systems, here first quick and dirty
#if DEBUG
            DebuggerContainer dc0 = new DebuggerContainer();
            ColorDef green = new ColorDef("green", Color.Green);
            ColorDef red = new ColorDef("red", Color.Red);
            ColorDef blue = new ColorDef("blue", Color.Blue);
            ColorDef black = new ColorDef("black", Color.Black);
            GeoObjectList lst = Debug.toShow;
            for (int i = 0; i < lst.Count; i++)
            {
                (lst[i] as IColorDef).ColorDef = green;
            }
            dc0.Add(lst);
            foreach (ParEpi pe in usedForIntersection) // these are then in twice
            {
                Solid sld = pe.AsBox;
                sld.ColorDef = black;
                //dc0.Add(sld);
            }
            lst = otherBS.Debug.toShow;
            for (int i = 0; i < lst.Count; i++)
            {
                (lst[i] as IColorDef).ColorDef = red;
            }
            dc0.Add(lst);
            for (int i = 0; i < ips.Count; i += 2)
            {
                Line l = Line.TwoPoints(ips[i].p3d, ips[i + 1].p3d);
                l.ColorDef = blue;
                dc0.Add(l, i);
            }
            DebuggerContainer dc3d = new DebuggerContainer();
            DebuggerContainer dc1 = new DebuggerContainer();
            foreach (ParEpi pe in octtree.GetAllObjects())
            {
                Polyline pl = Polyline.Construct();
                pl.SetRectangle(new GeoPoint(pe.uvPatch.GetLowerLeft()), pe.uvPatch.Width * GeoVector.XAxis, pe.uvPatch.Height * GeoVector.YAxis);
                pl.ColorDef = blue;
                dc1.Add(pl, pe.id);
            }
            foreach (KeyValuePair<double, List<GeoPoint3d2d2d>> item in uInts)
            {
                for (int j = 0; j < item.Value.Count; j++)
                {
                    Point pnt = Point.Construct();
                    pnt.Symbol = PointSymbol.Cross;
                    pnt.Location = new GeoPoint(item.Value[j].uv1);
                    pnt.ColorDef = green;
                    dc1.Add(pnt, j);
                    Point pnt3 = Point.Construct();
                    pnt3.Symbol = PointSymbol.Cross;
                    pnt3.Location = item.Value[j].p3d;
                    pnt3.ColorDef = green;
                    dc3d.Add(pnt3, 1000 + j);
                }
            }
            foreach (KeyValuePair<double, List<GeoPoint3d2d2d>> item in vInts)
            {
                for (int j = 0; j < item.Value.Count; j++)
                {
                    Point pnt = Point.Construct();
                    pnt.Symbol = PointSymbol.Cross;
                    pnt.Location = new GeoPoint(item.Value[j].uv1);
                    pnt.ColorDef = red;
                    dc1.Add(pnt, j);
                    Point pnt3 = Point.Construct();
                    pnt3.Symbol = PointSymbol.Cross;
                    pnt3.Location = item.Value[j].p3d;
                    pnt3.ColorDef = red;
                    dc3d.Add(pnt3, 2000 + j);
                }
            }

            DebuggerContainer dc2 = new DebuggerContainer();
            foreach (ParEpi pe in otherBS.octtree.GetAllObjects())
            {
                Polyline pl = Polyline.Construct();
                pl.SetRectangle(new GeoPoint(pe.uvPatch.GetLowerLeft()), pe.uvPatch.Width * GeoVector.XAxis, pe.uvPatch.Height * GeoVector.YAxis);
                pl.ColorDef = blue;
                dc2.Add(pl, pe.id);
            }
            foreach (KeyValuePair<double, List<GeoPoint3d2d2d>> item in ouInts)
            {
                for (int j = 0; j < item.Value.Count; j++)
                {
                    Point pnt = Point.Construct();
                    pnt.Symbol = PointSymbol.Cross;
                    pnt.Location = new GeoPoint(item.Value[j].uv2);
                    pnt.ColorDef = green;
                    dc2.Add(pnt, j);
                    Point pnt3 = Point.Construct();
                    pnt3.Symbol = PointSymbol.Cross;
                    pnt3.Location = item.Value[j].p3d;
                    pnt3.ColorDef = green;
                    dc3d.Add(pnt3, 3000 + j);
                }
            }
            foreach (KeyValuePair<double, List<GeoPoint3d2d2d>> item in ovInts)
            {
                for (int j = 0; j < item.Value.Count; j++)
                {
                    Point pnt = Point.Construct();
                    pnt.Symbol = PointSymbol.Cross;
                    pnt.Location = new GeoPoint(item.Value[j].uv2);
                    pnt.ColorDef = red;
                    dc2.Add(pnt, j);
                    Point pnt3 = Point.Construct();
                    pnt3.Symbol = PointSymbol.Cross;
                    pnt3.Location = item.Value[j].p3d;
                    pnt3.ColorDef = red;
                    dc3d.Add(pnt3, 4000 + j);
                }
            }
            DebuggerContainer dc4 = new DebuggerContainer();
            {
                foreach (KeyValuePair<double, List<GeoPoint3d2d2d>> item in uInts)
                {
                    ICurve crv = this.surface.FixedU(item.Key, thisBounds.Bottom, thisBounds.Top);
                    dc4.Add(crv as IGeoObject, 3);
                }
                foreach (KeyValuePair<double, List<GeoPoint3d2d2d>> item in ovInts)
                {
                    ICurve crv = this.surface.FixedV(item.Key, thisBounds.Left, thisBounds.Right);
                    dc4.Add(crv as IGeoObject, 3);
                }
                foreach (KeyValuePair<double, List<GeoPoint3d2d2d>> item in ouInts)
                {
                    ICurve crv = other.FixedU(item.Key, otherBounds.Bottom, otherBounds.Top);
                    dc4.Add(crv as IGeoObject, 3);
                }
                foreach (KeyValuePair<double, List<GeoPoint3d2d2d>> item in ovInts)
                {
                    ICurve crv = other.FixedV(item.Key, otherBounds.Left, otherBounds.Right);
                    dc4.Add(crv as IGeoObject, 3);
                }
            }

#endif

            if (ips.Count > 0)
            {
                List<List<GeoPoint3d2d2d>> curveParts = new List<List<GeoPoint3d2d2d>>();
                List<GeoPoint3d2d2d> currentList = new List<GeoPoint3d2d2d>();
                GeoPoint3d2d2d sp = ips[0];
                GeoPoint3d2d2d ep = ips[1];
                ips.RemoveRange(0, 2); // or better from the end?
                currentList.Add(sp);
                currentList.Add(ep);
                while (ips.Count >= 2)
                {
                    bool found = false;
                    for (int i = 0; i < ips.Count; i++)
                    {
                        if (ips[i].Equals(currentList[0]))
                        {   // append at the front
                            if ((i & 0x1) == 0)
                            {   // even index, this one and the next apply
                                currentList.Insert(0, ips[i]);
                                currentList.Insert(0, ips[i + 1]);
                                ips.RemoveRange(i, 2);
                                found = true;
                                break;
                            }
                            else
                            {
                                currentList.Insert(0, ips[i]);
                                currentList.Insert(0, ips[i - 1]);
                                ips.RemoveRange(i - 1, 2);
                                found = true;
                                break;
                            }
                        }
                        if (ips[i].Equals(currentList[currentList.Count - 1]))
                        {   // append at the front
                            if ((i & 0x1) == 0)
                            {   // even index, this one and the next apply
                                currentList.Add(ips[i]);
                                currentList.Add(ips[i + 1]);
                                ips.RemoveRange(i, 2);
                                found = true;
                                break;
                            }
                            else
                            {
                                currentList.Add(ips[i]);
                                currentList.Add(ips[i - 1]);
                                ips.RemoveRange(i - 1, 2);
                                found = true;
                                break;
                            }
                        }
                    }
                    if (!found)
                    {
                        curveParts.Add(currentList);
                        currentList = new List<GeoPoint3d2d2d>();
                        currentList.Add(ips[0]);
                        currentList.Add(ips[1]);
                        ips.RemoveRange(0, 2);
                    }
                }
                curveParts.Add(currentList); // now contains all pieces
                bool concatenated = true;
                while (curveParts.Count > 1 && concatenated)
                {   // if two grid lines of the two uv subdivisions almost intersect, there can be a gap. This is closed here
                    // this is unattractive, it could probably be solved if the intersection points of the grid lines with the respective other surface were determined first (uInts etc.)
                    // and then very close points were merged
                    concatenated = false;
                    double eps = (GetRawExtent().Size + otherBS.GetRawExtent().Size) * 1e-5;
                    for (int i = 0; i < curveParts.Count; i++)
                    {
                        for (int j = 0; j < curveParts.Count; j++)
                        {
                            if (i != j)
                            {
                                if ((curveParts[i][0].p3d | curveParts[j][0].p3d) < eps)
                                {
                                    curveParts[i].Reverse();
                                    curveParts[i].AddRange(curveParts[j]);
                                    curveParts.RemoveAt(j);
                                    concatenated = true;
                                    break;
                                }
                                if ((curveParts[i][curveParts[i].Count - 1].p3d | curveParts[j][0].p3d) < eps)
                                {
                                    curveParts[i].AddRange(curveParts[j]);
                                    curveParts.RemoveAt(j);
                                    concatenated = true;
                                    break;
                                }
                                if ((curveParts[i][0].p3d | curveParts[j][curveParts[j].Count - 1].p3d) < eps)
                                {
                                    curveParts[i].InsertRange(0, curveParts[j]);
                                    curveParts.RemoveAt(j);
                                    concatenated = true;
                                    break;
                                }
                                if ((curveParts[i][curveParts[i].Count - 1].p3d | curveParts[j][curveParts[j].Count - 1].p3d) < eps)
                                {
                                    curveParts[j].Reverse();
                                    curveParts[i].AddRange(curveParts[j]);
                                    curveParts.RemoveAt(j);
                                    concatenated = true;
                                    break;
                                }
                            }
                        }
                        if (concatenated) break;
                    }
                }
                List<ICurve> res = new List<GeoObject.ICurve>();
                for (int i = 0; i < curveParts.Count; i++)
                {
                    List<GeoPoint> pnts = new List<GeoPoint>();
                    for (int j = 0; j < curveParts[i].Count; j++)
                    {
                        if (j == 0 || !Precision.IsEqual(pnts[pnts.Count - 1], curveParts[i][j].p3d))
                            pnts.Add(curveParts[i][j].p3d);
                    }
                    if (pnts.Count > 1) res.Add(new InterpolatedDualSurfaceCurve(surface, thisBounds, other, otherBounds, pnts));
                }
                return res.ToArray();
            }
            return new ICurve[0];
        }

        private static List<GeoPoint3d2d2d> FindPatchIntersection(ParallelepipedHull bs1, ParallelepipedHull bs2, ParEpi pe1, ParEpi pe2, Dictionary<double, List<GeoPoint3d2d2d>> uInts1, Dictionary<double, List<GeoPoint3d2d2d>> vInts1, Dictionary<double, List<GeoPoint3d2d2d>> uInts2, Dictionary<double, List<GeoPoint3d2d2d>> vInts2, BoundingRect bounds1, BoundingRect bounds2)
        {
            // intersect the 4 edges of this patch with the other patch
            List<GeoPoint3d2d2d> res = new List<GeoPoint3d2d2d>();// uv1 is for bs1, uv2 for bs2
            List<GeoPoint3d2d2d> ips322;
            GeoPoint[] ips;
            GeoPoint2D[] uvOnFaces;
            double[] uOnCurve3Ds;
            double up1 = 0.0, vp1 = 0.0, up2 = 0.0, vp2 = 0.0; // the periods
            if (bs1.surface.IsUPeriodic) up1 = bs1.surface.UPeriod;
            if (bs1.surface.IsVPeriodic) vp1 = bs1.surface.VPeriod;
            if (bs2.surface.IsUPeriodic) up2 = bs2.surface.UPeriod;
            if (bs2.surface.IsVPeriodic) vp2 = bs2.surface.VPeriod;
            BoundingRect pe1uvPatch = pe1.uvPatch;
            BoundingRect pe2uvPatch = pe2.uvPatch;
            pe1uvPatch.Inflate(pe1uvPatch.Width * 1e-5, pe1uvPatch.Height * 1e-5);
            pe2uvPatch.Inflate(pe2uvPatch.Width * 1e-5, pe2uvPatch.Height * 1e-5);
            if (!uInts1.TryGetValue(pe1.uvPatch.Left, out ips322))
            {
                // we use the surface.Intersect method here, since the hull always iterates the same way
                bs2.surface.Intersect(bs1.surface.FixedU(pe1.uvPatch.Left, bounds1.Bottom, bounds1.Top), bounds2, out ips, out uvOnFaces, out uOnCurve3Ds);
                ips322 = new List<GeoPoint3d2d2d>();
                uInts1[pe1.uvPatch.Left] = ips322;
                for (int i = 0; i < ips.Length; i++)
                {
                    GeoPoint2D uv = bs1.surface.PositionOf(ips[i]);
                    uv.x = pe1.uvPatch.Left; // align for comparison
                    SurfaceHelper.AdjustPeriodic(bs1.surface, pe1.uvPatch, ref uv);
                    ips322.Add(new GeoPoint3d2d2d(ips[i], uv, uvOnFaces[i]));
                }
            }
            for (int i = 0; i < ips322.Count; i++)
            {
                if (pe1uvPatch.ContainsPeriodic(ips322[i].uv1, up1, vp1) && pe2uvPatch.ContainsPeriodic(ips322[i].uv2, up2, vp2)) res.Add(ips322[i]);
            }
            if (!uInts1.TryGetValue(pe1.uvPatch.Right, out ips322))
            {
                bs2.surface.Intersect(bs1.surface.FixedU(pe1.uvPatch.Right, bounds1.Bottom, bounds1.Top), bounds2, out ips, out uvOnFaces, out uOnCurve3Ds);
                ips322 = new List<GeoPoint3d2d2d>();
                uInts1[pe1.uvPatch.Right] = ips322;
                for (int i = 0; i < ips.Length; i++)
                {
                    GeoPoint2D uv = bs1.surface.PositionOf(ips[i]);
                    uv.x = pe1.uvPatch.Right; // align for comparison
                    SurfaceHelper.AdjustPeriodic(bs1.surface, pe1.uvPatch, ref uv);
                    ips322.Add(new GeoPoint3d2d2d(ips[i], uv, uvOnFaces[i]));
                }
            }
            for (int i = 0; i < ips322.Count; i++)
            {
                if (pe1uvPatch.ContainsPeriodic(ips322[i].uv1, up1, vp1) && pe2uvPatch.ContainsPeriodic(ips322[i].uv2, up2, vp2)) res.Add(ips322[i]);
            }
            if (!vInts1.TryGetValue(pe1.uvPatch.Bottom, out ips322))
            {
                bs2.surface.Intersect(bs1.surface.FixedV(pe1.uvPatch.Bottom, bounds1.Left, bounds1.Right), bounds2, out ips, out uvOnFaces, out uOnCurve3Ds);
                ips322 = new List<GeoPoint3d2d2d>();
                vInts1[pe1.uvPatch.Bottom] = ips322;
                for (int i = 0; i < ips.Length; i++)
                {
                    GeoPoint2D uv = bs1.surface.PositionOf(ips[i]);
                    uv.y = pe1.uvPatch.Bottom; // align for comparison
                    SurfaceHelper.AdjustPeriodic(bs1.surface, pe1.uvPatch, ref uv);
                    ips322.Add(new GeoPoint3d2d2d(ips[i], uv, uvOnFaces[i]));
                }
            }
            for (int i = 0; i < ips322.Count; i++)
            {
                if (pe1uvPatch.ContainsPeriodic(ips322[i].uv1, up1, vp1) && pe2uvPatch.ContainsPeriodic(ips322[i].uv2, up2, vp2)) res.Add(ips322[i]);
            }
            if (!vInts1.TryGetValue(pe1.uvPatch.Top, out ips322))
            {
                bs2.surface.Intersect(bs1.surface.FixedV(pe1.uvPatch.Top, bounds1.Left, bounds1.Right), bounds2, out ips, out uvOnFaces, out uOnCurve3Ds);
                ips322 = new List<GeoPoint3d2d2d>();
                vInts1[pe1.uvPatch.Top] = ips322;
                for (int i = 0; i < ips.Length; i++)
                {
                    GeoPoint2D uv = bs1.surface.PositionOf(ips[i]);
                    uv.y = pe1.uvPatch.Top; // align for comparison
                    SurfaceHelper.AdjustPeriodic(bs1.surface, pe1.uvPatch, ref uv);
                    ips322.Add(new GeoPoint3d2d2d(ips[i], uv, uvOnFaces[i]));
                }
            }
            for (int i = 0; i < ips322.Count; i++)
            {
                if (pe1uvPatch.ContainsPeriodic(ips322[i].uv1, up1, vp1) && pe2uvPatch.ContainsPeriodic(ips322[i].uv2, up2, vp2)) res.Add(ips322[i]);
            }
            // now with swapped roles:
            if (!uInts2.TryGetValue(pe2.uvPatch.Left, out ips322))
            {
                bs1.surface.Intersect(bs2.surface.FixedU(pe2.uvPatch.Left, bounds2.Bottom, bounds2.Top), bounds1, out ips, out uvOnFaces, out uOnCurve3Ds);
                ips322 = new List<GeoPoint3d2d2d>();
                uInts2[pe2.uvPatch.Left] = ips322;
                for (int i = 0; i < ips.Length; i++)
                {
                    GeoPoint2D uv = bs2.surface.PositionOf(ips[i]);
                    uv.x = pe2.uvPatch.Left; // align for comparison
                    SurfaceHelper.AdjustPeriodic(bs2.surface, pe2.uvPatch, ref uv);
                    ips322.Add(new GeoPoint3d2d2d(ips[i], uvOnFaces[i], uv));
                }
            }
            for (int i = 0; i < ips322.Count; i++)
            {
                if (pe1uvPatch.ContainsPeriodic(ips322[i].uv1, up1, vp1) && pe2uvPatch.ContainsPeriodic(ips322[i].uv2, up2, vp2)) res.Add(ips322[i]);
            }
            if (!uInts2.TryGetValue(pe2.uvPatch.Right, out ips322))
            {
                bs1.surface.Intersect(bs2.surface.FixedU(pe2.uvPatch.Right, bounds2.Bottom, bounds2.Top), bounds1, out ips, out uvOnFaces, out uOnCurve3Ds);
                ips322 = new List<GeoPoint3d2d2d>();
                uInts2[pe2.uvPatch.Right] = ips322;
                for (int i = 0; i < ips.Length; i++)
                {
                    GeoPoint2D uv = bs2.surface.PositionOf(ips[i]);
                    uv.x = pe2.uvPatch.Right; // align for comparison
                    SurfaceHelper.AdjustPeriodic(bs2.surface, pe2.uvPatch, ref uv);
                    ips322.Add(new GeoPoint3d2d2d(ips[i], uvOnFaces[i], uv));
                }
            }
            for (int i = 0; i < ips322.Count; i++)
            {
                if (pe1uvPatch.ContainsPeriodic(ips322[i].uv1, up1, vp1) && pe2uvPatch.ContainsPeriodic(ips322[i].uv2, up2, vp2)) res.Add(ips322[i]);
            }
            if (!vInts2.TryGetValue(pe2.uvPatch.Bottom, out ips322))
            {
                bs1.surface.Intersect(bs2.surface.FixedV(pe2.uvPatch.Bottom, bounds2.Left, bounds2.Right), bounds1, out ips, out uvOnFaces, out uOnCurve3Ds);
                ips322 = new List<GeoPoint3d2d2d>();
                vInts2[pe2.uvPatch.Bottom] = ips322;
                for (int i = 0; i < ips.Length; i++)
                {
                    GeoPoint2D uv = bs2.surface.PositionOf(ips[i]);
                    uv.y = pe2.uvPatch.Bottom; // align for comparison
                    SurfaceHelper.AdjustPeriodic(bs2.surface, pe2.uvPatch, ref uv);
                    ips322.Add(new GeoPoint3d2d2d(ips[i], uvOnFaces[i], uv));
                }
            }
            for (int i = 0; i < ips322.Count; i++)
            {
                GeoPoint2D dbg = bs2.surface.PositionOf(ips322[i].p3d);
                if (pe1uvPatch.ContainsPeriodic(ips322[i].uv1, up1, vp1) && pe2uvPatch.ContainsPeriodic(ips322[i].uv2, up2, vp2)) res.Add(ips322[i]);
            }
            if (!vInts2.TryGetValue(pe2.uvPatch.Top, out ips322))
            {
                bs1.surface.Intersect(bs2.surface.FixedV(pe2.uvPatch.Top, bounds2.Left, bounds2.Right), bounds1, out ips, out uvOnFaces, out uOnCurve3Ds);
                ips322 = new List<GeoPoint3d2d2d>();
                vInts2[pe2.uvPatch.Top] = ips322;
                for (int i = 0; i < ips.Length; i++)
                {
                    GeoPoint2D uv = bs2.surface.PositionOf(ips[i]);
                    uv.y = pe2.uvPatch.Top; // align for comparison
                    SurfaceHelper.AdjustPeriodic(bs2.surface, pe2.uvPatch, ref uv);
                    ips322.Add(new GeoPoint3d2d2d(ips[i], uvOnFaces[i], uv));
                }
            }
            for (int i = 0; i < ips322.Count; i++)
            {
                if (pe1uvPatch.ContainsPeriodic(ips322[i].uv1, up1, vp1) && pe2uvPatch.ContainsPeriodic(ips322[i].uv2, up2, vp2)) res.Add(ips322[i]);
            }
#if DEBUGx
            Face fc1 = Face.MakeFace(bs1.surface, new SimpleShape(Border.MakeRectangle(pe1.uvPatch)));
            Face fc2 = Face.MakeFace(bs2.surface, new SimpleShape(Border.MakeRectangle(pe2.uvPatch)));
            DebuggerContainer dc = new DebuggerContainer();
            dc.Add(fc1);
            dc.Add(fc2);
            for (int i = 0; i < res.Count; i++)
            {
                dc.Add(res[i].p3d, Color.Blue, i);
            }
#endif
            // in res one would have to remove identical ones, those that fall exactly on corners of the patches
            double prec = (pe1.Size + pe2.Size) * 1e-5; // changed to 1e-7, because too much was removed. The result then had a gap, very bad for BRep! Is this needed at all?
            for (int i = res.Count - 1; i > 0; --i)
            {
                for (int j = 0; j < i; j++)
                {
                    if ((res[i].p3d | res[j].p3d) < prec)
                    {
                        res.RemoveAt(i);
                        break;
                    }
                }
            }
            if (res.Count == 0 || res.Count == 2) return res;
            if (res.Count == 1)
            {   // grazed exactly one corner
                res.Clear();
                return res;
            }
            // if there are more than 2, then they should be sorted accordingly. Each 2 consecutive ones are a connected curve segment
            // through the recursive call only packets of 2 are ever returned, even if several, but those are then sorted
            // tangencies cause problems!!!
            ParEpi[] sub1 = bs1.SubCubes(pe1);
            ParEpi[] sub2 = bs2.SubCubes(pe2);
            res.Clear();
            for (int i = 0; i < sub1.Length; i++)
            {
                for (int j = 0; j < sub2.Length; j++)
                {
                    if (sub1[i].Interferes(sub2[j]))
                    {
                        res.AddRange(FindPatchIntersection(bs1, bs2, sub1[i], sub2[j], uInts1, vInts1, uInts2, vInts2, bounds1, bounds2));
                    }
                }
            }
            return res;
        }
        internal GeoPoint2D[] PositionOfNormal(GeoVector normal)
        {
            List<GeoPoint2D> res = new List<GeoPoint2D>();
            foreach (ParEpi pe in octtree.GetAllObjects())
            {
                bool check = false; // check, whether normal is in the span of normals of this patch
                GeoVector span = Geometry.ReBase(normal, pe.nll, pe.nlr, pe.nur);
                if ((span.x >= 0 && span.y >= 0 && span.z >= 0) || (span.x <= 0 && span.y <= 0 && span.z <= 0)) check = true;
                if (!check)
                {
                    span = Geometry.ReBase(normal, pe.nll, pe.nur, pe.nul);
                    if ((span.x >= 0 && span.y >= 0 && span.z >= 0) || (span.x <= 0 && span.y <= 0 && span.z <= 0)) check = true;
                }
                if (check)
                {
                    // to implement: use GaussNewtonMinimizer to calculate a uv value, where normal is normal to the surface
                }
            }
            return res.ToArray();
        }

        internal ParEpi[] FindParEpis(IOctTreeInsertable t)
        {
            return octtree.GetObjectsCloseTo(t);
        }
    }
}
