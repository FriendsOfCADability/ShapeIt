using CADability.Attribute;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.Substitutes;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace CADability
{
    /// <summary>
    /// A lightweight indexed triangle mesh as read from an STL file. Vertices are welded by exact coordinate
    /// identity (STL files written from CAD systems repeat identical coordinates for adjacent triangles).
    /// After <see cref="Finish"/> has been called, triangle adjacency (via shared edges), triangle normals and
    /// triangle areas are available. No BRep objects (Face, Edge, Vertex) are created on this level, which keeps
    /// even large STL files manageable.
    /// </summary>
    public class StlTriangleMesh
    {
        private readonly List<GeoPoint> vertices = new List<GeoPoint>();
        private readonly List<int> triangleVertices = new List<int>(); // three vertex indices per triangle
        private readonly Dictionary<(double, double, double), int> vertexMap = new Dictionary<(double, double, double), int>();
        private int[] neighbors; // three entries per triangle: the triangle sharing the edge (corner i, corner i+1), -1 if there is none
        private GeoVector[] normals; // normalized normal of each triangle
        private double[] areas; // area of each triangle
        private BoundingBox extent = BoundingBox.EmptyBoundingBox;

        public int TriangleCount => triangleVertices.Count / 3;
        public int VertexCount => vertices.Count;
        public BoundingBox Extent => extent;
        public GeoPoint GetVertex(int vertexIndex) { return vertices[vertexIndex]; }
        public int GetTriangleVertex(int triangle, int corner) { return triangleVertices[3 * triangle + corner]; }
        public GeoPoint GetTrianglePoint(int triangle, int corner) { return vertices[triangleVertices[3 * triangle + corner]]; }
        /// <summary>normalized normal of the triangle (orientation follows the vertex order)</summary>
        public GeoVector GetNormal(int triangle) { return normals[triangle]; }
        public double GetArea(int triangle) { return areas[triangle]; }
        /// <summary>the triangle on the other side of the edge from corner <paramref name="side"/> to corner side+1, or -1</summary>
        public int GetNeighbor(int triangle, int side) { return neighbors[3 * triangle + side]; }

        /// <summary>
        /// Adds a triangle. The vertex order determines the orientation (normal = (p2-p1)^(p3-p1)).
        /// Degenerate triangles are ignored (returns false).
        /// </summary>
        public bool AddTriangle(GeoPoint p1, GeoPoint p2, GeoPoint p3)
        {
            GeoVector normal = (p2 - p1) ^ (p3 - p1);
            double maxEdge = Math.Max((p1 | p2), Math.Max((p2 | p3), (p3 | p1)));
            if (normal.Length < 1e-10 * maxEdge * maxEdge) return false; // degenerate (zero area) triangle
            int i1 = GetOrAddVertex(p1);
            int i2 = GetOrAddVertex(p2);
            int i3 = GetOrAddVertex(p3);
            if (i1 == i2 || i2 == i3 || i3 == i1) return false; // degenerate after welding
            triangleVertices.Add(i1);
            triangleVertices.Add(i2);
            triangleVertices.Add(i3);
            extent.MinMax(p1);
            extent.MinMax(p2);
            extent.MinMax(p3);
            return true;
        }

        private int GetOrAddVertex(GeoPoint p)
        {
            (double, double, double) key = (p.x, p.y, p.z);
            if (!vertexMap.TryGetValue(key, out int index))
            {
                index = vertices.Count;
                vertices.Add(p);
                vertexMap.Add(key, index);
            }
            return index;
        }

        /// <summary>
        /// Computes triangle adjacency, normals and areas. Must be called once after all triangles have been added.
        /// </summary>
        public void Finish()
        {
            int n = TriangleCount;
            neighbors = new int[3 * n];
            normals = new GeoVector[n];
            areas = new double[n];
            for (int i = 0; i < neighbors.Length; i++) neighbors[i] = -1;
            Dictionary<(int, int), int> openEdges = new Dictionary<(int, int), int>(); // edge -> triangle*3+side waiting for its partner
            for (int tri = 0; tri < n; tri++)
            {
                GeoPoint p1 = GetTrianglePoint(tri, 0);
                GeoPoint p2 = GetTrianglePoint(tri, 1);
                GeoPoint p3 = GetTrianglePoint(tri, 2);
                GeoVector nrm = (p2 - p1) ^ (p3 - p1);
                areas[tri] = nrm.Length / 2.0;
                normals[tri] = nrm.Normalized;
                for (int side = 0; side < 3; side++)
                {
                    int v1 = GetTriangleVertex(tri, side);
                    int v2 = GetTriangleVertex(tri, (side + 1) % 3);
                    (int, int) key = v1 < v2 ? (v1, v2) : (v2, v1);
                    if (openEdges.TryGetValue(key, out int other))
                    {
                        neighbors[3 * tri + side] = other / 3;
                        neighbors[other] = tri;
                        openEdges.Remove(key); // in a manifold mesh each edge is shared by exactly two triangles
                    }
                    else
                    {
                        openEdges[key] = 3 * tri + side;
                    }
                }
            }
        }
#if DEBUG
        /// <summary>
        /// Show all edges of the mesh as lines for debugging purposes. Each edge is shown twice.
        /// </summary>
        DebuggerContainer DebugEdges
        {
            get
            {
                DebuggerContainer res = new DebuggerContainer();
                for (int tri = 0; tri < triangleVertices.Count; tri += 3)
                {
                    res.Add(Line.TwoPoints(vertices[triangleVertices[tri]], vertices[triangleVertices[tri + 1]]));
                    res.Add(Line.TwoPoints(vertices[triangleVertices[tri + 1]], vertices[triangleVertices[tri + 2]]));
                    res.Add(Line.TwoPoints(vertices[triangleVertices[tri + 2]], vertices[triangleVertices[tri]]));
                }
                return res;
            }
        }
#endif
    }

    public enum RecognizedSurfaceKind { Unrecognized, Plane, Cylinder, Cone, Sphere, Torus, Nurbs }

    /// <summary>
    /// A connected set of mesh triangles together with the recognized surface (or null if no standard surface fits).
    /// </summary>
    [DebuggerDisplayAttribute("{DebugString}")]

    public class RecognizedRegion
    {
        public StlTriangleMesh mesh; // back reference to the mesh
        public List<int> Triangles;
        public ISurface Surface; // null if unrecognized
        private BoundingRect extent = BoundingRect.EmptyBoundingRect;
        public BoundingRect Extent
        {
            get
            {
                if (extent.IsEmpty())
                {
                    foreach (int tri in Triangles)
                    {
                        for (int corner = 0; corner < 3; corner++)
                        {
                            extent.MinMax(Surface.PositionOf(mesh.GetTrianglePoint(tri, corner)));
                        }
                    }
                }
                return extent;
            }
        }
        /// <summary>Discards the cached <see cref="Extent"/> after the triangle set has been changed.</summary>
        public void InvalidateExtent() { extent = BoundingRect.EmptyBoundingRect; }
        public RecognizedSurfaceKind Kind;
        public double MaxError; // maximum distance of the (sampled) region vertices from Surface
#if DEBUG
        public string DebugString => $"Triangles={Triangles.Count}, Kind={Kind}, MaxError={MaxError}";
        public DebuggerContainer Debug
        {
            get
            {
                ColorDef cd = new ColorDef();
                switch (Kind)
                {
                    case RecognizedSurfaceKind.Plane: cd.Color = Color.Green; break;
                    case RecognizedSurfaceKind.Cylinder: cd.Color = Color.Blue; break;
                    case RecognizedSurfaceKind.Cone: cd.Color = Color.Orange; break;
                    case RecognizedSurfaceKind.Sphere: cd.Color = Color.Red; break;
                    case RecognizedSurfaceKind.Torus: cd.Color = Color.Violet; break;
                    default: cd.Color = Color.Gray; break;
                }
                cd.Color = Color.White;
                DebuggerContainer res = new DebuggerContainer();
                for (int j = 0; j < Triangles.Count; j++)
                {
                    Face fc = Face.MakeFace(mesh.GetTrianglePoint(Triangles[j], 0), mesh.GetTrianglePoint(Triangles[j], 1), mesh.GetTrianglePoint(Triangles[j], 2));
                    res.Add(fc);
                }
                return res;
            }
        }

        public List<GeoPoint> DebugPoints
        {
            get
            {
                HashSet<int> vertices = new HashSet<int>();
                for (int j = 0; j < Triangles.Count; j++)
                {
                    vertices.Add(mesh.GetTriangleVertex(Triangles[j], 0));
                    vertices.Add(mesh.GetTriangleVertex(Triangles[j], 1));
                    vertices.Add(mesh.GetTriangleVertex(Triangles[j], 2));
                }
                return vertices.Select(v => mesh.GetVertex(v)).ToList();
            }
        }
#endif
    }

    /// <summary>
    /// Reverse engineering of a triangle mesh (stage 1): the mesh is segmented into regions of triangles which are
    /// connected with only small bending angles. For each region a standard surface (plane, cylinder, cone, sphere,
    /// torus) is recognized: axis candidates come from the Gauss map (the triangle normals on the unit sphere), the
    /// axis position from the normal lines (which intersect the axis of any surface of revolution), the surface
    /// parameters from the profile (distance from the axis over position along the axis). Each candidate surface is
    /// constructed directly from these estimates and accepted when the region vertices are within
    /// <see cref="Tolerance"/>. The result can be inspected as raw faces (surface bounded by its uv extent, not by
    /// the true outline) via <see cref="CreateRawFaces"/>.
    /// </summary>
    public class StlSurfaceReconstruction
    {
        private readonly StlTriangleMesh mesh;
        private readonly double precision; // estimated coordinate resolution of the STL data
        private const int maxFitPoints = 2500; // fits and validation work on at most this many sample points per region

        /// <summary>Two adjacent triangles bending less than this angle are considered to belong to the same surface.</summary>
        public double MaxBendAngle { get; set; } = 10.0 / 180.0 * Math.PI;
        /// <summary>Maximum allowed distance of the region vertices from a fitted surface to accept the surface.</summary>
        public double Tolerance { get; set; }
        public List<RecognizedRegion> Regions { get; private set; }
        private RecognizedRegion[] regionOfTriangle; // reverse lookup, valid after Recognize()
        private Dictionary<(int, int, int, int), ICurve> edgeCurves = new Dictionary<(int, int, int, int), ICurve>(); // (first two and last two indizes) -> curve along the edge
                                                                                                                      // to identify an already constructed egde, we need 4 indizes since there could be two different edges between start- and end index.

        public StlSurfaceReconstruction(StlTriangleMesh mesh, double precision)
        {
            this.mesh = mesh;
            this.precision = precision;
            Tolerance = 10 * precision;
        }

        /// <summary>
        /// Segments the mesh and tries to recognize a standard surface for each segment. The result is available
        /// in <see cref="Regions"/>.
        /// </summary>
        public void Recognize()
        {
            Regions = new List<RecognizedRegion>();
            int[] all = new int[mesh.TriangleCount];
            for (int i = 0; i < all.Length; i++) all[i] = i;
            List<List<int>> segments = SegmentTriangles(all, Math.Cos(MaxBendAngle));
            foreach (List<int> segment in segments)
            {
#if DEBUG
                DebuggerContainer dcs = new DebuggerContainer();
                for (int i = 0; i < segment.Count; i++)
                {
                    ColorDef cd = new ColorDef("Color" + i.ToString(), Color.FromArgb(63 + (i & 0x3) * 64, 63 + ((i / 4) & 0x3) * 64, 63 + ((i / 16) & 0x3) * 64));
                    Face fc = Face.MakeFace(this.mesh.GetTrianglePoint(segment[i], 0), this.mesh.GetTrianglePoint(segment[i], 1), this.mesh.GetTrianglePoint(segment[i], 2));
                    fc.ColorDef = cd;
                    dcs.Add(fc);
                }
#endif
                RecognizeSegment(segment);
            }
            // a nearly degenerate cone (tiny opening angle, apex far away) is numerically a cylinder; the extra
            // taper freedom lets it win over the cylinder during growth by absorbing a few more border triangles.
            // Replace such a cone by a cylinder when a cylinder actually fits, before merging (so the cylinder can
            // then merge with adjacent cylinders).
            PreferCylinderOverShallowCones();
            // a narrow cylinder strip (two rows of triangles) fits a sphere just as well at the vertices (the two
            // rows are two symmetric latitudes); prefer the cylinder, which lies closer to the triangle interiors.
            PreferCylinderOverSpheres();
            // the regions are a partition of the mesh: build the reverse lookup triangle -> region, which is
            // needed to find the boundaries between the regions (the edges of the shell to be built)
            RebuildRegionOfTriangle();
            // the greedy per-seed growth over-segments a single surface (a slightly tilted seed axis makes growing
            // stall part way, so a cylinder shell or a cone can end up split into several fragments). Reunite
            // adjacent regions that describe the same surface.
            MergeAdjacentRegions();
            // a cone is fragmented into cylinders too: the apex/opening angle is undetermined from a small seed
            // patch (a cone patch reads locally as a cylinder), so cylinder seeds win and split the cone into pieces
            // (angular sectors with fanning axes). Reunite adjacent cylinder pieces whose union is a single cone.
            MergeCylindersToCones();
            // final cleanup: the greedy growth lets the surface processed first absorb border triangles that
            // geometrically belong to the neighbor (a plane running tangentially into a cone eats the cone's
            // border triangles). Reassign such triangles to the region they fit better.
            RefineRegionBoundaries();
            // the regions are now a stable partition; whatever is still unrecognized is a freeform patch. Turn the
            // ones separated from their neighbors by a crease into freeform NURBS surfaces (they may overshoot the
            // border, the later surface intersection trims them to clean edges).
            FitNurbsToUnrecognizedRegions();
#if DEBUG
            DebuggerContainer dc = new DebuggerContainer();
            for (int i = 0; i < Regions.Count; i++)
            {
                ColorDef cd = new ColorDef("Color" + i.ToString(), Color.FromArgb((i & 0x3) * 64, ((i / 4) & 0x3) * 64, ((i / 16) & 0x3) * 64));
                for (int j = 0; j < Regions[i].Triangles.Count; j++)
                {
                    Face fc = Face.MakeFace(this.mesh.GetTrianglePoint(Regions[i].Triangles[j], 0), this.mesh.GetTrianglePoint(Regions[i].Triangles[j], 1), this.mesh.GetTrianglePoint(Regions[i].Triangles[j], 2));
                    fc.ColorDef = cd;
                    dc.Add(fc);
                }
            }
#endif
        }

        /// <summary>
        /// Replaces shallow cones (small opening angle) by a cylinder where a cylinder actually fits the region
        /// within <see cref="Tolerance"/> and no worse than the cone. A cone with a tiny opening angle has its apex
        /// far away and is numerically indistinguishable from a cylinder; the simpler cylinder is preferred. The
        /// opening angle only decides whether to test - the cylinder fit itself is the criterion, so a genuine
        /// shallow draft cone (where a cylinder does not fit) is kept.
        /// </summary>
        private void PreferCylinderOverShallowCones()
        {
            const double shallowAngle = 0.15; // radians (~8.6 degrees)
            for (int i = 0; i < Regions.Count; i++)
            {
                RecognizedRegion r = Regions[i];
                if (r.Kind != RecognizedSurfaceKind.Cone || !(r.Surface is ConicalSurface cone)) continue;
                if (cone.OpeningAngle.Radian >= shallowAngle) continue;
                double saved = Tolerance;
                Tolerance = double.MaxValue; // fit without the gate, we compare the error ourselves
                RecognizedRegion cyl = FitKind(RecognizedSurfaceKind.Cylinder, r.Triangles);
                Tolerance = saved;
                if (cyl != null && cyl.MaxError <= Tolerance && cyl.MaxError <= r.MaxError) Regions[i] = cyl;
            }
        }

        /// <summary>
        /// Replaces a sphere by a cylinder where the region is a narrow strip whose points lie on essentially only
        /// two parallel circles of equal radius. Two such circles pass exactly through both a sphere (as two
        /// symmetric latitudes) and a cylinder (as two cross sections), so the vertices alone cannot tell them apart
        /// - but the cylinder lies closer to the triangle interiors (the sphere bulges out between the circles), so
        /// it is the better explanation. The cylinder must fit within <see cref="Tolerance"/> (which requires equal
        /// radii) and the profile must have at most two rows (only two circles); a genuine spherical cap has points
        /// on many circles, so a cylinder does not fit it and it is kept.
        /// </summary>
        private void PreferCylinderOverSpheres()
        {
            for (int i = 0; i < Regions.Count; i++)
            {
                RecognizedRegion r = Regions[i];
                if (r.Kind != RecognizedSurfaceKind.Sphere) continue;
                double saved = Tolerance;
                Tolerance = double.MaxValue; // fit without the gate, we judge by the tolerance and row count ourselves
                RecognizedRegion cyl = FitKind(RecognizedSurfaceKind.Cylinder, r.Triangles);
                Tolerance = saved;
                if (cyl == null || cyl.MaxError > Tolerance || !(cyl.Surface is CylindricalSurface cs)) continue;
                GeoPoint[] pnts = RegionPoints(r.Triangles);
                BuildProfile(pnts, cs.Location, cs.Axis, out double[] t, out double[] d);
                if (CountProfileRows(t, d) <= 2) Regions[i] = cyl;
            }
        }

        private void RebuildRegionOfTriangle()
        {
            regionOfTriangle = new RecognizedRegion[mesh.TriangleCount];
            foreach (RecognizedRegion region in Regions)
            {
                foreach (int tri in region.Triangles) regionOfTriangle[tri] = region;
            }
        }

        /// <summary>
        /// The region the triangle belongs to. Only valid after <see cref="Recognize"/> has been called; the
        /// regions are a partition of the mesh, so every triangle belongs to exactly one region.
        /// </summary>
        public RecognizedRegion GetRegion(int triangle)
        {
            return regionOfTriangle[triangle];
        }

        #region merging adjacent regions
        /// <summary>
        /// Maximum distance of the region vertices from the merged surface for a merge to be accepted. Slightly
        /// more generous than <see cref="Tolerance"/>, because reconciling the axes of two fragments into one
        /// surface can make the combined fit a little worse than each fragment on its own.
        /// </summary>
        public double MergeTolerance { get; set; }

        /// <summary>
        /// Reunites neighboring regions that describe the same standard surface. Two adjacent regions of the same
        /// kind are merged when their surface parameters are roughly compatible (a cheap, tolerant pre-check) and a
        /// single surface, refitted over the union of their triangles, stays within <see cref="MergeTolerance"/>
        /// for all vertices. This is order independent (unlike the greedy growth) and therefore recovers a full
        /// surface even when the growth fragmented it. Repeated until no more merges happen.
        /// </summary>
        public void MergeAdjacentRegions()
        {
            if (MergeTolerance <= 0.0) MergeTolerance = 2.0 * Tolerance;
            bool changed = true;
            while (changed)
            {
                changed = false;
                Dictionary<RecognizedRegion, HashSet<RecognizedRegion>> adjacency = BuildRegionAdjacency();
                foreach (KeyValuePair<RecognizedRegion, HashSet<RecognizedRegion>> entry in adjacency)
                {
                    RecognizedRegion a = entry.Key;
                    if (a.Surface == null) continue;
                    foreach (RecognizedRegion b in entry.Value)
                    {
                        // Merge candidates: same kind, an unrecognized leftover piece, or - across kinds - a pair
                        // where one side is a plane. A small fragment of a curved surface is locally flat and often
                        // gets recognized as a plane although its points lie on the neighbor's cylinder/cone; such a
                        // plane is absorbed. Cross-kind merges between two curved surfaces (e.g. sphere and cylinder)
                        // are NOT allowed, so a genuine small curved feature next to a larger surface is preserved.
                        // TryMergeRegions decides by validating the union against the actual surfaces, not their
                        // parameters, so no further pre-check is needed here.
                        bool sameKind = b.Surface != null && b.Kind == a.Kind;
                        bool unrecognized = b.Surface == null;
                        bool planeInvolved = a.Kind == RecognizedSurfaceKind.Plane || (b.Surface != null && b.Kind == RecognizedSurfaceKind.Plane);
                        if (!sameKind && !unrecognized && !planeInvolved) continue;
                        RecognizedRegion merged = TryMergeRegions(a, b);
                        if (merged == null) continue;
                        Regions.Remove(a);
                        Regions.Remove(b);
                        Regions.Add(merged);
                        RebuildRegionOfTriangle();
                        changed = true;
                        break;
                    }
                    if (changed) break; // adjacency is stale after a merge, rebuild it
                }
            }
        }

        /// <summary>region -> set of neighboring regions (sharing at least one triangle edge)</summary>
        private Dictionary<RecognizedRegion, HashSet<RecognizedRegion>> BuildRegionAdjacency()
        {
            Dictionary<RecognizedRegion, HashSet<RecognizedRegion>> adjacency = new Dictionary<RecognizedRegion, HashSet<RecognizedRegion>>();
            foreach (RecognizedRegion region in Regions)
            {
                HashSet<RecognizedRegion> neighbors = new HashSet<RecognizedRegion>();
                foreach (var edge in GetBoundaryEdges(region))
                {
                    if (edge.neighbor != null && edge.neighbor != region) neighbors.Add(edge.neighbor);
                }
                adjacency[region] = neighbors;
            }
            return adjacency;
        }

        /// <summary>
        /// Cheap, tolerant compatibility pre-check for two regions of the same kind: do their surface parameters
        /// roughly agree? This only filters obviously unrelated neighbors; the real decision is made by
        /// <see cref="TryMergeRegions"/> which refits and validates one surface over the union.
        /// </summary>
        private bool SurfacesCompatible(RecognizedRegion a, RecognizedRegion b)
        {
            const double cosDir = 0.98; // axis/normal directions within ~11 degrees
            double size = mesh.Extent.Size;
            double posTol = 0.05 * size + 10.0 * precision; // generous position tolerance
            switch (a.Kind)
            {
                case RecognizedSurfaceKind.Plane:
                    {
                        PlaneSurface pa = (PlaneSurface)a.Surface, pb = (PlaneSurface)b.Surface;
                        if (Math.Abs(pa.Normal.Normalized * pb.Normal.Normalized) < cosDir) return false;
                        return Math.Abs(pa.Plane.Distance(pb.Location)) < posTol; // roughly coplanar
                    }
                case RecognizedSurfaceKind.Cylinder:
                    {
                        CylindricalSurface ca = (CylindricalSurface)a.Surface, cb = (CylindricalSurface)b.Surface;
                        if (Math.Abs(ca.Axis.Normalized * cb.Axis.Normalized) < cosDir) return false;
                        if (Math.Abs(ca.RadiusX - cb.RadiusX) > 0.1 * Math.Max(ca.RadiusX, cb.RadiusX) + posTol) return false;
                        return Geometry.DistPL(cb.Location, ca.Location, ca.Axis) < 0.1 * ca.RadiusX + posTol; // axis lines close
                    }
                case RecognizedSurfaceKind.Cone:
                    {
                        ConicalSurface ka = (ConicalSurface)a.Surface, kb = (ConicalSurface)b.Surface;
                        if (Math.Abs(ka.Axis.Normalized * kb.Axis.Normalized) < cosDir) return false;
                        if (Math.Abs(ka.OpeningAngle.Radian - kb.OpeningAngle.Radian) > 0.2) return false; // ~11 degrees
                        return (ka.Location | kb.Location) < 0.25 * size + posTol; // apexes roughly coincide
                    }
                case RecognizedSurfaceKind.Sphere:
                    {
                        SphericalSurface sa = (SphericalSurface)a.Surface, sb = (SphericalSurface)b.Surface;
                        if (Math.Abs(sa.RadiusX - sb.RadiusX) > 0.1 * Math.Max(sa.RadiusX, sb.RadiusX) + posTol) return false;
                        return (sa.Location | sb.Location) < 0.1 * sa.RadiusX + posTol;
                    }
                case RecognizedSurfaceKind.Torus:
                    {
                        ToroidalSurface ta = (ToroidalSurface)a.Surface, tb = (ToroidalSurface)b.Surface;
                        if (Math.Abs(ta.ZAxis.Normalized * tb.ZAxis.Normalized) < cosDir) return false;
                        if (Math.Abs(ta.MajorRadius - tb.MajorRadius) > 0.1 * Math.Max(ta.MajorRadius, tb.MajorRadius) + posTol) return false;
                        if (Math.Abs(ta.MinorRadius - tb.MinorRadius) > 0.1 * Math.Max(ta.MinorRadius, tb.MinorRadius) + posTol) return false;
                        return (ta.Location | tb.Location) < 0.25 * size + posTol;
                    }
                default:
                    return false;
            }
        }

        /// <summary>
        /// Attempts to merge two adjacent regions into one. Accepts the merge only when a single surface stays
        /// within <see cref="MergeTolerance"/> for all union vertices. The two regions need not be the same kind: a
        /// small fragment of a curved surface is often locally flat and gets recognized as a plane, yet its points
        /// lie on the neighbor's cylinder/cone - such a plane is absorbed into the cylinder because the cylinder
        /// fits the union while the plane does not. Returns the merged region (with the winning surface and its
        /// kind), or null when no single surface fits the union (there is a real edge between them).
        /// </summary>
        private RecognizedRegion TryMergeRegions(RecognizedRegion a, RecognizedRegion b)
        {
            List<int> union = new List<int>(a.Triangles.Count + b.Triangles.Count);
            union.AddRange(a.Triangles);
            union.AddRange(b.Triangles);
            GeoPoint[] pnts = RegionPoints(union);
            // The union describes a single surface if a surface that already fits one fragment also fits the other.
            // The existing fragment surfaces are well fitted (their axis was refined incrementally while growing);
            // a from-scratch fit over the whole union is unreliable for large or strongly curved regions - a small
            // axis error blows up a cone apex - so it is only tried as an extra candidate and used when it happens
            // to be better. Validate against the existing surfaces and keep the one with the smallest union error.
            ISurface best = null;
            double bestError = double.MaxValue;
            RecognizedSurfaceKind bestKind = a.Surface != null ? a.Kind : b.Kind;
            void consider(ISurface s, RecognizedSurfaceKind k)
            {
                if (s == null) return;
                double e = MaxDistance(s, pnts);
                if (e < bestError) { bestError = e; best = s; bestKind = k; }
            }
            // Cheap path first: if one fragment's (already well fitted) surface fits the whole union, merge with it.
            // This covers the common case (two pieces of the same surface) and the cross-kind absorption above with
            // just distance evaluations, no fit.
            if (a.Surface != null) consider(a.Surface, a.Kind);
            if (b.Surface != null) consider(b.Surface, b.Kind);
            if (best == null || bestError > MergeTolerance)
            {
                // Neither existing surface fits. A from-scratch fit over the union can still find a common surface,
                // but it is unreliable for large curved regions (its single-shot axis estimate can blow up), so it
                // is only a fallback and only used when it actually fits. Restrict it to cases where it makes sense:
                // an unrecognized leftover piece absorbed into a's surface, or two compatible same-kind fragments
                // reconciled into one. It runs rarely (only for pairs the cheap path could not merge), keeping the
                // merge fast.
                RecognizedSurfaceKind fitKind = a.Surface != null ? a.Kind : b.Kind;
                bool doFallback = (b.Surface == null)
                    || (a.Surface != null && b.Surface != null && a.Kind == b.Kind && SurfacesCompatible(a, b));
                if (doFallback)
                {
                    double savedTolerance = Tolerance;
                    Tolerance = double.MaxValue; // no tolerance gate, we compare the error ourselves
                    RecognizedRegion fresh = FitKind(fitKind, union);
                    Tolerance = savedTolerance;
                    if (fresh != null) consider(fresh.Surface, fitKind);
                }
            }
            if (best == null || bestError > MergeTolerance) return null;
            return new RecognizedRegion { mesh = this.mesh, Triangles = union, Surface = best, Kind = bestKind, MaxError = bestError };
        }
        #endregion

        #region cylinder to cone merging
        /// <summary>An opening angle (full apex angle) below this is treated as a degenerate, near-cylindrical cone.</summary>
        private const double shallowConeAngle = 0.15;

        /// <summary>
        /// Reunites cylinder pieces that together form a single cone. A cone patch reads locally as a cylinder (its
        /// apex/opening angle needs a wide support), so the growth splits a cone into cylinder fragments. Adjacent
        /// cylinder (or already merged cone) regions are merged whenever a single cone, fitted over their union,
        /// stays within <see cref="MergeTolerance"/> and is not degenerate (opening angle above
        /// <see cref="shallowConeAngle"/>, the same threshold that <see cref="PreferCylinderOverShallowCones"/> uses
        /// in the opposite direction). The cone-fits-the-union test is the decision: a genuine stepped shaft or a
        /// cylinder merely tangent to the cone does not fit a common cone and is left alone. Repeated until no more
        /// merges happen, so a cone grows by absorbing one adjacent cylinder after another.
        /// </summary>
        public void MergeCylindersToCones()
        {
            if (MergeTolerance <= 0.0) MergeTolerance = 2.0 * Tolerance;
            bool changed = true;
            while (changed)
            {
                changed = false;
                Dictionary<RecognizedRegion, HashSet<RecognizedRegion>> adjacency = BuildRegionAdjacency();
                foreach (KeyValuePair<RecognizedRegion, HashSet<RecognizedRegion>> entry in adjacency)
                {
                    RecognizedRegion a = entry.Key;
                    if (a.Kind != RecognizedSurfaceKind.Cylinder && a.Kind != RecognizedSurfaceKind.Cone) continue;
                    foreach (RecognizedRegion b in entry.Value)
                    {
                        if (b.Kind != RecognizedSurfaceKind.Cylinder && b.Kind != RecognizedSurfaceKind.Cone) continue;
                        // cylinder+cylinder, cylinder+cone and cone+cone are all tried: a cone fragmented into pieces
                        // (cylinders or several cone patches) is reunited. Genuinely different cones are not fused
                        // because a single cone would not fit their union within the tolerance (the gate below).
                        RecognizedRegion merged = TryMergeToCone(a, b);
                        if (merged == null) continue;
                        Regions.Remove(a);
                        Regions.Remove(b);
                        Regions.Add(merged);
                        RebuildRegionOfTriangle();
                        changed = true;
                        break;
                    }
                    if (changed) break; // adjacency is stale after a merge
                }
            }
        }

        /// <summary>
        /// Tries to merge two adjacent regions into a single cone. Fits a cone over the union (seeded from the
        /// existing cone recognition with the tolerance gate lifted, then refined with <see cref="FitConeLM"/>) and
        /// accepts it when it stays within <see cref="MergeTolerance"/> and is not a degenerate near-cylinder.
        /// Returns the merged cone region, or null.
        /// </summary>
        private RecognizedRegion TryMergeToCone(RecognizedRegion a, RecognizedRegion b)
        {
            List<int> union = new List<int>(a.Triangles.Count + b.Triangles.Count);
            union.AddRange(a.Triangles);
            union.AddRange(b.Triangles);
            GeoPoint[] pnts = RegionPoints(union);
            double savedTolerance = Tolerance;
            Tolerance = double.MaxValue; // seed without the gate, we judge by MergeTolerance after the refine
            RecognizedRegion seed = FitKind(RecognizedSurfaceKind.Cone, union);
            Tolerance = savedTolerance;
            if (seed == null || !(seed.Surface is ConicalSurface sc)) return null;
            ConicalSurface refined = FitConeLM(pnts, sc.Location, sc.Axis, sc.OpeningAngle.Radian / 2.0) ?? sc;
            double err = MaxDistance(refined, pnts);
            if (err > MergeTolerance) return null;
            if (refined.OpeningAngle.Radian < shallowConeAngle) return null; // near-cylinder: keep the cylinders
            return new RecognizedRegion { mesh = this.mesh, Triangles = union, Surface = refined, Kind = RecognizedSurfaceKind.Cone, MaxError = err };
        }

        /// <summary>
        /// Least squares cone fit with MathNet's Levenberg-Marquardt minimizer, refining apex (3), axis direction
        /// (3, normalized in the residual) and semi angle (1). The residual is the perpendicular distance of a point
        /// to the cone ruling in its meridian plane, rho*cos(a) - t*sin(a) (t along the axis, rho from the axis),
        /// which is zero on the cone. The axis is oriented so the points lie on the t &gt; 0 nappe. Returns null on
        /// failure or a degenerate result.
        /// </summary>
        private ConicalSurface FitConeLM(GeoPoint[] pnts, GeoPoint apex, GeoVector axisDir, double semiAngle)
        {
            GeoVector Z0 = axisDir.Normalized;
            double meanT = 0.0; foreach (GeoPoint p in pnts) meanT += (p - apex) * Z0;
            if (meanT < 0) Z0 = -Z0; // put the points on the t > 0 side of the apex
            MathNet.Numerics.LinearAlgebra.Vector<double> model(MathNet.Numerics.LinearAlgebra.Vector<double> par, MathNet.Numerics.LinearAlgebra.Vector<double> x)
            {
                GeoPoint A = new GeoPoint(par[0], par[1], par[2]);
                GeoVector n = new GeoVector(par[3], par[4], par[5]);
                double L = n.Length;
                double alpha = par[6];
                double[] res = new double[pnts.Length];
                if (L > 1e-9 && alpha > 1e-6 && alpha < Math.PI / 2 - 1e-6)
                {
                    GeoVector nhat = (1.0 / L) * n;
                    double ca = Math.Cos(alpha), sa = Math.Sin(alpha);
                    for (int i = 0; i < pnts.Length; i++)
                    {
                        GeoVector v = pnts[i] - A;
                        double t = v * nhat;
                        double rho2 = v * v - t * t;
                        double rho = rho2 > 0 ? Math.Sqrt(rho2) : 0.0;
                        res[i] = rho * ca - t * sa; // signed distance to the cone surface
                    }
                }
                return MathNet.Numerics.LinearAlgebra.CreateVector.Dense(res);
            }
            try
            {
                MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer minimizer =
                    new MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer(0.001, 1e-18, 1e-18, 1e-18, 100);
                var obj = MathNet.Numerics.Optimization.ObjectiveFunction.NonlinearModel(model,
                    MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(pnts.Length),
                    MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(pnts.Length));
                var start = MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(new double[] { apex.x, apex.y, apex.z, Z0.x, Z0.y, Z0.z, semiAngle });
                var result = minimizer.FindMinimum(obj, start);
                var pr = result.MinimizingPoint;
                GeoVector axis = new GeoVector(pr[3], pr[4], pr[5]);
                double al = pr[6];
                if (axis.Length < 1e-9 || al <= 1e-4 || al >= Math.PI / 2 - 1e-4) return null;
                GeoVector ax = axis.Normalized;
                ax.ArbitraryNormals(out GeoVector dx, out GeoVector dy);
                return new ConicalSurface(new GeoPoint(pr[0], pr[1], pr[2]), dx, dy, ax, al);
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Least squares cylinder fit with MathNet's Levenberg-Marquardt minimizer, refining a point on the axis
        /// (3), the axis direction (3, normalized in the residual) and the radius (1). The residual is the signed
        /// distance to the surface, rho - R (rho = distance from the axis). Returns null on failure/degeneracy.
        /// </summary>
        private CylindricalSurface FitCylinderLM(GeoPoint[] pnts, GeoPoint axisPoint, GeoVector axisDir, double radius)
        {
            MathNet.Numerics.LinearAlgebra.Vector<double> model(MathNet.Numerics.LinearAlgebra.Vector<double> par, MathNet.Numerics.LinearAlgebra.Vector<double> x)
            {
                GeoPoint A = new GeoPoint(par[0], par[1], par[2]);
                GeoVector n = new GeoVector(par[3], par[4], par[5]);
                double L = n.Length;
                double R = par[6];
                double[] res = new double[pnts.Length];
                if (L > 1e-9)
                {
                    GeoVector nhat = (1.0 / L) * n;
                    for (int i = 0; i < pnts.Length; i++)
                    {
                        GeoVector v = pnts[i] - A;
                        double t = v * nhat;
                        double rho2 = v * v - t * t;
                        res[i] = (rho2 > 0 ? Math.Sqrt(rho2) : 0.0) - R;
                    }
                }
                return MathNet.Numerics.LinearAlgebra.CreateVector.Dense(res);
            }
            try
            {
                GeoVector n0 = axisDir.Normalized;
                MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer minimizer =
                    new MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer(0.001, 1e-18, 1e-18, 1e-18, 100);
                var obj = MathNet.Numerics.Optimization.ObjectiveFunction.NonlinearModel(model,
                    MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(pnts.Length),
                    MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(pnts.Length));
                var start = MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(new double[] { axisPoint.x, axisPoint.y, axisPoint.z, n0.x, n0.y, n0.z, radius });
                var result = minimizer.FindMinimum(obj, start);
                var pr = result.MinimizingPoint;
                GeoVector axis = new GeoVector(pr[3], pr[4], pr[5]);
                double R = pr[6];
                if (axis.Length < 1e-9 || R <= precision) return null;
                GeoVector ax = axis.Normalized;
                ax.ArbitraryNormals(out GeoVector dx, out GeoVector dy);
                return new CylindricalSurface(new GeoPoint(pr[0], pr[1], pr[2]), R * dx, R * dy, ax);
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Least squares sphere fit with MathNet's Levenberg-Marquardt minimizer, refining the center (3) and the
        /// radius (1). The residual is the signed distance to the surface, |P - center| - R. Returns null on failure.
        /// </summary>
        private SphericalSurface FitSphereLM(GeoPoint[] pnts, GeoPoint center, double radius)
        {
            MathNet.Numerics.LinearAlgebra.Vector<double> model(MathNet.Numerics.LinearAlgebra.Vector<double> par, MathNet.Numerics.LinearAlgebra.Vector<double> x)
            {
                GeoPoint c = new GeoPoint(par[0], par[1], par[2]);
                double R = par[3];
                double[] res = new double[pnts.Length];
                for (int i = 0; i < pnts.Length; i++) res[i] = (pnts[i] - c).Length - R;
                return MathNet.Numerics.LinearAlgebra.CreateVector.Dense(res);
            }
            try
            {
                MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer minimizer =
                    new MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer(0.001, 1e-18, 1e-18, 1e-18, 100);
                var obj = MathNet.Numerics.Optimization.ObjectiveFunction.NonlinearModel(model,
                    MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(pnts.Length),
                    MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(pnts.Length));
                var start = MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(new double[] { center.x, center.y, center.z, radius });
                var result = minimizer.FindMinimum(obj, start);
                var pr = result.MinimizingPoint;
                double R = pr[3];
                if (R <= precision) return null;
                GeoPoint c = new GeoPoint(pr[0], pr[1], pr[2]);
                return new SphericalSurface(c, R * GeoVector.XAxis, R * GeoVector.YAxis, R * GeoVector.ZAxis);
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Refines a grown standard surface with a least squares (Levenberg-Marquardt) fit over its points after the
        /// single-shot refit. Dispatches to the fit for the surface kind; planes are already least squares optimal
        /// (PCA) and unrecognized regions have no surface, so both return null (no change). Torus is deliberately
        /// excluded: its all-parameter fit refines the axis, which overfits on a small vertex-seed patch (a
        /// degenerate horn torus with minor approx major); tori are recognized on their own robust path (the tube
        /// march in <see cref="ExtractTubeTori"/>), which is where their least squares refine belongs.
        /// </summary>
        private ISurface RefineSurfaceLM(RecognizedSurfaceKind kind, ISurface surface, GeoPoint[] pnts)
        {
            switch (kind)
            {
                case RecognizedSurfaceKind.Cylinder when surface is CylindricalSurface cy:
                    return FitCylinderLM(pnts, cy.Location, cy.Axis, cy.RadiusX);
                case RecognizedSurfaceKind.Cone when surface is ConicalSurface co:
                    return FitConeLM(pnts, co.Location, co.Axis, co.OpeningAngle.Radian / 2.0);
                case RecognizedSurfaceKind.Sphere when surface is SphericalSurface sp:
                    return FitSphereLM(pnts, sp.Location, sp.RadiusX);
                default:
                    return null;
            }
        }
        #endregion

        #region boundary refinement
        /// <summary>
        /// After the regions form a stable partition, reconsiders the triangles on the borders between two
        /// recognized regions: a triangle assigned to region A but lying closer to the surface of its neighbor B is
        /// moved to B. This repairs the greedy growth at tangential transitions, where the surface processed first
        /// (e.g. a plane running tangentially into a cone) absorbs a few border triangles that geometrically belong
        /// to the other surface. Near the tangent line such a triangle fits both surfaces within the tolerance, so
        /// the decision is made by the smaller vertex distance and, when that is ambiguous (a true tangential
        /// contact where both distances are tiny), by which region holds the majority of the triangle's edge
        /// neighbors. Every move is gated by a refit: the receiving region must still fit within
        /// <see cref="Tolerance"/> with the triangle and the donating region must still fit without it, so a move
        /// can never make the recognition worse. Only recognized regions take part; unrecognized (freeform) patches
        /// are left untouched. Repeated for a few passes (a move can enable another on an adjacent border).
        /// </summary>
        private void RefineRegionBoundaries()
        {
            const int maxPasses = 4;
            for (int pass = 0; pass < maxPasses; pass++)
            {
                // collect all cheap-valid boundary moves of this pass (largest gain first); the expensive refit
                // gate is only evaluated for the moves that are actually applied
                List<(double score, int tri, RecognizedRegion from, RecognizedRegion to)> candidates =
                    new List<(double, int, RecognizedRegion, RecognizedRegion)>();
                foreach (RecognizedRegion a in Regions)
                {
                    if (a.Surface == null) continue; // only reassign between two recognized surfaces
                    if (a.Triangles.Count <= MinTrianglesFor(a.Kind)) continue; // keep the donor large enough to survive
                    foreach (var edge in GetBoundaryEdges(a))
                    {
                        RecognizedRegion b = edge.neighbor;
                        if (b == null || b.Surface == null) continue; // no surface to move the triangle onto
                        int t = edge.triangle;
                        double dB = TriangleDistance(b.Surface, t);
                        if (dB > Tolerance) continue; // t must lie on B within the tolerance
                        double dA = TriangleDistance(a.Surface, t);
                        int nA = 0, nB = 0; // edge neighbors of t in each of the two regions
                        for (int side = 0; side < 3; side++)
                        {
                            int nb = mesh.GetNeighbor(t, side);
                            if (nb < 0) continue;
                            RecognizedRegion rn = regionOfTriangle[nb];
                            if (rn == a) nA++;
                            else if (rn == b) nB++;
                        }
                        bool distanceMove = dB < dA; // strictly closer to B
                        bool majorityMove = nB > nA && dB <= dA; // tangential tie broken by the neighborhood
                        if (!distanceMove && !majorityMove) continue;
                        double score = (dA - dB) + (nB > nA ? 0.5 * Tolerance : 0.0);
                        candidates.Add((score, t, a, b));
                    }
                }
                if (candidates.Count == 0) break;
                candidates.Sort((x, y) => y.score.CompareTo(x.score));
                HashSet<RecognizedRegion> touched = new HashSet<RecognizedRegion>();
                bool anyApplied = false;
                foreach (var cand in candidates)
                {
                    RecognizedRegion a = cand.from, b = cand.to;
                    int t = cand.tri;
                    // a move changes both its regions; skip any later candidate that touches a changed region and
                    // let it be reconsidered next pass against the refitted surfaces (its neighbor counts are stale)
                    if (touched.Contains(a) || touched.Contains(b)) continue;
                    // refit gate: B must still fit with t, A must still fit without it
                    List<int> bTris = new List<int>(b.Triangles) { t };
                    RecognizedRegion newB = FitKind(b.Kind, bTris);
                    if (newB == null) continue;
                    List<int> aTris = new List<int>(a.Triangles);
                    aTris.Remove(t);
                    RecognizedRegion newA = FitKind(a.Kind, aTris);
                    if (newA == null) continue; // removing t would break A -> leave the triangle where it is
                                                // commit
                    b.Triangles.Add(t);
                    b.Surface = newB.Surface;
                    b.MaxError = newB.MaxError;
                    b.InvalidateExtent();
                    a.Triangles.Remove(t);
                    a.Surface = newA.Surface;
                    a.MaxError = newA.MaxError;
                    a.InvalidateExtent();
                    touched.Add(a);
                    touched.Add(b);
                    anyApplied = true;
                }
                RebuildRegionOfTriangle();
                if (!anyApplied) break;
            }
        }

        /// <summary>the smallest triangle count a region of this kind can still be recognized with (plane: 2, else 3)</summary>
        private static int MinTrianglesFor(RecognizedSurfaceKind kind)
        {
            return kind == RecognizedSurfaceKind.Plane ? 2 : 3;
        }
        #endregion

        /// <summary>
        /// The directed boundary edges of the region: all triangle edges whose neighbor triangle belongs to a
        /// different region. Each item is (triangle, side, fromVertex, toVertex, neighbor region), where the edge
        /// runs from <c>mesh.GetTriangleVertex(triangle, side)</c> to <c>mesh.GetTriangleVertex(triangle, (side+1)%3)</c>.
        /// The direction follows the triangle orientation, so the boundary loops of a region run consistently
        /// around the region (counterclockwise seen from outside). The neighbor region is null where the mesh
        /// itself has a border (open shell). Connected sequences of boundary edges with the same neighbor region
        /// are the topological edges of the shell to be built; a chain should also be split at vertices where more
        /// than two regions meet (the future topological vertices).
        /// </summary>
        public IEnumerable<(int triangle, int side, int fromVertex, int toVertex, RecognizedRegion neighbor)> GetBoundaryEdges(RecognizedRegion region)
        {
            foreach (int tri in region.Triangles)
            {
                for (int side = 0; side < 3; side++)
                {
                    int nb = mesh.GetNeighbor(tri, side);
                    RecognizedRegion nbRegion = nb < 0 ? null : regionOfTriangle[nb];
                    if (nbRegion == region) continue; // inner edge
                    yield return (tri, side, mesh.GetTriangleVertex(tri, side), mesh.GetTriangleVertex(tri, (side + 1) % 3), nbRegion);
                }
            }
        }

        /// <summary>
        /// Returns the recognition result as a list of raw faces for visual inspection: each recognized surface is
        /// returned as a face bounded by the uv extent of its region, colored by surface type (plane: green,
        /// cylinder: blue, cone: orange, sphere: red, torus: violet). Unrecognized regions are added as gray
        /// triangles, each region in its own shade.
        /// </summary>
        public GeoObjectList CreateRawFaces()
        {
            if (Regions == null) Recognize();
            GeoObjectList res = new GeoObjectList();
            Random rnd = new Random(4711); // fixed seed: identical colors on each run
            foreach (RecognizedRegion region in Regions)
            {
                if (region.Surface != null)
                {
#if DEBUG
                    // stage 2 scaffolding (result discarded); the surface intersection is still fragile for some
                    // torus/plane configurations, do not let it abort the recognition during development
                    // try { CreateFaceFromRegion(region); } catch (Exception) { }
#endif
                    Face fc = MakeRawFace(region);
                    if (fc != null)
                    {
                        fc.ColorDef = ColorFor(region.Kind, rnd);
                        res.Add(fc);
                        continue;
                    }
                }
                AddTriangleFaces(res, region.Triangles, ColorFor(RecognizedSurfaceKind.Unrecognized, rnd));
            }
            return res;
        }

        public Face CreateFaceFromRegion(RecognizedRegion region)
        {
            if (region.Surface != null)
            {
                // collect the boundary edges of the region, grouped by the neighboring region
                Dictionary<RecognizedRegion, List<(int from, int to)>> regionToBoundaryEdges = new Dictionary<RecognizedRegion, List<(int from, int to)>>();
                List<(int from, int to)> openBorder = new List<(int from, int to)>(); // where the mesh itself has a border (open shell)
                foreach (var edge in GetBoundaryEdges(region))
                {
                    List<(int from, int to)> l;
                    if (edge.neighbor == null) l = openBorder;
                    else if (!regionToBoundaryEdges.TryGetValue(edge.neighbor, out l))
                    {
                        regionToBoundaryEdges[edge.neighbor] = l = new List<(int from, int to)>();
                    }
                    l.Add((edge.fromVertex, edge.toVertex));
                }
                Dictionary<(int, int), ICurve> curves = new Dictionary<(int, int), ICurve>();
                foreach (KeyValuePair<RecognizedRegion, List<(int from, int to)>> kv in regionToBoundaryEdges)
                {
                    List<List<int>> chains = SortEdgeChains(kv.Value);
                    // each chain is a polyline (as vertex indices) along which this region borders kv.Key;
                    // in a closed loop the first vertex is repeated at the end
                    for (int i = 0; i < chains.Count; i++) // usually only one
                    {
                        if (kv.Key.Surface != null) // we need more than one segment. TODO: if the chain is a single segment, we could still create a face with a single edge, but it would be a degenerate face (zero area). So we skip it for now.
                        {
                            (int a, int b, int c, int d) key;
                            if (chains[i].Count > 2)
                            {
                                key = (chains[i][chains[i].Count - 1], chains[i][chains[i].Count - 2], chains[i][1], chains[i][0]);
                            }
                            else
                            {
                                key = (chains[i][0], chains[i][0], chains[i][1], chains[i][1]);
                            }
                            // there is a chain of vertices from
                            if (edgeCurves.TryGetValue((key.d, key.c, key.b, key.a), out ICurve c))
                            {   // the curve was already created in the opposite direction, we can reuse it
                                curves[(chains[i][0], chains[i][chains[i].Count - 1])] = c;
                            }
                            else
                            {
                                List<GeoPoint> points = new List<GeoPoint>();
                                for (int j = 0; j < chains[i].Count; j++)
                                {
                                    points.Add(mesh.GetVertex(chains[i][j]));
                                }
                                if (points.Count == 2 && !(region.Surface is PlaneSurface && kv.Key.Surface is PlaneSurface)) points.Insert(1, new GeoPoint(points[0], points[1])); // for a single segment we need at least 3 points to get the right part of a closed curve
                                c = Surfaces.Intersect(region.Surface, region.Extent, kv.Key.Surface, kv.Key.Extent, points);
                                edgeCurves[key] = c;
                                curves[(chains[i][0], chains[i][chains[i].Count - 1])] = c;
                            }
                        }
                        else { }
                    }
                }
                // now curves should contain all the edges of the face to be created, each curve in the correct direction (from vertex 0 to vertex n-1 of the chain)
                // we can use the same chainsorting to bring the curves into the correct order.
                List<(int, int)> curveEndpoints = curves.Keys.ToList();
                List<List<int>> sortedChains = SortEdgeChains(curveEndpoints);
                // TODO: make a face from the chains
#if DEBUG
                DebuggerContainer dce = new DebuggerContainer();
                for (int i = 0; i < sortedChains.Count; i++)
                {
                    for (int j = 0; j < sortedChains[i].Count - 1; j++)
                    {
                        if (curves.TryGetValue((sortedChains[i][j], sortedChains[i][j + 1]), out ICurve c))
                        {
                            dce.Add(c as IGeoObject);
                        }
                    }
                }
#endif
            }
            return null;
        }

        /// <summary>
        /// Sorts directed edges (fromVertex, toVertex) into connected chains (v1,v2)->(v2,v3)->...->(vn-1,vn).
        /// Each chain is returned as its vertex sequence [v1, v2, ... vn]; for a closed loop the first vertex is
        /// repeated at the end (vn == v1). Open chains are found first (starting at vertices with more outgoing
        /// than incoming edges), the remaining edges form closed loops. At a vertex with several unused outgoing
        /// edges (more than two regions meet there) the chain simply ends and another chain starts - which is the
        /// desired behavior, such vertices become topological vertices anyway.
        /// </summary>
        private static List<List<int>> SortEdgeChains(List<(int from, int to)> edges)
        {
            Dictionary<int, List<int>> outgoing = new Dictionary<int, List<int>>(); // vertex -> indices of edges starting there
            Dictionary<int, int> balance = new Dictionary<int, int>(); // outgoing minus incoming edges per vertex
            for (int i = 0; i < edges.Count; i++)
            {
                if (!outgoing.TryGetValue(edges[i].from, out List<int> l)) outgoing[edges[i].from] = l = new List<int>();
                l.Add(i);
                balance.TryGetValue(edges[i].from, out int bf);
                balance[edges[i].from] = bf + 1;
                balance.TryGetValue(edges[i].to, out int bt);
                balance[edges[i].to] = bt - 1;
            }
            bool[] used = new bool[edges.Count];
            List<List<int>> res = new List<List<int>>();

            List<int> follow(int startEdge)
            {
                List<int> chain = new List<int> { edges[startEdge].from, edges[startEdge].to };
                used[startEdge] = true;
                int current = edges[startEdge].to;
                while (outgoing.TryGetValue(current, out List<int> candidates))
                {
                    int nextEdge = -1;
                    foreach (int e in candidates)
                    {
                        if (!used[e]) { nextEdge = e; break; }
                    }
                    if (nextEdge < 0) break; // end of an open chain or the loop is closed
                    used[nextEdge] = true;
                    current = edges[nextEdge].to;
                    chain.Add(current);
                }
                return chain;
            }

            // open chains start at vertices with more outgoing than incoming edges
            for (int i = 0; i < edges.Count; i++)
            {
                if (!used[i] && balance[edges[i].from] > 0) res.Add(follow(i));
            }
            // everything else belongs to closed loops
            for (int i = 0; i < edges.Count; i++)
            {
                if (!used[i]) res.Add(follow(i));
            }
            return res;
        }

        #region segmentation
        /// <summary>
        /// Region growing: groups the provided triangles into connected components, where two adjacent triangles
        /// belong to the same component if their normals bend less than the angle given by <paramref name="minCos"/>.
        /// </summary>
        private List<List<int>> SegmentTriangles(IEnumerable<int> subset, double minCos)
        {
            HashSet<int> remaining = new HashSet<int>(subset);
            List<List<int>> res = new List<List<int>>();
            Stack<int> toVisit = new Stack<int>();
            foreach (int seed in subset)
            {
                if (!remaining.Contains(seed)) continue;
                List<int> region = new List<int>();
                remaining.Remove(seed);
                toVisit.Push(seed);
                while (toVisit.Count > 0)
                {
                    int tri = toVisit.Pop();
                    region.Add(tri);
                    for (int side = 0; side < 3; side++)
                    {
                        int nb = mesh.GetNeighbor(tri, side);
                        if (nb < 0 || !remaining.Contains(nb)) continue;
                        if (mesh.GetNormal(tri) * mesh.GetNormal(nb) >= minCos)
                        {
                            remaining.Remove(nb);
                            toVisit.Push(nb);
                        }
                    }
                }
                res.Add(region);
            }
            return res;
        }

        /// <summary>Two triangles bond into the same similarity cluster when their area and aspect ratio agree within these factors.</summary>
        private const double shapeAreaFactor = 2.5;
        private const double shapeAspectFactor = 2.0;

        /// <summary>Ratio of the longest to the shortest edge of the triangle (>= 1); large for the long, thin slivers of a fillet band.</summary>
        private double TriangleAspect(int tri)
        {
            GeoPoint p0 = mesh.GetTrianglePoint(tri, 0), p1 = mesh.GetTrianglePoint(tri, 1), p2 = mesh.GetTrianglePoint(tri, 2);
            double e0 = p0 | p1, e1 = p1 | p2, e2 = p2 | p0;
            double max = Math.Max(e0, Math.Max(e1, e2)), min = Math.Min(e0, Math.Min(e1, e2));
            return min > 0 ? max / min : double.MaxValue;
        }

        /// <summary>
        /// Region growing by triangle shape similarity (area and aspect ratio), independent of the surface. A
        /// cylindrical fillet that is meshed as a single band of long, thin, nearly identical triangles forms one
        /// cluster; the seed two-ring around such a triangle would immediately leave the band (into the adjacent
        /// planes or the corner sphere), so the surface is never seeded there. A similarity cluster stays on the
        /// band and provides the elongated support where the cylinder is well determined.
        /// </summary>
        private List<List<int>> SegmentBySimilarity(IEnumerable<int> subset)
        {
            HashSet<int> remaining = new HashSet<int>(subset);
            List<List<int>> res = new List<List<int>>();
            Stack<int> toVisit = new Stack<int>();
            foreach (int seed in subset)
            {
                if (!remaining.Contains(seed)) continue;
                List<int> region = new List<int>();
                remaining.Remove(seed);
                toVisit.Push(seed);
                while (toVisit.Count > 0)
                {
                    int tri = toVisit.Pop();
                    region.Add(tri);
                    double area = mesh.GetArea(tri), aspect = TriangleAspect(tri);
                    for (int side = 0; side < 3; side++)
                    {
                        int nb = mesh.GetNeighbor(tri, side);
                        if (nb < 0 || !remaining.Contains(nb)) continue;
                        double na = mesh.GetArea(nb), nr = TriangleAspect(nb);
                        if (Math.Max(area, na) <= shapeAreaFactor * Math.Min(area, na)
                            && Math.Max(aspect, nr) <= shapeAspectFactor * Math.Min(aspect, nr))
                        {
                            remaining.Remove(nb);
                            toVisit.Push(nb);
                        }
                    }
                }
                res.Add(region);
            }
            return res;
        }
        #endregion

        #region recognition
        private void RecognizeSegment(List<int> triangles)
        {
            // quick path: the whole segment is a single standard surface
            RecognizedRegion whole = TryRecognize(triangles);
            if (whole != null)
            {
                Regions.Add(whole);
                return;
            }
            // The segment contains more than one surface although it has no inner hard edges: tangentially
            // connected surfaces (e.g. a box with rounded edges, or a cylinder running tangentially into two
            // planes) are merged into a single segment by the bend criterion. Such segments are decomposed by
            // growing surfaces from good seed neighborhoods: each surface "eats" all triangles it can explain.
            GrowSurfaces(triangles);
        }

        private class Seed
        {
            public int Vertex;
            public double Score; // residual of the best surface of the seed neighborhood: good seeds are processed first
            public List<int> Neighborhood; // the triangles of the two-ring around the vertex
            public List<RecognizedRegion> Candidates; // the surfaces fitting the neighborhood
        }

        private class GrownSurface
        {
            public List<int> Triangles;
            public ISurface Surface;
            public RecognizedSurfaceKind Kind;
            public double MaxError;
            public double Area;
        }

        private void GrowSurfaces(List<int> triangles)
        {
#if DEBUG
            ColorDef cd = new ColorDef("growSurface", Color.GreenYellow);
            DebuggerContainer dc = new DebuggerContainer();
            for (int j = 0; j < triangles.Count; j++)
            {
                Face fc = Face.MakeFace(mesh.GetTrianglePoint(triangles[j], 0), mesh.GetTrianglePoint(triangles[j], 1), mesh.GetTrianglePoint(triangles[j], 2));
                dc.Add(fc);
            }
#endif
            HashSet<int> remaining = new HashSet<int>(triangles);
            Dictionary<int, List<int>> vertexToTriangles = new Dictionary<int, List<int>>();
            foreach (int tri in triangles)
            {
                for (int corner = 0; corner < 3; corner++)
                {
                    int v = mesh.GetTriangleVertex(tri, corner);
                    if (!vertexToTriangles.TryGetValue(v, out List<int> incident)) vertexToTriangles[v] = incident = new List<int>();
                    incident.Add(tri);
                }
            }
            // A seed at every vertex, its neighborhood is the two-ring of remaining triangles around the vertex,
            // its score the residual of the best surface through the neighborhood. Seeds in the interior of a
            // single surface get good scores; seeds on transitions (whose neighborhood mixes two surfaces) get bad
            // scores or no candidates at all - they are never processed, because by the time it would be their
            // turn, their triangles have already been eaten from better seeds. Vertices on the segment border work
            // as seeds too (a cylinder segment of only two vertex rows has no inner vertices at all), because the
            // neighborhood only contains triangles of this segment.
            List<Seed> seeds = new List<Seed>();
            foreach (int vertex in vertexToTriangles.Keys)
            {
                Seed seed = MakeSeed(vertex, vertexToTriangles, remaining);
                if (seed != null) seeds.Add(seed);
            }
            // Additional seeds from triangle shape similarity: a fillet cylinder meshed as a single band of long
            // thin slivers is not seeded by the vertex two-rings (they leave the band at once), but the band is one
            // similarity cluster and gives the elongated support that determines the cylinder. Marked with
            // Vertex = -1 so the rescoring keeps them (their fitted surfaces stay valid over whatever remains).
            foreach (List<int> cluster in SegmentBySimilarity(triangles))
            {
                if (cluster.Count < 4) continue;
                List<RecognizedRegion> candidates = SurfaceCandidates(cluster, false, null);
                if (candidates.Count == 0) continue;
                double score = double.MaxValue;
                foreach (RecognizedRegion cand in candidates) if (cand.MaxError < score) score = cand.MaxError;
                seeds.Add(new Seed { Vertex = -1, Score = score, Neighborhood = cluster, Candidates = candidates });
            }
            bool opened = false; // whether a dominant surface has been extracted from this segment
            while (remaining.Count >= 2 && seeds.Count > 0)
            {
                // best fit first; the grow-refit fixpoint in GrowSurface then recovers the full surface regardless
                // of whether the seed sits in the interior or at the border of the surface
                seeds.Sort((a, b) => a.Score.CompareTo(b.Score));
                double remainingArea = 0.0;
                foreach (int tri in remaining) remainingArea += mesh.GetArea(tri);
                GrownSurface extracted = null;
                foreach (Seed seed in seeds)
                {
                    // grow each surface candidate of this seed and let them compete
                    GrownSurface winner = null;
                    foreach (RecognizedRegion candidate in seed.Candidates)
                    {
                        GrownSurface grown = GrowSurface(candidate, remaining);
                        if (grown == null) continue;
                        if (winner == null || grown.Area > 1.1 * winner.Area) winner = grown;
                        else if (grown.Area >= 0.9 * winner.Area && KindRank(grown.Kind) < KindRank(winner.Kind)) winner = grown; // prefer the simpler surface when it explains (almost) the same area
                    }
                    if (winner == null) continue;
                    if (QualifiesForExtraction(opened, winner, remainingArea))
                    {
                        extracted = winner;
                        break;
                    }
                }
                if (extracted == null) break;
                opened = true;
                Regions.Add(new RecognizedRegion { mesh = this.mesh, Triangles = extracted.Triangles, Surface = extracted.Surface, Kind = extracted.Kind, MaxError = extracted.MaxError });
                remaining.ExceptWith(extracted.Triangles);
                // rescore the seeds whose neighborhood lost triangles, keep all others unchanged
                HashSet<int> eaten = new HashSet<int>(extracted.Triangles);
                List<Seed> updated = new List<Seed>();
                foreach (Seed seed in seeds)
                {
                    // similarity-band seeds (Vertex < 0) are kept unchanged: their fitted surfaces still grow over
                    // whatever remains, and if their cluster is gone the grow simply yields nothing
                    if (seed.Vertex < 0) { updated.Add(seed); continue; }
                    bool stale = false;
                    foreach (int tri in seed.Neighborhood)
                    {
                        if (eaten.Contains(tri)) { stale = true; break; }
                    }
                    if (!stale)
                    {
                        updated.Add(seed);
                        continue;
                    }
                    Seed rescored = MakeSeed(seed.Vertex, vertexToTriangles, remaining);
                    if (rescored != null) updated.Add(rescored);
                }
                seeds = updated;
            }
            // Tori do not survive the vertex-seed growth: a torus fitted to a small seed neighborhood is degenerate
            // (a slim torus reads locally as a cylinder, its major axis is unrecoverable from a small patch), so it
            // stalls and never qualifies. Recover tori from what is left by marching along the tube, recovering the
            // major axis from the rotating local cylinder axes and growing the torus with that fixed axis.
            ExtractTubeTori(remaining);
            // what is left: connected components get a last chance as a whole (e.g. a fillet which became free
            // when its tangential neighbors were extracted), otherwise they remain unrecognized
            foreach (List<int> component in SegmentTriangles(remaining, Math.Cos(MaxBendAngle)))
            {
                RecognizedRegion r = TryRecognize(component);
                if (r == null) r = new RecognizedRegion { mesh = this.mesh, Triangles = component, Surface = null, Kind = RecognizedSurfaceKind.Unrecognized, MaxError = double.MaxValue };
                Regions.Add(r);
            }
        }

        /// <summary>Minimum triangle count for a recovered torus to be accepted (guards against spurious small fits).</summary>
        private const int minTorusTriangles = 40;

        /// <summary>
        /// Recovers tori from the leftover triangles (those the vertex-seed growth could not explain). Samples clean
        /// local-cylinder start points, marches each tube (<see cref="MarchTube"/>), recovers the major axis and
        /// splits the march into per-torus runs (<see cref="SegmentTubeRuns"/>), refines each torus
        /// (<see cref="BuildRefinedTorus"/>) and grows it with the fixed axis (<see cref="GrowTorus"/>). Accepted
        /// tori are added to <see cref="Regions"/> and removed from <paramref name="remaining"/>.
        /// </summary>
        private void ExtractTubeTori(HashSet<int> remaining)
        {
            if (remaining.Count < minTorusTriangles) return;
            double extent = mesh.Extent.Size;
            // candidate start triangles: sample the leftover and keep those with a clean local cylinder
            List<(int tri, double res)> starts = new List<(int, double)>();
            List<int> rem = new List<int>(remaining);
            int stepScan = Math.Max(1, rem.Count / 300);
            for (int i = 0; i < rem.Count; i += stepScan)
            {
                List<int> disk = TriangleDisk(rem[i], remaining, tubeDiskSize);
                if (disk.Count < tubeDiskSize / 2) continue;
                if (FitLocalCylinder(disk, out _, out _, out double r, out double res) && r > 10 * precision && r < 0.3 * extent && res < 0.1 * extent)
                    starts.Add((rem[i], res));
            }
            starts.Sort((a, b) => a.res.CompareTo(b.res));
            foreach ((int tri, double _) in starts)
            {
                if (!remaining.Contains(tri)) continue; // already consumed by an earlier torus
                List<TubeSample> samples = MarchTube(tri, remaining);
                if (samples.Count < 6) continue;
                GeoVector axis = RecoverTubeAxis(samples, out double coplanarity);
                if (coplanarity > 0.2) continue; // the axes are not coplanar: no single surface of revolution
                foreach (TorusFromTube run in SegmentTubeRuns(samples, axis, 0.15, 4))
                {
                    HashSet<int> union = new HashSet<int>();
                    foreach (TubeSample s in run.Samples)
                        foreach (int t in TriangleDisk(s.Triangle, remaining, tubeDiskSize)) union.Add(t);
                    if (union.Count < 4) continue;
                    ToroidalSurface seed = BuildRefinedTorus(run, RegionPoints(new List<int>(union)), out double _);
                    if (seed == null) continue;
                    List<int> seedTriangles = run.Samples.Select(s => s.Triangle).Where(remaining.Contains).ToList();
                    GrownSurface grown = GrowTorus(seed, seedTriangles, remaining);
                    if (grown == null || grown.MaxError > Tolerance || grown.Triangles.Count < minTorusTriangles) continue;
                    Regions.Add(new RecognizedRegion { mesh = this.mesh, Triangles = grown.Triangles, Surface = grown.Surface, Kind = RecognizedSurfaceKind.Torus, MaxError = grown.MaxError });
                    remaining.ExceptWith(grown.Triangles);
                }
            }
            // second pass: leftover connected components the tube march cannot handle. On a self-intersecting
            // (spindle) torus the surface lies close to the axis, where the parallel curvature exceeds the meridian
            // curvature, so the local cylinder axis points across the surface (not toroidally) and the march stalls
            // after a couple of steps. Such an isolated component does not need the march at all: it is a single
            // surface of revolution, so its axis comes directly from the normal lines (they all meet the axis).
            foreach (List<int> comp in SegmentTriangles(remaining, Math.Cos(MaxBendAngle)))
            {
                if (comp.Count < minTorusTriangles) continue;
                ToroidalSurface seed = TryDirectTorusFit(comp);
                if (seed == null) continue;
                GrownSurface grown = GrowTorus(seed, comp, remaining, axisKnown: true);
                if (grown == null || grown.MaxError > Tolerance || grown.Triangles.Count < minTorusTriangles) continue;
                Regions.Add(new RecognizedRegion { mesh = this.mesh, Triangles = grown.Triangles, Surface = grown.Surface, Kind = RecognizedSurfaceKind.Torus, MaxError = grown.MaxError });
                remaining.ExceptWith(grown.Triangles);
            }
        }

        /// <summary>
        /// Direct torus fit for an isolated connected component (no tube march): the axis direction comes from the
        /// normal lines (all normals of a surface of revolution meet the axis), the profile circle gives the major
        /// and minor radius, and <see cref="FitTorusLM"/> refines all parameters. Self-intersecting (spindle) tori
        /// are allowed. Returns the fitted torus when it stays within <see cref="Tolerance"/>, else null.
        /// </summary>
        private ToroidalSurface TryDirectTorusFit(List<int> triangles)
        {
            GeoPoint[] pnts = RegionPoints(triangles);
            if (pnts.Length < 8) return null;
            GeoPoint centroid = new GeoPoint(pnts);
            int n = Math.Min(triangles.Count, maxFitPoints);
            GeoPoint[] centroids = new GeoPoint[n]; GeoVector[] triNormals = new GeoVector[n]; GeoPoint[] ndirs = new GeoPoint[n];
            double step = triangles.Count / (double)n;
            for (int i = 0; i < n; i++)
            {
                int tri = triangles[(int)(i * step)];
                triNormals[i] = mesh.GetNormal(tri);
                ndirs[i] = GeoPoint.Origin + triNormals[i];
                centroids[i] = new GeoPoint(mesh.GetTrianglePoint(tri, 0), mesh.GetTrianglePoint(tri, 1), mesh.GetTrianglePoint(tri, 2));
            }
            List<GeoVector> axisCandidates = new List<GeoVector>();
            if (AxisFromNormalLines(centroids, triNormals, out GeoVector nlAxis)) AddAxisCandidate(axisCandidates, nlAxis);
            double gaussDist = PcaPlaneFit(ndirs, out GeoPoint _, out GeoVector gaussNormal, out GeoVector _);
            if (gaussDist < 0.3) AddAxisCandidate(axisCandidates, gaussNormal);
            double pointsPlaneDist = PcaPlaneFit(pnts, out GeoPoint _, out GeoVector pointsNormal, out GeoVector _);
            if (pointsPlaneDist != double.MaxValue) AddAxisCandidate(axisCandidates, pointsNormal);
            ToroidalSurface best = null; double bestError = double.MaxValue;
            foreach (GeoVector axis in axisCandidates)
            {
                if (!AxisPosition(centroids, triNormals, axis, centroid, out GeoPoint axisPoint)) continue;
                BuildProfile(pnts, axisPoint, axis, out double[] t, out double[] d);
                GeoPoint2D[] profile = new GeoPoint2D[pnts.Length];
                for (int i = 0; i < pnts.Length; i++) profile[i] = new GeoPoint2D(t[i], d[i]);
                Geometry.CircleFitLs(profile, out GeoPoint2D pc, out double minorRadius);
                // for a spindle torus the profile circle is folded at the axis and its center comes out on the far
                // side (negative major radius); the magnitude is the correct major radius, FitTorusLM sorts out the rest
                double majorRadius = Math.Abs(pc.y);
                if (double.IsNaN(minorRadius) || minorRadius <= precision || majorRadius <= precision) continue;
                ToroidalSurface ts = FitTorusLM(pnts, axisPoint + pc.x * axis, axis, majorRadius, minorRadius);
                if (ts == null) continue;
                double e = MaxDistance(ts, pnts);
                if (e < bestError) { bestError = e; best = ts; }
            }
            return (best != null && bestError <= Tolerance) ? best : null;
        }

        /// <summary>
        /// Grows a torus over the remaining triangles, keeping its axis direction fixed. Like <see cref="GrowSurface"/>
        /// it eats every neighbor within <see cref="Tolerance"/> and refits when the frontier runs dry, repeating to
        /// a fixpoint - but the refit keeps the (robustly recovered) axis and only adjusts center and radii
        /// (<see cref="RefineTorusFull"/>), instead of re-estimating the axis from scratch as <see cref="FitKind"/>
        /// would (which is exactly the estimate that fails for a torus). Growth stops at the tangential junction to a
        /// neighboring torus because that torus has a different radius and does not fit. Returns null when the final
        /// surface does not validate.
        /// </summary>
        private GrownSurface GrowTorus(ToroidalSurface surface, List<int> seedTriangles, HashSet<int> remaining, bool axisKnown = false)
        {
            HashSet<int> grown = new HashSet<int>();
            Queue<int> frontier = new Queue<int>();

            void walk()
            {
                while (frontier.Count > 0)
                {
                    int tri = frontier.Dequeue();
                    for (int side = 0; side < 3; side++)
                    {
                        int nb = mesh.GetNeighbor(tri, side);
                        if (nb < 0 || grown.Contains(nb) || !remaining.Contains(nb)) continue;
                        if (!TriangleFits(surface, nb)) continue;
                        grown.Add(nb);
                        frontier.Enqueue(nb);
                    }
                }
            }

            // refit keeping the (robust) march axis fixed, only center and radii - used while growing, where the
            // region may still be a partial arc that would not constrain the axis
            void refitFixedAxis()
            {
                GeoPoint[] pts = RegionPoints(new List<int>(grown));
                GeoPoint c = surface.Location; GeoVector Z = surface.ZAxis; double R = surface.MajorRadius, r = surface.MinorRadius;
                RefineTorusRadii(ref c, Z, ref R, ref r, pts);
                RefineTorusFull(ref c, Z, ref R, ref r, pts, 10);
                if (r > precision && R > precision) // r may exceed R (a self-intersecting spindle torus)
                {
                    Z.ArbitraryNormals(out GeoVector dx, out GeoVector dy);
                    surface = new ToroidalSurface(c, dx, dy, Z, R, r);
                }
            }

            // (re)grow from a set of seed triangles with the current surface; every collected triangle fits within
            // the tolerance by construction, so the region stays consistent with the surface
            void seedGrow(IEnumerable<int> seeds)
            {
                grown = new HashSet<int>();
                frontier.Clear();
                foreach (int tri in seeds)
                    if (remaining.Contains(tri) && TriangleFits(surface, tri)) { grown.Add(tri); frontier.Enqueue(tri); }
                walk();
            }

            seedGrow(seedTriangles);
            if (grown.Count == 0) return null;
            // phase 1: grow to a fixpoint with the fixed march axis (accurate enough to collect the whole tube,
            // though the slightly-off axis leaves holes and frayed edges). Skipped when the axis is already known
            // from a full-region fit (the direct path), where the fixed-axis refit (correct only for ring tori)
            // must not touch a spindle torus.
            if (!axisKnown)
                for (int iter = 0; iter < 8; iter++)
                {
                    int before = grown.Count;
                    refitFixedAxis();
                    seedGrow(new List<int>(grown));
                    if (grown.Count <= before) break;
                }
            // phase 2: the region now spans a wide arc, so the axis is well constrained. Refine ALL parameters
            // (including the axis direction) with the precise Gauss-Newton fit and re-grow with it - this fills the
            // holes and frayed edges the fixed axis left. Repeat until it no longer grows.
            for (int iter = 0; iter < 8; iter++)
            {
                int before = grown.Count;
                ToroidalSurface acc = FitTorus(surface.Location, surface.ZAxis, surface.MajorRadius, surface.MinorRadius, RegionPoints(new List<int>(grown)));
                if (acc == null) break;
                surface = acc;
                seedGrow(new List<int>(grown));
                if (grown.Count <= before) break;
            }
            List<int> grownList = new List<int>(grown);
            double maxError = MaxDistance(surface, RegionPoints(grownList));
            if (maxError > Tolerance) return null;
            double area = 0.0;
            foreach (int tri in grownList) area += mesh.GetArea(tri);
            return new GrownSurface { Triangles = grownList, Surface = surface, Kind = RecognizedSurfaceKind.Torus, MaxError = maxError, Area = area };
        }

        private Seed MakeSeed(int vertex, Dictionary<int, List<int>> vertexToTriangles, HashSet<int> remaining)
        {
            List<int> neighborhood = TwoRing(vertex, vertexToTriangles, remaining);
            if (neighborhood.Count < 3) return null;
            List<RecognizedRegion> candidates = SurfaceCandidates(neighborhood, false, null);
            if (candidates.Count == 0) return null;
            double score = double.MaxValue;
            foreach (RecognizedRegion c in candidates)
            {
                if (c.MaxError < score) score = c.MaxError;
            }
            return new Seed { Vertex = vertex, Score = score, Neighborhood = neighborhood, Candidates = candidates };
        }

        /// <summary>the remaining triangles of the two-ring around the vertex</summary>
        private List<int> TwoRing(int vertex, Dictionary<int, List<int>> vertexToTriangles, HashSet<int> remaining)
        {
            HashSet<int> tris = new HashSet<int>();
            HashSet<int> ringVertices = new HashSet<int>();
            foreach (int tri in vertexToTriangles[vertex])
            {
                if (!remaining.Contains(tri)) continue;
                tris.Add(tri);
                for (int corner = 0; corner < 3; corner++) ringVertices.Add(mesh.GetTriangleVertex(tri, corner));
            }
            foreach (int rv in ringVertices)
            {
                if (!vertexToTriangles.TryGetValue(rv, out List<int> incident)) continue;
                foreach (int tri in incident)
                {
                    if (remaining.Contains(tri)) tris.Add(tri);
                }
            }
            return new List<int>(tris);
        }

        /// <summary>
        /// Grows the seed surface over the remaining triangles: a triangle is eaten when all its vertices are
        /// within <see cref="Tolerance"/> of the surface. While growing, the surface is refitted with all collected
        /// vertices (the initial surface from the small seed neighborhood is inaccurate, refitting lets it settle
        /// in), after a refit the rejected neighbors are tested again. When the frontier runs dry, the surface is
        /// refitted once more over everything collected and growth is retried, because the improved surface often
        /// reaches a little further; this repeats until a fixpoint. Only when even that adds nothing is a last
        /// attempt made per stable state: the single best-fitting rejected border neighbor is pulled in and the
        /// surface is refitted over the union. A slightly off seed axis can leave the true surface just outside the
        /// tolerance at the border, and reconciling the axis over the union recovers it - <see cref="FitKind"/>
        /// only returns a surface when the whole union stays within <see cref="Tolerance"/>, so this can never
        /// degrade the collected set below the tolerance. Returns null when the final surface does not validate.
        /// </summary>
        private GrownSurface GrowSurface(RecognizedRegion seedCandidate, HashSet<int> remaining)
        {
            ISurface surface = seedCandidate.Surface;
            HashSet<int> grown = new HashSet<int>();
            Queue<int> frontier = new Queue<int>();
            int lastRefit = 4;

            // Eat every remaining neighbor that lies on the current surface. Refit whenever the collected set has
            // doubled: this corrects the axis as more points come in, so the surface stays accurate over long
            // shells (the re-enqueue re-tests all grown against the improved surface). O(log n) refits while
            // growing, so this stays cheap.
            void walk()
            {
                while (frontier.Count > 0)
                {
                    int tri = frontier.Dequeue();
                    for (int side = 0; side < 3; side++)
                    {
                        int nb = mesh.GetNeighbor(tri, side);
                        if (nb < 0 || grown.Contains(nb) || !remaining.Contains(nb)) continue;
                        if (!TriangleFits(surface, nb)) continue;
                        grown.Add(nb);
                        frontier.Enqueue(nb);
                    }
                    if (grown.Count >= 2 * lastRefit)
                    {
                        RecognizedRegion refit = FitKind(seedCandidate.Kind, new List<int>(grown));
                        if (refit != null)
                        {
                            surface = refit.Surface;
                            frontier.Clear();
                            foreach (int g in grown) frontier.Enqueue(g);
                        }
                        lastRefit = grown.Count;
                    }
                }
            }

            foreach (int tri in seedCandidate.Triangles)
            {
                if (remaining.Contains(tri) && TriangleFits(surface, tri))
                {
                    grown.Add(tri);
                    frontier.Enqueue(tri);
                }
            }
            if (grown.Count == 0) return null;
            lastRefit = Math.Max(grown.Count, 4);
            walk();

            // Fixpoint growth beyond the plain frontier walk: refit over everything collected and walk again (the
            // refined surface may reach further); when that no longer grows the set, make one attempt to absorb the
            // best-fitting rejected border neighbor and refit with it, then walk again. The loop ends when neither
            // a refit nor the best-neighbor attempt changes the set - a real edge stops it because no single
            // surface of this kind then fits the union within the tolerance (FitKind returns null).
            while (true)
            {
                int before = grown.Count;
                RecognizedRegion refit = FitKind(seedCandidate.Kind, new List<int>(grown));
                if (refit != null)
                {
                    surface = refit.Surface;
                    lastRefit = grown.Count;
                    foreach (int g in grown) frontier.Enqueue(g);
                    walk();
                }
                if (grown.Count > before) continue; // the refit opened new ground, keep growing
                                                    // stable set: try to grow across the border via the single closest rejected neighbor
                int best = BestRejectedNeighbor(surface, grown, remaining);
                if (best < 0) break;
                List<int> withBest = new List<int>(grown) { best };
                RecognizedRegion extended = FitKind(seedCandidate.Kind, withBest);
                if (extended == null) break; // adding it cannot stay within tolerance -> real edge, stop
                surface = extended.Surface;
                grown.Add(best);
                lastRefit = grown.Count;
                foreach (int g in grown) frontier.Enqueue(g);
                walk();
            }
            List<int> grownList = new List<int>(grown);
            // final refit; when it does not improve, keep the surface which collected the triangles
            GeoPoint[] pnts = RegionPoints(grownList);
            double maxError = MaxDistance(surface, pnts);
            RecognizedRegion finalRefit = FitKind(seedCandidate.Kind, grownList);
            if (finalRefit != null && finalRefit.MaxError < maxError)
            {
                surface = finalRefit.Surface;
                maxError = finalRefit.MaxError;
            }
            // the single-shot direct fits (profile regression, algebraic circle/sphere fit) are only moderately
            // accurate; a least squares (Levenberg-Marquardt) refine over the whole grown region averages out the
            // STL rounding and reaches the true surface. This matters for the extraction quality gates, where a real
            // surface that fits just at the threshold with the coarse fit clearly passes once fitted accurately.
            // Only worth it when the coarse fit is already within reach of the tolerance: a local refine cannot pull
            // a region that is far off (e.g. a deformed sphere many tolerances away) under the tolerance, and running
            // it on every failing grow of a large freeform region dominates the runtime.
            if (maxError <= 4 * Tolerance)
            {
                ISurface lmSurface = RefineSurfaceLM(seedCandidate.Kind, surface, pnts);
                if (lmSurface != null)
                {
                    double lmError = MaxDistance(lmSurface, pnts);
                    if (lmError < maxError) { surface = lmSurface; maxError = lmError; }
                }
            }
            if (maxError > Tolerance) return null;
            double area = 0.0;
            foreach (int tri in grownList) area += mesh.GetArea(tri);
            return new GrownSurface { Triangles = grownList, Surface = surface, Kind = seedCandidate.Kind, MaxError = maxError, Area = area };
        }

        /// <summary>the best surface of the given kind for the triangles, or null</summary>
        private RecognizedRegion FitKind(RecognizedSurfaceKind kind, List<int> triangles)
        {
            List<RecognizedRegion> candidates = SurfaceCandidates(triangles, false, kind);
            return candidates.Count > 0 ? candidates[0] : null;
        }

        private bool TriangleFits(ISurface surface, int triangle)
        {
            for (int corner = 0; corner < 3; corner++)
            {
                if (Math.Abs(surface.GetDistance(mesh.GetTrianglePoint(triangle, corner))) > Tolerance) return false;
            }
            return true;
        }

        /// <summary>the largest distance of any of the triangle's three vertices from the surface</summary>
        private double TriangleDistance(ISurface surface, int triangle)
        {
            double d = 0.0;
            for (int corner = 0; corner < 3; corner++)
            {
                double dc = Math.Abs(surface.GetDistance(mesh.GetTrianglePoint(triangle, corner)));
                if (dc > d) d = dc;
            }
            return d;
        }

        /// <summary>
        /// Among the border neighbors of the grown set (remaining triangles adjacent to it that do not fit the
        /// surface yet), the one lying closest to the surface, or -1 if there is none. Used for a last-resort
        /// growth attempt: refitting the surface with this triangle can reconcile a slightly off seed axis and let
        /// the growth continue across a border where the original surface fell just outside the tolerance.
        /// </summary>
        private int BestRejectedNeighbor(ISurface surface, HashSet<int> grown, HashSet<int> remaining)
        {
            int best = -1;
            double bestDist = double.MaxValue;
            foreach (int tri in grown)
            {
                for (int side = 0; side < 3; side++)
                {
                    int nb = mesh.GetNeighbor(tri, side);
                    if (nb < 0 || grown.Contains(nb) || !remaining.Contains(nb)) continue;
                    double d = TriangleDistance(surface, nb);
                    if (d < bestDist) { bestDist = d; best = nb; }
                }
            }
            return best;
        }

        private static int KindRank(RecognizedSurfaceKind kind)
        {
            switch (kind)
            {
                case RecognizedSurfaceKind.Plane: return 0;
                case RecognizedSurfaceKind.Cylinder: return 1;
                case RecognizedSurfaceKind.Cone: return 2;
                case RecognizedSurfaceKind.Sphere: return 2;
                case RecognizedSurfaceKind.Torus: return 3;
                default: return 4;
            }
        }

        private bool QualifiesForExtraction(bool opened, GrownSurface grown, double remainingArea)
        {
            // The first extraction from a segment must explain a dominant part of it: on a smooth freeform region
            // (e.g. the spiral walls of a rope holder) every seed fits some small local patch, extracting those
            // would shred the region into meaningless pieces, so nothing is extracted at all. Once a dominant
            // surface has been found, the segment is known to be a composite of standard surfaces and smaller
            // pieces are extracted as well - either substantial ones or (almost) exact ones, the latter for planes
            // consisting of only two large triangles.
            if (grown.MaxError > Tolerance) return false;
            if (!opened)
            {
                // an osculating cylinder can cover a substantial part of a slowly winding freeform surface (e.g. a
                // spiral wall) while its error stays just below the tolerance; real curved surfaces fit much better,
                // so the opening curved surface must be clearly within the tolerance. A plane cannot osculate a
                // winding freeform over a large area, so a dominant plane may open the segment up to the full
                // tolerance (it may pick up a thin tangent strip of an adjoining fillet, reassigned later).
                if (grown.Kind == RecognizedSurfaceKind.Plane) return grown.Area >= 0.15 * remainingArea;
                return grown.Area >= 0.15 * remainingArea && grown.MaxError <= 0.5 * Tolerance;
            }
            if (grown.Area >= 0.15 * remainingArea) return true;
            return grown.Triangles.Count >= 6 || grown.MaxError <= 0.1 * Tolerance;
        }

        private RecognizedRegion TryRecognize(List<int> triangles)
        {
            List<RecognizedRegion> candidates = SurfaceCandidates(triangles, true, null);
            return candidates.Count > 0 ? candidates[0] : null;
        }

        /// <summary>
        /// Finds standard surfaces which fit the given triangles within <see cref="Tolerance"/>. All surfaces are
        /// constructed directly from geometric estimates and then validated against the region vertices. The
        /// GaussNewtonMinimizer fits are not used here: their convergence criterion works on algebraic residuals
        /// whose scale depends on radius and point count, which makes them reject perfectly good solutions (or
        /// accept prematurely), depending on the "precision" parameter.
        /// The result is ordered by increasing complexity (plane, cylinder, cone, sphere, torus) and contains at
        /// most one (the best) surface of each kind. With <paramref name="firstOnly"/> only the first (simplest)
        /// fitting surface is returned, with <paramref name="onlyKind"/> the search is restricted to a single kind
        /// (used for refitting a surface while it grows).
        /// </summary>
        private List<RecognizedRegion> SurfaceCandidates(List<int> triangles, bool firstOnly, RecognizedSurfaceKind? onlyKind)
        {
            List<RecognizedRegion> res = new List<RecognizedRegion>();
            GeoPoint[] pnts = RegionPoints(triangles);

            // 1. plane. A single triangle is always trivially planar, but that is not a meaningful recognition
            // (every triangle would qualify), so a region needs at least two triangles to be recognized as a plane.
            if ((onlyKind == null || onlyKind == RecognizedSurfaceKind.Plane) && triangles.Count >= 2)
            {
                double planeDist = PcaPlaneFit(pnts, out GeoPoint planeLoc, out GeoVector planeNormal, out GeoVector _);
                if (planeDist <= Tolerance)
                {
                    PlaneSurface ps = new PlaneSurface(new Plane(planeLoc, planeNormal));
                    res.Add(new RecognizedRegion { mesh = this.mesh, Triangles = triangles, Surface = ps, Kind = RecognizedSurfaceKind.Plane, MaxError = planeDist });
                    if (firstOnly) return res;
                }
                if (onlyKind != null) return res;
            }
            if (triangles.Count < 3) return res; // too small for anything but a plane

            GeoPoint centroid = new GeoPoint(pnts);

            // triangle normals and centroids (sampled)
            GeoPoint[] ndirs = new GeoPoint[Math.Min(triangles.Count, maxFitPoints)];
            GeoPoint[] centroids = new GeoPoint[ndirs.Length];
            GeoVector[] triNormals = new GeoVector[ndirs.Length];
            GeoVector meanNormal = GeoVector.NullVector;
            double step = triangles.Count / (double)ndirs.Length;
            for (int i = 0; i < ndirs.Length; i++)
            {
                int tri = triangles[(int)(i * step)];
                triNormals[i] = mesh.GetNormal(tri);
                meanNormal = meanNormal + triNormals[i];
                ndirs[i] = GeoPoint.Origin + triNormals[i]; // the normal as a point on the unit sphere (Gauss map)
                centroids[i] = new GeoPoint(mesh.GetTrianglePoint(tri, 0), mesh.GetTrianglePoint(tri, 1), mesh.GetTrianglePoint(tri, 2));
            }

            // Candidates for the axis of a surface of revolution:
            // - Gauss map: for cylinders and cones the triangle normals lie on a circle on the unit sphere, the
            //   normal of the circle plane is the axis direction.
            // - the normal of a plane through the points (e.g. for flat toruses like a fillet around a hole)
            // - for shallow regions (large radius, small bending) the Gauss map plane is nearly degenerate; there
            //   the cross product of the mean normal with the main spread direction of the normals is a better guess.
            List<GeoVector> axisCandidates = new List<GeoVector>();
            double gaussDist = PcaPlaneFit(ndirs, out GeoPoint _, out GeoVector gaussNormal, out GeoVector gaussSpread);
            if (gaussDist < 0.3) AddAxisCandidate(axisCandidates, gaussNormal);
            if (!meanNormal.IsNullVector() && gaussDist != double.MaxValue)
            {
                GeoVector crossAxis = meanNormal.Normalized ^ gaussSpread;
                if (crossAxis.Length > 1e-6) AddAxisCandidate(axisCandidates, crossAxis.Normalized);
            }
            double pointsPlaneDist = PcaPlaneFit(pnts, out GeoPoint _, out GeoVector pointsNormal, out GeoVector _);
            if (pointsPlaneDist != double.MaxValue) AddAxisCandidate(axisCandidates, pointsNormal);
            // - the line which best intersects all normal lines (the normal lines of a surface of revolution all
            //   intersect the axis). In contrast to the Gauss map plane this also works on small, finely
            //   tessellated patches, where the normal tips cover only a tiny arc and the PCA plane through them is
            //   dominated by the arc tangent instead of its curvature.
            if (AxisFromNormalLines(centroids, triNormals, out GeoVector normalLinesAxis)) AddAxisCandidate(axisCandidates, normalLinesAxis);

            // 2. cylinder and cone: the profile (position along the axis, distance from the axis) is a line
            if (onlyKind == null || onlyKind == RecognizedSurfaceKind.Cylinder || onlyKind == RecognizedSurfaceKind.Cone)
            {
                RecognizedRegion bestCylinder = null, bestCone = null;
                foreach (GeoVector axis in axisCandidates)
                {
                    if (!AxisPosition(centroids, triNormals, axis, centroid, out GeoPoint axisPoint)) continue;
                    BuildProfile(pnts, axisPoint, axis, out double[] t, out double[] d);
                    if (onlyKind == null || onlyKind == RecognizedSurfaceKind.Cylinder)
                    {
                        RecognizedRegion r = TryCylinder(triangles, pnts, axisPoint, axis);
                        if (r != null && (bestCylinder == null || r.MaxError < bestCylinder.MaxError)) bestCylinder = r;
                    }
                    if (onlyKind == null || onlyKind == RecognizedSurfaceKind.Cone)
                    {
                        RecognizedRegion r = TryCone(triangles, pnts, axisPoint, axis, t, d);
                        if (r != null && (bestCone == null || r.MaxError < bestCone.MaxError)) bestCone = r;
                    }
                }
                if (bestCylinder != null)
                {
                    res.Add(bestCylinder);
                    if (firstOnly) return res;
                }
                if (bestCone != null)
                {
                    res.Add(bestCone);
                    if (firstOnly) return res;
                }
                if (onlyKind != null) return res;
            }

            // 3. sphere: all normal lines pass (almost) through the center
            if (onlyKind == null || onlyKind == RecognizedSurfaceKind.Sphere)
            {
                if (NormalLinesIntersection(centroids, triNormals, out GeoPoint sphereCenter))
                {
                    double r0 = 0.0;
                    for (int i = 0; i < pnts.Length; i++) r0 += pnts[i] | sphereCenter;
                    r0 /= pnts.Length;
                    if (r0 > precision)
                    {
                        SphericalSurface ss = new SphericalSurface(sphereCenter, r0 * GeoVector.XAxis, r0 * GeoVector.YAxis, r0 * GeoVector.ZAxis);
                        double maxDist = MaxDistance(ss, pnts);
                        if (maxDist <= Tolerance)
                        {
                            res.Add(new RecognizedRegion { mesh = this.mesh, Triangles = triangles, Surface = ss, Kind = RecognizedSurfaceKind.Sphere, MaxError = maxDist });
                            if (firstOnly) return res;
                        }
                    }
                }
                if (onlyKind != null) return res;
            }

            // 4. torus, deliberately last: cylinder and cone take priority. Two rows of points on two circles lie
            // on infinitely many toruses (the fit is underdetermined and can even absorb radial noise near the tube
            // poles), so a torus is only accepted when at least three parallel circles support it.
            if (onlyKind == null || onlyKind == RecognizedSurfaceKind.Torus)
            {
                RecognizedRegion bestTorus = null;
                foreach (GeoVector axis in axisCandidates)
                {
                    if (!AxisPosition(centroids, triNormals, axis, centroid, out GeoPoint axisPoint)) continue;
                    BuildProfile(pnts, axisPoint, axis, out double[] t, out double[] d);
                    RecognizedRegion r = TryTorus(triangles, pnts, axisPoint, axis, t, d);
                    if (r != null && (bestTorus == null || r.MaxError < bestTorus.MaxError)) bestTorus = r;
                }
                if (bestTorus != null) res.Add(bestTorus);
            }
            return res;
        }

        private static void AddAxisCandidate(List<GeoVector> candidates, GeoVector axis)
        {
            foreach (GeoVector cand in candidates)
            {
                if (Math.Abs(cand * axis) > 0.999) return; // (anti)parallel to an existing candidate
            }
            candidates.Add(axis);
        }

        /// <summary>position along the axis and distance from the axis for each point (the profile of a surface of revolution)</summary>
        private static void BuildProfile(GeoPoint[] pnts, GeoPoint axisPoint, GeoVector axis, out double[] t, out double[] d)
        {
            t = new double[pnts.Length];
            d = new double[pnts.Length];
            for (int i = 0; i < pnts.Length; i++)
            {
                GeoVector v = pnts[i] - axisPoint;
                t[i] = v * axis;
                d[i] = (v - t[i] * axis).Length;
            }
        }

        private RecognizedRegion TryCylinder(List<int> triangles, GeoPoint[] pnts, GeoPoint axisPoint, GeoVector axis)
        {
            // The axis position estimated from the normal lines can be systematically biased: thin sliver
            // triangles have wobbling normals, and when the triangulation pattern is uniform (e.g. all diagonals
            // in the same direction) the wobble does not average out. The vertices themselves are exact, so the
            // position is refined with a least squares circle through the points projected along the axis.
            Plane perp = new Plane(axisPoint, axis);
            GeoPoint2D[] pnts2d = new GeoPoint2D[pnts.Length];
            for (int i = 0; i < pnts.Length; i++) pnts2d[i] = perp.Project(pnts[i]);
            Geometry.CircleFitLs(pnts2d, out GeoPoint2D center2d, out double circleRadius);
            if (!double.IsNaN(center2d.x) && !double.IsNaN(circleRadius) && !double.IsInfinity(circleRadius) && circleRadius > precision)
            {
                axisPoint = perp.ToGlobal(center2d);
            }
            double radius = 0.0;
            for (int i = 0; i < pnts.Length; i++)
            {
                GeoVector v = pnts[i] - axisPoint;
                radius += (v - (v * axis) * axis).Length;
            }
            radius /= pnts.Length;
            if (radius <= precision) return null;
            axis.ArbitraryNormals(out GeoVector dirx, out GeoVector diry);
            CylindricalSurface cs = new CylindricalSurface(axisPoint, radius * dirx, radius * diry, axis);
            double maxDist = MaxDistance(cs, pnts);
            if (maxDist > Tolerance) return null;
            return new RecognizedRegion { mesh = this.mesh, Triangles = triangles, Surface = cs, Kind = RecognizedSurfaceKind.Cylinder, MaxError = maxDist };
        }

        private RecognizedRegion TryCone(List<int> triangles, GeoPoint[] pnts, GeoPoint axisPoint, GeoVector axis, double[] t, double[] d)
        {
            // linear regression d = m*t + b: the distance from the axis grows linearly along the axis of a cone
            int n = pnts.Length;
            double st = 0.0, sd = 0.0, stt = 0.0, std = 0.0;
            for (int i = 0; i < n; i++)
            {
                st += t[i];
                sd += d[i];
                stt += t[i] * t[i];
                std += t[i] * d[i];
            }
            double denom = stt - st * st / n;
            if (Math.Abs(denom) < 1e-13) return null;
            double m = (std - st * sd / n) / denom;
            double b = (sd - m * st) / n;
            if (Math.Abs(m) < 0.002) return null; // almost a cylinder, the apex would be extremely far away
            if (m < 0) { m = -m; axis = -axis; } // orient the axis towards the opening side (t changes sign, b is unchanged)
            GeoPoint apex = axisPoint + (-b / m) * axis;
            double semiAngle = Math.Atan(m);
            axis.ArbitraryNormals(out GeoVector dirx, out GeoVector diry);
            ConicalSurface cs = new ConicalSurface(apex, dirx, diry, axis, semiAngle);
            double maxDist = MaxDistance(cs, pnts);
            if (maxDist > Tolerance) return null;
            return new RecognizedRegion { mesh = this.mesh, Triangles = triangles, Surface = cs, Kind = RecognizedSurfaceKind.Cone, MaxError = maxDist };
        }

        private RecognizedRegion TryTorus(List<int> triangles, GeoPoint[] pnts, GeoPoint axisPoint, GeoVector axis, double[] t, double[] d)
        {
            // the points of a torus lie on parallel circles ("rows"); a torus is determined by at least three of
            // them, with only two rows a cylinder or cone is always the better (simpler) explanation
            if (CountProfileRows(t, d) < 3) return null;
            // in the profile the points of a torus lie on a circle around (center plane position, major radius)
            GeoPoint2D[] profile = new GeoPoint2D[pnts.Length];
            for (int i = 0; i < pnts.Length; i++) profile[i] = new GeoPoint2D(t[i], d[i]);
            Geometry.CircleFitLs(profile, out GeoPoint2D profileCenter, out double minorRadius);
            double majorRadius = profileCenter.y;
            if (double.IsNaN(minorRadius) || minorRadius <= precision || majorRadius <= precision) return null;
            // reject spindle (self-intersecting) toruses: a real CAD torus is a ring torus with minor < major. A
            // spindle torus whose inner equator |major - minor| happens to match a cylinder radius would otherwise
            // be fitted to a narrow strip of a cylinder shell (a degenerate over-fit).
            if (minorRadius >= majorRadius) return null;
            GeoPoint loc = axisPoint + profileCenter.x * axis;
            axis.ArbitraryNormals(out GeoVector dirx, out GeoVector diry);
            ToroidalSurface ts = new ToroidalSurface(loc, dirx, diry, axis, majorRadius, minorRadius);
            double maxDist = MaxDistance(ts, pnts);
            if (maxDist > Tolerance) return null;
            return new RecognizedRegion { mesh = this.mesh, Triangles = triangles, Surface = ts, Kind = RecognizedSurfaceKind.Torus, MaxError = maxDist };
        }

        /// <summary>
        /// Counts the "rows" of the profile: clusters of points with (almost) the same position along the axis and
        /// the same distance from the axis. Each row corresponds to a parallel circle of a surface of revolution.
        /// </summary>
        private int CountProfileRows(double[] t, double[] d)
        {
            double tmin = double.MaxValue, tmax = double.MinValue, dmin = double.MaxValue, dmax = double.MinValue;
            for (int i = 0; i < t.Length; i++)
            {
                if (t[i] < tmin) tmin = t[i];
                if (t[i] > tmax) tmax = t[i];
                if (d[i] < dmin) dmin = d[i];
                if (d[i] > dmax) dmax = d[i];
            }
            double eps = Math.Max(4 * Tolerance, 1e-3 * Math.Max(tmax - tmin, dmax - dmin));
            (double, double)[] profile = new (double, double)[t.Length];
            for (int i = 0; i < t.Length; i++) profile[i] = (t[i], d[i]);
            Array.Sort(profile);
            int rows = 0;
            int start = 0;
            for (int i = 1; i <= profile.Length; i++)
            {
                if (i == profile.Length || profile[i].Item1 - profile[i - 1].Item1 > eps)
                {   // a group with (almost) the same t: split it by d (a torus can have two rows at the same height)
                    double[] ds = new double[i - start];
                    for (int j = start; j < i; j++) ds[j - start] = profile[j].Item2;
                    Array.Sort(ds);
                    rows++;
                    for (int j = 1; j < ds.Length; j++) if (ds[j] - ds[j - 1] > eps) rows++;
                    start = i;
                }
            }
            return rows;
        }

        /// <summary>
        /// Least squares plane through the points (principal component analysis). Returns the maximum distance of
        /// the points from the plane, or double.MaxValue if the points are (nearly) collinear.
        /// </summary>
        private static double PcaPlaneFit(GeoPoint[] pnts, out GeoPoint location, out GeoVector normal, out GeoVector mainSpread)
        {
            location = new GeoPoint(pnts);
            normal = GeoVector.ZAxis;
            mainSpread = GeoVector.XAxis;
            double[,] cov = new double[3, 3];
            for (int i = 0; i < pnts.Length; i++)
            {
                GeoVector v = pnts[i] - location;
                cov[0, 0] += v.x * v.x; cov[0, 1] += v.x * v.y; cov[0, 2] += v.x * v.z;
                cov[1, 1] += v.y * v.y; cov[1, 2] += v.y * v.z;
                cov[2, 2] += v.z * v.z;
            }
            cov[1, 0] = cov[0, 1]; cov[2, 0] = cov[0, 2]; cov[2, 1] = cov[1, 2];
            Jacobi3(cov, out double[] eigenValues, out double[,] eigenVectors);
            // find the eigenvectors of the smallest and the largest eigenvalue
            int smallest = 0, largest = 0;
            for (int i = 1; i < 3; i++)
            {
                if (eigenValues[i] < eigenValues[smallest]) smallest = i;
                if (eigenValues[i] > eigenValues[largest]) largest = i;
            }
            int middle = 3 - smallest - largest;
            if (eigenValues[largest] <= 0.0 || eigenValues[middle] < 1e-10 * eigenValues[largest]) return double.MaxValue; // the points are (nearly) collinear
            normal = new GeoVector(eigenVectors[0, smallest], eigenVectors[1, smallest], eigenVectors[2, smallest]).Normalized;
            mainSpread = new GeoVector(eigenVectors[0, largest], eigenVectors[1, largest], eigenVectors[2, largest]).Normalized;
            double maxDist = 0.0;
            for (int i = 0; i < pnts.Length; i++)
            {
                double dist = Math.Abs((pnts[i] - location) * normal);
                if (dist > maxDist) maxDist = dist;
            }
            return maxDist;
        }

        /// <summary>
        /// Eigenvalues and eigenvectors of a symmetric 3x3 matrix. The eigenvectors are the columns of
        /// <paramref name="eigenVectors"/>, in the order of <paramref name="eigenValues"/>.
        /// </summary>
        private static void Jacobi3(double[,] m, out double[] eigenValues, out double[,] eigenVectors)
        {
            JacobiEigen(m, 3, out eigenValues, out eigenVectors);
        }

        /// <summary>
        /// Eigenvalues and eigenvectors of a symmetric n x n matrix (cyclic Jacobi rotations). The eigenvectors are
        /// the columns of <paramref name="eigenVectors"/>, in the order of <paramref name="eigenValues"/>.
        /// </summary>
        private static void JacobiEigen(double[,] m, int n, out double[] eigenValues, out double[,] eigenVectors)
        {
            double[,] a = new double[n, n];
            Array.Copy(m, a, n * n);
            double[,] v = new double[n, n];
            double scale = 0.0;
            for (int i = 0; i < n; i++)
            {
                v[i, i] = 1.0;
                scale += Math.Abs(a[i, i]);
            }
            if (scale == 0.0) scale = 1.0;
            for (int sweep = 0; sweep < 50; sweep++)
            {
                double off = 0.0;
                for (int p = 0; p < n - 1; p++)
                {
                    for (int q = p + 1; q < n; q++) off += a[p, q] * a[p, q];
                }
                if (off < 1e-30 * scale * scale) break;
                for (int p = 0; p < n - 1; p++)
                {
                    for (int q = p + 1; q < n; q++)
                    {
                        if (Math.Abs(a[p, q]) < 1e-30 * scale) continue;
                        double theta = (a[q, q] - a[p, p]) / (2.0 * a[p, q]);
                        double t = Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1.0));
                        if (theta == 0.0) t = 1.0;
                        double c = 1.0 / Math.Sqrt(t * t + 1.0);
                        double s = t * c;
                        for (int k = 0; k < n; k++)
                        {
                            double akp = a[k, p], akq = a[k, q];
                            a[k, p] = c * akp - s * akq;
                            a[k, q] = s * akp + c * akq;
                        }
                        for (int k = 0; k < n; k++)
                        {
                            double apk = a[p, k], aqk = a[q, k];
                            a[p, k] = c * apk - s * aqk;
                            a[q, k] = s * apk + c * aqk;
                        }
                        for (int k = 0; k < n; k++)
                        {
                            double vkp = v[k, p], vkq = v[k, q];
                            v[k, p] = c * vkp - s * vkq;
                            v[k, q] = s * vkp + c * vkq;
                        }
                    }
                }
            }
            eigenValues = new double[n];
            for (int i = 0; i < n; i++) eigenValues[i] = a[i, i];
            eigenVectors = v;
        }

        /// <summary>
        /// The direction of the line which best intersects all normal lines (location[i], normal[i]) in the least
        /// squares sense. The normal lines of any surface of revolution intersect its axis, so this is an estimate
        /// for the axis direction of cylinders, cones and toruses. Unlike the Gauss map plane it also works on
        /// small patches: in Pluecker coordinates a line X = (l, m) intersects the line (p, n) exactly when
        /// l*(p^n) + m*n = 0, which is linear in (l, m) - the best solution is the eigenvector of the smallest
        /// eigenvalue of a symmetric 6x6 matrix. Returns false in degenerate cases (e.g. all normals parallel).
        /// </summary>
        private static bool AxisFromNormalLines(GeoPoint[] location, GeoVector[] normal, out GeoVector axisDir)
        {
            axisDir = GeoVector.ZAxis;
            // shift to the centroid and scale to size ~1, so that the direction part l and the moment part m of
            // the Pluecker coordinates are comparable in magnitude
            GeoPoint centroid = new GeoPoint(location);
            double size = 0.0;
            for (int i = 0; i < location.Length; i++) size += location[i] | centroid;
            size /= location.Length;
            if (size < 1e-12) return false;
            double[,] mtx = new double[6, 6];
            for (int i = 0; i < location.Length; i++)
            {
                GeoVector p = (1.0 / size) * (location[i] - centroid);
                GeoVector n = normal[i];
                GeoVector pxn = new GeoVector(p.y * n.z - p.z * n.y, p.z * n.x - p.x * n.z, p.x * n.y - p.y * n.x);
                double[] row = new double[] { pxn.x, pxn.y, pxn.z, n.x, n.y, n.z };
                for (int r = 0; r < 6; r++)
                {
                    for (int c = r; c < 6; c++) mtx[r, c] += row[r] * row[c];
                }
            }
            for (int r = 1; r < 6; r++)
            {
                for (int c = 0; c < r; c++) mtx[r, c] = mtx[c, r];
            }
            JacobiEigen(mtx, 6, out double[] eigenValues, out double[,] eigenVectors);
            int smallest = 0;
            for (int i = 1; i < 6; i++)
            {
                if (eigenValues[i] < eigenValues[smallest]) smallest = i;
            }
            GeoVector l = new GeoVector(eigenVectors[0, smallest], eigenVectors[1, smallest], eigenVectors[2, smallest]);
            if (l.Length < 0.01) return false; // the direction part vanishes: no proper axis (e.g. parallel normals)
            axisDir = l.Normalized;
            return true;
        }

        /// <summary>
        /// The normal lines of a surface of revolution intersect the axis (each normal line lies in a meridian
        /// plane). With a given axis direction this finds the axis position as the line which is closest to all
        /// normal lines (least squares, linear in the two unknowns). Returns false if the direction of the normals
        /// gives no usable information (e.g. all normals parallel to the axis).
        /// </summary>
        private static bool AxisPosition(GeoPoint[] location, GeoVector[] normal, GeoVector axis, GeoPoint seed, out GeoPoint axisPoint)
        {
            axisPoint = seed;
            axis.ArbitraryNormals(out GeoVector u, out GeoVector v);
            // axis line: seed + alpha*u + beta*v; distance to the normal line i is ((location[i] - x0) * w) with
            // w = normalized common perpendicular of axis and normal[i]
            double a11 = 0.0, a12 = 0.0, a22 = 0.0, b1 = 0.0, b2 = 0.0;
            int valid = 0;
            for (int i = 0; i < location.Length; i++)
            {
                GeoVector w = axis ^ normal[i];
                double len = w.Length;
                if (len < 1e-6) continue; // normal (nearly) parallel to the axis
                w = (1.0 / len) * w;
                double g = (location[i] - seed) * w;
                double a = u * w;
                double b = v * w;
                a11 += a * a; a12 += a * b; a22 += b * b;
                b1 += a * g; b2 += b * g;
                valid++;
            }
            if (valid < 3) return false;
            double det = a11 * a22 - a12 * a12;
            if (Math.Abs(det) < 1e-9 * valid * valid) return false; // all normals in (nearly) the same meridian plane
            double alpha = (b1 * a22 - b2 * a12) / det;
            double beta = (a11 * b2 - a12 * b1) / det;
            axisPoint = seed + alpha * u + beta * v;
            return true;
        }

        /// <summary>
        /// Least squares point closest to all lines (location[i], direction[i]). Returns false if the system is
        /// (nearly) singular, which is the case for parallel normals (plane) or normals through a common axis (cylinder).
        /// </summary>
        private static bool NormalLinesIntersection(GeoPoint[] location, GeoVector[] direction, out GeoPoint intersection)
        {
            // minimize sum of squared distances to the lines: (sum (I - n*nT)) x = sum (I - n*nT) p
            double[,] m = new double[3, 3];
            double[] b = new double[3];
            for (int i = 0; i < location.Length; i++)
            {
                GeoVector n = direction[i];
                double[,] q = new double[3, 3] {
                    { 1 - n.x * n.x, -n.x * n.y, -n.x * n.z },
                    { -n.y * n.x, 1 - n.y * n.y, -n.y * n.z },
                    { -n.z * n.x, -n.z * n.y, 1 - n.z * n.z } };
                for (int r = 0; r < 3; r++)
                {
                    for (int c = 0; c < 3; c++) m[r, c] += q[r, c];
                    b[r] += q[r, 0] * location[i].x + q[r, 1] * location[i].y + q[r, 2] * location[i].z;
                }
            }
            intersection = GeoPoint.Origin;
            // Gaussian elimination with partial pivoting
            int[] perm = { 0, 1, 2 };
            double scale = Math.Abs(m[0, 0]) + Math.Abs(m[1, 1]) + Math.Abs(m[2, 2]);
            if (scale == 0.0) return false;
            for (int col = 0; col < 3; col++)
            {
                int pivot = col;
                for (int r = col + 1; r < 3; r++) if (Math.Abs(m[r, col]) > Math.Abs(m[pivot, col])) pivot = r;
                if (Math.Abs(m[pivot, col]) < 1e-8 * scale) return false; // singular: no well defined intersection point
                if (pivot != col)
                {
                    for (int c = 0; c < 3; c++) { double tmp = m[col, c]; m[col, c] = m[pivot, c]; m[pivot, c] = tmp; }
                    double tb = b[col]; b[col] = b[pivot]; b[pivot] = tb;
                }
                for (int r = col + 1; r < 3; r++)
                {
                    double f = m[r, col] / m[col, col];
                    for (int c = col; c < 3; c++) m[r, c] -= f * m[col, c];
                    b[r] -= f * b[col];
                }
            }
            double z = b[2] / m[2, 2];
            double y = (b[1] - m[1, 2] * z) / m[1, 1];
            double x = (b[0] - m[0, 1] * y - m[0, 2] * z) / m[0, 0];
            intersection = new GeoPoint(x, y, z);
            return true;
        }

        /// <summary>distinct vertex positions of the region, sampled down to at most maxFitPoints</summary>
        private GeoPoint[] RegionPoints(List<int> triangles)
        {
            HashSet<int> vertexIndices = new HashSet<int>();
            List<GeoPoint> points = new List<GeoPoint>();
            foreach (int tri in triangles)
            {
                for (int corner = 0; corner < 3; corner++)
                {
                    int vi = mesh.GetTriangleVertex(tri, corner);
                    if (vertexIndices.Add(vi)) points.Add(mesh.GetVertex(vi));
                }
            }
            if (points.Count <= maxFitPoints) return points.ToArray();
            GeoPoint[] res = new GeoPoint[maxFitPoints];
            double step = points.Count / (double)maxFitPoints;
            for (int i = 0; i < maxFitPoints; i++) res[i] = points[(int)(i * step)];
            return res;
        }

        private static double MaxDistance(ISurface surface, GeoPoint[] pnts)
        {
            double maxDist = 0.0;
            for (int i = 0; i < pnts.Length; i++)
            {
                double d = Math.Abs(surface.GetDistance(pnts[i]));
                if (d > maxDist) maxDist = d;
            }
            return maxDist;
        }
        #endregion

        #region tube marching (torus bootstrapping)
        // A slim torus reads locally as a cylinder (of the tube/minor radius). Marching along the tube and fitting a
        // local cylinder at each step yields a sequence of axis directions that rotate coplanarly - their common
        // normal is the torus major axis Z (robust, unlike a direct torus fit on a small patch). The axis lines are
        // tangent to the tube centerline circle, so a circle fit of the axis samples recovers the major radius and
        // center. Several tangentially connected tori appear as one continuous march and are split where the
        // centerline curvature (circle fit) breaks. These routines feed the torus growth (recognition stage).

        private const int tubeDiskSize = 40;        // triangles per local cylinder disk
        private const int tubeMaxSteps = 200;       // safety cap for the march (also stops on loop closure)
        private const int torusFitMinPoints = 400;  // only refine the torus axis over at least this many points (needs a wide arc)

        /// <summary>One local cylinder fitted to a small disk while marching along a tube.</summary>
        private class TubeSample
        {
            public int Triangle;      // the disk's center triangle (seed for the later torus growth)
            public GeoPoint Center;   // centroid of the disk
            public GeoPoint AxisPoint;// a point on the local cylinder axis
            public GeoVector AxisDir; // local cylinder axis direction, oriented along the march
            public double Radius;     // local tube (minor) radius
            public double Residual;   // local cylinder fit residual
        }

        /// <summary>A torus recovered from a run of tube samples with a consistent centerline circle.</summary>
        private class TorusFromTube
        {
            public GeoPoint Center;
            public GeoVector Axis;
            public double MajorRadius;
            public double MinorRadius;
            public double CircleResidual; // max deviation of the axis samples from the centerline circle
            public List<TubeSample> Samples;
        }

        /// <summary>
        /// Ungated least squares cylinder fit to a small triangle disk: returns a point on the axis, the axis
        /// direction, the radius and the maximum residual. The axis direction candidates are the same as in
        /// <see cref="SurfaceCandidates"/> (Gauss map, mean normal cross spread, normal lines); the best (smallest
        /// residual) is returned. No tolerance gate - on a tube of a torus the residual exceeds the tolerance, but
        /// the axis direction is what matters here.
        /// </summary>
        private bool FitLocalCylinder(List<int> triangles, out GeoPoint axisPoint, out GeoVector axisDir, out double radius, out double residual)
        {
            axisPoint = GeoPoint.Origin; axisDir = GeoVector.ZAxis; radius = double.NaN; residual = double.NaN;
            GeoPoint[] pnts = RegionPoints(triangles);
            if (pnts.Length < 4) return false;
            GeoPoint centroid = new GeoPoint(pnts);
            int n = Math.Min(triangles.Count, maxFitPoints);
            GeoPoint[] ndirs = new GeoPoint[n]; GeoVector[] triNormals = new GeoVector[n]; GeoPoint[] centroids = new GeoPoint[n];
            GeoVector meanNormal = GeoVector.NullVector; double step = triangles.Count / (double)n;
            for (int i = 0; i < n; i++)
            {
                int tri = triangles[(int)(i * step)];
                triNormals[i] = mesh.GetNormal(tri);
                meanNormal = meanNormal + triNormals[i];
                ndirs[i] = GeoPoint.Origin + triNormals[i];
                centroids[i] = new GeoPoint(mesh.GetTrianglePoint(tri, 0), mesh.GetTrianglePoint(tri, 1), mesh.GetTrianglePoint(tri, 2));
            }
            List<GeoVector> axisCandidates = new List<GeoVector>();
            double gaussDist = PcaPlaneFit(ndirs, out GeoPoint _, out GeoVector gaussNormal, out GeoVector gaussSpread);
            if (gaussDist < 0.3) AddAxisCandidate(axisCandidates, gaussNormal);
            if (!meanNormal.IsNullVector() && gaussDist != double.MaxValue)
            {
                GeoVector cross = meanNormal.Normalized ^ gaussSpread;
                if (cross.Length > 1e-6) AddAxisCandidate(axisCandidates, cross.Normalized);
            }
            if (AxisFromNormalLines(centroids, triNormals, out GeoVector nlAxis)) AddAxisCandidate(axisCandidates, nlAxis);
            foreach (GeoVector axis in axisCandidates)
            {
                Plane perp = new Plane(centroid, axis);
                GeoPoint2D[] p2d = new GeoPoint2D[pnts.Length];
                for (int i = 0; i < pnts.Length; i++) p2d[i] = perp.Project(pnts[i]);
                Geometry.CircleFitLs(p2d, out GeoPoint2D center2d, out double rad);
                if (double.IsNaN(rad) || double.IsInfinity(rad) || rad <= precision) continue;
                GeoPoint ap = perp.ToGlobal(center2d);
                axis.ArbitraryNormals(out GeoVector dx, out GeoVector dy);
                CylindricalSurface cs = new CylindricalSurface(ap, rad * dx, rad * dy, axis);
                double res = MaxDistance(cs, pnts);
                if (double.IsNaN(residual) || res < residual) { residual = res; radius = rad; axisPoint = ap; axisDir = axis.Normalized; }
            }
            return !double.IsNaN(radius);
        }

        /// <summary>
        /// Marches along a tube starting at <paramref name="startTriangle"/>, fitting a local cylinder at each step
        /// and stepping along the (consistently oriented) axis direction by a multiple of the local radius. Stops
        /// at an open border, when the local fit fails, on loop closure (a disk center repeats) or at
        /// <see cref="tubeMaxSteps"/>. The returned samples span the tube and their rotating axes encode the torus.
        /// </summary>
        private List<TubeSample> MarchTube(int startTriangle, HashSet<int> allowed)
        {
            List<TubeSample> samples = new List<TubeSample>();
            HashSet<int> usedCenters = new HashSet<int>();
            int current = startTriangle;
            GeoVector marchDir = GeoVector.NullVector;
            double stepLen = 0.0;
            for (int k = 0; k < tubeMaxSteps; k++)
            {
                if (!usedCenters.Add(current)) break; // loop closed / revisited a center
                List<int> disk = TriangleDisk(current, allowed, tubeDiskSize);
                if (!FitLocalCylinder(disk, out GeoPoint ap, out GeoVector T, out double r, out double res)) break;
                if (k > 0 && T * marchDir < 0) T = -T; // keep a consistent rotational sense
                GeoPoint center = RegionCentroid(disk);
                samples.Add(new TubeSample { Triangle = current, Center = center, AxisPoint = ap, AxisDir = T, Radius = r, Residual = res });
                marchDir = T;
                if (stepLen == 0.0) stepLen = 1.0 * r; // step ~ one tube radius; set from the first (clean) sample
                GeoPoint target = center + stepLen * T;
                // walk toward the target along the mesh adjacency (the next disk is about one step away), which is
                // O(local) instead of scanning all remaining triangles per step - the dominant cost on large regions
                int next = current;
                double bestDist = TriCentroid(next) | target;
                bool improved = true;
                while (improved)
                {
                    improved = false;
                    for (int side = 0; side < 3; side++)
                    {
                        int nb = mesh.GetNeighbor(next, side);
                        if (nb < 0 || !allowed.Contains(nb)) continue;
                        double dd = TriCentroid(nb) | target;
                        if (dd < bestDist) { bestDist = dd; next = nb; improved = true; }
                    }
                }
                if (bestDist > stepLen) break; // ran off the tube
                current = next;
            }
            return samples;
        }

        /// <summary>
        /// The common normal of the tube-sample axis directions (their smallest principal axis): the torus major
        /// axis. <paramref name="coplanarity"/> is the largest |axisDir·Z| (0 = perfectly coplanar, i.e. a clean
        /// surface of revolution); a large value means the axes do not lie in a plane and there is no single torus.
        /// </summary>
        private static GeoVector RecoverTubeAxis(List<TubeSample> samples, out double coplanarity)
        {
            double[,] cov = new double[3, 3];
            foreach (TubeSample s in samples)
            {
                GeoVector t = s.AxisDir;
                cov[0, 0] += t.x * t.x; cov[0, 1] += t.x * t.y; cov[0, 2] += t.x * t.z;
                cov[1, 1] += t.y * t.y; cov[1, 2] += t.y * t.z; cov[2, 2] += t.z * t.z;
            }
            cov[1, 0] = cov[0, 1]; cov[2, 0] = cov[0, 2]; cov[2, 1] = cov[1, 2];
            Jacobi3(cov, out double[] ev, out double[,] evec);
            int smallest = 0;
            for (int i = 1; i < 3; i++) if (ev[i] < ev[smallest]) smallest = i;
            GeoVector Z = new GeoVector(evec[0, smallest], evec[1, smallest], evec[2, smallest]).Normalized;
            coplanarity = 0;
            foreach (TubeSample s in samples) coplanarity = Math.Max(coplanarity, Math.Abs(s.AxisDir.Normalized * Z));
            return Z;
        }

        /// <summary>
        /// Splits a continuous tube march into per-torus runs by the centerline curvature (1/R). Tangentially
        /// connected tori have a continuous axis direction, so a residual threshold lets a run bleed across the
        /// junction; the major radius, however, jumps there. A windowed local circle fit gives a local major radius
        /// per sample; a run is extended while that radius stays within <paramref name="relRadiusJump"/> (relative)
        /// of the run's running mean, and split where it jumps. The relative test is scale free (a 15 vs 10 torus is
        /// a 33% jump, well above the in-tube noise). Each run of at least <paramref name="minRun"/> samples is
        /// fitted with a single centerline circle and returned as a <see cref="TorusFromTube"/>; the transitional
        /// samples straddling a junction fall into short ranges that are dropped.
        /// </summary>
        private List<TorusFromTube> SegmentTubeRuns(List<TubeSample> samples, GeoVector axis, double relRadiusJump, int minRun)
        {
            List<TorusFromTube> runs = new List<TorusFromTube>();
            int n = samples.Count;
            if (n < minRun) return runs;
            axis.ArbitraryNormals(out GeoVector e1, out GeoVector e2);
            // project the axis points (which lie on the centerline, all at the same height along the axis) - using
            // their centroid as origin puts the recovered center at the correct centerline height. The surface
            // centroids would sit off the centerline plane by up to the minor radius and shift the center along Z.
            GeoPoint origin = new GeoPoint(samples.Select(s => s.AxisPoint).ToArray());
            GeoPoint2D[] pts = new GeoPoint2D[n];
            for (int i = 0; i < n; i++)
            {
                GeoVector rel = samples[i].AxisPoint - origin;
                pts[i] = new GeoPoint2D(rel * e1, rel * e2);
            }
            // local major radius and centerline center per sample from a small centered window: a run must keep
            // both the curvature (radius) and the center stable. Two tori with the same radius but different centers
            // (e.g. an S-bend that reverses curvature) share the tangent direction and radius, so only the center
            // separates them - the center flips to the other side and jumps by about twice the radius.
            const int w = 2;
            const double centerFrac = 0.5; // a center move beyond half the major radius is a junction
            double[] rloc = new double[n];
            GeoPoint2D[] cloc = new GeoPoint2D[n];
            for (int i = 0; i < n; i++)
            {
                int lo = Math.Max(0, i - w), hi = Math.Min(n - 1, i + w);
                rloc[i] = TryFitCircle(pts, lo, hi, out cloc[i], out double rl) ? rl : double.NaN;
            }
            // walk and split where either the radius or the center jumps
            List<(int from, int to)> ranges = new List<(int, int)>();
            int start = 0; double sumR = 0, sumCx = 0, sumCy = 0; int cnt = 0;
            for (int i = 0; i < n; i++)
            {
                double meanR = cnt > 0 ? sumR / cnt : rloc[i];
                double meanCx = cnt > 0 ? sumCx / cnt : cloc[i].x, meanCy = cnt > 0 ? sumCy / cnt : cloc[i].y;
                double centerMove = Math.Sqrt((cloc[i].x - meanCx) * (cloc[i].x - meanCx) + (cloc[i].y - meanCy) * (cloc[i].y - meanCy));
                bool jump = double.IsNaN(rloc[i])
                    || (cnt > 0 && Math.Abs(rloc[i] - meanR) > relRadiusJump * meanR)
                    || (cnt > 0 && centerMove > centerFrac * meanR);
                if (jump)
                {
                    if (i - start >= minRun) ranges.Add((start, i - 1));
                    start = i; sumR = 0; sumCx = 0; sumCy = 0; cnt = 0;
                    if (double.IsNaN(rloc[i])) { start = i + 1; continue; }
                }
                sumR += rloc[i]; sumCx += cloc[i].x; sumCy += cloc[i].y; cnt++;
            }
            if (n - start >= minRun) ranges.Add((start, n - 1));

            // a torus (centerline circle) from a set of samples, using the shared projection
            TorusFromTube MakeRun(List<TubeSample> rs)
            {
                GeoPoint2D[] q = new GeoPoint2D[rs.Count];
                double[] radii = new double[rs.Count];
                for (int j = 0; j < rs.Count; j++)
                {
                    GeoVector rel = rs[j].AxisPoint - origin;
                    q[j] = new GeoPoint2D(rel * e1, rel * e2);
                    radii[j] = rs[j].Radius;
                }
                if (!TryFitCircle(q, 0, q.Length - 1, out GeoPoint2D c, out double R)) return null;
                Array.Sort(radii);
                return new TorusFromTube
                {
                    Center = origin + c.x * e1 + c.y * e2,
                    Axis = axis,
                    MajorRadius = R,
                    MinorRadius = radii[radii.Length / 2],
                    CircleResidual = CircleResidual(q, 0, q.Length - 1, c, R),
                    Samples = rs
                };
            }

            // Each range becomes one torus seed. A single noisy sample can split one torus into two adjacent runs;
            // that is left as is - the two seeds grow into adjacent torus regions which the existing
            // MergeAdjacentRegions (after growth) reunites. Merging here would instead risk forcing two genuinely
            // different but similar tori together.
            foreach ((int from, int to) in ranges)
            {
                TorusFromTube run = MakeRun(samples.GetRange(from, to - from + 1));
                if (run != null) runs.Add(run);
            }
            return runs;
        }

        /// <summary>
        /// Refines the two radii and the center of a torus whose axis direction is already known (from the tube
        /// march), by a least squares circle fit in the meridian plane: each surface point is mapped to (axial
        /// position along the axis, radial distance from the axis) and a circle is fitted to that cloud - its center
        /// gives the major radius (radial) and the centerline height (axial), its radius the minor radius. The axis
        /// direction and the in-plane center are kept. This removes the dominant residual of the raw recovery (a
        /// single median minor radius) and produces a torus tight enough to hand to the surface growth.
        /// </summary>
        private void RefineTorusRadii(ref GeoPoint center, GeoVector axis, ref double majorRadius, ref double minorRadius, GeoPoint[] pnts)
        {
            GeoPoint2D[] hr = new GeoPoint2D[pnts.Length];
            for (int i = 0; i < pnts.Length; i++)
            {
                GeoVector v = pnts[i] - center;
                double h = v * axis;
                double rho = (v - h * axis).Length;
                hr[i] = new GeoPoint2D(h, rho);
            }
            Geometry.CircleFitLs(hr, out GeoPoint2D c, out double rFit);
            if (double.IsNaN(rFit) || double.IsInfinity(rFit) || rFit <= precision) return;
            majorRadius = c.y;
            minorRadius = rFit;
            center = center + c.x * axis; // shift the center to the true centerline height along the axis
        }

        /// <summary>
        /// Full least squares refinement of a torus with a fixed axis direction: Gauss-Newton on the center (3),
        /// major and minor radius (5 parameters), minimizing the point-to-surface distance
        /// f = sqrt((rho - R)^2 + h^2) - r (rho = distance from the axis, h = position along it). Unlike
        /// <see cref="RefineTorusRadii"/> this also corrects the in-plane center, which is the dominant remaining
        /// error after the raw recovery. Seeded from a good guess it converges in a few iterations.
        /// </summary>
        private void RefineTorusFull(ref GeoPoint center, GeoVector axis, ref double majorRadius, ref double minorRadius, GeoPoint[] pnts, int iterations)
        {
            GeoVector Z = axis.Normalized;
            for (int iter = 0; iter < iterations; iter++)
            {
                double[,] A = new double[5, 5]; double[] g = new double[5];
                foreach (GeoPoint p in pnts)
                {
                    GeoVector v = p - center;
                    double h = v * Z;
                    GeoVector w = v - h * Z;
                    double rho = w.Length;
                    if (rho < 1e-9) continue;
                    GeoVector wh = (1.0 / rho) * w;
                    double dR = rho - majorRadius;
                    double d = Math.Sqrt(dR * dR + h * h);
                    if (d < 1e-12) continue;
                    double f = d - minorRadius;
                    GeoVector gc = (-1.0 / d) * (dR * wh + h * Z); // d f / d center
                    double[] J = { gc.x, gc.y, gc.z, (majorRadius - rho) / d, -1.0 };
                    for (int a = 0; a < 5; a++) { for (int b = 0; b < 5; b++) A[a, b] += J[a] * J[b]; g[a] += J[a] * f; }
                }
                for (int k = 0; k < 5; k++) A[k, k] *= 1.0 + 1e-6; // mild damping against singularity
                double[] dx = new double[5];
                if (!GaussSolve(A, g, dx, 5)) break;
                center = center - new GeoVector(dx[0], dx[1], dx[2]);
                majorRadius -= dx[3]; minorRadius -= dx[4];
                if (minorRadius <= 0 || majorRadius <= 0) break;
                if (Math.Abs(dx[0]) + Math.Abs(dx[1]) + Math.Abs(dx[2]) + Math.Abs(dx[3]) + Math.Abs(dx[4]) < 1e-6 * precision) break;
            }
        }

        /// <summary>Solves the n x n linear system A x = b by Gaussian elimination with partial pivoting.</summary>
        private static bool GaussSolve(double[,] A, double[] b, double[] x, int n)
        {
            double[,] m = (double[,])A.Clone();
            double[] rhs = (double[])b.Clone();
            for (int col = 0; col < n; col++)
            {
                int piv = col;
                for (int r = col + 1; r < n; r++) if (Math.Abs(m[r, col]) > Math.Abs(m[piv, col])) piv = r;
                if (Math.Abs(m[piv, col]) < 1e-20) return false;
                if (piv != col)
                {
                    for (int c = 0; c < n; c++) { double t = m[col, c]; m[col, c] = m[piv, c]; m[piv, c] = t; }
                    double tb = rhs[col]; rhs[col] = rhs[piv]; rhs[piv] = tb;
                }
                for (int r = col + 1; r < n; r++)
                {
                    double fct = m[r, col] / m[col, col];
                    for (int c = col; c < n; c++) m[r, c] -= fct * m[col, c];
                    rhs[r] -= fct * rhs[col];
                }
            }
            for (int r = n - 1; r >= 0; r--)
            {
                double s = rhs[r];
                for (int c = r + 1; c < n; c++) s -= m[r, c] * x[c];
                x[r] = s / m[r, r];
            }
            return true;
        }

        /// <summary>
        /// Least squares torus fit with MathNet's Levenberg-Marquardt minimizer, refining all torus parameters
        /// including the axis direction. Parametrized by the center (3), the axis vector n whose length is the major
        /// radius (3) and the minor radius (1); the residual is the true signed point-to-surface distance, so the
        /// problem is well scaled and needs no error-tolerance tuning. Seeded from the tube-march estimate it
        /// converges in a few iterations. Returns null on failure or when the result is not a ring torus.
        /// (This replaces GaussNewtonMinimizer.TorusFit, whose absolute error tolerance - precision^2 compared
        /// against the summed squared r^2-d^2 residuals, which scale with 4*r^2*pointCount - is never reached, so it
        /// reported failure even for good fits.)
        /// </summary>
        private ToroidalSurface FitTorusLM(GeoPoint[] pnts, GeoPoint center, GeoVector axisDir, double majorRadius, double minorRadius)
        {
            majorRadius = Math.Abs(majorRadius); // a folded spindle profile can seed a negative major radius
            MathNet.Numerics.LinearAlgebra.Vector<double> model(MathNet.Numerics.LinearAlgebra.Vector<double> p, MathNet.Numerics.LinearAlgebra.Vector<double> x)
            {
                GeoPoint c = new GeoPoint(p[0], p[1], p[2]);
                GeoVector n = new GeoVector(p[3], p[4], p[5]);
                double R = n.Length;
                double r = p[6];
                double[] res = new double[pnts.Length];
                if (R > 1e-9)
                {
                    GeoVector nhat = (1.0 / R) * n;
                    for (int i = 0; i < pnts.Length; i++)
                    {
                        GeoVector d = pnts[i] - c;
                        double h = d * nhat;                         // position along the axis
                        double rho2 = d * d - h * h;                 // squared distance from the axis
                        double rho = rho2 > 0 ? Math.Sqrt(rho2) : 0.0;
                        // distance to the torus surface. For a self-intersecting (spindle) torus the generating arc
                        // lies across the axis, so a surface point may match the generating circle on the far side of
                        // the axis (distance rho+R) rather than the near side (rho-R); the nearer of the two is the
                        // true surface distance and reduces to rho-R for an ordinary ring torus.
                        double e1 = Math.Sqrt((rho - R) * (rho - R) + h * h) - r;
                        double e2 = Math.Sqrt((rho + R) * (rho + R) + h * h) - r;
                        res[i] = Math.Abs(e1) <= Math.Abs(e2) ? e1 : e2;
                    }
                }
                return MathNet.Numerics.LinearAlgebra.CreateVector.Dense(res);
            }
            try
            {
                GeoVector n0 = majorRadius * axisDir.Normalized;
                MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer minimizer =
                    new MathNet.Numerics.Optimization.LevenbergMarquardtMinimizer(0.001, 1e-18, 1e-18, 1e-18, 100);
                var obj = MathNet.Numerics.Optimization.ObjectiveFunction.NonlinearModel(model,
                    MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(pnts.Length),
                    MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(pnts.Length));
                var start = MathNet.Numerics.LinearAlgebra.CreateVector.Dense<double>(new double[] { center.x, center.y, center.z, n0.x, n0.y, n0.z, minorRadius });
                var result = minimizer.FindMinimum(obj, start);
                var pr = result.MinimizingPoint;
                GeoPoint loc = new GeoPoint(pr[0], pr[1], pr[2]);
                GeoVector normal = new GeoVector(pr[3], pr[4], pr[5]);
                double majrad = normal.Length;
                double minrad = Math.Abs(pr[6]);
                // a self-intersecting (spindle) torus with minor > major is allowed here: the tube march runs on the
                // leftover after the real cylinders are already extracted, so there is no cylinder to over-fit
                if (majrad <= precision || minrad <= precision) return null;
                Plane pln = new Plane(loc, normal);
                return new ToroidalSurface(pln.Location, pln.DirectionX, pln.DirectionY, pln.Normal, majrad, minrad);
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Fits a torus to the given points, seeded with the tube-march estimate (center, axis direction, major and
        /// minor radius). Two stages: first a cheap polish that keeps the (robustly recovered) axis fixed and
        /// corrects center and radii, which is always available as a fallback; then the full all-parameter
        /// Gauss-Newton fit <see cref="GaussNewtonMinimizer.TorusFit"/> which also refines the axis direction. The
        /// STL points are rounded to the file precision, but that rounding averages out over the many points, so the
        /// all-parameter fit reaches the true CAD torus; seeded from the polished values it converges in a couple of
        /// iterations. Returns the more accurate of the two (or null when neither is a valid ring torus).
        /// </summary>
        private ToroidalSurface FitTorus(GeoPoint center, GeoVector axis, double majorRadius, double minorRadius, GeoPoint[] pnts)
        {
            GeoPoint c = center; GeoVector Z = axis.Normalized; double R = majorRadius, r = minorRadius;
            RefineTorusRadii(ref c, Z, ref R, ref r, pnts);
            RefineTorusFull(ref c, Z, ref R, ref r, pnts, 10);
            ToroidalSurface fixedAxis = null;
            if (r > precision && R > precision) // r may exceed R (a self-intersecting spindle torus)
            {
                Z.ArbitraryNormals(out GeoVector dxf, out GeoVector dyf);
                fixedAxis = new ToroidalSurface(c, dxf, dyf, Z, R, r);
            }
            // The all-parameter fit also refines the axis direction, which needs a large, widely spanning support to
            // be well conditioned: on a small patch (a short arc) it would tilt the axis onto the patch - the very
            // small-patch overfit the fixed march axis avoids. So run it only once enough points are collected
            // (a nearly full turn of a tube); until then keep the robust march axis. TorusFit encodes the major
            // radius as the length of the axis vector.
            if (pnts.Length >= torusFitMinPoints)
            {
                // seed from the original estimate, not from the fixed-axis polish above: that polish (meridian
                // circle fit / fixed-axis Gauss-Newton) is only correct for ring tori, and would hand a corrupted
                // seed to the all-parameter fit for a spindle torus
                ToroidalSurface gnTs = FitTorusLM(pnts, center, axis.Normalized, majorRadius, minorRadius);
                if (gnTs != null && (fixedAxis == null || MaxDistance(gnTs, pnts) <= MaxDistance(fixedAxis, pnts))) return gnTs;
            }
            return fixedAxis;
        }

        /// <summary>
        /// Builds a toroidal surface from a recovered tube run and fits it over the union of its sample disks.
        /// Returns null (with maxError = MaxValue) when the result is not a ring torus.
        /// </summary>
        private ToroidalSurface BuildRefinedTorus(TorusFromTube run, GeoPoint[] unionPoints, out double maxError)
        {
            ToroidalSurface ts = FitTorus(run.Center, run.Axis, run.MajorRadius, run.MinorRadius, unionPoints);
            maxError = ts != null ? MaxDistance(ts, unionPoints) : double.MaxValue;
            return ts;
        }

        private static bool TryFitCircle(GeoPoint2D[] pts, int from, int to, out GeoPoint2D center, out double radius)
        {
            GeoPoint2D[] sub = new GeoPoint2D[to - from + 1];
            for (int i = from; i <= to; i++) sub[i - from] = pts[i];
            Geometry.CircleFitLs(sub, out center, out radius);
            return !double.IsNaN(radius) && !double.IsInfinity(radius) && radius > 0;
        }

        private static double CircleResidual(GeoPoint2D[] pts, int from, int to, GeoPoint2D center, double radius)
        {
            double res = 0;
            for (int i = from; i <= to; i++)
            {
                double d = Math.Sqrt((pts[i].x - center.x) * (pts[i].x - center.x) + (pts[i].y - center.y) * (pts[i].y - center.y));
                res = Math.Max(res, Math.Abs(d - radius));
            }
            return res;
        }

        /// <summary>A connected disk of at most <paramref name="size"/> triangles grown from a start triangle.</summary>
        private List<int> TriangleDisk(int startTri, HashSet<int> allowed, int size)
        {
            HashSet<int> visited = new HashSet<int> { startTri };
            Queue<int> q = new Queue<int>(); q.Enqueue(startTri);
            while (q.Count > 0 && visited.Count < size)
            {
                int t = q.Dequeue();
                for (int side = 0; side < 3; side++)
                {
                    int nb = mesh.GetNeighbor(t, side);
                    if (nb >= 0 && allowed.Contains(nb) && visited.Add(nb)) q.Enqueue(nb);
                }
            }
            return new List<int>(visited);
        }

        private GeoPoint TriCentroid(int tri) => new GeoPoint(mesh.GetTrianglePoint(tri, 0), mesh.GetTrianglePoint(tri, 1), mesh.GetTrianglePoint(tri, 2));

        private GeoPoint RegionCentroid(List<int> triangles)
        {
            GeoPoint[] c = new GeoPoint[triangles.Count];
            for (int i = 0; i < triangles.Count; i++) c[i] = TriCentroid(triangles[i]);
            return new GeoPoint(c);
        }

        #endregion

        #region freeform NURBS surfaces
        /// <summary>
        /// Maximum allowed distance of the region vertices from a fitted freeform NURBS surface for it to be
        /// accepted. A freeform patch approximates a noisy triangle mesh (and its grid resampling smooths it), so
        /// this is more generous than <see cref="Tolerance"/>. Regions that do not reach it stay unrecognized.
        /// </summary>
        public double NurbsTolerance { get; set; }

        /// <summary>
        /// The steepest surface slope (as the cosine of the angle between a triangle normal and the projection
        /// plane normal) still treated as a valid height field. A triangle steeper than this folds the projection,
        /// so the region cannot be parametrized over that plane; it is then left unrecognized (a cylinder- or
        /// sphere-based parametrization for such strongly curved patches is future work).
        /// </summary>
        private const double nurbsMaxSlopeCos = 0.2; // ~78 degrees

        /// <summary>Cubic in both directions - a good default for a smooth freeform patch.</summary>
        private const int nurbsDegree = 3;

        /// <summary>
        /// Upper bound on the grid nodes (and hence NURBS poles) per direction. The natural resolution is one node
        /// per mean triangle edge (finer than the mesh adds no information); this only caps very large regions to
        /// keep the pole net and the downstream cost bounded.
        /// </summary>
        private const int nurbsMaxGrid = 60;

        /// <summary>Grid overshoot beyond the region border, in cells, so the surface extends for the later trimming.</summary>
        private const double nurbsOvershoot = 1.5;

        /// <summary>
        /// Turns unrecognized regions into freeform NURBS surfaces where possible. This handles the "hard edge"
        /// case: the region is separated from its neighbors by a crease, so the NURBS surface may safely overshoot
        /// the triangle border (the later surface intersection trims it to clean edges). See
        /// <see cref="TryFitNurbsRegion"/> for the method. Regions that cannot be represented as a height field over
        /// a single plane (they fold) are left unrecognized.
        /// </summary>
        private void FitNurbsToUnrecognizedRegions()
        {
            if (NurbsTolerance <= 0.0) NurbsTolerance = 3.0 * Tolerance;
            foreach (RecognizedRegion region in Regions)
            {
                if (region.Surface != null) continue; // only unrecognized regions
                if (TryFitNurbsRegion(region, out NurbsSurface nurbs, out double maxError))
                {
                    region.Surface = nurbs;
                    region.Kind = RecognizedSurfaceKind.Nurbs;
                    region.MaxError = maxError;
                    region.InvalidateExtent();
                }
            }
        }

        /// <summary>
        /// Tries to fit a freeform NURBS surface to a region. The region vertices are projected onto their PCA plane;
        /// when the projection does not fold (all triangles face the same side of the plane, so the region is a
        /// height field over it), a regular grid is laid out on the plane - slightly larger than the region, so the
        /// surface overshoots the border for the later trimming. Each grid node gets its 3D point from the triangle
        /// its plane position falls into (barycentric interpolation); nodes beyond the border extrapolate from the
        /// nearest triangle's plane. A NURBS surface is interpolated through the grid and accepted when all region
        /// vertices stay within <see cref="NurbsTolerance"/>. Returns false (and leaves the region unrecognized) when
        /// the region folds over the plane or the interpolated surface does not fit.
        /// </summary>
        private bool TryFitNurbsRegion(RecognizedRegion region, out NurbsSurface nurbs, out double maxError)
        {
            nurbs = null;
            maxError = double.MaxValue;
            GeoPoint[] pnts = RegionPoints(region.Triangles);
            if (pnts.Length < (nurbsDegree + 1) * (nurbsDegree + 1)) return false; // too few points for a meaningful patch
            // 1. projection plane and its in-plane axes (u along the largest spread, v perpendicular, w the normal)
            if (PcaPlaneFit(pnts, out GeoPoint loc, out GeoVector w, out GeoVector u) == double.MaxValue) return false;
            w = w.Normalized;
            u = u.Normalized;
            GeoVector v = (w ^ u).Normalized;
            // 2. fold check: every triangle must face the same side of the plane (a consistent graph over it) and not
            // be too steep; otherwise the region cannot be represented as a height field over this plane
            double signSum = 0.0;
            foreach (int tri in region.Triangles) signSum += (mesh.GetNormal(tri) * w) * mesh.GetArea(tri);
            double sign = signSum >= 0.0 ? 1.0 : -1.0;
            foreach (int tri in region.Triangles)
                if (sign * (mesh.GetNormal(tri) * w) < nurbsMaxSlopeCos) return false;
            // 3. project the region triangles onto the plane and record their 2D bounding boxes for fast lookup
            int nt = region.Triangles.Count;
            GeoPoint2D[][] proj2d = new GeoPoint2D[nt][];
            GeoPoint[][] pts3d = new GeoPoint[nt][];
            double[] minx = new double[nt], miny = new double[nt], maxx = new double[nt], maxy = new double[nt];
            BoundingRect box = BoundingRect.EmptyBoundingRect;
            for (int i = 0; i < nt; i++)
            {
                int tri = region.Triangles[i];
                GeoPoint2D[] c2 = new GeoPoint2D[3];
                GeoPoint[] c3 = new GeoPoint[3];
                for (int corner = 0; corner < 3; corner++)
                {
                    GeoPoint p = mesh.GetTrianglePoint(tri, corner);
                    GeoVector d = p - loc;
                    c2[corner] = new GeoPoint2D(d * u, d * v);
                    c3[corner] = p;
                    box.MinMax(c2[corner]);
                }
                proj2d[i] = c2;
                pts3d[i] = c3;
                minx[i] = Math.Min(c2[0].x, Math.Min(c2[1].x, c2[2].x));
                maxx[i] = Math.Max(c2[0].x, Math.Max(c2[1].x, c2[2].x));
                miny[i] = Math.Min(c2[0].y, Math.Min(c2[1].y, c2[2].y));
                maxy[i] = Math.Max(c2[0].y, Math.Max(c2[1].y, c2[2].y));
            }
            // 4. grid resolution from the mesh density (one node per mean triangle edge)
            double cell = MeanRegionEdge(region.Triangles);
            if (cell <= 0.0) return false;
            BoundingRect coreBox = box; // the true region extent (before any overshoot)

            // builds the grid at the given resolution cap and overshoot (in cells) and returns the interpolated
            // surface with its max error over the region vertices
            NurbsSurface buildFit(int cap, double overshootCells, out double err)
            {
                err = double.MaxValue;
                BoundingRect gb = coreBox;
                gb.Inflate(overshootCells * cell, overshootCells * cell);
                int gnu = Math.Max(nurbsDegree + 1, Math.Min(cap, (int)Math.Round(gb.Width / cell) + 1));
                int gnv = Math.Max(nurbsDegree + 1, Math.Min(cap, (int)Math.Round(gb.Height / cell) + 1));
                double du = gb.Width / (gnu - 1), dv = gb.Height / (gnv - 1);
                GeoPoint[,] g = new GeoPoint[gnu, gnv];
                for (int i = 0; i < gnu; i++)
                {
                    double su = gb.Left + i * du;
                    for (int j = 0; j < gnv; j++)
                        g[i, j] = SampleHeightField(new GeoPoint2D(su, gb.Bottom + j * dv), proj2d, pts3d, minx, miny, maxx, maxy);
                }
                NurbsSurface ns;
                try { ns = new NurbsSurface(g, nurbsDegree, nurbsDegree, false, false); }
                catch (Exception) { return null; }
                err = MaxDistance(ns, pnts);
                return ns;
            }

            // 5./6. build, sample and interpolate the grid, then validate against the region vertices
            nurbs = buildFit(nurbsMaxGrid, nurbsOvershoot, out maxError);
            if (nurbs == null) return false;
            return maxError <= NurbsTolerance;
        }

        /// <summary>
        /// The 3D point on the region surface at the plane position <paramref name="q"/>: the point of the triangle
        /// whose projection contains <paramref name="q"/> (barycentric interpolation), or - for a position beyond the
        /// region border - an extrapolation from the plane of the nearest triangle.
        /// </summary>
        private static GeoPoint SampleHeightField(GeoPoint2D q, GeoPoint2D[][] proj2d, GeoPoint[][] pts3d,
            double[] minx, double[] miny, double[] maxx, double[] maxy)
        {
            // inside case: only triangles whose 2D bounding box contains q can contain it
            for (int i = 0; i < proj2d.Length; i++)
            {
                if (q.x < minx[i] || q.x > maxx[i] || q.y < miny[i] || q.y > maxy[i]) continue;
                Barycentric(q, proj2d[i], out double a, out double b, out double c);
                if (a >= -1e-9 && b >= -1e-9 && c >= -1e-9) return Combine(pts3d[i], a, b, c);
            }
            // outside all triangles: extrapolate from the nearest triangle (its barycentric coordinates run outside
            // [0,1], which extends that triangle's plane linearly - good enough, the overshoot is trimmed away later)
            int nearest = 0;
            double nearestDist = double.MaxValue;
            for (int i = 0; i < proj2d.Length; i++)
            {
                double d = Dist2DTriangle(q, proj2d[i]);
                if (d < nearestDist) { nearestDist = d; nearest = i; }
            }
            Barycentric(q, proj2d[nearest], out double ea, out double eb, out double ec);
            return Combine(pts3d[nearest], ea, eb, ec);
        }

        /// <summary>The point a*t0 + b*t1 + c*t2 (barycentric combination of the three triangle corners).</summary>
        private static GeoPoint Combine(GeoPoint[] t, double a, double b, double c)
        {
            return new GeoPoint(a * t[0].x + b * t[1].x + c * t[2].x,
                                a * t[0].y + b * t[1].y + c * t[2].y,
                                a * t[0].z + b * t[1].z + c * t[2].z);
        }

        /// <summary>Barycentric coordinates of <paramref name="p"/> with respect to the 2D triangle <paramref name="t"/>.</summary>
        private static void Barycentric(GeoPoint2D p, GeoPoint2D[] t, out double a, out double b, out double c)
        {
            double denom = (t[1].y - t[2].y) * (t[0].x - t[2].x) + (t[2].x - t[1].x) * (t[0].y - t[2].y);
            if (Math.Abs(denom) < 1e-30) { a = b = c = 1.0 / 3.0; return; } // degenerate 2D triangle
            a = ((t[1].y - t[2].y) * (p.x - t[2].x) + (t[2].x - t[1].x) * (p.y - t[2].y)) / denom;
            b = ((t[2].y - t[0].y) * (p.x - t[2].x) + (t[0].x - t[2].x) * (p.y - t[2].y)) / denom;
            c = 1.0 - a - b;
        }

        /// <summary>Distance of the 2D point <paramref name="p"/> from the triangle <paramref name="t"/> (0 if inside).</summary>
        private static double Dist2DTriangle(GeoPoint2D p, GeoPoint2D[] t)
        {
            Barycentric(p, t, out double a, out double b, out double c);
            if (a >= 0.0 && b >= 0.0 && c >= 0.0) return 0.0;
            return Math.Min(Dist2DSegment(p, t[0], t[1]), Math.Min(Dist2DSegment(p, t[1], t[2]), Dist2DSegment(p, t[2], t[0])));
        }

        /// <summary>Distance of the 2D point <paramref name="p"/> from the segment a-b.</summary>
        private static double Dist2DSegment(GeoPoint2D p, GeoPoint2D a, GeoPoint2D b)
        {
            double vx = b.x - a.x, vy = b.y - a.y;
            double wx = p.x - a.x, wy = p.y - a.y;
            double len2 = vx * vx + vy * vy;
            double t = len2 > 0.0 ? (wx * vx + wy * vy) / len2 : 0.0;
            if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
            double dx = wx - t * vx, dy = wy - t * vy;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>The mean edge length of the region triangles (a measure of the local mesh density).</summary>
        private double MeanRegionEdge(List<int> triangles)
        {
            double sum = 0.0;
            int count = 0;
            foreach (int tri in triangles)
            {
                GeoPoint p0 = mesh.GetTrianglePoint(tri, 0), p1 = mesh.GetTrianglePoint(tri, 1), p2 = mesh.GetTrianglePoint(tri, 2);
                sum += (p0 | p1) + (p1 | p2) + (p2 | p0);
                count += 3;
            }
            return count > 0 ? sum / count : 0.0;
        }
        #endregion

        #region raw faces for visualization
        private Face MakeRawFace(RecognizedRegion region)
        {
            try
            {
                GeoPoint[] pnts = RegionPoints(region.Triangles);
                List<double> us = new List<double>(pnts.Length);
                List<double> vs = new List<double>(pnts.Length);
                for (int i = 0; i < pnts.Length; i++)
                {
                    GeoPoint2D uv = region.Surface.PositionOf(pnts[i]);
                    if (double.IsNaN(uv.x) || double.IsNaN(uv.y) || double.IsInfinity(uv.x) || double.IsInfinity(uv.y)) continue;
                    us.Add(uv.x);
                    vs.Add(uv.y);
                }
                if (us.Count < 3) return null;
                if (region.Surface.IsUPeriodic) AdjustPeriodic(us, region.Surface.UPeriod);
                if (region.Surface.IsVPeriodic) AdjustPeriodic(vs, region.Surface.VPeriod);
                BoundingRect br = BoundingRect.EmptyBoundingRect;
                for (int i = 0; i < us.Count; i++) br.MinMax(new GeoPoint2D(us[i], vs[i]));
                if (region.Surface.IsUPeriodic && br.Width > region.Surface.UPeriod) br.Right = br.Left + region.Surface.UPeriod;
                if (region.Surface.IsVPeriodic && br.Height > region.Surface.VPeriod) br.Top = br.Bottom + region.Surface.VPeriod;
                double minSize = 1e-8;
                if (br.Width < minSize) br.Inflate(minSize, 0.0);
                if (br.Height < minSize) br.Inflate(0.0, minSize);
                return Face.MakeFace(region.Surface, br);
            }
            catch (Exception)
            {
                return null; // the raw face is only for visualization, a failure here should not break the import
            }
        }

        /// <summary>
        /// Moves periodic parameter values into a common branch: the branch cut is placed into the largest gap of
        /// the value distribution, so that a region crossing the natural seam gets a connected parameter interval.
        /// </summary>
        private static void AdjustPeriodic(List<double> values, double period)
        {
            for (int i = 0; i < values.Count; i++)
            {
                double v = values[i] % period;
                if (v < 0) v += period;
                values[i] = v;
            }
            List<double> sorted = new List<double>(values);
            sorted.Sort();
            double maxGap = sorted[0] + period - sorted[sorted.Count - 1]; // the gap across the period boundary
            double branchStart = sorted[0];
            for (int i = 1; i < sorted.Count; i++)
            {
                double gap = sorted[i] - sorted[i - 1];
                if (gap > maxGap)
                {
                    maxGap = gap;
                    branchStart = sorted[i];
                }
            }
            for (int i = 0; i < values.Count; i++)
            {
                if (values[i] < branchStart) values[i] += period;
            }
        }

        private void AddTriangleFaces(GeoObjectList list, List<int> triangles, ColorDef color)
        {
            const int maxTriangleFaces = 10000; // don't flood the list when a huge freeform region was not recognized
            int count = Math.Min(triangles.Count, maxTriangleFaces);
            for (int i = 0; i < count; i++)
            {
                try
                {
                    Face fc = Face.MakeFace(mesh.GetTrianglePoint(triangles[i], 0), mesh.GetTrianglePoint(triangles[i], 1), mesh.GetTrianglePoint(triangles[i], 2));
                    fc.ColorDef = color;
                    list.Add(fc);
                }
                catch (Exception) { } // skip degenerate triangles
            }
        }

        private static int colorCounter = 0;
        private static ColorDef ColorFor(RecognizedSurfaceKind kind, Random rnd)
        {
            int r, g, b;
            switch (kind)
            {
                case RecognizedSurfaceKind.Plane: r = 144; g = 238; b = 144; break; // light green
                case RecognizedSurfaceKind.Cylinder: r = 100; g = 149; b = 237; break; // cornflower blue
                case RecognizedSurfaceKind.Cone: r = 255; g = 165; b = 0; break; // orange
                case RecognizedSurfaceKind.Sphere: r = 205; g = 92; b = 92; break; // indian red
                case RecognizedSurfaceKind.Torus: r = 186; g = 85; b = 211; break; // orchid
                default: r = 150; g = 150; b = 150; break; // gray
            }
            // slight variation, so adjacent faces of the same kind can be distinguished
            r = Math.Max(0, Math.Min(255, r + rnd.Next(-25, 26)));
            g = Math.Max(0, Math.Min(255, g + rnd.Next(-25, 26)));
            b = Math.Max(0, Math.Min(255, b + rnd.Next(-25, 26)));
            Color clr = Color.FromArgb(r, g, b);
            return new ColorDef(kind.ToString() + (++colorCounter).ToString(), clr);
        }
        #endregion
    }
}
