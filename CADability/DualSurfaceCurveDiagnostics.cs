using CADability.Curve2D;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;

namespace CADability
{
    /// <summary>
    /// Opt-in measurement of how <see cref="InterpolatedDualSurfaceCurve"/> and its nested
    /// <see cref="InterpolatedDualSurfaceCurve.ProjectedCurve"/> behave on real data. It answers the questions the
    /// planned cleanup of these two classes depends on:
    /// <list type="bullet">
    /// <item>which way a point of the curve is actually computed, and how often that fails silently,</item>
    /// <item>whether the per-curve flag "isTangential" matches the geometry along the base points,</item>
    /// <item>whether the uv values the curve stores agree with what <see cref="ISurface.PositionOf"/> returns for
    /// the domain of the surface, i.e. whether the curve's own bounds are still needed,</item>
    /// <item>who creates, clones and mutates these curves.</item>
    /// </list>
    /// <para>
    /// It is switched on by setting the environment variable <see cref="EnvironmentVariable"/> to the path of the
    /// report to write. Like <see cref="DomainDiagnostics"/> it counts and logs, it never asserts and never throws,
    /// and while it is off every hook is a single test of a static readonly bool.
    /// </para>
    /// <para>
    /// While it is on, it evaluates the surfaces at the base points (PositionOf, GetNormal) and walks the stack for
    /// every construction. That may fill caches earlier than a run without the measurement would, so reference runs
    /// for the regression baselines are made with the measurement switched off.
    /// </para>
    /// </summary>
    public static class DualSurfaceCurveDiagnostics
    {
        /// <summary>
        /// Name of the environment variable. Set it to a file path to switch the measurement on, e.g.
        /// <c>CADABILITY_IDSC_DIAGNOSTICS=C:\temp\idsc.txt</c>.
        /// </summary>
        public const string EnvironmentVariable = "CADABILITY_IDSC_DIAGNOSTICS";

        /// <summary>True when the measurement is switched on.</summary>
        public static readonly bool Enabled;

        /// <summary>|n1 x n2| of the unit normals below this value counts as a point where the surfaces touch. The
        /// same threshold as the marching uses.</summary>
        private const double tangentialSine = 1e-3;
        private const int maxSamples = 40;
        private static readonly TimeSpan writeInterval = TimeSpan.FromSeconds(2);

        private static readonly string reportPath;
        private static readonly object sync = new object();
        private static readonly Stopwatch sinceLastWrite = Stopwatch.StartNew();
        private static long observations;

        // ---- constructions --------------------------------------------------------------------------------

        private enum Contact { Transversal, TouchesAtAnEnd, TouchesInside, Tangential, NoNormal }
        private static readonly string[] contactNames =
            { "transversal", "touches at an end only", "touches inside", "tangential throughout", "no normal available" };

        private sealed class CreatorStats
        {
            public long count;
            public double milliseconds;
            public long basePoints;
            public long flagTrue;
            public long flagContradicts;
        }
        private static readonly Dictionary<string, CreatorStats> creators = new Dictionary<string, CreatorStats>();
        // [Contact, isTangential ? 1 : 0]
        private static readonly long[,] contactByFlag = new long[5, 2];

        /// <summary>How a stored uv value relates to a reference value for the same 3d point.</summary>
        private enum UvAgreement { Equal, PeriodShift, OtherParameter, FirstOff, SecondOff }

        // per surface type, [0 = end point, 1 = inner point, UvAgreement]
        private static readonly Dictionary<string, long[,]> uvAtConstruction = new Dictionary<string, long[,]>();
        private static readonly Dictionary<string, long[,]> uvAtProjection = new Dictionary<string, long[,]>();
        // per surface type, the ends of the 2d spline against the stored end points, [UvAgreement]
        private static readonly Dictionary<string, long[]> projectedEnds = new Dictionary<string, long[]>();

        private enum BoundsAgreement { BoundsEmpty, DomainNotSet, Infinite, Equal, PeriodShift, Different }
        private static readonly string[] boundsNames = { "bounds empty", "domain not set", "infinite", "equal", "period shift", "different" };
        private static readonly Dictionary<string, long[]> boundsAtConstruction = new Dictionary<string, long[]>();
        private static readonly Dictionary<string, long[]> boundsAtProjection = new Dictionary<string, long[]>();

        // ---- point refinement -----------------------------------------------------------------------------

        private static readonly Dictionary<string, long> refinementBranches = new Dictionary<string, long>();
        // per branch: [on both surfaces and in the plane (eps), within 100 eps, further off, tangent disagrees with plane normal]
        private static readonly Dictionary<string, long[]> refinementQuality = new Dictionary<string, long[]>();
        // [sine bucket: < 1e-3, < 1e-2, >= 1e-2, no normal][isTangential ? 1 : 0], results of a solver only
        private static readonly long[,] refinementContactByFlag = new long[4, 2];

        // ---- everything else ------------------------------------------------------------------------------

        private static readonly Dictionary<string, long> operations = new Dictionary<string, long>();
        private static readonly Dictionary<string, long> domainWrites = new Dictionary<string, long>();

        private static readonly List<string> fallbackSamples = new List<string>();
        private static readonly List<string> offSurfaceSamples = new List<string>();
        private static readonly List<string> uvSamples = new List<string>();
        private static readonly List<string> endsSamples = new List<string>();
        private static readonly List<string> failureSamples = new List<string>();

        static DualSurfaceCurveDiagnostics()
        {
            try
            {
                reportPath = Environment.GetEnvironmentVariable(EnvironmentVariable);
                Enabled = !string.IsNullOrWhiteSpace(reportPath);
                if (Enabled) AppDomain.CurrentDomain.ProcessExit += (s, e) => WriteReport();
            }
            catch
            {
                Enabled = false;
            }
        }

        /// <summary>
        /// State captured at the beginning of a constructor, before the constructor may have written a domain onto
        /// one of the surfaces.
        /// </summary>
        internal sealed class ConstructionProbe
        {
            internal long startTimestamp;
            internal BoundingRect bounds1, bounds2, domain1, domain2;
            internal bool hasDomain1, hasDomain2;
        }

        /// <summary>
        /// Called first thing in a constructor of <see cref="InterpolatedDualSurfaceCurve"/>. Returns null while
        /// the measurement is off, which makes the matching <see cref="EndConstruction"/> a no-op.
        /// </summary>
        internal static ConstructionProbe BeginConstruction(ISurface surface1, BoundingRect bounds1, ISurface surface2, BoundingRect bounds2)
        {
            if (!Enabled) return null;
            try
            {
                ConstructionProbe probe = new ConstructionProbe
                {
                    startTimestamp = Stopwatch.GetTimestamp(),
                    bounds1 = bounds1,
                    bounds2 = bounds2
                };
                probe.hasDomain1 = TryGetDomain(surface1, out probe.domain1);
                probe.hasDomain2 = TryGetDomain(surface2, out probe.domain2);
                return probe;
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Called last thing in a constructor (or after deserialization, then with an explicit
        /// <paramref name="creator"/>). Records who made the curve and what it looks like.
        /// </summary>
        internal static void EndConstruction(ConstructionProbe probe, ISurface surface1, ISurface surface2,
            InterpolatedDualSurfaceCurve.SurfacePoint[] basePoints, bool isTangential, string creator = null)
        {
            if (probe == null || !Enabled) return;
            try
            {
                double milliseconds = (Stopwatch.GetTimestamp() - probe.startTimestamp) * 1000.0 / Stopwatch.Frequency;
                if (creator == null) creator = Creator();
                Contact contact = ClassifyContact(surface1, surface2, basePoints);
                bool contradicts = (isTangential && (contact == Contact.Transversal || contact == Contact.TouchesAtAnEnd))
                                || (!isTangential && contact == Contact.Tangential);
                double tol3d = Tolerance3d(basePoints);

                // evaluate outside the lock, the surfaces may take their time
                UvAgreement[] uv1 = ClassifyStoredUv(surface1, basePoints, true, tol3d);
                UvAgreement[] uv2 = ClassifyStoredUv(surface2, basePoints, false, tol3d);
                BoundsAgreement b1 = ClassifyBounds(surface1, probe.bounds1, probe.hasDomain1, probe.domain1);
                BoundsAgreement b2 = ClassifyBounds(surface2, probe.bounds2, probe.hasDomain2, probe.domain2);

                lock (sync)
                {
                    if (!creators.TryGetValue(creator, out CreatorStats stats))
                    {
                        stats = new CreatorStats();
                        creators[creator] = stats;
                    }
                    stats.count++;
                    stats.milliseconds += milliseconds;
                    stats.basePoints += basePoints.Length;
                    if (isTangential) stats.flagTrue++;
                    if (contradicts) stats.flagContradicts++;
                    contactByFlag[(int)contact, isTangential ? 1 : 0]++;
                    AddUv(uvAtConstruction, surface1, uv1, creator, "construction, surface1", basePoints, true);
                    AddUv(uvAtConstruction, surface2, uv2, creator, "construction, surface2", basePoints, false);
                    Increment(boundsAtConstruction, TypeName(surface1), (int)b1, boundsNames.Length);
                    Increment(boundsAtConstruction, TypeName(surface2), (int)b2, boundsNames.Length);
                    Observed();
                }
            }
            catch
            {   // a measurement must never change the outcome of the run it measures
            }
        }

        /// <summary>
        /// Called where the nested ProjectedCurve has just built its 2d approximation. <paramref name="bounds"/>
        /// is the bounds field of the 3d curve which was used to adjust the periodic values.
        /// </summary>
        internal static void ObserveProjectedCurve(ISurface surface, BoundingRect bounds, InterpolatedDualSurfaceCurve.SurfacePoint[] basePoints,
            bool onSurface1, BSpline2D spline)
        {
            if (!Enabled || spline == null) return;
            try
            {
                double tol3d = Tolerance3d(basePoints);
                bool hasDomain = TryGetDomain(surface, out BoundingRect domain);
                BoundsAgreement b = ClassifyBounds(surface, bounds, hasDomain, domain);
                UvAgreement[] uv = ClassifyStoredUv(surface, basePoints, onSurface1, tol3d);
                int last = basePoints.Length - 1;
                GeoPoint2D storedStart = onSurface1 ? basePoints[0].psurface1 : basePoints[0].psurface2;
                GeoPoint2D storedEnd = onSurface1 ? basePoints[last].psurface1 : basePoints[last].psurface2;
                GeoPoint2D splineStart = spline.PointAt(0.0);
                GeoPoint2D splineEnd = spline.PointAt(1.0);
                UvAgreement startAgreement = Compare(surface, splineStart, storedStart, basePoints[0].p3d, tol3d);
                UvAgreement endAgreement = Compare(surface, splineEnd, storedEnd, basePoints[last].p3d, tol3d);
                string type = TypeName(surface);
                lock (sync)
                {
                    Increment(boundsAtProjection, type, (int)b, boundsNames.Length);
                    AddUv(uvAtProjection, surface, uv, null, "2d curve, " + (onSurface1 ? "surface1" : "surface2"), basePoints, onSurface1);
                    Increment(projectedEnds, type, (int)startAgreement, 5);
                    Increment(projectedEnds, type, (int)endAgreement, 5);
                    if ((startAgreement != UvAgreement.Equal || endAgreement != UvAgreement.Equal) && endsSamples.Count < maxSamples)
                    {
                        endsSamples.Add(string.Format(CultureInfo.InvariantCulture, "{0,-26} start {1} spline={2} stored={3}   end {4} spline={5} stored={6}",
                            type, startAgreement, Format(splineStart), Format(storedStart), endAgreement, Format(splineEnd), Format(storedEnd)));
                    }
                    Observed();
                }
            }
            catch
            {
            }
        }

        /// <summary>A point was found in the cache of already computed positions.</summary>
        internal static void RecordCacheHit()
        {
            if (!Enabled) return;
            try
            {
                lock (sync)
                {
                    Increment(refinementBranches, "cache hit");
                    Observed();
                }
            }
            catch
            {
            }
        }

        /// <summary>
        /// A point of the curve has been computed by one of the branches of the point refinement.
        /// <paramref name="isFallback"/> marks the path that returns the point of the approximation without any
        /// refinement, but stores it as if it were exact.
        /// </summary>
        internal static void RecordRefinement(string branch, bool isFallback, bool planeFromSpline, bool isTangential,
            ISurface surface1, GeoPoint2D uv1, ISurface surface2, GeoPoint2D uv2, Plane plane, GeoPoint p)
        {
            if (!Enabled) return;
            try
            {
                string key = branch + (planeFromSpline ? ", plane from the spline" : ", plane from the base polygon");
                GeoPoint p1 = surface1.PointAt(uv1);
                GeoPoint p2 = surface2.PointAt(uv2);
                double gap = p1 | p2;
                double offPlane = Math.Abs(plane.Distance(p));
                int quality = (gap <= Precision.eps && offPlane <= Precision.eps) ? 0
                            : (gap <= 100 * Precision.eps && offPlane <= 100 * Precision.eps) ? 1 : 2;
                double sine = Sine(surface1, uv1, surface2, uv2, out GeoVector tangent);
                int sineBucket = double.IsNaN(sine) ? 3 : sine < tangentialSine ? 0 : sine < 1e-2 ? 1 : 2;
                // a transversal result whose tangent n1 x n2 is far from the normal of the plane most probably lies
                // on another branch of the intersection
                bool tangentDisagrees = !isFallback && sineBucket == 2
                    && Math.Abs(tangent.Normalized * plane.Normal.Normalized) < Math.Cos(20.0 / 180.0 * Math.PI);
                string stack = (isFallback || quality == 2) ? Callers(6) : null;
                lock (sync)
                {
                    Increment(refinementBranches, key);
                    if (!refinementQuality.TryGetValue(key, out long[] q))
                    {
                        q = new long[4];
                        refinementQuality[key] = q;
                    }
                    q[quality]++;
                    if (tangentDisagrees) q[3]++;
                    if (!isFallback) refinementContactByFlag[sineBucket, isTangential ? 1 : 0]++;
                    string sample = string.Format(CultureInfo.InvariantCulture, "{0}: gap={1:G3} offPlane={2:G3} sine={3:G3} isTangential={4}\n      <- {5}",
                        key, gap, offPlane, sine, isTangential, stack);
                    if (isFallback && fallbackSamples.Count < maxSamples) fallbackSamples.Add(sample);
                    else if (!isFallback && quality == 2 && offSurfaceSamples.Count < maxSamples) offSurfaceSamples.Add(sample);
                    Observed();
                }
            }
            catch
            {
            }
        }

        /// <summary>The curve has written a domain onto a surface which had none.</summary>
        internal static void RecordDomainWrite(string where, ISurface surface)
        {
            if (!Enabled) return;
            try
            {
                lock (sync)
                {
                    Increment(domainWrites, where + " on " + TypeName(surface));
                    Observed();
                }
            }
            catch
            {
            }
        }

        /// <summary>Counts an operation, e.g. a clone of the 3d curve made by a 2d curve.</summary>
        internal static void Count(string operation)
        {
            if (!Enabled) return;
            try
            {
                lock (sync)
                {
                    Increment(operations, operation);
                    Observed();
                }
            }
            catch
            {
            }
        }

        /// <summary>
        /// Split(double) assumes that base point i lies at the parameter i/(n-1), but the parameter of the curve comes
        /// from its approximating spline. <paramref name="index"/> is the base point Split derived from that assumption:
        /// the one at <paramref name="position"/> when <paramref name="atBasePoint"/>, otherwise the one after which the
        /// new point is inserted.
        /// </summary>
        internal static void RecordSplit(Func<GeoPoint, double> positionOf, InterpolatedDualSurfaceCurve.SurfacePoint[] basePoints,
            int index, double position, bool atBasePoint)
        {
            if (!Enabled) return;
            try
            {
                string result;
                if (index < 0 || index >= basePoints.Length || (!atBasePoint && index + 1 >= basePoints.Length))
                {
                    result = "IDSC.Split(double): position outside the curve";
                }
                else if (atBasePoint)
                {
                    result = Math.Abs(positionOf(basePoints[index].p3d) - position) < 1e-6
                        ? "IDSC.Split(double): at a base point, which really is at that parameter"
                        : "IDSC.Split(double): at a base point, which is NOT at that parameter";
                }
                else
                {
                    double before = positionOf(basePoints[index].p3d), after = positionOf(basePoints[index + 1].p3d);
                    result = before <= position + 1e-9 && position <= after + 1e-9
                        ? "IDSC.Split(double): new point lies between the base points it is inserted between"
                        : "IDSC.Split(double): new point lies OUTSIDE the base points it is inserted between";
                }
                lock (sync)
                {
                    Increment(operations, result);
                    Observed();
                }
            }
            catch
            {
            }
        }

        /// <summary>A constructor threw and the caller swallowed the exception.</summary>
        internal static void RecordConstructionFailure(string where, Exception ex)
        {
            if (!Enabled) return;
            try
            {
                lock (sync)
                {
                    Increment(operations, "constructor threw, caught in " + where);
                    if (failureSamples.Count < maxSamples)
                        failureSamples.Add(where + ": " + ex.GetType().Name + ": " + ex.Message + "\n      at " + FirstFrames(ex, 4));
                    Observed();
                }
            }
            catch
            {
            }
        }

        // ---- classification -------------------------------------------------------------------------------

        private static Contact ClassifyContact(ISurface surface1, ISurface surface2, InterpolatedDualSurfaceCurve.SurfacePoint[] basePoints)
        {
            bool anyKnown = false, allTouch = true, touchAtEnd = false, touchInside = false;
            for (int i = 0; i < basePoints.Length; i++)
            {
                double sine = Sine(surface1, basePoints[i].psurface1, surface2, basePoints[i].psurface2, out _);
                if (double.IsNaN(sine)) continue;
                anyKnown = true;
                if (sine >= tangentialSine) allTouch = false;
                else if (i == 0 || i == basePoints.Length - 1) touchAtEnd = true;
                else touchInside = true;
            }
            if (!anyKnown) return Contact.NoNormal;
            if (allTouch) return Contact.Tangential;
            if (touchInside) return Contact.TouchesInside;
            if (touchAtEnd) return Contact.TouchesAtAnEnd;
            return Contact.Transversal;
        }

        /// <summary>|n1 x n2| of the unit normals, NaN where a normal is not available (e.g. at a pole).</summary>
        private static double Sine(ISurface surface1, GeoPoint2D uv1, ISurface surface2, GeoPoint2D uv2, out GeoVector cross)
        {
            cross = GeoVector.NullVector;
            GeoVector n1 = surface1.GetNormal(uv1);
            GeoVector n2 = surface2.GetNormal(uv2);
            if (!(n1.Length > 0.0) || !(n2.Length > 0.0)) return double.NaN;
            cross = n1.Normalized ^ n2.Normalized;
            return cross.Length;
        }

        private static UvAgreement[] ClassifyStoredUv(ISurface surface, InterpolatedDualSurfaceCurve.SurfacePoint[] basePoints, bool onSurface1, double tol3d)
        {
            UvAgreement[] res = new UvAgreement[basePoints.Length];
            for (int i = 0; i < basePoints.Length; i++)
            {
                GeoPoint2D stored = onSurface1 ? basePoints[i].psurface1 : basePoints[i].psurface2;
                GeoPoint2D reference = surface.PositionOf(basePoints[i].p3d);
                res[i] = Compare(surface, stored, reference, basePoints[i].p3d, tol3d);
            }
            return res;
        }

        /// <summary>
        /// Compares two parameter values <paramref name="first"/> and <paramref name="second"/> which should both
        /// describe <paramref name="p3d"/>.
        /// </summary>
        private static UvAgreement Compare(ISurface surface, GeoPoint2D first, GeoPoint2D second, GeoPoint p3d, double tol3d)
        {
            if ((surface.PointAt(first) | p3d) > tol3d) return UvAgreement.FirstOff;
            if ((surface.PointAt(second) | p3d) > tol3d) return UvAgreement.SecondOff;
            double du = first.x - second.x;
            double dv = first.y - second.y;
            bool shifted = false;
            if (surface.IsUPeriodic && surface.UPeriod > 0.0)
            {
                double k = Math.Round(du / surface.UPeriod);
                if (k != 0.0)
                {
                    du -= k * surface.UPeriod;
                    shifted = true;
                }
            }
            if (surface.IsVPeriodic && surface.VPeriod > 0.0)
            {
                double k = Math.Round(dv / surface.VPeriod);
                if (k != 0.0)
                {
                    dv -= k * surface.VPeriod;
                    shifted = true;
                }
            }
            double tolU = 1e-6 * Math.Max(1.0, surface.IsUPeriodic ? surface.UPeriod : Math.Max(Math.Abs(first.x), Math.Abs(second.x)));
            double tolV = 1e-6 * Math.Max(1.0, surface.IsVPeriodic ? surface.VPeriod : Math.Max(Math.Abs(first.y), Math.Abs(second.y)));
            if (Math.Abs(du) <= tolU && Math.Abs(dv) <= tolV) return shifted ? UvAgreement.PeriodShift : UvAgreement.Equal;
            return UvAgreement.OtherParameter; // same 3d point, different parameters: a pole or another singularity
        }

        private static BoundsAgreement ClassifyBounds(ISurface surface, BoundingRect bounds, bool hasDomain, BoundingRect domain)
        {
            if (bounds.IsEmpty()) return BoundsAgreement.BoundsEmpty;
            if (!hasDomain) return BoundsAgreement.DomainNotSet;
            switch (DomainDiagnostics.Classify(surface, bounds, domain, out _, out _, out _))
            {
                case DomainAgreement.NotSet: return BoundsAgreement.DomainNotSet;
                case DomainAgreement.Infinite: return BoundsAgreement.Infinite;
                case DomainAgreement.Equal: return BoundsAgreement.Equal;
                case DomainAgreement.PeriodShift: return BoundsAgreement.PeriodShift;
                default: return BoundsAgreement.Different;
            }
        }

        private static bool TryGetDomain(ISurface surface, out BoundingRect domain)
        {
            domain = BoundingRect.EmptyBoundingRect;
            if (surface == null) return false;
            try
            {
                domain = surface.Domain;
                return !domain.IsEmpty();
            }
            catch
            {   // ScaledSurface throws on Domain
                return false;
            }
        }

        private static double Tolerance3d(InterpolatedDualSurfaceCurve.SurfacePoint[] basePoints)
        {
            BoundingBox ext = BoundingBox.EmptyBoundingBox;
            for (int i = 0; i < basePoints.Length; i++) ext.MinMax(basePoints[i].p3d);
            return Math.Max(100 * Precision.eps, 1e-6 * ext.Size);
        }

        // ---- bookkeeping ----------------------------------------------------------------------------------

        private static void AddUv(Dictionary<string, long[,]> into, ISurface surface, UvAgreement[] agreement, string creator, string origin,
            InterpolatedDualSurfaceCurve.SurfacePoint[] basePoints, bool onSurface1)
        {
            string type = TypeName(surface);
            if (!into.TryGetValue(type, out long[,] counts))
            {
                counts = new long[2, 5];
                into[type] = counts;
            }
            int last = agreement.Length - 1;
            for (int i = 0; i <= last; i++)
            {
                UvAgreement a = agreement[i];
                counts[(i == 0 || i == last) ? 0 : 1, (int)a]++;
                if (a != UvAgreement.Equal && uvSamples.Count < maxSamples)
                {
                    GeoPoint2D stored = onSurface1 ? basePoints[i].psurface1 : basePoints[i].psurface2;
                    uvSamples.Add(string.Format(CultureInfo.InvariantCulture, "{0,-26} {1,-24} point {2} of {3}: {4}, stored {5}{6}",
                        type, origin, i, last + 1, a, Format(stored), creator == null ? "" : "   <- " + creator));
                }
            }
        }

        private static void Increment(Dictionary<string, long[]> into, string key, int index, int length)
        {
            if (!into.TryGetValue(key, out long[] counts))
            {
                counts = new long[length];
                into[key] = counts;
            }
            counts[index]++;
        }

        private static void Increment(Dictionary<string, long> into, string key)
        {
            into.TryGetValue(key, out long n);
            into[key] = n + 1;
        }

        /// <summary>Must be called under the lock. Writes the report every <see cref="writeInterval"/>, because the
        /// test host does not reliably run ProcessExit handlers.</summary>
        private static void Observed()
        {
            observations++;
            if (observations <= 4 || sinceLastWrite.Elapsed > writeInterval)
            {
                WriteReportNoLock();
                sinceLastWrite.Restart();
            }
        }

        private static string TypeName(ISurface surface) => surface == null ? "(null)" : surface.GetType().Name;

        private static string Format(GeoPoint2D p) => string.Format(CultureInfo.InvariantCulture, "({0:G8}, {1:G8})", p.x, p.y);

        /// <summary>
        /// The two methods which asked for the new curve: the first frame outside the constructors of
        /// <see cref="InterpolatedDualSurfaceCurve"/> and the one above it.
        /// </summary>
        private static string Creator()
        {
            StackTrace stack = new StackTrace(1, false);
            List<string> names = new List<string>();
            for (int i = 0; i < stack.FrameCount && names.Count < 2; i++)
            {
                MethodBase method = stack.GetFrame(i)?.GetMethod();
                if (method == null) continue;
                Type type = method.DeclaringType;
                if (type == typeof(DualSurfaceCurveDiagnostics)) continue;
                if (method is ConstructorInfo && type == typeof(InterpolatedDualSurfaceCurve)) continue;
                names.Add(Name(type) + "." + method.Name);
            }
            return names.Count > 0 ? string.Join(" <- ", names) : "(unknown)";
        }

        private static string Callers(int depth)
        {
            StackTrace stack = new StackTrace(2, false);
            List<string> names = new List<string>();
            for (int i = 0; i < stack.FrameCount && names.Count < depth; i++)
            {
                MethodBase method = stack.GetFrame(i)?.GetMethod();
                if (method == null) continue;
                names.Add(Name(method.DeclaringType) + "." + method.Name);
            }
            return string.Join(" <- ", names);
        }

        private static string FirstFrames(Exception ex, int depth)
        {
            StackTrace stack = new StackTrace(ex, false);
            List<string> names = new List<string>();
            for (int i = 0; i < stack.FrameCount && names.Count < depth; i++)
            {
                MethodBase method = stack.GetFrame(i)?.GetMethod();
                if (method != null) names.Add(Name(method.DeclaringType) + "." + method.Name);
            }
            return string.Join(" <- ", names);
        }

        /// <summary>The type name, with the enclosing type for nested types (so that the two ProjectedCurve classes
        /// can be told apart) and without the compiler generated closure classes.</summary>
        private static string Name(Type type)
        {
            if (type == null) return "?";
            while (type.Name.StartsWith("<", StringComparison.Ordinal) && type.DeclaringType != null) type = type.DeclaringType;
            return type.DeclaringType != null ? type.DeclaringType.Name + "+" + type.Name : type.Name;
        }

        // ---- report ---------------------------------------------------------------------------------------

        /// <summary>
        /// Writes the report. Called automatically while the measurement runs and when the process exits; public
        /// so a single test can ask for it explicitly.
        /// </summary>
        public static void WriteReport()
        {
            if (!Enabled) return;
            lock (sync) WriteReportNoLock();
        }

        private static void WriteReportNoLock()
        {
            try
            {
                StringBuilder sb = new StringBuilder();
                sb.AppendLine("InterpolatedDualSurfaceCurve diagnostics");
                sb.AppendLine("========================================");
                sb.AppendLine("observations: " + observations.ToString(CultureInfo.InvariantCulture));
                sb.AppendLine();

                AppendCreators(sb);
                AppendContact(sb);
                AppendUv(sb, "3. Stored uv of the base points against PositionOf, at construction", uvAtConstruction);
                AppendBounds(sb, "4. Bounds handed to the constructor against the surface's Domain at that moment", boundsAtConstruction);
                AppendBounds(sb, "5a. Bounds field of the curve against the surface's Domain, when the 2d curve is built", boundsAtProjection);
                AppendUv(sb, "5b. Stored uv of the base points against PositionOf, when the 2d curve is built", uvAtProjection);
                AppendEnds(sb);
                AppendRefinement(sb);
                AppendCounters(sb, "7. Domains written onto a surface by the curve", domainWrites);
                AppendCounters(sb, "8. Operations", operations);
                AppendSamples(sb, "Samples: fallback, the unrefined point was stored as exact", fallbackSamples);
                AppendSamples(sb, "Samples: a solver result accepted although it is off the surfaces or off the plane", offSurfaceSamples);
                AppendSamples(sb, "Samples: stored uv not equal to PositionOf", uvSamples);
                AppendSamples(sb, "Samples: ends of the 2d spline not equal to the stored end points", endsSamples);
                AppendSamples(sb, "Samples: exceptions from the constructor", failureSamples);

                File.WriteAllText(reportPath, sb.ToString());
            }
            catch
            {   // reporting must not break the run either
            }
        }

        private static void AppendCreators(StringBuilder sb)
        {
            Caption(sb, "1. Constructions by creator (frame that called the constructor <- its caller)");
            if (creators.Count == 0)
            {
                sb.AppendLine("  (nothing observed)");
                sb.AppendLine();
                return;
            }
            sb.AppendLine("  flag contradicts = isTangential is true but the surfaces intersect transversally at all inner points,");
            sb.AppendLine("  or isTangential is false and the surfaces touch at every base point.");
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,10} {1,12} {2,9} {3,7} {4,10} {5,11}  {6}",
                "count", "ms total", "ms avg", "points", "tang=true", "contradicts", "creator"));
            long total = 0;
            double totalMs = 0.0;
            foreach (KeyValuePair<string, CreatorStats> e in creators.OrderByDescending(e => e.Value.milliseconds))
            {
                CreatorStats s = e.Value;
                total += s.count;
                totalMs += s.milliseconds;
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,10} {1,12:F0} {2,9:F2} {3,7:F1} {4,10} {5,11}  {6}",
                    s.count, s.milliseconds, s.milliseconds / s.count, (double)s.basePoints / s.count, s.flagTrue, s.flagContradicts, e.Key));
            }
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,10} {1,12:F0}  TOTAL", total, totalMs));
            sb.AppendLine();
        }

        private static void AppendContact(StringBuilder sb)
        {
            Caption(sb, "2. Contact of the surfaces along the base points against the flag isTangential (|n1 x n2| < 1e-3 is a touching point)");
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-26} {1,14} {2,14}", "", "flag false", "flag true"));
            for (int i = 0; i < contactNames.Length; i++)
            {
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-26} {1,14} {2,14}", contactNames[i], contactByFlag[i, 0], contactByFlag[i, 1]));
            }
            sb.AppendLine();
        }

        private static readonly string[] uvNames = { "equal", "period shift", "other param", "stored off", "PositionOf off" };

        private static void AppendUv(StringBuilder sb, string caption, Dictionary<string, long[,]> counts)
        {
            Caption(sb, caption);
            if (counts.Count == 0)
            {
                sb.AppendLine("  (nothing observed)");
                sb.AppendLine();
                return;
            }
            sb.AppendLine("  other param = both describe the same 3d point with parameters that do not differ by periods (pole).");
            sb.AppendLine("  stored off  = the stored uv does not reproduce the 3d point of the base point.");
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-30} {1,-6} {2,12} {3,12} {4,12} {5,12} {6,14}",
                "surface type", "point", uvNames[0], uvNames[1], uvNames[2], uvNames[3], uvNames[4]));
            foreach (KeyValuePair<string, long[,]> e in counts.OrderBy(e => e.Key, StringComparer.Ordinal))
            {
                for (int row = 0; row < 2; row++)
                {
                    long[,] c = e.Value;
                    sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-30} {1,-6} {2,12} {3,12} {4,12} {5,12} {6,14}",
                        row == 0 ? e.Key : "", row == 0 ? "ends" : "inner", c[row, 0], c[row, 1], c[row, 2], c[row, 3], c[row, 4]));
                }
            }
            sb.AppendLine();
        }

        private static void AppendBounds(StringBuilder sb, string caption, Dictionary<string, long[]> counts)
        {
            Caption(sb, caption);
            if (counts.Count == 0)
            {
                sb.AppendLine("  (nothing observed)");
                sb.AppendLine();
                return;
            }
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-30} {1,14} {2,14} {3,10} {4,10} {5,13} {6,10}",
                "surface type", boundsNames[0], boundsNames[1], boundsNames[2], boundsNames[3], boundsNames[4], boundsNames[5]));
            foreach (KeyValuePair<string, long[]> e in counts.OrderBy(e => e.Key, StringComparer.Ordinal))
            {
                long[] c = e.Value;
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-30} {1,14} {2,14} {3,10} {4,10} {5,13} {6,10}",
                    e.Key, c[0], c[1], c[2], c[3], c[4], c[5]));
            }
            sb.AppendLine();
        }

        private static void AppendEnds(StringBuilder sb)
        {
            Caption(sb, "5c. Ends of the 2d spline (PointAt(0), PointAt(1)) against the stored end points (StartPoint, EndPoint)");
            if (projectedEnds.Count == 0)
            {
                sb.AppendLine("  (nothing observed)");
                sb.AppendLine();
                return;
            }
            sb.AppendLine("  stored off = the stored end point does not reproduce the 3d end point; spline off: the spline end does not.");
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-30} {1,12} {2,12} {3,12} {4,12} {5,12}",
                "surface type", "equal", "period shift", "other param", "spline off", "stored off"));
            foreach (KeyValuePair<string, long[]> e in projectedEnds.OrderBy(e => e.Key, StringComparer.Ordinal))
            {
                long[] c = e.Value;
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-30} {1,12} {2,12} {3,12} {4,12} {5,12}",
                    e.Key, c[0], c[1], c[2], c[3], c[4]));
            }
            sb.AppendLine();
        }

        private static void AppendRefinement(StringBuilder sb)
        {
            Caption(sb, "6. Point refinement (ApproximatePosition)");
            if (refinementBranches.Count == 0)
            {
                sb.AppendLine("  (nothing observed)");
                sb.AppendLine();
                return;
            }
            sb.AppendLine("  exact = |p1 - p2| and the distance to the plane both <= Precision.eps; near = within 100 eps; off = further.");
            sb.AppendLine("  wrong tangent = transversal result whose n1 x n2 is more than 20 degrees off the plane normal (other branch?).");
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,12} {1,10} {2,10} {3,10} {4,13}  {5}",
                "calls", "exact", "near", "off", "wrong tangent", "branch"));
            foreach (KeyValuePair<string, long> e in refinementBranches.OrderByDescending(e => e.Value))
            {
                refinementQuality.TryGetValue(e.Key, out long[] q);
                if (q == null) sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,12} {1,10} {2,10} {3,10} {4,13}  {5}", e.Value, "", "", "", "", e.Key));
                else sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,12} {1,10} {2,10} {3,10} {4,13}  {5}", e.Value, q[0], q[1], q[2], q[3], e.Key));
            }
            sb.AppendLine();
            sb.AppendLine("  |n1 x n2| at the points found by a solver, against the flag isTangential of the curve:");
            string[] buckets = { "< 1e-3 (touching)", "< 1e-2", ">= 1e-2 (transversal)", "no normal" };
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-24} {1,14} {2,14}", "", "flag false", "flag true"));
            for (int i = 0; i < buckets.Length; i++)
            {
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-24} {1,14} {2,14}", buckets[i], refinementContactByFlag[i, 0], refinementContactByFlag[i, 1]));
            }
            sb.AppendLine();
        }

        private static void AppendCounters(StringBuilder sb, string caption, Dictionary<string, long> counts)
        {
            Caption(sb, caption);
            if (counts.Count == 0) sb.AppendLine("  (nothing observed)");
            foreach (KeyValuePair<string, long> e in counts.OrderByDescending(e => e.Value))
            {
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,12}  {1}", e.Value, e.Key));
            }
            sb.AppendLine();
        }

        private static void AppendSamples(StringBuilder sb, string caption, List<string> samples)
        {
            Caption(sb, caption + " (at most " + maxSamples + ")");
            if (samples.Count == 0) sb.AppendLine("  (none)");
            else foreach (string s in samples) sb.AppendLine("  " + s);
            sb.AppendLine();
        }

        private static void Caption(StringBuilder sb, string caption)
        {
            sb.AppendLine(caption);
            sb.AppendLine(new string('-', Math.Min(caption.Length, 110)));
        }
    }
}
