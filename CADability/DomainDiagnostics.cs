using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

namespace CADability.GeoObject
{
    /// <summary>
    /// How the domain stored on the surface relates to the domain the face derives from its 2d edges.
    /// </summary>
    public enum DomainAgreement
    {
        /// <summary>The surface carries no domain yet (empty rectangle).</summary>
        NotSet,
        /// <summary>One of the two rectangles is unbounded, so they cannot be compared numerically.</summary>
        Infinite,
        /// <summary>Both describe the same rectangle.</summary>
        Equal,
        /// <summary>Same rectangle, but moved by a whole number of periods. Harmless for the geometry.</summary>
        PeriodShift,
        /// <summary>They describe different rectangles. This is the interesting case.</summary>
        Different
    }

    /// <summary>
    /// Opt-in measurement of the invariant "<see cref="Face.Domain"/> equals <see cref="ISurface.Domain"/>",
    /// which the ongoing unification of the two concepts relies on.
    /// <para>
    /// It is switched on by setting the environment variable <see cref="EnvironmentVariable"/> to the path of
    /// the report to write; without it every hook is a single test of a static readonly bool, which the JIT
    /// removes. The report is written when the process exits and, as a safety net, every
    /// <see cref="flushInterval"/> observations.
    /// </para>
    /// <para>
    /// It counts and logs, it never asserts and never throws: MSTest translates Debug.Fail into a
    /// DebugAssertException and would turn every measured deviation into a failing test, which would make the
    /// suite useless as an instrument. Any exception inside the diagnostic itself is swallowed for the same
    /// reason - a measurement must not change what it measures.
    /// </para>
    /// </summary>
    public static class DomainDiagnostics
    {
        /// <summary>
        /// Name of the environment variable. Set it to a file path to switch the measurement on, e.g.
        /// <c>CADABILITY_DOMAIN_DIAGNOSTICS=C:\temp\domain.txt</c>.
        /// </summary>
        public const string EnvironmentVariable = "CADABILITY_DOMAIN_DIAGNOSTICS";

        /// <summary>
        /// True when the measurement is switched on. Kept as a static readonly field so the hooks cost
        /// nothing when it is off.
        /// </summary>
        public static readonly bool Enabled;

        private static readonly string reportPath;
        private const int flushInterval = 250000;
        private const int maxSamplesPerKind = 150;

        private static readonly object sync = new object();
        // one long[] per surface type, indexed by (int)DomainAgreement
        private static readonly Dictionary<string, long[]> whenAreaWasCached = new Dictionary<string, long[]>();
        private static readonly Dictionary<string, long[]> whenAreaWasRecalculated = new Dictionary<string, long[]>();
        private static readonly Dictionary<string, long[]> whenAreaWritesBack = new Dictionary<string, long[]>();
        private static readonly List<string> differentSamples = new List<string>();
        private static readonly List<string> degenerateStacks = new List<string>();
        // "file:line" of an AdjustPeriodic call site -> [calls, moved something, bounds was the surface domain]
        private static readonly Dictionary<string, long[]> adjustCallSites = new Dictionary<string, long[]>();
        private static long adjustCalls;
        private static readonly List<string> periodShiftSamples = new List<string>();
        private static long observations;

        static DomainDiagnostics()
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
        /// Called from the <see cref="Face.Domain"/> getter. <paramref name="areaWasCached"/> tells whether the
        /// face had to rebuild its area first - if it did, it has just written the same value onto the surface
        /// and agreement is guaranteed by construction, which is why the two cases are counted separately.
        /// </summary>
        internal static void ObserveFaceDomain(ISurface surface, BoundingRect faceDomain, int faceHashCode, bool areaWasCached)
        {
            Observe(areaWasCached ? whenAreaWasCached : whenAreaWasRecalculated,
                    areaWasCached ? "Face.Domain, area cached" : "Face.Domain, area rebuilt",
                    surface, faceDomain, faceHashCode);
        }

        /// <summary>
        /// Called where <see cref="Face.Area"/> writes its extent onto the surface. What it measures is how far
        /// the surface domain had drifted before it got silently corrected.
        /// </summary>
        internal static void ObserveAreaWriteBack(ISurface surface, BoundingRect fromArea, int faceHashCode)
        {
            Observe(whenAreaWritesBack, "Area write-back", surface, fromArea, faceHashCode);
        }

        private static void Observe(Dictionary<string, long[]> into, string origin, ISurface surface, BoundingRect faceDomain, int faceHashCode)
        {
            if (!Enabled || surface == null) return;
            try
            {
                BoundingRect surfaceDomain = surface.Domain;
                DomainAgreement agreement = Classify(surface, faceDomain, surfaceDomain,
                                                     out double uShift, out double vShift, out double residual);
                string type = surface.GetType().Name;
                lock (sync)
                {
                    if (!into.TryGetValue(type, out long[] counts))
                    {
                        counts = new long[5];
                        into[type] = counts;
                    }
                    counts[(int)agreement]++;
                    observations++;

                    // A domain of exactly [0,0 .. 0,0] is a default constructed BoundingRect, which is not
                    // the same as EmptyBoundingRect and therefore slips through every IsEmpty() guard. Capture
                    // where it comes from, a few times, so the producer can be identified.
                    if (agreement == DomainAgreement.Different && degenerateStacks.Count < 3
                        && surfaceDomain.Left == 0.0 && surfaceDomain.Right == 0.0
                        && surfaceDomain.Bottom == 0.0 && surfaceDomain.Top == 0.0)
                    {
                        degenerateStacks.Add(type + " face=" + faceHashCode + Environment.NewLine + Environment.StackTrace);
                    }
                    if (agreement == DomainAgreement.Different && differentSamples.Count < maxSamplesPerKind)
                    {
                        differentSamples.Add(Sample(origin, type, faceHashCode, faceDomain, surfaceDomain,
                                                    "residual=" + residual.ToString("G4", CultureInfo.InvariantCulture)));
                    }
                    else if (agreement == DomainAgreement.PeriodShift && periodShiftSamples.Count < maxSamplesPerKind)
                    {
                        periodShiftSamples.Add(Sample(origin, type, faceHashCode, faceDomain, surfaceDomain,
                                                      "shift=(" + uShift.ToString(CultureInfo.InvariantCulture) + ","
                                                      + vShift.ToString(CultureInfo.InvariantCulture) + ")"));
                    }

                    // The test host does not reliably run ProcessExit handlers, so the report is
                    // written eagerly at the start and at a fixed interval afterwards.
                    if (observations <= 8 || observations % flushInterval == 0) WriteReportNoLock();
                }
            }
            catch
            {   // a measurement must never change the outcome of the run it measures
            }
        }

        /// <summary>
        /// Called from every SurfaceHelper.AdjustPeriodic overload. What matters is not how often a call site
        /// runs but how often it actually moves something: a site that never moved anything over a whole
        /// suite run is a no-op on every tested path and can go.
        /// </summary>
        internal static void RecordAdjust(ISurface surface, BoundingRect bounds, string callerFile, int callerLine, bool moved)
        {
            if (!Enabled) return;
            try
            {
                // Whether the caller aimed at the surface's own domain is the question that cannot be answered
                // at the call site, because the rectangle is usually a local variable there.
                bool boundsIsDomain;
                try { boundsIsDomain = bounds == surface.Domain; }
                catch { boundsIsDomain = false; } // ScaledSurface throws on Domain
                string key = (callerFile == null ? "?" : System.IO.Path.GetFileName(callerFile)) + ":" + callerLine;
                lock (sync)
                {
                    if (!adjustCallSites.TryGetValue(key, out long[] counts))
                    {
                        counts = new long[3];
                        adjustCallSites[key] = counts;
                    }
                    counts[0]++;
                    if (moved) counts[1]++;
                    if (boundsIsDomain) counts[2]++;
                    adjustCalls++;
                    if (adjustCalls % flushInterval == 0) WriteReportNoLock();
                }
            }
            catch
            {
            }
        }

        /// <summary>
        /// Decides how the two rectangles relate. <paramref name="uShift"/> and <paramref name="vShift"/> are
        /// the number of whole periods between them, <paramref name="residual"/> is what is left over after
        /// removing those periods, relative to the size of the face domain.
        /// </summary>
        internal static DomainAgreement Classify(ISurface surface, BoundingRect faceDomain, BoundingRect surfaceDomain,
                                                out double uShift, out double vShift, out double residual)
        {
            uShift = vShift = residual = 0.0;
            if (surfaceDomain.IsEmpty()) return DomainAgreement.NotSet;
            if (surfaceDomain.IsInfinite || faceDomain.IsInfinite || faceDomain.IsEmpty()) return DomainAgreement.Infinite;

            double sizeU = Math.Max(Math.Abs(faceDomain.Width), 1e-10);
            double sizeV = Math.Max(Math.Abs(faceDomain.Height), 1e-10);

            bool uIsTranslation = Axis(surfaceDomain.Left - faceDomain.Left, surfaceDomain.Right - faceDomain.Right,
                                       surface.IsUPeriodic, surface.UPeriod, sizeU, out uShift, out double residualU);
            bool vIsTranslation = Axis(surfaceDomain.Bottom - faceDomain.Bottom, surfaceDomain.Top - faceDomain.Top,
                                       surface.IsVPeriodic, surface.VPeriod, sizeV, out vShift, out double residualV);

            residual = Math.Max(residualU / sizeU, residualV / sizeV);
            if (!uIsTranslation || !vIsTranslation) return DomainAgreement.Different;
            if (uShift != 0.0 || vShift != 0.0) return DomainAgreement.PeriodShift;
            return DomainAgreement.Equal;
        }

        /// <summary>
        /// Looks at one axis. Returns true when both edges moved by the same amount and that amount is either
        /// zero or a whole number of periods.
        /// </summary>
        private static bool Axis(double dLow, double dHigh, bool isPeriodic, double period, double size,
                                 out double shiftInPeriods, out double residual)
        {
            shiftInPeriods = 0.0;
            double scale = isPeriodic && period > 0.0 ? period : Math.Max(size, 1.0);
            double tolerance = scale * 1e-6;

            if (Math.Abs(dLow - dHigh) > tolerance)
            {   // the two edges did not move together, so this is not a translation at all
                residual = Math.Max(Math.Abs(dLow), Math.Abs(dHigh));
                return false;
            }
            if (isPeriodic && period > 0.0)
            {
                double k = Math.Round(dLow / period);
                residual = Math.Abs(dLow - k * period);
                if (residual <= tolerance)
                {
                    shiftInPeriods = k;
                    return true;
                }
                return false;
            }
            residual = Math.Abs(dLow);
            return residual <= tolerance;
        }

        private static string Sample(string origin, string type, int faceHashCode, BoundingRect faceDomain, BoundingRect surfaceDomain, string extra)
        {
            return string.Format(CultureInfo.InvariantCulture,
                                 "{0,-26} {1,-28} face={2,-12} faceDomain={3}  surfaceDomain={4}  {5}",
                                 origin, type, faceHashCode, Format(faceDomain), Format(surfaceDomain), extra);
        }

        private static string Format(BoundingRect r)
        {
            if (r.IsEmpty()) return "[empty]";
            return string.Format(CultureInfo.InvariantCulture, "[{0:G6},{1:G6} .. {2:G6},{3:G6}]",
                                 r.Left, r.Bottom, r.Right, r.Top);
        }

        /// <summary>
        /// Writes the report. Called automatically when the process exits; public so a single test can ask for
        /// it explicitly.
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
                sb.AppendLine("Face.Domain versus ISurface.Domain");
                sb.AppendLine("==================================");
                sb.AppendLine("observations: " + observations.ToString(CultureInfo.InvariantCulture));
                sb.AppendLine();
                sb.AppendLine("NotSet      the surface carries no domain yet");
                sb.AppendLine("Infinite    one of the two is unbounded");
                sb.AppendLine("Equal       identical rectangles");
                sb.AppendLine("PeriodShift identical, moved by whole periods - harmless");
                sb.AppendLine("Different   genuinely different rectangles - the interesting case");
                sb.AppendLine();

                AppendTable(sb, "Face.Domain, area was cached (the meaningful case)", whenAreaWasCached);
                AppendTable(sb, "Face.Domain, area was rebuilt first (agreement is by construction)", whenAreaWasRecalculated);
                AppendTable(sb, "Area writing its extent back onto the surface (how far it had drifted)", whenAreaWritesBack);

                AppendSamples(sb, "Samples: Different", differentSamples);
                AppendSamples(sb, "Samples: PeriodShift", periodShiftSamples);
                AppendSamples(sb, "Stacks: surface domain is a default constructed BoundingRect", degenerateStacks);
                AppendAdjustTable(sb);

                File.WriteAllText(reportPath, sb.ToString());
            }
            catch
            {   // reporting must not break the run either
            }
        }

        private static void AppendTable(StringBuilder sb, string caption, Dictionary<string, long[]> counts)
        {
            sb.AppendLine(caption);
            sb.AppendLine(new string('-', caption.Length));
            if (counts.Count == 0)
            {
                sb.AppendLine("  (nothing observed)");
                sb.AppendLine();
                return;
            }
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-30} {1,12} {2,12} {3,12} {4,12} {5,12}",
                                        "surface type", "NotSet", "Infinite", "Equal", "PeriodShift", "Different"));
            long[] total = new long[5];
            foreach (KeyValuePair<string, long[]> entry in counts.OrderByDescending(e => e.Value[(int)DomainAgreement.Different])
                                                                .ThenBy(e => e.Key, StringComparer.Ordinal))
            {
                for (int i = 0; i < 5; i++) total[i] += entry.Value[i];
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-30} {1,12} {2,12} {3,12} {4,12} {5,12}",
                                            entry.Key, entry.Value[0], entry.Value[1], entry.Value[2], entry.Value[3], entry.Value[4]));
            }
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-30} {1,12} {2,12} {3,12} {4,12} {5,12}",
                                        "TOTAL", total[0], total[1], total[2], total[3], total[4]));
            sb.AppendLine();
        }

        private static void AppendAdjustTable(StringBuilder sb)
        {
            sb.AppendLine("SurfaceHelper.AdjustPeriodic per call site, never-moved first");
            sb.AppendLine("------------------------------------------------------------");
            sb.AppendLine("  onOwnDomain = calls where the rectangle handed in was exactly the surface's Domain.");
            sb.AppendLine("  A site with moved = 0 and onOwnDomain = calls, whose value comes straight out of");
            sb.AppendLine("  PositionOf of that same surface, is a no-op by the contract on ISurface.Domain.");
            if (adjustCallSites.Count == 0)
            {
                sb.AppendLine("  (nothing observed)");
                sb.AppendLine();
                return;
            }
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-40} {1,12} {2,12} {3,14}",
                                        "call site", "calls", "moved", "onOwnDomain"));
            foreach (KeyValuePair<string, long[]> entry in adjustCallSites.OrderBy(e => e.Value[1])
                                                                         .ThenByDescending(e => e.Value[0]))
            {
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-40} {1,12} {2,12} {3,14}",
                                            entry.Key, entry.Value[0], entry.Value[1], entry.Value[2]));
            }
            sb.AppendLine();
        }

        private static void AppendSamples(StringBuilder sb, string caption, List<string> samples)
        {
            sb.AppendLine(caption + " (at most " + maxSamplesPerKind + ")");
            sb.AppendLine(new string('-', caption.Length));
            if (samples.Count == 0) sb.AppendLine("  (none)");
            else foreach (string s in samples) sb.AppendLine("  " + s);
            sb.AppendLine();
        }
    }
}
