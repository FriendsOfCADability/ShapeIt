using CADability.Curve2D;
using CADability.Substitutes;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace CADability.GeoObject
{
    public partial class Surfaces
    {
        #region marching from seed to seed

        /// <summary>
        /// The maximum angle by which the tangent of the intersection curve may turn in a single step. A
        /// bigger turn means the step was too long for the curvature here and it is retried with half the step.
        /// </summary>
        private const double marchingMaxBend = Math.PI / 6.0; // 30 degrees
        /// <summary>
        /// |n1 x n2| below this value counts as tangential: there the direction of the intersection curve
        /// cannot be computed from the two normals any more.
        /// </summary>
        private const double marchingTangentialSine = 1e-3;
        /// <summary>How often a step is halved before the branch gives up.</summary>
        private const int marchingMaxHalvings = 6;

        /// <summary>
        /// Finds the intersection curves of two arbitrary surfaces which run between the given points
        /// (<paramref name="seeds"/>). The seeds are points on both surfaces, usually the endpoints of the
        /// curves we are looking for, e.g. where an edge of one face penetrates the other face. This is the
        /// general method: it makes no assumption about the kind of the surfaces and is used when there is no
        /// special (analytic) intersection for the two surfaces at hand.
        /// <para>
        /// From every seed the intersection curve is followed in both directions - in four directions when the
        /// two surfaces touch at that seed and the intersection curve has a node there, see
        /// <see cref="ContactAt(ISurface, ISurface, GeoPoint, double, double)"/>. All these branches march at
        /// the same pace, so two branches which run towards each other on the same curve meet somewhere in the
        /// middle, where they are combined into one curve. A single step goes the step length along the current
        /// tangent, erects the plane perpendicular to the tangent there and intersects this plane with both
        /// surfaces; when there is no such point, or when the curve bends too much, the step is repeated with
        /// half the step length.
        /// </para>
        /// <para>
        /// A branch which leaves the given domains is dropped: the returned curves always lie completely inside
        /// <paramref name="bounds1"/> and <paramref name="bounds2"/> or on their border. And there is no
        /// guarantee that any two seeds are connected: with more than two seeds usually only some pairs are,
        /// and two seeds may well be connected by two different curves, which is the normal case when both of
        /// them are touching points or when the intersection is a closed curve.
        /// </para>
        /// </summary>
        /// <param name="surface1">the first surface</param>
        /// <param name="bounds1">the domain of interest on the first surface</param>
        /// <param name="surface2">the second surface</param>
        /// <param name="bounds2">the domain of interest on the second surface</param>
        /// <param name="seeds">points on both surfaces, at least two, which are the endpoints of the
        /// intersection curves</param>
        /// <returns>the intersection curves found, an empty array when there are none</returns>
        public static IDualSurfaceCurve[] GetIntersectionCurves(ISurface surface1, BoundingRect bounds1,
            ISurface surface2, BoundingRect bounds2, List<GeoPoint> seeds)
        {
            if (seeds == null || seeds.Count < 2) return new IDualSurfaceCurve[0];

            // seeds which coincide would produce the same branches twice
            List<GeoPoint> points = new List<GeoPoint>();
            for (int i = 0; i < seeds.Count; i++)
            {
                bool duplicate = false;
                for (int j = 0; j < points.Count; j++) if (Precision.IsEqual(points[j], seeds[i])) { duplicate = true; break; }
                if (!duplicate) points.Add(seeds[i]);
            }
            if (points.Count < 2) return new IDualSurfaceCurve[0];

            // The seeds must be inside the domains. They usually come from an edge, i.e. from the border of the
            // domain, and a seed a rounding error outside would kill its branches with their very first step.
            for (int i = 0; i < points.Count; i++)
            {
                GeoPoint2D uv = surface1.PositionOf(points[i]);
                SurfaceHelper.AdjustPeriodic(surface1, bounds1, ref uv);
                bounds1.MinMax(uv);
                uv = surface2.PositionOf(points[i]);
                SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref uv);
                bounds2.MinMax(uv);
            }
            bounds1.Inflate(bounds1.Size * 1e-6);
            bounds2.Inflate(bounds2.Size * 1e-6);

            // the step length: a tenth of the smallest distance between two seeds, so that a seed cannot be
            // stepped over without noticing it
            double minSeedDistance = double.MaxValue;
            for (int i = 0; i < points.Count - 1; i++)
            {
                for (int j = i + 1; j < points.Count; j++)
                {
                    double d = points[i] | points[j];
                    if (d < minSeedDistance) minSeedDistance = d;
                }
            }
            double nominalStep = minSeedDistance * 0.1;
            if (nominalStep <= 0.0) return new IDualSurfaceCurve[0];

            // a rough measure of the size of the two patches, only used to limit the number of steps
            BoundingBox extent = surface1.GetPatchExtent(bounds1, true);
            extent.MinMax(surface2.GetPatchExtent(bounds2, true));
            double size = extent.Size;

            MarchingContext ctx = new MarchingContext
            {
                surface1 = surface1,
                bounds1 = bounds1,
                surface2 = surface2,
                bounds2 = bounds2,
                seeds = points,
                nominalStep = nominalStep,
                // the two surface points of a step must coincide this well, otherwise the solver has stalled
                // somewhere without having found a common point
                tolerance = Math.Max(Precision.eps, nominalStep * 1e-5)
            };
            ctx.debug.Init(ctx);

            List<MarchingBranch> branches = new List<MarchingBranch>();
            for (int i = 0; i < points.Count; i++)
            {
                GeoPoint2D uv1 = surface1.PositionOf(points[i]), uv2 = surface2.PositionOf(points[i]);
                SurfaceHelper.AdjustPeriodic(surface1, bounds1, ref uv1);
                SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref uv2);
                InterpolatedDualSurfaceCurve.SurfacePoint start = new InterpolatedDualSurfaceCurve.SurfacePoint(points[i], uv1, uv2);
                foreach (GeoVector dir in StartDirections(ctx, uv1, uv2))
                {   // both ways along each direction: two branches at a normal seed, four where the surfaces
                    // touch and two branches of the intersection curve cross
                    branches.Add(new MarchingBranch(i, branches.Count, start, dir, nominalStep));
                    branches.Add(new MarchingBranch(i, branches.Count, start, -dir, nominalStep));
                }
            }
            ctx.debug.ShowBranchStart(branches);

            List<IDualSurfaceCurve> res = new List<IDualSurfaceCurve>();
            // the points each curve was made of, kept for the duplicate test at the end
            List<List<InterpolatedDualSurfaceCurve.SurfacePoint>> resPoints = new List<List<InterpolatedDualSurfaceCurve.SurfacePoint>>();
            int maxRounds = (int)Math.Min(5000.0, Math.Max(100.0, 20.0 * size / nominalStep));
            for (int round = 0; round < maxRounds; round++)
            {
                bool anyActive = false;
                for (int i = 0; i < branches.Count; i++)
                {
                    if (!branches[i].active) continue;
                    MarchingResult stepResult = branches[i].Advance(ctx);
                    if (stepResult == MarchingResult.Continued) anyActive = true;
                    else if (stepResult == MarchingResult.EndedAtSeed)
                    {   // this branch alone is a complete curve, from its own seed to the seed it stopped at.
                        // This happens when there is no branch marching towards it, which is typically the case
                        // at a tangential seed, where no branch can start.
                        AddCurve(ctx, res, resPoints, branches[i].points);
                    }
                }
                // two branches which have come close enough, in distance and in direction, are the two halves
                // of one curve
                for (int i = 0; i < branches.Count - 1; i++)
                {
                    if (!branches[i].active) continue;
                    for (int j = i + 1; j < branches.Count; j++)
                    {
                        if (!branches[j].active) continue;
                        if (!CanJoin(branches[i], branches[j])) continue;
                        List<InterpolatedDualSurfaceCurve.SurfacePoint> joined =
                            new List<InterpolatedDualSurfaceCurve.SurfacePoint>(branches[i].points);
                        for (int k = branches[j].points.Count - 1; k >= 0; k--) joined.Add(branches[j].points[k]);
                        ctx.debug.ShowJoin(branches[i], branches[j]);
                        AddCurve(ctx, res, resPoints, joined);
                        branches[i].active = false;
                        branches[j].active = false;
                        break;
                    }
                }
                if (!anyActive) break;
            }

            // The same curve may have been found twice: when a branch ends at a seed instead of meeting the
            // branch coming from there, that other branch marches on and ends at this branch's seed. The test
            // uses the marched points, not the curves: two curves may well connect the same two seeds and only
            // their course tells them apart.
            for (int i = 0; i < res.Count - 1; i++)
            {
                for (int j = res.Count - 1; j > i; j--)
                {
                    if (SameCourse(resPoints[i], resPoints[j], nominalStep))
                    {
                        res.RemoveAt(j);
                        resPoints.RemoveAt(j);
                    }
                }
            }
            ctx.debug.ShowSummary(branches);
            ctx.debug.ShowResult(res);
            return res.ToArray();
        }

        /// <summary>
        /// The directions in which the intersection curve leaves the seed, one for each branch of the curve:
        /// normally the single direction n1 x n2, and where the two surfaces touch the two directions of the
        /// crossing branches. Each of them is used forwards and backwards by the caller.
        /// </summary>
        private static List<GeoVector> StartDirections(MarchingContext ctx, GeoPoint2D uv1, GeoPoint2D uv2)
        {
            List<GeoVector> res = new List<GeoVector>();
            GeoVector n1 = ctx.surface1.GetNormal(uv1), n2 = ctx.surface2.GetNormal(uv2);
            if (n1.IsNullVector() || n2.IsNullVector()) return res;
            GeoVector dir = n1.Normalized ^ n2.Normalized;
            if (dir.Length > marchingTangentialSine)
            {   // the normal case: the surfaces intersect transversally here, a single curve passes through
                // this point and its tangent is perpendicular to both normals
                res.Add(dir.Normalized);
                return res;
            }
            // The surfaces touch here. When the intersection curve has a node, the two branches cross with the
            // directions the contact provides, so there are four ways to leave this point. An isolated contact
            // is the whole intersection here, and where the surfaces osculate (ContactType.Degenerate) there is
            // no direction to be computed: no branch starts at such a seed, but a branch coming from elsewhere
            // may still end there. The tolerances are generous, because the normals are known to be parallel
            // already and a seed which comes from somewhere else may be a little off the surfaces.
            SurfaceContact contact = ContactAt(ctx.surface1, uv1, ctx.surface2, uv2, ctx.nominalStep, 1e-2);
            if (contact != null && contact.Type == ContactType.Crossing) res.AddRange(contact.BranchDirections);
            return res;
        }

        /// <summary>
        /// A single step: from <paramref name="from"/> the given step length along <paramref name="direction"/>,
        /// then the plane perpendicular to the direction through that point is intersected with both surfaces.
        /// The result is checked, because the solver also returns true when it stalls somewhere without having
        /// found a common point of the three surfaces.
        /// </summary>
        private static bool MarchingStep(MarchingContext ctx, GeoPoint from, GeoPoint2D uvFrom1, GeoPoint2D uvFrom2,
            GeoVector direction, double step, ref GeoPoint ip, ref GeoPoint2D uv1, ref GeoPoint2D uv2)
        {
            GeoPoint target = from + step * direction;
            PlaneSurface ps = new PlaneSurface(new Plane(target, direction));
            GeoPoint2D uvPlane = GeoPoint2D.Origin; // the target point is the origin of that plane
            uv1 = ctx.surface1.PositionOf(target);
            uv2 = ctx.surface2.PositionOf(target);
            // on a periodic surface PositionOf may return a parameter a full period away from the last point,
            // which would make the solver jump to a totally different place
            InterpolatedDualSurfaceCurve.SurfacePoint.FixSurfacePoint2D(ref uv1, uvFrom1,
                ctx.surface1.IsUPeriodic, ctx.surface1.UPeriod, ctx.surface1.IsVPeriodic, ctx.surface1.VPeriod);
            InterpolatedDualSurfaceCurve.SurfacePoint.FixSurfacePoint2D(ref uv2, uvFrom2,
                ctx.surface2.IsUPeriodic, ctx.surface2.UPeriod, ctx.surface2.IsVPeriodic, ctx.surface2.VPeriod);
            ip = target;
            if (!SurfaceIntersectionSolvers.SurfacesIntersectionLM_Analytic9(ps, ctx.surface1, ctx.surface2,
                ref uvPlane, ref uv1, ref uv2, ref ip)) return false;
            GeoPoint p1 = ctx.surface1.PointAt(uv1), p2 = ctx.surface2.PointAt(uv2);
            if ((p1 | p2) > ctx.tolerance) return false; // no common point: the solver stopped in a local minimum
            if (Math.Abs((p1 - target) * direction) > ctx.tolerance) return false; // not in the plane
            // every point of that plane is at least one step away from "from", so a much bigger distance means
            // the solver has found some other part of the intersection, not the continuation of our curve
            if ((ip | from) > 2.0 * step) return false;
            return true;
        }

        /// <summary>
        /// Whether the two branches have met: their heads are close enough, they run towards each other and
        /// their tangents fit. Two branches which start at the same seed in opposite directions are moving away
        /// from each other and are not joined - unless they come back, which is a closed curve.
        /// </summary>
        private static bool CanJoin(MarchingBranch branch1, MarchingBranch branch2)
        {
            if (branch1.points.Count < 2 || branch2.points.Count < 2) return false; // both must have made a step
            double step = Math.Max(branch1.step, branch2.step);
            GeoVector delta = branch2.Head - branch1.Head;
            double distance = delta.Length;
            if (distance > 1.5 * step) return false;
            if (new Angle(branch1.direction, -branch2.direction).Radian > 2.0 * marchingMaxBend) return false;
            if (distance > Precision.eps)
            {
                if (delta * branch1.direction <= 0.0 || delta * branch2.direction >= 0.0) return false; // not approaching
                if (Geometry.DistPL(branch2.Head, branch1.Head, branch1.direction) > 0.5 * step) return false; // side by side
            }
            return true;
        }

        /// <summary>
        /// Makes an <see cref="InterpolatedDualSurfaceCurve"/> from the points marched so far and adds it to the
        /// result. Points too close to their predecessor are removed, they would only destabilize the
        /// interpolation, but the first and the last point, which are the seeds, are kept exactly.
        /// </summary>
        private static void AddCurve(MarchingContext ctx, List<IDualSurfaceCurve> res,
            List<List<InterpolatedDualSurfaceCurve.SurfacePoint>> resPoints,
            List<InterpolatedDualSurfaceCurve.SurfacePoint> points)
        {
            if (points.Count < 2) return;
            double minDistance = ctx.nominalStep * 0.1;
            List<InterpolatedDualSurfaceCurve.SurfacePoint> basePoints = new List<InterpolatedDualSurfaceCurve.SurfacePoint>();
            basePoints.Add(points[0]);
            for (int i = 1; i < points.Count - 1; i++)
            {
                if ((points[i].p3d | basePoints[basePoints.Count - 1].p3d) > minDistance) basePoints.Add(points[i]);
            }
            InterpolatedDualSurfaceCurve.SurfacePoint last = points[points.Count - 1];
            while (basePoints.Count > 1 && (last.p3d | basePoints[basePoints.Count - 1].p3d) < minDistance)
            {
                basePoints.RemoveAt(basePoints.Count - 1);
            }
            basePoints.Add(last);
            if (basePoints.Count < 2) return;
            if (basePoints.Count < 3 && (basePoints[0].p3d | last.p3d) < minDistance) return; // nothing but a point
            try
            {
                res.Add(new InterpolatedDualSurfaceCurve(ctx.surface1, ctx.bounds1, ctx.surface2, ctx.bounds2,
                    basePoints.ToArray()));
                resPoints.Add(basePoints);
            }
            catch (Exception)
            {   // these points do not make a usable curve, which the constructor decides: better to lose this
                // one curve than all the others
                ctx.debug.ShowFailedCurve(basePoints);
            }
        }

        /// <summary>
        /// Whether the two point sequences describe the same curve: same endpoints and the same course in
        /// between. The marched points are compared, not the curves made of them: a dual surface curve
        /// evaluated between its base points may leave its own course at a node, where it can continue on the
        /// other branch of the intersection, and that would make two really different curves look alike.
        /// </summary>
        private static bool SameCourse(List<InterpolatedDualSurfaceCurve.SurfacePoint> points1,
            List<InterpolatedDualSurfaceCurve.SurfacePoint> points2, double tolerance)
        {
            GeoPoint sp1 = points1[0].p3d, ep1 = points1[points1.Count - 1].p3d;
            GeoPoint sp2 = points2[0].p3d, ep2 = points2[points2.Count - 1].p3d;
            bool sameEnds = ((sp1 | sp2) < tolerance && (ep1 | ep2) < tolerance)
                         || ((sp1 | ep2) < tolerance && (ep1 | sp2) < tolerance);
            if (!sameEnds) return false;
            for (int i = 1; i < points2.Count - 1; i++)
            {   // the points of the two sequences do not correspond to each other, so each point is measured
                // against the whole polygon of the other sequence
                double distance = double.MaxValue;
                for (int j = 0; j < points1.Count - 1; j++)
                {
                    distance = Math.Min(distance, DistanceToSegment(points2[i].p3d, points1[j].p3d, points1[j + 1].p3d));
                }
                if (distance > tolerance) return false;
            }
            return true;
        }

        /// <summary>The distance of a point to a segment, not to the line through it.</summary>
        private static double DistanceToSegment(GeoPoint p, GeoPoint startPoint, GeoPoint endPoint)
        {
            GeoVector dir = endPoint - startPoint;
            double length2 = dir * dir;
            if (length2 < 1e-30) return p | startPoint;
            double par = ((p - startPoint) * dir) / length2;
            if (par < 0.0) par = 0.0;
            else if (par > 1.0) par = 1.0;
            return p | (startPoint + par * dir);
        }

        /// <summary>Everything the marching needs to know, so that a branch can do its steps on its own.</summary>
        private class MarchingContext
        {
            public ISurface surface1, surface2;
            public BoundingRect bounds1, bounds2;
            public List<GeoPoint> seeds;
            /// <summary>the step length a branch uses when nothing forces it to be more careful</summary>
            public double nominalStep;
            /// <summary>how well the points of the two surfaces must coincide to be a point of the curve</summary>
            public double tolerance;
            public MarchingDebug debug = new MarchingDebug();
        }

        /// <summary>What a single step of a branch did.</summary>
        private enum MarchingResult
        {
            /// <summary>the branch made a step and is still marching</summary>
            Continued,
            /// <summary>the branch reached a seed: it is a complete curve now</summary>
            EndedAtSeed,
            /// <summary>the branch cannot be continued or has left the domain: it is dropped</summary>
            Dead
        }

        /// <summary>
        /// One branch of the intersection curve, marching away from its seed. All branches advance one step per
        /// round, so that two branches on the same curve running towards each other meet in the middle.
        /// </summary>
        private class MarchingBranch
        {
            /// <summary>index of the seed this branch starts at</summary>
            public readonly int startSeed;
            /// <summary>only used to give each branch its own color when debugging</summary>
            public readonly int index;
            /// <summary>the points found so far, the first one being the seed</summary>
            public readonly List<InterpolatedDualSurfaceCurve.SurfacePoint> points;
            /// <summary>unit tangent at the head, pointing the way this branch is marching</summary>
            public GeoVector direction;
            /// <summary>the current step length, halved at difficult places</summary>
            public double step;
            /// <summary>false when this branch has stopped, for whatever reason</summary>
            public bool active;

            public MarchingBranch(int startSeed, int index, InterpolatedDualSurfaceCurve.SurfacePoint start,
                GeoVector direction, double step)
            {
                this.startSeed = startSeed;
                this.index = index;
                this.direction = direction;
                this.step = step;
                points = new List<InterpolatedDualSurfaceCurve.SurfacePoint> { start };
                active = true;
            }

            public InterpolatedDualSurfaceCurve.SurfacePoint HeadPoint { get { return points[points.Count - 1]; } }
            public GeoPoint Head { get { return points[points.Count - 1].p3d; } }

            /// <summary>
            /// Makes a single step: one step length along the tangent, then the plane perpendicular to the
            /// tangent there intersected with both surfaces. When there is no such point, or when the curve
            /// bends too much for this step length, it is tried again with half the step.
            /// </summary>
            public MarchingResult Advance(MarchingContext ctx)
            {
                GeoPoint from = Head;
                GeoPoint2D uvFrom1 = HeadPoint.psurface1, uvFrom2 = HeadPoint.psurface2;
                double stepLength = step;
                for (int i = 0; i < marchingMaxHalvings; i++, stepLength /= 2.0)
                {
                    GeoPoint ip = GeoPoint.Origin;
                    GeoPoint2D uv1 = uvFrom1, uv2 = uvFrom2;
                    ctx.debug.ShowProbe(this, from + stepLength * direction);
                    if (!MarchingStep(ctx, from, uvFrom1, uvFrom2, direction, stepLength, ref ip, ref uv1, ref uv2)) continue;
                    GeoVector tangent = ctx.surface1.GetNormal(uv1).Normalized ^ ctx.surface2.GetNormal(uv2).Normalized;
                    if (tangent.Length < marchingTangentialSine)
                    {   // the normals became parallel: we are running into a touching point, where the tangent of
                        // the curve cannot be computed this way. A smaller step may still get there.
                        ctx.debug.ShowTangential(this, ip, uv1, uv2);
                        continue;
                    }
                    tangent.Norm();
                    // the cross product of the normals flips its sign when the curve passes a touching point,
                    // whereas the curve itself keeps its direction there
                    if (tangent * direction < 0.0) tangent = -tangent;
                    if (new Angle(tangent, direction).Radian > marchingMaxBend ||
                        new Angle(ip - from, direction).Radian > marchingMaxBend)
                    {
                        ctx.debug.ShowTooSharp(this, ip, uv1, uv2);
                        continue; // too much bending for this step length
                    }
                    SurfaceHelper.AdjustPeriodic(ctx.surface1, ctx.bounds1, ref uv1);
                    SurfaceHelper.AdjustPeriodic(ctx.surface2, ctx.bounds2, ref uv2);
                    points.Add(new InterpolatedDualSurfaceCurve.SurfacePoint(ip, uv1, uv2));
                    direction = tangent;
                    // after a difficult place the step length grows back to its nominal value, otherwise a single
                    // tight bend would slow down the whole branch
                    step = (i == 0) ? Math.Min(ctx.nominalStep, step * 1.3) : stepLength;
                    ctx.debug.ShowStep(this);

                    int crossed = CrossedSeed(ctx);
                    if (crossed >= 0)
                    {
                        EndAtSeed(ctx, crossed, true);
                        return MarchingResult.EndedAtSeed;
                    }
                    if (!ctx.bounds1.Contains(uv1) || !ctx.bounds2.Contains(uv2))
                    {   // the curve leaves the domain of one of the surfaces, so it is not one of the curves we
                        // are looking for: those lie completely inside the domains or on their border
                        ctx.debug.ShowDead(this, "left the domain");
                        active = false;
                        return MarchingResult.Dead;
                    }
                    return MarchingResult.Continued;
                }
                // No step was possible. This is what happens when we approach a point where the two surfaces
                // touch: when there is a seed just ahead, this is where the curve ends.
                int ahead = SeedAhead(ctx);
                if (ahead >= 0)
                {
                    EndAtSeed(ctx, ahead, false);
                    return MarchingResult.EndedAtSeed;
                }
                ctx.debug.ShowDead(this, "no step possible");
                active = false;
                return MarchingResult.Dead;
            }

            /// <summary>The seed the last step has stepped over, -1 when there is none.</summary>
            private int CrossedSeed(MarchingContext ctx)
            {
                if (points.Count < 2) return -1;
                GeoPoint previous = points[points.Count - 2].p3d;
                for (int i = 0; i < ctx.seeds.Count; i++)
                {   // a closed curve comes back to its own seed, but not within the first steps
                    if (i == startSeed && points.Count < 4) continue;
                    if (Geometry.IsNearSegment(previous, Head, ctx.seeds[i], marchingMaxBend)) return i;
                }
                return -1;
            }

            /// <summary>A seed within the reach of a few steps ahead, -1 when there is none.</summary>
            private int SeedAhead(MarchingContext ctx)
            {
                GeoPoint ahead = Head + 4.0 * step * direction;
                for (int i = 0; i < ctx.seeds.Count; i++)
                {
                    if (i == startSeed && points.Count < 4) continue;
                    if (Geometry.IsNearSegment(Head, ahead, ctx.seeds[i], marchingMaxBend)) return i;
                }
                return -1;
            }

            /// <summary>
            /// Ends this branch exactly at the given seed: the curves have to end at the seeds, which are the
            /// vertices of the model, so the last point is not the marched one but the seed itself.
            /// </summary>
            private void EndAtSeed(MarchingContext ctx, int seedIndex, bool replaceHead)
            {
                GeoPoint seed = ctx.seeds[seedIndex];
                GeoPoint2D uv1 = ctx.surface1.PositionOf(seed), uv2 = ctx.surface2.PositionOf(seed);
                SurfaceHelper.AdjustPeriodic(ctx.surface1, ctx.bounds1, ref uv1);
                SurfaceHelper.AdjustPeriodic(ctx.surface2, ctx.bounds2, ref uv2);
                InterpolatedDualSurfaceCurve.SurfacePoint sp = new InterpolatedDualSurfaceCurve.SurfacePoint(seed, uv1, uv2);
                if (replaceHead && points.Count > 1) points[points.Count - 1] = sp;
                else points.Add(sp);
                ctx.debug.ShowSeedReached(this, seed);
                active = false;
            }
        }

        /// <summary>
        /// Shows how the marching proceeds, which is what you want to see when something goes wrong. There are
        /// four containers to look at in the debugger:
        /// <list type="bullet">
        /// <item><c>dc</c>: the two surface patches, the seeds and the 3d points and steps of all branches</item>
        /// <item><c>dc21</c>: the domain of the first surface with the 2d points of all branches</item>
        /// <item><c>dc22</c>: the same for the second surface</item>
        /// <item><c>dcres</c>: the curves which are returned</item>
        /// </list>
        /// Each branch has its own color and the debug hint of a point is the index of its branch. Rejected
        /// points are shown as well: DeepPink where a step was tried, Magenta where the surfaces became
        /// tangential, DarkCyan where the curve bent too much and Gray where a branch was dropped. In a release
        /// build all of this is compiled away, the methods are <see cref="ConditionalAttribute"/>.
        /// </summary>
        private class MarchingDebug
        {
#if DEBUG
            /// <summary>the two surfaces with their domains as faces and the accumulated 3d points</summary>
            public DebuggerContainer dc = new DebuggerContainer();
            /// <summary>the domain of the first surface and the accumulated 2d points</summary>
            public DebuggerContainer dc21 = new DebuggerContainer();
            /// <summary>the domain of the second surface and the accumulated 2d points</summary>
            public DebuggerContainer dc22 = new DebuggerContainer();
            /// <summary>the curves of the result</summary>
            public DebuggerContainer dcres = new DebuggerContainer();
            private MarchingContext ctx;
            private static readonly Color[] branchColors = new Color[]
            {
                Color.Red, Color.Green, Color.Blue, Color.Orange, Color.Brown, Color.DarkViolet,
                Color.DarkGreen, Color.DarkBlue, Color.Gold, Color.Teal, Color.Maroon, Color.OliveDrab
            };
            private static Color BranchColor(MarchingBranch branch)
            {
                return branchColors[branch.index % branchColors.Length];
            }
            private void Show(MarchingBranch branch, GeoPoint p3d, GeoPoint2D uv1, GeoPoint2D uv2, Color color)
            {
                dc.Add(p3d, color, branch.index);
                dc21.Add(uv1, color, branch.index);
                dc22.Add(uv2, color, branch.index);
            }
#endif
            [Conditional("DEBUG")]
            public void Init(MarchingContext context)
            {
#if DEBUG
                ctx = context;
                try
                {
                    dc.Add(Face.MakeFace(ctx.surface1, ctx.bounds1), Color.Red);
                    dc.Add(Face.MakeFace(ctx.surface2, ctx.bounds2), Color.Green);
                }
                catch (Exception) { } // the patches are only there to see where we are
                dc21.Add(ctx.bounds1.ToBorder(), Color.Red, 0);
                dc22.Add(ctx.bounds2.ToBorder(), Color.Green, 0);
                for (int i = 0; i < ctx.seeds.Count; i++)
                {
                    dc.Add(ctx.seeds[i], Color.Black, i);
                    dc21.Add(ctx.surface1.PositionOf(ctx.seeds[i]), Color.Black, i);
                    dc22.Add(ctx.surface2.PositionOf(ctx.seeds[i]), Color.Black, i);
                }
#endif
            }
            /// <summary>The direction in which each branch starts, as a line of one step length.</summary>
            [Conditional("DEBUG")]
            public void ShowBranchStart(List<MarchingBranch> branches)
            {
#if DEBUG
                for (int i = 0; i < branches.Count; i++)
                {
                    dc.Add(Line.TwoPoints(branches[i].Head, branches[i].Head + branches[i].step * branches[i].direction),
                        BranchColor(branches[i]), branches[i].index);
                }
#endif
            }
            /// <summary>Where a step is being tried, before it is known whether it works.</summary>
            [Conditional("DEBUG")]
            public void ShowProbe(MarchingBranch branch, GeoPoint probe)
            {
#if DEBUG
                dc.Add(probe, Color.DeepPink, branch.index);
#endif
            }
            /// <summary>A step which ended on a touching point, where the curve has no computable tangent.</summary>
            [Conditional("DEBUG")]
            public void ShowTangential(MarchingBranch branch, GeoPoint p3d, GeoPoint2D uv1, GeoPoint2D uv2)
            {
#if DEBUG
                Show(branch, p3d, uv1, uv2, Color.Magenta);
#endif
            }
            /// <summary>A step which was rejected because the curve bends too much.</summary>
            [Conditional("DEBUG")]
            public void ShowTooSharp(MarchingBranch branch, GeoPoint p3d, GeoPoint2D uv1, GeoPoint2D uv2)
            {
#if DEBUG
                Show(branch, p3d, uv1, uv2, Color.DarkCyan);
#endif
            }
            /// <summary>An accepted step, drawn as the segment from the previous point in the branch color.</summary>
            [Conditional("DEBUG")]
            public void ShowStep(MarchingBranch branch)
            {
#if DEBUG
                Color color = BranchColor(branch);
                InterpolatedDualSurfaceCurve.SurfacePoint sp = branch.HeadPoint;
                Show(branch, sp.p3d, sp.psurface1, sp.psurface2, color);
                if (branch.points.Count > 1)
                {
                    InterpolatedDualSurfaceCurve.SurfacePoint previous = branch.points[branch.points.Count - 2];
                    dc.Add(Line.TwoPoints(previous.p3d, sp.p3d), color, branch.index);
                    dc21.Add(new Line2D(previous.psurface1, sp.psurface1), color, branch.index);
                    dc22.Add(new Line2D(previous.psurface2, sp.psurface2), color, branch.index);
                }
#endif
            }
            /// <summary>Where a branch reached a seed and thereby became a complete curve.</summary>
            [Conditional("DEBUG")]
            public void ShowSeedReached(MarchingBranch branch, GeoPoint seed)
            {
#if DEBUG
                dc.Add(seed, BranchColor(branch), branch.index);
#endif
            }
            /// <summary>The connection of the two branches which met.</summary>
            [Conditional("DEBUG")]
            public void ShowJoin(MarchingBranch branch1, MarchingBranch branch2)
            {
#if DEBUG
                if ((branch1.Head | branch2.Head) > Precision.eps)
                {
                    dc.Add(Line.TwoPoints(branch1.Head, branch2.Head), Color.Black, branch1.index);
                }
                dc21.Add(new Line2D(branch1.HeadPoint.psurface1, branch2.HeadPoint.psurface1), Color.Black, branch1.index);
                dc22.Add(new Line2D(branch1.HeadPoint.psurface2, branch2.HeadPoint.psurface2), Color.Black, branch1.index);
#endif
            }
            /// <summary>Where a branch was dropped, and why.</summary>
            [Conditional("DEBUG")]
            public void ShowDead(MarchingBranch branch, string reason)
            {
#if DEBUG
                dc.Add(branch.Head, Color.Gray, branch.index);
                Trace.WriteLine("branch " + branch.index.ToString() + " from seed " + branch.startSeed.ToString()
                    + " dropped: " + reason);
#endif
            }
            /// <summary>What became of each branch, written to the trace output.</summary>
            [Conditional("DEBUG")]
            public void ShowSummary(List<MarchingBranch> branches)
            {
#if DEBUG
                for (int i = 0; i < branches.Count; i++)
                {
                    MarchingBranch b = branches[i];
                    Trace.WriteLine("branch " + b.index.ToString() + " from seed " + b.startSeed.ToString()
                        + ": " + b.points.Count.ToString() + " points, " + (b.active ? "still marching" : "stopped")
                        + ", head " + b.Head.ToString());
                }
#endif
            }
            /// <summary>Points which did not make a usable curve.</summary>
            [Conditional("DEBUG")]
            public void ShowFailedCurve(List<InterpolatedDualSurfaceCurve.SurfacePoint> points)
            {
#if DEBUG
                for (int i = 0; i < points.Count; i++) dcres.Add(points[i].p3d, Color.Red, i);
#endif
            }
            /// <summary>The curves which are being returned.</summary>
            [Conditional("DEBUG")]
            public void ShowResult(List<IDualSurfaceCurve> curves)
            {
#if DEBUG
                for (int i = 0; i < curves.Count; i++)
                {
                    if (curves[i].Curve3D is IGeoObject go) dcres.Add(go, branchColors[i % branchColors.Length], i);
                }
#endif
            }
        }

        #endregion
    }
}
