using System;
using System.Collections.Generic;

namespace CADability.GeoObject
{
    /// <summary>
    /// How two surfaces behave at a point where they touch, i.e. where they share a point and their
    /// normals are parallel. The type is decided by the difference of the second fundamental forms.
    /// </summary>
    public enum ContactType
    {
        /// <summary>
        /// The intersection curve has a node here: two branches cross, so four branches leave the point.
        /// This is the case that makes a surface/surface intersection fail if it is not known in advance -
        /// the intersection curve is not a regular curve here. <see cref="SurfaceContact.BranchDirections"/>
        /// holds the tangents of the two branches.
        /// </summary>
        Crossing,
        /// <summary>
        /// The surfaces touch but do not cross: locally the intersection is this single point.
        /// </summary>
        Isolated,
        /// <summary>
        /// The contact is of higher order: the surfaces osculate, and they usually touch along a whole
        /// curve rather than at a point (two cylinders of equal radius with parallel axes, a sphere
        /// inside a cylinder of the same radius, coincident surfaces). See the remarks on
        /// <see cref="Surfaces.TangentialContacts"/> about how such a contact is reported.
        /// </summary>
        Degenerate
    }

    /// <summary>
    /// A point at which two surfaces touch: same point, parallel normals. Depending on
    /// <see cref="Type"/> this is a node of the intersection curve, an isolated touching point, or a
    /// sample of a whole curve of contact.
    /// </summary>
    public class SurfaceContact
    {
        /// <summary>The point of contact.</summary>
        public GeoPoint Location;
        /// <summary>Parameters of <see cref="Location"/> on the first surface.</summary>
        public GeoPoint2D uv1;
        /// <summary>Parameters of <see cref="Location"/> on the second surface.</summary>
        public GeoPoint2D uv2;
        /// <summary>The common normal, normalized and oriented like the normal of the first surface.</summary>
        public GeoVector Normal;
        /// <summary>What the intersection curve does here.</summary>
        public ContactType Type;
        /// <summary>
        /// For <see cref="ContactType.Crossing"/> the two (unit) tangents of the crossing branches, so that
        /// a marching intersection can be started in the four directions +-BranchDirections[0/1]. Empty
        /// for the other types.
        /// </summary>
        public GeoVector[] BranchDirections = new GeoVector[0];
        /// <summary>Distance between the two surface points, a measure of how exact the contact is.</summary>
        public double DistanceDefect;
        /// <summary>
        /// Angle between the two normals in radian, 0 for a perfect contact. Values noticeably above 0
        /// mean the surfaces are only ALMOST tangent here - which for an intersection algorithm is just as
        /// dangerous as an exact contact, which is why such points are reported rather than discarded.
        /// </summary>
        public double AngleDefect;

        public override string ToString()
        {
            return Type.ToString() + " at " + Location.ToString();
        }
    }

    public partial class Surfaces
    {
        /// <summary>
        /// Upper limit on the number of returned contacts. A contact along a whole curve produces one
        /// sample after another, so the search has to stop somewhere; see the remarks on
        /// <see cref="TangentialContacts"/>.
        /// </summary>
        private const int maxContacts = 16;

        /// <summary>
        /// Finds the points at which the two surfaces touch: points that lie on both surfaces and where
        /// the normals are parallel. These are exactly the singular points of the intersection curve -
        /// where it is of type <see cref="ContactType.Crossing"/> the curve has a node, two branches
        /// crossing, and a marching intersection has to know about it in advance.
        /// <para>
        /// Implemented for the natural quadrics, which are all canal surfaces - the envelope of a one
        /// parameter family of spheres with centre c(t) and radius r(t):
        /// sphere (a single sphere), cylinder (spine the axis, r constant), cone (spine the axis, r linear)
        /// and torus (spine the centre circle, r constant). For such surfaces the normal at a point is
        /// radial with respect to the centre of the corresponding sphere, so a contact point with common
        /// normal n satisfies
        /// </para>
        /// <para>
        /// P = c1(t1) + r1(t1)*n = c2(t2) + r2(t2)*n
        /// </para>
        /// <para>
        /// which says c1 - c2 is parallel to n and the two spheres touch, plus the envelope condition
        /// n*c'(t) = -r'(t) on each side, saying that P lies on both characteristic circles. That reduces
        /// the four unknowns (u1,v1,u2,v2) to the two spine parameters (t1,t2).
        /// </para>
        /// <para>
        /// A PLANE is handled by the same two conditions with n fixed to its normal, which leaves a single
        /// spine parameter. What comes out is worth knowing before using this: a plane touches a sphere at
        /// one isolated point, and it touches a cylinder or a cone along a whole ruling - never at a point,
        /// because a constant or linear radius makes the envelope condition independent of t, so it holds
        /// everywhere or nowhere. The TORUS is the only one of the four where a plane produces a node: at
        /// the inner equator, where the surface is saddle shaped, and above all at the bitangent plane
        /// through the centre, which touches at two points and cuts the torus in the two Villarceau circles
        /// crossing exactly there. Two planes touch only if they are the same plane.
        /// </para>
        /// <para>
        /// Anything that is none of these surfaces yields an empty result - it is not a statement that
        /// there is no contact.
        /// </para>
        /// <para>
        /// The system is overdetermined by one equation, which is not a flaw of the formulation: tangency
        /// is a codimension one condition, so in real models it is hardly ever met exactly. Points that
        /// come close within <paramref name="precision"/> are therefore reported WITH their measured
        /// defect (<see cref="SurfaceContact.DistanceDefect"/>, <see cref="SurfaceContact.AngleDefect"/>)
        /// rather than discarded; missing a near-node is the more expensive error.
        /// </para>
        /// <para>
        /// A contact along a whole curve (a plane on a cylinder or a cone, equal cylinders with parallel
        /// axes, a sphere inside a cylinder of the same radius, coincident surfaces) is not condensed into
        /// one result: it appears as several <see cref="ContactType.Degenerate"/> samples spread along the
        /// contact curve, at most <c>maxContacts</c> of them. The type is the reliable signal there, not
        /// the number of points.
        /// </para>
        /// </summary>
        /// <param name="surface1">the first surface</param>
        /// <param name="bounds1">the parameter domain of interest on the first surface</param>
        /// <param name="surface2">the second surface</param>
        /// <param name="bounds2">the parameter domain of interest on the second surface</param>
        /// <param name="precision">geometric tolerance: how far apart the two surface points may be and
        /// still count as a contact</param>
        /// <returns>the contact points inside both domains, empty when there are none or when a surface is
        /// not one of the supported types</returns>
        public static SurfaceContact[] TangentialContacts(ISurface surface1, BoundingRect bounds1,
            ISurface surface2, BoundingRect bounds2, double precision)
        {
            List<GeoPoint> candidates = new List<GeoPoint>();
            List<GeoVector> candidateNormals = new List<GeoVector>();
            PlaneSurface plane1 = surface1 as PlaneSurface, plane2 = surface2 as PlaneSurface;
            if (plane1 != null && plane2 != null)
            {
                CoincidentPlanes(plane1, bounds1, plane2, precision, candidates, candidateNormals);
            }
            else if (plane1 != null || plane2 != null)
            {
                PlaneSurface plane = plane1 ?? plane2;
                ISurface other = plane1 != null ? surface2 : surface1;
                BoundingRect otherBounds = plane1 != null ? bounds2 : bounds1;
                CanalForm canal = CanalForm.Create(other, otherBounds, precision);
                if (canal == null) return new SurfaceContact[0];
                PlaneContacts(plane, canal, precision, candidates, candidateNormals);
            }
            else
            {
                CanalForm canal1 = CanalForm.Create(surface1, bounds1, precision);
                CanalForm canal2 = CanalForm.Create(surface2, bounds2, precision);
                if (canal1 == null || canal2 == null) return new SurfaceContact[0];
                for (int s = 0; s < 2; s++)
                {
                    double sigma = s == 0 ? 1.0 : -1.0;
                    CoincidentSphereContacts(canal1, canal2, sigma, precision, candidates, candidateNormals);
                    RegularContacts(canal1, canal2, sigma, precision, candidates, candidateNormals);
                }
            }

            List<SurfaceContact> res = new List<SurfaceContact>();
            for (int i = 0; i < candidates.Count; i++)
            {
                SurfaceContact contact = Verify(surface1, bounds1, surface2, bounds2,
                    candidates[i], candidateNormals[i], precision);
                if (contact == null) continue;
                bool duplicate = false;
                for (int j = 0; j < res.Count; j++)
                {
                    if ((res[j].Location | contact.Location) < precision * 10) { duplicate = true; break; }
                }
                if (!duplicate) res.Add(contact);
                if (res.Count >= maxContacts) break; // a contact along a curve would go on forever
            }
            return res.ToArray();
        }

        /// <summary>
        /// Classifies a contact point that has been found in some other way: a point which lies on both
        /// surfaces and where the two normals are parallel, i.e. point in the same or in opposite
        /// directions. This is the second half of what <see cref="TangentialContacts"/> does, made
        /// available for points that do not come from its own search - e.g. from a marching intersection
        /// which runs into a point where the normals become parallel, or from a vertex which is known to
        /// be a touching point.
        /// <para>
        /// The result says what the intersection curve does here: <see cref="ContactType.Crossing"/> means
        /// the curve has a node and <see cref="SurfaceContact.BranchDirections"/> holds the two unit
        /// tangents of the crossing branches, so the four directions +-BranchDirections[0] and
        /// +-BranchDirections[1] are the ones in which the intersection leaves this point.
        /// <see cref="ContactType.Isolated"/> means the surfaces touch without crossing, and
        /// <see cref="ContactType.Degenerate"/> means the contact is of higher order, which is what a
        /// whole curve of contact looks like at each of its points. For the latter two
        /// <see cref="SurfaceContact.BranchDirections"/> is empty.
        /// </para>
        /// <para>
        /// The tangency need not be exact: the classification uses the second fundamental forms of both
        /// surfaces taken with respect to the normal of the FIRST surface, which is what makes them
        /// comparable, also when the second surface is oriented the other way round. How far the given
        /// point is from a perfect contact is reported in <see cref="SurfaceContact.DistanceDefect"/> and
        /// <see cref="SurfaceContact.AngleDefect"/>. Unlike <see cref="TangentialContacts"/> this method
        /// does not check any parameter domain, the caller decides where the point may be. It works for
        /// any surface, not only for the natural quadrics.
        /// </para>
        /// </summary>
        /// <param name="surface1">the first surface</param>
        /// <param name="uv1">parameters of the point on the first surface</param>
        /// <param name="surface2">the second surface</param>
        /// <param name="uv2">parameters of the point on the second surface</param>
        /// <param name="precision">geometric tolerance: how far apart the two surface points may be and
        /// still count as the same point</param>
        /// <param name="maxAngle">how far the two normals may deviate from being parallel (in radian) and
        /// still count as a contact</param>
        /// <returns>the classified contact, or null when the two points are farther apart than
        /// <paramref name="precision"/>, when a normal is not defined or when the normals are not parallel
        /// within <paramref name="maxAngle"/></returns>
        public static SurfaceContact ContactAt(ISurface surface1, GeoPoint2D uv1,
            ISurface surface2, GeoPoint2D uv2, double precision, double maxAngle = 1e-3)
        {
            GeoPoint p1 = surface1.PointAt(uv1), p2 = surface2.PointAt(uv2);
            if ((p1 | p2) > precision) return null;

            GeoVector n1 = surface1.GetNormal(uv1), n2 = surface2.GetNormal(uv2);
            if (n1.IsNullVector() || n2.IsNullVector()) return null;
            n1.Norm();
            n2.Norm();
            double cosine = Math.Abs(n1 * n2);
            if (cosine > 1.0) cosine = 1.0;
            double angle = Math.Acos(cosine); // 0 for parallel normals, regardless of their orientation
            if (angle > maxAngle) return null;

            SurfaceContact res = new SurfaceContact
            {
                Location = new GeoPoint(p1, p2),
                uv1 = uv1,
                uv2 = uv2,
                Normal = n1,
                DistanceDefect = p1 | p2,
                AngleDefect = angle
            };
            Classify(surface1, uv1, surface2, uv2, n1, res);
            return res;
        }

        /// <summary>
        /// Same as <see cref="ContactAt(ISurface, GeoPoint2D, ISurface, GeoPoint2D, double, double)"/>,
        /// but for a point given in 3d: the parameters are determined with
        /// <see cref="ISurface.PositionOf"/> on both surfaces. On a periodic surface that yields some
        /// valid parameter pair, not necessarily the one of a particular domain; when that matters, use
        /// the overload which takes the parameters.
        /// </summary>
        /// <param name="surface1">the first surface</param>
        /// <param name="surface2">the second surface</param>
        /// <param name="location">the point, which is expected to lie on both surfaces</param>
        /// <param name="precision">geometric tolerance: how far the point may be from either surface</param>
        /// <param name="maxAngle">how far the two normals may deviate from being parallel (in radian) and
        /// still count as a contact</param>
        /// <returns>the classified contact, or null when this is not a contact point</returns>
        public static SurfaceContact ContactAt(ISurface surface1, ISurface surface2, GeoPoint location,
            double precision, double maxAngle = 1e-3)
        {
            GeoPoint2D uv1 = surface1.PositionOf(location), uv2 = surface2.PositionOf(location);
            if (((surface1.PointAt(uv1) | location) > precision) ||
                ((surface2.PointAt(uv2) | location) > precision)) return null;
            return ContactAt(surface1, uv1, surface2, uv2, precision, maxAngle);
        }


        #region the canal form of a natural quadric

        /// <summary>
        /// A surface written as the envelope of a one parameter family of spheres: centre c(t), signed
        /// radius r(t). The sign of the radius carries the orientation of the normal, P = c(t) + r(t)*n,
        /// which lets the second nappe of a cone be described by the same formula as the first.
        /// <para>
        /// The spine is a point (sphere), a line (cylinder, cone) or a circle (torus), and r is constant
        /// except for the cone, where it is linear. Everything the solver needs is expressed through
        /// <see cref="Centre"/>, <see cref="SpineTangent"/>, <see cref="Radius"/> and
        /// <see cref="RadiusSlope"/>, so the solver itself does not know which surface it is looking at.
        /// </para>
        /// </summary>
        private class CanalForm
        {
            private enum SpineKind { Point, Line, Circle }

            private SpineKind kind;
            private GeoPoint origin;        // the point, a point of the line, or the centre of the circle
            private GeoVector dir;          // unit direction of the line
            private GeoVector xdir, ydir;   // unit basis of the circle's plane
            private double circleRadius;    // radius of the spine circle
            private double radius0;         // r at t = 0
            private double radiusSlope;     // dr/dt, non zero only for the cone
            private double tmin, tmax;      // the part of the spine that matters for the given domain
            private bool periodic;          // the spine is a circle

            public GeoPoint Centre(double t)
            {
                switch (kind)
                {
                    case SpineKind.Point: return origin;
                    case SpineKind.Line: return origin + t * dir;
                    default: return origin + (circleRadius * Math.Cos(t)) * xdir + (circleRadius * Math.Sin(t)) * ydir;
                }
            }

            /// <summary>The unit tangent of the spine. A null vector when the spine is a single point.</summary>
            public GeoVector SpineTangent(double t)
            {
                switch (kind)
                {
                    case SpineKind.Point: return GeoVector.NullVector;
                    case SpineKind.Line: return dir;
                    default: return -Math.Sin(t) * xdir + Math.Cos(t) * ydir;
                }
            }

            /// <summary>The signed radius of the sphere at t.</summary>
            public double Radius(double t) { return radius0 + radiusSlope * t; }

            /// <summary>dr/ds along the spine, i.e. differentiated by ARC LENGTH so that it can be
            /// compared with a unit normal times the unit spine tangent.</summary>
            public double RadiusSlope(double t)
            {
                if (kind == SpineKind.Circle) return 0.0; // r is constant on a torus
                return radiusSlope;                       // the line spines are parametrized by arc length
            }

            public double TMin { get { return tmin; } }
            public double TMax { get { return tmax; } }
            public bool Periodic { get { return periodic; } }
            /// <summary>True when the spine has no parameter at all, i.e. the surface is a single sphere.</summary>
            public bool IsPoint { get { return kind == SpineKind.Point; } }

            /// <summary>
            /// Describes the surface as a canal surface, or returns null when it is not one of the supported
            /// types or not circular (an elliptical cylinder, a scaled sphere). The result is verified
            /// against the surface itself before it is returned, so a wrong assumption about how a surface
            /// stores its axes cannot silently produce wrong contact points.
            /// </summary>
            public static CanalForm Create(ISurface surface, BoundingRect bounds, double precision)
            {
                CanalForm res = null;
                if (surface is SphericalSurface sph)
                {
                    double r = sph.RadiusX;
                    if (Math.Abs(sph.RadiusY - r) > precision || Math.Abs(sph.RadiusZ - r) > precision) return null;
                    res = new CanalForm
                    {
                        kind = SpineKind.Point,
                        origin = sph.Location,
                        radius0 = r,
                        radiusSlope = 0.0,
                        tmin = 0.0,
                        tmax = 0.0,
                        periodic = false
                    };
                }
                else if (surface is CylindricalSurface cyl)
                {
                    double r = cyl.RadiusX;
                    if (Math.Abs(cyl.RadiusY - r) > precision) return null; // elliptical
                    res = new CanalForm
                    {
                        kind = SpineKind.Line,
                        origin = cyl.Location,
                        dir = cyl.Axis.Normalized,
                        radius0 = r,
                        radiusSlope = 0.0,
                        periodic = false
                    };
                    res.SetLineRange(surface, bounds);
                }
                else if (surface is ConicalSurface con)
                {
                    // the sphere inscribed at distance t from the apex has radius t*sin(semiangle)
                    double semi = con.OpeningAngle.Radian / 2.0;
                    if (semi <= 0.0 || semi >= Math.PI / 2.0) return null;
                    res = new CanalForm
                    {
                        kind = SpineKind.Line,
                        origin = con.Location, // the apex
                        dir = con.ZAxis.Normalized,
                        radius0 = 0.0,
                        radiusSlope = Math.Sin(semi),
                        periodic = false
                    };
                    res.SetLineRange(surface, bounds);
                }
                else if (surface is ToroidalSurface tor)
                {
                    GeoVector x = tor.XAxis, y = tor.YAxis;
                    if (Math.Abs(x.Length - y.Length) > precision) return null; // not circular
                    res = new CanalForm
                    {
                        kind = SpineKind.Circle,
                        origin = tor.Location,
                        xdir = x.Normalized,
                        ydir = y.Normalized,
                        circleRadius = tor.MajorRadius,
                        radius0 = tor.MinorRadius,
                        radiusSlope = 0.0,
                        tmin = 0.0,
                        tmax = 2.0 * Math.PI,
                        periodic = true
                    };
                }
                if (res == null) return null;
                return res.Matches(surface, bounds, precision) ? res : null;
            }

            /// <summary>The part of a line spine that the given domain projects onto, generously enlarged.</summary>
            private void SetLineRange(ISurface surface, BoundingRect bounds)
            {
                double lo = double.MaxValue, hi = double.MinValue;
                for (int i = 0; i <= 4; i++)
                {
                    for (int j = 0; j <= 4; j++)
                    {
                        GeoPoint2D uv = new GeoPoint2D(bounds.Left + bounds.Width * i / 4.0,
                                                       bounds.Bottom + bounds.Height * j / 4.0);
                        double t = (surface.PointAt(uv) - origin) * dir;
                        if (t < lo) lo = t;
                        if (t > hi) hi = t;
                    }
                }
                double margin = Math.Max(hi - lo, 1.0) * 0.25;
                tmin = lo - margin;
                tmax = hi + margin;
            }

            /// <summary>
            /// Checks the derived description against the surface: for sample points of the domain, the
            /// distance to the corresponding sphere centre must be the radius of that sphere. Guards the
            /// assumptions this class makes about how each surface stores its axes and radii.
            /// </summary>
            private bool Matches(ISurface surface, BoundingRect bounds, double precision)
            {
                double tol = Math.Max(precision, 1e-6) * 10.0;
                for (int i = 0; i <= 3; i++)
                {
                    for (int j = 0; j <= 3; j++)
                    {
                        GeoPoint2D uv = new GeoPoint2D(bounds.Left + bounds.Width * i / 3.0,
                                                       bounds.Bottom + bounds.Height * j / 3.0);
                        GeoPoint p = surface.PointAt(uv);
                        double t = ParameterOf(p);
                        if (Math.Abs((p | Centre(t)) - Math.Abs(Radius(t))) > tol) return false;
                    }
                }
                return true;
            }

            /// <summary>The spine parameter whose sphere carries the given point.</summary>
            private double ParameterOf(GeoPoint p)
            {
                switch (kind)
                {
                    case SpineKind.Point:
                        return 0.0;
                    case SpineKind.Line:
                        {
                            // for a cone the sphere touching at p is not the one at the foot of the
                            // perpendicular: |p - c(t)| = |r(t)| solved for t along the axis
                            double t = (p - origin) * dir;
                            if (radiusSlope == 0.0) return t;
                            double d2 = (p - origin) * (p - origin) - t * t; // squared distance from the axis
                            // |p-c(t)|^2 = (t-tp)^2 + d2 = (radius0 + radiusSlope*t)^2, solved for t
                            double a = 1.0 - radiusSlope * radiusSlope;
                            double b = -2.0 * t - 2.0 * radius0 * radiusSlope;
                            double c = t * t + d2 - radius0 * radius0;
                            if (Math.Abs(a) < 1e-12) return t;
                            // For a point ON the cone the touching sphere is unique, so this discriminant
                            // is identically zero and round-off pushes it to either side of it. Clamping is
                            // therefore the correct reading; bailing out here would silently return the
                            // projection onto the axis, which is a different point of the spine.
                            double disc = Math.Max(0.0, b * b - 4.0 * a * c);
                            disc = Math.Sqrt(disc);
                            double t1 = (-b + disc) / (2.0 * a), t2 = (-b - disc) / (2.0 * a);
                            // take the root on the same nappe as p
                            return Math.Abs(t1 - t) < Math.Abs(t2 - t) ? t1 : t2;
                        }
                    default:
                        {
                            GeoVector v = p - origin;
                            return Math.Atan2(v * ydir, v * xdir);
                        }
                }
            }
        }

        #endregion

        #region solving for the contact points

        /// <summary>
        /// The branch where the two spheres COINCIDE, i.e. c1(t1) = c2(t2) and r1 = sigma*r2. The common
        /// normal is then not determined by the line of centres and the two envelope conditions decide it
        /// alone: n*c1' = -r1', n*c2' = -sigma*r2', |n| = 1. This is the branch that produces the two
        /// crossing points of two equally thick cylinders whose axes meet - there it reduces to n*d1 = 0,
        /// n*d2 = 0, i.e. n = +-(d1 x d2)/|d1 x d2|.
        /// </summary>
        private static void CoincidentSphereContacts(CanalForm canal1, CanalForm canal2, double sigma,
            double precision, List<GeoPoint> points, List<GeoVector> normals)
        {
            List<double[]> minima = GridMinima(canal1, canal2,
                (t1, t2) => (canal1.Centre(t1) | canal2.Centre(t2)));
            foreach (double[] tt in minima)
            {
                double t1 = tt[0], t2 = tt[1];
                if (!Polish(canal1, canal2, ref t1, ref t2,
                    (a, b, r) =>
                    {
                        GeoVector d = canal2.Centre(b) - canal1.Centre(a);
                        r[0] = d.x; r[1] = d.y; r[2] = d.z;
                    })) continue;
                if ((canal1.Centre(t1) | canal2.Centre(t2)) > precision) continue;
                if (Math.Abs(canal1.Radius(t1) - sigma * canal2.Radius(t2)) > precision) continue;

                GeoVector a = canal1.SpineTangent(t1), b = canal2.SpineTangent(t2);
                double alpha = -canal1.RadiusSlope(t1), beta = -sigma * canal2.RadiusSlope(t2);
                foreach (GeoVector n in UnitVectorsWithGivenProjections(a, alpha, b, beta))
                {
                    points.Add(canal1.Centre(t1) + canal1.Radius(t1) * n);
                    normals.Add(n);
                }
            }
        }

        /// <summary>
        /// The regular branch: the two spheres touch but do not coincide, so the common normal is the
        /// direction of the line of centres. Three residuals in the two spine parameters - the spheres
        /// touching, and the two envelope conditions - overdetermined by one, as tangency must be.
        /// </summary>
        private static void RegularContacts(CanalForm canal1, CanalForm canal2, double sigma,
            double precision, List<GeoPoint> points, List<GeoVector> normals)
        {
            Action<double, double, double[]> residual = (t1, t2, r) =>
            {
                GeoPoint c1 = canal1.Centre(t1), c2 = canal2.Centre(t2);
                double r1 = canal1.Radius(t1), r2 = sigma * canal2.Radius(t2);
                GeoVector d = c2 - c1;
                double dl = d.Length, k = r1 - r2;
                r[0] = dl - Math.Abs(k);
                if (dl < 1e-13)
                {   // the coincident branch is handled elsewhere; keep the residual finite here
                    r[1] = r[2] = 0.0;
                    return;
                }
                GeoVector n = (k < 0.0 ? -1.0 / dl : 1.0 / dl) * d;
                r[1] = n * canal1.SpineTangent(t1) + canal1.RadiusSlope(t1);
                r[2] = n * canal2.SpineTangent(t2) + sigma * canal2.RadiusSlope(t2);
            };

            double[] res = new double[3];
            List<double[]> minima = GridMinima(canal1, canal2, (t1, t2) =>
            {
                residual(t1, t2, res);
                return Math.Sqrt(res[0] * res[0] + res[1] * res[1] + res[2] * res[2]);
            });
            foreach (double[] tt in minima)
            {
                double t1 = tt[0], t2 = tt[1];
                if (!Polish(canal1, canal2, ref t1, ref t2, (a, b, r) => residual(a, b, r))) continue;
                residual(t1, t2, res);
                if (Math.Abs(res[0]) > precision) continue;
                // the envelope residuals are dimensionless, an angle in effect
                double angleTol = Math.Max(1e-4, precision);
                if (Math.Abs(res[1]) > angleTol || Math.Abs(res[2]) > angleTol) continue;

                GeoPoint c1 = canal1.Centre(t1), c2 = canal2.Centre(t2);
                double rad1 = canal1.Radius(t1), rad2 = sigma * canal2.Radius(t2);
                GeoVector dd = c2 - c1;
                if (dd.Length < 1e-13) continue; // that is the coincident branch
                double kk = rad1 - rad2;
                GeoVector nn = (kk < 0.0 ? -1.0 : 1.0) * dd.Normalized;
                points.Add(c1 + rad1 * nn);
                normals.Add(nn);
            }
        }

        /// <summary>
        /// A plane against a canal surface. The plane has one normal N everywhere, so with n = eps*N
        /// (eps = +-1) the two conditions of a contact are
        /// <para>
        /// (c(t) - Q)*N = -eps*r(t)          the sphere of the family touches the plane
        /// </para><para>
        /// eps*(N*c'(t)) = -r'(t)            P lies on the characteristic circle
        /// </para>
        /// two equations in the single spine parameter t - again overdetermined by one, as tangency must
        /// be. What each surface does with it is worth knowing:
        /// <list type="bullet">
        /// <item>sphere: no t at all, the first equation says distance = radius, one point</item>
        /// <item>cylinder: r' = 0, so the second equation says the axis is parallel to the plane and does
        /// not involve t - it holds for every t or for none, which is why a plane touches a cylinder along
        /// a whole ruling and never at a point</item>
        /// <item>cone: the second equation fixes the angle between N and the axis, and the first then
        /// reduces to "the plane passes through the apex" - again a whole ruling</item>
        /// <item>torus: the second equation gives tan(t) = (N*x)/(N*y), so up to TWO solutions half a turn
        /// apart. That is the bitangent plane of a torus, which touches it at two points and cuts it in the
        /// two Villarceau circles crossing there - the only pairing of a plane with one of these surfaces
        /// that produces a node</item>
        /// </list>
        /// </summary>
        private static void PlaneContacts(PlaneSurface plane, CanalForm canal, double precision,
            List<GeoPoint> points, List<GeoVector> normals)
        {
            GeoVector normal = plane.Normal.Normalized;
            GeoPoint origin = plane.Location;
            for (int s = 0; s < 2; s++)
            {
                double eps = s == 0 ? 1.0 : -1.0;
                Action<double, double[]> residual = (t, r) =>
                {
                    r[0] = (canal.Centre(t) - origin) * normal + eps * canal.Radius(t);
                    GeoVector tangent = canal.SpineTangent(t);
                    r[1] = tangent.IsNullVector() ? canal.RadiusSlope(t)
                                                  : eps * (normal * tangent) + canal.RadiusSlope(t);
                };
                double[] r2 = new double[2];
                foreach (double start in Minima1D(canal, t =>
                {
                    residual(t, r2);
                    return Math.Sqrt(r2[0] * r2[0] + r2[1] * r2[1]);
                }))
                {
                    double t = start;
                    Polish1D(canal, ref t, residual);
                    residual(t, r2);
                    if (Math.Abs(r2[0]) > precision) continue;
                    if (Math.Abs(r2[1]) > Math.Max(1e-4, precision)) continue;
                    GeoVector n = eps * normal;
                    points.Add(canal.Centre(t) + canal.Radius(t) * n);
                    normals.Add(n);
                    if (points.Count > 4 * maxContacts) return;
                }
            }
        }

        /// <summary>
        /// Two planes touch only when they are the same plane, and then they touch everywhere. One
        /// representative point is reported, which comes out as <see cref="ContactType.Degenerate"/>.
        /// </summary>
        private static void CoincidentPlanes(PlaneSurface plane1, BoundingRect bounds1, PlaneSurface plane2,
            double precision, List<GeoPoint> points, List<GeoVector> normals)
        {
            GeoVector n1 = plane1.Normal.Normalized, n2 = plane2.Normal.Normalized;
            if ((n1 ^ n2).Length > 1e-6) return; // not parallel: they cross transversally
            GeoPoint p = plane1.PointAt(bounds1.GetCenter());
            if (Math.Abs((p - plane2.Location) * n2) > precision) return; // parallel but apart
            points.Add(p);
            normals.Add(n1);
        }

        /// <summary>Local minima of a scalar measure over a single spine parameter.</summary>
        private static List<double> Minima1D(CanalForm canal, Func<double, double> measure)
        {
            List<double> res = new List<double>();
            if (canal.IsPoint) { res.Add(0.0); return res; }
            const int count = 96;
            double[] t = new double[count];
            double[] value = new double[count];
            double step = canal.Periodic ? (canal.TMax - canal.TMin) / count : (canal.TMax - canal.TMin) / (count - 1);
            for (int i = 0; i < count; i++) { t[i] = canal.TMin + i * step; value[i] = measure(t[i]); }
            for (int i = 0; i < count; i++)
            {
                int before = Wrap(i - 1, count, canal.Periodic), after = Wrap(i + 1, count, canal.Periodic);
                bool isMinimum = (before < 0 || value[before] >= value[i]) && (after < 0 || value[after] >= value[i]);
                if (isMinimum) res.Add(t[i]);
            }
            return res;
        }

        /// <summary>Damped Gauss-Newton on two residuals in one unknown, with a numerical derivative.</summary>
        private static void Polish1D(CanalForm canal, ref double t, Action<double, double[]> residual)
        {
            double[] r = new double[2], rp = new double[2], rm = new double[2];
            double h = Math.Max(Math.Abs(canal.TMax - canal.TMin), 1.0) * 1e-6;
            double lambda = 1e-6;
            residual(t, r);
            double err = r[0] * r[0] + r[1] * r[1];
            for (int iteration = 0; iteration < 40; iteration++)
            {
                if (err < 1e-24) break;
                residual(t + h, rp);
                residual(t - h, rm);
                double j0 = (rp[0] - rm[0]) / (2.0 * h), j1 = (rp[1] - rm[1]) / (2.0 * h);
                double a = j0 * j0 + j1 * j1, b = -(j0 * r[0] + j1 * r[1]);
                bool stepTaken = false;
                for (int attempt = 0; attempt < 8; attempt++)
                {
                    double d = a + lambda * Math.Max(a, 1e-12);
                    if (d < 1e-30) { lambda *= 10.0; continue; }
                    double next = t + b / d;
                    residual(next, rp);
                    double newErr = rp[0] * rp[0] + rp[1] * rp[1];
                    if (newErr < err)
                    {
                        t = next; err = newErr; r[0] = rp[0]; r[1] = rp[1];
                        lambda = Math.Max(lambda * 0.3, 1e-9);
                        stepTaken = true;
                        break;
                    }
                    lambda *= 10.0;
                }
                if (!stepTaken) break;
            }
        }

        /// <summary>
        /// All unit vectors n with n*a = alpha and n*b = beta. A null vector for a or b means "no
        /// condition", which is what a sphere's degenerate spine gives.
        /// </summary>
        private static List<GeoVector> UnitVectorsWithGivenProjections(GeoVector a, double alpha, GeoVector b, double beta)
        {
            List<GeoVector> res = new List<GeoVector>();
            bool hasA = !a.IsNullVector(), hasB = !b.IsNullVector();
            if (!hasA && !hasB) return res; // sphere against sphere: every direction, reported by the caller's own means
            if (!hasA) { a = b; alpha = beta; hasB = false; }
            if (!hasB)
            {   // one condition only: n on a circle of directions. Take a few representatives - a contact
                // along a whole circle is reported as several Degenerate samples, see the remarks on
                // TangentialContacts.
                if (Math.Abs(alpha) > 1.0) return res;
                GeoVector e1 = a.Normalized;
                GeoVector e2 = ArbitraryPerpendicular(e1);
                GeoVector e3 = (e1 ^ e2).Normalized;
                double s = Math.Sqrt(Math.Max(0.0, 1.0 - alpha * alpha));
                for (int i = 0; i < 8; i++)
                {
                    double phi = i * Math.PI / 4.0;
                    res.Add(alpha * e1 + (s * Math.Cos(phi)) * e2 + (s * Math.Sin(phi)) * e3);
                }
                return res;
            }
            GeoVector cross = a ^ b;
            if (cross.Length < 1e-9) return res; // parallel spines: not an isolated direction
            // particular solution inside span{a,b}, then move along a x b to reach unit length
            double aa = a * a, ab = a * b, bb = b * b;
            double det = aa * bb - ab * ab;
            if (Math.Abs(det) < 1e-13) return res;
            double p = (alpha * bb - beta * ab) / det;
            double q = (beta * aa - alpha * ab) / det;
            GeoVector particular = p * a + q * b;
            double rest = 1.0 - particular * particular;
            if (rest < -1e-9) return res;
            double t = Math.Sqrt(Math.Max(0.0, rest));
            GeoVector k = cross.Normalized;
            res.Add(particular + t * k);
            if (t > 1e-9) res.Add(particular - t * k);
            return res;
        }

        private static GeoVector ArbitraryPerpendicular(GeoVector v)
        {
            GeoVector any = Math.Abs(v.x) < 0.9 ? GeoVector.XAxis : GeoVector.YAxis;
            return (v ^ any).Normalized;
        }

        /// <summary>Coarse sampling of the two spine parameters, collecting the local minima of a scalar
        /// measure as starting points for the polish step.</summary>
        private static List<double[]> GridMinima(CanalForm canal1, CanalForm canal2, Func<double, double, double> measure)
        {
            int n1 = canal1.IsPoint ? 1 : 48;
            int n2 = canal2.IsPoint ? 1 : 48;
            double[] t1 = SampleParameters(canal1, n1);
            double[] t2 = SampleParameters(canal2, n2);
            double[,] value = new double[t1.Length, t2.Length];
            for (int i = 0; i < t1.Length; i++)
                for (int j = 0; j < t2.Length; j++)
                    value[i, j] = measure(t1[i], t2[j]);

            List<double[]> res = new List<double[]>();
            for (int i = 0; i < t1.Length; i++)
            {
                for (int j = 0; j < t2.Length; j++)
                {
                    bool isMinimum = true;
                    for (int di = -1; di <= 1 && isMinimum; di++)
                    {
                        for (int dj = -1; dj <= 1; dj++)
                        {
                            if (di == 0 && dj == 0) continue;
                            int ii = Wrap(i + di, t1.Length, canal1.Periodic);
                            int jj = Wrap(j + dj, t2.Length, canal2.Periodic);
                            if (ii < 0 || jj < 0) continue;
                            if (value[ii, jj] < value[i, j]) { isMinimum = false; break; }
                        }
                    }
                    if (isMinimum) res.Add(new double[] { t1[i], t2[j] });
                }
            }
            return res;
        }

        private static double[] SampleParameters(CanalForm canal, int count)
        {
            if (count <= 1) return new double[] { 0.0 };
            double[] res = new double[count];
            // a periodic spine is sampled without repeating the wrap-around point
            double step = canal.Periodic ? (canal.TMax - canal.TMin) / count : (canal.TMax - canal.TMin) / (count - 1);
            for (int i = 0; i < count; i++) res[i] = canal.TMin + i * step;
            return res;
        }

        private static int Wrap(int i, int count, bool periodic)
        {
            if (i >= 0 && i < count) return i;
            if (!periodic) return -1;
            return (i % count + count) % count;
        }

        /// <summary>
        /// Damped Gauss-Newton on three residuals in the two spine parameters, with a numerical Jacobian.
        /// The system is overdetermined, so this converges to the least squares minimum; whether that
        /// minimum is actually zero is decided by the caller.
        /// </summary>
        private static bool Polish(CanalForm canal1, CanalForm canal2, ref double t1, ref double t2,
            Action<double, double, double[]> residual)
        {
            double[] r = new double[3], rp = new double[3], rm = new double[3];
            double h1 = Math.Max(Math.Abs(canal1.TMax - canal1.TMin), 1.0) * 1e-6;
            double h2 = Math.Max(Math.Abs(canal2.TMax - canal2.TMin), 1.0) * 1e-6;
            double lambda = 1e-6;
            residual(t1, t2, r);
            double err = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
            for (int iteration = 0; iteration < 40; iteration++)
            {
                if (err < 1e-24) break;
                double[] j1 = new double[3], j2 = new double[3];
                residual(t1 + h1, t2, rp); residual(t1 - h1, t2, rm);
                for (int k = 0; k < 3; k++) j1[k] = (rp[k] - rm[k]) / (2.0 * h1);
                residual(t1, t2 + h2, rp); residual(t1, t2 - h2, rm);
                for (int k = 0; k < 3; k++) j2[k] = (rp[k] - rm[k]) / (2.0 * h2);

                // normal equations of the 3x2 system, with Levenberg damping
                double a11 = 0.0, a12 = 0.0, a22 = 0.0, b1 = 0.0, b2 = 0.0;
                for (int k = 0; k < 3; k++)
                {
                    a11 += j1[k] * j1[k]; a12 += j1[k] * j2[k]; a22 += j2[k] * j2[k];
                    b1 -= j1[k] * r[k]; b2 -= j2[k] * r[k];
                }
                bool stepTaken = false;
                // additive damping, not multiplicative: a spine that is a single point contributes a zero
                // column to the Jacobian, and scaling zero by (1+lambda) leaves the system singular forever
                double diagonal = Math.Max(Math.Max(a11, a22), 1e-12);
                for (int attempt = 0; attempt < 8; attempt++)
                {
                    double d11 = a11 + lambda * diagonal, d22 = a22 + lambda * diagonal;
                    double det = d11 * d22 - a12 * a12;
                    if (Math.Abs(det) < 1e-30) { lambda *= 10.0; continue; }
                    double s1 = (b1 * d22 - b2 * a12) / det;
                    double s2 = (b2 * d11 - b1 * a12) / det;
                    double n1 = t1 + s1, n2 = t2 + s2;
                    residual(n1, n2, rp);
                    double newErr = rp[0] * rp[0] + rp[1] * rp[1] + rp[2] * rp[2];
                    if (newErr < err)
                    {
                        t1 = n1; t2 = n2; err = newErr;
                        for (int k = 0; k < 3; k++) r[k] = rp[k];
                        lambda = Math.Max(lambda * 0.3, 1e-9);
                        stepTaken = true;
                        break;
                    }
                    lambda *= 10.0;
                }
                if (!stepTaken) break;
            }
            return !double.IsNaN(t1) && !double.IsNaN(t2);
        }

        #endregion

        #region verification and classification

        /// <summary>
        /// Turns a candidate point into a <see cref="SurfaceContact"/>: checks that it really lies on both
        /// surfaces inside their domains, measures how exact the contact is, and classifies it by the
        /// difference of the second fundamental forms.
        /// </summary>
        private static SurfaceContact Verify(ISurface surface1, BoundingRect bounds1,
            ISurface surface2, BoundingRect bounds2, GeoPoint candidate, GeoVector normal, double precision)
        {
            GeoPoint2D uv1 = surface1.PositionOf(candidate);
            GeoPoint2D uv2 = surface2.PositionOf(candidate);
            SurfaceHelper.AdjustPeriodic(surface1, bounds1, ref uv1);
            SurfaceHelper.AdjustPeriodic(surface2, bounds2, ref uv2);
            if (!bounds1.ContainsEps(uv1, -1e-6) || !bounds2.ContainsEps(uv2, -1e-6)) return null;

            GeoPoint p1 = surface1.PointAt(uv1), p2 = surface2.PointAt(uv2);
            double distance = Math.Max(p1 | candidate, p2 | candidate);
            if (distance > precision * 10.0) return null;

            GeoVector n1 = surface1.GetNormal(uv1), n2 = surface2.GetNormal(uv2);
            if (n1.IsNullVector() || n2.IsNullVector()) return null;
            n1.Norm();
            n2.Norm();
            double angle = Math.Abs(n1 * n2);
            if (angle > 1.0) angle = 1.0;
            angle = Math.Acos(angle); // 0 for parallel, regardless of orientation

            SurfaceContact res = new SurfaceContact
            {
                Location = new GeoPoint(p1, p2),
                uv1 = uv1,
                uv2 = uv2,
                Normal = n1,
                DistanceDefect = p1 | p2,
                AngleDefect = angle
            };
            Classify(surface1, uv1, surface2, uv2, n1, res);
            return res;
        }

        /// <summary>
        /// Decides what the intersection curve does at a contact point, from D = II1 - II2, the difference
        /// of the second fundamental forms taken in a common orthonormal basis of the tangent plane and
        /// with the SAME normal orientation. D indefinite means the two branches of the intersection curve
        /// cross here, and the null directions of D are their tangents; D definite means the surfaces touch
        /// without crossing; D singular means the contact is of higher order.
        /// </summary>
        private static void Classify(ISurface surface1, GeoPoint2D uv1, ISurface surface2, GeoPoint2D uv2,
            GeoVector normal, SurfaceContact contact)
        {
            contact.Type = ContactType.Degenerate;
            GeoVector e1 = ArbitraryPerpendicular(normal);
            GeoVector e2 = (normal ^ e1).Normalized;

            if (!SecondFundamentalForm(surface1, uv1, normal, e1, e2, out double l1, out double m1, out double n1)) return;
            if (!SecondFundamentalForm(surface2, uv2, normal, e1, e2, out double l2, out double m2, out double n2)) return;

            double d11 = l1 - l2, d12 = m1 - m2, d22 = n1 - n2;
            double scale = Math.Max(Math.Abs(d11), Math.Max(Math.Abs(d12), Math.Abs(d22)));
            if (scale < 1e-12) return; // the surfaces osculate: contact of higher order
            double det = d11 * d22 - d12 * d12;
            double tol = 1e-6 * scale * scale;
            if (det > tol) { contact.Type = ContactType.Isolated; return; }
            if (det >= -tol) return;                       // Degenerate

            contact.Type = ContactType.Crossing;
            // D(w,w) = 0 for w = cos(phi)*e1 + sin(phi)*e2, i.e. d22*L^2 + 2*d12*L + d11 = 0 with L = tan(phi)
            List<GeoVector> branches = new List<GeoVector>();
            if (Math.Abs(d22) < 1e-12 * scale)
            {
                branches.Add(e2); // phi = 90 degrees is a root
                if (Math.Abs(d12) > 1e-12 * scale)
                {
                    double lambda = -d11 / (2.0 * d12);
                    branches.Add((e1 + lambda * e2).Normalized);
                }
            }
            else
            {
                double disc = d12 * d12 - d11 * d22;
                if (disc < 0.0) disc = 0.0;
                disc = Math.Sqrt(disc);
                branches.Add((e1 + ((-d12 + disc) / d22) * e2).Normalized);
                branches.Add((e1 + ((-d12 - disc) / d22) * e2).Normalized);
            }
            contact.BranchDirections = branches.ToArray();
        }

        /// <summary>
        /// The second fundamental form of the surface at uv, expressed in the orthonormal tangent basis
        /// (e1,e2) and with respect to the given normal orientation, so that the forms of two different
        /// surfaces can be subtracted.
        /// </summary>
        private static bool SecondFundamentalForm(ISurface surface, GeoPoint2D uv, GeoVector normal,
            GeoVector e1, GeoVector e2, out double l, out double m, out double n)
        {
            l = m = n = 0.0;
            surface.Derivative2At(uv, out GeoPoint location, out GeoVector du, out GeoVector dv,
                out GeoVector duu, out GeoVector dvv, out GeoVector duv);
            double scale = Math.Max(du.Length, dv.Length);
            // express e1 and e2 in the (du,dv) basis of the tangent plane
            double guu = du * du, guv = du * dv, gvv = dv * dv;
            double det = guu * gvv - guv * guv;
            if (du.IsNullVector() || dv.IsNullVector() || Math.Abs(det) < 1e-12 * scale * scale * scale * scale)
            {   // The PARAMETRIZATION degenerates here - a pole of a sphere, the apex of a cone - while the
                // surface itself is perfectly smooth. A plane resting on a sphere touches it at exactly such
                // a point, so this is not an exotic case and must not end up as "cannot classify".
                return NormalCurvatures(surface, uv, normal, e1, e2, Math.Max(scale, 1e-6) * 1e-3,
                    out l, out m, out n);
            }
            double L = duu * normal, M = duv * normal, N = dvv * normal;
            Coefficients(e1, du, dv, guu, guv, gvv, det, out double p1, out double q1);
            Coefficients(e2, du, dv, guu, guv, gvv, det, out double p2, out double q2);
            l = p1 * p1 * L + 2.0 * p1 * q1 * M + q1 * q1 * N;
            m = p1 * p2 * L + (p1 * q2 + p2 * q1) * M + q1 * q2 * N;
            n = p2 * p2 * L + 2.0 * p2 * q2 * M + q2 * q2 * N;
            return true;
        }

        /// <summary>
        /// The second fundamental form measured on the surface instead of read off its derivatives: the
        /// normal curvature in a direction w is 2*d/h^2, where d is how far the surface has moved away from
        /// the tangent plane a step h along w. Needs no parameter basis and therefore still works where the
        /// parametrization degenerates. Only the sign of the determinant of the difference of the two forms
        /// is used downstream, which this is well able to decide.
        /// </summary>
        private static bool NormalCurvatures(ISurface surface, GeoPoint2D uv, GeoVector normal,
            GeoVector e1, GeoVector e2, double step, out double l, out double m, out double n)
        {
            l = m = n = 0.0;
            GeoPoint p = surface.PointAt(uv);
            if (!NormalCurvature(surface, p, normal, e1, step, out l)) return false;
            if (!NormalCurvature(surface, p, normal, e2, step, out n)) return false;
            // II(w,w) = (l + 2m + n)/2 for w the unit bisector of e1 and e2
            if (!NormalCurvature(surface, p, normal, (e1 + e2).Normalized, step, out double diagonal)) return false;
            m = diagonal - (l + n) / 2.0;
            return true;
        }

        private static bool NormalCurvature(ISurface surface, GeoPoint p, GeoVector normal,
            GeoVector direction, double step, out double curvature)
        {
            curvature = 0.0;
            if (step <= 0.0) return false;
            GeoPoint2D uv = surface.PositionOf(p + step * direction);
            GeoPoint q = surface.PointAt(uv);
            if ((q | p) > 10.0 * step) return false; // the step did not stay in the neighbourhood
            curvature = 2.0 * ((q - p) * normal) / (step * step);
            return true;
        }

        private static void Coefficients(GeoVector w, GeoVector du, GeoVector dv,
            double guu, double guv, double gvv, double det, out double p, out double q)
        {
            double wu = w * du, wv = w * dv;
            p = (wu * gvv - wv * guv) / det;
            q = (wv * guu - wu * guv) / det;
        }

        #endregion
    }
}
