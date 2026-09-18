using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for <see cref="Surfaces.TangentialContacts"/>: the points where two surfaces touch, which are
    /// the singular points of their intersection curve. Every case here has an answer that can be written
    /// down, so the tests state what is right rather than what the code happened to produce.
    /// </summary>
    [TestClass]
    public class SurfaceContactTests
    {
        public TestContext TestContext { get; set; }

        private const double precision = 1e-6;

        private static CylindricalSurface Cylinder(GeoPoint location, GeoVector axis, double radius)
        {
            GeoVector dx = radius * ArbitraryPerpendicular(axis);
            GeoVector dy = radius * (axis.Normalized ^ dx.Normalized);
            return new CylindricalSurface(location, dx, dy, axis.Normalized);
        }

        private static SphericalSurface Sphere(GeoPoint centre, double radius)
        {
            return new SphericalSurface(centre, radius * GeoVector.XAxis, radius * GeoVector.YAxis, radius * GeoVector.ZAxis);
        }

        private static GeoVector ArbitraryPerpendicular(GeoVector v)
        {
            GeoVector any = System.Math.Abs(v.Normalized.x) < 0.9 ? GeoVector.XAxis : GeoVector.YAxis;
            return (v ^ any).Normalized;
        }


        private static PlaneSurface PlaneThrough(GeoPoint location, GeoVector normal)
        {
            return new PlaneSurface(new Plane(location, normal));
        }

        private static ToroidalSurface Torus(GeoPoint centre, double major, double minor)
        {
            return new ToroidalSurface(centre, GeoVector.XAxis, GeoVector.YAxis, GeoVector.ZAxis, major, minor);
        }

        private static BoundingRect FullTorus => new BoundingRect(0, 0, 2 * System.Math.PI, 2 * System.Math.PI);

        /// <summary>A domain large enough to hold everything these tests construct.</summary>
        private static BoundingRect Wide => new BoundingRect(-20, -200, 20, 200);

        private void Dump(string what, SurfaceContact[] contacts)
        {
            TestContext.WriteLine($"{what}: {contacts.Length} contact(s)");
            foreach (SurfaceContact c in contacts)
                TestContext.WriteLine($"   {c.Type,-10} at {c.Location} n = {c.Normal} " +
                    $"dist {c.DistanceDefect:E2} angle {c.AngleDefect:E2} branches {c.BranchDirections.Length}");
        }

        /// <summary>
        /// The classic case: two cylinders of equal radius whose axes meet. The two spheres of radius r
        /// centred at the meeting point coincide, so the common normal is decided by the two envelope
        /// conditions alone - n perpendicular to both axes. That gives exactly two contact points,
        /// P = O +- r*(d1 x d2)/|d1 x d2|, and the branches of the intersection curve run along the angle
        /// bisectors of the two axes.
        /// </summary>
        [TestMethod]
        public void equal_cylinders_with_crossing_axes_touch_at_two_nodes()
        {
            const double r = 10.0;
            CylindricalSurface c1 = Cylinder(GeoPoint.Origin, GeoVector.XAxis, r);
            CylindricalSurface c2 = Cylinder(GeoPoint.Origin, GeoVector.YAxis, r);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(c1, Wide, c2, Wide, precision);
            Dump("equal cylinders, perpendicular axes", contacts);

            Assert.AreEqual(2, contacts.Length, "expected exactly the two crossing points");
            foreach (SurfaceContact c in contacts)
            {
                Assert.AreEqual(ContactType.Crossing, c.Type);
                Assert.AreEqual(0.0, c.DistanceDefect, 1e-6);
                Assert.AreEqual(0.0, c.AngleDefect, 1e-6);
                // P = O +- r*(x cross y) = (0,0,+-r)
                Assert.AreEqual(0.0, c.Location.x, 1e-6);
                Assert.AreEqual(0.0, c.Location.y, 1e-6);
                Assert.AreEqual(r, System.Math.Abs(c.Location.z), 1e-6);
                // the branch tangents are the bisectors of the two axes, i.e. (1,+-1,0)/sqrt(2)
                Assert.AreEqual(2, c.BranchDirections.Length, "a node has two crossing branches");
                foreach (GeoVector b in c.BranchDirections)
                {
                    Assert.AreEqual(0.0, b.z, 1e-6, "a branch tangent must lie in the common tangent plane");
                    Assert.AreEqual(System.Math.Abs(b.x), System.Math.Abs(b.y), 1e-6,
                        "the branches run along the bisectors of the two axes");
                }
            }
            Assert.AreNotEqual(System.Math.Sign(contacts[0].Location.z), System.Math.Sign(contacts[1].Location.z),
                "the two points are on opposite sides");
        }

        /// <summary>The same, but with the axes at an oblique angle - the closed form holds for any angle.</summary>
        [TestMethod]
        public void equal_cylinders_with_oblique_axes_touch_at_two_nodes()
        {
            const double r = 7.0;
            GeoVector d1 = GeoVector.XAxis;
            GeoVector d2 = new GeoVector(1, 2, 0).Normalized;
            CylindricalSurface c1 = Cylinder(GeoPoint.Origin, d1, r);
            CylindricalSurface c2 = Cylinder(GeoPoint.Origin, d2, r);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(c1, Wide, c2, Wide, precision);
            Dump("equal cylinders, oblique axes", contacts);

            Assert.AreEqual(2, contacts.Length);
            GeoVector expected = (d1 ^ d2).Normalized;
            foreach (SurfaceContact c in contacts)
            {
                Assert.AreEqual(ContactType.Crossing, c.Type);
                GeoVector fromOrigin = c.Location - GeoPoint.Origin;
                Assert.AreEqual(r, fromOrigin.Length, 1e-6);
                Assert.AreEqual(1.0, System.Math.Abs(fromOrigin.Normalized * expected), 1e-6,
                    "the contact points lie along the common perpendicular of the two axes");
            }
        }

        /// <summary>
        /// The negative case that is easy to forget: cylinders of DIFFERENT radius whose axes meet are
        /// nowhere tangent, their intersection curve is regular everywhere, and the method has to say so.
        /// </summary>
        [TestMethod]
        public void cylinders_of_different_radius_have_no_contact()
        {
            CylindricalSurface c1 = Cylinder(GeoPoint.Origin, GeoVector.XAxis, 10.0);
            CylindricalSurface c2 = Cylinder(GeoPoint.Origin, GeoVector.YAxis, 6.0);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(c1, Wide, c2, Wide, precision);
            Dump("cylinders of different radius", contacts);
            Assert.AreEqual(0, contacts.Length, "different radii cannot touch tangentially here");
        }

        /// <summary>
        /// The configuration of the RPC case ConesTangentAtMantlePoint: a cone and the same cone rotated by
        /// 180 degrees about the surface normal at P = (7.5,0,6), a point of the ruling. The rotation fixes
        /// P and fixes the tangent plane there, so the two cones touch at P with common normal (24,0,10),
        /// and the intersection curve has a node.
        /// </summary>
        [TestMethod]
        public void cones_tangent_at_a_mantle_point_are_found()
        {
            // cone: apex (0,0,24), base radius 10 at z = 0, so tan(semiangle) = 10/24
            double semi = System.Math.Atan2(10.0, 24.0);
            ConicalSurface cone1 = new ConicalSurface(new GeoPoint(0, 0, 24),
                GeoVector.XAxis, GeoVector.YAxis, -GeoVector.ZAxis, semi);

            GeoPoint p = new GeoPoint(7.5, 0, 6);
            GeoVector n = new GeoVector(24, 0, 10).Normalized;
            ModOp turn = ModOp.Rotate(p, n, SweepAngle.Deg(180));
            ConicalSurface cone2 = cone1.Clone() as ConicalSurface;
            cone2.Modify(turn);

            BoundingRect bounds = new BoundingRect(-System.Math.PI, 0, System.Math.PI, 30);
            SurfaceContact[] contacts = Surfaces.TangentialContacts(cone1, bounds, cone2, bounds, precision);
            Dump("two cones tangent at a mantle point", contacts);

            SurfaceContact found = null;
            foreach (SurfaceContact c in contacts)
            {
                if ((c.Location | p) < 1e-4) { found = c; break; }
            }
            Assert.IsNotNull(found, $"the contact at {p} was not found");
            Assert.AreEqual(ContactType.Crossing, found.Type, "the intersection curve has a node there");
            Assert.AreEqual(1.0, System.Math.Abs(found.Normal.Normalized * n), 1e-5,
                "the common normal must be the surface normal (24,0,10)");
            Assert.AreEqual(2, found.BranchDirections.Length);
            foreach (GeoVector b in found.BranchDirections)
                Assert.AreEqual(0.0, b.Normalized * n, 1e-5, "a branch tangent lies in the common tangent plane");
        }

        /// <summary>Two spheres touching from outside: one point, and the surfaces do not cross there.</summary>
        [TestMethod]
        public void touching_spheres_give_one_isolated_contact()
        {
            SphericalSurface s1 = Sphere(GeoPoint.Origin, 5.0);
            SphericalSurface s2 = Sphere(new GeoPoint(8.0, 0, 0), 3.0);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(s1, Wide, s2, Wide, precision);
            Dump("two spheres touching externally", contacts);

            Assert.AreEqual(1, contacts.Length);
            Assert.AreEqual(ContactType.Isolated, contacts[0].Type, "spheres touching from outside do not cross");
            Assert.AreEqual(0.0, (contacts[0].Location | new GeoPoint(5, 0, 0)), 1e-6);
            Assert.AreEqual(0, contacts[0].BranchDirections.Length);
        }

        /// <summary>
        /// A sphere inside a cylinder of the same radius touches along a whole circle. That is not a node
        /// and not an isolated point - the method must report Degenerate rather than invent branches.
        /// </summary>
        [TestMethod]
        public void sphere_in_a_cylinder_of_equal_radius_is_degenerate()
        {
            const double r = 5.0;
            CylindricalSurface cyl = Cylinder(GeoPoint.Origin, GeoVector.ZAxis, r);
            SphericalSurface sph = Sphere(GeoPoint.Origin, r);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(cyl, Wide, sph, Wide, precision);
            Dump("sphere inside an equally wide cylinder", contacts);

            Assert.IsTrue(contacts.Length > 0, "the contact circle must be reported, not swallowed");
            foreach (SurfaceContact c in contacts)
            {
                Assert.AreEqual(ContactType.Degenerate, c.Type, "contact along a curve is not a node");
                Assert.AreEqual(0, c.BranchDirections.Length, "no branch directions for a degenerate contact");
                // every reported point is on the contact circle x^2+y^2 = r^2, z = 0
                Assert.AreEqual(r, System.Math.Sqrt(c.Location.x * c.Location.x + c.Location.y * c.Location.y), 1e-5);
                Assert.AreEqual(0.0, c.Location.z, 1e-5);
            }
        }

        /// <summary>A plane touches a sphere in a single point, and the sphere stays on one side.</summary>
        [TestMethod]
        public void plane_and_sphere_touch_at_one_isolated_point()
        {
            SphericalSurface sph = Sphere(new GeoPoint(0, 0, 5), 5.0);
            PlaneSurface pln = PlaneThrough(GeoPoint.Origin, GeoVector.ZAxis);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(pln, Wide, sph, Wide, precision);
            Dump("plane and sphere", contacts);

            Assert.AreEqual(1, contacts.Length);
            Assert.AreEqual(ContactType.Isolated, contacts[0].Type);
            Assert.AreEqual(0.0, (contacts[0].Location | GeoPoint.Origin), 1e-6);
        }

        /// <summary>A plane that does not reach the sphere has no contact at all.</summary>
        [TestMethod]
        public void plane_missing_the_sphere_has_no_contact()
        {
            SphericalSurface sph = Sphere(new GeoPoint(0, 0, 5), 4.0);
            PlaneSurface pln = PlaneThrough(GeoPoint.Origin, GeoVector.ZAxis);
            SurfaceContact[] contacts = Surfaces.TangentialContacts(pln, Wide, sph, Wide, precision);
            Dump("plane one unit away from the sphere", contacts);
            Assert.AreEqual(0, contacts.Length);
        }

        /// <summary>
        /// A plane can never touch a cylinder in a point: a constant radius makes the envelope condition
        /// independent of the spine parameter, so it holds for the whole axis or nowhere. The contact is a
        /// ruling, reported as several Degenerate samples along it.
        /// </summary>
        [TestMethod]
        public void plane_and_cylinder_touch_along_a_ruling()
        {
            const double r = 4.0;
            CylindricalSurface cyl = Cylinder(new GeoPoint(0, 0, r), GeoVector.XAxis, r);
            PlaneSurface pln = PlaneThrough(GeoPoint.Origin, GeoVector.ZAxis);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(pln, Wide, cyl, Wide, precision);
            Dump("plane tangent to a cylinder", contacts);

            Assert.IsTrue(contacts.Length > 1, "the contact is a whole ruling, not a point");
            foreach (SurfaceContact c in contacts)
            {
                Assert.AreEqual(ContactType.Degenerate, c.Type);
                Assert.AreEqual(0.0, c.Location.z, 1e-5, "every contact point is on the plane z = 0");
                Assert.AreEqual(0.0, c.Location.y, 1e-5, "and on the ruling y = 0");
            }
        }

        /// <summary>
        /// The bitangent plane of a torus: it passes through the centre, touches at TWO points, and cuts
        /// the torus in the two Villarceau circles, which cross exactly there. Among the natural quadrics
        /// this is the only pairing with a plane that produces a node.
        /// For R = 10, r = 3 the plane normal is (-r/R, 0, sqrt(1-(r/R)^2)) and the two contacts are at
        /// (+-9.1, 0, +-2.8618176043).
        /// </summary>
        [TestMethod]
        public void bitangent_plane_of_a_torus_gives_two_nodes()
        {
            const double R = 10.0, r = 3.0;
            ToroidalSurface tor = Torus(GeoPoint.Origin, R, r);
            GeoVector n = new GeoVector(-r / R, 0, System.Math.Sqrt(1.0 - (r / R) * (r / R)));
            PlaneSurface pln = PlaneThrough(GeoPoint.Origin, n);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(pln, Wide, tor, FullTorus, precision);
            Dump("bitangent (Villarceau) plane of a torus", contacts);

            Assert.AreEqual(2, contacts.Length, "the bitangent plane touches at exactly two points");
            foreach (SurfaceContact c in contacts)
            {
                Assert.AreEqual(ContactType.Crossing, c.Type, "the two Villarceau circles cross here");
                Assert.AreEqual(2, c.BranchDirections.Length);
                Assert.AreEqual(9.1, System.Math.Abs(c.Location.x), 1e-5);
                Assert.AreEqual(2.8618176043, System.Math.Abs(c.Location.z), 1e-5);
                Assert.AreEqual(0.0, c.Location.y, 1e-5);
            }
        }

        /// <summary>
        /// A plane tangent at the INNER equator of a torus. That point has negative Gaussian curvature, so
        /// the plane crosses the surface there - one node - while the same plane on the outside touches an
        /// elliptic point and does not cross.
        /// </summary>
        [TestMethod]
        public void plane_at_the_inner_equator_of_a_torus_is_a_node_the_outer_one_is_not()
        {
            const double R = 10.0, r = 3.0;
            ToroidalSurface tor = Torus(GeoPoint.Origin, R, r);

            SurfaceContact[] inner = Surfaces.TangentialContacts(
                PlaneThrough(new GeoPoint(R - r, 0, 0), GeoVector.XAxis), Wide, tor, FullTorus, precision);
            Dump("plane at the inner equator", inner);
            Assert.AreEqual(1, inner.Length);
            Assert.AreEqual(ContactType.Crossing, inner[0].Type, "the inner equator is a saddle point");
            Assert.AreEqual(0.0, (inner[0].Location | new GeoPoint(R - r, 0, 0)), 1e-5);

            SurfaceContact[] outer = Surfaces.TangentialContacts(
                PlaneThrough(new GeoPoint(R + r, 0, 0), GeoVector.XAxis), Wide, tor, FullTorus, precision);
            Dump("plane at the outer equator", outer);
            Assert.AreEqual(1, outer.Length);
            Assert.AreEqual(ContactType.Isolated, outer[0].Type, "the outer equator is an elliptic point");
            Assert.AreEqual(0.0, (outer[0].Location | new GeoPoint(R + r, 0, 0)), 1e-5);
        }

        /// <summary>A plane perpendicular to the torus axis touches along a whole circle.</summary>
        [TestMethod]
        public void plane_perpendicular_to_the_torus_axis_touches_along_a_circle()
        {
            const double R = 10.0, r = 3.0;
            ToroidalSurface tor = Torus(GeoPoint.Origin, R, r);
            PlaneSurface pln = PlaneThrough(new GeoPoint(0, 0, r), GeoVector.ZAxis);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(pln, Wide, tor, FullTorus, precision);
            Dump("plane perpendicular to the torus axis", contacts);

            Assert.IsTrue(contacts.Length > 1, "the contact is the whole top circle");
            foreach (SurfaceContact c in contacts)
            {
                Assert.AreEqual(ContactType.Degenerate, c.Type);
                Assert.AreEqual(r, c.Location.z, 1e-5);
                Assert.AreEqual(R, System.Math.Sqrt(c.Location.x * c.Location.x + c.Location.y * c.Location.y), 1e-5);
            }
        }

        /// <summary>
        /// <see cref="Surfaces.ContactAt"/> on a point that is known from elsewhere: the top point of two
        /// equal cylinders with crossing axes. It has to find the same node and the same two branch
        /// directions - the angle bisectors of the axes - as the search does.
        /// </summary>
        [TestMethod]
        public void contact_at_a_known_point_finds_the_crossing_branches()
        {
            const double r = 10.0;
            CylindricalSurface c1 = Cylinder(GeoPoint.Origin, GeoVector.XAxis, r);
            CylindricalSurface c2 = Cylinder(GeoPoint.Origin, GeoVector.YAxis, r);
            GeoPoint known = new GeoPoint(0, 0, r); // the touching point, found in some other way

            SurfaceContact contact = Surfaces.ContactAt(c1, c2, known, precision);
            Assert.IsNotNull(contact);
            Dump("known point on two equal cylinders", new SurfaceContact[] { contact });

            Assert.AreEqual(ContactType.Crossing, contact.Type);
            Assert.AreEqual(0.0, (contact.Location | known), 1e-6);
            Assert.AreEqual(0.0, contact.DistanceDefect, 1e-6);
            Assert.AreEqual(0.0, contact.AngleDefect, 1e-6);
            Assert.AreEqual(0.0, System.Math.Abs(contact.Normal * GeoVector.ZAxis) - 1.0, 1e-6);
            Assert.AreEqual(2, contact.BranchDirections.Length);
            GeoVector b1 = new GeoVector(1, 1, 0).Normalized, b2 = new GeoVector(1, -1, 0).Normalized;
            foreach (GeoVector b in contact.BranchDirections)
            {
                double along1 = System.Math.Abs(b.Normalized * b1), along2 = System.Math.Abs(b.Normalized * b2);
                Assert.AreEqual(1.0, System.Math.Max(along1, along2), 1e-5, "branch along an angle bisector");
            }
            Assert.AreEqual(0.0, contact.BranchDirections[0].Normalized * contact.BranchDirections[1].Normalized,
                1e-5, "here the two branches are perpendicular");
        }

        /// <summary>
        /// The classification must not depend on how the two surfaces are oriented: reversing one of them
        /// only flips the common normal, the intersection curve does the same thing as before.
        /// </summary>
        [TestMethod]
        public void contact_at_a_known_point_ignores_the_orientation()
        {
            const double r = 10.0;
            CylindricalSurface c1 = Cylinder(GeoPoint.Origin, GeoVector.XAxis, r);
            CylindricalSurface c2 = Cylinder(GeoPoint.Origin, GeoVector.YAxis, r);
            c2.ReverseOrientation();
            GeoPoint known = new GeoPoint(0, 0, r);

            SurfaceContact contact = Surfaces.ContactAt(c1, c2, known, precision);
            Assert.IsNotNull(contact, "opposite normals are a contact just as parallel ones are");
            Dump("known point, second cylinder reversed", new SurfaceContact[] { contact });
            Assert.AreEqual(ContactType.Crossing, contact.Type);
            Assert.AreEqual(0.0, contact.AngleDefect, 1e-6);
            Assert.AreEqual(2, contact.BranchDirections.Length);
        }

        /// <summary>Two spheres touching from outside: the known point is an isolated contact.</summary>
        [TestMethod]
        public void contact_at_a_known_point_recognizes_an_isolated_touch()
        {
            SphericalSurface s1 = Sphere(GeoPoint.Origin, 5.0);
            SphericalSurface s2 = Sphere(new GeoPoint(8.0, 0, 0), 3.0);
            GeoPoint known = new GeoPoint(5.0, 0, 0);

            SurfaceContact contact = Surfaces.ContactAt(s1, s2, known, precision);
            Assert.IsNotNull(contact);
            Dump("known point on two touching spheres", new SurfaceContact[] { contact });
            Assert.AreEqual(ContactType.Isolated, contact.Type);
            Assert.AreEqual(0, contact.BranchDirections.Length);
        }

        /// <summary>
        /// A point where the two surfaces cross transversally is not a contact: the normals are not
        /// parallel there and the method says so instead of classifying something.
        /// </summary>
        [TestMethod]
        public void a_transversal_point_is_not_a_contact()
        {
            CylindricalSurface c1 = Cylinder(GeoPoint.Origin, GeoVector.XAxis, 10.0);
            CylindricalSurface c2 = Cylinder(GeoPoint.Origin, GeoVector.YAxis, 5.0);
            // both surfaces contain this point, but they cross there
            GeoPoint onBoth = new GeoPoint(0, System.Math.Sqrt(100.0 - 25.0), 5.0);
            Assert.AreEqual(0.0, (c1.PointAt(c1.PositionOf(onBoth)) | onBoth), 1e-6);
            Assert.AreEqual(0.0, (c2.PointAt(c2.PositionOf(onBoth)) | onBoth), 1e-6);

            Assert.IsNull(Surfaces.ContactAt(c1, c2, onBoth, precision));
        }

        /// <summary>A point that is not on the second surface at all is rejected as well.</summary>
        [TestMethod]
        public void a_point_off_the_second_surface_is_not_a_contact()
        {
            CylindricalSurface c1 = Cylinder(GeoPoint.Origin, GeoVector.XAxis, 10.0);
            SphericalSurface s2 = Sphere(new GeoPoint(0, 0, 30.0), 5.0);
            Assert.IsNull(Surfaces.ContactAt(c1, s2, new GeoPoint(0, 0, 10.0), precision));
        }

        #region the general search: the same situations on surfaces which are not natural quadrics

        // The quadrics above have a closed form solution and their own code path. Everything here is built
        // from NURBS instead - an exact circle as a rational quadratic BSpline, carried into a surface by
        // extrusion, ruling, sweeping or revolution - so it is the same geometry with none of the knowledge:
        // a cylinder that is a SurfaceOfLinearExtrusion, a cone that is a RuledSurface, a torus that is a
        // SweptCurveSurface or a SurfaceOfRevolution. The answers therefore stay the ones written down above,
        // which is what makes these tests worth having: they state what is right, not what came out.

        /// <summary>The classical nine pole rational quadratic circle, exact and not an approximation.</summary>
        private static BSpline NurbsCircle(GeoPoint centre, GeoVector dx, GeoVector dy)
        {
            double[,] q = { { 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 }, { -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 }, { 1, 0 } };
            GeoPoint[] poles = new GeoPoint[9];
            double[] weights = new double[9];
            double w = System.Math.Sqrt(2.0) / 2.0;
            for (int i = 0; i < 9; i++)
            {
                poles[i] = centre + q[i, 0] * dx + q[i, 1] * dy;
                weights[i] = (i % 2 == 0) ? 1.0 : w;
            }
            BSpline res = BSpline.Construct();
            res.SetData(2, poles, weights, new double[] { 0, 1, 2, 3, 4 }, new int[] { 3, 2, 2, 2, 3 }, false);
            return res;
        }

        /// <summary>A circle of the given radius in the plane through centre with the given normal.</summary>
        private static BSpline NurbsCircle(GeoPoint centre, GeoVector normal, double radius)
        {
            GeoVector dx = ArbitraryPerpendicular(normal);
            GeoVector dy = (normal.Normalized ^ dx).Normalized;
            return NurbsCircle(centre, radius * dx, radius * dy);
        }

        /// <summary>A cylinder about the given axis, as a SurfaceOfLinearExtrusion of an exact NURBS circle.</summary>
        private static SurfaceOfLinearExtrusion ExtrudedCylinder(GeoPoint location, GeoVector axis, double radius)
        {
            return new SurfaceOfLinearExtrusion(NurbsCircle(location, axis, radius), axis.Normalized, 0.0, 1.0);
        }

        /// <summary>The natural parameter domain of a surface, which is where these tests look for contacts.</summary>
        private static BoundingRect Natural(ISurface surface)
        {
            surface.GetNaturalBounds(out double umin, out double umax, out double vmin, out double vmax);
            return new BoundingRect(umin, vmin, umax, vmax);
        }

        /// <summary>The domain of an extrusion, whose v is unlimited: the circle once, the axis this far.</summary>
        private static BoundingRect ExtrusionDomain(double halfLength)
        {
            return new BoundingRect(0.0, -halfLength, 1.0, halfLength);
        }

        /// <summary>
        /// The same situation as <see cref="equal_cylinders_with_crossing_axes_touch_at_two_nodes"/>, but
        /// neither surface is a CylindricalSurface: both are extrusions of a NURBS circle, so nothing about
        /// them is known in closed form. The answer has to be the same - the two nodes at (+-10, 0, 0), where
        /// the normal of the first cylinder (x, y, 0) and that of the second (x, 0, z) can only be parallel
        /// if y = z = 0.
        /// </summary>
        [TestMethod]
        public void nurbs_cylinders_with_crossing_axes_touch_at_two_nodes()
        {
            const double r = 10.0;
            SurfaceOfLinearExtrusion c1 = ExtrudedCylinder(GeoPoint.Origin, GeoVector.ZAxis, r);
            SurfaceOfLinearExtrusion c2 = ExtrudedCylinder(GeoPoint.Origin, GeoVector.YAxis, r);
            BoundingRect domain = ExtrusionDomain(30.0);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(c1, domain, c2, domain, precision);
            Dump("two NURBS cylinders, perpendicular axes", contacts);

            Assert.AreEqual(2, contacts.Length, "expected exactly the two crossing points");
            foreach (SurfaceContact c in contacts)
            {
                Assert.AreEqual(ContactType.Crossing, c.Type);
                Assert.AreEqual(2, c.BranchDirections.Length, "a node has two crossing branches");
                Assert.AreEqual(0.0, c.DistanceDefect, 1e-6);
                Assert.AreEqual(0.0, c.AngleDefect, 1e-5);
                Assert.AreEqual(r, System.Math.Abs(c.Location.x), 1e-6);
                Assert.AreEqual(0.0, c.Location.y, 1e-6);
                Assert.AreEqual(0.0, c.Location.z, 1e-6);
            }
            Assert.AreNotEqual(System.Math.Sign(contacts[0].Location.x), System.Math.Sign(contacts[1].Location.x),
                "the two nodes are on opposite sides");
        }

        /// <summary>
        /// The same two cylinders with DIFFERENT radii. Their normals are still parallel only where
        /// y = z = 0, but there the two surfaces are 4 apart, so there is no contact at all - and a point
        /// where the normals are parallel without the surfaces meeting must not be reported as one. This is
        /// the test that the second condition of the general search is actually checked.
        /// </summary>
        [TestMethod]
        public void nurbs_cylinders_of_different_radius_do_not_touch()
        {
            SurfaceOfLinearExtrusion c1 = ExtrudedCylinder(GeoPoint.Origin, GeoVector.ZAxis, 10.0);
            SurfaceOfLinearExtrusion c2 = ExtrudedCylinder(GeoPoint.Origin, GeoVector.YAxis, 6.0);
            BoundingRect domain = ExtrusionDomain(30.0);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(c1, domain, c2, domain, precision);
            Dump("two NURBS cylinders of different radius", contacts);
            Assert.AreEqual(0, contacts.Length, "they cross transversally, they do not touch");
        }

        /// <summary>
        /// A NURBS cylinder against a real CylindricalSurface: one operand has a canal form and the other
        /// does not, which is the mixed case. The two nodes must come out the same way.
        /// </summary>
        [TestMethod]
        public void nurbs_cylinder_against_a_quadric_cylinder_touches_at_two_nodes()
        {
            const double r = 10.0;
            SurfaceOfLinearExtrusion c1 = ExtrudedCylinder(GeoPoint.Origin, GeoVector.ZAxis, r);
            CylindricalSurface c2 = Cylinder(GeoPoint.Origin, GeoVector.YAxis, r);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(c1, ExtrusionDomain(30.0), c2, Wide, precision);
            Dump("NURBS cylinder against a quadric cylinder", contacts);

            Assert.AreEqual(2, contacts.Length);
            foreach (SurfaceContact c in contacts)
            {
                Assert.AreEqual(ContactType.Crossing, c.Type);
                Assert.AreEqual(r, System.Math.Abs(c.Location.x), 1e-6);
                Assert.AreEqual(0.0, c.Location.y, 1e-6);
                Assert.AreEqual(0.0, c.Location.z, 1e-6);
            }
        }

        /// <summary>
        /// A cylinder as a RuledSurface between two exact NURBS circles, against a plane tangent to it. As
        /// for the quadric cylinder the contact is a whole ruling and not a point, so what has to come out
        /// is several Degenerate samples along that line and no invented branches.
        /// </summary>
        [TestMethod]
        public void ruled_nurbs_cylinder_and_tangent_plane_touch_along_a_ruling()
        {
            const double r = 4.0;
            // the axis is the x axis at height r, so the plane z = 0 touches along y = 0, z = 0
            RuledSurface cyl = new RuledSurface(
                NurbsCircle(new GeoPoint(-20, 0, r), r * GeoVector.YAxis, r * GeoVector.ZAxis),
                NurbsCircle(new GeoPoint(20, 0, r), r * GeoVector.YAxis, r * GeoVector.ZAxis));
            PlaneSurface pln = PlaneThrough(GeoPoint.Origin, GeoVector.ZAxis);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(pln, Wide, cyl, Natural(cyl), precision);
            Dump("plane tangent to a ruled NURBS cylinder", contacts);

            Assert.IsTrue(contacts.Length > 1, "the contact is a whole ruling, not a point");
            foreach (SurfaceContact c in contacts)
            {
                Assert.AreEqual(ContactType.Degenerate, c.Type);
                Assert.AreEqual(0, c.BranchDirections.Length, "no branch directions for a degenerate contact");
                Assert.AreEqual(0.0, c.Location.z, 1e-5, "every contact point is on the plane z = 0");
                Assert.AreEqual(0.0, c.Location.y, 1e-5, "and on the ruling y = 0");
            }
        }

        /// <summary>
        /// A torus built by sweeping an exact NURBS circle along another one - a SweptCurveSurface, the
        /// surface <c>solid.sweep</c> produces - against the plane tangent at its INNER equator. That point
        /// is a saddle, so the plane crosses the surface there and the contact is a node; the same plane at
        /// the outer equator touches an elliptic point and does not cross. Both answers are the ones the
        /// ToroidalSurface gives in
        /// <see cref="plane_at_the_inner_equator_of_a_torus_is_a_node_the_outer_one_is_not"/>.
        /// </summary>
        [TestMethod]
        public void swept_nurbs_torus_against_the_equator_planes()
        {
            const double R = 30.0, r = 8.0;
            SweptCurveSurface tor = new SweptCurveSurface(
                NurbsCircle(new GeoPoint(R, 0, 0), r * GeoVector.XAxis, r * GeoVector.ZAxis),
                NurbsCircle(GeoPoint.Origin, R * GeoVector.XAxis, R * GeoVector.YAxis));
            BoundingRect domain = Natural(tor);

            SurfaceContact[] inner = Surfaces.TangentialContacts(
                PlaneThrough(new GeoPoint(R - r, 0, 0), GeoVector.XAxis), Wide, tor, domain, precision);
            Dump("swept NURBS torus, plane at the inner equator", inner);
            Assert.AreEqual(1, inner.Length);
            Assert.AreEqual(ContactType.Crossing, inner[0].Type, "the inner equator is a saddle point");
            Assert.AreEqual(0.0, (inner[0].Location | new GeoPoint(R - r, 0, 0)), 1e-5);

            SurfaceContact[] outer = Surfaces.TangentialContacts(
                PlaneThrough(new GeoPoint(R + r, 0, 0), GeoVector.XAxis), Wide, tor, domain, precision);
            Dump("swept NURBS torus, plane at the outer equator", outer);
            Assert.AreEqual(1, outer.Length);
            Assert.AreEqual(ContactType.Isolated, outer[0].Type, "the outer equator is an elliptic point");
            Assert.AreEqual(0.0, (outer[0].Location | new GeoPoint(R + r, 0, 0)), 1e-5);
        }

        /// <summary>
        /// The same torus as a SurfaceOfRevolution - the fourth of the surfaces this search had to reach -
        /// against the plane tangent at its inner equator. Same saddle, same node.
        /// </summary>
        [TestMethod]
        public void revolved_nurbs_torus_against_the_inner_equator_plane()
        {
            const double R = 30.0, r = 8.0;
            SurfaceOfRevolution tor = new SurfaceOfRevolution(
                NurbsCircle(new GeoPoint(R, 0, 0), r * GeoVector.XAxis, r * GeoVector.ZAxis),
                GeoPoint.Origin, GeoVector.ZAxis);

            SurfaceContact[] contacts = Surfaces.TangentialContacts(
                PlaneThrough(new GeoPoint(R - r, 0, 0), GeoVector.XAxis), Wide, tor, Natural(tor), precision);
            Dump("revolved NURBS torus, plane at the inner equator", contacts);

            Assert.AreEqual(1, contacts.Length);
            Assert.AreEqual(ContactType.Crossing, contacts[0].Type, "the inner equator is a saddle point");
            Assert.AreEqual(0.0, (contacts[0].Location | new GeoPoint(R - r, 0, 0)), 1e-5);
        }

        /// <summary>
        /// The NURBS twin of <see cref="cones_tangent_at_a_mantle_point_are_found"/> and the case the merging
        /// of near coincident results exists for: two truncated cones ruled between exact NURBS circles,
        /// coneB being coneA turned by 180 degrees about the surface normal at P = (14.25, 0, 9.5).
        /// <para>
        /// That rotation fixes P and fixes the ruling through P - it is perpendicular to the axis of the
        /// rotation, so it is mapped onto itself - which makes the whole ruling a common line of BOTH cones
        /// with a common tangent plane: the two mantles touch along the entire ruling, and at P the
        /// intersection curve has a node on top of that. Along a contact curve the system solved for a
        /// contact point is singular, so several starts end a few micrometers apart around the node, and
        /// each of them used to be reported as its own crossing - three nodes where the model has one, which
        /// a boolean operation turns into three vertices. What has to come out is ONE node, at P.
        /// </para>
        /// <para>
        /// The domains are the halves of the u range, which is how the boolean operation sees the two
        /// mantles: they are split at the seam of the NURBS circle, and it is on those halves that the
        /// cluster appeared.
        /// </para>
        /// </summary>
        [TestMethod]
        public void ruled_nurbs_cones_tangent_at_a_mantle_point_give_one_node()
        {
            RuledSurface coneA = new RuledSurface(
                NurbsCircle(GeoPoint.Origin, 19 * GeoVector.XAxis, 19 * GeoVector.YAxis),
                NurbsCircle(new GeoPoint(0, 0, 18), 10 * GeoVector.XAxis, 10 * GeoVector.YAxis));
            GeoPoint p = new GeoPoint(14.25, 0, 9.5);
            GeoVector n = new GeoVector(2, 0, 1).Normalized; // the surface normal of coneA at p
            RuledSurface coneB = coneA.Clone() as RuledSurface;
            coneB.Modify(ModOp.Rotate(p, n, SweepAngle.Deg(180)));

            BoundingRect left = new BoundingRect(0.0, 0.0, 0.5, 1.0), right = new BoundingRect(0.5, 0.0, 1.0, 1.0);
            foreach ((BoundingRect d1, BoundingRect d2, string what) in new[]
            {
                (Natural(coneA), Natural(coneB), "the whole mantles"),
                (left, right, "the left half against the right one"),
                (right, left, "the right half against the left one")
            })
            {
                SurfaceContact[] contacts = Surfaces.TangentialContacts(coneA, d1, coneB, d2, precision);
                Dump("two ruled NURBS cones tangent at a mantle point, " + what, contacts);

                SurfaceContact node = null;
                int nodes = 0;
                foreach (SurfaceContact c in contacts)
                {
                    if (c.Type != ContactType.Crossing) continue;
                    ++nodes;
                    node = c;
                }
                Assert.AreEqual(1, nodes, $"{what}: the single node at {p} must be reported exactly once");
                Assert.AreEqual(0.0, (node.Location | p), 1e-4, $"{what}: the node is at {p}");
                Assert.AreEqual(2, node.BranchDirections.Length, "a node has two crossing branches");
                foreach (GeoVector b in node.BranchDirections)
                    Assert.AreEqual(0.0, b.Normalized * n, 1e-4, "a branch tangent lies in the common tangent plane");
                // The representative kept must be the one AT the node: next to it, along the contact
                // ruling, the two branches close up into the double root of the tangential contact.
                double cosine = System.Math.Abs(node.BranchDirections[0] * node.BranchDirections[1]);
                Assert.IsTrue(cosine < 0.5, $"{what}: the two branches of the node must be clearly distinct, "
                    + $"the angle between them is only {System.Math.Acos(cosine) * 180.0 / System.Math.PI:F2} degrees");
                // the contact along the ruling is what it is - samples of it, never invented branches
                foreach (SurfaceContact c in contacts)
                    if (c.Type != ContactType.Crossing)
                        Assert.AreEqual(ContactType.Degenerate, c.Type, "the ruling is a tangential contact curve");
            }
        }

        /// <summary>
        /// A plane that does not reach the NURBS cylinder has no contact - and, just as important, costs the
        /// coarse scan and nothing else: the bounding boxes do not even overlap.
        /// </summary>
        [TestMethod]
        public void plane_missing_the_nurbs_cylinder_has_no_contact()
        {
            SurfaceOfLinearExtrusion cyl = ExtrudedCylinder(new GeoPoint(0, 0, 0), GeoVector.ZAxis, 10.0);
            PlaneSurface pln = PlaneThrough(new GeoPoint(11, 0, 0), GeoVector.XAxis);
            SurfaceContact[] contacts = Surfaces.TangentialContacts(pln, Wide, cyl, ExtrusionDomain(30.0), precision);
            Dump("plane one unit away from the NURBS cylinder", contacts);
            Assert.AreEqual(0, contacts.Length);
        }

        #endregion
    }
}
