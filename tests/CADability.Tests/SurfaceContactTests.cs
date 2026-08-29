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
    }
}
