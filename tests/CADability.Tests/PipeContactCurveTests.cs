using CADability.Curve2D;
using CADability.GeoObject;

namespace CADability.Tests
{
    /// <summary>
    /// Tests for the curve along which a pipe (<see cref="SweptCircleSurface"/>) touches a surface, as a fillet touches the
    /// faces it connects. The spines are chosen on an offset of the touched surface, so the curve of contact is known
    /// exactly: on a plane it is the spine moved by the radius, on a cylinder the spine scaled radially onto the cylinder.
    /// The curve is reached through the public <see cref="ISurface.GetDualSurfaceCurves"/>, which uses the tangential
    /// pipe intersection for this case.
    /// </summary>
    [TestClass]
    public class PipeContactCurveTests
    {
        private const double radius = 1.0;

        private static BoundingRect Wide => new BoundingRect(-100, -100, 100, 100);

        /// <summary>A planar, wavy spine at the height of the radius above the xy-plane.</summary>
        private static ICurve WavySpine()
        {
            GeoPoint[] points = new GeoPoint[7];
            for (int i = 0; i < points.Length; i++) points[i] = new GeoPoint(3.0 * i, 2.0 * System.Math.Sin(0.9 * i), radius);
            BSpline bsp = BSpline.Construct();
            bsp.ThroughPoints(points, 3, false);
            return bsp;
        }

        /// <summary>An arc of the ellipse, in which the plane z = 0.3 x cuts the cylinder with the radius <paramref name="r"/> around the z-axis.</summary>
        private static ICurve EllipticSpine(double r)
        {
            Ellipse e = Ellipse.Construct();
            e.SetEllipseArcCenterAxis(GeoPoint.Origin, new GeoVector(r, 0, 0.3 * r), new GeoVector(0, r, 0), 0.2, 1.5);
            return e;
        }

        private static IDualSurfaceCurve Contact(ISurface surface, ICurve spine, System.Func<GeoPoint, GeoPoint> exactContact)
        {
            SweptCircleSurface pipe = new SweptCircleSurface(spine, radius);
            BoundingRect pipeBounds = new BoundingRect(0.0, -10, 1.0, 10);
            List<GeoPoint> seeds = [exactContact(spine.PointAt(0.1)), exactContact(spine.PointAt(0.9))];
            IDualSurfaceCurve[] curves = surface.GetDualSurfaceCurves(Wide, pipe, pipeBounds, seeds, null);
            Assert.AreEqual(1, curves.Length, "one curve of contact");
            Assert.AreEqual("PipeContactCurve", curves[0].Curve3D.GetType().Name, "the exact curve of contact");
            return curves[0];
        }

        /// <param name="contactError">The distance of a point of the touched surface from the curve of contact</param>
        private static void CheckCurve(IDualSurfaceCurve dsc, System.Func<GeoPoint, GeoPoint> exactContact, ICurve spine, System.Func<GeoPoint, double> contactError)
        {
            ICurve curve = dsc.Curve3D;
            for (int i = 0; i <= 20; i++)
            {
                double pos = i / 20.0;
                GeoPoint p = curve.PointAt(pos);
                // on the curve of contact
                Assert.AreEqual(0.0, contactError(p), 1e-9, $"point at {pos}");
                // on both surfaces, with the exact 2d curves
                Assert.AreEqual(0.0, dsc.Surface1.PointAt(dsc.Curve2D1.PointAt(pos)) | p, 1e-9, $"2d curve on the surface at {pos}");
                Assert.AreEqual(0.0, dsc.Surface2.PointAt(dsc.Curve2D2.PointAt(pos)) | p, 1e-8, $"2d curve on the pipe at {pos}");
                // the analytic derivatives
                const double h = 1e-5;
                GeoVector numeric = (1.0 / (2 * h)) * (curve.PointAt(pos + h) - curve.PointAt(pos - h));
                GeoVector analytic = curve.DirectionAt(pos);
                Assert.AreEqual(0.0, (numeric - analytic).Length / analytic.Length, 1e-6, $"derivative at {pos}");
                foreach ((ISurface surface, ICurve2D c2d) in new[] { (dsc.Surface1, dsc.Curve2D1), (dsc.Surface2, dsc.Curve2D2) })
                {
                    GeoVector2D numeric2d = (1.0 / (2 * h)) * (c2d.PointAt(pos + h) - c2d.PointAt(pos - h));
                    GeoVector2D analytic2d = c2d.DirectionAt(pos);
                    Assert.AreEqual(0.0, (numeric2d - analytic2d).Length / analytic2d.Length, 1e-6, $"2d derivative at {pos}");
                }
                // the position of a point of the curve
                Assert.AreEqual(pos, curve.PositionOf(p), 1e-9, $"position of the point at {pos}");
            }
            // end points at the seeds
            Assert.AreEqual(0.0, curve.StartPoint | exactContact(spine.PointAt(0.1)), 1e-9, "start point");
            Assert.AreEqual(0.0, curve.EndPoint | exactContact(spine.PointAt(0.9)), 1e-9, "end point");
        }

        /// <summary>The distance of <paramref name="p"/> from <paramref name="curve"/>, refined by Newton (PositionOf alone is not precise enough).</summary>
        private static double DistanceToCurve(ICurve curve, GeoPoint p)
        {
            double t = curve.PositionOf(p);
            for (int i = 0; i < 10; i++)
            {
                GeoVector d = curve.DirectionAt(t);
                t += ((p - curve.PointAt(t)) * d) / (d * d);
            }
            return curve.PointAt(t) | p;
        }

        [TestMethod]
        public void the_contact_with_a_plane_is_the_spine_moved_by_the_radius()
        {
            ICurve spine = WavySpine();
            PlaneSurface plane = new PlaneSurface(Plane.XYPlane);
            GeoPoint onPlane(GeoPoint s) => new GeoPoint(s.x, s.y, s.z - radius);
            IDualSurfaceCurve dsc = Contact(plane, spine, onPlane);
            CheckCurve(dsc, onPlane, spine, p => DistanceToCurve(spine, new GeoPoint(p.x, p.y, p.z + radius)) + System.Math.Abs(p.z));
        }

        [TestMethod]
        public void the_contact_with_a_cylinder_from_outside_and_inside()
        {
            const double cylinderRadius = 5.0;
            CylindricalSurface cylinder = new CylindricalSurface(GeoPoint.Origin, cylinderRadius * GeoVector.XAxis, cylinderRadius * GeoVector.YAxis, GeoVector.ZAxis);
            GeoPoint onCylinder(GeoPoint s)
            {
                double scale = cylinderRadius / System.Math.Sqrt(s.x * s.x + s.y * s.y);
                return new GeoPoint(scale * s.x, scale * s.y, s.z);
            }
            foreach (double spineRadius in new[] { cylinderRadius + radius, cylinderRadius - radius })
            {
                ICurve spine = EllipticSpine(spineRadius);
                IDualSurfaceCurve dsc = Contact(cylinder, spine, onCylinder);
                // the spine point of a contact point is on the same ray from the axis, it must be in the plane z = 0.3 x
                double contactError(GeoPoint p)
                {
                    double scale = spineRadius / System.Math.Sqrt(p.x * p.x + p.y * p.y);
                    return System.Math.Abs(p.z - 0.3 * scale * p.x) / System.Math.Sqrt(1.09) + System.Math.Abs(cylinder.GetDistance(p));
                }
                CheckCurve(dsc, onCylinder, spine, contactError);
            }
        }

        [TestMethod]
        public void the_curve_of_contact_survives_json_and_parts_of_it_stay_exact()
        {
            ICurve spine = WavySpine();
            PlaneSurface plane = new PlaneSurface(Plane.XYPlane);
            GeoPoint onPlane(GeoPoint s) => new GeoPoint(s.x, s.y, s.z - radius);
            ICurve curve = Contact(plane, spine, onPlane).Curve3D;
            ICurve read = (ICurve)JsonSerialize.FromString(JsonSerialize.ToString(curve));
            for (int i = 0; i <= 10; i++) Assert.AreEqual(0.0, read.PointAt(i / 10.0) | curve.PointAt(i / 10.0), 1e-12, "read back");
            ICurve[] parts = curve.Split(0.3);
            Assert.AreEqual(2, parts.Length);
            Assert.AreEqual(0.0, parts[0].EndPoint | curve.PointAt(0.3), 1e-12, "split point");
            Assert.AreEqual(0.0, parts[1].PointAt(0.5) | curve.PointAt(0.65), 1e-12, "second part");
            ICurve reversed = curve.Clone();
            reversed.Reverse();
            Assert.AreEqual(0.0, reversed.PointAt(0.25) | curve.PointAt(0.75), 1e-12, "reversed");
            Assert.IsTrue(reversed.SameGeometry(curve, Precision.eps), "same geometry");
        }
    }
}
