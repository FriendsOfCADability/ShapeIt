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
    /* Die Flächen in OpenCascade sind folgende:
     * Geom_CylindricalSurface, Geom_BezierSurface, Geom_BSplineSurface (=NURBS), Geom_ConicalSurface, 
     * Geom_CylindricalSurface, Geom_OffsetSurface, Geom_SphericalSurface, Geom_SurfaceOfLinearExtrusion,
     * Geom_SurfaceOfRevolution, Geom_ToroidalSurface.
     * 
     * Man kann aus allen Flächen NURBS machen, insofern müssen wir nicht alle implementieren, aber das ist
     * halt nur die Annäherung.
     * 
     * Man muss für alle Flächen die sie bestimmenden Daten finden, meist sieht man das im Konstruktor.
     * Eine Achse (Ax3) ist durch location, directionx und direction y bestimmt. Geom_SurfaceOfLinearExtrusion und 
     * Geom_SurfaceOfRevolution haben zusätzlich eine Kurve als Datum, (BasisCurve), die man mit CndHlp3D
     * static Edge FromCurve(Geom.Curve curve) in ein Edge und von dortaus mit IGeoObjectImpl.FromHlp3DEdge
     * in ein IGeoObject und somit in ein ICurve umwandeln kann. Die einzigen Daten, die ISurface abgeleitete Objekte
     * haben sind double, GeoPoint, GeoVector, ICurve. Das ist alles serialisierbar.
     */

    public enum RuledSurfaceMode { notRuled, ruledInU, ruledInV, planar, local }
    /// <summary>
    /// The ISurface interface must be implemented by all 3-dimensional unbound surfaces that are used by the
    /// <see cref="Face"/> object. The surface has a well defined 2-dimensional coordinate system, usually referred to
    /// as the u/v system.
    /// </summary>

    public interface ISurface
    {
        /// <summary>
        /// Returns a clone of this surface modified by the given ModOp.
        /// </summary>
        /// <param name="m">how to modify</param>
        ISurface GetModified(ModOp m);
        /// <summary>
        /// Returns a 3-dimensional curve from the given 2-dimensional curve. the 2-dimensional curve
        /// is interpreted in the u/v system of the surface.
        /// </summary>
        /// <param name="curve2d">the base curve</param>
        /// <returns>corresponding 3-d curve</returns>
        ICurve Make3dCurve(ICurve2D curve2d);
        /// <summary>
        /// Returns the normal vector (perpendicular to the surface) at the given u/v point
        /// </summary>
        /// <param name="uv">position of normal</param>
        /// <returns>normal vector</returns>
        GeoVector GetNormal(GeoPoint2D uv);
        /// <summary>
        /// Returns the direction at the given u/v point in direction of the u-axis
        /// </summary>
        /// <param name="uv">position</param>
        /// <returns>direction</returns>
        GeoVector UDirection(GeoPoint2D uv);
        /// <summary>
        /// Returns the direction at the given u/v point in direction of the v-axis
        /// </summary>
        /// <param name="uv">position</param>
        /// <returns>direction</returns>
        GeoVector VDirection(GeoPoint2D uv);
        /// <summary>
        /// Returns the 3-dimensional point at the given u/v point
        /// </summary>
        /// <param name="uv">position</param>
        /// <returns>point</returns>
        GeoPoint PointAt(GeoPoint2D uv);
        /// <summary>
        /// Returns the u/v position of the given point. It is assumed that the point is on the surface,
        /// if not the result is undetermined.
        /// </summary>
        /// <param name="p">point</param>
        /// <returns>position</returns>
        GeoPoint2D PositionOf(GeoPoint p);
        /// <summary>
        /// Returns the point and the two derivations of the suface in a single call. It returns the same result as calling
        /// <see cref="PointAt"/>, <see cref="VDirection"/> und <see cref="VDirection"/> succesively but is often faster
        /// than the three seperate calls.
        /// </summary>
        /// <param name="uv">Point in the parameter space</param>
        /// <param name="location">Resulting 3D point</param>
        /// <param name="du">Resulting derivation in u</param>
        /// <param name="dv">Resulting derivation in v</param>
        void DerivativeAt(GeoPoint2D uv, out GeoPoint location, out GeoVector du, out GeoVector dv);
        /// <summary>
        /// Returns the point, the two first derivations and the three second derivations of the surface at the provided parameter position.
        /// 
        /// </summary>
        /// <param name="uv">Point in the parameter space</param>
        /// <param name="location">Resulting 3D point</param>
        /// <param name="du">Resulting derivation in u</param>
        /// <param name="dv">Resulting derivation in v</param>
        /// <param name="duu"></param>
        /// <param name="dvv"></param>
        /// <param name="duv"></param>
        void Derivative2At(GeoPoint2D uv, out GeoPoint location, out GeoVector du, out GeoVector dv, out GeoVector duu, out GeoVector dvv, out GeoVector duv);
        /// <summary>
        /// Returns the intersection curve(s) of this surface with the given plane. An empty array is returned if there is no intersection.
        /// umin, umax, vmin, vmax define the Parameterspace of this surface (not of the PlaneSurface) for the intersection. It is also the periodic domain
        /// in which the 2d curve for this surface will be returned, if this surface is periodic. the resulting curves may exceed the area provided by umin, umax, vmin, vmax.
        /// </summary>
        /// <param name="pl">plane to intersect with</param>
        /// <returns>intersection curves</returns>
        IDualSurfaceCurve[] GetPlaneIntersection(PlaneSurface pl, double umin, double umax, double vmin, double vmax, double precision);
        /// <summary>
        /// Returns curves where direction is perpendicular to the normal vector
        /// </summary>
        /// <param name="direction"></param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <returns></returns>
        ICurve2D[] GetTangentCurves(GeoVector direction, double umin, double umax, double vmin, double vmax);
        /// <summary>
        /// Gets spans of the parameterspace that are guaranteed to contain only one inflection point.
        /// The returned intu should contain umin as first and umax as last value (same with v)
        /// </summary>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <param name="intu"></param>
        /// <param name="intv"></param>
        void GetSafeParameterSteps(double umin, double umax, double vmin, double vmax, out double[] intu, out double[] intv);
        /// <summary>
        /// Returns true if the given projection makes the surface disappear, i.e. degenerate to an edge.
        /// </summary>
        /// <param name="p">the projection</param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <returns>true if vanishing, falso otherwise</returns>
        bool IsVanishingProjection(Projection p, double umin, double umax, double vmin, double vmax);
        /// <summary>
        /// Returns the intersectionpoints of this surface with the line given by the parameters.
        /// Teh returned point are in the parametric (u/v) space of this surface.
        /// </summary>
        /// <param name="startPoint">startpoint of the line</param>
        /// <param name="direction">direction of the line</param>
        /// <returns></returns>
        GeoPoint2D[] GetLineIntersection(GeoPoint startPoint, GeoVector direction);
        /// <summary>
        /// Returns true, if this surface is periodic in the u direction (e.g. a cylinder)
        /// false otherwise.
        /// </summary>
        bool IsUPeriodic { get; }
        /// <summary>
        /// Returns true, if this surface is periodic in the v direction (e.g. a torus)
        /// false otherwise.
        /// </summary>
        bool IsVPeriodic { get; }
        /// <summary>
        /// Returns the u priod of this surface if it is <see cref="IsUPeriodic">u periodic</see>
        /// 0.0 otherwise
        /// </summary>
        double UPeriod { get; }
        /// <summary>
        /// Returns the v priod of this surface if it is <see cref="IsVPeriodic">v periodic</see>
        /// 0.0 otherwise
        /// </summary>
        double VPeriod { get; }
        /// <summary>
        /// returns the values for the u parameter where this surface is singular i.e. changing
        /// v with this u parameter fixed doesn't change the 3D point.
        /// </summary>
        /// <returns>list of u singularities</returns>
        double[] GetUSingularities();

        /// <summary>
        /// returns the values for the v parameter where this surface is singular i.e. changing
        /// u with this v parameter fixed doesn't change the 3D point.
        /// </summary>
        /// <returns>list of v singularities</returns>
        double[] GetVSingularities();
        /// <summary>
        /// Makes a <see cref="Face"/> from this surface with the given bounds in the parametric (u/v) space.
        /// </summary>
        /// <param name="simpleShape">the bounds</param>
        /// <returns>the created face or null</returns>
        Face MakeFace(CADability.Shapes.SimpleShape simpleShape);
        /// <summary>
        /// Gets the minimum and maximumm valus for the z-coordinate of a rectangular patch (in parametric space) 
        /// of this surface under a certain projection
        /// </summary>
        /// <param name="p">the projection</param>
        /// <param name="umin">left bound of the rectangular patch</param>
        /// <param name="umax">right bound of the rectangular patch</param>
        /// <param name="vmin">bottom bound of the rectangular patch</param>
        /// <param name="vmax">top bound of the rectangular patch</param>
        /// <param name="zMin">returned minimum</param>
        /// <param name="zMax">returned maximum</param>
        void GetZMinMax(Projection p, double umin, double umax, double vmin, double vmax, ref double zMin, ref double zMax);
        /// <summary>
        /// Modifies this surface into a more canonical form and returns the modification for the parametric
        /// space which reverses this modification in 2d. Curves in the parametric space of this surface will become
        /// invalid unless modified by the returned transformation.
        /// </summary>
        /// <returns>2d modification for the parametric space</returns>
        ModOp2D MakeCanonicalForm();
        /// <summary>
        /// Returns an identical but independant copy of this surface
        /// </summary>
        /// <returns></returns>
        ISurface Clone();
        /// <summary>
        /// Modifies this surface with the given operation
        /// </summary>
        /// <param name="m">how to modif</param>
        void Modify(ModOp m);
        /// <summary>
        /// Copies the data of the given surface to this surface. The two surfaces are guaranteed to be of the
        /// same type. (Used after <see cref="Clone"/> and <see cref="Modify"/> to restore the original values).
        /// </summary>
        /// <param name="CopyFrom">where to copy the data from</param>
        void CopyData(ISurface CopyFrom);
        /// <summary>
        /// Create a NurbSurface as an approximation of this surface
        /// </summary>
        /// <param name="precision"></param>
        /// <returns></returns>
        NurbsSurface Approximate(double umin, double umax, double vmin, double vmax, double precision);
        /// <summary>
        /// Returns the projection of the given curve in 2D coordinates. Should only be used for curves 
        /// that are close to the surface.
        /// </summary>
        /// <param name="curve"></param>
        /// <returns></returns>
        ICurve2D GetProjectedCurve(ICurve curve, double precision); // muss mit BoundingRect domain erweitert werden
        /// <summary>
        /// Returns the intersection of the provided <paramref name="curve"/> with this surface. 
        /// The result may be empty.
        /// </summary>
        /// <param name="curve">The curve to be intersected with</param>
        /// <param name="ips">Resulting 3d intersection points</param>
        /// <param name="uvOnFaces">u/v values of the intersection points on this surface</param>
        /// <param name="uOnCurve3Ds">u parameter of intersection points on the curve</param>
        void Intersect(ICurve curve, BoundingRect uvExtent, out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve3Ds);
        /// <summary>
        /// Returns a double value, if this surface is parallel to the other surface. Returns double.MaxValue otherwise.
        /// In most cases the bounds may be ignored.
        /// </summary>
        /// <param name="thisBounds"></param>
        /// <param name="other"></param>
        /// <param name="otherBounds"></param>
        /// <returns></returns>
        double IsParallel(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds);

        /// <summary>
        /// Returns the intersection curves between this surface and the provided other surface.
        /// Both surfaces are bound by rectangles.
        /// </summary>
        /// <param name="thisBounds">Bounds for this surface</param>
        /// <param name="other">Other surface</param>
        /// <param name="otherBounds">Bounds of other surface</param>
        /// <returns>Array of intersection curves</returns>
        ICurve[] Intersect(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds);
        /// <summary>
        /// Reverses the orientation of this surface. The normal vector will point to the other side after this operation.
        /// The returned <see cref="ModOp2D"/> determins how (u,v) coordinates of the parameter space have to be 
        /// transformed to define the same 3d point.
        /// </summary>
        /// <returns>Transformation of the parameter space</returns>
        ModOp2D ReverseOrientation();
        /// <summary>
        /// Returns true if this surface and the other surface are geometrically identical, i.e. describe the same surface
        /// in 3D space. The may have a different u/v system. The returned <paramref name="firstToSecond"/> contains
        /// the ModOp to convert from the u/v system of the first surface to the second surface.
        /// </summary>
        /// <param name="thisBounds">Bounds for this surface</param>
        /// <param name="other">Other surface</param>
        /// <param name="otherBounds">Bounds of other surface</param>
        /// <param name="precision">Required precision</param>
        /// <param name="firstToSecond">Transformation between different u/v systems</param>
        /// <returns>True if the surfaces are geometrically equal</returns>
        bool SameGeometry(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, double precision, out ModOp2D firstToSecond);
        /// <summary>
        /// Returns a surface that is "parallel" to this surface, i.e. each point on this surface corresponds a 
        /// point on the returned surface that has the same (u,v) coordinates and has the 3d coordinates oft the
        /// point plus offset*Normal at this point
        /// </summary>
        /// <param name="offset">Offset to this surface</param>
        /// <returns>The offset surface</returns>
        ISurface GetOffsetSurface(double offset);
        /// <summary>
        /// Returns the natural bounds of the surface. The returned values may be infinite
        /// </summary>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        void GetNaturalBounds(out double umin, out double umax, out double vmin, out double vmax);
        /// <summary>
        /// Checks whether this surface restricted by the provided parameters interferes with the provided cube.
        /// </summary>
        /// <param name="cube">Bounding cube for the test</param>
        /// <param name="umin">Minimum for the u parameter</param>
        /// <param name="umax">Maximum for the u parameter</param>
        /// <param name="vmin">Minimum for the v parameter</param>
        /// <param name="vmax">Maximum for the v parameter</param>
        /// <returns>true if the cube and the surface interfere</returns>
        bool HitTest(BoundingBox cube, double umin, double umax, double vmin, double vmax);
        /// <summary>
        /// Returns true, if this surface interferes with the provided cube. If this is the case
        /// uv will contain a point (in the parameter system of the surface) which is inside the cube
        /// </summary>
        bool HitTest(BoundingBox cube, out GeoPoint2D uv);
        /// <summary>
        /// Returns true, if this surface divides the space into two parts. If the surfaces is Oriented 
        /// <see cref="Orientation"/> returns a valid result
        /// </summary>
        bool Oriented { get; }
        /// <summary>
        /// Returns the orientation of the provided point. The sign of the result may be used to distinguish
        /// between inside and outside.
        /// </summary>
        /// <param name="p"></param>
        /// <returns></returns>
        double Orientation(GeoPoint p);
        /// <summary>
        /// Returns an array of points in parametric space where there are extrema in direction of x-, y- or z-axis.
        /// The normal vector in a extremum is parallel to one of the axis and the surface has a relative maximum or
        /// minimum in this direction.
        /// </summary>
        /// <returns>s.a.</returns>
        GeoPoint2D[] GetExtrema();
        /// <summary>
        /// Returns the extent of a patch of the surface clipped rectangular in the 2d parameter space
        /// </summary>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <returns></returns>
        BoundingBox GetPatchExtent(BoundingRect uvPatch, bool rough = false);
        /// <summary>
        /// Returns a curve where the u parameter of this surface is fixed and the v parameter starts a vmin and ends at vmax
        /// </summary>
        /// <param name="u"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <returns></returns>
        ICurve FixedU(double u, double vmin, double vmax);
        /// <summary>
        /// Returns a curve where the v parameter of this surface is fixed and the u parameter starts a umin and ends at umax
        /// </summary>
        /// <param name="v"></param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <returns></returns>
        ICurve FixedV(double v, double umin, double umax);
        double[] GetPolynomialParameters();
        /// <summary>
        /// Set the bounds of the surface to match the periodicity
        /// </summary>
        /// <param name="boundingRect"></param>
        void SetBounds(BoundingRect boundingRect);
        /// <summary>
        /// Returns a list of perpendicular foot points of the surface. The list may be empty
        /// </summary>
        /// <param name="fromHere">Source point for the perpendicular foot</param>
        /// <returns>Array of foot-points, may be empty</returns>
        GeoPoint2D[] PerpendicularFoot(GeoPoint fromHere);
        bool HasDiscontinuousDerivative(out ICurve2D[] discontinuities);
        /// <summary>
        /// If this surface is periodic in u or v or both return a non-periodic surface
        /// which describes the same geometric surface but with a different parametric system.
        /// It also removes poles and is useful for non-periodic surfaces with a single pole.
        /// </summary>
        /// <param name="orientedCurves">3d curves, which describe the outline, that will be used. this is needed by some surfaces to determine the maximum definition area</param>
        /// <returns></returns>
        ISurface GetNonPeriodicSurface(ICurve[] orientedCurves);
        /// <summary>
        /// Returns a parallelepiped (a prism with parallelograms) defined by the parameters <paramref name="loc"/>,
        /// <paramref name="dir1"/>, <paramref name="dir2"/>, <paramref name="dir3"/> which completeley covers or encloses
        /// the patch of the surface defined by the <paramref name="uvpatch"/>. There are obviously many solutions
        /// to this problem but a parallelepiped with minimum volume would be preferred. This method is used
        /// to optimate intersection algorithms.
        /// </summary>
        /// <param name="uvpatch">The patch of the surface in parametric space</param>
        /// <param name="loc">One vertex of the result</param>
        /// <param name="dir1">One of the three vectors of the parallelepiped</param>
        /// <param name="dir2">One of the three vectors of the parallelepiped</param>
        /// <param name="dir3">One of the three vectors of the parallelepiped</param>
        void GetPatchHull(BoundingRect uvpatch, out GeoPoint loc, out GeoVector dir1, out GeoVector dir2, out GeoVector dir3);
        /// <summary>
        /// returns wheather the surface is linear in u or v direction
        /// </summary>
        RuledSurfaceMode IsRuled { get; }
        /// <summary>
        /// used internally. the maximum distance of the 3d curve, formed by the uv-line from sp to ep, to the 3d-line from PointAt(sp) to PointAt(ep)
        /// </summary>
        /// <param name="sp"></param>
        /// <param name="ep"></param>
        /// <param name="surface"></param>
        /// <param name="mp">The uv point where this distance occurres</param>
        /// <returns></returns>
        double MaxDist(GeoPoint2D sp, GeoPoint2D ep, out GeoPoint2D mp);
        /// <summary>
        /// Returns the intersection curves between this surface and the provided other surface.
        /// Both surfaces are bound by rectangles.
        /// </summary>
        /// <param name="thisBounds">Bounds for this surface</param>
        /// <param name="other">Other surface</param>
        /// <param name="otherBounds">Bounds of other surface</param>
        /// <returns>Array of intersection curves</returns>
        ICurve Intersect(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, GeoPoint seed);
        /// <summary>
        /// Returns intersectionCurves 
        /// </summary>
        /// <param name="bounds1"></param>
        /// <param name="surface2"></param>
        /// <param name="bounds2"></param>
        /// <returns></returns>
        IDualSurfaceCurve[] GetDualSurfaceCurves(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, List<GeoPoint> seeds, List<Tuple<double, double, double, double>> extremePositions = null);
        /// <summary>
        /// Returns a List of self intersection curves in the u/v system
        /// </summary>
        /// <param name="bounds"></param>
        /// <returns></returns>
        ICurve2D[] GetSelfIntersections(BoundingRect bounds);
        /// <summary>
        /// Returns a list of points where the surfaces touch each other (where the surfaces are tangential)
        /// </summary>
        /// <param name="thisBounds"></param>
        /// <param name="other"></param>
        /// <param name="otherBounds"></param>
        /// <returns></returns>
        GeoPoint[] GetTouchingPoints(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds);
        /// <summary>
        /// Returns a simpler form of this surface: SurfaceOfLinearExtrusion might be a cylinder, NURBS might be a sphere etc.
        /// Returns null, if there is no simpler form
        /// </summary>
        /// <param name="precision">maximal allowed deviation of the new surface</param>
        /// <returns></returns>
        ISurface GetCanonicalForm(double precision, BoundingRect? bounds = null);
        /// <summary>
        /// Find positions in the uv system, where the connection of the points are perpendicular on both surfaces. the resulting list <paramref name="extremePositions"/> may be 
        /// only partially filled, the missing uv values may be double.NaN, because this is what we need in most cases.
        /// </summary>
        /// <param name="thisBounds"></param>
        /// <param name="other"></param>
        /// <param name="otherBounds"></param>
        /// <param name="extremePositions"></param>
        /// <returns></returns>
        int GetExtremePositions(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, out List<Tuple<double, double, double, double>> extremePositions);
        /// <summary>
        /// Find positions on the surface and on the curve where the connection of these points are perpendicular to the surface and to the curve
        /// </summary>
        /// <param name="domain">valid area for the surface</param>
        /// <param name="curve3D">the curve</param>
        /// <param name="positions">the positions found: first two doubles are u,v on the surface, third is u on the curve</param>
        /// <returns></returns>
        int GetExtremePositions(BoundingRect domain, ICurve curve3D, out List<Tuple<double, double, double>> positions);
        /// <summary>
        /// Returns the positive distance of the provided point <paramref name="p"/> to the (unlimited) surface.
        /// </summary>
        /// <param name="p"></param>
        /// <returns></returns>
        double GetDistance(GeoPoint p);
        /// <summary>
        /// returns true, if the provided direction can be interpreted as an extrusion direction of the surface
        /// </summary>
        /// <param name="direction"></param>
        /// <returns></returns>
        bool IsExtruded(GeoVector direction);
        /// <summary>
        /// returns true, if the provided direction can be interpreted as a surface of rotation around the provided <paramref name="rotationAxis"/>
        /// </summary>
        /// <param name="direction"></param>
        /// <returns></returns>
        bool IsRotated(Axis rotationAxis);
        /// <summary>
        /// Returns a context menu to change certain parameters of the surface of a face
        /// </summary>
        /// <param name="frame"></param>
        /// <param name="face"></param>
        /// <returns></returns>
        MenuWithHandler[] GetContextMenuForParametrics(IFrame frame, Face face);
        bool UvChangesWithModification { get; }
        /// <summary>
        /// Gets the IPropertyEntry to display this surface in the property grid (may be null)
        /// </summary>
        IPropertyEntry GetPropertyEntry(IFrame frame);
        /// <summary>
        /// Modifies this surface to best approximate the provided points. 
        /// </summary>
        /// <param name="toPoints"></param>
        /// <returns>The sum of the squared errors</returns>
        double Fit(IEnumerable<GeoPoint> toPoints);
        bool IsCurveOnSurface(ICurve curve);
        BoundingRect GetBounds();
        /// <summary>
        /// Performs a fast conservative test whether the given line segment
        /// may intersect this surface.
        /// Returns false if an intersection can be safely excluded.
        /// </summary>
        bool MayIntersectSegment(GeoPoint a, GeoPoint b);
    }

    public static class SurfaceExtension
    {
        /// <summary>
        ///  
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="p"></param>
        public static void ExtendBoundsTo(this ISurface surface, GeoPoint p)
        {
            GeoPoint2D uv = surface.PositionOf(p);
            BoundingRect ext = surface.GetBounds();
            if (ext.IsInfinite) ext = BoundingRect.EmptyBoundingRect;

            double[] us = surface.GetUSingularities();
            double[] vs = surface.GetVSingularities();
            bool uPole = false, vPole = false;
            for (int i = 0; i < us.Length; i++) if (Math.Abs(uv.x - us[i]) < 1e-6) uPole = true;
            for (int i = 0; i < vs.Length; i++) if (Math.Abs(uv.y - vs[i]) < 1e-6) vPole = true;

            if (!ext.IsEmpty()) SurfaceHelper.AdjustPeriodic(surface, ext, ref uv);

            if (uPole) ext.MinMaxHeight(uv.y); // only adjust the height of ext
            else if (vPole) ext.MinMaxWidth(uv.x); // only adjust the width of ext
            else ext.MinMax(uv); // adjust both width and height

            surface.SetBounds(ext);
        }
        public static void SetBoundsTo(this ISurface surface, params GeoPoint[] pp)
        {
            BoundingRect ext = BoundingRect.EmptyBoundingRect;
            for (int j = 0; j < pp.Length; j++)
            {
                GeoPoint p = pp[j];
                GeoPoint2D uv = surface.PositionOf(p);

                double[] us = surface.GetUSingularities();
                double[] vs = surface.GetVSingularities();
                bool uPole = false, vPole = false;
                for (int i = 0; i < us.Length; i++) if (Math.Abs(uv.x - us[i]) < 1e-6) uPole = true;
                for (int i = 0; i < vs.Length; i++) if (Math.Abs(uv.y - vs[i]) < 1e-6) vPole = true;

                if (!ext.IsEmpty()) SurfaceHelper.AdjustPeriodic(surface, ext, ref uv);

                if (uPole) ext.MinMaxHeight(uv.y); // only adjust the height of ext
                else if (vPole) ext.MinMaxWidth(uv.x); // only adjust the width of ext
                else ext.MinMax(uv); // adjust both width and height
            }
            surface.SetBounds(ext);
        }

        /// <summary>
        /// Adjusts the provided UV coordinates to align with a pole on the surface, if the coordinates are near a
        /// singularity.
        /// </summary>
        /// <remarks>This method checks if the provided UV coordinates are close to any singularities
        /// (poles) on the surface. If a singularity is detected within a small tolerance, the UV coordinates are
        /// adjusted to align with the specified <paramref name="alignTo"/> point.</remarks>
        /// <param name="surface">The surface on which the alignment is performed.</param>
        /// <param name="uv">The UV coordinates to adjust. This parameter is passed by reference and will be modified if alignment
        /// occurs.</param>
        /// <param name="alignTo">The 3D point to which the UV coordinates should align if they are near a singularity.</param>
        public static void AlignIfPole(this ISurface surface, ref GeoPoint2D uv, GeoPoint alignTo)
        {
            double[] us = surface.GetUSingularities();
            for (int i = 0; i < us.Length; i++)
            {
                if (Math.Abs(uv.x - us[i]) < 1e-6)
                {
                    GeoPoint2D alignTo2D = surface.PositionOf(alignTo);
                    uv.y = alignTo2D.y;
                }
            }
            double[] vs = surface.GetVSingularities();
            for (int i = 0; i < vs.Length; i++)
            {
                if (Math.Abs(uv.y - vs[i]) < 1e-6)
                {
                    GeoPoint2D alignTo2D = surface.PositionOf(alignTo);
                    uv.x = alignTo2D.x;
                }
            }
        }

        public static NurbsSurface ToNurbs(this ISurface surface, double precision)
        {
            if (surface is NurbsSurface ns) return ns;

            double[] us = surface.GetUSingularities();
            double[] vs = surface.GetVSingularities();
            BoundingRect domain = surface.GetBounds();
            for (int i = 0; i < us.Length; i++)
            {
                if (us[i] >= domain.Left && us[i] <= domain.Right) return null; // poles are not allowed
            }
            for (int i = 0; i < vs.Length; i++)
            {
                if (vs[i] >= domain.Bottom && vs[i] <= domain.Top) return null; // poles are not allowed
            }
            double du = domain.Width / 100.0;
            if (surface.IsUPeriodic) du = surface.UPeriod / 100.0;
            double dv = domain.Height / 100.0;
            if (surface.IsVPeriodic) dv = surface.VPeriod / 100.0;
            domain.Inflate(du, dv);
            int ui = 0;
            ICurve tst = surface.FixedV(domain.Bottom, domain.Left, domain.Right);
            ui = Math.Max(ui, (int)(tst.Length / precision));
            tst = surface.FixedV(domain.Top, domain.Left, domain.Right);
            ui = Math.Max(ui, (int)(tst.Length / precision));
            tst = surface.FixedV((domain.Bottom + domain.Top) / 2, domain.Left, domain.Right);
            ui = Math.Max(ui, (int)(tst.Length / precision));
            ui = Math.Max(ui, 2); // at least 2 in u direction
            int vi = 0;
            tst = surface.FixedU(domain.Left, domain.Bottom, domain.Top);
            vi = Math.Max(vi, (int)(tst.Length / precision));
            tst = surface.FixedU(domain.Right, domain.Bottom, domain.Top);
            vi = Math.Max(vi, (int)(tst.Length / precision));
            tst = surface.FixedU((domain.Left + domain.Right) / 2, domain.Bottom, domain.Top);
            vi = Math.Max(vi, (int)(tst.Length / precision));
            vi = Math.Max(vi, 2); // at least 2 in v direction

            GeoPoint[,] pts = new GeoPoint[ui + 1, vi + 1];
            for (int i = 0; i <= ui; i++)
            {
                double u = domain.Left + (domain.Right - domain.Left) * i / ui;
                for (int j = 0; j <= vi; j++)
                {
                    double v = domain.Bottom + (domain.Top - domain.Bottom) * j / vi;
                    pts[i, j] = surface.PointAt(new GeoPoint2D(u, v));
                }
            }


            NurbsSurface res = new NurbsSurface(pts, 3, 3, false, false);
            return res;
        }
    }

    /// <summary>
    /// Soll von Flächen implementiert werden, deren Schnitte mit Ebenen einfach sind (Linien, Ellipsen) um numerisch iterierte
    /// Lösungen zu vermeiden
    /// </summary>
    internal interface ISurfacePlaneIntersection
    {
        ICurve2D[] GetPlaneIntersection(Plane plane, double umin, double umax, double vmin, double vmax);
    }
    /// <summary>
    /// Interface of a surface rotating in u direction. Should be implemented by all surfaces, which can be looked at this was
    /// </summary>
    public interface ISurfaceOfRevolution
    {
        /// <summary>
        /// The axis for the revolution
        /// </summary>
        Axis Axis { get; }
        /// <summary>
        /// The curve, that is rotated
        /// </summary>
        ICurve Curve { get; }
    }
    /// <summary>
    /// Interface of an extrusion surface. All surfaces, that can be interpreted as an extrusion should implement this interface
    /// </summary>
    public interface ISurfaceOfExtrusion
    {
        /// <summary>
        /// The axis of extrusion. The axis must start at the bottom of the domain and go up to the top of the domain (or left and right, depending on <see cref="ExtrusionDirectionIsV"/>).
        /// </summary>
        ICurve Axis(BoundingRect domain);
        IOrientation Orientation { get; }
        /// <summary>
        /// The curve, which is moved along the axis, e.g. an arc, when this is a <see cref="CylindricalSurface"/>
        /// </summary>
        ICurve ExtrudedCurve { get; }
        /// <summary>
        /// The direction of extrusion is the v-parameter of the surface (when false, it is the u-parameter)
        /// </summary>
        bool ExtrusionDirectionIsV { get; }
        /// <summary>
        /// Modify the axis to make it pass the provided <paramref name="throughPoint"/> (and stay parallel to the original axis)
        /// </summary>
        /// <param name="throughPoint"></param>
        /// <returns>true, when possible</returns>
        bool ModifyAxis(GeoPoint throughPoint);
    }

    /// <summary>
    /// This surface interface is mainly for fillets
    /// </summary>
    public interface ISurfaceOfArcExtrusion : ISurfaceOfExtrusion
    {
        double Radius { get; set; }
    }
    public interface INonPeriodicSurfaceConversion
    {
        GeoPoint2D ToPeriodic(GeoPoint2D uv);
        GeoPoint2D FromPeriodic(GeoPoint2D uv);
        ICurve2D ToPeriodic(ICurve2D curve2d);
        ICurve2D FromPeriodic(ICurve2D curve2d);
    }

    internal class DirMinimum
    /* Todo:
     * In SR1 Derivation2At durch DerivationAt ersetzen. Poff!
     * Setzbarmachung von domainstretchfactor und tolerancefactor
    */
    {
        // Setzbare Variablen die als Paramter dienen
        private ISurface surface; // Das Surface auf dem gearbeitet wird
        private BoundingRect Rect; // Suchbereich
        private GeoPoint2D x; // Startwert, wird im Verlauf geändert! Default: Rect.GetCenter() bzw. (start+end)/2
        private GeoVector direction; // Suchrichtung, essentiell
        private BoundingRect domain; // Maximaler Suchbereich, default Rect (recommend)
        private BoundingRect surfacedomain; // Maximal domain of the surface. 
        private double a = 1; // Startschrittweite für Linesearch, default 1
        private double c = 0.0001; // Armijokonstante 1, default: 0.0001 (recommend)
        private double d = 0.1; // Eigenwertkorrektur für Hesse-Matrix, default: 0.1
        private double e = 10; // Eigenwertkorrektur für inverse der Hesse-Approx-Matrix, default = 10;
        private double epsilon = 0.0000000001; // Genauigkeit für Norm des Gradienten, default 0.0000000001
        private double r = 0.0000001; // Stabilitätskonstante für SR1, default 0.0000001 ; vgl. p. 145 in Numerical Optimization
        private double plength = 1; // Maximale Länge des Abstiegsvektors p, es sollte eher dies als a geändert werden!
        private double allowedoutruns = 5; // Maximale Anzahl an Iterationsschritten, die Punkte außerhalb von Rect liegen, default 5
        private int smallchange = 7; // Gibt an, wie oft sich der Gradient nur Minimal ändern darf, default 7
        private double smallchangefactor = 0.9; // Faktor um wieviel der Gradient je Schritt kleiner werden soll, default 0.1
        private int maxtries = 3; // Wie oft soll bei uv.x=4, uv.x=1, neu versucht werden

        private double domainstretchfactor = 1.03; // Gibt an, um wieviel doamin gestreckt werden soll, wenn der nächste Pkt. nicht zulässig ist.
        private double tolerancefactor = 1000; // Gibt an, um welchen Faktor der Gradient größer sein darf als epsilon um dann noch ggfs. die Schrittweite "manuell" zu setzen oder domain zu vergrößern.

        // Rein interne Variablen
        private GeoPoint2D p, ap, xap, uv, start, end;
        private bool success;
        private double steplength;
        private GeoPoint loctemp = new GeoPoint(0, 0, 0), value;
        private GeoVector dutemp, dvtemp, duutemp, dvvtemp, duvtemp;
        private GeoVector2D dir;
        private double loc, du, dv, duu, dvv, duv;
        private int stagnation;
        // Die beiden Schnuckies sollten jeweils auf 0 gesetzt werden.
        private double gradnorm = 0;
        private double lastgradnorm = 0;
        private double du2, dv2;
        private double l, t, temp1, temp2, temp3;
        private double gradient1, gradient2, p1D, hesse;
        private double y1, y2; // (y1,y2) is typically just (du2-du,dv2-dv)
        private int outrun;
        private int tries;

        // Gibt an, welches Verfahren zuletzt benutzt wurde, dies ist für etwaige Continue-Methoden existenziell, sonst produziert sich da mglws. sehr viel Käse.
        private char verfahren = '0'; // 1 -> NewtonRect, 2 -> NewtonLine, 3 -> SR1Recht, 4 -> SR1Line

        // Standardkonstruktor
        // Erhält die Fläche und bestimmt den maximalen Definitionsbereich selbiger.
        public DirMinimum(ISurface surface)
        {
            this.surface = surface;
            double umin, vmin, umax, vmax;
            surface.GetNaturalBounds(out umin, out umax, out vmin, out vmax);
            surfacedomain = new BoundingRect(umin, vmin, umax, vmax);
        }


        // Bei einem Fehlschlag kann über diese Methode der letzte zulässige Punkt, der Funktionswert und die Norm des Gradienten abgerufen werden
        public void GetLast(out GeoPoint2D x, out GeoPoint loc, out double norm)
        {
            if (verfahren == '1' || verfahren == '2')
            {
                x = this.x;
            }
            else
            {
                x = this.xap;
            }
            loc = this.loctemp;
            norm = this.gradnorm;
        }
        public GeoPoint2D Getxap()
        {
            return xap;
        }
        // Diese Mehtode bestimmt den Abstand von x in Richtung p zum Rechteckt. Zweck ist hierbei, zu wissen, wie nah man am Rand ist um ggfs. das Verfahren mit neuem Punkt zu starten.
        private double DistanceToBorder(ref BoundingRect Rect)
        {
            if (p.x > 0)
            {
                if (p.y > 0)
                {
                    temp1 = System.Math.Min((Rect.Right - x.x) / p.x, (Rect.Top - x.y) / p.y);
                    return System.Math.Sqrt(System.Math.Pow(temp1 * p.x, 2) + System.Math.Pow(temp1 * p.y, 2));
                }
                else if (p.y < 0)
                {
                    temp1 = System.Math.Min((Rect.Right - x.x) / p.x, (Rect.Bottom - x.y) / p.y);
                    return System.Math.Sqrt(System.Math.Pow(temp1 * p.x, 2) + System.Math.Pow(temp1 * p.y, 2));
                }
                else
                {
                    temp1 = (Rect.Right - x.x) / p.x;
                    return System.Math.Sqrt(System.Math.Pow(temp1 * p.x, 2) + System.Math.Pow(temp1 * p.y, 2));
                }
            }
            else if (p.x < 0)
            {
                if (p.y > 0)
                {
                    temp1 = System.Math.Min((Rect.Left - x.x) / p.x, (Rect.Top - x.y) / p.y);
                    return System.Math.Sqrt(System.Math.Pow(temp1 * p.x, 2) + System.Math.Pow(temp1 * p.y, 2));
                }
                else if (p.y < 0)
                {
                    temp1 = System.Math.Min((Rect.Left - x.x) / p.x, (Rect.Bottom - x.y) / p.y);
                    return System.Math.Sqrt(System.Math.Pow(temp1 * p.x, 2) + System.Math.Pow(temp1 * p.y, 2));
                }
                else
                {
                    temp1 = (Rect.Left - x.x) / p.x;
                    return System.Math.Sqrt(System.Math.Pow(temp1 * p.x, 2) + System.Math.Pow(temp1 * p.y, 2));
                }
            }
            else
            {
                if (p.y > 0)
                {
                    temp1 = (Rect.Top - x.y) / p.y;
                    return System.Math.Sqrt(System.Math.Pow(temp1 * p.x, 2) + System.Math.Pow(temp1 * p.y, 2));
                }
                else if (p.y < 0)
                {
                    temp1 = (Rect.Bottom - x.y) / p.y;
                    return System.Math.Sqrt(System.Math.Pow(temp1 * p.x, 2) + System.Math.Pow(temp1 * p.y, 2));
                }
                else
                {
                    return -1; // This usually can't happen.
                }
            }
        }
        public double DistanceToBorder()
        {
            return DistanceToBorder(ref this.Rect);
        }


        // Liefert false, falls ein Parameter <= 0 ist.
        public bool SetAllParams(double a, double c, double d, double e, double epsilon, double r, double plength, int allowedoutruns, int smallchange, double smallchangefactor, int maxtries, double domainstretchfactor, double tolerancefactor)
        {
            if ((a > 0) && (c > 0) && (d > 0) && (e > 0) && (epsilon > 0) && (r > 0) && (plength > 0) && (allowedoutruns >= 0) && (smallchange > 0) && (smallchangefactor > 0) && (maxtries > 0) && (domainstretchfactor > 1) && (tolerancefactor > 1))
            {
                this.a = a;
                this.c = c;
                this.d = d;
                this.e = e;
                this.epsilon = epsilon;
                this.r = r;
                this.plength = plength;
                this.allowedoutruns = allowedoutruns;
                this.smallchange = smallchange;
                this.smallchangefactor = smallchangefactor;
                this.maxtries = maxtries;
                this.tolerancefactor = tolerancefactor;
                this.domainstretchfactor = domainstretchfactor;
                return true;
            }
            return false;
        }
        public bool SetEpsilon(double epsilon)
        {
            if (epsilon > 0)
            {
                this.epsilon = epsilon;
                return true;
            }
            return false;
        }
        public bool SetPlength(double plength)
        {
            if (plength > 0)
            {
                this.plength = plength;
                return true;
            }
            return false;
        }
        public bool SetSmallchange(int smallchange, double smallchangefactor)
        {
            if ((smallchange > 0) && (smallchangefactor > 0))
            {
                this.smallchange = smallchange;
                this.smallchangefactor = smallchangefactor;
                return true;
            }
            return false;
        }
        public bool SetInitialSteplength(double a)
        {
            if (a > 0)
            {
                this.a = a;
                return true;
            }
            return false;
        }
        public bool SetAllowedOutruns(int allowedoutruns)
        {
            if (allowedoutruns >= 0)
            {
                this.allowedoutruns = allowedoutruns;
                return true;
            }
            return false;
        }
        public bool SetMaxtries(int maxtries)
        {
            if (maxtries > 0)
            {
                this.maxtries = maxtries;
                return true;
            }
            return false;
        }
        public bool SetDomainStrachfactor(double domainstretchfactor, double tolerancefactor)
        {
            if ((domainstretchfactor) > 1 && (tolerancefactor > 1))
            {
                this.domainstretchfactor = domainstretchfactor;
                this.tolerancefactor = tolerancefactor;
                return true;
            }
            return false;
        }
        public void SetDefaults()
        {
            a = 1; // Startschrittweite für Linesearch, default 1
            c = 0.0001; // Armijokonstante 1, default: 0.0001 (recommend)
            d = 0.1; // Eigenwertkorrektur für Hesse-Matrix, default: 0.1
            e = 10; // Eigenwertkorrektur für inverse der Hesse-Approx-Matrix, default = 10;
            epsilon = 0.0000000001; // Genauigkeit für Norm des Gradienten, default 0.0000000001
            r = 0.0000001; // Stabilitätskonstante für SR1, default 0.0000001 ; vgl. p. 145 in Numerical Optimization
            plength = 1; // Maximale Länge des Abstiegsvektors p, es sollte eher dies als a geändert werden!
            allowedoutruns = 5; // Maximale Anzahl an Iterationsschritten, die Punkte außerhalb von Rect liegen, default 5
            smallchange = 7; // Gibt an, wie oft sich der Gradient nur Minimal ändern darf, default 7
            smallchangefactor = 0.9; // Faktor um wieviel der Gradient je Schritt kleiner werden soll, default 0.1
            maxtries = 3; // Wie oft soll bei uv.x=4, d.h. effektive Schrittweite 0 neu versucht werden
            tolerancefactor = 1000;
            domainstretchfactor = 1.03;
        }

        // If you have effecticly steplength 0, and you have enough space left to border in directtion p, you can manually set the steplength and define a new startpoint x
        // If you use Newton, just call one of the standardmethods below with startpoint xap, if you use SR1, then use one of continuemethods to use approximation of the hessian from the last step
        public bool Setxap(double steplength)
        {
            xap = new GeoPoint2D(x.x + steplength * p.x, x.y + steplength * p.y);
            if (Rect.Contains(xap))
            {
                return true;
            }
            return false;
        }


        // Die Methoden liefern bei Erfolg true, und die Werte. 
        // Bei false gilt: 
        // uv.x=0 -> Schrittweite 0, Verfahren wird stationär, 
        // uv.x=1 -> Definitonsbereich des Surface wurde verlassen
        // uv.x=2 -> Gegebenes Rechteck wurde mehr als 5 mal verlassen, die Lsg. liegt vmtl. außerhalb
        // uv.x=3 -> Kein nennenswerter Fortschritt
        // uv.x=4 -> Schrittweise > 0, aber praktisch 0, d.h. x = xap. Wahrscheinlich nahe am Minimum, mglws. aber auch am Rand und die Fläche ist dort flach
        // uv.x=5 -> Gradient ist hinreichend klein, die Hessematrix aber nicht pos. defintit., d.h. es liegt entweder ein Maximum oder Sattelpunkt vor, oder aber es lässt sich keine Aussagen treffen
        // uv.x=6 -> Falsche Fortsetzungsmethode ausgewählt.
        // uv.x=7 -> Das Minimum wurde gefunden, liegt aber nicht in Rect.
        // uv.x=8 -> uv.x=1 & Gradient ist um mindestens tolarancefactor größer als epsilon, es steht zu erwarten, dass kein Minimum ex.
        // Beachte: uv.x=5 kann nur eintreten bei einem Newtonverfahren. Im Quasi-Newtonverfahren kann nur die geschätze Hesse-Matrix bestimmt werden, diese mglws. indefinit.
        // Beachte: Ist Rect größer als der Definitonsbereich des Surface, so gibt es möglicherweise einen Fehler!
        // Beachte: value wird bei false immer als (0,0,0) zurückgeliefert.
        // Beachte: domain und Rect werden automatisch verkleinert, falls surfacedomain nicht domain enthält bzw. domain nicht Rect.

        // Methoden für Suche nach dem Minimum auf gegebenem Rechteckt mittels Newtonverfahren
        public bool GetNewtonRect(GeoVector dir, BoundingRect Rect, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, Rect);
            this.Rect = domain;
            x = Rect.GetCenter();
            this.direction = dir;

            ManageNewtonRect();

            uv = this.uv;
            value = this.value;
            return success;
        }
        public bool GetNewtonRect(GeoVector dir, BoundingRect Rect, GeoPoint2D x0, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, Rect);
            this.Rect = domain;

            if (!Rect.Contains(x0))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            this.x = x0;
            this.direction = dir;

            ManageNewtonRect();

            uv = this.uv;
            value = this.value;
            return success;
        }
        public bool GetNewtonRect(GeoVector dir, BoundingRect Rect, BoundingRect domain, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, domain);
            this.Rect = BoundingRect.Common(this.domain, Rect);
            x = Rect.GetCenter();
            this.direction = dir;

            ManageNewtonRect();

            uv = this.uv;
            value = this.value;
            return success;
        }
        public bool GetNewtonRect(GeoVector dir, BoundingRect Rect, BoundingRect domain, GeoPoint2D x0, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, domain);
            this.Rect = BoundingRect.Common(this.domain, Rect);
            if (!Rect.Contains(x0))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            this.x = x0;
            this.direction = dir;

            ManageNewtonRect();

            uv = this.uv;
            value = this.value;
            return success;
        }
        // Continuemethods fpr NewtonRect. Be sure, you didn't changed x, if you do so, the derivates used for your x are the ones from the x you override
        // Plus, we don't check if x is in domain, we don't expect, you changed x
        // If you want to change x, use one of the methods above
        public bool ContinueNewtonRect(out GeoPoint2D uv, out GeoPoint value)
        {
            if (verfahren == '1')
            {
                success = NewtonRectangle();
                uv = this.uv;
                value = this.value;
                return success;
            }
            uv = new GeoPoint2D(6, 0);
            value = new GeoPoint(0, 0, 0);
            return false;
        }
        public bool ContinueNewtonRect(BoundingRect domain, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, domain);
            if (!domain.Contains(x))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            if (verfahren == '1')
            {
                success = NewtonRectangle();
                uv = this.uv;
                value = this.value;
                return success;
            }
            uv = new GeoPoint2D(6, 0);
            value = new GeoPoint(0, 0, 0);
            return false;
        }

        // Methoden für Suche nach dem Minimum auf gegebener Geraden mittels Newtonverfahren
        public bool GetNewtonLine(GeoVector dir, GeoPoint2D start, GeoPoint2D end, out GeoPoint2D uv, out GeoPoint value)
        {
            Rect = new BoundingRect(Math.Min(start.x, end.x), Math.Min(start.y, end.y), Math.Max(start.x, end.x), Math.Max(start.y, end.y)); domain = Rect;
            Rect = BoundingRect.Common(surfacedomain, Rect);
            domain = Rect;
            x = Rect.GetCenter();
            //x = new GeoPoint2D(start, end);
            this.direction = dir;
            this.start = start;
            this.end = end;

            ManageNewtonLine();

            uv = this.uv;
            value = this.value;
            return success;
        }
        public bool GetNewtonLine(GeoVector dir, GeoPoint2D start, GeoPoint2D end, GeoPoint2D x0, out GeoPoint2D uv, out GeoPoint value)
        {
            Rect = BoundingRect.Common(surfacedomain, new BoundingRect(new GeoPoint2D(start, end), System.Math.Abs(end.x - start.x), System.Math.Abs(end.y - start.y)));
            this.domain = Rect;
            x = x0;
            if (!Rect.Contains(x))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            if (!(((end.x - start.x) / x.x) == ((end.y - start.y) / x.y)))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }

            this.direction = dir;

            this.start = start;
            this.end = end;

            ManageNewtonLine();

            uv = this.uv;
            value = this.value;
            return success;
        }
        public bool GetNewtonLine(GeoVector dir, GeoPoint2D start, GeoPoint2D end, BoundingRect domain, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, domain);
            this.Rect = BoundingRect.Common(this.domain, new BoundingRect(new GeoPoint2D(start, end), System.Math.Abs(end.x - start.x), System.Math.Abs(end.y - start.y)));
            x = Rect.GetCenter();
            //x = new GeoPoint2D(start, end);
            this.direction = dir;

            this.start = start;
            this.end = end;

            ManageNewtonLine();

            uv = this.uv;
            value = this.value;
            return success;
        }
        public bool GetNewtonLine(GeoVector dir, GeoPoint2D start, GeoPoint2D end, BoundingRect domain, GeoPoint2D x0, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, domain);
            this.Rect = BoundingRect.Common(this.domain, new BoundingRect(new GeoPoint2D(start, end), System.Math.Abs(end.x - start.x), System.Math.Abs(end.y - start.y)));

            x = x0;
            if (!Rect.Contains(x))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            if (!(((end.x - start.x) / x.x) == ((end.y - start.y) / x.y)))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }

            this.direction = dir;

            this.start = start;
            this.end = end;

            ManageNewtonLine();

            uv = this.uv;
            value = this.value;
            return success;
        }
        // Continuemethods for NewtonRect. Be sure, you didn't changed x, if you do so, the derivates used for your x are the ones from the x you override
        // Plus, we don't check if x is in domain, we don't expect, you changed x
        // If you want to change x, use one of the methods above
        public bool ContinueNewtonLine(out GeoPoint2D uv, out GeoPoint value)
        {
            if (verfahren == '2')
            {
                success = NewtonLine();
                uv = this.uv;
                value = this.value;
                return success;
            }
            uv = new GeoPoint2D(6, 0);
            value = new GeoPoint(0, 0, 0);
            return false;
        }
        public bool ContinueNewtonLine(BoundingRect domain, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, domain);
            if (!domain.Contains(x))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            if (verfahren == '1')
            {
                success = NewtonLine();
                uv = this.uv;
                value = this.value;
                return success;
            }
            uv = new GeoPoint2D(6, 0);
            value = new GeoPoint(0, 0, 0);
            return false;
        }

        // Methoden für Suche nach dem Minimum auf gegebenem Rechteckt mittels SR1
        public bool GetSR1Rect(GeoVector dir, BoundingRect Rect, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, Rect);
            this.Rect = domain;
            x = Rect.GetCenter();
            this.direction = dir;

            ManageSR1Rect();

            uv = this.uv;
            value = this.value;
            return success;

        }
        public bool GetSR1Rect(GeoVector dir, BoundingRect Rect, GeoPoint2D x0, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, Rect);
            this.Rect = domain;
            if (!Rect.Contains(x0))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            this.x = x0;
            this.direction = dir;

            ManageSR1Rect();

            uv = this.uv;
            value = this.value;
            return success;
        }
        public bool GetSR1Rect(GeoVector dir, BoundingRect Rect, BoundingRect domain, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, domain);
            this.Rect = BoundingRect.Common(this.domain, Rect);

            x = Rect.GetCenter();
            this.direction = dir;

            ManageSR1Rect();

            uv = this.uv;
            value = this.value;
            return success;
        }
        public bool GetSR1Rect(GeoVector dir, BoundingRect Rect, BoundingRect domain, GeoPoint2D x0, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, domain);
            this.Rect = BoundingRect.Common(this.domain, Rect);
            if (!Rect.Contains(x0))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            this.x = x0;
            this.direction = dir;

            ManageSR1Rect();

            uv = this.uv;
            value = this.value;
            return success;
        }
        // It's strongly recommend to use these one heres only if xap is nearly x or if, that's the standard case, you change Rect/domain and want from x go further.
        // Continuemethods don't call the Managemethods. You want them twice? Call them twice.
        public bool ContinueSR1Rect(out GeoPoint2D uv, out GeoPoint value)
        {
            if (verfahren == '3')
            {
                if (!domain.Contains(xap))
                {
                    uv = new GeoPoint2D(1, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }
                surface.DerivativeAt(xap, out loctemp, out dutemp, out dvtemp);
                loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
                du2 = direction * dutemp;
                dv2 = direction * dvtemp;
                gradnorm = System.Math.Sqrt(du2 * du2 + dv2 * dv2);
                uv = this.uv;
                value = this.value;
                return SR1Rectangle();
            }
            uv = new GeoPoint2D(6, 0);
            value = new GeoPoint(0, 0, 0);
            return false;
        }
        public bool ContinueSR1Rect(GeoPoint2D xap0, out GeoPoint2D uv, GeoPoint value)
        {
            // Überprüft ob der manuell gesetzte Startpunkt zulässig ist
            if (!Rect.Contains(xap0))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            this.xap = xap0;
            return ContinueSR1Rect(out uv, out value);
        }
        public bool ContinueSR1Rect(BoundingRect domain, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(domain, surfacedomain);
            return ContinueSR1Rect(out uv, out value);
        }
        public bool ContinueSR1Rect(GeoPoint2D xap0, BoundingRect domain, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(domain, surfacedomain);
            if (!Rect.Contains(xap0))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            this.xap = xap0;
            return ContinueSR1Rect(out uv, out value);
        }

        // Methoden für Suche nach dem Minimum auf gegebener Geraden mittels SR1
        public bool GetSR1Line(GeoVector dir, GeoPoint2D start, GeoPoint2D end, out GeoPoint2D uv, out GeoPoint value)
        {
            //this.Rect = new BoundingRect(new GeoPoint2D(start.x, start.y), System.Math.Abs(end.x-start.x),System.Math.Abs(end.y-start.y));
            Rect = BoundingRect.Common(new BoundingRect(Math.Min(start.x, end.x), Math.Min(start.y, end.y), Math.Max(start.x, end.x), Math.Max(start.y, end.y)), surfacedomain);
            domain = Rect;
            x = new GeoPoint2D(start, end);
            this.start = start;
            this.end = end;
            this.direction = dir;

            ManageSR1Line();
            uv = this.uv;
            value = this.value;

            return success;
        }
        public bool GetSR1Line(GeoVector dir, GeoPoint2D start, GeoPoint2D end, GeoPoint2D x0, out GeoPoint2D uv, out GeoPoint value)
        {
            Rect = BoundingRect.Common(new BoundingRect(Math.Min(start.x, end.x), Math.Min(start.y, end.y), Math.Max(start.x, end.x), Math.Max(start.y, end.y)), surfacedomain);
            this.domain = Rect;
            x = x0;
            if (!Rect.Contains(x))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            // Testet, ob der Startpunkt auf der Geraden liegt entlang der gesucht wird.
            if (!(((end.x - start.x) / x.x) == ((end.y - start.y) / x.y)))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }

            this.direction = dir;

            ManageSR1Line();
            uv = this.uv;
            value = this.value;

            return success;
        }
        public bool GetSR1Line(GeoVector dir, GeoPoint2D start, GeoPoint2D end, BoundingRect domain, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(domain, surfacedomain);
            this.Rect = BoundingRect.Common(new BoundingRect(new GeoPoint2D(start, end), System.Math.Abs(end.x - start.x), System.Math.Abs(end.y - start.y)), this.domain);
            x = new GeoPoint2D(start, end);
            this.direction = dir;

            ManageSR1Line();
            uv = this.uv;
            value = this.value;

            return success;
        }
        public bool GetSR1Line(GeoVector dir, GeoPoint2D start, GeoPoint2D end, BoundingRect domain, GeoPoint2D x0, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(domain, surfacedomain);
            this.Rect = BoundingRect.Common(new BoundingRect(new GeoPoint2D(start, end), System.Math.Abs(end.x - start.x), System.Math.Abs(end.y - start.y)), this.domain);
            x = x0;
            if (!Rect.Contains(x))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            // Testet, ob der Startpunkt auf der gegebenen Geraden liegt.
            if (!(((end.x - start.x) / x.x) == ((end.y - start.y) / x.y)))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }

            this.direction = dir;

            ManageSR1Line();
            uv = this.uv;
            value = this.value;

            return success;
        }
        // It's strongly recommend to use these one heres only if xap is nearly x or if, that's the standard case, you change Rect/domain and want from x go further.
        // Continuemethods don't call the Managemethods.
        public bool ContinueSR1Line(out GeoPoint2D uv, out GeoPoint value)
        {
            if (verfahren == '4')
            {
                if (!domain.Contains(xap))
                {
                    uv = new GeoPoint2D(1, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }
                surface.DerivativeAt(xap, out loctemp, out dutemp, out dvtemp);
                loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
                du2 = direction * dutemp;
                dv2 = direction * dvtemp;
                gradient2 = du2 * dir.x + dv2 * dir.y;
                gradnorm = System.Math.Abs(gradient2);
                uv = this.uv;
                value = this.value;
                return SR1Line();
            }
            uv = new GeoPoint2D(6, 0);
            value = new GeoPoint(0, 0, 0);
            return false;
        }
        public bool ContinueSR1Line(GeoPoint2D xap0, out GeoPoint2D uv, GeoPoint value)
        {
            // Überprüft ob der manuell gesetzte Startpunkt zulässig ist
            if (!Rect.Contains(xap0))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            this.xap = xap0;
            return ContinueSR1Line(out uv, out value);
        }
        public bool ContinueSR1Line(BoundingRect domain, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, domain);
            return ContinueSR1Line(out uv, out value);
        }
        public bool ContinueSR1Line(GeoPoint2D xap0, BoundingRect domain, out GeoPoint2D uv, out GeoPoint value)
        {
            this.domain = BoundingRect.Common(surfacedomain, domain);
            if (!Rect.Contains(xap0))
            {
                uv = new GeoPoint2D(1, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            this.xap = xap0;
            return ContinueSR1Line(out uv, out value);
        }

        // Manager für das Newtonverfahren. Regelt Verhalten bei Fehlern und versucht diese ggfs. zu korrigieren
        // Setzt die Variable succcess. uv und value setzt das Newtonverfahren.
        // Eventuell auch hier ContinueMethoden bereitstellen! Spart ggfs. einmal ableiten doppelt berechnen!
        private void ManageNewtonRect()
        {
            tries = 0;
            success = StartNewtonRectangle();
            do
            {
                tries++;
                if (success)
                {
                    return;
                }
                else
                {
                    // Falls also praktische Schrittweite 0, wird neuer Startpunkt versucht.
                    if (uv.x == 4)
                    {
                        Setxap(DistanceToBorder() / 2);
                        x = xap;
                        success = StartNewtonRectangle();
                    }
                    else
                    {
                        if (uv.x == 1)
                        {
                            if (Stretchdomain())
                            {
                                success = NewtonRectangle();
                            }
                            else
                            {
                                success = false;
                                return;
                            }
                        }
                        // Bei anderen Fehlern wird die Schleife beendet!
                        else
                        {
                            return;
                        }
                    }
                }
            } while (tries <= maxtries);
        }
        private void ManageNewtonLine()
        {
            tries = 0;
            success = StartNewtonLine();
            do
            {
                tries++;
                if (success)
                {
                    return;
                }
                else
                {
                    // Falls also praktische Schrittweite 0, wird neuer Startpunkt versucht.
                    if (uv.x == 4)
                    {
                        Setxap(DistanceToBorder() / 2);
                        x = xap;
                        success = StartNewtonLine();
                    }
                    else
                    {
                        if (uv.x == 1)
                        {
                            if (Stretchdomain())
                            {
                                success = NewtonLine();
                            }
                            else
                            {
                                success = false;
                                return;
                            }
                        }
                        else
                        {
                            return;
                        }
                    }
                }
            } while (tries <= maxtries);
        }
        private void ManageSR1Rect()
        {
            tries = 0;
            success = StartSR1Rectangle();
            do
            {
                tries++;
                if (success)
                {
                    return;
                }
                else
                {
                    // Falls also praktische Schrittweite 0, wird neuer Startpunkt versucht.
                    if (uv.x == 4)
                    {
                        Setxap(DistanceToBorder() / 2);
                        x = xap;
                        success = StartSR1Rectangle();
                    }
                    else
                    {
                        if (uv.x == 1)
                        {
                            if (Stretchdomain())
                            {
                                if (InterpolationStepLength())
                                {
                                    success = ContinueSR1Rect(out uv, out value);
                                }
                            }
                            else
                            {
                                success = false;
                                return;
                            }
                        }
                        else
                        {
                            return;
                        }
                    }
                }
            } while (tries <= maxtries);
        }
        private void ManageSR1Line()
        {
            tries = 0;
            success = StartSR1Line();
            do
            {
                tries++;
                if (success)
                {
                    return;
                }
                else
                {
                    // Falls also praktische Schrittweite 0, wird neuer Startpunkt versucht.
                    if (uv.x == 4)
                    {
                        Setxap(DistanceToBorder() / 2);
                        x = xap;
                        success = StartSR1Line();
                    }
                    else
                    {
                        if (uv.x == 1)
                        {
                            if (Stretchdomain())
                            {
                                if (InterpolationStepLength())
                                {
                                    success = ContinueSR1Line(out uv, out value);
                                }
                            }
                            else
                            {
                                success = false;
                                return;
                            }
                        }
                        else
                        {
                            return;
                        }
                    }
                }
            } while (tries <= maxtries);
        }

        // Newtonverfahren, diese unterteilen sich wg. der Continuemethoden in zwei Teile. Der erste berechnet die Ableitung im Startpunkt. Ist die schon bekannt, kann dies übersprungen werden
        private bool StartNewtonRectangle()
        {
            verfahren = '1';

            surface.Derivative2At(x, out loctemp, out dutemp, out dvtemp, out duutemp, out dvvtemp, out duvtemp);
            loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
            du = direction * dutemp;
            dv = direction * dvtemp;
            duu = direction * duutemp;
            dvv = direction * dvvtemp;
            duv = direction * duvtemp;
            gradnorm = System.Math.Sqrt(du * du + dv * dv);
            lastgradnorm = gradnorm;
            return NewtonRectangle();
        }
        private bool NewtonRectangle()
        {
            // Newton-Verfahren mit Hessematrix-Modifikation
            // See Numerical Optimization, Nocedal Wright, p.48 ff for further information

            outrun = 0;
            stagnation = 0;

            while (gradnorm > epsilon)
            {
                // Sicherstellen, dass der kleinere Eigenwert "groß" genug ist, d.h. größer d und insbes. positiv
                l = (duu + dvv) / 2 - System.Math.Sqrt(System.Math.Pow((duu - dvv) / 2, 2) + duv * duv);
                t = System.Math.Max(0, d - l);
                duu = duu + t;
                dvv = dvv + t;
                // Weil beide Eigenwerte nun > 0 sind, ist die Determinante der mod. Hesseschen ungleich 0
                p = new GeoPoint2D(-(dvv * du - duv * dv) / (duu * dvv - duv * duv), -(duu * dv - duv * du) / ((duu * dvv - duv * duv)));
                // Setzt p auf Länge plength falls p länger ist
                if (p.x * p.x + p.y * p.y > plength)
                {
                    p = new GeoPoint2D(p.x * plength / System.Math.Sqrt(p.x * p.x + p.y * p.y), p.y * plength / System.Math.Sqrt(p.x * p.x + p.y * p.y));
                }

                if (!this.InterpolationStepLength())
                {
                    uv = new GeoPoint2D(steplength, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }
                // Falls die effektive Schrittweite 0 ist
                if (x == xap)
                {
                    uv = new GeoPoint2D(4, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

                x = xap;
                // Überprüft ob der neue Wert im geg. Definitonsbereich ist
                if (!Rect.Contains(x))
                {
                    outrun++;
                }
                if (outrun > allowedoutruns)
                {
                    uv = new GeoPoint2D(2, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

                surface.Derivative2At(x, out loctemp, out dutemp, out dvtemp, out duutemp, out dvvtemp, out duvtemp);
                loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
                du = direction * dutemp;
                dv = direction * dvtemp;
                duu = direction * duutemp;
                dvv = direction * dvvtemp;
                duv = direction * duvtemp;
                gradnorm = System.Math.Sqrt(du * du + dv * dv);
                // Hier wird sichergestellt, dass genügend Fortschritt gemacht wird
                if (!CheckChange())
                {
                    uv = new GeoPoint2D(3, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }
            }
            if (((duu + dvv) / 2 - System.Math.Sqrt(System.Math.Pow((duu - dvv) / 2, 2) + duv * duv)) <= 0)
            {
                uv = new GeoPoint2D(5, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            if (!Rect.Contains(x))
            {
                uv = new GeoPoint2D(7, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            uv = x;
            value = loctemp;
            return true;
        }
        private bool StartNewtonLine()
        {
            verfahren = '2';
            surface.Derivative2At(x, out loctemp, out dutemp, out dvtemp, out duutemp, out dvvtemp, out duvtemp);
            loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
            du = direction * dutemp;
            dv = direction * dvtemp;
            duu = direction * duutemp;
            dvv = direction * dvvtemp;
            duv = direction * duvtemp;
            gradient1 = du * dir.x + dv * dir.y;
            gradnorm = System.Math.Abs(gradient1);
            lastgradnorm = gradnorm;
            return NewtonLine();
        }
        private bool NewtonLine()
        {
            // Newton-Verfahren mit Hessematrix-Modifikation
            // See Numerical Optimization, Nocedal Wright, p.48 ff for further information
            outrun = 0;
            stagnation = 0;

            dir = new GeoVector2D(end.x - start.x, end.y - start.y).Normalized;

            while (gradnorm > epsilon)
            {
                hesse = dir.x * (dir.x * duu + dir.y * duv) + dir.y * (dir.x * duv + dir.y * dvv);
                hesse = hesse + System.Math.Max(0, d - hesse);

                p = new GeoPoint2D((-gradient1 / hesse) * dir.x, (-gradient1 / hesse) * dir.y);

                if (!this.InterpolationStepLength())
                {
                    uv = new GeoPoint2D(steplength, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

                // Falls die effektive Schrittweite 0 ist
                if (x == xap)
                {
                    uv = new GeoPoint2D(4, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

                x = xap;
                if (!Rect.Contains(x))
                {
                    outrun++;
                }
                if (outrun > allowedoutruns)
                {
                    uv = new GeoPoint2D(2, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

                surface.Derivative2At(x, out loctemp, out dutemp, out dvtemp, out duutemp, out dvvtemp, out duvtemp);
                loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
                du = direction * dutemp;
                dv = direction * dvtemp;
                duu = direction * duutemp;
                dvv = direction * dvvtemp;
                duv = direction * duvtemp;
                gradient1 = du * dir.x + dv * dir.y;
                gradnorm = System.Math.Abs(gradient1);
                if (!CheckChange())
                {
                    uv = new GeoPoint2D(3, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }
            }
            if ((dir.x * (dir.x * duu + dir.y * duv) + dir.y * (dir.x * duv + dir.y * dvv)) <= 0)
            {
                uv = new GeoPoint2D(5, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            if (!Rect.Contains(x))
            {
                uv = new GeoPoint2D(7, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            uv = x;
            value = loctemp;
            return true;
        }


        // SR1-Verfahren. Dieses unterteilt sich wg. der Continue-Methode in zwei Teile. Im ersten wird die erste Näherung bestimmt, hat man schon eine, dann kann man das eigentliche Verfahren aufrufen
        private bool StartSR1Rectangle()
        {
            verfahren = '3';
            duv = 0;

            surface.DerivativeAt(x, out loctemp, out dutemp, out dvtemp);
            loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
            du = direction * dutemp;
            dv = direction * dvtemp;

            p = new GeoPoint2D(-du, -dv);

            // The first thing we have to do is build up a first good approx. of the Hessian, see Nocedal, Wright, p142 ff.
            if (!this.InterpolationStepLength())
            {
                uv = new GeoPoint2D(steplength, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }

            ap = new GeoPoint2D(steplength * p.x, steplength * p.y);

            surface.DerivativeAt(xap, out loctemp, out dutemp, out dvtemp);
            du2 = direction * dutemp;
            dv2 = direction * dvtemp;

            // Wir testen auch mal diese Punkt ob er gut ist, falls ja, hören wir eben auf
            if (System.Math.Sqrt(du2 * du2 + dv2 * dv2) < epsilon)
            {
                uv = xap;
                value = loctemp;
                return true;
            }

            y1 = du2 - du;
            y2 = dv2 - dv;
            duu = (y1 * ap.x + y2 * ap.y) / (y1 * y1 + y2 * y2);
            dvv = duu;

            // Now we have the first Approximation. Let's make Eigenvaluecorrection
            l = (duu + dvv) / 2 - System.Math.Sqrt(System.Math.Pow((duu - dvv) / 2, 2) + duv * duv);
            if (l <= 0)
            {
                t = -l + e;
                duu = duu + t;
                dvv = dvv + t;
            }

            // Neue Richtung bestimmen, eigentlich geht's hier erst richtig los
            p = new GeoPoint2D(-(duu * du), -(dvv * dv));
            if (!this.InterpolationStepLength())
            {
                uv = new GeoPoint2D(steplength, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }

            surface.DerivativeAt(xap, out loctemp, out dutemp, out dvtemp);
            loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
            du2 = direction * dutemp;
            dv2 = direction * dvtemp;

            gradnorm = System.Math.Sqrt(du2 * du2 + dv2 * dv2);

            return SR1Rectangle();
        }
        private bool SR1Rectangle()
        {
            // SR1 or SSR1 namely Symmetric (Scaled) Rank One is Quasi-Newton-Method
            // For further Information, see Nocedal, Wright: Numerical Optimization, p. 144 ff.
            // and Hassan, Mansor, June: Convergence of a Positive-Definit Symmetric Scaled Rank One Method, in
            // Matematika, 2002, Jilid 18, bil. 2
            // Note, we use some different method: If the approxed Hessian isn't positive definit, we enforce this by Eigenvaluemodification
            // Thus we have still a descent direction and the above Paper should be applicable. Since nearly the lokal minimizer Hessian is pos. def., we expect
            // the approximation to be pos. def. to, plus we get a descent direction.
            // Note: Since we use here duu, dvv, duv, this are in truth the approxed entries of the inverse of the hessian

            outrun = 0;
            stagnation = 0;

            while (gradnorm > epsilon)
            {
                // Neue Hesse-Approx. Diese Hängt ab vom Gradient in x_k und x_(k+1) und der Schrittweite, sowieso der vorherigen
                ap = new GeoPoint2D(steplength * p.x, steplength * p.y);
                y1 = du2 - du;
                y2 = dv2 - dv;
                temp1 = ap.x - (duu * y1 + duv * y2);
                temp2 = ap.y - (duv * y1 + dvv * y2);
                temp3 = temp1 * y1 + temp2 * y2;
                // Offensichtlich wird hier geteilt, die Zahl sollte eine gewisse Mindestgröße haben, p.145 in Numerical Optimization
                if (System.Math.Abs(temp3) >= (r * System.Math.Sqrt(y1 * y1 + y2 * y2) * System.Math.Sqrt(temp1 * temp1 + temp2 * temp2)))
                {
                    duu = duu + (temp1 * temp1) / temp3;
                    duv = duv + (temp1 * temp2) / temp3;
                    dvv = dvv + (temp2 * temp2) / temp3;

                    // Sicherstellen, dass der kleinere Eigenwert "groß" genug ist, d.h. größer d und insbes. positiv
                    l = (duu + dvv) / 2 - System.Math.Sqrt(System.Math.Pow((duu - dvv) / 2, 2) + duv * duv);
                    if (l <= 0)
                    {
                        t = -l + e;
                        duu = duu + t;
                        dvv = dvv + t;
                    }
                }
                // Bestimmung der neuen Richtung und des neuen Punktes und Verwerfung des alten
                du = du2;
                dv = dv2;
                x = xap;
                p = new GeoPoint2D(-(duu * du + duv * dv), -(duv * du + dvv * dv));
                if ((p.x * p.x + p.y * p.y) > plength)
                {
                    p = new GeoPoint2D(p.x * plength / System.Math.Sqrt(p.x * p.x + p.y * p.y), p.y * plength / System.Math.Sqrt(p.x * p.x + p.y * p.y));
                }

                if (!this.InterpolationStepLength())
                {
                    uv = new GeoPoint2D(steplength, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

                // Falls die effektive Schrittweite 0 ist
                if (x == xap)
                {
                    uv = new GeoPoint2D(4, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

                // Überprüft ob der neue Wert im geg. Definitonsbereich ist
                if (!Rect.Contains(xap))
                {
                    outrun++;
                }
                if (outrun > allowedoutruns)
                {
                    x = xap;
                    uv = new GeoPoint2D(2, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

                // Bestimmt die Werte am neuen Punkt.
                surface.DerivativeAt(xap, out loctemp, out dutemp, out dvtemp);
                loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
                du2 = direction * dutemp;
                dv2 = direction * dvtemp;
                gradnorm = System.Math.Sqrt(du2 * du2 + dv2 * dv2);

                if (!CheckChange())
                {
                    x = xap;
                    uv = new GeoPoint2D(3, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

            }
            if (!Rect.Contains(xap))
            {
                uv = new GeoPoint2D(7, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            uv = xap;
            value = loctemp;
            return true;
        }
        private bool StartSR1Line()
        {
            verfahren = '4';
            dir = new GeoVector2D(end.x - start.x, end.y - start.y).Normalized;

            surface.DerivativeAt(x, out loctemp, out dutemp, out dvtemp);
            loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
            du = direction * dutemp;
            dv = direction * dvtemp;
            gradient1 = dir.x * du + dir.y * dv;

            // Testet, ob der Startpkt schon gut genug ist
            if (System.Math.Abs(gradient1) < epsilon)
            {
                uv = x;
                value = loctemp;
                return true;
            }

            p1D = -gradient1;

            p = new GeoPoint2D(-dir.x * gradient1, -dir.y * gradient1);

            if (!this.InterpolationStepLength())
            {
                uv = new GeoPoint2D(steplength, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }


            surface.DerivativeAt(xap, out loctemp, out dutemp, out dvtemp);
            du2 = direction * dutemp;
            dv2 = direction * dvtemp;
            gradient2 = dir.x * du2 + dir.y * dv2;

            // Testet, ob der Punkt vielleicht schon gut genug ist
            if (System.Math.Abs(gradient2) < epsilon)
            {
                uv = xap;
                value = loctemp;
                return true;
            }

            hesse = steplength * p1D;

            // Anpassung der Eigenwerte auf 0 < l < e
            if (hesse < 0)
            {
                hesse = e;
            }
            else
            {
                hesse = System.Math.Min(hesse, e);
            }


            p1D = -hesse * gradient1;
            p = new GeoPoint2D(dir.x * p1D, dir.y * p1D);

            if (!this.InterpolationStepLength())
            {
                uv = new GeoPoint2D(steplength, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }

            // Obiges diente soweit dazu, eine ganz gute erste Annäherung zu finden. Genaueres steht im SR1Rectangle

            surface.DerivativeAt(xap, out loctemp, out dutemp, out dvtemp);
            loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
            du2 = direction * dutemp;
            dv2 = direction * dvtemp;
            gradient2 = dir.x * du2 + dir.y * dv2;

            gradnorm = System.Math.Abs(gradient2);

            return SR1Line();
        }
        private bool SR1Line()
        {

            // SR1 or SSR1 namely Symmetric (Scaled) Rank One is Quasi-Newton-Method
            // For further Information, see Nocedal, Wright: Numerical Optimization, p. 144 ff.
            // and Hassan, Mansor, June: Convergence of a Positive-Definit Symmetric Scaled Rank One Method, in
            // Matematika, 2002, Jilid 18, bil. 2
            // Note, we use some different method: If the approxed Hessian isn't positive definit, we enforce this by Eigenvaluemodification
            // Thus we have still a descent direction and the above Paper should be applicable. Since nearly the lokal minimizer Hessian is pos. def., we expect
            // the approximation to be pos. def. to, plus we get a descent direction.
            // Note: hesse is in truth the approximation of the inverse of the hessian

            outrun = 0;
            stagnation = 0;

            while (gradnorm > epsilon)
            {
                // Neue Hesseapproximation, siehe SR1Rectangle    
                y1 = gradient2 - gradient1;
                temp1 = (steplength * p1D) - (hesse * y1);
                temp3 = temp1 * y1;
                // Offensichtlich wird nun geteilt, der Nenner sollte eine gewinne Mindesgröße besitzen.
                // Tatsächlich sollte temp3 aber nicht zu klein sein, da aber die Eigenwerte durch e beschränkt sind, darf temp3 auch nahezu 0 sein.
                if (temp3 != 0)
                {
                    // SR1-Update
                    hesse = hesse + temp1 * temp1 / temp3;

                    // Anpassung Eigenwert auf 0 < l < e
                    if (hesse < 0)
                    {
                        hesse = e;
                    }
                    else
                    {
                        hesse = System.Math.Min(hesse, e);
                    }
                }
                du = du2;
                dv = dv2;
                x = xap;
                gradient1 = gradient2;
                p1D = -hesse * gradient2;
                if (p1D > plength)
                {
                    p1D = plength * System.Math.Sign(p1D);
                }
                p = new GeoPoint2D(dir.x * p1D, dir.y * p1D);

                if (!this.InterpolationStepLength())
                {
                    uv = new GeoPoint2D(steplength, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

                // Wird de facto kein Fortschritt erziehlt, so beenden wir den Algorithmus
                if (x == xap)
                {
                    uv = new GeoPoint2D(4, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

                // Guckt, ob wir unseren vorg. Bereich verlassen haben, sollte nicht zu oft vorkommen.
                if (!Rect.Contains(xap))
                {
                    outrun++;
                }
                if (outrun > allowedoutruns)
                {
                    x = xap;
                    uv = new GeoPoint2D(2, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }

                surface.DerivativeAt(xap, out loctemp, out dutemp, out dvtemp);
                loc = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
                du2 = direction * dutemp;
                dv2 = direction * dvtemp;
                gradient2 = dir.x * du2 + dir.y * dv2;
                gradnorm = System.Math.Abs(gradient2);

                if (!CheckChange())
                {
                    x = xap;
                    uv = new GeoPoint2D(3, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }
            }
            if (!Rect.Contains(xap))
            {
                uv = new GeoPoint2D(7, 0);
                value = new GeoPoint(0, 0, 0);
                return false;
            }
            uv = xap;
            value = loctemp;
            return true;
        }

        // Schrittweitenbestimmung
        // Diese Methode setzt die Schrittweite und berechnet den Punkt xap als x+steplength*p
        private bool InterpolationStepLength()
        {
            // Numerical Optimisation, Nocedal & Wright, p. 57 ff.
            // a is the start value, typically 1. Aim is, to satisfy Armijo-Condition (see p. 33)
            // x is the actual point and p the choosen direction

            GeoPoint loctemp;

            double locx = loc;
            double abl = p.x * du + p.y * dv; // Ableitung von \Phi, geg. als Richtungsabl. des Surface in Richtung p
            double a1, a2;

            // Stellt sicher, dass die Schrittweite nicht aus dem Defintionsbereich des Surface hinausgeht.
            // Die Methode domaincheck und Newparam setzen xap und steplength
            // Die Methode setzt auch den Fehlercode der auftreten kann, dies geschieht durch setzen von steplength, dieses wird von der diese Methode aufrufenden Methode in uv.x gesetzt
            this.Newparam(a);
            if (!this.domainCheck(20))
            {
                return false;
            }
            a1 = steplength;

            loctemp = surface.PointAt(xap);
            double locxa1p = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;

            // Testet ob geg. a schon gut genug ist.
            if (locxa1p <= (locx + c * a1 * abl))
            {
                return true;
            }

            // Quadratische Interpolation, zur Bestimmung einer besseren Schrittweite
            if ((locxa1p - locx - abl * a1) == 0)
            {
                a2 = -(abl * a1 * a1) / (2 * (locxa1p - locx - abl * (a1 + (1E-010))));
            }
            else
            {
                a2 = -(abl * a1 * a1) / (2 * (locxa1p - locx - abl * a1));
            }

            // Stellt sicher, dass die Schrittweite nicht aus dem Defintionsbereich des Surface hinausgeht.
            this.Newparam(a2);
            if (!this.domainCheck(20))
            {
                return false;
            }
            a2 = this.steplength;

            loctemp = surface.PointAt(xap);
            double locxa2p = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
            if (locxa2p <= (locx + c * a2 * abl))
            {
                steplength = a2;
                return true;
            }
            double s, t, a1swap;

            // Ist die Schrittweite immernoch zu groß, so wird kubisch interpoliert, dies geschieht iterativ, bis Wolfe-Bed. erfüllt ist.
            do
            {
                // Berechnet neuen zu prüf. Wert und überschreibt die ungenutzen Var. für den nächsten Schritt
                s = (locxa2p - locx - abl * a2) / (a2 * a2 * (a2 - a1)) - (locxa1p - locx - abl * a1) / (a1 * a1 * (a2 - a1));
                // Es sollte noch dafür gesorgt werden, dass s nicht 0 wird. Idee warum das sein kann fehlt zZ.
                t = -a1 * (locxa1p - locx - abl * a2) / (a2 * a2 * (a2 - a1)) + a2 * (locxa1p - locx - abl * a1) / (a1 * a1 * (a2 - a1));
                a1swap = a1;
                a1 = a2;
                locxa1p = locxa2p;
                a2 = ((-t + System.Math.Sqrt(t * t - 3 * s * abl)) / (3 * s));

                // Stellt sicher, dass die Schrittweite nicht aus dem Defintionsbereich des Surface hinausgeht.
                this.Newparam(a2);
                if (!this.domainCheck(20))
                {
                    return false;
                }
                a2 = this.steplength;

                loctemp = surface.PointAt(xap);
                locxa2p = direction.x * loctemp.x + direction.y * loctemp.y + direction.z * loctemp.z;
            } while (locxa2p > (locx + c * a2 * abl));

            return true;
        }

        // If the gradient isn't to big this function sets new domain, if this domain is in direction bigger, it returns true, else false
        // Is the gradient to big, we return false and conclude, there's no local minimizer on Rect
        private bool Stretchdomain()
        {
            if (gradnorm <= epsilon * tolerancefactor)
            {
                temp1 = DistanceToBorder(ref domain);
                domain = BoundingRect.Common(surfacedomain, domain * domainstretchfactor); // Hier könnte noch Feintuning hin!
                // Wir überprüfen hiermit, ob die neue domain in Richtung p größer wurde. Sollte dies nicht der Fall sein, brechen wir ab.
                if (temp1 == DistanceToBorder(ref domain))
                {
                    uv = new GeoPoint2D(1, 0);
                    value = new GeoPoint(0, 0, 0);
                    return false;
                }
                return true;
            }
            uv = new GeoPoint2D(8, 0);
            value = new GeoPoint(0, 0, 0);
            return false;
        }


        // Überprüft Definitionsbereiche
        private void Newparam(double a)
        {
            this.steplength = a;
            this.xap = new GeoPoint2D(x.x + a * p.x, x.y + a * p.y);
        } // Neue Schrittweite
        private bool domainCheck(int n)
        {
            if (this.steplength == 0)
            {
                return false;
            }
            for (int i = 0; i <= n; i++)
            {
                if (domain.Contains(xap))
                {
                    return true;
                }
                else
                {
                    this.steplength = steplength / 2;
                    this.xap = new GeoPoint2D(x.x + steplength * p.x, x.y + steplength * p.y);
                }
            }
            if (domain.Contains(xap))
            {
                return true;
            }
            this.steplength = 1;
            return false;
        }

        // Hilfsmittel um mangelnde Konvergenz auszusperren
        private bool CheckChange()
        {
            if (lastgradnorm * smallchangefactor <= gradnorm)
            {
                stagnation++;
                if (stagnation > smallchange)
                {
                    return false;
                }
            }
            lastgradnorm = gradnorm;
            return true;
        }

    }

    // TODO: implement IPropertyEntry with GetPropertyEntry and protected virtual methods, remove IShowPropertyImpl
    /// <summary>
    /// Internal helper class for <see cref="ISurface"/> implementation.
    /// </summary>
    public abstract class ISurfaceImpl : ISurface, IOctTreeInsertable, IJsonSerialize
    {
        protected GeoPoint2D[] extrema; // Achtung, muss bei Modify auf null gesetzt werden
        internal BoundingRect usedArea = BoundingRect.EmptyBoundingRect;
        internal ParallelepipedHull parallelepipedHull;
        internal virtual ParallelepipedHull ParallelepipedHull
        {
            get
            {
                if (parallelepipedHull == null)
                {
                    BoundingRect ext = new BoundingRect();
                    GetNaturalBounds(out ext.Left, out ext.Right, out ext.Bottom, out ext.Top);
                    if (this is NurbsSurface)
                    {   // make sure not to exceed bound of a NURBS surface
                        NurbsSurface ns = (this as NurbsSurface);
                        if (!ns.IsUPeriodic)
                        {
                            ext.Left = Math.Max(ns.UKnots[0], ext.Left);
                            ext.Right = Math.Min(ns.UKnots[ns.UKnots.Length - 1], ext.Right);
                        }
                        if (!ns.IsVPeriodic)
                        {
                            ext.Bottom = Math.Max(ns.VKnots[0], ext.Bottom);
                            ext.Top = Math.Min(ns.VKnots[ns.VKnots.Length - 1], ext.Top);
                        }
                    }
                    else if (this is NonPeriodicSurface)
                    {
                        if (!usedArea.IsEmpty() && (ext.IsInfinite || ext.IsEmpty() || ext.IsInvalid())) ext = usedArea;
                    }
                    else if (usedArea != BoundingRect.EmptyBoundingRect)
                    {
                        // the usedArea can differ from natural bounds in periodic cases: we may not restrict to 0..2*pi!
                        //if (!ext.IsEmpty()) ext = BoundingRect.Intersect(ext, usedArea * 1.01);
                        ext = usedArea * 1.01;
                        // make it slightly bigger. This is often the extent of the Face.Area, which is sometimes not very accurate
                        // and it makes problems with Intersect
                    }
                    if (ext.IsInfinite) ext = usedArea * 1.01;
                    parallelepipedHull = new ParallelepipedHull(this, ext); // removed ext*1.01, because NURBS surfaces are not well defined outside of their bounds

                }
                return parallelepipedHull;
            }
        }


#if DEBUG
        virtual public GeoObjectList DebugGrid
        {
            get
            {
                double umin = usedArea.Left;
                double umax = usedArea.Right;
                double vmin = usedArea.Bottom;
                double vmax = usedArea.Top;
                if (usedArea == BoundingRect.EmptyBoundingRect)
                {
                    GetNaturalBounds(out umin, out umax, out vmin, out vmax);
                }
                if (umin == double.MinValue)
                {
                    if (IsUPeriodic)
                    {
                        umin = 0;
                        umax = UPeriod;
                    }
                    else
                    {
                        umin = 0;
                        umax = 100;
                    }
                }
                if (vmin == double.MinValue)
                {
                    if (IsVPeriodic)
                    {
                        vmin = 0;
                        vmax = VPeriod;
                    }
                    else
                    {
                        vmin = 0;
                        vmax = 100;
                    }

                }
                GeoObjectList res = new GeoObjectList();
                int n = 25;
                for (int i = 0; i <= n; i++)
                {   // über die Diagonale
                    GeoPoint[] pu = new GeoPoint[n + 1];
                    GeoPoint[] pv = new GeoPoint[n + 1];
                    for (int j = 0; j <= n; j++)
                    {
                        pu[j] = PointAt(new GeoPoint2D(umin + j * (umax - umin) / n, vmin + i * (vmax - vmin) / n));
                        pv[j] = PointAt(new GeoPoint2D(umin + i * (umax - umin) / n, vmin + j * (vmax - vmin) / n));
                    }
                    try
                    {
                        Polyline plu = Polyline.Construct();
                        plu.SetPoints(pu, false);
                        res.Add(plu);
                    }
                    catch (PolylineException)
                    {   // ein Pol!
                        Point pntu = Point.Construct();
                        pntu.Location = pu[0];
                        pntu.Symbol = PointSymbol.Cross;
                        res.Add(pntu);
                    }
                    try
                    {
                        Polyline plv = Polyline.Construct();
                        plv.SetPoints(pv, false);
                        res.Add(plv);
                    }
                    catch (PolylineException)
                    {
                        Point pntv = Point.Construct();
                        pntv.Location = pv[0];
                        pntv.Symbol = PointSymbol.Cross;
                        res.Add(pntv);
                    }
                }
                GeoPoint2D c2d = new GeoPoint2D((umax + umin) / 2, (vmax + vmin) / 2);
                GeoPoint c3d = PointAt(c2d);
                Line centerNormal = Line.TwoPoints(c3d, c3d + res.GetExtent().Size * 0.1 * GetNormal(c2d));
                res.Add(centerNormal);
                return res;
            }
        }
        virtual public GeoObjectList DebugDirectionsGrid
        {
            get
            {
                double umin = usedArea.Left;
                double umax = usedArea.Right;
                double vmin = usedArea.Bottom;
                double vmax = usedArea.Top;
                if (usedArea == BoundingRect.EmptyBoundingRect)
                {
                    GetNaturalBounds(out umin, out umax, out vmin, out vmax);
                }
                if (umin == double.MinValue)
                {
                    if (IsUPeriodic)
                    {
                        umin = 0;
                        umax = UPeriod;
                    }
                    else
                    {
                        umin = 0;
                        umax = 100;
                    }
                }
                if (vmin == double.MinValue)
                {
                    if (IsVPeriodic)
                    {
                        vmin = 0;
                        vmax = VPeriod;
                    }
                    else
                    {
                        vmin = 0;
                        vmax = 100;
                    }

                }
                GeoObjectList res = new GeoObjectList();
                int n = 25;
                double length = 0.0;
                for (int i = 0; i <= n; i++)
                {   // über die Diagonale
                    GeoPoint[] pu = new GeoPoint[n + 1];
                    GeoPoint[] pv = new GeoPoint[n + 1];
                    for (int j = 0; j <= n; j++)
                    {
                        pu[j] = PointAt(new GeoPoint2D(umin + j * (umax - umin) / n, vmin + i * (vmax - vmin) / n));
                        pv[j] = PointAt(new GeoPoint2D(umin + i * (umax - umin) / n, vmin + j * (vmax - vmin) / n));
                    }
                    try
                    {
                        Polyline plu = Polyline.Construct();
                        plu.SetPoints(pu, false);
                        length += plu.Length;
                    }
                    catch (PolylineException)
                    {   // ein Pol!
                    }
                    try
                    {
                        Polyline plv = Polyline.Construct();
                        plv.SetPoints(pv, false);
                        length += plv.Length;
                    }
                    catch (PolylineException)
                    {
                    }
                }
                length /= 50.0; // durchschnittliche Länge einer linie
                length /= 25.0; // durchschnittliche Maschengröße
                Attribute.ColorDef cdu = new Attribute.ColorDef("diru", Color.Red);
                Attribute.ColorDef cdv = new Attribute.ColorDef("dirv", Color.Green);
                for (int i = 0; i <= n; i++)
                {
                    for (int j = 0; j <= n; j++)
                    {
                        GeoVector diru = UDirection(new GeoPoint2D(umin + j * (umax - umin) / n, vmin + i * (vmax - vmin) / n));
                        GeoVector dirv = VDirection(new GeoPoint2D(umin + j * (umax - umin) / n, vmin + i * (vmax - vmin) / n));
                        GeoPoint loc = PointAt(new GeoPoint2D(umin + j * (umax - umin) / n, vmin + i * (vmax - vmin) / n));
                        Line l1 = Line.TwoPoints(loc, loc + length * diru.Normalized);
                        l1.ColorDef = cdu;
                        Line l2 = Line.TwoPoints(loc, loc + length * dirv.Normalized);
                        l2.ColorDef = cdv;
                        res.Add(l1);
                        res.Add(l2);
                    }
                }

                return res;
            }
        }
        virtual public Face DebugAsFace
        {
            get
            {
                BoundingRect ext = usedArea;
                if (ext == BoundingRect.EmptyBoundingRect)
                {
                    GetNaturalBounds(out ext.Left, out ext.Right, out ext.Bottom, out ext.Top);
                    if (IsUPeriodic)
                    {
                        ext.Left = 0;
                        ext.Right = UPeriod;
                    }
                    else
                    {
                        ext.Left = 0;
                        ext.Right = 100;
                    }
                    if (IsVPeriodic)
                    {
                        ext.Bottom = 0;
                        ext.Top = VPeriod;
                    }
                    else
                    {
                        ext.Bottom = 0;
                        ext.Top = 100;
                    }

                }
                return Face.MakeFace(this, new CADability.Shapes.SimpleShape(ext));
            }
        }
        static int idcounter = 0;
        public int uniqueid;
#endif
        protected ISurfaceImpl(BoundingRect? usedArea = null)
        {
#if DEBUG
            uniqueid = idcounter++;
#endif
            if (usedArea.HasValue) this.usedArea = usedArea.Value;
        }
        protected void InvalidateSecondaryData()
        {
            extrema = null;
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.FixedU (double, double, double)"/>
        /// </summary>
        /// <param name="u"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <returns></returns>
        public abstract ICurve FixedU(double u, double vmin, double vmax);
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.FixedV (double, double, double)"/>
        /// </summary>
        /// <param name="u"></param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <returns></returns>
        public abstract ICurve FixedV(double u, double umin, double umax);
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetModified (ModOp)"/>
        /// </summary>
        /// <param name="m"></param>
        /// <returns></returns>
        public abstract ISurface GetModified(ModOp m);
        //{
        //    throw new ApplicationException("GetModified must be implemented");
        //}
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.Make3dCurve (ICurve2D)"/>
        /// </summary>
        /// <param name="curve2d"></param>
        /// <returns></returns>
        public virtual ICurve Make3dCurve(CADability.Curve2D.ICurve2D curve2d)
        {
            if (curve2d is Curve2DAspect)
            {
                ICurve res = (curve2d as Curve2DAspect).Get3DCurve(this);
                if (res != null) return res;
            }
            if (curve2d is BSpline2D)
            {   // selber machen OCas liefert manchmal Unsinn
                BSpline res = BSpline.Construct();
                List<GeoPoint> points = new List<GeoPoint>();
                List<double> pos = new List<double>();
                BSpline2D b2d = curve2d as BSpline2D;
                double[] knots = b2d.Knots;
                for (int i = 0; i < knots.Length; ++i)
                {
                    GeoPoint2D uv = b2d.PointAtParam(knots[i]);
                    pos.Add(knots[i]);
                    points.Add((this as ISurface).PointAt(uv));
                }
                bool closed = Precision.IsEqual(points[0], points[points.Count - 1]);
                // bool ok = res.ThroughPoints(points.ToArray(), Math.Max(3, b2d.Degree), closed);
                bool ok = res.ThroughPoints(points.ToArray(), 3, closed);
                // TODO: hier noch mit Genauigkeit iterieren...
                if (ok)
                {
                    return res;
                }
                else
                {
                    Line l = Line.Construct();
                    l.SetTwoPoints((this as ISurface).PointAt(b2d.StartPoint), (this as ISurface).PointAt(b2d.EndPoint));
                    return l;
                }
            }
            else if (curve2d is InterpolatedDualSurfaceCurve.ProjectedCurve)
            {
                InterpolatedDualSurfaceCurve.ProjectedCurve pc = curve2d as InterpolatedDualSurfaceCurve.ProjectedCurve;
                if ((pc.IsOnSurface1 && pc.Curve3D.Surface1 == this) || (!pc.IsOnSurface1 && pc.Curve3D.Surface2 == this))
                {
                    // es kann sich nur um die ganze Curve3D handeln oder einen Teil davon
                    double pos1 = pc.Curve3D.PositionOf(PointAt(pc.StartPoint));
                    double pos2 = pc.Curve3D.PositionOf(PointAt(pc.EndPoint));
                    if (pos1 >= 0.0 && pos1 <= 1.0 && pos2 >= 0.0 && pos2 <= 1.0)
                    {
                        bool reversed = false;
                        if (pos2 < pos1)
                        {
                            reversed = true;
                            double tmp = pos1;
                            pos1 = pos2;
                            pos2 = tmp;
                        }
                        ICurve res = pc.Curve3D.Clone() as ICurve;
                        res.Trim(pos1, pos2);
                        if (reversed) res.Reverse();
                        return res;
                    }
                }
            }
            // kein else, sondern das folgende ist der Notfall, wenn sonst nichts greift
            {
                if (curve2d is Line2D) // dieser Text könnte eigentlich in der Basismethode stehen
                {
                    if (Math.Abs(curve2d.StartDirection.x) < Precision.eps)
                    {
                        return FixedU(curve2d.StartPoint.x, curve2d.StartPoint.y, curve2d.EndPoint.y);
                    }
                    else if (Math.Abs(curve2d.StartDirection.y) < Precision.eps)
                    {
                        return FixedV(curve2d.StartPoint.y, curve2d.StartPoint.x, curve2d.EndPoint.x);
                    }
                }

                // hier brachial mit einer gewissen Anzahl von Punkten
                int n = 10; // einfach mal so, muss man ggf. ändern
                GeoPoint[] pnts = new GeoPoint[n + 1];

                for (int i = 0; i < n + 1; i++)
                {
                    pnts[i] = (this as ISurface).PointAt(curve2d.PointAt((double)i / (double)n));
                }
                bool closed = Precision.IsEqual(pnts[0], pnts[pnts.Length - 1]);
                BSpline res = BSpline.Construct();
                bool ok = res.ThroughPoints(pnts, 3, closed);
                // TODO: hier noch mit Genauigkeit iterieren...
                if (ok)
                {
                    return res;
                }
                else
                {
                    Line l = Line.Construct();
                    l.SetTwoPoints((this as ISurface).PointAt(curve2d.StartPoint), (this as ISurface).PointAt(curve2d.EndPoint));
                    return l;
                }
            }
            //CndHlp3D.Surface sf = Helper;
            //GeneralCurve2D g2d = curve2d as GeneralCurve2D;
            //CndHlp3D.Edge edge = sf.Make3DCurve(g2d.Entity2D);
            //return IGeoObjectImpl.FromHlp3DEdge(edge) as ICurve;
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetNormal (GeoPoint2D)"/>
        /// </summary>
        /// <param name="uv"></param>
        /// <returns></returns>
        public virtual GeoVector GetNormal(GeoPoint2D uv)
        {
#if DEBUG
            //GeoVector du = new GeoVector(
            //    Differentiate.FirstDerivative(x => PointAt(new GeoPoint2D(x, uv.y)).x, uv.x),
            //    Differentiate.FirstDerivative(x => PointAt(new GeoPoint2D(x, uv.y)).y, uv.x),
            //    Differentiate.FirstDerivative(x => PointAt(new GeoPoint2D(x, uv.y)).z, uv.x));
            //GeoVector dv = new GeoVector(
            //    Differentiate.FirstDerivative(y => PointAt(new GeoPoint2D(uv.x, y)).x, uv.y),
            //    Differentiate.FirstDerivative(y => PointAt(new GeoPoint2D(uv.x, y)).y, uv.y),
            //    Differentiate.FirstDerivative(y => PointAt(new GeoPoint2D(uv.x, y)).z, uv.y));
            //SweepAngle au = new SweepAngle(du, UDirection(uv));
            //SweepAngle av = new SweepAngle(dv, VDirection(uv));
            //if (Math.Abs(au) > 0.1 || Math.Abs(av) > 0.1)
            //{

            //}
#endif
            return (this as ISurface).UDirection(uv) ^ (this as ISurface).VDirection(uv);
            // return new GeoVector(Helper.GetNormal(uv.ToCndHlp()));
        }
        public abstract GeoVector UDirection(GeoPoint2D uv);
        public abstract GeoVector VDirection(GeoPoint2D uv);
        public abstract GeoPoint PointAt(GeoPoint2D uv);
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.PositionOf (GeoPoint)"/>
        /// </summary>
        /// <param name="p"></param>
        /// <returns></returns>
        public virtual GeoPoint2D PositionOf(GeoPoint p)
        {
            GeoPoint2D res;
            if (ParallelepipedHull.PositionOf(p, out res))
            {
                return res;
            }
            else
            {

                // hat nicht konvergiert, der Punkt liegt möglicherweise
                // knapp außerhalb der NaturalBounds
                double umin, umax, vmin, vmax;
                GetNaturalBounds(out umin, out umax, out vmin, out vmax);
                double[] us = GetUSingularities();
                double[] vs = GetVSingularities();
                double minerror = double.MaxValue;
                res = GeoPoint2D.Origin;
                for (int i = 0; i < us.Length; i++)
                {
                    GeoPoint pole = PointAt(new GeoPoint2D(us[i], (vmax + vmin) / 2.0));
                    double d = pole | p;
                    if (d < minerror)
                    {
                        minerror = d;
                        res = new GeoPoint2D(us[i], (vmax + vmin) / 2.0);
                    }
                }
                for (int i = 0; i < vs.Length; i++)
                {
                    GeoPoint pole = PointAt(new GeoPoint2D((umax + umin) / 2.0, vs[i]));
                    double d = pole | p;
                    if (d < minerror)
                    {
                        minerror = d;
                        res = new GeoPoint2D((umax + umin) / 2.0, vs[i]);
                    }
                }
                if (umin != double.MinValue && vmin != double.MinValue) // das sollte als Gültigkeitsabfrage genügen
                {
                    double u, v, d;
                    GeoPoint2D p2d;
                    GeoPoint p0;
                    if (us.Length == 0 || Math.Abs(us[0] - umin) > 1e-6) // not a pole
                    {
                        v = FixedU(umin, vmin, vmax).PositionOf(p);
                        v = vmin + v * (vmax - vmin);
                        p2d = new GeoPoint2D(umin, v);
                        p0 = (this as ISurface).PointAt(p2d);
                        d = p0 | p;
                        if (d < minerror)
                        {
                            minerror = d;
                            res = p2d;
                        }
                    }
                    if (us.Length == 0 || Math.Abs(us[us.Length - 1] - umax) > 1e-6) // not a pole
                    {
                        v = FixedU(umax, vmin, vmax).PositionOf(p);
                        v = vmin + v * (vmax - vmin);
                        p2d = new GeoPoint2D(umax, v);
                        p0 = (this as ISurface).PointAt(p2d);
                        d = p0 | p;
                        if (d < minerror)
                        {
                            minerror = d;
                            res = p2d;
                        }
                    }
                    if (vs.Length == 0 || Math.Abs(vs[0] - vmin) > 1e-6) // not a pole
                    {
                        u = FixedV(vmin, umin, umax).PositionOf(p);
                        u = umin + u * (umax - umin);
                        p2d = new GeoPoint2D(u, vmin);
                        p0 = (this as ISurface).PointAt(p2d);
                        d = p0 | p;
                        if (d < minerror)
                        {
                            minerror = d;
                            res = p2d;
                        }
                    }
                    if (vs.Length == 0 || Math.Abs(vs[vs.Length - 1] - vmax) > 1e-6) // not a pole
                    {
                        u = FixedV(vmax, umin, umax).PositionOf(p);
                        u = umin + u * (umax - umin);
                        p2d = new GeoPoint2D(u, vmax);
                        p0 = (this as ISurface).PointAt(p2d);
                        d = p0 | p;
                        if (d < minerror)
                        {
                            minerror = d;
                            res = p2d;
                        }
                    }
                    // hier liegt res auf einer Kante. In diesem Punkt legen wir jetzt die Tangentialebene an
                    // und bestimmen den Punkt auf dieser Ebene. Sonst kleben die Punkte außerhalb immer an der
                    // Kante und das ist schlecht für GetProjectedCurve
                    GeoVector dirx;
                    GeoVector diry;
                    GeoPoint loc;
                    this.DerivativeAt(res, out loc, out dirx, out diry);
                    Matrix mtx = DenseMatrix.OfRowArrays(dirx, diry, dirx ^ diry);
                    Vector b = new DenseVector(p - loc);
                    if (!Precision.IsNullVector(dirx) && !Precision.IsNullVector(diry))
                    {
                        Vector x = (Vector)mtx.Transpose().Solve(b);
                        if (x.IsValid())
                        {
                            GeoPoint2D res1 = new GeoPoint2D(res.x + x[0], res.y + x[1]);
                            double du = umax - umin;
                            double dv = vmax - vmin;
                            if (res1.x >= umin - du / 2.0 && res1.x <= umax + du / 2.0 && res1.y >= vmin - dv / 2.0 && res1.y <= vmax + dv / 2.0) res = res1;
                            // res = res1;
                            // ACHTUNG: den Punkt nicht künstlich in die Grenzen drücken aber nicht zu weit entfernt
                        }
                    }
                }
                return res;
            }
            // return new GeoPoint2D(Helper.PositionOf(p.ToCndHlp()));
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.DerivativeAt (GeoPoint2D, out GeoPoint, out GeoVector, out GeoVector)"/>
        /// </summary>
        /// <param name="uv"></param>
        /// <param name="location"></param>
        /// <param name="du"></param>
        /// <param name="dv"></param>
        public virtual void DerivativeAt(GeoPoint2D uv, out GeoPoint location, out GeoVector du, out GeoVector dv)
        {
            location = (this as ISurface).PointAt(uv);
            du = (this as ISurface).UDirection(uv);
            dv = (this as ISurface).VDirection(uv);
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.Derivative2At (GeoPoint2D, out GeoPoint, out GeoVector, out GeoVector, out GeoVector, out GeoVector, out GeoVector)"/> numerically. If possible, implement a derivative
        /// </summary>
        /// <param name="uv"></param>
        /// <param name="location"></param>
        /// <param name="du"></param>
        /// <param name="dv"></param>
        /// <param name="duu"></param>
        /// <param name="dvv"></param>
        /// <param name="duv"></param>
        public virtual void Derivative2At(GeoPoint2D uv, out GeoPoint location, out GeoVector du, out GeoVector dv, out GeoVector duu, out GeoVector dvv, out GeoVector duv)
        {
            BoundingRect uvminmax = usedArea;
            double hu, hv;
            if (!uvminmax.IsInvalid() && !uvminmax.IsEmpty() && !uvminmax.IsInfinite)
            {
                hu = 1e-6 * (uvminmax.Width);
                hv = 1e-6 * (uvminmax.Height);
            }
            else
            {
                hu = 1e-6; hv = 1e-6;
            }
            // 1) location + 1. Ableitungen analytisch
            location = PointAt(uv);
            du = UDirection(uv);
            dv = VDirection(uv);

            // 2) 2. Ableitungen numerisch (Punkt-basiert)

            GeoPoint SuP = PointAt(new GeoPoint2D(uv.x + hu, uv.y));
            GeoPoint SuM = PointAt(new GeoPoint2D(uv.x - hu, uv.y));
            GeoPoint SvP = PointAt(new GeoPoint2D(uv.x, uv.y + hv));
            GeoPoint SvM = PointAt(new GeoPoint2D(uv.x, uv.y - hv));

            duu = (1 / (hu * hu)) * (SuP.ToVector() - 2 * location.ToVector() + SuM.ToVector());
            dvv = (1 / (hu * hu)) * (SvP.ToVector() - 2 * location.ToVector() + SvM.ToVector());

            GeoPoint SuvPP = PointAt(new GeoPoint2D(uv.x + hu, uv.y + hv));
            GeoPoint SuvPM = PointAt(new GeoPoint2D(uv.x + hu, uv.y - hv));
            GeoPoint SuvMP = PointAt(new GeoPoint2D(uv.x - hu, uv.y + hv));
            GeoPoint SuvMM = PointAt(new GeoPoint2D(uv.x - hu, uv.y - hv));

            duv = (SuvPP.ToVector() - SuvPM.ToVector() - SuvMP.ToVector() + SuvMM.ToVector()) / (4 * hu * hv);
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetPlaneIntersection (PlaneSurface, double, double, double, double, double)"/>
        /// </summary>
        /// <param name="pl"></param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <param name="precision"></param>
        /// <returns></returns>
        public virtual IDualSurfaceCurve[] GetPlaneIntersection(PlaneSurface pl, double umin, double umax, double vmin, double vmax, double precision)
        {
            return ParallelepipedHull.GetPlaneIntersection(pl, umin, umax, vmin, vmax, precision);
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetLineIntersection (GeoPoint, GeoVector)"/>
        /// </summary>
        /// <param name="startPoint"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        public virtual GeoPoint2D[] GetLineIntersection(GeoPoint startPoint, GeoVector direction)
        {
            Polynom impl = GetImplicitPolynomial();
            if (impl != null)
            {
                List<GeoPoint2D> res = new List<GeoPoint2D>();
                Polynom toSolve = impl.Substitute(new Polynom(direction.x, "u", startPoint.x, ""), new Polynom(direction.y, "u", startPoint.y, ""), new Polynom(direction.z, "u", startPoint.z, ""));
                double[] roots = toSolve.Roots();
                for (int i = 0; i < roots.Length; i++)
                {
                    GeoPoint p1 = startPoint + roots[i] * direction;
                    res.Add(PositionOf(p1));
                }
                return res.ToArray();
            }
            return ParallelepipedHull.GetLineIntersection(startPoint, direction);
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetSafeParameterSteps (double, double, double, double, out double[], out double[])"/>
        /// </summary>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <param name="intu"></param>
        /// <param name="intv"></param>
        public virtual void GetSafeParameterSteps(double umin, double umax, double vmin, double vmax, out double[] intu, out double[] intv)
        {
            int n = 4;
            intu = new double[n];
            intv = new double[n];
            for (int i = 0; i < n; i++)
            {
                intu[i] = umin + i * (umax - umin) / (n - 1);
                intv[i] = vmin + i * (vmax - vmin) / (n - 1);
            }
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetTangentCurves (GeoVector, double, double, double, double)"/>
        /// </summary>
        /// <param name="direction"></param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <returns></returns>
        public virtual ICurve2D[] GetTangentCurves(GeoVector direction, double umin, double umax, double vmin, double vmax)
        {
            FindTangentCurves ftc = new FindTangentCurves(this);
            return ftc.GetTangentCurves(direction, umin, umax, vmin, vmax);
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.IsVanishingProjection (Projection, double, double, double, double)"/>
        /// </summary>
        /// <param name="p"></param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <returns></returns>
        public virtual bool IsVanishingProjection(Projection p, double umin, double umax, double vmin, double vmax)
        {
            return false;
        }
        public virtual bool IsUPeriodic
        {
            get
            {
                throw new NotImplementedException();
            }
        }
        public virtual bool IsVPeriodic
        {
            get
            {
                throw new NotImplementedException();
            }
        }
        public virtual double UPeriod
        {
            get
            {
                if (!IsUPeriodic) return 0.0;
                throw new NotImplementedException();
            }
        }
        public virtual double VPeriod
        {
            get
            {
                if (!IsVPeriodic) return 0.0;
                throw new NotImplementedException();
            }
        }
        public virtual bool IsUClosed
        {
            get
            {
                return IsUPeriodic;
            }
        }
        public virtual bool IsVClosed
        {
            get
            {
                return IsVPeriodic;
            }
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetUSingularities ()"/>
        /// </summary>
        /// <returns></returns>
        public virtual double[] GetUSingularities()
        {
            return new double[0];
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetVSingularities ()"/>
        /// </summary>
        /// <returns></returns>
        public virtual double[] GetVSingularities()
        {
            return new double[0];
        }

        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.MakeFace (SimpleShape)"/>
        /// </summary>
        /// <param name="simpleShape"></param>
        /// <returns></returns>
        public virtual Face MakeFace(CADability.Shapes.SimpleShape simpleShape)
        {
            Face res = Face.Construct();
            Edge[] outline = new Edge[simpleShape.Outline.Segments.Length];
            for (int i = 0; i < outline.Length; i++)
            {
                outline[i] = new Edge(res, Make3dCurve(simpleShape.Outline.Segments[i]), res, simpleShape.Outline.Segments[i], true);
            }
            Edge[][] holes = new Edge[simpleShape.NumHoles][];
            for (int j = 0; j < holes.Length; j++)
            {
                holes[j] = new Edge[simpleShape.Hole(j).Segments.Length];
                for (int i = 0; i < holes[j].Length; i++)
                {
                    holes[j][i] = new Edge(res, Make3dCurve(simpleShape.Hole(j).Segments[i]), res, simpleShape.Hole(j).Segments[i], true);
                }
            }
            res.Set(this, outline, holes);
            return res;
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetZMinMax (Projection, double, double, double, double, ref double, ref double)"/>
        /// </summary>
        /// <param name="p"></param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <param name="zMin"></param>
        /// <param name="zMax"></param>
        public virtual void GetZMinMax(Projection p, double umin, double umax, double vmin, double vmax, ref double zMin, ref double zMax)
        {
            throw new NotImplementedException("GetZMinMax must be implemented by derived surface");
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.MakeCanonicalForm ()"/>
        /// </summary>
        /// <returns></returns>
        public virtual ModOp2D MakeCanonicalForm()
        {
            return ModOp2D.Identity;
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.Clone ()"/>
        /// </summary>
        /// <returns></returns>
        public virtual ISurface Clone()
        {
            throw new NotImplementedException("Clone must be implemented by derived surface");
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.Modify (ModOp)"/>
        /// </summary>
        /// <param name="m"></param>
        public virtual void Modify(ModOp m)
        {
            throw new NotImplementedException("Modify must be implemented by derived surface");
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.CopyData (ISurface)"/>
        /// </summary>
        /// <param name="CopyFrom"></param>
        public virtual void CopyData(ISurface CopyFrom)
        {
            throw new NotImplementedException("CopyData must be implemented by derived surface");
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.Approximate (double, double, double, double, double)"/>
        /// </summary>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <param name="precision"></param>
        /// <returns></returns>
        public virtual NurbsSurface Approximate(double umin, double umax, double vmin, double vmax, double precision)
        {
            throw new NotImplementedException();
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetProjectedCurve (ICurve, double)"/>
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="precision"></param>
        /// <returns></returns>
        public virtual ICurve2D GetProjectedCurve(ICurve curve, double precision)
        {   // ganz schlecht, unbedingt in die Objekte verlegen und StandardKurven erkennen
            if (curve is Polyline)
            {
                Polyline pl = curve as Polyline;
                List<ICurve2D> res = new List<ICurve2D>();
                GeoPoint2D lastEndPoint = GeoPoint2D.Invalid;
                for (int i = 0; i < pl.Vertices.Length - 1; i++)
                {
                    Line ln = Line.Construct();
                    ln.SetTwoPoints(pl.Vertices[i], pl.Vertices[i + 1]);
                    ICurve2D c2d = GetProjectedCurve(ln, precision);
                    if (c2d != null)
                    {
                        if (lastEndPoint.IsValid) SurfaceHelper.AdjustPeriodicStartPoint(this, lastEndPoint, c2d);
                        lastEndPoint = c2d.EndPoint;
                        res.Add(c2d);
                    }
                }
                return new Path2D(res.ToArray());
            }
            if (curve is InterpolatedDualSurfaceCurve)
            {
                if (this == (curve as InterpolatedDualSurfaceCurve).Surface1) // oder besser geometrische Gleichheit prüfen
                {
                    return (curve as InterpolatedDualSurfaceCurve).CurveOnSurface1;
                }
                else if (this == (curve as InterpolatedDualSurfaceCurve).Surface2)
                {
                    return (curve as InterpolatedDualSurfaceCurve).CurveOnSurface2;
                }
                // there is a bug with SameGeometry and modifications. we use normal ProjectedCurve instead
                //ModOp2D firstToSecond;
                //if (this.SameGeometry(this.usedArea, (curve as InterpolatedDualSurfaceCurve).Surface1, ((curve as InterpolatedDualSurfaceCurve).Surface1 as ISurfaceImpl).usedArea, precision, out firstToSecond)) // oder besser geometrische Gleichheit prüfen
                //{
                //    if (firstToSecond.IsAlmostIdentity(precision)) return (curve as InterpolatedDualSurfaceCurve).CurveOnSurface1;
                //    else if (!firstToSecond.IsNull) return (curve as InterpolatedDualSurfaceCurve).CurveOnSurface1.GetModified(firstToSecond); // ist die ModOp so richtigrum?
                //}
                //else if (this.SameGeometry(this.usedArea, (curve as InterpolatedDualSurfaceCurve).Surface2, ((curve as InterpolatedDualSurfaceCurve).Surface2 as ISurfaceImpl).usedArea, precision, out firstToSecond)) // oder besser geometrische Gleichheit prüfen
                //{
                //    if (firstToSecond.IsAlmostIdentity(precision)) return (curve as InterpolatedDualSurfaceCurve).CurveOnSurface2;
                //    else if (!firstToSecond.IsNull) return (curve as InterpolatedDualSurfaceCurve).CurveOnSurface2.GetModified(firstToSecond); // ist die ModOp so richtigrum?
                //}
            }
            if (!IsUPeriodic && !IsVPeriodic)
            {
                BoundingRect restricted = BoundingRect.EmptyBoundingRect;
                if (usedArea.Left > double.MinValue && usedArea.Right < double.MaxValue && usedArea.Bottom > double.MinValue && usedArea.Top < double.MaxValue && !usedArea.IsEmpty())
                {
                    restricted = usedArea;
                }
                else
                {
                    GetNaturalBounds(out double umin, out double umax, out double vmin, out double vmax);
                    if (umin > double.MinValue && umax < double.MaxValue && vmin > double.MinValue && vmax < double.MaxValue) restricted = new BoundingRect(umin, vmin, umax, vmax);
                }
                return new ProjectedCurve(curve, this, true, restricted, precision);
            }
            if (usedArea.IsInfinite)
                return new ProjectedCurve(curve, this, true, BoundingRect.EmptyBoundingRect, precision);
            else
                return new ProjectedCurve(curve, this, true, usedArea, precision);

        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.Intersect (ICurve, BoundingRect, out GeoPoint[], out GeoPoint2D[], out double[])"/>
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="uvExtent"></param>
        /// <param name="ips"></param>
        /// <param name="uvOnFaces"></param>
        /// <param name="uOnCurve3Ds"></param>
        public virtual void Intersect(ICurve curve, BoundingRect uvExtent, out GeoPoint[] ips, out GeoPoint2D[] uvOnFaces, out double[] uOnCurve3Ds)
        {   // implement special cases with their surfaces
            if (curve is IDualSurfaceCurve dsc)
            {   // when it is a dualSurfaceCurve on an offset surface, there will be no intersection
                if (IsOffset(dsc.Surface1, out double offset) || IsOffset(dsc.Surface2, out offset))
                {
                    ips = []; uvOnFaces = []; uOnCurve3Ds = [];
                    return;
                }
            }
            if (curve is InterpolatedDualSurfaceCurve || curve is BSpline)
            {
                //for some curves it is alot faster to use the TetraederHull for intersection.
                // it is typically much slimmer than the ParallelepipedHull
                if (curve is GeneralCurve gc)
                {
                    gc.TetraederHull.Intersect(this, uvExtent, out ips, out uvOnFaces, out uOnCurve3Ds);
                    return;
                }
                else if (curve is BSpline bsp)
                {
                    bsp.TetraederHull.Intersect(this, uvExtent, out ips, out uvOnFaces, out uOnCurve3Ds);
                    return;
                }
            }
            ParallelepipedHull.Intersect(curve, uvExtent, out ips, out uvOnFaces, out uOnCurve3Ds);
        }

        private bool IsOffset(ISurface other, out double offset)
        {
            offset = 0.0;
            if (this.GetType() != other.GetType()) return false;
            if (this is CylindricalSurface cs1 && other is CylindricalSurface cs2)
            {
                if (Precision.SameAxis(new Axis(cs1.Location, cs1.Axis), new Axis(cs2.Location, cs2.Axis)))
                {
                    return true; // offset should be implemented but is currently not used
                }
            }
            if (this is ConicalSurface cns1 && other is ConicalSurface cns2)
            {
                if (Precision.SameAxis(new Axis(cns1.Location, cns1.Axis), new Axis(cns2.Location, cns2.Axis)))
                {
                    return true; // offset should be implemented but is currently not used
                }
            }
            if (this is SphericalSurface ss1 && other is SphericalSurface ss2)
            {
                if (Precision.IsEqual(ss1.Location, ss2.Location))
                {
                    return true; // offset should be implemented but is currently not used
                }
            }
            if (this is ToroidalSurface ts1 && other is ToroidalSurface ts2)
            {
                if (Precision.SameAxis(new Axis(ts1.Location, ts1.Axis), new Axis(ts2.Location, ts2.Axis)) &&
                    Precision.IsEqual(ts1.MajorRadius, ts2.MajorRadius))
                {
                    return true; // offset should be implemented but is currently not used
                }
            }
            return false;
        }

        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.Intersect (BoundingRect, ISurface, BoundingRect)"/>
        /// </summary>
        /// <param name="thisBounds"></param>
        /// <param name="other"></param>
        /// <param name="otherBounds"></param>
        /// <returns></returns>
        public virtual ICurve[] Intersect(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds)
        {
            GetExtremePositions(thisBounds, other, otherBounds, out List<Tuple<double, double, double, double>> extremePositions);
            if (usedArea.IsEmpty() || usedArea.IsInfinite) usedArea = thisBounds;
            return ParallelepipedHull.Intersect(thisBounds, other, otherBounds, null, extremePositions);
        }
        public virtual ICurve Intersect(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, GeoPoint seed)
        {
            if (usedArea.IsEmpty()) usedArea = thisBounds;
            ICurve[] sol = ParallelepipedHull.Intersect(thisBounds, other, otherBounds, new List<GeoPoint>(new GeoPoint[] { seed }));
            if (sol == null || sol.Length == 0) sol = Intersect(thisBounds, other, otherBounds);
            for (int i = 0; i < sol.Length; i++)
            {
                if (sol[i].DistanceTo(seed) < Precision.eps) return sol[i];
            }
            return null;
        }

        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.ReverseOrientation ()"/>
        /// </summary>
        /// <returns></returns>
        public virtual ModOp2D ReverseOrientation()
        {
            throw new NotImplementedException("ReverseOrientation must be implemented");
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.SameGeometry (BoundingRect, ISurface, BoundingRect, double, out ModOp2D)"/>
        /// </summary>
        /// <param name="thisBounds"></param>
        /// <param name="other"></param>
        /// <param name="otherBounds"></param>
        /// <param name="precision"></param>
        /// <param name="firstToSecond"></param>
        /// <returns></returns>
        public virtual bool SameGeometry(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, double precision, out ModOp2D firstToSecond)
        {
            // hier bräuchte man die allgemeine triangulierung oder so ...
            if (this.GetType() != other.GetType())
            {
                firstToSecond = ModOp2D.Null;
                return false;
            }
            return Surfaces.Overlapping(this, thisBounds, other, otherBounds, precision, out firstToSecond);
        }
        public virtual double IsParallel(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds)
        {   // should be implemented for standard surfaces
            return double.MaxValue;
        }

        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetOffsetSurface (double)"/>
        /// </summary>
        /// <param name="offset"></param>
        /// <returns></returns>
        public virtual ISurface GetOffsetSurface(double offset)
        {
            return new OffsetSurface(this, offset);
        }
        public virtual ISurface GetOffsetSurface(double offset, out ModOp2D mod)
        {
            mod = ModOp2D.Null;
            return null;
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetNaturalBounds (out double, out double, out double, out double)"/>
        /// </summary>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        public virtual void GetNaturalBounds(out double umin, out double umax, out double vmin, out double vmax)
        {
            if (usedArea.IsEmpty())
            {
                umin = double.MinValue;
                umax = double.MaxValue;
                vmin = double.MinValue;
                vmax = double.MaxValue;
            }
            else
            {
                umin = usedArea.Left;
                umax = usedArea.Right;
                vmin = usedArea.Bottom;
                vmax = usedArea.Top;
            }
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.HitTest (BoundingBox, double, double, double, double)"/>
        /// </summary>
        /// <param name="cube"></param>
        /// <param name="umin"></param>
        /// <param name="umax"></param>
        /// <param name="vmin"></param>
        /// <param name="vmax"></param>
        /// <returns></returns>
        public virtual bool HitTest(BoundingBox cube, double umin, double umax, double vmin, double vmax)
        {
            throw new NotImplementedException("HitTest must be implemented");
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.HitTest (BoundingBox, out GeoPoint2D)"/>
        /// </summary>
        /// <param name="cube"></param>
        /// <param name="uv"></param>
        /// <returns></returns>
        public virtual bool HitTest(BoundingBox cube, out GeoPoint2D uv)
        {
            Polynom implicitSurface = GetImplicitPolynomial();
            uv = GeoPoint2D.Origin;
            if (implicitSurface != null)
            {
                double cntDist = implicitSurface.Eval(cube.GetCenter());
                // if (cube.DiagonalLength / 2 < Math.Abs(cntDist)) return false; // the cube is too far away from the surface
                // the distance only works for certain surfaces (plane, sphere, cylinder, not for torus, cone)
                int cntSign = Math.Sign(cntDist);
                GeoPoint[] vertices = cube.Points;
                int[] vertexSign = new int[vertices.Length];
                bool differentSign = false;
                for (int i = 0; i < vertices.Length; i++)
                {
                    vertexSign[i] = Math.Sign(implicitSurface.Eval(vertices[i]));
                    differentSign |= vertexSign[i] != cntSign;
                }
                int[,] ln = cube.LineNumbers;
                if (differentSign)
                {   // there must be at least 3 edges of the cube passing through the surface
                    // we should return them all, but the parameter allows only one
                    for (int i = 0; i < 12; i++)
                    {
                        if (vertexSign[ln[i, 0]] != vertexSign[ln[i, 1]])
                        {   // the line from vertices[ln[i,0]] to vertices[ln[i,1]] intersects the surface.
                            GeoPoint startPoint = vertices[ln[i, 0]];
                            GeoVector direction = vertices[ln[i, 1]] - startPoint;
                            Polynom toSolve = implicitSurface.Substitute(Polynom.Line3d(startPoint, direction));
                            double[] roots = toSolve.Roots();
                            for (int j = 0; j < roots.Length; j++)
                            {
                                if (roots[j] >= 0 && roots[j] <= 1)
                                {
                                    GeoPoint p = startPoint + roots[j] * direction;
                                    uv = PositionOf(p);
                                    return true;
                                }
                            }

                        }
                    }
                }
                else
                {
                    // no cube edge intersects the surface (an odd number of times) and the surface is close to the cube
                    // test all edges of the cube against the surface
                    for (int i = 0; i < 12; i++)
                    {
                        GeoPoint startPoint = vertices[ln[i, 0]];
                        GeoVector direction = vertices[ln[i, 1]] - startPoint;
                        Polynom toSolve = implicitSurface.Substitute(Polynom.Line3d(startPoint, direction));
                        double[] roots = toSolve.Roots();
                        for (int j = 0; i < roots.Length; i++)
                        {
                            if (roots[j] >= 0 && roots[j] <= 1)
                            {
                                GeoPoint p = startPoint + roots[j] * direction;
                                uv = PositionOf(p);
                                return true;
                            }
                        }
                    }
                }
#if DEBUG
                // find a plane which separates the surface and the cube
                // doesn't help
                //PolynomVector normalPolynom = new PolynomVector(implicitSurface.Derivate(1, 0, 0), implicitSurface.Derivate(0, 1, 0), implicitSurface.Derivate(0, 0, 1));
                //GeoPoint center = cube.GetCenter();
                //GeoVector normal = normalPolynom.Eval(center);
                //normal.ArbitraryNormals(out GeoVector dirx, out GeoVector diry);
                //Polynom plane = implicitSurface.Substitute(new Polynom(center.x, "", dirx.x, "u", diry.x, "v"), new Polynom(center.y, "", dirx.y, "u", diry.y, "v"), new Polynom(center.z, "", dirx.z, "u", diry.z, "v"));
                //double[] uminmax = plane.Derivate(1, 0).Roots();
                //double[] vminmax = plane.Derivate(0, 1).Roots();
#endif
                // no cube edge intersects the surface any number of times
                // Maybe a sphere or a torus touch the cube without intersection one of it's edges
                GeoPoint2D[] extrema = this.GetExtrema(); // the extrema in axis direction
                for (int i = 0; i < extrema.Length; i++)
                {
                    if (cube.Contains(PointAt(extrema[i])))
                    {
                        uv = extrema[i];
                        return true;
                    }
                }
                // a torus may pass through the cube without interfering with the edges: check the foot points from the center
                GeoPoint2D[] footPoints = this.PerpendicularFoot(cube.GetCenter());
                for (int i = 0; i < footPoints.Length; i++)
                {
                    if (cube.Contains(PointAt(footPoints[i])))
                    {
                        uv = footPoints[i];
                        return true;
                    }
                }
                return false;
            }
            return this.ParallelepipedHull.HitTest(cube, out uv);
        }
        private bool DebugNewHitTest(BoundingBox cube, out GeoPoint2D uv)
        {
            Polynom implicitSurface = GetImplicitPolynomial();
            uv = GeoPoint2D.Origin;
            if (implicitSurface != null)
            {
                double cntDist = implicitSurface.Eval(cube.GetCenter());
                if (cube.DiagonalLength / 2 < Math.Abs(cntDist)) return false; // the cube is too far away from the surface
                int cntSign = Math.Sign(cntDist);
                GeoPoint[] vertices = cube.Points;
                int[] vertexSign = new int[vertices.Length];
                bool differentSign = false;
                for (int i = 0; i < vertices.Length; i++)
                {
                    vertexSign[i] = Math.Sign(implicitSurface.Eval(vertices[i]));
                    differentSign |= vertexSign[i] != cntSign;
                }
                int[,] ln = cube.LineNumbers;
                if (differentSign)
                {   // there must be at least 3 edges of the cube passing through the surface
                    // we should return them all, but the parameter allows only one
                    for (int i = 0; i < 12; i++)
                    {
                        if (vertexSign[ln[i, 0]] != vertexSign[ln[i, 1]])
                        {   // the line from vertices[ln[i,0]] to vertices[ln[i,1]] intersects the surface.
                            GeoPoint startPoint = vertices[ln[i, 0]];
                            GeoVector direction = vertices[ln[i, 1]] - startPoint;
                            Polynom toSolve = implicitSurface.Substitute(Polynom.Line3d(startPoint, direction));
                            double[] roots = toSolve.Roots();
                            for (int j = 0; i < roots.Length; i++)
                            {
                                if (roots[j] >= 0 && roots[j] <= 1)
                                {
                                    GeoPoint p = startPoint + roots[j] * direction;
                                    uv = PositionOf(p);
                                    return true;
                                }
                            }

                        }
                    }
                }
                else
                {
                    // no cube edge intersects the surface (an odd number of times) and the surface is close to the cube
                    // test all edges of the cube against the surface
                    for (int i = 0; i < 12; i++)
                    {
                        GeoPoint startPoint = vertices[ln[i, 0]];
                        GeoVector direction = vertices[ln[i, 1]] - startPoint;
                        Polynom toSolve = implicitSurface.Substitute(Polynom.Line3d(startPoint, direction));
                        double[] roots = toSolve.Roots();
                        for (int j = 0; i < roots.Length; i++)
                        {
                            if (roots[j] >= 0 && roots[j] <= 1)
                            {
                                GeoPoint p = startPoint + roots[j] * direction;
                                uv = PositionOf(p);
                                return true;
                            }
                        }
                    }
                }
                // no cube edge intersects the surface any number of times
                // Maybe a sphere or a torus touch the cube without intersection one of it's edges
                GeoPoint2D[] extrema = this.GetExtrema(); // the extrema in axis direction
                for (int i = 0; i < extrema.Length; i++)
                {
                    if (cube.Contains(PointAt(extrema[i])))
                    {
                        uv = extrema[i];
                        return true;
                    }
                }
                // a torus may pass through the cube without interfering with the edges: check the foot points from the center
                GeoPoint2D[] footPoints = this.PerpendicularFoot(cube.GetCenter());
                for (int i = 0; i < footPoints.Length; i++)
                {
                    if (cube.Contains(PointAt(footPoints[i])))
                    {
                        uv = footPoints[i];
                        return true;
                    }
                }
                return false;
            }
            return false;
        }
        public virtual bool Oriented
        {
            get
            {
                return false;
            }
        }
        public virtual RuledSurfaceMode IsRuled
        {
            get
            {
                return RuledSurfaceMode.notRuled;
            }
        }

        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.Orientation (GeoPoint)"/>
        /// </summary>
        /// <param name="p"></param>
        /// <returns></returns>
        public virtual double Orientation(GeoPoint p)
        {
            return 0.0;
        }
        protected virtual double[] GetSaveUSteps()
        {
            return new double[0];
        }
        protected virtual double[] GetSaveVSteps()
        {
            return new double[0];
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetExtrema ()"/>
        /// </summary>
        /// <returns></returns>
        public virtual GeoPoint2D[] GetExtrema()
        {
            if (extrema == null)
            {
                List<GeoPoint2D> res = new List<GeoPoint2D>();
                double[] usteps = GetSaveUSteps();
                if (usteps.Length > 0)
                {
                    double[] vsteps = GetSaveVSteps();
                    double epsu = (usteps[usteps.Length - 1] - usteps[0]) * 1e-6;
                    double epsv = (vsteps[vsteps.Length - 1] - vsteps[0]) * 1e-6;
                    for (int i = 1; i < usteps.Length; ++i)
                    {
                        for (int j = 1; j < vsteps.Length; ++j)
                        {
                            GeoPoint2D u00 = new GeoPoint2D(usteps[i - 1], vsteps[j - 1]);
                            GeoPoint2D u01 = new GeoPoint2D(usteps[i - 1], vsteps[j]);
                            GeoPoint2D u10 = new GeoPoint2D(usteps[i], vsteps[j - 1]);
                            GeoPoint2D u11 = new GeoPoint2D(usteps[i], vsteps[j]);
                            GeoVector v00 = GetNormal(u00); // nicht normieren wg. Nullvektoren
                            GeoVector v01 = GetNormal(u01);
                            GeoVector v10 = GetNormal(u10);
                            GeoVector v11 = GetNormal(u11);
                            GeoPoint2D p;
                            //if (ApproxExtreme(GeoVector.XAxis, u00, u01, u10, u11, v00, v01, v10, v11, epsu, epsv, out p)) res.Add(p);
                            //if (ApproxExtreme(-GeoVector.XAxis, u00, u01, u10, u11, v00, v01, v10, v11, epsu, epsv, out p)) res.Add(p);
                            //if (ApproxExtreme(GeoVector.YAxis, u00, u01, u10, u11, v00, v01, v10, v11, epsu, epsv, out p)) res.Add(p);
                            //if (ApproxExtreme(-GeoVector.YAxis, u00, u01, u10, u11, v00, v01, v10, v11, epsu, epsv, out p)) res.Add(p);
                            //if (ApproxExtreme(GeoVector.ZAxis, u00, u01, u10, u11, v00, v01, v10, v11, epsu, epsv, out p)) res.Add(p);
                            //if (ApproxExtreme(-GeoVector.ZAxis, u00, u01, u10, u11, v00, v01, v10, v11, epsu, epsv, out p)) res.Add(p);
                            // jeweils zwei Dreiecke untersuchen
                            if (FindExtreme(GeoVector.XAxis, u00, u01, u10, v00, v01, v10, epsu, epsv, out p)) res.Add(p);
                            if (FindExtreme(GeoVector.XAxis, u11, u01, u10, v11, v01, v10, epsu, epsv, out p)) res.Add(p);
                            if (FindExtreme(GeoVector.YAxis, u00, u01, u10, v00, v01, v10, epsu, epsv, out p)) res.Add(p);
                            if (FindExtreme(GeoVector.YAxis, u11, u01, u10, v11, v01, v10, epsu, epsv, out p)) res.Add(p);
                            if (FindExtreme(GeoVector.ZAxis, u00, u01, u10, v00, v01, v10, epsu, epsv, out p)) res.Add(p);
                            if (FindExtreme(GeoVector.ZAxis, u11, u01, u10, v11, v01, v10, epsu, epsv, out p)) res.Add(p);
                        }
                    }
                }
                extrema = res.ToArray();
            }
            return extrema;
        }
        private static double[] AllCurveAxtrema(ICurve curve)
        {
            double[] ex = curve.GetExtrema(GeoVector.XAxis);
            double[] ey = curve.GetExtrema(GeoVector.YAxis);
            double[] ez = curve.GetExtrema(GeoVector.ZAxis);
            double[] res = new double[ex.Length + ey.Length + ez.Length];
            int j = 0;
            for (int i = 0; i < ex.Length; ++i)
            {
                res[j] = ex[i];
                ++j;
            }
            for (int i = 0; i < ey.Length; ++i)
            {
                res[j] = ey[i];
                ++j;
            }
            for (int i = 0; i < ez.Length; ++i)
            {
                res[j] = ez[i];
                ++j;
            }
            return res;
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetPatchExtent (BoundingRect)"/>
        /// </summary>
        /// <param name="uvPatch"></param>
        /// <returns></returns>
        public virtual BoundingBox GetPatchExtent(BoundingRect uvPatch, bool rough)
        {   // kann natürlich in den einzelnen flächen besser gelöst werden
            BoundingBox res = BoundingBox.EmptyBoundingBox;
            GeoPoint2D[] extr = GetExtrema();
            for (int i = 0; i < extr.Length; ++i)
            {
                if (uvPatch.Contains(extr[i]))
                {
                    res.MinMax((this as ISurface).PointAt(extr[i]));
                }
            }
            ICurve fix;
            double[] cex;
            fix = FixedU(uvPatch.Left, uvPatch.Bottom, uvPatch.Top);
            if (fix != null)
            {
                res.MinMax(fix.StartPoint);
                res.MinMax(fix.EndPoint);
                cex = AllCurveAxtrema(fix);
                for (int i = 0; i < cex.Length; ++i)
                {
                    res.MinMax(fix.PointAt(cex[i]));
                }
            }
            fix = FixedU(uvPatch.Right, uvPatch.Bottom, uvPatch.Top);
            if (fix != null)
            {
                res.MinMax(fix.StartPoint);
                res.MinMax(fix.EndPoint);
                cex = AllCurveAxtrema(fix);
                for (int i = 0; i < cex.Length; ++i)
                {
                    res.MinMax(fix.PointAt(cex[i]));
                }
            }
            fix = FixedV(uvPatch.Bottom, uvPatch.Left, uvPatch.Right);
            if (fix != null)
            {
                res.MinMax(fix.StartPoint);
                res.MinMax(fix.EndPoint);
                cex = AllCurveAxtrema(fix);
                for (int i = 0; i < cex.Length; ++i)
                {
                    res.MinMax(fix.PointAt(cex[i]));
                }
            }
            fix = FixedV(uvPatch.Top, uvPatch.Left, uvPatch.Right);
            if (fix != null)
            {
                res.MinMax(fix.StartPoint);
                res.MinMax(fix.EndPoint);
                cex = AllCurveAxtrema(fix);
                for (int i = 0; i < cex.Length; ++i)
                {
                    res.MinMax(fix.PointAt(cex[i]));
                }
            }
            return res;
        }

        public static Polynom GetSectionInPlane(Plane plane, Polynom F)
        {

            // baue x(u,v), y(u,v), z(u,v)
            Polynom xUV = new Polynom(
                plane.Location.x, "",
                plane.DirectionX.x, "x",
                plane.DirectionY.x, "y"
            );
            Polynom yUV = new Polynom(
                plane.Location.y, "",
                plane.DirectionX.y, "x",
                plane.DirectionY.y, "y"
            );
            Polynom zUV = new Polynom(
                plane.Location.z, "",
                plane.DirectionX.z, "x",
                plane.DirectionY.z, "y"
            );

            return F.Substitute(xUV, yUV, zUV); // jetzt dim==2
        }

        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetPolynomialParameters ()"/>
        /// </summary>
        /// <returns></returns>
        public virtual double[] GetPolynomialParameters()
        {
            return null;
        }
        public virtual Polynom GetImplicitPolynomial()
        {
            double[] par = GetPolynomialParameters();
            if (par != null)
            {
                List<object> def = new List<object>(20);

                if (par[0] != 0.0) { def.Add(par[0]); def.Add("x2"); }
                if (par[1] != 0.0) { def.Add(par[1]); def.Add("y2"); }
                if (par[2] != 0.0) { def.Add(par[2]); def.Add("z2"); }
                if (par[3] != 0.0) { def.Add(par[3]); def.Add("xy"); }
                if (par[4] != 0.0) { def.Add(par[4]); def.Add("yz"); }
                if (par[5] != 0.0) { def.Add(par[5]); def.Add("xz"); }
                if (par[6] != 0.0) { def.Add(par[6]); def.Add("x"); }
                if (par[7] != 0.0) { def.Add(par[7]); def.Add("y"); }
                if (par[8] != 0.0) { def.Add(par[8]); def.Add("z"); }
                if (par[9] != 0.0) { def.Add(par[9]); def.Add(""); }

                if (def.Count == 0)
                {
                    return new Polynom(0.0, "");
                }
                return new Polynom(def.ToArray());
            }
            return null;
        }

        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.SetBounds (BoundingRect)"/>
        /// </summary>
        /// <param name="boundingRect"></param>
        public virtual void SetBounds(BoundingRect boundingRect)
        {
            usedArea = boundingRect;
        }
        public virtual BoundingRect GetBounds()
        {
            //if (usedArea.IsEmpty()) // no! we need the empty bound e.g. in ExtendBoundsTo
            //{
            //    GetNaturalBounds(out double umin, out double umax, out double vmin, out double vmax);
            //    return new BoundingRect(umin, vmin, umax, vmax);
            //}
            return usedArea;
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.PerpendicularFoot (GeoPoint)"/>
        /// </summary>
        /// <param name="fromHere"></param>
        /// <returns></returns>
        public virtual GeoPoint2D[] PerpendicularFoot(GeoPoint fromHere)
        {
            GeoPoint2D pos = PositionOf(fromHere);
            GeoPoint loc;
            GeoVector du, dv;
            DerivativeAt(pos, out loc, out du, out dv);
            double d = Geometry.DistPL(fromHere, loc, du ^ dv);
            // if (Precision.IsEqual(fromHere, loc) || Precision.SameDirection(du ^ dv, fromHere - loc, false))
            // bei PositionOf in der BoxedSurfaces ist das Abbruchkriterium der Abstand des Punktes von der Normalen in pos.
            // hier sollte man also das gleiche Kriterium wählen
            if (Precision.IsEqual(fromHere, loc) || d < Precision.eps * 100)
            {
                return new GeoPoint2D[] { pos };
            }
            else
            {
                return new GeoPoint2D[0];
            }
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.HasDiscontinuousDerivative (out ICurve2D[])"/>
        /// </summary>
        /// <param name="discontinuities"></param>
        /// <returns></returns>
        public virtual bool HasDiscontinuousDerivative(out ICurve2D[] discontinuities)
        {   // kommt nur bei einigen komischen Flächen vor und bei NURBS mit degree==1 und mehr als 2 knots
            discontinuities = null;
            return false;
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetNonPeriodicSurface (ICurve[])"/>
        /// </summary>
        /// <param name="maxOutline"></param>
        /// <returns></returns>
        public virtual ISurface GetNonPeriodicSurface(ICurve[] orientedCurves)
        {
            return null;
        }
        /// <summary>
        /// Implements <see cref="CADability.GeoObject.ISurface.GetPatchHull (BoundingRect, out GeoPoint, out GeoVector, out GeoVector, out GeoVector)"/>
        /// </summary>
        /// <param name="uvpatch"></param>
        /// <param name="loc"></param>
        /// <param name="dir1"></param>
        /// <param name="dir2"></param>
        /// <param name="dir3"></param>
        public virtual void GetPatchHull(BoundingRect uvpatch, out GeoPoint loc, out GeoVector dir1, out GeoVector dir2, out GeoVector dir3)
        {
            ParallelepipedHull.GetPatchHull(uvpatch, out loc, out dir1, out dir2, out dir3);
        }
        public virtual GeoPoint[] GetTouchingPoints(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds)
        {
            throw new NotImplementedException();
        }
        public virtual ISurface GetCanonicalForm(double precision, BoundingRect? bounds)
        {
            return null;
        }
        public GeoPoint2D PositionOf(GeoPoint p, BoundingRect domain)
        {
            GeoPoint2D res = PositionOf(p);
            SurfaceHelper.AdjustPeriodic(this, domain, ref res);
            return res;
        }
        public virtual int GetExtremePositions(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, out List<Tuple<double, double, double, double>> extremePositions)
        {
            extremePositions = new List<Tuple<double, double, double, double>>();
            GeoPoint2D uv11 = thisBounds.GetCenter();
            GeoPoint2D uv22 = otherBounds.GetCenter();
            try
            {
                if (Surfaces.PerpendicularConnection(this, thisBounds, other, otherBounds, ref uv11, ref uv22))
                {
                    if ((this.PointAt(uv11) | other.PointAt(uv22)) < Precision.eps)
                    {
                        // this is an intersection point. try to find another intersection point and take the point in the middle
                        // this will not be an extreme point but a good additional point for BoxedSurface.Intersect additionalSearchPositions
                        foreach (GeoPoint2D uv111 in new GeoPoint2D[] { thisBounds.GetLowerLeft(), thisBounds.GetLowerRight(), thisBounds.GetUpperLeft(), thisBounds.GetUpperRight() })
                        {   // try with different starting points to find more intersection points
                            GeoPoint2D uv2 = other.PositionOf(this.PointAt(uv111));
                            GeoPoint2D uv1 = uv111;
                            if (Surfaces.PerpendicularConnection(this, thisBounds, other, otherBounds, ref uv11, ref uv22) && (uv1 | uv11) > Precision.eps && (uv2 | uv22) > Precision.eps)
                            {   //we have another intersection point and we will use the point in between
                                if ((this.PointAt(uv1) | other.PointAt(uv2)) < Precision.eps)
                                {   // a second intersection point
                                    extremePositions.Add(new Tuple<double, double, double, double>((uv1.x + uv11.x) / 2.0, (uv1.y + uv11.y) / 2.0, (uv2.x + uv22.x) / 2.0, (uv2.y + uv22.y) / 2.0));
                                }
                                else
                                {   // a real extreme point, which is not an intersection point
                                    extremePositions.Add(new Tuple<double, double, double, double>(uv1.x, uv1.y, uv2.x, uv2.y));
                                }
                                break;
                            }
                        }
                    }
                    else
                    {
                        if (thisBounds.Contains(uv11)) extremePositions.Add(new Tuple<double, double, double, double>(uv11.x, uv11.y, double.NaN, double.NaN));
                        if (otherBounds.Contains(uv22)) extremePositions.Add(new Tuple<double, double, double, double>(double.NaN, double.NaN, uv22.x, uv22.y));
                    }
                }
            }
            catch (NotImplementedException) { } // some surfaces don't implement second derivation (e.g. OffsetSurface) which is needed in GaussNewtonMinimizer.SurfaceExtrema
            // else: search with different starting points
            return extremePositions.Count;
        }
        public virtual int GetExtremePositions(BoundingRect domain, ICurve curve3D, out List<Tuple<double, double, double>> positions)
        {   // to implement
            positions = new List<Tuple<double, double, double>>();
            GeoPoint2D uv1 = domain.GetCenter();
            double u2 = 0.5;
            double maxerror = GaussNewtonMinimizer.SurfaceCurveExtrema(this, domain, curve3D, 0.0, 1.0, ref uv1, ref u2);
            if (maxerror < Precision.eps)
            {
                positions.Add(new Tuple<double, double, double>(uv1.x, uv1.y, u2));
            }
            return positions.Count;
        }
        /// <summary>
        /// Distance from point p to the surface. The result is always &gt;= 0 (unfortunately the orientation is not
        /// considered here).
        /// </summary>
        /// <param name="p"></param>
        /// <returns></returns>
        public virtual double GetDistance(GeoPoint p)
        {
            GeoPoint2D uv = PositionOf(p);
            // GeoVector n = GetNormal(uv).Normalized;
            GeoPoint p0 = PointAt(uv);
            return p | p0;
        }
        public virtual bool MayIntersectSegment(GeoPoint a, GeoPoint b)
        {   // most surfaces can do better!
            GeoPoint2D[] uv = GetLineIntersection(a, b - a);
            for (int i = 0; i < uv.Length; i++)
            {
                double par = Geometry.LinePar(a, b, PointAt(uv[i]));
                if (par >= 0 && par <= 1) return true;
            }
            return false;
        }

        public virtual bool IsExtruded(GeoVector direction)
        {
            return false;
        }
        public virtual bool IsRotated(Axis rotationAxis)
        {
            return false;
        }
        public virtual MenuWithHandler[] GetContextMenuForParametrics(IFrame frame, Face face)
        {
            return new MenuWithHandler[0];
        }
        public virtual bool UvChangesWithModification => false;
        public virtual bool CanStretch => false;
        public virtual double Fit(IEnumerable<GeoPoint> points)
        {
            // we are looking for the best transformation of the points so that the distance to the surface of all points becomes minimal
            // transformation: orthonormal with scaling and translation
            GeoPoint[] pnts = points.ToArray();

#if DEBUG
            {
                double err = 0.0;
                for (int i = 0; i < pnts.Length; i++) err += pnts[i] & PointAt(PositionOf(pnts[i])); // squared distance
                double derr = 0.0;
                for (int i = 0; i < pnts.Length; i++) derr += this.GetDistance(pnts[i]);
            }
            //for (int i = 0; i < pnts.Length; i++)
            //{
            //    pnts[i] = new GeoPoint(pnts[i].x, pnts[i].y, pnts[i].z + 1);
            //}
#endif
            Vector<double> observedX = new DenseVector(pnts.Length); // there is no need to set values
            Vector<double> observedY = new DenseVector(pnts.Length); // this is the data we want to achieve, namely 0.0
            LevenbergMarquardtMinimizer lm = new LevenbergMarquardtMinimizer(gradientTolerance: 1e-15, maximumIterations: 20);
            NonlinearMinimizationResult mres;
            if (CanStretch)
            {
                IObjectiveModel iom = ObjectiveFunction.NonlinearModel(
                    new Func<Vector<double>, Vector<double>, Vector<double>>(delegate (Vector<double> vd, Vector<double> ox) // function
                    {
                        // 
                        DenseVector res = new DenseVector(pnts.Length);
                        double a = vd[0]; // rotation around x-axis
                        double b = vd[1]; // rotation around y-axis
                        double c = vd[2]; // rotation around z-axis
                        double fx = vd[3]; // scaling factor x
                        double fy = vd[4]; // scaling factor y
                        double fz = vd[5]; // scaling factor z
                        double sa = Math.Sin(a);
                        double ca = Math.Cos(a);
                        double sb = Math.Sin(b);
                        double cb = Math.Cos(b);
                        double sc = Math.Sin(c);
                        double cc = Math.Cos(c);
                        GeoVector t = new GeoVector(vd[6], vd[7], vd[8]); // translation 
                        Matrix m = DenseMatrix.OfArray(new double[,] { { fx, 0, 0 }, { 0, fy, 0 }, { 0, 0, fz } }) *
                            DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                            DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                            DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                        ModOp trsf = new ModOp(m.ToArray(), t);
                        for (int i = 0; i < pnts.Length; i++)
                        {
                            GeoPoint p = trsf * pnts[i];
                            GeoPoint2D uv = PositionOf(p);
                            GeoPoint o = PointAt(uv);
                            GeoVector n = GetNormal(uv).Normalized;
                            res[i] = n * (p - o); // (signed) distance from the plane
                        }
#if DEBUG
                        double err = 0.0;
                        for (int i = 0; i < pnts.Length; i++) err += res[i] * res[i];
#endif
                        return res;
                    }),
                    new Func<Vector<double>, Vector<double>, Matrix<double>>(delegate (Vector<double> vd, Vector<double> ox) // derivatives
                    {
                        var prime = new DenseMatrix(pnts.Length, 9);
                        double a = vd[0]; // rotation around x-axis
                        double b = vd[1]; // rotation around y-axis
                        double c = vd[2]; // rotation around z-axis
                        double fx = vd[3]; // scaling factor x
                        double fy = vd[4]; // scaling factor y
                        double fz = vd[5]; // scaling factor z
                        double sa = Math.Sin(a);
                        double ca = Math.Cos(a);
                        double sb = Math.Sin(b);
                        double cb = Math.Cos(b);
                        double sc = Math.Sin(c);
                        double cc = Math.Cos(c);
                        GeoVector t = new GeoVector(vd[6], vd[7], vd[8]); // translation 
                        Matrix m = DenseMatrix.OfArray(new double[,] { { fx, 0, 0 }, { 0, fy, 0 }, { 0, 0, fz } }) *
                            DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                            DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                            DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                        ModOp trsf = new ModOp(m.ToArray(), t);
                        for (int i = 0; i < pnts.Length; i++)
                        {
                            GeoPoint p = trsf * pnts[i];
                            GeoPoint2D uv = PositionOf(p);
                            GeoPoint o = PointAt(uv);
                            GeoVector n = GetNormal(uv).Normalized;

                            prime[i, 0] = fy * n.y * (sb * sc * (ca * p.y - sa * p.z) + cc * (-ca * p.z - sa * p.y)) + fx * n.x * (sb * cc * (ca * p.y - sa * p.z) - sc * (-ca * p.z - sa * p.y)) + cb * fz * n.z * (ca * p.y - sa * p.z);
                            prime[i, 1] = fz * n.z * (-sb * (ca * p.z + sa * p.y) - cb * p.x) + sc * fy * n.y * (cb * (ca * p.z + sa * p.y) - sb * p.x) + cc * fx * n.x * (cb * (ca * p.z + sa * p.y) - sb * p.x);
                            prime[i, 2] = fx * n.x * (-sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - cc * (ca * p.y - sa * p.z)) + fy * n.y * (cc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - sc * (ca * p.y - sa * p.z));
                            prime[i, 3] = n.x * (cc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - sc * (ca * p.y - sa * p.z));
                            prime[i, 4] = n.y * (sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) + cc * (ca * p.y - sa * p.z));
                            prime[i, 5] = n.z * (cb * (ca * p.z + sa * p.y) - sb * p.x);
                            prime[i, 6] = n.x;
                            prime[i, 7] = n.y;
                            prime[i, 8] = n.z;
                        }
                        return prime;
                    }), observedX, observedY);
                mres = lm.FindMinimum(iom, new DenseVector(new double[] { 0, 0, 0, 1, 1, 1, 0, 0, 0 }));
                if (true)
                {
                    double a = mres.MinimizingPoint[0]; // rotation around x-axis
                    double b = mres.MinimizingPoint[1]; // rotation around y-axis
                    double c = mres.MinimizingPoint[2]; // rotation around z-axis
                    double fx = mres.MinimizingPoint[3]; // scaling factor x
                    double fy = mres.MinimizingPoint[4]; // scaling factor y
                    double fz = mres.MinimizingPoint[5]; // scaling factor z
                    double sa = Math.Sin(a);
                    double ca = Math.Cos(a);
                    double sb = Math.Sin(b);
                    double cb = Math.Cos(b);
                    double sc = Math.Sin(c);
                    double cc = Math.Cos(c);
                    GeoVector t = new GeoVector(mres.MinimizingPoint[6], mres.MinimizingPoint[7], mres.MinimizingPoint[8]); // translation 
                    Matrix m = DenseMatrix.OfArray(new double[,] { { fx, 0, 0 }, { 0, fy, 0 }, { 0, 0, fz } }) *
                        DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                        DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                        DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                    ModOp trsf = new ModOp(m.ToArray(), t);
                    Modify(trsf.GetInverse());
                }
            }
            else
            {
                IObjectiveModel iom = ObjectiveFunction.NonlinearModel(
                    new Func<Vector<double>, Vector<double>, Vector<double>>(delegate (Vector<double> vd, Vector<double> ox) // function
                    {
                        // 
                        DenseVector res = new DenseVector(pnts.Length);
                        double a = vd[0]; // rotation around x-axis
                        double b = vd[1]; // rotation around y-axis
                        double c = vd[2]; // rotation around z-axis
                        double f = vd[3]; // scaling factor
                        double sa = Math.Sin(a);
                        double ca = Math.Cos(a);
                        double sb = Math.Sin(b);
                        double cb = Math.Cos(b);
                        double sc = Math.Sin(c);
                        double cc = Math.Cos(c);
                        GeoVector t = new GeoVector(vd[4], vd[5], vd[6]); // translation 
                        Matrix m = DenseMatrix.CreateDiagonal(3, 3, f) *
                            DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                            DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                            DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                        ModOp trsf = new ModOp(m.ToArray(), t);
                        for (int i = 0; i < pnts.Length; i++)
                        {
                            GeoPoint p = trsf * pnts[i];
                            GeoPoint2D uv = PositionOf(p);
                            GeoPoint o = PointAt(uv);
                            res[i] = (p - o) * (p - o); // square distance
                        }
#if DEBUG
                        double err = 0.0;
                        for (int i = 0; i < pnts.Length; i++) err += res[i];
#endif
                        return res;
                    }),
                    new Func<Vector<double>, Vector<double>, Matrix<double>>(delegate (Vector<double> vd, Vector<double> ox) // derivatives
                    {
                        var prime = new DenseMatrix(pnts.Length, 7);

                        double a = vd[0]; // rotation around x-axis
                        double b = vd[1]; // rotation around y-axis
                        double c = vd[2]; // rotation around z-axis
                        double f = vd[3]; // scaling factor
                        double sa = Math.Sin(a);
                        double ca = Math.Cos(a);
                        double sb = Math.Sin(b);
                        double cb = Math.Cos(b);
                        double sc = Math.Sin(c);
                        double cc = Math.Cos(c);
                        GeoVector t = new GeoVector(vd[4], vd[5], vd[6]); // translation 
                        Matrix m = DenseMatrix.CreateDiagonal(3, 3, f) *
                            DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                            DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                            DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                        ModOp trsf = new ModOp(m.ToArray(), t);
                        for (int i = 0; i < pnts.Length; i++)
                        {
                            GeoPoint p = trsf * pnts[i];
                            GeoPoint2D uv = PositionOf(p);
                            GeoPoint o = PointAt(uv);

                            prime[i, 0] = -2 * cb * f * (ca * p.y - sa * p.z) * (-t.z - f * (cb * (ca * p.z + sa * p.y) - sb * p.x) + o.z) - 2 * f * (sb * sc * (ca * p.y - sa * p.z) + cc * (-ca * p.z - sa * p.y)) * (-t.y - f * (sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) + cc * (ca * p.y - sa * p.z)) + o.y) - 2 * f * (sb * cc * (ca * p.y - sa * p.z) - sc * (-ca * p.z - sa * p.y)) * (-t.x - f * (cc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - sc * (ca * p.y - sa * p.z)) + o.x);
                            prime[i, 1] = -2 * f * (-sb * (ca * p.z + sa * p.y) - cb * p.x) * (-t.z - f * (cb * (ca * p.z + sa * p.y) - sb * p.x) + o.z) - 2 * sc * f * (cb * (ca * p.z + sa * p.y) - sb * p.x) * (-t.y - f * (sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) + cc * (ca * p.y - sa * p.z)) + o.y) - 2 * cc * f * (cb * (ca * p.z + sa * p.y) - sb * p.x) * (-t.x - f * (cc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - sc * (ca * p.y - sa * p.z)) + o.x);
                            prime[i, 2] = -2 * f * (cc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - sc * (ca * p.y - sa * p.z)) * (-t.y - f * (sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) + cc * (ca * p.y - sa * p.z)) + o.y) - 2 * f * (-sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - cc * (ca * p.y - sa * p.z)) * (-t.x - f * (cc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - sc * (ca * p.y - sa * p.z)) + o.x);
                            prime[i, 3] = 2 * (sb * p.x - cb * (ca * p.z + sa * p.y)) * (-t.z - f * (cb * (ca * p.z + sa * p.y) - sb * p.x) + o.z) + 2 * (-sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - cc * (ca * p.y - sa * p.z)) * (-t.y - f * (sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) + cc * (ca * p.y - sa * p.z)) + o.y) + 2 * (sc * (ca * p.y - sa * p.z) - cc * (sb * (ca * p.z + sa * p.y) + cb * p.x)) * (-t.x - f * (cc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - sc * (ca * p.y - sa * p.z)) + o.x);
                            prime[i, 4] = -2 * (-t.x - f * (cc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - sc * (ca * p.y - sa * p.z)) + o.x);
                            prime[i, 5] = -2 * (-t.y - f * (sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) + cc * (ca * p.y - sa * p.z)) + o.y);
                            prime[i, 6] = -2 * (-t.z - f * (cb * (ca * p.z + sa * p.y) - sb * p.x) + o.z);
                        }
                        return prime;
                    }), observedX, observedY);
                mres = lm.FindMinimum(iom, new DenseVector(new double[] { 0, 0, 0, 1, 0, 0, 0 }));
                if (true)
                {
                    double a = mres.MinimizingPoint[0]; // rotation around x-axis
                    double b = mres.MinimizingPoint[1]; // rotation around y-axis
                    double c = mres.MinimizingPoint[2]; // rotation around z-axis
                    double f = mres.MinimizingPoint[3]; // scaling factor
                    double sa = Math.Sin(a);
                    double ca = Math.Cos(a);
                    double sb = Math.Sin(b);
                    double cb = Math.Cos(b);
                    double sc = Math.Sin(c);
                    double cc = Math.Cos(c);
                    GeoVector t = new GeoVector(mres.MinimizingPoint[4], mres.MinimizingPoint[5], mres.MinimizingPoint[6]); // translation 
                    Matrix m = DenseMatrix.CreateDiagonal(3, 3, f) *
                        DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                        DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                        DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                    ModOp trsf = new ModOp(m.ToArray(), t);
#if DEBUG
                    double err1 = 0.0;
                    double derr1 = 0.0;
                    for (int i = 0; i < pnts.Length; i++)
                    {
                        GeoPoint p = trsf * pnts[i];
                        GeoPoint2D uv = PositionOf(p);
                        GeoPoint o = PointAt(uv);
                        double r = (p - o) * (p - o); // (signed) distance from the plane
                        err1 += r;
                        derr1 += GetDistance(trsf * pnts[i]);
                    }
#endif
                    Modify(trsf.GetInverse());
#if DEBUG
                    double err2 = 0.0;
                    double derr2 = 0.0;
                    for (int i = 0; i < pnts.Length; i++)
                    {
                        GeoPoint p = pnts[i];
                        GeoPoint2D uv = PositionOf(p);
                        GeoPoint o = PointAt(uv);
                        GeoVector n = GetNormal(uv).Normalized;
                        double r = n * (p - o); // (signed) distance from the plane
                        err2 += r;
                        derr2 += GetDistance(pnts[i]);
                    }
#endif
                }
            }
#if DEBUG
            {
                double err = 0.0;
                for (int i = 0; i < pnts.Length; i++) err += pnts[i] & PointAt(PositionOf(pnts[i])); // squared distance
            }
#endif
            return mres.ModelInfoAtMinimum.Value;
        }
        public virtual double FitOld(IEnumerable<GeoPoint> points)
        {
            // we are looking for the best transformation of the points so that the distance to the surface of all points becomes minimal
            // transformation: orthonormal with scaling and translation
            GeoPoint[] pnts = points.ToArray();

#if DEBUG
            {
                double err = 0.0;
                for (int i = 0; i < pnts.Length; i++) err += pnts[i] & PointAt(PositionOf(pnts[i])); // squared distance
                double derr = 0.0;
                for (int i = 0; i < pnts.Length; i++) derr += this.GetDistance(pnts[i]);
            }
#endif
            Vector<double> observedX = new DenseVector(pnts.Length); // there is no need to set values
            Vector<double> observedY = new DenseVector(pnts.Length); // this is the data we want to achieve, namely 0.0
            LevenbergMarquardtMinimizer lm = new LevenbergMarquardtMinimizer(gradientTolerance: 1e-15, maximumIterations: 20);
            NonlinearMinimizationResult mres;
            if (CanStretch)
            {
                IObjectiveModel iom = ObjectiveFunction.NonlinearModel(
                    new Func<Vector<double>, Vector<double>, Vector<double>>(delegate (Vector<double> vd, Vector<double> ox) // function
                    {
                        // 
                        DenseVector res = new DenseVector(pnts.Length);
                        double a = vd[0]; // rotation around x-axis
                        double b = vd[1]; // rotation around y-axis
                        double c = vd[2]; // rotation around z-axis
                        double fx = vd[3]; // scaling factor x
                        double fy = vd[4]; // scaling factor y
                        double fz = vd[5]; // scaling factor z
                        double sa = Math.Sin(a);
                        double ca = Math.Cos(a);
                        double sb = Math.Sin(b);
                        double cb = Math.Cos(b);
                        double sc = Math.Sin(c);
                        double cc = Math.Cos(c);
                        GeoVector t = new GeoVector(vd[6], vd[7], vd[8]); // translation 
                        Matrix m = DenseMatrix.OfArray(new double[,] { { fx, 0, 0 }, { 0, fy, 0 }, { 0, 0, fz } }) *
                            DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                            DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                            DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                        ModOp trsf = new ModOp(m.ToArray(), t);
                        for (int i = 0; i < pnts.Length; i++)
                        {
                            GeoPoint p = trsf * pnts[i];
                            GeoPoint2D uv = PositionOf(p);
                            GeoPoint o = PointAt(uv);
                            GeoVector n = GetNormal(uv).Normalized;
                            res[i] = n * (p - o); // (signed) distance from the plane
                        }
#if DEBUG
                        double err = 0.0;
                        for (int i = 0; i < pnts.Length; i++) err += res[i] * res[i];
#endif
                        return res;
                    }),
                    new Func<Vector<double>, Vector<double>, Matrix<double>>(delegate (Vector<double> vd, Vector<double> ox) // derivatives
                    {
                        var prime = new DenseMatrix(pnts.Length, 9);
                        double a = vd[0]; // rotation around x-axis
                        double b = vd[1]; // rotation around y-axis
                        double c = vd[2]; // rotation around z-axis
                        double fx = vd[3]; // scaling factor x
                        double fy = vd[4]; // scaling factor y
                        double fz = vd[5]; // scaling factor z
                        double sa = Math.Sin(a);
                        double ca = Math.Cos(a);
                        double sb = Math.Sin(b);
                        double cb = Math.Cos(b);
                        double sc = Math.Sin(c);
                        double cc = Math.Cos(c);
                        GeoVector t = new GeoVector(vd[6], vd[7], vd[8]); // translation 
                        Matrix m = DenseMatrix.OfArray(new double[,] { { fx, 0, 0 }, { 0, fy, 0 }, { 0, 0, fz } }) *
                            DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                            DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                            DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                        ModOp trsf = new ModOp(m.ToArray(), t);
                        for (int i = 0; i < pnts.Length; i++)
                        {
                            GeoPoint p = trsf * pnts[i];
                            GeoPoint2D uv = PositionOf(p);
                            GeoPoint o = PointAt(uv);
                            GeoVector n = GetNormal(uv).Normalized;

                            prime[i, 0] = fy * n.y * (sb * sc * (ca * p.y - sa * p.z) + cc * (-ca * p.z - sa * p.y)) + fx * n.x * (sb * cc * (ca * p.y - sa * p.z) - sc * (-ca * p.z - sa * p.y)) + cb * fz * n.z * (ca * p.y - sa * p.z);
                            prime[i, 1] = fz * n.z * (-sb * (ca * p.z + sa * p.y) - cb * p.x) + sc * fy * n.y * (cb * (ca * p.z + sa * p.y) - sb * p.x) + cc * fx * n.x * (cb * (ca * p.z + sa * p.y) - sb * p.x);
                            prime[i, 2] = fx * n.x * (-sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - cc * (ca * p.y - sa * p.z)) + fy * n.y * (cc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - sc * (ca * p.y - sa * p.z));
                            prime[i, 3] = n.x * (cc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - sc * (ca * p.y - sa * p.z));
                            prime[i, 4] = n.y * (sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) + cc * (ca * p.y - sa * p.z));
                            prime[i, 5] = n.z * (cb * (ca * p.z + sa * p.y) - sb * p.x);
                            prime[i, 6] = n.x;
                            prime[i, 7] = n.y;
                            prime[i, 8] = n.z;
                        }
                        return prime;
                    }), observedX, observedY);
                mres = lm.FindMinimum(iom, new DenseVector(new double[] { 0, 0, 0, 1, 1, 1, 0, 0, 0 }));
                if (true)
                {
                    double a = mres.MinimizingPoint[0]; // rotation around x-axis
                    double b = mres.MinimizingPoint[1]; // rotation around y-axis
                    double c = mres.MinimizingPoint[2]; // rotation around z-axis
                    double fx = mres.MinimizingPoint[3]; // scaling factor x
                    double fy = mres.MinimizingPoint[4]; // scaling factor y
                    double fz = mres.MinimizingPoint[5]; // scaling factor z
                    double sa = Math.Sin(a);
                    double ca = Math.Cos(a);
                    double sb = Math.Sin(b);
                    double cb = Math.Cos(b);
                    double sc = Math.Sin(c);
                    double cc = Math.Cos(c);
                    GeoVector t = new GeoVector(mres.MinimizingPoint[6], mres.MinimizingPoint[7], mres.MinimizingPoint[8]); // translation 
                    Matrix m = DenseMatrix.OfArray(new double[,] { { fx, 0, 0 }, { 0, fy, 0 }, { 0, 0, fz } }) *
                        DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                        DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                        DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                    ModOp trsf = new ModOp(m.ToArray(), t);
                    Modify(trsf.GetInverse());
                }
            }
            else
            {
                IObjectiveModel iom = ObjectiveFunction.NonlinearModel(
                    new Func<Vector<double>, Vector<double>, Vector<double>>(delegate (Vector<double> vd, Vector<double> ox) // function
                    {
                        // 
                        DenseVector res = new DenseVector(pnts.Length);
                        double a = vd[0]; // rotation around x-axis
                        double b = vd[1]; // rotation around y-axis
                        double c = vd[2]; // rotation around z-axis
                        double f = vd[3]; // scaling factor
                        double sa = Math.Sin(a);
                        double ca = Math.Cos(a);
                        double sb = Math.Sin(b);
                        double cb = Math.Cos(b);
                        double sc = Math.Sin(c);
                        double cc = Math.Cos(c);
                        GeoVector t = new GeoVector(vd[4], vd[5], vd[6]); // translation 
                        Matrix m = DenseMatrix.CreateDiagonal(3, 3, f) *
                            DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                            DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                            DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                        ModOp trsf = new ModOp(m.ToArray(), t);
                        for (int i = 0; i < pnts.Length; i++)
                        {
                            GeoPoint p = trsf * pnts[i];
                            GeoPoint2D uv = PositionOf(p);
                            GeoPoint o = PointAt(uv);
                            GeoVector n = GetNormal(uv).Normalized;
                            res[i] = n * (p - o); // (signed) distance from the plane
                        }
#if DEBUG
                        double err = 0.0;
                        for (int i = 0; i < pnts.Length; i++) err += res[i] * res[i];
#endif
                        return res;
                    }),
                    new Func<Vector<double>, Vector<double>, Matrix<double>>(delegate (Vector<double> vd, Vector<double> ox) // derivatives
                    {
                        var prime = new DenseMatrix(pnts.Length, 7);

                        double a = vd[0]; // rotation around x-axis
                        double b = vd[1]; // rotation around y-axis
                        double c = vd[2]; // rotation around z-axis
                        double f = vd[3]; // scaling factor
                        double sa = Math.Sin(a);
                        double ca = Math.Cos(a);
                        double sb = Math.Sin(b);
                        double cb = Math.Cos(b);
                        double sc = Math.Sin(c);
                        double cc = Math.Cos(c);
                        GeoVector t = new GeoVector(vd[4], vd[5], vd[6]); // translation 
                        Matrix m = DenseMatrix.CreateDiagonal(3, 3, f) *
                            DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                            DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                            DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                        ModOp trsf = new ModOp(m.ToArray(), t);
                        for (int i = 0; i < pnts.Length; i++)
                        {
                            GeoPoint p = trsf * pnts[i];
                            GeoPoint2D uv = PositionOf(p);
                            GeoPoint o = PointAt(uv);
                            GeoVector n = GetNormal(uv).Normalized;

                            prime[i, 0] = f * n.y * (sb * sc * (ca * p.y - sa * p.z) + cc * (-ca * p.z - sa * p.y)) + f * n.x * (sb * cc * (ca * p.y - sa * p.z) - sc * (-ca * p.z - sa * p.y)) + cb * f * n.z * (ca * p.y - sa * p.z);
                            prime[i, 1] = f * n.z * (-sb * (ca * p.z + sa * p.y) - cb * p.x) + sc * f * n.y * (cb * (ca * p.z + sa * p.y) - sb * p.x) + cc * f * n.x * (cb * (ca * p.z + sa * p.y) - sb * p.x);
                            prime[i, 2] = f * n.z * (-sb * (ca * p.z + sa * p.y) - cb * p.x) + sc * f * n.y * (cb * (ca * p.z + sa * p.y) - sb * p.x) + cc * f * n.x * (cb * (ca * p.z + sa * p.y) - sb * p.x);
                            prime[i, 3] = n.y * (sc * (sb * (ca * p.z + sa * p.y) + cb * p.x) + cc * (ca * p.y - sa * p.z)) + n.x * (cc * (sb * (ca * p.z + sa * p.y) + cb * p.x) - sc * (ca * p.y - sa * p.z)) + n.z * (cb * (ca * p.z + sa * p.y) - sb * p.x);
                            prime[i, 4] = n.x;
                            prime[i, 5] = n.y;
                            prime[i, 6] = n.z;
                        }
                        return prime;
                    }), observedX, observedY);
                mres = lm.FindMinimum(iom, new DenseVector(new double[] { 0, 0, 0, 1, 0, 0, 0 }));
                if (true)
                {
                    double a = mres.MinimizingPoint[0]; // rotation around x-axis
                    double b = mres.MinimizingPoint[1]; // rotation around y-axis
                    double c = mres.MinimizingPoint[2]; // rotation around z-axis
                    double f = mres.MinimizingPoint[3]; // scaling factor
                    double sa = Math.Sin(a);
                    double ca = Math.Cos(a);
                    double sb = Math.Sin(b);
                    double cb = Math.Cos(b);
                    double sc = Math.Sin(c);
                    double cc = Math.Cos(c);
                    GeoVector t = new GeoVector(mres.MinimizingPoint[4], mres.MinimizingPoint[5], mres.MinimizingPoint[6]); // translation 
                    Matrix m = DenseMatrix.CreateDiagonal(3, 3, f) *
                        DenseMatrix.OfArray(new double[,] { { cc, -sc, 0 }, { sc, cc, 0 }, { 0, 0, 1 } }) *
                        DenseMatrix.OfArray(new double[,] { { cb, 0, sb }, { 0, 1, 0 }, { -sb, 0, cb } }) *
                        DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, ca, -sa }, { 0, sa, ca } });
                    ModOp trsf = new ModOp(m.ToArray(), t);
#if DEBUG
                    double err1 = 0.0;
                    double derr1 = 0.0;
                    for (int i = 0; i < pnts.Length; i++)
                    {
                        GeoPoint p = trsf * pnts[i];
                        GeoPoint2D uv = PositionOf(p);
                        GeoPoint o = PointAt(uv);
                        GeoVector n = GetNormal(uv).Normalized;
                        double r = n * (p - o); // (signed) distance from the plane
                        err1 += r * r;
                        derr1 += GetDistance(trsf * pnts[i]);
                    }
#endif
                    Modify(trsf.GetInverse());
#if DEBUG
                    double err2 = 0.0;
                    double derr2 = 0.0;
                    for (int i = 0; i < pnts.Length; i++)
                    {
                        GeoPoint p = pnts[i];
                        GeoPoint2D uv = PositionOf(p);
                        GeoPoint o = PointAt(uv);
                        GeoVector n = GetNormal(uv).Normalized;
                        double r = n * (p - o); // (signed) distance from the plane
                        err2 += r * r;
                        derr2 += GetDistance(pnts[i]);
                    }
#endif
                }
            }
#if DEBUG
            {
                double err = 0.0;
                for (int i = 0; i < pnts.Length; i++) err += pnts[i] & PointAt(PositionOf(pnts[i])); // squared distance
            }
#endif
            return mres.ModelInfoAtMinimum.Value;
        }
        public abstract IPropertyEntry GetPropertyEntry(IFrame frame);
#if DEBUG
        // Starte mit dem Mittelpunkt
        // Betrache die Kurve f(u) = u²*d2+u*d1+d0 (d2 ist die 2. Ableitung in der Richtung der beiden Punkte, d1 die 1., d0 der Punkt selbst),
        // Der Abstand f(u) zur Sekante ist gegeben durch |(f(u)-sp3d)^(f(u)-ep3d)|, welches es zu maximieren gilt, also Ableitung = 0
        // Da das zu schwierig ist, suche eine ModOp, die sp3d->(0,0,0) und ep3d->(1,0,0) abbildet (nur Skalierung und Drehung, keine Verzerrung), dann unterscheiden
        // sich die beiden Vektoren des Kreuzprodukts nur in der x-Komponente, das Ergebnis ist
        // aber immer noch 4. Potenz in und davon muss noch die Länge genommen werden, also 8. Potenz, das ist nicht lösbar
        // Wenn man das aber ausrechnet bleibt für das Kreuzprodukt nur f(u), also 2. Potenz:
        // [0, f(u).z*(f(u).x-1) - (f(u).z*f(u).x, f(u).x*f(u).y-f(u).y*(f(u).x-1)] = 
        // [0, f(u).z*((f(u).x-1) - f(u).x), f(u).y*(f(u).x-(f(u).x-1))] = 
        // [0, f(u).z*(-1), f(u).y*(1))]
        // Die Länge davon ist 4. Potenz, Abgeleitet 3. Potenz, das kann man 0 setzen!
        // (u^2*d2z+u*d1z+d0z)^2+(u^2*d2y+u*d1y+d0y)^2                                         (1*)
        // die 2. Ableitung: 2*(d1z+2*d2z*u)^2+2*(d1y+2*d2y*u)^2+4*d2z*(d0z+d1z*u+d2z*u^2)+4*d2y*(d0y+d1y*u+d2y*u^2)
        // um Minimum von Maximum unterscheiden zu können

        // Dazu Maxima Eingabe:
        // (u^2*d2z+u* d1z+d0z)^2+(u^2*d2y+u* d1z+d0y)^2; /* Länge des (nicht normierten quadratischen) Kreuzproduktes gemäß (1*); */;
        // diff(%, u);
        // u^3*ratsimp(%/u^3); /* Koeefizienten von u bestimmen */;

        // Ergibt: (4*d2y^2+4*d2z^2)*u^3+(6*d1z*d2y+6*d1z*d2z)*u^2+(4*d1z^2+4*d0y*d2y+4*d0z*d2z)*u+(2*d0y+2*d0z)*d1z

        // Der Versuch, die 3D Kurve anzunähern bringt nichts:
        public virtual double MaxDistN(GeoPoint2D sp, GeoPoint2D ep, out GeoPoint2D mp)
        {
            GeoPoint sp3d = PointAt(sp);
            GeoPoint ep3d = PointAt(ep);
            GeoVector dir = ep3d - sp3d;
            double len = dir.Length;
            double cosa = dir * GeoVector.XAxis / len;
            double sina = Math.Sqrt(1 - cosa * cosa);
            ModOp toUnit = ModOp.Scale(1.0 / len) * ModOp.Rotate(dir ^ GeoVector.XAxis, sina, cosa) * ModOp.Translate(-sp3d.x, -sp3d.y, -sp3d.z);

            mp = new GeoPoint2D(sp, ep);
            //double dbg1 = 1.0, dbg2 = 1.0, dbg3 = 1.0; das war der Versuch, die richtigen Faktoren für d2 zu finden
            //for (int ii = 0; ii < 27; ii++)
            //{
            //    switch (ii % 3)
            //    {
            //        case 0: dbg1 = 0.5; break;
            //        case 1: dbg1 = 1.0; break;
            //        case 2: dbg1 = 2.0; break;
            //    }
            //    switch ((ii / 3) % 3)
            //    {
            //        case 0: dbg2 = 0.5; break;
            //        case 1: dbg2 = 1.0; break;
            //        case 2: dbg2 = 2.0; break;
            //    }
            //    switch ((ii / 9) % 3)
            //    {
            //        case 0: dbg3 = 0.5; break;
            //        case 1: dbg3 = 1.0; break;
            //        case 2: dbg3 = 2.0; break;
            //    }
            //    System.Diagnostics.Trace.WriteLine("dbg1, dbg2, dbg3: " + dbg1.ToString() + ", " + dbg2.ToString() + ", " + dbg3.ToString());

            double u0 = 0.5;
            for (int k = 0; k < 10; ++k)
            {
                mp = sp + u0 * (ep - sp);
                GeoPoint location;
                GeoVector du, dv, duu, dvv, duv;
                this.Derivative2At(mp, out location, out du, out dv, out duu, out dvv, out duv);

                double len2 = sp | ep;
                double a = (ep.x - sp.x) / len2;
                double b = (ep.y - sp.y) / len2;
                GeoVector d1 = a * (toUnit * du) + b * (toUnit * dv);
                GeoVector d2 = 0.5 * (a * a * 1.0 * (toUnit * duu) + b * b * 1.0 * (toUnit * dvv) + a * b * 1.0 * (toUnit * duv));
                GeoPoint d0 = toUnit * location; // sind die Koeffizienten für f(u), f(0) liefert d0, u ist also auf mp bezogen
                GeoPoint foot = Geometry.DropPL(location, sp3d, ep3d);
                double err = d1.Normalized * (toUnit * (foot - location).Normalized);
                if (Math.Abs(err) < 1e-5) return location | foot; // der Winkel zwischen dem Lot und der Kurvenrichtung ist fast Null
#if DEBUG
                DebuggerContainer dc = new DebuggerContainer();
                ModOp fromUnit = toUnit.GetInverse();
                List<GeoPoint> dbgpnts = new List<GeoPoint>();
                List<GeoPoint> dbgsrf = new List<GeoPoint>();
                for (double d = -0.5; d <= 0.5; d += 0.01)
                {
                    GeoPoint pdbg = fromUnit * (d0 + d * d * d2 + d * d1);
                    dbgpnts.Add(pdbg);
                    pdbg = PointAt(sp + (d + 0.5) * (ep - sp));
                    dbgsrf.Add(pdbg);
                }
                Polyline pldbg = Polyline.Construct();
                pldbg.SetPoints(dbgpnts.ToArray(), false);
                dc.Add(pldbg);
                pldbg = Polyline.Construct();
                pldbg.SetPoints(dbgsrf.ToArray(), false);
                dc.Add(pldbg);
                Line dbgline = Line.Construct();
                dbgline.SetTwoPoints(sp3d, ep3d);
                dc.Add(dbgline);
#endif
                double[] x = new double[3];
                int n = Geometry.ragle3fast(4 * d2.y * d2.y + 4 * d2.z * d2.z, 6 * d1.y * d2.y + 6 * d1.z * d2.z, (2 * d1.y * d1.y + 2 * d1.z * d1.z + 4 * d0.y * d2.y + 4 * d0.z * d2.z), 2 * d0.z * d1.z + 2 * d0.y * d1.y, x);
                double offset = double.MaxValue;
                for (int i = 0; i < n; i++)
                {
                    if (Math.Abs(x[i]) < Math.Abs(offset))
                    {
                        double diff2 = 2 * sqr(d1.z + 2 * d2.z * x[i]) + 2 * sqr(d1.y + 2 * d2.y * x[i]) + 4 * d2.z * (d0.z + d1.z * x[i] + d2.z * x[i] * x[i]) + 4 * d2.y * (d0.y + d1.y * x[i] + d2.y * x[i] * x[i]);
                        // nur wenn die 2. Ableitung negativ ist, ist es ein Maximum
                        if (diff2 <= 0.0)
                        {
                            offset = x[i];
                            // wenn wir außerhalb von [0,1] geraten, dann auf [0,1] eingrenzen
                            if (k == 0)
                            {   // Wenn es einen Wendepunkt gibt im Segment, dann kann das im 1. Schritt (bei 0.5) rausfliegen
                                if (u0 + offset <= 0.0) offset = -0.25;
                                if (u0 + offset >= 1.0) offset = 0.25;
                            }
                        }
                    }
                }
                if (u0 + offset > 0.0 && u0 + offset < 1.0)
                {
                    u0 += offset;
                }
                else
                    break;
#if DEBUG
                System.Diagnostics.Trace.WriteLine("err, offest: " + err.ToString() + ", " + offset.ToString());
                GeoPoint pu0 = fromUnit * (d0 + offset * offset * d2 + offset * d1);
                GeoPoint ft = Geometry.DropPL(pu0, sp3d, ep3d);
                dbgline = Line.Construct();
                dbgline.SetTwoPoints(pu0, ft);
                dc.Add(dbgline);
                pu0 = PointAt(sp + u0 * (ep - sp));
                dbgline = Line.Construct();
                dbgline.SetTwoPoints(pu0, ft);
                dc.Add(dbgline);
#endif
            }
            //}
            {   // das Verfahren konvergiert nicht, zumindest nicht in dem Abschnitt
                // das sollte nur vorkommen, wenn die Funktion ausgeartet ist, also Ableitungen 0 oder so
                double max = 0;
                mp = GeoPoint2D.Origin; // wg. Compiler
                for (double d = 0.25; d < 1; d += 0.25)
                {
                    GeoPoint2D mpd = new GeoPoint2D(sp, ep, d);
                    GeoPoint mp3d = PointAt(mpd);
                    double dd = Geometry.DistPL(mp3d, sp3d, ep3d);
                    if (dd > max)
                    {
                        mp = mpd;
                        max = dd;
                    }
                }
                return max;
            }
        }
#endif
        protected static double sqr(double x) { return x * x; }
        protected static double cube(double x) { return x * x * x; }
        protected static double quad(double x) { return x * x * x * x; }
        protected static double exp32(double x) { return Math.Sqrt(x * x * x); }
        protected static double exp52(double x) { return Math.Sqrt(x * x * x * x * x); }
        public virtual double MaxDist(GeoPoint2D sp, GeoPoint2D ep, out GeoPoint2D mp)
        {
            GeoPoint sp3d, ep3d; // start und enpunkt in 3d, der maximale Abstand zu dieser Linie wird gesucht
            sp3d = PointAt(sp);
            ep3d = PointAt(ep);
            // Adaptive sampling: split the segment at the surface's "safe" grid lines (knots,
            // periodicity quadrants, ...), which mark where the surface's curvature behavior
            // changes. Between two such crossings the deviation from the chord is assumed to
            // have a single extremum, so 3 samples per sub-interval are enough - the same
            // sampling the previous, non-adaptive version always used over the whole [0,1].
            // Surfaces that don't override GetSaveUSteps/GetSaveVSteps (no known grid) fall
            // back to exactly that previous behavior, since there are no crossings to split at.
            List<double> breaks = new List<double>();
            breaks.Add(0.0);
            AddGridCrossings(GetSaveUSteps(), sp.x, ep.x, breaks);
            AddGridCrossings(GetSaveVSteps(), sp.y, ep.y, breaks);
            breaks.Add(1.0);
            breaks.Sort();

            double max = 0;
            mp = GeoPoint2D.Origin; // wg. Compiler
            for (int i = 1; i < breaks.Count; ++i)
            {
                double t0 = breaks[i - 1], t1 = breaks[i];
                if (t1 - t0 < 1e-8) continue; // duplicate/coincident crossing, degenerate sub-interval
                for (double d = 0.25; d < 1; d += 0.25)
                {
                    GeoPoint2D mpd = new GeoPoint2D(sp, ep, t0 + d * (t1 - t0));
                    GeoPoint mp3d = PointAt(mpd);
                    double dd = Geometry.DistPL(mp3d, sp3d, ep3d);
                    if (dd > max)
                    {
                        mp = mpd;
                        max = dd;
                    }
                }
            }
            return max;

        }
        // Adds t in (0,1) to breaks for every point where the sp->ep segment (parametrized as
        // s + t*(e-s)) crosses one of the given grid coordinates (u- or v-steps).
        private static void AddGridCrossings(double[] steps, double s, double e, List<double> breaks)
        {
            if (steps == null || steps.Length == 0) return;
            double d = e - s;
            if (Math.Abs(d) < 1e-12) return;
            for (int i = 0; i < steps.Length; ++i)
            {
                double t = (steps[i] - s) / d;
                if (t > 1e-8 && t < 1.0 - 1e-8) breaks.Add(t);
            }
        }
        public virtual bool IsCurveOnSurface(ICurve curve)
        {
            if (GetDistance(curve.StartPoint) > Precision.eps) return false; // surfaces with natural bounds should overwrite this method
            if (GetDistance(curve.EndPoint) > Precision.eps) return false;
            for (int i = 0; i < 5; i++)
            {   // this is a rough test only. Overwrite when necessary
                GeoPoint2D uv = PositionOf(curve.PointAt(i / 4.0));
                GeoVector cross = GetNormal(uv).Normalized ^ curve.DirectionAt(i / 4.0).Normalized;
                // the following condition was too strict for InterpolatedDualSurfaceCurves, extend by factor 10
                if (Math.Abs((GetNormal(uv).Normalized * curve.DirectionAt(i / 4.0).Normalized)) > 10 * Precision.eps) return false;
            }
            return true;
        }

        private GeoPoint2D[] newtonFindTangent(GeoVector dir3d, GeoPoint2D sp, GeoPoint2D ep, GeoVector sn, GeoVector en, double precision)
        {
            double scsn = dir3d * sn; // Skalarprodukt am Anfang und am Ende, sollte verschiedene Vorzeichen haben, sonst schneidet die Linie die Fläche
            double scen = dir3d * en;
            if (Math.Sign(scsn) == Math.Sign(scen))
            {   // gleiche Richtung, d.h. aufteilen, sollte selten vorkommen
                if (scsn == 0.0)
                {   // an beiden Stellen tangential, wir suchen hier nur dazwischen, wenn in der Mitte nicht tangential ist
                    GeoPoint2D mp = new GeoPoint2D(sp, ep);
                    GeoPoint mp3d;
                    GeoVector mn;
                    ParallelepipedHull.RawPointNormalAt(mp, out mp3d, out mn); // Punkt und Normale auf die Fläche am Mittelpunkt
                    double scmn = dir3d * mn;
                    if (scmn != 0.0)
                    {
                        GeoPoint2D[] t1 = newtonFindTangent(dir3d, sp, mp, sn, mn, precision);
                        GeoPoint2D[] t2 = newtonFindTangent(dir3d, mp, ep, mn, en, precision);
                        if (t1.Length == 0) return t2;
                        if (t2.Length == 0) return t1;
                        GeoPoint2D[] res = new GeoPoint2D[t1.Length + t2.Length];
                        Array.Copy(t1, res, t1.Length);
                        Array.Copy(t2, 0, res, t1.Length, t2.Length);
                        return res;
                    }
                    else
                    {
                        return new GeoPoint2D[0]; // d.h. überall tangential
                    }
                }
                else
                {   // gleiche Richtung, in der Mitte aufteilen, wenn nicht schon zu klein
                    if ((sp | ep) < precision) return new GeoPoint2D[0];
                    GeoPoint2D mp = new GeoPoint2D(sp, ep);
                    GeoPoint mp3d;
                    GeoVector mn;
                    ParallelepipedHull.RawPointNormalAt(mp, out mp3d, out mn); // Punkt und Normale auf die Fläche am Mittelpunkt
                    double scmn = dir3d * mn;
                    if (Math.Sign(scmn) != Math.Sign(scsn))
                    {
                        GeoPoint2D[] t1 = newtonFindTangent(dir3d, sp, mp, sn, mn, precision);
                        GeoPoint2D[] t2 = newtonFindTangent(dir3d, mp, ep, mn, en, precision);
                        if (t1.Length == 0) return t2;
                        if (t2.Length == 0) return t1;
                        GeoPoint2D[] res = new GeoPoint2D[t1.Length + t2.Length];
                        Array.Copy(t1, res, t1.Length);
                        Array.Copy(t2, 0, res, t1.Length, t2.Length);
                        return res;
                    }
                    else
                    {   // pathologischer Fall: in der Nähe einer Singularität oder so
                        return new GeoPoint2D[0]; // d.h. überall tangential
                    }
                }
            }
            else
            {
                bool doNewton = true; // wenn das Ergebnis mal schlechter wird, dann auf Bisection umschalten
                int sgnsp = Math.Sign(scsn);
                int sgnep = Math.Sign(scen);
                if (sgnsp == 0) sgnsp = -sgnep;
                if (sgnep == 0) sgnep = -sgnsp; // jetzt sicher -1 und +1
                int dbgc = 0;
                while ((sp | ep) > precision && dbgc < 10)
                {
                    ++dbgc;
                    double s = 0.5;
                    if (doNewton)
                    {
                        s = -scsn / (scen - scsn); // sollte zwischen 0 und 1 liegen, da verschiedene Vorzeichen wird nie durch 0 geteilt
                        if (s < 0.1) s = 0.1;
                        if (s > 0.9) s = 0.9; // damit man nicht am Anfang oder Ende kleben bleibt
                    }
                    GeoPoint2D mp = Geometry.LinePos(sp, ep, s);
                    // GeoVector mn = GetNormal(mp);// Normalenvektor am Zwischenpunkt, leider normiert, passt nicht mit dem anderen zusammen
                    GeoPoint mp3d;
                    GeoVector mn;
                    ParallelepipedHull.RawPointNormalAt(mp, out mp3d, out mn); // Punkt und Normale auf die Fläche am Mittelpunkt
                    double scmn = dir3d * mn;
                    if (scmn == 0.0) return new GeoPoint2D[] { mp }; // genauer Treffer
                    if (Math.Sign(scmn) != sgnsp)
                    {   // mit dem Anfangsstück weitermachen
                        ep = mp;
                        if (Math.Abs(scen) < Math.Abs(scmn)) doNewton = false; // muss kleiner werden, sonst kein Newton
                        scen = scmn;
                    }
                    else
                    {
                        sp = mp;
                        if (Math.Abs(scsn) < Math.Abs(scmn)) doNewton = false;
                        scsn = scmn;
                    }
                }
                return new GeoPoint2D[] { new GeoPoint2D(sp, ep) };
            }

        }

        private bool NewtonIntersect(Polynom implicitSurface, BoundingRect uvExtent, ICurve curve, out double[] uOnCurve3Ds)
        {
            TetraederHull th = new TetraederHull(curve);
            double[] ips = th.Intersect(implicitSurface);
            uOnCurve3Ds = null;
            return false;
        }
        private bool FindExtreme(GeoVector dir, GeoPoint2D par1, GeoPoint2D par2, GeoPoint2D par3, GeoVector v1, GeoVector v2, GeoVector v3, double epsu, double epsv, out GeoPoint2D p)
        {   // die Vektoren sind nicht normiert, es kann vorkommen, dass es null-Vektoren gibt
            // An 3 Stellen der Oberfläche sind die Normalenvektoeren gegeben. Wenn sie die gewünschte Richtung einschließen
            // dann sollte die Stelle an der der Normalenvektor und dir identisch sind innerhalb des dreiecks liegen
            // bei Sattelflächen kann es auch nach außen wendern
            Matrix m = DenseMatrix.OfRowArrays(v1, v2, v3);
            Vector b = new DenseVector(dir);
            try
            {
                Vector s = (Vector)m.Transpose().Solve(b);
                if ((s[0] >= 0.0 && s[1] >= 0.0 && s[2] >= 0.0) ||
                    (s[0] <= 0.0 && s[1] <= 0.0 && s[2] <= 0.0))
                {   // die gesuchte Richtung wird positiv oder negativ aufgespannt
                    if (Math.Abs(par1.x - par2.x) + Math.Abs(par2.x - par3.x) + Math.Abs(par3.x - par1.x) > epsu ||
                    Math.Abs(par1.y - par2.y) + Math.Abs(par2.y - par3.y) + Math.Abs(par3.y - par1.y) > epsv)
                    {   // noch nicht genau genug
                        // Zwischenpunkte betrachten: die Dreieckseiten werden halbiert und die vier entstehenden
                        // Dreiecke weiter betrachtet
                        GeoPoint2D par12 = new GeoPoint2D(par1, par2);
                        GeoPoint2D par23 = new GeoPoint2D(par2, par3);
                        GeoPoint2D par31 = new GeoPoint2D(par3, par1);
                        GeoVector v12 = GetNormal(par12);
                        GeoVector v23 = GetNormal(par23);
                        GeoVector v31 = GetNormal(par31);
                        // die 4 Teildreicke betrachten
                        if (FindExtreme(dir, par12, par23, par31, v12, v23, v31, epsu, epsv, out p)) return true;
                        if (FindExtreme(dir, par1, par12, par31, v1, v12, v31, epsu, epsv, out p)) return true;
                        if (FindExtreme(dir, par2, par12, par23, v2, v12, v23, epsu, epsv, out p)) return true;
                        if (FindExtreme(dir, par3, par23, par31, v3, v23, v31, epsu, epsv, out p)) return true;
                        // hier angekommen liegt das gesuchte Ergebnis außerhalb des Dreiecks durch par1,par2,par3
                        // Das passiert bei Sattelflächen (z.B. beim Torus)
                        // es ist also eine der drei ursprünglichen Dreiecksseiten, aus dem wir hier rausgefallen sind
                        m = DenseMatrix.OfRowArrays(v1, v12, v2);
                        try
                        {
                            s = (Vector)m.Transpose().Solve(b);
                            if ((s[0] >= 0.0 && s[1] >= 0.0 && s[2] >= 0.0) ||
                                (s[0] <= 0.0 && s[1] <= 0.0 && s[2] <= 0.0))
                            {
                                // die Verbindung par1<->par2 macht ein Problem
                                // spiegele par3 an par12 und suche in den beiden Dreiecken
                                // die Seite par1<->par2 kommt in der neuen Suche nicht vor
                                GeoPoint2D par4 = par12 + (par12 - par3);
                                GeoVector v4 = GetNormal(par4).Normalized;
                                if (FindExtreme(dir, par1, par12, par4, v1, v12, v4, epsu, epsv, out p)) return true;
                                if (FindExtreme(dir, par2, par12, par4, v2, v12, v4, epsu, epsv, out p)) return true;
                            }
                        }
                        catch (ApplicationException) { } // macht nix, liegen in einer Ebene, waren also nicht der Auslöser für das Problem
                        // analog mit den beiden anderen Seiten
                        m = DenseMatrix.OfRowArrays(v2, v23, v3);
                        try
                        {
                            s = (Vector)m.Transpose().Solve(b);
                            if ((s[0] >= 0.0 && s[1] >= 0.0 && s[2] >= 0.0) ||
                                (s[0] <= 0.0 && s[1] <= 0.0 && s[2] <= 0.0))
                            {
                                GeoPoint2D par4 = par23 + (par23 - par1);
                                GeoVector v4 = GetNormal(par4).Normalized;
                                if (FindExtreme(dir, par2, par23, par4, v2, v23, v4, epsu, epsv, out p)) return true;
                                if (FindExtreme(dir, par3, par23, par4, v3, v23, v4, epsu, epsv, out p)) return true;
                            }
                        }
                        catch (ApplicationException) { }
                        m = DenseMatrix.OfRowArrays(v3, v31, v1);
                        try
                        {
                            s = (Vector)m.Transpose().Solve(b);
                            if ((s[0] >= 0.0 && s[1] >= 0.0 && s[2] >= 0.0) ||
                                (s[0] <= 0.0 && s[1] <= 0.0 && s[2] <= 0.0))
                            {
                                GeoPoint2D par4 = par31 + (par31 - par2);
                                GeoVector v4 = GetNormal(par4).Normalized;
                                if (FindExtreme(dir, par3, par31, par4, v3, v31, v4, epsu, epsv, out p)) return true;
                                if (FindExtreme(dir, par1, par31, par4, v1, v31, v4, epsu, epsv, out p)) return true;
                            }
                        }
                        catch (ApplicationException) { }

                        return false; // sollte nicht drankommen
                    }
                    else
                    {   // die Mitte nehmen
                        p = new GeoPoint2D((par1.x + par2.x + par3.x) / 3.0, (par1.y + par2.y + par3.y) / 3.0);
                        return true;
                    }
                }
                // hier liegt die Richtung nicht in der aufgespannten Fläche
            }
            catch (ApplicationException)
            {   // nicht lösbar, also eben oder in einer Richtung linear
            }
            p = GeoPoint2D.Origin;
            return false;
        }
        private bool ApproxExtreme(GeoVector dir, GeoPoint2D par1, GeoPoint2D par2, GeoPoint2D par3, GeoPoint2D par4, GeoVector v1, GeoVector v2, GeoVector v3, GeoVector v4, double epsu, double epsv, out GeoPoint2D p)
        {
            // KONVERGIERT NICHT GUT, man müsste mit den 2. Ableitungen arbeiten
            // Alle Vektoren sind normiert.
            // Bestimme den schlechtesten, damit dieser nachher ausgetauscht wird
            int worst = 0;
            double d = double.MaxValue;
            double sc = dir * v1;
            if (sc < d)
            {
                worst = 1;
                d = sc;
            }
            sc = Math.Abs(dir * v2);
            if (sc < d)
            {
                worst = 2;
                d = sc;
            }
            sc = Math.Abs(dir * v3);
            if (sc < d)
            {
                worst = 3;
                d = sc;
            }
            sc = Math.Abs(dir * v4);
            if (sc < d)
            {
                worst = 4;
                d = sc;
            }
            while (d < 1 - 1e-6)
            {
                // Gleichungssystem:
                // a1*par1u + b1*par1v + c1 = v1x
                // a2*par1u + b2*par1v + c2 = v1y
                // a3*par1u + b3*par1v + c3 = v1z und das Gleiche mit par2, par3 und par4 bzw v2, v3 und v4
                // ergibt 12 Gleichungen mit 9 Unbekannten. Würde man nur die drei auf einer Linie liegenden
                // parameterwerte nehmen, wäre das System linear abhängig
                // Aus der Lösung (a1,b1,c1,a2,b2,c2,a3,b3,c3) ergibt sich folgendes:
                // a1*paru + b1*parv + c1 = dirx
                // a2*paru + b2*parv + c2 = diry
                // a3*paru + b3*parv + c3 = dirz wobei hier paru ubd parv gesucht sind. Zu beachten ist, dass elle Vekoren
                // normiert sein müssen. Das System ist überbestimmt
                Matrix m = new DenseMatrix(12, 9);
                m[0, 0] = par1.x; m[0, 1] = par1.y; m[0, 2] = 1.0;
                m[1, 3] = par1.x; m[1, 4] = par1.y; m[1, 5] = 1.0;
                m[2, 6] = par1.x; m[2, 7] = par1.y; m[2, 8] = 1.0;

                m[3, 0] = par2.x; m[3, 1] = par2.y; m[3, 2] = 1.0;
                m[4, 3] = par2.x; m[4, 4] = par2.y; m[4, 5] = 1.0;
                m[5, 6] = par2.x; m[5, 7] = par2.y; m[5, 8] = 1.0;

                m[6, 0] = par3.x; m[6, 1] = par3.y; m[6, 2] = 1.0;
                m[7, 3] = par3.x; m[7, 4] = par3.y; m[7, 5] = 1.0;
                m[8, 6] = par3.x; m[8, 7] = par3.y; m[8, 8] = 1.0;

                m[9, 0] = par3.x; m[9, 1] = par3.y; m[9, 2] = 1.0;
                m[10, 3] = par3.x; m[10, 4] = par3.y; m[10, 5] = 1.0;
                m[11, 6] = par3.x; m[11, 7] = par3.y; m[11, 8] = 1.0;

                Vector b = new DenseVector(12);
                b[0] = v1.x;
                b[1] = v1.y;
                b[2] = v1.z;
                b[3] = v2.x;
                b[4] = v2.y;
                b[5] = v2.z;
                b[6] = v3.x;
                b[7] = v3.y;
                b[8] = v3.z;
                b[9] = v4.x;
                b[10] = v4.y;
                b[11] = v4.z;
                try
                {
                    Vector s = (Vector)m.Solve(b);
                    m = new DenseMatrix(3, 2);
                    m[0, 0] = s[0];
                    m[0, 1] = s[1];
                    m[1, 0] = s[3];
                    m[1, 1] = s[4];
                    m[2, 0] = s[6];
                    m[2, 1] = s[7];
                    b = new DenseVector(3);
                    b[0] = dir.x - s[2];
                    b[1] = dir.y - s[5];
                    b[2] = dir.z - s[8];
                    s = (Vector)m.Solve(b);
                    p.x = s[0];
                    p.y = s[1];
                    GeoVector v = GetNormal(p).Normalized;

                    double newsc = dir * v;
                    if (newsc < d)
                    {   // konvergiert nicht
                        p = GeoPoint2D.Origin;
                        return false;
                    }
                    switch (worst)
                    {
                        case 1:
                            par1 = p;
                            v1 = v;
                            break;
                        case 2:
                            par2 = p;
                            v2 = v;
                            break;
                        case 3:
                            par3 = p;
                            v3 = v;
                            break;
                        case 4:
                            par4 = p;
                            v4 = v;
                            break;
                    }
                    d = double.MaxValue;
                    sc = Math.Abs(dir * v1);
                    if (sc < d)
                    {
                        worst = 1;
                        d = sc;
                    }
                    sc = Math.Abs(dir * v2);
                    if (sc < d)
                    {
                        worst = 2;
                        d = sc;
                    }
                    sc = Math.Abs(dir * v3);
                    if (sc < d)
                    {
                        worst = 3;
                        d = sc;
                    }
                    sc = Math.Abs(dir * v4);
                    if (sc < d)
                    {
                        worst = 4;
                        d = sc;
                    }
                }
                catch (ApplicationException)
                {
                    p = GeoPoint2D.Origin;
                    return false;
                }
            }
            p = new GeoPoint2D((par1.x + par2.x + par3.x + par4.x) / 4.0, (par1.y + par2.y + par3.y + par4.y) / 4.0);
            GeoVector dbg = GetNormal(p).Normalized;
            return true;
        }
        internal static void Debug(Model m)
        {
        }
        internal void CheckZMinMax(Projection p, double u, double v, ref double zMin, ref double zMax)
        {
            GeoPoint pp = p.UnscaledProjection * (this as ISurface).PointAt(new GeoPoint2D(u, v));
            if (pp.z < zMin) zMin = pp.z;
            if (pp.z > zMax) zMax = pp.z;
        }
        public BSpline Refine(GeoPoint[] geopoints, int degree, bool closed, PlaneSurface pl, double precision)
        {
            Plane pln = new Plane(pl.Location, pl.DirectionX, pl.DirectionY);
            List<GeoPoint> p = new List<GeoPoint>(geopoints);
            BSpline bsp = BSpline.Construct();
            bsp.ThroughPoints(p.ToArray(), degree, closed);
            return bsp; //Debug

            //Unreachable code
            /*
            int maxThroughPoints = 200;
            int next_index = 1;
            int initp = p.Count;
            bool tocheck = false;
            //System.Diagnostics.Trace.WriteLine("==> Einmal mit: "+ p.Count + " Punkten");
            double us = 0;
            double ue = 1;
            do
            {
                double um = us;
                int i = next_index;
                if (i == 1)
                {
                    //um = (us + (bsp as ICurve).PositionOf(p[i])) / 2;
                    um = (us + bsp.PositionOfThroughPoint(i)) / 2;
                }
                else if (i == p.Count - 1)
                {
                    //um = ((bsp as ICurve).PositionOf(p[i - 1]) + ue) / 2;
                    um = (bsp.PositionOfThroughPoint(i) + ue) / 2;
                }
                else
                {
                    //um = ((bsp as ICurve).PositionOf(p[i - 1]) + (bsp as ICurve).PositionOf(p[i])) / 2;
                    um = (bsp.PositionOfThroughPoint(i - 1) + bsp.PositionOfThroughPoint(i)) / 2;
                }

                GeoPoint pm = (bsp as ICurve).PointAt(um);
                GeoVector dirline = pln.Normal ^ (bsp as ICurve).DirectionAt(um);
                GeoPoint2D[] sp = GetLineIntersection(pm, dirline);
                GeoPoint pcor = new GeoPoint();
                if (sp.Length != 0)
                {
                    double dmi = double.MaxValue;
                    for (int k = 0; k < sp.Length; k++)
                    {
                        GeoPoint tmp = (this as ISurface).PointAt(sp[k]);
                        double d = Geometry.Dist(pm, tmp);
                        if (d < dmi)
                        {
                            dmi = d;
                            pcor = tmp;
                        }
                    }
                    if (dmi <= precision)
                    {
                        next_index += 1;
                    }
                    else
                    {
                        next_index += 2;
                        p.Insert(i, pcor);
                        tocheck = true;
                    }
                }
                else
                {
                    next_index += 1;
                }
            } while (next_index < p.Count);

            if (tocheck && p.Count < maxThroughPoints)
            {
                return Refine(p.ToArray(), degree, closed, pl, precision);
            }
            else
            {
                if (initp == p.Count)
                    return bsp;
                else
                {
                    BSpline bspr = BSpline.Construct();
                    bspr.ThroughPoints(p.ToArray(), degree, closed);
                    return bspr;
                }
            }
            */
        }

        #region IOctTreeInsertable Members

        BoundingBox IOctTreeInsertable.GetExtent(double precision)
        {   // IOctTreeInsertable wird nur verwendet um octtree.GetObjectsCloseTo aufzurufen, das sollte keinen extent verlangen
            throw new Exception("The method or operation is not implemented.");
        }

        bool IOctTreeInsertable.HitTest(ref BoundingBox cube, double precision)
        {
            GeoPoint2D uv;
            return HitTest(cube, out uv);
        }

        bool IOctTreeInsertable.HitTest(Projection projection, BoundingRect rect, bool onlyInside)
        {
            throw new Exception("The method or operation is not implemented.");
        }

        bool IOctTreeInsertable.HitTest(Projection.PickArea area, bool onlyInside)
        {
            throw new Exception("The method or operation is not implemented.");
        }

        double IOctTreeInsertable.Position(GeoPoint fromHere, GeoVector direction, double precision)
        {
            throw new Exception("The method or operation is not implemented.");
        }
        private static double DistanceOfLineIntersection(ISurface s1, ISurface s2, GeoPoint startPoint, GeoVector lineDirection)
        {
            GeoPoint2D[] ip1 = s1.GetLineIntersection(startPoint, lineDirection);
            if (ip1 == null || ip1.Length == 0) return double.NaN;
            GeoPoint2D[] ip2 = s2.GetLineIntersection(startPoint, lineDirection);
            if (ip2 == null || ip2.Length == 0) return double.NaN;
            var uv1 = ip1.MinBy(uv => s1.PointAt(uv) | startPoint);
            var uv2 = ip2.MinBy(uv => s2.PointAt(uv) | startPoint);
            double pos1 = Geometry.LinePar(startPoint, lineDirection, s1.PointAt(uv1));
            double pos2 = Geometry.LinePar(startPoint, lineDirection, s2.PointAt(uv2));
            return pos2 - pos1;
        }

        private IDualSurfaceCurve[] NewGetDualSurfaceCurves(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, List<GeoPoint> seeds, out int numTangentialSeeds)
        {
            // the bounds must include the seeds, otherwise the steps for seeking the next point immediately stop, because we are outside the bounds
            for (int i = 0; i < seeds.Count; i++)
            {
                GeoPoint2D uv = PositionOf(seeds[i]);
                SurfaceHelper.AdjustPeriodic(this, thisBounds, ref uv);
                thisBounds.MinMax(uv);
                uv = other.PositionOf(seeds[i]);
                SurfaceHelper.AdjustPeriodic(other, otherBounds, ref uv);
                otherBounds.MinMax(uv);

            }
            thisBounds.Inflate(1e-6);
            otherBounds.Inflate(1e-6);
#if DEBUG
            // for debugging there are 3 views:
            // dc: the two surfaces with their domains as faces end the accumulated 3d points
            // dc21: the uv area of this surface/ donmain and accumulated 2d points
            // dc22: same for the other surface
            DebuggerContainer dc = new DebuggerContainer();
            dc.Add(Face.MakeFace(this, thisBounds), Color.Red);
            dc.Add(Face.MakeFace(other, otherBounds), Color.Green);
            DebuggerContainer dc21 = new DebuggerContainer();
            dc21.Add(thisBounds.ToBorder(), Color.Red, 0);
            DebuggerContainer dc22 = new DebuggerContainer();
            dc22.Add(otherBounds.ToBorder(), Color.Green, 0);
            for (int i = 0; i < seeds.Count; i++)
            {
                dc.Add(seeds[i], Color.Blue, i);
                dc21.Add(this.PositionOf(seeds[i]), Color.Blue, i);
                dc22.Add(other.PositionOf(seeds[i]), Color.Blue, i);
            }
#endif
            numTangentialSeeds = 0;
            if (seeds.Count < 2) return null;
            HashSet<ParallelepipedHull.ParEpi> pes;
            if (other is ISurfaceImpl si) pes = ParallelepipedHull.GetCommonParEpis((other as ISurfaceImpl).ParallelepipedHull);
            else return null;
            if (pes.Count == 0) return new IDualSurfaceCurve[0]; // no intersection, because there are no common parepis
            // find a rough estimate of the size of the intersection curves
            BoundingBox ext = BoundingBox.EmptyBoundingBox;
            foreach (var parepi in pes)
            {
                ext.MinMax(parepi.pll);
                ext.MinMax(parepi.pur);
            }
            double totalSize = ext.Size;
            double maxBend = Math.PI / 6; // maximum bending angle between two steps (30°)
            List<IDualSurfaceCurve> res = new List<IDualSurfaceCurve>();
            double minDist = double.MaxValue;
            for (int i = 0; i < seeds.Count - 1; i++)
            {
                for (int j = i + 1; j < seeds.Count; j++)
                {
                    double d = seeds[i] | seeds[j];
                    if (d > Precision.eps && d < minDist) minDist = d;
                }
            }
            foreach (GeoPoint seed in seeds)
            {
                double stepLength = Math.Min(totalSize * 0.05, minDist * 0.1); // 5% of the size of the intersection curves or 1/10 of the minimal distance between seeds
                GeoPoint2D seeduvthis = PositionOf(seed);
                GeoPoint2D seeduvother = other.PositionOf(seed);
                SurfaceHelper.AdjustPeriodic(this, thisBounds, ref seeduvthis);
                SurfaceHelper.AdjustPeriodic(other, otherBounds, ref seeduvother);
                GeoVector dir = (GetNormal(seeduvthis).Normalized ^ other.GetNormal(seeduvother).Normalized);
                if (Precision.IsNullVector(dir))
                {
                    ++numTangentialSeeds;
                    continue; // tangential surfaces, cannot proceed
                }
                dir.Norm();
                //DerivationAt(uvthis, out GeoPoint p3d, out GeoVector duthis, out GeoVector dvthis);
                //other.DerivationAt(uvother, out GeoPoint op3d, out GeoVector duother, out GeoVector dvother);
                // we are looking for a reasonable length for a step in the direction of the intersection curve
                //double stepLength = (thisBounds.Width * duthis.Length + thisBounds.Height * dvthis.Length) + (otherBounds.Width * duother.Length + otherBounds.Height * dvother.Length);
                //stepLength *= 0.01; // 1% of the average extension of the surfaces
                bool ok = false;
                for (int i = 0; i < 10; i++) // try 10 times to find a reasonable stepLength
                {
                    PlaneSurface ps = new PlaneSurface(new Plane(seed + stepLength * dir, dir));
                    GeoPoint2D uvPlane = GeoPoint2D.Origin;
                    GeoPoint ip = seed;
                    GeoPoint2D uvthis = seeduvthis;
                    GeoPoint2D uvother = seeduvother;
                    if (!BoxedSurfaceExtension.SurfacesIntersectionLM(ps, this, other, ref uvPlane, ref uvthis, ref uvother, ref ip))
                    {
                        stepLength /= 2;
                    }
                    else
                    {
                        GeoVector nextdir = (GetNormal(uvthis) ^ other.GetNormal(uvother)).Normalized;
                        SweepAngle sa = new SweepAngle(dir, nextdir);
                        if (Math.Abs(sa) > maxBend)
                        {
                            stepLength /= 2;
                        }
                        else
                        {
                            ok = true;
                            break; // stepLength is ok
                        }
                    }
                }
                if (!ok) continue; // could not find a reasonable stepLength

                foreach (int forward in new[] { 1, -1 })
                {
                    Plane lastplane = new Plane(seed, dir); // to check whether we cross a seed
                    List<GeoPoint> ips = new List<GeoPoint>();
                    ips.Add(seed);
                    List<InterpolatedDualSurfaceCurve.SurfacePoint> surfacePoints = new List<InterpolatedDualSurfaceCurve.SurfacePoint>();
                    surfacePoints.Add(new InterpolatedDualSurfaceCurve.SurfacePoint(seed, seeduvthis, seeduvother));
                    GeoPoint endedAtSeed = GeoPoint.Invalid; // if we crossed a seed, we stop there
                    GeoPoint2D uvthis = seeduvthis;
                    GeoPoint2D uvother = seeduvother;
                    while (true)
                    {
                        GeoPoint2D uvt = uvthis, uvo = uvother;
                        GeoPoint2D uvPlane = GeoPoint2D.Origin;
                        double sl = stepLength;
                        ok = false;
                        bool reversed = false;
                        for (int i = 0; i < 5; i++) // try 5 times to find the next intersection point with decreasing step length
                        {
                            GeoPoint ip = ips.Last();
                            uvthis = uvt; uvother = uvo;
                            uvPlane = GeoPoint2D.Origin;
                            PlaneSurface ps = new PlaneSurface(new Plane(ip + forward * sl * dir, dir));
                            ip = ip + forward * sl * dir;
                            uvthis = this.PositionOf(ip);
                            uvother = other.PositionOf(ip);
                            reversed = false;
#if DEBUG
                            dc.Add(ip, Color.DeepPink, 9999);
                            dc21.Add(uvthis, Color.DeepPink, 9999);
                            dc22.Add(uvother, Color.DeepPink, 9999);
#endif
                            if (!SurfaceIntersectionSolvers.SurfacesIntersectionLM_Analytic9(ps, this, other, ref uvPlane, ref uvthis, ref uvother, ref ip))
                            {
                                //bool ok2 = SurfaceIntersectionSolvers.SurfacesIntersectionLM_Analytic6(ps, this, other, ref uvPlane, ref uvthis, ref uvother, ref ip);
                                //this.DerivationAt(uvthis, out GeoPoint loc1, out GeoVector du1, out GeoVector dv1);
                                //SurfaceIntersectionSolvers.NumericalDerivationAt(this, uvthis, out GeoPoint loc2, out GeoVector du2, out GeoVector dv2);

                                //bool ok1 = SurfaceIntersectionSolvers.SurfacesIntersectionLM_CentralFD9(ps, this, other, ref uvPlane, ref uvthis, ref uvother, ref ip);

                                sl = sl / 2;
                                GeoVector nt = GetNormal(uvthis).Normalized;
                                GeoVector no = other.GetNormal(uvother).Normalized;
                                GeoVector probeDirection = (nt + no).Normalized;
                                SweepAngle sw = new SweepAngle(nt, no);
                                if (Math.Abs(sw) < 0.1)
                                {   // we are approaching a touching point
                                    GeoVector searchDirection = (dir ^ probeDirection).Normalized;
                                    DistanceOfLineIntersection(this, other, ip, probeDirection);
                                }
                            }
                            else
                            {
                                SurfaceHelper.AdjustPeriodic(this, thisBounds, ref uvthis);
                                SurfaceHelper.AdjustPeriodic(other, otherBounds, ref uvother);
                                GeoVector nextdir = (GetNormal(uvthis) ^ other.GetNormal(uvother)).Normalized;
                                if (Precision.IsNullVector(nextdir))
                                {   // exactely on a touching point
#if DEBUG
                                    dc.Add(ip, Color.Magenta, 9999);
                                    dc21.Add(uvthis, Color.Magenta, 9999);
                                    dc22.Add(uvother, Color.Magenta, 9999);
#endif
                                    sl /= 2;
                                }
                                else
                                {
                                    nextdir.Norm();
                                    SweepAngle sa = new SweepAngle(dir, nextdir);
                                    // if we cross a touching point, the direction may reverse. We have to consider touching points seperately!
                                    if (Math.Abs(sa) > maxBend && Math.Abs(Math.PI - sa) < maxBend)
                                    {
                                        // very sharp bending, probably reversing the direction: we could have crossed a touching point!
                                        reversed = true;
                                        ok = false;
                                        ips.Add(ip);
                                        surfacePoints.Add(new InterpolatedDualSurfaceCurve.SurfacePoint(ip, uvthis, uvother));
                                        dir = nextdir; // uvthis and uvother are already adjusted
                                        break; // intersection point found with small bending angle
                                    }
                                    else if (Math.Abs(sa) > maxBend) // bending too sharp
                                    {
#if DEBUG
                                        dc.Add(ip, Color.DarkCyan, 9999);
                                        dc21.Add(uvthis, Color.DarkCyan, 9999);
                                        dc22.Add(uvother, Color.DarkCyan, 9999);
#endif
                                        sl /= 2;
                                    }
                                    else
                                    {
#if DEBUG
                                        dc.Add(ip, Color.DarkSalmon, 9999);
                                        dc21.Add(uvthis, Color.DarkSalmon, 9999);
                                        dc22.Add(uvother, Color.DarkSalmon, 9999);
                                        if (surfacePoints.Count > 0)
                                        {
                                            dc.Add(Line.TwoPoints(ips[ips.Count - 1], ip), Color.Orange, 9999);
                                            dc21.Add(new Line2D(surfacePoints[surfacePoints.Count - 1].psurface1, uvthis), Color.Orange, 9999);
                                            dc22.Add(new Line2D(surfacePoints[surfacePoints.Count - 1].psurface2, uvother), Color.Orange, 9999);
                                        }
#endif
                                        ok = true;
                                        ips.Add(ip);
                                        surfacePoints.Add(new InterpolatedDualSurfaceCurve.SurfacePoint(ip, uvthis, uvother));
                                        dir = nextdir; // uvthis and uvother are already adjusted
                                        break; // intersection point found with small bending angle
                                    }
                                }
                            }
                        }
                        if (!ok)
                        {   // maybe we are approaching a tangential point and SurfacesIntersectionLM doesnt converge any more
                            if (ips.Count > 2 && reversed)
                            {
                                foreach (GeoPoint seedTest in seeds)
                                {
                                    if (!Precision.IsEqual(seedTest, ips[0]) && Geometry.IsNearSegment(ips[ips.Count - 1], ips[ips.Count - 2], seedTest, maxBend))
                                    {
                                        // there is a close seed, not too much bended from last point, lets use it as the endpoint of our sequence
                                        ips[ips.Count - 1] = seedTest; // replace the last intersection point with the seed
                                        uvthis = PositionOf(seedTest);
                                        uvother = other.PositionOf(seedTest);
                                        surfacePoints[surfacePoints.Count - 1] = new InterpolatedDualSurfaceCurve.SurfacePoint(seedTest, uvthis, uvother);
                                        break;
                                    }
                                }
                            }
                            break; // finish the search for the next intersection point, we could not find one
                        }
                        Plane currentplane = new Plane(ips.Last(), dir);
                        SurfaceHelper.AdjustPeriodic(this, thisBounds, ref uvthis);
                        SurfaceHelper.AdjustPeriodic(other, otherBounds, ref uvother);
                        // test for crossing a seed before checking the bounds, because the last intersection point could be outside the bounds
                        foreach (GeoPoint seedTest in seeds)
                        {
                            if (!Precision.IsEqual(seedTest, ips[0]) && !Precision.IsEqual(seedTest, ips[ips.Count - 1]) && lastplane.Distance(seedTest) * currentplane.Distance(seedTest) < 0)
                            {   // this seed is on different sides of the planes
                                if (Geometry.IsNearSegment(ips[ips.Count - 2], ips[ips.Count - 1], seedTest, maxBend))
                                {   // we crossed a seed, stop here
                                    ok = false;
                                    endedAtSeed = seedTest;
                                    ips[ips.Count - 1] = seedTest; // replace the last intersection point with the seed
                                    uvthis = PositionOf(seedTest);
                                    uvother = other.PositionOf(seedTest);
                                    SurfaceHelper.AdjustPeriodic(this, thisBounds, ref uvthis);
                                    SurfaceHelper.AdjustPeriodic(other, otherBounds, ref uvother);
                                    surfacePoints[surfacePoints.Count - 1] = new InterpolatedDualSurfaceCurve.SurfacePoint(seedTest, uvthis, uvother);
                                    break;
                                }
                            }
                        }
                        lastplane = currentplane;
                        if (!ok) break;
                        if (!thisBounds.Contains(uvthis) || !otherBounds.Contains(uvother)) break; // outside the bounds of one of the surfaces
                    }
                    // Polyline pl = Polyline.FromPoints(ips.ToArray());
                    if (ips.Count > 1 && seeds.Any(p => Precision.IsEqual(p, ips[0])) && seeds.Any(p => Precision.IsEqual(p, ips[ips.Count - 1])))
                    { // start and endpoint are seeds
                        res.Add(new InterpolatedDualSurfaceCurve(this, thisBounds, other, otherBounds, surfacePoints.ToArray()));
                        if (seeds.Count == 2 && !Precision.IsEqual(ips[0], ips[ips.Count - 1]))
                        {   // only two seeds and the curve uses both, so we are done
                            // this is a very common case, we return the first intersection curve we found
                            return res.ToArray();
                        }
                    }
                }
            }
            // we have to check, whether there are duplicate curves in the result
            for (int i = 0; i < res.Count - 1; i++)
            {
                for (int j = i + 1; j < res.Count; j++)
                {
                    if (Precision.IsEqual(res[i].Curve3D.EndPoint, res[j].Curve3D.StartPoint) && Precision.IsEqual(res[i].Curve3D.StartPoint, res[j].Curve3D.EndPoint))
                    {   // it still could be two segments of a closed curve
                        if (res[i].Curve3D.DistanceTo(res[j].Curve3D.PointAt(0.5)) < Precision.eps)
                        {
                            res.RemoveAt(j); // two identical curves in opposite direction
                            break;
                        }
                    }
                }
            }
            return res.ToArray();
        }
        public virtual IDualSurfaceCurve[] GetDualSurfaceCurves(BoundingRect thisBounds, ISurface other, BoundingRect otherBounds, List<GeoPoint> seeds, List<Tuple<double, double, double, double>> extremePositions)
        {
            if ((extremePositions == null || extremePositions.Count == 0) && seeds.Count >= 2)
            {
                // we are testing here with a new and hopefully more robust and faster approach
                try
                {
                    IDualSurfaceCurve[] testWithNewAlgorithm = NewGetDualSurfaceCurves(thisBounds, other, otherBounds, seeds, out int numTangentialSeeds);
                    if (testWithNewAlgorithm != null && testWithNewAlgorithm.Length > 0) return testWithNewAlgorithm;
                    if (seeds.Count == numTangentialSeeds && seeds.Count == 2)
                    {   // two seeds, both tangential, try to make an InterpolatedDualSurfaceCurve
                        InterpolatedDualSurfaceCurve dsc = new InterpolatedDualSurfaceCurve(this, thisBounds, other, otherBounds, seeds[0], seeds[1], true);
                    }
                }
                catch (Exception)
                {

                }
            }
            // fallback to old method
            if (this is ISurfaceOfRevolution && other is ISurfaceOfRevolution && !(this is SurfaceOfRevolution) && !(other is SurfaceOfRevolution)) // SurfaceOfRevolution rotates in v!
            {
                ISurfaceOfRevolution tsor = (this as ISurfaceOfRevolution);
                ISurfaceOfRevolution osor = (other as ISurfaceOfRevolution);
                if (Precision.SameDirection(tsor.Axis.Direction, osor.Axis.Direction, false))
                {
                    if (Geometry.DistPL(osor.Axis.Location, tsor.Axis.Location, tsor.Axis.Direction) < Precision.eps)
                    {   // identical axis of this surface and other surface
                        // The plane given by the location and common axis, and the startpoint of the 3d curve. The plane coordinate system is normalized
                        try
                        {
                            ICurve tfu = this.FixedU(thisBounds.Left, thisBounds.Bottom, thisBounds.Top);
                            ICurve ofu = other.FixedU(other.PositionOf(tfu.StartPoint).x, otherBounds.Bottom, otherBounds.Top);
                            Plane cpln;
                            if (Curves.GetCommonPlane(tfu, ofu, out cpln))
                            {
                                double[] ippars = Curves.Intersect(tfu, ofu, true);

                                //ICurve tcrv = tsor.Curve;
                                //GeoVector tdir = tcrv.StartDirection;
                                //if (Precision.SameDirection(tdir, tsor.Axis.Direction, false)) tdir = tcrv.EndDirection;
                                //if (Precision.SameDirection(tdir, tsor.Axis.Direction, false)) tdir = tcrv.StartPoint - tsor.Axis.Location;
                                //Plane tpln = new Plane(tsor.Axis.Location, tsor.Axis.Direction, tdir);
                                //ICurve2D t2d = tcrv.GetProjectedCurve(tpln);
                                //ICurve ocrv = osor.Curve;
                                //GeoVector odir = ocrv.StartDirection;
                                //if (Precision.SameDirection(odir, tsor.Axis.Direction, false)) odir = ocrv.EndDirection;
                                //if (Precision.SameDirection(odir, tsor.Axis.Direction, false)) odir = ocrv.StartPoint - tsor.Axis.Location;
                                //Plane opln = new Plane(tsor.Axis.Location, tsor.Axis.Direction, odir);
                                //ICurve2D o2d = ocrv.GetProjectedCurve(opln);
                                //GeoPoint2DWithParameter[] ips = t2d.Intersect(o2d);
                                List<DualSurfaceCurve> dscs = new List<DualSurfaceCurve>();
                                for (int i = 0; i < ippars.Length; i++)
                                {
                                    GeoPoint ip = tfu.PointAt(ippars[i]);
                                    GeoPoint2D tuv = this.PositionOf(ip);
                                    GeoPoint2D ouv = other.PositionOf(ip);
                                    ICurve c3d = this.FixedV(tuv.y, thisBounds.Left, thisBounds.Right);
                                    ICurve oc3d = other.FixedV(ouv.y, otherBounds.Left, otherBounds.Right);
                                    if (c3d is Ellipse && oc3d is Ellipse)
                                    {
                                        Plane pln = (c3d as Ellipse).Plane;
                                        Arc2D tarc = c3d.GetProjectedCurve(pln) as Arc2D; // must be an arc
                                        Arc2D oarc = oc3d.GetProjectedCurve(pln) as Arc2D; // must also be an arc
                                        if (tarc != null && oarc != null)
                                        {
                                            GeoPoint2D tsp = this.PositionOf(c3d.StartPoint);
                                            GeoPoint2D tep = this.PositionOf(c3d.EndPoint);
                                            SurfaceHelper.AdjustPeriodic(this, thisBounds, ref tsp);
                                            SurfaceHelper.AdjustPeriodic(this, thisBounds, ref tep);
                                            GeoPoint2D osp = other.PositionOf(c3d.StartPoint);
                                            GeoPoint2D oep = other.PositionOf(c3d.EndPoint);
                                            SurfaceHelper.AdjustPeriodic(other, otherBounds, ref osp);
                                            SurfaceHelper.AdjustPeriodic(other, otherBounds, ref oep);
                                            Line2D tc2d = new Line2D(tsp, tep);
                                            Line2D oc2d = new Line2D(osp, oep);
                                            // double pt = c3d.PositionOf(this.PointAt(tc2d.PointAt(0.5)));
                                            // double po = c3d.PositionOf(other.PointAt(oc2d.PointAt(0.5)));
                                            // if ((0 < pt && pt < 1) && (0 < po && po < 1)) // should be 0.5 // && was || before, but this was definitely wrong! we must check, whether tarc and oarc overlap
                                            // previous line was wrong, comment was right: we must check, whether tarc and oarc overlap
                                            // These are two 2d arcs on the same planeso at least two start/endpoints must be within the other arc
                                            int inside = 0;
                                            double overlap = 0.0;
                                            double tp = tarc.PositionOf(oarc.StartPoint);
                                            overlap += 0.5 - Math.Abs(tp - 0.5);
                                            if (tp >= 0 && tp <= 1) ++inside;
                                            tp = tarc.PositionOf(oarc.EndPoint);
                                            overlap += 0.5 - Math.Abs(tp - 0.5);
                                            if (tp >= 0 && tp <= 1) ++inside;
                                            tp = oarc.PositionOf(tarc.StartPoint);
                                            overlap += 0.5 - Math.Abs(tp - 0.5);
                                            if (tp >= 0 && tp <= 1) ++inside;
                                            tp = oarc.PositionOf(tarc.EndPoint);
                                            overlap += 0.5 - Math.Abs(tp - 0.5);
                                            if (tp >= 0 && tp <= 1) ++inside;
                                            // it still might be, that the two arcs touch at all endpoints but describe different halfs of a circle
                                            // if all tp are 0 or 1, the resulting overlap will be 0
                                            if (overlap < 10 * Precision.eps)
                                            {   // either full identical or opposite arcs
                                                tp = tarc.PositionOf(oarc.PointAt(0.5));
                                            }
                                            if (inside >= 2 && tp > 0 && tp < 1) dscs.Add(new DualSurfaceCurve(c3d, this, tc2d, other, oc2d));
                                            // else: did choose wrong part of arc, no common intersection curve
                                        }
                                    }
                                }
                                return dscs.ToArray();
                            }
                        }
                        catch
                        {
                            // plane Exception possible?
                        }
                    }
                }
            }

            // sollte für Ebene, Cylinder, Kegel, Kugel, Torus überschrieben werden
            ModOp2D mop;
            if (SameGeometry(thisBounds, other, otherBounds, Precision.eps, out mop)) return new IDualSurfaceCurve[0]; // surfaces are identical, no intersection
            ICurve[] cvs = ParallelepipedHull.Intersect(thisBounds, other, otherBounds, seeds, extremePositions);
            if ((cvs.Length == 0 && seeds != null && seeds.Count == 2))
            {
                InterpolatedDualSurfaceCurve idscv = new InterpolatedDualSurfaceCurve(this, thisBounds, other, otherBounds, seeds[0], seeds[1]);
                return new IDualSurfaceCurve[] { idscv };
            }
            DualSurfaceCurve[] res = new DualSurfaceCurve[cvs.Length];
            for (int i = 0; i < res.Length; i++)
            {
                res[i] = new DualSurfaceCurve(cvs[i], this, this.GetProjectedCurve(cvs[i], Precision.eps), other, other.GetProjectedCurve(cvs[i], Precision.eps));
            }
            return res;
        }

        public virtual ICurve2D[] GetSelfIntersections(BoundingRect bounds)
        {
            return null;
        }

        void IJsonSerialize.GetObjectData(IJsonWriteData data)
        {
            data.AddProperty("Domain", usedArea);
        }

        void IJsonSerialize.SetObjectData(IJsonReadData data)
        {
            usedArea = data.GetProperty<BoundingRect>("Domain");
        }


        #endregion
    }
    internal class FindTangentCurves
    {   // während der Kurvenfindung braucht es einige Daten die global bleiben, damit nicht so viel über
        // die Parameter übergeben werden muss
        class UnableToFindZero : ApplicationException
        {
            int uind;
            int vind;
            public UnableToFindZero(int uind, int vind)
            {
                this.uind = uind;
                this.vind = vind;
            }
        }
        ISurface surface; // die Kurve selbst
        GeoVector direction; // die Betrachtungsrichtung
        double[] usteps; // Parameterschritte
        double[] vsteps;
        double?[,] uSections;
        double?[,] vSections;
        bool[,] uSectionsUsed;
        bool[,] vSectionsUsed;
        double precisionU, precisionV;
        public FindTangentCurves(ISurface surface)
        {
            this.surface = surface;
        }
        private void FindCurveFromUSection(List<GeoPoint2D> curve, int uind, int vind)
        {
            // suche eine Kante, die einen Nullpunkt hat
            // ind ist der Index in den uSections, par der v-Parameter
            if (CheckUSection(curve, uind, vind - 1, uSections[vind, uind].Value, vsteps[vind])) return;
            if (CheckUSection(curve, uind, vind + 1, uSections[vind, uind].Value, vsteps[vind])) return;
            if (CheckVSection(curve, uind, vind, uSections[vind, uind].Value, vsteps[vind])) return;
            if (CheckVSection(curve, uind + 1, vind, uSections[vind, uind].Value, vsteps[vind])) return;
            if (CheckVSection(curve, uind, vind - 1, uSections[vind, uind].Value, vsteps[vind])) return;
            if (CheckVSection(curve, uind + 1, vind - 1, uSections[vind, uind].Value, vsteps[vind])) return;
        }
        private void FindCurveFromVSection(List<GeoPoint2D> curve, int uind, int vind)
        {
            // suche eine Kante, die einen Nullpunkt hat
            // ind ist der Index in den uSections, par der v-Parameter
            if (CheckVSection(curve, uind - 1, vind, usteps[uind], vSections[uind, vind].Value)) return;
            if (CheckVSection(curve, uind + 1, vind, usteps[uind], vSections[uind, vind].Value)) return;
            if (CheckUSection(curve, uind, vind, usteps[uind], vSections[uind, vind].Value)) return;
            if (CheckUSection(curve, uind, vind + 1, usteps[uind], vSections[uind, vind].Value)) return;
            if (CheckUSection(curve, uind - 1, vind, usteps[uind], vSections[uind, vind].Value)) return;
            if (CheckUSection(curve, uind - 1, vind + 1, usteps[uind], vSections[uind, vind].Value)) return;
        }
        private bool CheckUSection(List<GeoPoint2D> curve, int uind, int vind, double u, double v)
        {
            if (uind >= 0 && uind < usteps.Length - 1 && vind >= 0 && vind < vsteps.Length)
            {
                if (uSections[vind, uind].HasValue && !uSectionsUsed[vind, uind])
                {
                    {
                        AddIntermediatePoints(curve, u, v, uSections[vind, uind].Value, vsteps[vind]);
                        curve.Add(new GeoPoint2D(uSections[vind, uind].Value, vsteps[vind]));
                        uSectionsUsed[vind, uind] = true;
                        FindCurveFromUSection(curve, uind, vind);
                        return true;
                    }
                }
            }
            return false;
        }
        private bool CheckVSection(List<GeoPoint2D> curve, int uind, int vind, double u, double v)
        {
            if (uind >= 0 && uind < usteps.Length && vind >= 0 && vind < vsteps.Length - 1)
            {
                if (vSections[uind, vind].HasValue && !vSectionsUsed[uind, vind])
                {
                    {
                        AddIntermediatePoints(curve, u, v, usteps[uind], vSections[uind, vind].Value);
                        curve.Add(new GeoPoint2D(usteps[uind], vSections[uind, vind].Value));
                        vSectionsUsed[uind, vind] = true;
                        FindCurveFromVSection(curve, uind, vind);
                        return true;
                    }
                }
            }
            return false;
        }
        private void AddIntermediatePoints(List<GeoPoint2D> curve, double u1, double v1, double u2, double v2)
        {
            double u = (u1 + u2) / 2.0;
            double v = (v1 + v2) / 2.0;

            double tanuv1 = GetTangentValue(u1, v1);
            double tanuv2 = GetTangentValue(u2, v2);

            double umin = 0.0, umax = 0.0, vmin = 0.0, vmax = 0.0;
            int uind = 0, vind = 0;
            for (int i = 0; i < usteps.Length; ++i)
            {
                if (usteps[i] < u)
                {
                    umin = umax = usteps[i];
                    uind = i;
                }
                else
                {
                    umax = usteps[i];
                    break;
                }
            }
            for (int i = 0; i < vsteps.Length; ++i)
            {
                if (vsteps[i] < v)
                {
                    vmin = vmax = vsteps[i];
                    vind = i;
                }
                else
                {
                    vmax = vsteps[i];
                    break;
                }
            }
            double fneg = double.MaxValue;
            double fpos = double.MaxValue;
            double du = v2 - v1;    // senkrecht dazu
            double dv = u1 - u2;
            if (du > 0.0)
            {
                fpos = (umax - u) / du;
                fneg = (u - umin) / du;
            }
            else if (du < 0.0)
            {
                fpos = (umin - u) / du;
                fneg = (u - umax) / du;
            }
            if (dv > 0.0)
            {
                fpos = Math.Min(fpos, (vmax - v) / dv);
                fneg = Math.Min(fneg, (v - vmin) / dv);
            }
            else if (dv < 0.0)
            {
                fpos = Math.Min(fpos, (vmin - v) / dv);
                fneg = Math.Min(fneg, (v - vmax) / dv);
            }
            // wir gehen in der Mitte quer zur Verbindungsstrecke, dabei ist das Problem, wie weit darf oder muss man ausladen
            // um einen Vorzeichenwechsel zu erreichen und ohne eine weiter Kurve einzugreifen.
            // ob es dafür einen vernünftige Lösung gibt???
            // fpos bzw. fneg sind so gewählt, dass das Kästchen nicht verlassen wird.
            // Wenn es dabei keine Nullstelle gibt, muss feiner aufgetilt werden. Das hat zur Folge,
            // dass die Kurve ein Kästchen nicht auf einer Seite verlassen kann und wieder in die selbe Seite
            // eintreten kann.
            double tan1 = GetTangentValue(u + fpos * du, v + fpos * dv);
            double tan2 = GetTangentValue(u - fneg * du, v - fneg * dv);
            if (DifferentSign(tan1, tan2))
            {
                double u0, v0;
                FindZero(u + fpos * du, v + fpos * dv, tan1, u - fneg * du, v - fneg * dv, tan2, out u0, out v0);
                // der neue Punkt muss in der Nähe sein, sonst kann es sein, dass man immer hin und her springt zwischen 2 verschiedenen Kurven
                if ((new GeoPoint2D(u0, v0) | new GeoPoint2D(u, v)) > (new GeoPoint2D(u1, v1) | new GeoPoint2D(u2, v2)) / 2.0)
                {
                    // der neue Punkt liegt außerhalb des Kreises um die Strecke (u1,v1) -> (u2,v2)
                    // damit wären die neuen Abschnitte länger als der bestehende und es kann sein, dass wir nicht konvergieren
                    // also hier einfach den Zwischenpunkt unter den Tisch fallen lassen
                }
                else if ((Math.Abs(u - u0) < precisionU && Math.Abs(v - v0) < precisionV) || curve.Count > 1000)
                {
                    curve.Add(new GeoPoint2D(u0, v0));
                }
                else
                {
                    AddIntermediatePoints(curve, u1, v1, u0, v0);
                    curve.Add(new GeoPoint2D(u0, v0));
                    AddIntermediatePoints(curve, u0, v0, u2, v2);
                }
            }
            else
            {
                // es kann keine Nullstelle gefunden werden, wir müssen halt feiner Aufteilen
                throw new UnableToFindZero(uind, vind);
            }
        }
        private void FindZero(double u1, double v1, double tan1, double u2, double v2, double tan2, out double u, out double v)
        {   // erstmal als primitive bisection imeplementieren
            u = (u1 + u2) / 2.0;
            v = (v1 + v2) / 2.0;
            for (int i = 0; i < 48; i++)
            {
                double tan = GetTangentValue(u, v);
                if (DifferentSign(tan, tan1))
                {
                    tan2 = tan;
                    u2 = u;
                    v2 = v;
                }
                else
                {
                    tan1 = tan;
                    u1 = u;
                    v1 = v;
                }
                u = (u1 + u2) / 2.0;
                v = (v1 + v2) / 2.0;
            }
        }
        private static bool DifferentSign(double d1, double d2)
        {   // am linken rand gehört die null dazu, am rechten nicht
            return (d1 > 0.0 && d2 <= 0.0) || (d1 <= 0.0 && d2 > 0.0);
        }
        private double GetTangentValue(double u, double v)
        {   // das Skalarprodukt von Blickrichtung und Normalenvektor. Das wird null bei tangentialer Blickrichtung.
            return direction * surface.GetNormal(new GeoPoint2D(u, v));
        }
        public virtual ICurve2D[] GetTangentCurves(GeoVector direction, double umin, double umax, double vmin, double vmax)
        {
            this.direction = direction;
            precisionU = Math.Abs(umax - umin) * 1e-3;
            precisionV = Math.Abs(vmax - vmin) * 1e-3;
            /*
             * Erzeuge ein Schachbrett aus den GetSafeParameterSteps. Finde alle unterteilungspunkte auf den Kanten.
             * Erzeuge offene oder geschlossene 2d Punktfolgen aus diesen unterbrechungen. Ein Schachfeld sollte
             * keine oder zwei Kanten unterbrochen haben. Ungerade ist nicht möglich. Sind 4 Kanten unterbrochen,
             * so müsste man nochmals unterteilen. Schneiden können sich die Konturlinien nicht, dass sie sich
             * berühren, kann ich mir nicht vorstellen, ist aber theoretisch denkbar. (Konturlinien teilen die 
             * Fläche in einen positiven und einen negativen Bereich, deshalb können sie sich nicht schneiden)
             * Dann erzeuge Zwischenpunkte in der Punktfolge. Wenn der Mittelpunkt und der Zwischenpunkt im 3D
             * (oder besser in der projektionsfläche von direction) ein gewisses Maß unterschreiten, dann ist man fertig.
             * Mache aus den punktfolgen NURBS Kurven.
             */
            surface.GetSafeParameterSteps(umin, umax, vmin, vmax, out usteps, out vsteps);
            // das einzige Problem sind hier Singularitäten. Hier ein Versuch, damit umzugehen:
            // Singularitäten kommen nur am Anfang oder Ende vor, also diese bereiche überprüfen
            GeoVector ntest = surface.GetNormal(new GeoPoint2D(umin, (vmin + vmax) / 2.0));
            if (Precision.IsNullVector(ntest))
            {
                usteps[0] += (usteps[1] - usteps[0]) * 1e-3;
            }
            ntest = surface.GetNormal(new GeoPoint2D(umax, (vmin + vmax) / 2.0));
            if (Precision.IsNullVector(ntest))
            {
                usteps[usteps.Length - 1] -= (usteps[usteps.Length - 1] - usteps[usteps.Length - 2]) * 1e-3;
            }
            ntest = surface.GetNormal(new GeoPoint2D((umin + umax) / 2.0, vmin));
            if (Precision.IsNullVector(ntest))
            {
                vsteps[0] += (vsteps[1] - vsteps[0]) * 1e-3;
            }
            ntest = surface.GetNormal(new GeoPoint2D((umin + umax) / 2.0, vmax));
            if (Precision.IsNullVector(ntest))
            {
                vsteps[vsteps.Length - 1] -= (vsteps[vsteps.Length - 1] - vsteps[vsteps.Length - 2]) * 1e-3;
            }
            bool success = false;
            while (!success)
            {
                success = true; // wird nur im catch wieder auf false gesetzt
                double[,] vertex = new double[usteps.Length, vsteps.Length];
                // das sind die Ecken im Schachbrett
                for (int i = 0; i < usteps.Length; i++)
                {
                    for (int j = 0; j < vsteps.Length; j++)
                    {
                        vertex[i, j] = GetTangentValue(usteps[i], vsteps[j]);
                    }
                }
                // Jetzt haben alle Knoten einen Wert. Wenn zwei benachbarte verschiedenes Vorzeichen haben, dann
                // interpolieren und den Punkt bestimmen. Mit diesem Punkt eine neue Kette beginnen und die Kante als
                // benutzt markieren. Jetzt gucken, wos möglicherweise weitergeht, wenns keine Randkante ist.
                // Damit man nicht vorwärts und rückwärts suchen muss, am besten mit den Randkanten anfangen.
                // Wenn die alle aufgebraucht sind, gibt es nur noch geschlossene innere kanten.
                uSections = new double?[vsteps.Length, usteps.Length - 1];
                vSections = new double?[usteps.Length, vsteps.Length - 1];
                uSectionsUsed = new bool[vsteps.Length, usteps.Length - 1];
                vSectionsUsed = new bool[usteps.Length, vsteps.Length - 1];
                // uSections sind die Nullpunkte bei festem v in u-Richtung
                // uSectionsUsed besagt, ob sie schon benutzt wurden

                try
                {
                    for (int j = 0; j < vsteps.Length; j++)
                    {
                        for (int i = 0; i < usteps.Length - 1; i++)
                        {
                            if (DifferentSign(vertex[i, j], vertex[i + 1, j]))
                            {
                                double u, v;
                                FindZero(usteps[i], vsteps[j], vertex[i, j], usteps[i + 1], vsteps[j], vertex[i + 1, j], out u, out v);
                                uSections[j, i] = u;
                            }
                        }
                    }
                    for (int i = 0; i < usteps.Length; i++)
                    {
                        for (int j = 0; j < vsteps.Length - 1; j++)
                        {
                            if (DifferentSign(vertex[i, j], vertex[i, j + 1]))
                            {
                                double u, v;
                                FindZero(usteps[i], vsteps[j], vertex[i, j], usteps[i], vsteps[j + 1], vertex[i, j + 1], out u, out v);
                                vSections[i, j] = v;
                            }
                        }
                    }

                    // Achtung: wenn Maschen entstehen, bei denen 2 Kurven durchgehen, dann  muss man feiner aufteilen
                    List<List<GeoPoint2D>> allCurves = new List<List<GeoPoint2D>>();
                    for (int i = 0; i < usteps.Length - 1; ++i)
                    {
                        // unterer Rand in U-Richtung
                        if (uSections[0, i].HasValue && !uSectionsUsed[0, i])
                        {
                            List<GeoPoint2D> curve = new List<GeoPoint2D>();
                            allCurves.Add(curve);
                            curve.Add(new GeoPoint2D(uSections[0, i].Value, vsteps[0]));
                            uSectionsUsed[0, i] = true;
                            FindCurveFromUSection(curve, i, 0);
                        }
                        // oberer Rand in U-Richtung
                        if (uSections[vsteps.Length - 1, i].HasValue && !uSectionsUsed[vsteps.Length - 1, i])
                        {
                            List<GeoPoint2D> curve = new List<GeoPoint2D>();
                            allCurves.Add(curve);
                            curve.Add(new GeoPoint2D(uSections[vsteps.Length - 1, i].Value, vsteps[vsteps.Length - 1]));
                            uSectionsUsed[vsteps.Length - 1, i] = true;
                            FindCurveFromUSection(curve, i, vsteps.Length - 1);
                        }
                    }
                    for (int i = 0; i < vsteps.Length - 1; ++i)
                    {
                        // linker Rand
                        if (vSections[0, i].HasValue && !vSectionsUsed[0, i])
                        {
                            List<GeoPoint2D> curve = new List<GeoPoint2D>();
                            allCurves.Add(curve);
                            curve.Add(new GeoPoint2D(usteps[0], vSections[0, i].Value));
                            vSectionsUsed[0, i] = true;
                            FindCurveFromVSection(curve, 0, i);
                        }
                        // rechter Rand
                        if (vSections[usteps.Length - 1, i].HasValue && !vSectionsUsed[usteps.Length - 1, i])
                        {
                            List<GeoPoint2D> curve = new List<GeoPoint2D>();
                            allCurves.Add(curve);
                            curve.Add(new GeoPoint2D(usteps[usteps.Length - 1], vSections[usteps.Length - 1, i].Value));
                            vSectionsUsed[usteps.Length - 1, i] = true;
                            FindCurveFromVSection(curve, usteps.Length - 1, i);
                        }
                    }
                    // und jetzt noch die inneren Kurven, die müssen immer geschlossen sein...
                    for (int i = 0; i < usteps.Length - 1; ++i)
                    {
                        for (int j = 1; j < vsteps.Length - 2; ++j)
                        {
                            // in U-Richtung
                            if (uSections[j, i].HasValue && !uSectionsUsed[j, i])
                            {
                                List<GeoPoint2D> curve = new List<GeoPoint2D>();
                                allCurves.Add(curve);
                                curve.Add(new GeoPoint2D(uSections[j, i].Value, vsteps[j]));
                                uSectionsUsed[j, i] = true;
                                FindCurveFromUSection(curve, i, j);
                                // throw new ApplicationException("hier noch das schließen implementieren");
                                // hier noch schließen
                            }
                        }
                    }
                    for (int i = 0; i < vsteps.Length - 1; ++i)
                    {
                        for (int j = 1; j < usteps.Length - 2; ++j)
                        {
                            if (vSections[j, i].HasValue && !vSectionsUsed[j, i])
                            {
                                List<GeoPoint2D> curve = new List<GeoPoint2D>();
                                allCurves.Add(curve);
                                curve.Add(new GeoPoint2D(usteps[j], vSections[j, i].Value));
                                vSectionsUsed[j, i] = true;
                                FindCurveFromVSection(curve, j, i);
                                // throw new ApplicationException("hier noch das schließen implementieren");
                                // hier noch schließen
                            }
                        }
                    }
                    // jetzt liegen alle Konturlinien als offene oder geschlossene Polylinien vor
                    // man könnte natürlich NURBS draus machen, das muss man noch sehen
                    List<ICurve2D> res = new List<ICurve2D>();
                    for (int i = 0; i < allCurves.Count; i++)
                    {
                        if (allCurves[i].Count > 1)
                        {
                            try
                            {
                                Polyline2D p2d = new Polyline2D(allCurves[i].ToArray());
                                res.Add(p2d);
                            }
                            catch (Polyline2DException)
                            {   // alle Punkte einer Kurve identisch
                            }
                        }
                    }
                    // Es muss noch implementiert werden: Singularitäten finden, denn dort müssen die
                    // Flächen aufgeteilt werden. Singularitäten sind die Punkte, an denen die 3D Konturkurve in Richtung
                    // der Blickrichtung geht. Dazu muss man das Minimum der Länge des Kreuzproduktes der direction mit zwei
                    // aufeinanderfolgenden Punkten (beide normiert auf die Länge 1) bestimmen. Wo das Minimum auftritt
                    // könnte eine Singularität liegen, ggf. dort noch Zwischenpunkte suchen.
                    // Das Thema Singularitäten ist doch etwas umfangreicher, denn es müssten nicht nur die hier gefundenen
                    // Konturlinien überprüft werden, sondern auch allgemein alle Randlinien der Faces. 
                    for (int i = 0; i < allCurves.Count; i++)
                    {
                    }
                    return res.ToArray();
                }
                catch (UnableToFindZero)
                {
                    // hier kommen wir hin, wenn in einem Kästchen die Kurve nicht bestimmt werden konnte.
                    // es gibt zwei Gründe: die Kurve verlässt an einer Kante das Kästchen und tritt an der selben Kante
                    // wieder ein, ODER es sind zwei Kurven, die an verschiedenen Kanten eintreten und an einer Kante
                    // wieder austreten. In beiden Fällen muss man genauer aufteilen. Diese Fälle sind aber sehr selten
                    // und tief in der Rekursion, so dass ExceptionHandling in diesem Fall OK ist.
                    // Notbremse bei patologischen Fällen, damit keine Endlosschleife.
                    if (usteps.Length + vsteps.Length < 1000)
                    {
                        success = false;
                        // usteps und vsteps mit Zwischenpunkten versehen und das ganze Spielchen von neuem machen
                        double[] dusteps = new double[usteps.Length * 2 - 1];
                        double[] dvsteps = new double[vsteps.Length * 2 - 1];
                        for (int i = 0; i < usteps.Length - 1; i++)
                        {
                            dusteps[2 * i] = usteps[i];
                            dusteps[2 * i + 1] = (usteps[i] + usteps[i + 1]) / 2.0;
                        }
                        for (int i = 0; i < vsteps.Length - 1; i++)
                        {
                            dvsteps[2 * i] = vsteps[i];
                            dvsteps[2 * i + 1] = (vsteps[i] + vsteps[i + 1]) / 2.0;
                        }
                        dusteps[usteps.Length * 2 - 2] = usteps[usteps.Length - 1];
                        dvsteps[vsteps.Length * 2 - 2] = vsteps[vsteps.Length - 1];
                        usteps = dusteps;
                        vsteps = dvsteps;
                    }
                    // ansonsten die Hoffnung aufgeben und kein Ergebnis liefern...
                }
            }
            return new ICurve2D[0];
        }
    }


    public class SurfaceHelper
    {
        public IEnumerator<ICurve> BoundingCurves(ISurface srf, BoundingRect ext)
        {
            yield return srf.FixedU(ext.Left, ext.Bottom, ext.Top);
            yield return srf.FixedU(ext.Right, ext.Bottom, ext.Top);
            yield return srf.FixedV(ext.Bottom, ext.Left, ext.Right);
            yield return srf.FixedV(ext.Top, ext.Left, ext.Right);

        }
        public static void AdjustPeriodic(double uperiod, double vperiod, ref GeoPoint2D p)
        {
            if (uperiod > 0.0)
            {
                while (p.x > uperiod) p.x -= uperiod;
                while (p.x < 0.0) p.x += uperiod;
            }
            if (vperiod > 0.0)
            {
                while (p.y > vperiod) p.y -= vperiod;
                while (p.y < 0.0) p.y += vperiod;
            }
        }
        internal static void AdjustPeriodic(ISurface surface, BoundingRect bounds, SimpleShape ss)
        {
            if (surface.IsUPeriodic || surface.IsVPeriodic)
            {
                GeoPoint2D mp = ss.GetExtent().GetCenter();
                double du = 0.0, dv = 0.0;
                if (surface.IsUPeriodic)
                {
                    double um = (bounds.Left + bounds.Right) / 2;
                    while (Math.Abs(mp.x + du - um) > Math.Abs(mp.x + du - surface.UPeriod - um)) du -= surface.UPeriod;
                    while (Math.Abs(mp.x + du - um) > Math.Abs(mp.x + du + surface.UPeriod - um)) du += surface.UPeriod;
                }
                if (surface.IsVPeriodic)
                {
                    double vm = (bounds.Bottom + bounds.Top) / 2;
                    while (Math.Abs(mp.y + dv - vm) > Math.Abs(mp.y + dv - surface.VPeriod - vm)) dv -= surface.VPeriod;
                    while (Math.Abs(mp.y + dv - vm) > Math.Abs(mp.y + dv + surface.VPeriod - vm)) dv += surface.VPeriod;
                }
                if (du != 0.0 || dv != 0.0)
                {
                    ss.Move(du, dv);
                }
            }

        }
        internal static (double du, double dv) AdjustPeriodic(ISurface surface, BoundingRect bounds, Border bdr)
        {
            if (surface.IsUPeriodic || surface.IsVPeriodic)
            {
                GeoPoint2D mp = bdr.Extent.GetCenter();
                double du = 0.0, dv = 0.0;
                if (surface.IsUPeriodic)
                {
                    double um = (bounds.Left + bounds.Right) / 2;
                    while (Math.Abs(mp.x + du - um) > Math.Abs(mp.x + du - surface.UPeriod - um)) du -= surface.UPeriod;
                    while (Math.Abs(mp.x + du - um) > Math.Abs(mp.x + du + surface.UPeriod - um)) du += surface.UPeriod;
                }
                if (surface.IsVPeriodic)
                {
                    double vm = (bounds.Bottom + bounds.Top) / 2;
                    while (Math.Abs(mp.y + dv - vm) > Math.Abs(mp.y + dv - surface.VPeriod - vm)) dv -= surface.VPeriod;
                    while (Math.Abs(mp.y + dv - vm) > Math.Abs(mp.y + dv + surface.VPeriod - vm)) dv += surface.VPeriod;
                }
                if (du != 0.0 || dv != 0.0)
                {
                    bdr.Move(du, dv);
                }
                return (du, dv);
            }
            return (0, 0);
        }
        public static void AdjustPeriodic(ISurface surface, BoundingRect bounds, ICurve2D cv2d)
        {
            if (surface.IsUPeriodic || surface.IsVPeriodic)
            {
                GeoPoint2D mp = cv2d.PointAt(0.5);
                double du = 0.0, dv = 0.0;
                if (surface.IsUPeriodic)
                {
                    double um = (bounds.Left + bounds.Right) / 2;
                    while (Math.Abs(mp.x + du - um) > Math.Abs(mp.x + du - surface.UPeriod - um)) du -= surface.UPeriod;
                    while (Math.Abs(mp.x + du - um) > Math.Abs(mp.x + du + surface.UPeriod - um)) du += surface.UPeriod;
                }
                if (surface.IsVPeriodic)
                {
                    double vm = (bounds.Bottom + bounds.Top) / 2;
                    while (Math.Abs(mp.y + dv - vm) > Math.Abs(mp.y + dv - surface.VPeriod - vm)) dv -= surface.VPeriod;
                    while (Math.Abs(mp.y + dv - vm) > Math.Abs(mp.y + dv + surface.VPeriod - vm)) dv += surface.VPeriod;
                }
                if (du != 0.0 || dv != 0.0)
                {
                    cv2d.Move(du, dv);
                }
            }
        }
        internal static void AdjustPeriodicStartPoint(ISurface surface, GeoPoint2D startPoint, ICurve2D cv2d)
        {
            if (surface.IsUPeriodic || surface.IsVPeriodic)
            {
                GeoPoint2D sp = cv2d.StartPoint;
                double du = 0.0, dv = 0.0;
                if (surface.IsUPeriodic)
                {
                    double spx = startPoint.x;
                    while (Math.Abs(sp.x + du - spx) > Math.Abs(sp.x + du - surface.UPeriod - spx)) du -= surface.UPeriod;
                    while (Math.Abs(sp.x + du - spx) > Math.Abs(sp.x + du + surface.UPeriod - spx)) du += surface.UPeriod;
                }
                if (surface.IsVPeriodic)
                {
                    double spy = startPoint.y;
                    while (Math.Abs(sp.y + dv - spy) > Math.Abs(sp.y + dv - surface.VPeriod - spy)) dv -= surface.VPeriod;
                    while (Math.Abs(sp.y + dv - spy) > Math.Abs(sp.y + dv + surface.VPeriod - spy)) dv += surface.VPeriod;
                }
                if (du != 0.0 || dv != 0.0)
                {
                    cv2d.Move(du, dv);
                }
            }
        }
        internal static void AdjustPeriodicStartPoint(ISurface surface, GeoPoint2D startPoint, ref GeoPoint2D toAdjust)
        {
            if (surface.IsUPeriodic || surface.IsVPeriodic)
            {
                double du = 0.0, dv = 0.0;
                if (surface.IsUPeriodic)
                {
                    double spx = startPoint.x;
                    while (Math.Abs(toAdjust.x + du - spx) > Math.Abs(toAdjust.x + du - surface.UPeriod - spx)) du -= surface.UPeriod;
                    while (Math.Abs(toAdjust.x + du - spx) > Math.Abs(toAdjust.x + du + surface.UPeriod - spx)) du += surface.UPeriod;
                }
                if (surface.IsVPeriodic)
                {
                    double spy = startPoint.y;
                    while (Math.Abs(toAdjust.y + dv - spy) > Math.Abs(toAdjust.y + dv - surface.VPeriod - spy)) dv -= surface.VPeriod;
                    while (Math.Abs(toAdjust.y + dv - spy) > Math.Abs(toAdjust.y + dv + surface.VPeriod - spy)) dv += surface.VPeriod;
                }
                if (du != 0.0 || dv != 0.0)
                {
                    toAdjust.x += du;
                    toAdjust.y += dv;
                }
            }
        }

        /// <summary>
        /// Modifies the provided points by adding multiples of the period (in u and/or v) so that the resulting
        /// sequence is "compact": there are no jumps between consecutive points which are bigger than half of the
        /// period. The order of the points is not changed, only periodic offsets are applied. Finally the whole
        /// sequence is moved by a common multiple of the period to be as close as possible to the provided
        /// <paramref name="bounds"/> (which may be empty, then no such move is performed).
        /// </summary>
        /// <param name="surface">the surface which defines the periodicity</param>
        /// <param name="bounds">the domain the points should be close to, may be empty</param>
        /// <param name="points">the points to modify (in place)</param>
        internal static void UnwrapPeriodic(ISurface surface, BoundingRect bounds, GeoPoint2D[] points)
        {
            if (points == null || points.Length == 0) return;
            if (!surface.IsUPeriodic && !surface.IsVPeriodic) return;
            bool useBounds = !bounds.IsEmpty();
            if (surface.IsUPeriodic && surface.UPeriod > 0.0)
            {
                double period = surface.UPeriod;
                double min = points[0].x, max = points[0].x;
                for (int i = 1; i < points.Length; i++)
                {   // remove the jump to the previous point, which is always possible except for a jump of exactly half a period
                    points[i].x -= Math.Round((points[i].x - points[i - 1].x) / period) * period;
                    if (points[i].x < min) min = points[i].x;
                    if (points[i].x > max) max = points[i].x;
                }
                if (useBounds)
                {   // move the whole (now connected) sequence as close as possible to the bounds
                    double um = (bounds.Left + bounds.Right) / 2;
                    double d = -Math.Round(((min + max) / 2 - um) / period) * period;
                    if (d != 0.0)
                    {
                        for (int i = 0; i < points.Length; i++) points[i].x += d;
                    }
                }
            }
            if (surface.IsVPeriodic && surface.VPeriod > 0.0)
            {
                double period = surface.VPeriod;
                double min = points[0].y, max = points[0].y;
                for (int i = 1; i < points.Length; i++)
                {
                    points[i].y -= Math.Round((points[i].y - points[i - 1].y) / period) * period;
                    if (points[i].y < min) min = points[i].y;
                    if (points[i].y > max) max = points[i].y;
                }
                if (useBounds)
                {
                    double vm = (bounds.Bottom + bounds.Top) / 2;
                    double d = -Math.Round(((min + max) / 2 - vm) / period) * period;
                    if (d != 0.0)
                    {
                        for (int i = 0; i < points.Length; i++) points[i].y += d;
                    }
                }
            }
        }

        internal static void AdjustPeriodic(ISurface surface, BoundingRect bounds, GeoPoint2D[] points)
        {
            if (surface.IsUPeriodic || surface.IsVPeriodic)
            {
                GeoPoint2D mp = points[points.Length / 2]; // der mittlere Punkt
                double du = 0.0, dv = 0.0;
                if (surface.IsUPeriodic)
                {
                    double um = (bounds.Left + bounds.Right) / 2;
                    while (Math.Abs(mp.x + du - um) > Math.Abs(mp.x + du - surface.UPeriod - um)) du -= surface.UPeriod;
                    while (Math.Abs(mp.x + du - um) > Math.Abs(mp.x + du + surface.UPeriod - um)) du += surface.UPeriod;
                }
                if (surface.IsVPeriodic)
                {
                    double vm = (bounds.Bottom + bounds.Top) / 2;
                    while (Math.Abs(mp.y + dv - vm) > Math.Abs(mp.y + dv - surface.VPeriod - vm)) dv -= surface.VPeriod;
                    while (Math.Abs(mp.y + dv - vm) > Math.Abs(mp.y + dv + surface.VPeriod - vm)) dv += surface.VPeriod;
                }
                if (du != 0.0 || dv != 0.0)
                {
                    GeoVector2D d = new GeoVector2D(du, dv);
                    for (int i = 0; i < points.Length; i++)
                    {
                        points[i] += d;
                    }
                }
            }
        }
        public static void AdjustPeriodic(ISurface surface, BoundingRect bounds, ref GeoPoint2D p2d)
        {
            if (surface.IsUPeriodic || surface.IsVPeriodic)
            {
                if (surface.IsUPeriodic)
                {
                    double um = (bounds.Left + bounds.Right) / 2;
                    while (Math.Abs(p2d.x - um) > Math.Abs(p2d.x - surface.UPeriod - um)) p2d.x -= surface.UPeriod;
                    while (Math.Abs(p2d.x - um) > Math.Abs(p2d.x + surface.UPeriod - um)) p2d.x += surface.UPeriod;
                }
                if (surface.IsVPeriodic)
                {
                    double vm = (bounds.Bottom + bounds.Top) / 2;
                    while (Math.Abs(p2d.y - vm) > Math.Abs(p2d.y - surface.VPeriod - vm)) p2d.y -= surface.VPeriod;
                    while (Math.Abs(p2d.y - vm) > Math.Abs(p2d.y + surface.VPeriod - vm)) p2d.y += surface.VPeriod;
                }
            }
        }
        public static void AdjustPeriodic(ISurface surface, ref GeoPoint2D p2d)
        {
            if (surface is ISurfaceImpl si) AdjustPeriodic(surface, si.usedArea, ref p2d);
        }
        internal static void AdjustUPeriodic(ISurface surface, BoundingRect bounds, ref double u)
        {
            if (surface.IsUPeriodic)
            {
                double um = (bounds.Left + bounds.Right) / 2;
                while (Math.Abs(u - um) > Math.Abs(u - surface.UPeriod - um)) u -= surface.UPeriod;
                while (Math.Abs(u - um) > Math.Abs(u + surface.UPeriod - um)) u += surface.UPeriod;
            }
        }
        internal static void AdjustUPeriodic(ISurface surface, double umin, double umax, ref double u)
        {
            if (surface.IsUPeriodic)
            {
                double um = (umin + umax) / 2;
                while (Math.Abs(u - um) > Math.Abs(u - surface.UPeriod - um)) u -= surface.UPeriod;
                while (Math.Abs(u - um) > Math.Abs(u + surface.UPeriod - um)) u += surface.UPeriod;
            }
        }
        internal static void AdjustVPeriodic(ISurface surface, double vmin, double vmax, ref double v)
        {
            if (surface.IsVPeriodic)
            {
                double k = Math.Round(((vmin + vmax) / 2 - v) / surface.VPeriod);
                v += k * surface.VPeriod;
            }
        }
        public static void MinMaxCurvature(ISurface surface, GeoPoint2D uv, out ICurve minCurvature, out ICurve maxCurvature)
        {   // to be tested
            surface.Derivative2At(uv, out GeoPoint location, out GeoVector du, out GeoVector dv, out GeoVector duu, out GeoVector dvv, out GeoVector duv);
            Matrix I = DenseMatrix.Create(2, 2, 0);
            I[0, 0] = du * du;
            I[0, 1] = I[1, 0] = du * dv;
            I[1, 1] = dv * dv;
            GeoVector n = (du ^ dv).Normalized;
            Matrix II = DenseMatrix.Create(2, 2, 0);
            II[0, 0] = duu * n;
            II[0, 1] = I[1, 0] = duv * n;
            II[1, 1] = dvv * n;
            Matrix S = (Matrix)I.Inverse().Multiply(II); // Weingarten
            Evd<double> evd = S.Evd();
            Vector<System.Numerics.Complex> eigenValues = evd.EigenValues;
            Matrix eigenVectors = (Matrix)evd.EigenVectors;
            minCurvature = maxCurvature = null;
            if (eigenValues.Count > 0)
            {
                if (eigenValues[0].Imaginary == 0.0)
                {
                    GeoVector dir = eigenVectors[0, 0] * du + eigenVectors[1, 0] * dv;
                    if (eigenValues[0].Real != 0.0)
                    {
                        double rad = 1.0 / eigenValues[0].Real;
                        GeoPoint cnt = location + rad * n;
                        Ellipse elli = Ellipse.Construct();
                        elli.SetCirclePlaneCenterRadius(new Plane(cnt, n, dir), cnt, Math.Abs(rad));
                        minCurvature = elli;
                    }
                    else
                    {
                        Line line = Line.Construct();
                        line.SetTwoPoints(location, location + dir);
                        minCurvature = line;
                    }
                }
            }
            if (eigenValues.Count > 1) // what about a double eigenvalue?
            {
                if (eigenValues[1].Imaginary == 0.0)
                {
                    GeoVector dir = eigenVectors[0, 1] * du + eigenVectors[1, 1] * dv;
                    if (eigenValues[1].Real != 0.0)
                    {
                        double rad = 1.0 / eigenValues[1].Real;
                        GeoPoint cnt = location + rad * n;
                        Ellipse elli = Ellipse.Construct();
                        elli.SetCirclePlaneCenterRadius(new Plane(cnt, n, dir), cnt, Math.Abs(rad));
                        minCurvature = elli;
                    }
                    else
                    {
                        Line line = Line.Construct();
                        line.SetTwoPoints(location, location + dir);
                        minCurvature = line;
                    }
                }
            }
        }

        public static GeoPoint2D[] GetExtrema(ISurface surface, BoundingRect domain, GeoVector dir)
        {
            ISurface srf = surface.Clone();
            if (!Precision.SameDirection(dir, GeoVector.ZAxis, false))
            {   // modify the surface, so that dir is the z-axis
                srf.Modify(ModOp.Rotate(GeoPoint.Origin, dir, GeoVector.ZAxis));
            }
            GeoPoint2D[] uv = srf.GetExtrema();
            List<GeoPoint2D> res = new List<GeoPoint2D>();
            for (int i = 0; i < uv.Length; i++)
            {
                if (Precision.SameDirection(srf.GetNormal(uv[i]), GeoVector.ZAxis, false)) res.Add(uv[i]);
            }
            return res.ToArray();
        }
    }
}
