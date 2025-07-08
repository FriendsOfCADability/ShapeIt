using CADability;
using CADability.GeoObject;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using System.Text;
using System.Threading.Tasks;

namespace ShapeIt
{
    internal class SweptCircle : ISurfaceImpl, ISerializable
    {
        // https://chatgpt.com/c/686bffb7-52b0-8013-955c-8331d5ce2ad2
        private ICurve spine;
        private double radius;

        public SweptCircle(ICurve spine, double radius)
        {
            this.spine = spine;
            this.radius = radius;
        }

        private (GeoVector T, GeoVector N) FrenetSystem(double u)
        {
            spine.TryPointDeriv2At(u, out GeoPoint point, out GeoVector deriv1, out GeoVector deriv2);
            GeoVector T = deriv1.Normalized;
            GeoVector N = (deriv2 - (deriv2 * T) * T).Normalized; 
            return (T, N);
        }
        public override ICurve FixedU(double u, double vmin, double vmax)
        {
            throw new NotImplementedException();
        }

        public override ICurve FixedV(double u, double umin, double umax)
        {
            throw new NotImplementedException();
        }

        public override ISurface GetModified(ModOp m)
        {
            throw new NotImplementedException();
        }

        public override IPropertyEntry GetPropertyEntry(IFrame frame)
        {
            throw new NotImplementedException();
        }

        public override GeoPoint PointAt(GeoPoint2D uv)
        {
            throw new NotImplementedException();
        }

        public override GeoVector UDirection(GeoPoint2D uv)
        {
            throw new NotImplementedException();
        }

        public override GeoVector VDirection(GeoPoint2D uv)
        {
            throw new NotImplementedException();
        }
    }
}
