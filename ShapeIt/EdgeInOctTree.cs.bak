using CADability;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeIt
{
    internal class EdgeInOctTree : IOctTreeInsertable
    {
        public Edge Edge { get; private set; }
        public EdgeInOctTree(Edge edge) { Edge = edge; }

        public BoundingCube GetExtent(double precision)
        {
            return Edge.Curve3D.GetExtent();
        }

        public bool HitTest(ref BoundingCube cube, double precision)
        {
            return Edge.Curve3D.HitTest(cube);
        }

        public bool HitTest(Projection projection, BoundingRect rect, bool onlyInside)
        {
            throw new NotImplementedException();
        }

        public bool HitTest(Projection.PickArea area, bool onlyInside)
        {
            throw new NotImplementedException();
        }

        public double Position(GeoPoint fromHere, GeoVector direction, double precision)
        {
            throw new NotImplementedException();
        }
    }

}
