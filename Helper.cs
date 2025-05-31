using CADability.GeoObject;
using CADability;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CADability.Attribute;

namespace ShapeIt
{
    internal static class Helper
    {
        static public IGeoObject ToGeoObject(object o)
        {
            if (o is IGeoObject go) { return go; }
            if (o is Edge edg) { return edg.Curve3D as IGeoObject; }
            if (o is Vertex vtx)
            {
                CADability.GeoObject.Point pnt = CADability.GeoObject.Point.Construct();
                pnt.Location = vtx.Position;
                pnt.Symbol = PointSymbol.Cross;
                return pnt;
            }
            return null;
        }

        private static LineWidth edgeLineWidth = new LineWidth("selectedEdge", 0.7);
        /// <summary>
        /// In order to display feedback edges as thick curves we clone geoobjects with linewidth and set a thicker line witdh. Faces and other objects stay unchanged
        /// </summary>
        /// <param name="go"></param>
        /// <returns></returns>
        internal static IGeoObject ThickLines(IGeoObject go)
        {
            if (go is ILineWidth)
            {
                IGeoObject clone = go.Clone();
                (clone as ILineWidth).LineWidth = edgeLineWidth;
                return clone;
            }
            return go;
        }
        public static GeoObjectList ThickCurvesFromEdges(IEnumerable<Edge> edges)
        {
            GeoObjectList res = new GeoObjectList();
            foreach (Edge e in edges)
            {
                IGeoObject cloned = (e.Curve3D as IGeoObject).Clone();
                if (cloned is ILineWidth lw) lw.LineWidth = edgeLineWidth;
                res.Add(cloned);
            }
            return res;
        }
        public static GeoObjectList ThickCurvesFromCurves(IEnumerable<ICurve> curves)
        {
            GeoObjectList res = new GeoObjectList();
            foreach (ICurve crv in curves)
            {
                IGeoObject cloned = (crv as IGeoObject).Clone();
                if (cloned is ILineWidth lw) lw.LineWidth = edgeLineWidth;
                res.Add(cloned);
            }
            return res;
        }

    }
}
