using CADability.Curve2D;
using CADability;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShapeIt
{
    /// <summary>
    /// Methods, which belong to Face. Maybe we should include these methods to the Face class in CADability later, when everything is tested
    /// </summary>
    internal static class FaceExtensions
    {
        public static Face GetRawOffset(this Face face, double dist)
        {
            // das kopieren der 2d Kurven in diese doppelte Liste muss gemacht werden, um die Reihenfolge beizubehalten
            List<List<ICurve2D>> outlineAndHoles = new List<List<ICurve2D>>();
            List<ICurve2D> crvo = new List<ICurve2D>();
            for (int i = 0; i < face.OutlineEdges.Length; i++)
            {
                crvo.Add(face.OutlineEdges[i].Curve2D(face));
            }
            outlineAndHoles.Add(crvo);
            for (int i = 0; i < face.HoleCount; i++)
            {
                List<ICurve2D> crvh = new List<ICurve2D>();
                for (int j = 0; j < face.HoleEdges(i).Length; j++)
                {
                    crvh.Add(face.HoleEdges(i)[j].Curve2D(face));
                }
                outlineAndHoles.Add(crvh);
            }
            ISurface offsetSurface;
            ModOp2D mod = ModOp2D.Null;
            if (face.Surface is ConicalSurface) // da gibts vielleicht noch andere Fälle?
            {
                offsetSurface = (face.Surface as ConicalSurface).GetOffsetSurface(dist, out mod);
            }
            else
            {
                offsetSurface = face.Surface.GetOffsetSurface(dist);
            }
            if (offsetSurface == null) return null;
            if (!mod.IsNull)
            {
                for (int i = 0; i < outlineAndHoles.Count; i++)
                {
                    for (int j = 0; j < outlineAndHoles[i].Count; j++)
                    {
                        outlineAndHoles[i][j] = outlineAndHoles[i][j].GetModified(mod);
                    }
                }
            }
            return Face.MakeFace(offsetSurface, outlineAndHoles);
        }
    }
}
