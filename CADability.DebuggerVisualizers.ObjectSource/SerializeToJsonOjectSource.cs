using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using System.Runtime.Serialization;
using System.Runtime.CompilerServices;
using Microsoft.VisualStudio.DebuggerVisualizers;
using CADability.GeoObject;
using System.Collections.Generic;

namespace CADability.DebuggerVisualizers.ObjectSource
{
    public class SerializeToJsonOjectSource : VisualizerObjectSource
    {
        public override void GetData(object target, Stream outgoingData)
        {
            if (target is IGeoObject go)
            {
                target = go.Clone(); // clone the object, so that any references to other objects (e.g. Face->Edge->otherFace) are removed    
            }
            else if (target is DebuggerContainer dc)
            {
                target = dc.toShow; // the contained objects
            }
            else if (target is IEnumerable<Edge> goEnum)
            {
                GeoObjectList goList = new GeoObjectList();
                foreach (Edge edge2 in goEnum)
                {
                    goList.Add(edge2.Curve3D.Clone() as IGeoObjectImpl);
                }
                target = goList;
            }
            else if (target is GeoPoint gp)
            {
                Point pnt = Point.Construct();
                pnt.Location = gp;
                pnt.Symbol = PointSymbol.Cross;
                target = pnt;
            }
            string json = JsonSerialize.ToString(target);
            using (StreamWriter writer = new StreamWriter(outgoingData))
            {
                writer.Write(json);
            }
        }
    }
}
