using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using System.Runtime.Serialization;
using System.Runtime.CompilerServices;
using Microsoft.VisualStudio.DebuggerVisualizers;
using CADability.GeoObject;

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
            string json = JsonSerialize.ToString(target);
            using (StreamWriter writer = new StreamWriter(outgoingData))
            {
                writer.Write(json);
            }
        }
    }
}
