/// -----------------------------------------------------------------------------------------------------------------------------------------
/// How to use Debugger Visualizers for Visual Studio 2022:
/// Copy all files from ...\CADability\CADability.DebuggerVisualizers\bin\Debug folder to
/// ...\My Documents\Visual Studio 2022\Visualizers
/// No need to create any subfolders!
/// See: https://learn.microsoft.com/en-us/visualstudio/debugger/how-to-install-a-visualizer?view=vs-2022
/// This applies to Visual Studio 2022 and 2019

using CADability.Attribute;
using CADability.Curve2D;
using CADability.DebuggerVisualizers;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.UserInterface;
using Microsoft.VisualStudio.DebuggerVisualizers;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Reflection;
using Point = CADability.GeoObject.Point;

#region "Types to be visualized"
//Border
[assembly: System.Diagnostics.DebuggerVisualizer(typeof(BorderVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.Shapes.Border), Description = "CADability Border Visualizer")]

//GeoObjectList
[assembly: System.Diagnostics.DebuggerVisualizer(typeof(GeoObjectListVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.GeoObject.GeoObjectList), Description = "CADability GeoObjectList Visualizer")]

//Simple Shape
[assembly: System.Diagnostics.DebuggerVisualizer(typeof(SimpleShapeVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.Shapes.SimpleShape), Description = "CADability Simple Shape Visualizer")]

//Compound Shape
[assembly: System.Diagnostics.DebuggerVisualizer(typeof(CompoundShapeVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.Shapes.CompoundShape), Description = "CADability Compound Shape Visualizer")]

//GeoPoint2D
[assembly: System.Diagnostics.DebuggerVisualizer(typeof(GeoPoint2DVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.GeoPoint2D), Description = "CADability GeoPoint2D Visualizer")]

//GeoPoint
[assembly: System.Diagnostics.DebuggerVisualizer(typeof(GeoPointVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.GeoPoint), Description = "CADability GeoPoint Visualizer")]

//ICurve2D Implementations
[assembly: System.Diagnostics.DebuggerVisualizer(typeof(Curve2DVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.Curve2D.GeneralCurve2D), Description = "CADability GeneralCurve2D Visualizer")]

[assembly: System.Diagnostics.DebuggerVisualizer(typeof(Curve2DVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.Curve2DAspect), Description = "CADability Curve2DAspect Visualizer")]

//IGeoObject Implementations
[assembly: System.Diagnostics.DebuggerVisualizer(typeof(GeoObjectVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.GeoObject.IGeoObjectImpl), Description = "CADability GeoObject Visualizer 3")]

//ICurve Implementations
[assembly: System.Diagnostics.DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.GeoObject.BSpline), Description = "CADability ICurve Visualizer")]

[assembly: System.Diagnostics.DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.GeoObject.Ellipse), Description = "CADability ICurve Visualizer")]

[assembly: System.Diagnostics.DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.GeoObject.GeneralCurve), Description = "CADability ICurve Visualizer")]

[assembly: System.Diagnostics.DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.GeoObject.Line), Description = "CADability ICurve Visualizer")]

[assembly: System.Diagnostics.DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.GeoObject.Path), Description = "CADability ICurve Visualizer")]

[assembly: System.Diagnostics.DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.GeoObject.Polyline), Description = "CADability ICurve Visualizer")]

//IDebuggerVisualizer Implementations
[assembly: System.Diagnostics.DebuggerVisualizer(typeof(GeneralDebuggerVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.BRepItem), Description = "CADability IDebuggerVisualizer Visualizer")]

[assembly: System.Diagnostics.DebuggerVisualizer(typeof(GeneralDebuggerVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.Curve2D.BSpline2D), Description = "CADability IDebuggerVisualizer Visualizer")]

[assembly: System.Diagnostics.DebuggerVisualizer(typeof(GeneralDebuggerVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.GeoObject.BoxedSurfaceEx.ParEpi), Description = "CADability IDebuggerVisualizer Visualizer")]

[assembly: System.Diagnostics.DebuggerVisualizer(typeof(GeneralDebuggerVisualizer), typeof(SerializeToJsonOjectSource),
Target = typeof(CADability.DebuggerContainer), Description = "CADability DebuggerContainer Visualizer")]
#endregion

namespace CADability.DebuggerVisualizers
{
    class SerializeToJsonOjectSource : VisualizerObjectSource
    {
        public override void GetData(object target, Stream outgoingData)
        {
            string json = JsonSerialize.ToString(target);
            using (StreamWriter writer = new StreamWriter(outgoingData))
            {
                writer.Write(json);
            }
        }
    }
    internal class Trace
    {
        static public void Clear()
        {
            if (File.Exists(@"C:\Temp\CADability.Trace.txt"))
                File.Delete(@"C:\Temp\CADability.Trace.txt");
        }
        static public void WriteLine(string text)
        {
            lock (typeof(Trace))
            {
                using (StreamWriter w = File.AppendText(@"C:\Temp\CADability.Trace.txt"))
                {
                    w.WriteLine(text);
                }
            }
        }
    }

    internal class CheckInstanceCounters
    {
        public static void Check()
        {
            Assembly ThisAssembly = Assembly.GetExecutingAssembly();
            Type[] types = ThisAssembly.GetTypes();
            System.Diagnostics.Trace.WriteLine("--- Instance Counters ---");
            System.GC.Collect();
            System.GC.WaitForPendingFinalizers();
            System.GC.Collect();
            System.GC.WaitForPendingFinalizers();
            long mem = System.GC.GetTotalMemory(true);
            System.Diagnostics.Trace.WriteLine("memory used: " + mem.ToString());
            for (int i = 0; i < types.Length; ++i)
            {
                FieldInfo fi = types[i].GetField("InstanceCounter", BindingFlags.Static | BindingFlags.NonPublic);
                if (fi != null)
                {
                    object val = fi.GetValue(null);
                    try
                    {
                        int n = (int)val;
                        System.Diagnostics.Trace.WriteLine(types[i].Name + ": " + n.ToString());
                    }
                    catch (InvalidCastException)
                    {
                    }
                }
            }
            System.Diagnostics.Trace.WriteLine("--- End End   End End ---");
        }
    }

    /// <summary>
    /// Creates a DebugForm as defined in CADability.Forms. 
    /// CADability.Forms.exe must be accessible at runtime to be able to debug
    /// </summary>
    static class CF
    {
        /// <summary>
        /// Load the assembly of CADability.Forms and instantiate the class
        /// </summary>
        public static IDebugForm DebugForm
        {
            get
            {
                IDebugForm res = new DebugForm();
                return res;
                // old way via reflection:
                Assembly cf = Assembly.Load("CADability.DebuggerVisualizers");
                Type tp = cf.GetType("CADability.DebuggerVisualizers.DebugForm");
                if (tp is null)
                    throw new ApplicationException("Failed to get type: CADability.DebuggerVisualizers.DebugForm");
                else
                {
                    ConstructorInfo ci = tp.GetConstructor(new Type[0]);
                    if (ci is null)
                        throw new ApplicationException("Failed to get Constructor of CADability.DebuggerVisualizers.DebugForm");
                    else
                    {
                        object df = ci.Invoke(new object[0]);
                        return df as IDebugForm;
                    }
                }
            }
        }
    }
    public static class DebuggerExtensions
    {
        public static DebuggerContainer Show(this IEnumerable<object> obj)
        {
            return DebuggerContainer.Show(obj);
        }
    }

    /* So benutzt man den DebuggerVisualizer aus dem Command Window:
     * ? GeneralDebuggerVisualizer.TestShowVisualizer(res.DebugEdges3D);
     */
    public class GeneralDebuggerVisualizer : DialogDebuggerVisualizer
    {
        public GeneralDebuggerVisualizer() : base(FormatterPolicy.Json)
        {
        }

        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            IDebugForm form = CF.DebugForm;
            Model m = form.Model;
            using (var stream = objectProvider.GetData() as Stream)
            using (var reader = new StreamReader(stream))
            {
                string json = reader.ReadToEnd();
                // Now you can deserialize json with your custom deserializer
                object o = JsonSerialize.FromString(json);
                if (o is IDebuggerVisualizer dv)
                {
                    m.Add(dv.GetList());
                }
                else if (o is IGeoObject)
                {
                    m.Add(o as IGeoObject);
                }
                form.ShowDialog(windowService);
            }
        }
    }
    class VisualizerHelper
    {
        static private ColorDef pointColor = null;
        static public ColorDef PointColor
        {
            get
            {
                if (pointColor == null)
                {
                    pointColor = new ColorDef("auto point", Color.Brown);
                }
                return pointColor;
            }
        }
        static private ColorDef curveColor = null;
        static public ColorDef CurveColor
        {
            get
            {
                if (curveColor == null)
                {
                    curveColor = new ColorDef("auto point", Color.DarkCyan);
                }
                return curveColor;
            }
        }
        static private ColorDef faceColor = null;
        static public ColorDef FaceColor
        {
            get
            {
                if (faceColor == null)
                {
                    faceColor = new ColorDef("auto point", Color.GreenYellow);
                }
                return faceColor;
            }
        }
        static public IGeoObject AssertColor(IGeoObject go)
        {
            if (go is IColorDef cd && cd.ColorDef == null)
            {
                if (go is GeoObject.Point) cd.ColorDef = PointColor;
                if (go is ICurve) cd.ColorDef = CurveColor;
                if (go is Face) cd.ColorDef = FaceColor;
                if (go is Shell) cd.ColorDef = FaceColor;
                if (go is Solid) cd.ColorDef = FaceColor;
            }
            return go;
        }
        static public GeoObjectList AssertColor(GeoObjectList list)
        {
            foreach (IGeoObject go in list)
            {
                if (go is IColorDef cd && cd.ColorDef == null)
                {
                    if (go is GeoObject.Point) cd.ColorDef = PointColor;
                    if (go is ICurve) cd.ColorDef = CurveColor;
                    if (go is Face) cd.ColorDef = FaceColor;
                    if (go is Shell) cd.ColorDef = FaceColor;
                    if (go is Solid) cd.ColorDef = FaceColor;
                }
            }
            return list;
        }

    }
    public class GeoObjectVisualizer : DialogDebuggerVisualizer
    {
        public GeoObjectVisualizer() : base(FormatterPolicy.Json)
        {
        }

        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            using (var stream = objectProvider.GetData() as Stream)
            using (var reader = new StreamReader(stream))
            {
                string json = reader.ReadToEnd();
                // Now you can deserialize json with your custom deserializer
                IGeoObjectImpl go = JsonSerialize.FromString(json) as IGeoObjectImpl;

                DebugForm form = new DebugForm();
                Model m = form.Model;
                m.Add(VisualizerHelper.AssertColor(go));
                form.ShowDialog(windowService);
            }
        }

        public static void TestShowVisualizer(object objectToVisualize)
        {
            VisualizerDevelopmentHost visualizerHost = new VisualizerDevelopmentHost(objectToVisualize, typeof(GeoObjectVisualizer), typeof(SerializeToJsonOjectSource));
            visualizerHost.ShowVisualizer();
        }
    }

    public class GeoObjectListVisualizer : DialogDebuggerVisualizer
    {
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            IDebugForm form = CF.DebugForm;
            Model m = form.Model;

            GeoObjectList list = (GeoObjectList)objectProvider.GetObject();

            if (list.Count > 0)
            {
                for (int i = 0; i < list.Count; ++i)
                {
                    IntegerProperty ip = new IntegerProperty(i, "Debug.Hint");
                    list[i].UserData.Add("ListIndex", ip);
                    m.Add(VisualizerHelper.AssertColor(list[i]));
                }
                m.Add(list);
            }
            form.ShowDialog(windowService);
        }
        public static void TestShowVisualizer(object objectToVisualize)
        {
            VisualizerDevelopmentHost visualizerHost = new VisualizerDevelopmentHost(objectToVisualize, typeof(GeoObjectListVisualizer));
            visualizerHost.ShowVisualizer();
        }
    }

    public class BorderVisualizer : DialogDebuggerVisualizer
    {
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            IDebugForm form = CF.DebugForm;
            Model m = form.Model;

            Border bdr = (Border)objectProvider.GetObject();
            for (int i = 0; i < bdr.DebugList.Count; ++i)
            {
                IGeoObject toAdd = bdr.DebugList[i];
                IntegerProperty ip = new IntegerProperty(i, "Debug.Hint");
                toAdd.UserData.Add("Debug", ip);
                VisualizerHelper.AssertColor(toAdd);
                m.Add(toAdd);
            }

            form.ShowDialog(windowService);
        }

        public static void TestBorderVisualizer(object objectToVisualize)
        {
            VisualizerDevelopmentHost myHost = new VisualizerDevelopmentHost(objectToVisualize, typeof(BorderVisualizer));
            myHost.ShowVisualizer();
        }
    }

    public class Curve2DVisualizer : DialogDebuggerVisualizer
    {
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            IDebugForm form = CF.DebugForm;
            Model m = form.Model;

            ICurve2D gc2d = (ICurve2D)objectProvider.GetObject();
            m.Add(VisualizerHelper.AssertColor(gc2d.MakeGeoObject(Plane.XYPlane)));

            form.ShowDialog(windowService);
        }
    }

    public class CurveVisualizer : DialogDebuggerVisualizer
    {
        public CurveVisualizer() : base(FormatterPolicy.Json) { }
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            IDebugForm form = CF.DebugForm;
            Model m = form.Model;
            using (var stream = objectProvider.GetData() as Stream)
            using (var reader = new StreamReader(stream))
            {
                string json = reader.ReadToEnd();
                // Now you can deserialize json with your custom deserializer
                IGeoObjectImpl go = JsonSerialize.FromString(json) as IGeoObjectImpl;

                m.Add(VisualizerHelper.AssertColor(go));
                form.ShowDialog(windowService);
                VisualizerHelper.AssertColor(go);
                m.Add(go);

                form.ShowDialog(windowService);
            }
        }
    }

    public class GeoPoint2DVisualizer : DialogDebuggerVisualizer
    {
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            IDebugForm form = CF.DebugForm;
            Model m = form.Model;

            GeoPoint2D p = (GeoPoint2D)objectProvider.GetObject();
            Point pnt = Point.Construct();
            pnt.Location = new GeoPoint(p);
            pnt.Symbol = PointSymbol.Cross;
            VisualizerHelper.AssertColor(pnt);
            m.Add(pnt);

            form.ShowDialog(windowService);
        }
    }

    public class GeoPointVisualizer : DialogDebuggerVisualizer
    {
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            IDebugForm form = CF.DebugForm;
            Model m = form.Model;

            GeoPoint p = (GeoPoint)objectProvider.GetObject();
            Point pnt = Point.Construct();
            pnt.Location = p;
            pnt.Symbol = PointSymbol.Cross;
            VisualizerHelper.AssertColor(pnt);
            m.Add(pnt);

            form.ShowDialog(windowService);
        }
    }

    public class CompoundShapeVisualizer : DialogDebuggerVisualizer
    {
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            IDebugForm form = CF.DebugForm;
            Model m = form.Model;

            CompoundShape compoundShape = (CompoundShape)objectProvider.GetObject();
            m.Add(VisualizerHelper.AssertColor(compoundShape.DebugList));

            form.ShowDialog(windowService);
        }
    }

    public class SimpleShapeVisualizer : DialogDebuggerVisualizer
    {
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            IDebugForm form = CF.DebugForm;
            Model m = form.Model;

            SimpleShape simpleShape = (SimpleShape)objectProvider.GetObject();
            m.Add(VisualizerHelper.AssertColor(simpleShape.DebugList));

            form.ShowDialog(windowService);
        }

        /// <summary>
        /// Damit kann man den Visualizer zum Debuggen im Context von CADability aufrufen, sonst läuft er immer im
        /// Context des Debuggers
        /// </summary>
        /// <param name="objectToVisualize">The object to display in the visualizer.</param>
        public static void TestShowVisualizer(object objectToVisualize)
        {
            VisualizerDevelopmentHost visualizerHost = new VisualizerDevelopmentHost(objectToVisualize, typeof(SimpleShapeVisualizer));
            visualizerHost.ShowVisualizer();
        }
    }
}