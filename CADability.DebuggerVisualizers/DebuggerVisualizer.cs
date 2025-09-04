/// -----------------------------------------------------------------------------------------------------------------------------------------
/// How to use Debugger Visualizers for Visual Studio 2022:
/// Copy all files from ...\CADability\CADability.DebuggerVisualizers\bin\Debug folder to
/// ...\My Documents\Visual Studio 2022\Visualizers
/// No need to create any subfolders!
/// See: https://learn.microsoft.com/en-us/visualstudio/debugger/how-to-install-a-visualizer?view=vs-2022
/// This applies to Visual Studio 2022 and 2019

using CADability;
using CADability.Attribute;
using CADability.Curve2D;
using CADability.DebuggerVisualizers;
using CADability.DebuggerVisualizers.ObjectSource;
using CADability.GeoObject;
using CADability.Shapes;
using CADability.UserInterface;
using Microsoft.VisualStudio.DebuggerVisualizers;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Windows.Forms;
using Point = CADability.GeoObject.Point;

#region "Types to be visualized"

//Border
[assembly: DebuggerVisualizer(typeof(BorderVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.Shapes.Border, CADability", Description = "CADability Border Visualizer")]

//GeoObjectList
[assembly: DebuggerVisualizer(typeof(GeoObjectListVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.GeoObject.GeoObjectList, CADability", Description = "CADability GeoObjectList Visualizer")]

//Simple Shape
[assembly: DebuggerVisualizer(typeof(SimpleShapeVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.Shapes.SimpleShape, CADability", Description = "CADability Simple Shape Visualizer")]

//Compound Shape
[assembly: DebuggerVisualizer(typeof(CompoundShapeVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.Shapes.CompoundShape, CADability", Description = "CADability Compound Shape Visualizer")]

//GeoPoint2D
[assembly: DebuggerVisualizer(typeof(GeoPoint2DVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.GeoPoint2D, CADability", Description = "CADability GeoPoint2D Visualizer")]

//GeoPoint
[assembly: DebuggerVisualizer(typeof(GeoPointVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.GeoPoint, CADability", Description = "CADability GeoPoint Visualizer")]

//ICurve2D Implementations
[assembly: DebuggerVisualizer(typeof(Curve2DVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.Curve2D.GeneralCurve2D, CADability", Description = "CADability GeneralCurve2D Visualizer")]

[assembly: DebuggerVisualizer(typeof(Curve2DVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.Curve2DAspect, CADability", Description = "CADability Curve2DAspect Visualizer")]

//IGeoObject Implementations
[assembly: DebuggerVisualizer(typeof(GeoObjectVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.GeoObject.IGeoObjectImpl, CADability", Description = "CADability GeoObject Visualizer 4")]

//ICurve Implementations
[assembly: DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.GeoObject.BSpline", Description = "CADability ICurve Visualizer")]

[assembly: DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.GeoObject.Ellipse, CADability", Description = "CADability ICurve Visualizer")]

[assembly: DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.GeoObject.GeneralCurve, CADability", Description = "CADability ICurve Visualizer")]

[assembly: DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.GeoObject.Line, CADability", Description = "CADability ICurve Visualizer")]

[assembly: DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.GeoObject.Path, CADability", Description = "CADability ICurve Visualizer")]

[assembly: DebuggerVisualizer(typeof(CurveVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.GeoObject.Polyline, CADability", Description = "CADability ICurve Visualizer")]

//IDebuggerVisualizer Implementations
[assembly: DebuggerVisualizer(typeof(GeneralDebuggerVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.BRepItem, CADability", Description = "CADability IDebuggerVisualizer Visualizer")]

[assembly: DebuggerVisualizer(typeof(GeneralDebuggerVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.Curve2D.BSpline2D, CADability", Description = "CADability IDebuggerVisualizer Visualizer")]

[assembly: DebuggerVisualizer(typeof(GeneralDebuggerVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.GeoObject.BoxedSurfaceEx.ParEpi, CADability", Description = "CADability IDebuggerVisualizer Visualizer")]

[assembly: DebuggerVisualizer(typeof(GeneralDebuggerVisualizer), typeof(SerializeToJsonOjectSource),
TargetTypeName = "CADability.DebuggerContainer, CADability", Description = "CADability DebuggerContainer Visualizer")]

//[assembly: DebuggerVisualizer(typeof(GeneralDebuggerVisualizer), typeof(SerializeToJsonOjectSource),
//Target = typeof(System.Collections.Generic.List<CADability.Edge>), Description = "CADability Edges Visualizer")]
//[assembly: DebuggerVisualizer(typeof(GeneralDebuggerVisualizer), typeof(SerializeToJsonOjectSource),
//Target = typeof(CADability.Edge[]), Description = "CADability Edges Visualizer")]
//[assembly: DebuggerVisualizer(typeof(GeneralDebuggerVisualizer), typeof(SerializeToJsonOjectSource),
//Target = typeof(System.Collections.Generic.HashSet<CADability.Edge>), Description = "CADability Edges Visualizer")]
#endregion



namespace CADability.DebuggerVisualizers
{
 
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
        public static object GetObject(IVisualizerObjectProvider objectProvider)
        {
            using (var stream = objectProvider.GetData() as Stream)
            using (var reader = new StreamReader(stream))
            {
                string json = reader.ReadToEnd();
                return JsonSerialize.FromString(json) ;
            }
        }

        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            DebugForm form = new DebugForm();
            Model m = form.Model;
            using (var stream = objectProvider.GetData() as Stream)
            using (var reader = new StreamReader(stream))
            {
                string json = reader.ReadToEnd();
                object o = JsonSerialize.FromString(json);
                if (o is IDebuggerVisualizer dv)
                {
                    m.Add(dv.GetList());
                }
                else if (o is IGeoObject)
                {
                    m.Add(o as IGeoObject);
                }
                else if (o is GeoObjectList l)
                {
                    m.Add(l);
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
        public GeoObjectListVisualizer() : base(FormatterPolicy.Json) { }
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            DebugForm form = new DebugForm();
            Model m = form.Model;

            GeoObjectList list = (GeoObjectList)GeneralDebuggerVisualizer.GetObject(objectProvider);

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
        public BorderVisualizer() : base(FormatterPolicy.Json) { }

        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            DebugForm form = new DebugForm();
            Model m = form.Model;

            Border bdr = (Border)GeneralDebuggerVisualizer.GetObject(objectProvider);
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
        public Curve2DVisualizer() : base(FormatterPolicy.Json) { }
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            DebugForm form = new DebugForm();

            Model m = form.Model;

            ICurve2D gc2d = (ICurve2D)GeneralDebuggerVisualizer.GetObject(objectProvider);
            m.Add(VisualizerHelper.AssertColor(gc2d.MakeGeoObject(Plane.XYPlane)));

            form.ShowDialog(windowService);
        }
    }

    public class CurveVisualizer : DialogDebuggerVisualizer
    {
        public CurveVisualizer() : base(FormatterPolicy.Json) { }
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            DebugForm form = new DebugForm();

            Model m = form.Model;
            using (var stream = objectProvider.GetData() as Stream)
            using (var reader = new StreamReader(stream))
            {
                string json = reader.ReadToEnd();
                // Now you can deserialize json with your custom deserializer
                IGeoObjectImpl go = JsonSerialize.FromString(json) as IGeoObjectImpl;

                m.Add(VisualizerHelper.AssertColor(go));
                form.ShowDialog(windowService);
            }
        }
    }

    public class GeoPoint2DVisualizer : DialogDebuggerVisualizer
    {
        public GeoPoint2DVisualizer() : base(FormatterPolicy.Json) { }
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            DebugForm form = new DebugForm();

            Model m = form.Model;

            GeoPoint2D p = (GeoPoint2D)GeneralDebuggerVisualizer.GetObject(objectProvider);
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
        public GeoPointVisualizer() : base(FormatterPolicy.Json) { }
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            DebugForm form = new DebugForm();

            Model m = form.Model;

            GeoPoint p = (GeoPoint)GeneralDebuggerVisualizer.GetObject(objectProvider);
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
        public CompoundShapeVisualizer() : base(FormatterPolicy.Json) { }
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            DebugForm form = new DebugForm();

            Model m = form.Model;

            CompoundShape compoundShape = (CompoundShape)GeneralDebuggerVisualizer.GetObject(objectProvider);
            m.Add(VisualizerHelper.AssertColor(compoundShape.DebugList));

            form.ShowDialog(windowService);
        }
    }

    public class SimpleShapeVisualizer : DialogDebuggerVisualizer
    {
        public SimpleShapeVisualizer() : base(FormatterPolicy.Json) { }
        protected override void Show(IDialogVisualizerService windowService, IVisualizerObjectProvider objectProvider)
        {
            DebugForm form = new DebugForm();

            Model m = form.Model;

            SimpleShape simpleShape = (SimpleShape)GeneralDebuggerVisualizer.GetObject(objectProvider);
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