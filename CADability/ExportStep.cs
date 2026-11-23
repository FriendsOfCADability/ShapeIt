﻿using CADability.Attribute;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace CADability
{
    /// <summary>
    /// Implemented by the objects which can be written to a STEP file. <paramref name="topLevel"/> is true for an object of
    /// the model: it returns the representation item which represents the whole object (MANIFOLD_SOLID_BREP for a solid,
    /// SHELL_BASED_SURFACE_MODEL for a shell or face, a bounded curve for a curve). Otherwise it returns the entity which is
    /// used as part of another entity (e.g. the CLOSED_SHELL of a solid, the unbounded curve of an edge).
    /// </summary>
    public interface IExportStep
    {
        int Export(ExportStep export, bool topLevel);
    }

    /// <summary>
    /// Writes the active model of a project as a STEP file (ISO 10303-21, AP214 "automotive_design").
    /// The model is written as a single part: one PRODUCT with one shape. Its SHAPE_REPRESENTATION is related to
    /// an ADVANCED_BREP_SHAPE_REPRESENTATION with the solids, a MANIFOLD_SURFACE_SHAPE_REPRESENTATION with the shells
    /// and faces and a GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION with the curves, as far as there are such
    /// objects. Blocks and paths are resolved into their parts, there is no assembly structure. The colors are written
    /// as STYLED_ITEMs of a MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION.
    /// </summary>
    public class ExportStep
    {
        // The fixed entities at the beginning of the DATA section: the contexts and the units.
        // All entities written by the objects get numbers starting with firstFreeItem.
        private const string contextEntities =
@"#1=APPLICATION_CONTEXT('core data for automotive mechanical design processes');
#2=APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#1);
#3=PRODUCT_CONTEXT('',#1,'mechanical');
#4=PRODUCT_DEFINITION_CONTEXT('part definition',#1,'design');
#5=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#6=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
#7=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());
#8=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-03),#5,'distance_accuracy_value','confusion accuracy');
#9=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#8))GLOBAL_UNIT_ASSIGNED_CONTEXT((#5,#6,#7))REPRESENTATION_CONTEXT('',''));";
        private const string productContext = "#3";
        private const string productDefinitionContext = "#4";
        private const string representationContext = "#9";
        private const int firstFreeItem = 10;

        private TextWriter stream;
        private int currentItem;
        private List<int> styledItemList;
        public double Precision { get; internal set; }

        public Dictionary<Vertex, int> VertexToDefInd;
        public Dictionary<Edge, int> EdgeToDefInd;
        /// <summary>
        /// The PRESENTATION_STYLE_ASSIGNMENT for faces, shells and solids of each color
        /// </summary>
        public Dictionary<ColorDef, int> StyledItems;
        /// <summary>
        /// The PRESENTATION_STYLE_ASSIGNMENT for curves of each color
        /// </summary>
        internal Dictionary<ColorDef, int> CurveStyles;
        /// <summary>
        /// The COLOUR_RGB of each color
        /// </summary>
        internal Dictionary<ColorDef, int> Colours;

        public ExportStep()
        {
            Precision = 1e-6;
            Reset();
        }

        private void Reset()
        {
            VertexToDefInd = new Dictionary<Vertex, int>();
            EdgeToDefInd = new Dictionary<Edge, int>();
            StyledItems = new Dictionary<ColorDef, int>();
            CurveStyles = new Dictionary<ColorDef, int>();
            Colours = new Dictionary<ColorDef, int>();
            styledItemList = new List<int>();
            currentItem = firstFreeItem;
        }

        public byte[] WriteToByteArray(Project pr)
        {
            MemoryStream memoryStream = new MemoryStream();
            using (StreamWriter writer = new StreamWriter(memoryStream, new UTF8Encoding(false)))
            {
                Write(writer, pr.GetActiveModel(), pr.FileName);
            }
            return memoryStream.ToArray();
        }

        public void WriteToFile(string fileName, Project pr)
        {
            using (StreamWriter writer = new StreamWriter(fileName, false, new UTF8Encoding(false)))
            {
                Write(writer, pr.GetActiveModel(), fileName);
            }
        }

        /// <summary>
        /// Writes the objects of <paramref name="model"/> as a single part. <paramref name="fileName"/> is used for the
        /// FILE_NAME of the header and as the name of the part (without path and extension).
        /// </summary>
        private void Write(TextWriter writer, Model model, string fileName)
        {
            stream = writer;
            Reset();
            List<IGeoObject> objects = new List<IGeoObject>();
            foreach (IGeoObject go in model) Flatten(go, objects);

            string fileNameOnly = string.IsNullOrEmpty(fileName) ? "" : System.IO.Path.GetFileName(fileName);
            string partName = string.IsNullOrEmpty(fileName) ? "" : System.IO.Path.GetFileNameWithoutExtension(fileName);
            List<IGeoObject> bodies = objects.FindAll(go => go is Solid || go is Shell || go is Face);
            if (bodies.Count == 1)
            {   // a single named body names the part: the import names a single body after its product, so it keeps its name
                string bodyName = (bodies[0] as Solid)?.Name ?? (bodies[0] as Shell)?.Name ?? (bodies[0] as Face)?.Name;
                if (!string.IsNullOrWhiteSpace(bodyName)) partName = bodyName;
            }
            if (string.IsNullOrWhiteSpace(partName)) partName = "part";

            WriteHeader(fileNameOnly);

            List<int> solids = new List<int>();
            List<int> surfaceModels = new List<int>();
            List<int> curveSets = new List<int>();
            foreach (IGeoObject go in objects)
            {
                if (go is Solid)
                {
                    solids.Add((go as IExportStep).Export(this, true));
                }
                else if (go is Shell || go is Face)
                {
                    surfaceModels.Add((go as IExportStep).Export(this, true));
                }
                else if (go is ICurve && go is IExportStep)
                {
                    int curve = (go as IExportStep).Export(this, true);
                    int gcs = WriteDefinition("GEOMETRIC_CURVE_SET('',(#" + curve.ToString() + "))");
                    if (go is IColorDef icd && icd.ColorDef != null) icd.ColorDef.MakeStepStyle(gcs, this, true);
                    curveSets.Add(gcs);
                }
            }

            string name = StepString(partName);
            List<int> representations = new List<int>();
            if (solids.Count > 0)
            {
                representations.Add(WriteDefinition("ADVANCED_BREP_SHAPE_REPRESENTATION(" + name + ",(" + ToString(solids.ToArray(), true) + ")," + representationContext + ")"));
            }
            if (surfaceModels.Count > 0)
            {
                representations.Add(WriteDefinition("MANIFOLD_SURFACE_SHAPE_REPRESENTATION(" + name + ",(" + ToString(surfaceModels.ToArray(), true) + ")," + representationContext + ")"));
            }
            if (curveSets.Count > 0)
            {
                representations.Add(WriteDefinition("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION(" + name + ",(" + ToString(curveSets.ToArray(), true) + ")," + representationContext + ")"));
            }
            int axis = WriteAxis2Placement3d(GeoPoint.Origin, GeoVector.ZAxis, GeoVector.XAxis);
            int shapeRepresentation = WriteDefinition("SHAPE_REPRESENTATION(" + name + ",(#" + axis.ToString() + ")," + representationContext + ")");
            foreach (int representation in representations)
            {   // without transformation: all representations use the coordinate system of the shape representation
                WriteDefinition("SHAPE_REPRESENTATION_RELATIONSHIP('','',#" + shapeRepresentation.ToString() + ",#" + representation.ToString() + ")");
            }

            int product = WriteDefinition("PRODUCT(" + name + "," + name + ",''," + "(" + productContext + "))");
            WriteDefinition("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#" + product.ToString() + "))");
            int formation = WriteDefinition("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',#" + product.ToString() + ",.NOT_KNOWN.)");
            int definition = WriteDefinition("PRODUCT_DEFINITION('design','',#" + formation.ToString() + "," + productDefinitionContext + ")");
            int definitionShape = WriteDefinition("PRODUCT_DEFINITION_SHAPE('','',#" + definition.ToString() + ")");
            WriteDefinition("SHAPE_DEFINITION_REPRESENTATION(#" + definitionShape.ToString() + ",#" + shapeRepresentation.ToString() + ")");

            if (styledItemList.Count > 0)
            {
                WriteDefinition("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',(" + ToString(styledItemList.ToArray(), true) + ")," + representationContext + ")");
            }
            stream.WriteLine("ENDSEC;");
            stream.WriteLine("END-ISO-10303-21;");
            stream.Flush();
        }

        /// <summary>
        /// Resolves blocks and paths into their parts. The parts of a path get the attributes (color) of the path.
        /// </summary>
        private static void Flatten(IGeoObject go, List<IGeoObject> objects)
        {
            if (go is Block blk)
            {
                foreach (IGeoObject child in blk.Children) Flatten(child, objects);
            }
            else if (go is GeoObject.Path path)
            {
                foreach (ICurve curve in path.Curves)
                {
                    if (curve is IGeoObject part)
                    {
                        IGeoObject clone = part.Clone();
                        clone.CopyAttributes(path);
                        Flatten(clone, objects);
                    }
                }
            }
            else if (go is IExportStep)
            {
                objects.Add(go);
            }
        }

        private void WriteHeader(string fileName)
        {
            Version version = typeof(ExportStep).Assembly.GetName().Version;
            string system = "CADability " + version.ToString(3);
            stream.WriteLine("ISO-10303-21;");
            stream.WriteLine("HEADER;");
            stream.WriteLine("FILE_DESCRIPTION(('CADability model'),'2;1');");
            stream.WriteLine("FILE_NAME(" + StepString(fileName) + "," + StepString(DateTime.Now.ToString("yyyy-MM-dd'T'HH:mm:ss", CultureInfo.InvariantCulture))
                + ",(''),(''),"
                + StepString(system) + "," + StepString(system) + ",'');");
            stream.WriteLine("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));");
            stream.WriteLine("ENDSEC;");
            stream.WriteLine("DATA;");
            stream.WriteLine(contextEntities.Replace("\r\n", "\n").Replace("\n", stream.NewLine));
        }

        /// <summary>
        /// Writes a STYLED_ITEM, which is collected in the MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION
        /// </summary>
        internal int WriteStyledItem(int presentationStyleAssignment, int item)
        {
            int styledItem = WriteDefinition("STYLED_ITEM('',(#" + presentationStyleAssignment.ToString() + "),#" + item.ToString() + ")");
            styledItemList.Add(styledItem);
            return styledItem;
        }

        /// <summary>
        /// Returns <paramref name="s"/> as a STEP string literal including the quotes: apostrophe and backslash are
        /// doubled, characters outside of the printable ASCII range are encoded as \X2\...\X0\ (or \X4\...\X0\).
        /// </summary>
        public static string StepString(string s)
        {
            StringBuilder res = new StringBuilder("'");
            if (s != null)
            {
                int i = 0;
                while (i < s.Length)
                {
                    char c = s[i];
                    if (c == '\'')
                    {
                        res.Append("''");
                        ++i;
                    }
                    else if (c == '\\')
                    {
                        res.Append("\\\\");
                        ++i;
                    }
                    else if (c >= ' ' && c <= '~')
                    {
                        res.Append(c);
                        ++i;
                    }
                    else
                    {   // a sequence of characters which must be encoded
                        int start = i;
                        bool surrogates = false;
                        while (i < s.Length && (s[i] < ' ' || s[i] > '~'))
                        {
                            if (char.IsSurrogate(s[i])) surrogates = true;
                            ++i;
                        }
                        if (surrogates)
                        {
                            res.Append("\\X4\\");
                            for (int j = start; j < i; j++)
                            {
                                int codePoint;
                                if (char.IsHighSurrogate(s[j]) && j + 1 < i && char.IsLowSurrogate(s[j + 1]))
                                {
                                    codePoint = char.ConvertToUtf32(s[j], s[j + 1]);
                                    ++j;
                                }
                                else codePoint = s[j];
                                res.Append(codePoint.ToString("X8"));
                            }
                        }
                        else
                        {
                            res.Append("\\X2\\");
                            for (int j = start; j < i; j++) res.Append(((int)s[j]).ToString("X4"));
                        }
                        res.Append("\\X0\\");
                    }
                }
            }
            res.Append("'");
            return res.ToString();
        }

        public int WriteDefinition(string s)
        {
            int nr = currentItem;
            ++currentItem;
            stream.WriteLine("#" + nr.ToString() + "=" + s + ";");
            return nr;
        }
        public string ToString(double d)
        {
            string res = d.ToString("G", CultureInfo.InvariantCulture);
            if (res.IndexOf('.') == -1)
            {
                int ind;
                if ((ind = res.IndexOf('E')) >= 0) res = res.Insert(ind, ".");
                else res = res + ".";
            }
            return res;
        }
        public int WriteAxis2Placement3d(GeoPoint location, GeoVector normal, GeoVector xdir)
        {
            int nl = (location as IExportStep).Export(this, false);
            int n = (normal.Normalized as IExportStep).Export(this, false);
            int nx = (xdir.Normalized as IExportStep).Export(this, false);
            return WriteDefinition("AXIS2_PLACEMENT_3D('',#" + nl.ToString() + ",#" + n.ToString() + ",#" + nx.ToString() + ")");
        }
        public int WriteAxis1Placement3d(GeoPoint location, GeoVector normal)
        {
            int nl = (location as IExportStep).Export(this, false);
            int n = (normal.Normalized as IExportStep).Export(this, false);
            return WriteDefinition("AXIS1_PLACEMENT('',#" + nl.ToString() + ",#" + n.ToString() + ")");
        }

        internal string ToString(int[] ints, bool makeReference)
        {
            StringBuilder res = new StringBuilder();
            for (int i = 0; i < ints.Length; i++)
            {
                if (i == 0)
                {
                    if (makeReference) res.Append("#");
                }
                else
                {
                    if (makeReference) res.Append(",#");
                    else res.Append(",");
                }
                res.Append(ints[i].ToString());
            }
            return res.ToString();
        }
        internal string ToString(double[] vals)
        {
            StringBuilder res = new StringBuilder();
            for (int i = 0; i < vals.Length; i++)
            {
                if (i != 0)
                {
                    res.Append(",");
                }
                res.Append(ToString(vals[i]));
            }
            return res.ToString();
        }

        internal string Write(GeoPoint[] p)
        {
            StringBuilder res = new StringBuilder();
            for (int i = 0; i < p.Length; i++)
            {
                if (i == 0) res.Append("#");
                else res.Append(",#");
                res.Append((p[i] as IExportStep).Export(this, false).ToString());
            }
            return res.ToString();
        }
    }
}
