using CADability;
using CADability.GeoObject;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;

namespace ShapeIt
{
    /// <summary>
    /// Where a feature made by a template comes from: the template, the instantiation (a unique id), and the values of
    /// its value parameters as JSON, so that the feature can be found as a whole and made again with changed values.
    /// The faces of the feature also carry its <see cref="Name"/> as a feature name of the shell (see
    /// <see cref="Shell.AddFeature"/>), which the feature commands of the application work with.
    /// </summary>
    public sealed class TemplateFeatureInfo : IJsonSerialize
    {
        public string Name { get; private set; } = "";
        public string Template { get; private set; } = "";
        public string Id { get; private set; } = "";
        /// <summary>The value parameters as a JSON object, e.g. {"d":8,"pos":[10,10,20],"entry":"chamfer"}.</summary>
        public string Parameters { get; private set; } = "{}";

        public TemplateFeatureInfo() { } // for JSON
        public TemplateFeatureInfo(string name, string template, string id, string parameters)
        {
            Name = name;
            Template = template;
            Id = id;
            Parameters = parameters;
        }

        public void GetObjectData(IJsonWriteData data)
        {
            data.AddProperty("Name", Name);
            data.AddProperty("Template", Template);
            data.AddProperty("Id", Id);
            data.AddProperty("Parameters", Parameters);
        }

        public void SetObjectData(IJsonReadData data)
        {
            Name = data.GetStringProperty("Name");
            Template = data.GetStringProperty("Template");
            Id = data.GetStringProperty("Id");
            Parameters = data.GetStringProperty("Parameters");
        }
    }

    /// <summary>
    /// The template features a face belongs to, kept in its UserData under <see cref="TemplateFeatures.Key"/>. Usually
    /// one; a template run inside another template adds the outer feature to the faces of the inner one.
    /// </summary>
    public sealed class TemplateFeatureList : IJsonSerialize, ICloneable
    {
        public List<TemplateFeatureInfo> Features { get; } = new List<TemplateFeatureInfo>();

        public TemplateFeatureList() { } // for JSON

        // a split face gets a list of its own (UserData clones ICloneable values); the infos themselves do not change
        public object Clone()
        {
            TemplateFeatureList res = new TemplateFeatureList();
            res.Features.AddRange(Features);
            return res;
        }

        public void GetObjectData(IJsonWriteData data)
        {
            data.AddProperty("Features", Features.ToArray());
        }

        public void SetObjectData(IJsonReadData data)
        {
            TemplateFeatureInfo[] features = data.GetProperty<TemplateFeatureInfo[]>("Features");
            if (features != null) Features.AddRange(features);
        }
    }

    /// <summary>
    /// Marks the faces a template run has made. A face of the result is new when it is no part of a face of the
    /// solids the template got as input (its solid and face parameters): the faces where material was taken away or
    /// added. Faces of the input that were only trimmed or split stay as they were, with the marks they had.
    /// </summary>
    public static class TemplateFeatures
    {
        /// <summary>The UserData key of the <see cref="TemplateFeatureList"/> of a face.</summary>
        public const string Key = "ShapeIt.TemplateFeature";

        /// <summary>The template features of a face, empty when it was made by no template.</summary>
        public static IReadOnlyList<TemplateFeatureInfo> Of(Face face)
        {
            return face.UserData.GetData(Key) is TemplateFeatureList list ? list.Features : Array.Empty<TemplateFeatureInfo>();
        }

        /// <summary>The faces of a shell that belong to the template feature with the given id.</summary>
        public static List<Face> FacesOf(Shell shell, string id)
        {
            return shell.Faces.Where(f => Of(f).Any(info => info.Id == id)).ToList();
        }

        /// <summary>
        /// Whether a face of a result is part of a face of the input: on the same surface, and either it contains an
        /// inner point of an input face (a face that was merged with an added one) or an input face contains an inner
        /// point of it (a face that was trimmed or split).
        /// </summary>
        public static bool IsOld(Face face, IEnumerable<Shell> inputs)
        {
            foreach (Shell input in inputs)
            {
                if (input.FindSimilarFace(face) != null) return true;
                foreach (Face old in input.Faces)
                {
                    if (!face.Surface.SameGeometry(face.Domain, old.Surface, old.Domain, Precision.eps, out _)) continue;
                    GeoPoint2D uv = face.Surface.PositionOf(old.Surface.PointAt(old.Area.GetSomeInnerPoint()));
                    if (face.Contains(ref uv, false)) return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Marks the new faces of the result solids as one feature per solid: a feature name of the shell, the label of
        /// the template followed by the next free number ("Through hole 3"), and the template, a new id and the
        /// parameter values.
        /// </summary>
        /// <returns>the id of the feature, null when the result has no new faces</returns>
        public static string? Mark(IEnumerable<Solid> results, IEnumerable<Solid> inputs, string template, string label, string parameters)
        {
            List<Shell> inputShells = inputs.SelectMany(s => s.Shells).ToList();
            string id = Guid.NewGuid().ToString("N");
            bool marked = false;
            foreach (Solid result in results)
            {
                foreach (Shell shell in result.Shells)
                {
                    List<Face> newFaces = shell.Faces.Where(f => !IsOld(f, inputShells)).ToList();
                    if (newFaces.Count == 0) continue;
                    // a new face may have inherited the feature names of an input face (a fillet takes the attributes
                    // of its primary face): they do not belong to it
                    HashSet<string> inputNames = new HashSet<string>(inputShells.SelectMany(s => s.GetFeatures().Keys));
                    foreach (Face face in newFaces)
                    {
                        if (face.UserData.GetData("CADability.Feature") is HashSet<string> names) names.ExceptWith(inputNames);
                    }
                    string name = NextName(shell, label);
                    shell.AddFeature(name, newFaces);
                    TemplateFeatureInfo info = new TemplateFeatureInfo(name, template, id, parameters);
                    foreach (Face face in newFaces)
                    {
                        if (face.UserData.GetData(Key) is TemplateFeatureList)
                        {   // marks of an inner template stay, marks inherited from an input face go
                            TemplateFeatureList list = new TemplateFeatureList();
                            foreach (TemplateFeatureInfo inner in Of(face).Where(i => !IsMarkOfInput(i, inputShells))) list.Features.Add(inner);
                            list.Features.Add(info);
                            face.UserData[Key] = list;
                        }
                        else
                        {
                            TemplateFeatureList list = new TemplateFeatureList();
                            list.Features.Add(info);
                            face.UserData[Key] = list;
                        }
                    }
                    marked = true;
                }
            }
            return marked ? id : null;
        }

        // a mark some face of the input carries: inherited by a new face through CopyAttributes, it does not describe it
        private static bool IsMarkOfInput(TemplateFeatureInfo info, List<Shell> inputs)
        {
            return inputs.Any(shell => shell.Faces.Any(f => Of(f).Any(i => i.Id == info.Id)));
        }

        /// <summary>The label followed by the smallest number above those of the features of the shell with this label.</summary>
        private static string NextName(Shell shell, string label)
        {
            int highest = 0;
            foreach (string name in shell.GetFeatures().Keys)
            {
                if (name.StartsWith(label + " ", StringComparison.Ordinal)
                    && int.TryParse(name.Substring(label.Length + 1), NumberStyles.None, CultureInfo.InvariantCulture, out int n))
                    highest = Math.Max(highest, n);
            }
            return label + " " + (highest + 1).ToString(CultureInfo.InvariantCulture);
        }

        /// <summary>
        /// The values of the value parameters as a JSON object: numbers, true/false, strings and points (as [x, y, z]).
        /// Objects (solids, faces, sketches) are left out, they are the input, not a value.
        /// </summary>
        public static string ParameterValues(IEnumerable<string> names, Func<string, object?> value)
        {
            using MemoryStream stream = new MemoryStream();
            using (Utf8JsonWriter writer = new Utf8JsonWriter(stream))
            {
                writer.WriteStartObject();
                foreach (string name in names)
                {
                    switch (value(name))
                    {
                        case double d: writer.WriteNumber(name, d); break;
                        case int i: writer.WriteNumber(name, i); break;
                        case bool b: writer.WriteBoolean(name, b); break;
                        case string s: writer.WriteString(name, s); break;
                        case GeoPoint p:
                            writer.WriteStartArray(name);
                            writer.WriteNumberValue(p.x);
                            writer.WriteNumberValue(p.y);
                            writer.WriteNumberValue(p.z);
                            writer.WriteEndArray();
                            break;
                    }
                }
                writer.WriteEndObject();
            }
            return Encoding.UTF8.GetString(stream.ToArray());
        }
    }
}
