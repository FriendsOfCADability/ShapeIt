using System;
using System.IO;
using System.Xml;

namespace ShapeIt
{
    /// <summary>
    /// The 'icon' of a template: SVG text shown in the template list and on the template toolbar. Templates may come
    /// from other people as files, so an icon is only accepted when it is plain graphics: well-formed SVG without
    /// a DTD, without scripts, event handlers, embedded documents or images, and without references to anything
    /// outside the icon itself.
    /// </summary>
    public static class TemplateIcon
    {
        /// <summary>Upper limit of the SVG text in characters: an icon is a few shapes, not a picture.</summary>
        public const int MaxLength = 16384;

        private const string SvgNamespace = "http://www.w3.org/2000/svg";

        // elements that run code, embed other documents or load resources
        private static readonly string[] forbiddenElements = { "script", "foreignObject", "image", "iframe", "object", "embed", "audio", "video", "style", "feImage" };

        /// <summary>
        /// Plain SVG for a template without an icon of its own: a frame with a bracket pair, "parametric part".
        /// </summary>
        public const string Generic =
            "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 64 64\">"
            + "<rect x=\"6\" y=\"6\" width=\"52\" height=\"52\" rx=\"8\" fill=\"#e8eef6\" stroke=\"#494c4e\" stroke-width=\"4\"/>"
            + "<path d=\"M26 18h-6v28h6M38 18h6v28h-6\" fill=\"none\" stroke=\"#2f6db5\" stroke-width=\"5\" stroke-linejoin=\"round\"/>"
            + "</svg>";

        /// <summary>Why the SVG text cannot be a template icon, null when it can.</summary>
        public static string? Problem(string svg)
        {
            if (string.IsNullOrWhiteSpace(svg)) return "The icon is empty.";
            if (svg.Length > MaxLength) return $"The icon has {svg.Length} characters, at most {MaxLength} are allowed.";
            XmlReaderSettings settings = new XmlReaderSettings
            {
                DtdProcessing = DtdProcessing.Prohibit,
                XmlResolver = null,
                IgnoreComments = true,
                IgnoreProcessingInstructions = true,
            };
            try
            {
                using XmlReader reader = XmlReader.Create(new StringReader(svg), settings);
                bool root = true;
                while (reader.Read())
                {
                    if (reader.NodeType != XmlNodeType.Element) continue;
                    if (root)
                    {
                        if (reader.LocalName != "svg" || reader.NamespaceURI != SvgNamespace)
                            return $"The icon must be an <svg> element in the namespace {SvgNamespace}.";
                        root = false;
                    }
                    foreach (string forbidden in forbiddenElements)
                    {
                        if (string.Equals(reader.LocalName, forbidden, StringComparison.OrdinalIgnoreCase))
                            return $"The icon must not contain <{reader.LocalName}>.";
                    }
                    if (!reader.MoveToFirstAttribute()) continue;
                    do
                    {
                        string name = reader.LocalName, value = reader.Value;
                        if (name.StartsWith("on", StringComparison.OrdinalIgnoreCase))
                            return $"The icon must not contain the event handler '{reader.Name}'.";
                        if (name == "href" && !value.TrimStart().StartsWith("#"))
                            return $"The icon may only refer to its own elements ('#id'), not to '{value}'.";
                        if (ReferencesOutside(value))
                            return $"The icon may only refer to its own elements (url(#id)), not in '{reader.Name}'.";
                    } while (reader.MoveToNextAttribute());
                    reader.MoveToElement();
                }
                if (root) return "The icon contains no <svg> element.";
            }
            catch (XmlException ex)
            {
                return "The icon is no well-formed SVG: " + ex.Message;
            }
            return null;
        }

        /// <summary>Whether an attribute value contains a url(...) that is not a reference to an element of the icon.</summary>
        private static bool ReferencesOutside(string value)
        {
            int i = 0;
            while ((i = value.IndexOf("url(", i, StringComparison.OrdinalIgnoreCase)) >= 0)
            {
                i += 4;
                string rest = value.Substring(i).TrimStart(' ', '\'', '"');
                if (!rest.StartsWith("#")) return true;
            }
            return false;
        }
    }
}
