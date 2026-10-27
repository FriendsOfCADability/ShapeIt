using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Nodes;

namespace ShapeIt
{
    /// <summary>
    /// The texts of templates in other languages. The texts of a template itself (in template.begin) are the base
    /// language, English: they are what MCP clients read and what the construction is recorded with. A translation
    /// only replaces texts shown to the user and never touches names, values or expressions, so the construction
    /// exists once for all languages.
    /// <para>
    /// A template file may hold its translations next to the templates, and a translation file may add a language
    /// later without changing the template file:
    /// <code>
    /// "translations": {
    ///   "de": {
    ///     "label": "Bohrungen",                      // the label of the file (its group on the toolbar)
    ///     "templates": {
    ///       "sacklochbohrung": {
    ///         "label": "...", "description": "...", "category": "...",
    ///         "parameters": { "d": { "label": "...", "description": "...", "group": "...",
    ///                                "options": { "chamfer": "angefast" } } },   // enum options by value
    ///         "presets": { "moduleSeries1": { "label": "...", "description": "...",
    ///                                         "options": { "0.5": "0,5" } } },   // preset options by base label
    ///         "checks": [ "...", "..." ]             // the messages of the checks, in their order
    ///       } } } }
    /// </code>
    /// A translation file has "translations" but no "templates"; its file label belongs to the template file whose
    /// name is the part of its own name before the first dot (BohrungTemplate.fr.json translates BohrungTemplate.json).
    /// Whatever a translation leaves out is shown in the base language.
    /// </para>
    /// </summary>
    public sealed class TemplateTranslations
    {
        // template name -> language -> translation of the template
        private readonly Dictionary<string, Dictionary<string, JsonElement>> templates = new Dictionary<string, Dictionary<string, JsonElement>>();
        // file id -> language -> label of the file
        private readonly Dictionary<string, Dictionary<string, string>> fileLabels = new Dictionary<string, Dictionary<string, string>>();

        /// <summary>
        /// The language code templates use for the language of the application: CADability names its languages
        /// ("deutsch", "english"), template files use ISO 639-1 codes ("de", "en").
        /// </summary>
        public static string LanguageCode(string? applicationLanguage)
        {
            switch (applicationLanguage?.ToLowerInvariant())
            {
                case null: case "": return "en";
                case "deutsch": case "german": return "de";
                case "english": return "en";
                case "français": case "francais": case "french": return "fr";
                case "italiano": case "italian": return "it";
                case "español": case "espanol": case "spanish": return "es";
                default: return applicationLanguage.ToLowerInvariant();
            }
        }

        /// <summary>The language code of the active language of the application.</summary>
        public static string CurrentLanguage => LanguageCode(StringTable.ActiveLanguage);

        /// <summary>
        /// Takes the "translations" of a template file or a translation file. A later file replaces the translation of
        /// a template it translates too, the way a later template file replaces a template of the same name.
        /// </summary>
        /// <param name="translations">the "translations" object: language code -> translations of that language</param>
        /// <param name="fileId">the template file the file label belongs to</param>
        public void Add(JsonElement translations, string fileId)
        {
            if (translations.ValueKind != JsonValueKind.Object) return;
            foreach (JsonProperty language in translations.EnumerateObject())
            {
                if (language.Value.ValueKind != JsonValueKind.Object) continue;
                string code = language.Name.ToLowerInvariant();
                if (language.Value.TryGetProperty("label", out JsonElement label) && label.ValueKind == JsonValueKind.String)
                {
                    if (!fileLabels.TryGetValue(fileId, out Dictionary<string, string>? labels)) fileLabels[fileId] = labels = new Dictionary<string, string>();
                    labels[code] = label.GetString()!;
                }
                if (language.Value.TryGetProperty("templates", out JsonElement list) && list.ValueKind == JsonValueKind.Object)
                {
                    foreach (JsonProperty template in list.EnumerateObject())
                    {
                        if (template.Value.ValueKind != JsonValueKind.Object) continue;
                        if (!templates.TryGetValue(template.Name, out Dictionary<string, JsonElement>? languages)) templates[template.Name] = languages = new Dictionary<string, JsonElement>();
                        languages[code] = template.Value.Clone();
                    }
                }
            }
        }

        /// <summary>The translation of a template into a language, undefined when there is none.</summary>
        public JsonElement ForTemplate(string templateName, string language)
        {
            if (templates.TryGetValue(templateName, out Dictionary<string, JsonElement>? languages) && languages.TryGetValue(language, out JsonElement translation))
                return translation;
            return default;
        }

        /// <summary>The label of a template file in a language, null when there is none.</summary>
        public string? FileLabel(string fileId, string language)
        {
            return fileLabels.TryGetValue(fileId, out Dictionary<string, string>? labels) && labels.TryGetValue(language, out string? label) ? label : null;
        }

        /// <summary>The languages a template is translated into.</summary>
        public IEnumerable<string> LanguagesOf(string templateName)
        {
            return templates.TryGetValue(templateName, out Dictionary<string, JsonElement>? languages) ? languages.Keys : Array.Empty<string>();
        }

        /// <summary>The key of an enum option in a translation: a string option is its text, other values their JSON text.</summary>
        public static string OptionKey(JsonElement value)
        {
            return value.ValueKind == JsonValueKind.String ? value.GetString()! : value.GetRawText();
        }

        /// <summary>
        /// The 'params' of a template.begin with the texts of the translation: labels, descriptions, groups and the
        /// category of the template, the labels of enum options and presets, and the messages of the checks. Without
        /// a translation the params are returned as they are.
        /// </summary>
        public static JsonElement Localize(JsonElement beginParams, JsonElement translation)
        {
            if (beginParams.ValueKind != JsonValueKind.Object || translation.ValueKind != JsonValueKind.Object) return beginParams;
            JsonObject result = JsonNode.Parse(beginParams.GetRawText())!.AsObject();
            foreach (string property in new[] { "label", "description", "category" }) SetText(result, property, translation, property);
            if (translation.TryGetProperty("parameters", out JsonElement parameterTranslations) && parameterTranslations.ValueKind == JsonValueKind.Object
                && result["parameters"] is JsonArray parameters)
            {
                foreach (JsonNode? parameter in parameters)
                {
                    if (parameter is not JsonObject p || p["name"]?.GetValueKind() != JsonValueKind.String) continue;
                    if (!parameterTranslations.TryGetProperty(p["name"]!.GetValue<string>(), out JsonElement pt) || pt.ValueKind != JsonValueKind.Object) continue;
                    SetText(p, "label", pt, "label");
                    if (p["input"] is not JsonObject input) continue;
                    SetText(input, "description", pt, "description");
                    SetText(input, "group", pt, "group");
                    if (pt.TryGetProperty("options", out JsonElement optionLabels) && optionLabels.ValueKind == JsonValueKind.Object && input["options"] is JsonArray options)
                    {
                        for (int i = 0; i < options.Count; i++)
                        {   // an option given as its bare value becomes {value, label}
                            JsonNode? option = options[i];
                            JsonNode? value = option is JsonObject o ? o["value"] : option;
                            if (value == null) continue;
                            string key = OptionKey(JsonDocument.Parse(value.ToJsonString()).RootElement);
                            if (!optionLabels.TryGetProperty(key, out JsonElement label) || label.ValueKind != JsonValueKind.String) continue;
                            if (option is JsonObject optionObject) optionObject["label"] = label.GetString();
                            else options[i] = new JsonObject { ["value"] = value.DeepClone(), ["label"] = label.GetString() };
                        }
                    }
                }
            }
            if (translation.TryGetProperty("presets", out JsonElement presetTranslations) && presetTranslations.ValueKind == JsonValueKind.Object
                && result["presets"] is JsonArray presets)
            {
                foreach (JsonNode? preset in presets)
                {
                    if (preset is not JsonObject p || p["name"]?.GetValueKind() != JsonValueKind.String) continue;
                    if (!presetTranslations.TryGetProperty(p["name"]!.GetValue<string>(), out JsonElement pt) || pt.ValueKind != JsonValueKind.Object) continue;
                    SetText(p, "label", pt, "label");
                    SetText(p, "description", pt, "description");
                    if (pt.TryGetProperty("options", out JsonElement optionLabels) && optionLabels.ValueKind == JsonValueKind.Object && p["options"] is JsonArray options)
                    {
                        foreach (JsonNode? option in options)
                        {
                            if (option is not JsonObject o || o["label"]?.GetValueKind() != JsonValueKind.String) continue;
                            if (optionLabels.TryGetProperty(o["label"]!.GetValue<string>(), out JsonElement label) && label.ValueKind == JsonValueKind.String)
                                o["label"] = label.GetString();
                        }
                    }
                }
            }
            if (translation.TryGetProperty("checks", out JsonElement messages) && messages.ValueKind == JsonValueKind.Array && result["checks"] is JsonArray checks)
            {
                int i = 0;
                foreach (JsonElement message in messages.EnumerateArray())
                {
                    if (i >= checks.Count) break;
                    if (message.ValueKind == JsonValueKind.String && checks[i] is JsonObject check) check["message"] = message.GetString();
                    i++;
                }
            }
            return JsonDocument.Parse(result.ToJsonString()).RootElement.Clone();
        }

        private static void SetText(JsonObject target, string property, JsonElement translation, string translationProperty)
        {
            if (translation.TryGetProperty(translationProperty, out JsonElement text) && text.ValueKind == JsonValueKind.String) target[property] = text.GetString();
        }

        /// <summary>
        /// What a translation names that the template does not have: parameters, enum options, presets and their
        /// options, more checks than there are. Such a text would never be shown, typically because the template
        /// changed after it was translated. Also reports a check message whose {placeholders} differ from the base
        /// message: the translation would show other values.
        /// </summary>
        public static List<string> Problems(string templateName, JsonElement beginParams, JsonElement translation)
        {
            List<string> problems = new List<string>();
            if (translation.ValueKind != JsonValueKind.Object) return problems;
            Dictionary<string, JsonElement> parameters = new Dictionary<string, JsonElement>();
            if (beginParams.TryGetProperty("parameters", out JsonElement ps) && ps.ValueKind == JsonValueKind.Array)
            {
                foreach (JsonElement p in ps.EnumerateArray())
                {
                    if (p.TryGetProperty("name", out JsonElement n) && n.ValueKind == JsonValueKind.String) parameters[n.GetString()!] = p;
                }
            }
            if (translation.TryGetProperty("parameters", out JsonElement pts) && pts.ValueKind == JsonValueKind.Object)
            {
                foreach (JsonProperty pt in pts.EnumerateObject())
                {
                    if (!parameters.TryGetValue(pt.Name, out JsonElement p)) { problems.Add($"{templateName}: no parameter '{pt.Name}'."); continue; }
                    if (!pt.Value.TryGetProperty("options", out JsonElement optionLabels) || optionLabels.ValueKind != JsonValueKind.Object) continue;
                    HashSet<string> keys = new HashSet<string>();
                    if (p.TryGetProperty("input", out JsonElement input) && input.TryGetProperty("options", out JsonElement options) && options.ValueKind == JsonValueKind.Array)
                    {
                        foreach (JsonElement option in options.EnumerateArray())
                        {
                            JsonElement value = option.ValueKind == JsonValueKind.Object && option.TryGetProperty("value", out JsonElement v) ? v : option;
                            keys.Add(OptionKey(value));
                        }
                    }
                    foreach (JsonProperty optionLabel in optionLabels.EnumerateObject())
                    {
                        if (!keys.Contains(optionLabel.Name)) problems.Add($"{templateName}: parameter '{pt.Name}' has no option '{optionLabel.Name}'.");
                    }
                }
            }
            Dictionary<string, JsonElement> presets = new Dictionary<string, JsonElement>();
            if (beginParams.TryGetProperty("presets", out JsonElement prs) && prs.ValueKind == JsonValueKind.Array)
            {
                foreach (JsonElement p in prs.EnumerateArray())
                {
                    if (p.TryGetProperty("name", out JsonElement n) && n.ValueKind == JsonValueKind.String) presets[n.GetString()!] = p;
                }
            }
            if (translation.TryGetProperty("presets", out JsonElement prts) && prts.ValueKind == JsonValueKind.Object)
            {
                foreach (JsonProperty pt in prts.EnumerateObject())
                {
                    if (!presets.TryGetValue(pt.Name, out JsonElement p)) { problems.Add($"{templateName}: no preset '{pt.Name}'."); continue; }
                    if (!pt.Value.TryGetProperty("options", out JsonElement optionLabels) || optionLabels.ValueKind != JsonValueKind.Object) continue;
                    HashSet<string> labels = new HashSet<string>();
                    foreach (TemplatePresetOption option in TemplateParameterRules.ReadPresets(JsonDocument.Parse("[" + p.GetRawText() + "]").RootElement)[0].Options) labels.Add(option.Label);
                    foreach (JsonProperty optionLabel in optionLabels.EnumerateObject())
                    {
                        if (!labels.Contains(optionLabel.Name)) problems.Add($"{templateName}: preset '{pt.Name}' has no option '{optionLabel.Name}'.");
                    }
                }
            }
            if (translation.TryGetProperty("checks", out JsonElement messages) && messages.ValueKind == JsonValueKind.Array)
            {
                List<string> baseMessages = new List<string>();
                if (beginParams.TryGetProperty("checks", out JsonElement checks) && checks.ValueKind == JsonValueKind.Array)
                {
                    foreach (JsonElement check in checks.EnumerateArray())
                        baseMessages.Add(check.TryGetProperty("message", out JsonElement m) && m.ValueKind == JsonValueKind.String ? m.GetString()! : "");
                }
                int i = 0;
                foreach (JsonElement message in messages.EnumerateArray())
                {
                    if (i >= baseMessages.Count) { problems.Add($"{templateName}: {messages.GetArrayLength()} check messages, but only {baseMessages.Count} checks."); break; }
                    if (message.ValueKind == JsonValueKind.String)
                    {
                        List<string> wanted = Placeholders(baseMessages[i]), given = Placeholders(message.GetString()!);
                        wanted.Sort(StringComparer.Ordinal);
                        given.Sort(StringComparer.Ordinal);
                        if (!wanted.SequenceEqual(given, StringComparer.Ordinal)) problems.Add($"{templateName}: check {i + 1} has other placeholders than the base message.");
                    }
                    i++;
                }
            }
            return problems;
        }

        /// <summary>The {expressions} of a message, without the escaped braces "{{" and "}}".</summary>
        private static List<string> Placeholders(string message)
        {
            List<string> res = new List<string>();
            for (int i = 0; i < message.Length; i++)
            {
                if (message[i] == '{' && i + 1 < message.Length && message[i + 1] == '{') { i++; continue; }
                if (message[i] != '{') continue;
                int end = message.IndexOf('}', i + 1);
                if (end < 0) break;
                res.Add(message.Substring(i + 1, end - i - 1));
                i = end;
            }
            return res;
        }
    }
}
