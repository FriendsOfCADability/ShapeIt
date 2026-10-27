using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using CADability.GeoObject;
using ShapeIt;
using Path = System.IO.Path;

namespace CADability.Tests
{
    /// <summary>
    /// Translations of templates: the texts of template.begin are English, a "translations" part of the template
    /// file or a separate translation file replaces what the user sees. Names, values and expressions stay.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class TemplateTranslationsTests
    {
        private static string ShippedTemplates([System.Runtime.CompilerServices.CallerFilePath] string thisFile = "")
            => Path.Combine(Path.GetDirectoryName(thisFile)!, "..", "..", "..", "ShapeIt", "Templates");

        private const string Begin = @"{""name"":""t"",""label"":""Hole"",""description"":""A hole."",""category"":""Holes"",
            ""parameters"":[
              {""name"":""d"",""value"":8,""label"":""Diameter"",""input"":{""kind"":""length"",""group"":""Hole"",""description"":""Diameter of the hole.""}},
              {""name"":""entry"",""value"":""sharp"",""label"":""Entry"",""input"":{""kind"":""enum"",""options"":[{""value"":""sharp"",""label"":""sharp""},""round"",2]}}],
            ""presets"":[{""name"":""series"",""label"":""Series"",""options"":[{""label"":""0.5"",""values"":{""d"":0.5}},{""label"":""1"",""values"":{""d"":1}}]}],
            ""checks"":[{""condition"":""d < 20"",""message"":""The diameter ({d}) is too large."",""parameters"":[""d""]}]}";

        private const string German = @"{""label"":""Bohrung"",""description"":""Eine Bohrung."",""category"":""Bohrungen"",
            ""parameters"":{""d"":{""label"":""Durchmesser"",""group"":""Bohrung"",""description"":""Durchmesser der Bohrung.""},
                          ""entry"":{""label"":""Eintritt"",""options"":{""sharp"":""scharfkantig"",""round"":""gerundet"",""2"":""zwei""}}},
            ""presets"":{""series"":{""label"":""Reihe"",""options"":{""0.5"":""0,5""}}},
            ""checks"":[""Der Durchmesser ({d}) ist zu groß.""]}";

        private static JsonElement Json(string text) => JsonDocument.Parse(text).RootElement.Clone();

        [TestMethod]
        public void LocalizeReplacesTheTextsOnly()
        {
            JsonElement begin = Json(Begin);
            JsonElement localized = TemplateTranslations.Localize(begin, Json(German));
            Assert.AreEqual("Bohrung", localized.GetProperty("label").GetString());
            Assert.AreEqual("Eine Bohrung.", localized.GetProperty("description").GetString());
            Assert.AreEqual("Bohrungen", localized.GetProperty("category").GetString());
            JsonElement d = localized.GetProperty("parameters")[0];
            Assert.AreEqual("Durchmesser", d.GetProperty("label").GetString());
            Assert.AreEqual("Bohrung", d.GetProperty("input").GetProperty("group").GetString());
            Assert.AreEqual("Durchmesser der Bohrung.", d.GetProperty("input").GetProperty("description").GetString());
            Assert.AreEqual(8, d.GetProperty("value").GetDouble(), "values stay");
            // options keep their values, a bare option becomes {value, label}
            List<TemplateEnumOption> options = TemplateParameterRules.ReadEnumOptions(localized.GetProperty("parameters")[1].GetProperty("input"));
            CollectionAssert.AreEqual(new object[] { "sharp", "round", 2.0 }, options.Select(o => o.Value).ToArray());
            CollectionAssert.AreEqual(new[] { "scharfkantig", "gerundet", "zwei" }, options.Select(o => o.Label).ToArray());
            List<TemplatePreset> presets = TemplateParameterRules.ReadPresets(localized.GetProperty("presets"));
            Assert.AreEqual("Reihe", presets[0].Label);
            CollectionAssert.AreEqual(new[] { "0,5", "1" }, presets[0].Options.Select(o => o.Label).ToArray(), "an option without translation keeps its label");
            // the localized rules give the localized message, with the placeholder evaluated
            List<ParameterProblem> problems = TemplateParameterRules.Check(localized.GetProperty("parameters"), localized.GetProperty("checks"),
                new Dictionary<string, object> { ["d"] = 30.0, ["entry"] = "sharp" });
            Assert.AreEqual("Der Durchmesser (30) ist zu groß.", problems.Single().Message);
            // the base is untouched
            Assert.AreEqual("Hole", begin.GetProperty("label").GetString());
            Assert.AreEqual(0, TemplateTranslations.Problems("t", begin, Json(German)).Count);
        }

        [TestMethod]
        public void ProblemsNameWhatTheTemplateDoesNotHave()
        {
            string wrong = @"{""parameters"":{""diameter"":{""label"":""x""},""entry"":{""options"":{""chamfer"":""angefast""}}},
                ""presets"":{""series"":{""options"":{""0.7"":""0,7""}},""other"":{}},
                ""checks"":[""Der Durchmesser ist zu groß."",""zweite""]}";
            List<string> problems = TemplateTranslations.Problems("t", Json(Begin), Json(wrong));
            string all = string.Join("\n", problems);
            StringAssert.Contains(all, "no parameter 'diameter'");
            StringAssert.Contains(all, "no option 'chamfer'");
            StringAssert.Contains(all, "preset 'series' has no option '0.7'");
            StringAssert.Contains(all, "no preset 'other'");
            StringAssert.Contains(all, "check 1 has other placeholders");
            StringAssert.Contains(all, "2 check messages, but only 1 checks");
        }

        [TestMethod]
        public void LanguageCodesOfTheApplication()
        {
            Assert.AreEqual("de", TemplateTranslations.LanguageCode("deutsch"));
            Assert.AreEqual("en", TemplateTranslations.LanguageCode("english"));
            Assert.AreEqual("en", TemplateTranslations.LanguageCode(null));
            Assert.AreEqual("fr", TemplateTranslations.LanguageCode("fr"));
        }

        /// <summary>
        /// Every shipped template file: the base texts are what the server reports, the German translation is
        /// complete for what the user sees and names nothing the templates do not have.
        /// </summary>
        [TestMethod]
        public void ShippedTranslationsAreComplete()
        {
            Dictionary<string, List<JsonElement>> templates = new();
            TemplateTranslations translations = new TemplateTranslations();
            List<TemplateFile> files = new List<TemplateFile>();
            List<string> loadProblems = TemplateLibrary.LoadDirectories(new[] { ShippedTemplates() }, templates, files, translations);
            Assert.AreEqual(0, loadProblems.Count, string.Join("\n", loadProblems));
            Assert.IsTrue(templates.Count >= 5);
            foreach (TemplateFile file in files) Assert.IsNotNull(translations.FileLabel(file.Id, "de"), file.Id);
            List<string> problems = new List<string>();
            foreach (KeyValuePair<string, List<JsonElement>> template in templates)
            {
                JsonElement begin = template.Value[0].GetProperty("params");
                JsonElement de = translations.ForTemplate(template.Key, "de");
                if (de.ValueKind != JsonValueKind.Object) { problems.Add($"{template.Key}: no German translation."); continue; }
                problems.AddRange(TemplateTranslations.Problems(template.Key, begin, de));
                // complete: every text shown has a German counterpart
                if (!de.TryGetProperty("label", out _)) problems.Add($"{template.Key}: no label.");
                if (begin.TryGetProperty("description", out _) && !de.TryGetProperty("description", out _)) problems.Add($"{template.Key}: no description.");
                foreach (JsonElement parameter in begin.GetProperty("parameters").EnumerateArray())
                {
                    string name = parameter.GetProperty("name").GetString()!;
                    JsonElement input = parameter.TryGetProperty("input", out JsonElement i) ? i : default;
                    if (!de.GetProperty("parameters").TryGetProperty(name, out JsonElement pt)) { problems.Add($"{template.Key}: parameter {name} untranslated."); continue; }
                    foreach (string text in new[] { "label", "description", "group" })
                    {
                        bool inBase = text == "label" ? parameter.TryGetProperty(text, out _) : input.ValueKind == JsonValueKind.Object && input.TryGetProperty(text, out _);
                        if (inBase && !pt.TryGetProperty(text, out _)) problems.Add($"{template.Key}: {name}.{text} untranslated.");
                    }
                    foreach (TemplateEnumOption option in TemplateParameterRules.ReadEnumOptions(input))
                    {
                        if (!pt.TryGetProperty("options", out JsonElement o) || !o.TryGetProperty(Convert.ToString(option.Value, System.Globalization.CultureInfo.InvariantCulture)!, out _))
                            problems.Add($"{template.Key}: option {option.Value} of {name} untranslated.");
                    }
                }
                if (begin.TryGetProperty("checks", out JsonElement checks) && checks.GetArrayLength() > 0
                    && (!de.TryGetProperty("checks", out JsonElement messages) || messages.GetArrayLength() != checks.GetArrayLength()))
                    problems.Add($"{template.Key}: check messages untranslated.");
            }
            Assert.AreEqual(0, problems.Count, string.Join("\n", problems));
        }

        /// <summary>The German check of the spur gear, as the application shows it.</summary>
        [TestMethod]
        public void GermanMessageOfTheSpurGear()
        {
            Dictionary<string, List<JsonElement>> templates = new();
            TemplateTranslations translations = new TemplateTranslations();
            TemplateLibrary.LoadDirectories(new[] { ShippedTemplates() }, templates, null, translations);
            JsonElement localized = TemplateTranslations.Localize(templates["stirnrad"][0].GetProperty("params"), translations.ForTemplate("stirnrad", "de"));
            Dictionary<string, object> values = new()
            {
                ["center"] = GeoPoint.Origin, ["m"] = 2.0, ["z"] = 10.0, ["b"] = 20.0, ["x"] = 0.0, ["alpha"] = 20.0,
                ["haP"] = 1.0, ["hfP"] = 1.25, ["Asn"] = 0.0, ["dBore"] = 0.0
            };
            List<ParameterProblem> problems = TemplateParameterRules.Check(localized.GetProperty("parameters"), localized.GetProperty("checks"), values);
            StringAssert.Contains(problems.Single().Message, "Unterschnitt: Bei z = 10 und α = 20° braucht das Rad eine Profilverschiebung x ≥ 0.4151 oder mindestens 18 Zähne.");
            Assert.AreEqual("Zahnräder", translations.FileLabel("StirnradTemplate", "de"));
        }

        /// <summary>A translation file adds a language without touching the template file; its file label belongs to
        /// the template file named by the part of its name before the first dot.</summary>
        [TestMethod]
        public void TranslationFileAddsALanguage()
        {
            string folder = Path.Combine(Path.GetTempPath(), "ShapeItTranslationTests_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(folder);
            try
            {
                File.WriteAllText(Path.Combine(folder, "Holes.json"),
                    "{\"label\":\"Holes\",\"templates\":[{\"name\":\"t\",\"items\":[{\"method\":\"template.begin\",\"id\":1,\"params\":" + Begin + "}]}],"
                    + "\"translations\":{\"de\":{\"label\":\"Bohrungen\",\"templates\":{\"t\":" + German + "}}}}");
                File.WriteAllText(Path.Combine(folder, "Holes.fr.json"),
                    "{\"translations\":{\"fr\":{\"label\":\"Perçages\",\"templates\":{\"t\":{\"label\":\"Perçage\",\"parameters\":{\"d\":{\"label\":\"Diamètre\"}}}}}}}");
                Dictionary<string, List<JsonElement>> templates = new();
                TemplateTranslations translations = new TemplateTranslations();
                List<TemplateFile> files = new List<TemplateFile>();
                List<string> problems = TemplateLibrary.LoadDirectories(new[] { folder }, templates, files, translations);
                Assert.AreEqual(0, problems.Count, string.Join("\n", problems));
                Assert.AreEqual(1, templates.Count);
                Assert.AreEqual(1, files.Count, "a translation file is no group of templates");
                Assert.AreEqual("Perçages", translations.FileLabel("Holes", "fr"));
                Assert.AreEqual("Bohrungen", translations.FileLabel("Holes", "de"));
                CollectionAssert.AreEquivalent(new[] { "de", "fr" }, translations.LanguagesOf("t").ToArray());
                JsonElement french = TemplateTranslations.Localize(templates["t"][0].GetProperty("params"), translations.ForTemplate("t", "fr"));
                Assert.AreEqual("Perçage", french.GetProperty("label").GetString());
                Assert.AreEqual("Diamètre", french.GetProperty("parameters")[0].GetProperty("label").GetString());
                Assert.AreEqual("Diameter of the hole.", french.GetProperty("parameters")[0].GetProperty("input").GetProperty("description").GetString(), "untranslated texts stay English");
                // the toolbar shows the file label in the language asked for
                List<TemplateToolbarEntry> entries = TemplateLibrary.ToolbarEntries(templates.Keys, files, n => n, f => translations.FileLabel(f.Id, "fr") ?? f.Label);
                Assert.AreEqual("t", entries.Single().Label, "a single template stands alone");
            }
            finally { Directory.Delete(folder, true); }
        }
    }
}
