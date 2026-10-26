using CADability;
using CADability.UserInterface;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;

namespace ShapeIt
{
    /// <summary>
    /// The template files ShapeIt loads at startup: first the templates shipped with the program (the folder
    /// "Templates" next to the executable), then those of the user, from the folder of the global setting
    /// "Templates.Directory" or, when it is empty, from Documents\ShapeIt\Templates. A user template replaces a
    /// shipped one of the same name, so a shipped template can be adapted without touching the installation.
    /// </summary>
    public static class TemplateLibrary
    {
        /// <summary>The global setting with the folder of the user's templates; empty means <see cref="DefaultUserDirectory"/>.</summary>
        public const string DirectorySetting = "Templates.Directory";

        /// <summary>Documents\ShapeIt\Templates: the user adds, edits and exchanges these files, so they belong where the user finds them.</summary>
        public static string DefaultUserDirectory
            => Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "ShapeIt", "Templates");

        /// <summary>The templates shipped with the program, read-only.</summary>
        public static string ShippedDirectory => Path.Combine(AppContext.BaseDirectory, "Templates");

        /// <summary>The folder of the user's templates: the setting (environment variables expanded) or the default.</summary>
        public static string UserDirectory
        {
            get
            {
                string configured = Settings.GlobalSettings.GetStringValue(DirectorySetting, "");
                return string.IsNullOrWhiteSpace(configured) ? DefaultUserDirectory : Environment.ExpandEnvironmentVariables(configured.Trim());
            }
        }

        /// <summary>
        /// Adds the setting to the global settings, as an editable text in the group "Templates", unless it exists.
        /// </summary>
        public static void EnsureSetting()
        {
            if (Settings.GlobalSettings.ContainsSetting("Templates")) return;
            Settings templates = new Settings("Setting.Templates");
            Settings.GlobalSettings.AddSetting("Templates", templates);
            templates.AddSetting("Directory", new StringProperty("", "Setting.Templates.Directory"));
        }

        /// <summary>
        /// Reads the templates of a template file ({"templates": [{"name": ..., "items": [...]}, ...]}) into the
        /// dictionary, replacing templates of the same name.
        /// </summary>
        /// <returns>the names of the templates read</returns>
        /// <exception cref="Exception">the file cannot be read or is no template file</exception>
        public static List<string> ReadFile(string fileName, Dictionary<string, List<JsonElement>> templates)
        {
            using FileStream stream = File.OpenRead(fileName);
            using JsonDocument document = JsonDocument.Parse(stream);
            if (!document.RootElement.TryGetProperty("templates", out JsonElement list) || list.ValueKind != JsonValueKind.Array)
                throw new InvalidDataException("No \"templates\" array.");
            List<string> names = new List<string>();
            foreach (JsonElement template in list.EnumerateArray())
            {
                if (!template.TryGetProperty("name", out JsonElement nameEl) || nameEl.ValueKind != JsonValueKind.String) continue;
                if (!template.TryGetProperty("items", out JsonElement items) || items.ValueKind != JsonValueKind.Array) continue;
                string? name = nameEl.GetString();
                if (string.IsNullOrEmpty(name)) continue;
                templates[name] = items.EnumerateArray().Select(item => item.Clone()).ToList();
                names.Add(name);
            }
            return names;
        }

        /// <summary>
        /// Loads the *.json files of the given folders in this order (a later one replaces templates of the same
        /// name), each folder in the order of the file names. A folder that does not exist is skipped; a file that
        /// cannot be read does not stop the others.
        /// </summary>
        /// <returns>one message per file that could not be read</returns>
        public static List<string> LoadDirectories(IEnumerable<string> directories, Dictionary<string, List<JsonElement>> templates)
        {
            List<string> problems = new List<string>();
            foreach (string directory in directories)
            {
                if (!Directory.Exists(directory)) continue;
                string[] files;
                try { files = Directory.GetFiles(directory, "*.json"); }
                catch (Exception ex) { problems.Add($"{directory}: {ex.Message}"); continue; }
                Array.Sort(files, StringComparer.OrdinalIgnoreCase);
                foreach (string file in files)
                {
                    try { ReadFile(file, templates); }
                    catch (Exception ex) { problems.Add($"{file}: {ex.Message}"); }
                }
            }
            return problems;
        }

        /// <summary>
        /// Loads the shipped and the user's templates. The default user folder is created, so that the user sees
        /// where templates go; a configured folder that does not exist is reported instead.
        /// </summary>
        /// <returns>one message per file or folder that could not be read</returns>
        public static List<string> LoadAll(Dictionary<string, List<JsonElement>> templates)
        {
            List<string> problems = new List<string>();
            string userDirectory = UserDirectory;
            if (!Directory.Exists(userDirectory))
            {
                if (userDirectory == DefaultUserDirectory)
                {
                    try { Directory.CreateDirectory(userDirectory); }
                    catch (Exception ex) { problems.Add($"{userDirectory}: {ex.Message}"); }
                }
                else problems.Add($"{userDirectory}: the folder of the setting {DirectorySetting} does not exist.");
            }
            problems.AddRange(LoadDirectories(new[] { ShippedDirectory, userDirectory }, templates));
            return problems;
        }
    }
}
