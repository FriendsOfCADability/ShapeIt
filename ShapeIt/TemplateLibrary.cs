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
    /// What a template file brought: its id (the file name without extension), the label and icon of the group
    /// (the optional "label" and "icon" of the file, see <see cref="TemplateLibrary.ReadFile"/>) and the names of
    /// its templates.
    /// </summary>
    public sealed record TemplateFile(string Id, string Label, string? Icon, IReadOnlyList<string> Names);

    /// <summary>
    /// One button of the template toolbar: a single template, or a group of templates from one file behind a
    /// drop-down button (<see cref="GroupId"/> not null) when the file holds more than one of the current templates.
    /// </summary>
    public sealed record TemplateToolbarEntry(string Label, string? GroupId, string? GroupIcon, IReadOnlyList<string> Templates);

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
        /// Reads the templates of a template file into the dictionary, replacing templates of the same name. The file
        /// is {"label": ..., "icon": ..., "templates": [{"name": ..., "items": [...]}, ...]}; "label" and "icon"
        /// (SVG text, see <see cref="TemplateIcon"/>) are optional and describe the drop-down button of the file's
        /// templates on the toolbar. Without a label the file name is used; an icon that is not accepted is ignored.
        /// </summary>
        /// <exception cref="Exception">the file cannot be read or is no template file</exception>
        public static TemplateFile ReadFile(string fileName, Dictionary<string, List<JsonElement>> templates)
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
            string id = Path.GetFileNameWithoutExtension(fileName);
            string label = document.RootElement.TryGetProperty("label", out JsonElement l) && l.ValueKind == JsonValueKind.String ? l.GetString()! : id;
            string? icon = document.RootElement.TryGetProperty("icon", out JsonElement i) && i.ValueKind == JsonValueKind.String
                && TemplateIcon.Problem(i.GetString()!) == null ? i.GetString() : null;
            return new TemplateFile(id, label, icon, names);
        }

        /// <summary>
        /// Loads the *.json files of the given folders in this order (a later one replaces templates of the same
        /// name), each folder in the order of the file names. A folder that does not exist is skipped; a file that
        /// cannot be read does not stop the others.
        /// </summary>
        /// <param name="read">receives the files read, in this order (they group the templates on the toolbar)</param>
        /// <returns>one message per file that could not be read</returns>
        public static List<string> LoadDirectories(IEnumerable<string> directories, Dictionary<string, List<JsonElement>> templates, List<TemplateFile>? read = null)
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
                    try
                    {
                        TemplateFile templateFile = ReadFile(file, templates);
                        read?.Add(templateFile);
                    }
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
        public static List<string> LoadAll(Dictionary<string, List<JsonElement>> templates, List<TemplateFile>? read = null)
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
            problems.AddRange(LoadDirectories(new[] { ShippedDirectory, userDirectory }, templates, read));
            return problems;
        }

        /// <summary>
        /// The buttons of the template toolbar. A template belongs to the last file read that contains it (a user file
        /// takes it over from a shipped one), files of the same id form one group with the label and icon of the last
        /// of them. A group with at least two current templates becomes a drop-down button, the others stand alone;
        /// so do templates that come from no file (recorded in this session). Buttons and the templates of a group are
        /// ordered by label.
        /// </summary>
        /// <param name="templateNames">the current templates</param>
        /// <param name="files">the template files in the order they were read</param>
        /// <param name="label">the label of a template</param>
        public static List<TemplateToolbarEntry> ToolbarEntries(IEnumerable<string> templateNames, IEnumerable<TemplateFile> files, Func<string, string> label)
        {
            Dictionary<string, TemplateFile> owner = new Dictionary<string, TemplateFile>();
            Dictionary<string, TemplateFile> lastOfId = new Dictionary<string, TemplateFile>();
            foreach (TemplateFile file in files)
            {
                foreach (string name in file.Names) owner[name] = file;
                lastOfId[file.Id] = file;
            }
            StringComparer byLabel = StringComparer.CurrentCultureIgnoreCase;
            List<TemplateToolbarEntry> entries = new List<TemplateToolbarEntry>();
            foreach (var group in templateNames.Distinct().GroupBy(name => owner.TryGetValue(name, out TemplateFile? file) ? file.Id : null))
            {
                List<string> members = group.OrderBy(label, byLabel).ToList();
                if (group.Key != null && members.Count > 1)
                {
                    TemplateFile file = lastOfId[group.Key];
                    entries.Add(new TemplateToolbarEntry(file.Label, GroupMenuId(group.Key), file.Icon, members));
                }
                else
                {
                    foreach (string name in members) entries.Add(new TemplateToolbarEntry(label(name), null, null, new[] { name }));
                }
            }
            return entries.OrderBy(e => e.Label, byLabel).ToList();
        }

        /// <summary>
        /// The menu id of the drop-down button of a template file. Characters other than letters, digits, '_', '-'
        /// and '.' are replaced: the id ends up in XPath queries of the menu resource.
        /// </summary>
        public static string GroupMenuId(string fileId) => "RpcTemplate.Group." + System.Text.RegularExpressions.Regex.Replace(fileId, @"[^A-Za-z0-9_.\-]", "_");
    }
}
