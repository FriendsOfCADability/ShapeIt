using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using ShapeIt;

namespace CADability.Tests
{
    /// <summary>
    /// Loading template files at startup: shipped templates first, the user's after them, so that a user template
    /// replaces a shipped one of the same name; a broken file is reported without stopping the others.
    /// </summary>
    [TestClass]
    [TestCategory("RPC")]
    public class TemplateLibraryTests
    {
        private string root = "";

        [TestInitialize]
        public void CreateFolders()
        {
            root = Path.Combine(Path.GetTempPath(), "ShapeItTemplateLibraryTests_" + System.Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(Path.Combine(root, "shipped"));
            Directory.CreateDirectory(Path.Combine(root, "user"));
        }

        [TestCleanup]
        public void RemoveFolders()
        {
            if (Directory.Exists(root)) Directory.Delete(root, true);
        }

        /// <summary>A template file with templates that consist of a template.begin carrying the given label.</summary>
        private void WriteTemplates(string folder, string file, params (string name, string label)[] templates)
        {
            List<string> entries = new List<string>();
            foreach ((string name, string label) in templates)
                entries.Add($"{{\"name\":\"{name}\",\"items\":[{{\"method\":\"template.begin\",\"id\":1,\"params\":{{\"name\":\"{name}\",\"label\":\"{label}\",\"parameters\":[]}}}}]}}");
            File.WriteAllText(Path.Combine(root, folder, file), "{\"templates\":[" + string.Join(",", entries) + "]}");
        }

        private static string Label(List<JsonElement> items) => items[0].GetProperty("params").GetProperty("label").GetString()!;

        [TestMethod]
        public void UserTemplatesReplaceShippedOnesOfTheSameName()
        {
            WriteTemplates("shipped", "a.json", ("gear", "shipped gear"), ("box", "shipped box"));
            WriteTemplates("user", "mine.json", ("gear", "my gear"), ("plate", "my plate"));
            Dictionary<string, List<JsonElement>> templates = new();
            List<string> problems = TemplateLibrary.LoadDirectories(new[] { Path.Combine(root, "shipped"), Path.Combine(root, "user") }, templates);
            Assert.AreEqual(0, problems.Count, string.Join("\n", problems));
            Assert.AreEqual(3, templates.Count);
            Assert.AreEqual("my gear", Label(templates["gear"]));
            Assert.AreEqual("shipped box", Label(templates["box"]));
            Assert.AreEqual("my plate", Label(templates["plate"]));
        }

        [TestMethod]
        public void BrokenFileIsReportedAndTheOthersAreLoaded()
        {
            WriteTemplates("user", "a.json", ("box", "box"));
            File.WriteAllText(Path.Combine(root, "user", "b.json"), "{ this is no json");
            File.WriteAllText(Path.Combine(root, "user", "c.json"), "{\"something\":1}");
            WriteTemplates("user", "d.json", ("plate", "plate"));
            Dictionary<string, List<JsonElement>> templates = new();
            List<string> problems = TemplateLibrary.LoadDirectories(new[] { Path.Combine(root, "user") }, templates);
            Assert.AreEqual(2, problems.Count, string.Join("\n", problems));
            StringAssert.Contains(problems[0], "b.json");
            StringAssert.Contains(problems[1], "c.json");
            CollectionAssert.AreEquivalent(new[] { "box", "plate" }, new List<string>(templates.Keys));
        }

        [TestMethod]
        public void MissingFolderIsSkipped()
        {
            Dictionary<string, List<JsonElement>> templates = new();
            Assert.AreEqual(0, TemplateLibrary.LoadDirectories(new[] { Path.Combine(root, "nowhere") }, templates).Count);
            Assert.AreEqual(0, templates.Count);
        }

        [TestMethod]
        public void ShippedFolderHoldsTheGear()
        {
            // the source of the folder "Templates" next to the program
            string shipped = Path.Combine(Path.GetDirectoryName(ThisFile())!, "..", "..", "..", "ShapeIt", "Templates");
            Dictionary<string, List<JsonElement>> templates = new();
            Assert.AreEqual(0, TemplateLibrary.LoadDirectories(new[] { shipped }, templates).Count);
            Assert.IsTrue(templates.ContainsKey("stirnrad"));
        }

        [TestMethod]
        public void FileLabelAndIconDescribeTheGroup()
        {
            string icon = TemplateIcon.Generic.Replace("\"", "\\\"");
            File.WriteAllText(Path.Combine(root, "user", "gears.json"),
                "{\"label\":\"Zahnräder\",\"icon\":\"" + icon + "\",\"templates\":[]}");
            File.WriteAllText(Path.Combine(root, "user", "plain.json"),
                "{\"icon\":\"<svg xmlns=\\\"http://www.w3.org/2000/svg\\\"><script/></svg>\",\"templates\":[]}");
            Dictionary<string, List<JsonElement>> templates = new();
            TemplateFile gears = TemplateLibrary.ReadFile(Path.Combine(root, "user", "gears.json"), templates);
            Assert.AreEqual("gears", gears.Id);
            Assert.AreEqual("Zahnräder", gears.Label);
            Assert.AreEqual(TemplateIcon.Generic, gears.Icon);
            TemplateFile plain = TemplateLibrary.ReadFile(Path.Combine(root, "user", "plain.json"), templates);
            Assert.AreEqual("plain", plain.Label, "without a label the file name");
            Assert.IsNull(plain.Icon, "an icon that is not accepted is ignored");
        }

        private static TemplateFile File_(string id, params string[] names) => new TemplateFile(id, "Group " + id, null, names);

        [TestMethod]
        public void FilesWithSeveralTemplatesBecomeDropDownButtons()
        {
            TemplateFile gears = File_("gears", "spur", "rack", "worm");
            TemplateFile single = File_("single", "box");
            // a user file takes "worm" over: the gear group keeps two templates
            TemplateFile mine = File_("mine", "worm");
            List<TemplateToolbarEntry> entries = TemplateLibrary.ToolbarEntries(
                new[] { "spur", "rack", "worm", "box", "recorded" }, new[] { gears, single, mine }, name => name);
            Assert.AreEqual(4, entries.Count, string.Join(", ", entries.ConvertAll(e => e.Label)));
            // ordered by label: box, Group gears, recorded, worm
            Assert.AreEqual("box", entries[0].Label);
            Assert.IsNull(entries[0].GroupId);
            Assert.AreEqual("Group gears", entries[1].Label);
            Assert.AreEqual("RpcTemplate.Group.gears", entries[1].GroupId);
            CollectionAssert.AreEqual(new[] { "rack", "spur" }, (System.Collections.ICollection)entries[1].Templates);
            Assert.AreEqual("recorded", entries[2].Label, "a template from no file stands alone");
            Assert.AreEqual("worm", entries[3].Label);
            Assert.IsNull(entries[3].GroupId, "a file with a single template needs no drop-down");
        }

        [TestMethod]
        public void GroupIdIsSafeForTheMenuResource()
        {
            Assert.AreEqual("RpcTemplate.Group.Zahnr_der_O_Brien", TemplateLibrary.GroupMenuId("Zahnräder O'Brien"));
        }

        private static string ThisFile([System.Runtime.CompilerServices.CallerFilePath] string thisFile = "") => thisFile;

        [TestMethod]
        public void DefaultUserFolderIsInTheDocuments()
        {
            string documents = System.Environment.GetFolderPath(System.Environment.SpecialFolder.MyDocuments);
            Assert.AreEqual(Path.Combine(documents, "ShapeIt", "Templates"), TemplateLibrary.DefaultUserDirectory);
        }
    }
}
