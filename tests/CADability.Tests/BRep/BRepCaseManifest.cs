using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;
using ShapeIt;

namespace CADability.Tests.BRep
{
    /// <summary>
    /// What the harness expects from a case. Without this classification the suite would be permanently red -
    /// most of these files exist precisely because the operation is broken - and a permanently red suite is
    /// one nobody looks at.
    /// </summary>
    public enum CaseStatus
    {
        /// <summary>Must run, must produce a valid result and must match the baseline.</summary>
        Ok,
        /// <summary>Known to be broken. Must still match the baseline; if it suddenly succeeds, the test says so
        /// (that is progress and the case should be promoted to <see cref="Ok"/>).</summary>
        KnownFail,
        /// <summary>The input geometry is already inconsistent - an import/authoring bug, not a BRep bug.
        /// Reported, never run, does not fail the build.</summary>
        CorruptInput,
        /// <summary>The file does not follow the marker convention yet. Reported, never run, does not fail the build.</summary>
        NeedsFixup,
        /// <summary>Deliberately excluded (too slow, duplicate, ...).</summary>
        Skip
    }

    public class CaseEntry
    {
        public string Name { get; set; } = "";
        [JsonConverter(typeof(JsonStringEnumConverter))]
        public CaseStatus Status { get; set; } = CaseStatus.Ok;
        /// <summary>Operation to use when the project file itself does not name one unambiguously.</summary>
        [JsonConverter(typeof(JsonStringEnumConverter))]
        public BRepOperationKind? Operation { get; set; }
        public double? Parameter { get; set; }
        public double? SecondaryParameter { get; set; }
        public int? TimeoutSeconds { get; set; }
        public double? RelativeTolerance { get; set; }
        /// <summary>Free text: what the bug is, which issue it belongs to.</summary>
        public string? Comment { get; set; }
    }

    public class CaseDefaults
    {
        public int TimeoutSeconds { get; set; } = 120;
        public double RelativeTolerance { get; set; } = 1e-6;
    }

    /// <summary>
    /// cases.json next to the .cdb.json files: the per-case expectations. Kept outside the project files so that
    /// the drawings stay untouched and the classification is visible in one place (and in the diff).
    /// </summary>
    public class BRepCaseManifest
    {
        public CaseDefaults Defaults { get; set; } = new CaseDefaults();
        public List<CaseEntry> Cases { get; set; } = new List<CaseEntry>();

        [JsonIgnore]
        private Dictionary<string, CaseEntry> byName = new Dictionary<string, CaseEntry>(StringComparer.OrdinalIgnoreCase);

        private static readonly JsonSerializerOptions Options = new JsonSerializerOptions
        {
            PropertyNameCaseInsensitive = true,
            ReadCommentHandling = JsonCommentHandling.Skip,
            AllowTrailingCommas = true,
            WriteIndented = true
        };

        public static BRepCaseManifest Load(string path)
        {
            BRepCaseManifest manifest;
            if (File.Exists(path)) manifest = JsonSerializer.Deserialize<BRepCaseManifest>(File.ReadAllText(path), Options) ?? new BRepCaseManifest();
            else manifest = new BRepCaseManifest();
            manifest.byName = new Dictionary<string, CaseEntry>(StringComparer.OrdinalIgnoreCase);
            foreach (CaseEntry entry in manifest.Cases) manifest.byName[entry.Name] = entry;
            return manifest;
        }

        public void Save(string path) => File.WriteAllText(path, JsonSerializer.Serialize(this, Options));

        /// <summary>An unlisted file defaults to <see cref="CaseStatus.Ok"/> - a new file has to prove itself.</summary>
        public CaseEntry Get(string caseName)
        {
            if (byName.TryGetValue(caseName, out CaseEntry? entry)) return entry;
            return new CaseEntry { Name = caseName, Status = CaseStatus.Ok };
        }

        public bool Contains(string caseName) => byName.ContainsKey(caseName);
    }
}
