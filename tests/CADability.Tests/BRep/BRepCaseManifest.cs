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
        /// <summary>
        /// How often to run this case within one test run, default 1. Everything the operation and the summary
        /// do is supposed to be a function of the input file alone, so a second run has to produce the same
        /// fingerprint. Where it does not, the case is not a regression test but a coin toss, and comparing it
        /// against a baseline says nothing - which is why a repeat that disagrees fails the suite and also stops
        /// the case from being regenerated. Worth setting on the cases that have been seen to move.
        /// </summary>
        public int? Repeat { get; set; }
        /// <summary>Free text: what the bug is, which issue it belongs to.</summary>
        public string? Comment { get; set; }
    }

    public class CaseDefaults
    {
        public int TimeoutSeconds { get; set; } = 120;
        // 1e-4 and not tighter: the operations are not bit-reproducible across runs, see the readme.
        public double RelativeTolerance { get; set; } = 1e-4;
    }

    /// <summary>
    /// The two switches for a manual run. They live in the file rather than in environment variables so that
    /// they can be used from the Visual Studio test explorer: edit cases.json, run the tests, edit it back.
    /// <para>
    /// Leaving one of them on would quietly turn the suite into something else - a run that rewrites its own
    /// expectations, or one that only looks at a single case. <see cref="BRepRegressionTests"/> therefore has a
    /// test that fails as long as either is set: it cannot be forgotten and it cannot be committed unnoticed.
    /// </para>
    /// </summary>
    public class RunOptions
    {
        /// <summary>Restrict the run to this single case, e.g. "UniteBug14". Empty means: all cases.</summary>
        public string? Only { get; set; }
        /// <summary>Rewrite the baselines from the current behaviour - only after judging the differences.</summary>
        public bool Regenerate { get; set; }
    }

    /// <summary>
    /// cases.json next to the .cdb.json files: the per-case expectations. Kept outside the project files so that
    /// the drawings stay untouched and the classification is visible in one place (and in the diff).
    /// </summary>
    public class BRepCaseManifest
    {
        public CaseDefaults Defaults { get; set; } = new CaseDefaults();
        public RunOptions Run { get; set; } = new RunOptions();
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
