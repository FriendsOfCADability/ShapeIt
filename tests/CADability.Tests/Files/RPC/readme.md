# RPC regression cases

Every `*.json` in this directory except `run.json` is one regression case, driven through the MCP server's
JSON-RPC interface. A case is a plain list of tool calls plus the recorded result they must produce.

Two things make these files worth having next to the BRep cases:

- **They are what a client actually sends.** A case usually starts life as a real MCP session that went wrong.
  Copy the calls into a file, debug with it, fix the code, record the result — and the session has become a
  regression test.
- **They are not limited to the boolean operations.** Anything the toolset can do is a case: rounding,
  chamfering, sweeps, patterns, features. The result is compared through the same invariants either way.

The harness is `RpcRegressionTests`. Running a file needs no HTTP and no MCP client: `MCPServer` is
constructed directly on a `Project.CreateSimpleProject()`, `SuppressDialogs` is set, and every entry of
`RPCCalls` goes to `MCPServer.ProcessMethod(JsonElement)` — the same path `MainForm.DebugRPC` takes, so a
failing test can be stepped through in the app with `-r <file>`, on the same objects with the same hash codes.

A file the harness cannot understand is **reported as a failure**, never silently skipped. A case that
quietly does nothing is worse than no case at all.

## File layout

```json
{
  "Description": "Free prose. What the case builds and why it exists.",
  "Expected":    "Free prose. What the right answer is and how it was derived.",
  "CaseStatus":  "Ok",
  "Repeat":      1,

  "Verify": ["roundedBlock"],

  "Baseline": {
    "roundedBlock": {
      "verified": "2026-08-14 GH: four cylindrical faces and four spherical corners, volume checked by hand",
      "faces": "14", "edges": "36", "vertices": "24", "poleEdges": "0",
      "holeLoops": "0", "euler": "2", "closed": "true", "openEdges": "0",
      "volume": "23764.6018", "area": "5482.3311",
      "edgeLength": "612.5664",
      "extent": "0, 0, 0, 40, 30, 20",
      "surfaces": "CylindricalSurface:4 PlaneSurface:6 SphericalSurface:4",
      "consistent": "true"
    }
  },

  "RPCCalls": [ … ]
}
```

Everything case-specific lives in the case file. Only the two switches for a manual run are separate, in
`run.json` — they apply to the whole run, not to one case.

Any field the harness does not know is ignored, so prose fields such as `Actual`, `Note` or a dated history
of earlier measurements can be kept freely. They have proved their worth while a defect was open; they are
documentation, never the pass criterion.

### Fields the harness reads

| field | meaning |
|---|---|
| `CaseStatus` | `Ok`, `KnownFail` or `Skip`. Missing means `Ok`. Deliberately **not** called `Status`: the case files use that name for free prose, and a machine read enum next to it would be a trap |
| `Repeat` | how often the whole case is run, each time from a fresh project. Default 1 |
| `RelativeTolerance` | overrides the default `1e-4` for this case |
| `TimeoutSeconds` | overrides the default for this case |
| `Verify` | the workspace names whose result is recorded and compared. Without it: every object committed to the model |
| `Baseline` | the recorded result, one entry per verified name |
| `ExpectError` | the calls that are allowed to fail, see below |
| `RPCCalls` | the JSON-RPC requests, executed in order |

| status | meaning |
|---|---|
| `Ok` | every call must succeed and every baseline must match |
| `KnownFail` | known to be broken; must still match its baseline. If it passes, the test says so — promote it to `Ok` |
| `Skip` | deliberately excluded. Never run |

`KnownFail` matters more here than for the BRep cases: a case is usually written *while* the defect is open,
so it has to be able to live in the suite before it is green. A known failure normally keeps a baseline of its
WRONG result, so that a change becomes visible — a match then means "unchanged", never "passes now", and a
difference is the interesting event, whether it is the fix or a new defect on top.

A known failure may also have **no baseline at all**. That is the honest record for a case whose result is not
reproducible in the first place: pinning one of several values would only produce noise. Nothing is compared
then, and the case reports as inconclusive until someone makes it reproducible. `HelicalThreadOnRod` is the
example — same topology in every run, but its area spreads by 28 percent.

## What is recorded

The baseline is the invariant fingerprint of the result, not its topology: face order, split points, seam
positions and parametrization change legitimately when the algorithms change, and comparing them would drown
the real regressions in false alarms. It is produced by `ShellMetrics.Describe` (`ShapeIt/BRepSummary.cs`) —
the same code the BRep baselines use, so the fields mean exactly what
[`Files/BRep/readme.md`](../BRep/readme.md) says they mean.

Why the whole fingerprint and not just the volume — for a rounding case, the volume is the *weakest* field:

| field | catches |
|---|---|
| `surfaces` | no cylindrical/spherical faces were produced at all, or a plane where a cylinder belongs |
| `faces`, `edges`, `vertices` | the wrong edge was rounded, or one was silently skipped |
| `euler`, `closed`, `openEdges`, `consistent` | the result is broken although the volume looks right |
| `volume`, `area` | wrong radius |

`surfaces` is the single most informative line for a feature operation, and it reads well in a git diff.

Integers and strings are compared exactly, floating point values with `RelativeTolerance` (default `1e-4`),
using the object's own extent as the absolute floor. The default is not tighter because a BRep case was still
seen to differ once in five runs; counts, the Euler characteristic and the surface histogram are exact and are
what actually catches regressions.

### Results with several solids

A boolean operation may legitimately return several parts under one name. The parts are recorded as separate
entries, `<name>#0`, `<name>#1`, …, plus a `solids` count on the name itself:

```json
"hollowDifference": { "solids": "2" },
"hollowDifference#0": { "volume": "640.4273", … },
"hollowDifference#1": { "volume": "33.5104", … }
```

The parts are **sorted canonically** by `ShellMetrics.SortCanonically` (volume, then face count, then area), never by the
order the operation happened to return them. Internal ordering has been observed to vary between runs — a `HashSet`
iteration order once made a whole case fail about one run in five — and a baseline must not depend on it.

### Empty results and missing names

A verified name that does not exist after the run is recorded as `{ "exists": "false" }`. That is the normal
record for an operation whose result is legitimately empty: `solid.boolean` answers with `"empty": true` and
stores nothing, so the name is never created.

### Calls that are allowed to fail

By default **every call must succeed** — a response carrying an `error` fails the case regardless of the
baseline. This is deliberate: the usual symptom of a broken feature operation is an exception, and then there
is no result left to compare. A case that wants to pin down an error declares it:

```json
"ExpectError": [ { "id": 7, "code": "E_INVALID_PARAMS" } ]
```

`id` is the `id` of the call in `RPCCalls`. Report defects by that id — it is the fastest way to point at the
call that broke.

## Recording a result

The right result is **captured, not typed**: run the case once you have judged its outcome correct in the
application, let the harness write the fingerprint, then read the diff.

1. Copy the calls of the session into a new file, add `Description` and `"CaseStatus": "KnownFail"`.
2. Debug with `-r <file>` in ShapeIt.
3. Once fixed and judged correct, set `Regenerate` in `run.json` and run the case.
4. Review the diff. Plausible? Add the `verified` note and set `CaseStatus` to `Ok`.
5. Set `Regenerate` back to `false`.

**Regenerating refuses to write an unstable case.** The regenerate run executes the case `Repeat` times and
only writes when all runs produce the same fingerprint; otherwise it fails and names the field that moved.
Without that guard a single lucky run would freeze a flaky state as "correct" — two of the defects found while
this corpus was built were invisible in a single run, at failure rates of about 21% and 30%. Any case that
has ever flaked gets `"Repeat": 8`.

The `verified` entry is metadata: never compared, and **carried over when the baseline is regenerated**, so
the note survives. A baseline marked this way is treated as more than a snapshot — a difference against it is
reported first and as a regression, and regenerating one produces an explicit warning.

Where the correct value can be *computed* rather than observed — the boolean cases in this directory have
expectations like `1000·(√2−1)`, `28π/3` or `1657 − 271.086607` — write it into the baseline by hand and say
so in the `verified` note. A calculated value states what is right; a captured one only states what was.

## run.json

The two switches for a manual run. They live in a file rather than in environment variables so they work
from the Visual Studio test explorer without any setup:

```json
{
  "Only": "",
  "Regenerate": false
}
```

- **`Only`** — run just this one case, by file name without extension, e.g. `"TouchingEdgeInFace"`.
- **`Regenerate`** — write the baselines from the current behaviour instead of comparing against them.

Set one, run the tests, set it back. As long as either is set, the test **`RunSwitchesAreTurnedOff` fails** —
on purpose: a half finished manual run must not look green, and must not slip into a commit.

The environment variables `RPC_CASE=<name>` and `RPC_REGEN=1` take precedence, for command line runs. The
`VAR=value command` prefix is bash syntax; in PowerShell it is `$env:RPC_REGEN = "1"` on a line of its own,
and `$env:RPC_REGEN = $null` to switch it off again.

## Running

```bash
dotnet test tests/CADability.Tests/CADability.Tests.csproj --filter TestCategory=RPC
```

In Visual Studio: *Test → Test Explorer*, filter for `RpcRegressionTests`.

Each case is its own test, so the explorer names the failing file and it can be re-run alone. Every case and
every repeat runs on a **fresh `Project` and a fresh `MCPServer`** — not an optimization to skip: BRep results
depend on what ran before in the same process, which is documented for the BRep cases too, and cases have been
seen to pass alone and fail inside a batch.

## What the export button writes

The MCP server window can export a finished session as a case file. It writes `RPCCalls` from the recorded
protocol — requests only, in order, with their original ids — plus a `Description` stub and
`"CaseStatus": "KnownFail"`. It writes no `Baseline`: the result is recorded by a regenerate run, after the
outcome has been judged, never by the exporter itself.
