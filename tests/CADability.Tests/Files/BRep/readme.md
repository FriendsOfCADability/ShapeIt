# BRep regression cases

Every `*.cdb.json` in this directory is one regression case for the BRep operations. The case is described
inside the project file itself, so the very same file can be opened in ShapeIt for debugging — with the same
objects and the same hash codes, which is what makes conditional breakpoints usable.

The harness is [`BRepRegressionTests`](../../BRepRegressionTests.cs). Reading a case, running it and
summarizing the result live in the ShapeIt project (`BRepTestCase.cs`, `BRepRunner.cs`, `BRepSummary.cs`),
because `MainForm.AutoDebug` uses exactly the same code: a failing test can be reproduced in the app, on the
same objects, with the same hash codes.

## Convention inside the project file

| what | how it is marked |
|---|---|
| first operand | style `Operand1` |
| second operand | style `Operand2` |
| edges to round/chamfer | curves with the style `EdgeMarker` (matched against the edges of `Operand1`) |
| the operation | a single text object |
| the known good result (optional) | a solid/shell with the style `Expected` |

The text object names the operation and, where needed, its parameter:

```
Difference        Unite            Intersect        UniteAll
RoundEdges: 2.5   ChamferEdges: 1  ChamferEdges: 1, 2
```

Accepted spellings: `Difference`/`Subtract`, `Unite`/`Union`, `Intersect`/`Intersection`,
`UniteAll`/`UnionAll`, `RoundEdges`/`Fillet`, `ChamferEdges`/`Chamfer`. Numbers are always read with the
invariant culture, so `2.5` is two and a half on a German machine too.

`UniteAll` is the exception to the operand marking: it unites *every* solid of the model, in the order of
their extent centers, so `Operand1`/`Operand2` are not needed there.

A file the harness cannot understand is **reported as a failure**, never silently skipped — a case that
quietly does nothing is worse than no case at all.

## cases.json

Per-case expectations. A file that is not listed defaults to `Ok`.

| status | meaning |
|---|---|
| `Ok` | must produce a valid result and must match its baseline |
| `KnownFail` | known to be broken; must still match its baseline. If it succeeds, the test says so — promote it to `Ok` |
| `CorruptInput` | the operands already fail `Shell.CheckConsistency()`; an import/authoring bug, not a BRep bug. Never run |
| `NeedsFixup` | the file does not follow the convention yet. Never run |
| `Skip` | deliberately excluded |

`cases.json` can also supply the `Operation` and its parameters for a file whose text object is missing or
ambiguous, and override `TimeoutSeconds` / `RelativeTolerance` per case.

## Baselines

`Baselines/<case>.txt` holds the invariants of the input and of the result. Deliberately *not* the topology
itself: face order, split points, seam positions and parametrization change legitimately when the algorithms
change, and comparing them would drown the real regressions in false alarms.

| key | meaning |
|---|---|
| `faces`, `vertices` | plain counts |
| `edges` | edges of the solid — **pole edges are not counted** |
| `poleEdges` | edges without a 3d curve (cone apex, sphere pole). In CADability such an edge always starts and ends at the same vertex; if the two criteria ever disagree, the value says so out loud |
| `holeLoops` | number of inner loops over all faces |
| `euler` | `V − E + F − R`, see below |
| `volume` | `Shell.Volume()` |
| `area` | 3d surface area, summed over the **triangulation** of all faces — not the 2d `Face.Area` |
| `edgeLength` | total length of all 3d edge curves |
| `extent` | bounding box, `xmin, ymin, zmin, xmax, ymax, zmax` |
| `surfaces` | histogram of surface types |
| `consistent` | `Shell.CheckConsistency()` |
| `closed`, `openEdges` | from `OpenEdgesExceptPoles` — a pole edge has no secondary face, but it does not make a shell open (which is why `Shell.IsClosed` is not used here) |

**`euler`** is `V − E + F − R` with `E` counting real edges only and `R` the number of inner loops.
Subtracting `R` is what makes the number usable for faces with holes — the plain `V − E + F` needs every
face to be a disk. For a single closed shell the result is `2 − 2·genus`: 2 for anything sphere-like, 0 with
one through hole, −2 with two. It is a topology fingerprint, not an assertion; a change in the diff means
faces, edges or loops got lost or duplicated.

Integers and strings are compared exactly, floating point numbers with a relative tolerance
(`RelativeTolerance`, default `1e-4`), using the size of the input as the absolute floor.

The tolerance is that loose on purpose: the operations are **not bit-reproducible across runs**.
`DifferenceBug15` was seen to differ by 5e-6 in volume during a full run, while it reproduces exactly when
run on its own — which suggests an iteration order somewhere that depends on reference hash codes, and those
depend on what ran before in the same process. The counts, the Euler characteristic, the surface histogram
and the status are compared exactly and are unaffected; they are what actually catches regressions.

## Marking a result as correct

Once you have judged a result correct, say so in a comment line at the top of its baseline:

```
# verified 2026-08-01: the subtraction leaves exactly the two expected parts
```

Comment lines (anything starting with `#`) are ignored by the comparison and are **carried over when the
baseline is regenerated**, so the note survives `BREP_REGEN=1`. Debugging never touches these files anyway.

A baseline marked this way is treated as more than a snapshot: a difference is reported separately and
first, as a regression rather than just a change, and regenerating one produces an explicit warning.

```bash
dotnet test tests/CADability.Tests/CADability.Tests.csproj --filter FullyQualifiedName~BRepRegressionTests
```

In Visual Studio: *Test → Test Explorer*, filter for `BRepRegressionTests`, run.

### The two switches for a manual run

They live in `cases.json`, so they work from the Visual Studio test explorer without setting anything up:

```json
"Run": {
  "Only": "",
  "Regenerate": false
}
```

- **`Only`** — run just this one case, e.g. `"UniteBug14"`. Empty means all cases.
- **`Regenerate`** — write the baselines from the current behaviour instead of comparing against them. Only
  after you judged a difference to be an improvement; review the changed files under `Baselines/` afterwards.
  Hand written `#` comments (the `# verified` notes) are carried over.

Set one, run the tests, set it back. As long as either is set, the test **`BaselineSwitchesAreTurnedOff`
fails** — on purpose: a half finished manual run must not look green, and must not slip into a commit.

The environment variables `BREP_REGEN=1` and `BREP_CASE=<name>` still work and take precedence, for command
line runs. Note that the `VAR=value command` prefix is bash syntax; in PowerShell it is `$env:BREP_REGEN = "1"`
on a line of its own, and `$env:BREP_REGEN = $null` to switch it off again.

Note that the tests run against a **Debug** build, where `Debug.Assert` failures are turned into exceptions
by the test host. Several of the known failures are assertion hits rather than wrong results.
