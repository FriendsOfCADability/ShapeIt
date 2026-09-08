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
| `faces`, `edges`, `vertices` | counted over the **patches**, not over the faces as they happen to be split — see below |
| `poleVertices` | vertices carrying an edge without a 3d curve (cone apex, sphere pole). In CADability such an edge always starts and ends at the same vertex; if the two criteria ever disagree, the value says so out loud |
| `holeLoops` | number of inner loops, i.e. every boundary loop of a patch except its first |
| `euler` | `V − E + F − R` over the faces as they are, see below |
| `volume` | `ShellMetrics.IntegratedVolume()` — integrated over the parameter domain, not summed over the triangles, see below |
| `area` | 3d surface area, summed over the **triangulation** of all faces — not the 2d `Face.Area` |
| `edgeLength` | total length of the 3d curves of the counted edges; a seam inside a patch does not count |
| `extent` | bounding box, `xmin, ymin, zmin, xmax, ymax, zmax` |
| `surfaces` | histogram of surface types, one entry per patch |
| `consistent` | `Shell.CheckConsistency()` |
| `closed`, `openEdges` | from `OpenEdgesExceptPoles` — a pole edge has no secondary face, but it does not make a shell open (which is why `Shell.IsClosed` is not used here) |

### Patches: why the counts do not count faces

Where a shell is cut into faces is not part of what the shell means, so it must not be part of the
comparison. Two rules push it around all the time:

- CADability requires a periodic surface to be split as soon as the whole cycle is used — the boundary of a
  face must always span **less** than a full period. That makes many BRep operations simpler and costs
  almost nothing in data, but it means a plain cylinder is two faces, and *where* the seams sit is arbitrary.
- A boolean operation splits faces while it works and merges them back with `Shell.CombineConnectedFaces()`.
  How far that merge gets is an implementation detail: it stops at half a period and it refuses a few surface
  types it cannot combine.

So `faces`, `edges`, `vertices`, `holeLoops`, `poleVertices`, `edgeLength` and `surfaces` are counted over
**patches** instead: a patch is a maximal set of connected faces lying on one and the same surface (same
geometry, same orientation — the test `CombineConnectedFaces` uses, without the half period limit). The
edges between the faces of a patch do not exist, a chain of edges between the same two patches counts once,
and a vertex where exactly two of those chains meet is an artefact of a split and is not counted either.

Nothing is merged for real — this is counting only. `CombineConnectedFaces()` would be the obvious way to
canonicalize, but it performs real geometry surgery, it can fail on exactly the broken data this harness
exists for, and it is itself only canonical up to half a period.

The test **`CountsDoNotDependOnHowFacesAreSplit`** holds this against real data: for every operand it
compares the counts of the shell as read with the counts of the same shell after `CombineConnectedFaces()`.
Those two are split differently by construction, so any count that still depends on the splitting shows up.

`euler` is the exception — it is read off the faces as they are, because it is *already* invariant: cutting
a face in two adds one face and one edge, or one face, three edges and two vertices, and `V − E + F − R`
does not move either way. It is also the one number a patch complex cannot always express, since a closed
patch such as a whole sphere has no boundary at all.

Two places where `V − E + F − R` over the patches therefore does *not* reproduce `euler`, both harmless
and both deterministic:

- a **closed patch** — a whole sphere is one patch with no boundary, so it contributes 1 instead of 2.
- **boundary loops that touch at a vertex** are one connected component and are counted as one loop, so
  `holeLoops` comes out short. That is the union of touching rings in `UniteBug4/5/6`, one or two short.
  The raw `euler` has the same blind spot, because `Face.HoleCount` counts a pinched outline as one loop
  as well. Telling them apart would mean walking the boundary in order rather than by connectivity.

**`euler`** is `V − E + F − R` with `E` counting real edges only and `R` the number of inner loops.
Subtracting `R` is what makes the number usable for faces with holes — the plain `V − E + F` needs every
face to be a disk. For a single closed shell the result is `2 − 2·genus`: 2 for anything sphere-like, 0 with
one through hole, −2 with two. It is a topology fingerprint, not an assertion; a change in the diff means
faces, edges or loops got lost or duplicated.

Integers and strings are compared exactly, floating point numbers with a relative tolerance
(`RelativeTolerance`, default `1e-4`), using the size of the input as the absolute floor.

### Why the numbers used to move between runs

`volume`, `area` and `extent` come from the triangulation, and `Face.AssureTriangles(0.0)` — the precision the
rest of the code passes — **reuses whatever triangulation happens to exist**, however coarse it was made, and
invents `extent size / 10` when there is none. So the values depended on what had run before in the same
process: `DifferenceBug3` produced a 10% different area depending on whether it ran alone or in a full suite,
and its area was understated by 58% in both cases.

`ShellMetrics.PrecisionFor` therefore derives an explicit precision from the exact geometry — vertex positions
and edge curves, never from a triangulation — and passes that to the volume, `GetExtent` and
`GetTriangulation`. Same number in every run, at the price of roughly a third more runtime.

That fixed *which* mesh is used, but not the fact that a mesh decided the answer at all. Replacing
`Triangulation.cs` with the CDT triangulator made the point: the same **unchanged** operand measured 3.6e-3
differently, because a different mesh is a different set of flat triangles. `UniteBug6` showed it on `in1`, an
input the operation never touches. Three things were done about it on 2026-09-07:

- **`volume` is integrated, not summed.** `ShellMetrics.IntegratedVolume` evaluates
  `1/3 ∫∫ S·(Sᵤ×Sᵥ) du dv` over the parameter domain of each face. Only `PointAt` and `DerivativeAt` appear
  under the integral — the 3d boundary curves never do, they only decide the region, and that region lives in
  the parameter plane. So an `InterpolatedDualSurfaceCurve` costs nothing here and a NURBS face is no
  different from a plane. It is **exact** for a planar face (the integrand is constant there) and for one whose
  domain is the full rectangle (cylinder, cone and torus mantles); a patch with a curved uv outline keeps a
  first order correction. Held against closed forms in `VolumeTests`: the cone frustum and the torus segment
  come out bit identical on five different meshes, the hemisphere within 6.5e-4 against 0.77% for the triangle
  sum.
- **The area quadrature is no longer switched off by accident.** It refused any over-coverage beyond `1e-6`
  while allowing a 2% shortfall; the uv triangles of a trimmed face routinely stick out by a few parts per
  million, so all eight cylindrical faces of `DifferenceBug9` fell back to the flat sum. Its area was 0.79%
  short and never converged. The limit is now 2% in both directions.
- **The mesh is four times finer** (`size / 4000`), which is what the remaining first order paths need.

The tolerance stays at `1e-4`. Counts, the Euler characteristic, the surface histogram and the status are
compared exactly and were never affected; they are what actually catches regressions.

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
